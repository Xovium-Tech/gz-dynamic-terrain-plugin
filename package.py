#!/usr/bin/env python3
"""Package current source and optional matching Gazebo Linux binaries with hashes."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import platform
import re
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parent
SOURCE_NAMES = (
    'CMakeLists.txt', 'README.md', 'LICENSE', 'build.sh', 'package.py', 'package.sh',
    'cmake', 'include', 'src', 'docs', 'examples', 'tests', 'tools',
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources():
    result = []
    for name in SOURCE_NAMES:
        source = ROOT / name
        for path in sorted(source.rglob('*')) if source.is_dir() else [source]:
            if (path.is_file() and not path.is_symlink()
                    and '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.log')):
                result.append(path)
    return result


def git_output(*args):
    try:
        return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def cache_values(build):
    values = {}
    for line in (build / 'CMakeCache.txt').read_text().splitlines():
        if not line.startswith(('#', '//')) and '=' in line and ':' in line.split('=', 1)[0]:
            name, value = line.split('=', 1)
            values[name.split(':', 1)[0]] = value
    return values


def add_json(archive, name, data):
    payload = (json.dumps(data, indent=2, sort_keys=True) + '\n').encode()
    info = tarfile.TarInfo(name)
    info.size, info.mode = len(payload), 0o644
    archive.addfile(info, io.BytesIO(payload))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--with-binaries', type=Path, metavar='BUILD',
                        help='Include freshly built, matching Harmonic or Jetty libraries')
    args = parser.parse_args()
    output = args.output.resolve()
    if output.is_relative_to(ROOT):
        parser.error('Choose an output archive outside the source folder')
    source_files = sources()
    source_manifest = {
        'schema': 1, 'project': 'gz-dynamic-terrain-plugin',
        'upstream': 'https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin',
        'git_head': git_output('rev-parse', 'HEAD'),
        'working_tree_modified': bool(git_output('status', '--porcelain')),
        'files': {str(path.relative_to(ROOT)): digest(path) for path in source_files},
    }
    with tempfile.TemporaryDirectory(prefix='terrain-package-') as temporary:
        binary_files = []
        binary_manifest = None
        if args.with_binaries:
            build = args.with_binaries.resolve()
            cache = cache_values(build)
            if Path(cache.get('CMAKE_HOME_DIRECTORY', '')).resolve() != ROOT:
                parser.error('The build directory does not belong to this source checkout')
            distro = cache.get('GZ_DISTRO')
            if distro not in ('harmonic', 'jetty'):
                parser.error('The build does not identify a supported Gazebo distribution')
            names = ['libgz-dynamic-terrain-core.so', 'libgz-dynamic-terrain-system.so']
            if cache.get('BUILD_GUI') == 'ON':
                names.append('libDynamicTerrainGui.so')
            built = [build / name for name in names]
            if not all(path.is_file() for path in built):
                parser.error('Build all requested libraries before packaging binaries')
            inputs = [path for path in source_files if path.name == 'CMakeLists.txt'
                      or path.relative_to(ROOT).parts[0] in ('src', 'include', 'cmake')]
            if max(path.stat().st_mtime_ns for path in inputs) > min(path.stat().st_mtime_ns for path in built):
                parser.error('Compile inputs are newer than the binaries; rebuild before packaging')
            # Installing into a private temporary prefix applies the configured
            # relocatable RPATH without touching the user's simulator installation.
            stage = Path(temporary) / 'install'
            subprocess.run(['cmake', '--install', str(build), '--prefix', str(stage)], check=True,
                           stdout=subprocess.DEVNULL)
            libraries = {}
            for name in names:
                matches = list(stage.rglob(name))
                if len(matches) != 1:
                    raise RuntimeError(f'Expected one installed {name}, found {len(matches)}')
                library = matches[0]
                dynamic = subprocess.check_output(['readelf', '-d', str(library)], text=True)
                dependencies = subprocess.check_output(['ldd', str(library)], text=True)
                if 'not found' in dependencies:
                    raise RuntimeError(f'{name} has unresolved dependencies: {dependencies}')
                libraries[name] = {
                    'sha256': digest(library),
                    'needed': re.findall(r'\(NEEDED\).*\[(.*?)\]', dynamic),
                    'resolved_dependencies': dependencies.splitlines(),
                }
                binary_files.append((library, 'lib/' + name))
            binary_manifest = {
                'schema': 1, 'project': 'gz-dynamic-terrain-plugin',
                'gazebo_distribution': distro, 'platform': platform.system(),
                'architecture': platform.machine(), 'os_release': platform.freedesktop_os_release(),
                'build_type': cache.get('CMAKE_BUILD_TYPE'),
                'opencv_configuration': cache.get('OpenCV_DIR'),
                'source_files': source_manifest['files'], 'libraries': libraries,
                'requirements': 'Use the recorded Gazebo distribution and compatible listed shared libraries; rebuild from source on another ABI.',
            }
        output.parent.mkdir(parents=True, exist_ok=True)
        with tarfile.open(output, 'w:gz') as archive:
            for path in source_files:
                archive.add(path, arcname=str(path.relative_to(ROOT)), recursive=False)
            add_json(archive, 'SOURCE_MANIFEST.json', source_manifest)
            for path, name in binary_files:
                archive.add(path, arcname=name, recursive=False)
            if binary_manifest:
                add_json(archive, 'BINARY_MANIFEST.json', binary_manifest)
    print(output)


if __name__ == '__main__':
    main()
