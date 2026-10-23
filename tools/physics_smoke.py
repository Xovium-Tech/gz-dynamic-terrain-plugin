#!/usr/bin/env python3
"""Build a real Gazebo contact/replacement fixture against existing tested libraries."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', required=True, type=Path,
                        help='Existing Makefiles Harmonic/Jetty build with SystemPluginSmoke_TEST enabled')
    parser.add_argument('--libraries', type=Path,
                        help='Optional alternate baseline library directory, same Gazebo ABI')
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    build = args.build.resolve()
    libraries = (args.libraries or build).resolve()
    report = args.report.resolve()
    source = Path(__file__).resolve().with_name('terrain_physics_smoke.cc')
    target = build / 'CMakeFiles/SystemPluginSmoke_TEST.dir'
    flags = {}
    for line in (target / 'flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    with tempfile.TemporaryDirectory(prefix='terrain-physics-build-') as temporary:
        obj = str(Path(temporary) / 'fixture.o')
        binary = str(Path(temporary) / 'TerrainPhysicsSmoke')
        subprocess.run(['/usr/bin/c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'],
                        *flags['CXX_FLAGS'], '-c', str(source), '-o', obj], check=True)
        link = shlex.split((target / 'link.txt').read_text())
        for index, part in enumerate(link):
            if part.startswith('CMakeFiles/SystemPluginSmoke_TEST.dir/') and part.endswith('.o'):
                link[index] = obj
            elif Path(part).name in ('libgz-dynamic-terrain-core.so', 'libgz-dynamic-terrain-system.so'):
                link[index] = str(libraries / Path(part).name)
            elif part.startswith('-Wl,-rpath,'):
                link[index] = part.replace(str(build), str(libraries))
        link[link.index('-o')+1] = binary
        subprocess.run(link, cwd=build, check=True)
        report.parent.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = Path(temporary).name
        environment['GZ_IP'] = '127.0.0.1'
        subprocess.run([binary, str(libraries / 'libgz-dynamic-terrain-system.so'), str(report)],
                       check=True, timeout=90, env=environment)


if __name__ == '__main__':
    main()
