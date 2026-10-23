#!/usr/bin/env python3
"""Create the minimal release ZIP using the same attributes as GitHub archives."""
import argparse, json, re, subprocess, zipfile
from pathlib import Path, PurePosixPath
root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--output', required=True, type=Path)
p.add_argument('--ref', default='HEAD')
a = p.parse_args()
repo = 'gz-dynamic-terrain-plugin'
if (root / 'DynamicTerrain.uplugin').exists():
    version = json.loads((root / 'DynamicTerrain.uplugin').read_text())['VersionName']
else:
    version = re.search(r'project\([^)]*VERSION\s+(\d+\.\d+\.\d+)', (root / 'CMakeLists.txt').read_text()).group(1)
name = repo + '-v' + version
a.output.mkdir(parents=True, exist_ok=True)
archive = a.output.resolve() / (name + '.zip')
subprocess.run(['git', '-C', str(root), 'archive', '--format=zip', '--prefix=' + name + '/', '-o', str(archive), a.ref], check=True)
with zipfile.ZipFile(archive) as z:
    names = {i.filename.removeprefix(name + '/') for i in z.infolist() if not i.is_dir()}
    assert {'README.md', 'LICENSE'} <= names
    for path in names:
        parts = PurePosixPath(path).parts
        assert not {'docs','images','videos','examples','tests','.github','build','Binaries','Intermediate','Saved','__pycache__'}.intersection(parts), path
        assert not path.endswith(('.so','.o','.a','.pyc','.log')), path
print(archive)
