#!/usr/bin/env python3
"""Build a test-only reference member; production headers need no test hooks.

Run on a host/container with the existing C++ compiler and PCL development
headers. Only temporary build files are generated; the source stays unchanged.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--iterations', type=int, default=20000)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cxx', default='c++')
    options = parser.parse_args()
    directory = Path(__file__).resolve().parent
    source = directory.parent
    pkg = None
    for name in ['pcl_common', 'pcl_common-1.12', 'pcl_common-1.10']:
        if subprocess.run(['pkg-config', '--exists', name]).returncode == 0:
            pkg = name
            break
    if pkg is None:
        raise SystemExit('PCL development headers/pkg-config metadata are required')
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', pkg], text=True))
    # Some installed PCL pkg-config files omit Eigen's include directory.
    if Path('/usr/include/eigen3/Eigen/Core').exists():
        flags.append('-I/usr/include/eigen3')
    header = (source / 'ikd_Tree.h').read_text()
    declaration = '    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, float max_dist = INFINITY);'
    assert header.count(declaration) == 1
    header = header.replace(declaration, declaration + '\n' + declaration.replace('Nearest_Search(', 'Nearest_Search_Reference('))
    implementation = (source / 'ikd_Tree.cpp').read_text()
    insertion = '// Manual Instatiations'
    # Match the spelling used in the upstream file without modifying it.
    insertion = next(line for line in implementation.splitlines() if line.startswith('// Manual Inst'))
    implementation = implementation.replace(insertion, (directory / 'nearest_search_reference.inc').read_text() + '\n' + insertion, 1)
    with tempfile.TemporaryDirectory(prefix='ikd-nn-parity-') as temporary:
        build = Path(temporary)
        (build / 'ikd_Tree.h').write_text(header)
        (build / 'ikd_Tree.cpp').write_text(implementation)
        binary = build / 'nearest_search_parity'
        command = [options.cxx, '-std=c++14', '-O3', '-pthread', '-I' + str(build),
                   str(directory / 'nearest_search_parity.cpp'), str(build / 'ikd_Tree.cpp'),
                   '-o', str(binary), *flags]
        subprocess.run(command, check=True)
        result = subprocess.run([str(binary), str(options.iterations)], text=True, capture_output=True, check=True)
        print(result.stdout, end='')
        rows = [line[len('RESULT_JSON '):] for line in result.stdout.splitlines() if line.startswith('RESULT_JSON ')]
        assert len(rows) == 1
        summary = json.loads(rows[0])
        summary['compiler_command'] = command
        summary['compiler_version'] = subprocess.check_output([options.cxx, '--version'], text=True).splitlines()[0]
        summary['pcl_pkg_config_name'] = pkg
        if options.output:
            options.output.write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
