#!/usr/bin/env python3

"""
Runs fix_inertia from an install tree.
"""

from os.path import isdir, join, dirname, realpath
import sys


def main():
    # Ensure that we can import pydrake, accommodating symlinks.
    prefix_dir = dirname(dirname(realpath(__file__)))
    assert isdir(join(prefix_dir, "bin")), f"Bad location: {prefix_dir}"
    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    site_dir = join(prefix_dir, f"lib/python{version}/site-packages")
    sys.path.insert(0, site_dir)

    # Execute the imported main.
    from pydrake.multibody.fix_inertia import _main
    _main()


assert __name__ == "__main__"
main()
