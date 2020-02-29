#!/usr/bin/env python
"""qpOASES python distutils setup script."""

#
#  This file is part of qpOASES.
#
#  qpOASES -- An Implementation of the Online Active Set Strategy.
#  Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
#  Christian Kirches et al. All rights reserved.
#
#  qpOASES is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  qpOASES is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with qpOASES; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

#
#   Filename:  setup.py
#   Author:    Sebastian F. Walter, Manuel Kudruss (thanks to Felix Lenders)
#   Version:   3.2
#   Date:      2013-2017
#

import sys
import numpy as np

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize

# HACK for passing source files via command line
sources_index = sys.argv.index('--sources')
sources = sys.argv[(sources_index + 1):]
sys.argv = sys.argv[:sources_index]

extra_params = {
    'include_dirs': [
        np.get_include()
    ],
    'extra_compile_args': [
        "-O2",
        "-Wno-unused-variable",
        "-Wall",
        "-pedantic",
        "-Wshadow",
        "-Wfloat-equal",
        "-O3",
        "-Wconversion",
        "-Wsign-conversion",
        "-finline-functions",
        "-fPIC",
        "-DLINUX",
        "-D__USE_LONG_INTEGERS__",
        "-D__USE_LONG_FINTS__",
        "-D__NO_COPYRIGHT__",
        ],
    'extra_link_args': ["-Wl,--as-needed"],
    'language': 'c++'
}

ext_modules = [
    Extension("qpoases", sources, **extra_params),
]

setup(
    name='qpOASES interface',
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(ext_modules),
)
