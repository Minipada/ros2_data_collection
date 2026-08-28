#!/usr/bin/env python3
import os
import shutil

os.chdir(os.path.dirname(os.path.realpath(__file__)))

if shutil.which('doxygen') is None:
    exit("You must install doxygen")

# clean
if os.path.exists('docs'):
    shutil.rmtree('docs')

# build
if os.system('doxygen docsrc/Doxyfile') != 0:
    exit(1)

# disable jekyll on gh-pages, because jekyll ignores html files
# started with underscore, and doxygen does generate such filenames
open('docs/.nojekyll', 'w').close()
