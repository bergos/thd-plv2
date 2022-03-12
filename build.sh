#!/bin/bash

# setup virtual environment
python3 -m venv venv
. ./venv/bin/activate

# install sphinx-book-theme and markdown parser
pip install sphinx-book-theme myst-parser

# build the html from the markdown files
sphinx-build -M html . docs
