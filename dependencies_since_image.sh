#!/bin/bash
set -e
set -x

# Important: always use the python-X library if available,
# rather than doing "pip install X".

sudo apt install -y \
	python-frozendict \
	libxslt-dev \
	libxml2-dev \
	python-lxml \
	python-bs4 \
	python-tables

sudo apt remove -y \
	python-ruamel.yaml \
	python-ruamel.ordereddict

# These don't have an APT package

pip install --upgrade --user \
	PyContracts \
	QuickApp \
	conftools \
	comptests \
	procgraph \
	pymongo \
	ruamel.yaml==0.15.34

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
