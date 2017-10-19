#!/bin/bash
set -e
set -x

sudo apt install -y \
	python-frozendict \
	libxslt-dev \
	libxml2-dev \
	python-lxml \
	python-bs4 \
	python-ruamel.yaml \
	python-ruamel.ordereddict \
	python-pymongo

pip install --upgrade --user \
	PyContracts \
	QuickApp \
	conftools \
	comptests \
	procgraph

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
