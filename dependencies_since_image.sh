#!/bin/bash
set -e
set -x
sudo apt install -y python-frozendict libxslt-dev libxml2-dev
pip install --upgrade --user \
	ruamel.yaml \
	bs4 \
	lxml \
	PyContracts \
	QuickApp \
	conftools \
	comptests \
	procgraph

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel 
