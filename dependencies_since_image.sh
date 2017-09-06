#!/bin/bash
set -e
set -x
sudo apt install -y python-frozendict 
pip install --upgrade --user PyContracts QuickApp conftools comptests procgraph
sudo pip install -U pip setuptools wheel
sudo pip install ruamel.yaml --upgrade
