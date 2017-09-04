#!/bin/bash
set -e
set -x
sudo apt install -y python-frozendict python-ruamel.yaml
pip install --upgrade --user PyContracts QuickApp conftools comptests procgraph
