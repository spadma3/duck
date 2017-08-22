#!/usr/bin/env python
from easy_node.node_description.doc_generation import generate_easy_node_docs
from duckietown_utils.exceptions import wrap_script_entry_point
from duckietown_utils import col_logging  # @UnusedImport

if __name__ == '__main__':
    wrap_script_entry_point(generate_easy_node_docs)