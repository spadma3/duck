# -*- coding: utf-8 -*-
from .resolution import Suggestion

class Entry():
    def __init__(self, desc, check, diagnosis, resolutions, only_run_if):
        self.desc = desc
        self.check = check
        self.diagnosis = diagnosis
        self.resolutions = resolutions
        self.only_run_if = only_run_if

Diagnosis = Suggestion


def SeeDocs(id_topic):
    msg = '➡ See duckumentation at http://purl.org/dth/%s' % id_topic
    return Suggestion(msg)

        