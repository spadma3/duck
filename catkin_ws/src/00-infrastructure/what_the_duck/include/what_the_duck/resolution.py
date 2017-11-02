# -*- coding: utf-8 -*-
class Suggestion():
    def __init__(self, s):
        self.s = s.strip()
        
    def __str__(self):
        return self.s