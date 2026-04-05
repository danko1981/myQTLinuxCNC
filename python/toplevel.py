# -*- coding: utf-8 -*-
import remap

def change_tool(self, **words):
    # Questa funzione fa da ponte tra LinuxCNC e la logica nel file remap.py
    return remap.manual_change_with_probe(self, **words)
