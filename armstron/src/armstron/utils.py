from __future__ import print_function
import sys
import os
import yaml


def check_any(conditions):
    out = False
    for condition in conditions:
        if condition:
            out = True
            break
    
    return out

def check_all(conditions):
    out = True
    for condition in conditions:
        if not condition:
            out = False
            break
    
    return out


def load_yaml(filename):
    out = None
    try:
        with open(filename, 'r') as f:
            out = yaml.safe_load(f)
    except:
        pass    
    return out


def save_yaml(data, filename):
    out=None
    try:
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=None)
        out=True
    except:
        pass
    
    return out