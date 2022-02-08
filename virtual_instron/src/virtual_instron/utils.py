from __future__ import print_function
import rospy
import sys
import os


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
