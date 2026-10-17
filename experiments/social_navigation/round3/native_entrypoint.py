#!/usr/bin/env python3
"""Compatibility launcher for the unchanged upstream CLI.
Legacy test.py passes positional args to reset; Gym's OrderEnforcing wrapper
accepts keyword args only. Returning the raw registered env preserves dynamics.
"""
import runpy
import gym

original_make=gym.make
def make_raw(*args,**kwargs):
    return original_make(*args,**kwargs).unwrapped
gym.make=make_raw
runpy.run_path('/opt/ComplexityNav/crowd_nav/test.py',run_name='__main__')
