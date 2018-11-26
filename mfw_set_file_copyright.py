#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import subprocess
import os, fnmatch
import datetime
from enum import Enum
import argparse
import ntpath
import re
import subprocess


def main():
  subprocess.call("python ./doc/doc-tools/copyright/set_file_copyright.py . -e ./doc/.copyright-exclude -t ./doc/doc-tools/copyright/LICENSE_TEMPLATE.c -fdv", shell=True)

if __name__ == '__main__':
  main()
