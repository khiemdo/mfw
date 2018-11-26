#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# This maps the correct author names and emails in the log files

import subprocess

subprocess.call(['git', 'config', '--local', 'log.mailmap', 'true'])