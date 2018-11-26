#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import unicodedata
from datetime import datetime as dt

hash_length = 8

def get_current_hash():
  try:
    hash = subprocess.check_output(["git", "rev-parse", "HEAD"], stderr=subprocess.STDOUT)
  except subprocess.CalledProcessError:
    hash = 'n/a'

  try:
    isDirty = subprocess.call(["git", "diff-index", "--quiet", "HEAD"], stderr=subprocess.STDOUT)
  except subprocess.CalledProcessError:
    isDirty = True

  if(isDirty):
    dirty = "-dirty"
  else:
    dirty = ''

  return (str(hash[0:hash_length])+dirty)

def get_version():
  try:
    f = open(os.path.dirname(os.path.abspath(__file__)) + '/../../MFW_VERSION', 'r')
    version = f.readline()
    f.close()
  except IOError:
    version = 'n/a'

  return version

def get_time():
  return(dt.now().strftime('%Y%m%d %H:%M:%S'))
  
def get_user():
  try:
    user = subprocess.check_output(["git", "config", "user.name"], stderr=subprocess.STDOUT)
  except subprocess.CalledProcessError:
    user = 'n/a'
    
  return unicodedata.normalize('NFKD', unicode(user[:-1], 'utf-8')).encode('ascii', 'ignore')

def create_header():
  buildstring = ''.join(['#define BUILD_DATE Version: ', get_version(), ' - Commit Hash: ', get_current_hash(), ' - Build Date: ', get_time(), ' by ', get_user(), ' - Active Config:'])
  #buildstring = 
  outstring = '\n'.join([
  '#ifndef VERSION_H_',
  '#define VERSION_H_',
  '',  
  '#ifndef ACTIVE_CONFIG',
  '#warning "Active config is unknown. Please set it with --define ACTIVE_CONFIG=XYZ in compile options. Meanwhile, ACTIVE_CONFIG is set to UNKNOWN"',
  '#define ACTIVE_CONFIG UNKNOWN',
  '#endif',
  '',
  '#define XSTR(s) STR(s)',
  '#define STR(s) #s',
  '',
  buildstring,
  '#define BUILD_STRING XSTR(BUILD_DATE ACTIVE_CONFIG)',
  '',
  '#endif //VERSION_H_',
  ''])

  try:
    f = open(os.path.dirname(os.path.abspath(__file__)) + '/version.h', 'w+')
    f.write(outstring)
    f.close()
  except IOError as err:
    print("[Create Version Header] - Failed miserably in writing header file: " + str(err))

def create_body():
  outstring = '\n'.join([
  '#include "version.h"',
  '',
  'extern const char* buildString = BUILD_STRING;',
  ''])
  try:
    f = open(os.path.dirname(os.path.abspath(__file__)) + '/version.c', 'w+')
    f.write(outstring)
    f.close()
  except IOError as err:
    print("[Create Version Header] - Failed miserably in writing body file: " + str(err))
    
def tup():
  path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + '../../../submodules/betCOM/')
  print(path)
  subprocess.call(['tup', '--no-environ-check'], cwd=path)

def git_skip_files():

  subprocess.call(['git', 'update-index', '--skip-worktree',
                   os.path.dirname(os.path.abspath(__file__)) + '/version.h',
                   os.path.dirname(os.path.abspath(__file__)) + '/version.c'])
  return


if __name__ == '__main__':
  #tup()
  create_header()
  create_body()
  git_skip_files()
