#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import subprocess

def parse_version_file(fn):
    with open(fn, "r") as fh:
        return [int(v) for v in fh.readline().split(".")]


def find_firmware_version(device):
    version_fn = "{}_VERSION".format(device)
    path = os.path.abspath("")
    parent, _ = os.path.split(path)
    while path != os.path.abspath("/"):
        if version_fn in os.listdir(path):
            fn = os.path.join(path, version_fn)
            return parse_version_file(fn)

        path = parent
        parent, _ = os.path.split(path)
    return (0, 0, 0)


def find_protobuf_version():
    out = subprocess.check_output(["protoc", "--version"])
    return [int(i) for i in out.split(" ")[1].split(".")]

def find_nanopb_version():
    with open("c-nanopb/nanopb_generator.py", "r") as fh:
        for line in fh:
            if line.startswith("nanopb_version") and "=" in line and "\"" in line:
                version_numbers = line.split("=")[1].strip()[1:-1].split(".")
                return [int(version_numbers[0].split("-")[1]),
                       int(version_numbers[1]),
                       int(version_numbers[2].split("-")[0])]

def make_version_message_string(firmware, major, minor, rev):
    template = '''#FIRMWARE_version: #MAJOR.#MINOR.#REV'''
    template = template.replace("#MAJOR", str(major))
    template = template.replace("#MINOR", str(minor))
    template = template.replace("#REV", str(rev))
    template = template.replace("#FIRMWARE", str(firmware))
    print template

make_version_message_string(*(["BetCOM"] + find_firmware_version("BetCOM")))
make_version_message_string(*(["protobuf"] + find_protobuf_version()))
make_version_message_string(*(["nanopb"] + find_nanopb_version()))
