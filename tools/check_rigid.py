#!/usr/bin/env python3
"""校验 rigidify 管线的输出：XML 是否干净、tail 是否有多余内容。"""
import sys
import xml.etree.ElementTree as ET

path = sys.argv[1]
text = open(path).read()
try:
    root = ET.fromstring(text)
except ET.ParseError as e:
    print("PARSE_FAIL:", e)
    print("TAIL:", repr(text[-120:]))
    sys.exit(1)

links = len(list(root.iter("link")))
joints = list(root.iter("joint"))
types = {}
for j in joints:
    t = j.get("type")
    types[t] = types.get(t, 0) + 1

print("PARSE_OK root=%s name=%s" % (root.tag, root.get("name")))
print("links=%d joints=%d" % (links, len(joints)))
print("joint_types=%s" % types)
print("TAIL=%r" % text[-60:])
