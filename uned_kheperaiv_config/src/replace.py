#!/usr/bin/env python3
# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Replace strings like ${key} with a value."""

import sys
from typing import Dict, List


def replace(s: str, d: Dict[str, str]) -> str:
    for k, v in d.items():
        long_k = '${' + k + '}'
        s = s.replace(long_k, v)
    return s


def parse_args(argv: List[str]) -> Dict[str, str]:
    d = {}
    for a in argv:
        p = a.split('=')
        if len(p) != 2:
            print('ignoring "%r"' % a, file=sys.stderr)
            continue
        d[p[0]] = p[1]
    return d


if len(sys.argv) < 2:
    print('usage: replace.py something.xml foo=1 bar=2.0 fee=string random="also a string"',
          file=sys.stderr)
    sys.exit(1)

f = open(sys.argv[1], 'r')
print(replace(f.read(), parse_args(sys.argv[2:])))
