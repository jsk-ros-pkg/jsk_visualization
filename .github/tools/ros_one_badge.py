# Software License Agreement (BSD License)
#
# Copyright (c) 2026, JSK Lab, University of Tokyo.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the JSK Lab nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Write the endpoint of shields.io (https://shields.io/badges/endpoint-badge) of the version of jsk_visualization of ROS One
# (https://ros.packages.techfak.net), the newest of the Packages of its distributions, for the badge of README.md:
#   python3 .github/tools/ros_one_badge.py ros_one.json
# published on the branch badges by .github/workflows/ros_one_badge.yml (from opencv_apps).

from __future__ import print_function

import json
import re
import sys
import urllib.request

REPOSITORY = 'https://ros.packages.techfak.net/dists/%s/main/binary-amd64/Packages'
DISTRIBUTIONS = ['noble', 'jammy']
PACKAGE = 'ros-one-jsk-visualization'


def versions(distribution):
    """Return the upstream versions (e.g. 3.0.0 of 3.0.0-1noble.20261116.0102) of PACKAGE in the distribution."""
    with urllib.request.urlopen(REPOSITORY % distribution, timeout=60) as response:
        packages = response.read().decode('utf-8', 'replace')
    found = []
    for stanza in packages.split('\n\n'):
        if re.search(r'^Package: %s$' % re.escape(PACKAGE), stanza, re.M):
            version = re.search(r'^Version: ([^\s-]+)', stanza, re.M)
            if version:
                found.append(version.group(1))
    return found


def main():
    found = []
    for distribution in DISTRIBUTIONS:
        try:
            found += versions(distribution)
        except OSError as e:
            print('failed to read %s: %s' % (distribution, e), file=sys.stderr)
    newest = max(found, key=lambda v: [int(n) if n.isdigit() else 0 for n in v.split('.')]) if found else None
    endpoint = {'schemaVersion': 1, 'label': 'ROS One', 'message': ('v' + newest) if newest else 'apt',
                'color': '22314E' if newest else 'lightgrey'}
    with open(sys.argv[1], 'w') as f:
        json.dump(endpoint, f)
    print(endpoint)


if __name__ == '__main__':
    main()
