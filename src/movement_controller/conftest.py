# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
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
"""
Root conftest.py — ensure installed (generated) interfaces take priority over source.

This file MUST live at the pytest rootdir (same directory as setup.cfg) so that
pytest loads it before adding the rootdir to sys.path in prepend mode.

When pytest's --import-mode=prepend adds this rootdir to sys.path[0], it causes
`movement_controller/` (the source package, without generated action/msg/srv) to
shadow the colcon-installed merged package. This hook removes the rootdir from
sys.path so that the installed package is found instead.
"""
import os
import sys


def pytest_configure(config):  # noqa: ARG001
    """Remove package source root from sys.path before test collection starts."""
    # This file lives at the rootdir; __file__ gives us that path directly.
    pkg_root = os.path.dirname(os.path.abspath(__file__))
    cwd = os.getcwd()

    # Remove all sys.path entries pointing to this source root so that the
    # installed (generated) interfaces take priority over the source package.
    sys.path[:] = [
        p for p in sys.path
        if p not in ('', cwd, pkg_root)
        and not (p and os.path.realpath(p) == os.path.realpath(pkg_root))
    ]

    # Evict any movement_controller already imported from the source directory
    stale = [
        k for k in sys.modules
        if k == 'movement_controller' or k.startswith('movement_controller.')
    ]
    for key in stale:
        del sys.modules[key]
