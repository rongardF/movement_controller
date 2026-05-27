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
pytest loads it before test collection begins.

Problem: Python's default sys.path always contains '' (the empty string), which
resolves to the current working directory at import time.  When pytest is invoked
from within the source root (e.g. `python -m pytest` or `colcon test`), CWD is
`src/movement_controller/`, so Python finds `movement_controller/` (the bare source
copy, lacking generated action/msg/srv interfaces) *before* the colcon-installed
merged package at `install/.../site-packages/`.  This hook removes the CWD-based
entries from sys.path so the installed package is found instead.

Note: This is independent of --import-mode.  Even under --import-mode=importlib
(the configured mode in setup.cfg), test source files are imported via importlib
without needing the rootdir on sys.path — but ordinary `import movement_controller`
statements inside those tests still use the standard import machinery and are
affected by sys.path.
"""
import os
import sys

_CONFIGURED = False


def pytest_configure(config):  # noqa: ARG001
    """Remove package source root from sys.path before test collection starts."""
    global _CONFIGURED
    if _CONFIGURED:
        return
    _CONFIGURED = True

    # This file lives at the rootdir; __file__ gives us that path directly.
    pkg_root = os.path.dirname(os.path.abspath(__file__))

    # Remove sys.path entries that resolve to this source root (prevents the bare
    # source package from shadowing the installed one via the CWD '' entry).
    sys.path[:] = [
        p for p in sys.path
        if p != ''
        and not (p and os.path.realpath(p) == os.path.realpath(pkg_root))
    ]

    # Evict any movement_controller modules already imported from the source root
    # so the next import resolves to the installed (generated) package instead.
    stale = [
        k for k, mod in sys.modules.items()
        if (k == 'movement_controller' or k.startswith('movement_controller.'))
        and getattr(mod, '__file__', None)
        and os.path.realpath(getattr(mod, '__file__', ''))
           .startswith(os.path.realpath(pkg_root))
    ]
    for key in stale:
        del sys.modules[key]
