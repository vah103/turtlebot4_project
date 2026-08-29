#!/usr/bin/env python3
"""ROS Jazzy launcher for MapEx with LaMa isolated in its legacy Python env."""

import mapex
from mapex_lama_bridge import LamaEnsembleBridge


# MapExExplorer resolves the module-global LamaEnsemble at construction time.
# Replace only the inference backend; all policy/visibility/Nav2 logic remains in
# mapex.py and nf_basic.py.
mapex.LamaEnsemble = LamaEnsembleBridge


if __name__ == "__main__":
    mapex.main()
