#!/usr/bin/env python3
"""Compatibility entrypoint for the CI-GOV-001I admission core.

Historical router-v1 callers and router-v2 import this path. The executable
semantics now live in admission-core-v3, which repairs informational-only known
relevant closures without weakening unknown/fail-closed required-job fanout.
"""
from run_ruleset_generic_ci_admission_core_v3 import *  # noqa: F401,F403

if __name__ == "__main__":
    raise SystemExit(main())
