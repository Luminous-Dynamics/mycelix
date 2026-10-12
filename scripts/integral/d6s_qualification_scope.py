#!/usr/bin/env python3
"""Classify changed paths for the D6S canonical reference qualification gate.

The self-test and production classifier share the exact same matcher so changes
to the scope policy have a deterministic, dependency-free regression check.
"""

from __future__ import annotations

import re
import sys
from collections.abc import Iterable

_RELEVANT_PATTERNS = (
    re.compile(r"^docs/integral/d6s-canon-1-.*$"),
    re.compile(r"^docs/integral/d6s-canon-2-.*$"),
    re.compile(r"^scripts/integral/verify_d6s_canon_[^/]*\.py$"),
    re.compile(r"^mycelix-manufacturing/Cargo\.(?:toml|lock)$"),
    re.compile(r"^mycelix-manufacturing/crates/(?:cos_conformance|manufacturing_common)/"),
)


def is_relevant(path: str) -> bool:
    """Return whether a repository-relative changed path affects D6S inputs."""
    return any(pattern.search(path) is not None for pattern in _RELEVANT_PATTERNS)


# These are representative source, fixture, verifier, manifest, and negative
# control paths. Keep them colocated with the matcher to prevent test drift.
_SELF_TEST_CASES = (
    ("docs/integral/d6s-canon-1-golden-vectors.json", True),
    ("docs/integral/d6s-canon-1-manifest.json", True),
    ("docs/integral/d6s-canon-2-authority-boundary-fixture.json", True),
    ("docs/integral/d6s-canon-2-manifest.json", True),
    ("scripts/integral/verify_d6s_canon_1.py", True),
    ("scripts/integral/verify_d6s_canon_2_authority_boundary.py", True),
    ("mycelix-manufacturing/Cargo.toml", True),
    ("mycelix-manufacturing/Cargo.lock", True),
    ("mycelix-manufacturing/crates/cos_conformance/tests/d6s_canon_1_golden_vectors.rs", True),
    ("mycelix-manufacturing/crates/manufacturing_common/src/lib.rs", True),
    (".github/workflows/d6s-canonical-qualification.yml", False),
    ("mycelix-desci/docs/SCIENTIFIC_CREDENTIAL_REGISTRY_PROTOCOL_RESOLUTION.md", False),
    ("mycelix-manufacturing/crates/workorders/coordinator/src/lib.rs", False),
    ("README.md", False),
    ("scripts/integral/d6s_qualification_scope.py", False),
)


def _self_test() -> int:
    failures = []
    for path, expected in _SELF_TEST_CASES:
        actual = is_relevant(path)
        if actual != expected:
            failures.append(
                f"{path!r}: expected relevant={str(expected).lower()}, "
                f"got relevant={str(actual).lower()}"
            )
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"D6S qualification scope classifier: {len(_SELF_TEST_CASES)} cases passed")
    return 0


def classify_paths(paths: Iterable[str]) -> bool:
    """Return true when any changed path matches the D6S contract scope."""
    return any(is_relevant(path) for path in paths)


def main(argv: list[str]) -> int:
    if argv == ["--self-test"]:
        return _self_test()
    if argv:
        print(f"usage: {sys.argv[0]} [--self-test]", file=sys.stderr)
        return 2
    paths = (line.rstrip("\n") for line in sys.stdin)
    print("true" if classify_paths(paths) else "false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
