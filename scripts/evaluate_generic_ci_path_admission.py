#!/usr/bin/env python3
"""Evaluate generic Mycelix CI pull-request path admission.

The v1 theorem is deliberately tri-state:

    known relevant -> AdmitKnownRelevant
    all proven irrelevant -> SkipProvenIrrelevant
    any unknown/unclassified path -> AdmitUnknown

Unknown is conservative admission, never skip.

A skip result is scheduling evidence only. It is not CI PASS, product
qualification, or proof that the changed content is harmless outside the
generic Mycelix CI workflow.
"""

from __future__ import annotations

import argparse
import copy
import fnmatch
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_ID = "generic-mycelix-ci-path-admission-v1"
AUTHORITY = "SchedulingOnly"
DECISIONS = {
    "AdmitKnownRelevant",
    "AdmitUnknown",
    "SkipProvenIrrelevant",
    "AdmissionProfileInvalid",
}


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Decision:
    disposition: str
    path_classes: dict[str, str]
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "profile_id": PROFILE_ID,
            "disposition": self.disposition,
            "path_classes": self.path_classes,
            "reason": self.reason,
            "generic_ci_required": self.disposition
            in {"AdmitKnownRelevant", "AdmitUnknown", "AdmissionProfileInvalid"},
            "grants_ci_pass": False,
            "grants_product_qualification": False,
        }


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ProfileError(message)


def _patterns(entries: Any, field: str) -> list[str]:
    require(isinstance(entries, list) and entries, f"{field} must be a non-empty list")
    out: list[str] = []
    for index, entry in enumerate(entries):
        require(isinstance(entry, dict), f"{field}[{index}] must be an object")
        pattern = entry.get("pattern")
        require(
            isinstance(pattern, str) and bool(pattern.strip()),
            f"{field}[{index}].pattern must be non-empty",
        )
        require(
            not pattern.startswith("/") and ".." not in pattern.split("/"),
            f"{field}[{index}] contains unsafe pattern {pattern!r}",
        )
        reason = entry.get("reason")
        require(
            isinstance(reason, str) and bool(reason.strip()),
            f"{field}[{index}].reason must be non-empty",
        )
        out.append(pattern)
    require(len(out) == len(set(out)), f"{field} contains duplicate patterns")
    return out


def _validate_path(path: Any) -> str:
    if not isinstance(path, str) or not path:
        raise ProfileError("changed path must be a non-empty string")
    if path.startswith("/") or "\\" in path:
        raise ProfileError(f"changed path must be repo-relative POSIX path: {path!r}")
    parts = path.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        raise ProfileError(f"changed path contains unsafe segment: {path!r}")
    return path


def _matches(path: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatchcase(path, pattern) for pattern in patterns)


def evaluate(profile: dict[str, Any], paths: list[Any]) -> Decision:
    try:
        validate_profile(profile, run_fixtures=False)
        if not isinstance(paths, list) or not paths:
            raise ProfileError("changed path set must be a non-empty list")
        normalized = [_validate_path(path) for path in paths]
    except ProfileError as exc:
        return Decision("AdmissionProfileInvalid", {}, str(exc))

    relevant = _patterns(profile["known_relevant"], "known_relevant")
    irrelevant = _patterns(profile["proven_irrelevant"], "proven_irrelevant")

    classes: dict[str, str] = {}
    for path in normalized:
        if _matches(path, relevant):
            classes[path] = "KnownRelevant"
        elif _matches(path, irrelevant):
            classes[path] = "ProvenIrrelevant"
        else:
            classes[path] = "Unknown"

    if any(value == "KnownRelevant" for value in classes.values()):
        return Decision(
            "AdmitKnownRelevant",
            classes,
            "at least one changed path is in the frozen generic-CI dependency surface",
        )
    if any(value == "Unknown" for value in classes.values()):
        return Decision(
            "AdmitUnknown",
            classes,
            "no known-relevant path matched, but at least one path is unclassified; "
            "unknown defaults to admit",
        )
    return Decision(
        "SkipProvenIrrelevant",
        classes,
        "every changed path is explicitly classified as proven irrelevant to generic CI",
    )


def validate_profile(profile: Any, *, run_fixtures: bool = True) -> None:
    require(isinstance(profile, dict), "profile root must be an object")
    require(profile.get("profile_id") == PROFILE_ID, "unexpected profile_id")
    require(profile.get("profile_version") == 1, "profile_version must be 1")
    require(profile.get("issue") == 1359, "issue must be 1359")
    require(profile.get("authority") == AUTHORITY, "authority must be SchedulingOnly")
    require(profile.get("unknown_policy") == "AdmitUnknown", "unknown policy must admit")

    frozen = profile.get("frozen_main")
    require(
        isinstance(frozen, str)
        and len(frozen) == 40
        and all(c in "0123456789abcdef" for c in frozen),
        "frozen_main must be lowercase 40-hex",
    )

    workflow = profile.get("workflow")
    require(isinstance(workflow, dict), "workflow must be an object")
    require(
        workflow.get("path") == ".github/workflows/ci.yml",
        "workflow.path must be .github/workflows/ci.yml",
    )
    blob = workflow.get("blob_sha")
    require(
        isinstance(blob, str)
        and len(blob) == 40
        and all(c in "0123456789abcdef" for c in blob),
        "workflow.blob_sha must be lowercase 40-hex",
    )

    precedence = profile.get("decision_precedence")
    require(
        precedence == ["AdmitKnownRelevant", "AdmitUnknown", "SkipProvenIrrelevant"],
        "decision_precedence must preserve relevant > unknown > skip",
    )

    relevant = _patterns(profile.get("known_relevant"), "known_relevant")
    irrelevant = _patterns(profile.get("proven_irrelevant"), "proven_irrelevant")

    required_relevant = {
        ".github/workflows/ci.yml",
        "docs/ci/generic_ci_path_admission_v1.json",
        "scripts/evaluate_generic_ci_path_admission.py",
        "mycelix-workspace/**",
        "crates/mycelix-zkp-core/**",
        "crates/luminous-sim-core/**",
        "nix/modules/holochain-base.nix",
    }
    missing = required_relevant.difference(relevant)
    require(not missing, f"required relevant patterns missing: {sorted(missing)}")

    require("docs/lex-net/**" in irrelevant, "v1 proven-irrelevant fixture is missing")
    require("docs/**" not in irrelevant, "broad docs/** exclusion is forbidden in v1")
    require("**" not in irrelevant, "universal skip pattern is forbidden")

    overlap = set(relevant).intersection(irrelevant)
    require(not overlap, f"pattern classified both relevant and irrelevant: {sorted(overlap)}")

    latent = [
        entry
        for entry in profile["known_relevant"]
        if entry.get("pattern") == "crates/luminous-sim-core/**"
    ]
    require(
        len(latent) == 1 and latent[0].get("latent") is True,
        "luminous-sim-core must remain an explicit latent relevant dependency",
    )

    closures = profile.get("job_closures")
    require(isinstance(closures, dict) and closures, "job_closures must be non-empty")
    required_jobs = {
        "format",
        "test-commons",
        "test-civic",
        "test-hearth",
        "test-finance",
        "test-finance-integration",
        "test-governance",
        "test-identity",
        "test-personal",
        "test-attribution",
        "test-bridge",
        "test-sdk",
        "test-prism",
    }
    require(set(closures) == required_jobs, "job_closures must match generic CI jobs")
    for job, patterns in closures.items():
        require(
            isinstance(patterns, list) and patterns,
            f"job_closures[{job!r}] must be a non-empty list",
        )
        for pattern in patterns:
            require(
                pattern in relevant,
                f"job_closures[{job!r}] references non-relevant pattern {pattern!r}",
            )

    nonclaims = profile.get("nonclaims")
    require(isinstance(nonclaims, list) and len(nonclaims) >= 4, "nonclaims too weak")
    text = "\n".join(str(item).lower() for item in nonclaims)
    for phrase in ("scheduling", "not ci pass", "admitunknown"):
        require(phrase in text, f"nonclaims must include {phrase!r}")

    if run_fixtures:
        fixtures = profile.get("fixtures")
        require(isinstance(fixtures, list) and fixtures, "fixtures must be non-empty")
        seen_decisions: set[str] = set()
        for index, fixture in enumerate(fixtures):
            require(isinstance(fixture, dict), f"fixtures[{index}] must be an object")
            expect = fixture.get("expect")
            require(expect in DECISIONS, f"fixtures[{index}] has unknown expect {expect!r}")
            result = evaluate(profile, fixture.get("paths"))
            require(
                result.disposition == expect,
                f"fixture {index} expected {expect}, got {result.disposition}: {result.reason}",
            )
            seen_decisions.add(expect)
        require(
            {
                "AdmitKnownRelevant",
                "AdmitUnknown",
                "SkipProvenIrrelevant",
                "AdmissionProfileInvalid",
            }.issubset(seen_decisions),
            "fixtures must cover every v1 disposition",
        )


def self_test(profile: dict[str, Any]) -> None:
    validate_profile(profile)

    bad = copy.deepcopy(profile)
    bad["unknown_policy"] = "SkipUnknown"
    try:
        validate_profile(bad)
    except ProfileError:
        pass
    else:
        raise ProfileError("self-test expected unknown-policy rejection")

    bad = copy.deepcopy(profile)
    bad["proven_irrelevant"].append(
        {"pattern": "docs/**", "reason": "too broad for v1"}
    )
    try:
        validate_profile(bad)
    except ProfileError:
        pass
    else:
        raise ProfileError("self-test expected broad docs exclusion rejection")

    result = evaluate(
        profile,
        ["docs/lex-net/readme.md", "future-shared/runtime/new.rs"],
    )
    require(result.disposition == "AdmitUnknown", "mixed unknown path must admit")

    # Semantic glob overlap is intentionally fail-safe: even if a future profile
    # contains a narrower KnownRelevant pattern inside a broader ProvenIrrelevant
    # glob, relevant classification must win. The static exact-string overlap
    # check above is not sufficient to prove this precedence property.
    overlap_profile = copy.deepcopy(profile)
    overlap_profile["known_relevant"].append(
        {
            "pattern": "docs/lex-net/runtime/**",
            "reason": "synthetic overlap fixture proving relevant-over-skip precedence",
        }
    )
    validate_profile(overlap_profile, run_fixtures=False)
    overlap_result = evaluate(
        overlap_profile,
        ["docs/lex-net/runtime/authority.rs"],
    )
    require(
        overlap_result.disposition == "AdmitKnownRelevant",
        "semantic glob overlap must prefer KnownRelevant over ProvenIrrelevant",
    )
    require(
        overlap_result.path_classes.get("docs/lex-net/runtime/authority.rs")
        == "KnownRelevant",
        "overlap path must be classified KnownRelevant",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "profile",
        nargs="?",
        default="docs/ci/generic_ci_path_admission_v1.json",
    )
    parser.add_argument("paths", nargs="*")
    parser.add_argument("--paths-json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    try:
        profile = json.loads(Path(args.profile).read_text(encoding="utf-8"))
        validate_profile(profile)
        if args.self_test:
            self_test(profile)
            print(
                json.dumps(
                    {
                        "profile_id": PROFILE_ID,
                        "self_test": "PASS",
                        "authority": AUTHORITY,
                        "grants_ci_pass": False,
                        "grants_product_qualification": False,
                    },
                    sort_keys=True,
                )
            )
            return 0

        if args.paths_json is not None:
            paths = json.loads(args.paths_json)
        else:
            paths = args.paths

        result = evaluate(profile, paths)
        print(json.dumps(result.as_dict(), sort_keys=True))

        return 2 if result.disposition == "AdmissionProfileInvalid" else 0
    except (ProfileError, json.JSONDecodeError, OSError) as exc:
        print(
            json.dumps(
                {
                    "profile_id": PROFILE_ID,
                    "disposition": "AdmissionProfileInvalid",
                    "reason": str(exc),
                    "generic_ci_required": True,
                    "grants_ci_pass": False,
                    "grants_product_qualification": False,
                },
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
