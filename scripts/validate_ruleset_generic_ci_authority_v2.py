#!/usr/bin/env python3
"""Static validator v2 for the CI-GOV-001I authority source.

V1 owns workflow/toolchain/job-census checks. V2 adds the transitive routing
binding introduced by the informational-only admission repair.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

VALIDATOR_ID = "ruleset-generic-ci-authority-static-validator-v2"
CORE = "scripts/run_ruleset_generic_ci_admission_core_v3.py"
SHIM = "scripts/run_ruleset_generic_ci_admission_v1.py"
ROUTER_V2 = "scripts/run_ruleset_generic_ci_admission_v2.py"
WORKFLOW = ".github/workflows/ruleset-generic-ci-authority.yml"
MANIFEST = "docs/ci/ruleset_generic_ci_authority_v1.json"
V1_VALIDATOR = "scripts/validate_ruleset_generic_ci_authority_v1.py"

class ValidationError(ValueError):
    pass


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValidationError(message)


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    require(spec is not None and spec.loader is not None, f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def validate(root: Path) -> dict[str, Any]:
    v1 = load_module(root / V1_VALIDATOR, "_authority_static_v1")
    manifest_path = root / MANIFEST
    workflow_path = root / WORKFLOW
    manifest = v1.load_manifest(manifest_path)
    v1_result = v1.validate(manifest, workflow_path.read_text(encoding="utf-8"))
    require(v1_result.get("valid") is True, "v1 static authority validation did not pass")

    components = manifest.get("authority_components")
    require(isinstance(components, dict), "authority_components missing")
    require(components.get("admission_core_v3") == CORE, "manifest admission_core_v3 binding missing")
    require(components.get("admission_router_v1") == SHIM, "manifest router-v1 binding drift")
    require(components.get("admission_router_v2") == ROUTER_V2, "manifest router-v2 binding drift")

    routing = manifest.get("routing_semantics")
    require(isinstance(routing, dict), "routing_semantics missing")
    require(routing.get("informational_only_known_relevant_allowed") is True,
            "informational-only known-relevant semantics not enabled")
    require(routing.get("unknown_selects_all_required") is True,
            "unknown required fanout invariant missing")
    require(routing.get("unobservable_selects_all_required_and_denies_authority") is True,
            "unobservable fail-closed invariant missing")

    core = (root / CORE).read_text(encoding="utf-8")
    shim = (root / SHIM).read_text(encoding="utf-8")
    router_v2 = (root / ROUTER_V2).read_text(encoding="utf-8")
    workflow = workflow_path.read_text(encoding="utf-8")

    compile(core, CORE, "exec")
    compile(shim, SHIM, "exec")
    compile(router_v2, ROUTER_V2, "exec")

    require("from run_ruleset_generic_ci_admission_core_v3 import *" in shim,
            "router-v1 compatibility shim does not import core-v3")
    require("raise SystemExit(main())" in shim,
            "router-v1 compatibility shim CLI forwarding missing")
    require("admitted PR selected no required jobs" not in core,
            "obsolete required-job-only admission invariant survived")
    require("admitted PR selected no jobs" in core,
            "non-empty admitted selection invariant missing")
    require("nix/modules/holochain-base.nix" in core,
            "informational-only regression fixture missing")
    require("informational_only_admission" in core,
            "informational-only self-test result marker missing")
    require("informational_only.selected_jobs == (\"test-finance-integration\",)" in core,
            "informational-only exact closure assertion missing")
    require("set(selected) == set(manifest[\"required_jobs\"])" in core,
            "unknown full-required-fanout assertion missing")

    require('parser.add_argument("--v1", default="scripts/run_ruleset_generic_ci_admission_v1.py")' in router_v2,
            "router-v2 no longer imports the compatibility path")
    require("return v1.decide(" in router_v2,
            "router-v2 no longer delegates selection semantics to compatibility core")
    require("python3 scripts/run_ruleset_generic_ci_admission_v1.py --self-test" in workflow,
            "workflow no longer self-tests compatibility/core routing")
    require("python3 scripts/run_ruleset_generic_ci_admission_v2.py --self-test" in workflow,
            "workflow router-v2 self-test missing")
    require("--receipt-output \"$RUNNER_TEMP/ruleset-admission.json\"" in workflow,
            "workflow runtime router-v2 receipt binding missing")

    return {
        "validator_id": VALIDATOR_ID,
        "valid": True,
        "v1_validator_id": v1_result.get("validator_id"),
        "authority_id": manifest.get("authority_id"),
        "admission_core": CORE,
        "informational_only_known_relevant_allowed": True,
        "unknown_full_required_fanout": True,
        "grants_merge_authority": False,
        "grants_product_qualification": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    args = parser.parse_args()
    try:
        result = validate(Path(args.root))
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
        return 0
    except (ValidationError, OSError, json.JSONDecodeError, SyntaxError) as exc:
        print(json.dumps({
            "validator_id": VALIDATOR_ID,
            "valid": False,
            "reason": str(exc),
            "grants_merge_authority": False,
            "grants_product_qualification": False,
        }, sort_keys=True, separators=(",", ":")))
        return 2

if __name__ == "__main__":
    raise SystemExit(main())
