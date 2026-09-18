#!/usr/bin/env python3
"""Static validator for CI-GOV-001I ruleset generic CI authority v1.

Standard-library only. This validates the frozen source graph; it does not claim
that GitHub has activated or executed the required-workflow ruleset.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

VALIDATOR_ID = "ruleset-generic-ci-authority-static-validator-v1"
EXPECTED_WORKFLOW = ".github/workflows/ruleset-generic-ci-authority.yml"
EXPECTED_MANIFEST = "docs/ci/ruleset_generic_ci_authority_v1.json"
EXPECTED_REQUIRED = [
    "format","test-commons","test-civic","test-hearth","test-finance",
    "test-governance","test-identity","test-personal","test-attribution",
    "test-bridge","test-sdk","test-prism",
]
EXPECTED_INFORMATIONAL = ["test-finance-integration"]
EXPECTED_SELECTORS = {
    "format":"format",
    "test-commons":"commons",
    "test-civic":"civic",
    "test-hearth":"hearth",
    "test-finance":"finance",
    "test-finance-integration":"finance_integration",
    "test-governance":"governance",
    "test-identity":"identity",
    "test-personal":"personal",
    "test-attribution":"attribution",
    "test-bridge":"bridge",
    "test-sdk":"sdk",
    "test-prism":"prism",
}
HEX40 = re.compile(r"^[0-9a-f]{40}$")


class ValidationError(ValueError):
    pass


def require(ok: bool, message: str) -> None:
    if not ok:
        raise ValidationError(message)


def load_manifest(path: Path) -> dict[str, Any]:
    root = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(root, dict), "manifest root must be object")
    require(root.get("authority_id") == "ruleset-generic-ci-authority-v1", "manifest identity drift")
    require(root.get("issue") == 1685, "manifest issue drift")
    require(root.get("authority") == "MergeAuthorityCandidate", "manifest authority drift")
    require(root.get("required_workflow_path") == EXPECTED_WORKFLOW, "required workflow path drift")
    require(root.get("events") == ["pull_request","merge_group"], "event set drift")
    require(root.get("required_jobs") == EXPECTED_REQUIRED, "required job order/census drift")
    require(root.get("informational_jobs") == EXPECTED_INFORMATIONAL, "informational job census drift")
    runner = root.get("runner")
    require(isinstance(runner, dict), "runner manifest missing")
    require(runner.get("scheduler") == "ubuntu-24.04" and runner.get("product") == "ubuntu-24.04",
            "runner family drift")
    toolchains = root.get("toolchains")
    require(isinstance(toolchains, dict), "toolchains manifest missing")
    require(toolchains.get("rust") == "1.96.0", "Rust version drift")
    require(toolchains.get("node") == "20.20.2", "Node version drift")
    require(toolchains.get("determinate_nix") == "3.22.4", "Determinate Nix version drift")
    require(toolchains.get("nix_installer_revision") == "2f5c8e53e0caa0857114853ee1d4dc8d07d96dd3",
            "Nix installer revision drift")
    require(toolchains.get("cargo_locked_required") is True, "Cargo locked policy missing")
    require(toolchains.get("required_rust_target_cache") == "disabled-v1",
            "required Rust cache policy drift")
    trust = root.get("trust_model")
    require(isinstance(trust, dict), "trust model missing")
    require(trust.get("authority_checkout") == "github.workflow_sha", "authority checkout drift")
    require(trust.get("target_checkout") == "github.sha", "target checkout drift")
    require(trust.get("persist_credentials") is False, "persistent checkout credentials forbidden")
    require(trust.get("required_jobs_cross_run_target_cache") is False,
            "required job target cache must remain disabled")
    baseline = root.get("baseline_generic_ci")
    require(isinstance(baseline, dict), "baseline generic CI binding missing")
    require(HEX40.fullmatch(str(baseline.get("main_sha",""))) is not None, "baseline main SHA invalid")
    require(HEX40.fullmatch(str(baseline.get("workflow_blob",""))) is not None, "baseline workflow blob invalid")
    require(baseline.get("workflow_path") == ".github/workflows/ci.yml", "baseline workflow path drift")
    impl = root.get("implementation_parent")
    require(isinstance(impl, str) and HEX40.fullmatch(impl) is not None, "implementation parent invalid")
    actions = root.get("external_actions")
    require(isinstance(actions, dict) and actions, "external action lock missing")
    for action, sha in actions.items():
        require(isinstance(action, str) and action.count("/") == 1, f"bad action identity {action!r}")
        require(isinstance(sha, str) and HEX40.fullmatch(sha) is not None, f"bad action SHA {action}")
    return root


def job_ids(text: str) -> list[str]:
    lines = text.splitlines()
    try:
        start = lines.index("jobs:") + 1
    except ValueError as exc:
        raise ValidationError("jobs root missing") from exc
    out = []
    for line in lines[start:]:
        m = re.fullmatch(r"  ([A-Za-z0-9_-]+):", line)
        if m:
            out.append(m.group(1))
    return out


def action_uses(text: str) -> list[tuple[str,str]]:
    out=[]
    for match in re.finditer(r"(?m)^\s*uses:\s*([A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+)@([^\s#]+)", text):
        out.append((match.group(1),match.group(2)))
    return out


def validate(manifest: dict[str,Any], text: str) -> dict[str,Any]:
    require(text.startswith("name: Ruleset Generic CI Authority\n\non:\n  pull_request:\n  merge_group:\n"),
            "event header drift")
    require("pull_request_target:" not in text, "pull_request_target forbidden")
    require("workflow_dispatch:" not in text, "workflow_dispatch forbidden in merge authority v1")
    require("\n  push:" not in text, "push event forbidden in merge authority v1")
    require("cancel-in-progress:" not in text, "cancel-in-progress forbidden")
    require("\nconcurrency:" not in text, "workflow concurrency forbidden in authority v1")
    require("secrets." not in text, "secret use forbidden")
    require("permissions:\n  contents: read\n  pull-requests: read\n" in text, "least-privilege permission block drift")
    require(re.search(r"(?m)^\s+[A-Za-z-]+:\s+write\s*$", text) is None, "write permission forbidden")

    actual_jobs = job_ids(text)
    expected_jobs = ["changes"] + EXPECTED_REQUIRED[:5] + ["test-finance-integration"] + EXPECTED_REQUIRED[5:] + ["ci-pass"]
    require(actual_jobs == expected_jobs, f"job census/order drift: {actual_jobs!r}")

    actions = manifest["external_actions"]
    seen=set()
    for identity, sha in action_uses(text):
        require(HEX40.fullmatch(sha) is not None, f"unpinned action {identity}@{sha}")
        require(identity in actions, f"action absent from manifest: {identity}")
        require(actions[identity] == sha, f"action SHA drift for {identity}")
        seen.add(identity)
    require(seen == set(actions), f"manifest/workflow action usage mismatch: missing={set(actions)-seen}")

    runs_on = re.findall(r"(?m)^\s*runs-on:\s*(\S+)\s*$", text)
    require(runs_on and set(runs_on) == {"ubuntu-24.04"}, f"runner label drift: {set(runs_on)!r}")
    require("ubuntu-latest" not in text, "moving ubuntu-latest label forbidden")
    require("Swatinem/rust-cache" not in text, "cross-run Rust target cache forbidden")

    require(text.count("ref: ${{ github.workflow_sha }}") == 2, "authority checkout count drift")
    require(text.count("path: .authority") == 2, "authority checkout path count drift")
    target_refs = text.count("ref: ${{ github.sha }}") + text.count("ref: '${{ github.sha }}'")
    require(target_refs == 13, f"target checkout count drift: {target_refs}")
    require(text.count("persist-credentials: false") == 15, "checkout credential persistence count drift")

    require("python3 scripts/run_ruleset_generic_ci_admission_v2.py \\" in text,
            "router-v2 runtime wiring missing")
    require("python3 scripts/run_ruleset_generic_ci_admission_v2.py --self-test" in text,
            "router-v2 self-test wiring missing")
    require(text.count("python3 scripts/evaluate_ruleset_generic_ci_summary_v2.py") >= 2,
            "summary-v2 wiring missing")
    require("test \"$(git -C .authority rev-parse HEAD)\" = \"$GITHUB_WORKFLOW_SHA\"" in text,
            "workflow_sha checkout binding missing")
    require("GITHUB_TOKEN: ${{ github.token }}" in text, "trusted PR observation token binding missing")

    for job, selector in EXPECTED_SELECTORS.items():
        require(f"  {job}:" in text, f"job {job} missing")
        require(f"if: needs.changes.outputs.{selector} == 'true'" in text,
                f"selector drift for {job}")

    require("toolchain: 1.96.0" in text or "toolchain: '1.96.0'" in text, "Rust 1.96.0 missing")
    require("dtolnay/rust-toolchain@stable" not in text, "moving Rust stable forbidden")
    require("node-version: 20.20.2" in text, "Node 20.20.2 missing")
    require('test "$(node --version)" = "v20.20.2"' in text, "Node runtime verification missing")

    for line in text.splitlines():
        s=line.strip()
        if re.search(r"\bcargo (test|build|check)\b", s) and "cargo fmt" not in s:
            require("--locked" in s or s.endswith("\\") or "nix develop" in s,
                    f"Cargo lock policy missing from command line: {s!r}")
    require("cargo test --workspace --locked" in text, "locked required tests absent")
    require("cargo build --locked --target wasm32-unknown-unknown" in text,
            "locked Prism wasm build absent")
    require("cargo check --locked -p prism-tauri" in text, "locked Prism Tauri check absent")

    require(text.count("continue-on-error: true") == 1, "continue-on-error surface drift")
    require("id: typescript\n        continue-on-error: true" in text,
            "only baseline TypeScript measurement may continue on error")
    require("typescript_observation: ${{ steps.typescript.outcome }}" in text,
            "TypeScript observation output missing")

    require("observation: ${{ steps.measure.outputs.observation }}" in text,
            "Finance Integration observation output missing")
    require("echo \"observation=$observation\" >> \"$GITHUB_OUTPUT\"" in text,
            "Finance Integration typed observation emission missing")
    require("exit 0\n\n  test-governance:" in text,
            "Finance Integration measurement wrapper must terminate scheduler-successfully")
    require("Determinate Nix 3.22.4" in text, "Nix runtime verification missing")
    require("source-revision: 2f5c8e53e0caa0857114853ee1d4dc8d07d96dd3" in text,
            "Nix installer revision missing")

    require("prism-zkp prism-knowledge-bridge prism-bridge prism-shell prism-ui; do" in text,
            "Prism test census drift")

    ci_pass_tail = text.split("\n  ci-pass:\n",1)[1]
    for name in ["changes"] + EXPECTED_REQUIRED + EXPECTED_INFORMATIONAL:
        require(f"      - {name}\n" in ci_pass_tail, f"ci-pass needs missing {name}")

    return {
        "validator_id": VALIDATOR_ID,
        "valid": True,
        "authority_id": manifest["authority_id"],
        "required_job_count": len(EXPECTED_REQUIRED),
        "informational_job_count": len(EXPECTED_INFORMATIONAL),
        "job_count": len(actual_jobs),
        "action_lock_count": len(actions),
        "runner": "ubuntu-24.04",
        "rust": manifest["toolchains"]["rust"],
        "node": manifest["toolchains"]["node"],
        "grants_merge_authority": False,
        "grants_product_qualification": False,
    }


def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--manifest",default=EXPECTED_MANIFEST)
    parser.add_argument("--workflow",default=EXPECTED_WORKFLOW)
    args=parser.parse_args()
    try:
        manifest=load_manifest(Path(args.manifest))
        result=validate(manifest,Path(args.workflow).read_text(encoding="utf-8"))
        print(json.dumps(result,sort_keys=True,separators=(",",":")))
        return 0
    except (ValidationError,OSError,json.JSONDecodeError) as exc:
        print(json.dumps({
            "validator_id":VALIDATOR_ID,
            "valid":False,
            "reason":str(exc),
            "grants_merge_authority":False,
            "grants_product_qualification":False,
        },sort_keys=True,separators=(",",":")))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
