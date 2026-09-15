#!/usr/bin/env python3
"""Validate REGEN-Q001 qualification receipts with Python standard library only."""

from __future__ import annotations

import copy
import json
import re
import sys
from pathlib import Path
from typing import Any

SCHEMA = "mycelix.regen.qualification-receipt-v1"
SUBJECT_CLASSES = {"ProductHead", "IntegrationMerge", "ReleaseArtifact", "ExternalFixture"}
DEPENDENCY_STATES = {"ProductFrozen", "ExecutionResolved", "Unresolved"}
RESULTS = {"pass", "fail", "indeterminate"}
SHA1 = re.compile(r"^[0-9a-f]{40}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")
QUALIFIED_DIGEST = re.compile(r"^[a-z0-9][a-z0-9._-]{0,63}:[0-9a-f]+$")

TOP_KEYS = {
    "schema",
    "receipt_id",
    "subject",
    "campaign",
    "environment",
    "dependencies",
    "fixtures",
    "assertions",
    "result",
    "proposition",
    "non_claims",
    "evidence_refs",
}
SUBJECT_KEYS = {"class", "declared_identity", "observed_identity", "git"}
GIT_KEYS = {"repository", "commit_sha", "tree_sha", "parent_shas"}
CAMPAIGN_KEYS = {"provider", "workflow_ref", "run_ref", "job_ref", "executed"}
ENV_KEYS = {"runner", "operating_system", "toolchains"}
DEPENDENCY_KEYS = {"state", "identities"}
DEPENDENCY_ID_KEYS = {"kind", "digest"}
FIXTURE_KEYS = {"name", "sha256", "git_blob"}
ASSERTION_REQUIRED = {"id", "result"}
ASSERTION_ALLOWED = {"id", "result", "details"}


class ReceiptError(ValueError):
    """Receipt violates the frozen REGEN-Q001 contract."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ReceiptError(message)


def require_object(value: Any, field: str) -> dict[str, Any]:
    require(isinstance(value, dict), f"{field} must be an object")
    return value


def require_exact_keys(obj: dict[str, Any], keys: set[str], field: str) -> None:
    actual = set(obj)
    require(actual == keys, f"{field} keys mismatch: missing={sorted(keys-actual)} extra={sorted(actual-keys)}")


def require_text(value: Any, field: str, *, max_len: int = 4096) -> str:
    require(isinstance(value, str) and value != "", f"{field} must be non-empty text")
    require(len(value.encode("utf-8")) <= max_len, f"{field} exceeds {max_len} UTF-8 bytes")
    require(not any(ord(ch) < 32 for ch in value), f"{field} contains a control character")
    return value


def require_string_list(value: Any, field: str, *, min_items: int = 0, max_items: int = 128) -> list[str]:
    require(isinstance(value, list), f"{field} must be an array")
    require(min_items <= len(value) <= max_items, f"{field} item count out of bounds")
    result = [require_text(item, f"{field}[]", max_len=1024) for item in value]
    require(len(result) == len(set(result)), f"{field} contains duplicates")
    return result


def validate_receipt(receipt: Any) -> None:
    root = require_object(receipt, "receipt")
    require_exact_keys(root, TOP_KEYS, "receipt")
    require(root["schema"] == SCHEMA, "unsupported receipt schema")
    require_text(root["receipt_id"], "receipt_id", max_len=256)

    subject = require_object(root["subject"], "subject")
    require_exact_keys(subject, SUBJECT_KEYS, "subject")
    require(subject["class"] in SUBJECT_CLASSES, "unknown subject class")
    declared = require_text(subject["declared_identity"], "subject.declared_identity", max_len=512)
    observed = require_text(subject["observed_identity"], "subject.observed_identity", max_len=512)

    git = subject["git"]
    if git is not None:
        git = require_object(git, "subject.git")
        require_exact_keys(git, GIT_KEYS, "subject.git")
        require_text(git["repository"], "subject.git.repository", max_len=256)
        require(isinstance(git["commit_sha"], str) and SHA1.fullmatch(git["commit_sha"]), "invalid subject.git.commit_sha")
        require(git["tree_sha"] is None or (isinstance(git["tree_sha"], str) and SHA1.fullmatch(git["tree_sha"])), "invalid subject.git.tree_sha")
        parents = git["parent_shas"]
        require(isinstance(parents, list) and len(parents) <= 16, "subject.git.parent_shas must be a bounded array")
        require(all(isinstance(item, str) and SHA1.fullmatch(item) for item in parents), "invalid parent SHA")
        require(len(parents) == len(set(parents)), "duplicate parent SHA")

    if subject["class"] in {"ProductHead", "IntegrationMerge"}:
        require(git is not None, f"{subject['class']} requires Git identity")
        require(SHA1.fullmatch(declared) is not None, "Git subject declared_identity must be a lowercase 40-hex SHA")
        require(SHA1.fullmatch(observed) is not None, "Git subject observed_identity must be a lowercase 40-hex SHA")
        require(git["commit_sha"] == observed, "subject.git.commit_sha must equal observed_identity")

    campaign = require_object(root["campaign"], "campaign")
    require_exact_keys(campaign, CAMPAIGN_KEYS, "campaign")
    require_text(campaign["provider"], "campaign.provider", max_len=128)
    require_text(campaign["workflow_ref"], "campaign.workflow_ref", max_len=512)
    require_text(campaign["run_ref"], "campaign.run_ref", max_len=256)
    require(campaign["job_ref"] is None or isinstance(campaign["job_ref"], str), "campaign.job_ref must be text or null")
    if isinstance(campaign["job_ref"], str):
        require_text(campaign["job_ref"], "campaign.job_ref", max_len=256)
    require(isinstance(campaign["executed"], bool), "campaign.executed must be boolean")

    environment = require_object(root["environment"], "environment")
    require_exact_keys(environment, ENV_KEYS, "environment")
    require_text(environment["runner"], "environment.runner", max_len=256)
    require_text(environment["operating_system"], "environment.operating_system", max_len=256)
    require_string_list(environment["toolchains"], "environment.toolchains", max_items=64)

    dependencies = require_object(root["dependencies"], "dependencies")
    require_exact_keys(dependencies, DEPENDENCY_KEYS, "dependencies")
    require(dependencies["state"] in DEPENDENCY_STATES, "unknown dependency state")
    identities = dependencies["identities"]
    require(isinstance(identities, list) and len(identities) <= 128, "dependencies.identities must be a bounded array")
    seen_dependency_kinds: set[str] = set()
    for index, item in enumerate(identities):
        identity = require_object(item, f"dependencies.identities[{index}]")
        require_exact_keys(identity, DEPENDENCY_ID_KEYS, f"dependencies.identities[{index}]")
        kind = require_text(identity["kind"], f"dependencies.identities[{index}].kind", max_len=128)
        digest = require_text(identity["digest"], f"dependencies.identities[{index}].digest", max_len=512)
        require(QUALIFIED_DIGEST.fullmatch(digest) is not None, f"invalid qualified dependency digest {digest!r}")
        require(kind not in seen_dependency_kinds, f"duplicate dependency identity kind {kind}")
        seen_dependency_kinds.add(kind)
    if dependencies["state"] in {"ProductFrozen", "ExecutionResolved"}:
        require(bool(identities), f"{dependencies['state']} requires at least one dependency identity")

    fixtures = root["fixtures"]
    require(isinstance(fixtures, list) and len(fixtures) <= 128, "fixtures must be a bounded array")
    seen_fixture_names: set[str] = set()
    for index, item in enumerate(fixtures):
        fixture = require_object(item, f"fixtures[{index}]")
        require_exact_keys(fixture, FIXTURE_KEYS, f"fixtures[{index}]")
        name = require_text(fixture["name"], f"fixtures[{index}].name", max_len=256)
        require(name not in seen_fixture_names, f"duplicate fixture name {name}")
        seen_fixture_names.add(name)
        require(isinstance(fixture["sha256"], str) and SHA256.fullmatch(fixture["sha256"]), f"invalid fixtures[{index}].sha256")
        require(fixture["git_blob"] is None or (isinstance(fixture["git_blob"], str) and SHA1.fullmatch(fixture["git_blob"])), f"invalid fixtures[{index}].git_blob")

    assertions = root["assertions"]
    require(isinstance(assertions, list) and 1 <= len(assertions) <= 256, "assertions must be a non-empty bounded array")
    seen_assertions: set[str] = set()
    assertion_results: list[str] = []
    for index, item in enumerate(assertions):
        assertion = require_object(item, f"assertions[{index}]")
        keys = set(assertion)
        require(ASSERTION_REQUIRED <= keys <= ASSERTION_ALLOWED, f"assertions[{index}] keys are invalid")
        assertion_id = require_text(assertion["id"], f"assertions[{index}].id", max_len=256)
        require(assertion_id not in seen_assertions, f"duplicate assertion id {assertion_id}")
        seen_assertions.add(assertion_id)
        result = assertion["result"]
        require(result in RESULTS, f"invalid assertion result for {assertion_id}")
        assertion_results.append(result)
        if "details" in assertion:
            require_text(assertion["details"], f"assertions[{index}].details", max_len=2048)

    result = root["result"]
    require(result in RESULTS, "invalid receipt result")
    executed = campaign["executed"]
    if result == "pass":
        require(executed, "PASS receipt requires campaign.executed=true")
        require(all(item == "pass" for item in assertion_results), "PASS receipt contains non-PASS assertion")
        require(declared == observed, "PASS receipt declared/observed subject identity mismatch")
    elif result == "fail":
        require(executed, "FAIL receipt requires campaign.executed=true")
        require(any(item == "fail" for item in assertion_results), "FAIL receipt has no failed assertion")
    else:
        require(not (not executed and any(item == "pass" for item in assertion_results)), "unexecuted indeterminate receipt cannot contain PASS assertions")

    require_text(root["proposition"], "proposition", max_len=4096)
    require_string_list(root["non_claims"], "non_claims", min_items=1)
    require_string_list(root["evidence_refs"], "evidence_refs", min_items=1)


def load_json(path: Path) -> Any:
    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        obj: dict[str, Any] = {}
        for key, value in pairs:
            if key in obj:
                raise ReceiptError(f"duplicate JSON key {key!r}")
            obj[key] = value
        return obj

    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle, object_pairs_hook=reject_duplicate_keys)


def minimal_valid_receipt() -> dict[str, Any]:
    sha = "1" * 40
    return {
        "schema": SCHEMA,
        "receipt_id": "self-test",
        "subject": {
            "class": "ProductHead",
            "declared_identity": sha,
            "observed_identity": sha,
            "git": {
                "repository": "example/repo",
                "commit_sha": sha,
                "tree_sha": "2" * 40,
                "parent_shas": ["3" * 40],
            },
        },
        "campaign": {
            "provider": "self-test",
            "workflow_ref": "self-test",
            "run_ref": "1",
            "job_ref": "1",
            "executed": True,
        },
        "environment": {
            "runner": "self-test",
            "operating_system": "self-test",
            "toolchains": ["python self-test"],
        },
        "dependencies": {
            "state": "ExecutionResolved",
            "identities": [{"kind": "lock", "digest": "sha256:" + "4" * 64}],
        },
        "fixtures": [],
        "assertions": [{"id": "subject.exact", "result": "pass"}],
        "result": "pass",
        "proposition": "Self-test proposition.",
        "non_claims": ["No scientific claim."],
        "evidence_refs": ["self-test:1"],
    }


def run_self_test() -> None:
    base = minimal_valid_receipt()
    validate_receipt(base)

    mutations: list[tuple[str, Any]] = []

    value = copy.deepcopy(base)
    value["subject"]["observed_identity"] = "5" * 40
    value["subject"]["git"]["commit_sha"] = "5" * 40
    mutations.append(("product-head-pass-subject-mismatch", value))

    value = copy.deepcopy(base)
    value["campaign"]["executed"] = False
    mutations.append(("pass-without-execution", value))

    value = copy.deepcopy(base)
    value["assertions"].append({"id": "other", "result": "indeterminate"})
    mutations.append(("pass-with-indeterminate-assertion", value))

    value = copy.deepcopy(base)
    value["result"] = "fail"
    mutations.append(("fail-without-failed-assertion", value))

    value = copy.deepcopy(base)
    value["assertions"].append({"id": "subject.exact", "result": "pass"})
    mutations.append(("duplicate-assertion", value))

    value = copy.deepcopy(base)
    value["dependencies"]["identities"] = []
    mutations.append(("resolved-without-dependency-identity", value))

    value = copy.deepcopy(base)
    value["fixtures"] = [
        {"name": "fixture", "sha256": "6" * 64, "git_blob": None},
        {"name": "fixture", "sha256": "7" * 64, "git_blob": None},
    ]
    mutations.append(("duplicate-fixture", value))

    value = copy.deepcopy(base)
    value["unexpected"] = True
    mutations.append(("unknown-top-level-field", value))

    for name, mutation in mutations:
        try:
            validate_receipt(mutation)
        except ReceiptError:
            continue
        raise AssertionError(f"self-test mutation unexpectedly accepted: {name}")

    print(json.dumps({"result": "ok", "accepted": 1, "rejected_mutations": len(mutations)}, sort_keys=True))


def main(argv: list[str]) -> int:
    if argv == ["--self-test"]:
        run_self_test()
        return 0
    if not argv:
        print("usage: verify-regen-qualification-receipt.py [--self-test] RECEIPT.json [...]", file=sys.stderr)
        return 2

    for arg in argv:
        path = Path(arg)
        try:
            receipt = load_json(path)
            validate_receipt(receipt)
        except (OSError, json.JSONDecodeError, ReceiptError) as exc:
            print(f"{path}: INVALID: {exc}", file=sys.stderr)
            return 1
        print(f"{path}: valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
