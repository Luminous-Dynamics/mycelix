# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

"""Structural and adversarial tests for the machine/workload binding v1 contract."""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import jsonschema
import pytest


IDENTITY_DOCS = Path(__file__).resolve().parents[3] / "docs" / "identity"
SCHEMA_PATH = IDENTITY_DOCS / "machine-binding-v1.schema.json"
EXAMPLE_PATH = IDENTITY_DOCS / "machine-binding-v1.example.json"


@pytest.fixture(scope="module")
def schema() -> dict[str, Any]:
    value = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(value)
    return value


@pytest.fixture(scope="module")
def example() -> dict[str, Any]:
    return json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def validator(schema: dict[str, Any]) -> jsonschema.Draft202012Validator:
    return jsonschema.Draft202012Validator(
        schema,
        format_checker=jsonschema.FormatChecker(),
    )


def test_example_conforms_to_draft_2020_12(
    validator: jsonschema.Draft202012Validator,
    example: dict[str, Any],
) -> None:
    errors = sorted(validator.iter_errors(example), key=lambda error: list(error.absolute_path))
    assert not errors, "\n".join(error.message for error in errors)


def test_binding_signature_is_structurally_present(example: dict[str, Any]) -> None:
    signature = example["binding_signature"]
    assert signature["context"] == "org.mycelix.machine-binding.v1"
    assert signature["encoding"] == "RFC8785-JCS"
    assert signature["algorithm"] == "Ed25519"
    assert len(signature["payload_sha256"]) == 64
    # The example signature is only a shape fixture; crypto verification is a
    # separate verifier responsibility and is intentionally not implied here.
    assert len(signature["signature_base64"]) == 88


@pytest.mark.parametrize(
    ("case_name", "mutate"),
    [
        (
            "unknown-root-field",
            lambda value: value.update({"implicit_authorization": True}),
        ),
        (
            "bound-workload-without-svid",
            lambda value: value["workload"].update(
                {"state": "bound", "mechanism": "spiffe-x509-svid"}
            ),
        ),
        (
            "verified-attestation-without-independent-evidence",
            lambda value: value["hardware_attestation"].update(
                {"status": "verified", "mechanism": "tpm2-quote"}
            ),
        ),
        (
            "revoked-controller-without-lifecycle-receipt",
            lambda value: value["controller"].update({"status": "revoked"}),
        ),
        (
            "revoked-key-without-lifecycle-receipt",
            lambda value: value["key_epoch"].update({"state": "revoked"}),
        ),
        (
            "malformed-closure-digest",
            lambda value: value["software"].update(
                {"system_closure_sha256": "not-a-sha256"}
            ),
        ),
        (
            "decommissioned-subject-without-lifecycle-receipt",
            lambda value: value["subject"].update({"status": "decommissioned"}),
        ),
    ],
)
def test_negative_controls_fail_closed(
    validator: jsonschema.Draft202012Validator,
    example: dict[str, Any],
    case_name: str,
    mutate: Any,
) -> None:
    candidate = copy.deepcopy(example)
    mutate(candidate)
    errors = list(validator.iter_errors(candidate))
    assert errors, f"negative control unexpectedly validated: {case_name}"


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def test_duplicate_json_object_keys_are_rejected() -> None:
    raw = EXAMPLE_PATH.read_text(encoding="utf-8")
    duplicate = raw.replace('"version": 1,', '"version": 1,\n  "version": 1,', 1)
    assert duplicate != raw
    with pytest.raises(ValueError, match="duplicate JSON key"):
        json.loads(duplicate, object_pairs_hook=_reject_duplicate_pairs)
