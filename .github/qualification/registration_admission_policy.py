#!/usr/bin/env python3
"""Pinned producer policy for non-authoritative qualification registration admission."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from types import ModuleType
from typing import Any

POLICY_SCHEMA = "mycelix-qualification-registration-admission-policy-v1"
POLICY_DOMAIN = b"MYCELIX_QUALIFICATION_REGISTRATION_ADMISSION_POLICY_V1\0"
POLICY_IMPL_DOMAIN = b"MYCELIX_QUALIFICATION_REGISTRATION_ADMISSION_POLICY_IMPLEMENTATION_V1\0"
CORE_IMPL_DOMAIN = b"MYCELIX_QUALIFICATION_REGISTRATION_ADMISSION_IMPLEMENTATION_V1\0"

SUPPORTED_CORE_GIT_BLOB_SHA1 = "919484245c7e7d92800ed7760d83c68e775abd8d"
SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT = "1e965637f95566e286c5c5c01f4ac7e7b0137368c5dffd27c11bf63119759ecc"
SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT = "1038c07ed7529dff979edba34f44428a759ff6e7fb85a74b862ba75701ec421e"
SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT = "89838000d0f673b669fbf00ad648a86c54638e2c637a73760db54d1cd8cae855"

_IMPL_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT = hashlib.sha256(POLICY_IMPL_DOMAIN + _IMPL_BYTES).hexdigest()
del _IMPL_BYTES


class PolicyError(RuntimeError):
    pass


def _git_blob_sha1(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode("ascii") + b"\0" + data).hexdigest()


def _core_commitment(data: bytes) -> str:
    return hashlib.sha256(CORE_IMPL_DOMAIN + data).hexdigest()


def _verify_core_file(
    path: Path,
    *,
    expected_blob: str = SUPPORTED_CORE_GIT_BLOB_SHA1,
    expected_commitment: str = SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT,
) -> bytes:
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise PolicyError("cannot read registration admission core") from exc
    if _git_blob_sha1(data) != expected_blob:
        raise PolicyError("registration admission core Git blob mismatch")
    if _core_commitment(data) != expected_commitment:
        raise PolicyError("registration admission core implementation mismatch")
    return data


def _load_core() -> tuple[ModuleType, Path]:
    path = Path(__file__).with_name("registration_admission.py")
    _verify_core_file(path)
    spec = importlib.util.spec_from_file_location("registration_admission_core", path)
    if spec is None or spec.loader is None:
        raise PolicyError("cannot construct registration admission core loader")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        raise PolicyError("cannot import registration admission core") from exc
    if getattr(module, "IMPLEMENTATION_COMMITMENT", None) != SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT:
        raise PolicyError("loaded registration admission core identity mismatch")
    return module, path


def _core_still_matches(path: Path) -> None:
    _verify_core_file(path)


def _load_inputs(
    core: ModuleType,
    receipt_path: Path,
    intent_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    try:
        receipt = core._load(receipt_path, "preflight receipt")
        intent = core._load(intent_path, "registration intent")
    except Exception as exc:
        admission_error = getattr(core, "AdmissionError", None)
        if admission_error is not None and isinstance(exc, admission_error):
            raise PolicyError(str(exc)) from exc
        raise
    if not isinstance(receipt, dict) or not isinstance(intent, dict):
        raise PolicyError("registration core loader returned non-object input")
    return receipt, intent


def _require_supported_producers(receipt: dict[str, Any], intent: dict[str, Any]) -> None:
    if intent.get("preflight_implementation_commitment") != SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT:
        raise PolicyError("unsupported preflight implementation in registration intent")
    if intent.get("environment_adapter_implementation_commitment") != SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT:
        raise PolicyError("unsupported environment adapter implementation in registration intent")

    if receipt.get("classification") == "ELIGIBLE":
        if receipt.get("preflight_implementation_commitment") != SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT:
            raise PolicyError("unsupported preflight implementation in eligible receipt")
        if receipt.get("bound_preflight_implementation_commitment") != SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT:
            raise PolicyError("unsupported bound preflight implementation in eligible receipt")
        if receipt.get("environment_adapter_implementation_commitment") != SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT:
            raise PolicyError("unsupported environment adapter implementation in eligible receipt")
        environment = receipt.get("preflight_environment")
        if not isinstance(environment, dict):
            raise PolicyError("eligible receipt lacks preflight environment")
        if environment.get("adapter_implementation_commitment") != SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT:
            raise PolicyError("unsupported embedded environment adapter implementation")


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def _policy_commitment(value: dict[str, Any]) -> str:
    return hashlib.sha256(POLICY_DOMAIN + _canonical(value)).hexdigest()


def _authority_ceiling(result: dict[str, Any]) -> None:
    for field in (
        "registration_authority",
        "workflow_dispatched",
        "workflow_identity_verified",
        "qualification_authority",
    ):
        if result.get(field) is not False:
            raise PolicyError(f"registration core broadened authority: {field}")
    if result.get("qualification_result") is not None:
        raise PolicyError("registration core attempted qualification result")


def evaluate_with_core(
    core: ModuleType,
    core_path: Path | None,
    receipt: dict[str, Any],
    intent: dict[str, Any],
) -> dict[str, Any]:
    _require_supported_producers(receipt, intent)
    try:
        inner = core.evaluate(receipt, intent)
    except Exception as exc:
        admission_error = getattr(core, "AdmissionError", None)
        if admission_error is not None and isinstance(exc, admission_error):
            raise PolicyError(str(exc)) from exc
        raise
    if core_path is not None:
        _core_still_matches(core_path)
    if not isinstance(inner, dict):
        raise PolicyError("registration core result is not an object")
    _authority_ceiling(inner)
    if inner.get("classification") not in {"ADMISSIBLE_TO_REQUEST", "REFUSED"}:
        raise PolicyError("registration core returned unsupported classification")

    result = dict(inner)
    core_impl = result.pop("registration_admission_implementation_commitment", None)
    if core_impl != SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT:
        raise PolicyError("registration core result implementation identity mismatch")
    core_admission = result.pop("admission_commitment", None)

    result["schema"] = POLICY_SCHEMA
    result["registration_admission_policy_implementation_commitment"] = IMPLEMENTATION_COMMITMENT
    result["registration_admission_core_implementation_commitment"] = SUPPORTED_CORE_IMPLEMENTATION_COMMITMENT
    result["supported_preflight_implementation_commitment"] = SUPPORTED_PREFLIGHT_IMPLEMENTATION_COMMITMENT
    result["supported_environment_adapter_implementation_commitment"] = SUPPORTED_ENVIRONMENT_ADAPTER_IMPLEMENTATION_COMMITMENT
    result["producer_revision_supported"] = True
    result["receipt_authenticity_verified"] = False
    if core_admission is not None:
        result["registration_admission_core_commitment"] = core_admission
    if result["classification"] == "ADMISSIBLE_TO_REQUEST":
        result["registration_admission_policy_commitment"] = _policy_commitment(result)
    return result


def evaluate(receipt: dict[str, Any], intent: dict[str, Any]) -> dict[str, Any]:
    core, core_path = _load_core()
    return evaluate_with_core(core, core_path, receipt, intent)


def _invalid(reason: str) -> dict[str, Any]:
    return {
        "schema": POLICY_SCHEMA,
        "registration_admission_policy_implementation_commitment": IMPLEMENTATION_COMMITMENT,
        "classification": "INVALID",
        "reason": reason,
        "producer_revision_supported": False,
        "receipt_authenticity_verified": False,
        "registration_authority": False,
        "workflow_dispatched": False,
        "workflow_identity_verified": False,
        "qualification_result": None,
        "qualification_authority": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--intent", required=True)
    args = parser.parse_args()
    try:
        core, core_path = _load_core()
        receipt, intent = _load_inputs(core, Path(args.receipt), Path(args.intent))
        result = evaluate_with_core(core, core_path, receipt, intent)
    except (OSError, PolicyError) as exc:
        result = _invalid(str(exc))
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), ensure_ascii=False))
    return {"ADMISSIBLE_TO_REQUEST": 0, "REFUSED": 2, "INVALID": 4}.get(result["classification"], 4)


if __name__ == "__main__":
    raise SystemExit(main())
