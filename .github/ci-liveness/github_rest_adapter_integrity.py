#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import importlib.util
import json
import tempfile
from pathlib import Path
from typing import Any

SCHEMA = "mycelix-github-rest-evidence-ci-receipt-integrity-v1"
IMPLEMENTATION_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_RECEIPT_INTEGRITY_IMPL_V1\0"
INTEGRITY_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_RECEIPT_INTEGRITY_V1\0"
CORE_OBSERVATION_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_OBSERVATION_V1\0"
POLICY_DOMAIN = b"MYCELIX_GITHUB_REST_EVIDENCE_CI_CORE_ADAPTER_POLICY_V1\0"

POLICY_GIT_BLOB = "2bfbaba7696347cce016cd2344b7127af873c9d8"
POLICY_IMPLEMENTATION = "754030d5cd6557975c34032d3433167043c570818db25feae90d812ddb51ccb2"
CORE_ADAPTER_GIT_BLOB = "603fd2b77bc588701dbcdae376ae31f5b49ce13b"
CORE_ADAPTER_IMPLEMENTATION = "68fdfa639868b9300aad36cfa84dede8b2056cc280cd223d9e2d97a76cefa1bf"
SEMANTIC_CORE_HEAD = "4190f855eb0f3c03a7a6b0decee84dd7edba07b4"

CORE_RECEIPT_FIELDS = {
    "schema",
    "adapter_profile_id",
    "adapter_profile_commitment_sha256",
    "adapter_implementation_commitment_sha256",
    "supported_core_head",
    "observed_at_utc",
    "core_manifest_v1",
    "core_observation_v1",
    "provider_diagnostics",
    "observation_source_authenticity_verified",
    "github_api_response_authenticity_verified",
    "runner_identity_attested",
    "semantic_classification_performed",
    "qualification_result",
    "theorem_result",
    "qualification_authority",
    "evidence_authority",
    "failover_authority",
    "rerun_authority",
    "dispatch_authority",
}
POLICY_FIELDS = {
    "adapter_policy_schema",
    "adapter_policy_implementation_commitment_sha256",
    "adapter_core_git_blob_sha1",
    "adapter_core_contract_validated",
    "adapter_policy_commitment_sha256",
}
FULL_INPUT_FIELDS = CORE_RECEIPT_FIELDS | {"adapter_observation_commitment_sha256"} | POLICY_FIELDS

class IntegrityError(ValueError):
    pass

_IMPL_BYTES = Path(__file__).read_bytes()
IMPLEMENTATION_COMMITMENT_SHA256 = hashlib.sha256(
    IMPLEMENTATION_DOMAIN + _IMPL_BYTES
).hexdigest()
del _IMPL_BYTES

def _git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

def _safe_json(value: Any, context: str = "value") -> None:
    if value is None or isinstance(value, (str, bool)):
        return
    if isinstance(value, int) and not isinstance(value, bool):
        if abs(value) > (1 << 53) - 1:
            raise IntegrityError(f"{context} integer outside safe range")
        return
    if isinstance(value, float):
        raise IntegrityError(f"{context} float not accepted")
    if isinstance(value, list):
        for i, item in enumerate(value):
            _safe_json(item, f"{context}[{i}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise IntegrityError(f"{context} key must be string")
            _safe_json(item, f"{context}.{key}")
        return
    raise IntegrityError(f"{context} unsupported value")

def canonical_json(value: Any) -> bytes:
    _safe_json(value)
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode()

def _load_exact_module(path: Path, expected_blob: str, module_name: str):
    data = path.read_bytes()
    if _git_blob(data) != expected_blob:
        raise IntegrityError(f"{module_name} blob mismatch")
    with tempfile.TemporaryDirectory(prefix="mycelix-ci-integrity-") as td:
        frozen = Path(td) / f"{module_name}.py"
        frozen.write_bytes(data)
        if frozen.read_bytes() != data or _git_blob(frozen.read_bytes()) != expected_blob:
            raise IntegrityError(f"{module_name} materialization mismatch")
        spec = importlib.util.spec_from_file_location(module_name, frozen)
        if spec is None or spec.loader is None:
            raise IntegrityError(f"cannot load {module_name}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        post = frozen.read_bytes()
        if post != data or _git_blob(post) != expected_blob:
            raise IntegrityError(f"{module_name} changed during import")
        return mod

def _verify_result(result: Any) -> dict[str, Any]:
    if not isinstance(result, dict) or set(result) != FULL_INPUT_FIELDS:
        raise IntegrityError("policy result field mismatch")
    if result.get("adapter_implementation_commitment_sha256") != CORE_ADAPTER_IMPLEMENTATION:
        raise IntegrityError("core implementation commitment mismatch")
    if result.get("supported_core_head") != SEMANTIC_CORE_HEAD:
        raise IntegrityError("semantic core head mismatch")
    if result.get("adapter_policy_implementation_commitment_sha256") != POLICY_IMPLEMENTATION:
        raise IntegrityError("policy implementation commitment mismatch")
    if result.get("adapter_core_git_blob_sha1") != CORE_ADAPTER_GIT_BLOB:
        raise IntegrityError("policy core blob mismatch")
    if result.get("adapter_core_contract_validated") is not True:
        raise IntegrityError("core contract not validated")

    core_receipt = {k: result[k] for k in CORE_RECEIPT_FIELDS}
    expected_core = hashlib.sha256(
        CORE_OBSERVATION_DOMAIN + canonical_json(core_receipt)
    ).hexdigest()
    if result["adapter_observation_commitment_sha256"] != expected_core:
        raise IntegrityError("projection receipt commitment mismatch")

    policy_payload = {
        k: v for k, v in result.items() if k != "adapter_policy_commitment_sha256"
    }
    expected_policy = hashlib.sha256(
        POLICY_DOMAIN + canonical_json(policy_payload)
    ).hexdigest()
    if result["adapter_policy_commitment_sha256"] != expected_policy:
        raise IntegrityError("policy commitment mismatch")

    for key in (
        "observation_source_authenticity_verified",
        "github_api_response_authenticity_verified",
        "runner_identity_attested",
        "semantic_classification_performed",
        "qualification_authority",
        "evidence_authority",
        "failover_authority",
        "rerun_authority",
        "dispatch_authority",
    ):
        if result.get(key) is not False:
            raise IntegrityError(f"authority ceiling broadened: {key}")
    if result.get("qualification_result") is not None or result.get("theorem_result") is not None:
        raise IntegrityError("semantic result surfaced by provider stack")
    return result

def adapt(
    run_raw: Any,
    jobs_raw: Any,
    profile_raw: Any,
    observed_at_utc: str,
    policy_path: Path | None = None,
    core_path: Path | None = None,
) -> dict[str, Any]:
    here = Path(__file__).parent
    pp = policy_path or here / "github_rest_adapter_policy.py"
    cp = core_path or here / "github_rest_adapter.py"
    policy = _load_exact_module(pp, POLICY_GIT_BLOB, "github_rest_adapter_policy")
    if getattr(policy, "IMPLEMENTATION_COMMITMENT_SHA256", None) != POLICY_IMPLEMENTATION:
        raise IntegrityError("policy implementation identity mismatch")
    result = policy.adapt(run_raw, jobs_raw, profile_raw, observed_at_utc, core_path=cp)
    result = _verify_result(result)
    out = dict(result)
    out["adapter_integrity_schema"] = SCHEMA
    out["adapter_integrity_implementation_commitment_sha256"] = IMPLEMENTATION_COMMITMENT_SHA256
    out["adapter_core_observation_commitment_verified"] = True
    out["adapter_policy_commitment_verified"] = True
    payload = dict(out)
    out["adapter_integrity_commitment_sha256"] = hashlib.sha256(
        INTEGRITY_DOMAIN + canonical_json(payload)
    ).hexdigest()
    return out
