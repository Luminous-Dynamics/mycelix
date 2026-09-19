#!/usr/bin/env python3
"""Independent preregistered Holochain WasmHash oracle for IG-007D1B.

Standard-library only by design. This is an independent oracle, not
Holochain-native qualification authority.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
from typing import Any

SCHEMA = "ig-007d1b-independent-wasmhash-oracle-output-v1"
PROFILE_SCHEMA = "ig-007d1b-wasmhash-oracle-profile-v1"
WASM_PREFIX = bytes.fromhex("842a24")


def derive_wasm_hash(data: bytes) -> dict[str, str]:
    core = hashlib.blake2b(data, digest_size=32).digest()
    h16 = hashlib.blake2b(core, digest_size=16).digest()
    loc = bytes(h16[i] ^ h16[i + 4] ^ h16[i + 8] ^ h16[i + 12] for i in range(4))
    raw = WASM_PREFIX + core + loc
    if len(raw) != 39:
        raise AssertionError(f"unexpected raw HoloHash length: {len(raw)}")
    display = "u" + base64.urlsafe_b64encode(raw).rstrip(b"=").decode("ascii")
    return {"core32_hex": core.hex(), "loc4_hex": loc.hex(), "raw39_hex": raw.hex(), "display": display}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def load_profile(path: Path) -> tuple[dict[str, Any], str]:
    raw = path.read_bytes()
    profile = json.loads(raw)
    if profile.get("schema") != PROFILE_SCHEMA:
        raise SystemExit(f"profile schema mismatch: {profile.get('schema')!r}")
    if profile.get("authority") != "PrequalificationToolingOnly":
        raise SystemExit("profile authority must remain PrequalificationToolingOnly")
    if profile.get("claim_ceiling") != "NoQualificationAuthority":
        raise SystemExit("profile claim ceiling drift")
    if profile.get("grants_d1b_claim") is not False or profile.get("grants_product_qualification") is not False:
        raise SystemExit("profile may not grant qualification authority")

    required = profile.get("required_future_input") or {}
    if required.get("state") != "UNBOUND_UNTIL_D1A_EXACT_HEAD_PASS":
        raise SystemExit("D1A future-input state drift")
    if any(required.get(k) is not None for k in ("d1a_exact_head", "d1a_run_id", "d1a_corpus_commitment")):
        raise SystemExit("prequalification profile must not pre-bind unqualified D1A evidence")

    oracle = profile.get("independent_oracle") or {}
    expected_oracle = {
        "implementation": "python-stdlib-v1",
        "shared_holochain_hash_crate_dependency": False,
        "content": "exact_raw_wasm_bytes",
        "core_hash": "BLAKE2b-256",
        "type_prefix_hex": "842a24",
        "location_hash": "BLAKE2b-128(core32)",
        "location_fold": "loc[i]=h16[i]^h16[i+4]^h16[i+8]^h16[i+12] for i=0..3",
        "raw_length_bytes": 39,
        "display": "u + URL_SAFE_BASE64_NO_PAD(raw39)",
        "primary_authority_representation": "raw39_hex",
    }
    if oracle != expected_oracle:
        raise SystemExit("independent oracle algorithm contract drift")

    pinned = profile.get("pinned_holochain") or {}
    expected_pinned = {
        "repository": "holochain/holochain",
        "commit": "a6d4e805a0971ccbc0dcb3f3ed6a9e2fac980a3b",
        "dna_wasm_source_path": "crates/holochain_types/src/dna/wasm.rs",
        "dna_wasm_source_git_blob_sha1": "8678f5e3b093921ce2c4acd0bcaca375f8c6691e",
        "source_observation": "DnaWasm hashable content is its exact code bytes",
    }
    if pinned != expected_pinned:
        raise SystemExit("pinned Holochain source contract drift")

    census = profile.get("semantic_census") or []
    if len(census) != 18:
        raise SystemExit(f"semantic census must contain exactly 18 entries, got {len(census)}")
    if [entry.get("ordinal") for entry in census] != list(range(1, 19)):
        raise SystemExit("semantic census ordinal drift")
    names = [entry.get("manifest_name") for entry in census]
    artifacts = [entry.get("artifact_filename") for entry in census]
    if len(set(names)) != 18 or len(set(artifacts)) != 18:
        raise SystemExit("semantic census contains duplicate names or artifacts")
    if sum(entry.get("class") == "Integrity" for entry in census) != 9:
        raise SystemExit("semantic census must contain exactly 9 integrity zomes")
    if sum(entry.get("class") == "Coordinator" for entry in census) != 9:
        raise SystemExit("semantic census must contain exactly 9 coordinator zomes")
    threshold = [entry for entry in census if entry.get("manifest_name") == "threshold_signing"]
    if len(threshold) != 1 or threshold[0].get("artifact_filename") != "threshold_signing_coordinator.wasm":
        raise SystemExit("threshold_signing alias binding drift")

    integrity_names = {entry["manifest_name"] for entry in census if entry.get("class") == "Integrity"}
    coordinator_names = {entry["manifest_name"] for entry in census if entry.get("class") == "Coordinator"}
    dep_rows = profile.get("coordinator_dependencies") or []
    if {row.get("manifest_name") for row in dep_rows} != coordinator_names:
        raise SystemExit("coordinator dependency census coverage drift")
    for row in dep_rows:
        deps = row.get("dependencies")
        if not isinstance(deps, list) or not deps:
            raise SystemExit(f"invalid dependency list for {row.get('manifest_name')!r}")
        if len(deps) != len(set(deps)):
            raise SystemExit(f"duplicate dependency for {row.get('manifest_name')!r}")
        unknown = set(deps) - integrity_names
        if unknown:
            raise SystemExit(f"unknown integrity dependency for {row.get('manifest_name')!r}: {sorted(unknown)!r}")
    return profile, sha256_bytes(raw)


def self_test(profile: dict[str, Any]) -> list[dict[str, str]]:
    results: list[dict[str, str]] = []
    vectors = profile.get("golden_vectors") or []
    if len(vectors) < 2:
        raise SystemExit("at least two frozen golden vectors are required")
    for vector in vectors:
        observed = derive_wasm_hash(bytes.fromhex(vector["input_hex"]))
        for field in ("core32_hex", "loc4_hex", "raw39_hex", "display"):
            if observed[field] != vector[field]:
                raise SystemExit(f"golden vector {vector.get('name')!r} mismatch {field}")
        results.append({"name": vector["name"], "status": "PASS"})
    return results


def derive_corpus(profile: dict[str, Any], corpus_dir: Path) -> list[dict[str, Any]]:
    if not corpus_dir.is_dir():
        raise SystemExit(f"corpus directory does not exist: {corpus_dir}")
    census = profile["semantic_census"]
    expected = [entry["artifact_filename"] for entry in census]
    observed = sorted(path.name for path in corpus_dir.glob("*.wasm") if path.is_file())
    if observed != sorted(expected):
        raise SystemExit(f"corpus filename census mismatch: expected={sorted(expected)!r} observed={observed!r}")
    rows: list[dict[str, Any]] = []
    for entry in census:
        path = corpus_dir / entry["artifact_filename"]
        data = path.read_bytes()
        rows.append({
            "ordinal": entry["ordinal"],
            "class": entry["class"],
            "manifest_name": entry["manifest_name"],
            "artifact_filename": entry["artifact_filename"],
            "artifact_sha256": sha256_bytes(data),
            "artifact_size": len(data),
            **derive_wasm_hash(data),
        })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--corpus-dir", type=Path)
    args = parser.parse_args()

    profile, profile_file_sha256 = load_profile(args.profile)
    golden = self_test(profile)
    output: dict[str, Any] = {
        "schema": SCHEMA,
        "oracle": "python-stdlib-v1",
        "authority": "IndependentOracleOnly",
        "profile_schema": profile["schema"],
        "profile_file_sha256": profile_file_sha256,
        "profile_canonical_sha256": canonical_sha256(profile),
        "pinned_holochain_commit": profile["pinned_holochain"]["commit"],
        "golden_vectors": golden,
        "grants_d1b_claim": False,
        "grants_product_qualification": False,
    }
    if args.corpus_dir is not None:
        output["corpus"] = derive_corpus(profile, args.corpus_dir)
        output["corpus_entry_count"] = len(output["corpus"])
        output["d1a_binding_state"] = profile["required_future_input"]["state"]
    elif not args.self_test:
        parser.error("provide --self-test and/or --corpus-dir")
    print(json.dumps(output, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
