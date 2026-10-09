#!/usr/bin/env python3
"""Independent controls for parent JWS signing-input par_hash linkage."""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import delegation_chain_par_hash as checker  # noqa: E402

SCHEMA = "mycelix.delegation-chain-par-hash.v1"
FROZEN_MAX_TOKEN_COUNT = 9
FROZEN_MAX_SIGNING_INPUT_BYTES = 64 * 1024
FROZEN_MAX_CHAIN_SIGNING_INPUT_BYTES = 256 * 1024


def b64u(value: bytes) -> str:
    return base64.urlsafe_b64encode(value).rstrip(b"=").decode("ascii")


def independently_valid_digest(value: Any) -> bool:
    if not isinstance(value, str) or len(value) != 43:
        return False
    if any(ch not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-"
           for ch in value):
        return False
    try:
        decoded = base64.urlsafe_b64decode(value + "=")
    except (ValueError, base64.binascii.Error):
        return False
    return len(decoded) == 32 and b64u(decoded) == value


def independently_valid_signing_input(value: Any) -> bool:
    if not isinstance(value, str):
        return False
    try:
        encoded = value.encode("ascii")
    except UnicodeEncodeError:
        return False
    if len(encoded) > FROZEN_MAX_SIGNING_INPUT_BYTES:
        return False
    parts = value.split(".")
    if len(parts) != 2:
        return False
    for part in parts:
        if not part or any(ch not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-"
                            for ch in part):
            return False
        try:
            decoded = base64.urlsafe_b64decode(part + "=" * ((4 - len(part) % 4) % 4))
        except (ValueError, base64.binascii.Error):
            return False
        if b64u(decoded) != part:
            return False
    return True


def independent_par_hash(signing_input: str) -> str:
    if not independently_valid_signing_input(signing_input):
        raise ValueError("invalid exact JWS signing input")
    return b64u(hashlib.sha256(signing_input.encode("ascii")).digest())


def segment(content: dict[str, Any]) -> str:
    return b64u(json.dumps(content, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, allow_nan=False).encode("utf-8"))


def signing_input(index: int) -> str:
    header = {"alg": "EdDSA", "typ": "JWT", "kid": f"fixture-key-{index}"}
    payload = {"jti": f"fixture-token-{index}", "test": "not-a-real-signed-token"}
    return segment(header) + "." + segment(payload)


def valid_chain(count: int = 4) -> dict[str, Any]:
    hops: list[dict[str, Any]] = []
    for index in range(count):
        raw_input = signing_input(index)
        claims: dict[str, Any] = {"jti": f"token-{index}"}
        if index:
            claims["par_hash"] = independent_par_hash(hops[index - 1]["signing_input"])
        hops.append({"id": f"hop-{index}", "signing_input": raw_input, "claims": claims})
    return {"schema": SCHEMA, "hops": hops}


def cases() -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}

    raw = valid_chain()
    raw["hops"][2]["claims"]["par_hash"] = "AQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQE"
    out["wrong-par-hash"] = raw

    raw = valid_chain()
    raw["hops"][2]["claims"]["par_hash"] = "B" * 43
    out["noncanonical-par-hash"] = raw

    raw = valid_chain()
    del raw["hops"][2]["claims"]["par_hash"]
    out["missing-par-hash"] = raw

    raw = valid_chain()
    raw["hops"][1]["signing_input"] = signing_input(1)  # child claim still commits to old root input?
    raw["hops"][0]["signing_input"] = signing_input(3)
    out["parent-token-reassociation"] = raw

    raw = valid_chain()
    raw["hops"][0]["claims"]["par_hash"] = independent_par_hash(raw["hops"][0]["signing_input"])
    out["root-par-hash-present"] = raw

    raw = valid_chain()
    raw["hops"][2]["signing_input"] = "not.a.valid.signing.input"
    out["malformed-signing-input"] = raw

    raw = valid_chain()
    raw["hops"][2]["signing_input"] = "é." + segment({"p": 1})
    out["non-ascii-signing-input"] = raw

    raw = valid_chain()
    oversized_payload = segment({"blob": "x" * 50000})
    raw["hops"][2]["signing_input"] = segment({"alg": "EdDSA"}) + "." + oversized_payload
    out["oversized-signing-input"] = raw

    raw = valid_chain(count=9)
    for index, hop in enumerate(raw["hops"]):
        hop["signing_input"] = segment({"alg": "EdDSA", "kid": f"large-{index}"}) + "." + segment(
            {"blob": "x" * 23000, "index": index}
        )
    for index in range(1, len(raw["hops"])):
        raw["hops"][index]["claims"]["par_hash"] = independent_par_hash(
            raw["hops"][index - 1]["signing_input"]
        )
    out["aggregate-signing-input-size"] = raw

    raw = valid_chain()
    raw["hops"] = raw["hops"] * 3
    out["over-depth-chain"] = raw

    raw = valid_chain()
    raw["hops"][2]["id"] = raw["hops"][1]["id"]
    out["duplicate-hop-id"] = raw

    raw = valid_chain()
    raw["schema"] = "unknown-v99"
    out["unknown-schema"] = raw
    return out


def independently_expected(raw: dict[str, Any]) -> set[tuple[str, int | None]]:
    hops = raw.get("hops")
    if raw.get("schema") != SCHEMA:
        return {("unsupported-schema", None)}
    if not isinstance(hops, list) or not hops:
        return {("malformed-chain", None)}
    if len(hops) > FROZEN_MAX_TOKEN_COUNT:
        return {("implementation-chain-depth-exceeded", None)}
    total_input_bytes = sum(len(hop["signing_input"].encode("utf-8"))
                            for hop in hops
                            if isinstance(hop, dict) and isinstance(hop.get("signing_input"), str))
    if total_input_bytes > FROZEN_MAX_CHAIN_SIGNING_INPUT_BYTES:
        return {("chain-signing-input-size-exceeded", None)}
    findings: set[tuple[str, int | None]] = set()
    ids: list[str] = []
    for index, hop in enumerate(hops):
        if not isinstance(hop, dict) or not isinstance(hop.get("claims"), dict):
            findings.add(("hop-claims-malformed", index))
            continue
        hop_id = hop.get("id")
        if not isinstance(hop_id, str) or not hop_id:
            findings.add(("hop-id-missing", index))
            hop_id = f"<index-{index}>"
        ids.append(hop_id)
        raw_input = hop.get("signing_input")
        if not independently_valid_signing_input(raw_input):
            findings.add(("signing-input-invalid", index))
        claims = hop["claims"]
        if index == 0:
            if "par_hash" in claims:
                findings.add(("root-par-hash-must-be-absent", index))
        else:
            parent_input = hops[index - 1].get("signing_input") if isinstance(hops[index - 1], dict) else None
            try:
                expected = independent_par_hash(parent_input)
            except (TypeError, ValueError):
                findings.add(("parent-signing-input-unverifiable", index))
                expected = None
            observed = claims.get("par_hash")
            if not isinstance(observed, str) or not observed:
                findings.add(("par-hash-missing", index))
            elif len(observed) != 43 or any(
                ch not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-"
                for ch in observed
            ):
                findings.add(("par-hash-malformed", index))
            elif not independently_valid_digest(observed):
                findings.add(("par-hash-malformed", index))
            elif expected is not None and observed != expected:
                findings.add(("par-hash-mismatch", index))
    if len(ids) != len(set(ids)):
        findings.add(("duplicate-hop-id", None))
    return findings


def audit(raw: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any] | None:
    ids={hop["id"]: i for i, hop in enumerate(raw.get("hops", []))
         if isinstance(hop, dict) and isinstance(hop.get("id"), str)}
    found: set[tuple[str, int | None]] = set()
    for item in observed.get("findings", []):
        hop_id=item.get("hop_id")
        index=ids.get(hop_id) if hop_id is not None else item.get("hop_index")
        found.add((str(item.get("code")), index))
    expected=independently_expected(raw)
    if found != expected:
        return {"kind":"finding-set-disagrees-with-independent-replay",
                "expected":sorted((code,-1 if idx is None else idx) for code,idx in expected),
                "observed":sorted((code,-1 if idx is None else idx) for code,idx in found)}
    total_input_bytes = sum(len(hop["signing_input"].encode("utf-8"))
                            for hop in raw.get("hops", [])
                            if isinstance(hop, dict) and isinstance(hop.get("signing_input"), str))
    if (raw.get("schema") != SCHEMA or len(raw.get("hops", [])) > FROZEN_MAX_TOKEN_COUNT
            or total_input_bytes > FROZEN_MAX_CHAIN_SIGNING_INPUT_BYTES):
        expected_status="UNSUPPORTED_OR_UNDECIDABLE"
    else:
        expected_status="PARENT_SIGNING_INPUT_LINKAGE_PASS" if not expected else "INVALID_CHAIN"
    if observed.get("status") != expected_status:
        return {"kind":"status-disagrees-with-independent-replay",
                "expected":expected_status,"observed":observed.get("status")}
    return None


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    receipt: dict[str,Any]={"schema":"mycelix.par-hash-differential-receipt.v1",
                            "status":"RUNNING","qualification":"NOT_CLAIMED",
                            "controls":[],"mutations":[]}
    try:
        require(checker.MAX_TOKEN_COUNT==FROZEN_MAX_TOKEN_COUNT,
                "maximum chain token count differs from independently frozen value")
        require(checker.MAX_SIGNING_INPUT_BYTES==FROZEN_MAX_SIGNING_INPUT_BYTES,
                "per-token signing-input limit differs from independently frozen value")
        require(checker.MAX_CHAIN_SIGNING_INPUT_BYTES==FROZEN_MAX_CHAIN_SIGNING_INPUT_BYTES,
                "aggregate signing-input limit differs from independently frozen value")
        valid=valid_chain()
        baseline=checker.evaluate_par_hash_chain(valid)
        mismatch=audit(valid,baseline)
        require(mismatch is None,"valid parent-hash chain disagrees: "+str(mismatch))
        receipt["controls"].append({"id":"valid-four-token-chain",
                                    "status":baseline["status"],"independent_replay":"PASS"})
        all_cases=cases()
        for name,raw in all_cases.items():
            observed=checker.evaluate_par_hash_chain(raw)
            mismatch=audit(raw,observed)
            require(mismatch is None,name+": independent replay mismatch: "+str(mismatch))
            receipt["controls"].append({"id":name,"status":observed["status"],
                                        "independent_replay":"PASS",
                                        "findings":[row["code"] for row in observed.get("findings",[])]})

        mutations=(
            ("par-hash-comparison-omitted","wrong-par-hash","par-hash-mismatch"),
            ("root-par-hash-rejection-omitted","root-par-hash-present","root-par-hash-must-be-absent"),
            ("canonical-signing-input-rejection-omitted","malformed-signing-input","signing-input-invalid"),
        )
        original=checker.evaluate_par_hash_chain
        for mutation_id,case_id,remove_code in mutations:
            raw=all_cases[case_id]
            def mutant(candidate:dict[str,Any],original=original,remove_code=remove_code)->dict[str,Any]:
                result=copy.deepcopy(original(candidate))
                result["findings"]=[row for row in result.get("findings",[]) if row.get("code")!=remove_code]
                if not result["findings"]:
                    result["status"]="PARENT_SIGNING_INPUT_LINKAGE_PASS"
                return result
            checker.evaluate_par_hash_chain=mutant
            try:
                observed=checker.evaluate_par_hash_chain(raw)
                mismatch=audit(raw,observed)
            finally:
                checker.evaluate_par_hash_chain=original
            require(mismatch is not None and mismatch.get("kind")=="finding-set-disagrees-with-independent-replay",
                    mutation_id+": omitted check escaped independent detection")
            receipt["mutations"].append({"id":mutation_id,"detected":True,
                                         "mismatch_kind":mismatch["kind"]})

        receipt["source_head"]=subprocess.run(["git","rev-parse","HEAD"],text=True,
                                              capture_output=True,check=True,timeout=15).stdout.strip()
        checker_path=HERE/"delegation_chain_par_hash.py"
        receipt["checker_sha256"]=hashlib.sha256(checker_path.read_bytes()).hexdigest()
        receipt["test_sha256"]=hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
        receipt["status"]="PASS"
        receipt["summary"]={"positive_controls":1,
                            "negative_controls":len(receipt["controls"])-1,
                            "mutants_injected":len(receipt["mutations"]),
                            "mutants_detected":sum(x["detected"] for x in receipt["mutations"]),
                            "independent_hash":"SHA256_OVER_EXACT_ASCII_JWS_SIGNING_INPUT",
                            "qualification":"NOT_CLAIMED"}
        args.output.write_text(json.dumps(receipt,sort_keys=True,indent=2)+"\n",encoding="utf-8")
        print("PAR_HASH DIFFERENTIAL PASS: 1 positive + 12 negative controls")
        print("PAR_HASH MUTATION SENSITIVITY PASS: 3 of 3 omitted checks detected")
        print("QUALIFICATION NOT CLAIMED: no compact JWS parsing or signature verification")
        return 0
    except Exception as error:
        receipt["status"]="FAIL"
        receipt["error"]=str(error)
        args.output.write_text(json.dumps(receipt,sort_keys=True,indent=2)+"\n",encoding="utf-8")
        print("PAR_HASH DIFFERENTIAL FAIL: "+str(error),file=sys.stderr)
        return 1


if __name__=="__main__":
    raise SystemExit(main())
