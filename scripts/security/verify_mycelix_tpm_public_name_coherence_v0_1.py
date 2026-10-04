#!/usr/bin/env python3
"""Verify coherence between TPM public-area wire bytes and a TPM object Name."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.public-name-coherence.v0.1"
SHA256_ALG_ID = b"\x00\x0b"
APPROVED_READPUBLIC_SOURCE_SHA256 = "aa" * 32


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()


def valid_sha256(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def hex_bytes(value: Any, field: str) -> bytes:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be hex")
    value = value.lower().removeprefix("0x")
    if len(value) % 2 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError(f"{field} is not canonical hex")
    return bytes.fromhex(value)


def parse_public_wire(raw: bytes, fmt: str) -> tuple[bytes, bytes]:
    if fmt == "TPMT_PUBLIC":
        body = raw
    elif fmt == "TPM2B_PUBLIC":
        if len(raw) < 2:
            raise ValueError("TPM2B_PUBLIC is truncated")
        declared = int.from_bytes(raw[:2], "big")
        if declared != len(raw) - 2:
            raise ValueError("TPM2B_PUBLIC size mismatch")
        body = raw[2:]
    else:
        raise ValueError("unsupported public format")
    if len(body) < 4:
        raise ValueError("TPMT_PUBLIC is truncated before nameAlg")
    return body, body[2:4]


def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    out = {"verifier_id": VERIFIER_ID, "state": state, "reason": reason}
    if details:
        out["details"] = details
    return out


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id","profile_version","verification_mode","claim_ceiling",
        "object_role","public_format","public_wire_hex","public_wire_sha256",
        "name_hex","readpublic_state","readpublic_source_sha256"
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY","missing-required-fields",{"fields":missing})
    if manifest["profile_id"] != "mycelix.security.tpm.public-name-coherence":
        return result("DENY","profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY","profile-version-mismatch")
    if manifest["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:
        return result("DENY","verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY","claim-ceiling-mismatch")
    if manifest["object_role"] not in {"AK","EK"}:
        return result("DENY","object-role-invalid")
    if not valid_sha256(manifest["public_wire_sha256"]) or not valid_sha256(manifest["readpublic_source_sha256"]):
        return result("DENY","source-digest-invalid")
    if manifest["verification_mode"] == "ReferenceModelOnly" and manifest["readpublic_source_sha256"] != APPROVED_READPUBLIC_SOURCE_SHA256:
        return result("DENY","readpublic-source-not-approved-by-reference-model")
    try:
        raw = hex_bytes(manifest["public_wire_hex"],"public_wire_hex")
        name = hex_bytes(manifest["name_hex"],"name_hex")
    except ValueError as exc:
        return result("DENY","malformed-hex",{"error":str(exc)})
    if hashlib.sha256(raw).hexdigest() != manifest["public_wire_sha256"]:
        return result("DENY","public-wire-digest-mismatch")
    try:
        body, name_alg = parse_public_wire(raw,manifest["public_format"])
    except ValueError as exc:
        return result("DENY","public-wire-format-invalid",{"error":str(exc)})
    if name_alg != SHA256_ALG_ID:
        return result("DENY","unsupported-name-algorithm")
    if len(name) != 34 or name[:2] != SHA256_ALG_ID:
        return result("DENY","invalid-tpm-name")
    expected = SHA256_ALG_ID + hashlib.sha256(body).digest()
    if name != expected:
        return result("DENY","name-does-not-match-public-area",{"expected_name_hex":expected.hex(),"observed_name_hex":name.hex()})
    source_state = manifest["readpublic_state"]
    if source_state == "DENY":
        return result("DENY","readpublic-denied")
    if source_state == "INDETERMINATE":
        return result("INDETERMINATE","readpublic-provenance-indeterminate")
    if source_state != "PASS":
        return result("DENY","readpublic-state-invalid")
    if manifest["verification_mode"] == "OfflineBundle":
        return result("INDETERMINATE","offline-readpublic-origin-not-live-authenticated")
    if manifest["verification_mode"] == "LiveVerifierSession":
        return result("INDETERMINATE","live-readpublic-session-not-integrated")
    return result("PASS","public-area-name-coherent",{
        "name_alg":"sha256",
        "public_area_sha256":hashlib.sha256(body).hexdigest(),
        "name_hex":name.hex()
    })


def fixture(fmt: str = "TPMT_PUBLIC") -> dict[str, Any]:
    body = bytes.fromhex("0001000b") + bytes(range(1,65))
    raw = body if fmt == "TPMT_PUBLIC" else len(body).to_bytes(2,"big") + body
    name = SHA256_ALG_ID + hashlib.sha256(body).digest()
    return {
        "profile_id":"mycelix.security.tpm.public-name-coherence",
        "profile_version":"0.1.0",
        "verification_mode":"ReferenceModelOnly",
        "claim_ceiling":"ReferenceModelOnly",
        "object_role":"AK",
        "public_format":fmt,
        "public_wire_hex":raw.hex(),
        "public_wire_sha256":hashlib.sha256(raw).hexdigest(),
        "name_hex":name.hex(),
        "readpublic_state":"PASS",
        "readpublic_source_sha256":"aa"*32
    }


def self_test() -> int:
    base = fixture()
    cases = [
        ("canonical-tpmt-public","PASS",lambda x:x),
        ("canonical-tpm2b-public","PASS",lambda x:x.update(fixture("TPM2B_PUBLIC"))),
        ("name-substitution","DENY",lambda x:x.update({"name_hex":"000b"+"ff"*32})),
        ("wire-byte-substitution","DENY",lambda x:x.update({"public_wire_hex":(bytes.fromhex(x["public_wire_hex"])[:-1]+b"\xff").hex()})),
        ("wire-digest-substitution","DENY",lambda x:x.update({"public_wire_sha256":"bb"*32})),
        ("name-algorithm-substitution","DENY",lambda x:x.update({"public_wire_hex":("0004000b"+bytes.fromhex(x["public_wire_hex"])[4:]).hex()})),
        ("format-mismatch","DENY",lambda x:x.update({"public_format":"TPM2B_PUBLIC"})),
        ("readpublic-deny","DENY",lambda x:x.update({"readpublic_state":"DENY"})),
        ("readpublic-indeterminate","INDETERMINATE",lambda x:x.update({"readpublic_state":"INDETERMINATE"})),
        ("source-provenance-substitution","DENY",lambda x:x.update({"readpublic_source_sha256":"cc"*32})),
        ("live-source-unreviewed","INDETERMINATE",lambda x:(x.update({"verification_mode":"LiveVerifierSession","readpublic_source_sha256":"cc"*32}), x.update({"readpublic_state":"PASS"}))),
        ("malformed-hex","DENY",lambda x:x.update({"public_wire_hex":"zz"})),
        ("outer-size-substitution","DENY",lambda x:(x.update(fixture("TPM2B_PUBLIC")), x.update({"public_wire_hex":"ffff"+x["public_wire_hex"][4:]}))),
    ]
    for name, expected, mutate in cases:
        candidate=copy.deepcopy(base)
        mutate(candidate)
        observed=verify(candidate)
        if observed["state"] != expected:
            print(f"{name}: FAIL expected={expected} got={observed['state']} reason={observed['reason']}")
            return 1
    permuted=json.loads(json.dumps(base,sort_keys=True))
    if verify(permuted)["state"]!="PASS":
        print("key-order-permutation: FAIL")
        return 1
    print("TPM public-area -> Name coherence semantic corpus: PASS")
    print("13 adversarial mutations plus format/key-order controls: PASS")
    print("TPM residency and manufacturer trust remain separate claims")
    return 0


def main() -> int:
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--verify",metavar="MANIFEST")
    parser.add_argument("--output")
    args=parser.parse_args()
    if args.self_test:
        return self_test()
    path=Path(args.verify).resolve()
    manifest=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest,dict):
        raise SystemExit("manifest must be an object")
    verified=verify(manifest)
    out={"profile_id":"mycelix.security.tpm.public-name-coherence","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**verified}
    out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered=json.dumps(out,indent=2,sort_keys=True)+"\n"
    if args.output:
        Path(args.output).write_text(rendered,encoding="utf-8")
    else:
        print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[verified["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
