#!/usr/bin/env python3
"""Derive AK TPMA_OBJECT security attributes from exact TPMT_PUBLIC bytes."""
from __future__ import annotations
import argparse, copy, hashlib, json
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ak-public-attributes.v0.1"
SHA256_ID = b"\x00\x0b"
APPROVED_READPUBLIC_SOURCE_SHA256 = "aa" * 32
FIXED_TPM = 1 << 1
FIXED_PARENT = 1 << 4
SENSITIVE_DATA_ORIGIN = 1 << 5
RESERVED_MASK = (1 << 0) | (1 << 3) | (0b11 << 8) | (0xF << 12) | (0xFFF << 20)

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

def parse_public(raw: bytes, fmt: str) -> bytes:
    if fmt == "TPM2B_PUBLIC":
        if len(raw) < 2:
            raise ValueError("TPM2B_PUBLIC truncated")
        declared = int.from_bytes(raw[:2], "big")
        if declared != len(raw) - 2:
            raise ValueError("TPM2B_PUBLIC size mismatch")
        raw = raw[2:]
    elif fmt != "TPMT_PUBLIC":
        raise ValueError("unsupported public format")
    if len(raw) < 8:
        raise ValueError("TPMT_PUBLIC truncated before objectAttributes")
    return raw

def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    out = {"verifier_id": VERIFIER_ID, "state": state, "reason": reason}
    if details:
        out["details"] = details
    return out

def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {"profile_id","profile_version","verification_mode","claim_ceiling","object_role","public_format","public_wire_hex","public_wire_sha256","name_hex","readpublic_state","readpublic_source_sha256"}
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY","missing-required-fields",{"fields":missing})
    if manifest["profile_id"] != "mycelix.security.tpm.ak-public-attributes":
        return result("DENY","profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY","profile-version-mismatch")
    if manifest["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:
        return result("DENY","verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY","claim-ceiling-mismatch")
    if manifest["object_role"] != "AK":
        return result("DENY","object-role-invalid")
    if not valid_sha256(manifest["public_wire_sha256"]) or not valid_sha256(manifest["readpublic_source_sha256"]):
        return result("DENY","digest-invalid")
    if manifest["verification_mode"] == "ReferenceModelOnly" and manifest["readpublic_source_sha256"] != APPROVED_READPUBLIC_SOURCE_SHA256:
        return result("DENY","readpublic-source-not-approved")
    try:
        raw = hex_bytes(manifest["public_wire_hex"],"public_wire_hex")
        name = hex_bytes(manifest["name_hex"],"name_hex")
        body = parse_public(raw,manifest["public_format"])
    except ValueError as exc:
        return result("DENY","invalid-public-input",{"error":str(exc)})
    if hashlib.sha256(raw).hexdigest() != manifest["public_wire_sha256"]:
        return result("DENY","public-wire-digest-mismatch")
    if body[2:4] != SHA256_ID:
        return result("DENY","unsupported-name-algorithm")
    if len(name) != 34 or name[:2] != SHA256_ID:
        return result("DENY","invalid-tpm-name")
    expected_name = SHA256_ID + hashlib.sha256(body).digest()
    if name != expected_name:
        return result("DENY","name-does-not-match-public-area")
    attrs = int.from_bytes(body[4:8], "big")
    if attrs & RESERVED_MASK:
        return result("DENY","reserved-object-attribute-set",{"object_attributes_hex":body[4:8].hex()})
    details = {
        "object_attributes_hex":body[4:8].hex(),
        "fixedTPM":bool(attrs & FIXED_TPM),
        "fixedParent":bool(attrs & FIXED_PARENT),
        "sensitiveDataOrigin":bool(attrs & SENSITIVE_DATA_ORIGIN),
        "public_area_sha256":hashlib.sha256(body).hexdigest(),
        "name_hex":name.hex(),
    }
    if not details["fixedTPM"]:
        return result("DENY","ak-fixedTPM-derived-clear",details)
    if not details["fixedParent"]:
        return result("DENY","ak-fixedParent-derived-clear",details)
    if manifest["readpublic_state"] == "DENY":
        return result("DENY","readpublic-denied",details)
    if manifest["readpublic_state"] == "INDETERMINATE":
        return result("INDETERMINATE","readpublic-indeterminate",details)
    if manifest["readpublic_state"] != "PASS":
        return result("DENY","readpublic-state-invalid",details)
    if manifest["verification_mode"] != "ReferenceModelOnly":
        return result("INDETERMINATE","live-or-offline-readpublic-not-authorized",details)
    return result("PASS","ak-public-attributes-coherent",details)

def fixture() -> dict[str,Any]:
    body = bytes.fromhex("0001000b00000032") + (b"\x00" * 64)
    name = SHA256_ID + hashlib.sha256(body).digest()
    return {"profile_id":"mycelix.security.tpm.ak-public-attributes","profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","object_role":"AK","public_format":"TPMT_PUBLIC","public_wire_hex":body.hex(),"public_wire_sha256":hashlib.sha256(body).hexdigest(),"name_hex":name.hex(),"readpublic_state":"PASS","readpublic_source_sha256":APPROVED_READPUBLIC_SOURCE_SHA256}

def self_test() -> int:
    base = fixture()
    def mutate_attrs(value: dict[str,Any], attrs: int) -> None:
        body = bytearray(hex_bytes(value["public_wire_hex"],"public_wire_hex"))
        body[4:8] = attrs.to_bytes(4,"big")
        value["public_wire_hex"] = bytes(body).hex()
        value["public_wire_sha256"] = hashlib.sha256(body).hexdigest()
        value["name_hex"] = (SHA256_ID + hashlib.sha256(body).digest()).hex()
    cases = [
        ("canonical-fixed-ak","PASS",lambda x:x),
        ("fixedTPM-cleared-in-wire","DENY",lambda x:mutate_attrs(x,0x30)),
        ("fixedParent-cleared-in-wire","DENY",lambda x:mutate_attrs(x,0x22)),
        ("both-fixed-cleared-in-wire","DENY",lambda x:mutate_attrs(x,0x20)),
        ("sensitive-origin-cleared-with-recomputed-name","PASS",lambda x:mutate_attrs(x,0x12)),
        ("reserved-bit-set","DENY",lambda x:mutate_attrs(x,0x33)),
        ("public-wire-digest-substitution","DENY",lambda x:x.update({"public_wire_sha256":"bb"*32})),
        ("public-wire-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:-2]+"ff"})),
        ("name-substitution","DENY",lambda x:x.update({"name_hex":"000b"+"ff"*32})),
        ("name-alg-substitution","DENY",lambda x:x.update({"public_wire_hex":"00010004"+x["public_wire_hex"][8:]})),
        ("format-mismatch","DENY",lambda x:x.update({"public_format":"TPM2B_PUBLIC"})),
        ("readpublic-deny","DENY",lambda x:x.update({"readpublic_state":"DENY"})),
        ("readpublic-indeterminate","INDETERMINATE",lambda x:x.update({"readpublic_state":"INDETERMINATE"})),
        ("reference-source-substitution","DENY",lambda x:x.update({"readpublic_source_sha256":"cc"*32})),
        ("live-source-unreviewed","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession","readpublic_source_sha256":"cc"*32})),
    ]
    for name, expected, mutate in cases:
        candidate = copy.deepcopy(base)
        mutate(candidate)
        observed = verify(candidate)
        if observed["state"] != expected:
            print(f"{name}: FAIL expected={expected} got={observed['state']} reason={observed['reason']}")
            return 1
    permuted = json.loads(json.dumps(base,sort_keys=True))
    if verify(permuted)["state"] != "PASS":
        print("key-order-permutation: FAIL")
        return 1
    print("AK public attributes semantic corpus: PASS")
    print("15 adversarial mutations plus canonical case: PASS")
    print("fixedTPM/fixedParent are derived only from TPMT_PUBLIC bytes")
    return 0

def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--verify", metavar="MANIFEST")
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    path = Path(args.verify).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise SystemExit("manifest must be an object")
    verified = verify(manifest)
    output = {"profile_id":"mycelix.security.tpm.ak-public-attributes","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**verified}
    output["content_sha256"] = canonical_hash({k:v for k,v in output.items() if k!="content_sha256"})
    rendered = json.dumps(output,indent=2,sort_keys=True)+"\n"
    if args.output:
        Path(args.output).write_text(rendered,encoding="utf-8")
    else:
        print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[verified["state"]]

if __name__ == "__main__":
    raise SystemExit(main())
