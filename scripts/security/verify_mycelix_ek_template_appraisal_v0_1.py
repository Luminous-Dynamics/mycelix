#!/usr/bin/env python3
"""Appraise a TPM EK public area against the reviewed TCG RSA-2048 L-1 template."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-template-appraisal.v0.1"
SHA256_ID = b"\x00\x0b"
RSA_ID = b"\x00\x01"
AES_ID = b"\x00\x06"
CFB_ID = b"\x00\x43"
NULL_SCHEME = b"\x00\x10"
EXPECTED_ATTRS = bytes.fromhex("000300b2")
EXPECTED_POLICY = bytes.fromhex(
    "837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa"
)


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()


def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def hex_bytes(value: Any, field: str) -> bytes:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be hex")
    value = value.lower().removeprefix("0x")
    if len(value) % 2 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError(f"{field} is not canonical hex")
    return bytes.fromhex(value)


def parse_public(raw: bytes, fmt: str) -> tuple[bytes, dict[str, Any]]:
    if fmt == "TPM2B_PUBLIC":
        if len(raw) < 2:
            raise ValueError("TPM2B_PUBLIC truncated")
        declared = int.from_bytes(raw[:2], "big")
        if declared != len(raw) - 2:
            raise ValueError("TPM2B_PUBLIC size mismatch")
        raw = raw[2:]
    elif fmt != "TPMT_PUBLIC":
        raise ValueError("unsupported public format")

    if len(raw) < 12:
        raise ValueError("TPMT_PUBLIC truncated")
    offset = 0
    kind = raw[offset:offset+2]; offset += 2
    name_alg = raw[offset:offset+2]; offset += 2
    attrs = raw[offset:offset+4]; offset += 4
    policy_size = int.from_bytes(raw[offset:offset+2], "big"); offset += 2
    if policy_size > len(raw) - offset:
        raise ValueError("authPolicy truncated")
    policy = raw[offset:offset+policy_size]; offset += policy_size
    if kind != RSA_ID:
        raise ValueError("EK type is not RSA")
    if len(raw) < offset + 18:
        raise ValueError("RSA public parameters truncated")

    symmetric_alg = raw[offset:offset+2]; offset += 2
    symmetric_bits = raw[offset:offset+2]; offset += 2
    symmetric_mode = raw[offset:offset+2]; offset += 2
    scheme = raw[offset:offset+2]; offset += 2
    key_bits = raw[offset:offset+2]; offset += 2
    exponent = raw[offset:offset+4]; offset += 4
    unique_size = int.from_bytes(raw[offset:offset+2], "big"); offset += 2
    if unique_size > len(raw) - offset:
        raise ValueError("RSA unique field truncated")
    unique = raw[offset:offset+unique_size]; offset += unique_size
    if offset != len(raw):
        raise ValueError("unexpected trailing TPMT_PUBLIC bytes")

    return raw, {
        "type": kind,
        "name_alg": name_alg,
        "attributes": attrs,
        "auth_policy": policy,
        "symmetric_algorithm": symmetric_alg,
        "symmetric_key_bits": symmetric_bits,
        "symmetric_mode": symmetric_mode,
        "scheme": scheme,
        "key_bits": key_bits,
        "exponent": exponent,
        "unique_size": unique_size,
        "unique": unique,
    }


def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    out = {"verifier_id": VERIFIER_ID, "state": state, "reason": reason}
    if details:
        out["details"] = details
    return out


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id","profile_version","verification_mode","claim_ceiling","object_role",
        "public_format","public_wire_hex","public_wire_sha256","name_hex","qualified_name_hex",
        "creation_provenance"
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY","missing-required-fields",{"fields":missing})
    if manifest["profile_id"] != "mycelix.security.tpm.ek-template-appraisal":
        return result("DENY","profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY","profile-version-mismatch")
    if manifest["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:
        return result("DENY","verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY","claim-ceiling-mismatch")
    if manifest["object_role"] != "EK":
        return result("DENY","object-role-invalid")
    if not valid_hash(manifest["public_wire_sha256"]):
        return result("DENY","public-wire-digest-invalid")
    cp = manifest["creation_provenance"]
    if not isinstance(cp, dict):
        return result("DENY","creation-provenance-invalid")
    for field in ("state","tool_id","hierarchy","template_mode","transcript_sha256"):
        if field not in cp:
            return result("DENY","missing-creation-provenance-field",{"field":field})
    if not valid_hash(cp["transcript_sha256"]):
        return result("DENY","creation-transcript-digest-invalid")
    if cp["tool_id"] != "tpm2_createek":
        return result("DENY","creation-tool-mismatch")
    if cp["hierarchy"] != "TPM_RH_ENDORSEMENT":
        return result("DENY","creation-hierarchy-mismatch")
    if cp["template_mode"] not in {"default-low-range","manufacturer-defined"}:
        return result("DENY","creation-template-mode-invalid")
    try:
        raw = hex_bytes(manifest["public_wire_hex"],"public_wire_hex")
        name = hex_bytes(manifest["name_hex"],"name_hex")
        qname = hex_bytes(manifest["qualified_name_hex"],"qualified_name_hex")
        _body, parsed = parse_public(raw,manifest["public_format"])
    except ValueError as exc:
        return result("DENY","public-wire-invalid",{"error":str(exc)})

    if hashlib.sha256(raw).hexdigest() != manifest["public_wire_sha256"]:
        return result("DENY","public-wire-digest-mismatch")
    if parsed["type"] != RSA_ID:
        return result("DENY","template-type-mismatch")
    if parsed["name_alg"] != SHA256_ID:
        return result("DENY","template-name-algorithm-mismatch")
    if parsed["attributes"] != EXPECTED_ATTRS:
        return result("DENY","template-attributes-mismatch")
    if parsed["auth_policy"] != EXPECTED_POLICY:
        return result("DENY","template-auth-policy-mismatch")
    if parsed["symmetric_algorithm"] != AES_ID:
        return result("DENY","template-symmetric-algorithm-mismatch")
    if parsed["symmetric_key_bits"] != bytes.fromhex("0080"):
        return result("DENY","template-symmetric-keybits-mismatch")
    if parsed["symmetric_mode"] != CFB_ID:
        return result("DENY","template-symmetric-mode-mismatch")
    if parsed["scheme"] != NULL_SCHEME:
        return result("DENY","template-scheme-mismatch")
    if parsed["key_bits"] != bytes.fromhex("0800"):
        return result("DENY","template-keybits-mismatch")
    if parsed["exponent"] != bytes(4):
        return result("DENY","template-exponent-mismatch")
    if parsed["unique_size"] != 256:
        return result("DENY","template-unique-size-mismatch")
    if len(name) != 34 or name[:2] != SHA256_ID:
        return result("DENY","invalid-ek-name")
    if len(qname) != 34 or qname[:2] != SHA256_ID:
        return result("DENY","invalid-ek-qualified-name")
    if cp["state"] == "DENY":
        return result("DENY","creation-provenance-denied")
    if cp["state"] == "INDETERMINATE":
        return result("INDETERMINATE","creation-provenance-indeterminate")

    if cp["template_mode"] == "manufacturer-defined":
        return result("INDETERMINATE","manufacturer-template-requires-independent-profile-authorization")
    if manifest["verification_mode"] != "ReferenceModelOnly":
        return result("INDETERMINATE","live-or-offline-ek-profile-execution-not-integrated")

    return result("PASS","tcg-ek-rsa2048-l1-template-coherent",{
        "template_id":"L-1",
        "specification":"TCG EK Credential Profile 2.7",
        "public_wire_sha256":manifest["public_wire_sha256"],
        "name_hex":name.hex(),
        "qualified_name_hex":qname.hex()
    })


def fixture() -> dict[str, Any]:
    body = (
        bytes.fromhex("0001000b000300b2")
        + bytes.fromhex("0020") + EXPECTED_POLICY
        + bytes.fromhex("00060080004300100800")
        + bytes.fromhex("00000000")
        + bytes.fromhex("0100") + bytes(256)
    )
    return {
        "profile_id":"mycelix.security.tpm.ek-template-appraisal",
        "profile_version":"0.1.0",
        "verification_mode":"ReferenceModelOnly",
        "claim_ceiling":"ReferenceModelOnly",
        "object_role":"EK",
        "public_format":"TPMT_PUBLIC",
        "public_wire_hex":body.hex(),
        "public_wire_sha256":hashlib.sha256(body).hexdigest(),
        "name_hex":(SHA256_ID+hashlib.sha256(body).digest()).hex(),
        "qualified_name_hex":(SHA256_ID+hashlib.sha256(b"qname"+body).digest()).hex(),
        "creation_provenance":{
            "state":"PASS","tool_id":"tpm2_createek","hierarchy":"TPM_RH_ENDORSEMENT",
            "template_mode":"default-low-range","transcript_sha256":"dd"*32
        }
    }


def as_tpm2b_public(manifest: dict[str, Any], bad_size: int | None = None) -> None:
    raw = hex_bytes(manifest["public_wire_hex"], "public_wire_hex")
    size = len(raw) if bad_size is None else bad_size
    wrapped = size.to_bytes(2, "big") + raw
    manifest["public_format"] = "TPM2B_PUBLIC"
    manifest["public_wire_hex"] = wrapped.hex()


def self_test() -> int:
    base=fixture()
    cases=[
        ("canonical-valid","PASS",lambda x:x),
        ("public-type-substitution","DENY",lambda x:x.update({"public_wire_hex":"0002"+x["public_wire_hex"][4:]})),
        ("name-alg-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:4]+"0004"+x["public_wire_hex"][8:]})),
        ("attributes-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:8]+"000300f2"+x["public_wire_hex"][16:]})),
        ("auth-policy-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:20]+"ff"+x["public_wire_hex"][22:]})),
        ("symmetric-alg-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:84]+"0007"+x["public_wire_hex"][88:]})),
        ("symmetric-keybits-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:88]+"0100"+x["public_wire_hex"][92:]})),
        ("symmetric-mode-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:92]+"0040"+x["public_wire_hex"][96:]})),
        ("scheme-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:96]+"0014"+x["public_wire_hex"][100:]})),
        ("keybits-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:100]+"0400"+x["public_wire_hex"][104:]})),
        ("exponent-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:104]+"00000001"+x["public_wire_hex"][112:]})),
        ("unique-size-substitution","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"][:112]+"0080"+x["public_wire_hex"][116:]})),
        ("trailing-bytes","DENY",lambda x:x.update({"public_wire_hex":x["public_wire_hex"]+"00"})),
        ("tpm2b-size-substitution","DENY",lambda x:as_tpm2b_public(x, bad_size=0xffff)),
        ("name-substitution","DENY",lambda x:x.update({"name_hex":"000b"+"ff"*32})),
        ("public-wire-digest-substitution","DENY",lambda x:x.update({"public_wire_sha256":"ab"*32})),
        ("creation-hierarchy-substitution","DENY",lambda x:x["creation_provenance"].update({"hierarchy":"TPM_RH_OWNER"})),
        ("creation-tool-substitution","DENY",lambda x:x["creation_provenance"].update({"tool_id":"tpm2_createprimary"})),
        ("creation-provenance-indeterminate","INDETERMINATE",lambda x:x["creation_provenance"].update({"state":"INDETERMINATE"})),
        ("manufacturer-template-mode","INDETERMINATE",lambda x:x["creation_provenance"].update({"template_mode":"manufacturer-defined"})),
        ("offline-bundle","INDETERMINATE",lambda x:x.update({"verification_mode":"OfflineBundle"})),
        ("live-verifier","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession"})),
        ("creation-transcript-substitution","DENY",lambda x:x["creation_provenance"].update({"transcript_sha256":"ac"*32})),
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
    print("EK template appraisal semantic corpus: PASS")
    print("23 adversarial mutations plus canonical case: PASS")
    print("Manufacturer authenticity and hierarchy execution remain separate")
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
    out={"profile_id":"mycelix.security.tpm.ek-template-appraisal","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**verified}
    out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered=json.dumps(out,indent=2,sort_keys=True)+"\n"
    if args.output:
        Path(args.output).write_text(rendered,encoding="utf-8")
    else:
        print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[verified["state"]]


if __name__=="__main__":
    raise SystemExit(main())
