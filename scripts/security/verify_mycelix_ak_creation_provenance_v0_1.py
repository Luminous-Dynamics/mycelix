#!/usr/bin/env python3
"""Verify TPM AK creation provenance under an explicit claim boundary."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ak-creation-provenance.v0.1"
SHA256_ALG_ID = "000b"


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    ).hexdigest()


def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        c in "0123456789abcdef" for c in value
    )


def normalize_hex(value: Any, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be hex")
    value = value.lower().removeprefix("0x")
    if len(value) % 2 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError(f"{field} is not canonical hex")
    return value


def validate_tpm_name(value: Any, field: str) -> str:
    normalized = normalize_hex(value, field)
    if len(normalized) != 68 or normalized[:4] != SHA256_ALG_ID:
        raise ValueError(f"{field} must be a SHA-256 TPM Name")
    return normalized


def result(state: str, reason: str, details: dict[str, Any] | None = None) -> dict[str, Any]:
    out = {"verifier_id": VERIFIER_ID, "state": state, "reason": reason}
    if details:
        out["details"] = details
    return out


def verify(manifest: dict[str, Any]) -> dict[str, Any]:
    required = {
        "profile_id","profile_version","verification_mode","claim_ceiling",
        "session_id","tpm_identity_digest","ek","ak","signing_key",
        "creation_data","creation_record","certification"
    }
    missing = sorted(required - set(manifest))
    if missing:
        return result("DENY", "missing-required-fields", {"fields": missing})

    if manifest["profile_id"] != "mycelix.security.tpm.ak-creation-provenance":
        return result("DENY", "profile-id-mismatch")
    if manifest["profile_version"] != "0.1.0":
        return result("DENY", "profile-version-mismatch")
    if manifest["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:
        return result("DENY", "verification-mode-invalid")
    if manifest["claim_ceiling"] != "ReferenceModelOnly":
        return result("DENY", "claim-ceiling-mismatch")
    if not valid_hash(manifest["tpm_identity_digest"]):
        return result("DENY", "tpm-identity-digest-invalid")

    ek = manifest["ek"]; ak = manifest["ak"]
    creation = manifest["creation_data"]
    record = manifest["creation_record"]
    cert = manifest["certification"]
    signer = manifest["signing_key"]
    for name, value in (("ek",ek),("ak",ak),("signing_key",signer),("creation_data",creation),("creation_record",record),("certification",cert)):
        if not isinstance(value, dict):
            return result("DENY", f"{name}-section-invalid")

    for section_name, section in (("creation_data", creation), ("creation_record", record), ("certification", cert)):
        if section.get("state") not in {"PASS", "DENY", "INDETERMINATE"}:
            return result("DENY", f"{section_name}-state-invalid")

    for section, fields in (
        (ek, ("public_sha256","name_hex","qualified_name_hex")),
        (ak, ("public_sha256","name_hex")),
        (signer, ("public_sha256","role")),
        (creation, ("state","parent_name_alg_hex","parent_name_hex","parent_qualified_name_hex","pcr_select","pcr_digest_hex","locality","outside_info_sha256","wire_sha256")),
        (record, ("state","object_name_alg","object_name_hex","object_public_sha256","creation_data_sha256","creation_hash_hex","creation_ticket_sha256","creation_data_wire_sha256")),
        (cert, ("state","attestation_type","session_id","tpm_identity_digest","certified_object_name_hex","certified_creation_hash_hex","signing_key_sha256","qualifying_data_sha256","signature_sha256","signature_verification_scope","ticket_validation_state")),
    ):
        for field in fields:
            if field not in section:
                return result("DENY","missing-field",{"field":field})

    for field in ("ek.public_sha256","ak.public_sha256","signing_key.public_sha256"):
        section, key = field.split(".")
        if not valid_hash(manifest[section][key]):
            return result("DENY","key-public-digest-invalid",{"field":field})

    try:
        ek_name = validate_tpm_name(ek["name_hex"], "ek.name_hex")
        ek_qname = validate_tpm_name(ek["qualified_name_hex"], "ek.qualified_name_hex")
        ak_name = validate_tpm_name(ak["name_hex"], "ak.name_hex")
        parent_name = validate_tpm_name(creation["parent_name_hex"], "creation_data.parent_name_hex")
        parent_qname = validate_tpm_name(creation["parent_qualified_name_hex"], "creation_data.parent_qualified_name_hex")
        object_name = validate_tpm_name(record["object_name_hex"], "creation_record.object_name_hex")
        certified_name = validate_tpm_name(cert["certified_object_name_hex"], "certification.certified_object_name_hex")
        certified_creation_hash = normalize_hex(cert["certified_creation_hash_hex"], "certification.certified_creation_hash_hex")
        creation_hash = normalize_hex(record["creation_hash_hex"], "creation_record.creation_hash_hex")
    except ValueError as exc:
        return result("DENY","invalid-hex",{"error":str(exc)})

    if creation["parent_name_alg_hex"].lower().removeprefix("0x") != SHA256_ALG_ID:
        return result("DENY","unsupported-parent-name-algorithm")
    if parent_name != ek_name:
        return result("DENY","creation-parent-name-does-not-equal-ek")
    if parent_qname != ek_qname:
        return result("DENY","creation-parent-qname-does-not-equal-ek")
    if object_name != ak_name:
        return result("DENY","creation-object-name-does-not-equal-ak")
    if len(certified_creation_hash) != 64 or len(creation_hash) != 64:
        return result("DENY","creation-hash-length-invalid")
    if signer.get("role") != "creation-attestation-signer":
        return result("DENY","signing-key-role-invalid")
    if record["object_public_sha256"] != ak["public_sha256"]:
        return result("DENY","creation-object-public-binding-mismatch")
    if record["object_name_alg"] != "sha256":
        return result("DENY","unsupported-object-name-algorithm")
    if not valid_hash(record["creation_data_sha256"]):
        return result("DENY","creation-data-digest-invalid")
    if record["creation_data_sha256"] != creation_hash:
        return result("DENY","creation-hash-does-not-bind-creation-data-digest")
    if not valid_hash(record["creation_ticket_sha256"]):
        return result("DENY","creation-ticket-digest-invalid")
    if not valid_hash(record["creation_data_wire_sha256"]):
        return result("DENY","creation-data-wire-digest-invalid")
    if record["creation_data_wire_sha256"] != creation["wire_sha256"]:
        return result("DENY","creation-data-wire-binding-mismatch")
    if not valid_hash(creation["outside_info_sha256"]) or not valid_hash(creation["wire_sha256"]):
        return result("DENY","creation-data-provenance-digest-invalid")
    if not isinstance(creation["pcr_select"], str) or not creation["pcr_select"]:
        return result("DENY","creation-pcr-selection-invalid")
    try:
        pcr_digest = normalize_hex(creation["pcr_digest_hex"], "creation_data.pcr_digest_hex")
    except ValueError as exc:
        return result("DENY","creation-pcr-digest-invalid",{"error":str(exc)})
    if len(pcr_digest) != 64:
        return result("DENY","creation-pcr-digest-length-invalid")
    if not isinstance(creation["locality"], int) or not 0 <= creation["locality"] <= 4:
        return result("DENY","creation-locality-invalid")
    if creation["state"] == "DENY" or record["state"] == "DENY":
        return result("DENY","creation-record-denied")
    if creation["state"] == "INDETERMINATE" or record["state"] == "INDETERMINATE":
        return result("INDETERMINATE","creation-record-indeterminate")

    if cert["session_id"] != manifest["session_id"]:
        return result("DENY","certification-session-mismatch")
    if cert["tpm_identity_digest"] != manifest["tpm_identity_digest"]:
        return result("DENY","certification-tpm-identity-mismatch")
    if cert["attestation_type"] != "TPM2_CREATION":
        return result("DENY","certification-type-mismatch")
    if certified_name != object_name:
        return result("DENY","certified-object-name-mismatch")
    if certified_creation_hash != creation_hash:
        return result("DENY","certified-creation-hash-mismatch")
    for field in ("signing_key_sha256","qualifying_data_sha256","signature_sha256"):
        if not valid_hash(cert[field]):
            return result("DENY","certification-digest-invalid",{"field":field})
    if cert["signature_verification_scope"] != "ReferenceModelOnly":
        return result("INDETERMINATE","signature-verification-outside-reference-model")
    if cert["state"] == "DENY":
        return result("DENY","creation-certification-denied")
    if cert["state"] == "INDETERMINATE":
        return result("INDETERMINATE","creation-certification-indeterminate")
    if cert["state"] != "PASS":
        return result("DENY","creation-certification-state-invalid")
    if cert["ticket_validation_state"] == "DENY":
        return result("DENY","creation-ticket-validation-denied")
    if cert["ticket_validation_state"] == "INDETERMINATE":
        return result("INDETERMINATE","creation-ticket-validation-indeterminate")
    if cert["ticket_validation_state"] != "PASS":
        return result("DENY","creation-ticket-validation-state-invalid")

    if manifest["verification_mode"] == "OfflineBundle":
        return result("INDETERMINATE","offline-creation-certificate-not-live-authenticated")
    if manifest["verification_mode"] == "LiveVerifierSession":
        return result("INDETERMINATE","live-creation-certification-not-integrated")
    return result(
        "PASS",
        "ak-creation-provenance-verified",
        {
            "ek_name": ek_name,
            "ek_qualified_name": ek_qname,
            "ak_name": ak_name,
            "creation_hash": creation_hash,
            "creation_data_wire_sha256": creation["wire_sha256"],
        },
    )


def fixture() -> dict[str, Any]:
    ek_name = "000b" + "11" * 32
    ek_qname = "000b" + "22" * 32
    ak_name = "000b" + "33" * 32
    creation_hash = "aa" * 32
    tpm_id = "44" * 32
    return {
        "profile_id":"mycelix.security.tpm.ak-creation-provenance",
        "profile_version":"0.1.0",
        "verification_mode":"ReferenceModelOnly",
        "claim_ceiling":"ReferenceModelOnly",
        "session_id":"creation-self-test",
        "tpm_identity_digest":tpm_id,
        "ek":{"public_sha256":"55"*32,"name_hex":ek_name,"qualified_name_hex":ek_qname},
        "ak":{"public_sha256":"66"*32,"name_hex":ak_name},
        "signing_key":{"public_sha256":"cc"*32,"role":"creation-attestation-signer"},
        "creation_data":{
            "state":"PASS","parent_name_alg_hex":"000b",
            "parent_name_hex":ek_name,"parent_qualified_name_hex":ek_qname,
            "pcr_select":"sha256:0,2,4,7","pcr_digest_hex":"77"*32,"locality":0,
            "outside_info_sha256":"88"*32,"wire_sha256":"99"*32
        },
        "creation_record":{
            "state":"PASS","object_name_alg":"sha256","object_name_hex":ak_name,
            "object_public_sha256":"66"*32,"creation_data_sha256":creation_hash,
            "creation_hash_hex":creation_hash,"creation_ticket_sha256":"b2"*32,
            "creation_data_wire_sha256":"99"*32
        },
        "certification":{
            "state":"PASS","attestation_type":"TPM2_CREATION",
            "session_id":"creation-self-test","tpm_identity_digest":tpm_id,
            "certified_object_name_hex":ak_name,
            "certified_creation_hash_hex":creation_hash,
            "signing_key_sha256":"c3"*32,"qualifying_data_sha256":"d4"*32,
            "signature_sha256":"e5"*32,"signature_verification_scope":"ReferenceModelOnly",
            "ticket_validation_state":"PASS","creation_ticket_sha256":"b2"*32
        }
    }


def self_test() -> int:
    base = fixture()
    cases = [
        ("canonical-valid","PASS",lambda x: x),
        ("parent-name-substitution","DENY",lambda x: x["creation_data"].update({"parent_name_hex":"000b"+"fe"*32})),
        ("parent-qname-substitution","DENY",lambda x: x["creation_data"].update({"parent_qualified_name_hex":"000b"+"fd"*32})),
        ("parent-name-alg-substitution","DENY",lambda x: x["creation_data"].update({"parent_name_alg_hex":"0004"})),
        ("ak-name-substitution","DENY",lambda x: x["ak"].update({"name_hex":"000b"+"fc"*32})),
        ("ak-public-substitution","DENY",lambda x: x["ak"].update({"public_sha256":"fb"*32})),
        ("creation-data-hash-substitution","DENY",lambda x: x["creation_record"].update({"creation_data_sha256":"fa"*32})),
        ("creation-ticket-substitution","DENY",lambda x: x["creation_record"].update({"creation_ticket_sha256":"f9"*32})),
        ("certified-object-name-substitution","DENY",lambda x: x["certification"].update({"certified_object_name_hex":"000b"+"f8"*32})),
        ("certified-creation-hash-substitution","DENY",lambda x: x["certification"].update({"certified_creation_hash_hex":"f7"*32})),
        ("certification-type-substitution","DENY",lambda x: x["certification"].update({"attestation_type":"TPM2_QUOTE"})),
        ("signature-unverified","DENY",lambda x: x["certification"].update({"state":"DENY"})),
        ("ticket-validation-deny","DENY",lambda x: x["certification"].update({"ticket_validation_state":"DENY"})),
        ("ticket-validation-indeterminate","INDETERMINATE",lambda x: x["certification"].update({"ticket_validation_state":"INDETERMINATE"})),
        ("creation-data-indeterminate","INDETERMINATE",lambda x: x["creation_data"].update({"state":"INDETERMINATE"})),
        ("certification-indeterminate","INDETERMINATE",lambda x: x["certification"].update({"state":"INDETERMINATE"})),
        ("session-substitution","DENY",lambda x: x["certification"].update({"session_id":"other"})),
        ("tpm-substitution","DENY",lambda x: x["certification"].update({"tpm_identity_digest":"ee"*32})),
        ("signing-key-substitution","DENY",lambda x: x["certification"].update({"signing_key_sha256":"dd"*32})),
        ("wire-hash-substitution","DENY",lambda x: x["creation_data"].update({"wire_sha256":"cc"*32})),
        ("creation-state-invalid","DENY",lambda x: x["creation_data"].update({"state":"BROKEN"})),
        ("offline-bundle","INDETERMINATE",lambda x: x.update({"verification_mode":"OfflineBundle"})),
    ]
    for name, expected, mutate in cases:
        candidate = copy.deepcopy(base)
        mutate(candidate)
        observed = verify(candidate)
        if observed["state"] != expected:
            print(f"{name}: FAIL expected={expected} got={observed['state']} reason={observed['reason']}")
            return 1
    permuted = json.loads(json.dumps(base, sort_keys=True))
    if verify(permuted)["state"] != "PASS":
        print("key-order-permutation: FAIL")
        return 1
    print("AK creation provenance semantic corpus: PASS")
    print("21 adversarial mutations plus canonical case: PASS")
    print("wire-format and live-verifier boundaries remain explicit")
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
    out = {
        "profile_id":"mycelix.security.tpm.ak-creation-provenance",
        "profile_version":"0.1.0",
        "verifier_id":VERIFIER_ID,
        "input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
        **verified,
    }
    out["content_sha256"] = canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered = json.dumps(out, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[verified["state"]]


if __name__ == "__main__":
    raise SystemExit(main())
