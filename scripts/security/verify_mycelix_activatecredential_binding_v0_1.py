#!/usr/bin/env python3
"""Verify TPM ActivateCredential association receipts."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
from typing import Any

CAPTURE_SOURCE_SCRIPT = Path(__file__).with_name("capture_mycelix_activatecredential_v0_1.py")

def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

VERIFIER_ID = "mycelix.tpm.activatecredential-binding.v0.1"
def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()

def valid_hash(value: Any) -> bool:
    return isinstance(value,str) and len(value)==64 and all(c in "0123456789abcdef" for c in value)

def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details is not None: out["details"]=details
    return out

def session_binding(m:dict[str,Any])->str:
    return canonical_hash({
        "verification_mode":m["verification_mode"],
        "session_id":m["session_id"],
        "tpm_identity_digest":m["tpm_identity_digest"],
        "ek_public_sha256":m["ek_public_sha256"],
        "ek_public_wire_sha256":m["ek_public_wire_sha256"],
        "ak_name_sha256":m["ak_name_sha256"],
        "registrar_secret_sha256":m["registrar_secret_sha256"],
        "recovered_secret_sha256":m["recovered_secret_sha256"],
        "makecredential_blob_sha256":m["makecredential_blob_sha256"],
        "makecredential_transcript_sha256":m["makecredential_transcript_sha256"],
        "activatecredential_transcript_sha256":m["activatecredential_transcript_sha256"],
        "capture_source_sha256":m["capture_source_sha256"],
        "ak_authorization_file_sha256":m["ak_authorization_file_sha256"],
        "makecredential_returncode":m["makecredential_returncode"],
        "activatecredential_returncode":m["activatecredential_returncode"],
    })

def verify(m:dict[str,Any])->dict[str,Any]:
    required={
        "profile_id","profile_version","verification_mode","claim_ceiling","session_id",
        "tpm_identity_digest","ek_public_sha256","ek_public_wire_sha256","ak_name_sha256",
        "registrar_secret_sha256","recovered_secret_sha256","makecredential_blob_sha256",
        "makecredential_transcript_sha256","activatecredential_transcript_sha256",
        "capture_source_sha256","makecredential_returncode","activatecredential_returncode",
        "secret_equality_state","session_binding_sha256"
    }
    missing=sorted(required-set(m))
    if missing:return result("DENY","missing-required-fields",{"fields":missing})
    if m["profile_id"]!="mycelix.security.tpm.activatecredential-binding":return result("DENY","profile-id-mismatch")
    if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    if not isinstance(m["session_id"],str) or not m["session_id"]:return result("DENY","session-id-invalid")
    for field in ("tpm_identity_digest","ek_public_sha256","ek_public_wire_sha256","ak_name_sha256","registrar_secret_sha256","recovered_secret_sha256","makecredential_blob_sha256","makecredential_transcript_sha256","activatecredential_transcript_sha256","capture_source_sha256","session_binding_sha256"):
        if not valid_hash(m[field]):return result("DENY","digest-invalid",{"field":field})
    if not CAPTURE_SOURCE_SCRIPT.is_file():
        return result("DENY","activation-capture-source-missing")
    if m["capture_source_sha256"] != sha256_file(CAPTURE_SOURCE_SCRIPT):
        return result("DENY","activation-capture-source-mismatch")
    if m["makecredential_returncode"]!=0:return result("DENY","makecredential-failed")
    if m["activatecredential_returncode"]!=0:return result("DENY","activatecredential-failed")
    if m["secret_equality_state"]=="DENY":return result("DENY","activatecredential-secret-equality-denied")
    if m["secret_equality_state"]=="INDETERMINATE":return result("INDETERMINATE","activatecredential-secret-equality-indeterminate")
    if m["secret_equality_state"]!="PASS":return result("DENY","secret-equality-state-invalid")
    if m["registrar_secret_sha256"]!=m["recovered_secret_sha256"]:return result("DENY","challenge-recovery-digest-mismatch")
    if session_binding(m)!=m["session_binding_sha256"]:return result("DENY","session-binding-mismatch")
    return result("PASS","activatecredential-receipt-coherent",{
        "ek_public_sha256":m["ek_public_sha256"],
        "ek_public_wire_sha256":m["ek_public_wire_sha256"],
        "ak_name_sha256":m["ak_name_sha256"],
        "registrar_secret_sha256":m["registrar_secret_sha256"],
        "recovered_secret_sha256":m["recovered_secret_sha256"],
        "makecredential_blob_sha256":m["makecredential_blob_sha256"],
        "verification_mode":m["verification_mode"]
    })

def fixture()->dict[str,Any]:
    secret=bytes.fromhex("00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff")
    ss=hashlib.sha256(secret).hexdigest()
    m={
        "profile_id":"mycelix.security.tpm.activatecredential-binding",
        "profile_version":"0.1.0",
        "verification_mode":"ReferenceModelOnly",
        "claim_ceiling":"ReferenceModelOnly",
        "session_id":"activatecredential-self-test",
        "tpm_identity_digest":"11"*32,
        "ek_public_sha256":"22"*32,
        "ek_public_wire_sha256":"33"*32,
        "ak_name_sha256":"44"*32,
        "registrar_secret_sha256":ss,
        "recovered_secret_sha256":ss,
        "makecredential_blob_sha256":"55"*32,
        "makecredential_transcript_sha256":"66"*32,
        "activatecredential_transcript_sha256":"77"*32,
        "capture_source_sha256":sha256_file(CAPTURE_SOURCE_SCRIPT),
        "makecredential_returncode":0,
        "activatecredential_returncode":0,
        "secret_equality_state":"PASS"
    }
    m["session_binding_sha256"]=session_binding(m)
    return m

def self_test()->int:
    base=fixture()
    cases=[
        ("canonical","PASS",lambda x:None),
        ("registrar-secret-substitution","DENY",lambda x:x.update({"registrar_secret_sha256":"aa"*32})),
        ("recovered-secret-substitution","DENY",lambda x:x.update({"recovered_secret_sha256":"bb"*32})),
        ("makecredential-blob-substitution","DENY",lambda x:x.update({"makecredential_blob_sha256":"cc"*32})),
        ("makecredential-transcript-substitution","DENY",lambda x:x.update({"makecredential_transcript_sha256":"dd"*32})),
        ("activatecredential-transcript-substitution","DENY",lambda x:x.update({"activatecredential_transcript_sha256":"ee"*32})),
        ("ek-public-substitution","DENY",lambda x:x.update({"ek_public_sha256":"ff"*32})),
        ("ek-wire-substitution","DENY",lambda x:x.update({"ek_public_wire_sha256":"12"*32})),
        ("ak-name-substitution","DENY",lambda x:x.update({"ak_name_sha256":"13"*32})),
        ("makecredential-failure","DENY",lambda x:x.update({"makecredential_returncode":1})),
        ("activatecredential-failure","DENY",lambda x:x.update({"activatecredential_returncode":1})),
        ("equality-deny","DENY",lambda x:x.update({"secret_equality_state":"DENY"})),
        ("equality-indeterminate","INDETERMINATE",lambda x:x.update({"secret_equality_state":"INDETERMINATE"})),
        ("verification-mode-live","PASS",lambda x:x.update({"verification_mode":"LiveVerifierSession"})),
        ("source-substitution","DENY",lambda x:x.update({"capture_source_sha256":"14"*32})),
        ("session-substitution","DENY",lambda x:x.update({"session_id":"attacker"}))
    ]
    for name,expected,mut in cases:
        c=json.loads(json.dumps(base))
        mut(c)
        c["session_binding_sha256"]=session_binding(c)
        got=verify(c)["state"]
        if got!=expected:
            print(f"{name}: FAIL expected={expected} got={got}")
            return 1
    print("ActivateCredential association semantic corpus: PASS")
    print("15 adversarial cases plus canonical reference model: PASS")
    return 0

def main()->int:
    p=argparse.ArgumentParser()
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument("--self-test",action="store_true")
    g.add_argument("--verify",metavar="MANIFEST")
    a=p.parse_args()
    if a.self_test:return self_test()
    m=json.loads(Path(a.verify).read_text(encoding="utf-8"))
    v=verify(m)
    print(json.dumps(v,indent=2,sort_keys=True))
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]

if __name__=="__main__":
    raise SystemExit(main())
