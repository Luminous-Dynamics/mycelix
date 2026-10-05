#!/usr/bin/env python3
from __future__ import annotations
import argparse,copy,hashlib,json
from pathlib import Path
from typing import Any
VERIFIER_ID="mycelix.tpm.ek-cert-san-identity-binding.v0.1"
EXTRACTOR_VERIFIER_ID="mycelix.tpm.ek-cert-san-extractor.v0.1"
def canonical_hash(v:Any)->str:
    return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()
def valid_hash(v:Any)->bool:
    return isinstance(v,str) and len(v)==64 and all(c in "0123456789abcdef" for c in v)
def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details is not None: out["details"]=details
    return out
def session_binding(m:dict[str,Any])->str:
    san=m["san_extraction"];tpm=m["tpm_identity"]
    return canonical_hash({"session_id":m["session_id"],"certificate_der_sha256":m["certificate_der_sha256"],"san_extraction_sha256":m["san_extraction_sha256"],"tpm_source_sha256":tpm["source_sha256"],"manufacturer_source_sha256":tpm["manufacturer_source_sha256"],"model_source_sha256":tpm["model_source_sha256"],"part_number_source_sha256":tpm["part_number_source_sha256"],"issuance_firmware_source_sha256":tpm["issuance_firmware_source_sha256"],"manufacturer":san.get("details",{}).get("manufacturer"),"model":san.get("details",{}).get("model"),"version":san.get("details",{}).get("version"),"part_number":tpm.get("part_number"),"issuance_firmware":tpm.get("issuance_firmware")})
def verify(m:dict[str,Any])->dict[str,Any]:
    required={"profile_id","profile_version","verification_mode","claim_ceiling","session_id","certificate_der_sha256","san_extraction","san_extraction_sha256","tpm_identity","session_binding_sha256"}
    missing=sorted(required-set(m))
    if missing:return result("DENY","missing-required-fields",{"fields":missing})
    if m["profile_id"]!="mycelix.security.tpm.ek-cert-san-identity-binding":return result("DENY","profile-id-mismatch")
    if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    if not valid_hash(m["certificate_der_sha256"]) or not valid_hash(m["san_extraction_sha256"]) or not valid_hash(m["session_binding_sha256"]):return result("DENY","digest-invalid")
    san=m["san_extraction"]
    if not isinstance(san,dict) or san.get("verifier_id")!=EXTRACTOR_VERIFIER_ID:return result("DENY","san-extractor-identity-invalid")
    if san.get("state")=="INDETERMINATE":return result("INDETERMINATE","san-extraction-indeterminate")
    if san.get("state")!="PASS":return result("DENY","san-extraction-not-pass")
    if san.get("details",{}).get("certificate_der_sha256")!=m["certificate_der_sha256"]:return result("DENY","san-certificate-digest-mismatch")
    if canonical_hash(san)!=m["san_extraction_sha256"]:return result("DENY","san-extraction-receipt-digest-mismatch")
    tpm=m["tpm_identity"]
    for field in ("source_sha256","manufacturer_source_sha256","model_source_sha256","part_number_source_sha256","issuance_firmware_source_sha256"):
        if not valid_hash(tpm.get(field)):return result("DENY","tpm-source-digest-invalid",{"field":field})
    for field in ("manufacturer_name","model","model_state","part_number","part_number_state","issuance_firmware_state"):
        if field not in tpm:return result("DENY","tpm-identity-field-missing",{"field":field})
    if m["session_binding_sha256"]!=session_binding(m):return result("DENY","session-binding-mismatch")
    d={"manufacturer_certificate":san["details"].get("manufacturer"),"manufacturer_observed":tpm.get("manufacturer_name"),"model_certificate":san["details"].get("model"),"model_observed":tpm.get("model"),"firmware_certificate_at_issuance":san["details"].get("version"),"firmware_observed_at_issuance":tpm.get("issuance_firmware"),"current_firmware":tpm.get("current_firmware")}
    if d["manufacturer_certificate"]!=d["manufacturer_observed"]:return result("DENY","manufacturer-mismatch",d)
    if tpm["model_state"]!="PASS":return result("INDETERMINATE","model-source-indeterminate",d)
    if d["model_certificate"]!=d["model_observed"]:return result("DENY","model-mismatch",d)
    if tpm["part_number_state"]!="PASS":return result("INDETERMINATE","part-number-source-indeterminate",d)
    if d["part_number_certificate"]!=d["part_number_observed"]:return result("DENY","part-number-mismatch",d)
    if tpm["issuance_firmware_state"]=="DENY":return result("DENY","issuance-firmware-source-denied",d)
    if tpm["issuance_firmware_state"]=="INDETERMINATE":return result("INDETERMINATE","issuance-firmware-source-indeterminate",d)
    if d["firmware_certificate_at_issuance"]!=d["firmware_observed_at_issuance"]:return result("DENY","issuance-firmware-mismatch",d)
    if m["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","live-origin-not-authorized-by-reference-model",d)
    return result("PASS","ek-san-identity-bound-to-observed-tpm-properties",d)
def fixture()->dict[str,Any]:
    san={"verifier_id":EXTRACTOR_VERIFIER_ID,"state":"PASS","details":{"certificate_der_sha256":"11"*32,"manufacturer":"TEST-MFR","model":"TEST-MODEL","version":"FW-1.0"}}
    tpm={"source_sha256":"22"*32,"manufacturer_source_sha256":"23"*32,"model_source_sha256":"24"*32,"part_number_source_sha256":"25"*32,"issuance_firmware_source_sha256":"26"*32,"manufacturer_name":"TEST-MFR","model":"TEST-MODEL","model_state":"PASS","part_number":"TEST-MODEL","part_number_state":"PASS","issuance_firmware":"FW-1.0","issuance_firmware_state":"PASS","current_firmware":"FW-2.0"}
    m={"profile_id":"mycelix.security.tpm.ek-cert-san-identity-binding","profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","session_id":"san-self-test","certificate_der_sha256":"11"*32,"san_extraction":san,"tpm_identity":tpm}
    m["san_extraction_sha256"]=canonical_hash(san);m["session_binding_sha256"]=session_binding(m);return m
def self_test()->int:
    base=fixture()
    cases=[("canonical-valid","PASS",lambda x:x),("manufacturer-mismatch","DENY",lambda x:x["tpm_identity"].update({"manufacturer_name":"OTHER"})),("model-mismatch","DENY",lambda x:x["tpm_identity"].update({"model":"OTHER"})),("part-number-mismatch","DENY",lambda x:x["tpm_identity"].update({"part_number":"OTHER"})),("model-indeterminate","INDETERMINATE",lambda x:x["tpm_identity"].update({"model_state":"INDETERMINATE"})),("part-number-indeterminate","INDETERMINATE",lambda x:x["tpm_identity"].update({"part_number_state":"INDETERMINATE"})),("firmware-indeterminate","INDETERMINATE",lambda x:x["tpm_identity"].update({"issuance_firmware_state":"INDETERMINATE"})),("firmware-mismatch","DENY",lambda x:x["tpm_identity"].update({"issuance_firmware":"FW-9.0"})),("san-certificate-splice","DENY",lambda x:x.update({"certificate_der_sha256":"33"*32})),("san-receipt-splice","DENY",lambda x:x.update({"san_extraction_sha256":"44"*32})),("extractor-substitution","DENY",lambda x:x["san_extraction"].update({"verifier_id":"other"})),("source-splice","DENY",lambda x:x["tpm_identity"].update({"source_sha256":"55"*32})),("session-splice","DENY",lambda x:x.update({"session_id":"attacker"})),("offline-origin","INDETERMINATE",lambda x:x.update({"verification_mode":"OfflineBundle"})),("live-origin","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession"}))]
    for name,expected,mut in cases:
        c=copy.deepcopy(base);mut(c);o=verify(c)
        if o["state"]!=expected:print(f"{name}: FAIL expected={expected} got={o['state']} reason={o['reason']}");return 1
    p=json.loads(json.dumps(base,sort_keys=True))
    if verify(p)["state"]!="PASS":print("key-order-permutation: FAIL");return 1
    print("EK SAN identity binding semantic corpus: PASS");print("14 adversarial mutations plus canonical and key-order control: PASS");return 0
def main()->int:
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument("--self-test",action="store_true");g.add_argument("--verify",metavar="MANIFEST");p.add_argument("--output");a=p.parse_args()
    if a.self_test:return self_test()
    m=json.loads(Path(a.verify).read_text(encoding="utf-8"));v=verify(m);o={"profile_id":"mycelix.security.tpm.ek-cert-san-identity-binding","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(Path(a.verify).read_bytes()).hexdigest(),**v};o["content_sha256"]=canonical_hash({k:v for k,v in o.items() if k!="content_sha256"});r=json.dumps(o,indent=2,sort_keys=True)+"\n"
    if a.output:Path(a.output).write_text(r,encoding="utf-8")
    else:print(r,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]
if __name__=="__main__":raise SystemExit(main())
