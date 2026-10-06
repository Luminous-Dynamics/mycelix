#!/usr/bin/env python3
from __future__ import annotations
import argparse,copy,hashlib,json
from pathlib import Path
from typing import Any
VERIFIER_ID="mycelix.tpm.ek-cert-san-identity-binding.v0.1"
EXTRACTOR_VERIFIER_ID="mycelix.tpm.ek-cert-san-extractor.v0.1"
EXTRACTOR_SCRIPT=Path(__file__).with_name("extract_mycelix_ek_cert_san_v0_1.py")
def canonical_hash(v:Any)->str:
    return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()
def valid_hash(v:Any)->bool:
    return isinstance(v,str) and len(v)==64 and all(c in "0123456789abcdef" for c in v)
def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details is not None: out["details"]=details
    return out
def execute_extractor(cert:bytes)->tuple[dict[str,Any],str,str]:
    if not EXTRACTOR_SCRIPT.is_file():
        raise RuntimeError("EK SAN extractor missing")
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-reexecution-") as td:
        work=Path(td)
        cert_path=work/"leaf.der"
        output_path=work/"extractor-output.json"
        cert_path.write_bytes(cert)
        proc=subprocess.run(
            [sys.executable,str(EXTRACTOR_SCRIPT),"--extract",str(cert_path),"--output",str(output_path)],
            cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False
        )
        if proc.returncode not in (0,1,2):
            raise RuntimeError(f"EK SAN extractor execution error: {proc.stderr}")
        if not output_path.is_file():
            raise RuntimeError("EK SAN extractor produced no output")
        try:
            generated=json.loads(output_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"EK SAN extractor output invalid: {exc}") from exc
        return generated,hashlib.sha256(cert).hexdigest(),hashlib.sha256(output_path.read_bytes()).hexdigest()

def validate_extractor_output(generated:dict[str,Any],cert_sha256:str)->None:
    if generated.get("verifier_id")!=EXTRACTOR_VERIFIER_ID:
        raise ValueError("extractor-verifier-id-mismatch")
    if generated.get("input_sha256")!=cert_sha256:
        raise ValueError("extractor-input-digest-mismatch")
    if generated.get("source_sha256")!=hashlib.sha256(EXTRACTOR_SCRIPT.read_bytes()).hexdigest():
        raise ValueError("extractor-source-digest-mismatch")
    content=generated.get("content_sha256")
    if not valid_hash(content):
        raise ValueError("extractor-content-digest-invalid")
    if content!=canonical_hash({k:v for k,v in generated.items() if k!="content_sha256"}):
        raise ValueError("extractor-content-digest-mismatch")

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
    if not isinstance(san,dict):return result("DENY","san-extractor-identity-invalid")
    for field in ("verifier_id","state","input_sha256","output_sha256","source_sha256","content_sha256"):
        if field not in san:return result("DENY","san-extraction-field-missing",{"field":field})
    if san.get("verifier_id")!=EXTRACTOR_VERIFIER_ID:return result("DENY","san-extractor-identity-invalid")
    for field in ("input_sha256","output_sha256","source_sha256","content_sha256"):
        if not valid_hash(san.get(field)):return result("DENY","san-extraction-digest-invalid",{"field":field})
    if san.get("source_sha256")!=hashlib.sha256(EXTRACTOR_SCRIPT.read_bytes()).hexdigest():return result("DENY","san-extractor-source-mismatch")
    if san.get("input_sha256")!=m["certificate_der_sha256"]:return result("DENY","san-extractor-input-mismatch")
    if san.get("state")=="INDETERMINATE":return result("INDETERMINATE","san-extraction-indeterminate")
    if san.get("state")!="PASS":return result("DENY","san-extraction-not-pass")
    if san.get("details",{}).get("certificate_der_sha256")!=m["certificate_der_sha256"]:return result("DENY","san-certificate-digest-mismatch")
    try:
        generated,cert_sha256,output_sha256=execute_extractor(cert)
        validate_extractor_output(generated,cert_sha256)
    except (OSError,RuntimeError,ValueError) as exc:
        return result("DENY","san-extractor-reexecution-failed",{"error":str(exc)})
    if cert_sha256!=m["certificate_der_sha256"]:return result("DENY","san-extractor-certificate-digest-mismatch")
    if output_sha256!=san["output_sha256"]:return result("DENY","san-extractor-output-digest-mismatch")
    if canonical_hash(generated)!=m["san_extraction_sha256"]:return result("DENY","san-extraction-reexecution-receipt-mismatch")
    tpm=m["tpm_identity"]
    for field in ("source_sha256","manufacturer_source_sha256","model_source_sha256","part_number_source_sha256","issuance_firmware_source_sha256"):
        if not valid_hash(tpm.get(field)):return result("DENY","tpm-source-digest-invalid",{"field":field})
    for field in ("manufacturer_name","model","model_state","part_number","part_number_state","issuance_firmware_state"):
        if field not in tpm:return result("DENY","tpm-identity-field-missing",{"field":field})
    if m["session_binding_sha256"]!=session_binding(m):return result("DENY","session-binding-mismatch")
    d={"manufacturer_certificate":san["details"].get("manufacturer"),"manufacturer_observed":tpm.get("manufacturer_name"),"model_certificate":san["details"].get("model"),"model_observed":tpm.get("model"),"part_number_certificate":san["details"].get("model"),"part_number_observed":tpm.get("part_number"),"firmware_certificate_at_issuance":san["details"].get("version"),"firmware_observed_at_issuance":tpm.get("issuance_firmware"),"current_firmware":tpm.get("current_firmware")}
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
    if shutil.which("openssl") is None:
        raise RuntimeError("openssl unavailable")
    cfg="""[req]
distinguished_name=req_dn
prompt=no
x509_extensions=ext
[req_dn]
CN=Mycelix EK SAN Reexecution Fixture
[ext]
subjectAltName=@alts
basicConstraints=critical,CA:false
[alts]
dirName=dn
[dn]
tcg-at-tpmManufacturer=TEST-MFR
tcg-at-tpmModel=TEST-MODEL
tcg-at-tpmVersion=FW-1.0
"""
    with tempfile.TemporaryDirectory(prefix="mycelix-san-fixture-") as td:
        work=Path(td);cfgp=work/"openssl.cnf";key=work/"key.pem";cert=work/"cert.pem";der=work/"leaf.der";out=work/"extractor.json"
        cfgp.write_text(cfg,encoding="utf-8")
        p=subprocess.run(["openssl","req","-new","-x509","-newkey","rsa:2048","-nodes","-days","7","-keyout",str(key),"-out",str(cert),"-config",str(cfgp)],cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
        if p.returncode!=0: raise RuntimeError(f"fixture certificate generation failed: {p.stderr}")
        p=subprocess.run(["openssl","x509","-in",str(cert),"-outform","DER","-out",str(der)],cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
        if p.returncode!=0: raise RuntimeError(f"fixture DER conversion failed: {p.stderr}")
        cert_bytes=der.read_bytes()
        p=subprocess.run([sys.executable,str(EXTRACTOR_SCRIPT),"--extract",str(der),"--output",str(out)],cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
        if p.returncode!=0 or not out.is_file(): raise RuntimeError(f"fixture extractor failed: {p.stderr}")
        san=json.loads(out.read_text(encoding="utf-8"))
        cert_sha=hashlib.sha256(cert_bytes).hexdigest()
        tpm={"source_sha256":"22"*32,"manufacturer_source_sha256":"23"*32,"model_source_sha256":"24"*32,"part_number_source_sha256":"25"*32,"issuance_firmware_source_sha256":"26"*32,"manufacturer_name":"TEST-MFR","model":"TEST-MODEL","model_state":"PASS","part_number":"TEST-MODEL","part_number_state":"PASS","issuance_firmware":"FW-1.0","issuance_firmware_state":"PASS","current_firmware":"FW-2.0"}
        m={"profile_id":"mycelix.security.tpm.ek-cert-san-identity-binding","profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","session_id":"san-self-test","certificate_der_sha256":cert_sha,"san_extraction":san,"tpm_identity":tpm}
        m["san_extraction_sha256"]=canonical_hash(san);m["session_binding_sha256"]=session_binding(m);return m

def self_test()->int:
    base=fixture()
    cases=[("canonical-valid","PASS",lambda x:x),("manufacturer-mismatch","DENY",lambda x:x["tpm_identity"].update({"manufacturer_name":"OTHER"})),("model-mismatch","DENY",lambda x:x["tpm_identity"].update({"model":"OTHER"})),("part-number-mismatch","DENY",lambda x:x["tpm_identity"].update({"part_number":"OTHER"})),("model-indeterminate","INDETERMINATE",lambda x:x["tpm_identity"].update({"model_state":"INDETERMINATE"})),("part-number-indeterminate","INDETERMINATE",lambda x:x["tpm_identity"].update({"part_number_state":"INDETERMINATE"})),("firmware-indeterminate","INDETERMINATE",lambda x:x["tpm_identity"].update({"issuance_firmware_state":"INDETERMINATE"})),("firmware-mismatch","DENY",lambda x:x["tpm_identity"].update({"issuance_firmware":"FW-9.0"})),("san-certificate-splice","DENY",lambda x:x.update({"certificate_der_sha256":"33"*32})),("san-receipt-splice","DENY",lambda x:x.update({"san_extraction_sha256":"44"*32})),("extractor-substitution","DENY",lambda x:x["san_extraction"].update({"verifier_id":"other"})),("extractor-source-substitution","DENY",lambda x:x["san_extraction"].update({"source_sha256":"12"*32})),("extractor-input-substitution","DENY",lambda x:x["san_extraction"].update({"input_sha256":"13"*32})),("extractor-output-substitution","DENY",lambda x:x["san_extraction"].update({"output_sha256":"14"*32})),("source-splice","DENY",lambda x:x["tpm_identity"].update({"source_sha256":"55"*32})),("session-splice","DENY",lambda x:x.update({"session_id":"attacker"})),("offline-origin","INDETERMINATE",lambda x:x.update({"verification_mode":"OfflineBundle"})),("live-origin","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession"}))]
    for name,expected,mut in cases:
        c=copy.deepcopy(base);mut(c);o=verify(c)
        if o["state"]!=expected:print(f"{name}: FAIL expected={expected} got={o['state']} reason={o['reason']}");return 1
    p=json.loads(json.dumps(base,sort_keys=True))
    if verify(p)["state"]!="PASS":print("key-order-permutation: FAIL");return 1
    print("EK SAN identity binding semantic corpus: PASS");print("17 adversarial mutations plus canonical and key-order control: PASS");return 0
def main()->int:
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument("--self-test",action="store_true");g.add_argument("--verify",metavar="MANIFEST");p.add_argument("--output");a=p.parse_args()
    if a.self_test:return self_test()
    m=json.loads(Path(a.verify).read_text(encoding="utf-8"));v=verify(m);o={"profile_id":"mycelix.security.tpm.ek-cert-san-identity-binding","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(Path(a.verify).read_bytes()).hexdigest(),**v};o["content_sha256"]=canonical_hash({k:v for k,v in o.items() if k!="content_sha256"});r=json.dumps(o,indent=2,sort_keys=True)+"\n"
    if a.output:Path(a.output).write_text(r,encoding="utf-8")
    else:print(r,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]
if __name__=="__main__":raise SystemExit(main())
