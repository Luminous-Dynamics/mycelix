#!/usr/bin/env python3
"""Bind TCG EK certificate SAN identity fields to exact TPM fixed properties."""
from __future__ import annotations
import argparse,base64,copy,hashlib,json,re,shutil,subprocess,tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID="mycelix.tpm.ek-cert-san-property-binding.v0.1"
MAPPING_ID="mycelix.synthetic.ek-san-property-map.v0.1"
MAPPING_CONTENT={
 "manufacturer":"4D594358",
 "vendor_tpm_type":"00000000",
 "firmware_version_1":"00000000",
 "firmware_version_2":"00010002",
 "certificate_manufacturer":"id:4D594358",
 "certificate_model":"SyntheticEK-Model",
 "certificate_version":"id:00010002",
}
MAPPING_SHA256="994e81d09a2bed5d2a1a569e7dded6459392676a26d18552dbb4f3599b4ff3e7"
FIXTURE_CERT_DER=base64.b64decode("MIIDOjCCAiKgAwIBAgIBBzANBgkqhkiG9w0BAQsFADAfMR0wGwYDVQQDDBRNeWNlbGl4IFN5bnRoZXRpYyBFSzAeFw0yNjEwMDUyMjQ4MDJaFw0zNjEwMDIyMjQ5MDJaMB8xHTAbBgNVBAMMFE15Y2VsaXggU3ludGhldGljIEVLMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAAlAiIZEv8RMiji8E8xXvKv0ScWAG8fQCpTYCsf30sEP2WdoFZdVb4jUtcaMFMyeAdHNrlrtnZFZxDFXQNewOQEpFg53BVEHUeUyxQBlkv6o43WJoIKrIHk1kfIh7FkRXhtg7T22j6DEE0Y5wR6eEVAQ4iy8YQohbxt35HNaWoadO8wgCSw28ECuiqoWTYvT65oanRwGZUdcrm63YEx5G8kw0PJwwwOmeLrY6lUyDkOC3VZ9JxqP8Na15yKCasvKIDRHn4gztIRGYOTwM3M2Xbai6ZeHxn9SPeSCVST1sI3fL5tuS3qX6Z418+Ii12r4rOvta/QuwluBB0Jk87esbBsQIDAQABo4GAMH4wXgYDVR0RAQH/BFQwUqRQME4xFjAUBgVngQUCAQwLaWQ6NEQ1OTQzNTgxHDAaBgVngQUCAgwRU3ludGhldGljRUstTW9kZWwxFjAUBgVngQUCAwwLaWQ6MDAwMTAwMDIwDAYDVR0TAQH/BAIwADAOBgNVHQ8BAf8EBAMCBSAwDQYJKoZIhvcNAQELBQADggEBACEJ79Q6WYyFzlz/LHDV3KQRN4MWnkri0zQXECGL699kvuY+ZeSnG+uw+/OY5h2rX29gQgYteROtWZTLMm4lvnO9UOTIqxsln47Y0IFAcS3ua5r+J8GOvMFRDEUczfVGECKDBZqhrTKtLF5ApOoS09bp/nNXM01MLFlET2e1n0HJAMkTKER6aa7yqbT+5HE0eDN3Gn2h1cGo9hOb6Ga3tPvyMuSWLoMyXjGp7LM6uMRtme6ENowFaVAIMR8WpkG2iN9/b8d+SUp3RSYygAvKfHink/afex8AEEPFjGMFecRmHvrx/xNgfAJJlYCCUbTIr56sY9JG+5/werDU+LOFQRE=")
FIXTURE_BAD_CERT_DER=base64.b64decode("MIIDMjCCAhqgAwIBAgIBCDANBgkqhkiG9w0BAQsFADAfMR0wGwYDVQQDDBRNeWNlbGl4IFN5bnRoZXRpYyBFSzAeFw0yNjEwMDUyMjQ4MzhaFw0zNjEwMDIyMjQ5MzhaMB8xHTAbBgNVBAMMFE15Y2VsaXggU3ludGhldGljIEVLMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAmmzLEBRz/Ebof6fx/Zuh9Pv9q0x4otvdT/KcwXxbxT3WNd1OdWVThNpkZzxRGtGajqgQ7lPDS0+RFRV98cl8Nf0BOZe8RdpN/PdEMDORE0r5WyGl9SZfX59RRg8i9LN67LGxqx1XvDEjSWWUSLWYQnUkcrjMCVgxNKr/dUOTigxnj0Md+1/ukTHfCf3JHAs9AkXTNLnKHEkbZav65OHwP3jU6fchxSIS9uAiAJjMQli/J6tI6l2y5K65OjZnziaWuD8ekwOy0F0m24ckSwblhh7mcDYcojFEuLX6FWTqyC13xz/ZmIGi83rr6bfszcixyAErXq1h3YIXXWCFstJOHwIDAQABo3kwdzBXBgNVHREBAf8ETTBLpEkwRzEWMBQGBWeBBQIBDAtpZDo0RDU5NDM1ODEVMBMGBWeBBQICDApPdGhlck1vZGVsMRYwFAYFZ4EFAgMMC2lkOjAwMDEwMDAzMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMA0GCSqGSIb3DQEBCwUAA4IBAQBfNIy/bxha+iwZkrfkyYXQxZlwz8UAhQ4cZmg6kmwaMHGVtd55rXocGKTJt1ODWY3Fbz4yZaLsVexliXCE7LQoJNzdjjCmAtFQt45xQ9aNJ5hRji8LQOq+jArdpoNiaVf5R5cUAwLBptW9ThoSHmguA44TKkx5naMIKC35SdeCjcpkwoxKd/yq+SB3EZIRS6110H0m5Oncj1PCEbjid68C2FP84aRfXKvrXcb/ELhC23ifcn4ShaAQlJhlyABFlDC47gFcwfWVUhAvKHXnnnAsJw5AotOLSDNQbofh5kixU+OP22VrQl09AZJRzdOLWc8dH57l5cdY+nvssU5hHbcZ")
MANUFACTURER_OID="2.23.133.2.1"
MODEL_OID="2.23.133.2.2"
VERSION_OID="2.23.133.2.3"

def canonical_hash(v:Any)->str:
 return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()

def valid_hash(v:Any)->bool:
 return isinstance(v,str) and len(v)==64 and all(c in "0123456789abcdef" for c in v)

def cert_san(cert_der:bytes,work:Path)->dict[str,str]:
 if shutil.which("openssl") is None: raise RuntimeError("openssl executable not found")
 p=work/"leaf.der";p.write_bytes(cert_der)
 r=subprocess.run(["openssl","x509","-inform","DER","-in",str(p),"-noout","-ext","subjectAltName"],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
 if r.returncode!=0: raise RuntimeError(f"OpenSSL SAN extraction failed: {r.stderr}")
 out={}
 for oid,key in ((MANUFACTURER_OID,"manufacturer"),(MODEL_OID,"model"),(VERSION_OID,"version")):
  m=re.search(rf"/{re.escape(oid)}=([^/\n]+)",r.stdout)
  if not m: raise ValueError(f"missing SAN OID {oid}")
  out[key]=m.group(1).strip()
 return out

def expected_from_properties(p:dict[str,Any],registry_id:str,registry_sha256:str)->tuple[str,dict[str,str]|None]:
 if registry_id!=MAPPING_ID or registry_sha256!=MAPPING_SHA256:return "DENY",None
 raw={f:p.get(f) for f in ("manufacturer_hex","vendor_tpm_type_hex","firmware_version_1_hex","firmware_version_2_hex")}
 if any(not isinstance(v,str) or not re.fullmatch(r"[0-9A-Fa-f]{8}",v) for v in raw.values()):return "DENY",None
 key=tuple(raw[k].upper() for k in ("manufacturer_hex","vendor_tpm_type_hex","firmware_version_1_hex","firmware_version_2_hex"))
 ref=tuple(MAPPING_CONTENT[k] for k in ("manufacturer","vendor_tpm_type","firmware_version_1","firmware_version_2"))
 if key!=ref:return "INDETERMINATE",None
 return "PASS",{
  "manufacturer":MAPPING_CONTENT["certificate_manufacturer"],
  "model":MAPPING_CONTENT["certificate_model"],
  "version":MAPPING_CONTENT["certificate_version"],
 }

def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
 out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
 if details is not None:out["details"]=details
 return out

def session_binding(m:dict[str,Any])->str:
 return canonical_hash({
  "certificate_sha256":m["leaf_certificate_sha256"],
  "properties_fixed":m["properties_fixed"],
  "properties_source_sha256":m["properties_source_sha256"],
  "registry_id":m["registry_id"],
  "registry_sha256":m["registry_sha256"],
  "spki_binding_state":m["spki_binding_state"],
 })

def verify(m:dict[str,Any])->dict[str,Any]:
 req={"profile_id","profile_version","verification_mode","claim_ceiling","leaf_certificate_der_base64","leaf_certificate_sha256","properties_fixed","properties_source_sha256","registry_id","registry_sha256","spki_binding_state","session_binding_sha256"}
 missing=sorted(req-set(m))
 if missing:return result("DENY","missing-required-fields",{"fields":missing})
 if m["profile_id"]!="mycelix.security.tpm.ek-cert-san-property-binding":return result("DENY","profile-id-mismatch")
 if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
 if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
 if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
 try: cert=base64.b64decode(m["leaf_certificate_der_base64"],validate=True)
 except Exception as exc:return result("DENY","certificate-base64-invalid",{"error":str(exc)})
 if not valid_hash(m["leaf_certificate_sha256"]) or hashlib.sha256(cert).hexdigest()!=m["leaf_certificate_sha256"]:return result("DENY","certificate-digest-mismatch")
 if not valid_hash(m["properties_source_sha256"]):return result("DENY","property-source-digest-invalid")
 mapping_digest=hashlib.sha256(json.dumps(MAPPING_CONTENT,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()
 if mapping_digest!=MAPPING_SHA256:return result("DENY","built-in-mapping-integrity-failure")
 state,expected=expected_from_properties(m["properties_fixed"],m["registry_id"],m["registry_sha256"])
 if state=="DENY":return result("DENY","registry-not-authorized")
 if state=="INDETERMINATE":return result("INDETERMINATE","vendor-specific-property-mapping-unknown")
 with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-") as td:
  try:observed=cert_san(cert,Path(td))
  except (RuntimeError,ValueError) as exc:return result("DENY","certificate-san-parse-failed",{"error":str(exc)})
 if observed!=expected:return result("DENY","certificate-san-does-not-match-tpm-properties",{"observed":observed,"expected":expected})
 if m["spki_binding_state"]=="DENY":return result("DENY","spki-binding-denied")
 if m["spki_binding_state"]=="INDETERMINATE":return result("INDETERMINATE","spki-binding-indeterminate")
 if m["spki_binding_state"]!="PASS":return result("DENY","spki-binding-state-invalid")
 if m["session_binding_sha256"]!=session_binding(m):return result("DENY","session-binding-mismatch")
 if m["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","live-property-mapping-not-authorized-by-reference-model")
 return result("PASS","ek-certificate-san-bound-to-tpm-properties",{"observed":observed,"expected":expected,"registry_id":MAPPING_ID,"registry_sha256":MAPPING_SHA256})

def fixture()->dict[str,Any]:
 cert=FIXTURE_CERT_DER;props={"manufacturer_hex":"4D594358","vendor_tpm_type_hex":"00000000","firmware_version_1_hex":"00000000","firmware_version_2_hex":"00010002"}
 m={"profile_id":"mycelix.security.tpm.ek-cert-san-property-binding","profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","leaf_certificate_der_base64":base64.b64encode(cert).decode(),"leaf_certificate_sha256":hashlib.sha256(cert).hexdigest(),"properties_fixed":props,"properties_source_sha256":"66"*32,"registry_id":MAPPING_ID,"registry_sha256":MAPPING_SHA256,"spki_binding_state":"PASS","session_binding_sha256":""}
 m["session_binding_sha256"]=session_binding(m);return m

def self_test()->int:
 base=fixture()
 cases=[
  ("canonical-match","PASS",lambda x:x),
  ("certificate-san-substitution","DENY",lambda x:x.update({"leaf_certificate_der_base64":base64.b64encode(FIXTURE_BAD_CERT_DER).decode(),"leaf_certificate_sha256":hashlib.sha256(FIXTURE_BAD_CERT_DER).hexdigest()})),
  ("manufacturer-property-substitution","INDETERMINATE",lambda x:x["properties_fixed"].update({"manufacturer_hex":"4D594359"})),
  ("model-mapping-substitution","INDETERMINATE",lambda x:x["properties_fixed"].update({"vendor_tpm_type_hex":"00000001"})),
  ("firmware-property-substitution","INDETERMINATE",lambda x:x["properties_fixed"].update({"firmware_version_2_hex":"00010003"})),
  ("registry-id-substitution","DENY",lambda x:x.update({"registry_id":"other"})),
  ("registry-digest-substitution","DENY",lambda x:x.update({"registry_sha256":"77"*32})),
  ("property-source-substitution","DENY",lambda x:x.update({"properties_source_sha256":"88"*32})),
  ("certificate-digest-substitution","DENY",lambda x:x.update({"leaf_certificate_sha256":"99"*32})),
  ("spki-deny","DENY",lambda x:x.update({"spki_binding_state":"DENY"})),
  ("spki-indeterminate","INDETERMINATE",lambda x:x.update({"spki_binding_state":"INDETERMINATE"})),
  ("session-binding-substitution","DENY",lambda x:x.update({"session_binding_sha256":"aa"*32})),
 ]
 for name,expected,mut in cases:
  c=copy.deepcopy(base);mut(c)
  o=verify(c)
  if o["state"]!=expected:
   print(f"{name}: FAIL expected={expected} got={o['state']} reason={o['reason']}");return 1
 if verify(json.loads(json.dumps(base,sort_keys=True)))["state"]!="PASS":
  print("key-order-permutation: FAIL");return 1
 print("EK certificate SAN -> TPM property binding corpus: PASS")
 print("12 mutation/canonical cases plus key-order control: PASS")
 return 0

def main()->int:
 ap=argparse.ArgumentParser();g=ap.add_mutually_exclusive_group(required=True);g.add_argument("--self-test",action="store_true");g.add_argument("--verify",metavar="MANIFEST");ap.add_argument("--output");a=ap.parse_args()
 if a.self_test:return self_test()
 p=Path(a.verify).resolve();m=json.loads(p.read_text(encoding="utf-8"));v=verify(m)
 out={"profile_id":"mycelix.security.tpm.ek-cert-san-property-binding","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(p.read_bytes()).hexdigest(),**v}
 out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"});rendered=json.dumps(out,indent=2,sort_keys=True)+"\n"
 if a.output:Path(a.output).write_text(rendered,encoding="utf-8")
 else:print(rendered,end="")
 return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]

if __name__=="__main__":raise SystemExit(main())
