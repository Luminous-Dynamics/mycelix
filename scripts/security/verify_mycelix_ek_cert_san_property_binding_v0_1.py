#!/usr/bin/env python3
"""Bind TCG EK certificate SAN identity fields to exact TPM fixed properties."""
from __future__ import annotations
import argparse,base64,copy,hashlib,json,re,shutil,subprocess,tempfile,sys
from pathlib import Path
from typing import Any

VERIFIER_ID="mycelix.tpm.ek-cert-san-property-binding.v0.1"
PROPERTIES_VERIFIER_ID="mycelix.tpm.properties-fixed-capture.v0.1"
PROPERTIES_VERIFIER_SCRIPT=Path(__file__).with_name("capture_mycelix_tpm_properties_fixed_v0_1.py")
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
FIXTURE_URI_DIRNAME_CONFUSION_CERT_DER=base64.b64decode("MIIDWDCCAkCgAwIBAgIUIa3Tuiscdr/1u1RTKZKyiZUJgbAwDQYJKoZIhvcNAQELBQAwGzEZMBcGA1UEAwwQTXljZWxpeCBTQU4gVGVzdDAeFw0yNjEwMDYwODUwMjlaFw0yNjEwMTMwODUwMjlaMBsxGTAXBgNVBAMMEE15Y2VsaXggU0FOIFRlc3QwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQClcDSAF8uXBXbkdC+KIGJRROCPlsPJUTPYP2OGIMSNc9i6OB0jQn69bMKDVBmTfVrgWmew3aspE/wBAXWHm2Ag9zGBRwHAQnf3UpKkhrBytsMucgwX9n2qduJKcxBZ1UgB9CisJ9VNoTRx6hE6WLw8zXfCsbmlec6SuvsXqucGnZJl750rtcXBLnsE2v3Xc/JeotdqqZjVzn2gnlRMBVm48kfXw9k+OHeEXetVMoM8s02T+m2WU1cVExGY0W8kE0nvgvCehgM5f5ZQKKFgtU8m9n121PnGqOrEM9aA2iyRoSq/b4IQRZFlqMYG3g41z3hj1o/jMq4NASyu3OLsvO2jAgMBAAGjgZMwgZAwYQYDVR0RBFowWIZWaHR0cHM6Ly9leGFtcGxlLnRlc3QvMi4yMy4xMzMuMi4xPVRFU1QtTUZSLzIuMjMuMTMzLjIuMj1URVNULU1PREVMLzIuMjMuMTMzLjIuMz1GVy0xLjAwDAYDVR0TAQH/BAIwADAdBgNVHQ4EFgQUggbtZOEXjIEvm1bXhuMhg3gpTKowDQYJKoZIhvcNAQELBQADggEBAGFZ7CLWPzWVrz99Z8fU7cKWDLMyN+WRvsUC5sdQI7jfa+1mR117NbrIIT6NXE4ZwSD+vRQe0RaQ3dmz8Q5fuk4YKYIVO/lQA+QFbzwqYCtnydlXB1aW2YdvbnWuF99Ga81NT+/dbRCrG4YFPzPlSoJte3UeG3ZQILn5mcTNOVN5ovBjtUVdj5G7M6GXuNxp3u73QIceDrypm/Rx92wEX12tfk8MlDQC299aNf9LIwfTRt+dsoNffX2Q+5uKVSnH3E0zpvBnoHcKCkIRDhX6F1OlXmE0u8K+9InON1CJjADH79v4eUEJVuxjCVTFO811P15LYCe6HvThd2yvddEcP2s=")
FIXTURE_BAD_CERT_DER=base64.b64decode("MIIDMjCCAhqgAwIBAgIBCDANBgkqhkiG9w0BAQsFADAfMR0wGwYDVQQDDBRNeWNlbGl4IFN5bnRoZXRpYyBFSzAeFw0yNjEwMDUyMjQ4MzhaFw0zNjEwMDIyMjQ5MzhaMB8xHTAbBgNVBAMMFE15Y2VsaXggU3ludGhldGljIEVLMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAmmzLEBRz/Ebof6fx/Zuh9Pv9q0x4otvdT/KcwXxbxT3WNd1OdWVThNpkZzxRGtGajqgQ7lPDS0+RFRV98cl8Nf0BOZe8RdpN/PdEMDORE0r5WyGl9SZfX59RRg8i9LN67LGxqx1XvDEjSWWUSLWYQnUkcrjMCVgxNKr/dUOTigxnj0Md+1/ukTHfCf3JHAs9AkXTNLnKHEkbZav65OHwP3jU6fchxSIS9uAiAJjMQli/J6tI6l2y5K65OjZnziaWuD8ekwOy0F0m24ckSwblhh7mcDYcojFEuLX6FWTqyC13xz/ZmIGi83rr6bfszcixyAErXq1h3YIXXWCFstJOHwIDAQABo3kwdzBXBgNVHREBAf8ETTBLpEkwRzEWMBQGBWeBBQIBDAtpZDo0RDU5NDM1ODEVMBMGBWeBBQICDApPdGhlck1vZGVsMRYwFAYFZ4EFAgMMC2lkOjAwMDEwMDAzMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgUgMA0GCSqGSIb3DQEBCwUAA4IBAQBfNIy/bxha+iwZkrfkyYXQxZlwz8UAhQ4cZmg6kmwaMHGVtd55rXocGKTJt1ODWY3Fbz4yZaLsVexliXCE7LQoJNzdjjCmAtFQt45xQ9aNJ5hRji8LQOq+jArdpoNiaVf5R5cUAwLBptW9ThoSHmguA44TKkx5naMIKC35SdeCjcpkwoxKd/yq+SB3EZIRS6110H0m5Oncj1PCEbjid68C2FP84aRfXKvrXcb/ELhC23ifcn4ShaAQlJhlyABFlDC47gFcwfWVUhAvKHXnnnAsJw5AotOLSDNQbofh5kixU+OP22VrQl09AZJRzdOLWc8dH57l5cdY+nvssU5hHbcZ")
MANUFACTURER_OID="2.23.133.2.1"
MODEL_OID="2.23.133.2.2"
VERSION_OID="2.23.133.2.3"

def canonical_hash(v:Any)->str:
 return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()

def valid_hash(v:Any)->bool:
 return isinstance(v,str) and len(v)==64 and all(c in "0123456789abcdef" for c in v)

def der_tlv(data:bytes,offset:int)->tuple[int,bytes,int]:
 if offset>=len(data):raise ValueError("DER truncated before tag")
 tag=data[offset];offset+=1
 if offset>=len(data):raise ValueError("DER truncated before length")
 first=data[offset];offset+=1
 if first==0x80:raise ValueError("indefinite DER length forbidden")
 if first<0x80:
  length=first
 else:
  octets=first&0x7F
  if octets==0 or octets>4 or offset+octets>len(data):raise ValueError("invalid DER length")
  if data[offset]==0:raise ValueError("non-minimal DER length")
  length=int.from_bytes(data[offset:offset+octets],"big");offset+=octets
  if length<0x80:raise ValueError("non-minimal DER long length")
 end=offset+length
 if end>len(data):raise ValueError("DER value truncated")
 return tag,data[offset:end],end

def exact_der_tlv(data:bytes)->tuple[int,bytes]:
 tag,value,end=der_tlv(data,0)
 if end!=len(data):raise ValueError("DER trailing bytes")
 return tag,value

def decode_oid(value:bytes)->str:
 if not value:raise ValueError("empty OID")
 first=value[0]
 first_arc=min(first//40,2)
 second_arc=first-first_arc*40
 arcs=[first_arc,second_arc]
 acc=0;started=False
 for byte in value[1:]:
  if not started and byte==0x80:raise ValueError("non-minimal OID")
  acc=(acc<<7)|(byte&0x7F)
  started=True
  if not (byte&0x80):
   arcs.append(acc);acc=0;started=False
 if started:raise ValueError("unterminated OID")
 return ".".join(str(v) for v in arcs)

def decode_tcg_utf8(tag:int,value:bytes,field:str)->str:
 if tag!=0x0C:
  raise ValueError(f"{field} is not UTF8String")
 try:
  text=value.decode("utf-8")
 except UnicodeDecodeError as exc:
  raise ValueError(f"{field} is not valid UTF-8") from exc
 if not text:
  raise ValueError(f"{field} is empty")
 return text

def parse_tcg_dirname(name_der:bytes)->dict[str,str]:
 seq_tag,seq=exact_der_tlv(name_der)
 if seq_tag!=0x30:raise ValueError("directoryName is not an RDNSequence")
 targets={MANUFACTURER_OID,MODEL_OID,VERSION_OID}
 out={};counts={oid:0 for oid in targets}
 off=0
 while off<len(seq):
  set_tag,set_value,off=der_tlv(seq,off)
  if set_tag!=0x31:raise ValueError("RDN is not a SET")
  inner=0
  while inner<len(set_value):
   atv_tag,atv,inner=der_tlv(set_value,inner)
   if atv_tag!=0x30:raise ValueError("AttributeTypeAndValue is not a SEQUENCE")
   ao=0
   oid_tag,oid_bytes,ao=der_tlv(atv,ao)
   if oid_tag!=0x06:raise ValueError("RDN attribute type is not an OID")
   oid=decode_oid(oid_bytes)
   value_tag,value_bytes,ao=der_tlv(atv,ao)
   if ao!=len(atv):raise ValueError("RDN AttributeTypeAndValue has trailing bytes")
   if oid in targets:
    counts[oid]+=1
    if counts[oid]>1:raise ValueError(f"duplicate TCG EK SAN attribute {oid}")
    out[oid]=decode_tcg_utf8(value_tag,value_bytes,oid)
 return out

def parse_tcg_san(cert_der:bytes)->list[dict[str,str]]:
 cert_tag,cert_value=exact_der_tlv(cert_der)
 if cert_tag!=0x30:raise ValueError("certificate is not a SEQUENCE")
 tbs_tag,tbs,_=der_tlv(cert_value,0)
 if tbs_tag!=0x30:raise ValueError("TBSCertificate is not a SEQUENCE")
 san_payload=None;tbs_off=0
 while tbs_off<len(tbs):
  tag,value,tbs_off=der_tlv(tbs,tbs_off)
  if tag!=0xA3:continue
  ext_tag,ext_seq,ext_end=der_tlv(value,0)
  if ext_tag!=0x30 or ext_end!=len(value):raise ValueError("extensions wrapper is malformed")
  ext_off=0
  while ext_off<len(ext_seq):
   e_tag,e_value,ext_off=der_tlv(ext_seq,ext_off)
   if e_tag!=0x30:raise ValueError("Extension is not a SEQUENCE")
   eo=0
   oid_tag,oid_value,eo=der_tlv(e_value,eo)
   if oid_tag!=0x06:raise ValueError("Extension OID is not an OBJECT IDENTIFIER")
   oid=decode_oid(oid_value)
   if eo<len(e_value) and e_value[eo]==0x01:
    critical_tag,critical_value,eo=der_tlv(e_value,eo)
    if critical_tag!=0x01 or len(critical_value)!=1:raise ValueError("invalid extension critical flag")
   value_tag,octets,eo=der_tlv(e_value,eo)
   if value_tag!=0x04 or eo!=len(e_value):raise ValueError("invalid extension value")
   if oid=="2.5.29.17":
    if san_payload is not None:raise ValueError("duplicate subjectAltName extension")
    san_payload=octets
 if san_payload is None:raise ValueError("subjectAltName extension missing")
 san_tag,san_seq=exact_der_tlv(san_payload)
 if san_tag!=0x30:raise ValueError("subjectAltName is not GeneralNames")
 dirnames=[];off=0
 while off<len(san_seq):
  tag,value,off=der_tlv(san_seq,off)
  if tag!=0xA4:continue
  dirnames.append(parse_tcg_dirname(value))
 targets={MANUFACTURER_OID,MODEL_OID,VERSION_OID}
 if not dirnames:raise ValueError("no directoryName GeneralName present")
 complete=[d for d in dirnames if targets.issubset(d)]
 if len(complete)!=1:raise ValueError("expected exactly one directoryName carrying all TCG EK identity attributes")
 for oid in targets:
  if sum(1 for d in dirnames if oid in d)!=1:raise ValueError(f"TCG EK identity attribute {oid} appears outside the selected directoryName")
 return dirnames

def cert_san(cert_der:bytes,work:Path)->dict[str,str]:
 dirnames=parse_tcg_san(cert_der)
 selected=next(d for d in dirnames if {MANUFACTURER_OID,MODEL_OID,VERSION_OID}.issubset(d))
 return {
  "manufacturer":selected[MANUFACTURER_OID],
  "model":selected[MODEL_OID],
  "version":selected[VERSION_OID],
 }

def run_properties_verifier(binding:dict[str,Any])->dict[str,Any]:
 verifier_input=binding.get("verifier_input")
 if not isinstance(verifier_input,dict):
  return result("DENY","properties-verifier-input-invalid")
 if not PROPERTIES_VERIFIER_SCRIPT.is_file():
  return result("DENY","properties-verifier-missing")
 with tempfile.TemporaryDirectory(prefix="mycelix-ek-properties-") as td:
  work=Path(td)
  ip=work/"properties-input.json"
  op=work/"properties-output.json"
  ip.write_text(json.dumps(verifier_input,indent=2,sort_keys=True)+"\n",encoding="utf-8")
  proc=subprocess.run(
   [sys.executable,str(PROPERTIES_VERIFIER_SCRIPT),"--verify",str(ip),"--output",str(op)],
   cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False
  )
  if proc.returncode not in (0,1,2):
   return result("DENY","properties-verifier-execution-error",{"stderr":proc.stderr})
  if not op.is_file():
   return result("DENY","properties-verifier-produced-no-output")
  try:
   output=json.loads(op.read_text(encoding="utf-8"))
  except json.JSONDecodeError as exc:
   return result("DENY","properties-verifier-output-invalid",{"error":str(exc)})
  if output.get("verifier_id")!=PROPERTIES_VERIFIER_ID:
   return result("DENY","properties-verifier-id-mismatch")
  if binding.get("verifier_input_sha256")!=hashlib.sha256(ip.read_bytes()).hexdigest():
   return result("DENY","properties-verifier-input-digest-mismatch")
  if binding.get("verifier_output_sha256")!=hashlib.sha256(op.read_bytes()).hexdigest():
   return result("DENY","properties-verifier-output-digest-mismatch")
  return output

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

def session_binding(m:dict[str,Any],properties:dict[str,Any])->str:
 return canonical_hash({
  "certificate_sha256":m["leaf_certificate_sha256"],
  "properties_fixed":properties,
  "properties_source_sha256":m["properties_binding"]["source_output_sha256"],
  "properties_verifier_source_sha256":m["properties_binding"]["verifier_source_sha256"],
  "capture_result_sha256":m["properties_binding"].get("capture_result_sha256"),
  "capture_transcript_sha256":m["properties_binding"].get("capture_transcript_sha256"),
  "registry_id":m["registry_id"],
  "registry_sha256":m["registry_sha256"],
  "spki_binding_state":m["spki_binding_state"],
 })

def verify(m:dict[str,Any])->dict[str,Any]:
 req={"profile_id","profile_version","verification_mode","claim_ceiling","leaf_certificate_der_base64","leaf_certificate_sha256","properties_binding","registry_id","registry_sha256","spki_binding_state","session_binding_sha256"}
 missing=sorted(req-set(m))
 if missing:return result("DENY","missing-required-fields",{"fields":missing})
 if m["profile_id"]!="mycelix.security.tpm.ek-cert-san-property-binding":return result("DENY","profile-id-mismatch")
 if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
 if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
 if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
 try: cert=base64.b64decode(m["leaf_certificate_der_base64"],validate=True)
 except Exception as exc:return result("DENY","certificate-base64-invalid",{"error":str(exc)})
 if not valid_hash(m["leaf_certificate_sha256"]) or hashlib.sha256(cert).hexdigest()!=m["leaf_certificate_sha256"]:return result("DENY","certificate-digest-mismatch")
 pb=m["properties_binding"]
 if not isinstance(pb,dict):return result("DENY","properties-binding-invalid")
 for field in ("state","verifier_id","source_output","source_output_sha256","verifier_source_sha256","verifier_input","verifier_input_sha256","verifier_output_sha256"):
  if field not in pb:return result("DENY","properties-binding-field-missing",{"field":field})
 if pb["verifier_id"]!=PROPERTIES_VERIFIER_ID:return result("DENY","properties-verifier-id-mismatch")
 if pb["state"] not in {"PASS","INDETERMINATE"}:return result("DENY","properties-binding-state-invalid")
 for field in ("source_output_sha256","verifier_source_sha256","verifier_input_sha256","verifier_output_sha256"):
  if not valid_hash(pb[field]):return result("DENY","properties-binding-digest-invalid",{"field":field})
 if pb["verifier_source_sha256"]!=hashlib.sha256(PROPERTIES_VERIFIER_SCRIPT.read_bytes()).hexdigest():return result("DENY","properties-verifier-source-mismatch")
 if not isinstance(pb["source_output"],str):return result("DENY","properties-source-output-not-text")
 if hashlib.sha256(pb["source_output"].encode("utf-8")).hexdigest()!=pb["source_output_sha256"]:return result("DENY","properties-source-output-digest-mismatch")
 properties_result=run_properties_verifier(pb)
 if properties_result.get("verifier_id")!=PROPERTIES_VERIFIER_ID:return properties_result
 if properties_result.get("state")=="DENY":return result("DENY","properties-verifier-denied")
 if properties_result.get("state")=="INDETERMINATE":return result("INDETERMINATE","properties-verifier-indeterminate")
 properties_details=properties_result.get("details")
 if not isinstance(properties_details,dict) or not isinstance(properties_details.get("parsed_properties"),dict):
  return result("DENY","properties-verifier-result-missing-parsed-properties")
 properties_fixed=properties_details["parsed_properties"]
 if pb["parsed_properties"]!=properties_fixed:return result("DENY","properties-binding-parsed-result-mismatch")
 mapping_digest=hashlib.sha256(json.dumps(MAPPING_CONTENT,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()
 if mapping_digest!=MAPPING_SHA256:return result("DENY","built-in-mapping-integrity-failure")
 state,expected=expected_from_properties(properties_fixed,m["registry_id"],m["registry_sha256"])
 if state=="DENY":return result("DENY","registry-not-authorized")
 if state=="INDETERMINATE":return result("INDETERMINATE","vendor-specific-property-mapping-unknown")
 with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-") as td:
  try:observed=cert_san(cert,Path(td))
  except (RuntimeError,ValueError) as exc:return result("DENY","certificate-san-parse-failed",{"error":str(exc)})
 if observed!=expected:return result("DENY","certificate-san-does-not-match-tpm-properties",{"observed":observed,"expected":expected})
 if m["spki_binding_state"]=="DENY":return result("DENY","spki-binding-denied")
 if m["spki_binding_state"]=="INDETERMINATE":return result("INDETERMINATE","spki-binding-indeterminate")
 if m["spki_binding_state"]!="PASS":return result("DENY","spki-binding-state-invalid")
 if m["session_binding_sha256"]!=session_binding(m,properties_fixed):return result("DENY","session-binding-mismatch")
 if m["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","live-property-mapping-not-authorized-by-reference-model")
 return result("PASS","ek-certificate-san-bound-to-tpm-properties",{"observed":observed,"expected":expected,"registry_id":MAPPING_ID,"registry_sha256":MAPPING_SHA256})

def fixture()->dict[str,Any]:
 cert=FIXTURE_CERT_DER;props={"manufacturer_hex":"4D594358","vendor_tpm_type_hex":"00000000","firmware_version_1_hex":"00000000","firmware_version_2_hex":"00010002"}
 source="\n".join([
  "TPM2_PT_MANUFACTURER:",
  "  raw: 0x4D594358",
  '  value: "MYCX"',
  "TPM2_PT_VENDOR_TPM_TYPE:",
  "  raw: 0x00000000",
  '  value: ""',
  "TPM2_PT_VENDOR_STRING_1:",
  "  raw: 0x53594E54",
  '  value: "SYNT"',
  "TPM2_PT_VENDOR_STRING_2:",
  "  raw: 0x48455449",
  '  value: "HETI"',
  "TPM2_PT_VENDOR_STRING_3:",
  "  raw: 0x00000000",
  '  value: ""',
  "TPM2_PT_VENDOR_STRING_4:",
  "  raw: 0x00000000",
  '  value: ""',
  "TPM2_PT_FIRMWARE_VERSION_1:",
  "  raw: 0x00000000",
  '  value: ""',
  "TPM2_PT_FIRMWARE_VERSION_2:",
  "  raw: 0x00010002",
  '  value: ""',
 ])
 properties_input={
  "profile_id":"mycelix.security.tpm.properties-fixed-capture",
  "profile_version":"0.1.0",
  "claim_ceiling":"ReferenceModelOnly",
  "verification_mode":"ReferenceModelOnly",
  "command":["tpm2_getcap","properties-fixed"],
  "source_output":source,
  "source_output_sha256":hashlib.sha256(source.encode("utf-8")).hexdigest(),
  "parsed_properties":props,
 }
 with tempfile.TemporaryDirectory(prefix="fixture-properties-") as td:
  ip=Path(td)/"input.json"
  op=Path(td)/"output.json"
  ip.write_text(json.dumps(properties_input,indent=2,sort_keys=True)+"\n",encoding="utf-8")
  subprocess.run([sys.executable,str(PROPERTIES_VERIFIER_SCRIPT),"--verify",str(ip),"--output",str(op)],cwd=Path(td),check=False,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
  input_sha=hashlib.sha256(ip.read_bytes()).hexdigest()
  output_sha=hashlib.sha256(op.read_bytes()).hexdigest()
 m={"profile_id":"mycelix.security.tpm.ek-cert-san-property-binding","profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","leaf_certificate_der_base64":base64.b64encode(cert).decode(),"leaf_certificate_sha256":hashlib.sha256(cert).hexdigest(),"properties_binding":{"state":"PASS","verifier_id":PROPERTIES_VERIFIER_ID,"source_output":source,"source_output_sha256":hashlib.sha256(source.encode("utf-8")).hexdigest(),"parsed_properties":props,"verifier_source_sha256":hashlib.sha256(PROPERTIES_VERIFIER_SCRIPT.read_bytes()).hexdigest(),"verifier_input_sha256":input_sha,"verifier_output_sha256":output_sha,"verifier_input":properties_input},"registry_id":MAPPING_ID,"registry_sha256":MAPPING_SHA256,"spki_binding_state":"PASS","session_binding_sha256":""}
 m["session_binding_sha256"]=session_binding(m,props);return m

def refresh_properties_binding(candidate:dict[str,Any])->None:
 pb=candidate["properties_binding"]
 vi=pb["verifier_input"]
 with tempfile.TemporaryDirectory(prefix="san-property-refresh-") as td:
  work=Path(td)
  ip=work/"properties-input.json"
  op=work/"properties-output.json"
  ip.write_text(json.dumps(vi,indent=2,sort_keys=True)+"\n",encoding="utf-8")
  proc=subprocess.run([sys.executable,str(PROPERTIES_VERIFIER_SCRIPT),"--verify",str(ip),"--output",str(op)],cwd=work,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
  if proc.returncode not in (0,1,2) or not op.is_file():
   raise RuntimeError("property verifier refresh failed")
  output=json.loads(op.read_text(encoding="utf-8"))
  pb["verifier_input_sha256"]=hashlib.sha256(ip.read_bytes()).hexdigest()
  pb["verifier_output_sha256"]=hashlib.sha256(op.read_bytes()).hexdigest()
  pb["parsed_properties"]=output.get("details",{}).get("parsed_properties",{})

def mutate_property_raw(candidate:dict[str,Any],field:str,new_raw:str)->None:
 pb=candidate["properties_binding"]
 vi=pb["verifier_input"]
 source=vi["source_output"]
 parsed=copy.deepcopy(vi["parsed_properties"])
 old=parsed[field]["raw_hex"]
 source=source.replace("0x"+old,"0x"+new_raw,1)
 parsed[field]["raw_hex"]=new_raw
 vi["source_output"]=source
 vi["source_output_sha256"]=hashlib.sha256(source.encode("utf-8")).hexdigest()
 vi["parsed_properties"]=parsed
 pb["source_output"]=source
 pb["source_output_sha256"]=vi["source_output_sha256"]
 refresh_properties_binding(candidate)

def self_test()->int:
 base=fixture()
 cases=[
  ("canonical-match","PASS",lambda x:x),
  ("properties-command-substitution","DENY",lambda x:x["properties_binding"]["verifier_input"].update({"command":["host-tool","properties-fixed"]})),
  ("properties-source-substitution","DENY",lambda x:x["properties_binding"].update({"source_output_sha256":"12"*32})),
  ("properties-result-substitution","DENY",lambda x:x["properties_binding"].update({"parsed_properties":{}})),
  ("properties-verifier-source-substitution","DENY",lambda x:x["properties_binding"].update({"verifier_source_sha256":"13"*32})),
  ("properties-input-digest-substitution","DENY",lambda x:x["properties_binding"].update({"verifier_input_sha256":"14"*32})),
  ("properties-output-digest-substitution","DENY",lambda x:x["properties_binding"].update({"verifier_output_sha256":"15"*32})),
  ("certificate-san-substitution","DENY",lambda x:x.update({"leaf_certificate_der_base64":base64.b64encode(FIXTURE_BAD_CERT_DER).decode(),"leaf_certificate_sha256":hashlib.sha256(FIXTURE_BAD_CERT_DER).hexdigest()})),
  ("certificate-uri-dirname-confusion","DENY",lambda x:x.update({"leaf_certificate_der_base64":base64.b64encode(FIXTURE_URI_DIRNAME_CONFUSION_CERT_DER).decode(),"leaf_certificate_sha256":hashlib.sha256(FIXTURE_URI_DIRNAME_CONFUSION_CERT_DER).hexdigest()})),
  ("manufacturer-property-substitution","INDETERMINATE",lambda x:mutate_property_raw(x,"TPM2_PT_MANUFACTURER","4D594359")),
  ("model-mapping-substitution","INDETERMINATE",lambda x:mutate_property_raw(x,"TPM2_PT_VENDOR_TPM_TYPE","00000001")),
  ("firmware-property-substitution","INDETERMINATE",lambda x:mutate_property_raw(x,"TPM2_PT_FIRMWARE_VERSION_2","00010003")),
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
 print("18 mutation/canonical cases plus key-order control: PASS")
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
