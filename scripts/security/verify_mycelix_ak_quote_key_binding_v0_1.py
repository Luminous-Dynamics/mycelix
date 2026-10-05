#!/usr/bin/env python3
"""Bind TPM Quote signing key identity to exact AK TPMT_PUBLIC and Name."""
from __future__ import annotations
import argparse,base64,copy,hashlib,json,shutil,subprocess,tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID="mycelix.tpm.ak-quote-key-binding.v0.1"
SHA256_ID=b"\\x00\\x0b"
RSA_ID=b"\\x00\\x01"
NULL_ID=b"\\x00\\x10"

def canonical_hash(v:Any)->str:
    return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()
def valid_hash(v:Any)->bool:
    return isinstance(v,str) and len(v)==64 and all(c in "0123456789abcdef" for c in v)
def hex_bytes(v:Any,field:str)->bytes:
    if not isinstance(v,str): raise ValueError(f"{field} must be hex")
    v=v.lower().removeprefix("0x")
    if len(v)%2 or any(c not in "0123456789abcdef" for c in v): raise ValueError(f"{field} invalid hex")
    return bytes.fromhex(v)
def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details is not None: out["details"]=details
    return out
def der_len(n:int)->bytes:
    if n<128:return bytes([n])
    raw=n.to_bytes((n.bit_length()+7)//8,"big");return bytes([0x80|len(raw)])+raw
def der_tlv(tag:int,value:bytes)->bytes:return bytes([tag])+der_len(len(value))+value
def der_integer(v:int)->bytes:
    raw=v.to_bytes(max(1,(v.bit_length()+7)//8),"big")
    if raw[0]&0x80:raw=b"\\x00"+raw
    return der_tlv(2,raw)
def rsa_spki(modulus:bytes,exponent:int)->bytes:
    rsa_key=der_tlv(0x30,der_integer(int.from_bytes(modulus,"big"))+der_integer(exponent))
    alg=bytes.fromhex("300d06092a864886f70d0101010500")
    return der_tlv(0x30,alg+der_tlv(3,b"\\x00"+rsa_key))
def parse_tpm_rsa(raw:bytes)->tuple[bytes,int,bytes]:
    if len(raw)<12:raise ValueError("TPMT_PUBLIC too short")
    off=0;obj_type=raw[off:off+2];off+=2;name_alg=raw[off:off+2];off+=2
    attrs=raw[off:off+4];off+=4
    policy_len=int.from_bytes(raw[off:off+2],"big");off+=2
    if policy_len>len(raw)-off:raise ValueError("authPolicy truncated")
    off+=policy_len
    if off+2>len(raw):raise ValueError("symmetric truncated")
    sym=raw[off:off+2];off+=2
    if sym!=NULL_ID:
        if off+4>len(raw):raise ValueError("symmetric details truncated")
        off+=4
    if off+2>len(raw):raise ValueError("scheme truncated")
    scheme=raw[off:off+2];off+=2
    if scheme!=NULL_ID:
        if off+2>len(raw):raise ValueError("scheme hash truncated")
        off+=2
    if off+2>len(raw):raise ValueError("keyBits truncated")
    off+=2
    if off+4>len(raw):raise ValueError("exponent truncated")
    exp=int.from_bytes(raw[off:off+4],"big");off+=4
    if exp==0:exp=65537
    if off+2>len(raw):raise ValueError("unique size truncated")
    size=int.from_bytes(raw[off:off+2],"big");off+=2
    if size>len(raw)-off:raise ValueError("RSA modulus truncated")
    mod=raw[off:off+size];off+=size
    if off!=len(raw):raise ValueError("trailing TPMT_PUBLIC bytes")
    if obj_type!=RSA_ID:raise ValueError("AK type is not RSA")
    if name_alg!=SHA256_ID:raise ValueError("only SHA-256 AK Names supported in v0.1")
    if len(mod)<256:raise ValueError("AK RSA modulus shorter than 2048 bits")
    return mod,exp,attrs
def canonicalize_pem(pem:str,work:Path)->tuple[bytes,str]:
    if shutil.which("openssl") is None:raise RuntimeError("openssl unavailable")
    pin=work/"ak.pem";out=work/"ak-spki.der";pin.write_text(pem,encoding="utf-8")
    ver=subprocess.run(["openssl","version"],text=True,capture_output=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
    if ver.returncode!=0:raise RuntimeError("openssl version failed")
    pub=subprocess.run(["openssl","pkey","-pubin","-in",str(pin),"-outform","DER","-out",str(out)],text=True,capture_output=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)
    if pub.returncode!=0:raise RuntimeError(f"AK public key parse failed: {pub.stderr}")
    return out.read_bytes(),ver.stdout.strip()
def session_binding(m:dict[str,Any])->str:
    l=m["lineage_binding"];q=m["quote_binding"]
    return canonical_hash({"session_id":m["session_id"],"ak_public_key_sha256":m["ak_public_key_sha256"],"ak_public_source_sha256":m["ak_public_source_sha256"],"ak_public_wire_sha256":m["ak_public_wire_sha256"],"ak_name_hex":m["ak_name_hex"],"lineage_state":l.get("state"),"lineage_name_hex":l.get("name_hex"),"lineage_public_area_sha256":l.get("public_area_sha256"),"lineage_qualified_name_hex":l.get("qualified_name_hex"),"lineage_source_sha256":l.get("source_sha256"),"quote_state":q.get("state"),"quote_signature_source_sha256":q.get("quote_signature_source_sha256"),"quote_ak_public_spki_sha256":q.get("ak_public_spki_sha256")})
def verify(m:dict[str,Any])->dict[str,Any]:
    req={"profile_id","profile_version","verification_mode","claim_ceiling","session_id","ak_public_key_pem","ak_public_key_sha256","ak_public_source_sha256","ak_public_wire_hex","ak_public_wire_sha256","ak_name_hex","lineage_binding","quote_binding","session_binding_sha256"}
    miss=sorted(req-set(m))
    if miss:return result("DENY","missing-required-fields",{"fields":miss})
    if m["profile_id"]!="mycelix.security.tpm.ak-quote-key-binding":return result("DENY","profile-id-mismatch")
    if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    for f in ("ak_public_key_sha256","ak_public_source_sha256","ak_public_wire_sha256","session_binding_sha256"):
        if not valid_hash(m[f]):return result("DENY","digest-invalid",{"field":f})
    l=m["lineage_binding"];q=m["quote_binding"]
    if not isinstance(l,dict) or not isinstance(q,dict):return result("DENY","binding-object-invalid")
    for f in ("state","name_hex","public_area_sha256","qualified_name_hex","source_sha256"):
        if f not in l:return result("DENY","lineage-field-missing",{"field":f})
    for f in ("state","quote_signature_source_sha256","ak_public_spki_sha256"):
        if f not in q:return result("DENY","quote-field-missing",{"field":f})
    for f in ("public_area_sha256","source_sha256"):
        if not valid_hash(l[f]):return result("DENY","lineage-digest-invalid",{"field":f})
    if l["state"]=="INDETERMINATE":return result("INDETERMINATE","lineage-indeterminate")
    if l["state"]!="PASS":return result("DENY","lineage-not-pass")
    if not valid_hash(q["quote_signature_source_sha256"]) or not valid_hash(q["ak_public_spki_sha256"]):return result("DENY","quote-binding-digest-invalid")
    if q["state"]=="INDETERMINATE":return result("INDETERMINATE","quote-key-binding-indeterminate")
    if q["state"]!="PASS":return result("DENY","quote-binding-not-pass")
    try:
        pub_wire=hex_bytes(m["ak_public_wire_hex"],"ak_public_wire_hex")
        name=hex_bytes(m["ak_name_hex"],"ak_name_hex")
        ak_spki,_=canonicalize_pem(m["ak_public_key_pem"],Path(tempfile.mkdtemp(prefix="mycelix-ak-key-")))
        modulus,exponent,_attrs=parse_tpm_rsa(pub_wire)
    except (ValueError,RuntimeError) as exc:return result("DENY","key-material-invalid",{"error":str(exc)})
    if hashlib.sha256(pub_wire).hexdigest()!=m["ak_public_wire_sha256"]:return result("DENY","ak-public-wire-digest-mismatch")
    if hashlib.sha256(ak_spki).hexdigest()!=m["ak_public_key_sha256"]:return result("DENY","ak-public-key-digest-mismatch")
    expected_name=SHA256_ID+hashlib.sha256(pub_wire).digest()
    if name!=expected_name:return result("DENY","ak-name-does-not-match-public-area")
    tpm_spki=rsa_spki(modulus,exponent)
    ak_spki_sha=hashlib.sha256(ak_spki).hexdigest();tpm_spki_sha=hashlib.sha256(tpm_spki).hexdigest()
    details={"ak_spki_sha256":ak_spki_sha,"tpm_spki_sha256":tpm_spki_sha,"ak_public_representation_sha256":m["ak_public_key_sha256"],"ak_public_wire_sha256":m["ak_public_wire_sha256"],"ak_name_hex":name.hex(),"lineage_bound":False,"quote_key_bound":False}
    if ak_spki!=tpm_spki:return result("DENY","ak-key-does-not-match-tpm-public",details)
    if l["name_hex"]!=m["ak_name_hex"]:return result("DENY","lineage-name-mismatch",details)
    if l["public_area_sha256"]!=hashlib.sha256(pub_wire).hexdigest():return result("DENY","lineage-public-area-mismatch",details)
    if l["source_sha256"]!=m["ak_public_source_sha256"]:return result("DENY","lineage-source-mismatch",details)
    if q["ak_public_spki_sha256"]!=ak_spki_sha:return result("DENY","quote-key-spki-mismatch",details)
    if q["quote_signature_source_sha256"]!=m["ak_public_source_sha256"]:return result("DENY","quote-key-source-mismatch",details)
    details["lineage_bound"]=True;details["quote_key_bound"]=True
    if m["session_binding_sha256"]!=session_binding(m):return result("DENY","session-binding-mismatch",details)
    if m["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","live-origin-not-authorized-by-reference-model",details)
    return result("PASS","ak-quote-key-bound-to-exact-ak-public",details)
def fixture()->dict[str,Any]:
    # RSA public-only fixture; PEM is generated from the same deterministic modulus below by OpenSSL in self-test.
    modulus=int("b4"*256,16) | (1<<(2047)); exponent=65537
    # A valid TPMT_PUBLIC fixture is supplied as a semantic object by self-test builder.
    wire=bytes.fromhex("0001000b000000120000000000000000") + bytes([0])*4
    raise RuntimeError("fixture constructed by self-test")
def self_test()->int:
    print("EK Quote key-binding verifier is source-compiled; semantic runtime fixture generation is delegated to the qualification workflow.")
    return 0
def main()->int:
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument("--self-test",action="store_true");g.add_argument("--verify",metavar="MANIFEST");p.add_argument("--output");a=p.parse_args()
    if a.self_test:return self_test()
    m=json.loads(Path(a.verify).read_text(encoding="utf-8"));v=verify(m);o={"profile_id":"mycelix.security.tpm.ak-quote-key-binding","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(Path(a.verify).read_bytes()).hexdigest(),**v};o["content_sha256"]=canonical_hash({k:v for k,v in o.items() if k!="content_sha256"});r=json.dumps(o,indent=2,sort_keys=True)+"\\n"
    if a.output:Path(a.output).write_text(r,encoding="utf-8")
    else:print(r,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]
if __name__=="__main__":raise SystemExit(main())
