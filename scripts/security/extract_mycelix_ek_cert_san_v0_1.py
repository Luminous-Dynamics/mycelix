#!/usr/bin/env python3
"""Extract TCG EK SAN directoryName identity attributes from exact X.509 DER."""
from __future__ import annotations
import argparse, hashlib, json, re, shutil, subprocess, tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID="mycelix.tpm.ek-cert-san-extractor.v0.1"
OID_MANUFACTURER="2.23.133.2.1"
OID_MODEL="2.23.133.2.2"
OID_VERSION="2.23.133.2.3"

def canonical_hash(v:Any)->str:
    return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()

def run(cmd:list[str],cwd:Path)->subprocess.CompletedProcess[str]:
    return subprocess.run(cmd,cwd=cwd,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=False)

def der_tlv(data:bytes,offset:int)->tuple[int,bytes,int]:
    if offset>=len(data): raise ValueError("DER truncated before tag")
    tag=data[offset]; offset+=1
    if offset>=len(data): raise ValueError("DER truncated before length")
    first=data[offset]; offset+=1
    if first==0x80: raise ValueError("indefinite DER length forbidden")
    if first<0x80:
        length=first
    else:
        octets=first&0x7F
        if octets==0 or octets>4 or offset+octets>len(data): raise ValueError("invalid DER length")
        if data[offset]==0: raise ValueError("non-minimal DER length")
        length=int.from_bytes(data[offset:offset+octets],"big"); offset+=octets
        if length<0x80: raise ValueError("non-minimal DER length")
    end=offset+length
    if end>len(data): raise ValueError("DER value truncated")
    return tag,data[offset:end],end

def exact_der_tlv(data:bytes)->tuple[int,bytes]:
    tag,value,end=der_tlv(data,0)
    if end!=len(data): raise ValueError("DER trailing bytes")
    return tag,value

def decode_oid(value:bytes)->str:
    if not value: raise ValueError("empty OID")
    first=value[0]
    first_arc=min(first//40,2)
    second_arc=first-first_arc*40
    arcs=[first_arc,second_arc]
    acc=0; started=False
    for byte in value[1:]:
        acc=(acc<<7)|(byte&0x7F)
        started=True
        if not (byte&0x80):
            arcs.append(acc); acc=0; started=False
    if started: raise ValueError("unterminated OID")
    return ".".join(str(v) for v in arcs)

def parse_tcg_utf8(tag:int,value:bytes,field:str)->str:
    if tag!=0x0C: raise ValueError(f"{field} is not UTF8String")
    try:
        text=value.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{field} is not valid UTF-8") from exc
    if not text: raise ValueError(f"{field} is empty")
    return text

def parse_tcg_dirname(name_der:bytes)->dict[str,str]:
    seq_tag,seq=exact_der_tlv(name_der)
    if seq_tag!=0x30: raise ValueError("directoryName is not an RDNSequence")
    targets={OID_MANUFACTURER,OID_MODEL,OID_VERSION}
    out={}; counts={oid:0 for oid in targets}; off=0
    while off<len(seq):
        set_tag,set_value,off=der_tlv(seq,off)
        if set_tag!=0x31: raise ValueError("RDN is not a SET")
        inner=0
        while inner<len(set_value):
            atv_tag,atv,inner=der_tlv(set_value,inner)
            if atv_tag!=0x30: raise ValueError("AttributeTypeAndValue is not a SEQUENCE")
            ao=0
            oid_tag,oid_bytes,ao=der_tlv(atv,ao)
            if oid_tag!=0x06: raise ValueError("RDN attribute type is not an OID")
            oid=decode_oid(oid_bytes)
            value_tag,value_bytes,ao=der_tlv(atv,ao)
            if ao!=len(atv): raise ValueError("RDN AttributeTypeAndValue has trailing bytes")
            if oid in targets:
                counts[oid]+=1
                if counts[oid]>1: raise ValueError(f"duplicate TCG EK SAN attribute {oid}")
                out[oid]=parse_tcg_utf8(value_tag,value_bytes,oid)
    return out

def parse_tcg_san(cert_der:bytes)->list[dict[str,str]]:
    cert_tag,cert_value=exact_der_tlv(cert_der)
    if cert_tag!=0x30: raise ValueError("certificate is not a SEQUENCE")
    tbs_tag,tbs,_=der_tlv(cert_value,0)
    if tbs_tag!=0x30: raise ValueError("TBSCertificate is not a SEQUENCE")
    san_payload=None; tbs_off=0
    while tbs_off<len(tbs):
        tag,value,tbs_off=der_tlv(tbs,tbs_off)
        if tag!=0xA3: continue
        ext_tag,ext_seq,ext_end=der_tlv(value,0)
        if ext_tag!=0x30 or ext_end!=len(value): raise ValueError("extensions wrapper is malformed")
        ext_off=0
        while ext_off<len(ext_seq):
            e_tag,e_value,ext_off=der_tlv(ext_seq,ext_off)
            if e_tag!=0x30: raise ValueError("Extension is not a SEQUENCE")
            eo=0
            oid_tag,oid_value,eo=der_tlv(e_value,eo)
            if oid_tag!=0x06: raise ValueError("Extension OID is not OBJECT IDENTIFIER")
            oid=decode_oid(oid_value)
            if eo<len(e_value) and e_value[eo]==0x01:
                critical_tag,critical_value,eo=der_tlv(e_value,eo)
                if critical_tag!=0x01 or len(critical_value)!=1: raise ValueError("invalid extension critical flag")
            value_tag,octets,eo=der_tlv(e_value,eo)
            if value_tag!=0x04 or eo!=len(e_value): raise ValueError("invalid extension value")
            if oid=="2.5.29.17":
                if san_payload is not None: raise ValueError("duplicate subjectAltName extension")
                san_payload=octets
    if san_payload is None: raise ValueError("subjectAltName extension missing")
    san_tag,san_seq=exact_der_tlv(san_payload)
    if san_tag!=0x30: raise ValueError("subjectAltName is not GeneralNames")
    dirnames=[]; off=0
    while off<len(san_seq):
        tag,value,off=der_tlv(san_seq,off)
        if tag==0xA4:
            dirnames.append(parse_tcg_dirname(value))
    if not dirnames: raise ValueError("no directoryName GeneralName present")
    complete=[d for d in dirnames if {OID_MANUFACTURER,OID_MODEL,OID_VERSION}.issubset(d)]
    if len(complete)!=1: raise ValueError("expected exactly one directoryName carrying all TCG EK identity attributes")
    for oid in (OID_MANUFACTURER,OID_MODEL,OID_VERSION):
        if sum(1 for d in dirnames if oid in d)!=1: raise ValueError(f"TCG EK identity attribute {oid} appears outside the selected directoryName")
    return dirnames

def extract(cert:bytes)->dict[str,Any]:
    try:
        dirnames=parse_tcg_san(cert)
    except ValueError as exc:
        return {"verifier_id":VERIFIER_ID,"state":"DENY","reason":"ek-san-der-parse-failed","details":{"certificate_der_sha256":hashlib.sha256(cert).hexdigest(),"error":str(exc)}}
    selected=next(d for d in dirnames if {OID_MANUFACTURER,OID_MODEL,OID_VERSION}.issubset(d))
    return {
        "verifier_id":VERIFIER_ID,
        "state":"PASS",
        "reason":"ek-san-directory-identity-extracted",
        "details":{
            "certificate_der_sha256":hashlib.sha256(cert).hexdigest(),
            "manufacturer":selected[OID_MANUFACTURER],
            "model":selected[OID_MODEL],
            "version":selected[OID_VERSION],
            "dir_name_count":len(dirnames),
            "required_oids":[OID_MANUFACTURER,OID_MODEL,OID_VERSION],
        }
    }

def self_test()->int:
    if shutil.which("openssl") is None:
        print("EK SAN extractor: INDETERMINATE (openssl unavailable)")
        return 2
    def make_cert(uri:bool)->bytes|None:
        cfg = """[req]
distinguished_name=req_dn
prompt=no
x509_extensions=ext
[req_dn]
CN=Mycelix EK SAN Fixture
[ext]
subjectAltName=@ek
basicConstraints=critical,CA:false
[ek]
"""
        if uri:
            cfg += "URI=https://example.test/2.23.133.2.1=TEST-MFR/2.23.133.2.2=TEST-MODEL/2.23.133.2.3=FW-1.0\n"
        else:
            cfg += "dirName=dn\n[dn]\ntcg-at-tpmManufacturer=TEST-MFR\ntcg-at-tpmModel=TEST-MODEL\ntcg-at-tpmVersion=FW-1.0\n"
        with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-test-") as td:
            work=Path(td); cfgp=work/"openssl.cnf"; key=work/"key.pem"; cert=work/"cert.pem"; der=work/"cert.der"
            cfgp.write_text(cfg,encoding="utf-8")
            p=run(["openssl","req","-new","-x509","-newkey","rsa:2048","-nodes","-days","7","-keyout",str(key),"-out",str(cert),"-config",str(cfgp)],work)
            if p.returncode!=0: return None
            p=run(["openssl","x509","-in",str(cert),"-outform","DER","-out",str(der)],work)
            if p.returncode!=0: return None
            return der.read_bytes()
    real=make_cert(False)
    if real is None:
        print("EK SAN fixture generation: FAIL")
        return 1
    result=extract(real); d=result.get("details",{})
    if result["state"]!="PASS" or d.get("manufacturer")!="TEST-MFR" or d.get("model")!="TEST-MODEL" or d.get("version")!="FW-1.0":
        print(f"EK SAN extraction: FAIL {result}")
        return 1
    confusion=make_cert(True)
    if confusion is None or extract(confusion)["state"]!="DENY":
        print("EK SAN URI/DirName type-confusion regression: FAIL")
        return 1
    print("EK SAN directoryName DER extractor semantic corpus: PASS")
    print("canonical directoryName + URI/DirName confusion control: PASS")
    return 0


def main()->int:
    p=argparse.ArgumentParser()
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument("--self-test",action="store_true")
    g.add_argument("--extract",metavar="CERT_DER")
    p.add_argument("--output")
    a=p.parse_args()
    if a.self_test:return self_test()
    cert_path=Path(a.extract).resolve()
    cert=cert_path.read_bytes()
    out=extract(cert)
    out["input_sha256"]=hashlib.sha256(cert).hexdigest()
    out["source_sha256"]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered=json.dumps(out,indent=2,sort_keys=True)+"\n"
    if a.output:Path(a.output).write_text(rendered,encoding="utf-8")
    else:print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[out["state"]]
if __name__=="__main__": raise SystemExit(main())
