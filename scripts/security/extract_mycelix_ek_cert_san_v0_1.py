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

def parse_dirname_value(text:str)->dict[str,str]:
    out:dict[str,str]={}
    for component in text.split("/"):
        component=component.strip()
        if not component or "=" not in component:
            continue
        oid,value=component.split("=",1)
        if oid in {OID_MANUFACTURER,OID_MODEL,OID_VERSION}:
            out[oid]=value
    return out

def extract(cert:bytes)->dict[str,Any]:
    if shutil.which("openssl") is None:
        return {"verifier_id":VERIFIER_ID,"state":"INDETERMINATE","reason":"openssl-unavailable"}
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-") as td:
        work=Path(td)
        cert_path=work/"cert.der"
        cert_path.write_bytes(cert)
        p=run(["openssl","x509","-inform","DER","-in",str(cert_path),"-noout","-ext","subjectAltName","-nameopt","RFC2253"],work)
        if p.returncode!=0:
            return {"verifier_id":VERIFIER_ID,"state":"DENY","reason":"certificate-parse-failed","details":{"stderr":p.stderr}}
        lines=p.stdout.splitlines()
        san_header=next((i for i,l in enumerate(lines) if l.strip().lower().startswith("x509v3 subject alternative name")),None)
        if san_header is None:
            return {"verifier_id":VERIFIER_ID,"state":"DENY","reason":"subject-alt-name-missing"}
        raw_dirnames=[]
        for line in lines[san_header+1:]:
            stripped=line.strip()
            if stripped.startswith("X509v3 "):
                break
            for match in re.finditer(r"DirName:([^,\n]+(?:/[^,\n]+)*)",stripped):
                raw_dirnames.append(match.group(1))
        attrs=[parse_dirname_value(d) for d in raw_dirnames]
        values={oid:next((a[oid] for a in attrs if oid in a),"") for oid in (OID_MANUFACTURER,OID_MODEL,OID_VERSION)}
        missing=[oid for oid,v in values.items() if not v]
        state="PASS" if not missing else "DENY"
        return {
            "verifier_id":VERIFIER_ID,
            "state":state,
            "reason":"ek-san-directory-identity-extracted" if state=="PASS" else "ek-san-required-identity-missing",
            "details":{
                "certificate_der_sha256":hashlib.sha256(cert).hexdigest(),
                "manufacturer":values[OID_MANUFACTURER],
                "model":values[OID_MODEL],
                "version":values[OID_VERSION],
                "dir_name_count":len(raw_dirnames),
                "raw_dirnames":raw_dirnames,
                "required_oids":[OID_MANUFACTURER,OID_MODEL,OID_VERSION],
            }
        }

def self_test()->int:
    if shutil.which("openssl") is None:
        print("EK SAN extractor: INDETERMINATE (openssl unavailable)")
        return 2
    cfg="""[req]
distinguished_name=req_dn
prompt=no
x509_extensions=ext
[req_dn]
CN=Mycelix EK SAN Fixture
[ext]
subjectAltName=@ek
basicConstraints=critical,CA:false
[ek]
2.23.133.2.1=TEST-MFR
2.23.133.2.2=TEST-MODEL
2.23.133.2.3=FW-1.0
"""
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-san-test-") as td:
        work=Path(td)
        cfgp=work/"openssl.cnf"; key=work/"key.pem"; cert=work/"cert.pem"; der=work/"cert.der"
        cfgp.write_text(cfg,encoding="utf-8")
        p=run(["openssl","req","-new","-x509","-newkey","rsa:2048","-nodes","-days","7","-keyout",str(key),"-out",str(cert),"-config",str(cfgp)],work)
        if p.returncode!=0:
            print("EK SAN fixture generation: FAIL")
            return 1
        p=run(["openssl","x509","-in",str(cert),"-outform","DER","-out",str(der)],work)
        if p.returncode!=0:
            print("EK SAN fixture DER conversion: FAIL")
            return 1
        result=extract(der.read_bytes())
        d=result.get("details",{})
        if result["state"]!="PASS" or d.get("manufacturer")!="TEST-MFR" or d.get("model")!="TEST-MODEL" or d.get("version")!="FW-1.0":
            print(f"EK SAN extraction: FAIL {result}")
            return 1
    print("EK SAN directoryName extractor semantic corpus: PASS")
    return 0

def main()->int:
    p=argparse.ArgumentParser()
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument("--self-test",action="store_true")
    g.add_argument("--extract",metavar="CERT_DER")
    p.add_argument("--output")
    a=p.parse_args()
    if a.self_test:return self_test()
    cert=Path(a.extract).read_bytes()
    out=extract(cert)
    out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered=json.dumps(out,indent=2,sort_keys=True)+"\n"
    if a.output:Path(a.output).write_text(rendered,encoding="utf-8")
    else:print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[out["state"]]
if __name__=="__main__": raise SystemExit(main())
