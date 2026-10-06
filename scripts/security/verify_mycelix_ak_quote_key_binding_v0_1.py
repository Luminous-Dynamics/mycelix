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
    if m["verifier_source_sha256"]!=hashlib.sha256(Path(__file__).read_bytes()).hexdigest():return result("DENY","verifier-source-digest-mismatch")
    l=m["lineage_binding"];q=m["quote_binding"]
    return canonical_hash({"verifier_source_sha256":m["verifier_source_sha256"],"session_id":m["session_id"],"ak_public_key_sha256":m["ak_public_key_sha256"],"ak_public_source_sha256":m["ak_public_source_sha256"],"ak_public_wire_sha256":m["ak_public_wire_sha256"],"ak_name_hex":m["ak_name_hex"],"lineage_state":l.get("state"),"lineage_name_hex":l.get("name_hex"),"lineage_public_area_sha256":l.get("public_area_sha256"),"lineage_qualified_name_hex":l.get("qualified_name_hex"),"lineage_source_sha256":l.get("source_sha256"),"quote_state":q.get("state"),"quote_signature_source_sha256":q.get("quote_signature_source_sha256"),"quote_ak_public_spki_sha256":q.get("ak_public_spki_sha256")})
def verify(m:dict[str,Any])->dict[str,Any]:
    req={"profile_id","profile_version","verification_mode","claim_ceiling","verifier_source_sha256","session_id","ak_public_key_pem","ak_public_key_sha256","ak_public_source_sha256","ak_public_wire_hex","ak_public_wire_sha256","ak_name_hex","lineage_binding","quote_binding","session_binding_sha256"}
    miss=sorted(req-set(m))
    if miss:return result("DENY","missing-required-fields",{"fields":miss})
    if m["profile_id"]!="mycelix.security.tpm.ak-quote-key-binding":return result("DENY","profile-id-mismatch")
    if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    for f in ("verifier_source_sha256","ak_public_key_sha256","ak_public_source_sha256","ak_public_wire_sha256","session_binding_sha256"):
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
        public_key_bytes=m["ak_public_key_pem"].encode("utf-8")
        with tempfile.TemporaryDirectory(prefix="mycelix-ak-key-") as td:
            ak_spki,_=canonicalize_pem(m["ak_public_key_pem"],Path(td))
            modulus,exponent,_attrs=parse_tpm_rsa(pub_wire)
    except (ValueError,RuntimeError) as exc:return result("DENY","key-material-invalid",{"error":str(exc)})
    if hashlib.sha256(pub_wire).hexdigest()!=m["ak_public_wire_sha256"]:return result("DENY","ak-public-wire-digest-mismatch")
    if hashlib.sha256(ak_spki).hexdigest()!=m["ak_public_key_sha256"]:return result("DENY","ak-public-key-spki-digest-mismatch")
    if hashlib.sha256(public_key_bytes).hexdigest()!=m["ak_public_source_sha256"]:return result("DENY","ak-public-source-digest-mismatch")
    expected_name=SHA256_ID+hashlib.sha256(pub_wire).digest()
    if name!=expected_name:return result("DENY","ak-name-does-not-match-public-area")
    tpm_spki=rsa_spki(modulus,exponent)
    ak_spki_sha=hashlib.sha256(ak_spki).hexdigest();tpm_spki_sha=hashlib.sha256(tpm_spki).hexdigest()
    details={"ak_spki_sha256":ak_spki_sha,"tpm_spki_sha256":tpm_spki_sha,"ak_public_representation_sha256":m["ak_public_key_sha256"],"ak_public_wire_sha256":m["ak_public_wire_sha256"],"ak_name_hex":name.hex(),"lineage_bound":False,"quote_key_bound":False}
    if ak_spki!=tpm_spki:return result("DENY","ak-key-does-not-match-tpm-public",details)
    if l["name_hex"]!=m["ak_name_hex"]:return result("DENY","lineage-name-mismatch",details)
    if l["public_area_sha256"]!=hashlib.sha256(pub_wire).hexdigest():return result("DENY","lineage-public-area-mismatch",details)
    if not valid_hash(l["source_sha256"]):return result("DENY","lineage-source-digest-invalid",details)
    if q["ak_public_spki_sha256"]!=ak_spki_sha:return result("DENY","quote-key-spki-mismatch",details)
    if not valid_hash(q["quote_signature_source_sha256"]):return result("DENY","quote-source-digest-invalid",details)
    details["lineage_bound"]=True;details["quote_key_bound"]=True
    if m["session_binding_sha256"]!=session_binding(m):return result("DENY","session-binding-mismatch",details)
    if m["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","live-origin-not-authorized-by-reference-model",details)
    return result("PASS","ak-quote-key-bound-to-exact-ak-public",details)
def build_wire(modulus:bytes)->bytes:
    # RSA signing AK: type=RSA, nameAlg=SHA256, fixedTPM|fixedParent|userWithAuth|sign,
    # empty authPolicy, NULL symmetric, RSASSA/SHA256 scheme, 2048 bits, exponent=0 (default 65537).
    return (
        bytes.fromhex("0001")
        + bytes.fromhex("000b")
        + bytes.fromhex("00040072")
        + bytes.fromhex("0000")
        + bytes.fromhex("0010")
        + bytes.fromhex("0014")
        + bytes.fromhex("000b")
        + bytes.fromhex("0800")
        + bytes.fromhex("00000000")
        + len(modulus).to_bytes(2,"big")
        + modulus
    )

def generate_fixture()->dict[str,Any]:
    if shutil.which("openssl") is None:
        raise RuntimeError("openssl unavailable")
    with tempfile.TemporaryDirectory(prefix="mycelix-ak-quote-fixture-") as td:
        work=Path(td);key=work/"ak.key";pem=work/"ak.pem";mod=work/"modulus.bin"
        p=subprocess.run(["openssl","genpkey","-algorithm","RSA","-pkeyopt","rsa_keygen_bits:2048","-out",str(key)],text=True,capture_output=True,check=False)
        if p.returncode!=0:raise RuntimeError("RSA fixture generation failed")
        p=subprocess.run(["openssl","pkey","-in",str(key),"-pubout","-out",str(pem)],text=True,capture_output=True,check=False)
        if p.returncode!=0:raise RuntimeError("RSA public export failed")
        p=subprocess.run(["openssl","rsa","-in",str(key),"-noout","-modulus"],text=True,capture_output=True,check=False)
        if p.returncode!=0:raise RuntimeError("RSA modulus export failed")
        modulus=bytes.fromhex(p.stdout.strip().split("=",1)[1])
        wire=build_wire(modulus)
        name=(SHA256_ID+hashlib.sha256(wire).digest()).hex()
        ak_spki=subprocess.run(["openssl","pkey","-pubin","-in",str(pem),"-outform","DER"],text=False,capture_output=True,check=False)
        if ak_spki.returncode!=0:raise RuntimeError("RSA SPKI canonicalization failed")
        spki_sha=hashlib.sha256(ak_spki.stdout).hexdigest()
        public_rep_sha=hashlib.sha256(pem.read_bytes()).hexdigest()
        m={
            "profile_id":"mycelix.security.tpm.ak-quote-key-binding",
            "profile_version":"0.1.0",
            "verification_mode":"ReferenceModelOnly",
            "claim_ceiling":"ReferenceModelOnly",
            "session_id":"ak-quote-self-test",
            "ak_public_key_pem":pem.read_text(encoding="utf-8"),
            "ak_public_key_sha256":spki_sha,
            "ak_public_source_sha256":public_rep_sha,
            "ak_public_wire_hex":wire.hex(),
            "ak_public_wire_sha256":hashlib.sha256(wire).hexdigest(),
            "ak_name_hex":name,
            "lineage_binding":{
                "state":"PASS","name_hex":name,"public_area_sha256":hashlib.sha256(wire).hexdigest(),
                "qualified_name_hex":(SHA256_ID+hashlib.sha256(b"parent"+bytes.fromhex(name)).digest()).hex(),
                "source_sha256":"22"*32
            },
            "quote_binding":{
                "state":"PASS","quote_signature_source_sha256":"33"*32,
                "ak_public_spki_sha256":spki_sha
            }
        }
        m["session_binding_sha256"]=session_binding(m)
        return m

def mutate_wire_byte(v:dict[str,Any],idx:int)->None:
    raw=bytearray(hex_bytes(v["ak_public_wire_hex"],"ak_public_wire_hex"));raw[idx]^=1
    v["ak_public_wire_hex"]=bytes(raw).hex()
    v["ak_public_wire_sha256"]=hashlib.sha256(raw).hexdigest()
    v["ak_name_hex"]=(SHA256_ID+hashlib.sha256(raw).digest()).hex()

def self_test()->int:
    try:
        base=generate_fixture()
    except RuntimeError as exc:
        print(f"AK Quote key-binding fixture: INDETERMINATE ({exc})")
        return 2
    cases=[
        ("canonical-valid","PASS",lambda x:x),
        ("public-key-digest-substitution","DENY",lambda x:x.update({"ak_public_key_sha256":"44"*32})),
        ("public-source-digest-substitution","DENY",lambda x:x.update({"ak_public_source_sha256":"55"*32})),
        ("public-wire-digest-substitution","DENY",lambda x:x.update({"ak_public_wire_sha256":"55"*32})),
        ("public-wire-substitution","DENY",lambda x:mutate_wire_byte(x,17)),
        ("name-substitution","DENY",lambda x:x.update({"ak_name_hex":"000b"+"66"*32})),
        ("lineage-name-substitution","DENY",lambda x:x["lineage_binding"].update({"name_hex":"000b"+"77"*32})),
        ("lineage-area-substitution","DENY",lambda x:x["lineage_binding"].update({"public_area_sha256":"88"*32})),
        ("lineage-source-substitution","DENY",lambda x:x["lineage_binding"].update({"source_sha256":"99"*32})),
        ("quote-key-substitution","DENY",lambda x:x["quote_binding"].update({"ak_public_spki_sha256":"aa"*32})),
        ("quote-source-substitution","DENY",lambda x:x["quote_binding"].update({"quote_signature_source_sha256":"bb"*32})),
        ("key-type-substitution","DENY",lambda x:x.update({"ak_public_wire_hex":"0002"+x["ak_public_wire_hex"][4:]})),
        ("rsa-modulus-substitution","DENY",lambda x:mutate_wire_byte(x,len(hex_bytes(x["ak_public_wire_hex"],"ak_public_wire_hex"))-1)),
        ("rsa-exponent-substitution","DENY",lambda x:mutate_wire_byte(x,18)),
        ("offline-origin","INDETERMINATE",lambda x:x.update({"verification_mode":"OfflineBundle"})),
        ("live-origin","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession"})),
    ]
    for name,expected,mut in cases:
        c=copy.deepcopy(base);mut(c);o=verify(c)
        if o["state"]!=expected:
            print(f"{name}: FAIL expected={expected} got={o['state']} reason={o['reason']}")
            return 1
    p=json.loads(json.dumps(base,sort_keys=True))
    if verify(p)["state"]!="PASS":
        print("key-order-permutation: FAIL")
        return 1
    print("AK Quote key-binding semantic corpus: PASS")
    print("15 adversarial mutations plus canonical and key-order control: PASS")
    return 0

def main()->int:
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument("--self-test",action="store_true");g.add_argument("--verify",metavar="MANIFEST");p.add_argument("--output");a=p.parse_args()
    if a.self_test:return self_test()
    m=json.loads(Path(a.verify).read_text(encoding="utf-8"));v=verify(m);o={"profile_id":"mycelix.security.tpm.ak-quote-key-binding","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(Path(a.verify).read_bytes()).hexdigest(),**v};o["content_sha256"]=canonical_hash({k:v for k,v in o.items() if k!="content_sha256"});r=json.dumps(o,indent=2,sort_keys=True)+"\\n"
    if a.output:Path(a.output).write_text(r,encoding="utf-8")
    else:print(r,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]
if __name__=="__main__":raise SystemExit(main())
