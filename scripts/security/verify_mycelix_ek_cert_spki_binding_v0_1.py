#!/usr/bin/env python3
"""Verify EK X.509 leaf SubjectPublicKeyInfo matches TPM EK public key."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-spki-binding.v0.1"
RSA_ID = b"\x00\x01"
SHA256_ID = b"\x00\x0b"
NULL_ID = b"\x00\x10"
RSA_ENCRYPTION_DER = bytes.fromhex("300d06092a864886f70d0101010500")
APPROVED_CERT_SOURCE_SHA256 = "11" * 32
APPROVED_EK_SOURCE_SHA256 = "22" * 32
FIXTURE_CERT_DER = bytes.fromhex("3082031b30820203a00302010202147d73e128cc4b3d74ce6ec071d53c19d967cb9493300d06092a864886f70d01010b0500301d311b301906035504030c124d7963656c697820454b2046697874757265301e170d3236313030343233333331365a170d3336313030313233333331365a301d311b301906035504030c124d7963656c697820454b204669787475726530820122300d06092a864886f70d01010105000382010f003082010a0282010100d019fb7bdf2679af2d0bf1d5438dae19f73cad698d171a5e3292506bcd05d8b85b7753f35b9c174105fab4613ccc54a67f43fa97c6ef9330a101897b39c3d0f730ee8001b0b53511846de96731104c242781a1fedee583b72c1205a8ace27bcea878ca22c3be355abd7989a3b8e0f9a384a0b6f3e3a8d8bfc35048d93fc07cf45957a52083ed2a49ce01016dcbbb66d4a39569de30285318f4f5a9ff364a80858cf16e84ee4182de3a282c972b6545ee7aa62d48202043cd006e2a5c84c575499b750226ad19c0a3faea19eb0b813a5e31907fb541ea11e3d3a05a39b120ba3b944dd87da4cb89c3e548d0a05e7b538b5e97c1ecd34290d0b0d2e413e369e6570203010001a3533051301d0603551d0e04160414b9bd4183c0c00b643103ca6e870fdc4c55ab2b8e301f0603551d23041830168014b9bd4183c0c00b643103ca6e870fdc4c55ab2b8e300f0603551d130101ff040530030101ff300d06092a864886f70d01010b05000382010100ca2bcafef856003426cc84a72807812577d1c5763b1e5e8abd039b345d8f58144fe8c3104006341b94ef297b5302a239f3c2bc2e7a29fec6ae850ffb02080517fff18f37d053df726a41771fd341ea69666a08bcf6a848a7411c931865fd39141d443cba99ba5adb7ccac83b7ce7a32e9c483db1cc5bafc2a1590fce0fdccb26f7a070a406c2d1e7f3ace9c2e9b4db2cb81c7326b00276fa0a0bce17b7b508183b9bb603139b38675ac64e90d07f7d03a1b1b869d59d592d1131760af3fe4fd4d9b52945be5ba6b2cbda66d3abbd54d4771bc9ebbb60c12532c1f21c74797fee2cb29cf2d793b27df8af375d88133ff10ca30b8f9a28f90517908bcc0f9fa3b1")
FIXTURE_TPM_PUBLIC = bytes.fromhex("0001000b000300b20020837197674484b3f81a90cc8d46a5d724fd52d76e06520b64f2a1da1b331469aa00060080004300100800000000000100d019fb7bdf2679af2d0bf1d5438dae19f73cad698d171a5e3292506bcd05d8b85b7753f35b9c174105fab4613ccc54a67f43fa97c6ef9330a101897b39c3d0f730ee8001b0b53511846de96731104c242781a1fedee583b72c1205a8ace27bcea878ca22c3be355abd7989a3b8e0f9a384a0b6f3e3a8d8bfc35048d93fc07cf45957a52083ed2a49ce01016dcbbb66d4a39569de30285318f4f5a9ff364a80858cf16e84ee4182de3a282c972b6545ee7aa62d48202043cd006e2a5c84c575499b750226ad19c0a3faea19eb0b813a5e31907fb541ea11e3d3a05a39b120ba3b944dd87da4cb89c3e548d0a05e7b538b5e97c1ecd34290d0b0d2e413e369e657")

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()

def valid_hash(value: Any) -> bool:
    return isinstance(value,str) and len(value)==64 and all(c in "0123456789abcdef" for c in value)

def hex_bytes(value: Any,field: str) -> bytes:
    if not isinstance(value,str):
        raise ValueError(f"{field} must be hex")
    value=value.lower().removeprefix("0x")
    if len(value)%2 or any(c not in "0123456789abcdef" for c in value):
        raise ValueError(f"{field} is not canonical hex")
    return bytes.fromhex(value)

def der_tlv_parse(data: bytes, offset: int) -> tuple[int, bytes, int]:
    if offset >= len(data):
        raise ValueError("DER truncated at tag")
    tag = data[offset]
    offset += 1
    if offset >= len(data):
        raise ValueError("DER truncated at length")
    first = data[offset]
    offset += 1
    if first & 0x80:
        count = first & 0x7F
        if count == 0 or count > 4 or offset + count > len(data):
            raise ValueError("DER invalid length")
        length = int.from_bytes(data[offset:offset + count], "big")
        offset += count
    else:
        length = first
    end = offset + length
    if end > len(data):
        raise ValueError("DER truncated value")
    return tag, data[offset:end], end


def require_rsa_certificate_spki(cert_der: bytes) -> dict[str, Any]:
    tag, cert_content, cert_end = der_tlv_parse(cert_der, 0)
    if tag != 0x30 or cert_end != len(cert_der):
        raise ValueError("certificate is not one DER SEQUENCE")
    tag, tbs, cursor = der_tlv_parse(cert_content, 0)
    if tag != 0x30:
        raise ValueError("TBSCertificate is not a SEQUENCE")
    # version is optional [0]; then serial, signature, issuer, validity, subject, SPKI
    tag, _content, next_cursor = der_tlv_parse(tbs, 0)
    cursor_tbs = next_cursor if tag == 0xA0 else 0
    for _ in range(5):
        _tag, _content, cursor_tbs = der_tlv_parse(tbs, cursor_tbs)
    spki_tag, spki_content, cursor_tbs = der_tlv_parse(tbs, cursor_tbs)
    if spki_tag != 0x30 or cursor_tbs != len(tbs):
        raise ValueError("SubjectPublicKeyInfo malformed")
    alg_tag, alg_content, end1 = der_tlv_parse(spki_content, 0)
    if alg_tag != 0x30:
        raise ValueError("SPKI AlgorithmIdentifier malformed")
    alg_offset = end1
    alg2_tag, alg2_content, alg2_end = der_tlv_parse(spki_content, alg_offset)
    if alg2_tag != 0x03 or alg2_end != len(spki_content):
        raise ValueError("SPKI public-key BIT STRING malformed")
    alg_cursor = 0
    oid_tag, oid_content, oid_end = der_tlv_parse(alg_content, alg_cursor)
    if oid_tag != 0x06 or oid_content != bytes.fromhex("2a864886f70d010101"):
        raise ValueError("SPKI algorithm is not rsaEncryption")
    param_tag, param_content, param_end = der_tlv_parse(alg_content, oid_end)
    if param_tag != 0x05 or param_content != b"" or param_end != len(alg_content):
        raise ValueError("rsaEncryption parameters must be ASN.1 NULL")
    bit_string = alg2_content
    if not bit_string or bit_string[0] != 0:
        raise ValueError("SPKI BIT STRING must have zero unused bits")
    key = bit_string[1:]
    key_tag, key_content, key_end = der_tlv_parse(key, 0)
    if key_tag != 0x30 or key_end != len(key):
        raise ValueError("RSAPublicKey is not a SEQUENCE")
    n_tag, n_content, n_end = der_tlv_parse(key_content, 0)
    e_tag, e_content, e_end = der_tlv_parse(key_content, n_end)
    if n_tag != 0x02 or e_tag != 0x02 or e_end != len(key_content):
        raise ValueError("RSAPublicKey INTEGER structure malformed")
    if not n_content or n_content[0] & 0x80:
        raise ValueError("RSAPublicKey modulus INTEGER must be positive")
    modulus = n_content.lstrip(b"\\x00")
    exponent = int.from_bytes(e_content, "big") if e_content else 0
    if len(modulus) != 256 or not (modulus[0] & 0x80):
        raise ValueError("RSA EK certificate modulus is not 2048 bits")
    if exponent != 65537:
        raise ValueError("RSA EK certificate public exponent is not 65537")
    return {
        "algorithm_oid": "1.2.840.113549.1.1.1",
        "algorithm_parameters": "NULL",
        "modulus_sha256": hashlib.sha256(modulus).hexdigest(),
        "public_exponent": exponent,
    }


def der_len(n:int)->bytes:
    if n < 128:
        return bytes([n])
    raw=n.to_bytes((n.bit_length()+7)//8,"big")
    return bytes([0x80|len(raw)])+raw

def der_tlv(tag:int,value:bytes)->bytes:
    return bytes([tag])+der_len(len(value))+value

def der_integer(value:int)->bytes:
    raw=value.to_bytes(max(1,(value.bit_length()+7)//8),"big")
    if raw[0]&0x80:
        raw=b"\x00"+raw
    return der_tlv(0x02,raw)

def rsa_spki_der(modulus:bytes,exponent:int)->bytes:
    rsa_key=der_tlv(0x30,der_integer(int.from_bytes(modulus,"big"))+der_integer(exponent))
    return der_tlv(0x30,RSA_ENCRYPTION_DER+der_tlv(0x03,b"\x00"+rsa_key))

def parse_tpm_rsa_public(raw:bytes,fmt:str)->tuple[bytes,int]:
    if fmt=="TPM2B_PUBLIC":
        if len(raw)<2:
            raise ValueError("TPM2B_PUBLIC truncated")
        declared=int.from_bytes(raw[:2],"big")
        if declared!=len(raw)-2:
            raise ValueError("TPM2B_PUBLIC size mismatch")
        raw=raw[2:]
    elif fmt!="TPMT_PUBLIC":
        raise ValueError("unsupported EK public format")
    if len(raw)<12:
        raise ValueError("TPMT_PUBLIC truncated")
    off=0
    obj_type=raw[off:off+2]; off+=2
    _name_alg=raw[off:off+2]; off+=2
    off+=4
    policy_size=int.from_bytes(raw[off:off+2],"big"); off+=2
    if policy_size>len(raw)-off:
        raise ValueError("authPolicy truncated")
    off+=policy_size
    if off+2>len(raw):
        raise ValueError("symmetric definition truncated")
    sym_alg=raw[off:off+2]; off+=2
    if sym_alg!=NULL_ID:
        if off+4>len(raw): raise ValueError("symmetric parameters truncated")
        off+=4
    if off+2>len(raw):
        raise ValueError("RSA scheme truncated")
    scheme=raw[off:off+2]; off+=2
    if scheme!=NULL_ID:
        if off+2>len(raw): raise ValueError("RSA scheme hash truncated")
        off+=2
    if off+2>len(raw):
        raise ValueError("RSA keyBits truncated")
    off+=2
    if off+4>len(raw):
        raise ValueError("RSA exponent truncated")
    exponent=int.from_bytes(raw[off:off+4],"big"); off+=4
    if off+2>len(raw):
        raise ValueError("RSA unique size truncated")
    unique_size=int.from_bytes(raw[off:off+2],"big"); off+=2
    if unique_size>len(raw)-off:
        raise ValueError("RSA modulus truncated")
    modulus=raw[off:off+unique_size]; off+=unique_size
    if off!=len(raw):
        raise ValueError("unexpected trailing TPMT_PUBLIC bytes")
    if obj_type!=RSA_ID:
        raise ValueError("EK public type is not RSA")
    if unique_size<256:
        raise ValueError("RSA EK modulus is shorter than 2048 bits")
    if exponent==0:
        exponent=65537
    return modulus,exponent

def extract_cert_spki(cert_der:bytes,work:Path)->tuple[bytes,str]:
    if shutil.which("openssl") is None:
        raise RuntimeError("openssl executable not found")
    cert_path=work/"ek-cert.der"
    pem_path=work/"ek-spki.pem"
    spki_path=work/"ek-spki.der"
    cert_path.write_bytes(cert_der)
    version=subprocess.run(["openssl","version"],text=True,capture_output=True,check=False)
    if version.returncode!=0:
        raise RuntimeError("openssl version failed")
    parse=subprocess.run(["openssl","x509","-inform","DER","-in",str(cert_path),"-noout"],text=True,capture_output=True,check=False)
    if parse.returncode!=0:
        raise RuntimeError(f"OpenSSL certificate parse failed: {parse.stderr}")
    pub=subprocess.run(["openssl","x509","-inform","DER","-in",str(cert_path),"-pubkey","-noout"],text=True,capture_output=True,check=False)
    if pub.returncode!=0:
        raise RuntimeError(f"OpenSSL SPKI extraction failed: {pub.stderr}")
    pem_path.write_text(pub.stdout,encoding="utf-8")
    canon=subprocess.run(["openssl","pkey","-pubin","-in",str(pem_path),"-outform","DER","-out",str(spki_path)],text=True,capture_output=True,check=False)
    if canon.returncode!=0:
        raise RuntimeError(f"OpenSSL SPKI canonicalization failed: {canon.stderr}")
    return spki_path.read_bytes(),version.stdout.strip()

def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details: out["details"]=details
    return out

def binding_hash(session_id:str,tpm_digest:str,cert_digest:str,public_digest:str)->str:
    return canonical_hash({"session_id":session_id,"tpm_identity_digest":tpm_digest,"certificate_der_sha256":cert_digest,"ek_public_wire_sha256":public_digest})

def verify(manifest:dict[str,Any])->dict[str,Any]:
    required={"profile_id","profile_version","session_id","tpm_identity_digest","verification_mode","claim_ceiling","certificate_der_hex","certificate_der_sha256","ek_public_format","ek_public_wire_hex","ek_public_wire_sha256","certificate_source_sha256","ek_public_source_sha256","session_binding_sha256"}
    missing=sorted(required-set(manifest))
    if missing:return result("DENY","missing-required-fields",{"fields":missing})
    if manifest["profile_id"]!="mycelix.security.tpm.ek-cert-spki-binding":return result("DENY","profile-id-mismatch")
    if manifest["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if manifest["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if manifest["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    if not isinstance(manifest["session_id"],str) or not manifest["session_id"]:return result("DENY","session-id-invalid")
    for field in ("tpm_identity_digest","certificate_der_sha256","ek_public_wire_sha256","certificate_source_sha256","ek_public_source_sha256","session_binding_sha256"):
        if not valid_hash(manifest[field]):return result("DENY","digest-invalid",{"field":field})
    if binding_hash(manifest["session_id"],manifest["tpm_identity_digest"],manifest["certificate_der_sha256"],manifest["ek_public_wire_sha256"])!=manifest["session_binding_sha256"]:
        return result("DENY","session-binding-mismatch")
    try:
        cert_der=hex_bytes(manifest["certificate_der_hex"],"certificate_der_hex")
        public_wire=hex_bytes(manifest["ek_public_wire_hex"],"ek_public_wire_hex")
    except ValueError as exc:
        return result("DENY","malformed-hex",{"error":str(exc)})
    if hashlib.sha256(cert_der).hexdigest()!=manifest["certificate_der_sha256"]:return result("DENY","certificate-digest-mismatch")
    if hashlib.sha256(public_wire).hexdigest()!=manifest["ek_public_wire_sha256"]:return result("DENY","ek-public-digest-mismatch")
    try:
        certificate_spki_profile = require_rsa_certificate_spki(cert_der)
    except ValueError as exc:
        return result("DENY","certificate-spki-profile-invalid",{"error":str(exc)})
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-spki-") as td:
        try:
            cert_spki,openssl_version=extract_cert_spki(cert_der,Path(td))
            modulus,exponent=parse_tpm_rsa_public(public_wire,manifest["ek_public_format"])
        except (RuntimeError,ValueError) as exc:
            return result("DENY","key-material-parse-failed",{"error":str(exc)})
        tpm_spki=rsa_spki_der(modulus,exponent)
    details={"certificate_spki_sha256":hashlib.sha256(cert_spki).hexdigest(),"certificate_spki_algorithm":certificate_spki_profile["algorithm_oid"],"certificate_spki_parameters":certificate_spki_profile["algorithm_parameters"],"certificate_modulus_sha256":certificate_spki_profile["modulus_sha256"],"tpm_spki_sha256":hashlib.sha256(tpm_spki).hexdigest(),"spki_equal":cert_spki==tpm_spki,"rsa_modulus_sha256":hashlib.sha256(modulus).hexdigest(),"rsa_exponent_hex":exponent.to_bytes(4,"big").hex(),"openssl_version":openssl_version,"certificate_der_sha256":manifest["certificate_der_sha256"],"ek_public_wire_sha256":manifest["ek_public_wire_sha256"],"session_binding_sha256":manifest["session_binding_sha256"]}
    if cert_spki!=tpm_spki:return result("DENY","certificate-spki-does-not-match-ek-public",details)
    if manifest["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","issuer-and-live-origin-not-authorized-by-this-theorem",details)
    if manifest["certificate_source_sha256"]!=APPROVED_CERT_SOURCE_SHA256:return result("DENY","reference-certificate-source-not-approved",details)
    if manifest["ek_public_source_sha256"]!=APPROVED_EK_SOURCE_SHA256:return result("DENY","reference-ek-source-not-approved",details)
    return result("PASS","ek-certificate-spki-bound-to-tpm-public",details)

def fixture()->dict[str,Any]:
    cert=FIXTURE_CERT_DER
    public=FIXTURE_TPM_PUBLIC
    cd=hashlib.sha256(cert).hexdigest()
    pd=hashlib.sha256(public).hexdigest()
    session="ek-spki-self-test"
    tpm="33"*32
    return {"profile_id":"mycelix.security.tpm.ek-cert-spki-binding","profile_version":"0.1.0","session_id":session,"tpm_identity_digest":tpm,"verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","certificate_der_hex":cert.hex(),"certificate_der_sha256":cd,"ek_public_format":"TPMT_PUBLIC","ek_public_wire_hex":public.hex(),"ek_public_wire_sha256":pd,"certificate_source_sha256":APPROVED_CERT_SOURCE_SHA256,"ek_public_source_sha256":APPROVED_EK_SOURCE_SHA256,"session_binding_sha256":binding_hash(session,tpm,cd,pd)}

def mutate_public_byte(value:dict[str,Any],index:int)->None:
    raw=bytearray(hex_bytes(value["ek_public_wire_hex"],"ek_public_wire_hex"))
    raw[index]^=1
    value["ek_public_wire_hex"]=bytes(raw).hex()
    value["ek_public_wire_sha256"]=hashlib.sha256(raw).hexdigest()
    value["session_binding_sha256"]=binding_hash(value["session_id"],value["tpm_identity_digest"],value["certificate_der_sha256"],value["ek_public_wire_sha256"])

def mutate_public_wrapper(value:dict[str,Any])->None:
    wrapped=len(FIXTURE_TPM_PUBLIC).to_bytes(2,"big")+FIXTURE_TPM_PUBLIC
    bad=bytearray(wrapped);bad[0:2]=b"\xff\xff"
    value["ek_public_format"]="TPM2B_PUBLIC"
    value["ek_public_wire_hex"]=bytes(bad).hex()
    value["ek_public_wire_sha256"]=hashlib.sha256(bad).hexdigest()
    value["session_binding_sha256"]=binding_hash(value["session_id"],value["tpm_identity_digest"],value["certificate_der_sha256"],value["ek_public_wire_sha256"])

def self_test()->int:
    base=fixture()
    cases=[
        ("canonical-rsa-match","PASS",lambda x:x),
        ("certificate-der-malformed","DENY",lambda x:x.update({"certificate_der_hex":"zz"})),
        ("certificate-digest-substitution","DENY",lambda x:x.update({"certificate_der_sha256":"44"*32})),
        ("ek-public-digest-substitution","DENY",lambda x:x.update({"ek_public_wire_sha256":"55"*32})),
        ("ek-modulus-substitution","DENY",lambda x:mutate_public_byte(x,len(FIXTURE_TPM_PUBLIC)-1)),
        ("ek-exponent-substitution","DENY",lambda x:mutate_public_byte(x,52)),
        ("ek-public-type-substitution","DENY",lambda x:x.update({"ek_public_wire_hex":"0002"+x["ek_public_wire_hex"][4:]})),
        ("tpm2b-size-substitution","DENY",mutate_public_wrapper),
        ("certificate-byte-substitution","DENY",lambda x:x.update({"certificate_der_hex":x["certificate_der_hex"][:-2]+"00"})),
        ("certificate-source-substitution","DENY",lambda x:x.update({"certificate_source_sha256":"66"*32})),
        ("certificate-spki-parameters-substitution","DENY",lambda x:x.update({"certificate_der_hex":x["certificate_der_hex"].replace("300d06092a864886f70d0101010500","300d06092a864886f70d0101010400")})),
        ("ek-public-source-substitution","DENY",lambda x:x.update({"ek_public_source_sha256":"77"*32})),
        ("session-binding-substitution","DENY",lambda x:x.update({"session_id":"attacker"})),
        ("tpm-identity-substitution","DENY",lambda x:x.update({"tpm_identity_digest":"88"*32})),
        ("offline-bundle","INDETERMINATE",lambda x:x.update({"verification_mode":"OfflineBundle"})),
        ("live-verifier","INDETERMINATE",lambda x:x.update({"verification_mode":"LiveVerifierSession"})),
    ]
    for name,expected,mut in cases:
        c=copy.deepcopy(base)
        mut(c)
        observed=verify(c)
        if observed["state"]!=expected:
            print(f"{name}: FAIL expected={expected} got={observed['state']} reason={observed['reason']}")
            return 1
    perm=json.loads(json.dumps(base,sort_keys=True))
    if verify(perm)["state"]!="PASS":
        print("key-order-permutation: FAIL")
        return 1
    print("EK certificate SPKI binding semantic corpus: PASS")
    print("16 adversarial mutations plus canonical case: PASS")
    print("issuer trust, validity, revocation, and manufacturer authenticity remain separate")
    return 0

def main()->int:
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--verify",metavar="MANIFEST")
    group.add_argument("--output",metavar="FILE")
    args=parser.parse_args()
    if args.self_test:return self_test()
    path=Path(args.verify).resolve()
    manifest=json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest,dict):raise SystemExit("manifest must be an object")
    verified=verify(manifest)
    output={"profile_id":"mycelix.security.tpm.ek-cert-spki-binding","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**verified}
    output["content_sha256"]=canonical_hash({k:v for k,v in output.items() if k!="content_sha256"})
    rendered=json.dumps(output,indent=2,sort_keys=True)+"\n"
    if args.output:Path(args.output).write_text(rendered,encoding="utf-8")
    else:print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[verified["state"]]

if __name__=="__main__":
    raise SystemExit(main())
