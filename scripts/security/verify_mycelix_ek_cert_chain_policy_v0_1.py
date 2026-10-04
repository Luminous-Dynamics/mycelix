#!/usr/bin/env python3
"""Verify EK certificate path, profile fields, revocation, and SPKI binding."""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

VERIFIER_ID = "mycelix.tpm.ek-cert-chain-policy.v0.1"
SPKI_VERIFIER_ID = "mycelix.tpm.ek-cert-spki-binding.v0.1"
EK_CERT_EKU_OID = "2.23.133.8.1"
REFERENCE_ROOT_SHA256 = "bf027c125d7641b37bb1e8e71eab5f480cc3b925eb33660b818b251faf5fa6bc"
REFERENCE_ROOT_SOURCE_SHA256 = "31" * 32
REFERENCE_TIME_UNIX = 1791158400
EXPIRED_TIME_UNIX = 2114380800
NOT_YET_VALID_TIME_UNIX = 1790985600

FIXTURE_ROOT_DER = base64.b64decode("MIIEQjCCAqqgAwIBAgIULEddzaMVkKOSjnAJ6SKUdx2J/VkwDQYJKoZIhvcNAQELBQAwJzElMCMGA1UEAwwcTXljZWxpeCBTeW50aGV0aWMgRUsgUm9vdCBDQTAeFw0yNjEwMDQyMzQ4NTZaFw0zNjEwMDEyMzQ4NTZaMCcxJTAjBgNVBAMMHE15Y2VsaXggU3ludGhldGljIEVLIFJvb3QgQ0EwggGiMA0GCSqGSIb3DQEBAQUAA4IBjwAwggGKAoIBgQDiIDnhv1z+2mRD53Bh/NBB/19rNc6zgEG87nT4LPgpgtznXMymI2zefft/FpaCUrfEb4G/kJqoLdY7EUhR6I96g5AGKCbjwUwCLV6twW6b5JsquxugJwvAJbtaQHUPaA58+A0CK12js3mYgEWxIsg9bIBsXoeRHsNI5mrkIEq1BaXXT09c/P4sxnBKS4tdSOmZTWcFKzgqwzpsmttwqzkkwkfNocDTyTrMxJDscWpXtCcnmwuxUHCnwWu5USftj2MxuAiod53iyXTpMlO0JYaHDQvlt/jucmwZpE0piHOKqpT3rdn4MKXflz4/6lhbEqe2tyj94eYVztm1jZtbnzVJjyWmHZ1xKRklvltG5dsNe2yAqa/ywcY7iSPFDR4/2W1u6sxd97cZ1fGjcdH30pgEptcLXXqEbQpyCT478oV2Wa20U0W05lG1h0GrN0hxzJLSBez9R+2jYdPMF2eNAfSpNi1n4poOUWB3R7p28XO/V1C4pbeWxip7q12FTdoLVZMCAwEAAaNmMGQwHQYDVR0OBBYEFJ1OmkQgfmin8ooqaiyKLtHzKUtUMB8GA1UdIwQYMBaAFJ1OmkQgfmin8ooqaiyKLtHzKUtUMBIGA1UdEwEB/wQIMAYBAf8CAQEwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3DQEBCwUAA4IBgQDG01ySs66RTs+2mX4MupC7j799d8yVG5iMbYgdHF+nBbaC29oV0qy7daUOyNKj9bXS71yGQsR+U7Dn13KReqRewWl11bckWc1WDVOS27CHKxVJSfWB1sXcoEHF2GYXpRw/THk/0BAR644SHPVeJ66kGKlm/7z/Q0/Hzg4TtCZlklOF8s1N/nZaO7gvOszwFKGr7caxincJlsuIbWf1/sYvjZ9kUpdCeKlz5qkY/V/4RbAmN0mDRtwy7MJ8sx93YbUb6xKpFGdkyISnsvncNbLwmVheMVl/5vG2kw0OytqP1FM3A/1Ylfnq7mjvrFSQDObDQmB8O+iAJZmJe49cpmThFXROQGCBpartPwQ4PAOg0nAQqcKmlhrylgOdjgJj8hbH7T5g4dRSUSoROH+Vz57K1Us87pS/X8fGqEpU2Qf9eaU3NwsVt0Q7LWDKLpUPN/72Go=")
FIXTURE_INTERMEDIATE_DER = base64.b64decode("MIIERTCCAq2gAwIBAgIUQaNPQeYEYRPJwGWSV4s6ZVU4/FIwDQYJKoZIhvcNAQELBQAwJzElMCMGA1UEAwwcTXljZWxpeCBTeW50aGV0aWMgRUsgUm9vdCBDQTAeFw0yNjEwMDQyMzQ4NTZaFw0zMzA4MDgyMzQ4NTZaMCoxKDAmBgNVBAMMH015Y2VsaXggU3ludGhldGljIEVLIElzc3VpbmcgQ0EwggGiMA0GCSqGSIb3DQEBAQUAA4IBjwAwggGKAoIBgQCzVdOFFaQ3ohVHW/QaWTTd4iYqZnYhpsao8QdAFrHgemZuCYkoeSIqlg0WflJ4Fb2D1D/3NzsWcy5m220WDpltqdjFGXJsgIPVUjAFIVwcN7wAMFrjZR+1LengnqzrntgxAJ6cwe+Sjg/dokqsro8WSpuMZclPczP8bC0y1gk1lt5NsAz6m1AyiJ7O2lS7eHPi1/Tq2v2FTmsFL1zgjVwz+HdhRAeM/r2QTyoupH9MvUtAahsv3SGc+WWThtZxJx1Ol8DHdkrl71kARFtaGCncLBZiuSfgLVAEbQnnGlX08XodcjSH/cenWX6RpMACduvCW3aR8t1LWYQXokwUl9XMS/1lfWzN/CrQOTRlR15YlnGtahQSZr/HpZY0qBpAjVDzJhHlw2L/FMv3CB3mdvLeLX4TZYo/SQFbJMjuxm15SRTwNCiFXcWMtZj2ASsNUtx+G3BZ/oD6+6jSlriWujv7arq6RdtTvzsHrDjtqIHjTq2TGySLxz4dWldcUlxs1AcCAwEAAaNmMGQwHQYDVR0OBBYEFAXOeSjqknJfLWel6KBbrENKE2p+MB8GA1UdIwQYMBaAFJ1OmkQgfmin8ooqaiyKLtHzKUtUMBIGA1UdEwEB/wQIMAYBAf8CAQ0DgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3DQEBCwUAA4IBgQDS0l4NR4q1QHws10dqr0dzEq/iNYJfbG8XC8ehvTW4CsYXPGYymVWxDRUXF/4toEXy1UdEbEvePdI24owyJiizAGfsQVBZcgWd8DdK7GYdk5bjz37tOo7S3alGx0w8qrwTwqQIAYQyyGIDWYS4oYCIpyZUbdKZLE9NF5ke2pns+8v2p0s8aEYY2W/AYd+2rP4JkZqrmTqscknxWhbWpMSsNdweF5pH/VoUakh9FnkygA30BgTRC6yF+4d2zoemsVAPmLqRFRG3FVz5pdQWzrky2bi3LqtknCmN2VCQeUGOj439Nm4dfzlSjQYz1BfmMSQe8mzNi9AjYY8k34NEfq0xangviTS1QkSi5UZoxnwG4iyzlhhx3UXao4PKMl+qF1CYFFuWKW1i4DGUb40mK3fW2+rd0ysZoWdbl/7X5J2RaGZGu8mezFCl1gCyLQp1Qa9Wgp1VI5dvAYzW6+7JlMza9vc7DT/Mm/v0fl0pqsR4qf/vWJu6QFNV0znej36zTU0=")
FIXTURE_LEAF_DER = base64.b64decode("MIIEKDCCApCgAwIBAgIUT+d+UrjGpprscGRrx9nqBIin2TowDQYJKoZIhvcNAQELBQAwKjEoMCYGA1UEAwwfTXljZWxpeCBTeW50aGV0aWMgRUsgSXNzdWluZyBDQTAeFw0yNjEwMDQyMzQ4NTZaFw0zNjEwMDEyMzQ4NTZaMEwxHTAbBgNVBAMMFE15Y2VsaXggU3ludGhldGljIEVLMQswCQYDVQQLDAJFSzEeMBwGA1UECgwVTXljZWxpeCBSZWZlcmVuY2UgTGFiMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA3vEofGkoVQGvCx5MlCmB0CKMb0qjjxd9yy41k4+JJ58x9mjhWPZcjq/ctSUR8jmq/6mZzEtxlnauQikeUNPQqDG/XGmNZtCGDM/jq33ceMyrmurSE4UTIhl4qtzTmmhRJuENODdQCC+lBp1RZk9PNb5gAI8oos5xEZUOV2puTIdxwlatIvCv4T7JECz274F2symTazugAShxWFBvNGgR/T1xMIqvbC/pIqlqkzXHPlBLRtbqFnHPLBQKlqtl6dn6SF2YICeKzFUXGczPeDWHPlSENHVSJQD3oRg3w3v6QCxuhvYtHe1/KZUXwwps0p6GWnOP95ndp58A9J8Ta1BepQIDAQABo4GjMIGgMB0GA1UdDgQWBBT/egxQaqXK/eTa0iT7iM7UYnWSAzAfBgNVHSMEGDAWgBQFznko6pJyXy1npeigW6xDShNqfjAMBgNVHRMBAf8EAjAAMA4GA1UdDwEB/wQEAwIFIDAQBgNVHSUECTAHBgVngQUIATAuBgNVHREBAf8EJDAihiB1cm46bXljZWxpeDpzeW50aGV0aWMtZWstZml4dHVyZTANBgkqhkiG9w0BAQsFAAOCAYEAJ+egBvgvWuoWJstMFGFcWBEGVFRG4z8A0aET/Dy2BDc/CSOTQt9q5rUxZEID/tXVaGJC+shI37y02WQ4YYvdEFNXjOXKqKUEO3P408rv5znWGVMBOxcINjgwMr694sHxlCnD/5sekmEEvrs88muM67idIZZ5Hp2mmZLDGWDYdMQ3Fyj9UohF7H+J/87A6H825AvtnAD8G3dERK1raL1jpYpqcahEaB1tJ1PgybQb5H7GtAI1Eh6uC7o24F0AsjTdv7SLk8S2sru1o0nPBe84YMhq8Kb/rioa1mArmIPwTSz0Shq9ETUNYCu4m4/Hec3gt9SY9lMavXmHcQf9aAkB8NRLe7kggwtgtrKYu6EJnLJwBVEoosZMZlFbaqvRBaVzA6JWO/le7qSu719wyjF0nSBIaVkL6pU6JNpNE8b88bLwHGvfFbFd6xf6CWM6rehZKgLzOuw9oaGN2AQBuoepY4hJuTtXmido/IxyDDRRQBKHd+7WlY2CCE5ml/YfpQ/H")
FIXTURE_BAD_USAGE_DER = base64.b64decode("MIIEGTCCAoGgAwIBAgICMDkwDQYJKoZIhvcNAQELBQAwKjEoMCYGA1UEAwwfTXljZWxpeCBTeW50aGV0aWMgRUsgSXNzdWluZyBDQTAeFw0yNjEwMDQyMzUwMDJaFw0zNjEwMDEyMzUwMDJaMEwxHTAbBgNVBAMMFE15Y2VsaXggU3ludGhldGljIEVLMQswCQYDVQQLDAJFSzEeMBwGA1UECgwVTXljZWxpeCBSZWZlcmVuY2UgTGFiMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA3vEofGkoVQGvCx5MlCmB0CKMb0qjjxd9yy41k4+JJ58x9mjhWPZcjq/ctSUR8jmq/6mZzEtxlnauQikeUNPQqDG/XGmNZtCGDM/jq33ceMyrmurSE4UTIhl4qtzTmmhRJuENODdQCC+lBp1RZk9PNb5gAI8oos5xEZUOV2puTIdxwlatIvCv4T7JECz274F2symTazugAShxWFBvNGgR/T1xMIqvbC/pIqlqkzXHPlBLRtbqFnHPLBQKlqtl6dn6SF2YICeKzFUXGczPeDWHPlSENHVSJQD3oRg3w3v6QCxuhvYtHe1/KZUXwwps0p6GWnOP95ndp58A9J8Ta1BepQIDAQABo4GmMIGjMB0GA1UdDgQWBBT/egxQaqXK/eTa0iT7iM7UYnWSAzAfBgNVHSMEGDAWgBQFznko6pJyXy1npeigW6xDShNqfjAMBgNVHRMBAf8EAjAAMA4GA1UdDwEB/wQEAwIHgDATBgNVHSUEDDAKBggrBgEFBQcDATAuBgNVHREBAf8EJDAihiB1cm46bXljZWxpeDpzeW50aGV0aWMtZWstZml4dHVyZTANBgkqhkiG9w0BAQsFAAOCAYEAGQ6Ng9fsxjfR1stVb/9x4MR4G7u1Eu2esnTmduFC6SZvkCoYlhDMmayToAHIoGS3N1RMc2CYYZwPnCzyNbO8B3ffbmw+pOmEW5VFiXS53LzvT3L69YHZccaKBVKyaUiHY+z2Id/xB5TrxzJVf5CgFXnCD/JQNf1cLgzN3LNzJfUyAfzV/g29ZBkvDLdB94LEL/ddNEdky+Lp5ZbQmrBREx2thtjwuNxkNt8BZbSwN3buUW9znlhR8MvOjBabmlKfrSX8JshHINHUbviqXcMeRuaEHlMhIjQUVPiqCUFoI57U2DfdqnolHEBN0gX1/dRTO4Dq0jJQW3jg80Ze9GdSvizwanTFh98j1jIWE2Q5AH34TuuExgDrc2eXq1QiEtxj6Yd6NErN31O06NK/vN45pENf4CygPmUIOep8U8NRDcpraxq0GhvhVsNjlT8VO0YBvyJuPPG1NqgTffjAWatZXNeFhNbYuW296+5XkEGoBeYDRSj4v/jv6n0x7gORmAwK")
FIXTURE_BAD_EKU_DER = base64.b64decode("MIIEGTCCAoGgAwIBAgICMDowDQYJKoZIhvcNAQELBQAwKjEoMCYGA1UEAwwfTXljZWxpeCBTeW50aGV0aWMgRUsgSXNzdWluZyBDQTAeFw0yNjEwMDQyMzUwMDJaFw0zNjEwMDEyMzUwMDJaMEwxHTAbBgNVBAMMFE15Y2VsaXggU3ludGhldGljIEVLMQswCQYDVQQLDAJFSzEeMBwGA1UECgwVTXljZWxpeCBSZWZlcmVuY2UgTGFiMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA3vEofGkoVQGvCx5MlCmB0CKMb0qjjxd9yy41k4+JJ58x9mjhWPZcjq/ctSUR8jmq/6mZzEtxlnauQikeUNPQqDG/XGmNZtCGDM/jq33ceMyrmurSE4UTIhl4qtzTmmhRJuENODdQCC+lBp1RZk9PNb5gAI8oos5xEZUOV2puTIdxwlatIvCv4T7JECz274F2symTazugAShxWFBvNGgR/T1xMIqvbC/pIqlqkzXHPlBLRtbqFnHPLBQKlqtl6dn6SF2YICeKzFUXGczPeDWHPlSENHVSJQD3oRg3w3v6QCxuhvYtHe1/KZUXwwps0p6GWnOP95ndp58A9J8Ta1BepQIDAQABo4GmMIGjMB0GA1UdDgQWBBT/egxQaqXK/eTa0iT7iM7UYnWSAzAfBgNVHSMEGDAWgBQFznko6pJyXy1npeigW6xDShNqfjAMBgNVHRMBAf8EAjAAMA4GA1UdDwEB/wQEAwIFIDATBgNVHSUEDDAKBggrBgEFBQcDAjAuBgNVHREBAf8EJDAihiB1cm46bXljZWxpeDpzeW50aGV0aWMtZWstZml4dHVyZTANBgkqhkiG9w0BAQsFAAOCAYEALVU8kn9tFLBorMKt20CN7ego2PL+kHDL4Uae37NjYwHMMGaRVoiKMCjPZw+99BfemjEDo1UbqhIP+D9Sqj0R8Ey/+ZDCuZS969CcI5kvb3trRYgwQlDVJOaZlUpb2rrLjUo50wMtH1VABhTckRn8iUxbBmjZGr87uLshD92FPLbEjA1cnS3MwbEdKP4L3Y3qZckXB635djblzJsBRx5v4+TXjVClKEE1rDRRzDffh1aShUEnIwDKrrocef6IDETuU0HYmJWGksJnvQXMKbEDHY06bq36wg+cStTV7If4OAwgFgyIo1ThOQ0iPC4hI9TDnA+ik87HamZ8USyz8weY7xjcgH5BWvFaJfz0IsUvOZpF9BhNta8Dc0wpMV9c6wo2foiEQuMpo/DvpJRNwlyobHzf+4Ay5K5NKfYpSJ/vBYCzLj0J0AFFcMrUPlNmFPITlAIB6ctzjEd8Yc+OTxw8zrlPK/tJdOdd6CaiEc/UhqW6QrbLthKZHchAsTddZwkL")
FIXTURE_CRL_DER = base64.b64decode("MIICAzBtAgEBMA0GCSqGSIb3DQEBCwUAMCoxKDAmBgNVBAMMH015Y2VsaXggU3ludGhldGljIEVLIElzc3VpbmcgQ0EXDTI2MTAwNDIzNDk0NloXDTI2MTEwMzIzNDk0NlqgDzANMAsGA1UdFAQEAgIgADANBgkqhkiG9w0BAQsFAAOCAYEAeLjI56s+b2V9C5zwNNh3BY9gxKSxiDm+nVooMepIyVqerEyP6CQ8ILXft6MkJl6leIHBKXzgslxnGKakeupy70cCrMCo0sGr7FHJs38EyTDgc7R+F4mXlTrLKUaYuqcyJpS28+sLorfWN06y3ms/ny5/1MLO0OVTQSsjXTQMWVId2FFVxm1g/6a9sn3UKdN+IpeCXY+ly+8X0Bae7ci3v7Hz6OMgorjNVj8N0fD/tOakvaigsbUJ3a2QH6GSjk5NhxaEhkzK5c9Pfqs3jLSb5EW9Zw3MIZKOUYsrAid/MnfUDuKFDv2sMzn8x4duLxRpg7geQrXcTCGNBT59h9yfIXUg3HmPqiDGUN66eAl/pj/HWKNjeekAjCbRbZn8VnuapcI8MXPZLSmS3UPntzgY7rzKr+ijrpnZ4PknKFAG/69pA1fs91GE+lPhnnFi8hj+4heHp25gLZIzm9cjOy4QXx2e9y1rRhfyTTwjKu0Gb76aCDZfsz7LyIFJUjfDFZfX")

def canonical_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()

def valid_hash(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)

def decode_b64(value: Any, field: str) -> bytes:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be base64")
    try:
        return base64.b64decode(value, validate=True)
    except Exception as exc:
        raise ValueError(f"{field} invalid base64: {exc}") from exc

def run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)

def x509_text(der: bytes, work: Path, prefix: str) -> str:
    path = work / f"{prefix}.der"
    path.write_bytes(der)
    p = run(["openssl","x509","-inform","DER","-in",str(path),"-noout","-text"], work)
    if p.returncode != 0:
        raise ValueError(f"OpenSSL x509 parse failed: {p.stderr}")
    return p.stdout

def x509_scalar(der: bytes, work: Path, flag: str) -> str:
    path = work / "scalar.der"
    path.write_bytes(der)
    p = run(["openssl","x509","-inform","DER","-in",str(path),"-noout",flag], work)
    if p.returncode != 0:
        raise ValueError(f"OpenSSL x509 {flag} failed: {p.stderr}")
    line = p.stdout.strip()
    return line.split("=",1)[1].strip() if "=" in line else line

def extension(text: str, name: str) -> tuple[bool,str|None]:
    pattern = re.compile(
        rf"X509v3 {re.escape(name)}:\\s*(?:\\(critical\\))?\\s*\\n\\s+([^\\n]+)",
        re.IGNORECASE,
    )
    match = pattern.search(text)
    if not match:
        pattern2 = re.compile(
            rf"X509v3 {re.escape(name)}:\\s+critical\\s*\\n\\s+([^\\n]+)",
            re.IGNORECASE,
        )
        match = pattern2.search(text)
    return bool(match and "critical" in match.group(0).lower()), match.group(1).strip() if match else None

def extension_value(text: str, name: str) -> str|None:
    _, value = extension(text, name)
    return value

def parse_aki(value: str|None) -> str:
    if not value:
        return ""
    if "keyid:" in value.lower():
        value = value.split(":",1)[1]
    return re.sub(r"[^0-9a-fA-F]","",value).lower()

def verify_chain(leaf: bytes, inter: bytes, root: bytes, crl: bytes, attime: int, work: Path) -> tuple[bool,str]:
    leaf_der, inter_der, root_der, crl_der = (work/"leaf.der",work/"inter.der",work/"root.der",work/"crl.der")
    leaf_der.write_bytes(leaf); inter_der.write_bytes(inter); root_der.write_bytes(root); crl_der.write_bytes(crl)
    leaf_p, inter_p, root_p, crl_p = (work/"leaf.pem",work/"inter.pem",work/"root.pem",work/"crl.pem")
    for src,dst,kind in ((leaf_der,leaf_p,"x509"),(inter_der,inter_p,"x509"),(root_der,root_p,"x509")):
        p=run(["openssl","x509","-inform","DER","-in",str(src),"-out",str(dst)],work)
        if p.returncode!=0:return False,p.stderr
    p=run(["openssl","crl","-inform","DER","-in",str(crl_der),"-out",str(crl_p)],work)
    if p.returncode!=0:return False,p.stderr
    p=run(["openssl","verify","-CAfile",str(root_p),"-untrusted",str(inter_p),"-x509_strict","-check_ss_sig","-crl_check","-CRLfile",str(crl_p),"-attime",str(attime),str(leaf_p)],work)
    return p.returncode==0,(p.stdout+p.stderr).strip()

def profile_ok(leaf_text: str) -> tuple[bool,dict[str,Any]]:
    version_ok = bool(re.search(r"Version:\\s*3\\s*\\(", leaf_text))
    serial = ""
    try:
        # caller supplies serial text separately in final details
        pass
    except Exception:
        pass
    bc_critical, bc = extension(leaf_text,"Basic Constraints")
    ku_critical, ku = extension(leaf_text,"Key Usage")
    eku_critical, eku = extension(leaf_text,"Extended Key Usage")
    aki_critical, aki = extension(leaf_text,"Authority Key Identifier")
    profile = {
        "version_3": version_ok,
        "basic_constraints_critical": bc_critical,
        "basic_constraints": bc,
        "key_usage_critical": ku_critical,
        "key_usage": ku,
        "extended_key_usage": eku,
        "extended_key_usage_critical": eku_critical,
        "authority_key_identifier": aki,
    }
    eku_ok = eku is None or EK_CERT_EKU_OID in eku or "Endorsement Key Certificate" in eku
    ok = (
        version_ok
        and bc_critical and bc is not None and bc.upper() == "CA:FALSE"
        and ku_critical and ku is not None and "Key Encipherment" in ku
        and eku_ok
        and parse_aki(aki) != ""
    )
    return ok, profile

def result(state:str,reason:str,details:dict[str,Any]|None=None)->dict[str,Any]:
    out={"verifier_id":VERIFIER_ID,"state":state,"reason":reason}
    if details:out["details"]=details
    return out

def session_binding(m:dict[str,Any], leaf_sha256:str, inter_sha256:str, root_sha256:str, crl_sha256:str)->str:
    return canonical_hash({
        "session_id":m["session_id"],
        "tpm_identity_digest":m["tpm_identity_digest"],
        "leaf_certificate_sha256":leaf_sha256,
        "intermediate_certificate_sha256":inter_sha256,
        "trust_anchor_root_sha256":root_sha256,
        "revocation_crl_sha256":crl_sha256,
    })

def verify(m:dict[str,Any])->dict[str,Any]:
    required={"profile_id","profile_version","verification_mode","claim_ceiling","session_id","tpm_identity_digest","leaf_certificate_der_base64","leaf_certificate_sha256","intermediate_certificate_der_base64","intermediate_certificate_sha256","trust_anchor_root_der_base64","trust_anchor_root_sha256","trust_anchor_state","trust_anchor_source_sha256","verification_time_unix","revocation","spki_binding","session_binding_sha256"}
    missing=sorted(required-set(m))
    if missing:return result("DENY","missing-required-fields",{"fields":missing})
    if m["profile_id"]!="mycelix.security.tpm.ek-cert-chain-policy":return result("DENY","profile-id-mismatch")
    if m["profile_version"]!="0.1.0":return result("DENY","profile-version-mismatch")
    if m["claim_ceiling"]!="ReferenceModelOnly":return result("DENY","claim-ceiling-mismatch")
    if m["verification_mode"] not in {"ReferenceModelOnly","OfflineBundle","LiveVerifierSession"}:return result("DENY","verification-mode-invalid")
    if not valid_hash(m["tpm_identity_digest"]):return result("DENY","tpm-identity-digest-invalid")
    try:
        leaf=decode_b64(m["leaf_certificate_der_base64"],"leaf_certificate_der_base64")
        inter=decode_b64(m["intermediate_certificate_der_base64"],"intermediate_certificate_der_base64")
        root=decode_b64(m["trust_anchor_root_der_base64"],"trust_anchor_root_der_base64")
        rev=m["revocation"]
        if not isinstance(rev,dict):return result("DENY","revocation-object-invalid")
        crl=decode_b64(rev.get("crl_der_base64",""),"revocation.crl_der_base64")
    except ValueError as exc:return result("DENY","certificate-input-invalid",{"error":str(exc)})
    for raw,field in ((leaf,"leaf_certificate_sha256"),(inter,"intermediate_certificate_sha256"),(root,"trust_anchor_root_sha256")):
        if not valid_hash(m[field]):return result("DENY","digest-invalid",{"field":field})
        if hashlib.sha256(raw).hexdigest()!=m[field]:return result("DENY","digest-mismatch",{"field":field})
    if m["trust_anchor_state"]=="DENY":return result("DENY","trust-anchor-denied")
    if m["trust_anchor_state"]=="INDETERMINATE":return result("INDETERMINATE","trust-anchor-indeterminate")
    if m["trust_anchor_root_sha256"]!=REFERENCE_ROOT_SHA256 or m["trust_anchor_source_sha256"]!=REFERENCE_ROOT_SOURCE_SHA256:
        return result("DENY","reference-trust-anchor-not-approved")
    if not isinstance(m["verification_time_unix"],int) or m["verification_time_unix"]<0:return result("DENY","verification-time-invalid")
    if rev.get("state")=="DENY":return result("DENY","ek-certificate-revoked")
    if rev.get("state")=="INDETERMINATE":return result("INDETERMINATE","ek-certificate-revocation-indeterminate")
    if rev.get("state")!="PASS":return result("DENY","revocation-state-invalid")
    if not valid_hash(rev.get("crl_der_sha256")) or hashlib.sha256(crl).hexdigest()!=rev["crl_der_sha256"]:
        return result("DENY","revocation-crl-digest-invalid")
    spki=m["spki_binding"]
    if not isinstance(spki,dict):return result("DENY","spki-binding-invalid")
    if spki.get("verifier_id")!=SPKI_VERIFIER_ID:return result("DENY","spki-verifier-id-mismatch")
    if spki.get("state")=="INDETERMINATE":return result("INDETERMINATE","spki-binding-indeterminate")
    if spki.get("state")!="PASS":return result("DENY","spki-binding-not-pass")
    if spki.get("certificate_sha256")!=m["leaf_certificate_sha256"]:return result("DENY","spki-certificate-digest-mismatch")
    expected_binding=session_binding(m,m["leaf_certificate_sha256"],m["intermediate_certificate_sha256"],m["trust_anchor_root_sha256"],rev["crl_der_sha256"])
    if m["session_binding_sha256"]!=expected_binding:return result("DENY","session-binding-mismatch")
    with tempfile.TemporaryDirectory(prefix="mycelix-ek-chain-") as td:
        work=Path(td)
        try:
            ok,chain_detail=verify_chain(leaf,inter,root,crl,m["verification_time_unix"],work)
            leaf_text=x509_text(leaf,work,"leaf")
            inter_text=x509_text(inter,work,"inter")
            serial=int(x509_scalar(leaf,work,"-serial"),16)
            subject=x509_scalar(leaf,work,"-subject")
            issuer=x509_scalar(leaf,work,"-issuer")
        except (ValueError,TypeError) as exc:
            return result("DENY","openssl-parse-error",{"error":str(exc)})
    if not ok:return result("DENY","certificate-path-validation-failed",{"openssl":chain_detail})
    profile_ok,profile=profile_ok(leaf_text)
    profile["serial_positive"]=serial>0
    profile["subject"]=subject
    profile["issuer"]=issuer
    if not serial>0:return result("DENY","leaf-serial-invalid",profile)
    if not profile_ok:return result("DENY","ek-leaf-profile-requirements-failed",profile)
    if not verify_aki_ski(leaf_text,inter_text):return result("DENY","authority-key-identifier-does-not-match-intermediate-ski",profile)
    if m["verification_mode"]!="ReferenceModelOnly":return result("INDETERMINATE","live-origin-not-authorized-by-reference-model",profile)
    return result("PASS","ek-certificate-chain-and-profile-policy-verified",{**profile,"trust_anchor_sha256":m["trust_anchor_root_sha256"],"verification_time_unix":m["verification_time_unix"],"revocation_state":rev["state"],"spki_certificate_sha256":spki["certificate_sha256"]})

def verify_aki_ski(leaf_text:str,inter_text:str)->bool:
    leaf_aki=parse_aki(extension_value(leaf_text,"Authority Key Identifier"))
    inter_ski=parse_aki(extension_value(inter_text,"Subject Key Identifier"))
    return bool(leaf_aki and inter_ski and leaf_aki==inter_ski)

def fixture()->dict[str,Any]:
    leaf=FIXTURE_LEAF_DER;inter=FIXTURE_INTERMEDIATE_DER;root=FIXTURE_ROOT_DER;crl=FIXTURE_CRL_DER
    ld=hashlib.sha256(leaf).hexdigest();id=hashlib.sha256(inter).hexdigest();rd=hashlib.sha256(root).hexdigest();cd=hashlib.sha256(crl).hexdigest()
    session="ek-chain-self-test";tpm="44"*32
    return {
      "profile_id":"mycelix.security.tpm.ek-cert-chain-policy","profile_version":"0.1.0","verification_mode":"ReferenceModelOnly","claim_ceiling":"ReferenceModelOnly","session_id":session,"tpm_identity_digest":tpm,
      "leaf_certificate_der_base64":base64.b64encode(leaf).decode(),"leaf_certificate_sha256":ld,
      "intermediate_certificate_der_base64":base64.b64encode(inter).decode(),"intermediate_certificate_sha256":id,
      "trust_anchor_root_der_base64":base64.b64encode(root).decode(),"trust_anchor_root_sha256":rd,"trust_anchor_state":"PASS","trust_anchor_source_sha256":REFERENCE_ROOT_SOURCE_SHA256,
      "verification_time_unix":REFERENCE_TIME_UNIX,
      "revocation":{"state":"PASS","method":"issuer-crl","crl_der_base64":base64.b64encode(crl).decode(),"crl_der_sha256":cd},
      "spki_binding":{"state":"PASS","verifier_id":SPKI_VERIFIER_ID,"certificate_sha256":ld,"ek_public_wire_sha256":"55"*32},
      "session_binding_sha256":canonical_hash({"session_id":session,"tpm_identity_digest":tpm,"leaf_certificate_sha256":ld,"intermediate_certificate_sha256":id,"trust_anchor_root_sha256":rd,"revocation_crl_sha256":cd})
    }

def mutate_certificate_sha256(v:dict[str,Any],bad:bytes)->None:
    v["leaf_certificate_der_base64"]=base64.b64encode(bad).decode()
    v["leaf_certificate_sha256"]=hashlib.sha256(bad).hexdigest()
    v["spki_binding"]["certificate_sha256"]=v["leaf_certificate_sha256"]
    v["session_binding_sha256"]=session_binding(
        v,
        v["leaf_certificate_sha256"],
        v["intermediate_certificate_sha256"],
        v["trust_anchor_root_sha256"],
        v["revocation"]["crl_der_sha256"],
    )

def mutate_root(v:dict[str,Any])->None:
    v["trust_anchor_root_der_base64"]=v["intermediate_certificate_der_base64"]
    v["trust_anchor_root_sha256"]=v["intermediate_certificate_sha256"]
    v["session_binding_sha256"]=session_binding(
        v,
        v["leaf_certificate_sha256"],
        v["intermediate_certificate_sha256"],
        v["trust_anchor_root_sha256"],
        v["revocation"]["crl_der_sha256"],
    )

def self_test()->int:
    base=fixture()
    cases=[
      ("canonical-valid","PASS",lambda x:x),
      ("root-substitution","DENY",mutate_root),
      ("intermediate-substitution","DENY",lambda x:x.update({"intermediate_certificate_der_base64":x["trust_anchor_root_der_base64"]})),
      ("leaf-byte-substitution","DENY",lambda x:mutate_certificate_sha256(x,bytes(FIXTURE_LEAF_DER[:-1])+b"\\x00")),
      ("expired-reference-time","DENY",lambda x:x.update({"verification_time_unix":EXPIRED_TIME_UNIX})),
      ("not-yet-valid-reference-time","DENY",lambda x:x.update({"verification_time_unix":NOT_YET_VALID_TIME_UNIX})),
      ("key-usage-profile-mismatch","DENY",lambda x:mutate_certificate_sha256(x,FIXTURE_BAD_USAGE_DER)),
      ("eku-profile-mismatch","DENY",lambda x:mutate_certificate_sha256(x,FIXTURE_BAD_EKU_DER)),
      ("trust-anchor-substitution","DENY",lambda x:x.update({"trust_anchor_source_sha256":"66"*32})),
      ("revocation-deny","DENY",lambda x:x["revocation"].update({"state":"DENY"})),
      ("revocation-indeterminate","INDETERMINATE",lambda x:x["revocation"].update({"state":"INDETERMINATE"})),
      ("spki-certificate-substitution","DENY",lambda x:x["spki_binding"].update({"certificate_sha256":"77"*32})),
      ("spki-indeterminate","INDETERMINATE",lambda x:x["spki_binding"].update({"state":"INDETERMINATE"})),
      ("session-binding-substitution","DENY",lambda x:x.update({"session_id":"attacker"})),
    ]
    for name,expected,mut in cases:
      c=copy.deepcopy(base);mut(c);o=verify(c)
      if o["state"]!=expected:
        print(f"{name}: FAIL expected={expected} got={o['state']} reason={o['reason']}");return 1
    p=json.loads(json.dumps(base,sort_keys=True))
    if verify(p)["state"]!="PASS":
      print("key-order-permutation: FAIL");return 1
    print("EK certificate chain policy semantic corpus: PASS")
    print("14 adversarial mutations plus canonical and key-order control: PASS")
    return 0

def main()->int:
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test",action="store_true")
    group.add_argument("--verify",metavar="MANIFEST")
    parser.add_argument("--output")
    args=parser.parse_args()
    if args.self_test:return self_test()
    path=Path(args.verify).resolve();m=json.loads(path.read_text(encoding="utf-8"));v=verify(m)
    out={"profile_id":"mycelix.security.tpm.ek-cert-chain-policy","profile_version":"0.1.0","verifier_id":VERIFIER_ID,"input_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),**v}
    out["content_sha256"]=canonical_hash({k:v for k,v in out.items() if k!="content_sha256"})
    rendered=json.dumps(out,indent=2,sort_keys=True)+"\\n"
    if args.output:Path(args.output).write_text(rendered,encoding="utf-8")
    else:print(rendered,end="")
    return {"PASS":0,"DENY":1,"INDETERMINATE":2}[v["state"]]

if __name__=="__main__":raise SystemExit(main())
