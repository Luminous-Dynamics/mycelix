#!/usr/bin/env python3
"""RFC 9942 / RFC 9162 COSE receipt verifier, research fixture profile."""
from __future__ import annotations
import base64, copy, hashlib, json, sys
from pathlib import Path
from verify_censoring_classification_anchor_witness_crypto import canonical, ed25519_verify

TS_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-ts-registry.v1"
FIXTURE_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-fixture.v1"
CAMPAIGN_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-campaign.v1"
HEAD_SCHEMA = "mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1"
WITNESS_REGISTRY_ID = "mycelix.research.anchor-witness-registry.v2"
VDS_ID = "mycelix.research.anchor-statement-sequence.v1"
TS_ID = "mycelix.research.anchor-cose-receipt-ts-registry.v1"
TS_KEY_ID = "cose-test-ts-k1"
EXPECTED_TS_REGISTRY_SHA = "sha256:21772a0a1dbb88358c80e5297d4ec4dd82c2ba2bca474a334bae45535ec8b331"
EXPECTED_TRUST_ROOT_SHA = "sha256:99ec416914eb75a7c953fbc74045d25fc83cb894c6e7a5bcd899c757a736b9d7"
COSE_ALG_EDDSA = -8
VDS_RFC9162_SHA256 = 1

def cbor_head(major, n):
    if n < 24: return bytes([(major << 5) | n])
    if n < 256: return bytes([(major << 5) | 24, n])
    if n < 65536: return bytes([(major << 5) | 25]) + n.to_bytes(2, "big")
    if n < 2**32: return bytes([(major << 5) | 26]) + n.to_bytes(4, "big")
    if n < 2**64: return bytes([(major << 5) | 27]) + n.to_bytes(8, "big")
    raise ValueError("integer-range")

def cbor_encode(v):
    if v is None: return b"\xf6"
    if v is False: return b"\xf4"
    if v is True: return b"\xf5"
    if isinstance(v, int): return cbor_head(0, v) if v >= 0 else cbor_head(1, -1-v)
    if isinstance(v, bytes): return cbor_head(2, len(v)) + v
    if isinstance(v, str):
        b = v.encode("utf-8")
        return cbor_head(3, len(b)) + b
    if isinstance(v, list): return cbor_head(4, len(v)) + b"".join(cbor_encode(x) for x in v)
    if isinstance(v, dict):
        pairs = [(cbor_encode(k), cbor_encode(val)) for k, val in v.items()]
        pairs.sort(key=lambda p: p[0])
        return cbor_head(5, len(pairs)) + b"".join(k+val for k, val in pairs)
    raise ValueError("unsupported-cbor-type")

def cbor_map_in_given_order(pairs):
    return cbor_head(5, len(pairs)) + b"".join(cbor_encode(k)+cbor_encode(v) for k,v in pairs)

def cbor_decode(data, pos=0):
    if pos >= len(data): raise ValueError("cbor-truncated")
    initial = data[pos]; pos += 1
    major, ai = initial >> 5, initial & 31
    if major == 7:
        if ai == 20: return False, pos
        if ai == 21: return True, pos
        if ai == 22: return None, pos
        raise ValueError("cbor-simple-or-float")
    if ai == 31: raise ValueError("cbor-indefinite-length")
    if ai < 24: n = ai
    elif ai == 24:
        if pos+1 > len(data): raise ValueError("cbor-truncated")
        n = data[pos]; pos += 1
    elif ai == 25:
        if pos+2 > len(data): raise ValueError("cbor-truncated")
        n = int.from_bytes(data[pos:pos+2], "big"); pos += 2
    elif ai == 26:
        if pos+4 > len(data): raise ValueError("cbor-truncated")
        n = int.from_bytes(data[pos:pos+4], "big"); pos += 4
    elif ai == 27:
        if pos+8 > len(data): raise ValueError("cbor-truncated")
        n = int.from_bytes(data[pos:pos+8], "big"); pos += 8
    else: raise ValueError("cbor-reserved-additional-info")
    if major == 0: return n, pos
    if major == 1: return -1-n, pos
    if major in (2,3):
        if pos+n > len(data): raise ValueError("cbor-truncated")
        raw = data[pos:pos+n]; pos += n
        return (raw if major == 2 else raw.decode("utf-8")), pos
    if major == 4:
        out=[]
        for _ in range(n):
            x,pos=cbor_decode(data,pos); out.append(x)
        return out,pos
    if major == 5:
        out={}
        for _ in range(n):
            k,pos=cbor_decode(data,pos); v,pos=cbor_decode(data,pos)
            if not isinstance(k,(int,str,bytes,bool)) or k in out: raise ValueError("cbor-invalid-or-duplicate-map-key")
            out[k]=v
        return out,pos
    raise ValueError("cbor-unsupported-major-type")

def decode_exact(data):
    value,pos=cbor_decode(data)
    if pos != len(data): raise ValueError("cbor-trailing-bytes")
    return value

def b64u(s):
    if not isinstance(s,str): raise ValueError("base64url-type")
    if any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_" for c in s): raise ValueError("base64url-alphabet")
    raw=base64.urlsafe_b64decode(s + "="*((4-len(s)%4)%4))
    return raw

def digest_obj(v): return "sha256:" + hashlib.sha256(canonical(v)).hexdigest()
def sha256(b): return hashlib.sha256(b).digest()
def node_hash(a,b): return sha256(b"\x01"+a+b)
def leaf_hash(entry_bytes): return sha256(b"\x00"+entry_bytes)
def mth(entries):
    n=len(entries)
    if n==0: return sha256(b"")
    if n==1: return leaf_hash(canonical(entries[0]))
    k=1<<((n-1).bit_length()-1)
    return node_hash(mth(entries[:k]),mth(entries[k:]))

def inclusion_root(index, size, entry_bytes, path):
    if not isinstance(size,int) or not isinstance(index,int) or size <= 0 or index < 0 or index >= size: return None
    if any(not isinstance(p,bytes) or len(p)!=32 for p in path): return None
    fn,sn=index,size-1
    r=leaf_hash(entry_bytes)
    for p in path:
        if sn==0: return None
        if (fn & 1) or fn==sn:
            r=node_hash(p,r)
            if not (fn & 1):
                while fn and not (fn & 1):
                    fn >>= 1; sn >>= 1
        else:
            r=node_hash(r,p)
        fn >>= 1; sn >>= 1
    return r if sn==0 else None

def consistency_valid(old_size,new_size,old_root,new_root,path):
    if not (isinstance(old_size,int) and isinstance(new_size,int) and 0 < old_size < new_size): return False
    if any(not isinstance(p,bytes) or len(p)!=32 for p in path) or not path: return False
    fn,sn=old_size-1,new_size-1
    while fn & 1:
        fn >>= 1; sn >>= 1
    if fn==0:
        fr=sr=old_root
        siblings=path
    else:
        fr=sr=path[0]
        siblings=path[1:]
    for p in siblings:
        if sn==0: return False
        if (fn & 1) or fn==sn:
            fr=node_hash(p,fr); sr=node_hash(p,sr)
            if not (fn & 1):
                while fn and not (fn & 1):
                    fn >>= 1; sn >>= 1
        else:
            sr=node_hash(sr,p)
        fn >>= 1; sn >>= 1
    return sn==0 and fr==old_root and sr==new_root

def witness_key(reg,w,kid,version):
    wi=reg.get("witnesses",{}).get(w)
    key=wi and wi.get("keys",{}).get(kid)
    if not key: return None,"unknown-witness-or-key"
    if key.get("algorithm")!="Ed25519": return None,"witness-key-algorithm"
    if version<key.get("valid_from_version",10**9): return None,"witness-key-not-yet-valid"
    if key.get("valid_until_version") is not None and version>key["valid_until_version"]: return None,"witness-key-expired"
    if key.get("revoked_at_version") is not None and version>=key["revoked_at_version"]: return None,"witness-key-revoked"
    if key.get("status")=="revoked": return None,"witness-key-revoked"
    try: return b64u(key["public_key"]),None
    except Exception: return None,"witness-key-encoding"

def verify_tree_head(att,w,registry,entries):
    fields={"schema","domain","algorithm","observer_id","key_id","registry_id","registry_version","vds_id","manifest_version","tree_size","root_hash","signature"}
    if not isinstance(att,dict) or set(att)!=fields: return None,"head-schema"
    if att["schema"]!=HEAD_SCHEMA or att["domain"]!=HEAD_SCHEMA: return None,"head-domain"
    if att["algorithm"]!="Ed25519": return None,"head-algorithm"
    if att["observer_id"]!=w: return None,"head-observer-binding"
    if att["registry_id"]!=WITNESS_REGISTRY_ID or att["registry_version"]!=2: return None,"head-registry-binding"
    if att["vds_id"]!=VDS_ID: return None,"head-vds-binding"
    n=att["tree_size"]
    if not isinstance(n,int) or n<1 or n>len(entries): return None,"head-tree-size"
    calculated=mth(entries[:n])
    try: claimed=bytes.fromhex(att["root_hash"][7:])
    except Exception: return None,"head-root-encoding"
    if not att["root_hash"].startswith("sha256:") or claimed!=calculated: return None,"head-root-mismatch"
    pub,e=witness_key(registry,w,att["key_id"],att["manifest_version"])
    if e: return None,"head-"+e
    payload={"schema":HEAD_SCHEMA,"domain":HEAD_SCHEMA,"algorithm":"Ed25519","observer_id":w,"key_id":att["key_id"],"witness_identity_commitment":registry["witnesses"][w]["identity_commitment"],"registry_id":att["registry_id"],"registry_version":att["registry_version"],"vds_id":att["vds_id"],"manifest_version":att["manifest_version"],"tree_size":n,"root_hash":att["root_hash"]}
    try: sig=b64u(att["signature"])
    except Exception: return None,"head-signature-encoding"
    if not ed25519_verify(pub,sig,canonical(payload)): return None,"head-signature-invalid"
    return att,None

def validate_head_quorum(head_fixture, wreg, vds, size_name):
    if wreg.get("registry_id")!=WITNESS_REGISTRY_ID or wreg.get("registry_version")!=2: return None,"witness-registry-binding"
    heads=head_fixture.get("heads",{}).get(size_name,{}).get("attestations",{})
    if len(heads)<3: return None,"head-below-threshold"
    vals=[]
    for w,att in heads.items():
        value,e=verify_tree_head(att,w,wreg,vds["entries"])
        if e: return None,e
        vals.append(value)
    claims={(h["manifest_version"],h["tree_size"],h["root_hash"]) for h in vals}
    if len(claims)!=1: return None,"head-equivocation"
    size=vals[0]["tree_size"]
    root=bytes.fromhex(vals[0]["root_hash"][7:])
    return {"tree_size":size,"root":root,"claims":vals},None

def valid_cose_envelope(wire):
    if not wire or wire[0]!=0xd2: return None,"cose-tag"
    obj=decode_exact(wire[1:])
    if cbor_encode(obj)!=wire[1:]: return None,"cose-noncanonical-envelope"
    if not isinstance(obj,list) or len(obj)!=4: return None,"cose-sign1-structure"
    protected_bytes,unprotected,payload,signature=obj
    if not isinstance(protected_bytes,bytes) or not isinstance(unprotected,dict) or not isinstance(signature,bytes) or len(signature)!=64: return None,"cose-field-type"
    protected=decode_exact(protected_bytes)
    if cbor_encode(protected)!=protected_bytes: return None,"cose-noncanonical-protected"
    if not isinstance(protected,dict): return None,"cose-protected-map"
    return {"object":obj,"protected_bytes":protected_bytes,"protected":protected,"unprotected":unprotected,"payload":payload,"signature":signature},None

def mutate_wire(fixture,kind,mutation):
    wire=bytes.fromhex(fixture["inclusion_receipt_cose_hex" if kind=="inclusion" else "consistency_receipt_cose_hex"])
    if mutation=="tag-removal": return wire[1:]
    parsed,err=valid_cose_envelope(wire)
    if err: return wire
    obj=parsed["object"]
    if mutation=="signature-bitflip":
        s=bytearray(obj[3]);s[-1]^=1;obj[3]=bytes(s)
    elif mutation=="attached-payload":
        obj[2]=bytes.fromhex(fixture["root_hash"][7:])
    elif mutation=="extra-unprotected-label":
        obj[1][123]=True
    elif mutation=="algorithm-substitution":
        p=decode_exact(obj[0]);p[1]=-7;obj[0]=cbor_encode(p)
    elif mutation=="vds-substitution":
        p=decode_exact(obj[0]);p[395]=999;obj[0]=cbor_encode(p)
    elif mutation=="key-id-substitution":
        p=decode_exact(obj[0]);p[4]=b"attacker-k1";obj[0]=cbor_encode(p)
    elif mutation=="noncanonical-protected-map":
        obj[0]=cbor_map_in_given_order([(395,1),(4,b"cose-test-ts-k1"),(1,-8)])
    elif mutation=="wrong-proof-label":
        vdp=obj[1][396]; old=vdp.get(-2, vdp.get(-1)); label=-1 if -2 in vdp else -2
        obj[1][396]={label:old}
    elif mutation in {"proof-path","proof-index-out-of-range","proof-tree-size","empty-inclusion-path","old-size-substitution","new-size-substitution"}:
        vdp=obj[1][396]; label=-1 if kind=="inclusion" else -2
        proof_blob=vdp[label][0]; proof=decode_exact(proof_blob)
        if mutation=="proof-path":
            path=proof[2]; p=bytearray(path[0]);p[0]^=1;path[0]=bytes(p)
        elif mutation=="proof-index-out-of-range": proof[1]=proof[0]
        elif mutation=="proof-tree-size": proof[0]-=1
        elif mutation=="empty-inclusion-path": proof[2]=[]
        elif mutation=="old-size-substitution": proof[0]-=1
        elif mutation=="new-size-substitution": proof[1]-=1
        vdp[label][0]=cbor_encode(proof)
    elif mutation=="drop-sign1-field":
        obj.pop()
    return b"\xd2"+cbor_encode(obj)

def verify_receipt(kind,fixture,tsreg,wreg,vds,heads,trust_root,wire):
    if tsreg.get("schema")!=TS_SCHEMA or tsreg.get("registry_id")!=TS_ID or tsreg.get("registry_version")!=1: return "registry-schema"
    if digest_obj(tsreg)!=EXPECTED_TS_REGISTRY_SHA: return "registry-pin"
    if fixture.get("ts_registry_sha256")!=EXPECTED_TS_REGISTRY_SHA or fixture.get("ts_registry_id")!=TS_ID: return "fixture-registry-binding"
    if digest_obj(wreg)!=trust_root.get("registry_sha256") or digest_obj(trust_root)!=EXPECTED_TRUST_ROOT_SHA: return "witness-trust-root-pin"
    if fixture.get("vds_id")!=VDS_ID or vds.get("vds_id")!=VDS_ID: return "vds-binding"
    n=fixture["tree_size"]
    if mth(vds["entries"][:n]).hex()!=fixture["root_hash"][7:]: return "fixture-root-binding"
    parsed,err=valid_cose_envelope(wire)
    if err: return err
    p=parsed["protected"]; unp=parsed["unprotected"]
    if set(p)!={1,4,395}: return "protected-header-profile"
    if p[1]!=COSE_ALG_EDDSA: return "algorithm-substitution"
    if p[395]!=VDS_RFC9162_SHA256: return "vds-substitution"
    if not isinstance(p[4],bytes): return "key-id-type"
    try: kid=p[4].decode("utf-8")
    except Exception: return "key-id-encoding"
    key=tsreg.get("keys",{}).get(kid)
    if not key or key.get("status")!="active" or key.get("algorithm")!=COSE_ALG_EDDSA: return "unknown-or-inactive-ts-key"
    if parsed["payload"] is not None: return "detached-payload-required"
    if set(unp)!={396} or not isinstance(unp[396],dict): return "unprotected-header-profile"
    label=-1 if kind=="inclusion" else -2
    if set(unp[396])!={label} or not isinstance(unp[396][label],list) or len(unp[396][label])!=1: return "proof-label-or-count"
    proof_blob=unp[396][label][0]
    if not isinstance(proof_blob,bytes): return "proof-encoding"
    proof=decode_exact(proof_blob)
    if cbor_encode(proof)!=proof_blob: return "proof-noncanonical"
    if not isinstance(proof,list) or len(proof)!=3 or not isinstance(proof[2],list): return "proof-shape"
    sig=parsed["signature"]
    try: pub=b64u(key["public_key"])
    except Exception: return "ts-public-key"
    if len(pub)!=32: return "ts-public-key"
    if kind=="inclusion":
        if proof[0]!=n or proof[1]!=fixture["leaf_index"]: return "inclusion-proof-context"
        candidate=fixture.get("candidate_entry")
        candidate_bytes=b64u(fixture.get("candidate_entry_base64url",""))
        if canonical(candidate)!=candidate_bytes: return "candidate-entry-bytes-mismatch"
        if candidate!=vds["entries"][proof[1]]: return "candidate-entry-binding"
        root=inclusion_root(proof[1],proof[0],candidate_bytes,proof[2])
        if root is None or root.hex()!=fixture["root_hash"][7:]: return "inclusion-proof-invalid"
        quorum,e=validate_head_quorum(heads,wreg,vds,"size_7")
        if e: return e
        if quorum["tree_size"]!=proof[0] or quorum["root"]!=root: return "head-quorum-binding"
        sig_structure=cbor_encode(["Signature1",parsed["protected_bytes"],b"",root])
        if not ed25519_verify(pub,sig,sig_structure): return "cose-signature-invalid"
        return "receipt-inclusion-valid"
    quorum7,e=validate_head_quorum(heads,wreg,vds,"size_7")
    if e: return e
    quorum4,e=validate_head_quorum(heads,wreg,vds,"size_4")
    if e: return e
    newer_root=quorum7["root"]
    sig_structure=cbor_encode(["Signature1",parsed["protected_bytes"],b"",newer_root])
    if not ed25519_verify(pub,sig,sig_structure): return "cose-signature-invalid"
    if proof[0]!=quorum4["tree_size"] or proof[1]!=quorum7["tree_size"]: return "consistency-proof-context"
    if fixture.get("consistency_old_tree_size")!=quorum4["tree_size"] or bytes.fromhex(fixture["consistency_old_root_hash"][7:])!=quorum4["root"]: return "old-head-binding"
    if not consistency_valid(proof[0],proof[1],quorum4["root"],newer_root,proof[2]): return "consistency-proof-invalid"
    return "receipt-consistency-valid"

def main():
    if len(sys.argv)!=9:
        print("usage: verifier TS_REGISTRY COSE_FIXTURE CAMPAIGN WITNESS_REGISTRY TRUST_ROOT VDS_FIXTURE TREE_HEAD_FIXTURE REPORT",file=sys.stderr)
        return 2
    tsp,fp,cp,wrp,trp,vdsp,hp,outp=sys.argv[1:]
    tsreg=json.loads(Path(tsp).read_text()); fixture=json.loads(Path(fp).read_text()); campaign=json.loads(Path(cp).read_text())
    wreg=json.loads(Path(wrp).read_text()); trust_root=json.loads(Path(trp).read_text()); vds=json.loads(Path(vdsp).read_text()); heads=json.loads(Path(hp).read_text())
    if fixture.get("schema")!=FIXTURE_SCHEMA or campaign.get("schema")!=CAMPAIGN_SCHEMA or campaign.get("case_count")!=22 or len(campaign.get("cases",[]))!=22: return 1
    ids=[x.get("case_id") for x in campaign["cases"]]
    if len(ids)!=len(set(ids)): return 1
    # Baseline preflight: both COSE receipts and both authenticated tree-head quorums must verify.
    for kind in ("inclusion","consistency"):
        result=verify_receipt(kind,fixture,tsreg,wreg,vds,heads,trust_root,bytes.fromhex(fixture[kind+"_receipt_cose_hex"]))
        if result not in {"receipt-inclusion-valid","receipt-consistency-valid"}:
            print(f"baseline-{kind}={result}",file=sys.stderr); return 1
    rows=[];failures=[]
    for case in campaign["cases"]:
        f=copy.deepcopy(fixture); reg=copy.deepcopy(tsreg)
        mutation=case.get("mutation")
        if mutation=="fixture-root-substitution": f["root_hash"]="sha256:"+"a"*64
        if mutation=="registry-key-substitution": reg["keys"][TS_KEY_ID]["public_key"]="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        wire=mutate_wire(f,case["kind"],mutation) if mutation else bytes.fromhex(f["inclusion_receipt_cose_hex" if case["kind"]=="inclusion" else "consistency_receipt_cose_hex"])
        actual=verify_receipt(case["kind"],f,reg,wreg,vds,heads,trust_root,wire)
        verdict="qualified" if actual in {"receipt-inclusion-valid","receipt-consistency-valid"} else "unresolved"
        row={"case_id":case["case_id"],"expected_verdict":case["expected_verdict"],"actual_verdict":verdict,"reason":actual}
        rows.append(row)
        if verdict!=case["expected_verdict"]: failures.append([case["case_id"],case["expected_verdict"],verdict,actual])
    report={"schema":"mycelix.continual-adaptation.censoring-classification-anchor-cose-receipt-report.v1","status":"research-evidence-only","case_count":len(rows),"cases":rows,"failures":failures}
    Path(outp).write_bytes(canonical(report)+b"\n")
    print(f"cases={len(rows)} failures={len(failures)}")
    return 1 if failures else 0

if __name__=="__main__": raise SystemExit(main())
