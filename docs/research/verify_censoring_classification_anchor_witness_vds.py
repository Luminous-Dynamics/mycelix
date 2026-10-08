#!/usr/bin/env python3
"""Research-only RFC 9162 Merkle VDS verifier."""
from __future__ import annotations
import copy,hashlib,json,sys
from pathlib import Path

CAMPAIGN="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-campaign.v1"
FIXTURE="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-fixture.v1"

def canonical(v): return json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(",",":")).encode("utf-8")
def H(b): return hashlib.sha256(b).digest()
def leaf(entry): return H(b"\x00"+canonical(entry))
def node(a,b): return H(b"\x01"+a+b)
def mth(ds):
    n=len(ds)
    if n==0:return H(b"")
    if n==1:return leaf(ds[0])
    k=1<<((n-1).bit_length()-1)
    return node(mth(ds[:k]),mth(ds[k:]))
def kpow(n): return 1<<((n-1).bit_length()-1)
def subproof(m,ds,known):
    n=len(ds)
    if m==n:return [] if known else [mth(ds)]
    k=kpow(n)
    if m<=k:return subproof(m,ds[:k],known)+[mth(ds[k:])]
    return subproof(m-k,ds[k:],False)+[mth(ds[:k])]
def verify_consistency(m,n,first_hash,second_hash,path):
    if not path or not (0<m<n): return False
    p=list(path)
    if m&(m-1)==0:p=[first_hash]+p
    fn,sn=m-1,n-1
    while fn&1: fn>>=1; sn>>=1
    fr=sr=p[0]
    for c in p[1:]:
        if sn==0:return False
        if (fn&1) or fn==sn:
            fr=node(c,fr); sr=node(c,sr)
            if not (fn&1):
                while fn and not (fn&1):fn>>=1;sn>>=1
        else: sr=node(sr,c)
        fn>>=1;sn>>=1
    return sn==0 and fr==first_hash and sr==second_hash
def hx(v):
    if not isinstance(v,str) or not v.startswith("sha256:") or len(v)!=71:raise ValueError("hash")
    return bytes.fromhex(v[7:])
def head(fixture,name,override=None):
    h=copy.deepcopy(fixture["heads"][name])
    if override is not None:h["root_hash"]=override
    return h
def evaluate(fixture,c):
    entries=fixture["entries"]
    first=head(fixture,c["first_head"],c.get("first_root_override"))
    if first["tree_size"]>len(entries):return "unresolved","tree-size-out-of-range"
    fr=hx(first["root_hash"])
    if mth(entries[:first["tree_size"]])!=fr:return "unresolved","first-head-root-mismatch"
    if "second_head" not in c:return "qualified","root-reconstructed"
    second=head(fixture,c["second_head"],c.get("second_root_override"))
    if second["tree_size"]>len(entries):return "unresolved","tree-size-out-of-range"
    sr=hx(second["root_hash"])
    if mth(entries[:second["tree_size"]])!=sr:return "unresolved","second-head-root-mismatch"
    if second["tree_size"]<first["tree_size"]:return "unresolved","rollback"
    if second["tree_size"]==first["tree_size"]:
        return ("qualified","same-head") if fr==sr else ("unresolved","equivocation")
    if c.get("proof")!="fixture":return "unresolved","missing-proof"
    if first["tree_size"]!=4 or second["tree_size"]!=7:return "unresolved","unsupported-fixture-pair"
    p=[hx(x) for x in fixture["consistency_proof_4_to_7"]]
    m=c.get("proof_mutation")
    if m=="replace-first":p[0]=H(p[0])
    elif m=="empty":p=[]
    elif m=="append-extra":p.append(H(b"extra"))
    elif m=="truncate":p=p[:-1]
    return ("qualified","consistency-proof") if verify_consistency(first["tree_size"],second["tree_size"],fr,sr,p) else ("unresolved","consistency-proof-invalid")
def main():
    if len(sys.argv)!=4:return 2
    fixture=json.loads(Path(sys.argv[1]).read_text());campaign=json.loads(Path(sys.argv[2]).read_text());out=Path(sys.argv[3])
    if fixture.get("schema")!=FIXTURE or campaign.get("schema")!=CAMPAIGN or campaign.get("case_count")!=15 or len(campaign.get("cases",[]))!=15:return 1
    ids=[c.get("case_id") for c in campaign["cases"]]
    if len(ids)!=len(set(ids)):return 1
    rows=[];failures=[]
    for c in campaign["cases"]:
        actual,reason=evaluate(fixture,c)
        row={"case_id":c["case_id"],"expected_verdict":c["expected_verdict"],"actual_verdict":actual,"reason":reason};rows.append(row)
        if actual!=c["expected_verdict"]:failures.append([c["case_id"],c["expected_verdict"],actual,reason])
    out.write_bytes(canonical({"schema":"mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-report.v1","status":"research-evidence-only","case_count":15,"cases":rows,"failures":failures})+b"\n")
    print(f"cases={len(rows)} failures={len(failures)}");return 1 if failures else 0
if __name__=="__main__":raise SystemExit(main())
