#!/usr/bin/env python3
"""Research-only dual-sided witness key-rotation ceremony verifier."""
from __future__ import annotations
import base64,copy,hashlib,json,sys
from pathlib import Path

ROT_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.v1"
CER_SCHEMA="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation-ceremony.v1"
ALG="Ed25519"; REG_ID="mycelix.research.anchor-witness-registry.v2"; PROPOSED_ID="mycelix.research.anchor-witness-registry.v3"

Q=2**255-19
L=2**252+27742317777372353535851937790883648493
D=(-121665*pow(121666,Q-2,Q))%Q
I=pow(2,(Q-1)//4,Q)
BY=(4*pow(5,Q-2,Q))%Q
inv=lambda x:pow(x,Q-2,Q)
def xrecover(y):
    xx=(y*y-1)*inv(D*y*y+1)%Q;x=pow(xx,(Q+3)//8,Q)
    if (x*x-xx)%Q:x=x*I%Q
    if x&1:x=Q-x
    return x
B=(xrecover(BY),BY,1,xrecover(BY)*BY%Q)
def add(P,N):
    X1,Y1,Z1,T1=P;X2,Y2,Z2,T2=N
    A=(Y1-X1)*(Y2-X2)%Q;Bv=(Y1+X1)*(Y2+X2)%Q;C=2*D*T1*T2%Q;Dv=2*Z1*Z2%Q
    E=(Bv-A)%Q;F=(Dv-C)%Q;G=(Dv+C)%Q;H=(Bv+A)%Q
    return E*F%Q,G*H%Q,F*G%Q,E*H%Q
def mul(P,n):
    R=(0,1,1,0)
    while n:
        if n&1:R=add(R,P)
        P=add(P,P);n>>=1
    return R
def enc(P):
    X,Y,Z,T=P;zi=inv(Z);x=X*zi%Q;y=Y*zi%Q
    return (y|((x&1)<<255)).to_bytes(32,"little")
def dec(b):
    if len(b)!=32:raise ValueError("point-length")
    y=int.from_bytes(b,"little");s=y>>255;y&=(1<<255)-1
    if y>=Q:raise ValueError("noncanonical-y")
    x=xrecover(y)
    if x==0 and s:raise ValueError("negative-zero")
    if x&1!=s:x=Q-x
    P=(x,y,1,x*y%Q)
    if enc(mul(P,L))!=enc((0,1,1,0)):raise ValueError("small-order-point")
    return P
def ed_verify(pub,sig,msg):
    if len(pub)!=32 or len(sig)!=64:return False
    try:
        A=dec(pub);R=dec(sig[:32]);S=int.from_bytes(sig[32:],"little")
        if S>=L:return False
        k=int.from_bytes(hashlib.sha512(sig[:32]+pub+msg).digest(),"little")%L
        return enc(mul(B,S))==enc(add(R,mul(A,k)))
    except Exception:return False
def canonical(v):return json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(",",":")).encode("utf-8")
def digest(v):return "sha256:"+hashlib.sha256(canonical(v)).hexdigest()
def b64d(v,n):
    if not isinstance(v,str):raise ValueError("encoding")
    raw=base64.urlsafe_b64decode(v+"="*((4-len(v)%4)%4))
    if len(raw)!=n:raise ValueError("length")
    return raw
def payload(role,w,k,r):
    return {"schema":ROT_SCHEMA,"domain":f"mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.{role}.v1","algorithm":ALG,"role":role,"witness_id":w,"key_id":k,"rotation":r}

def evaluate(base,proposal,ceremony,c):
    reg=copy.deepcopy(base);prop=copy.deepcopy(proposal);cer=copy.deepcopy(ceremony);r=cer["ceremony"]["rotation"]
    if c.get("remove_predecessor"):cer["ceremony"].pop("predecessor_approval",None)
    if c.get("remove_successor"):cer["ceremony"].pop("successor_proof_of_possession",None)
    if c.get("remove_governance"):cer["ceremony"]["governance_approvals"]=cer["ceremony"]["governance_approvals"][:-c["remove_governance"]]
    if c.get("target") and c.get("mutate_domain"):cer["ceremony"][c["target"]]["domain"]=c["mutate_domain"]
    if c.get("target") and c.get("mutate_algorithm"):cer["ceremony"][c["target"]]["algorithm"]=c["mutate_algorithm"]
    if c.get("governance_replace"):cer["ceremony"]["governance_approvals"][c["governance_index"]]=copy.deepcopy(c["governance_replace"])
    if c.get("successor_key_mismatch"):r["successor_public_key"]=c["successor_key_mismatch"]
    if c.get("activation_version") is not None:r["activation_version"]=c["activation_version"]
    if c.get("predecessor_until") is not None:r["predecessor_valid_until_version"]=c["predecessor_until"]
    if c.get("successor_from") is not None:r["successor_valid_from_version"]=c["successor_from"]
    if c.get("proposal_digest"):r["proposed_registry_sha256"]=c["proposal_digest"]
    if c.get("proposal_mutation"):prop["witnesses"]["w01"]["keys"]["w01-k2"][c["proposal_mutation"][0]]=c["proposal_mutation"][1]

    if r["current_registry_id"]!=REG_ID or r["current_registry_version"]!=2:return "unresolved","current-registry-binding"
    if r["proposed_registry_id"]!=PROPOSED_ID or r["proposed_registry_version"]!=3:return "unresolved","proposed-registry-binding"
    if digest(prop)!=r["proposed_registry_sha256"]:return "unresolved","proposal-digest"
    if r["predecessor_valid_until_version"]!=r["activation_version"]-1 or r["successor_valid_from_version"]!=r["activation_version"]:return "unresolved","rotation-version-overlap"

    old=prop["witnesses"]["w01"]["keys"]["w01-k1"];new=prop["witnesses"]["w01"]["keys"]["w01-k2"]
    if old["status"]!="retired" or old["valid_until_version"]!=3 or old["superseded_by"]!="w01-k2":return "unresolved","predecessor-registry-state"
    if new["status"]!="active" or new["valid_from_version"]!=4 or new["supersedes"]!="w01-k1":return "unresolved","successor-registry-state"
    if new["public_key"]!=r["successor_public_key"]:return "unresolved","successor-key-mismatch"

    pre=cer["ceremony"].get("predecessor_approval")
    if not pre:return "unresolved","missing-predecessor-approval"
    if pre["witness_id"]!="w01" or pre["key_id"]!="w01-k1" or pre["algorithm"]!=ALG:return "unresolved","predecessor-envelope"
    if pre["domain"]!="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.predecessor-approval.v1":return "unresolved","predecessor-domain"
    if not ed_verify(b64d(reg["witnesses"]["w01"]["keys"]["w01-k1"]["public_key"],32),b64d(pre["signature"],64),canonical(payload("predecessor-approval","w01","w01-k1",r))):return "unresolved","predecessor-signature"

    suc=cer["ceremony"].get("successor_proof_of_possession")
    if not suc:return "unresolved","missing-successor-possession"
    if suc["witness_id"]!="w01" or suc["key_id"]!="w01-k2" or suc["algorithm"]!=ALG:return "unresolved","successor-envelope"
    if suc["domain"]!="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.successor-possession.v1":return "unresolved","successor-domain"
    if not ed_verify(b64d(r["successor_public_key"],32),b64d(suc["signature"],64),canonical(payload("successor-possession","w01","w01-k2",r))):return "unresolved","successor-signature"

    gov=cer["ceremony"].get("governance_approvals",[]);seen=set()
    for a in gov:
        if a["witness_id"]=="w01":return "unresolved","governance-target-separation"
        if a["witness_id"] in seen:return "unresolved","duplicate-governance-witness"
        seen.add(a["witness_id"])
        if a["witness_id"] not in reg["witnesses"] or a["key_id"] not in reg["witnesses"][a["witness_id"]]["keys"]:return "unresolved","governance-key"
        if a["algorithm"]!=ALG:return "unresolved","governance-envelope"
        if a["domain"]!="mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation.governance-approval.v1":return "unresolved","governance-domain"
        pub=b64d(reg["witnesses"][a["witness_id"]]["keys"][a["key_id"]]["public_key"],32)
        if not ed_verify(pub,b64d(a["signature"],64),canonical(payload("governance-approval",a["witness_id"],a["key_id"],r))):return "unresolved","governance-signature"
    if len(seen)<3:return "unresolved","governance-below-threshold"
    return "qualified","rotation-authorized-and-possession-proven"

def main():
    if len(sys.argv)!=6:return 2
    reg=json.loads(Path(sys.argv[1]).read_text());prop=json.loads(Path(sys.argv[2]).read_text());cer=json.loads(Path(sys.argv[3]).read_text());camp=json.loads(Path(sys.argv[4]).read_text());out=Path(sys.argv[5])
    if cer.get("schema")!=CER_SCHEMA or camp.get("case_count")!=15 or len(camp.get("cases",[]))!=15:return 1
    ids=[x.get("case_id") for x in camp["cases"]]
    if len(ids)!=len(set(ids)):return 1
    rows=[];fail=[]
    for c in camp["cases"]:
        v,r=evaluate(reg,prop,cer,c);row={"case_id":c["case_id"],"expected_verdict":c["expected_verdict"],"actual_verdict":v,"reason":r};rows.append(row)
        if v!=c["expected_verdict"]:fail.append([c["case_id"],c["expected_verdict"],v,r])
    out.write_bytes(canonical({"schema":"mycelix.continual-adaptation.censoring-classification-anchor-witness-key-rotation-report.v1","status":"research-evidence-only","case_count":15,"cases":rows,"failures":fail})+b"\n");print(f"cases={len(rows)} failures={len(fail)}");return 1 if fail else 0
if __name__=="__main__":raise SystemExit(main())
