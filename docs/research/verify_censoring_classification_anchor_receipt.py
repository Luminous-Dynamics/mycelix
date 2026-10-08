#!/usr/bin/env python3
import base64,copy,hashlib,json,sys
from pathlib import Path
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

REG_ID='mycelix.research.anchor-receipt-ts-registry.v1'
TS_ID='mycelix.research.anchor-receipt-ts.v1'
SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-receipt.v1'
VDS_ID='mycelix.research.anchor-statement-sequence.v1'
WREG='mycelix.research.anchor-witness-registry.v2'
EXPECTED_TS_REG_SHA='sha256:efb549a023660010a07d70b7d78afbfd81664200fe4ecf1d0d542dc945f0bb54'

def canon(v):return json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(',',':')).encode()
def H(b):return hashlib.sha256(b).digest()
def digest(v):return 'sha256:'+H(canon(v)).hex()
def leaf(e):return H(b'\0'+canon(e))
def node(a,b):return H(b'\1'+a+b)
def mth(ds):
    n=len(ds)
    if n==0:return H(b'')
    if n==1:return leaf(ds[0])
    k=1<<((n-1).bit_length()-1);return node(mth(ds[:k]),mth(ds[k:]))
def hh(s):
    if not isinstance(s,str) or not s.startswith('sha256:') or len(s)!=71:raise ValueError('hash')
    return bytes.fromhex(s[7:])
def b64(s,n):
    if not isinstance(s,str):raise ValueError('base64')
    raw=base64.urlsafe_b64decode(s+'='*((4-len(s)%4)%4))
    if len(raw)!=n:raise ValueError('length')
    return raw

def witness_key(reg,w,kid,v):
    k=reg['witnesses'].get(w,{}).get('keys',{}).get(kid)
    if not k:return None,'unknown-key'
    if k['algorithm']!='Ed25519':return None,'key-algorithm'
    if v<k['valid_from_version']:return None,'key-not-yet-valid'
    if k.get('valid_until_version') is not None and v>k['valid_until_version']:return None,'key-expired-for-version'
    if k.get('revoked_at_version') is not None and v>=k['revoked_at_version']:return None,'key-revoked'
    if k['status']=='revoked':return None,'key-revoked'
    return b64(k['public_key'],32),None

def verify_head(att,w,reg,entries):
    fields=['schema','domain','algorithm','observer_id','key_id','registry_id','registry_version','vds_id','manifest_version','tree_size','root_hash','signature']
    if not isinstance(att,dict) or set(att)!=set(fields):return None,'head-schema'
    if att['schema']!='mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1':return None,'head-schema'
 if att['domain']!='mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1':return None,'head-domain'
 if att['algorithm']!='Ed25519':return None,'head-algorithm'
 if att['observer_id']!=w:return None,'head-observer-binding'
    if att['registry_id']!=WREG or att['registry_version']!=2:return None,'head-registry-binding'
    if att['vds_id']!=VDS_ID:return None,'head-vds-binding'
    if att['tree_size']!=7:return None,'head-tree-size'
    try:root=hh(att['root_hash']);sig=b64(att['signature'],64)
    except Exception:return None,'head-encoding'
    if mth(entries[:7])!=root:return None,'head-root-mismatch'
    pub,e=witness_key(reg,w,att['key_id'],att['manifest_version'])
    if e:return None,e
    payload={'schema':att['schema'],'domain':att['domain'],'algorithm':att['algorithm'],'observer_id':w,'key_id':att['key_id'],'witness_identity_commitment':reg['witnesses'][w]['identity_commitment'],'registry_id':att['registry_id'],'registry_version':att['registry_version'],'vds_id':att['vds_id'],'manifest_version':att['manifest_version'],'tree_size':att['tree_size'],'root_hash':att['root_hash']}
    try:Ed25519PublicKey.from_public_bytes(pub).verify(sig,canon(payload))
    except Exception:return None,'head-signature-invalid'
    return att,None

def head_quorum(head_fixture,reg,entries,expected_root):
    vals=[]
    for w,a in head_fixture['heads']['size_7']['attestations'].items():
        v,e=verify_head(a,w,reg,fixture['entries'])
        if e:return None,e
        vals.append(v)
    if len(vals)<3:return None,'head-below-threshold'
    target={(a['manifest_version'],a['tree_size'],a['root_hash']) for a in vals}
    if len(target)!=1:return None,'head-equivocation'
    q=digest({'tree_size':7,'root_hash':fixture['root_hash'],'head_digests':sorted(digest(a) for a in vals)})
    if q!=fixture['tree_head_quorum_digest']:return None,'head-quorum-digest'
    return vals[0],None

def verify_receipt(r,tsreg,fixture):
    fields=['schema','domain','algorithm','ts_id','key_id','claims','signature']
    if not isinstance(r,dict) or set(r)!=set(fields):return None,'receipt-schema'
    if r['schema']!=SCHEMA:return None,'receipt-schema'
    if r['domain']!=SCHEMA:return None,'receipt-domain'
    if r['algorithm']!='Ed25519':return None,'receipt-algorithm'
    if r['ts_id']!=TS_ID or r['key_id'] not in tsreg['keys']:return None,'receipt-ts-binding'
    c=r['claims'];req=['registry_id','registry_version','ts_id','vds_id','statement_id','statement_hash','manifest_version','tree_size','leaf_index','root_hash','tree_head_quorum_digest']
    if not isinstance(c,dict) or set(c)!=set(req):return None,'receipt-claims-schema'
    if c['registry_id']!=tsreg['registry_id'] or c['registry_version']!=tsreg['registry_version']:return None,'receipt-registry-binding'
    if c['ts_id']!=TS_ID or c['vds_id']!=VDS_ID:return None,'receipt-vds-binding'
    if c['tree_size']!=fixture['tree_size'] or c['root_hash']!=fixture['root_hash']:return None,'receipt-head-binding'
 if tsreg.get('algorithm')!='Ed25519' or not tsreg['keys'][r['key_id']].get('status')=='active':return None,'receipt-key-lifecycle'
    try:sig=b64(r['signature'],64);pub=b64(tsreg['keys'][r['key_id']]['public_key'],32)
    except Exception:return None,'signature-encoding'
    try:Ed25519PublicKey.from_public_bytes(pub).verify(sig,canon({'schema':r['schema'],'domain':r['domain'],'algorithm':r['algorithm'],'ts_id':r['ts_id'],'key_id':r['key_id'],'claims':c}))
    except Exception:return None,'signature-invalid'
    i=c['leaf_index']
    if not isinstance(i,int) or i<0 or i>=c['tree_size']:return None,'leaf-index'
    e=fixture['entries'][i]
    if c['statement_id']!=e['statement_id'] or c['statement_hash']!='sha256:'+leaf(e).hex():return None,'statement-binding'
    if c['manifest_version']!=e['manifest_version']:return None,'manifest-binding'
    return c,None

def inclusion(index,size,leaf_hash,root,path):
    if size<=0 or index<0 or index>=size:return False
    fn,sn=index,size-1;r=leaf_hash
    for p in path:
        if sn==0:return False
        if (fn&1) or fn==sn:
            r=node(p,r)
            if not(fn&1):
                while fn and not(fn&1):fn>>=1;sn>>=1
        else:r=node(r,p)
        fn>>=1;sn>>=1
    return sn==0 and r==root

def main():
    if len(sys.argv)!=6:return 2
    ts=json.load(open(sys.argv[1]));f=json.load(open(sys.argv[2]));camp=json.load(open(sys.argv[3]));wreg=json.load(open(sys.argv[4]));out=Path(sys.argv[5])
    if ts.get('registry_id')!=REG_ID or ts.get('registry_version')!=1 or digest(ts)!=EXPECTED_TS_REG_SHA:return 1
    if f.get('ts_registry_sha256')!=EXPECTED_TS_REG_SHA:return 1
    if camp.get('case_count')!=16 or len(camp.get('cases',[]))!=16:return 1
    rows=[];fails=[]
    for c in camp['cases']:
        r=copy.deepcopy(f['receipts'][str(c.get('receipt_id','0'))])
        for field,val in c.get('mutate_receipt',[]):
            (r.__setitem__(field,val) if field in r else r['claims'].__setitem__(field,val))
        rc,e=verify_receipt(r,ts,f)
        if e:v,reason='unresolved',e
        else:
            _,he=head_quorum(f,wreg)
            if he:v,reason='unresolved',he
            else:
                proof=[hh(x) for x in f['inclusion_proofs'][str(rc['leaf_index'])]]
                if c.get('proof_mutation')=='replace-first':proof[0]=H(proof[0])
                elif c.get('proof_mutation')=='truncate':proof=proof[:-1]
                elif c.get('proof_mutation')=='append-extra':proof.append(H(b'extra'))
                elif c.get('proof_mutation')=='empty':proof=[]
                ok=inclusion(rc['leaf_index'],rc['tree_size'],hh(rc['statement_hash']),hh(rc['root_hash']),proof)
                v,reason=('qualified','receipt-and-inclusion-proof') if ok else ('unresolved','inclusion-proof-invalid')
        row={'case_id':c['case_id'],'expected_verdict':c['expected_verdict'],'actual_verdict':v,'reason':reason};rows.append(row)
        if v!=c['expected_verdict']:fails.append([c['case_id'],c['expected_verdict'],v,reason])
    out.write_bytes(canon({'schema':'mycelix.continual-adaptation.censoring-classification-anchor-receipt-report.v1','status':'research-evidence-only','case_count':16,'cases':rows,'failures':fails})+b'\n')
    print(f'cases={len(rows)} failures={len(fails)}');return 1 if fails else 0
if __name__=='__main__':raise SystemExit(main())
