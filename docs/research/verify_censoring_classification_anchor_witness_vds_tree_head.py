#!/usr/bin/env python3
import base64,copy,hashlib,json,sys
from pathlib import Path
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head.v1'
DOMAIN=SCHEMA
REGID='mycelix.research.anchor-witness-registry.v2'
ALG='Ed25519'
EXPECTED_REGISTRY_SHA='sha256:29eae4dcdec3709a3245efeb3d47e642a0bdc7f49b6f13041b6b42849088d5cf'
VDS_ID='mycelix.research.anchor-statement-sequence.v1'

def canon(v): return json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(',',':')).encode()
def H(b): return hashlib.sha256(b).digest()
def leaf(e): return H(b'\0'+canon(e))
def node(a,b): return H(b'\1'+a+b)
def mth(ds):
    n=len(ds)
    if n==0:return H(b'')
    if n==1:return leaf(ds[0])
    k=1<<((n-1).bit_length()-1)
    return node(mth(ds[:k]),mth(ds[k:]))
def hh(s):
    if not isinstance(s,str) or not s.startswith('sha256:') or len(s)!=71:raise ValueError('hash')
    return bytes.fromhex(s[7:])
def b64(s,n):
    if not isinstance(s,str):raise ValueError('base64')
    raw=base64.urlsafe_b64decode(s+'='*((4-len(s)%4)%4))
    if len(raw)!=n:raise ValueError('length')
    return raw
def registry_digest(reg):return 'sha256:'+hashlib.sha256(canon(reg)).hexdigest()

def key(reg,w,kid,version):
    wi=reg['witnesses'].get(w)
    k=wi and wi['keys'].get(kid)
    if not k:return None,'unknown-key'
    if k['algorithm']!=ALG:return None,'key-algorithm'
    if version<k['valid_from_version']:return None,'key-not-yet-valid'
    if k['valid_until_version'] is not None and version>k['valid_until_version']:return None,'key-expired-for-version'
    if k['revoked_at_version'] is not None and version>=k['revoked_at_version']:return None,'key-revoked'
    if k['status']=='revoked':return None,'key-revoked'
    return b64(k['public_key'],32),None

def payload(att,identity):
    return {'schema':SCHEMA,'domain':DOMAIN,'algorithm':ALG,'observer_id':att['observer_id'],'key_id':att['key_id'],'witness_identity_commitment':identity,'registry_id':att['registry_id'],'registry_version':att['registry_version'],'vds_id':att['vds_id'],'manifest_version':att['manifest_version'],'tree_size':att['tree_size'],'root_hash':att['root_hash']}

def verify_head(att,reg,entries):
    fields=['schema','domain','algorithm','observer_id','key_id','registry_id','registry_version','vds_id','manifest_version','tree_size','root_hash','signature']
    if not isinstance(att,dict) or set(att)!=set(fields):return None,'signature-wrapping-or-schema'
    if att['schema']!=SCHEMA:return None,'head-schema'
    if att['domain']!=DOMAIN:return None,'head-domain'
    if att['algorithm']!=ALG:return None,'head-algorithm'
    if att['registry_id']!=REGID or att['registry_version']!=2:return None,'head-registry-binding'
    if att['vds_id']!=VDS_ID:return None,'head-vds-binding'
    if att['observer_id'] not in reg['witnesses']:return None,'unknown-witness'
    if not isinstance(att['manifest_version'],int) or att['manifest_version']<1:return None,'manifest-version'
    if not isinstance(att['tree_size'],int) or att['tree_size']<1 or att['tree_size']>len(entries):return None,'tree-size'
    try:root=hh(att['root_hash'])
    except Exception:return None,'root-encoding'
    if mth(entries[:att['tree_size']])!=root:return None,'head-root-mismatch'
    pub,e=key(reg,att['observer_id'],att['key_id'],att['manifest_version'])
    if e:return None,e
    try:sig=b64(att['signature'],64)
    except Exception:return None,'signature-encoding'
    try:Ed25519PublicKey.from_public_bytes(pub).verify(sig,canon(payload(att,reg['witnesses'][att['observer_id']]['identity_commitment'])))
    except Exception:return None,'signature-invalid'
    return att,None

def validate_set(attmap,reg,entries):
    if len(attmap)<3:return None,'below-threshold'
    vals=[]
    for w,a in attmap.items():
        v,e=verify_head(a,reg,entries)
        if e:return None,e
        if v['observer_id']!=w:return None,'observer-id-mismatch'
        vals.append(v)
    tuples={(a['manifest_version'],a['tree_size'],a['root_hash']) for a in vals}
    if len(tuples)!=1:return None,'equivocation'
    return vals[0],None

def consistency(m,n,first,second,path):
    if not path or not(0<m<n):return False
    p=list(path)
    if m&(m-1)==0:p=[first]+p
    fn,sn=m-1,n-1
    while fn&1:fn>>=1;sn>>=1
    fr=sr=p[0]
    for c in p[1:]:
        if sn==0:return False
        if (fn&1) or fn==sn:
            fr=node(c,fr);sr=node(c,sr)
            if not(fn&1):
                while fn and not(fn&1):fn>>=1;sn>>=1
        else:sr=node(sr,c)
        fn>>=1;sn>>=1
    return sn==0 and fr==first and sr==second

def eval_case(fixture,reg,c):
    entries=fixture['entries']
    heads=fixture['heads']
    attmap=copy.deepcopy(heads[c['base']]['attestations'])
    if c.get('remove_observers'):
        for w in c['remove_observers']:attmap.pop(w,None)
    if c.get('remove_observer'):attmap.pop(c['remove_observer'],None)
    if c.get('replace_observer'):attmap[c['replace_observer']]=copy.deepcopy(heads[c['replace_with']])
    if c.get('inject')=='fork_size_4':attmap['w02']=copy.deepcopy(heads['fork_size_4']['attestation'])
    if c.get('duplicate_as'):attmap[c['duplicate_as']]=copy.deepcopy(heads['fork_size_4']['attestation'])
    if c.get('mutate_observer'):
        a=attmap[c['mutate_observer']];a[c['mutation']]=True if c['mutation']=='extra_field' else c['value']
    first,e=validate_set(attmap,reg,entries)
    if e:return 'unresolved',e
    if c.get('second'):
        second_map=copy.deepcopy(heads[c['second']]['attestations'])
        second,e=validate_set(second_map,reg,entries)
        if e:return 'unresolved',e
        if second['tree_size']<first['tree_size']:return 'unresolved','rollback'
        if second['tree_size']==first['tree_size']:return ('qualified','same-head') if first['root_hash']==second['root_hash'] else ('unresolved','equivocation')
        p=[hh(x) for x in fixture['consistency_proof_4_to_7']]
        if c.get('proof_mutation')=='replace-first':p[0]=H(p[0])
        return ('qualified','consistency-proof') if consistency(first['tree_size'],second['tree_size'],hh(first['root_hash']),hh(second['root_hash']),p) else ('unresolved','consistency-proof-invalid')
    return 'qualified','authenticated-tree-head'

def main():
    if len(sys.argv)!=5:return 2
    reg=json.load(open(sys.argv[1]));fixture=json.load(open(sys.argv[2]));campaign=json.load(open(sys.argv[3]));out=Path(sys.argv[4])
    if reg.get('registry_id')!=REGID or reg.get('registry_version')!=2 or registry_digest(reg)!=EXPECTED_REGISTRY_SHA:return 1
    if campaign.get('case_count')!=21 or len(campaign.get('cases',[]))!=21:return 1
    ids=[c.get('case_id') for c in campaign['cases']]
    if len(ids)!=len(set(ids)):return 1
    rows=[];fails=[]
    for c in campaign['cases']:
        v,r=eval_case(fixture,reg,c)
        row={'case_id':c['case_id'],'expected_verdict':c['expected_verdict'],'actual_verdict':v,'reason':r};rows.append(row)
        if v!=c['expected_verdict']:fails.append([c['case_id'],c['expected_verdict'],v,r])
    out.write_bytes(canon({'schema':'mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-tree-head-report.v1','status':'research-evidence-only','case_count':len(rows),'cases':rows,'failures':fails})+b'\n')
    print(f'cases={len(rows)} failures={len(fails)}')
    return 1 if fails else 0

if __name__=='__main__':raise SystemExit(main())
