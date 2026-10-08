#!/usr/bin/env python3
"""Research-only cryptographic witness verifier.

The trust model is intentionally layered:
  pinned registry -> independently signed witness observations -> quorum -> non-equivocation.
No private witness keys are required by the verifier.
"""
from __future__ import annotations
import base64, copy, hashlib, json, sys
from pathlib import Path

ROOT_SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-witness-trust-root.v2'
REGISTRY_SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-witness-registry.v2'
CHECKPOINT_SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-witness-checkpoint.v3'
CAMPAIGN_SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-witness-crypto-campaign.v1'
ATTESTATION_SCHEMA='mycelix.continual-adaptation.censoring-classification-anchor-witness-signature.v2'
REGISTRY_ID='mycelix.research.anchor-witness-registry.v2'
ROOT_ID='mycelix.research.anchor-witness-root.v2'
AUTHORITY_ID='mycelix.research.anchor-authority.v1'
ALGORITHM='Ed25519'
DOMAIN='mycelix.continual-adaptation.censoring-classification-anchor-witness-attestation.v3'

# Minimal dependency-free Ed25519 verifier, following RFC 8032 group arithmetic.
Q=2**255-19
L=2**252+27742317777372353535851937790883648493
D=(-121665*pow(121666,Q-2,Q))%Q
I=pow(2,(Q-1)//4,Q)
B_Y=(4*pow(5,Q-2,Q))%Q

def inv(x:int)->int: return pow(x,Q-2,Q)
def xrecover(y:int)->int:
    xx=(y*y-1)*inv(D*y*y+1)%Q
    x=pow(xx,(Q+3)//8,Q)
    if (x*x-xx)%Q: x=x*I%Q
    if x&1: x=Q-x
    return x
B=(xrecover(B_Y),B_Y,1,(xrecover(B_Y)*B_Y)%Q)
def add(P,N):
    X1,Y1,Z1,T1=P; X2,Y2,Z2,T2=N
    A=(Y1-X1)*(Y2-X2)%Q; Bv=(Y1+X1)*(Y2+X2)%Q
    C=2*D*T1*T2%Q; Dv=2*Z1*Z2%Q
    E=(Bv-A)%Q; F=(Dv-C)%Q; G=(Dv+C)%Q; H=(Bv+A)%Q
    return E*F%Q,G*H%Q,F*G%Q,E*H%Q
def mul(P,n:int):
    R=(0,1,1,0)
    while n:
        if n&1: R=add(R,P)
        P=add(P,P); n>>=1
    return R
def encode(P)->bytes:
    X,Y,Z,T=P; zi=inv(Z); x=X*zi%Q; y=Y*zi%Q
    return (y|((x&1)<<255)).to_bytes(32,'little')
def decode(s:bytes):
    if len(s)!=32: raise ValueError('point-length')
    y=int.from_bytes(s,'little'); sign=y>>255; y&=(1<<255)-1
    if y>=Q: raise ValueError('noncanonical-y')
    x=xrecover(y)
    if x==0 and sign: raise ValueError('negative-zero')
    if (x&1)!=sign: x=Q-x
    P=(x,y,1,x*y%Q)
    if encode(mul(P,L))!=encode((0,1,1,0)): raise ValueError('small-order-point')
    return P
def ed25519_verify(public_key:bytes,signature:bytes,message:bytes)->bool:
    if len(public_key)!=32 or len(signature)!=64: return False
    try:
        A=decode(public_key); R=decode(signature[:32]); S=int.from_bytes(signature[32:],'little')
        if S>=L: return False
        k=int.from_bytes(hashlib.sha512(signature[:32]+public_key+message).digest(),'little')%L
        return encode(mul(B,S))==encode(add(R,mul(A,k)))
    except Exception:
        return False

def canonical(value:object)->bytes:
    return json.dumps(value,ensure_ascii=False,sort_keys=True,separators=(',',':')).encode('utf-8')
def digest(value:object)->str:
    return 'sha256:'+hashlib.sha256(canonical(value)).hexdigest()
def b64d(value,expected_len:int)->bytes:
    if not isinstance(value,str) or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_' for c in value): raise ValueError('base64url')
    if len(value)!=(4*expected_len+2)//3: raise ValueError('base64url-length')
    raw=base64.urlsafe_b64decode(value+'='*((4-len(value)%4)%4))
    if len(raw)!=expected_len: raise ValueError('decoded-length')
    return raw

def validate_registry(reg):
    if reg.get('schema')!=REGISTRY_SCHEMA or reg.get('status')!='research-witness-crypto-registry-only': return 'registry-schema'
    if reg.get('registry_id')!=REGISTRY_ID or reg.get('registry_version')!=2: return 'registry-identity'
    q,f,ws=reg.get('threshold'),reg.get('max_faulty'),reg.get('witnesses')
    if not isinstance(q,int) or not isinstance(f,int) or not isinstance(ws,dict): return 'registry-quorum'
    n=len(ws)
    if n==0 or not (1<=q<=n) or f<0 or 2*q<=n+f: return 'registry-quorum-intersection'
    if reg.get('signature_profile')!={'algorithm':ALGORITHM,'encoding':'base64url-no-padding','domain':DOMAIN}: return 'signature-profile'
    seen=set(); public_seen=set()
    for _w,wi in ws.items():
        if wi.get('role')!='anchor-witness' or not isinstance(wi.get('identity_commitment'),str) or not isinstance(wi.get('keys'),dict) or not wi['keys']: return 'witness-schema'
        for kid,key in wi['keys'].items():
            if kid in seen: return 'duplicate-key-id'
            seen.add(kid)
            if key.get('algorithm')!=ALGORITHM: return 'key-algorithm'
            try: raw=b64d(key.get('public_key',''),32)
            except Exception: return 'key-public-key'
            public_tag=raw.hex()
            if public_tag in public_seen: return 'duplicate-public-key'
            public_seen.add(public_tag)
            vf,vu,rv=key.get('valid_from_version'),key.get('valid_until_version'),key.get('revoked_at_version')
            if not isinstance(vf,int) or vf<1: return 'key-valid-from'
            if vu is not None and (not isinstance(vu,int) or vu<vf): return 'key-valid-until'
            if rv is not None and not isinstance(rv,int): return 'key-revoked-at'
            if key.get('status') not in {'active','retired','revoked'}: return 'key-status'
            if key.get('status')=='active' and (vu is not None or rv is not None): return 'active-key-bounds'
            if key.get('status') in {'retired','revoked'} and vu is None: return 'bounded-key-missing-end'
            if key.get('status')=='revoked' and rv is None: return 'revoked-key-missing-revocation'
            if key.get('status')=='retired' and rv is not None: return 'retired-revocation-conflict'
            if key.get('status')!='revoked' and rv is not None: return 'non-revoked-has-revocation'
            if key.get('supersedes') is not None:
                old=wi['keys'].get(key['supersedes'])
                if not isinstance(old,dict) or old.get('valid_until_version') is None or key['valid_from_version']<=old['valid_until_version']: return 'invalid-rotation-lineage'
                if old.get('superseded_by')!=kid: return 'asymmetric-successor-lineage'
            if key.get('superseded_by') is not None:
                new=wi['keys'].get(key['superseded_by'])
                if not isinstance(new,dict) or key.get('valid_until_version') is None or new.get('valid_from_version')<=key['valid_until_version']: return 'invalid-successor-lineage'
                if new.get('supersedes')!=kid: return 'asymmetric-successor-lineage'
    return None

def validate_root(root,reg,expected):
    if digest(root)!=expected: return 'root-pin'
    if root.get('schema')!=ROOT_SCHEMA or root.get('status')!='research-witness-crypto-trust-root-only': return 'root-schema'
    if root.get('trust_root_id')!=ROOT_ID or root.get('registry_id')!=REGISTRY_ID: return 'root-identity'
    if root.get('registry_version')!=reg.get('registry_version') or root.get('threshold')!=reg.get('threshold') or root.get('max_faulty')!=reg.get('max_faulty'): return 'root-parameters'
    if root.get('attestation_schema')!=ATTESTATION_SCHEMA: return 'root-attestation-schema'
    if root.get('registry_sha256')!=digest(reg): return 'registry-root-binding'
    return validate_registry(reg)

def validate_checkpoint(cp,root,reg):
    if cp.get('schema')!=CHECKPOINT_SCHEMA or cp.get('status')!='research-witness-crypto-checkpoint-only': return 'checkpoint-schema'
    if cp.get('root_reference_sha256')!=digest(root): return 'checkpoint-root-binding'
    if cp.get('registry_id')!=REGISTRY_ID or cp.get('registry_version')!=reg.get('registry_version'): return 'checkpoint-registry-binding'
    if not isinstance(cp.get('witness_attestations'),dict): return 'checkpoint-attestations'
    return None

def key_for(reg,w,kid,version):
    wi=reg['witnesses'].get(w)
    if not isinstance(wi,dict): return None,'unknown-witness'
    key=wi.get('keys',{}).get(kid)
    if not isinstance(key,dict): return None,'unknown-key'
    if key.get('algorithm')!=ALGORITHM: return None,'key-algorithm'
    try: pub=b64d(key['public_key'],32)
    except Exception: return None,'key-public-key'
    if version<key['valid_from_version']: return None,'key-not-yet-valid'
    if key.get('valid_until_version') is not None and version>key['valid_until_version']: return None,'key-expired-for-version'
    if key.get('revoked_at_version') is not None and version>=key['revoked_at_version']: return None,'key-revoked'
    if key.get('status')=='revoked': return None,'key-revoked'
    return pub,None

def verify_attestation(cp,w,att,root,reg):
    if not isinstance(att,dict): return None,'attestation-type'
    if set(att)!={'schema','witness_id','key_id','algorithm','domain','claims','signature'}: return None,'signature-wrapping-or-schema'
    if att.get('schema')!=ATTESTATION_SCHEMA: return None,'attestation-schema'
    if att.get('witness_id')!=w: return None,'witness-id-mismatch'
    if att.get('algorithm')!=ALGORITHM: return None,'algorithm-substitution'
    if att.get('domain')!=DOMAIN: return None,'domain-separation'
    c=att.get('claims')
    required={'registry_id','registry_version','root_reference_sha256','authority_id','manifest_version','manifest_sha256','previous_manifest_sha256'}
    if not isinstance(c,dict) or set(c)!=required: return None,'claims-schema'
    if c['registry_id']!=REGISTRY_ID or c['registry_version']!=reg['registry_version']: return None,'claims-registry-binding'
    if c['root_reference_sha256']!=cp['root_reference_sha256'] or c['root_reference_sha256']!=digest(root): return None,'claims-root-binding'
    if c['authority_id']!=AUTHORITY_ID: return None,'claims-authority'
    if not isinstance(c['manifest_version'],int) or c['manifest_version']<1: return None,'claims-version'
    pub,e=key_for(reg,w,att['key_id'],c['manifest_version'])
    if e: return None,e
    try: sig=b64d(att['signature'],64)
    except Exception: return None,'signature-encoding'
    payload={'schema':ATTESTATION_SCHEMA,'domain':DOMAIN,'algorithm':ALGORITHM,'witness_id':w,'key_id':att['key_id'],'witness_identity_commitment':reg['witnesses'][w]['identity_commitment'],'claims':c}
    if not ed25519_verify(pub,sig,canonical(payload)): return None,'signature-invalid'
    return c,None

def consensus(cp,root,reg):
    ids=list(cp['witness_attestations'])
    if len(ids)<reg['threshold']: return None,'below-threshold'
    claims=[]
    for w in ids:
        if w not in reg['witnesses']: return None,'unknown-witness'
        c,e=verify_attestation(cp,w,cp['witness_attestations'][w],root,reg)
        if e: return None,e
        claims.append(c)
    tuples={(c['authority_id'],c['manifest_version'],c['manifest_sha256'],c['previous_manifest_sha256'],c['root_reference_sha256']) for c in claims}
    if len(tuples)!=1: return None,'equivocation'
    return claims[0],None

def evaluate(case,baseline,forward,root,reg,expected):
    cp=copy.deepcopy(forward if case.get('base')=='candidate' else baseline)
    root2,reg2=copy.deepcopy(root),copy.deepcopy(reg)
    w=case.get('witness')
    if case.get('remove_witness'): cp['witness_attestations'].pop(case['remove_witness'],None)
    if case.get('remove_witnesses'):
        for x in case['remove_witnesses']: cp['witness_attestations'].pop(x,None)
    if case.get('fork_witness'): cp['witness_attestations'][case['fork_witness']]=copy.deepcopy(case['replacement_attestation'])
    if case.get('mutate_key_id'): cp['witness_attestations'][w]['key_id']=case['mutate_key_id']
    if case.get('mutate_domain'): cp['witness_attestations'][w]['domain']=case['mutate_domain']
    if case.get('mutate_algorithm'): cp['witness_attestations'][w]['algorithm']=case['mutate_algorithm']
    if case.get('replacement_attestation') is not None and not case.get('fork_witness'): cp['witness_attestations'][w]=copy.deepcopy(case['replacement_attestation'])
    if case.get('extra_field'): cp['witness_attestations'][w][case['extra_field']]=True
    if case.get('apply_manifest'): cp['witness_attestations'][w]['claims']['manifest_sha256']=case['apply_manifest']
    if case.get('apply_predecessor'): cp['witness_attestations'][w]['claims']['previous_manifest_sha256']=case['apply_predecessor']
    if case.get('tamper_witness'): cp['witness_attestations'][case['tamper_witness']]['claims'][case['tamper_field']]=case['tamper_value']
    if case.get('candidate_break_predecessor') or case.get('candidate_manifest_sha256') or case.get('candidate_gap'):
        for a in cp['witness_attestations'].values():
            if case.get('candidate_break_predecessor'): a['claims']['previous_manifest_sha256']='sha256:deadbeef'
            if case.get('candidate_manifest_sha256'): a['claims']['manifest_sha256']=case['candidate_manifest_sha256']
            if case.get('candidate_gap'): a['claims']['manifest_version']+=int(case['candidate_gap'])
    if case.get('root_threshold') is not None: root2['threshold']=case['root_threshold']
    if case.get('root_registry_sha256') is not None: root2['registry_sha256']=case['root_registry_sha256']
    if case.get('conflicting_root_reference'): cp['root_reference_sha256']=case['conflicting_root_reference']
    if case.get('registry_threshold') is not None: reg2['threshold']=case['registry_threshold']
    if case.get('registry_public_key_reuse'):
        tw,tk,sw,sk=case['registry_public_key_reuse']; reg2['witnesses'][tw]['keys'][tk]['public_key']=reg2['witnesses'][sw]['keys'][sk]['public_key']
    if case.get('registry_key_mutation'):
        tw,tk,field,value=case['registry_key_mutation']; reg2['witnesses'][tw]['keys'][tk][field]=value
    if case.get('registry_witness_id_swap'):
        a,b=case['registry_witness_id_swap']; reg2['witnesses'][a]['identity_commitment'],reg2['witnesses'][b]['identity_commitment']=reg2['witnesses'][b]['identity_commitment'],reg2['witnesses'][a]['identity_commitment']
    if case.get('unknown_witness'): cp['witness_attestations'][case['unknown_witness']]=copy.deepcopy(cp['witness_attestations']['w01'])
    if case.get('duplicate_witness') or case.get('same_witness_twice'): return 'unresolved','duplicate-witness'
    if case.get('reorder'): cp['witness_attestations']={k:cp['witness_attestations'][k] for k in reversed(list(cp['witness_attestations']))}
    registry_error=validate_registry(reg2)
    if registry_error: return 'unresolved',registry_error
    root_error=validate_root(root2,reg2,expected)
    if root_error: return 'unresolved',root_error
    if validate_checkpoint(cp,root2,reg2): return 'unresolved',validate_checkpoint(cp,root2,reg2)
    c,e=consensus(cp,root2,reg2)
    if e: return 'unresolved',e
    if case.get('base')=='candidate':
        bc,be=consensus(baseline,root,reg)
        if be: return 'unresolved','baseline-'+be
        if c['manifest_version']!=bc['manifest_version']+1: return 'unresolved','forward-version'
        if c['previous_manifest_sha256']!=bc['manifest_sha256']: return 'unresolved','forward-predecessor'
    return 'qualified','ok'

def known_answer_test()->bool:
    public=bytes.fromhex('d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a')
    signature=bytes.fromhex('e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b')
    return ed25519_verify(public,signature,b'')

def main()->int:
    if len(sys.argv)!=8: print('usage: verifier EXPECTED_ROOT_SHA TRUST_ROOT REGISTRY BASELINE FORWARD CAMPAIGN REPORT',file=sys.stderr); return 2
    expected,rootp,regp,basep,fwdp,campp,outp=sys.argv[1:]
    root=json.loads(Path(rootp).read_text()); reg=json.loads(Path(regp).read_text()); baseline=json.loads(Path(basep).read_text()); forward=json.loads(Path(fwdp).read_text()); campaign=json.loads(Path(campp).read_text())
    if campaign.get('schema')!=CAMPAIGN_SCHEMA or campaign.get('expected_trust_root_sha256')!=expected or campaign.get('case_count')!=22 or len(campaign.get('cases',[]))!=22: return 1
    if not known_answer_test():
        print('ed25519-known-answer-test-failed', file=sys.stderr); return 1
    ids=[c.get('case_id') for c in campaign['cases']]
    if len(ids)!=len(set(ids)): return 1
    rows=[]; failures=[]
    for case in campaign['cases']:
        actual,reason=evaluate(case,baseline,forward,root,reg,expected)
        rows.append({'case_id':case['case_id'],'expected_verdict':case['expected_verdict'],'actual_verdict':actual,'reason':reason})
        if actual!=case['expected_verdict']: failures.append([case['case_id'],case['expected_verdict'],actual,reason])
    report={'schema':'mycelix.continual-adaptation.censoring-classification-anchor-witness-crypto-report.v2','status':'research-evidence-only','case_count':22,'ed25519_known_answer_test':'pass','cases':rows,'failures':failures}
    Path(outp).write_bytes(canonical(report)+b'\n'); print(f'cases={len(rows)} failures={len(failures)}'); return 1 if failures else 0
if __name__=='__main__': raise SystemExit(main())
