#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json,sys
from pathlib import Path
CD=b'MYCELIX_QUALIFICATION_CAPSULE_V1\0';RD=b'MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V1\0'
def c(v):
 if v is None:return b'null'
 if v is True:return b'true'
 if v is False:return b'false'
 if isinstance(v,int)and not isinstance(v,bool):return str(v).encode()
 if isinstance(v,float):raise ValueError('float')
 if isinstance(v,str):return json.dumps(v,ensure_ascii=False,separators=(',',':')).encode()
 if isinstance(v,list):return b'['+b','.join(c(x)for x in v)+b']'
 if isinstance(v,dict):
  if not all(isinstance(k,str)for k in v):raise ValueError('key')
  return b'{'+b','.join(json.dumps(k,ensure_ascii=False).encode()+b':'+c(v[k])for k in sorted(v,key=lambda k:k.encode()))+b'}'
 raise ValueError(type(v).__name__)
def h(d,v):
 b=c(v);return hashlib.sha256(d+len(b).to_bytes(8,'big')+b).hexdigest()
def main(p):
 x=json.loads(Path(p).read_text())
 if x.get('vector_format_revision')!=1:raise SystemExit('vector version')
 m=x['manifest'];r=x['receipt'];body=dict(r);body.pop('receipt_commitment')
 if h(CD,m)!=x['capsule_commitment']:raise SystemExit('capsule commitment')
 if r['capsule_commitment']!=x['capsule_commitment']:raise SystemExit('receipt capsule')
 if h(RD,body)!=x['receipt_commitment']or r['receipt_commitment']!=x['receipt_commitment']:raise SystemExit('receipt commitment')
 if h(CD,dict(reversed(list(m.items()))))!=x['capsule_commitment']:raise SystemExit('object order')
 b=c(m)
 if 'café'.encode()not in b or '雪'.encode()not in b or 'é'.encode()not in b:raise SystemExit('utf8')
 try:c({'x':1.5})
 except ValueError:pass
 else:raise SystemExit('float accepted')
 print('independent_qcap_v1_capsule_vector=PASS')
 print('independent_qcap_v1_receipt_vector=PASS')
 print('independent_qcap_v1_utf8_canonicalization=PASS')
 print('independent_qcap_v1_vectors=PASS')
if __name__=='__main__':
 if len(sys.argv)!=2:raise SystemExit('usage: verify_qcap_vectors.py VECTOR.json')
 main(sys.argv[1])
