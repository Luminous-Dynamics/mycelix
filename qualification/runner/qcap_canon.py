from __future__ import annotations
import hashlib,json,re
from pathlib import PurePosixPath
CAPSULE_DOMAIN=b"MYCELIX_QUALIFICATION_CAPSULE_V1\0";RECEIPT_DOMAIN=b"MYCELIX_QUALIFICATION_ATTEMPT_RECEIPT_V2\0"
H40=re.compile(r"^[0-9a-f]{40}$");H64=re.compile(r"^[0-9a-f]{64}$");IDENT=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$");EMPTY_SHA256=hashlib.sha256(b"").hexdigest()
class CapsuleError(Exception):pass
def canonical_json(v):
 def e(x):
  if x is None:return"null"
  if x is True:return"true"
  if x is False:return"false"
  if isinstance(x,int)and not isinstance(x,bool):return str(x)
  if isinstance(x,float):raise CapsuleError("floats forbidden")
  if isinstance(x,str):return json.dumps(x,ensure_ascii=False,separators=(",",":"))
  if isinstance(x,list):return"["+",".join(e(i)for i in x)+"]"
  if isinstance(x,dict):
   if not all(isinstance(k,str)for k in x):raise CapsuleError("non-string object key")
   ks=sorted(x,key=lambda k:k.encode("utf-8"));return"{"+",".join(json.dumps(k,ensure_ascii=False,separators=(",",":"))+":"+e(x[k])for k in ks)+"}"
  raise CapsuleError(f"unsupported canonical type: {type(x).__name__}")
 return e(v).encode("utf-8")
def commitment(domain,v):
 b=canonical_json(v);return hashlib.sha256(domain+len(b).to_bytes(8,"big")+b).hexdigest()
def capsule_commitment(m):return commitment(CAPSULE_DOMAIN,m)
def receipt_commitment(r):return commitment(RECEIPT_DOMAIN,r)
def exact_keys(v,ks,w):
 if not isinstance(v,dict)or set(v)!=ks:raise CapsuleError(f"{w} keys mismatch")
def hex_value(v,p,w):
 if not isinstance(v,str)or not p.fullmatch(v):raise CapsuleError(f"{w} invalid")
 return v
def validate_profile(v,w):
 exact_keys(v,{"id","revision","digest"},w);r=v["revision"]
 if not isinstance(v["id"],str)or not IDENT.fullmatch(v["id"]):raise CapsuleError(f"{w}.id invalid")
 if not isinstance(r,int)or isinstance(r,bool)or r<1:raise CapsuleError(f"{w}.revision invalid")
 hex_value(v["digest"],H64,f"{w}.digest")
def validate_relpath(v,w):
 if not isinstance(v,str)or not v:raise CapsuleError(f"{w} invalid")
 p=PurePosixPath(v)
 if p.is_absolute()or".."in p.parts or"\\"in v or str(p)!=v:raise CapsuleError(f"{w} unsafe")
def sorted_unique_strings(v,w):
 if not isinstance(v,list)or not all(isinstance(x,str)for x in v):raise CapsuleError(f"{w} invalid")
 if v!=sorted(set(v),key=lambda x:x.encode("utf-8")):raise CapsuleError(f"{w} not sorted unique")
 return v
