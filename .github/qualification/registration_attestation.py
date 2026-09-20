#!/usr/bin/env python3
"""Verify detached Ed25519 attestation over the exact EVIDENCE-CI-004D1 package."""
from __future__ import annotations
import argparse,base64,hashlib,importlib.util,json,os,re,shutil,subprocess,tempfile
from pathlib import Path
from typing import Any
SCHEMA="mycelix-qualification-registration-attestation-verification-v1"; PROFILE_SCHEMA="mycelix-qualification-registration-attestation-signer-profile-v1"; ENVELOPE_SCHEMA="mycelix-qualification-registration-attestation-envelope-v1"; PAYLOAD_SCHEMA="mycelix-qualification-registration-attestation-payload-v1"; CONTEXT="mycelix-qualification-registration-attestation-v1"
POLICY_BLOB="7d13efb52a39a57969560f6e36d17b39ba82c65a"; POLICY_IMPL="d6e5fd5d821f9122f652f7e7a5f0233e1cf3e7821cc42f202811090d5801c373"; CORE_IMPL="bea3334619f757905e072182ded806f507faa6b1cee68080104fb6e0087b2390"; CORE_BLOB="13ac8bf1ce33e686a09b54ee97e2c1a6b9c4e736"
PROFILE_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_ATTESTATION_SIGNER_PROFILE_V1\0"; PAYLOAD_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_ATTESTATION_PAYLOAD_V1\0"; MESSAGE_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_ATTESTATION_MESSAGE_V1\0"; RESULT_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_ATTESTATION_VERIFICATION_V1\0"; IMPL_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_ATTESTATION_IMPLEMENTATION_V1\0"; POLICY_IMPL_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_POLICY_IMPLEMENTATION_V1\0"
RAW_KEY_LEN=32; MAX_OUT=1048576; H64=re.compile(r"^[0-9a-f]{64}$"); ID=re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,255}$"); SPKI=bytes.fromhex("302a300506032b6570032100")
PROFILE_FIELDS=set("schema profile_id signer_id key_algorithm public_key_spki_sha256 public_key_raw_sha256 attestation_context".split()); ENVELOPE_FIELDS=set("schema signer_profile_commitment payload_commitment signature_base64".split())
POLICY_ONLY=set("workflow_object_oracle_policy_implementation_commitment workflow_object_oracle_core_git_blob_sha1 workflow_object_oracle_core_implementation_commitment repository_identity_verified repository_origin workflow_object_oracle_policy_commitment".split())

POLICY_FIELDS=set("schema workflow_object_oracle_implementation_commitment classification workflow_identity_verifier_implementation_commitment workflow_identity_verification_commitment registration_intent_commitment repository subject_sha predecessor_sha qualification_workflow_path qualification_workflow_commit_sha commit_bytes_sha256 root_tree_sha1 traversed_trees qualification_workflow_blob_sha1 workflow_blob_bytes_sha256 object_verification_method git caller_head caller_status_sha256 sha1_collision_resistance_claimed git_implementation_trust_verified receipt_authenticity_verified registration_authority workflow_dispatched workflow_identity_verified workflow_object_chain_confirmed qualification_result qualification_authority workflow_object_oracle_commitment workflow_object_oracle_policy_implementation_commitment workflow_object_oracle_core_git_blob_sha1 workflow_object_oracle_core_implementation_commitment repository_identity_verified repository_origin workflow_object_oracle_policy_commitment".split())
_B=Path(__file__).read_bytes(); IMPLEMENTATION_COMMITMENT=hashlib.sha256(IMPL_DOMAIN+_B).hexdigest(); del _B
class Invalid(RuntimeError):pass
class Refused(RuntimeError):pass
class Unavailable(RuntimeError):pass
def git_blob(b:bytes)->str:return hashlib.sha1(b"blob "+str(len(b)).encode()+b"\0"+b).hexdigest()
def sha256_file(p:Path)->str:
 h=hashlib.sha256();
 with p.open("rb") as f:
  for b in iter(lambda:f.read(1048576),b""):h.update(b)
 return h.hexdigest()
def load_parent():
 p=Path(__file__).with_name("workflow_object_oracle_policy.py")
 try:b=p.read_bytes()
 except OSError as e:raise Unavailable("004D1 policy implementation unavailable") from e
 if git_blob(b)!=POLICY_BLOB:raise Invalid("004D1 policy Git blob mismatch")
 if hashlib.sha256(POLICY_IMPL_DOMAIN+b).hexdigest()!=POLICY_IMPL:raise Invalid("004D1 policy implementation mismatch")
 s=importlib.util.spec_from_file_location("workflow_object_oracle_policy_parent",p)
 if s is None or s.loader is None:raise Invalid("cannot load 004D1 policy implementation")
 mod=importlib.util.module_from_spec(s)
 try:s.loader.exec_module(mod)
 except Exception as e:raise Invalid("cannot import pinned 004D1 policy implementation") from e
 if getattr(mod,"IMPLEMENTATION_COMMITMENT",None)!=POLICY_IMPL:raise Invalid("loaded 004D1 policy identity mismatch")
 try:core,cp=mod.load_core()
 except Exception as e:raise Invalid("cannot load pinned 004D1 core") from e
 if getattr(core,"IMPLEMENTATION_COMMITMENT",None)!=CORE_IMPL or git_blob(cp.read_bytes())!=CORE_BLOB:raise Invalid("004D1 core identity mismatch")
 return mod,core
def h64(v:object,label:str)->str:
 if not isinstance(v,str) or not H64.fullmatch(v):raise Invalid(f"invalid {label}")
 return v
def validate_policy(policy:Any,core:Any,r:dict[str,Any],i:dict[str,Any],ic:str)->str:
 if set(r)!=POLICY_FIELDS:raise Invalid("004D1 policy result field set mismatch")
 required={"workflow_object_oracle_policy_implementation_commitment":POLICY_IMPL,"workflow_object_oracle_core_implementation_commitment":CORE_IMPL,"workflow_object_oracle_core_git_blob_sha1":CORE_BLOB,"classification":"WORKFLOW_OBJECT_CHAIN_CONFIRMED","repository_identity_verified":True,"workflow_identity_verified":True,"workflow_object_chain_confirmed":True,"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"qualification_result":None,"qualification_authority":False,"git_implementation_trust_verified":False,"sha1_collision_resistance_claimed":False}
 if r.get("schema")!="mycelix-qualification-workflow-object-oracle-policy-v1":raise Invalid("004D1 policy schema mismatch")
 for k,v in required.items():
  if r.get(k)!=v:raise Invalid(f"004D1 policy boundary mismatch: {k}")
 if r.get("registration_intent_commitment")!=ic:raise Refused("004D1 intent commitment mismatch")
 for k in ("repository","subject_sha","predecessor_sha","qualification_workflow_path","qualification_workflow_commit_sha","qualification_workflow_blob_sha1"):
  if r.get(k)!=i.get(k):raise Refused(f"004D1 field mismatch: {k}")
 if not isinstance(r.get("repository_origin"),str) or r["repository_origin"].lower()!=i["repository"].lower():raise Refused("004D1 repository origin mismatch")
 pc=h64(r.get("workflow_object_oracle_policy_commitment"),"004D1 policy commitment"); pm=dict(r);pm.pop("workflow_object_oracle_policy_commitment")
 if policy.policy_commitment(pm)!=pc:raise Invalid("004D1 policy commitment mismatch")
 inner=dict(r)
 for k in POLICY_ONLY:inner.pop(k,None)
 inner["schema"]=core.SCHEMA; oc=h64(inner.pop("workflow_object_oracle_commitment",None),"004D1 core commitment")
 if core.commitment(core.ORACLE_DOMAIN,inner)!=oc:raise Invalid("004D1 core commitment mismatch")
 return pc
def validate_profile(core:Any,p:dict[str,Any])->str:
 if set(p)!=PROFILE_FIELDS or p.get("schema")!=PROFILE_SCHEMA:raise Invalid("signer profile shape/schema mismatch")
 for k in ("profile_id","signer_id"):
  if not isinstance(p.get(k),str) or not ID.fullmatch(p[k]):raise Invalid(f"invalid signer profile {k}")
 if p.get("key_algorithm")!="ed25519":raise Invalid("unsupported signer key algorithm")
 h64(p.get("public_key_spki_sha256"),"signer SPKI SHA-256");h64(p.get("public_key_raw_sha256"),"signer raw-key SHA-256")
 if p.get("attestation_context")!=CONTEXT:raise Invalid("unsupported attestation context")
 return core.commitment(PROFILE_DOMAIN,p)
def payload(r:dict[str,Any],i:dict[str,Any],pc:str)->dict[str,Any]:
 return {"schema":PAYLOAD_SCHEMA,"workflow_object_oracle_policy_implementation_commitment":POLICY_IMPL,"workflow_object_oracle_policy_commitment":r["workflow_object_oracle_policy_commitment"],"workflow_object_oracle_core_implementation_commitment":CORE_IMPL,"workflow_object_oracle_commitment":r["workflow_object_oracle_commitment"],"workflow_identity_verification_commitment":r["workflow_identity_verification_commitment"],"registration_intent_commitment":r["registration_intent_commitment"],"signer_profile_commitment":pc,"repository":i["repository"],"subject_sha":i["subject_sha"],"predecessor_sha":i["predecessor_sha"],"qualification_workflow_path":i["qualification_workflow_path"],"qualification_workflow_commit_sha":i["qualification_workflow_commit_sha"],"qualification_workflow_blob_sha1":i["qualification_workflow_blob_sha1"],"commit_bytes_sha256":r["commit_bytes_sha256"],"workflow_blob_bytes_sha256":r["workflow_blob_bytes_sha256"],"repository_identity_verified":True,"workflow_identity_verified":True,"workflow_object_chain_confirmed":True,"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"qualification_result":None,"qualification_authority":False}
def envelope(e:dict[str,Any])->bytes:
 if set(e)!=ENVELOPE_FIELDS or e.get("schema")!=ENVELOPE_SCHEMA:raise Invalid("attestation envelope shape/schema mismatch")
 h64(e.get("signer_profile_commitment"),"signer profile commitment");h64(e.get("payload_commitment"),"payload commitment");s=e.get("signature_base64")
 if not isinstance(s,str) or not s or any(c.isspace() for c in s):raise Invalid("invalid signature base64")
 try:b=base64.b64decode(s.encode("ascii"),validate=True)
 except (ValueError,UnicodeEncodeError) as x:raise Invalid("invalid signature base64") from x
 if base64.b64encode(b).decode()!=s:raise Invalid("noncanonical signature base64")
 if len(b)!=64:raise Invalid("Ed25519 signature must be 64 bytes")
 return b
def env(home:Path,exe:Path)->dict[str,str]:home.mkdir(parents=True,exist_ok=True);return {"PATH":str(exe.parent),"HOME":str(home),"LC_ALL":"C","LANG":"C","TZ":"UTC","OPENSSL_CONF":os.devnull}
def run(exe:str,e:dict[str,str],args:list[str],fail=False)->bytes|None:
 try:c=subprocess.run([exe,*args],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=e,timeout=30)
 except (OSError,subprocess.TimeoutExpired) as x:raise Unavailable("OpenSSL invocation unavailable") from x
 if len(c.stdout)>MAX_OUT or len(c.stderr)>MAX_OUT:raise Unavailable("OpenSSL output exceeded size bound")
 if c.returncode:
  if fail:return None
  raise Invalid("OpenSSL operation failed")
 return c.stdout
def bind_openssl(explicit:str|None=None)->dict[str,str]:
 x=explicit or shutil.which("openssl")
 if not x:raise Unavailable("OpenSSL executable unavailable")
 try:p=Path(x).resolve(strict=True)
 except (OSError,RuntimeError) as e:raise Unavailable("OpenSSL executable unavailable") from e
 if not p.is_file() or not os.access(p,os.X_OK):raise Unavailable("OpenSSL executable is not executable")
 out={"path":str(p),"sha256":sha256_file(p)}
 with tempfile.TemporaryDirectory(prefix="mycelix-attest-openssl-") as td:v=run(str(p),env(Path(td)/"home",p),["version"])
 try:vs=v.decode("utf-8","strict").strip()
 except UnicodeDecodeError as e:raise Invalid("OpenSSL version is not UTF-8") from e
 if not vs.startswith("OpenSSL "):raise Invalid("unexpected OpenSSL version output")
 out["version"]=vs;return out
def same(o:dict[str,str])->bool:
 try:p=Path(o["path"]);return p.resolve(strict=True)==p and p.is_file() and os.access(p,os.X_OK) and sha256_file(p)==o["sha256"]
 except (OSError,RuntimeError,KeyError):return False
def read_key(path:Path)->bytes:
 try:
  with path.open("rb") as f:b=f.read(RAW_KEY_LEN+1)
 except OSError as e:raise Invalid("cannot read raw Ed25519 public key") from e
 if len(b)!=RAW_KEY_LEN:raise Invalid("raw Ed25519 public key must be exactly 32 bytes")
 return b
def key_identity(b:bytes)->dict[str,str]:
 if len(b)!=RAW_KEY_LEN:raise Invalid("raw Ed25519 public key must be exactly 32 bytes")
 der=SPKI+b
 return {"spki_sha256":hashlib.sha256(der).hexdigest(),"raw_sha256":hashlib.sha256(b).hexdigest()}
def sig_ok(o:dict[str,str],key:bytes,msg:bytes,sig:bytes)->bool:
 if len(key)!=RAW_KEY_LEN:raise Invalid("raw Ed25519 public key must be exactly 32 bytes")
 with tempfile.TemporaryDirectory(prefix="mycelix-attest-verify-") as td:
  r=Path(td);k=r/"public.der";m=r/"message.bin";s=r/"signature.bin";k.write_bytes(SPKI+key);m.write_bytes(msg);s.write_bytes(sig);x=run(o["path"],env(r/"home",Path(o["path"])),["pkeyutl","-verify","-pubin","-keyform","DER","-inkey",str(k),"-rawin","-in",str(m),"-sigfile",str(s)],True)
 return x is not None
def verify(r:dict[str,Any],i:dict[str,Any],p:dict[str,Any],key:bytes,e:dict[str,Any],openssl_path:str|None=None)->dict[str,Any]:
 policy,core=load_parent(); ic=core.validate_intent(i); polc=validate_policy(policy,core,r,i,ic); pc=validate_profile(core,p); sig=envelope(e); pl=payload(r,i,pc); plc=core.commitment(PAYLOAD_DOMAIN,pl)
 if e["signer_profile_commitment"]!=pc:raise Refused("attestation signer profile commitment mismatch")
 if e["payload_commitment"]!=plc:raise Refused("attestation payload commitment mismatch")
 o=bind_openssl(openssl_path);ki=key_identity(key)
 if ki["spki_sha256"]!=p["public_key_spki_sha256"]:raise Refused("public key SPKI identity does not match signer profile")
 if ki["raw_sha256"]!=p["public_key_raw_sha256"]:raise Refused("public key raw identity does not match signer profile")
 if not same(o):raise Invalid("OpenSSL executable drift before signature verification")
 msg=MESSAGE_DOMAIN+core.canonical(pl)
 if not sig_ok(o,key,msg,sig):raise Refused("detached Ed25519 signature mismatch")
 if not same(o):raise Invalid("OpenSSL executable drift after signature verification")
 out={"schema":SCHEMA,"registration_attestation_implementation_commitment":IMPLEMENTATION_COMMITMENT,"classification":"SIGNATURE_VALID_UNDER_PROFILE","workflow_object_oracle_policy_implementation_commitment":POLICY_IMPL,"workflow_object_oracle_policy_commitment":polc,"registration_intent_commitment":ic,"signer_profile_commitment":pc,"signed_payload_commitment":plc,"signature_sha256":hashlib.sha256(sig).hexdigest(),"profile_id":p["profile_id"],"signer_id":p["signer_id"],"public_key_spki_sha256":ki["spki_sha256"],"public_key_raw_sha256":ki["raw_sha256"],"public_key_input_sha256":hashlib.sha256(key).hexdigest(),"signed_message_sha256":hashlib.sha256(msg).hexdigest(),"openssl":o,"repository":i["repository"],"subject_sha":i["subject_sha"],"predecessor_sha":i["predecessor_sha"],"qualification_workflow_path":i["qualification_workflow_path"],"qualification_workflow_commit_sha":i["qualification_workflow_commit_sha"],"qualification_workflow_blob_sha1":i["qualification_workflow_blob_sha1"],"repository_identity_verified":True,"workflow_identity_verified":True,"workflow_object_chain_confirmed":True,"cryptographic_signature_verified":True,"signer_profile_bound":True,"signer_authority_verified":False,"receipt_authenticity_verified":False,"preflight_receipt_authenticity_verified":False,"crypto_implementation_trust_verified":False,"registration_authority":False,"workflow_dispatched":False,"qualification_result":None,"qualification_authority":False}
 out["attestation_verification_commitment"]=core.commitment(RESULT_DOMAIN,out);return out
def failure(c:str,reason:str)->dict[str,Any]:return {"schema":SCHEMA,"registration_attestation_implementation_commitment":IMPLEMENTATION_COMMITMENT,"classification":c,"reason":reason,"cryptographic_signature_verified":False,"signer_profile_bound":False,"signer_authority_verified":False,"receipt_authenticity_verified":False,"preflight_receipt_authenticity_verified":False,"crypto_implementation_trust_verified":False,"registration_authority":False,"workflow_dispatched":False,"qualification_result":None,"qualification_authority":False}
def main()->int:
 p=argparse.ArgumentParser(description=__doc__);p.add_argument("--oracle-policy-result",required=True);p.add_argument("--intent",required=True);p.add_argument("--signer-profile",required=True);p.add_argument("--public-key",required=True);p.add_argument("--attestation",required=True);a=p.parse_args()
 try:
  policy,core=load_parent();key=read_key(Path(a.public_key));out=verify(core.load(Path(a.oracle_policy_result),"004D1 policy result"),core.load(Path(a.intent),"registration intent"),core.load(Path(a.signer_profile),"signer profile"),key,core.load(Path(a.attestation),"attestation envelope"))
 except Refused as x:out=failure("REFUSED",str(x))
 except Unavailable as x:out=failure("UNAVAILABLE",str(x))
 except (Invalid,OSError) as x:out=failure("INVALID",str(x))
 print(json.dumps(out,sort_keys=True,separators=(",",":"),ensure_ascii=False));return {"SIGNATURE_VALID_UNDER_PROFILE":0,"REFUSED":2,"UNAVAILABLE":3,"INVALID":4}.get(out["classification"],4)
if __name__=="__main__":raise SystemExit(main())