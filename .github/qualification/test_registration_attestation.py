from __future__ import annotations
import base64,copy,hashlib,importlib.util,json,os,subprocess,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
S=importlib.util.spec_from_file_location("registration_attestation",HERE/"registration_attestation.py");assert S and S.loader
m=importlib.util.module_from_spec(S);S.loader.exec_module(m)
class Core:
 SCHEMA="mycelix-qualification-workflow-object-oracle-v1";ORACLE_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_V1\0";INTENT_DOMAIN=b"MYCELIX_QUALIFICATION_REGISTRATION_INTENT_V1\0"
 @staticmethod
 def canonical(v):return json.dumps(v,sort_keys=True,separators=(",",":"),ensure_ascii=False,allow_nan=False).encode()
 @classmethod
 def commitment(c,domain,v):return hashlib.sha256(domain+c.canonical(v)).hexdigest()
 @classmethod
 def validate_intent(c,i):return c.commitment(c.INTENT_DOMAIN,i)
class Policy:
 POLICY_DOMAIN=b"MYCELIX_QUALIFICATION_WORKFLOW_OBJECT_ORACLE_POLICY_V1\0"
 @classmethod
 def policy_commitment(c,v):return Core.commitment(c.POLICY_DOMAIN,v)
CORE=Core();POLICY=Policy()
def intent(**u):
 x={"schema":"mycelix-qualification-registration-intent-v1","intent_id":"amsap-r1","repository":"Luminous-Dynamics/mycelix","subject_sha":"1"*40,"predecessor_sha":"2"*40,"preflight_profile_id":"amsap-rustfmt-v1","preflight_profile_commitment":"3"*64,"preflight_implementation_commitment":"4"*64,"environment_adapter_implementation_commitment":"5"*64,"preflight_environment_commitment":"6"*64,"qualification_workflow_path":".github/workflows/amsap.yml","qualification_workflow_commit_sha":"7"*40,"qualification_workflow_blob_sha1":"8"*40,"registration_mode":"manual-request-v1"};x.update(u);return x
def policy(i,**u):
 ic=CORE.validate_intent(i);x={"schema":CORE.SCHEMA,"workflow_object_oracle_implementation_commitment":m.CORE_IMPL,"classification":"WORKFLOW_OBJECT_CHAIN_CONFIRMED","workflow_identity_verifier_implementation_commitment":"f932a24a11bb8d98c55b039fb0a1ddc951eacd64a159c03826d4af0675480761","workflow_identity_verification_commitment":"9"*64,"registration_intent_commitment":ic,"repository":i["repository"],"subject_sha":i["subject_sha"],"predecessor_sha":i["predecessor_sha"],"qualification_workflow_path":i["qualification_workflow_path"],"qualification_workflow_commit_sha":i["qualification_workflow_commit_sha"],"commit_bytes_sha256":"a"*64,"root_tree_sha1":"b"*40,"traversed_trees":[{"tree_sha1":"c"*40,"tree_sha256":"d"*64,"path_prefix":""}],"qualification_workflow_blob_sha1":i["qualification_workflow_blob_sha1"],"workflow_blob_bytes_sha256":"e"*64,"object_verification_method":"python-git-object-rehash-and-tree-traversal-v1","git":{"path":"/usr/bin/git","sha256":"f"*64,"version":"git version 2.51.0","object_format":"sha1"},"caller_head":"0"*40,"caller_status_sha256":"1"*64,"sha1_collision_resistance_claimed":False,"git_implementation_trust_verified":False,"receipt_authenticity_verified":False,"registration_authority":False,"workflow_dispatched":False,"workflow_identity_verified":True,"workflow_object_chain_confirmed":True,"qualification_result":None,"qualification_authority":False}
 x["workflow_object_oracle_commitment"]=CORE.commitment(CORE.ORACLE_DOMAIN,x);x["schema"]="mycelix-qualification-workflow-object-oracle-policy-v1";x.update({"workflow_object_oracle_policy_implementation_commitment":m.POLICY_IMPL,"workflow_object_oracle_core_git_blob_sha1":m.CORE_BLOB,"workflow_object_oracle_core_implementation_commitment":m.CORE_IMPL,"repository_identity_verified":True,"repository_origin":i["repository"]});x.update(u);x["workflow_object_oracle_policy_commitment"]=POLICY.policy_commitment(x);return x
def repolicy(x):
 x.pop("workflow_object_oracle_policy_commitment",None);x["workflow_object_oracle_policy_commitment"]=POLICY.policy_commitment(x)
def keypair(root,alg="ED25519"):
 root.mkdir(exist_ok=True);sk=root/"sk.pem";der=root/"pub.der";raw=root/"pub.raw"
 a=["openssl","genpkey","-algorithm",alg,"-out",str(sk)]
 if alg=="RSA":a=["openssl","genpkey","-algorithm","RSA","-pkeyopt","rsa_keygen_bits:2048","-out",str(sk)]
 if alg=="EC":a=["openssl","genpkey","-algorithm","EC","-pkeyopt","ec_paramgen_curve:P-256","-out",str(sk)]
 subprocess.run(a,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);subprocess.run(["openssl","pkey","-in",str(sk),"-pubout","-outform","DER","-out",str(der)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 b=der.read_bytes()
 if alg=="ED25519":assert len(b)==44 and b.startswith(m.SPKI);raw.write_bytes(b[len(m.SPKI):])
 else:raw.write_bytes(b)
 return sk,raw
def profile(pub,**u):
 ki=m.key_identity(pub);x={"schema":m.PROFILE_SCHEMA,"profile_id":"fixture-v1","signer_id":"operator:fixture","key_algorithm":"ed25519","public_key_spki_sha256":ki["spki_sha256"],"public_key_raw_sha256":ki["raw_sha256"],"attestation_context":m.CONTEXT};x.update(u);return x
def sign(sk,msg):
 with tempfile.TemporaryDirectory() as td:
  r=Path(td);(r/"m").write_bytes(msg);subprocess.run(["openssl","pkeyutl","-sign","-inkey",str(sk),"-rawin","-in",str(r/"m"),"-out",str(r/"s")],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);return (r/"s").read_bytes()
def bundle(root):
 sk,pk=keypair(root);pub=pk.read_bytes();i=intent();r=policy(i);p=profile(pub);pc=CORE.commitment(m.PROFILE_DOMAIN,p);pl=m.payload(r,i,pc);sig=sign(sk,m.MESSAGE_DOMAIN+CORE.canonical(pl));e={"schema":m.ENVELOPE_SCHEMA,"signer_profile_commitment":pc,"payload_commitment":CORE.commitment(m.PAYLOAD_DOMAIN,pl),"signature_base64":base64.b64encode(sig).decode()};return sk,pub,i,r,p,e
class T(unittest.TestCase):
 def setUp(self):
  self.t=tempfile.TemporaryDirectory();self.addCleanup(self.t.cleanup);self.root=Path(self.t.name);self.sk,self.pub,self.i,self.r,self.p,self.e=bundle(self.root);self.lp=patch.object(m,"load_parent",return_value=(POLICY,CORE));self.lp.start();self.addCleanup(self.lp.stop)
 def v(self,**k):return m.verify(k.get("r",self.r),k.get("i",self.i),k.get("p",self.p),k.get("key",self.pub),k.get("e",self.e))
 def test_exact(self):
  o=self.v();self.assertEqual(o["classification"],"SIGNATURE_VALID_UNDER_PROFILE");self.assertTrue(o["cryptographic_signature_verified"]);self.assertFalse(o["signer_authority_verified"]);self.assertFalse(o["receipt_authenticity_verified"]);self.assertFalse(o["registration_authority"]);self.assertFalse(o["workflow_dispatched"]);self.assertIsNone(o["qualification_result"])
 def test_mutated_signature(self):
  e=copy.deepcopy(self.e);b=bytearray(base64.b64decode(e["signature_base64"]));b[0]^=1;e["signature_base64"]=base64.b64encode(b).decode();self.assertRaises(m.Refused,self.v,e=e)
 def test_subject_and_workflow_transplants(self):
  for f,v in (("subject_sha","9"*40),("qualification_workflow_blob_sha1","9"*40),("qualification_workflow_commit_sha","a"*40)):
   x=copy.deepcopy(self.i);x[f]=v
   with self.subTest(f=f):self.assertRaises(m.Refused,self.v,i=x)
 def test_policy_and_core_commitments(self):
  x=copy.deepcopy(self.r);x["workflow_object_oracle_policy_commitment"]="0"*64;self.assertRaises(m.Invalid,self.v,r=x)
  x=copy.deepcopy(self.r);x["workflow_object_oracle_commitment"]="0"*64;repolicy(x);self.assertRaises(m.Invalid,self.v,r=x)
 def test_profile_transplant(self):
  p=copy.deepcopy(self.p);p["signer_id"]="operator:other";pc=CORE.commitment(m.PROFILE_DOMAIN,p);pl=m.payload(self.r,self.i,pc);e=copy.deepcopy(self.e);e["signer_profile_commitment"]=pc;e["payload_commitment"]=CORE.commitment(m.PAYLOAD_DOMAIN,pl);self.assertRaises(m.Refused,self.v,p=p,e=e)
 def test_other_key_and_non_ed25519(self):
  _,pk=keypair(self.root/"other");self.assertRaises(m.Refused,self.v,key=pk.read_bytes())
  for alg in ("RSA","EC"):
   _,pk=keypair(self.root/alg.lower(),alg)
   with self.subTest(alg=alg):self.assertRaises(m.Invalid,self.v,key=pk.read_bytes())
 def test_private_and_bad_key(self):
  self.assertRaises(m.Invalid,self.v,key=self.sk.read_bytes());self.assertRaises(m.Invalid,self.v,key=b"bad");self.assertRaises(m.Invalid,self.v,key=b"x"*33)
 def test_base64_and_length(self):
  e=copy.deepcopy(self.e);e["signature_base64"]+="\n";self.assertRaises(m.Invalid,self.v,e=e)
  e=copy.deepcopy(self.e);e["signature_base64"]=base64.b64encode(b"x"*63).decode();self.assertRaises(m.Invalid,self.v,e=e)
 def test_key_profile_hash_mismatch(self):
  p=copy.deepcopy(self.p);p["public_key_raw_sha256"]="0"*64;pc=CORE.commitment(m.PROFILE_DOMAIN,p);pl=m.payload(self.r,self.i,pc);e=copy.deepcopy(self.e);e["signer_profile_commitment"]=pc;e["payload_commitment"]=CORE.commitment(m.PAYLOAD_DOMAIN,pl);self.assertRaises(m.Refused,self.v,p=p,e=e)
 def test_bad_algorithm(self):
  p=copy.deepcopy(self.p);p["key_algorithm"]="rsa";self.assertRaises(m.Invalid,self.v,p=p)
 def test_parent_revision_and_authority_broadening(self):
  x=copy.deepcopy(self.r);x["workflow_object_oracle_policy_implementation_commitment"]="0"*64;repolicy(x);self.assertRaises(m.Invalid,self.v,r=x)
  for f,v in (("registration_authority",True),("workflow_dispatched",True),("qualification_authority",True),("qualification_result","PASS"),("receipt_authenticity_verified",True)):
   x=copy.deepcopy(self.r);x[f]=v
   inner=dict(x)
   for k in m.POLICY_ONLY:inner.pop(k,None)
   inner["schema"]=CORE.SCHEMA;inner.pop("workflow_object_oracle_commitment");x["workflow_object_oracle_commitment"]=CORE.commitment(CORE.ORACLE_DOMAIN,inner);repolicy(x)
   with self.subTest(f=f):self.assertRaises(m.Invalid,self.v,r=x)
 def test_envelope_commitment_mismatch(self):
  for f in ("signer_profile_commitment","payload_commitment"):
   e=copy.deepcopy(self.e);e[f]="0"*64
   with self.subTest(f=f):self.assertRaises(m.Refused,self.v,e=e)
 def test_openssl_unavailable_and_drift(self):
  with self.assertRaises(m.Unavailable):m.verify(self.r,self.i,self.p,self.pub,self.e,openssl_path="/missing/openssl")
  with patch.object(m,"same",side_effect=[True,False]):self.assertRaises(m.Invalid,self.v)
 def test_key_bytes_are_captured_once(self):
  p=self.root/"pub.pem";p.write_bytes(self.pub);b=p.read_bytes();p.write_bytes(b"changed");self.assertEqual(self.v(key=b)["classification"],"SIGNATURE_VALID_UNDER_PROFILE")
 def test_second_self_generated_profile_has_no_authority(self):
  sk,pk=keypair(self.root/"second");pub=pk.read_bytes();p=profile(pub,profile_id="second",signer_id="operator:anyone");pc=CORE.commitment(m.PROFILE_DOMAIN,p);pl=m.payload(self.r,self.i,pc);e={"schema":m.ENVELOPE_SCHEMA,"signer_profile_commitment":pc,"payload_commitment":CORE.commitment(m.PAYLOAD_DOMAIN,pl),"signature_base64":base64.b64encode(sign(sk,m.MESSAGE_DOMAIN+CORE.canonical(pl))).decode()};o=self.v(p=p,key=pub,e=e);self.assertTrue(o["cryptographic_signature_verified"]);self.assertFalse(o["signer_authority_verified"])
 def test_failure_ceiling(self):
  for c in ("REFUSED","INVALID","UNAVAILABLE"):
   o=m.failure(c,"x");self.assertFalse(o["registration_authority"]);self.assertFalse(o["workflow_dispatched"]);self.assertIsNone(o["qualification_result"]);self.assertFalse(o["signer_authority_verified"])
 def test_static_no_signing_or_dispatch(self):
  s=(HERE/"registration_attestation.py").read_text().lower()
  for x in ('"-sign"',"genpkey","import requests","import urllib","from github","actions/runs","create_workflow_dispatch","-outform","public.pem"):
   with self.subTest(x=x):self.assertNotIn(x,s)
 def test_commitments_move(self):
  pc=CORE.commitment(m.PROFILE_DOMAIN,self.p);a=CORE.commitment(m.PAYLOAD_DOMAIN,m.payload(self.r,self.i,pc));i=intent(subject_sha="9"*40);r=policy(i);b=CORE.commitment(m.PAYLOAD_DOMAIN,m.payload(r,i,pc));self.assertNotEqual(a,b)
 def test_004d1_field_surface_is_closed(self):
  x=copy.deepcopy(self.r);x["future_field"]="x";repolicy(x);self.assertRaises(m.Invalid,self.v,r=x)
  x=copy.deepcopy(self.r);x.pop("caller_head");repolicy(x);self.assertRaises(m.Invalid,self.v,r=x)
 def test_bounded_public_key_read(self):
  p=self.root/"key.raw";p.write_bytes(b"x"*33);self.assertRaises(m.Invalid,m.read_key,p)
  p.write_bytes(b"x"*31);self.assertRaises(m.Invalid,m.read_key,p)
  p.write_bytes(self.pub);self.assertEqual(m.read_key(p),self.pub)
 def test_parent_loader_fails_closed_without_exact_siblings(self):
  self.lp.stop();old=m.__file__
  try:
   d=self.root/"loader";d.mkdir();m.__file__=str(d/"registration_attestation.py")
   self.assertRaises(m.Unavailable,m.load_parent)
   (d/"workflow_object_oracle_policy.py").write_text("x=1\n")
   self.assertRaises(m.Invalid,m.load_parent)
  finally:
   m.__file__=old;self.lp.start()
 def test_impl_commitment(self):
  b=(HERE/"registration_attestation.py").read_bytes();self.assertEqual(m.IMPLEMENTATION_COMMITMENT,hashlib.sha256(m.IMPL_DOMAIN+b).hexdigest())
if __name__=="__main__":unittest.main()