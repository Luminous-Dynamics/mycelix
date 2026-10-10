#!/usr/bin/env python3
from __future__ import annotations
import argparse, base64, copy, hashlib, json, subprocess, tempfile
from pathlib import Path

SCHEMA="SYM-CIVIC-019-RECEIPT-PROOF-BINDING-V1"
CORPUS_SCHEMA="SYM-CIVIC-019-RECEIPT-PROOF-BINDING-CORPUS-V1"
EXPECTED_MUTATION_CASE_COUNT=25
MAX_SYNTHETIC_TREE_SIZE = 1 << 62
TAG_COSE_SIGN1=18
ALG_EDDSA=-8
VDS_RFC9162_SHA256=1
VDP_INCLUSION=-1
HP_ALG=1; HP_KID=4; HP_CWT=15; HP_X5CHAIN=33; HP_X5T=34; HP_RECEIPTS=394; HP_VDS=395; HP_VDP=396
CWT_ISS=1; CWT_SUB=2; HP_POLICY=1000; HP_BILATERAL=1001

class Reject(Exception): pass
class Unresolved(Exception): pass

class Node:
    __slots__=("value","raw","children")
    def __init__(self,value,raw,children=()):
        self.value=value; self.raw=raw; self.children=tuple(children)

def H(b): return hashlib.sha256(b).digest()
def hx(b): return H(b).hex()

class Reader:
    def __init__(self,b): self.b=b; self.i=0
    def parse(self):
        n=self.item()
        if self.i!=len(self.b): raise Reject("trailing CBOR")
        return n
    def head(self):
        b=self.b; s=self.i
        if s>=len(b): raise Reject("truncated CBOR")
        x=b[self.i]; self.i+=1; mt=x>>5; ai=x&31
        if ai<24:return mt,ai,s
        if ai==24:n=1
        elif ai==25:n=2
        elif ai==26:n=4
        elif ai==27:n=8
        else: raise Reject("indefinite/reserved CBOR")
        if self.i+n>len(b): raise Reject("truncated CBOR argument")
        v=int.from_bytes(b[self.i:self.i+n],"big"); self.i+=n
        if (ai==24 and v<24) or (ai==25 and v<256) or (ai==26 and v<65536) or (ai==27 and v<2**32):
            raise Reject("non-minimal CBOR additional-information encoding")
        return mt,v,s
    def item(self):
        mt,n,s=self.head(); b=self.b
        if mt==0:return Node(n,b[s:self.i])
        if mt==1:return Node(-1-n,b[s:self.i])
        if mt in (2,3):
            if self.i+n>len(b): raise Reject("truncated string")
            c=b[self.i:self.i+n]; self.i+=n
            if mt==2:v=bytes(c)
            else:
                try:v=c.decode()
                except UnicodeDecodeError as e: raise Reject("bad UTF-8") from e
            return Node(v,b[s:self.i])
        if mt==4:
            ch=[self.item() for _ in range(n)]
            return Node(tuple(x.value for x in ch),b[s:self.i],ch)
        if mt==5:
            ch=[]
            seen=[]
            for _ in range(n):
                k=self.item(); v=self.item()
                if any(eq(k.value,z.value) for z in seen): raise Reject("duplicate CBOR map key")
                seen.append(k); ch += [k,v]
            return Node(tuple((ch[i].value,ch[i+1].value) for i in range(0,len(ch),2)),b[s:self.i],ch)
        if mt==6:
            return Node(("tag",n),b[s:self.i] if False else b[s:self.i], [self.item()])  # replaced below
        if mt==7 and n in (20,21,22): return Node({20:False,21:True,22:None}[n],b[s:self.i])
        raise Reject("unsupported CBOR type")

# Patch tag handling without a second parser: the tag raw span must include the child.
_old_item=Reader.item
def _item(self):
    mt,n,s=self.head(); b=self.b
    if mt==6:
        child=self.item()
        return Node(("tag",n,child.value),b[s:self.i],[child])
    if mt==0:return Node(n,b[s:self.i])
    if mt==1:return Node(-1-n,b[s:self.i])
    if mt in (2,3):
        if self.i+n>len(b): raise Reject("truncated string")
        c=b[self.i:self.i+n]; self.i+=n
        if mt==2:v=bytes(c)
        else:
            try:v=c.decode()
            except UnicodeDecodeError as e: raise Reject("bad UTF-8") from e
        return Node(v,b[s:self.i])
    if mt==4:
        ch=[self.item() for _ in range(n)]; return Node(tuple(x.value for x in ch),b[s:self.i],ch)
    if mt==5:
        ch=[]; seen=[]
        for _ in range(n):
            k=self.item(); v=self.item()
            if any(eq(k.value,z.value) for z in seen): raise Reject("duplicate CBOR map key")
            seen.append(k); ch += [k,v]
        return Node(tuple((ch[i].value,ch[i+1].value) for i in range(0,len(ch),2)),b[s:self.i],ch)
    if mt==7 and n in (20,21,22): return Node({20:False,21:True,22:None}[n],b[s:self.i])
    raise Reject("unsupported CBOR type")
Reader.item=_item

def eq(a,b): return type(a) is type(b) and a==b

def git_blob_sha(data):
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\x00" + data).hexdigest()

def require_exact_keys(obj, expected, what):
    if not isinstance(obj, dict) or set(obj) != set(expected):
        raise Reject(what + " exact-key schema mismatch")

def assert_surface_bindings(corpus_path):
    root = Path(__file__).resolve().parents[2]
    manifest_path = root / "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_manifest_v1.json"
    surface = json.loads(manifest_path.read_text())
    if surface.get("schema") != "SYM-CIVIC-019-RECEIPT-PROOF-BINDING-MANIFEST-V1":
        raise Reject("qualification surface manifest schema mismatch")
    bindings = surface.get("files_git_blob_sha")
    expected = {
        "verifier": "scripts/qualification/sym_civic_019_receipt_proof_binding_v1.py",
        "corpus": "mycelix-workspace/docs/civic-resilience/sym_civic_019_receipt_proof_binding_v1.json",
        "workflow": ".github/workflows/sym-civic-019-receipt-proof-binding.yml",
        "external_interop_verifier": "scripts/qualification/sym_civic_019_scitt_cose_interop_v1.py",
        "external_interop_manifest": "mycelix-workspace/docs/civic-resilience/scitt-cose-v1/manifest.json",
    }
    if not isinstance(bindings, dict) or set(bindings) != set(expected):
        raise Reject("qualification surface manifest file-binding schema mismatch")
    if type(surface.get("mutation_case_count")) is not int or surface["mutation_case_count"] != EXPECTED_MUTATION_CASE_COUNT:
        raise Reject("qualification surface mutation-case count differs from verifier constant")
    for label, rel in expected.items():
        path = root / rel
        if not path.is_file():
            raise Reject("qualification surface file missing: " + rel)
        if git_blob_sha(path.read_bytes()) != bindings[label]:
            raise Reject("qualification surface file binding mismatch: " + label)

def pairs(n):
    if len(n.children)%2: raise Reject("not a map")
    return [(n.children[i],n.children[i+1]) for i in range(0,len(n.children),2)]
def get(n,k,req=True):
    for a,b in pairs(n):
        if type(a.value) is int and a.value==k:return b
    if req: raise Reject(f"missing map key {k}")
    return None
def arr(n,w):
    if not n.raw or (n.raw[0] >> 5) != 4 or not isinstance(n.value,tuple): raise Reject(w+" must be array")
    return n
def mp(n,w):
    if not n.raw or (n.raw[0] >> 5) != 5 or not isinstance(n.value,tuple) or len(n.children)%2: raise Reject(w+" must be map")
    return n
def bi(n,w):
    if not isinstance(n.value,bytes): raise Reject(w+" must be bstr")
    return n.value
def ti(n,w):
    if not isinstance(n.value,str): raise Reject(w+" must be tstr")
    return n.value
def ii(n,w):
    if type(n.value) is not int: raise Reject(w+" must be integer")
    return n.value

def chead(mt,n):
    if n<24:return bytes([mt<<5|n])
    if n<256:return bytes([mt<<5|24,n])
    if n<65536:return bytes([mt<<5|25])+n.to_bytes(2,"big")
    if n<2**32:return bytes([mt<<5|26])+n.to_bytes(4,"big")
    return bytes([mt<<5|27])+n.to_bytes(8,"big")
def cu(n): return chead(0,n)
def ci(n): return cu(n) if n>=0 else chead(1,-1-n)
def bs(b): return chead(2,len(b))+b
def ts(s): b=s.encode(); return chead(3,len(b))+b
def ar(xs): return chead(4,len(xs))+b"".join(xs)
def mpraw(xs): return chead(5,len(xs))+b"".join(ci(k)+v for k,v in xs)
def tag18(b): return chead(6,18)+b

def sig_structure(prot,payload): return ar([ts("Signature1"),bs(prot),bs(b""),payload])

def sign1(raw,what):
    top=Reader(raw).parse()
    if not (isinstance(top.value,tuple) and len(top.value)==3 and top.value[:2]==("tag",18)): raise Reject(what+" is not tagged COSE_Sign1")
    body=arr(top.children[0],what+" body")
    if len(body.children)!=4: raise Reject(what+" body arity")
    prot=bi(body.children[0],what+" protected"); uh=mp(body.children[1],what+" unprotected")
    payload=body.children[2]
    if payload.value is not None and not isinstance(payload.value,bytes): raise Reject(what+" payload")
    sig=bi(body.children[3],what+" signature")
    ph=mp(Reader(prot).parse(),what+" protected map")
    if {k.value for k,_ in pairs(ph)} & {k.value for k,_ in pairs(uh)}: raise Reject(what+" cross-bucket duplicate")
    if get(uh,2,False) is not None: raise Reject(what+" crit must be protected")
    critn=get(ph,2,False)
    if critn is not None:
        crit=arr(critn,what+" crit")
        labels=[x.value for x in crit.children]
        if not labels: raise Reject(what+" crit must be non-empty")
        if len(set(labels))!=len(labels): raise Reject(what+" duplicate crit label")
        protected_labels={k.value for k,_ in pairs(ph)}
        supported={1,2,4,15,395,396}
        for label in labels:
            if label not in protected_labels: raise Reject(what+" critical label not protected")
            if label not in supported: raise Reject(what+" unsupported critical header")
    pld=bs(payload.value) if payload.value is not None else b"\xf6"
    return {"raw":raw,"protected":prot,"pm":ph,"um":uh,"payload":payload.value,"sig":sig,
            "sig_structure":sig_structure(prot,pld)}

def claims(n,what):
    m=mp(n,what+" claims")
    return ti(get(m,1),what+" iss"),ti(get(m,2),what+" sub")

def statement_claims(raw):
    s=sign1(raw,"Signed Statement")
    h=profile_header(s,"Signed Statement",False)
    return h["iss"],h["sub"]

def profile_header(s,what,vds_required):
    ph=s["pm"]; alg=ii(get(ph,HP_ALG),what+" alg")
    if alg!=ALG_EDDSA: raise Reject(what+" alg mismatch")
    kidn=get(ph,HP_KID,False); x5t=get(ph,HP_X5T,False); x5c=get(ph,HP_X5CHAIN,False)
    if kidn is None and x5t is None and x5c is None: raise Reject(what+" missing kid without x5t/x5chain")
    kid=bi(kidn,what+" kid") if kidn is not None else None
    iss,sub=claims(get(ph,HP_CWT),what)
    vds=ii(get(ph,HP_VDS),what+" vds") if vds_required else None
    if vds_required and vds!=VDS_RFC9162_SHA256: raise Reject("unregistered/unsupported VDS")
    return {"alg":alg,"kid":kid,"iss":iss,"sub":sub,"vds":vds}

def key_pem(reg,kid):
    m=[x for x in reg if x["raw_kid_hex"]==kid.hex()]
    if len(m)!=1: raise Reject("TS key resolution is not unique")
    return m[0]["public_key_pem"].encode()

def openssl_verify(pem,msg,sig):
    with tempfile.TemporaryDirectory(prefix="mycelix019-") as td:
        k=Path(td)/"k.pem"; i=Path(td)/"m"; s=Path(td)/"s"
        k.write_bytes(pem); i.write_bytes(msg); s.write_bytes(sig)
        p=subprocess.run(["openssl","pkeyutl","-verify","-pubin","-inkey",str(k),"-rawin","-in",str(i),"-sigfile",str(s)],
                         text=True,capture_output=True)
        if p.returncode==0:return True
        if "Signature Verification Failure" in p.stdout+p.stderr:return False
        raise Unresolved("openssl verification backend failure")

def inclusion_a(leaf,idx,size,path):
    if size<1 or idx<0 or idx>=size: raise Reject("leaf_index >= tree_size")
    cur=leaf; fn=idx; sn=size-1; p=0
    while sn:
        need=(fn&1) or fn<sn
        if need:
            if p>=len(path): raise Reject("inclusion path exhausted")
            cur=H(b"\x01"+path[p]+cur) if (fn&1) else H(b"\x01"+cur+path[p]); p+=1
        fn//=2; sn//=2
    if p!=len(path): raise Reject("unused inclusion nodes")
    return cur

def inclusion_b(leaf,idx,size,path):
    if not size or not (0<=idx<size): raise Reject("reference leaf range")
    cur=leaf; a=idx; b=size-1; j=0
    while b:
        take=(a%2==1) or a<b
        if take:
            if j==len(path): raise Reject("reference short path")
            cur=H(b"\x01"+path[j]+cur) if a%2 else H(b"\x01"+cur+path[j]); j+=1
        a//=2; b//=2
    if j!=len(path): raise Reject("reference long path")
    return cur

def receipt_a(raw,stmt,corpus):
    s=sign1(raw,"Receipt"); h=profile_header(s,"Receipt",True)
    vd=mp(get(s["um"],HP_VDP),"Receipt VDP"); pa=arr(get(vd,VDP_INCLUSION),"Receipt inclusion")
    if len(pa.children)!=1: raise Reject("synthetic profile requires one proof")
    pb=bi(pa.children[0],"Receipt proof"); pn=arr(Reader(pb).parse(),"Receipt proof")
    if len(pn.children)!=3: raise Reject("proof arity")
    size=ii(pn.children[0],"tree_size"); idx=ii(pn.children[1],"leaf_index"); pathn=arr(pn.children[2],"path")
    if size > MAX_SYNTHETIC_TREE_SIZE: raise Reject("tree_size exceeds synthetic profile ceiling")
    path=[bi(x,"path node") for x in pathn.children]
    root=s["payload"]
    if root is None or len(root)!=32: raise Reject("root must be SHA-256 bstr")
    key_matches=[x for x in corpus["ts_key_registry"] if x["raw_kid_hex"]==h["kid"].hex()]
    if len(key_matches)!=1: raise Reject("TS key resolution is not unique")
    ts_key=key_matches[0]
    statement_iss,statement_sub=statement_claims(stmt)
    if h["iss"]!=ts_key["issuer"]: raise Reject("Receipt issuer != selected TS issuer")
    if h["sub"]!=statement_sub: raise Reject("Receipt subject != Signed Statement subject")
    pem=ts_key["public_key_pem"].encode()
    entry=bytes.fromhex(corpus["vds_entry_bytes_hex"])
    if entry!=stmt: raise Reject("synthetic VDS entry differs from Signed Statement bytes")
    leaf=H(b"\x00"+entry); calc=inclusion_a(leaf,idx,size,path)
    if calc!=root: raise Reject("proof_to_root mismatch")
    if not openssl_verify(pem,s["sig_structure"],s["sig"]): raise Reject("signature_to_root failure")
    return {"proof_to_root":"PASS","signature_to_root":"PASS","vds_id":h["vds"],"vdp_proof_type":VDP_INCLUSION,
            "vdp_entry_bytes_sha256":hx(pb),"leaf_input_sha256":hx(stmt),"leaf_commitment":H(b"\x00"+stmt).hex(),
            "tree_size":size,"leaf_index":idx,"root_hash":root.hex(),"receipt_bytes_sha256":hx(raw),
            "receipt_protected_bstr_sha256":hx(s["protected"])}

def receipt_b(raw,stmt,corpus):
    top=Reader(raw).parse()
    if not (isinstance(top.value,tuple) and len(top.value)==3 and top.value[0]=="tag" and top.value[1]==18): raise Reject("reference receipt tag")
    body=arr(top.children[0],"reference body")
    if len(body.children)!=4: raise Reject("reference body arity")
    pb=bi(body.children[0],"reference protected"); ph=mp(Reader(pb).parse(),"reference protected map"); uh=mp(body.children[1],"reference unprotected")
    if {k.value for k,_ in pairs(ph)} & {k.value for k,_ in pairs(uh)}: raise Reject("reference cross-bucket duplicate")
    if ii(get(ph,1),"reference alg")!=-8: raise Reject("reference alg")
    kid=bi(get(ph,4),"reference kid"); iss,sub=claims(get(ph,15),"reference")
    vds_id=ii(get(ph,395),"reference VDS")
    if vds_id!=1: raise Reject("reference VDS")
    vd=mp(get(uh,396),"reference VDP"); pa=arr(get(vd,-1),"reference inclusion")
    if len(pa.children)!=1: raise Reject("reference proof multiplicity")
    proof=bi(pa.children[0],"reference proof"); pn=arr(Reader(proof).parse(),"reference proof")
    if len(pn.children)!=3: raise Reject("reference proof arity")
    size=ii(pn.children[0],"reference tree size"); idx=ii(pn.children[1],"reference leaf index"); pathn=arr(pn.children[2],"reference path")
    if size > MAX_SYNTHETIC_TREE_SIZE: raise Reject("reference tree_size exceeds synthetic profile ceiling")
    path=[bi(x,"reference path node") for x in pathn.children]; payload=bi(body.children[2],"reference root"); sig=bi(body.children[3],"reference signature")
    key_matches=[x for x in corpus["ts_key_registry"] if x["raw_kid_hex"]==kid.hex()]
    if len(key_matches)!=1: raise Reject("reference TS key resolution is not unique")
    ts_key=key_matches[0]
    statement_iss,statement_sub=statement_claims(stmt)
    if iss!=ts_key["issuer"]: raise Reject("reference Receipt issuer != selected TS issuer")
    if sub!=statement_sub: raise Reject("reference Receipt subject != Signed Statement subject")
    entry=bytes.fromhex(corpus["vds_entry_bytes_hex"])
    if entry!=stmt: raise Reject("reference synthetic VDS entry differs from Signed Statement bytes")
    pem=ts_key["public_key_pem"].encode(); calc=inclusion_b(H(b"\x00"+entry),idx,size,path)
    if calc!=payload: raise Reject("reference proof_to_root mismatch")
    ss=ar([ts("Signature1"),bs(pb),bs(b""),body.children[2].raw])
    if not openssl_verify(pem,ss,sig): raise Reject("reference signature_to_root failure")
    return {
        "proof_to_root":"PASS",
        "signature_to_root":"PASS",
        "vds_id":vds_id,
        "vdp_proof_type":VDP_INCLUSION,
        "tree_size":size,
        "leaf_index":idx,
        "root_hash":payload.hex(),
    }

def safe(fn,*xs):
    try:return {"disposition":"PASS","detail":fn(*xs)}
    except Reject as e:return {"disposition":"REJECT","error":str(e)}
    except Exception as e:return {"disposition":"UNRESOLVED","error":type(e).__name__+":"+str(e)}

def independent(raw,stmt,corpus):
    a=safe(receipt_a,raw,stmt,corpus); b=safe(receipt_b,raw,stmt,corpus)
    if a["disposition"]!=b["disposition"]:
        return {"disposition":"UNRESOLVED","primary":a,"reference":b}
    if a["disposition"]=="PASS":
        ad=a.get("detail",{}); bd=b.get("detail",{})
        common=(
            ad.get("proof_to_root"),
            ad.get("signature_to_root"),
            ad.get("vds_id"),
            ad.get("vdp_proof_type"),
            ad.get("tree_size"),
            ad.get("leaf_index"),
            ad.get("root_hash"),
        )
        bcommon=(
            bd.get("proof_to_root"),
            bd.get("signature_to_root"),
            bd.get("vds_id"),
            bd.get("vdp_proof_type"),
            bd.get("tree_size"),
            bd.get("leaf_index"),
            bd.get("root_hash"),
        )
        if common != bcommon:
            return {"disposition":"UNRESOLVED","primary":a,"reference":b,"agreement":"FAIL"}
    return {"disposition":a["disposition"],"primary":a,"reference":b,"agreement":"PASS"}

def header_rebuild(n,repl=None,remove=None):
    repl=repl or {}; remove=remove or set(); xs=[]; seen=set()
    for k,v in pairs(n):
        if type(k.value) is not int: raise Reject("mutation helper only integer header labels")
        label=k.value
        if label in remove: continue
        if label in seen: raise Reject("mutation helper duplicate label")
        seen.add(label)
        xs.append((label,repl.get(label,v.raw)))
    for label,raw in sorted(repl.items()):
        if type(label) is not int: raise Reject("mutation helper only integer replacement labels")
        if label in remove or label in seen: continue
        xs.append((label,raw))
    return mpraw(xs)

def receipt_mut(raw,repl=None,remove=None,proof=None,tag=True,flip_sig=False,um_repl=None):
    s=sign1(raw,"receipt mutation"); ph=header_rebuild(s["pm"],repl,remove); uh=[]; um_repl=um_repl or {}; seen=set()
    for k,v in pairs(s["um"]):
        label=k.value
        if label in um_repl:
            uh.append((label,um_repl[label]))
        elif label==HP_VDP and proof is not None:
            uh.append((HP_VDP,mpraw([(-1,ar([bs(proof)]))])))
        else:
            uh.append((label,v.raw))
        seen.add(label)
    for label,raw_value in sorted(um_repl.items()):
        if type(label) is not int:
            raise Reject("mutation helper only integer unprotected labels")
        if label in remove if remove else False:
            continue
        if label in seen:
            continue
        uh.append((label,raw_value))
    sig=s["sig"]; sig=bytes([sig[0]^1])+sig[1:] if flip_sig else sig
    body=ar([bs(ph),mpraw(uh),bs(s["payload"]),bs(sig)])
    out=tag18(body) if tag else body
    if out==raw:
        raise Reject("receipt mutation produced byte-identical output")
    return out

def statement_mut(raw,repl=None,remove=None,payload=None):
    s=sign1(raw,"statement mutation"); ph=header_rebuild(s["pm"],repl,remove)
    body=ar([bs(ph),mpraw([]),bs(s["payload"] if payload is None else payload),bs(s["sig"])])
    out=tag18(body)
    if out==raw:
        raise Reject("statement mutation produced byte-identical output")
    return out

def validate_statement(raw):
    s=sign1(raw,"Signed Statement"); h=profile_header(s,"Signed Statement",False)
    if pairs(s["um"]): raise Reject("registered Signed Statement unprotected header must be empty")
    nested=sign1(s["payload"],"Nested sealed Output"); nh=profile_header(nested,"Nested sealed Output",False)
    if h["kid"]!=nh["kid"]: raise Reject("outer kid != nested sealing kid")
    ph=s["pm"]; pol=bi(get(ph,HP_POLICY),"outer policy"); bil=bi(get(ph,HP_BILATERAL),"outer bilateral")
    om=mp(Reader(nested["payload"]).parse(),"Reconciliation Output")
    if pol!=bi(get(om,100),"inner policy") or bil!=bi(get(om,101),"inner bilateral"): raise Reject("outer/nested policy binding")
    if claims(get(ph,HP_CWT),"outer")[1]!=claims(get(nested["pm"],HP_CWT),"nested")[1]: raise Reject("outer/nested subject binding")
    return {"signed_statement_bytes_sha256":hx(raw),"object_identity_commitment":hx(raw),"nested_output_bytes_sha256":hx(nested["payload"]),
            "outer_kid_hex":h["kid"].hex(),"nested_kid_hex":nh["kid"].hex(),
            "policy_hash":pol.hex(),"bilateral_agreement_hash":bil.hex(),"statement_to_entry_binding":"PASS"}

def transparent_validate(stmt,tr):
    s=sign1(stmt,"Signed Statement"); t=sign1(tr,"Transparent Statement")
    if t["protected"]!=s["protected"] or t["payload"]!=s["payload"] or t["sig"]!=s["sig"]: raise Reject("Transparent Statement changed Signed Statement bytes")
    if len(pairs(t["um"]))!=1: raise Reject("receipt sequence shape")
    a=arr(get(t["um"],HP_RECEIPTS),"receipt sequence"); rs=[bi(x,"receipt sequence entry") for x in a.children]
    return {"receipts":rs,"receipt_sequence_commitment":hx(a.raw)}

def b64raw(b): return base64.urlsafe_b64encode(b).rstrip(b"=").decode()

def merkle_root(entries):
    if not entries: return H(b"")
    leaves=[H(b"\x00"+x) for x in entries]
    def mth_hash(xs):
        if len(xs)==1: return xs[0]
        k=1 << ((len(xs)-1).bit_length()-1)
        return H(b"\x01"+mth_hash(xs[:k])+mth_hash(xs[k:]))
    return mth_hash(leaves)

def merkle_fixture_check(c):
    m=c["merkle_vectors"]
    entries=[bytes.fromhex(x) for x in m["entries_hex"]]
    expected=bytes.fromhex(m["root_hash_hex"])
    if merkle_root(entries)!=expected: raise Reject("reference MTH root mismatch")
    leaf=H(b"\x00"+entries[m["leaf_index"]])
    path=[bytes.fromhex(x) for x in m["proof_path_hex"]]
    a=inclusion_a(leaf,m["leaf_index"],m["tree_size"],path)
    b=inclusion_b(leaf,m["leaf_index"],m["tree_size"],path)
    if a!=expected or b!=expected: raise Reject("dual inclusion proof mismatch")

def profile_check(c):
    p=c["profile"]
    if (p.get("vds_id"),p.get("vdp_id"))!=(1,-1): raise Reject("profile selector mismatch")
    if len([x for x in c["vds_registry"] if x=={"id":1,"name":"RFC9162_SHA256","leaf_hash":"sha256(0x00 || VDSEntryBytes)"}])!=1: raise Reject("VDS registry mismatch")
    if len([x for x in c["vdp_registry"] if x=={"id":-1,"name":"inclusion","proof_shape":"[tree_size, leaf_index, inclusion_path]","vds_id":1}])!=1: raise Reject("VDP registry mismatch")

def key_check(c):
    seen=set()
    for x in c["ts_key_registry"]:
        raw=x["raw_kid_hex"]
        if bytes.fromhex(raw).hex()!=raw or b64raw(bytes.fromhex(raw))!=x["base64url_kid"]: raise Reject("non-canonical key identifier")
        for q in (("raw",raw),("b64",x["base64url_kid"])):
            if q in seen: raise Reject("duplicate effective TS kid")
            seen.add(q)

def mutate_byte(b):
    if not b:
        raise Reject("cannot mutate empty byte string")
    z=bytearray(b); z[-1]^=1
    out=bytes(z)
    if out==b:
        raise Reject("byte mutation produced identical output")
    return out

def parser_rejects_nonminimal_cbor():
    try:
        Reader(b"\xa1\x01\x38\x07").parse()
    except Reject:
        return {"disposition":"REJECT"}
    return {"disposition":"PASS"}

def run(corpus_path,report):
    c=json.loads(Path(corpus_path).read_text())
    require_exact_keys(c, {"profile","receipt_a_hex","receipt_b_hex","registered_statement_hex","schema","transparent_statement_hex","tree","ts_key_registry","vdp_registry","vds_registry","vds_entry_bytes_hex","merkle_vectors"}, "synthetic corpus")
    if c.get("schema") != CORPUS_SCHEMA: raise Reject("corpus schema mismatch")
    profile_check(c); key_check(c)
    assert_surface_bindings(Path(corpus_path))
    stmt=bytes.fromhex(c["registered_statement_hex"]); ra=bytes.fromhex(c["receipt_a_hex"]); rb=bytes.fromhex(c["receipt_b_hex"]); tr=bytes.fromhex(c["transparent_statement_hex"])
    entry=bytes.fromhex(c["vds_entry_bytes_hex"])
    if entry!=stmt: raise Reject("synthetic VDS entry binding mismatch")
    sinfo=validate_statement(stmt); seq=transparent_validate(stmt,tr)
    merkle_fixture_check(c)
    base_a=independent(ra,entry,c)
    base_b=independent(rb,entry,c)
    if base_a["disposition"] != "PASS" or base_b["disposition"] != "PASS":
        raise Reject("registered baseline receipts did not agree with both independent implementations")
    body=sign1(stmt,"statement")
    perm=tag18(ar([bs(body["protected"]),mpraw([(HP_RECEIPTS,ar([bs(rb),bs(ra)]))]),bs(body["payload"]),bs(body["sig"])]))
    pseq=transparent_validate(stmt,perm)
    cases={}
    def case(name,res,exp): cases[name]={"result":res,"qualification":exp}
    case("BASE_RECEIPT_A",independent(ra,entry,c),"PASS"); case("BASE_RECEIPT_B",independent(rb,entry,c),"PASS")
    case("STATEMENT_BYTE_MUTATION_FIXED_RECEIPT",independent(ra,mutate_byte(entry),c),"REJECT")
    x=independent(ra,mutate_byte(entry),c)
    rr=sign1(ra,"Receipt"); pem=key_pem(c["ts_key_registry"],bi(get(rr["pm"],HP_KID),"Receipt kid"))
    x["signature_alone"]={"disposition":"PASS" if openssl_verify(pem,rr["sig_structure"],rr["sig"]) else "REJECT"}
    case("VALID_SIGNATURE_WRONG_LEAF_PROOF_REJECTED",x,"REJECT")
    proof=bytes.fromhex(c["tree"]["proof_hex"]); bad=mutate_byte(proof)
    case("PROOF_PATH_MUTATION",independent(receipt_mut(ra,proof=bad),stmt,c),"REJECT")
    eq=ar([cu(1),cu(1),ar([])]); case("LEAF_INDEX_EQUALS_TREE_SIZE",independent(receipt_mut(ra,proof=eq),stmt,c),"REJECT")
    over_limit_proof=ar([cu(MAX_SYNTHETIC_TREE_SIZE+1),cu(0),ar([])])
    over_limit_result=independent(receipt_mut(ra,proof=over_limit_proof),stmt,c)
    tree_size_reason_match=(
        over_limit_result.get("disposition")=="REJECT"
        and over_limit_result.get("agreement")=="PASS"
        and over_limit_result.get("primary",{}).get("error")=="tree_size exceeds synthetic profile ceiling"
        and over_limit_result.get("reference",{}).get("error")=="reference tree_size exceeds synthetic profile ceiling"
    )
    case("TREE_SIZE_OVER_SYNTHETIC_PROFILE_CEILING",over_limit_result,"REJECT")
    case("VDS_SELECTOR_MUTATION",independent(receipt_mut(ra,repl={HP_VDS:ci(999)}),entry,c),"REJECT")
    case("MISSING_RECEIPT_KID",independent(receipt_mut(ra,remove={HP_KID}),entry,c),"REJECT")
    case("UNTAGGED_RECEIPT",independent(receipt_mut(ra,tag=False),entry,c),"REJECT")
    case("RECEIPT_SIGNATURE_MUTATION",independent(receipt_mut(ra,flip_sig=True),entry,c),"REJECT")
    dup=copy.deepcopy(c); dup["ts_key_registry"].append(copy.deepcopy(dup["ts_key_registry"][0])); 
    try:key_check(dup); disp="PASS"
    except Reject:disp="REJECT"
    case("DUPLICATE_EFFECTIVE_TS_KID",{"disposition":disp},"REJECT")
    missing=statement_mut(stmt,remove={HP_CWT})
    try:validate_statement(missing); disp="PASS"
    except Reject:disp="REJECT"
    case("MISSING_SIGNED_STATEMENT_CLAIMS",{"disposition":disp},"REJECT")
    nested=sign1(sign1(stmt,"s")["payload"],"n")
    bare=statement_mut(stmt,payload=nested["payload"])
    try:validate_statement(bare);disp="PASS"
    except Reject:disp="REJECT"
    case("BARE_OUTPUT_SUBSTITUTION",{"disposition":disp},"REJECT")
    badkid=statement_mut(stmt,repl={HP_KID:bs(b"DIFFERENT")})
    try:validate_statement(badkid);disp="PASS"
    except Reject:disp="REJECT"
    case("OUTER_NESTED_KID_MISMATCH",{"disposition":disp},"REJECT")
    case("SEMANTICALLY_EQUIVALENT_BYTE_VARIANT",independent(ra,statement_reordered(stmt),c),"REJECT")
    case("NON_MINIMAL_CBOR_ENCODING",parser_rejects_nonminimal_cbor(),"REJECT")
    case("RAW_BASE64URL_KID_EQUIVALENCE",{"disposition":"PASS" if b64raw(bytes.fromhex(c["ts_key_registry"][0]["raw_kid_hex"]))==c["ts_key_registry"][0]["base64url_kid"] else "REJECT"},"PASS")
    wrong_iss=mpraw([(1,ts("https://evil.invalid")),(2,ts(statement_claims(stmt)[1]))])
    wrong_sub=mpraw([(1,ts(c["ts_key_registry"][0]["issuer"])),(2,ts("wrong-subject"))])
    case("RECEIPT_ISSUER_MISMATCH",independent(receipt_mut(ra,repl={HP_CWT:wrong_iss}),entry,c),"REJECT")
    case("RECEIPT_SUBJECT_MISMATCH",independent(receipt_mut(ra,repl={HP_CWT:wrong_sub}),entry,c),"REJECT")
    case("VDP_ARRAY_MAJOR_TYPE_CONFUSION",independent(receipt_mut(ra,um_repl={HP_VDP:ar([])}),entry,c),"REJECT")
    case("PROOF_MAP_MAJOR_TYPE_CONFUSION",independent(receipt_mut(ra,proof=b"\xa0"),entry,c),"REJECT")
    case("UNKNOWN_CRITICAL_HEADER",independent(receipt_mut(ra,repl={2:ar([ci(999)])}),entry,c),"REJECT")
    case("UNPROTECTED_CRITICAL_HEADER",independent(receipt_mut(ra,um_repl={2:ar([ci(4)])}),entry,c),"REJECT")
    case("RECEIPT_SEQUENCE_PERMUTATION",{"disposition":"PASS","registered_object_identity_unchanged":hx(stmt)==sinfo["object_identity_commitment"],
         "sequence_commitment_changed":pseq["receipt_sequence_commitment"]!=seq["receipt_sequence_commitment"],
         "selected_receipt_changed":hx(seq["receipts"][0])!=hx(pseq["receipts"][0])},"PASS")
    bads=[]
    for n,v in cases.items():
        if v["result"].get("disposition")!=v["qualification"]: bads.append((n,v["qualification"],v["result"].get("disposition")))
    if not tree_size_reason_match:
        bads.append(("TREE_SIZE_OVER_SYNTHETIC_PROFILE_CEILING","both independent implementations must reject for the pinned ceiling reason","reason mismatch"))
    if len(cases) != EXPECTED_MUTATION_CASE_COUNT:
        diagnostic={
            "schema":SCHEMA,
            "claim_ceiling":"SYNTHETIC_RESEARCH_ONLY",
            "qualification":"FAIL",
            "expected_metamorphic_case_count":EXPECTED_MUTATION_CASE_COUNT,
            "metamorphic_case_count":len(cases),
            "case_ids":list(cases),
            "cases":cases,
            "failures":bads + [("MATRIX_CARDINALITY","expected exactly the pinned case count",len(cases))],
        }
        if report: Path(report).write_text(json.dumps(diagnostic,indent=2,sort_keys=True)+"\\n")
        raise Reject("mutation case count disagrees with verifier constant")
    result={"schema":SCHEMA,"claim_ceiling":"SYNTHETIC_RESEARCH_ONLY","issuer_signature_verification":"NOT_EVALUATED","exact_object_identity_scope":"BYTE_EXACT_ONLY","qualification":"FAIL" if bads else "PASS",
            "semantic_duplicate_key_rejection": True,
            "non_minimal_cbor_rejection": True,
            "statement":sinfo,"selected_receipt":{"receipt_bytes_sha256":hx(seq["receipts"][0]),"receipt_sequence_commitment":seq["receipt_sequence_commitment"]},
            "permuted_receipt_selection":{"receipt_bytes_sha256":hx(pseq["receipts"][0]),"receipt_sequence_commitment":pseq["receipt_sequence_commitment"]},
            "proof_binding":{"vds_id":1,"vdp_proof_type":-1,"vds_entry_bytes_sha256":hx(entry),"vdp_proof_bytes_sha256":hx(proof),"leaf_input_sha256":hx(entry),"leaf_commitment":H(b"\x00"+entry).hex(),
                             "tree_size":c["tree"]["tree_size"],"leaf_index":c["tree"]["leaf_index"],"root_hash":c["tree"]["root_hash_hex"],
                             "proof_to_root_verdict":"PASS","signature_to_root_verdict":"PASS"},
            "receipt_identity":{"receipt_bytes_sha256":hx(ra),"receipt_protected_bstr_sha256":receipt_a(ra,entry,c)["receipt_protected_bstr_sha256"], "issuer":statement_claims(ra)[0], "subject":statement_claims(ra)[1]},
            "independent_oracles":"AGREE","metamorphic_case_count":len(cases),"surface_binding":"VERIFIED","cases":cases}
    if bads: result["failures"]=bads
    if report: Path(report).write_text(json.dumps(result,indent=2,sort_keys=True)+"\n")
    if bads: raise SystemExit("qualification FAIL: "+repr(bads))
    print("SYM-CIVIC-019 RECEIPT PROOF BINDING=PASS")
    print("claim_ceiling=SYNTHETIC_RESEARCH_ONLY")
    print("P1 receipt validity=PASS")
    print("P2 exact-object identity=PASS")
    print("independent_oracles=AGREE")
    print("issuer_signature_verification=NOT_EVALUATED")
    print("semantic_duplicate_key_rejection=true")
    print("non_minimal_cbor_rejection=true")
    print("metamorphic_cases="+str(len(cases)))

def statement_reordered(stmt):
    s=sign1(stmt,"statement")
    pairs0=list(reversed(pairs(s["pm"])))
    ph=chead(5,len(pairs0))+b"".join(k.raw+v.raw for k,v in pairs0)
    out=tag18(ar([bs(ph),mpraw([]),bs(s["payload"]),bs(s["sig"])]))
    if out==stmt:
        raise Reject("statement reorder mutation produced byte-identical output")
    return out

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--corpus",required=True); ap.add_argument("--report"); a=ap.parse_args(); run(a.corpus,a.report)
