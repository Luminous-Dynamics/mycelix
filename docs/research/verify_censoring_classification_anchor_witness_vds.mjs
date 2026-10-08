#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

const CAMPAIGN="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-campaign.v1";
const FIXTURE="mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-fixture.v1";
const canonical=v=>v===null||typeof v!=="object"?JSON.stringify(v):Array.isArray(v)?"["+v.map(canonical).join(",")+"]":"{"+Object.keys(v).sort().map(k=>JSON.stringify(k)+":"+canonical(v[k])).join(",")+"}";
const H=b=>crypto.createHash("sha256").update(b).digest();
const leaf=e=>H(Buffer.concat([Buffer.from([0]),Buffer.from(canonical(e),"utf8")]));
const node=(a,b)=>H(Buffer.concat([Buffer.from([1]),a,b]));
function mth(ds){const n=ds.length;if(n===0)return H(Buffer.alloc(0));if(n===1)return leaf(ds[0]);const k=1<<((n-1).toString(2).length-1);return node(mth(ds.slice(0,k)),mth(ds.slice(k)));}
const kpow=n=>1<<((n-1).toString(2).length-1);
function subproof(m,ds,known){const n=ds.length;if(m===n)return known?[]:[mth(ds)];const k=kpow(n);return m<=k?[...subproof(m,ds.slice(0,k),known),mth(ds.slice(k))]:[...subproof(m-k,ds.slice(k),false),mth(ds.slice(0,k))];}
const asHash=s=>{if(typeof s!=="string"||!s.startsWith("sha256:")||s.length!==71)throw Error("hash");return Buffer.from(s.slice(7),"hex");};
function verifyConsistency(m,n,firstHash,secondHash,path){
  if(!path.length||m<=0||m>=n)return false;
  let p=[...path];if((m&(m-1))===0)p=[firstHash,...p];
  let fn=m-1,sn=n-1;while(fn&1){fn>>=1;sn>>=1;}
  let fr=p[0],sr=p[0];
  for(let i=1;i<p.length;i++){
    const c=p[i];if(sn===0)return false;
    if((fn&1)||fn===sn){fr=node(c,fr);sr=node(c,sr);if(!(fn&1))while(fn&&!((fn)&1)){fn>>=1;sn>>=1;}}
    else sr=node(sr,c);
    fn>>=1;sn>>=1;
  }
  return sn===0&&Buffer.compare(fr,firstHash)===0&&Buffer.compare(sr,secondHash)===0;
}
function head(fixture,name,override){const h=structuredClone(fixture.heads[name]);if(override!==undefined)h.root_hash=override;return h;}
function evaluate(f,c){
  const entries=f.entries;
  const first=head(f,c.first_head,c.first_root_override);
  if(first.tree_size>entries.length)return["unresolved","tree-size-out-of-range"];
  const fr=asHash(first.root_hash);
  if(Buffer.compare(mth(entries.slice(0,first.tree_size)),fr)!==0)return["unresolved","first-head-root-mismatch"];
  if(c.second_head===undefined)return["qualified","root-reconstructed"];
  const second=head(f,c.second_head,c.second_root_override);
  if(second.tree_size>entries.length)return["unresolved","tree-size-out-of-range"];
  const sr=asHash(second.root_hash);
  if(Buffer.compare(mth(entries.slice(0,second.tree_size)),sr)!==0)return["unresolved","second-head-root-mismatch"];
  if(second.tree_size<first.tree_size)return["unresolved","rollback"];
  if(second.tree_size===first.tree_size)return Buffer.compare(fr,sr)===0?["qualified","same-head"]:["unresolved","equivocation"];
  if(c.proof!=="fixture")return["unresolved","missing-proof"];
  if(first.tree_size!==4||second.tree_size!==7)return["unresolved","unsupported-fixture-pair"];
  let p=f.consistency_proof_4_to_7.map(asHash);
  if(c.proof_mutation==="replace-first")p[0]=H(p[0]);
  else if(c.proof_mutation==="empty")p=[];
  else if(c.proof_mutation==="append-extra")p.push(H(Buffer.from("extra")));
  else if(c.proof_mutation==="truncate")p=p.slice(0,-1);
  return verifyConsistency(first.tree_size,second.tree_size,fr,sr,p)?["qualified","consistency-proof"]:["unresolved","consistency-proof-invalid"];
}
const [fp,cp,op]=process.argv.slice(2);if(!op)process.exit(2);
const f=JSON.parse(fs.readFileSync(fp,"utf8")),c=JSON.parse(fs.readFileSync(cp,"utf8"));
if(f.schema!==FIXTURE||c.schema!==CAMPAIGN||c.case_count!==15||c.cases.length!==15)process.exit(1);
const ids=c.cases.map(x=>x.case_id);if(new Set(ids).size!==ids.length)process.exit(1);if(subproof(4,f.entries.slice(0,7),true).map(x=>"sha256:"+x.toString("hex")).join("|")!==f.consistency_proof_4_to_7.join("|"))process.exit(1);
const rows=[],failures=[];
for(const x of c.cases){const [v,r]=evaluate(f,x);const row={case_id:x.case_id,expected_verdict:x.expected_verdict,actual_verdict:v,reason:r};rows.push(row);if(v!==x.expected_verdict)failures.push([x.case_id,x.expected_verdict,v,r]);}
fs.writeFileSync(op,canonical({schema:"mycelix.continual-adaptation.censoring-classification-anchor-witness-vds-report.v1",status:"research-evidence-only",case_count:15,cases:rows,failures})+"\n");
console.log("cases="+rows.length+" failures="+failures.length);process.exit(failures.length?1:0);
