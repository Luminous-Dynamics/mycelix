package oracle

import (
 "crypto/sha256"
 "encoding/hex"
 "encoding/json"
 "fmt"
 "sort"
 "strings"
 "time"
)

type State string
const (
 Established State = "EstablishedWithinProfile"
 Rejected State = "Rejected"
 Indeterminate State = "Indeterminate"
 NotEvaluated State = "NotEvaluated"
 Unsupported State = "UnsupportedByProfile"
 NotApplicable State = "NotApplicable"
)
type Disposition string
const (
 Justified Disposition = "DecisionJustifiedAtAcceptedFrontier"
 NotJustified Disposition = "DecisionNotJustifiedAtAcceptedFrontier"
 Undecided Disposition = "Indeterminate"
)
type Stage string
const (
 Frontier Stage = "verifier_frontier"
 SourceTrust Stage = "source_trust"
 Semantic Stage = "semantic_projection"
 Query Stage = "query"
 Policy Stage = "policy"
 Authorization Stage = "authorization"
 Execution Stage = "execution"
 Outcome Stage = "outcome"
)
var stageOrder = []Stage{Frontier,SourceTrust,Semantic,Query,Policy,Authorization,Execution,Outcome}
var dimensions = []string{"authentication","authority","completeness","effect","independence","truth"}
const supportedProfileDigest = "f05c2d0f280b60c75240f9933f89ba5376ac9a7250f36a3832c14c679bafe902"

type Ref struct { ID, Schema, Digest string }
type Input struct { Slot string; Producer Stage; ReceiptID, Schema, Digest string; Required []string }
type StageReceipt struct {
 ID string; Stage Stage; Schema, OutputSchema, ProfileDigest, ScopeDigest, SourceCommit, BuildDigest, Digest string
 Claims map[string]State; Inputs []Input
}
type ExternalBinding struct { Consumer Stage; Slot, ReceiptID, Schema, Digest string }
type FrontierBinding struct { Kind string; Receipt Ref; Status string; Prefix State }
type ProgressBinding struct { Kind string; Receipt Ref; Status string; Universe State; OwnerInventoryDigest string }
type OwnerBinding struct { Receipt Ref; Authentication, Completeness State; FrontierDigest string }
type EvidenceBinding struct { FailureDomainGraph *Ref; LineageAttestation *Ref; IndependenceStatus State }
type AuthorityBinding struct { Receipt Ref; Principal, Scope string; Validity State }
type ActionBinding struct { Receipt Ref; Digest, Revision, CoverageRevision string; Count, Covered uint64; CoverageDigest string; Coverage Ref }
type OutcomeBinding struct { Receipt Ref; Status, Reconciliation, ExecutionDigest, ActionPlanDigest, MutationScope string; SameScopeMutationAttempt bool }
type Capsule struct {
 SchemaID, ProfileRef, ProfileDigest, DecisionID, QueryID, QueryDigest, QuerySchema, ScopeDigest, ValidAt string
 Policy Ref
 Temporal struct { OCELProfile, SourceCorpus, Projection, Replay Ref; FrontierDigest, SourceCorpusDigest string }
 Frontier FrontierBinding
 Progress ProgressBinding
 Owner OwnerBinding
 Evidence EvidenceBinding
 Authority AuthorityBinding
 Action ActionBinding
 Outcome OutcomeBinding
 Execution Ref
 Stages []StageReceipt
 External []ExternalBinding
 Claims map[string]State
 Disposition Disposition
 Unknowns, Conflicts []string
 Abstention string
}
type ValidationError struct { Code, Detail string }
func (e *ValidationError) Error() string { return e.Code+": "+e.Detail }
func fail(code, detail string) error { return &ValidationError{code,detail} }
func Code(err error) string { if e,ok:=err.(*ValidationError); ok { return e.Code }; return fmt.Sprintf("UNEXPECTED:%v",err) }
func digestOK(s string) bool {
 if len(s)!=64 { return false }; b,e:=hex.DecodeString(s); return e==nil && len(b)==32 && strings.ToLower(s)==s
}
func commitOK(s string) bool { return (len(s)==40 || len(s)==64) && strings.ToLower(s)==s && onlyHex(s) }
func onlyHex(s string) bool { for _,r:=range s { if !(r>='0'&&r<='9') && !(r>='a'&&r<='f') { return false } }; return true }
func refOK(r Ref) bool { return r.ID!="" && r.Schema!="" && digestOK(r.Digest) }
func stageClaims() map[Stage]map[string]bool {
 return map[Stage]map[string]bool{
  Frontier:{"authentication":true,"completeness":true},
  SourceTrust:{"authentication":true,"completeness":true},
  Semantic:{"completeness":true},
  Query:{"completeness":true,"truth":true},
  Policy:{"truth":true},
  Authorization:{"authority":true},
  Execution:{"effect":true},
  Outcome:{"effect":true,"truth":true},
 }
}
type edge struct { Producer Stage; Required []string }
func edges() map[Stage][]edge {
 return map[Stage][]edge{
  SourceTrust:{{Frontier,[]string{"authentication","completeness"}}},
  Semantic:{{Frontier,[]string{"authentication"}},{SourceTrust,[]string{"authentication"}}},
  Query:{{Frontier,[]string{"authentication","completeness"}},{SourceTrust,[]string{"authentication","completeness"}},{Semantic,[]string{"authentication","completeness"}}},
  Policy:{{Query,[]string{"authentication","completeness","truth"}}},
  Authorization:{{Frontier,[]string{"authentication"}},{Policy,[]string{"authentication","truth"}}},
  Execution:{{Authorization,[]string{"authentication","authority"}}},
  Outcome:{{SourceTrust,[]string{"authentication"}},{Semantic,[]string{"completeness"}},{Execution,[]string{"effect"}}},
 }
}
var externalSlots = map[string]Stage{
 "causal_event_frontier":Frontier,"progress_antichain":Frontier,"owner_universe":Frontier,
 "ocel_profile":Semantic,"source_corpus":Semantic,"projection":Semantic,"replay":Semantic,
 "policy":Policy,"authority":Authorization,"action_plan":Execution,"action_coverage":Execution,"outcome":Outcome,
}
func same(a,b []string) bool { if len(a)!=len(b){return false}; x:=append([]string{},a...); y:=append([]string{},b...); sort.Strings(x); sort.Strings(y); return strings.Join(x,"|")==strings.Join(y,"|") }
func established(c map[string]State, d string) bool { return c[d]==Established }
func validState(s State) bool {
 switch s { case Established,Rejected,Indeterminate,NotEvaluated,Unsupported,NotApplicable: return true }
 return false
}
func validateClaimMap(claims map[string]State) bool {
 if len(claims)!=len(dimensions) { return false }
 for _,d:=range dimensions { if !validState(claims[d]) { return false } }
 return true
}
func duplicateStrings(values []string) bool {
 seen:=map[string]bool{}
 for _,v:=range values { if seen[v] { return true }; seen[v]=true }
 return false
}

func externalRefs(c Capsule) map[string]Ref {
 return map[string]Ref{
  "causal_event_frontier": c.Frontier.Receipt,
  "progress_antichain": c.Progress.Receipt,
  "owner_universe": c.Owner.Receipt,
  "ocel_profile": c.Temporal.OCELProfile,
  "source_corpus": c.Temporal.SourceCorpus,
  "projection": c.Temporal.Projection,
  "replay": c.Temporal.Replay,
  "policy": c.Policy,
  "authority": c.Authority.Receipt,
  "action_plan": c.Action.Receipt,
  "action_coverage": c.Action.Coverage,
  "outcome": c.Outcome.Receipt,
 }
}

func Validate(c Capsule) error {
 if c.SchemaID!="mycelix.craft.decision-basis-capsule.v0.1" || c.ProfileRef!="mycelix.djcc.full-evidence-execution.v0.1" || c.ProfileDigest!=supportedProfileDigest { return fail("PROFILE_MISMATCH","unsupported DBC schema/profile or pinned DJCC profile digest") }
 for _,d:=range []string{c.ProfileDigest,c.QueryDigest,c.ScopeDigest} { if !digestOK(d) { return fail("DIGEST_INVALID","malformed profile/query/scope digest") } }
 if c.DecisionID=="" || c.QueryID=="" || c.QuerySchema=="" { return fail("IDENTITY_MISSING","decision/query identity missing") }
 parsedTime,err:=time.Parse("2006-01-02T15:04:05Z",c.ValidAt)
 if err!=nil || parsedTime.UTC().Format("2006-01-02T15:04:05Z")!=c.ValidAt { return fail("VALID_TIME_INVALID","valid_at must be canonical UTC seconds") }
 for _,r:=range []Ref{c.Policy,c.Frontier.Receipt,c.Progress.Receipt,c.Owner.Receipt,c.Authority.Receipt,c.Action.Receipt,c.Action.Coverage,c.Outcome.Receipt,c.Execution} {
  if !refOK(r) { return fail("RECEIPT_INVALID","required receipt ID/schema/digest malformed") }
 }
 if c.Frontier.Kind!="CausalEventFrontier" || c.Progress.Kind!="ProgressAntichain" { return fail("FRONTIER_PROGRESS_CONFLATED","event frontier and progress antichain must be distinct") }
 if c.Progress.Universe!=NotEvaluated { return fail("PROGRESS_UNIVERSE_OVERCLAIM","progress antichain source universe must remain NotEvaluated") }
 if c.Progress.OwnerInventoryDigest!=c.Owner.Receipt.Digest { return fail("PROGRESS_OWNER_BINDING_MISMATCH","progress antichain is bound to a different owner inventory") }
 if c.Owner.FrontierDigest!=c.Frontier.Receipt.Digest { return fail("OWNER_FRONTIER_BINDING_MISMATCH","owner inventory completeness is bound to a different event frontier") }
 if c.Owner.Authentication!=Established || c.Owner.Completeness!=Established {
  // A negative/incomplete owner inventory may still be represented for an
  // indeterminate certificate, but cannot support a positive completeness claim.
 }
 if c.Frontier.Status!="Valid" && c.Frontier.Status!="Incomplete" && c.Frontier.Status!="Invalid" { return fail("FRONTIER_STATUS_INVALID","unknown causal frontier status") }
 if c.Progress.Status!="StableRelativeToDeclaredProgress" && c.Progress.Status!="Provisional" { return fail("PROGRESS_STATUS_INVALID","unknown progress status") }
 if c.Authority.Principal=="" || c.Authority.Scope=="" { return fail("AUTHORITY_SCOPE_MISSING","principal and action scope are required") }
 if c.Action.Digest!=c.Action.CoverageDigest || c.Action.Revision=="" || c.Action.CoverageRevision!=c.Action.Revision || c.Action.Count==0 || c.Action.Covered>c.Action.Count { return fail("ACTION_DENOMINATOR_MISMATCH","coverage does not bind the exact plan digest and revision") }
 if c.Temporal.FrontierDigest!=c.Frontier.Receipt.Digest || c.Temporal.SourceCorpusDigest!=c.Temporal.SourceCorpus.Digest {
  return fail("TEMPORAL_INPUT_BINDING_MISMATCH","temporal projection/replay do not bind the selected frontier and source-corpus receipt")
 }
 if c.Outcome.Status!="Qualified" && c.Outcome.Status!="Unknown" && c.Outcome.Status!="NotEvaluated" { return fail("OUTCOME_STATUS_INVALID","unknown outcome status") }
 if c.Outcome.ExecutionDigest!=c.Execution.Digest || c.Outcome.ActionPlanDigest!=c.Action.Digest {
  return fail("OUTCOME_BINDING_MISMATCH","outcome must bind the exact execution transcript and action-plan digest")
 }
 if c.Outcome.Status=="Unknown" && c.Outcome.SameScopeMutationAttempt {
  return fail("OUTCOME_UNKNOWN_MUTATION_BARRIER","same-scope mutation attempted before exact outcome reconciliation")
 }
 if c.Outcome.MutationScope=="" { return fail("OUTCOME_SCOPE_MISSING","outcome mutation scope is required") }
 if c.Claims["independence"]==Established || c.Claims["independence"]==Rejected { return fail("INDEPENDENCE_PROFILE_UNSUPPORTED","full evidence profile does not qualify independence") }
 if c.Evidence.IndependenceStatus!=NotEvaluated { return fail("INDEPENDENCE_PROFILE_UNSUPPORTED","full evidence profile keeps independence NotEvaluated") }
 if c.Evidence.FailureDomainGraph!=nil && !refOK(*c.Evidence.FailureDomainGraph) { return fail("EVIDENCE_BINDING_INVALID","failure-domain graph reference is malformed") }
 if c.Evidence.LineageAttestation!=nil && !refOK(*c.Evidence.LineageAttestation) { return fail("EVIDENCE_BINDING_INVALID","lineage attestation reference is malformed") }
 byStage:=map[Stage]StageReceipt{}; ids:=map[string]bool{}
 for _,s:=range c.Stages {
  if s.ID=="" || ids[s.ID] { return fail("DUPLICATE_RECEIPT_ID",s.ID) }; ids[s.ID]=true
  if !validateClaimMap(s.Claims) { return fail("STAGE_CLAIMS_INVALID",string(s.Stage)) }
  if _,ok:=byStage[s.Stage]; ok { return fail("DUPLICATE_STAGE",string(s.Stage)) }
  if s.Schema=="" || s.OutputSchema=="" || !digestOK(s.Digest) || !digestOK(s.OutputDigest) || !digestOK(s.ProfileDigest) || !digestOK(s.ScopeDigest) || !digestOK(s.BuildDigest) || !commitOK(s.SourceCommit) || s.ProfileDigest!=c.ProfileDigest || s.ScopeDigest!=c.ScopeDigest { return fail("STAGE_RECEIPT_INVALID",string(s.Stage)) }
  byStage[s.Stage]=s
 }
 if len(byStage)!=len(stageOrder) { return fail("STAGE_SET_INCOMPLETE","exactly eight DJCC stages are required") }
 for _,st:=range stageOrder { if _,ok:=byStage[st]; !ok { return fail("STAGE_SET_INCOMPLETE",string(st)) } }
 if !validateClaimMap(c.Claims) { return fail("FINAL_CLAIMS_INVALID","all six claim dimensions must have valid states") }
 if duplicateStrings(c.Unknowns) || duplicateStrings(c.Conflicts) { return fail("REASON_DUPLICATE","unknown/conflict reason references must be unique") }
 matchesStageOutput:=func(r Ref,s StageReceipt) bool { return r.ID==s.ID && r.Schema==s.OutputSchema && r.Digest==s.OutputDigest }
 if !matchesStageOutput(c.Frontier.Receipt,byStage[Frontier]) { return fail("FRONTIER_STAGE_BINDING_MISMATCH","frontier receipt differs from verifier_frontier stage output") }
 if !matchesStageOutput(c.Policy,byStage[Policy]) { return fail("POLICY_STAGE_BINDING_MISMATCH","policy receipt differs from policy stage output") }
 if !matchesStageOutput(c.Authority.Receipt,byStage[Authorization]) { return fail("AUTHORITY_STAGE_BINDING_MISMATCH","authority receipt differs from authorization stage output") }
 if !matchesStageOutput(c.Execution,byStage[Execution]) { return fail("EXECUTION_STAGE_BINDING_MISMATCH","execution receipt differs from execution stage output") }
 if !matchesStageOutput(c.Outcome.Receipt,byStage[Outcome]) { return fail("OUTCOME_STAGE_BINDING_MISMATCH","outcome receipt differs from outcome stage output") }
 for st,s:=range byStage {
  required:=edges()[st]; if len(s.Inputs)!=len(required) { return fail("DEPENDENCY_EDGE_SET_MISMATCH",string(st)) }
  slots:=map[string]bool{}
  for _,in:=range s.Inputs { if slots[in.Slot] { return fail("DEPENDENCY_SLOT_DUPLICATE",in.Slot) }; slots[in.Slot]=true }
  for _,e:=range required {
   found:=false
   for _,in:=range s.Inputs {
    if in.Slot==string(e.Producer) && in.Producer==e.Producer {
     p:=byStage[e.Producer]
     if in.ReceiptID!=p.ID || in.Schema!=p.OutputSchema || in.Digest!=p.OutputDigest || !same(in.Required,e.Required) { return fail("PRODUCER_BINDING_MISMATCH",string(st)+"/"+in.Slot) }
     for _,dim:=range e.Required { if !established(p.Claims,dim) { return fail("PRODUCER_CLAIM_MISSING",string(e.Producer)+"/"+dim) } }
     found=true
    }
   }
   if !found { return fail("DEPENDENCY_EDGE_MISSING",string(st)+"/"+string(e.Producer)) }
  }
 }
 expected:=map[string]ExternalBinding{}
 for _,b:=range c.External {
  k:=string(b.Consumer)+"/"+b.Slot
  if _,ok:=expected[k]; ok { return fail("EXTERNAL_BINDING_DUPLICATE",k) }
  owner,ok:=externalSlots[b.Slot]
  if !ok || owner!=b.Consumer || !refOK(Ref{b.ReceiptID,b.Schema,b.Digest}) { return fail("EXTERNAL_BINDING_INVALID",k) }
  want:=externalRefs(c)[b.Slot]
  if b.ReceiptID!=want.ID || b.Schema!=want.Schema || b.Digest!=want.Digest { return fail("EXTERNAL_BINDING_MISMATCH",k) }
  expected[k]=b
 }
 if len(expected)!=len(externalSlots) { return fail("EXTERNAL_BINDINGS_INCOMPLETE","all 12 source/profile references required") }
 owned:=stageClaims()
 for st,s:=range byStage {
  for _,dim:=range dimensions {
   if s.Claims[dim]==Established && !owned[st][dim] {
    carried:=false
    for _,in:=range s.Inputs { for _,d:=range in.Required { if d==dim && established(byStage[in.Producer].Claims,dim) { carried=true } } }
    if !carried { return fail("CLAIM_CEILING_WIDENING",string(st)+"/"+dim) }
   }
  }
 }
 if established(c.Claims,"authentication") && (!established(byStage[Frontier].Claims,"authentication") || !established(byStage[SourceTrust].Claims,"authentication")) { return fail("FINAL_AUTHENTICATION_WIDENING","frontier and source trust must establish authentication") }
 if established(c.Claims,"authority") && (!established(byStage[Authorization].Claims,"authority") || c.Authority.Validity!=Established) { return fail("FINAL_AUTHORITY_WIDENING","authorization and authority receipt must establish authority") }
 if established(c.Claims,"completeness") && (!established(byStage[Frontier].Claims,"completeness") || !established(byStage[SourceTrust].Claims,"completeness") || !established(byStage[Semantic].Claims,"completeness") || !established(byStage[Query].Claims,"completeness") || c.Frontier.Status!="Valid" || c.Frontier.Prefix!=Established || c.Progress.Status!="StableRelativeToDeclaredProgress" || c.Owner.Authentication!=Established || c.Owner.Completeness!=Established) { return fail("FINAL_COMPLETENESS_WIDENING","frontier, stable progress and authenticated owner-universe receipt are separately required") }
 if established(c.Claims,"effect") && (!established(byStage[Execution].Claims,"effect") || !established(byStage[Outcome].Claims,"effect") || c.Outcome.Status!="Qualified" || (c.Outcome.Reconciliation!="Resolved" && c.Outcome.Reconciliation!="NotRequired")) { return fail("FINAL_EFFECT_WIDENING","qualified execution and reconciled outcome required") }
 if established(c.Claims,"truth") && (!established(byStage[Query].Claims,"truth") || !established(byStage[Policy].Claims,"truth") || !established(byStage[Outcome].Claims,"truth")) { return fail("FINAL_TRUTH_WIDENING","query, policy and outcome truth required") }
 if c.Disposition!=Justified && c.Disposition!=NotJustified && c.Disposition!=Undecided { return fail("DISPOSITION_INVALID","unknown composition disposition") }
 if c.Disposition==Justified {
  for _,d:=range []string{"authentication","authority","completeness","effect","truth"} { if !established(c.Claims,d) { return fail("JUSTIFIED_WITHOUT_REQUIRED_CLAIMS",d) } }
  if c.Action.Covered!=c.Action.Count || len(c.Unknowns)>0 || len(c.Conflicts)>0 || c.Abstention!="" || c.Frontier.Status!="Valid" || c.Progress.Status!="StableRelativeToDeclaredProgress" { return fail("JUSTIFIED_WITH_UNRESOLVED_BASIS","unresolved evidence, frontier or action basis") }
 }
 return nil
}
func CanonicalDigest(c Capsule) (string,error) {
 if err:=Validate(c); err!=nil { return "",err }
 c.Stages=append([]StageReceipt(nil),c.Stages...)
 for i:=range c.Stages { c.Stages[i].Inputs=append([]Input(nil),c.Stages[i].Inputs...) }
 c.External=append([]ExternalBinding(nil),c.External...)
 c.Unknowns=append([]string(nil),c.Unknowns...)
 c.Conflicts=append([]string(nil),c.Conflicts...)
 sort.Slice(c.Stages,func(i,j int)bool { for a,x:=range stageOrder { if c.Stages[i].Stage==x { for _,y:=range stageOrder[a+1:] { if c.Stages[j].Stage==y { return true } }; return false } }; return c.Stages[i].Stage<c.Stages[j].Stage })
 for i:=range c.Stages { sort.Slice(c.Stages[i].Inputs,func(a,b int)bool{return c.Stages[i].Inputs[a].Slot<c.Stages[i].Inputs[b].Slot}) }
 sort.Slice(c.External,func(i,j int)bool { if c.External[i].Consumer!=c.External[j].Consumer{return c.External[i].Consumer<c.External[j].Consumer}; return c.External[i].Slot<c.External[j].Slot })
 sort.Strings(c.Unknowns); sort.Strings(c.Conflicts)
 b,e:=json.Marshal(c); if e!=nil{return "",e}
 h:=sha256.Sum256(append([]byte("MYCELIX-CRAFT-DBC-V0.1\x00"),b...))
 return hex.EncodeToString(h[:]),nil
}
