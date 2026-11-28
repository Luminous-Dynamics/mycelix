package oracle

import (
 "crypto/sha256"
 "encoding/hex"
 "encoding/json"
 "fmt"
 "os"
 "path/filepath"
 "sort"
 "strings"
 "testing"
)

func d(n int) string { return fmt.Sprintf("%064x",n+1) }
func ref(id string,n int) Ref { return Ref{ID:id,Schema:"schema."+id,Digest:d(n)} }
func base() Capsule {
 c:=Capsule{
  SchemaID:"mycelix.craft.decision-basis-capsule.v0.1",ProfileRef:"mycelix.djcc.full-evidence-execution.v0.1",
  ProfileDigest:d(1),DecisionID:"decision-1",QueryID:"query-1",QueryDigest:d(2),QuerySchema:"query-v1",ScopeDigest:d(3),ValidAt:"2026-11-28T10:00:00Z",
  Policy:ref("policy",4),Frontier:FrontierBinding{Kind:"CausalEventFrontier",Receipt:ref("frontier",5),Status:"Valid",Prefix:Established},
  Progress:ProgressBinding{Kind:"ProgressAntichain",Receipt:ref("progress",6),Status:"StableRelativeToDeclaredProgress",Universe:NotEvaluated},
  Owner:OwnerBinding{Receipt:ref("owner-universe",7),Completeness:Established},
  Authority:AuthorityBinding{Receipt:ref("authority",8),Principal:"assessor-1",Scope:"zone-1/decision:close",Validity:Established},
  Action:ActionBinding{Receipt:ref("plan",9),Digest:d(9),Revision:"rev-1",Count:2,Covered:2,CoverageDigest:d(9),Coverage:ref("coverage",10)},
  Outcome:OutcomeBinding{Receipt:ref("outcome",11),Status:"Qualified",Reconciliation:"Resolved"},Execution:ref("execution",12),
  Claims:map[string]State{"authentication":Established,"authority":Established,"completeness":Established,"effect":Established,"truth":Established,"independence":NotEvaluated},Disposition:Justified,
 }
 ids:=map[Stage]string{}; schemas:=map[Stage]string{}; digests:=map[Stage]string{}
 for i,st:=range stageOrder { ids[st]=fmt.Sprintf("receipt-%s",st); schemas[st]=fmt.Sprintf("out-%s",st); digests[st]=d(20+i) }
 own:=map[Stage]map[string]State{
  Frontier:{"authentication":Established,"completeness":Established},SourceTrust:{"authentication":Established,"completeness":Established},
  Semantic:{"completeness":Established},Query:{"completeness":Established,"truth":Established},Policy:{"truth":Established},
  Authorization:{"authority":Established},Execution:{"effect":Established},Outcome:{"effect":Established,"truth":Established},
 }
 dep:=map[Stage][]edge{
  SourceTrust:{{Frontier,[]string{"authentication","completeness"}}},
  Semantic:{{Frontier,[]string{"authentication"}},{SourceTrust,[]string{"authentication"}}},
  Query:{{Frontier,[]string{"authentication","completeness"}},{SourceTrust,[]string{"authentication","completeness"}},{Semantic,[]string{"authentication","completeness"}}},
  Policy:{{Query,[]string{"authentication","completeness","truth"}}},
  Authorization:{{Frontier,[]string{"authentication"}},{Policy,[]string{"authentication","truth"}}},
  Execution:{{Authorization,[]string{"authentication","authority"}}},
  Outcome:{{SourceTrust,[]string{"authentication"}},{Semantic,[]string{"completeness"}},{Execution,[]string{"effect"}}},
 }
 for _,st:=range stageOrder { c.Stages=append(c.Stages,StageReceipt{ID:ids[st],Stage:st,Schema:"receipt-v1",OutputSchema:schemas[st],ProfileDigest:c.ProfileDigest,ScopeDigest:c.ScopeDigest,SourceCommit:strings.Repeat("a",40),BuildDigest:d(50+len(c.Stages)),Digest:d(60+len(c.Stages)),OutputDigest:digests[st],Claims:own[st]}) }
 for st,ds:=range dep { for _,e:=range ds { var p StageReceipt; for _,x:=range c.Stages { if x.Stage==e.Producer { p=x } }; for i:=range c.Stages { if c.Stages[i].Stage==st { c.Stages[i].Inputs=append(c.Stages[i].Inputs,Input{Slot:string(e.Producer),Producer:e.Producer,ReceiptID:p.ID,Schema:p.OutputSchema,Digest:p.OutputDigest,Required:e.Required}) } } } }
 for slot,consumer:=range externalSlots { c.External=append(c.External,ExternalBinding{Consumer:consumer,Slot:slot,ReceiptID:"external-"+slot,Schema:"schema-"+slot,Digest:d(100+len(c.External))}) }
 return c
}
func TestPositiveComposition(t *testing.T) { if err:=Validate(base()); err!=nil { t.Fatal(err) } }
func expectCode(t *testing.T,c Capsule,want string) { t.Helper(); if got:=Code(Validate(c)); got!=want { t.Fatalf("got %s, want %s",got,want) } }
func TestPartialOrderObjectsStayDistinct(t *testing.T) {
 c:=base(); c.Progress.Kind="CausalEventFrontier"; expectCode(t,c,"FRONTIER_PROGRESS_CONFLATED")
 c=base(); c.Progress.Universe=Established; expectCode(t,c,"PROGRESS_UNIVERSE_OVERCLAIM")
}
func TestCompletenessRequiresThreeSeparateEvidenceInputs(t *testing.T) {
 c:=base(); c.Frontier.Status="Incomplete"; expectCode(t,c,"FINAL_COMPLETENESS_WIDENING")
 c=base(); c.Owner.Completeness=NotEvaluated; expectCode(t,c,"FINAL_COMPLETENESS_WIDENING")
 c=base(); c.Progress.Status="Provisional"; expectCode(t,c,"FINAL_COMPLETENESS_WIDENING")
}
func TestIndependenceNotSupportedByThisDJCCProfile(t *testing.T) { c:=base(); c.Claims["independence"]=Established; expectCode(t,c,"INDEPENDENCE_PROFILE_UNSUPPORTED") }
func TestAuthorityAndOutcomeDoNotWiden(t *testing.T) {
 c:=base(); c.Authority.Validity=Rejected; expectCode(t,c,"FINAL_AUTHORITY_WIDENING")
 c=base(); c.Outcome.Status="Unknown"; expectCode(t,c,"FINAL_EFFECT_WIDENING")
 c=base(); c.Outcome.Reconciliation="Unresolved"; expectCode(t,c,"FINAL_EFFECT_WIDENING")
}
func TestActionPlanRevisionAndCoverageBinding(t *testing.T) {
 c:=base(); c.Action.CoverageDigest=d(99); expectCode(t,c,"ACTION_DENOMINATOR_MISMATCH")
 c=base(); c.Action.Covered=1; expectCode(t,c,"JUSTIFIED_WITH_UNRESOLVED_BASIS")
}
func TestStageReceiptAndEdgeSubstitutionFail(t *testing.T) {
 c:=base(); c.Stages[0].ID=c.Stages[1].ID; expectCode(t,c,"DUPLICATE_RECEIPT_ID")
 c=base(); c.Stages[1].Inputs[0].Digest=d(888); expectCode(t,c,"PRODUCER_BINDING_MISMATCH")
 c=base(); c.Stages[1].Claims["truth"]=Established; expectCode(t,c,"CLAIM_CEILING_WIDENING")
 c=base(); c.External=c.External[:len(c.External)-1]; expectCode(t,c,"EXTERNAL_BINDINGS_INCOMPLETE")
}
func TestCanonicalDigestDeterministicAndBindsPlan(t *testing.T) {
 c:=base(); first,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }
 for i,j:=0,len(c.Stages)-1;i<j;i,j=i+1,j-1 { c.Stages[i],c.Stages[j]=c.Stages[j],c.Stages[i] }
 second,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }; if first!=second { t.Fatal("stage permutation changed canonical DBC digest") }
 c.Action.Revision="rev-2"; c.Action.Digest=d(200); c.Action.CoverageDigest=d(200)
 third,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }; if first==third { t.Fatal("plan change did not change capsule digest") }
}
func TestKnownAnswerFixtureHasTwentyFourSeeds(t *testing.T) {
 b,err:=os.ReadFile(filepath.Join("..","fixtures","dbc-known-answer-v0.1.json")); if err!=nil { t.Fatal(err) }
 var m map[string]json.RawMessage; if err:=json.Unmarshal(b,&m); err!=nil { t.Fatal(err) }
 var schema string; var cases []json.RawMessage
 _=json.Unmarshal(m["schema_id"],&schema); _=json.Unmarshal(m["cases"],&cases)
 if schema!="mycelix.craft.dbc-known-answer.v0.1" || len(cases)!=24 { t.Fatalf("schema/case count mismatch: %s / %d",schema,len(cases)) }
}
func TestCanonicalDigestDoesNotAliasDifferentProfiles(t *testing.T) {
 c:=base(); one,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }
 c.ProfileDigest=d(250); two,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }
 if one==two { t.Fatal("profile digest was not bound") }
}
var _ = sha256.Sum256
var _ = hex.EncodeToString
var _ = sort.Strings
