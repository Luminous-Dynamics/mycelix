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
  ProfileDigest:supportedProfileDigest,DecisionID:"decision-1",QueryID:"query-1",QueryDigest:d(2),QuerySchema:"query-v1",ScopeDigest:d(3),ValidAt:"2026-11-28T10:00:00Z",
  Policy:ref("policy",4),Temporal:struct{ OCELProfile, SourceCorpus, Projection, Replay Ref; FrontierDigest, SourceCorpusDigest string }{ref("ocel-profile",13),ref("source-corpus",14),ref("projection",15),ref("replay",16),d(5),d(14)},
  Frontier:FrontierBinding{Kind:"CausalEventFrontier",Receipt:ref("frontier",5),Status:"Valid",Prefix:Established},
  Progress:ProgressBinding{Kind:"ProgressAntichain",Receipt:ref("progress",6),Status:"StableRelativeToDeclaredProgress",Universe:NotEvaluated,OwnerInventoryDigest:d(7)},
  Owner:OwnerBinding{Receipt:ref("owner-universe",7),Authentication:Established,Completeness:Established,FrontierDigest:d(5)},
  Evidence:EvidenceBinding{IndependenceStatus:NotEvaluated},
  Authority:AuthorityBinding{Receipt:ref("authority",8),Principal:"assessor-1",Scope:"zone-1/decision:close",Validity:Established},
  Action:ActionBinding{Receipt:ref("plan",9),Digest:d(9),Revision:"rev-1",CoverageRevision:"rev-1",Count:2,Covered:2,CoverageDigest:d(9),Coverage:ref("coverage",10)},
  Outcome:OutcomeBinding{Receipt:ref("outcome",11),Status:"Qualified",Reconciliation:"Resolved",ExecutionDigest:ref("execution",12).Digest,ActionPlanDigest:d(9),MutationScope:"water-zone-A/repair-episode-1"},Execution:ref("execution",12),
  Claims:map[string]State{"authentication":Established,"authority":Established,"completeness":Established,"effect":Established,"truth":Established,"independence":NotEvaluated},Disposition:Justified,
 }
 ids:=map[Stage]string{}; schemas:=map[Stage]string{}; digests:=map[Stage]string{}
 for i,st:=range stageOrder { ids[st]=fmt.Sprintf("receipt-%s",st); schemas[st]=fmt.Sprintf("out-%s",st); digests[st]=d(20+i) }
 own:=map[Stage]map[string]State{
  Frontier:{"authentication":Established,"completeness":Established},SourceTrust:{"authentication":Established,"completeness":Established},
  Semantic:{"authentication":Established,"completeness":Established},Query:{"authentication":Established,"completeness":Established,"truth":Established},Policy:{"authentication":Established,"completeness":Established,"truth":Established},
  Authorization:{"authentication":Established,"authority":Established},Execution:{"authentication":Established,"effect":Established},Outcome:{"effect":Established,"truth":Established},
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
 fullClaims:=func(partial map[string]State) map[string]State {
  result:=map[string]State{}
  for _,dim:=range dimensions { result[dim]=NotEvaluated }
  for dim,state:=range partial { result[dim]=state }
  return result
 }
 for _,st:=range stageOrder { c.Stages=append(c.Stages,StageReceipt{ID:ids[st],Stage:st,Schema:"receipt-v1",OutputSchema:schemas[st],ProfileDigest:c.ProfileDigest,ScopeDigest:c.ScopeDigest,SourceCommit:strings.Repeat("a",40),BuildDigest:d(50+len(c.Stages)),Digest:d(60+len(c.Stages)),OutputDigest:digests[st],Claims:fullClaims(own[st])}) }
 stageRef:=func(st Stage) Ref { for _,s:=range c.Stages { if s.Stage==st { return Ref{ID:s.ID,Schema:s.OutputSchema,Digest:s.OutputDigest} } }; return Ref{} }
 c.Frontier.Receipt=stageRef(Frontier)
 c.Policy=stageRef(Policy)
 c.Authority.Receipt=stageRef(Authorization)
 c.Execution=stageRef(Execution)
 c.Outcome.Receipt=stageRef(Outcome)
 c.Owner.FrontierDigest=c.Frontier.Receipt.Digest
 c.Progress.OwnerInventoryDigest=c.Owner.Receipt.Digest
 c.Temporal.FrontierDigest=c.Frontier.Receipt.Digest
 c.Temporal.SourceCorpusDigest=c.Temporal.SourceCorpus.Digest
 c.Outcome.ExecutionDigest=c.Execution.Digest
 c.Outcome.ActionPlanDigest=c.Action.Digest
 for st,ds:=range dep { for _,e:=range ds { var p StageReceipt; for _,x:=range c.Stages { if x.Stage==e.Producer { p=x } }; for i:=range c.Stages { if c.Stages[i].Stage==st { c.Stages[i].Inputs=append(c.Stages[i].Inputs,Input{Slot:string(e.Producer),Producer:e.Producer,ReceiptID:p.ID,Schema:p.OutputSchema,Digest:p.OutputDigest,Required:e.Required}) } } } }
 for slot,consumer:=range externalSlots { r:=externalRefs(c)[slot]; c.External=append(c.External,ExternalBinding{Consumer:consumer,Slot:slot,ReceiptID:r.ID,Schema:r.Schema,Digest:r.Digest}) }
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
 c.Action.Revision="rev-2"; c.Action.CoverageRevision="rev-2"; c.Action.Digest=d(200); c.Action.CoverageDigest=d(200)
 third,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }; if first==third { t.Fatal("plan change did not change capsule digest") }
}
type dbcKAT struct {
	ID string `json:"id"`
	Mutation string `json:"mutation"`
	ExpectedCode string `json:"expected_code"`
	ExpectedDisposition string `json:"expected_disposition"`
}
type dbcKATManifest struct {
	SchemaID string `json:"schema_id"`
	Cases []dbcKAT `json:"cases"`
}

func applyMutation(c Capsule, mutation string) Capsule {
	switch mutation {
	case "none":
	case "conflate_frontier_and_progress":
		c.Progress.Kind = "CausalEventFrontier"
	case "incomplete_frontier":
		c.Frontier.Status = "Incomplete"
	case "provisional_progress":
		c.Progress.Status = "Provisional"
	case "owner_universe_unknown":
		c.Owner.Completeness = NotEvaluated
	case "independence_without_profile":
		c.Claims["independence"] = Established
	case "expired_authority":
		c.Authority.Validity = Rejected
	case "policy_schema_substitution":
		for i := range c.External { if c.External[i].Slot == "policy" { c.External[i].Schema = "schema.substituted-policy" } }
	case "action_denominator_drift":
		c.Action.CoverageDigest = d(999)
	case "outcome_unknown_with_effect":
		c.Outcome.Status = "Unknown"
	case "duplicate_stage":
		c.Stages[1].Stage = c.Stages[0].Stage
	case "duplicate_receipt_id":
		c.Stages[1].ID = c.Stages[0].ID
	case "producer_digest_substitution":
		c.Stages[1].Inputs[0].Digest = d(888)
	case "producer_schema_substitution":
		c.Stages[1].Inputs[0].Schema = "schema.substituted"
	case "claim_ceiling_widening":
		c.Stages[0].Claims["truth"] = Established
	case "partial_action_coverage":
		c.Action.Covered = 1
	case "unresolved_outcome":
		c.Outcome.Reconciliation = "Unresolved"
	case "external_schema_substitution":
		for i := range c.External { if c.External[i].Slot == "source_corpus" { c.External[i].Schema = "schema.substituted-corpus" } }
	case "progress_universe_overclaim":
		c.Progress.Universe = Established
	case "producer_receipt_id_substitution":
		c.Stages[1].Inputs[0].ReceiptID = "receipt-substituted"
	case "missing_owner_universe":
		filtered := make([]ExternalBinding, 0, len(c.External)-1)
		for _, b := range c.External { if b.Slot != "owner_universe" { filtered = append(filtered, b) } }
		c.External = filtered
	case "dependency_slot_substitution":
		c.Stages[1].Inputs[0].Slot = "unrecognized-input-slot"
	case "duplicate_unknown":
		c.Unknowns = []string{"same-unknown", "same-unknown"}
	case "invalid_valid_at":
		c.ValidAt = "not-a-canonical-time"
	case "outcome_unknown_same_scope_mutation":
		c.Outcome.Status = "Unknown"
		c.Outcome.SameScopeMutationAttempt = true
	case "owner_frontier_substitution":
		c.Owner.FrontierDigest = d(777)
	case "progress_inventory_substitution":
		c.Progress.OwnerInventoryDigest = d(778)
	case "temporal_frontier_substitution":
		c.Temporal.FrontierDigest = d(779)
	case "outcome_execution_substitution":
		c.Outcome.ExecutionDigest = d(780)
	case "coverage_revision_drift":
		c.Action.CoverageRevision = "stale-plan-revision"
	default:
		panic("unimplemented DBC KAT mutation: " + mutation)
	}
	return c
}

func TestFrozenKnownAnswerMutationSuite(t *testing.T) {
	b, err := os.ReadFile(filepath.Join("..", "fixtures", "dbc-known-answer-v0.1.json"))
	if err != nil { t.Fatal(err) }
	var manifest dbcKATManifest
	if err := json.Unmarshal(b, &manifest); err != nil { t.Fatal(err) }
	if manifest.SchemaID != "mycelix.craft.dbc-known-answer.v0.1" { t.Fatalf("unexpected KAT schema %q", manifest.SchemaID) }
	if len(manifest.Cases) != 30 { t.Fatalf("expected 30 frozen DBC cases, got %d", len(manifest.Cases)) }
	seen := map[string]bool{}
	for _, item := range manifest.Cases {
		item := item
		t.Run(item.ID, func(t *testing.T) {
			if seen[item.ID] { t.Fatalf("duplicate DBC case ID %s", item.ID) }
			seen[item.ID] = true
			candidate := applyMutation(base(), item.Mutation)
			err := Validate(candidate)
			gotCode := ""
			if err != nil { gotCode = Code(err) }
			if gotCode != item.ExpectedCode {
				t.Fatalf("mutation %s: got code %q, expected %q (err=%v)", item.Mutation, gotCode, item.ExpectedCode, err)
			}
			gotDisposition := string(Justified)
			if err != nil { gotDisposition = string(Undecided) }
			if gotDisposition != item.ExpectedDisposition {
				t.Fatalf("mutation %s: got disposition %q, expected %q", item.Mutation, gotDisposition, item.ExpectedDisposition)
			}
		})
	}
	if len(seen) != len(manifest.Cases) { t.Fatalf("evaluated %d/%d unique DBC KATs", len(seen), len(manifest.Cases)) }
}
func TestCanonicalDigestBindsQueryAndPolicyInputs(t *testing.T) {
 c:=base(); one,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }
 c.QueryDigest=d(250); two,err:=CanonicalDigest(c); if err!=nil { t.Fatal(err) }
 if one==two { t.Fatal("query digest was not bound") }
}
var _ = sha256.Sum256
var _ = hex.EncodeToString
var _ = sort.Strings


func TestExternalBindingsMustMatchExactOwnerReceipt(t *testing.T) {
 c:=base()
 for i:=range c.External { if c.External[i].Slot=="policy" { c.External[i].Digest=d(999) } }
 expectCode(t,c,"EXTERNAL_BINDING_MISMATCH")
}

func TestTemporalSourceReferencesAreIndividuallyBound(t *testing.T) {
 c:=base()
 for i:=range c.External { if c.External[i].Slot=="source_corpus" { c.External[i].ReceiptID="wrong-corpus" } }
 expectCode(t,c,"EXTERNAL_BINDING_MISMATCH")
}

func TestPinnedDJCCProfileDigestIsRequired(t *testing.T) {
 c:=base(); c.ProfileDigest=d(222); expectCode(t,c,"PROFILE_MISMATCH")
}
func TestPolicyAndOutcomeStageReceiptsAreBound(t *testing.T) {
 c:=base(); c.Policy.Digest=d(333)
 for i:=range c.External { if c.External[i].Slot=="policy" { c.External[i].Digest=d(333) } }
 expectCode(t,c,"POLICY_STAGE_BINDING_MISMATCH")
}


func TestJSONWireRoundTripPreservesCanonicalDBC(t *testing.T) {
	original := base()
	before, err := CanonicalDigest(original)
	if err != nil { t.Fatal(err) }
	wire, err := EncodeJSON(original)
	if err != nil { t.Fatal(err) }
	decoded, err := DecodeJSON(wire)
	if err != nil { t.Fatalf("strict DBC JSON decode failed: %v", err) }
	if err := Validate(decoded); err != nil { t.Fatalf("decoded capsule did not validate: %v", err) }
	after, err := CanonicalDigest(decoded)
	if err != nil { t.Fatal(err) }
	if before != after { t.Fatalf("wire round-trip changed canonical DBC: %s != %s", before, after) }
}

func TestJSONWireDecoderRejectsUnknownFieldsAndTrailingValues(t *testing.T) {
	wire, err := EncodeJSON(base())
	if err != nil { t.Fatal(err) }
	unknown := strings.TrimSuffix(string(wire), "}") + `,"unvalidated_extension":true}`
	if _, err := DecodeJSON([]byte(unknown)); err == nil {
		t.Fatal("unknown top-level field was silently ignored")
	}
	if _, err := DecodeJSON(append(wire, []byte("{}")...)); err == nil {
		t.Fatal("a second JSON value after the capsule was accepted")
	}
}

func TestWireSchemaRequiresCrossOwnerBindings(t *testing.T) {
	b, err := os.ReadFile(filepath.Join("..", "schema", "decision-basis-capsule-v0.1.schema.json"))
	if err != nil { t.Fatal(err) }
	var schema map[string]any
	if err := json.Unmarshal(b, &schema); err != nil { t.Fatalf("schema is invalid JSON: %v", err) }
	properties, ok := schema["properties"].(map[string]any)
	if !ok { t.Fatal("schema has no top-level properties") }
	checks := map[string][]string{
		"progress_antichain": {"owner_inventory_digest"},
		"owner_universe": {"authentication", "frontier_digest"},
		"temporal": {"frontier_digest", "source_corpus_digest"},
		"action_plan": {"coverage_revision"},
		"outcome": {"execution_digest", "plan_digest", "mutation_scope", "same_scope_mutation_attempt"},
	}
	for field, requiredFields := range checks {
		obj, ok := properties[field].(map[string]any)
		if !ok { t.Fatalf("schema missing %s object", field) }
		required, ok := obj["required"].([]any)
		if !ok { t.Fatalf("%s has no required-field list", field) }
		for _, requiredField := range requiredFields {
			found := false
			for _, candidate := range required { if candidate == requiredField { found = true; break } }
			if !found { t.Errorf("%s does not require %s", field, requiredField) }
		}
	}
}
