package oracle

import (
	"encoding/json"
	"fmt"
	"os"
	"strings"
	"path/filepath"
	"testing"
)

func d(n int) string { return fmt.Sprintf("%064x", n+1) }
func ref(id, schema string, n int) Ref { return Ref{ID:id,Schema:schema,Digest:d(n)} }

func base() Proposal {
	return Proposal{
		SchemaID: SchemaID,
		ProposalID: "proposal-001",
		CapabilityBoundary: AdvisoryBoundary,
		AdvisoryOnly: true,
		AuthorizedEffects: []string{},
		PriorDecisionBasis: DecisionBasis{
			Receipt: ref("dbc-001","mycelix.craft.decision-basis-capsule.v0.1",1),
			ProfileRef: DBCProfile,
			Disposition: "Indeterminate",
			DecisionID: "decision-001",
			QueryID: "query-001",
			ScopeDigest: d(2),
			FrontierDigest: d(3),
			ValidAt: "2026-11-28T10:00:00Z",
		},
		LossProfile: ref("water-service-loss-v1","mycelix.craft.decision-loss-profile.v0.1",4),
		EVSI: EVSIEvaluation{
			Receipt: ref("evsi-001","mycelix.craft.evsi-evaluation-receipt.v0.1",5),
			FrontierDigest: d(3),
			ScopeDigest: d(2),
			DecisionID: "decision-001",
			EvaluatorDigest: d(6),
			LossProfileDigest: d(4),
			CandidateSetDigest: d(7),
			ObservationModelDigest: d(8),
			ModelStatus: "CalibratedWithinDeclaredModel",
			EVSI: 9.95,
			EIGBits: 0.0454146923337941,
			Costs: Cost{Money:1.0,Delay:0.25,Burden:0.25,Risk:0.1},
			Weights: CostWeights{Money:1,Delay:1,Burden:1,Risk:1},
			NetDecisionValue: 8.35,
		},
		Candidate: Candidate{
			CandidateID:"pressure-independent-lab-sample",
			Method:"qualified-laboratory-sample",
			TargetClaim:"water-quality-status",
			QualifiedRole:"accredited-sampler-and-laboratory",
			Mandatory:false,
			CandidateSetDigest:d(7),
			ObservationModelDigest:d(8),
			SafeUnderCurrentPolicy:true,
			AdmissibleForDecision:true,
			Available:true,
			InDomain:true,
			ConditionedOnCurrentEvidence:true,
			IndependenceRequired:false,
			FailureDomainProfileDigest:"",
			ExistingDependencyGraphDigest:"",
			ResultImpacts:[]ResultImpact{
				{ResultID:"unsafe",Disposition:"DecisionCouldChange",Explanation:"Unsafe result requires protective response"},
				{ResultID:"safe",Disposition:"DecisionCouldChange",Explanation:"Qualified safe result can change the release decision"},
				{ResultID:"uncertain",Disposition:"RemainsIndeterminate",Explanation:"Ambiguous result requires further review"},
			},
		},
		Disposition:RecommendOptional,
		IssuedAt:"2026-11-28T10:05:00Z",
		ValidUntil:"2026-11-28T10:35:00Z",
		InvalidatedBy:append([]string(nil),requiredInvalidators...),
		AbstentionReason:"",
	}
}

func codeOf(p Proposal) string {
	r := Validate(p)
	if len(r.Violations)==0 { return "" }
	return r.Violations[0].Code
}

type eapKAT struct {
	ID string `json:"id"`
	Mutation string `json:"mutation"`
	ExpectedCode string `json:"expected_code"`
	ExpectedDisposition string `json:"expected_disposition"`
}

type eapKATManifest struct {
	SchemaID string `json:"schema_id"`
	Cases []eapKAT `json:"cases"`
}

func applyMutation(p Proposal, mutation string) Proposal {
	switch mutation {
	case "none":
	case "stale_prior_frontier":
		p.EVSI.FrontierDigest=d(99)
	case "altered_loss_profile":
		p.EVSI.LossProfileDigest=d(99)
	case "high_eig_zero_evsi":
		p.EVSI.EVSI=0
		p.EVSI.EIGBits=1
		p.EVSI.NetDecisionValue=-1.6
	case "low_eig_high_evsi":
		p.EVSI.EIGBits=0.0001
	case "mandatory_evidence":
		p.Candidate.Mandatory=true
		p.Disposition=AcquireMandatory
		p.EVSI.EVSI=0
		p.EVSI.EIGBits=0
		p.EVSI.NetDecisionValue=-1.6
	case "common_cause_independence_overclaim":
		p.Candidate.IndependenceRequired=true
	case "out_of_domain_model":
		p.Candidate.InDomain=false
	case "no_result_can_change_decision":
		for i:=range p.Candidate.ResultImpacts { p.Candidate.ResultImpacts[i].Disposition="RemainsIndeterminate" }
	case "attempt_operational_effect":
		p.AuthorizedEffects=[]string{"dispatch-crew"}
	case "expired_proposal_interval":
		p.ValidUntil="2026-11-28T10:04:00Z"
	case "duplicate_result_id":
		p.Candidate.ResultImpacts[1].ResultID=p.Candidate.ResultImpacts[0].ResultID
	case "candidate_set_substitution":
		p.Candidate.CandidateSetDigest=d(999)
	case "observation_model_substitution":
		p.Candidate.ObservationModelDigest=d(999)
	case "proposal_predates_dbc":
		p.IssuedAt="2026-11-28T09:00:00Z"
	default:
		panic("unimplemented EAP mutation: "+mutation)
	}
	return p
}

func TestPositiveAdvisoryProposal(t *testing.T) {
	p:=base()
	r:=Validate(p)
	if len(r.Violations)!=0 { t.Fatalf("positive control rejected: %#v",r.Violations) }
	if r.Disposition!=RecommendOptional { t.Fatalf("got %s",r.Disposition) }
	if r.ProposalDigest=="" || !validDigest(r.ProposalDigest) { t.Fatalf("invalid proposal digest %q",r.ProposalDigest) }
}

func TestFrozenKnownAnswerMutationSuite(t *testing.T) {
	data,err:=os.ReadFile(filepath.Join("..","fixtures","eap-known-answer-v0.1.json"))
	if err!=nil { t.Fatal(err) }
	var manifest eapKATManifest
	if err:=json.Unmarshal(data,&manifest); err!=nil { t.Fatal(err) }
	if manifest.SchemaID!="mycelix.craft.eap-known-answer.v0.1" { t.Fatalf("unexpected schema %q",manifest.SchemaID) }
	if len(manifest.Cases)!=15 { t.Fatalf("expected 15 EAP KATs, got %d",len(manifest.Cases)) }
	seen:=map[string]bool{}
	for _,item:=range manifest.Cases {
		item:=item
		t.Run(item.ID,func(t *testing.T){
			if seen[item.ID] { t.Fatalf("duplicate EAP KAT %s",item.ID) }
			seen[item.ID]=true
			p:=applyMutation(base(),item.Mutation)
			got:=Validate(p)
			code:=""
			if len(got.Violations)>0 { code=got.Violations[0].Code }
			if code!=item.ExpectedCode { t.Fatalf("%s: got %q expected %q: %#v",item.Mutation,code,item.ExpectedCode,got.Violations) }
			if got.Disposition!=item.ExpectedDisposition { t.Fatalf("%s: got disposition %q expected %q",item.Mutation,got.Disposition,item.ExpectedDisposition) }
		})
	}
	if len(seen)!=len(manifest.Cases) { t.Fatalf("ran %d/%d KATs",len(seen),len(manifest.Cases)) }
}

func TestCanonicalProposalBindsPriorDBCAndLossProfile(t *testing.T) {
	p:=base()
	first,err:=CanonicalDigest(p); if err!=nil { t.Fatal(err) }
	p.PriorDecisionBasis.FrontierDigest=d(999)
	p.EVSI.FrontierDigest=d(999)
	second,err:=CanonicalDigest(p); if err!=nil { t.Fatal(err) }
	if first==second { t.Fatal("new prior DBC frontier did not change proposal commitment") }
	p=base()
	first,err=CanonicalDigest(p); if err!=nil { t.Fatal(err) }
	p.LossProfile.Digest=d(888)
	p.EVSI.LossProfileDigest=d(888)
	third,err:=CanonicalDigest(p); if err!=nil { t.Fatal(err) }
	if first==third { t.Fatal("new decision-loss profile did not change proposal commitment") }
}

func TestMandatoryEvidenceCannotBeSkippedForPositiveOptionalScore(t *testing.T) {
	p:=base()
	p.Candidate.Mandatory=true
	p.Disposition=DoNotAcquire
	if got:=codeOf(p); got!="EAP-MANDATORY-PRIORITY" { t.Fatalf("got %s",got) }
}

func TestSchemaIsValidJSONAndAdvisoryOnly(t *testing.T) {
	data,err:=os.ReadFile(filepath.Join("..","schema","evidence-acquisition-proposal-v0.1.schema.json"))
	if err!=nil { t.Fatal(err) }
	var schema map[string]any
	if err:=json.Unmarshal(data,&schema); err!=nil { t.Fatalf("schema invalid JSON: %v",err) }
	properties,ok:=schema["properties"].(map[string]any)
	if !ok { t.Fatal("schema properties missing") }
	capability,ok:=properties["capability_boundary"].(map[string]any)
	if !ok || capability["const"]!=AdvisoryBoundary { t.Fatal("schema does not pin AdvisoryOnly capability") }
	effects,ok:=properties["authorized_effects"].(map[string]any)
	if !ok || effects["maxItems"]!=float64(0) { t.Fatal("schema permits operational effects") }
	candidate:=properties["candidate"].(map[string]any)
	candidateRequired:=candidate["required"].([]any)
	for _,name:=range []string{"candidate_set_digest","observation_model_digest"} {
		found:=false
		for _,candidate:=range candidateRequired { if candidate==name { found=true;break } }
		if !found { t.Errorf("candidate schema does not require %s",name) }
	}
}


func TestEAPJSONWireRoundTripPreservesCanonicalDigest(t *testing.T) {
	p:=base()
	before,err:=CanonicalDigest(p)
	if err!=nil { t.Fatal(err) }
	wire,err:=EncodeJSON(p)
	if err!=nil { t.Fatal(err) }
	decoded,err:=DecodeJSON(wire)
	if err!=nil { t.Fatalf("decode EAP wire proposal: %v",err) }
	r:=Validate(decoded)
	if len(r.Violations)!=0 { t.Fatalf("decoded EAP proposal invalid: %#v",r.Violations) }
	after,err:=CanonicalDigest(decoded)
	if err!=nil { t.Fatal(err) }
	if before!=after { t.Fatalf("wire round trip changed proposal digest: %s != %s",before,after) }
}

func TestEAPJSONWireRejectsDuplicateUnknownAndTrailingFields(t *testing.T) {
	wire,err:=EncodeJSON(base())
	if err!=nil { t.Fatal(err) }
	duplicate:=strings.Replace(string(wire), `"proposal_id":"proposal-001"`, `"proposal_id":"proposal-001","proposal_id":"forged"`,1)
	if duplicate==string(wire) { t.Fatal("duplicate-key test fixture did not match proposal_id") }
	if _,err:=DecodeJSON([]byte(duplicate)); err==nil { t.Fatal("duplicate JSON key accepted") }
	unknown:=strings.TrimSuffix(string(wire),"}") + `, "forged_effect":true}`
	if _,err:=DecodeJSON([]byte(unknown)); err==nil { t.Fatal("unknown JSON field accepted") }
	if _,err:=DecodeJSON(append(wire,[]byte("{}")...)); err==nil { t.Fatal("trailing JSON value accepted") }
}

func TestEAPSchemaRequiresNoEffectsAndPriorBindings(t *testing.T) {
	data,err:=os.ReadFile(filepath.Join("..","schema","evidence-acquisition-proposal-v0.1.schema.json"))
	if err!=nil { t.Fatal(err) }
	var schema map[string]any
	if err:=json.Unmarshal(data,&schema); err!=nil { t.Fatalf("EAP schema invalid JSON: %v",err) }
	properties:=schema["properties"].(map[string]any)
	effects:=properties["authorized_effects"].(map[string]any)
	if effects["maxItems"]!=float64(0) { t.Fatal("schema permits authorized effects") }
	if _,exists:=properties["loss_profile"]; !exists { t.Fatal("EAP does not reference a separate decision-loss profile receipt") }
	prior:=properties["prior_decision_basis"].(map[string]any)
	priorRequired:=prior["required"].([]any)
	for _,name:=range []string{"receipt","frontier_digest","scope_digest"} {
		found:=false
		for _,candidate:=range priorRequired { if candidate==name { found=true;break } }
		if !found { t.Errorf("prior DBC schema does not require %s",name) }
	}
	evsi:=properties["evsi_evaluation"].(map[string]any)
	evsiRequired:=evsi["required"].([]any)
	for _,name:=range []string{"frontier_digest","scope_digest","decision_id","loss_profile_digest","candidate_set_digest","observation_model_digest"} {
		found:=false
		for _,candidate:=range evsiRequired { if candidate==name { found=true;break } }
		if !found { t.Errorf("EVSI schema does not require %s",name) }
	}
}
