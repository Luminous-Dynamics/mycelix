package oracle

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"sort"
	"strings"
	"time"
)

type Ref struct {
	ID string `json:"id"`
	Schema string `json:"schema_id"`
	Digest string `json:"digest"`
}

type Cost struct {
	Money float64 `json:"money"`
	Delay float64 `json:"delay"`
	Burden float64 `json:"burden"`
	Risk float64 `json:"risk"`
}

type CostWeights struct {
	Money float64 `json:"money_weight"`
	Delay float64 `json:"delay_weight"`
	Burden float64 `json:"burden_weight"`
	Risk float64 `json:"risk_weight"`
}

type DecisionBasis struct {
	Receipt Ref `json:"receipt"`
	ProfileRef string `json:"profile_ref"`
	Disposition string `json:"disposition"`
	DecisionID string `json:"decision_id"`
	QueryID string `json:"query_id"`
	ScopeDigest string `json:"scope_digest"`
	FrontierDigest string `json:"frontier_digest"`
	DecisionLossProfileDigest string `json:"decision_loss_profile_digest"`
	ValidAt string `json:"valid_at"`
}

type EVSIEvaluation struct {
	Receipt Ref `json:"receipt"`
	FrontierDigest string `json:"frontier_digest"`
	ScopeDigest string `json:"scope_digest"`
	DecisionID string `json:"decision_id"`
	EvaluatorDigest string `json:"evaluator_digest"`
	LossProfileDigest string `json:"loss_profile_digest"`
	CandidateSetDigest string `json:"candidate_set_digest"`
	ObservationModelDigest string `json:"observation_model_digest"`
	ModelStatus string `json:"model_status"`
	EVSI float64 `json:"evsi"`
	EIGBits float64 `json:"eig_bits"`
	Costs Cost `json:"costs"`
	Weights CostWeights `json:"cost_weights"`
	NetDecisionValue float64 `json:"net_decision_value"`
}

type ResultImpact struct {
	ResultID string `json:"result_id"`
	Disposition string `json:"decision_impact"`
	Explanation string `json:"explanation"`
}

type Candidate struct {
	CandidateID string `json:"candidate_id"`
	Method string `json:"method"`
	TargetClaim string `json:"target_claim"`
	QualifiedRole string `json:"qualified_role"`
	Mandatory bool `json:"mandatory"`
	SafeUnderCurrentPolicy bool `json:"safe_under_current_policy"`
	AdmissibleForDecision bool `json:"admissible_for_decision"`
	Available bool `json:"available"`
	InDomain bool `json:"in_domain"`
	ConditionedOnCurrentEvidence bool `json:"conditioned_on_current_evidence"`
	IndependenceRequired bool `json:"independence_required"`
	FailureDomainProfileDigest string `json:"failure_domain_profile_digest"`
	ExistingDependencyGraphDigest string `json:"existing_dependency_graph_digest"`
	ResultImpacts []ResultImpact `json:"result_impacts"`
}

type Proposal struct {
	SchemaID string `json:"schema_id"`
	ProposalID string `json:"proposal_id"`
	CapabilityBoundary string `json:"capability_boundary"`
	AdvisoryOnly bool `json:"advisory_only"`
	AuthorizedEffects []string `json:"authorized_effects"`
	PriorDecisionBasis DecisionBasis `json:"prior_decision_basis"`
	EVSI EVSIEvaluation `json:"evsi_evaluation"`
	Candidate Candidate `json:"candidate"`
	Disposition string `json:"disposition"`
	IssuedAt string `json:"issued_at"`
	ValidUntil string `json:"valid_until"`
	InvalidatedBy []string `json:"invalidated_by"`
	AbstentionReason string `json:"abstention_reason"`
}

type Violation struct {
	Code string `json:"code"`
	Path string `json:"path"`
	Message string `json:"message"`
}

type Report struct {
	ProposalDigest string `json:"proposal_digest"`
	Disposition string `json:"disposition"`
	Violations []Violation `json:"violations"`
}

const (
	SchemaID = "mycelix.craft.evidence-acquisition-proposal.v0.1"
	DBCProfile = "mycelix.djcc.full-evidence-execution.v0.1"
	AdvisoryBoundary = "AdvisoryOnly"
	RecommendOptional = "RecommendOptionalObservation"
	AcquireMandatory = "AcquireMandatoryEvidence"
	RequestReview = "RequestAuthorizedHumanReview"
	DoNotAcquire = "DoNotAcquireOptionalObservation"
	Abstain = "AbstainOrEscalate"
)

var requiredInvalidators = []string{
	"candidate_catalogue_changed",
	"dbc_frontier_changed",
	"decision_scope_changed",
	"observation_model_changed",
	"policy_or_loss_profile_changed",
}

func validDigest(value string) bool {
	if len(value) != 64 || strings.ToLower(value) != value {
		return false
	}
	b, err := hex.DecodeString(value)
	return err == nil && len(b) == 32
}

func validRef(ref Ref) bool {
	return strings.TrimSpace(ref.ID) != "" && strings.TrimSpace(ref.Schema) != "" && validDigest(ref.Digest)
}

func finiteNonNegative(value float64) bool {
	return !math.IsNaN(value) && !math.IsInf(value, 0) && value >= 0
}

func add(violations *[]Violation, code, path, message string) {
	*violations = append(*violations, Violation{Code: code, Path: path, Message: message})
}

func parseCanonicalUTC(value string) (time.Time, bool) {
	parsed, err := time.Parse("2006-01-02T15:04:05Z", value)
	if err != nil || parsed.UTC().Format("2006-01-02T15:04:05Z") != value {
		return time.Time{}, false
	}
	return parsed, true
}

func strictlySorted(values []string) bool {
	for i := 1; i < len(values); i++ {
		if values[i-1] >= values[i] { return false }
	}
	return true
}

func expectedNet(evsi float64, costs Cost, weights CostWeights) float64 {
	return evsi -
		costs.Money*weights.Money -
		costs.Delay*weights.Delay -
		costs.Burden*weights.Burden -
		costs.Risk*weights.Risk
}

func Validate(p Proposal) Report {
	violations := make([]Violation, 0)
	if p.SchemaID != SchemaID { add(&violations, "EAP-SCHEMA", "schema_id", "unsupported proposal schema") }
	switch p.Disposition {
	case RecommendOptional, AcquireMandatory, RequestReview, DoNotAcquire, Abstain:
	default:
		add(&violations, "EAP-DISPOSITION", "disposition", "unknown proposal disposition")
	}
	if strings.TrimSpace(p.ProposalID) == "" { add(&violations, "EAP-ID", "proposal_id", "proposal ID is required") }
	if p.CapabilityBoundary != AdvisoryBoundary || !p.AdvisoryOnly || len(p.AuthorizedEffects) != 0 {
		add(&violations, "EAP-ADVISORY-ONLY", "capability_boundary", "proposal must be advisory-only and authorize no operational effects")
	}
	if !validRef(p.PriorDecisionBasis.Receipt) || p.PriorDecisionBasis.Receipt.Schema != "mycelix.craft.decision-basis-capsule.v0.1" {
		add(&violations, "EAP-DBC-REF", "prior_decision_basis.receipt", "a valid prior DBC reference is required")
	}
	if p.PriorDecisionBasis.ProfileRef != DBCProfile || strings.TrimSpace(p.PriorDecisionBasis.DecisionID) == "" ||
		strings.TrimSpace(p.PriorDecisionBasis.QueryID) == "" || !validDigest(p.PriorDecisionBasis.ScopeDigest) ||
		!validDigest(p.PriorDecisionBasis.FrontierDigest) || !validDigest(p.PriorDecisionBasis.DecisionLossProfileDigest) {
		add(&violations, "EAP-DBC-BINDING", "prior_decision_basis", "prior DBC identity, profile, scope and frontier bindings are incomplete")
	}
	if _, ok := parseCanonicalUTC(p.PriorDecisionBasis.ValidAt); !ok {
		add(&violations, "EAP-DBC-TIME", "prior_decision_basis.valid_at", "prior DBC valid time must be canonical UTC seconds")
	}
	if !validRef(p.EVSI.Receipt) || !validDigest(p.EVSI.EvaluatorDigest) ||
		!validDigest(p.EVSI.LossProfileDigest) || !validDigest(p.EVSI.CandidateSetDigest) ||
		!validDigest(p.EVSI.ObservationModelDigest) {
		add(&violations, "EAP-EVSI-BINDING", "evsi_evaluation", "EVSI receipt, evaluator, loss, candidate-set and observation-model digests are required")
	}
	if p.EVSI.FrontierDigest != p.PriorDecisionBasis.FrontierDigest || p.EVSI.ScopeDigest != p.PriorDecisionBasis.ScopeDigest ||
		p.EVSI.DecisionID != p.PriorDecisionBasis.DecisionID {
		add(&violations, "EAP-EVSI-DBC-MISMATCH", "evsi_evaluation", "EVSI result must bind the exact prior DBC frontier, scope and decision")
	}
	if p.EVSI.LossProfileDigest != p.PriorDecisionBasis.DecisionLossProfileDigest {
		add(&violations, "EAP-LOSS-PROFILE-MISMATCH", "evsi_evaluation.loss_profile_digest", "EVSI result uses a different loss profile from the prior decision basis")
	}
	if !finiteNonNegative(p.EVSI.EVSI) || !finiteNonNegative(p.EVSI.EIGBits) {
		add(&violations, "EAP-EVSI-VALUE", "evsi_evaluation", "EVSI and EIG must be finite and non-negative")
	}
	costs := []float64{p.EVSI.Costs.Money, p.EVSI.Costs.Delay, p.EVSI.Costs.Burden, p.EVSI.Costs.Risk}
	weights := []float64{p.EVSI.Weights.Money, p.EVSI.Weights.Delay, p.EVSI.Weights.Burden, p.EVSI.Weights.Risk}
	for _, value := range costs {
		if !finiteNonNegative(value) { add(&violations, "EAP-COST", "evsi_evaluation.costs", "cost components must be finite and non-negative"); break }
	}
	for _, value := range weights {
		if !finiteNonNegative(value) { add(&violations, "EAP-COST-WEIGHTS", "evsi_evaluation.cost_weights", "cost weights must be finite and non-negative"); break }
	}
	if !finiteNonNegative(math.Abs(p.EVSI.NetDecisionValue)) {
		add(&violations, "EAP-NET-VALUE-NONFINITE", "evsi_evaluation.net_decision_value", "net decision value must be finite")
	}
	if math.Abs(p.EVSI.NetDecisionValue-expectedNet(p.EVSI.EVSI, p.EVSI.Costs, p.EVSI.Weights)) > 1e-8 {
		add(&violations, "EAP-NET-VALUE-MISMATCH", "evsi_evaluation.net_decision_value", "net decision value does not equal EVSI minus the explicitly weighted cost components")
	}
	if p.EVSI.ModelStatus != "CalibratedWithinDeclaredModel" && p.EVSI.ModelStatus != "Uncalibrated" &&
		p.EVSI.ModelStatus != "OutOfDomain" && p.EVSI.ModelStatus != "NotEvaluated" {
		add(&violations, "EAP-MODEL-STATUS", "evsi_evaluation.model_status", "unknown observation-model status")
	}
	if strings.TrimSpace(p.Candidate.CandidateID) == "" || strings.TrimSpace(p.Candidate.Method) == "" ||
		strings.TrimSpace(p.Candidate.TargetClaim) == "" || strings.TrimSpace(p.Candidate.QualifiedRole) == "" {
		add(&violations, "EAP-CANDIDATE-IDENTITY", "candidate", "candidate ID, method, target claim and qualified role are required")
	}
	if !validDigest(p.EVSI.CandidateSetDigest) { add(&violations, "EAP-CANDIDATE-SET", "evsi_evaluation.candidate_set_digest", "candidate set digest is invalid") }
	if p.Candidate.IndependenceRequired && (!validDigest(p.Candidate.FailureDomainProfileDigest) || !validDigest(p.Candidate.ExistingDependencyGraphDigest)) {
		add(&violations, "EAP-INDEPENDENCE-CONTEXT", "candidate", "independence-required proposal must bind a failure-domain profile and existing dependency graph")
	}
	seenResults := map[string]bool{}
	changesDecision := false
	for _, impact := range p.Candidate.ResultImpacts {
		if strings.TrimSpace(impact.ResultID) == "" || seenResults[impact.ResultID] {
			add(&violations, "EAP-RESULT-ID", "candidate.result_impacts", "result IDs must be non-empty and unique")
			break
		}
		seenResults[impact.ResultID] = true
		switch impact.Disposition {
		case "DecisionCouldChange":
			changesDecision = true
		case "RemainsIndeterminate", "NoDecisionChange":
		default:
			add(&violations, "EAP-RESULT-DISPOSITION", "candidate.result_impacts", "unknown result impact disposition")
		}
		if strings.TrimSpace(impact.Explanation) == "" {
			add(&violations, "EAP-RESULT-EXPLANATION", "candidate.result_impacts", "each possible result requires an explanation")
		}
	}
	if len(p.Candidate.ResultImpacts) == 0 {
		add(&violations, "EAP-RESULTS-MISSING", "candidate.result_impacts", "expected result vocabulary and decision impacts are required")
	}
	if (p.Disposition == RecommendOptional) {
		if p.Candidate.Mandatory {
			add(&violations, "EAP-MANDATORY-NOT-OPTIONAL", "disposition", "mandatory evidence cannot be represented as an optional EVSI recommendation")
		}
		if !p.Candidate.SafeUnderCurrentPolicy || !p.Candidate.AdmissibleForDecision ||
			!p.Candidate.Available || !p.Candidate.InDomain || !p.Candidate.ConditionedOnCurrentEvidence ||
			p.EVSI.ModelStatus != "CalibratedWithinDeclaredModel" {
			add(&violations, "EAP-OBSERVATION-NOT-QUALIFIED", "candidate", "optional recommendation requires safe, available, admissible, in-domain and current-evidence-conditioned observation model")
		}
		if p.EVSI.NetDecisionValue <= 0 {
			add(&violations, "EAP-NONPOSITIVE-NET-VALUE", "disposition", "optional observation cannot be recommended with non-positive net decision value")
		}
		if !changesDecision {
			add(&violations, "EAP-NO-DECISION-IMPACT", "candidate.result_impacts", "optional recommendation requires at least one result that can change the decision")
		}
	}
	if p.Candidate.Mandatory && p.Disposition != AcquireMandatory && p.Disposition != RequestReview {
		add(&violations, "EAP-MANDATORY-PRIORITY", "disposition", "mandatory evidence must be retained as a hard constraint or escalated for authorized review")
	}
	if !p.Candidate.Mandatory && p.Disposition == AcquireMandatory {
		add(&violations, "EAP-MANDATORY-MISMATCH", "disposition", "optional candidate cannot be mislabeled as mandatory evidence")
	}
	if p.Disposition == DoNotAcquire && p.EVSI.NetDecisionValue > 0 {
		add(&violations, "EAP-STOP-VALUE-MISMATCH", "disposition", "positive net decision value conflicts with DoNotAcquire")
	}
	if p.Disposition == Abstain || p.Disposition == RequestReview {
		if strings.TrimSpace(p.AbstentionReason) == "" {
			add(&violations, "EAP-ABSTENTION-REASON", "abstention_reason", "abstention or review requires an explicit reason")
		}
	}
	issued, issuedOK := parseCanonicalUTC(p.IssuedAt)
	expires, expiresOK := parseCanonicalUTC(p.ValidUntil)
	if !issuedOK || !expiresOK || !expires.After(issued) {
		add(&violations, "EAP-EXPIRY", "valid_until", "proposal issue and expiry must be canonical UTC seconds with expiry after issue time")
	}
	if !strictlySorted(p.InvalidatedBy) {
		add(&violations, "EAP-INVALIDATION-RULES", "invalidated_by", "invalidation triggers must be sorted and unique")
	}
	for _, required := range requiredInvalidators {
		found := false
		for _, actual := range p.InvalidatedBy { if actual == required { found = true; break } }
		if !found { add(&violations, "EAP-INVALIDATION-INCOMPLETE", "invalidated_by", "missing required invalidation trigger "+required) }
	}
	sort.Slice(violations, func(i,j int) bool {
		if violations[i].Code != violations[j].Code { return violations[i].Code < violations[j].Code }
		if violations[i].Path != violations[j].Path { return violations[i].Path < violations[j].Path }
		return violations[i].Message < violations[j].Message
	})
	canonical, err := json.Marshal(p)
	digest := ""
	if err == nil {
		sum := sha256.Sum256(append([]byte("MYCELIX-CRAFT-EAP-V0.1\x00"), canonical...))
		digest = hex.EncodeToString(sum[:])
	}
	disposition := p.Disposition
	if len(violations) > 0 { disposition = Abstain }
	return Report{ProposalDigest:digest,Disposition:disposition,Violations:violations}
}

func CanonicalDigest(p Proposal) (string,error) {
	report := Validate(p)
	if len(report.Violations) > 0 { return "", fmt.Errorf("%s: %s",report.Violations[0].Code,report.Violations[0].Message) }
	p.InvalidatedBy = append([]string(nil),p.InvalidatedBy...)
	p.AuthorizedEffects = append([]string(nil),p.AuthorizedEffects...)
	p.Candidate.ResultImpacts = append([]ResultImpact(nil),p.Candidate.ResultImpacts...)
	sort.Strings(p.InvalidatedBy)
	sort.Strings(p.AuthorizedEffects)
	sort.Slice(p.Candidate.ResultImpacts,func(i,j int)bool{return p.Candidate.ResultImpacts[i].ResultID<p.Candidate.ResultImpacts[j].ResultID})
	b,err:=json.Marshal(p)
	if err!=nil{return "",err}
	sum:=sha256.Sum256(append([]byte("MYCELIX-CRAFT-EAP-V0.1\x00"),b...))
	return hex.EncodeToString(sum[:]),nil
}
