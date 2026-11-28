package oracle

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"sort"
)

type StageID string
type Disposition string
type CompositionDisposition string
type ClaimState string

const (
	StageVerifierFrontier StageID = "verifier_frontier"
	StageSourceTrust      StageID = "source_trust"
	StageSemantic         StageID = "semantic_projection"
	StageQuery            StageID = "query"
	StagePolicy           StageID = "policy"
	StageAuthorization    StageID = "authorization"
	StageExecution        StageID = "execution"
	StageOutcome          StageID = "outcome"

	VerifiedForDeclaredClaim Disposition = "VerifiedForDeclaredClaim"
	Rejected                 Disposition = "Rejected"
	Indeterminate            Disposition = "Indeterminate"
	NotEvaluated             Disposition = "NotEvaluated"
	UnsupportedByProfile     Disposition = "UnsupportedByProfile"
	NotApplicable            Disposition = "NotApplicable"

	JustifiedAtFrontier CompositionDisposition = "DecisionJustifiedAtAcceptedFrontier"
	NotJustified        CompositionDisposition = "DecisionNotJustifiedAtAcceptedFrontier"
	CompositionUnknown  CompositionDisposition = "Indeterminate"
	ReplayMismatch      CompositionDisposition = "HistoricalReplayMismatch"
	CompositionUnsupported CompositionDisposition = "UnsupportedByProfile"

	Established ClaimState = "EstablishedWithinProfile"
	ClaimRejected ClaimState = "Rejected"
	ClaimUnknown ClaimState = "Indeterminate"
	ClaimNotEvaluated ClaimState = "NotEvaluated"
	ClaimUnsupported ClaimState = "UnsupportedByProfile"
	ClaimNotApplicable ClaimState = "NotApplicable"
)

var requiredStages = []StageID{
	StageVerifierFrontier,
	StageSourceTrust,
	StageSemantic,
	StageQuery,
	StagePolicy,
	StageAuthorization,
	StageExecution,
	StageOutcome,
}

type Mutation struct {
	Operator string `json:"operator"`
	Target   string `json:"target"`
}

type Controls struct {
	PositiveControlRef string `json:"positive_control_ref"`
	NegativeControlRef string `json:"negative_control_ref"`
}

type ClaimCeiling struct {
	Authentication ClaimState `json:"authentication"`
	Completeness   ClaimState `json:"completeness"`
	Truth          ClaimState `json:"truth"`
	Authority      ClaimState `json:"authority"`
	Effect         ClaimState `json:"effect"`
}

type Scenario struct {
	ID                          string               `json:"id"`
	Title                       string               `json:"title"`
	Class                       string               `json:"class"`
	RiskTier                    string               `json:"risk_tier"`
	SyntheticOnly               bool                 `json:"synthetic_only"`
	FixtureRef                  string               `json:"fixture_ref"`
	Mutation                    *Mutation            `json:"mutation"`
	Controls                    Controls             `json:"controls"`
	ExpectedStages              map[StageID]Disposition `json:"expected_stages"`
	ExpectedComposition         CompositionDisposition `json:"expected_composition"`
	ExpectedClaimCeiling        ClaimCeiling          `json:"expected_claim_ceiling"`
	Rationale                   string               `json:"rationale"`
}

type Manifest struct {
	Schema            string     `json:"schema"`
	Status            string     `json:"status"`
	ClaimBoundary     string     `json:"claim_boundary"`
	OracleRule        string     `json:"oracle_rule"`
	OracleVersion     string     `json:"oracle_version"`
	ControlSemantics  string     `json:"control_semantics"`
	StageIDs          []StageID  `json:"stage_ids"`
	Dispositions      []Disposition `json:"dispositions"`
	Cases             []Scenario `json:"cases"`
}

type ExpectedResult struct {
	Stages       map[StageID]Disposition
	Composition  CompositionDisposition
	ClaimCeiling ClaimCeiling
	Class        string
	RiskTier     string
}

func stages(frontier, source, semantic, query, policy, authorization, execution, outcome Disposition) map[StageID]Disposition {
	return map[StageID]Disposition{
		StageVerifierFrontier: frontier,
		StageSourceTrust: source,
		StageSemantic: semantic,
		StageQuery: query,
		StagePolicy: policy,
		StageAuthorization: authorization,
		StageExecution: execution,
		StageOutcome: outcome,
	}
}

func ceiling(authentication, completeness, truth, authority, effect ClaimState) ClaimCeiling {
	return ClaimCeiling{
		Authentication: authentication,
		Completeness: completeness,
		Truth: truth,
		Authority: authority,
		Effect: effect,
	}
}

// expected is maintained independently of the candidate manifest and verifier.
// Changes to an expected disposition require an explicit review of the rationale.
var expected = map[string]ExpectedResult{
	"DJCC-001": {
		Stages: stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim),
		Composition: JustifiedAtFrontier,
		ClaimCeiling: ceiling(Established, Established, Established, Established, Established),
		Class: "positive-control", RiskTier: "control",
	},
	"DJCC-002": {
		Stages: stages(VerifiedForDeclaredClaim, Indeterminate, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated),
		Composition: CompositionUnknown,
		ClaimCeiling: ceiling(ClaimUnknown, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated),
		Class: "hard-safety", RiskTier: "critical",
	},
	"DJCC-003": {
		Stages: stages(Indeterminate, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated),
		Composition: CompositionUnknown,
		ClaimCeiling: ceiling(ClaimNotEvaluated, ClaimUnknown, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated),
		Class: "hard-safety", RiskTier: "critical",
	},
	"DJCC-004": {
		Stages: stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, NotApplicable, NotApplicable),
		Composition: JustifiedAtFrontier,
		ClaimCeiling: ceiling(Established, Established, Established, Established, ClaimNotApplicable),
		Class: "historical-noninterference", RiskTier: "control",
	},
	"DJCC-005": {
		Stages: stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Rejected, NotEvaluated, NotEvaluated, NotEvaluated),
		Composition: ReplayMismatch,
		ClaimCeiling: ceiling(Established, Established, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated),
		Class: "hard-safety", RiskTier: "critical",
	},
	"DJCC-006": {
		Stages: stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Rejected, NotEvaluated, NotEvaluated),
		Composition: NotJustified,
		ClaimCeiling: ceiling(Established, Established, ClaimNotEvaluated, ClaimRejected, ClaimNotEvaluated),
		Class: "hard-safety", RiskTier: "critical",
	},
	"DJCC-007": {
		Stages: stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Rejected, NotEvaluated),
		Composition: CompositionUnknown,
		ClaimCeiling: ceiling(Established, Established, ClaimNotEvaluated, Established, ClaimUnknown),
		Class: "hard-safety", RiskTier: "critical",
	},
	"DJCC-008": {
		Stages: stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Indeterminate),
		Composition: CompositionUnknown,
		ClaimCeiling: ceiling(Established, ClaimUnknown, ClaimUnknown, Established, ClaimUnknown),
		Class: "hard-safety", RiskTier: "critical",
	},
}

func ExpectedFor(caseID string) (ExpectedResult, bool) {
	result, ok := expected[caseID]
	if !ok {
		return ExpectedResult{}, false
	}
	copied := make(map[StageID]Disposition, len(result.Stages))
	for key, value := range result.Stages {
		copied[key] = value
	}
	result.Stages = copied
	return result, true
}

func LoadManifest(path string) (Manifest, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return Manifest{}, fmt.Errorf("read DJCC seed manifest: %w", err)
	}
	var manifest Manifest
	if err := json.Unmarshal(raw, &manifest); err != nil {
		return Manifest{}, fmt.Errorf("decode DJCC seed manifest: %w", err)
	}
	return manifest, nil
}

// ValidateManifest compares the public seed manifest to an independently authored
// expectation table. This validates benchmark registry integrity only; it does
// not evaluate a candidate DJCC certificate or establish any trust claim.
func ValidateManifest(manifest Manifest) error {
	if manifest.Schema != "mycelix.djcc-benchmark-seeds.v0.1" {
		return errors.New("unsupported DJCC seed manifest schema")
	}
	if manifest.Status != "synthetic-protocol-seeds-not-yet-executable" {
		return errors.New("unexpected seed manifest status")
	}
	if manifest.ClaimBoundary == "" || manifest.OracleRule == "" || manifest.ControlSemantics == "" {
		return errors.New("seed manifest must state claim and oracle boundaries")
	}
	if manifest.OracleVersion != "mycelix.djcc-independent-go-oracle.v0.1" {
		return errors.New("unexpected independent-oracle version")
	}
	if len(manifest.StageIDs) != len(requiredStages) {
		return errors.New("stage ID registry is incomplete")
	}
	stageSet := make(map[StageID]struct{}, len(manifest.StageIDs))
	for _, stage := range manifest.StageIDs {
		if _, exists := stageSet[stage]; exists {
			return fmt.Errorf("duplicate stage ID %q", stage)
		}
		stageSet[stage] = struct{}{}
	}
	for _, required := range requiredStages {
		if _, exists := stageSet[required]; !exists {
			return fmt.Errorf("required stage ID %q is missing", required)
		}
	}
	if len(manifest.Cases) != len(expected) {
		return fmt.Errorf("expected %d frozen seed cases, received %d", len(expected), len(manifest.Cases))
	}
	seen := make(map[string]struct{}, len(manifest.Cases))
	controls := make(map[string]struct{}, len(manifest.Cases)*2)
	for _, scenario := range manifest.Cases {
		if _, duplicate := seen[scenario.ID]; duplicate {
			return fmt.Errorf("duplicate DJCC case ID %q", scenario.ID)
		}
		seen[scenario.ID] = struct{}{}
		want, exists := ExpectedFor(scenario.ID)
		if !exists {
			return fmt.Errorf("unregistered DJCC case ID %q", scenario.ID)
		}
		if !scenario.SyntheticOnly || scenario.FixtureRef == "" || scenario.Rationale == "" || scenario.Title == "" {
			return fmt.Errorf("%s lacks synthetic fixture identity or rationale", scenario.ID)
		}
		if scenario.Class != want.Class || scenario.RiskTier != want.RiskTier {
			return fmt.Errorf("%s has an unexpected class or risk tier", scenario.ID)
		}
		if len(scenario.ExpectedStages) != len(requiredStages) {
			return fmt.Errorf("%s must declare every stage exactly once", scenario.ID)
		}
		for _, stage := range requiredStages {
			if _, ok := scenario.ExpectedStages[stage]; !ok {
				return fmt.Errorf("%s omits expected stage %q", scenario.ID, stage)
			}
		}
		if !equalStages(scenario.ExpectedStages, want.Stages) {
			return fmt.Errorf("%s stage dispositions differ from independent oracle", scenario.ID)
		}
		if scenario.ExpectedComposition != want.Composition {
			return fmt.Errorf("%s composition disposition differs from independent oracle", scenario.ID)
		}
		if scenario.ExpectedClaimCeiling != want.ClaimCeiling {
			return fmt.Errorf("%s claim ceiling differs from independent oracle", scenario.ID)
		}
		if scenario.Class == "positive-control" && scenario.Mutation != nil {
			return fmt.Errorf("%s positive control must not contain a mutation", scenario.ID)
		}
		if scenario.Class == "hard-safety" && scenario.Mutation == nil {
			return fmt.Errorf("%s hard-safety case must declare a mutation", scenario.ID)
		}
		if scenario.Mutation != nil && (scenario.Mutation.Operator == "" || scenario.Mutation.Target == "") {
			return fmt.Errorf("%s mutation must name operator and target", scenario.ID)
		}
		for _, controlRef := range []string{scenario.Controls.PositiveControlRef, scenario.Controls.NegativeControlRef} {
			if controlRef == "" {
				return fmt.Errorf("%s is missing a positive/negative control", scenario.ID)
			}
			if _, duplicate := controls[controlRef]; duplicate {
				return fmt.Errorf("control fixture %q is reused", controlRef)
			}
			controls[controlRef] = struct{}{}
		}
	}
	return nil
}

func equalStages(left, right map[StageID]Disposition) bool {
	if len(left) != len(right) {
		return false
	}
	for stage, disposition := range right {
		if left[stage] != disposition {
			return false
		}
	}
	return true
}

func CaseIDs() []string {
	ids := make([]string, 0, len(expected))
	for id := range expected {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	return ids
}
