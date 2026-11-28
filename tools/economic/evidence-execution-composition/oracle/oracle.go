package oracle

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"reflect"
	"sort"
	"strings"
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
	Independence   ClaimState `json:"independence"`
}

type Scenario struct {
	ID                   string                  `json:"id"`
	Title                string                  `json:"title"`
	Class                string                  `json:"class"`
	RiskTier             string                  `json:"risk_tier"`
	SyntheticOnly        bool                    `json:"synthetic_only"`
	FixtureRef           string                  `json:"fixture_ref"`
	Mutation             *Mutation               `json:"mutation"`
	Controls             Controls                `json:"controls"`
	ExpectedStages       map[StageID]Disposition `json:"expected_stages"`
	ExpectedComposition  CompositionDisposition  `json:"expected_composition"`
	ExpectedClaimCeiling ClaimCeiling            `json:"expected_claim_ceiling"`
	Rationale            string                  `json:"rationale"`
}

type Manifest struct {
	Schema           string        `json:"schema"`
	Status           string        `json:"status"`
	ClaimBoundary    string        `json:"claim_boundary"`
	OracleRule       string        `json:"oracle_rule"`
	OracleVersion    string        `json:"oracle_version"`
	ControlSemantics string        `json:"control_semantics"`
	StageIDs         []StageID     `json:"stage_ids"`
	Dispositions     []Disposition `json:"dispositions"`
	Cases            []Scenario    `json:"cases"`
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
		StageSourceTrust:      source,
		StageSemantic:         semantic,
		StageQuery:            query,
		StagePolicy:           policy,
		StageAuthorization:    authorization,
		StageExecution:        execution,
		StageOutcome:          outcome,
	}
}

func ceiling(authentication, completeness, truth, authority, effect ClaimState, independence ...ClaimState) ClaimCeiling {
	independenceState := ClaimNotEvaluated
	if len(independence) > 0 {
		independenceState = independence[0]
	}
	return ClaimCeiling{
		Authentication: authentication,
		Completeness:   completeness,
		Truth:          truth,
		Authority:      authority,
		Effect:         effect,
		Independence:   independenceState,
	}
}

// expected is maintained independently of the candidate manifest and verifier.
// Changes to an expected disposition require an explicit review of the rationale.
var expected = map[string]ExpectedResult{
	"DJCC-001": {
		Stages:       stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim),
		Composition:  JustifiedAtFrontier,
		ClaimCeiling: ceiling(Established, Established, Established, Established, Established),
		Class:        "positive-control", RiskTier: "control",
	},
	"DJCC-002": {
		Stages:       stages(VerifiedForDeclaredClaim, Indeterminate, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated),
		Composition:  CompositionUnknown,
		ClaimCeiling: ceiling(ClaimUnknown, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated),
		Class:        "hard-safety", RiskTier: "critical",
	},
	"DJCC-003": {
		Stages:       stages(Indeterminate, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated, NotEvaluated),
		Composition:  CompositionUnknown,
		ClaimCeiling: ceiling(ClaimNotEvaluated, ClaimUnknown, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated),
		Class:        "hard-safety", RiskTier: "critical",
	},
	"DJCC-004": {
		Stages:       stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, NotApplicable, NotApplicable),
		Composition:  JustifiedAtFrontier,
		ClaimCeiling: ceiling(Established, Established, Established, Established, ClaimNotApplicable),
		Class:        "historical-noninterference", RiskTier: "control",
	},
	"DJCC-005": {
		Stages:       stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Rejected, NotEvaluated, NotEvaluated, NotEvaluated),
		Composition:  ReplayMismatch,
		ClaimCeiling: ceiling(Established, Established, ClaimNotEvaluated, ClaimNotEvaluated, ClaimNotEvaluated),
		Class:        "hard-safety", RiskTier: "critical",
	},
	"DJCC-006": {
		Stages:       stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Rejected, NotEvaluated, NotEvaluated),
		Composition:  NotJustified,
		ClaimCeiling: ceiling(Established, Established, ClaimNotEvaluated, ClaimRejected, ClaimNotEvaluated),
		Class:        "hard-safety", RiskTier: "critical",
	},
	"DJCC-007": {
		Stages:       stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Rejected, NotEvaluated),
		Composition:  CompositionUnknown,
		ClaimCeiling: ceiling(Established, Established, ClaimNotEvaluated, Established, ClaimUnknown),
		Class:        "hard-safety", RiskTier: "critical",
	},
	"DJCC-008": {
		Stages:       stages(VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, VerifiedForDeclaredClaim, Indeterminate),
		Composition:  CompositionUnknown,
		ClaimCeiling: ceiling(Established, ClaimUnknown, ClaimUnknown, Established, ClaimUnknown),
		Class:        "hard-safety", RiskTier: "critical",
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


type CompositionProfile struct {
	ProfileRef           string                         `json:"profile_ref"`
	RequiredClaims       []string                       `json:"required_claims"`
	AllowedNotApplicable []string                       `json:"allowed_not_applicable"`
	ClaimStages          map[string][]string            `json:"claim_stages"`
	DependencyClaims     map[string]map[string][]string `json:"dependency_claims"`
}

type ProfileManifest struct {
	Schema   string               `json:"schema"`
	Status   string               `json:"status"`
	Profiles []CompositionProfile `json:"profiles"`
}

var canonicalClaimOrder = []string{
	"authentication",
	"authority",
	"completeness",
	"effect",
	"independence",
	"truth",
}

var canonicalStageOrder = []string{
	"verifier_frontier",
	"source_trust",
	"semantic_projection",
	"query",
	"policy",
	"authorization",
	"execution",
	"outcome",
}

func LoadProfileManifest(path string) (ProfileManifest, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return ProfileManifest{}, fmt.Errorf("read DJCC profile manifest: %w", err)
	}
	var manifest ProfileManifest
	if err := json.Unmarshal(raw, &manifest); err != nil {
		return ProfileManifest{}, fmt.Errorf("decode DJCC profile manifest: %w", err)
	}
	return manifest, nil
}

func strictlyOrdered(values []string, order []string) bool {
	rank := make(map[string]int, len(order))
	for i, value := range order {
		rank[value] = i
	}
	last := -1
	for _, value := range values {
		current, ok := rank[value]
		if !ok || current <= last {
			return false
		}
		last = current
	}
	return true
}

type expectedProfileV01 struct {
	RequiredClaims       []string
	AllowedNotApplicable []string
	ClaimStages          map[string][]string
	DependencyClaims     map[string]map[string][]string
}

var expectedProfilesV01 = map[string]expectedProfileV01{
	"mycelix.djcc.full-evidence-execution.v0.1": {
		RequiredClaims:       []string{"authentication", "authority", "completeness", "effect", "truth"},
		AllowedNotApplicable: []string{},
		ClaimStages:          map[string][]string{
			"authentication": {"verifier_frontier", "source_trust"},
			"authority":      {"authorization"},
			"completeness":   {"verifier_frontier", "source_trust", "semantic_projection", "query"},
			"effect":         {"execution", "outcome"},
			"truth":          {"query", "policy", "outcome"},
		},
		DependencyClaims: map[string]map[string][]string{
			"source_trust": {
				"verifier_frontier": {"authentication", "completeness"},
			},
			"semantic_projection": {
				"verifier_frontier": {"authentication"},
				"source_trust":      {"authentication"},
			},
			"query": {
				"verifier_frontier":   {"authentication", "completeness"},
				"source_trust":        {"authentication", "completeness"},
				"semantic_projection": {"authentication", "completeness"},
			},
			"policy": {
				"query": {"authentication", "completeness", "truth"},
			},
			"authorization": {
				"verifier_frontier": {"authentication"},
				"policy":            {"authentication", "truth"},
			},
			"execution": {
				"authorization": {"authentication", "authority"},
			},
			"outcome": {
				"source_trust":        {"authentication"},
				"semantic_projection": {"completeness"},
				"execution":           {"effect"},
			},
		},
	},
	"mycelix.djcc.historical-decision-only.v0.1": {
		RequiredClaims:       []string{"authentication", "authority", "completeness", "truth"},
		AllowedNotApplicable: []string{"execution", "outcome"},
		ClaimStages:          map[string][]string{
			"authentication": {"verifier_frontier", "source_trust"},
			"authority":      {"authorization"},
			"completeness":   {"verifier_frontier", "source_trust", "semantic_projection", "query"},
			"truth":          {"query", "policy"},
		},
		DependencyClaims: map[string]map[string][]string{
			"source_trust": {
				"verifier_frontier": {"authentication", "completeness"},
			},
			"semantic_projection": {
				"verifier_frontier": {"authentication"},
				"source_trust":      {"authentication"},
			},
			"query": {
				"verifier_frontier":   {"authentication", "completeness"},
				"source_trust":        {"authentication", "completeness"},
				"semantic_projection": {"authentication", "completeness"},
			},
			"policy": {
				"query": {"authentication", "completeness", "truth"},
			},
			"authorization": {
				"verifier_frontier": {"authentication"},
				"policy":            {"authentication", "truth"},
			},
		},
	},
	"mycelix.djcc.independent-corroboration.v0.1": {
		RequiredClaims:       []string{"authentication", "authority", "completeness", "independence", "truth"},
		AllowedNotApplicable: []string{"execution", "outcome"},
		ClaimStages:          map[string][]string{
			"authentication": {"verifier_frontier", "source_trust"},
			"authority":      {"authorization"},
			"completeness":   {"verifier_frontier", "source_trust", "semantic_projection", "query"},
			"independence":   {"source_trust", "semantic_projection", "query"},
			"truth":          {"query", "policy"},
		},
		DependencyClaims: map[string]map[string][]string{
			"source_trust": {
				"verifier_frontier": {"authentication", "completeness"},
			},
			"semantic_projection": {
				"verifier_frontier": {"authentication"},
				"source_trust":      {"authentication"},
			},
			"query": {
				"verifier_frontier":   {"authentication", "completeness"},
				"source_trust":        {"authentication", "completeness", "independence"},
				"semantic_projection": {"authentication", "completeness", "independence"},
			},
			"policy": {
				"query": {"authentication", "completeness", "independence", "truth"},
			},
			"authorization": {
				"verifier_frontier": {"authentication"},
				"policy":            {"authentication", "truth"},
			},
		},
	},
}

func ValidateProfileAgainstIndependentOracle(profile CompositionProfile) error {
	expected, ok := expectedProfilesV01[profile.ProfileRef]
	if !ok {
		return fmt.Errorf("unregistered DJCC profile %q", profile.ProfileRef)
	}
	if !reflect.DeepEqual(profile.RequiredClaims, expected.RequiredClaims) {
		return fmt.Errorf("%s required root claim set differs from independent oracle", profile.ProfileRef)
	}
	if !reflect.DeepEqual(profile.AllowedNotApplicable, expected.AllowedNotApplicable) {
		return fmt.Errorf("%s NotApplicable stage set differs from independent oracle", profile.ProfileRef)
	}
	if !reflect.DeepEqual(profile.ClaimStages, expected.ClaimStages) {
		return fmt.Errorf("%s root claim-stage ownership differs from independent oracle", profile.ProfileRef)
	}
	if !reflect.DeepEqual(profile.DependencyClaims, expected.DependencyClaims) {
		return fmt.Errorf("%s dependency claim-edge matrix differs from independent oracle", profile.ProfileRef)
	}
	return nil
}

func ValidateProfileManifest(manifest ProfileManifest) error {
	if manifest.Schema != "mycelix.djcc-composition-profiles.v0.1" {
		return errors.New("unsupported DJCC composition profile manifest schema")
	}
	if manifest.Status != "proposed-structural-profiles-not-yet-authoritative" {
		return errors.New("unexpected DJCC profile manifest status")
	}
	if len(manifest.Profiles) != 3 {
		return fmt.Errorf("expected three DJCC V0.1 profiles, got %d", len(manifest.Profiles))
	}
	seenProfiles := make(map[string]struct{}, len(manifest.Profiles))
	stageRank := make(map[string]int, len(canonicalStageOrder))
	for index, stage := range canonicalStageOrder {
		stageRank[stage] = index
	}
	for _, profile := range manifest.Profiles {
		if strings.TrimSpace(profile.ProfileRef) == "" {
			return errors.New("DJCC profile reference is empty")
		}
		if _, duplicate := seenProfiles[profile.ProfileRef]; duplicate {
			return fmt.Errorf("duplicate DJCC profile %q", profile.ProfileRef)
		}
		seenProfiles[profile.ProfileRef] = struct{}{}
		if len(profile.RequiredClaims) == 0 || !strictlyOrdered(profile.RequiredClaims, canonicalClaimOrder) {
			return fmt.Errorf("%s required claims must be non-empty, canonical and unique", profile.ProfileRef)
		}
		if !strictlyOrdered(profile.AllowedNotApplicable, canonicalStageOrder) && len(profile.AllowedNotApplicable) != 0 {
			return fmt.Errorf("%s NotApplicable stages must be canonical and unique", profile.ProfileRef)
		}
		if len(profile.ClaimStages) != len(profile.RequiredClaims) {
			return fmt.Errorf("%s claim-stage map must cover each required claim exactly once", profile.ProfileRef)
		}
		if err := ValidateProfileAgainstIndependentOracle(profile); err != nil {
			return err
		}
		for _, claim := range profile.RequiredClaims {
			stages, ok := profile.ClaimStages[claim]
			if !ok || len(stages) == 0 {
				return fmt.Errorf("%s has no producing stages for claim %s", profile.ProfileRef, claim)
			}
			if !strictlyOrdered(stages, canonicalStageOrder) {
				return fmt.Errorf("%s stages for claim %s are not canonical/unique", profile.ProfileRef, claim)
			}
			for _, stage := range stages {
				if contains(profile.AllowedNotApplicable, stage) {
					return fmt.Errorf("%s marks %s NotApplicable while using it to establish %s", profile.ProfileRef, stage, claim)
				}
			}
		}
		for consumer, parents := range profile.DependencyClaims {
			consumerRank, ok := stageRank[consumer]
			if !ok {
				return fmt.Errorf("%s has unknown consumer stage %s", profile.ProfileRef, consumer)
			}
			for parent, claims := range parents {
				parentRank, ok := stageRank[parent]
				if !ok || parentRank >= consumerRank {
					return fmt.Errorf("%s has invalid dependency edge %s <- %s", profile.ProfileRef, consumer, parent)
				}
				if len(claims) == 0 || !strictlyOrdered(claims, canonicalClaimOrder) {
					return fmt.Errorf("%s edge %s <- %s has empty/non-canonical required claims", profile.ProfileRef, consumer, parent)
				}
			}
		}
	}
	return nil
}

func contains(values []string, wanted string) bool {
	for _, value := range values {
		if value == wanted {
			return true
		}
	}
	return false
}

func CaseIDs() []string {
	ids := make([]string, 0, len(expected))
	for id := range expected {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	return ids
}
