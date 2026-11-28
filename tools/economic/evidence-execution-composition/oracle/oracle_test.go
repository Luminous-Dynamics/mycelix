package oracle

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"encoding/json"
	"os"
	"strings"
	"testing"
)

func TestSeedManifestMatchesIndependentOracle(t *testing.T) {
	manifest, err := LoadManifest("../DJCC_V0_1_SEEDS.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := ValidateManifest(manifest); err != nil {
		t.Fatalf("DJCC seed manifest failed independent validation: %v", err)
	}
}

func TestAllRegisteredCasesArePresentExactlyOnce(t *testing.T) {
	manifest, err := LoadManifest("../DJCC_V0_1_SEEDS.json")
	if err != nil {
		t.Fatal(err)
	}
	if len(manifest.Cases) != len(CaseIDs()) {
		t.Fatalf("got %d cases, oracle expects %d", len(manifest.Cases), len(CaseIDs()))
	}
	seen := make(map[string]bool, len(manifest.Cases))
	for _, scenario := range manifest.Cases {
		if seen[scenario.ID] {
			t.Fatalf("duplicate case %s", scenario.ID)
		}
		seen[scenario.ID] = true
		if _, ok := ExpectedFor(scenario.ID); !ok {
			t.Fatalf("case %s has no independent expected result", scenario.ID)
		}
	}
	for _, id := range CaseIDs() {
		if !seen[id] {
			t.Errorf("manifest is missing oracle case %s", id)
		}
	}
}

func TestExpectedForReturnsDefensiveStageMap(t *testing.T) {
	first, ok := ExpectedFor("DJCC-001")
	if !ok {
		t.Fatal("DJCC-001 missing from oracle")
	}
	first.Stages[StagePolicy] = Rejected
	second, ok := ExpectedFor("DJCC-001")
	if !ok {
		t.Fatal("DJCC-001 missing from oracle on second read")
	}
	if second.Stages[StagePolicy] != VerifiedForDeclaredClaim {
		t.Fatal("caller mutation changed the oracle's expected result")
	}
}

func TestUnauthenticatedRootNeverComposesAsJustified(t *testing.T) {
	result, ok := ExpectedFor("DJCC-002")
	if !ok {
		t.Fatal("DJCC-002 missing from oracle")
	}
	if result.Stages[StageSourceTrust] != Indeterminate ||
		result.Composition != CompositionUnknown ||
		result.ClaimCeiling.Authentication != ClaimUnknown {
		t.Fatalf("unauthenticated root was promoted: %+v", result)
	}
}

func TestHiddenOwnerNeverBecomesFalseAbsence(t *testing.T) {
	result, ok := ExpectedFor("DJCC-003")
	if !ok {
		t.Fatal("DJCC-003 missing from oracle")
	}
	if result.Stages[StageVerifierFrontier] != Indeterminate ||
		result.ClaimCeiling.Completeness != ClaimUnknown ||
		result.Composition != CompositionUnknown {
		t.Fatalf("incomplete owner coverage was promoted: %+v", result)
	}
}

func TestHistoricalFrontierDoesNotImportLaterCorrection(t *testing.T) {
	result, ok := ExpectedFor("DJCC-004")
	if !ok {
		t.Fatal("DJCC-004 missing from oracle")
	}
	if result.Composition != JustifiedAtFrontier ||
		result.Stages[StageSemantic] != VerifiedForDeclaredClaim ||
		result.Stages[StageExecution] != NotApplicable {
		t.Fatalf("historical noninterference expectation changed: %+v", result)
	}
}

func TestOutcomeUnknownBlocksFurtherMutation(t *testing.T) {
	result, ok := ExpectedFor("DJCC-007")
	if !ok {
		t.Fatal("DJCC-007 missing from oracle")
	}
	if result.Stages[StageExecution] != Rejected ||
		result.ClaimCeiling.Effect != ClaimUnknown ||
		result.Composition != CompositionUnknown {
		t.Fatalf("unknown outcome was treated as safe to mutate: %+v", result)
	}
}

func TestProviderCompletionIsNotIndependentOutcome(t *testing.T) {
	result, ok := ExpectedFor("DJCC-008")
	if !ok {
		t.Fatal("DJCC-008 missing from oracle")
	}
	if result.Stages[StageOutcome] != Indeterminate ||
		result.ClaimCeiling.Effect != ClaimUnknown ||
		result.Composition != CompositionUnknown {
		t.Fatalf("provider completion was promoted to an observed outcome: %+v", result)
	}
}

func TestCertificateSchemaIsValidJSONAndContainsAllStages(t *testing.T) {
	raw, err := os.ReadFile("../../../../schemas/economic/decision-justification-composition-certificate-v0.1.schema.json")
	if err != nil {
		t.Fatal(err)
	}
	var schema map[string]any
	if err := json.Unmarshal(raw, &schema); err != nil {
		t.Fatalf("certificate schema is not valid JSON: %v", err)
	}
	if schema["$schema"] != "https://json-schema.org/draft/2020-12/schema" {
		t.Fatalf("unexpected JSON Schema dialect: %v", schema["$schema"])
	}
	properties, ok := schema["properties"].(map[string]any)
	if !ok {
		t.Fatal("certificate schema has no properties object")
	}
	stageEnvelope, ok := properties["stages"].(map[string]any)
	if !ok {
		t.Fatal("certificate schema has no stages envelope")
	}
	stageProperties, ok := stageEnvelope["properties"].(map[string]any)
	if !ok {
		t.Fatal("certificate stages has no property registry")
	}
	for _, stage := range requiredStages {
		rawStage, exists := stageProperties[string(stage)]
		if !exists {
			t.Errorf("certificate schema is missing stage %q", stage)
			continue
		}
		stageSchema, ok := rawStage.(map[string]any)
		if !ok {
			t.Errorf("stage %q is not a schema object", stage)
			continue
		}
		stageSchemaProperties, ok := stageSchema["properties"].(map[string]any)
		if !ok {
			t.Errorf("stage %q has no property registry", stage)
			continue
		}
		if _, exists := stageSchemaProperties["output_schema_ref"]; !exists {
			t.Errorf("stage %q is missing output_schema_ref", stage)
		}
		dependenciesSchema, ok := stageSchemaProperties["dependencies"].(map[string]any)
		if !ok {
			t.Errorf("stage %q has no dependency schema", stage)
			continue
		}
		dependencyItems, ok := dependenciesSchema["items"].(map[string]any)
		if !ok {
			t.Errorf("stage %q dependency items are not an object schema", stage)
			continue
		}
		dependencyProperties, ok := dependencyItems["properties"].(map[string]any)
		if !ok {
			t.Errorf("stage %q dependencies have no properties", stage)
			continue
		}
		for _, requiredDependencyField := range []string{"stage_id", "input_name", "required_claims", "output_digest", "schema_ref"} {
			if _, exists := dependencyProperties[requiredDependencyField]; !exists {
				t.Errorf("stage %q dependency schema is missing %q", stage, requiredDependencyField)
			}
		}
		dependencyRequired, ok := dependencyItems["required"].([]any)
		if !ok {
			t.Errorf("stage %q dependency schema has no required-field list", stage)
			continue
		}
		for _, requiredDependencyField := range []string{"stage_id", "input_name", "required_claims", "output_digest", "schema_ref"} {
			found := false
			for _, field := range dependencyRequired {
				if field == requiredDependencyField {
					found = true
					break
				}
			}
			if !found {
				t.Errorf("stage %q dependency schema does not require %q", stage, requiredDependencyField)
			}
		}
		claimSchema, ok := dependencyProperties["required_claims"].(map[string]any)
		if !ok || claimSchema["type"] != "array" || claimSchema["minItems"] != float64(1) || claimSchema["uniqueItems"] != true {
			t.Errorf("stage %q required_claims schema is not a non-empty unique array", stage)
			continue
		}
		claimItems, ok := claimSchema["items"].(map[string]any)
		if !ok {
			t.Errorf("stage %q required_claims schema has no item vocabulary", stage)
			continue
		}
		claimEnum, ok := claimItems["enum"].([]any)
		includesIndependence := false
		if ok {
			for _, claim := range claimEnum {
				if claim == "independence" { includesIndependence = true; break }
			}
		}
		if !includesIndependence { t.Errorf("stage %q required_claims schema omits independence", stage) }
	}
	stageRequired, ok := stageEnvelope["required"].([]any)
	if !ok || len(stageRequired) != len(requiredStages) {
		t.Fatalf("certificate schema must require all %d stages", len(requiredStages))
	}
}
func TestSharedDJCCStageEdgeCanonicalizationVectors(t *testing.T) {
	file, err := os.Open("../DJCC_STAGE_EDGE_V1_KAT.tsv")
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	seen := 0
	for line := 1; scanner.Scan(); line++ {
		raw := scanner.Text()
		if raw == "" || strings.HasPrefix(raw, "#") {
			continue
		}
		fields := strings.Split(raw, "\t")
		edge, expected, err := parseStageEdgeVectorV1(fields)
		if err != nil {
			t.Fatalf("line %d: %v", line, err)
		}
		actual, err := CanonicalStageEdgeBytesV1(edge)
		if err != nil {
			t.Fatalf("line %d (%s): %v", line, fields[0], err)
		}
		if !bytes.Equal(actual, expected) {
			t.Errorf("line %d (%s): preimage mismatch\n got %s\nwant %s",
				line, fields[0], hex.EncodeToString(actual), hex.EncodeToString(expected))
		}
		seen++
	}
	if err := scanner.Err(); err != nil {
		t.Fatal(err)
	}
	if seen != 4 {
		t.Fatalf("expected 4 shared stage-edge vectors, evaluated %d", seen)
	}
}

func TestCompositionProfilesDeclareClaimOwnersAndNoImplicitNotApplicable(t *testing.T) {
	manifest, err := LoadProfileManifest("../DJCC_PROFILES_V0_1.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := ValidateProfileManifest(manifest); err != nil {
		t.Fatalf("DJCC profiles failed independent validation: %v", err)
	}
	var decisionOnly *CompositionProfile
	for i := range manifest.Profiles {
		if manifest.Profiles[i].ProfileRef == "mycelix.djcc.historical-decision-only.v0.1" {
			decisionOnly = &manifest.Profiles[i]
			break
		}
	}
	if decisionOnly == nil {
		t.Fatal("historical decision-only profile is missing")
	}
	for _, stage := range []string{"execution", "outcome"} {
		if !contains(decisionOnly.AllowedNotApplicable, stage) {
			t.Errorf("decision-only profile must explicitly allow %s as NotApplicable", stage)
		}
	}
	if contains(decisionOnly.ClaimStages["truth"], "outcome") {
		t.Fatal("decision-only truth claim must not depend on an omitted physical outcome stage")
	}
}

func TestProfileManifestRejectsClaimMappedToNotApplicableStage(t *testing.T) {
	manifest, err := LoadProfileManifest("../DJCC_PROFILES_V0_1.json")
	if err != nil {
		t.Fatal(err)
	}
	manifest.Profiles[0].AllowedNotApplicable = append(manifest.Profiles[0].AllowedNotApplicable, "source_trust")
	if err := ValidateProfileManifest(manifest); err == nil {
		t.Fatal("profile allowed a stage to be NotApplicable while claiming it as a claim producer")
	}
}

func TestStageEdgeRejectsUnsortedOrUnknownRequiredClaims(t *testing.T) {
	var digest [32]byte
	edge := StageEdgeV1{
		ProducerStage:  "policy", ConsumerStage: "authorization",
		InputName:      "decision", OutputSchemaRef: "policy:v1",
		RequiredClaims: []string{"truth", "authentication"},
		OutputDigest:   digest,
	}
	if _, err := CanonicalStageEdgeBytesV1(edge); err == nil {
		t.Fatal("unsorted required claims accepted")
	}
	edge.RequiredClaims = []string{"nonexistent"}
	if _, err := CanonicalStageEdgeBytesV1(edge); err == nil {
		t.Fatal("unknown required claim accepted")
	}
}
func TestProfileManifestRejectsWeakenedDependencyClaim(t *testing.T) {
	manifest, err := LoadProfileManifest("../DJCC_PROFILES_V0_1.json")
	if err != nil {
		t.Fatal(err)
	}
	manifest.Profiles[0].DependencyClaims["query"]["source_trust"] = []string{"authentication"}
	if err := ValidateProfileManifest(manifest); err == nil {
		t.Fatal("profile weakened a query's source-trust completeness requirement")
	}
}

func TestProfileManifestDecisionOnlyDoesNotClaimOutcome(t *testing.T) {
	manifest, err := LoadProfileManifest("../DJCC_PROFILES_V0_1.json")
	if err != nil {
		t.Fatal(err)
	}
	profile := manifest.Profiles[1]
	if !contains(profile.AllowedNotApplicable, "execution") ||
		!contains(profile.AllowedNotApplicable, "outcome") {
		t.Fatal("decision-only profile must explicitly mark execution and outcome NotApplicable")
	}
	if contains(profile.ClaimStages["truth"], "outcome") {
		t.Fatal("decision-only profile cannot use an absent outcome to establish truth")
	}
}

func TestIndependentCorroborationProfileRequiresIndependentEvidence(t *testing.T) {
    manifest, err := LoadProfileManifest("../DJCC_PROFILES_V0_1.json")
    if err != nil { t.Fatal(err) }
    if err := ValidateProfileManifest(manifest); err != nil {
        t.Fatalf("DJCC profiles failed independent validation: %v", err)
    }
    var profile *CompositionProfile
    for i := range manifest.Profiles {
        if manifest.Profiles[i].ProfileRef == "mycelix.djcc.independent-corroboration.v0.1" {
            profile = &manifest.Profiles[i]
            break
        }
    }
    if profile == nil { t.Fatal("independent-corroboration profile is missing") }
    if !contains(profile.RequiredClaims, "independence") {
        t.Fatal("independent-corroboration profile does not require independence")
    }
    if contains(profile.ClaimStages["independence"], "outcome") {
        t.Fatal("independence profile cannot claim an unverified physical outcome")
    }
    for _, stage := range []string{"execution", "outcome"} {
        if !contains(profile.AllowedNotApplicable, stage) {
            t.Errorf("independence profile must explicitly allow %s as NotApplicable", stage)
        }
    }
}
