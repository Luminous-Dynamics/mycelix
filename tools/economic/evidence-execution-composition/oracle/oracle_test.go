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
		for _, requiredDependencyField := range []string{"stage_id", "input_name", "output_digest", "schema_ref"} {
			if _, exists := dependencyProperties[requiredDependencyField]; !exists {
				t.Errorf("stage %q dependency schema is missing %q", stage, requiredDependencyField)
			}
		}
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
	if seen != 3 {
		t.Fatalf("expected 3 shared stage-edge vectors, evaluated %d", seen)
	}
}

