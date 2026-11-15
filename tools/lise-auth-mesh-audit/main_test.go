package main

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
)

func TestValidateRecordRejectsSchemaDrift(t *testing.T) {
	allowAdditional := false
	schema := schemaDoc{
		Draft: draft202012, AdditionalProperties: &allowAdditional,
		Required: []string{"scenario_id", "variant", "family"},
		Properties: map[string]propertySchema{
			"scenario_id": {Type: "string", Pattern: "^AUTH-MESH-[0-9]{3}$"},
			"variant": {Enum: []string{"A", "B"}},
			"family": {Enum: []string{"pep_mediation"}},
		},
	}
	good := json.RawMessage(`{"scenario_id":"AUTH-MESH-001","variant":"A","family":"pep_mediation"}`)
	if err := validateRecord(good, schema, "good"); err != nil {
		t.Fatalf("valid record rejected: %v", err)
	}
	bad := json.RawMessage(`{"scenario_id":"AUTH-MESH-001","variant":"A","family":"decision_honoring"}`)
	if err := validateRecord(bad, schema, "bad"); err == nil {
		t.Fatal("schema enum drift was not rejected")
	}
}

func TestValidateRecordRejectsUnknownFields(t *testing.T) {
	allowAdditional := false
	schema := schemaDoc{Draft: draft202012, AdditionalProperties: &allowAdditional, Required: []string{"id"}, Properties: map[string]propertySchema{"id": {Type: "string"}}}
	data := json.RawMessage(`{"id":"x","unexpected":true}`)
	if err := validateRecord(data, schema, "record"); err == nil {
		t.Fatal("unknown property was accepted")
	}
}

func TestAuditDatasetPassesPairedFixture(t *testing.T) {
	root := t.TempDir()
	spec := fixtureSpec()
	writeFixture(t, root, spec)
	summary, err := auditDataset(root, spec)
	if err != nil {
		t.Fatalf("auditDataset rejected valid fixture: %v", err)
	}
	if summary.ScenarioCount != 2 || summary.PairCount != 1 || summary.OracleCount != 2 {
		t.Fatalf("unexpected summary: %+v", summary)
	}
}

func TestAuditDatasetRejectsMissingOracleRow(t *testing.T) {
	root := t.TempDir()
	spec := fixtureSpec()
	writeFixture(t, root, spec)
	oraclePath := filepath.Join(root, spec.OraclePath)
	var envelope map[string]any
	data, err := os.ReadFile(oraclePath)
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &envelope); err != nil {
		t.Fatal(err)
	}
	envelope["outcome_count"] = 1
	envelope["outcomes"] = envelope["outcomes"].([]any)[:1]
	writeJSON(t, oraclePath, envelope)
	if _, err := auditDataset(root, spec); err == nil {
		t.Fatal("missing oracle row was accepted")
	}
}

func TestAuditDatasetRejectsBadPair(t *testing.T) {
	root := t.TempDir()
	spec := fixtureSpec()
	writeFixture(t, root, spec)
	path := filepath.Join(root, spec.ScenarioPath)
	var envelope map[string]any
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &envelope); err != nil {
		t.Fatal(err)
	}
	records := envelope["scenarios"].([]any)
	records[1].(map[string]any)["variant"] = "A"
	writeJSON(t, path, envelope)
	if _, err := auditDataset(root, spec); err == nil {
		t.Fatal("duplicate A pair variant was accepted")
	}
}

func fixtureSpec() datasetSpec {
	return datasetSpec{
		Name: "fixture", ScenarioPath: "scenarios.json", OraclePath: "oracle.json",
		ScenarioSchemaPath: "scenario.schema.json", OracleSchemaPath: "oracle.schema.json",
		ScenarioPrefix: "AUTH-MESH-", ScenarioCountKey: "scenario_count", OutcomeCountKey: "outcome_count",
		ScenarioArrayKey: "scenarios", OutcomeArrayKey: "outcomes",
		ScenarioAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "scenario_count", "pair_count", "scenarios"},
		OracleAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "outcome_count", "outcomes"},
	}
}

func writeFixture(t *testing.T, root string, spec datasetSpec) {
	t.Helper()
	allowAdditional := false
	scenarioSchema := schemaDoc{Draft: draft202012, AdditionalProperties: &allowAdditional,
		Required: []string{"scenario_id", "pair_id", "variant", "family", "contrast"},
		Properties: map[string]propertySchema{
			"scenario_id": {Type: "string", Pattern: "^AUTH-MESH-[0-9]{3}$"},
			"pair_id": {Type: "string", Pattern: "^M[0-9]{2}$"},
			"variant": {Enum: []string{"A", "B"}},
			"family": {Enum: []string{"fixture_family"}},
			"contrast": {Type: "string", MinLength: 1},
			"declared_claim": {Type: "string", MinLength: 1},
			"observed_facts": {Type: "array", MinItems: 1, Items: &propertySchema{Type: "string", MinLength: 1}},
			"evaluator_instruction": {Type: "string", MinLength: 1},
		},
	}
	oracleSchema := schemaDoc{Draft: draft202012, AdditionalProperties: &allowAdditional,
		Required: []string{"scenario_id", "disposition"},
		Properties: map[string]propertySchema{
			"scenario_id": {Type: "string", Pattern: "^AUTH-MESH-[0-9]{3}$"},
			"disposition": {Enum: []string{"PERMIT", "DENY"}},
		},
	}
	scenarios := map[string]any{"corpus_id": "FIXTURE-v1", "status": "public_development_seed", "disclosure": "Public synthetic set; not a hidden or blinded holdout.", "scenario_count": 2, "pair_count": 1,
		"scenarios": []any{
			map[string]any{"scenario_id": "AUTH-MESH-001", "pair_id": "M01", "variant": "A", "family": "fixture_family", "contrast": "paired", "declared_claim": "a", "observed_facts": []string{"x"}, "evaluator_instruction": "evaluate"},
			map[string]any{"scenario_id": "AUTH-MESH-002", "pair_id": "M01", "variant": "B", "family": "fixture_family", "contrast": "paired", "declared_claim": "b", "observed_facts": []string{"y"}, "evaluator_instruction": "evaluate"},
		}}
	oracle := map[string]any{"corpus_id": "FIXTURE-v1", "status": "public_development_oracle", "disclosure": "Public oracle; calibration only.", "outcome_count": 2,
		"outcomes": []any{
			map[string]any{"scenario_id": "AUTH-MESH-001", "disposition": "PERMIT"},
			map[string]any{"scenario_id": "AUTH-MESH-002", "disposition": "DENY"},
		}}
	writeJSON(t, filepath.Join(root, spec.ScenarioPath), scenarios)
	writeJSON(t, filepath.Join(root, spec.OraclePath), oracle)
	writeJSON(t, filepath.Join(root, spec.ScenarioSchemaPath), scenarioSchema)
	writeJSON(t, filepath.Join(root, spec.OracleSchemaPath), oracleSchema)
}

func writeJSON(t *testing.T, path string, value any) {
	t.Helper()
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	data, err := json.MarshalIndent(value, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, append(data, '\n'), 0o644); err != nil {
		t.Fatal(err)
	}
}
