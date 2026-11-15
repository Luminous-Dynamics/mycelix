package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"strings"
)

const draft202012 = "https://json-schema.org/draft/2020-12/schema"

type schemaDoc struct {
	Draft                string                    `json:"$schema"`
	AdditionalProperties *bool                     `json:"additionalProperties"`
	Required             []string                  `json:"required"`
	Properties           map[string]propertySchema `json:"properties"`
}

type propertySchema struct {
	Type        string          `json:"type"`
	Pattern     string          `json:"pattern"`
	Enum        []string        `json:"enum"`
	MinLength   int             `json:"minLength"`
	MinItems    int             `json:"minItems"`
	UniqueItems bool            `json:"uniqueItems"`
	Items       *propertySchema `json:"items"`
}

type datasetSpec struct {
	Name                   string
	ScenarioPath           string
	OraclePath             string
	ScenarioSchemaPath     string
	OracleSchemaPath       string
	ScenarioPrefix         string
	ScenarioCountKey       string
	OutcomeCountKey        string
	ScenarioArrayKey       string
	OutcomeArrayKey        string
	ScenarioAllowedTopKeys []string
	OracleAllowedTopKeys   []string
}

type datasetSummary struct {
	Name             string            `json:"name"`
	CorpusID         string            `json:"corpus_id"`
	ScenarioCount    int               `json:"scenario_count"`
	PairCount        int               `json:"pair_count"`
	OracleCount      int               `json:"oracle_count"`
	FamilyCounts     map[string]int    `json:"family_counts"`
	DispositionCount map[string]int    `json:"disposition_counts,omitempty"`
	FileSHA256       map[string]string `json:"file_sha256"`
	Status           string            `json:"status"`
}

type pairState struct {
	variants map[string]bool
	family   string
	contrast string
}

var defaultSpecs = []datasetSpec{
	{
		Name: "auth-mesh",
		ScenarioPath: "docs/research/auth-mesh/scenarios.v1.json",
		OraclePath: "docs/research/auth-mesh/oracle.v1.json",
		ScenarioSchemaPath: "docs/research/auth-mesh/scenario.schema.json",
		OracleSchemaPath: "docs/research/auth-mesh/oracle.schema.json",
		ScenarioPrefix: "AUTH-MESH-", ScenarioCountKey: "scenario_count", OutcomeCountKey: "outcome_count",
		ScenarioArrayKey: "scenarios", OutcomeArrayKey: "outcomes",
		ScenarioAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "scenario_count", "pair_count", "property_axes", "scenarios"},
		OracleAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "outcome_count", "outcomes"},
	},
	{
		Name: "acs",
		ScenarioPath: "docs/research/auth-mesh/acs-scenarios.v1.json",
		OraclePath: "docs/research/auth-mesh/acs-oracle.v1.json",
		ScenarioSchemaPath: "docs/research/auth-mesh/acs-scenario.schema.json",
		OracleSchemaPath: "docs/research/auth-mesh/acs-oracle.schema.json",
		ScenarioPrefix: "ACS-", ScenarioCountKey: "scenario_count", OutcomeCountKey: "outcome_count",
		ScenarioArrayKey: "scenarios", OutcomeArrayKey: "outcomes",
		ScenarioAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "scenario_count", "pair_count", "dimensions", "scenarios"},
		OracleAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "outcome_count", "outcomes"},
	},
	{
		Name: "acs-extension",
		ScenarioPath: "docs/research/auth-mesh/acs-extension-scenarios.v1.json",
		OraclePath: "docs/research/auth-mesh/acs-extension-oracle.v1.json",
		ScenarioSchemaPath: "docs/research/auth-mesh/acs-extension-scenario.schema.json",
		OracleSchemaPath: "docs/research/auth-mesh/acs-extension-oracle.schema.json",
		ScenarioPrefix: "ACS-EXT-", ScenarioCountKey: "scenario_count", OutcomeCountKey: "outcome_count",
		ScenarioArrayKey: "scenarios", OutcomeArrayKey: "outcomes",
		ScenarioAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "scenario_count", "pair_count", "scenarios"},
		OracleAllowedTopKeys: []string{"corpus_id", "status", "disclosure", "outcome_count", "outcomes"},
	},
}

func main() {
	root := flag.String("root", ".", "repository root containing the LISE corpus files")
	jsonOutput := flag.Bool("json", false, "emit machine-readable JSON summary")
	flag.Parse()

	summaries := make([]datasetSummary, 0, len(defaultSpecs))
	var failures []string
	for _, spec := range defaultSpecs {
		summary, err := auditDataset(*root, spec)
		if err != nil {
			failures = append(failures, fmt.Sprintf("%s: %v", spec.Name, err))
			continue
		}
		summaries = append(summaries, summary)
	}

	if *jsonOutput {
		enc := json.NewEncoder(os.Stdout)
		enc.SetIndent("", "  ")
		_ = enc.Encode(map[string]any{"ok": len(failures) == 0, "datasets": summaries, "errors": failures})
	} else {
		for _, summary := range summaries {
			fmt.Printf("PASS %-14s scenarios=%d pairs=%d oracle=%d corpus=%s\n", summary.Name, summary.ScenarioCount, summary.PairCount, summary.OracleCount, summary.CorpusID)
		}
		for _, failure := range failures {
			fmt.Fprintf(os.Stderr, "FAIL %s\n", failure)
		}
	}
	if len(failures) > 0 {
		os.Exit(1)
	}
}

func auditDataset(root string, spec datasetSpec) (datasetSummary, error) {
	var summary datasetSummary
	scenarioPath := filepath.Join(root, spec.ScenarioPath)
	oraclePath := filepath.Join(root, spec.OraclePath)
	scenarioSchemaPath := filepath.Join(root, spec.ScenarioSchemaPath)
	oracleSchemaPath := filepath.Join(root, spec.OracleSchemaPath)

	scenarioBytes, scenarioHash, err := readHashed(scenarioPath)
	if err != nil {
		return summary, err
	}
	oracleBytes, oracleHash, err := readHashed(oraclePath)
	if err != nil {
		return summary, err
	}
	scenarioSchemaBytes, scenarioSchemaHash, err := readHashed(scenarioSchemaPath)
	if err != nil {
		return summary, err
	}
	oracleSchemaBytes, oracleSchemaHash, err := readHashed(oracleSchemaPath)
	if err != nil {
		return summary, err
	}

	scenarioEnvelope, err := decodeObject(scenarioBytes)
	if err != nil {
		return summary, fmt.Errorf("scenario envelope: %w", err)
	}
	oracleEnvelope, err := decodeObject(oracleBytes)
	if err != nil {
		return summary, fmt.Errorf("oracle envelope: %w", err)
	}
	if err := checkKeys(scenarioEnvelope, spec.ScenarioAllowedTopKeys, "scenario envelope"); err != nil {
		return summary, err
	}
	if err := checkKeys(oracleEnvelope, spec.OracleAllowedTopKeys, "oracle envelope"); err != nil {
		return summary, err
	}

	corpusID, err := rawString(scenarioEnvelope, "corpus_id")
	if err != nil {
		return summary, err
	}
	oracleCorpusID, err := rawString(oracleEnvelope, "corpus_id")
	if err != nil {
		return summary, err
	}
	if corpusID != oracleCorpusID {
		return summary, fmt.Errorf("corpus_id mismatch: scenarios=%q oracle=%q", corpusID, oracleCorpusID)
	}

	status, err := rawString(scenarioEnvelope, "status")
	if err != nil {
		return summary, err
	}
	if !strings.HasPrefix(status, "public_") {
		return summary, fmt.Errorf("unexpected corpus status %q; public calibration status required", status)
	}
	disclosure, err := rawString(scenarioEnvelope, "disclosure")
	if err != nil {
		return summary, err
	}
	lowerDisclosure := strings.ToLower(disclosure)
	if !strings.Contains(lowerDisclosure, "public") || !strings.Contains(lowerDisclosure, "not a hidden or blinded holdout") {
		return summary, errors.New("scenario disclosure must explicitly identify public corpus/holdout status")
	}
	oracleStatus, err := rawString(oracleEnvelope, "status")
	if err != nil {
		return summary, err
	}
	if !strings.HasPrefix(oracleStatus, "public_") {
		return summary, fmt.Errorf("unexpected oracle status %q; public oracle status required", oracleStatus)
	}
	oracleDisclosure, err := rawString(oracleEnvelope, "disclosure")
	if err != nil {
		return summary, err
	}
	if !strings.Contains(strings.ToLower(oracleDisclosure), "public") {
		return summary, errors.New("oracle disclosure must identify the answer key as public")
	}

	scenarioCount, err := rawInt(scenarioEnvelope, spec.ScenarioCountKey)
	if err != nil {
		return summary, err
	}
	pairCount, err := rawInt(scenarioEnvelope, "pair_count")
	if err != nil {
		return summary, err
	}
	outcomeCount, err := rawInt(oracleEnvelope, spec.OutcomeCountKey)
	if err != nil {
		return summary, err
	}

	scenarioRaw, err := rawArray(scenarioEnvelope, spec.ScenarioArrayKey)
	if err != nil {
		return summary, err
	}
	outcomeRaw, err := rawArray(oracleEnvelope, spec.OutcomeArrayKey)
	if err != nil {
		return summary, err
	}
	if len(scenarioRaw) != scenarioCount {
		return summary, fmt.Errorf("scenario_count says %d but array has %d records", scenarioCount, len(scenarioRaw))
	}
	if len(outcomeRaw) != outcomeCount {
		return summary, fmt.Errorf("outcome_count says %d but array has %d records", outcomeCount, len(outcomeRaw))
	}
	if len(scenarioRaw) != len(outcomeRaw) {
		return summary, fmt.Errorf("scenario/oracle denominator mismatch: %d vs %d", len(scenarioRaw), len(outcomeRaw))
	}

	scenarioSchema, err := decodeSchema(scenarioSchemaBytes)
	if err != nil {
		return summary, fmt.Errorf("scenario schema: %w", err)
	}
	oracleSchema, err := decodeSchema(oracleSchemaBytes)
	if err != nil {
		return summary, fmt.Errorf("oracle schema: %w", err)
	}
	if err := validateSchemaContract(scenarioSchema, "scenario"); err != nil {
		return summary, err
	}
	if err := validateSchemaContract(oracleSchema, "oracle"); err != nil {
		return summary, err
	}

	pairs := make(map[string]*pairState)
	seenScenarios := make(map[string]bool, len(scenarioRaw))
	families := make(map[string]int)
	scenarioIDs := make([]string, 0, len(scenarioRaw))
	for index, raw := range scenarioRaw {
		label := fmt.Sprintf("scenario[%d]", index)
		if err := validateRecord(raw, scenarioSchema, label); err != nil {
			return summary, err
		}
		obj, _ := decodeObject(raw)
		id, _ := rawString(obj, "scenario_id")
		expectedID := fmt.Sprintf("%s%03d", spec.ScenarioPrefix, index+1)
		if id != expectedID {
			return summary, fmt.Errorf("%s has id %q; expected deterministic id %q", label, id, expectedID)
		}
		if seenScenarios[id] {
			return summary, fmt.Errorf("duplicate scenario_id %q", id)
		}
		seenScenarios[id] = true
		scenarioIDs = append(scenarioIDs, id)

		pairID, _ := rawString(obj, "pair_id")
		variant, _ := rawString(obj, "variant")
		family, _ := rawString(obj, "family")
		contrast, _ := rawString(obj, "contrast")
		state := pairs[pairID]
		if state == nil {
			state = &pairState{variants: make(map[string]bool), family: family, contrast: contrast}
			pairs[pairID] = state
		} else if state.family != family || state.contrast != contrast {
			return summary, fmt.Errorf("pair %q changes family or contrast between variants", pairID)
		}
		if state.variants[variant] {
			return summary, fmt.Errorf("pair %q repeats variant %q", pairID, variant)
		}
		state.variants[variant] = true
		families[family]++
	}
	if len(pairs) != pairCount {
		return summary, fmt.Errorf("pair_count says %d but found %d distinct pair IDs", pairCount, len(pairs))
	}
	for pairID, state := range pairs {
		if len(state.variants) != 2 || !state.variants["A"] || !state.variants["B"] {
			return summary, fmt.Errorf("pair %q must contain exactly one A and one B variant", pairID)
		}
	}

	seenOutcomes := make(map[string]bool, len(outcomeRaw))
	dispositions := make(map[string]int)
	for index, raw := range outcomeRaw {
		label := fmt.Sprintf("oracle[%d]", index)
		if err := validateRecord(raw, oracleSchema, label); err != nil {
			return summary, err
		}
		obj, _ := decodeObject(raw)
		id, _ := rawString(obj, "scenario_id")
		if !seenScenarios[id] {
			return summary, fmt.Errorf("oracle contains orphan scenario_id %q", id)
		}
		if seenOutcomes[id] {
			return summary, fmt.Errorf("oracle repeats scenario_id %q", id)
		}
		seenOutcomes[id] = true
		if index < len(scenarioIDs) && scenarioIDs[index] != id {
			return summary, fmt.Errorf("oracle order mismatch at index %d: %q != %q", index, id, scenarioIDs[index])
		}
		if rawDisp, ok := obj["disposition"]; ok {
			var value string
			if err := json.Unmarshal(rawDisp, &value); err != nil {
				return summary, fmt.Errorf("%s disposition: %w", label, err)
			}
			dispositions[value]++
		} else if rawDisp, ok := obj["protocol_conformance"]; ok {
			var value string
			if err := json.Unmarshal(rawDisp, &value); err != nil {
				return summary, fmt.Errorf("%s protocol_conformance: %w", label, err)
			}
			dispositions[value]++
		}
	}
	if len(seenOutcomes) != len(seenScenarios) {
		return summary, fmt.Errorf("oracle covers %d scenario IDs; expected %d", len(seenOutcomes), len(seenScenarios))
	}

	return datasetSummary{
		Name: spec.Name, CorpusID: corpusID, ScenarioCount: len(scenarioRaw), PairCount: len(pairs), OracleCount: len(outcomeRaw),
		FamilyCounts: families, DispositionCount: dispositions, Status: status,
		FileSHA256: map[string]string{
			spec.ScenarioPath: scenarioHash, spec.OraclePath: oracleHash,
			spec.ScenarioSchemaPath: scenarioSchemaHash, spec.OracleSchemaPath: oracleSchemaHash,
		},
	}, nil
}

func readHashed(path string) ([]byte, string, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return nil, "", fmt.Errorf("read %s: %w", path, err)
	}
	sum := sha256.Sum256(b)
	return b, hex.EncodeToString(sum[:]), nil
}

func decodeObject(data []byte) (map[string]json.RawMessage, error) {
	dec := json.NewDecoder(bytes.NewReader(data))
	var obj map[string]json.RawMessage
	if err := dec.Decode(&obj); err != nil {
		return nil, err
	}
	if obj == nil {
		return nil, errors.New("expected JSON object")
	}
	if err := ensureEOF(dec); err != nil {
		return nil, err
	}
	return obj, nil
}

func ensureEOF(dec *json.Decoder) error {
	var extra any
	err := dec.Decode(&extra)
	if err == io.EOF {
		return nil
	}
	if err == nil {
		return errors.New("multiple JSON values in one file")
	}
	return err
}

func rawString(obj map[string]json.RawMessage, key string) (string, error) {
	raw, ok := obj[key]
	if !ok {
		return "", fmt.Errorf("missing required field %q", key)
	}
	var value string
	if err := json.Unmarshal(raw, &value); err != nil {
		return "", fmt.Errorf("field %q must be a string: %w", key, err)
	}
	if strings.TrimSpace(value) == "" {
		return "", fmt.Errorf("field %q must not be empty", key)
	}
	return value, nil
}

func rawInt(obj map[string]json.RawMessage, key string) (int, error) {
	raw, ok := obj[key]
	if !ok {
		return 0, fmt.Errorf("missing required integer field %q", key)
	}
	var value int
	if err := json.Unmarshal(raw, &value); err != nil {
		return 0, fmt.Errorf("field %q must be an integer: %w", key, err)
	}
	if value < 0 {
		return 0, fmt.Errorf("field %q must not be negative", key)
	}
	return value, nil
}

func rawArray(obj map[string]json.RawMessage, key string) ([]json.RawMessage, error) {
	raw, ok := obj[key]
	if !ok {
		return nil, fmt.Errorf("missing required array field %q", key)
	}
	var values []json.RawMessage
	if err := json.Unmarshal(raw, &values); err != nil {
		return nil, fmt.Errorf("field %q must be an array: %w", key, err)
	}
	return values, nil
}

func checkKeys(obj map[string]json.RawMessage, allowed []string, label string) error {
	set := make(map[string]bool, len(allowed))
	for _, key := range allowed {
		set[key] = true
	}
	for key := range obj {
		if !set[key] {
			return fmt.Errorf("%s has unexpected top-level field %q", label, key)
		}
	}
	return nil
}

func decodeSchema(data []byte) (schemaDoc, error) {
	var schema schemaDoc
	dec := json.NewDecoder(bytes.NewReader(data))
	if err := dec.Decode(&schema); err != nil {
		return schema, err
	}
	if err := ensureEOF(dec); err != nil {
		return schema, err
	}
	return schema, nil
}

func validateSchemaContract(schema schemaDoc, label string) error {
	if schema.Draft != draft202012 {
		return fmt.Errorf("%s schema has unexpected draft %q", label, schema.Draft)
	}
	if schema.AdditionalProperties == nil || *schema.AdditionalProperties {
		return fmt.Errorf("%s schema must set additionalProperties=false", label)
	}
	if len(schema.Required) == 0 || len(schema.Properties) == 0 {
		return fmt.Errorf("%s schema must declare required fields and properties", label)
	}
	seen := make(map[string]bool)
	for _, key := range schema.Required {
		if seen[key] {
			return fmt.Errorf("%s schema repeats required field %q", label, key)
		}
		seen[key] = true
		if _, ok := schema.Properties[key]; !ok {
			return fmt.Errorf("%s schema requires undeclared property %q", label, key)
		}
	}
	for key, property := range schema.Properties {
		if property.Pattern != "" {
			if _, err := regexp.Compile(property.Pattern); err != nil {
				return fmt.Errorf("%s schema property %q has invalid pattern: %w", label, key, err)
			}
		}
		if property.Type == "array" && property.Items == nil {
			return fmt.Errorf("%s schema property %q is array but has no items schema", label, key)
		}
		if property.Type == "" && len(property.Enum) == 0 {
			return fmt.Errorf("%s schema property %q has no supported type or enum", label, key)
		}
	}
	return nil
}

func validateRecord(data json.RawMessage, schema schemaDoc, label string) error {
	obj, err := decodeObject(data)
	if err != nil {
		return fmt.Errorf("%s: %w", label, err)
	}
	for _, required := range schema.Required {
		if _, ok := obj[required]; !ok {
			return fmt.Errorf("%s is missing required field %q", label, required)
		}
	}
	for key, raw := range obj {
		property, ok := schema.Properties[key]
		if !ok {
			if schema.AdditionalProperties != nil && !*schema.AdditionalProperties {
				return fmt.Errorf("%s has undeclared field %q", label, key)
			}
			continue
		}
		if err := validateProperty(raw, property); err != nil {
			return fmt.Errorf("%s.%s: %w", label, key, err)
		}
	}
	return nil
}

func validateProperty(raw json.RawMessage, schema propertySchema) error {
	if len(schema.Enum) > 0 {
		var value string
		if err := json.Unmarshal(raw, &value); err != nil {
			return fmt.Errorf("enum value must be a string: %w", err)
		}
		found := false
		for _, allowed := range schema.Enum {
			if value == allowed {
				found = true
				break
			}
		}
		if !found {
			return fmt.Errorf("value %q is not in enum %v", value, schema.Enum)
		}
	}
	switch schema.Type {
	case "string":
		var value string
		if err := json.Unmarshal(raw, &value); err != nil {
			return fmt.Errorf("must be a string: %w", err)
		}
		if len(value) < schema.MinLength {
			return fmt.Errorf("string length %d is below minLength %d", len(value), schema.MinLength)
		}
		if schema.Pattern != "" {
			matched, err := regexp.MatchString(schema.Pattern, value)
			if err != nil {
				return err
			}
			if !matched {
				return fmt.Errorf("value %q does not match %s", value, schema.Pattern)
			}
		}
	case "array":
		var values []json.RawMessage
		if err := json.Unmarshal(raw, &values); err != nil {
			return fmt.Errorf("must be an array: %w", err)
		}
		if len(values) < schema.MinItems {
			return fmt.Errorf("array length %d is below minItems %d", len(values), schema.MinItems)
		}
		seen := make(map[string]bool)
		for index, item := range values {
			if err := validateProperty(item, *schema.Items); err != nil {
				return fmt.Errorf("item[%d]: %w", index, err)
			}
			if schema.UniqueItems {
				canonical := string(bytes.TrimSpace(item))
				if seen[canonical] {
					return fmt.Errorf("duplicate array item %s", canonical)
				}
				seen[canonical] = true
			}
		}
	case "":
		// Enum-only properties are intentionally supported by this bounded schema subset.
	case "integer":
		var value int64
		if err := json.Unmarshal(raw, &value); err != nil {
			return fmt.Errorf("must be an integer: %w", err)
		}
	case "boolean":
		var value bool
		if err := json.Unmarshal(raw, &value); err != nil {
			return fmt.Errorf("must be a boolean: %w", err)
		}
	default:
		return fmt.Errorf("unsupported schema type %q", schema.Type)
	}
	return nil
}
