package oracle

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"strings"
	"time"
)

const benchmarkResultSchemaV01 = "mycelix.djcc-benchmark-result.v0.1"

func rejectDuplicateBenchmarkJSONKeys(raw []byte) error {
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.UseNumber()
	var parseValue func() error
	parseValue = func() error {
		token, err := decoder.Token()
		if err != nil {
			return err
		}
		delimiter, isDelimiter := token.(json.Delim)
		if !isDelimiter {
			return nil
		}
		switch delimiter {
		case '{':
			seen := make(map[string]struct{})
			for decoder.More() {
				keyToken, err := decoder.Token()
				if err != nil {
					return err
				}
				key, ok := keyToken.(string)
				if !ok {
					return fmt.Errorf("benchmark JSON object key is not a string")
				}
				if _, duplicate := seen[key]; duplicate {
					return fmt.Errorf("benchmark JSON object contains duplicate key %q", key)
				}
				seen[key] = struct{}{}
				if err := parseValue(); err != nil {
					return err
				}
			}
			end, err := decoder.Token()
			if err != nil || end != json.Delim('}') {
				return fmt.Errorf("benchmark JSON object is not properly terminated")
			}
		case '[':
			for decoder.More() {
				if err := parseValue(); err != nil {
					return err
				}
			}
			end, err := decoder.Token()
			if err != nil || end != json.Delim(']') {
				return fmt.Errorf("benchmark JSON array is not properly terminated")
			}
		default:
			return fmt.Errorf("unexpected JSON delimiter %q", delimiter)
		}
		return nil
	}
	if err := parseValue(); err != nil {
		return fmt.Errorf("validate benchmark JSON structure: %w", err)
	}
	var trailing any
	if err := decoder.Decode(&trailing); err != io.EOF {
		if err == nil {
			return fmt.Errorf("benchmark result contains trailing JSON")
		}
		return fmt.Errorf("decode trailing benchmark data: %w", err)
	}
	return nil
}

func decodeBenchmarkObject(raw []byte) (map[string]any, error) {
	if err := rejectDuplicateBenchmarkJSONKeys(raw); err != nil {
		return nil, err
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.UseNumber()
	var value map[string]any
	if err := decoder.Decode(&value); err != nil {
		return nil, fmt.Errorf("decode benchmark result: %w", err)
	}
	if value == nil {
		return nil, fmt.Errorf("benchmark result must be a JSON object")
	}
	var trailing any
	if err := decoder.Decode(&trailing); err != io.EOF {
		if err == nil {
			return nil, fmt.Errorf("benchmark result contains trailing JSON")
		}
		return nil, fmt.Errorf("decode trailing benchmark data: %w", err)
	}
	return value, nil
}

func benchmarkObject(parent map[string]any, key string) (map[string]any, error) {
	value, ok := parent[key].(map[string]any)
	if !ok {
		return nil, fmt.Errorf("%s must be an object", key)
	}
	return value, nil
}

func benchmarkString(parent map[string]any, key string) (string, error) {
	value, ok := parent[key].(string)
	if !ok || strings.TrimSpace(value) == "" {
		return "", fmt.Errorf("%s must be a non-empty string", key)
	}
	return value, nil
}

func benchmarkInteger(parent map[string]any, key string) (int64, error) {
	value, ok := parent[key].(json.Number)
	if !ok {
		return 0, fmt.Errorf("%s must be an integer", key)
	}
	integer, err := value.Int64()
	if err != nil || integer < 0 {
		return 0, fmt.Errorf("%s must be a non-negative integer", key)
	}
	return integer, nil
}

func benchmarkFloat(parent map[string]any, key string) (float64, error) {
	value, ok := parent[key].(json.Number)
	if !ok {
		return 0, fmt.Errorf("%s must be a number", key)
	}
	number, err := value.Float64()
	if err != nil || math.IsNaN(number) || math.IsInf(number, 0) {
		return 0, fmt.Errorf("%s must be a finite number", key)
	}
	return number, nil
}

func benchmarkBool(parent map[string]any, key string) (bool, error) {
	value, ok := parent[key].(bool)
	if !ok {
		return false, fmt.Errorf("%s must be a boolean", key)
	}
	return value, nil
}

func benchmarkStringSet(parent map[string]any, key string) (map[string]struct{}, bool, error) {
	raw, exists := parent[key]
	if !exists {
		return nil, false, nil
	}
	items, ok := raw.([]any)
	if !ok {
		return nil, true, fmt.Errorf("%s must be an array", key)
	}
	if len(items) > 128 {
		return nil, true, fmt.Errorf("%s exceeds the maximum of 128 identifiers", key)
	}
	values := make(map[string]struct{}, len(items))
	for index, item := range items {
		value, ok := item.(string)
		if !ok || strings.TrimSpace(value) != value || value == "" || len(value) > 128 {
			return nil, true, fmt.Errorf("%s[%d] must be a trimmed non-empty string of at most 128 bytes", key, index)
		}
		if _, duplicate := values[value]; duplicate {
			return nil, true, fmt.Errorf("%s contains duplicate value %q", key, value)
		}
		values[value] = struct{}{}
	}
	return values, true, nil
}

func benchmarkStringSetsEqual(left, right map[string]struct{}) bool {
	if len(left) != len(right) {
		return false
	}
	for value := range left {
		if _, exists := right[value]; !exists {
			return false
		}
	}
	return true
}

func validateBenchmarkFixtureRecords(
	corpus map[string]any,
	classIDs, positiveClassIDs, negativeClassIDs map[string]struct{},
	counts map[string]int64,
	fixtureCount, executableCount, symbolicCount int64,
) error {
	rawRecords, exists := corpus["fixture_records"]
	if !exists {
		return fmt.Errorf("qualification records require fixture_records")
	}
	records, ok := rawRecords.([]any)
	if !ok {
		return fmt.Errorf("fixture_records must be an array")
	}
	if symbolicCount != 0 {
		return fmt.Errorf("qualification records cannot contain symbolic fixtures")
	}
	if len(records) < 60 || int64(len(records)) != fixtureCount || int64(len(records)) != executableCount {
		return fmt.Errorf("fixture_records length must equal fixture_count and executable_fixture_count, with at least 60 records")
	}

	fixtureIDs := make(map[string]struct{}, len(records))
	fixtureDigests := make(map[string]struct{}, len(records))
	positiveSeen := make(map[string]struct{}, len(classIDs))
	negativeSeen := make(map[string]struct{}, len(classIDs))
	positiveCount, negativeCount := int64(0), int64(0)

	for index, rawRecord := range records {
		record, ok := rawRecord.(map[string]any)
		if !ok {
			return fmt.Errorf("fixture_records[%d] must be an object", index)
		}
		fixtureID, err := benchmarkString(record, "fixture_id")
		if err != nil || strings.TrimSpace(fixtureID) != fixtureID || len(fixtureID) > 128 {
			return fmt.Errorf("fixture_records[%d].fixture_id must be a trimmed non-empty string of at most 128 bytes", index)
		}
		if _, duplicate := fixtureIDs[fixtureID]; duplicate {
			return fmt.Errorf("fixture_records contains duplicate fixture_id %q", fixtureID)
		}
		fixtureIDs[fixtureID] = struct{}{}

		classID, err := benchmarkString(record, "class_id")
		if err != nil {
			return fmt.Errorf("fixture_records[%d]: %w", index, err)
		}
		if _, exists := classIDs[classID]; !exists {
			return fmt.Errorf("fixture_records[%d] references undeclared class %q", index, classID)
		}
		controlKind, err := benchmarkString(record, "control_kind")
		if err != nil {
			return fmt.Errorf("fixture_records[%d]: %w", index, err)
		}
		seenSet := positiveSeen
		declaredSet := positiveClassIDs
		switch controlKind {
		case "positive_control":
			positiveCount++
		case "hard_negative":
			seenSet = negativeSeen
			declaredSet = negativeClassIDs
			negativeCount++
		default:
			return fmt.Errorf("fixture_records[%d].control_kind is unsupported", index)
		}
		if _, declared := declaredSet[classID]; !declared {
			return fmt.Errorf("fixture_records[%d] control class %q is not declared for %s", index, classID, controlKind)
		}
		if _, duplicate := seenSet[classID]; duplicate {
			return fmt.Errorf("fixture_records contains multiple %s fixtures for class %q", controlKind, classID)
		}
		seenSet[classID] = struct{}{}

		digest, err := benchmarkString(record, "digest")
		if err != nil || !validBenchmarkHex(digest, "sha256:", 32) {
			return fmt.Errorf("fixture_records[%d].digest must be a lowercase sha256 digest", index)
		}
		if _, duplicate := fixtureDigests[digest]; duplicate {
			return fmt.Errorf("fixture_records contains duplicate content digest %q", digest)
		}
		fixtureDigests[digest] = struct{}{}

		for _, key := range []string{"media_type", "generator_version"} {
			if value, err := benchmarkString(record, key); err != nil || strings.TrimSpace(value) != value {
				return fmt.Errorf("fixture_records[%d].%s must be a trimmed non-empty string", index, key)
			}
		}
		byteLength, err := benchmarkInteger(record, "byte_length")
		if err != nil || byteLength == 0 {
			return fmt.Errorf("fixture_records[%d].byte_length must be positive", index)
		}
		if _, err := benchmarkInteger(record, "generator_seed"); err != nil {
			return fmt.Errorf("fixture_records[%d]: %w", index, err)
		}
	}

	if !benchmarkStringSetsEqual(positiveSeen, positiveClassIDs) {
		return fmt.Errorf("fixture_records must contain exactly one positive control for every declared class")
	}
	if !benchmarkStringSetsEqual(negativeSeen, negativeClassIDs) {
		return fmt.Errorf("fixture_records must contain exactly one hard negative for every declared class")
	}
	if positiveCount != counts["positive_control_total"] || negativeCount != counts["hard_negative_total"] {
		return fmt.Errorf("fixture record control counts must match correctness denominators")
	}
	return nil
}

func validBenchmarkHex(value, prefix string, byteLength int) bool {
	if !strings.HasPrefix(value, prefix) {
		return false
	}
	hex := strings.TrimPrefix(value, prefix)
	if len(hex) != byteLength*2 {
		return false
	}
	for _, character := range hex {
		if !((character >= '0' && character <= '9') || (character >= 'a' && character <= 'f')) {
			return false
		}
	}
	return true
}

func validateBenchmarkDigest(parent map[string]any, key string) error {
	value, err := benchmarkString(parent, key)
	if err != nil {
		return err
	}
	if !validBenchmarkHex(value, "sha256:", 32) {
		return fmt.Errorf("%s must be a lowercase sha256 digest", key)
	}
	return nil
}

func validateBenchmarkSHA(parent map[string]any, key string) error {
	value, err := benchmarkString(parent, key)
	if err != nil {
		return err
	}
	if !validBenchmarkHex(value, "", 20) {
		return fmt.Errorf("%s must be a lowercase 40-character Git SHA", key)
	}
	return nil
}

func validateBenchmarkPercentiles(parent map[string]any, key string) error {
	values, err := benchmarkObject(parent, key)
	if err != nil {
		return err
	}
	p50, err := benchmarkFloat(values, "p50")
	if err != nil {
		return fmt.Errorf("%s: %w", key, err)
	}
	p95, err := benchmarkFloat(values, "p95")
	if err != nil {
		return fmt.Errorf("%s: %w", key, err)
	}
	p99, err := benchmarkFloat(values, "p99")
	if err != nil {
		return fmt.Errorf("%s: %w", key, err)
	}
	if p50 < 0 || p95 < p50 || p99 < p95 {
		return fmt.Errorf("%s percentiles must be non-negative and monotone (p50 <= p95 <= p99)", key)
	}
	if interval, exists := values["confidence_interval_95"]; exists {
		items, ok := interval.([]any)
		if !ok || len(items) != 2 {
			return fmt.Errorf("%s confidence_interval_95 must have two numeric bounds", key)
		}
		lower, ok := items[0].(json.Number)
		if !ok {
			return fmt.Errorf("%s confidence interval lower bound must be numeric", key)
		}
		upper, ok := items[1].(json.Number)
		if !ok {
			return fmt.Errorf("%s confidence interval upper bound must be numeric", key)
		}
		lo, loErr := lower.Float64()
		hi, hiErr := upper.Float64()
		if loErr != nil || hiErr != nil || lo < 0 || hi < lo {
			return fmt.Errorf("%s confidence interval must be finite, non-negative and ordered", key)
		}
	}
	return nil
}

// ValidateBenchmarkResultV01 enforces the semantic invariants that JSON Schema
// cannot express across fields. It validates a benchmark record; it does not
// authenticate the run, certify its source, or establish any DJCC claim.
func ValidateBenchmarkResultV01(raw []byte) error {
	result, err := decodeBenchmarkObject(raw)
	if err != nil {
		return err
	}
	schema, err := benchmarkString(result, "schema")
	if err != nil || schema != benchmarkResultSchemaV01 {
		return fmt.Errorf("unsupported benchmark result schema")
	}
	for _, key := range []string{"benchmark_id", "run_id"} {
		if _, err := benchmarkString(result, key); err != nil {
			return err
		}
	}
	runClass, err := benchmarkString(result, "run_class")
	if err != nil {
		return err
	}
	if runClass != "smoke" && runClass != "exploratory" && runClass != "qualification" {
		return fmt.Errorf("unsupported run_class %q", runClass)
	}
	outcome, err := benchmarkString(result, "qualification_outcome")
	if err != nil {
		return err
	}
	if outcome != "pass" && outcome != "fail" && outcome != "indeterminate" {
		return fmt.Errorf("unsupported qualification_outcome %q", outcome)
	}

	started, hasStarted := result["started_at"]
	ended, hasEnded := result["ended_at"]
	if hasStarted != hasEnded {
		return fmt.Errorf("started_at and ended_at must be supplied together")
	}
	if hasStarted {
		startText, okStart := started.(string)
		endText, okEnd := ended.(string)
		if !okStart || !okEnd {
			return fmt.Errorf("benchmark timestamps must be strings")
		}
		startTime, startErr := time.Parse(time.RFC3339, startText)
		endTime, endErr := time.Parse(time.RFC3339, endText)
		if startErr != nil || endErr != nil || endTime.Before(startTime) {
			return fmt.Errorf("benchmark timestamps must be valid RFC3339 and ordered")
		}
	}

	source, err := benchmarkObject(result, "source")
	if err != nil {
		return err
	}
	for _, key := range []string{"head_sha", "base_sha"} {
		if err := validateBenchmarkSHA(source, key); err != nil {
			return err
		}
	}
	workingTreeClean, err := benchmarkBool(source, "working_tree_clean")
	if err != nil {
		return err
	}
	if err := validateBenchmarkDigest(source, "toolchain_lock_digest"); err != nil {
		return err
	}
	if value, exists := source["build_provenance_digest"]; exists {
		if _, ok := value.(string); !ok || !validBenchmarkHex(value.(string), "sha256:", 32) {
			return fmt.Errorf("build_provenance_digest must be a lowercase sha256 digest")
		}
	}

	corpus, err := benchmarkObject(result, "corpus")
	if err != nil {
		return err
	}
	if err := validateBenchmarkDigest(corpus, "manifest_digest"); err != nil {
		return err
	}
	fixtureCount, err := benchmarkInteger(corpus, "fixture_count")
	if err != nil {
		return err
	}
	executableCount, err := benchmarkInteger(corpus, "executable_fixture_count")
	if err != nil {
		return err
	}
	symbolicCount, err := benchmarkInteger(corpus, "symbolic_fixture_count")
	if err != nil {
		return err
	}
	if fixtureCount != executableCount+symbolicCount {
		return fmt.Errorf("fixture_count must equal executable_fixture_count + symbolic_fixture_count")
	}
	adversarialCount, err := benchmarkInteger(corpus, "adversarial_class_count")
	if err != nil {
		return err
	}

	adversarialClassIDs, hasAdversarialClassIDs, err := benchmarkStringSet(corpus, "adversarial_class_ids")
	if err != nil {
		return err
	}
	positiveControlClassIDs, hasPositiveControlClassIDs, err := benchmarkStringSet(corpus, "positive_control_class_ids")
	if err != nil {
		return err
	}
	hardNegativeClassIDs, hasHardNegativeClassIDs, err := benchmarkStringSet(corpus, "hard_negative_class_ids")
	if err != nil {
		return err
	}
	if hasAdversarialClassIDs && int64(len(adversarialClassIDs)) != adversarialCount {
		return fmt.Errorf("adversarial_class_count must equal the number of distinct adversarial_class_ids")
	}
	for key, ids := range map[string]map[string]struct{}{
		"positive_control_class_ids": positiveControlClassIDs,
		"hard_negative_class_ids":    hardNegativeClassIDs,
	} {
		if key == "positive_control_class_ids" && !hasPositiveControlClassIDs {
			continue
		}
		if key == "hard_negative_class_ids" && !hasHardNegativeClassIDs {
			continue
		}
		if !hasAdversarialClassIDs {
			return fmt.Errorf("%s requires adversarial_class_ids", key)
		}
		for id := range ids {
			if _, exists := adversarialClassIDs[id]; !exists {
				return fmt.Errorf("%s contains class %q not declared in adversarial_class_ids", key, id)
			}
		}
	}

	for _, key := range []string{"fixture_set_digest", "oracle_digest"} {
		if value, exists := corpus[key]; exists {
			text, ok := value.(string)
			if !ok || !validBenchmarkHex(text, "sha256:", 32) {
				return fmt.Errorf("%s must be a lowercase sha256 digest", key)
			}
		}
	}
	if value, exists := corpus["oracle_version"]; exists {
		if text, ok := value.(string); !ok || strings.TrimSpace(text) == "" {
			return fmt.Errorf("oracle_version must be a non-empty string")
		}
	}

	profile, err := benchmarkObject(result, "profile")
	if err != nil {
		return err
	}
	if _, err := benchmarkString(profile, "profile_ref"); err != nil {
		return err
	}
	for _, key := range []string{"profile_digest", "trust_policy_digest", "identity_registry_digest", "accepted_frontier_digest"} {
		if key == "profile_digest" {
			if err := validateBenchmarkDigest(profile, key); err != nil {
				return err
			}
			continue
		}
		if value, exists := profile[key]; exists {
			text, ok := value.(string)
			if !ok || !validBenchmarkHex(text, "sha256:", 32) {
				return fmt.Errorf("%s must be a lowercase sha256 digest", key)
			}
		}
	}

	workload, err := benchmarkObject(result, "workload")
	if err != nil {
		return err
	}
	owners, err := benchmarkInteger(workload, "owner_count")
	if err != nil || owners == 0 {
		return fmt.Errorf("owner_count must be positive")
	}
	for _, key := range []string{"event_count", "dependency_edge_count", "evidence_channel_count", "failure_domain_count", "dependency_fanout"} {
		if _, err := benchmarkInteger(workload, key); err != nil {
			return err
		}
	}
	for _, key := range []string{"delta_fraction", "common_cause_density"} {
		value, err := benchmarkFloat(workload, key)
		if err != nil || value < 0 || value > 1 {
			return fmt.Errorf("%s must be between 0 and 1", key)
		}
	}
	if tags, exists := workload["adversary_tags"]; exists {
		items, ok := tags.([]any)
		if !ok {
			return fmt.Errorf("adversary_tags must be an array")
		}
		seen := make(map[string]struct{}, len(items))
		for _, item := range items {
			tag, ok := item.(string)
			if !ok || strings.TrimSpace(tag) == "" {
				return fmt.Errorf("adversary_tags entries must be non-empty strings")
			}
			if _, duplicate := seen[tag]; duplicate {
				return fmt.Errorf("adversary_tags must be unique")
			}
			seen[tag] = struct{}{}
		}
	}

	environment, err := benchmarkObject(result, "environment")
	if err != nil {
		return err
	}
	for _, key := range []string{"os", "kernel", "architecture", "cpu_model"} {
		if _, err := benchmarkString(environment, key); err != nil {
			return err
		}
	}
	for _, key := range []string{"logical_cpu_count", "memory_bytes"} {
		value, err := benchmarkInteger(environment, key)
		if err != nil || value == 0 {
			return fmt.Errorf("%s must be a positive integer", key)
		}
	}

	execution, err := benchmarkObject(result, "execution")
	if err != nil {
		return err
	}
	if _, err := benchmarkString(execution, "command"); err != nil {
		return err
	}
	if _, err := benchmarkInteger(execution, "random_seed"); err != nil {
		return err
	}
	warmups, err := benchmarkInteger(execution, "warmup_trials")
	if err != nil || warmups == 0 {
		return fmt.Errorf("warmup_trials must be positive")
	}
	timedTrials, err := benchmarkInteger(execution, "timed_trials")
	if err != nil || timedTrials == 0 {
		return fmt.Errorf("timed_trials must be positive")
	}
	toolchains, err := benchmarkObject(execution, "toolchains")
	if err != nil || len(toolchains) == 0 {
		return fmt.Errorf("toolchains must be a non-empty object")
	}
	for name, version := range toolchains {
		if strings.TrimSpace(name) == "" {
			return fmt.Errorf("toolchain name is empty")
		}
		if _, ok := version.(string); !ok || strings.TrimSpace(version.(string)) == "" {
			return fmt.Errorf("toolchain %s must have a non-empty version", name)
		}
	}
	for _, key := range []string{"dependency_lock_digest", "environment_digest"} {
		if value, exists := execution[key]; exists {
			text, ok := value.(string)
			if !ok || !validBenchmarkHex(text, "sha256:", 32) {
				return fmt.Errorf("%s must be a lowercase sha256 digest", key)
			}
		}
	}

	correctness, err := benchmarkObject(result, "correctness")
	if err != nil {
		return err
	}
	countKeys := []string{
		"positive_control_total", "positive_control_accepted",
		"hard_negative_total", "hard_negative_false_safe",
		"safety_mutant_total", "safety_mutants_killed",
		"false_absence_count", "false_completeness_count", "false_authentication_count",
		"false_authority_count", "false_effect_count", "false_independence_count",
		"claim_ceiling_widening_count", "historical_frontier_leakage_count",
		"action_denominator_substitution_count", "unknown_outcome_retry_count",
		"canonicalization_vector_total", "canonicalization_vector_matches",
	}
	counts := make(map[string]int64, len(countKeys))
	for _, key := range countKeys {
		count, err := benchmarkInteger(correctness, key)
		if err != nil {
			return err
		}
		counts[key] = count
	}
	for _, pair := range [][2]string{
		{"positive_control_accepted", "positive_control_total"},
		{"hard_negative_false_safe", "hard_negative_total"},
		{"safety_mutants_killed", "safety_mutant_total"},
		{"canonicalization_vector_matches", "canonicalization_vector_total"},
	} {
		if counts[pair[0]] > counts[pair[1]] {
			return fmt.Errorf("%s exceeds %s", pair[0], pair[1])
		}
	}
	recall, err := benchmarkFloat(correctness, "correction_impact_recall")
	if err != nil || recall < 0 || recall > 1 {
		return fmt.Errorf("correction_impact_recall must be between 0 and 1")
	}
	precision, err := benchmarkFloat(correctness, "correction_impact_precision")
	if err != nil || precision < 0 || precision > 1 {
		return fmt.Errorf("correction_impact_precision must be between 0 and 1")
	}
	identical, err := benchmarkBool(correctness, "full_incremental_outputs_identical")
	if err != nil {
		return err
	}
	if notes, exists := correctness["notes"]; exists {
		if _, ok := notes.([]any); !ok {
			return fmt.Errorf("correctness.notes must be an array")
		}
	}

	dispositions, err := benchmarkObject(result, "dispositions")
	if err != nil {
		return err
	}
	for _, key := range []string{"stage_counts", "root_counts", "claim_ceiling_counts"} {
		countMap, err := benchmarkObject(dispositions, key)
		if err != nil || len(countMap) == 0 {
			return fmt.Errorf("%s must be a non-empty count map", key)
		}
		for name, rawCount := range countMap {
			if strings.TrimSpace(name) == "" {
				return fmt.Errorf("%s has an empty key", key)
			}
			value, ok := rawCount.(json.Number)
			if !ok {
				return fmt.Errorf("%s.%s must be an integer", key, name)
			}
			count, parseErr := value.Int64()
			if parseErr != nil || count < 0 {
				return fmt.Errorf("%s.%s must be a non-negative integer", key, name)
			}
		}
	}
	if value, exists := dispositions["abstention_count"]; exists {
		number, ok := value.(json.Number)
		if !ok {
			return fmt.Errorf("abstention_count must be an integer")
		}
		count, parseErr := number.Int64()
		if parseErr != nil || count < 0 {
			return fmt.Errorf("abstention_count must be non-negative")
		}
	}

	performance, err := benchmarkObject(result, "performance")
	if err != nil {
		return err
	}
	for _, key := range []string{"latency_ns", "cpu_time_ns", "peak_rss_bytes", "allocation_bytes", "proof_bytes", "io_bytes", "network_bytes"} {
		if err := validateBenchmarkPercentiles(performance, key); err != nil {
			return err
		}
	}
	for _, key := range []string{"cache_hit_ratio"} {
		if value, exists := performance[key]; exists {
			number, err := benchmarkFloat(performance, key)
			if err != nil || number < 0 || number > 1 {
				return fmt.Errorf("%s must be between 0 and 1", key)
			}
			_ = value
		}
	}
	for _, key := range []string{"incremental_to_full_cpu_ratio", "incremental_to_full_latency_ratio"} {
		if value, exists := performance[key]; exists {
			number, err := benchmarkFloat(performance, key)
			if err != nil || number < 0 {
				return fmt.Errorf("%s must be non-negative", key)
			}
		}
	}

	artifacts, ok := result["artifacts"].([]any)
	if !ok || len(artifacts) == 0 {
		return fmt.Errorf("artifacts must be a non-empty array")
	}
	for index, rawArtifact := range artifacts {
		artifact, ok := rawArtifact.(map[string]any)
		if !ok {
			return fmt.Errorf("artifacts[%d] must be an object", index)
		}
		for _, key := range []string{"kind", "media_type"} {
			if _, err := benchmarkString(artifact, key); err != nil {
				return fmt.Errorf("artifacts[%d]: %w", index, err)
			}
		}
		if err := validateBenchmarkDigest(artifact, "digest"); err != nil {
			return fmt.Errorf("artifacts[%d]: %w", index, err)
		}
		if _, err := benchmarkInteger(artifact, "byte_length"); err != nil {
			return fmt.Errorf("artifacts[%d]: %w", index, err)
		}
		if uri, exists := artifact["uri"]; exists {
			text, ok := uri.(string)
			if !ok || !strings.Contains(text, "://") {
				return fmt.Errorf("artifacts[%d].uri must be an absolute URI", index)
			}
		}
	}

	if runClass == "qualification" {
		if !workingTreeClean {
			return fmt.Errorf("qualification records require a clean source tree")
		}
		if adversarialCount < 30 || len(adversarialClassIDs) < 30 {
			return fmt.Errorf("qualification records require at least 30 distinct named adversarial classes")
		}
		if counts["positive_control_total"] < int64(len(adversarialClassIDs)) ||
			counts["hard_negative_total"] < int64(len(adversarialClassIDs)) {
			return fmt.Errorf("qualification requires a positive control and hard negative for every adversarial class")
		}
		if !hasAdversarialClassIDs || !hasPositiveControlClassIDs || !hasHardNegativeClassIDs {
			return fmt.Errorf("qualification records require named adversarial, positive-control and hard-negative class IDs")
		}
		if !benchmarkStringSetsEqual(adversarialClassIDs, positiveControlClassIDs) ||
			!benchmarkStringSetsEqual(adversarialClassIDs, hardNegativeClassIDs) {
			return fmt.Errorf("every adversarial class requires paired positive-control and hard-negative fixtures")
		}
		if err := validateBenchmarkFixtureRecords(
			corpus, adversarialClassIDs, positiveControlClassIDs, hardNegativeClassIDs,
			counts, fixtureCount, executableCount, symbolicCount,
		); err != nil {
			return err
		}
		if fixtureCount < 60 || executableCount < 60 {
			return fmt.Errorf("qualification records require at least 60 executable fixtures for 30 paired classes")
		}
		if _, exists := corpus["fixture_set_digest"]; !exists {
			return fmt.Errorf("qualification records require fixture_set_digest")
		}
		if err := validateBenchmarkDigest(corpus, "fixture_set_digest"); err != nil {
			return err
		}
		if _, err := benchmarkString(corpus, "oracle_version"); err != nil {
			return err
		}
		if err := validateBenchmarkDigest(corpus, "oracle_digest"); err != nil {
			return err
		}
		if symbolicCount != 0 {
			return fmt.Errorf("qualification records cannot count symbolic fixtures; all fixtures must be executable")
		}
		if timedTrials < 30 {
			return fmt.Errorf("qualification records require at least 30 timed trials")
		}
		for _, key := range []string{"positive_control_total", "hard_negative_total", "safety_mutant_total", "canonicalization_vector_total"} {
			if counts[key] == 0 {
				return fmt.Errorf("qualification records require non-zero %s", key)
			}
		}
	}
	if outcome == "pass" {
		if runClass != "qualification" {
			return fmt.Errorf("only qualification runs may claim pass")
		}
		if counts["positive_control_accepted"] != counts["positive_control_total"] {
			return fmt.Errorf("passing qualification must accept every positive control")
		}
		if counts["hard_negative_false_safe"] != 0 {
			return fmt.Errorf("passing qualification must have zero hard-negative false-safe results")
		}
		if counts["safety_mutants_killed"] != counts["safety_mutant_total"] {
			return fmt.Errorf("passing qualification must kill every safety-critical mutant")
		}
		for _, key := range []string{
			"false_absence_count", "false_completeness_count", "false_authentication_count",
			"false_authority_count", "false_effect_count", "false_independence_count",
			"claim_ceiling_widening_count", "historical_frontier_leakage_count",
			"action_denominator_substitution_count", "unknown_outcome_retry_count",
		} {
			if counts[key] != 0 {
				return fmt.Errorf("passing qualification requires zero %s", key)
			}
		}
		if recall != 1 {
			return fmt.Errorf("passing qualification requires 100%% correction-impact recall")
		}
		if counts["canonicalization_vector_matches"] != counts["canonicalization_vector_total"] {
			return fmt.Errorf("passing qualification requires full canonicalization-vector parity")
		}
		if !identical {
			return fmt.Errorf("passing qualification requires full/incremental replay parity")
		}
	}
	return nil
}

