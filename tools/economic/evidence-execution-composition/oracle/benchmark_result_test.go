package oracle

import (
	"encoding/json"
	"strings"
	"testing"
)

func validBenchmarkResultV01() map[string]any {
	const digest = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
	return map[string]any{
		"schema": benchmarkResultSchemaV01,
		"benchmark_id": "djcc-v0.1-candidate-vs-full-recompute",
		"run_id": "local-qualification-001",
		"run_class": "qualification",
		"qualification_outcome": "pass",
		"started_at": "2026-11-28T12:00:00Z",
		"ended_at": "2026-11-28T12:05:00Z",
		"source": map[string]any{
			"head_sha": strings.Repeat("a", 40),
			"base_sha": strings.Repeat("b", 40),
			"working_tree_clean": true,
			"toolchain_lock_digest": digest,
		},
		"corpus": map[string]any{
			"manifest_digest": digest,
			"fixture_count": 40,
			"executable_fixture_count": 40,
			"symbolic_fixture_count": 0,
			"adversarial_class_count": 30,
			"oracle_version": "independent-oracle-v0.1",
			"oracle_digest": digest,
		},
		"profile": map[string]any{
			"profile_ref": "mycelix.djcc.full-evidence-execution.v0.1",
			"profile_digest": digest,
		},
		"workload": map[string]any{
			"owner_count": 100,
			"event_count": 10000,
			"dependency_edge_count": 500,
			"evidence_channel_count": 4,
			"failure_domain_count": 12,
			"delta_fraction": 0.01,
			"dependency_fanout": 8,
			"common_cause_density": 0.1,
		},
		"environment": map[string]any{
			"os": "Linux",
			"kernel": "6.8.0",
			"architecture": "x86_64",
			"cpu_model": "CI runner",
			"logical_cpu_count": 8,
			"memory_bytes": 17179869184,
		},
		"execution": map[string]any{
			"command": "go test ./...",
			"random_seed": 42,
			"warmup_trials": 5,
			"timed_trials": 30,
			"toolchains": map[string]any{"go": "1.23.2", "rust": "1.96.0"},
		},
		"correctness": map[string]any{
			"positive_control_total": 2,
			"positive_control_accepted": 2,
			"hard_negative_total": 30,
			"hard_negative_false_safe": 0,
			"safety_mutant_total": 25,
			"safety_mutants_killed": 25,
			"false_absence_count": 0,
			"false_completeness_count": 0,
			"false_authentication_count": 0,
			"false_authority_count": 0,
			"false_effect_count": 0,
			"false_independence_count": 0,
			"claim_ceiling_widening_count": 0,
			"historical_frontier_leakage_count": 0,
			"action_denominator_substitution_count": 0,
			"unknown_outcome_retry_count": 0,
			"correction_impact_recall": 1.0,
			"correction_impact_precision": 0.95,
			"canonicalization_vector_total": 4,
			"canonicalization_vector_matches": 4,
			"full_incremental_outputs_identical": true,
		},
		"dispositions": map[string]any{
			"stage_counts": map[string]any{"VerifiedForDeclaredClaim": 100, "Indeterminate": 2},
			"root_counts": map[string]any{"DecisionJustifiedAtAcceptedFrontier": 10, "Indeterminate": 3},
			"claim_ceiling_counts": map[string]any{"EstablishedWithinProfile": 100, "NotEvaluated": 20},
		},
		"performance": map[string]any{
			"latency_ns": map[string]any{"p50": 1000, "p95": 2000, "p99": 3000},
			"cpu_time_ns": map[string]any{"p50": 800, "p95": 1700, "p99": 2500},
			"peak_rss_bytes": map[string]any{"p50": 100000, "p95": 120000, "p99": 140000},
			"allocation_bytes": map[string]any{"p50": 40000, "p95": 50000, "p99": 60000},
			"proof_bytes": map[string]any{"p50": 512, "p95": 700, "p99": 900},
			"io_bytes": map[string]any{"p50": 1000, "p95": 1500, "p99": 2000},
			"network_bytes": map[string]any{"p50": 0, "p95": 0, "p99": 0},
			"cache_hit_ratio": 0.5,
			"incremental_to_full_cpu_ratio": 0.2,
			"incremental_to_full_latency_ratio": 0.3,
		},
		"artifacts": []any{
			map[string]any{"kind": "raw-trials", "digest": digest, "byte_length": 2048, "media_type": "application/json"},
		},
	}
}

func encodeBenchmarkResult(t *testing.T, value map[string]any) []byte {
	t.Helper()
	raw, err := json.Marshal(value)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

func TestValidateBenchmarkResultV01AcceptsCompleteQualification(t *testing.T) {
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, validBenchmarkResultV01())); err != nil {
		t.Fatalf("valid benchmark result rejected: %v", err)
	}
}

func TestValidateBenchmarkResultV01RejectsFalseAuthenticationOnPass(t *testing.T) {
	value := validBenchmarkResultV01()
	value["correctness"].(map[string]any)["false_authentication_count"] = 1
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("passing qualification accepted a false-authentication event")
	}
}

func TestValidateBenchmarkResultV01RejectsIncompleteMutationKills(t *testing.T) {
	value := validBenchmarkResultV01()
	value["correctness"].(map[string]any)["safety_mutants_killed"] = 24
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("passing qualification accepted an unkilled safety mutant")
	}
}

func TestValidateBenchmarkResultV01RejectsPositiveControlMiss(t *testing.T) {
	value := validBenchmarkResultV01()
	value["correctness"].(map[string]any)["positive_control_accepted"] = 1
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("passing qualification accepted a missed positive control")
	}
}

func TestValidateBenchmarkResultV01RejectsCorrectionRecallBelowOne(t *testing.T) {
	value := validBenchmarkResultV01()
	value["correctness"].(map[string]any)["correction_impact_recall"] = 0.99
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("passing qualification accepted incomplete correction-impact recall")
	}
}

func TestValidateBenchmarkResultV01RejectsNonMonotonePercentiles(t *testing.T) {
	value := validBenchmarkResultV01()
	value["performance"].(map[string]any)["latency_ns"] = map[string]any{"p50": 1000, "p95": 900, "p99": 1100}
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("non-monotone latency percentiles accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsFixtureCountMismatch(t *testing.T) {
	value := validBenchmarkResultV01()
	value["corpus"].(map[string]any)["fixture_count"] = 41
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("fixture count mismatch accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsReplayMismatchOnPass(t *testing.T) {
	value := validBenchmarkResultV01()
	value["correctness"].(map[string]any)["full_incremental_outputs_identical"] = false
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("passing qualification accepted a full/incremental replay mismatch")
	}
}

func TestValidateBenchmarkResultV01RequiresFalseAuthenticationMetric(t *testing.T) {
	value := validBenchmarkResultV01()
	delete(value["correctness"].(map[string]any), "false_authentication_count")
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("benchmark result omitted false-authentication metric")
	}
}

func TestValidateBenchmarkResultV01AllowsFailingExploratoryRun(t *testing.T) {
	value := validBenchmarkResultV01()
	value["run_class"] = "exploratory"
	value["qualification_outcome"] = "indeterminate"
	value["source"].(map[string]any)["working_tree_clean"] = false
	value["corpus"].(map[string]any)["adversarial_class_count"] = 3
	value["execution"].(map[string]any)["timed_trials"] = 1
	value["correctness"].(map[string]any)["safety_mutants_killed"] = 0
	value["correctness"].(map[string]any)["full_incremental_outputs_identical"] = false
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err != nil {
		t.Fatalf("exploratory indeterminate run should remain recordable: %v", err)
	}
}

func TestValidateBenchmarkResultV01RejectsUnorderedTimestamps(t *testing.T) {
	value := validBenchmarkResultV01()
	value["ended_at"] = "2026-11-28T11:59:59Z"
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("unordered benchmark timestamps accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsMalformedTrailingData(t *testing.T) {
	raw := append(encodeBenchmarkResult(t, validBenchmarkResultV01()), []byte("not-json")...)
	if err := ValidateBenchmarkResultV01(raw); err == nil {
		t.Fatal("malformed trailing data accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsZeroOwners(t *testing.T) {
	value := validBenchmarkResultV01()
	value["workload"].(map[string]any)["owner_count"] = 0
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("zero-owner workload accepted")
	}
}
