package oracle

import (
	"encoding/json"
	"fmt"
	"strings"
	"testing"
)

func validBenchmarkResultV01() map[string]any {
	const digest = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
	classIDs := []any{
		"valid-positive-decision",
		"unauthenticated-source",
		"issuer-key-epoch-substitution",
		"selective-issuer-withholding",
		"hidden-owner-prefix-gap",
		"backdated-post-frontier-correction",
		"query-range-omission",
		"semantic-projection-drift",
		"policy-epoch-evaluator-substitution",
		"authority-substitution",
		"action-set-membership-swap",
		"unauthorized-denominator-amendment",
		"execution-replay-duplicate-effect",
		"outcome-unknown-new-mutation",
		"provider-completion-without-independent-outcome",
		"shared-operator-kms-domain",
		"incomplete-lineage-disclosure",
		"detached-mmr-payload-mismatch",
		"vds-profile-substitution",
		"unbound-mmr-index-tree-size",
		"permissive-invalid-proof-decoding",
		"checkpoint-rollback-split-view",
		"canonical-preimage-domain-separation",
		"auditor-credential-key-epoch-substitution",
		"correction-impact-omission",
		"correction-impact-overreach",
		"historical-policy-trust-leakage",
		"compound-valid-proof-incomplete-universe-common-cause",
		"mmr-peak-unpinned-accumulator",
		"approval-execution-action-join-mismatch",
	}
	fixtureRecords := make([]any, 0, 60)
	for classIndex, rawClassID := range classIDs {
		classID := rawClassID.(string)
		for controlIndex, controlKind := range []string{"positive_control", "hard_negative"} {
			fixtureNumber := classIndex*2 + controlIndex + 1
			fixtureRecords = append(fixtureRecords, map[string]any{
				"fixture_id":       fmt.Sprintf("djcc-fixture-%02d-%s", fixtureNumber, controlKind),
				"class_id":         classID,
				"control_kind":     controlKind,
				"digest":           "sha256:" + fmt.Sprintf("%064x", fixtureNumber),
				"byte_length":      128 + fixtureNumber,
				"media_type":       "application/json",
				"generator_version": "djcc-fixture-generator-v0.1",
				"generator_seed":   fixtureNumber,
			})
		}
	}
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
			"fixture_count": 60,
			"executable_fixture_count": 60,
			"symbolic_fixture_count": 0,
			"adversarial_class_count": 30,
			"adversarial_class_ids":     classIDs,
			"positive_control_class_ids": append([]any(nil), classIDs...),
			"hard_negative_class_ids":    append([]any(nil), classIDs...),
			"fixture_set_digest": digest,
			"fixture_records":    fixtureRecords,
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
			"positive_control_total": 30,
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

func TestValidateBenchmarkResultV01RejectsDuplicateJSONKeys(t *testing.T) {
	raw := string(encodeBenchmarkResult(t, validBenchmarkResultV01()))
	raw = strings.Replace(raw, `"run_id":"local-qualification-001"`, `"run_id":"shadow","run_id":"local-qualification-001"`, 1)
	if err := ValidateBenchmarkResultV01([]byte(raw)); err == nil {
		t.Fatal("duplicate run_id field accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsZeroOwners(t *testing.T) {
	value := validBenchmarkResultV01()
	value["workload"].(map[string]any)["owner_count"] = 0
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("zero-owner workload accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsQualificationBelowThirtyClasses(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	corpus["adversarial_class_count"] = 29
	corpus["adversarial_class_ids"] = corpus["adversarial_class_ids"].([]any)[:29]
	corpus["positive_control_class_ids"] = corpus["positive_control_class_ids"].([]any)[:29]
	corpus["hard_negative_class_ids"] = corpus["hard_negative_class_ids"].([]any)[:29]
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification with fewer than 30 adversarial classes accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsSymbolicFixturesOnQualification(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	corpus["symbolic_fixture_count"] = 1
	corpus["fixture_count"] = corpus["executable_fixture_count"].(int) + 1
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification counted symbolic fixtures as executable")
	}
}


func TestValidateBenchmarkResultV01RejectsDuplicateAdversarialClassIDs(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	ids := corpus["adversarial_class_ids"].([]any)
	ids[29] = ids[0]
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("duplicate adversarial class ID accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsAdversarialClassCountMismatch(t *testing.T) {
	value := validBenchmarkResultV01()
	value["corpus"].(map[string]any)["adversarial_class_count"] = 31
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("adversarial class count inconsistent with class IDs accepted")
	}
}


func TestValidateBenchmarkResultV01RejectsUnpairedPositiveControlClass(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	corpus["positive_control_class_ids"] = corpus["positive_control_class_ids"].([]any)[:29]
	value["correctness"].(map[string]any)["positive_control_total"] = 29
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification missing a positive control for one adversarial class accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsFewerThanSixtyExecutableFixtures(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	corpus["fixture_count"] = 59
	corpus["executable_fixture_count"] = 59
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification with fewer than 60 executable paired fixtures accepted")
	}
}

func TestValidateBenchmarkResultV01RequiresFixtureSetDigest(t *testing.T) {
	value := validBenchmarkResultV01()
	delete(value["corpus"].(map[string]any), "fixture_set_digest")
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification without fixture-set digest accepted")
	}
}


func TestValidateBenchmarkResultV01RejectsUnpairedHardNegativeClass(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	corpus["hard_negative_class_ids"] = corpus["hard_negative_class_ids"].([]any)[:29]
	value["correctness"].(map[string]any)["hard_negative_total"] = 29
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification missing a hard negative for one adversarial class accepted")
	}
}

func TestValidateBenchmarkResultV01RequiresOracleDigest(t *testing.T) {
	value := validBenchmarkResultV01()
	delete(value["corpus"].(map[string]any), "oracle_digest")
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification without independent oracle digest accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsControlClassOutsideAdversarialSet(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	ids := corpus["positive_control_class_ids"].([]any)
	ids[0] = "undeclared-class"
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("positive control for an undeclared adversarial class accepted")
	}
}


func TestValidateBenchmarkResultV01RejectsDuplicateFixtureIDs(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	records := corpus["fixture_records"].([]any)
	second := records[1].(map[string]any)
	second["fixture_id"] = records[0].(map[string]any)["fixture_id"]
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("duplicate fixture ID accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsDuplicateFixtureDigests(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	records := corpus["fixture_records"].([]any)
	records[1].(map[string]any)["digest"] = records[0].(map[string]any)["digest"]
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("duplicate fixture content digest accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsMissingFixtureRecord(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	records := corpus["fixture_records"].([]any)
	corpus["fixture_records"] = records[:59]
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("qualification missing a fixture record accepted")
	}
}

func TestValidateBenchmarkResultV01RejectsMultiplePositiveFixturesForClass(t *testing.T) {
	value := validBenchmarkResultV01()
	corpus := value["corpus"].(map[string]any)
	records := corpus["fixture_records"].([]any)
	records[1].(map[string]any)["control_kind"] = "positive_control"
	if err := ValidateBenchmarkResultV01(encodeBenchmarkResult(t, value)); err == nil {
		t.Fatal("duplicate positive control for a class accepted")
	}
}
