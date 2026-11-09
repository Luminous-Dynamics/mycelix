package main

import (
	"encoding/json"
	"fmt"
	"os"
	"reflect"
	"strings"
	"testing"
)

type challengePack struct {
	Schema         string          `json:"schema"`
	Version        string          `json:"version"`
	State          string          `json:"state"`
	ClaimCeiling   string          `json:"claim_ceiling"`
	EvaluatorTruth string          `json:"evaluator_truth"`
	CaseCount      int             `json:"case_count"`
	Cases          []challengeCase `json:"cases"`
}

type challengeCase struct {
	ID                  string   `json:"id"`
	Mutation            string   `json:"mutation"`
	ExpectedDisposition string   `json:"expected_disposition"`
	ExpectedReasonCodes []string `json:"expected_reason_codes"`
}

func baseInput() Input {
	return Input{
		Contract: inputContract,
		Snapshot: InventorySnapshot{
			SnapshotID:       "snapshot-dh-001",
			AsOf:             100,
			EvidenceFrontier: 120,
			Complete:         true,
			ScopedSourceIDs:  []string{"rx-a", "rx-b"},
			CoveredNodeIDs:   []string{"clock-root", "provider-a", "provider-b", "rx-a", "rx-b"},
		},
		Query: DependencyQuery{
			SourceIDs:                []string{"rx-a", "rx-b"},
			Domain:                   "clock",
			EventTime:                90,
			ExpectedSnapshotID:       "snapshot-dh-001",
			ExpectedEvidenceFrontier: 120,
		},
		Nodes: []Node{
			{ID: "clock-root", Kind: "dependency"},
			{ID: "provider-a", Kind: "dependency"},
			{ID: "provider-b", Kind: "dependency"},
			{ID: "rx-a", Kind: "source"},
			{ID: "rx-b", Kind: "source"},
		},
		Hyperedges: []DependencyHyperedge{
			{
				ID: "edge-a", SubjectIDs: []string{"rx-a"}, DependencyID: "provider-a", Domain: "clock",
				ValidFrom: int64Ptr(0), RecordedAt: 80, EvidenceRef: "evidence:a", State: "qualified",
			},
			{
				ID: "edge-b", SubjectIDs: []string{"rx-b"}, DependencyID: "provider-b", Domain: "clock",
				ValidFrom: int64Ptr(0), RecordedAt: 80, EvidenceRef: "evidence:b", State: "qualified",
			},
			{
				ID: "shared-root", SubjectIDs: []string{"provider-a", "provider-b"}, DependencyID: "clock-root", Domain: "clock",
				ValidFrom: int64Ptr(0), RecordedAt: 80, EvidenceRef: "evidence:root", State: "qualified",
			},
		},
	}
}

func int64Ptr(value int64) *int64 { return &value }

func mutate(input *Input, mutation string) {
	switch mutation {
	case "transitive_shared_ancestor":
	case "direct_multi_subject_hyperedge":
		input.Hyperedges = input.Hyperedges[:2]
		input.Hyperedges = append(input.Hyperedges, DependencyHyperedge{
			ID:           "one-clock-for-both",
			SubjectIDs:   []string{"rx-a", "rx-b"},
			DependencyID: "clock-root",
			Domain:       "clock",
			RecordedAt:   80,
			EvidenceRef:  "evidence:one-clock",
			State:        "qualified",
		})
	case "shared_power_does_not_imply_clock":
		input.Nodes = append(input.Nodes, Node{ID: "power-root", Kind: "dependency"})
		sortNodes(input.Nodes)
		input.Snapshot.CoveredNodeIDs = append(input.Snapshot.CoveredNodeIDs, "power-root")
		sortStrings(input.Snapshot.CoveredNodeIDs)
		input.Hyperedges = []DependencyHyperedge{{
			ID:           "power-only",
			SubjectIDs:   []string{"rx-a", "rx-b"},
			DependencyID: "power-root",
			Domain:       "power",
			ValidFrom:    int64Ptr(0),
			RecordedAt:   80,
			EvidenceRef:  "evidence:power-only",
			State:        "qualified",
		}}
		input.Query.Domain = "clock"
	case "shared_edge_expired_at_event_time":
		input.Hyperedges[2].ValidUntil = int64Ptr(50)
	case "stale_shared_edge":
		input.Hyperedges[2].State = "stale"
	case "source_missing_from_inventory_scope":
		input.Snapshot.ScopedSourceIDs = input.Snapshot.ScopedSourceIDs[:1]
	case "incomplete_inventory_snapshot":
		input.Snapshot.Complete = false
	case "source_missing_from_snapshot_coverage":
		input.Snapshot.CoveredNodeIDs = input.Snapshot.CoveredNodeIDs[:len(input.Snapshot.CoveredNodeIDs)-1]
	case "dependency_cycle":
		input.Hyperedges = append(input.Hyperedges, DependencyHyperedge{
			ID:           "cycle",
			SubjectIDs:   []string{"clock-root"},
			DependencyID: "provider-a",
			Domain:       "clock",
			ValidFrom:    int64Ptr(0),
			RecordedAt:   80,
			EvidenceRef:  "evidence:cycle",
			State:        "qualified",
		})
	case "tampered_closure_report":
	case "noncanonical_source_order":
		input.Query.SourceIDs = []string{"rx-b", "rx-a"}
	case "complete_inventory_no_common_ancestor":
		input.Hyperedges = input.Hyperedges[:2]
	default:
		panic("unknown challenge mutation: " + mutation)
	}
}

func sortNodes(nodes []Node) {
	for i := 0; i < len(nodes); i++ {
		for j := i + 1; j < len(nodes); j++ {
			if nodes[j].ID < nodes[i].ID {
				nodes[i], nodes[j] = nodes[j], nodes[i]
			}
		}
	}
}

func sortStrings(values []string) {
	for i := 0; i < len(values); i++ {
		for j := i + 1; j < len(values); j++ {
			if values[j] < values[i] {
				values[i], values[j] = values[j], values[i]
			}
		}
	}
}

func TestPublicChallengePack(t *testing.T) {
	data, err := os.ReadFile("../emsr-dependency-hypergraph/fixtures/dependency-hypergraph-challenges-0.1.json")
	if err != nil {
		t.Fatal(err)
	}
	var pack challengePack
	if err := json.Unmarshal(data, &pack); err != nil {
		t.Fatal(err)
	}
	if pack.Schema != "emsr-dependency-hypergraph-challenge-pack/0.1" || pack.Version != "0.1.0" ||
		pack.State != "FROZEN_PUBLIC_SYNTHETIC_CONFORMANCE" || pack.CaseCount != len(pack.Cases) ||
		pack.ClaimCeiling == "" || pack.EvaluatorTruth == "" {
		t.Fatal("challenge pack contract is not frozen or complete")
	}
	expectedIDs := []string{"DH-001", "DH-002", "DH-003", "DH-004", "DH-005", "DH-006", "DH-007", "DH-008", "DH-009", "DH-010", "DH-011", "DH-012"}
	actualIDs := make([]string, 0, len(pack.Cases))
	for _, challenge := range pack.Cases {
		actualIDs = append(actualIDs, challenge.ID)
	}
	if !reflect.DeepEqual(actualIDs, expectedIDs) {
		t.Fatalf("challenge discovery mismatch: %v", actualIDs)
	}

	for _, challenge := range pack.Cases {
		t.Run(challenge.ID, func(t *testing.T) {
			input := baseInput()
			mutate(&input, challenge.Mutation)
			inputBytes, err := json.Marshal(input)
			if err != nil {
				t.Fatal(err)
			}
			expected := referenceReport(input, sha256Hex(inputBytes))
			if challenge.ExpectedDisposition == "VERIFIER_REJECT" {
				expected.CommonDependencyIDs = []string{}
				reportBytes, _ := json.Marshal(expected)
				result := verify(inputBytes, reportBytes)
				if result.Status != "rejected" {
					t.Fatalf("tampered report unexpectedly verified: %+v", result)
				}
				return
			}
			if expected.Disposition != challenge.ExpectedDisposition {
				t.Fatalf("disposition %s, want %s; reasons=%v", expected.Disposition, challenge.ExpectedDisposition, expected.ReasonCodes)
			}
			for _, reason := range challenge.ExpectedReasonCodes {
				if !contains(expected.ReasonCodes, reason) {
					t.Fatalf("missing reason %q in %v", reason, expected.ReasonCodes)
				}
			}
			reportBytes, err := json.Marshal(expected)
			if err != nil {
				t.Fatal(err)
			}
			result := verify(inputBytes, reportBytes)
			if result.Status != "verified" {
				t.Fatalf("valid report did not verify: %+v", result)
			}
		})
	}
}

func TestTamperedInputByteBindingRejected(t *testing.T) {
	input := baseInput()
	inputBytes, _ := json.Marshal(input)
	expected := referenceReport(input, sha256Hex(inputBytes))
	reportBytes, _ := json.Marshal(expected)
	mutated := append([]byte(nil), inputBytes...)
	mutated = append(mutated, ' ')
	result := verify(mutated, reportBytes)
	if result.Status != "rejected" {
		t.Fatal("report with stale input digest was accepted")
	}
}

func TestInvalidAuthorityAndClaimCeilingRejected(t *testing.T) {
	input := baseInput()
	inputBytes, _ := json.Marshal(input)
	expected := referenceReport(input, sha256Hex(inputBytes))
	expected.Authority = "production_operation_authority"
	reportBytes, _ := json.Marshal(expected)
	result := verify(inputBytes, reportBytes)
	if result.Status != "rejected" {
		t.Fatal("authority escalation was accepted")
	}
}

func TestDecodeStrictRejectsDuplicateTopLevelProperties(t *testing.T) {
	_, err := decodeStrict[Input]([]byte(`{"contract":"first","contract":"second"}`))
	if err == nil {
		t.Fatal("duplicate top-level property was accepted")
	}
}

func TestDecodeStrictRejectsDuplicateNestedProperties(t *testing.T) {
	_, err := decodeStrict[Input]([]byte(`{"contract":"emsr-dependency-hypergraph-input/0.1","snapshot":{"snapshot_id":"first","snapshot_id":"second"}}`))
	if err == nil {
		t.Fatal("duplicate nested property was accepted")
	}
}

func TestDecodeStrictRejectsUnknownTopLevelProperties(t *testing.T) {
	_, err := decodeStrict[Input]([]byte(`{"contract":"emsr-dependency-hypergraph-input/0.1","unexpected":true}`))
	if err == nil {
		t.Fatal("unknown top-level property was accepted")
	}
}

func TestDecodeStrictRejectsTrailingJSONValues(t *testing.T) {
	_, err := decodeStrict[Input]([]byte(`{} {}`))
	if err == nil {
		t.Fatal("multiple JSON values were accepted")
	}
}

func TestVerifierRejectsDuplicateInputProperties(t *testing.T) {
	input := []byte(`{"contract":"first","contract":"second"}`)
	report := []byte(`{}`)
	result := verify(input, report)
	if result.Status != "rejected" {
		t.Fatalf("duplicate-key input was not rejected: %+v", result)
	}
}

func civilInput(domain string, sourceIDs []string, nodes []Node, edges []DependencyHyperedge) Input {
	sortStrings(sourceIDs)
	sortNodes(nodes)
	covered := make([]string, 0, len(nodes))
	for _, node := range nodes {
		covered = append(covered, node.ID)
	}
	sortStrings(covered)
	return Input{
		Contract: inputContract,
		Snapshot: InventorySnapshot{
			SnapshotID:       "civil-domain-snapshot-v0.1",
			AsOf:             100,
			EvidenceFrontier: 120,
			Complete:         true,
			ScopedSourceIDs:  append([]string(nil), sourceIDs...),
			CoveredNodeIDs:   covered,
		},
		Query: DependencyQuery{
			SourceIDs:                append([]string(nil), sourceIDs...),
			Domain:                   domain,
			EventTime:                90,
			ExpectedSnapshotID:       "civil-domain-snapshot-v0.1",
			ExpectedEvidenceFrontier: 120,
		},
		Nodes:      nodes,
		Hyperedges: edges,
	}
}

func civilEdge(id string, subjects []string, dependency, domain string, state string) DependencyHyperedge {
	sortStrings(subjects)
	return DependencyHyperedge{
		ID:           id,
		SubjectIDs:   subjects,
		DependencyID: dependency,
		Domain:       domain,
		ValidFrom:    int64Ptr(0),
		RecordedAt:   80,
		EvidenceRef:  "synthetic:" + id,
		State:        state,
	}
}

func assertCivilReport(t *testing.T, input Input, want string, common []string) Report {
	t.Helper()
	inputBytes, err := json.Marshal(input)
	if err != nil {
		t.Fatal(err)
	}
	report := referenceReport(input, sha256Hex(inputBytes))
	if report.Disposition != want {
		t.Fatalf("disposition %s, want %s; reasons=%v", report.Disposition, want, report.ReasonCodes)
	}
	if !reflect.DeepEqual(report.CommonDependencyIDs, common) {
		t.Fatalf("common dependencies %v, want %v", report.CommonDependencyIDs, common)
	}
	reportBytes, err := json.Marshal(report)
	if err != nil {
		t.Fatal(err)
	}
	verification := verify(inputBytes, reportBytes)
	if verification.Status != "verified" {
		t.Fatalf("cross-language report did not verify: %+v", verification)
	}
	return report
}

func TestCivilUASVendorFeedsSharingCollector(t *testing.T) {
	input := civilInput(
		"collector",
		[]string{"rf-feed-a", "rf-feed-b"},
		[]Node{
			{ID: "collector-a", Kind: "dependency"},
			{ID: "collector-b", Kind: "dependency"},
			{ID: "rf-feed-a", Kind: "source"},
			{ID: "rf-feed-b", Kind: "source"},
			{ID: "shared-collector", Kind: "dependency"},
		},
		[]DependencyHyperedge{
			civilEdge("a-to-collector", []string{"rf-feed-a"}, "collector-a", "collector", "qualified"),
			civilEdge("b-to-collector", []string{"rf-feed-b"}, "collector-b", "collector", "qualified"),
			civilEdge("collector-common-root", []string{"collector-a", "collector-b"}, "shared-collector", "collector", "qualified"),
		},
	)
	assertCivilReport(t, input, "SHARED_DEPENDENCY", []string{"shared-collector"})
}

func TestCivilMaritimeAISAndRadarSharePortPipeline(t *testing.T) {
	input := civilInput(
		"data_pipeline",
		[]string{"ais-provider", "radar-processor"},
		[]Node{
			{ID: "ais-aggregation", Kind: "dependency"},
			{ID: "ais-provider", Kind: "source"},
			{ID: "port-correlation-platform", Kind: "dependency"},
			{ID: "radar-processor", Kind: "source"},
			{ID: "radar-track-pipeline", Kind: "dependency"},
		},
		[]DependencyHyperedge{
			civilEdge("ais-to-platform", []string{"ais-provider"}, "ais-aggregation", "data_pipeline", "qualified"),
			civilEdge("radar-to-pipeline", []string{"radar-processor"}, "radar-track-pipeline", "data_pipeline", "qualified"),
			civilEdge("shared-port-platform", []string{"ais-aggregation", "radar-track-pipeline"}, "port-correlation-platform", "data_pipeline", "qualified"),
		},
	)
	assertCivilReport(t, input, "SHARED_DEPENDENCY", []string{"port-correlation-platform"})
}

func TestCivilAviationSharedPowerDoesNotImplySharedClock(t *testing.T) {
	input := civilInput(
		"clock",
		[]string{"adsb-feed", "mlat-feed"},
		[]Node{
			{ID: "adsb-clock", Kind: "dependency"},
			{ID: "adsb-feed", Kind: "source"},
			{ID: "airport-power-bus", Kind: "dependency"},
			{ID: "mlat-clock", Kind: "dependency"},
			{ID: "mlat-feed", Kind: "source"},
		},
		[]DependencyHyperedge{
			civilEdge("adsb-clock-edge", []string{"adsb-feed"}, "adsb-clock", "clock", "qualified"),
			civilEdge("mlat-clock-edge", []string{"mlat-feed"}, "mlat-clock", "clock", "qualified"),
			civilEdge("shared-power", []string{"adsb-feed", "mlat-feed"}, "airport-power-bus", "power", "qualified"),
		},
	)
	assertCivilReport(t, input, "NO_DECLARED_COMMON_DOMAIN", []string{})
	input.Query.Domain = "power"
	assertCivilReport(t, input, "SHARED_DEPENDENCY", []string{"airport-power-bus"})
}

func TestCivilPositiveControlUsesDeclaredNotPhysicalIndependenceSemantics(t *testing.T) {
	input := civilInput(
		"data_pipeline",
		[]string{"source-a", "source-b"},
		[]Node{
			{ID: "pipeline-a", Kind: "dependency"},
			{ID: "pipeline-b", Kind: "dependency"},
			{ID: "source-a", Kind: "source"},
			{ID: "source-b", Kind: "source"},
		},
		[]DependencyHyperedge{
			civilEdge("source-a-pipeline", []string{"source-a"}, "pipeline-a", "data_pipeline", "qualified"),
			civilEdge("source-b-pipeline", []string{"source-b"}, "pipeline-b", "data_pipeline", "qualified"),
		},
	)
	report := assertCivilReport(t, input, "NO_DECLARED_COMMON_DOMAIN", []string{})
	if !strings.Contains(report.ClaimCeiling, "not proof of physical or statistical independence") {
		t.Fatal("positive control exceeded the graph-derived claim ceiling")
	}
}

func TestDecodeStrictRejectsOversizedInputBeforeParsing(t *testing.T) {
	data := make([]byte, maxFileBytes+1)
	_, err := decodeStrict[Input](data)
	if err == nil {
		t.Fatal("oversized JSON input was accepted")
	}
}

func TestReferenceTraversalHandlesLongMaritimeDependencyChains(t *testing.T) {
	const depth = 10000
	nodes := []Node{
		{ID: "ais-observation-source", Kind: "source"},
		{ID: "radar-observation-source", Kind: "source"},
		{ID: "shared-port-core", Kind: "dependency"},
	}
	edges := make([]DependencyHyperedge, 0, 2*depth+2)

	for _, item := range []struct {
		prefix string
		source string
	}{
		{prefix: "ais-dependency", source: "ais-observation-source"},
		{prefix: "radar-dependency", source: "radar-observation-source"},
	} {
		previous := item.source
		for index := 0; index < depth; index++ {
			current := fmt.Sprintf("%s-%05d", item.prefix, index)
			nodes = append(nodes, Node{ID: current, Kind: "dependency"})
			edges = append(edges, civilEdge(
				fmt.Sprintf("%s-edge-%05d", item.prefix, index),
				[]string{previous},
				current,
				"data_pipeline",
				"qualified",
			))
			previous = current
		}
		edges = append(edges, civilEdge(
			item.prefix+"-to-shared-port-core",
			[]string{previous},
			"shared-port-core",
			"data_pipeline",
			"qualified",
		))
	}

	input := civilInput(
		"data_pipeline",
		[]string{"ais-observation-source", "radar-observation-source"},
		nodes,
		edges,
	)
	inputBytes, err := json.Marshal(input)
	if err != nil {
		t.Fatal(err)
	}
	report := referenceReport(input, sha256Hex(inputBytes))
	if report.Disposition != "SHARED_DEPENDENCY" || !reflect.DeepEqual(report.CommonDependencyIDs, []string{"shared-port-core"}) {
		t.Fatalf("unexpected long-chain report: disposition=%s common=%v reasons=%v", report.Disposition, report.CommonDependencyIDs, report.ReasonCodes)
	}
	reportBytes, err := json.Marshal(report)
	if err != nil {
		t.Fatal(err)
	}
	verification := verify(inputBytes, reportBytes)
	if verification.Status != "verified" {
		t.Fatalf("long-chain report failed independent verification: %+v", verification)
	}
}
