package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"reflect"
	"sort"
	"strings"
)

const (
	inputContract    = "emsr-dependency-hypergraph-input/0.1"
	reportContract   = "emsr-dependency-hypergraph-report/0.1"
	verifierContract = "emsr-dependency-hypergraph-go-verifier/0.1"
	claimCeiling     = "Declared common-dependency assessment over the exact supplied inventory snapshot and evidence frontier; not proof of physical or statistical independence, and not evidence that unknown common causes do not exist."
	authority        = "declared_dependency_assessment_only"
	maxNodes         = 100_000
	maxHyperedges    = 500_000
	maxFileBytes     = 128 << 20
)

type Input struct {
	Contract   string               `json:"contract"`
	Snapshot   InventorySnapshot    `json:"snapshot"`
	Query      DependencyQuery      `json:"query"`
	Nodes      []Node               `json:"nodes"`
	Hyperedges []DependencyHyperedge `json:"hyperedges"`
}

type InventorySnapshot struct {
	SnapshotID       string   `json:"snapshot_id"`
	AsOf             int64    `json:"as_of"`
	EvidenceFrontier int64    `json:"evidence_frontier"`
	Complete         bool     `json:"complete"`
	ScopedSourceIDs  []string `json:"scoped_source_ids"`
	CoveredNodeIDs   []string `json:"covered_node_ids"`
}

type DependencyQuery struct {
	SourceIDs                []string `json:"source_ids"`
	Domain                   string   `json:"domain"`
	EventTime                int64    `json:"event_time"`
	ExpectedSnapshotID       string   `json:"expected_snapshot_id"`
	ExpectedEvidenceFrontier int64    `json:"expected_evidence_frontier"`
}

type Node struct {
	ID   string `json:"id"`
	Kind string `json:"kind"`
}

type DependencyHyperedge struct {
	ID           string   `json:"id"`
	SubjectIDs   []string `json:"subject_ids"`
	DependencyID string   `json:"dependency_id"`
	Domain       string   `json:"domain"`
	ValidFrom    *int64   `json:"valid_from"`
	ValidUntil   *int64   `json:"valid_until"`
	RecordedAt   int64    `json:"recorded_at"`
	EvidenceRef  string   `json:"evidence_ref"`
	State        string   `json:"state"`
}

type Report struct {
	ReportContract      string              `json:"report_contract"`
	Disposition         string              `json:"disposition"`
	Authority           string              `json:"authority"`
	InputSHA256         string              `json:"input_sha256"`
	SnapshotID          string              `json:"snapshot_id"`
	EvidenceFrontier    int64               `json:"evidence_frontier"`
	EventTime           int64               `json:"event_time"`
	Domain              string              `json:"domain"`
	SourceIDs           []string            `json:"source_ids"`
	SourceAncestors     map[string][]string `json:"source_ancestors"`
	CommonDependencyIDs []string            `json:"common_dependency_ids"`
	ReasonCodes         []string            `json:"reason_codes"`
	ClaimCeiling        string              `json:"claim_ceiling"`
}

type VerificationCheck struct {
	ID     string `json:"id"`
	Result string `json:"result"`
}

type VerificationReport struct {
	VerifierContract string              `json:"verifier_contract"`
	Status           string              `json:"status"`
	Authority        string              `json:"authority"`
	Checks           []VerificationCheck `json:"checks"`
	Limitations      []string            `json:"limitations"`
}

func main() {
	if len(os.Args) != 4 || os.Args[1] != "verify" {
		fmt.Fprintln(os.Stderr, "usage: emsr-dependency-hypergraph-go-verifier verify <input.json> <report.json>")
		os.Exit(2)
	}
	input, err := readBounded(os.Args[2], maxFileBytes)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
	report, err := readBounded(os.Args[3], maxFileBytes)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
	result := verify(input, report)
	encoded, err := json.MarshalIndent(result, "", "  ")
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
	fmt.Println(string(encoded))
	if result.Status != "verified" {
		os.Exit(1)
	}
}

func readBounded(path string, maxBytes int64) ([]byte, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("open %q: %w", path, err)
	}
	defer file.Close()
	data, err := io.ReadAll(io.LimitReader(file, maxBytes+1))
	if err != nil {
		return nil, fmt.Errorf("read %q: %w", path, err)
	}
	if int64(len(data)) > maxBytes {
		return nil, fmt.Errorf("%q exceeds the %d-byte limit", path, maxBytes)
	}
	return data, nil
}

func decodeStrict[T any](data []byte) (T, error) {
	var value T
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&value); err != nil {
		return value, err
	}
	var trailing any
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		if err == nil {
			return value, errors.New("multiple JSON values are not permitted")
		}
		return value, err
	}
	return value, nil
}

func verify(inputBytes, reportBytes []byte) VerificationReport {
	limitations := []string{
		"This is a separately implemented Go verifier using the standard library; it is not organizationally independent because it is maintained in the same repository and project.",
		"A passing report establishes agreement with the declared input contract, not physical independence or the absence of unknown common causes.",
		"The input snapshot and its completeness declaration are assertions supplied by the caller; this verifier does not authenticate the inventory issuer.",
	}
	input, inputErr := decodeStrict[Input](inputBytes)
	report, reportErr := decodeStrict[Report](reportBytes)
	checks := make([]VerificationCheck, 0, 5)

	if inputErr != nil || reportErr != nil || !validEnums(input, report) {
		checks = append(checks, check("input-and-report-parse", false))
		return VerificationReport{
			VerifierContract: verifierContract,
			Status:           "rejected",
			Authority:        "cross_language_reference_agreement_only",
			Checks:            checks,
			Limitations:       limitations,
		}
	}

	expected := referenceReport(input, sha256Hex(inputBytes))
	checks = append(checks,
		check("input-byte-binding", report.InputSHA256 == expected.InputSHA256),
		check("query-snapshot-binding",
			report.SnapshotID == expected.SnapshotID &&
				report.EvidenceFrontier == expected.EvidenceFrontier &&
				report.EventTime == expected.EventTime &&
				report.Domain == expected.Domain &&
				reflect.DeepEqual(report.SourceIDs, expected.SourceIDs)),
		check("independent-closure",
			reflect.DeepEqual(report.SourceAncestors, expected.SourceAncestors) &&
				reflect.DeepEqual(report.CommonDependencyIDs, expected.CommonDependencyIDs)),
		check("derived-disposition",
			report.Disposition == expected.Disposition &&
				reflect.DeepEqual(report.ReasonCodes, expected.ReasonCodes)),
		check("authority-and-claim-ceiling",
			report.Authority == authority &&
				report.ClaimCeiling == claimCeiling &&
				report.ReportContract == reportContract),
	)

	passed := true
	for _, item := range checks {
		if item.Result != "pass" {
			passed = false
			break
		}
	}
	status := "rejected"
	if passed {
		status = "verified"
	}
	return VerificationReport{
		VerifierContract: verifierContract,
		Status:           status,
		Authority:        "cross_language_reference_agreement_only",
		Checks:            checks,
		Limitations:       limitations,
	}
}

func validEnums(input Input, report Report) bool {
	validDomains := map[string]bool{
		"clock": true, "oscillator": true, "gnss_reference": true,
		"correction_provider": true, "power": true, "network": true,
		"antenna_site": true, "firmware_lineage": true, "collector": true,
		"data_pipeline": true,
	}
	if !validDomains[input.Query.Domain] || !validDomains[report.Domain] {
		return false
	}
	for _, node := range input.Nodes {
		if node.Kind != "source" && node.Kind != "dependency" {
			return false
		}
	}
	validStates := map[string]bool{
		"qualified": true, "stale": true, "revoked": true,
		"conflicted": true, "unknown": true, "unavailable": true,
	}
	for _, edge := range input.Hyperedges {
		if !validDomains[edge.Domain] || !validStates[edge.State] {
			return false
		}
	}
	switch report.Disposition {
	case "SHARED_DEPENDENCY", "UNKNOWN_DEPENDENCY", "NO_DECLARED_COMMON_DOMAIN", "INVALID_INPUT":
		return true
	default:
		return false
	}
}

func referenceReport(input Input, inputHash string) Report {
	invalid := validateContract(input)
	if len(invalid) > 0 {
		return invalidReport(inputHash, invalid, input.Query.Domain, input.Query.EventTime,
			input.Snapshot.SnapshotID, input.Query.SourceIDs, input.Snapshot.EvidenceFrontier)
	}

	unknown := make([]string, 0)
	if !input.Snapshot.Complete {
		unknown = append(unknown, "inventory_snapshot_incomplete")
	}
	if input.Snapshot.AsOf < input.Query.EventTime {
		unknown = append(unknown, "inventory_snapshot_older_than_event_time")
	}

	sourceAncestors := make(map[string][]string, len(input.Query.SourceIDs))
	for _, source := range input.Query.SourceIDs {
		if !contains(input.Snapshot.ScopedSourceIDs, source) {
			unknown = append(unknown, "source_outside_inventory_scope")
		}
		if !contains(input.Snapshot.CoveredNodeIDs, source) {
			unknown = append(unknown, "source_not_covered_by_inventory_snapshot")
		}
		visiting := make(map[string]bool)
		done := make(map[string]bool)
		ancestors := make(map[string]bool)
		if visit(source, input, visiting, done, ancestors, &unknown) {
			return invalidReport(inputHash, []string{"dependency_cycle_detected"},
				input.Query.Domain, input.Query.EventTime, input.Snapshot.SnapshotID,
				input.Query.SourceIDs, input.Snapshot.EvidenceFrontier)
		}
		sourceAncestors[source] = sortedKeys(ancestors)
	}

	unknown = uniqueSorted(unknown)
	common := commonAncestors(sourceAncestors, input.Query.SourceIDs)
	disposition := "NO_DECLARED_COMMON_DOMAIN"
	reasons := []string{"complete_inventory_has_no_declared_common_ancestor"}
	if len(unknown) > 0 {
		disposition = "UNKNOWN_DEPENDENCY"
		reasons = unknown
	} else if len(common) > 0 {
		disposition = "SHARED_DEPENDENCY"
		reasons = []string{"qualified_common_dependency_ancestor"}
	}

	return Report{
		ReportContract:      reportContract,
		Disposition:         disposition,
		Authority:           authority,
		InputSHA256:         inputHash,
		SnapshotID:          input.Snapshot.SnapshotID,
		EvidenceFrontier:    input.Snapshot.EvidenceFrontier,
		EventTime:           input.Query.EventTime,
		Domain:              input.Query.Domain,
		SourceIDs:           cloneStrings(input.Query.SourceIDs),
		SourceAncestors:     sourceAncestors,
		CommonDependencyIDs: common,
		ReasonCodes:         reasons,
		ClaimCeiling:        claimCeiling,
	}
}

func visit(nodeID string, input Input, visiting, done, ancestors map[string]bool, unknown *[]string) bool {
	if done[nodeID] {
		return false
	}
	if visiting[nodeID] {
		return true
	}
	visiting[nodeID] = true

	if !contains(input.Snapshot.CoveredNodeIDs, nodeID) {
		*unknown = append(*unknown, "reachable_node_not_covered_by_inventory_snapshot")
	}
	for _, edge := range input.Hyperedges {
		if edge.Domain != input.Query.Domain ||
			!contains(edge.SubjectIDs, nodeID) ||
			!validAt(edge, input.Query.EventTime) {
			continue
		}
		if edge.State != "qualified" || edge.RecordedAt > input.Snapshot.EvidenceFrontier {
			*unknown = append(*unknown, "applicable_assertion_not_qualified_or_outside_frontier")
			continue
		}
		ancestors[edge.DependencyID] = true
		if visit(edge.DependencyID, input, visiting, done, ancestors, unknown) {
			return true
		}
	}
	delete(visiting, nodeID)
	done[nodeID] = true
	return false
}

func validateContract(input Input) []string {
	errorsFound := make([]string, 0)
	add := func(condition bool, reason string) {
		if condition {
			errorsFound = append(errorsFound, reason)
		}
	}
	add(input.Contract != inputContract, "unsupported_input_contract")
	add(input.Query.ExpectedSnapshotID != input.Snapshot.SnapshotID, "query_snapshot_id_mismatch")
	add(input.Query.ExpectedEvidenceFrontier != input.Snapshot.EvidenceFrontier, "query_evidence_frontier_mismatch")
	add(strings.TrimSpace(input.Snapshot.SnapshotID) == "", "snapshot_id_empty")
	add(input.Snapshot.AsOf > input.Snapshot.EvidenceFrontier, "snapshot_as_of_after_evidence_frontier")
	add(len(input.Query.SourceIDs) < 2, "query_requires_at_least_two_sources")
	add(!strictlySortedUnique(input.Query.SourceIDs), "query_sources_must_be_sorted_unique")
	add(!strictlySortedUnique(input.Snapshot.ScopedSourceIDs), "snapshot_scope_sources_must_be_sorted_unique")
	add(!strictlySortedUnique(input.Snapshot.CoveredNodeIDs), "snapshot_covered_nodes_must_be_sorted_unique")
	add(len(input.Nodes) == 0 || len(input.Nodes) > maxNodes, "node_count_out_of_bounds")
	add(len(input.Hyperedges) > maxHyperedges, "hyperedge_count_exceeds_limit")

	nodeIDs := make(map[string]bool, len(input.Nodes))
	nodeKinds := make(map[string]string, len(input.Nodes))
	for _, node := range input.Nodes {
		add(strings.TrimSpace(node.ID) == "", "node_id_empty")
		add(nodeIDs[node.ID], "duplicate_node_id")
		nodeIDs[node.ID] = true
		nodeKinds[node.ID] = node.Kind
	}
	for _, source := range input.Snapshot.ScopedSourceIDs {
		add(nodeKinds[source] != "source", "snapshot_scope_entry_is_not_a_source")
	}
	for _, source := range input.Query.SourceIDs {
		switch {
		case !nodeIDs[source]:
			errorsFound = append(errorsFound, "queried_source_missing_from_graph")
		case nodeKinds[source] != "source":
			errorsFound = append(errorsFound, "queried_node_is_not_a_source")
		}
	}

	edgeIDs := make(map[string]bool, len(input.Hyperedges))
	for _, edge := range input.Hyperedges {
		add(strings.TrimSpace(edge.ID) == "" || edgeIDs[edge.ID], "empty_or_duplicate_hyperedge_id")
		edgeIDs[edge.ID] = true
		add(len(edge.SubjectIDs) == 0 || !strictlySortedUnique(edge.SubjectIDs), "hyperedge_subjects_must_be_nonempty_sorted_unique")
		add(strings.TrimSpace(edge.EvidenceRef) == "", "hyperedge_evidence_reference_empty")
		add(edge.ValidFrom != nil && edge.ValidUntil != nil && *edge.ValidFrom >= *edge.ValidUntil,
			"hyperedge_valid_time_interval_empty_or_inverted")
		add(contains(edge.SubjectIDs, edge.DependencyID), "hyperedge_dependency_cannot_be_its_own_subject")
		switch {
		case !nodeIDs[edge.DependencyID]:
			errorsFound = append(errorsFound, "hyperedge_dependency_node_missing")
		case nodeKinds[edge.DependencyID] != "dependency":
			errorsFound = append(errorsFound, "hyperedge_dependency_must_reference_dependency_node")
		}
		for _, subject := range edge.SubjectIDs {
			add(!nodeIDs[subject], "hyperedge_subject_node_missing")
		}
	}
	for _, id := range input.Snapshot.CoveredNodeIDs {
		add(!nodeIDs[id], "snapshot_covers_unknown_node")
	}
	return uniqueSorted(errorsFound)
}

func invalidReport(inputHash string, reasons []string, domain string, eventTime int64,
	snapshotID string, sourceIDs []string, evidenceFrontier int64) Report {
	return Report{
		ReportContract:      reportContract,
		Disposition:         "INVALID_INPUT",
		Authority:           authority,
		InputSHA256:         inputHash,
		SnapshotID:          snapshotID,
		EvidenceFrontier:    evidenceFrontier,
		EventTime:           eventTime,
		Domain:              domain,
		SourceIDs:           cloneStrings(sourceIDs),
		SourceAncestors:     map[string][]string{},
		CommonDependencyIDs: []string{},
		ReasonCodes:         uniqueSorted(reasons),
		ClaimCeiling:        claimCeiling,
	}
}

func commonAncestors(closures map[string][]string, sources []string) []string {
	if len(sources) == 0 {
		return []string{}
	}
	first := make(map[string]bool)
	for _, item := range closures[sources[0]] {
		first[item] = true
	}
	for _, source := range sources[1:] {
		current := make(map[string]bool)
		for _, item := range closures[source] {
			current[item] = true
		}
		for item := range first {
			if !current[item] {
				delete(first, item)
			}
		}
	}
	return sortedKeys(first)
}

func validAt(edge DependencyHyperedge, instant int64) bool {
	return (edge.ValidFrom == nil || *edge.ValidFrom <= instant) &&
		(edge.ValidUntil == nil || instant < *edge.ValidUntil)
}

func contains(values []string, target string) bool {
	for _, value := range values {
		if value == target {
			return true
		}
	}
	return false
}

func strictlySortedUnique(values []string) bool {
	for index := 1; index < len(values); index++ {
		if values[index-1] >= values[index] {
			return false
		}
	}
	return true
}

func uniqueSorted(values []string) []string {
	if len(values) == 0 {
		return []string{}
	}
	sort.Strings(values)
	output := make([]string, 0, len(values))
	for _, value := range values {
		if len(output) == 0 || output[len(output)-1] != value {
			output = append(output, value)
		}
	}
	return output
}

func sortedKeys(values map[string]bool) []string {
	output := make([]string, 0, len(values))
	for value := range values {
		output = append(output, value)
	}
	sort.Strings(output)
	return output
}

func sha256Hex(value []byte) string {
	digest := sha256.Sum256(value)
	return hex.EncodeToString(digest[:])
}

func check(id string, passed bool) VerificationCheck {
	result := "fail"
	if passed {
		result = "pass"
	}
	return VerificationCheck{ID: id, Result: result}
}

func cloneStrings(values []string) []string {
	output := make([]string, len(values))
	copy(output, values)
	return output
}
