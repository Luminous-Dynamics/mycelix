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
	"sort"
	"strings"
)

type Node struct {
	NodeID          string `json:"node_id"`
	NodeKind        string `json:"node_kind"`
	ClaimDimension  string `json:"claim_dimension"`
}

type Edge struct {
	FromNodeID string `json:"from_node_id"`
	ToNodeID   string `json:"to_node_id"`
	Relation   string `json:"relation"`
}

type ChangeEvent struct {
	EventID     string `json:"event_id"`
	TargetNodeID string `json:"target_node_id"`
	EventType   string `json:"event_type"`
	Observation string `json:"observation"`
}

type Scenario struct {
	ScenarioID            string        `json:"scenario_id"`
	PairID                string        `json:"pair_id"`
	Variant               string        `json:"variant"`
	Family                string        `json:"family"`
	Nodes                 []Node        `json:"nodes"`
	DependencyEdges       []Edge        `json:"dependency_edges"`
	ChangeEvents          []ChangeEvent `json:"change_events"`
	ChangeCoverage        string        `json:"change_coverage"`
	UnknownScopeClaimNodes []string      `json:"unknown_scope_claim_nodes"`
	OwnerSemantics        []string      `json:"owner_semantics"`
}

type ScenarioEnvelope struct {
	CorpusID       string     `json:"corpus_id"`
	Status         string     `json:"status"`
	Disclosure     string     `json:"disclosure"`
	ScenarioCount  int        `json:"scenario_count"`
	PairCount      int        `json:"pair_count"`
	Scenarios      []Scenario `json:"scenarios"`
}

type Outcome struct {
	ScenarioID          string   `json:"scenario_id"`
	AffectedClaimNodes  []string `json:"affected_claim_nodes"`
	UnknownClaimNodes   []string `json:"unknown_claim_nodes"`
	UnaffectedClaimNodes []string `json:"unaffected_claim_nodes"`
	RejectedEventIDs    []string `json:"rejected_event_ids"`
	ReasonCode          string   `json:"reason_code"`
}

type OracleEnvelope struct {
	CorpusID    string    `json:"corpus_id"`
	Status      string    `json:"status"`
	Disclosure  string    `json:"disclosure"`
	OutcomeCount int      `json:"outcome_count"`
	Outcomes    []Outcome `json:"outcomes"`
}

type PairState struct {
	Family   string
	Variants map[string]bool
}

type DatasetSummary struct {
	CorpusID      string         `json:"corpus_id"`
	ScenarioCount int            `json:"scenario_count"`
	PairCount     int            `json:"pair_count"`
	OutcomeCount  int            `json:"outcome_count"`
	FamilyCounts  map[string]int `json:"family_counts"`
	ChangedClaims int            `json:"changed_claim_count"`
	UnknownClaims int            `json:"unknown_claim_count"`
	UnaffectedClaims int         `json:"unaffected_claim_count"`
	ScenarioSHA256 string        `json:"scenario_sha256"`
	OracleSHA256  string         `json:"oracle_sha256"`
}

func main() {
	scenarioPath := flag.String("scenarios", "docs/research/assurance-invalidation/closure/scenarios.v1.json", "scenario envelope path")
	oraclePath := flag.String("oracle", "docs/research/assurance-invalidation/closure/oracle.v1.json", "oracle envelope path")
	jsonOutput := flag.Bool("json", false, "emit machine-readable JSON")
	flag.Parse()

	summary, err := audit(*scenarioPath, *oraclePath)
	if err != nil {
		if *jsonOutput {
			_ = json.NewEncoder(os.Stdout).Encode(map[string]any{"ok": false, "error": err.Error()})
		} else {
			fmt.Fprintln(os.Stderr, "FAIL:", err)
		}
		os.Exit(1)
	}
	if *jsonOutput {
		enc := json.NewEncoder(os.Stdout)
		enc.SetIndent("", "  ")
		_ = enc.Encode(map[string]any{"ok": true, "summary": summary})
	} else {
		fmt.Printf("PASS LISE dependency closure: scenarios=%d pairs=%d outcomes=%d changed_claims=%d unknown_claims=%d unaffected_claims=%d
",
			summary.ScenarioCount, summary.PairCount, summary.OutcomeCount, summary.ChangedClaims, summary.UnknownClaims, summary.UnaffectedClaims)
	}
}

func audit(scenarioPath, oraclePath string) (DatasetSummary, error) {
	var summary DatasetSummary
	scenarioBytes, err := os.ReadFile(scenarioPath)
	if err != nil {
		return summary, fmt.Errorf("read scenarios: %w", err)
	}
	oracleBytes, err := os.ReadFile(oraclePath)
	if err != nil {
		return summary, fmt.Errorf("read oracle: %w", err)
	}
	var scenarios ScenarioEnvelope
	if err := decodeStrict(scenarioBytes, &scenarios); err != nil {
		return summary, fmt.Errorf("decode scenario envelope: %w", err)
	}
	var oracle OracleEnvelope
	if err := decodeStrict(oracleBytes, &oracle); err != nil {
		return summary, fmt.Errorf("decode oracle envelope: %w", err)
	}

	if scenarios.CorpusID == "" || scenarios.CorpusID != oracle.CorpusID {
		return summary, fmt.Errorf("corpus ID mismatch: scenarios=%q oracle=%q", scenarios.CorpusID, oracle.CorpusID)
	}
	if scenarios.Status != "public_development_seed" || oracle.Status != "public_development_oracle" {
		return summary, errors.New("both envelopes must declare public development/calibration status")
	}
	if !strings.Contains(strings.ToLower(scenarios.Disclosure), "not a hidden or blinded holdout") ||
		!strings.Contains(strings.ToLower(oracle.Disclosure), "public") {
		return summary, errors.New("scenario and oracle disclosures must make public calibration status explicit")
	}
	if scenarios.ScenarioCount != len(scenarios.Scenarios) || oracle.OutcomeCount != len(oracle.Outcomes) {
		return summary, errors.New("declared scenario/outcome count does not match array length")
	}
	if len(scenarios.Scenarios) == 0 || len(scenarios.Scenarios) != len(oracle.Outcomes) {
		return summary, errors.New("scenario and oracle denominators must be equal and nonzero")
	}
	if scenarios.PairCount <= 0 {
		return summary, errors.New("pair_count must be positive")
	}

	pairs := make(map[string]*PairState)
	seenIDs := make(map[string]bool, len(scenarios.Scenarios))
	families := make(map[string]int)
	changedCount, unknownCount, unaffectedCount := 0, 0, 0

	for index, scenario := range scenarios.Scenarios {
		expectedID := fmt.Sprintf("CLOSE-%03d", index+1)
		if scenario.ScenarioID != expectedID {
			return summary, fmt.Errorf("scenario[%d] ID %q; expected %q", index, scenario.ScenarioID, expectedID)
		}
		if seenIDs[scenario.ScenarioID] {
			return summary, fmt.Errorf("duplicate scenario ID %q", scenario.ScenarioID)
		}
		seenIDs[scenario.ScenarioID] = true

		pair := pairs[scenario.PairID]
		if pair == nil {
			pair = &PairState{Family: scenario.Family, Variants: make(map[string]bool)}
			pairs[scenario.PairID] = pair
		}
		if pair.Family != scenario.Family {
			return summary, fmt.Errorf("pair %s contains more than one family", scenario.PairID)
		}
		if scenario.Variant != "A" && scenario.Variant != "B" {
			return summary, fmt.Errorf("scenario %s has invalid pair variant %q", scenario.ScenarioID, scenario.Variant)
		}
		if pair.Variants[scenario.Variant] {
			return summary, fmt.Errorf("pair %s repeats variant %s", scenario.PairID, scenario.Variant)
		}
		pair.Variants[scenario.Variant] = true
		families[scenario.Family]++

		actual, err := evaluateScenario(scenario)
		if err != nil {
			return summary, fmt.Errorf("%s: %w", scenario.ScenarioID, err)
		}
		expected := oracle.Outcomes[index]
		if expected.ScenarioID != scenario.ScenarioID {
			return summary, fmt.Errorf("oracle order/link mismatch at row %d: %q != %q", index, expected.ScenarioID, scenario.ScenarioID)
		}
		if err := compareOutcome(actual, expected); err != nil {
			return summary, fmt.Errorf("%s oracle mismatch: %w", scenario.ScenarioID, err)
		}
		changedCount += len(actual.AffectedClaimNodes)
		unknownCount += len(actual.UnknownClaimNodes)
		unaffectedCount += len(actual.UnaffectedClaimNodes)
	}
	if len(pairs) != scenarios.PairCount {
		return summary, fmt.Errorf("pair_count says %d but found %d pair IDs", scenarios.PairCount, len(pairs))
	}
	for pairID, pair := range pairs {
		if len(pair.Variants) != 2 || !pair.Variants["A"] || !pair.Variants["B"] {
			return summary, fmt.Errorf("pair %s must contain exactly one A and one B variant", pairID)
		}
	}

	return DatasetSummary{
		CorpusID: scenarios.CorpusID, ScenarioCount: len(scenarios.Scenarios), PairCount: len(pairs),
		OutcomeCount: len(oracle.Outcomes), FamilyCounts: families, ChangedClaims: changedCount,
		UnknownClaims: unknownCount, UnaffectedClaims: unaffectedCount,
		ScenarioSHA256: digest(scenarioBytes), OracleSHA256: digest(oracleBytes),
	}, nil
}

func evaluateScenario(s Scenario) (Outcome, error) {
	nodes := make(map[string]Node, len(s.Nodes))
	adjacency := make(map[string][]string, len(s.Nodes))
	indegree := make(map[string]int, len(s.Nodes))
	claimNodes := make(map[string]bool)
	for _, node := range s.Nodes {
		if node.NodeID == "" {
			return Outcome{}, errors.New("node ID must not be empty")
		}
		if _, exists := nodes[node.NodeID]; exists {
			return Outcome{}, fmt.Errorf("duplicate node ID %q", node.NodeID)
		}
		switch node.NodeKind {
		case "DEPENDENCY":
			if node.ClaimDimension != "" {
				return Outcome{}, fmt.Errorf("dependency node %q must not declare a claim dimension", node.NodeID)
			}
		case "CLAIM_DIMENSION":
			if node.ClaimDimension == "" {
				return Outcome{}, fmt.Errorf("claim node %q must declare its dimension", node.NodeID)
			}
			claimNodes[node.NodeID] = true
		default:
			return Outcome{}, fmt.Errorf("node %q has unknown node_kind %q", node.NodeID, node.NodeKind)
		}
		nodes[node.NodeID] = node
		indegree[node.NodeID] = 0
	}
	if len(claimNodes) == 0 {
		return Outcome{}, errors.New("scenario must contain at least one claim dimension node")
	}

	seenEdges := make(map[string]bool)
	for _, edge := range s.DependencyEdges {
		from, fromOK := nodes[edge.FromNodeID]
		_, toOK := nodes[edge.ToNodeID]
		if !fromOK || !toOK {
			return Outcome{}, fmt.Errorf("dangling dependency edge %q -> %q", edge.FromNodeID, edge.ToNodeID)
		}
		if from.NodeKind == "CLAIM_DIMENSION" {
			return Outcome{}, fmt.Errorf("claim node %q cannot be a dependency source", edge.FromNodeID)
		}
		if edge.FromNodeID == edge.ToNodeID {
			return Outcome{}, fmt.Errorf("self edge on %q", edge.FromNodeID)
		}
		if edge.Relation != "SUPPORTS_OR_CONDITIONS" {
			return Outcome{}, fmt.Errorf("unsupported edge relation %q", edge.Relation)
		}
		key := edge.FromNodeID + " " + edge.ToNodeID
		if seenEdges[key] {
			return Outcome{}, fmt.Errorf("duplicate dependency edge %q -> %q", edge.FromNodeID, edge.ToNodeID)
		}
		seenEdges[key] = true
		adjacency[edge.FromNodeID] = append(adjacency[edge.FromNodeID], edge.ToNodeID)
		indegree[edge.ToNodeID]++
	}
	if err := ensureAcyclic(nodes, adjacency, indegree); err != nil {
		return Outcome{}, err
	}

	unknownScope := make(map[string]bool)
	for _, nodeID := range s.UnknownScopeClaimNodes {
		if !claimNodes[nodeID] {
			return Outcome{}, fmt.Errorf("unknown-scope reference %q is not a claim node", nodeID)
		}
		if unknownScope[nodeID] {
			return Outcome{}, fmt.Errorf("duplicate unknown-scope claim %q", nodeID)
		}
		unknownScope[nodeID] = true
	}
	if s.ChangeCoverage != "COMPLETE" && s.ChangeCoverage != "INCOMPLETE" && s.ChangeCoverage != "UNKNOWN" {
		return Outcome{}, fmt.Errorf("unsupported change coverage %q", s.ChangeCoverage)
	}

	affected := make(map[string]bool)
	unknown := make(map[string]bool)
	rejected := make(map[string]bool)
	eventIDs := make(map[string]bool)
	for _, event := range s.ChangeEvents {
		if event.EventID == "" || event.EventType == "" {
			return Outcome{}, errors.New("change event ID and type must not be empty")
		}
		if eventIDs[event.EventID] {
			return Outcome{}, fmt.Errorf("duplicate change event ID %q", event.EventID)
		}
		eventIDs[event.EventID] = true
		target, ok := nodes[event.TargetNodeID]
		if !ok || target.NodeKind != "DEPENDENCY" {
			return Outcome{}, fmt.Errorf("event %q target %q is not a dependency node", event.EventID, event.TargetNodeID)
		}
		switch event.Observation {
		case "OBSERVED_QUALIFIED":
			for claim := range reachableClaims(event.TargetNodeID, adjacency, nodes) {
				affected[claim] = true
			}
		case "UNKNOWN":
			for claim := range reachableClaims(event.TargetNodeID, adjacency, nodes) {
				unknown[claim] = true
			}
		case "UNSUPPORTED":
			rejected[event.EventID] = true
		default:
			return Outcome{}, fmt.Errorf("event %q has unknown observation state %q", event.EventID, event.Observation)
		}
	}
	if s.ChangeCoverage == "INCOMPLETE" || s.ChangeCoverage == "UNKNOWN" {
		for claim := range unknownScope {
			unknown[claim] = true
		}
	}
	// An unresolved coverage gap has precedence over a partially observed impact.
	for claim := range unknown {
		delete(affected, claim)
	}
	unaffected := make(map[string]bool)
	for claim := range claimNodes {
		if !affected[claim] && !unknown[claim] {
			unaffected[claim] = true
		}
	}
	return Outcome{
		ScenarioID: s.ScenarioID,
		AffectedClaimNodes: sortedKeys(affected),
		UnknownClaimNodes: sortedKeys(unknown),
		UnaffectedClaimNodes: sortedKeys(unaffected),
		RejectedEventIDs: sortedKeys(rejected),
	}, nil
}

func reachableClaims(start string, adjacency map[string][]string, nodes map[string]Node) map[string]bool {
	visited := make(map[string]bool)
	claims := make(map[string]bool)
	queue := []string{start}
	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]
		if visited[current] {
			continue
		}
		visited[current] = true
		if node, ok := nodes[current]; ok && node.NodeKind == "CLAIM_DIMENSION" {
			claims[current] = true
		}
		queue = append(queue, adjacency[current]...)
	}
	return claims
}

func ensureAcyclic(nodes map[string]Node, adjacency map[string][]string, indegree map[string]int) error {
	queue := make([]string, 0, len(nodes))
	local := make(map[string]int, len(indegree))
	for id, degree := range indegree {
		local[id] = degree
		if degree == 0 {
			queue = append(queue, id)
		}
	}
	visited := 0
	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]
		visited++
		for _, next := range adjacency[current] {
			local[next]--
			if local[next] == 0 {
				queue = append(queue, next)
			}
		}
	}
	if visited != len(nodes) {
		return errors.New("dependency graph contains a cycle")
	}
	return nil
}

func compareOutcome(actual, expected Outcome) error {
	checks := []struct {
		name string
		a, e []string
	}{
		{"affected_claim_nodes", actual.AffectedClaimNodes, expected.AffectedClaimNodes},
		{"unknown_claim_nodes", actual.UnknownClaimNodes, expected.UnknownClaimNodes},
		{"unaffected_claim_nodes", actual.UnaffectedClaimNodes, expected.UnaffectedClaimNodes},
		{"rejected_event_ids", actual.RejectedEventIDs, expected.RejectedEventIDs},
	}
	for _, check := range checks {
		if !sameStrings(check.a, check.e) {
			return fmt.Errorf("%s: computed %v, oracle %v", check.name, check.a, check.e)
		}
	}
	if expected.ReasonCode == "" {
		return errors.New("oracle reason_code must not be empty")
	}
	return nil
}

func sameStrings(a, b []string) bool {
	aa, bb := append([]string(nil), a...), append([]string(nil), b...)
	sort.Strings(aa)
	sort.Strings(bb)
	if len(aa) != len(bb) {
		return false
	}
	for i := range aa {
		if aa[i] != bb[i] {
			return false
		}
	}
	return true
}

func sortedKeys(values map[string]bool) []string {
	out := make([]string, 0, len(values))
	for value := range values {
		out = append(out, value)
	}
	sort.Strings(out)
	return out
}

func decodeStrict(data []byte, target any) error {
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(target); err != nil {
		return err
	}
	var extra any
	if err := dec.Decode(&extra); err == io.EOF {
		return nil
	} else if err == nil {
		return errors.New("multiple JSON values in file")
	} else {
		return err
	}
}

func digest(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}
