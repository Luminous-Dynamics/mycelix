package main

import "testing"

func TestQualifiedChangeTraversesTransitiveDependencies(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-001",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "dep:policy", NodeKind: "DEPENDENCY"},
			{NodeID: "dep:import", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:effect", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "effect_authority"},
			{NodeID: "claim:history", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "historical_record"},
		},
		DependencyEdges: []Edge{
			{FromNodeID: "dep:policy", ToNodeID: "dep:import", Relation: "SUPPORTS_OR_CONDITIONS"},
			{FromNodeID: "dep:import", ToNodeID: "claim:effect", Relation: "SUPPORTS_OR_CONDITIONS"},
		},
		ChangeEvents: []ChangeEvent{
			{EventID: "EV-1", TargetNodeID: "dep:policy", EventType: "POLICY_CHANGED", Observation: "OBSERVED_QUALIFIED"},
		},
	}
	got, err := evaluateScenario(s)
	if err != nil {
		t.Fatal(err)
	}
	if !sameStrings(got.AffectedClaimNodes, []string{"claim:effect"}) {
		t.Fatalf("affected claims = %v", got.AffectedClaimNodes)
	}
	if !sameStrings(got.UnaffectedClaimNodes, []string{"claim:history"}) {
		t.Fatalf("unaffected claims = %v", got.UnaffectedClaimNodes)
	}
}

func TestIncompleteCoverageMarksOnlyDeclaredUnknownScope(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-002",
		ChangeCoverage: "INCOMPLETE",
		Nodes: []Node{
			{NodeID: "dep:static", NodeKind: "DEPENDENCY"},
			{NodeID: "dep:unknown-dynamic", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:routes", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "complete_mediation"},
			{NodeID: "claim:pep", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "pep_identity"},
		},
		DependencyEdges: []Edge{
			{FromNodeID: "dep:static", ToNodeID: "claim:routes", Relation: "SUPPORTS_OR_CONDITIONS"},
			{FromNodeID: "dep:unknown-dynamic", ToNodeID: "claim:routes", Relation: "SUPPORTS_OR_CONDITIONS"},
			{FromNodeID: "dep:static", ToNodeID: "claim:pep", Relation: "SUPPORTS_OR_CONDITIONS"},
		},
		UnknownScopeClaimNodes: []string{"claim:routes"},
	}
	got, err := evaluateScenario(s)
	if err != nil {
		t.Fatal(err)
	}
	if !sameStrings(got.UnknownClaimNodes, []string{"claim:routes"}) {
		t.Fatalf("unknown claims = %v", got.UnknownClaimNodes)
	}
	if !sameStrings(got.UnaffectedClaimNodes, []string{"claim:pep"}) {
		t.Fatalf("unaffected claims = %v", got.UnaffectedClaimNodes)
	}
}

func TestUnrelatedQualifiedChangeLeavesClaimUnaffected(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-003",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "dep:tokenizer", NodeKind: "DEPENDENCY"},
			{NodeID: "dep:policy", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:model", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "model_behavior"},
			{NodeID: "claim:policy", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "policy_semantics"},
		},
		DependencyEdges: []Edge{
			{FromNodeID: "dep:tokenizer", ToNodeID: "claim:model", Relation: "SUPPORTS_OR_CONDITIONS"},
			{FromNodeID: "dep:policy", ToNodeID: "claim:policy", Relation: "SUPPORTS_OR_CONDITIONS"},
		},
		ChangeEvents: []ChangeEvent{
			{EventID: "EV-2", TargetNodeID: "dep:tokenizer", EventType: "TOKENIZER_CHANGED", Observation: "OBSERVED_QUALIFIED"},
		},
	}
	got, err := evaluateScenario(s)
	if err != nil {
		t.Fatal(err)
	}
	if !sameStrings(got.AffectedClaimNodes, []string{"claim:model"}) {
		t.Fatalf("affected claims = %v", got.AffectedClaimNodes)
	}
	if !sameStrings(got.UnaffectedClaimNodes, []string{"claim:policy"}) {
		t.Fatalf("unaffected claims = %v", got.UnaffectedClaimNodes)
	}
}

func TestUnknownEventDoesNotBecomeNoChange(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-004",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "dep:key", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:authority", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "current_key_authority"},
		},
		DependencyEdges: []Edge{{FromNodeID: "dep:key", ToNodeID: "claim:authority", Relation: "SUPPORTS_OR_CONDITIONS"}},
		ChangeEvents: []ChangeEvent{{EventID: "EV-3", TargetNodeID: "dep:key", EventType: "KEY_STATE", Observation: "UNKNOWN"}},
	}
	got, err := evaluateScenario(s)
	if err != nil {
		t.Fatal(err)
	}
	if !sameStrings(got.UnknownClaimNodes, []string{"claim:authority"}) {
		t.Fatalf("unknown claims = %v", got.UnknownClaimNodes)
	}
	if len(got.UnaffectedClaimNodes) != 0 {
		t.Fatalf("unknown claim was treated as unaffected: %v", got.UnaffectedClaimNodes)
	}
}

func TestDependencyCycleFailsClosed(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-005",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "dep:a", NodeKind: "DEPENDENCY"},
			{NodeID: "dep:b", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:c", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "c"},
		},
		DependencyEdges: []Edge{
			{FromNodeID: "dep:a", ToNodeID: "dep:b", Relation: "SUPPORTS_OR_CONDITIONS"},
			{FromNodeID: "dep:b", ToNodeID: "dep:a", Relation: "SUPPORTS_OR_CONDITIONS"},
			{FromNodeID: "dep:b", ToNodeID: "claim:c", Relation: "SUPPORTS_OR_CONDITIONS"},
		},
	}
	if _, err := evaluateScenario(s); err == nil {
		t.Fatal("cyclic dependency graph was accepted")
	}
}

func TestDanglingEdgeFailsClosed(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-006",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "dep:a", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:c", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "c"},
		},
		DependencyEdges: []Edge{
			{FromNodeID: "dep:a", ToNodeID: "claim:missing", Relation: "SUPPORTS_OR_CONDITIONS"},
		},
	}
	if _, err := evaluateScenario(s); err == nil {
		t.Fatal("dangling dependency edge was accepted")
	}
}

func TestClaimNodeCannotTargetChangeEvent(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-007",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "claim:c", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "c"},
		},
		ChangeEvents: []ChangeEvent{
			{EventID: "EV-4", TargetNodeID: "claim:c", EventType: "INVALID_TARGET", Observation: "OBSERVED_QUALIFIED"},
		},
	}
	if _, err := evaluateScenario(s); err == nil {
		t.Fatal("claim node was accepted as change-event target")
	}
}

func TestUnsupportedEventIsNotTreatedAsQualifiedChange(t *testing.T) {
	s := Scenario{
		ScenarioID: "CLOSE-008",
		ChangeCoverage: "COMPLETE",
		Nodes: []Node{
			{NodeID: "dep:a", NodeKind: "DEPENDENCY"},
			{NodeID: "claim:c", NodeKind: "CLAIM_DIMENSION", ClaimDimension: "c"},
		},
		DependencyEdges: []Edge{{FromNodeID: "dep:a", ToNodeID: "claim:c", Relation: "SUPPORTS_OR_CONDITIONS"}},
		ChangeEvents: []ChangeEvent{{EventID: "EV-5", TargetNodeID: "dep:a", EventType: "UNVERIFIED_EVENT", Observation: "UNSUPPORTED"}},
	}
	got, err := evaluateScenario(s)
	if err != nil {
		t.Fatal(err)
	}
	if len(got.AffectedClaimNodes) != 0 || len(got.UnknownClaimNodes) != 0 || !sameStrings(got.RejectedEventIDs, []string{"EV-5"}) {
		t.Fatalf("unexpected unsupported-event result: %+v", got)
	}
}
