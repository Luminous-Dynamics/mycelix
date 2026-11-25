package sagaverifier

import (
	"os"
	"reflect"
	"strings"
	"testing"
)

func causalRef(owner, event string) CausalEventRefV1 {
	return CausalEventRefV1{OwnerID: owner, EventID: event}
}

func causalEvent(owner, event string, sequence uint64, predecessor *CausalEventRefV1, causedBy ...CausalEventRefV1) CausalEventV1 {
	return CausalEventV1{
		OwnerID: owner, EventID: event, OwnerSequence: sequence,
		OwnerPredecessor: predecessor, CausedBy: causedBy,
	}
}

func causalSampleV1() []CausalEventV1 {
	p0 := causalRef("payments", "p0")
	t0 := causalRef("treasury", "t0")
	return []CausalEventV1{
		causalEvent("payments", "p0", 0, nil),
		causalEvent("treasury", "t0", 0, nil),
		causalEvent("payments", "p1", 1, &p0),
		causalEvent("hearth", "h0", 0, nil, p0, t0),
	}
}

func TestCausalProjectionPreservesIndependentOwnerEffects(t *testing.T) {
	projection, err := ProjectCausalGraphV1(causalSampleV1())
	if err != nil {
		t.Fatal(err)
	}
	if projection.EventCount != 4 || projection.EdgeCount != 3 {
		t.Fatalf("unexpected graph counts: %+v", projection)
	}
	wantLayers := [][]CausalEventRefV1{
		{causalRef("payments", "p0"), causalRef("treasury", "t0")},
		{causalRef("hearth", "h0"), causalRef("payments", "p1")},
	}
	if !reflect.DeepEqual(projection.TopologicalLayers, wantLayers) {
		t.Fatalf("layers = %#v, want %#v", projection.TopologicalLayers, wantLayers)
	}
}

func TestCausalProjectionOwnerFrontiersAreIndependent(t *testing.T) {
	projection, err := ProjectCausalGraphV1(causalSampleV1())
	if err != nil {
		t.Fatal(err)
	}
	want := []CausalOwnerFrontierV1{
		{OwnerID: "hearth", Sequence: 0, EventID: "h0"},
		{OwnerID: "payments", Sequence: 1, EventID: "p1"},
		{OwnerID: "treasury", Sequence: 0, EventID: "t0"},
	}
	if !reflect.DeepEqual(projection.OwnerFrontiers, want) {
		t.Fatalf("frontiers = %#v, want %#v", projection.OwnerFrontiers, want)
	}
}

func TestCausalProjectionDigestIsInputOrderIndependent(t *testing.T) {
	events := causalSampleV1()
	first, err := ProjectCausalGraphV1(events)
	if err != nil {
		t.Fatal(err)
	}
	reversed := make([]CausalEventV1, len(events))
	for i := range events {
		reversed[len(events)-1-i] = events[i]
	}
	second, err := ProjectCausalGraphV1(reversed)
	if err != nil {
		t.Fatal(err)
	}
	if first.GraphDigest != second.GraphDigest ||
		!reflect.DeepEqual(first.TopologicalLayers, second.TopologicalLayers) ||
		!reflect.DeepEqual(first.OwnerFrontiers, second.OwnerFrontiers) {
		t.Fatalf("input order changed projection: first=%+v second=%+v", first, second)
	}
}

func TestCausalGraphDigestMatchesSharedKnownAnswer(t *testing.T) {
	contents, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-payment-causal-graph-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	expected := strings.TrimSpace(string(contents))
	projection, err := ProjectCausalGraphV1(causalSampleV1())
	if err != nil {
		t.Fatal(err)
	}
	if projection.GraphDigest != expected {
		t.Fatalf("digest = %s, want %s", projection.GraphDigest, expected)
	}
}

func TestCausalProjectionRejectsOwnerSequenceGap(t *testing.T) {
	p0 := causalRef("payments", "p0")
	events := []CausalEventV1{
		causalEvent("payments", "p0", 0, nil),
		causalEvent("payments", "p2", 2, &p0),
	}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalOwnerSequenceGap {
		t.Fatalf("error = %v, want %v", err, ErrCausalOwnerSequenceGap)
	}
}

func TestCausalProjectionRejectsWrongOwnerPredecessor(t *testing.T) {
	t0 := causalRef("treasury", "t0")
	events := []CausalEventV1{
		causalEvent("payments", "p0", 0, nil),
		causalEvent("payments", "p1", 1, &t0),
		causalEvent("treasury", "t0", 0, nil),
	}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalWrongOwnerPredecessor {
		t.Fatalf("error = %v, want %v", err, ErrCausalWrongOwnerPredecessor)
	}
}

func TestCausalProjectionRejectsMissingCausalReference(t *testing.T) {
	missing := causalRef("payments", "missing")
	events := []CausalEventV1{causalEvent("treasury", "t0", 0, nil, missing)}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalMissingReference {
		t.Fatalf("error = %v, want %v", err, ErrCausalMissingReference)
	}
}

func TestCausalProjectionRejectsCycle(t *testing.T) {
	p0 := causalRef("payments", "p0")
	t0 := causalRef("treasury", "t0")
	events := []CausalEventV1{
		causalEvent("payments", "p0", 0, nil, t0),
		causalEvent("treasury", "t0", 0, nil, p0),
	}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalCycle {
		t.Fatalf("error = %v, want %v", err, ErrCausalCycle)
	}
}

func TestCausalProjectionRejectsDuplicateOwnerScopedEventReference(t *testing.T) {
	p0 := causalRef("payments", "p0")
	events := []CausalEventV1{
		causalEvent("payments", "p0", 0, nil),
		causalEvent("payments", "p0", 1, &p0),
	}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalDuplicateEventReference {
		t.Fatalf("error = %v, want %v", err, ErrCausalDuplicateEventReference)
	}
}

func TestCausalProjectionRejectsDuplicateCausalReference(t *testing.T) {
	p0 := causalRef("payments", "p0")
	events := []CausalEventV1{
		causalEvent("payments", "p0", 0, nil),
		causalEvent("treasury", "t0", 0, nil, p0, p0),
	}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalDuplicateReference {
		t.Fatalf("error = %v, want %v", err, ErrCausalDuplicateReference)
	}
}

func TestCausalProjectionDoesNotInventGlobalOrder(t *testing.T) {
	events := []CausalEventV1{
		causalEvent("owner-z", "z0", 0, nil),
		causalEvent("owner-a", "a0", 0, nil),
	}
	projection, err := ProjectCausalGraphV1(events)
	if err != nil {
		t.Fatal(err)
	}
	if len(projection.TopologicalLayers) != 1 || len(projection.TopologicalLayers[0]) != 2 {
		t.Fatalf("independent events were globally ordered: %#v", projection.TopologicalLayers)
	}
}

func TestCausalProjectionAllowsSameNativeEventIDUnderDifferentOwners(t *testing.T) {
	events := []CausalEventV1{
		causalEvent("payments", "commit", 0, nil),
		causalEvent("treasury", "commit", 0, nil),
	}
	projection, err := ProjectCausalGraphV1(events)
	if err != nil {
		t.Fatal(err)
	}
	if projection.EventCount != 2 || projection.EdgeCount != 0 ||
		len(projection.TopologicalLayers) != 1 || len(projection.TopologicalLayers[0]) != 2 {
		t.Fatalf("cross-owner event IDs were conflated: %+v", projection)
	}
}

func TestCausalProjectionRejectsDuplicateOwnerSequence(t *testing.T) {
	events := []CausalEventV1{
		causalEvent("payments", "p0", 0, nil),
		causalEvent("payments", "p1", 0, nil),
	}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalDuplicateOwnerSequence {
		t.Fatalf("error = %v, want %v", err, ErrCausalDuplicateOwnerSequence)
	}
}

func TestCausalProjectionRejectsSelfCausality(t *testing.T) {
	self := causalRef("payments", "p0")
	events := []CausalEventV1{causalEvent("payments", "p0", 0, nil, self)}
	if _, err := ProjectCausalGraphV1(events); err != ErrCausalSelfReference {
		t.Fatalf("error = %v, want %v", err, ErrCausalSelfReference)
	}
}

func TestCausalProjectionRejectsEmptyGraph(t *testing.T) {
	if _, err := ProjectCausalGraphV1(nil); err != ErrCausalEmptyGraph {
		t.Fatalf("error = %v, want %v", err, ErrCausalEmptyGraph)
	}
}
