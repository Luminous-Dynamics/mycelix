package progressantichain

import (
	"strings"
	"testing"
)

const (
	digestA = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
)

func testScope(owners ...string) ProgressScopeV1 {
	return ProgressScopeV1{
		SourceRef: "directory:source",
		RegistryRef: "registry:main",
		RegistryEpoch: "epoch:7",
		ProfileRef: "universe:v1",
		QueryScopeDigest: digestA,
		OwnerRefs: owners,
	}
}

func testCut(values map[string]uint64) ProgressCutV1 {
	cut := ProgressCutV1{}
	for owner, sequence := range values {
		cut.Owners = append(cut.Owners, OwnerSequenceV1{OwnerRef: owner, Sequence: sequence})
	}
	return cut
}

func testAntichain(scope ProgressScopeV1, values ...map[string]uint64) ProgressAntichainV1 {
	result := ProgressAntichainV1{Scope: scope}
	for _, point := range values {
		result.Points = append(result.Points, testCut(point))
	}
	return result
}

func TestIncomparableOwnerCutsRemainAnAntichain(t *testing.T) {
	scope := testScope("owner:A", "owner:B")
	antichain := testAntichain(scope,
		map[string]uint64{"owner:A": 5, "owner:B": 2},
		map[string]uint64{"owner:A": 2, "owner:B": 5},
	)
	if _, err := ValidateProgressAntichainV1(antichain); err != nil {
		t.Fatalf("incomparable owner cuts rejected: %v", err)
	}
	result := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 3, "owner:B": 3}))
	if result.Disposition != ProgressStableRelativeToDeclared {
		t.Fatalf("query should be stable relative to incomparable progress points: %+v", result)
	}
	if result.ProgressAuthentication != NotEvaluated || result.SourceUniverseCompleteness != NotEvaluated {
		t.Fatalf("structural progress promoted trust: %+v", result)
	}
}

func TestAnyPredecessorKeepsQueryProvisional(t *testing.T) {
	antichain := testAntichain(testScope("owner:A", "owner:B"),
		map[string]uint64{"owner:A": 5, "owner:B": 2},
		map[string]uint64{"owner:A": 2, "owner:B": 5},
	)
	result := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 5, "owner:B": 5}))
	if result.Disposition != ProgressProvisional {
		t.Fatalf("query with an outstanding predecessor finalized: %+v", result)
	}
}

func TestSingleOwnerBoundaryMatchesScalarOrder(t *testing.T) {
	antichain := testAntichain(testScope("owner:A"), map[string]uint64{"owner:A": 10})
	before := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 9}))
	at := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 10}))
	if before.Disposition != ProgressStableRelativeToDeclared {
		t.Fatalf("query before frontier should be stable relative to declaration: %+v", before)
	}
	if at.Disposition != ProgressProvisional {
		t.Fatalf("query at frontier must remain provisional: %+v", at)
	}
}

func TestEmptyAntichainIsOnlyDeclaredCompletion(t *testing.T) {
	antichain := testAntichain(testScope("owner:A"))
	result := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 0}))
	if result.Disposition != ProgressStableRelativeToDeclared {
		t.Fatalf("empty progress antichain did not close declared progress: %+v", result)
	}
	if result.ProgressAuthentication != NotEvaluated || result.SourceUniverseCompleteness != NotEvaluated {
		t.Fatalf("empty antichain promoted source trust: %+v", result)
	}
	if !strings.Contains(result.Reason, "empty") {
		t.Fatalf("empty-antichain semantics not explicit: %+v", result)
	}
}

func TestDominatedPointsAreRejected(t *testing.T) {
	antichain := testAntichain(testScope("owner:A", "owner:B"),
		map[string]uint64{"owner:A": 2, "owner:B": 2},
		map[string]uint64{"owner:A": 3, "owner:B": 3},
	)
	if _, err := ValidateProgressAntichainV1(antichain); err == nil {
		t.Fatal("dominated point was accepted into antichain")
	}
}

func TestDuplicatePointsAreRejected(t *testing.T) {
	point := map[string]uint64{"owner:A": 2}
	antichain := testAntichain(testScope("owner:A"), point, point)
	if _, err := ValidateProgressAntichainV1(antichain); err == nil {
		t.Fatal("duplicate progress points accepted")
	}
}

func TestMissingAndForeignOwnerCoordinatesFailClosed(t *testing.T) {
	scope := testScope("owner:A", "owner:B")
	missing := testAntichain(scope, map[string]uint64{"owner:A": 1})
	foreign := testAntichain(scope, map[string]uint64{"owner:A": 1, "owner:C": 1})
	for name, candidate := range map[string]ProgressAntichainV1{"missing": missing, "foreign": foreign} {
		t.Run(name, func(t *testing.T) {
			if _, err := ValidateProgressAntichainV1(candidate); err == nil {
				t.Fatal("malformed owner coordinates accepted")
			}
		})
	}
}

func TestDuplicateOwnerCoordinateRejected(t *testing.T) {
	scope := testScope("owner:A")
	cut := ProgressCutV1{Owners: []OwnerSequenceV1{
		{OwnerRef: "owner:A", Sequence: 1},
		{OwnerRef: "owner:A", Sequence: 2},
	}}
	antichain := ProgressAntichainV1{Scope: scope, Points: []ProgressCutV1{cut}}
	if _, err := ValidateProgressAntichainV1(antichain); err == nil {
		t.Fatal("duplicate owner coordinate accepted")
	}
}

func TestScopeAndEpochSubstitutionChangeDigest(t *testing.T) {
	scope := testScope("owner:A")
	antichain := testAntichain(scope, map[string]uint64{"owner:A": 3})
	first, err := ValidateProgressAntichainV1(antichain)
	if err != nil {
		t.Fatal(err)
	}
	nextScope := scope
	nextScope.RegistryEpoch = "epoch:8"
	changed, err := ValidateProgressAntichainV1(testAntichain(nextScope, map[string]uint64{"owner:A": 3}))
	if err != nil {
		t.Fatal(err)
	}
	if first == changed {
		t.Fatal("registry epoch substitution did not change antichain digest")
	}
}

func TestProgressAdvanceAcceptsIncomparableMonotoneSuccessors(t *testing.T) {
	scope := testScope("owner:A", "owner:B")
	previous := testAntichain(scope, map[string]uint64{"owner:A": 2, "owner:B": 2})
	next := testAntichain(scope,
		map[string]uint64{"owner:A": 3, "owner:B": 2},
		map[string]uint64{"owner:A": 2, "owner:B": 3},
	)
	result := AdvanceProgressAntichainV1(previous, next)
	if result.Disposition != AdvanceMonotone {
		t.Fatalf("valid incomparable advance rejected: %+v", result)
	}
	if result.ProgressAuthentication != NotEvaluated {
		t.Fatalf("advance result promoted source authentication: %+v", result)
	}
}

func TestProgressAdvanceRejectsRegression(t *testing.T) {
	scope := testScope("owner:A", "owner:B")
	previous := testAntichain(scope, map[string]uint64{"owner:A": 2, "owner:B": 2})
	next := testAntichain(scope, map[string]uint64{"owner:A": 1, "owner:B": 3})
	result := AdvanceProgressAntichainV1(previous, next)
	if result.Disposition != AdvanceIndeterminate {
		t.Fatalf("regressing progress point accepted: %+v", result)
	}
}

func TestProgressAdvanceRejectsEpochOrScopeChange(t *testing.T) {
	scope := testScope("owner:A")
	previous := testAntichain(scope, map[string]uint64{"owner:A": 2})
	nextScope := scope
	nextScope.RegistryEpoch = "epoch:8"
	next := testAntichain(nextScope, map[string]uint64{"owner:A": 3})
	result := AdvanceProgressAntichainV1(previous, next)
	if result.Disposition != AdvanceIndeterminate {
		t.Fatalf("epoch substitution treated as monotone progress: %+v", result)
	}
}

func TestProgressAdvanceEmptyFrontierSemantics(t *testing.T) {
	scope := testScope("owner:A")
	previous := testAntichain(scope, map[string]uint64{"owner:A": 5})
	closed := testAntichain(scope)
	if got := AdvanceProgressAntichainV1(previous, closed); got.Disposition != AdvanceMonotone {
		t.Fatalf("progress completion was rejected: %+v", got)
	}
	reopened := AdvanceProgressAntichainV1(closed, previous)
	if reopened.Disposition != AdvanceIndeterminate {
		t.Fatalf("empty frontier reopened without new scope: %+v", reopened)
	}
}

func TestProgressDigestsAreIndependentOfInputOrdering(t *testing.T) {
	first := testAntichain(testScope("owner:A", "owner:B"),
		map[string]uint64{"owner:A": 5, "owner:B": 2},
		map[string]uint64{"owner:A": 2, "owner:B": 5},
	)
	second := testAntichain(testScope("owner:B", "owner:A"),
		map[string]uint64{"owner:B": 5, "owner:A": 2},
		map[string]uint64{"owner:B": 2, "owner:A": 5},
	)
	a, err := ValidateProgressAntichainV1(first)
	if err != nil {
		t.Fatal(err)
	}
	b, err := ValidateProgressAntichainV1(second)
	if err != nil {
		t.Fatal(err)
	}
	if a != b {
		t.Fatalf("canonical digest depends on caller ordering: %s != %s", a, b)
	}
}

func TestMalformedFailureCertificateIsDeterministic(t *testing.T) {
	antichain := testAntichain(testScope("owner:A"), map[string]uint64{"owner:B": 2})
	first := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 2}))
	for i := 0; i < 100; i++ {
		next := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 2}))
		if first.CertificateDigest != next.CertificateDigest || first.Reason != next.Reason {
			t.Fatalf("nondeterministic failure certificate: %+v != %+v", first, next)
		}
	}
}

func TestDistinctMalformedSubmissionsHaveDistinctFailureCertificates(t *testing.T) {
	scope := testScope("owner:A", "owner:B")
	first := ProgressAntichainV1{Scope: scope, Points: []ProgressCutV1{testCut(map[string]uint64{"owner:A": 1})}}
	second := ProgressAntichainV1{Scope: scope, Points: []ProgressCutV1{testCut(map[string]uint64{"owner:A": 2})}}
	a := EvaluateProgressAtV1(first, testCut(map[string]uint64{"owner:A": 3, "owner:B": 3}))
	b := EvaluateProgressAtV1(second, testCut(map[string]uint64{"owner:A": 3, "owner:B": 3}))
	if a.Disposition != ProgressIndeterminate || b.Disposition != ProgressIndeterminate {
		t.Fatalf("malformed antichain unexpectedly evaluated: a=%+v b=%+v", a, b)
	}
	if a.CertificateDigest == b.CertificateDigest || a.ProgressAntichainDigest == b.ProgressAntichainDigest {
		t.Fatal("distinct malformed antichain submissions collapsed to one receipt")
	}
}

func TestMalformedQueryCutIsBoundIntoFailureCertificate(t *testing.T) {
	antichain := testAntichain(testScope("owner:A", "owner:B"), map[string]uint64{"owner:A": 1, "owner:B": 1})
	first := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 2}))
	second := EvaluateProgressAtV1(antichain, testCut(map[string]uint64{"owner:A": 3}))
	if first.Disposition != ProgressIndeterminate || second.Disposition != ProgressIndeterminate {
		t.Fatalf("malformed query cut unexpectedly evaluated: first=%+v second=%+v", first, second)
	}
	if first.CertificateDigest == second.CertificateDigest || first.QueryCutDigest == second.QueryCutDigest {
		t.Fatal("distinct malformed query cuts collapsed to one receipt")
	}
}

func TestMalformedAdvanceSubmissionsAreBoundIntoCertificate(t *testing.T) {
	scope := testScope("owner:A", "owner:B")
	oldA := ProgressAntichainV1{Scope: scope, Points: []ProgressCutV1{testCut(map[string]uint64{"owner:A": 1})}}
	oldB := ProgressAntichainV1{Scope: scope, Points: []ProgressCutV1{testCut(map[string]uint64{"owner:A": 2})}}
	next := testAntichain(scope, map[string]uint64{"owner:A": 3, "owner:B": 3})
	a := AdvanceProgressAntichainV1(oldA, next)
	b := AdvanceProgressAntichainV1(oldB, next)
	if a.Disposition != AdvanceIndeterminate || b.Disposition != AdvanceIndeterminate {
		t.Fatalf("malformed previous antichain unexpectedly advanced: a=%+v b=%+v", a, b)
	}
	if a.CertificateDigest == b.CertificateDigest || a.PreviousAntichainDigest == b.PreviousAntichainDigest {
		t.Fatal("distinct malformed previous antichains collapsed to one receipt")
	}
}
