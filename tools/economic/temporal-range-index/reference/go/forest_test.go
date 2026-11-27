package temporalrange

import (
	"crypto/sha256"
	"fmt"
	"testing"
)

func forestOwnerBindingV1(owner, history string, sequence, treeSize uint64, root [32]byte, gaps []OwnerGapRangeV1) OwnerIntervalIndexBindingV1 {
	return OwnerIntervalIndexBindingV1{
		OwnerRef: owner, OwnerHistoryRef: history,
		OwnerSequence: sequence, TreeSize: treeSize,
		OwnerLogRootDigest:    fmt.Sprintf("sha256:%064x", sequence+100),
		IntervalIndexRoot:     root,
		ProjectionAuditDigest: fmt.Sprintf("sha256:%064x", sequence+200),
		Gaps:                  gaps,
	}
}

func TestFrontierForestComposesPerOwnerIntervalProofs(t *testing.T) {
	scope := intervalScopeFixtureV1()
	treeA, err := NewIntervalTreeV1(scope, []IntervalRecordV1{
		intervalRecordFixtureV1("grant:A:1", 10, 30),
		intervalRecordFixtureV1("grant:A:2", 40, 50),
	})
	if err != nil {
		t.Fatal(err)
	}
	treeB, err := NewIntervalTreeV1(scope, []IntervalRecordV1{
		intervalRecordFixtureV1("grant:B:1", 12, 22),
		intervalRecordFixtureV1("grant:B:2", 32, 42),
	})
	if err != nil {
		t.Fatal(err)
	}

	// The owner sequence cuts are deliberately incomparable in magnitude.
	// Their lexical manifest order is canonical encoding, not causal order.
	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 100, 2, treeA.RootHash(), nil),
			forestOwnerBindingV1("owner:B", "history:B", 2, 2, treeB.RootHash(), nil),
		},
	}
	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(15, 0)
	proofA, _ := treeA.ProvePointQueryV1(query)
	proofB, _ := treeB.ProvePointQueryV1(query)
	queryProof := TemporalFrontierQueryProofV1{
		Manifest: manifest, Query: query,
		OwnerProofs: []OwnerIntervalQueryProofV1{
			{OwnerRef: "owner:A", Proof: proofA},
			{OwnerRef: "owner:B", Proof: proofB},
		},
	}
	result, err := VerifyTemporalFrontierQueryProofV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:B", "owner:A"}, query, queryProof, root,
	)
	if err != nil {
		t.Fatal(err)
	}
	if result.Disposition != CompleteForDeclaredIndexForestV1 {
		t.Fatalf("unexpected disposition: %s", result.Disposition)
	}
	if result.ProjectionCompleteness != ProjectionCompletenessNotEvaluatedV1 {
		t.Fatalf("projection claim was upgraded: %s", result.ProjectionCompleteness)
	}
	if result.OwnerCount != 2 || result.HasOwnerGaps {
		t.Fatalf("unexpected forest status: %+v", result)
	}
	if len(result.Records) != 2 {
		t.Fatalf("record count=%d want=2", len(result.Records))
	}
	if result.Records[0].OwnerRef != "owner:A" || result.Records[1].OwnerRef != "owner:B" {
		t.Fatalf("owner provenance lost or reordered: %+v", result.Records)
	}
}

func TestFrontierForestRejectsOmittedOrSubstitutedOwners(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("x", 1, 9)})
	if err != nil {
		t.Fatal(err)
	}
	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 1, 1, tree.RootHash(), nil),
		},
	}
	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(5, 0)
	intervalProof, _ := tree.ProvePointQueryV1(query)
	proof := TemporalFrontierQueryProofV1{
		Manifest: manifest, Query: query,
		OwnerProofs: []OwnerIntervalQueryProofV1{{OwnerRef: "owner:A", Proof: intervalProof}},
	}
	if _, err := VerifyTemporalFrontierQueryProofV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:A", "owner:B"}, query, proof, root,
	); err != ErrForestOwnerSetMismatch {
		t.Fatalf("omitted owner did not fail closed: %v", err)
	}
	proof.OwnerProofs[0].OwnerRef = "owner:B"
	if _, err := VerifyTemporalFrontierQueryProofV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:A"}, query, proof, root,
	); err != ErrForestOwnerProofMismatch {
		t.Fatalf("owner proof substitution did not fail closed: %v", err)
	}
}

func TestFrontierForestPreservesGapsAndProjectionUncertainty(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("x", 1, 9)})
	if err != nil {
		t.Fatal(err)
	}
	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 10, 8, tree.RootHash(), []OwnerGapRangeV1{{First: 3, Last: 4}}),
		},
	}
	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(5, 0)
	intervalProof, _ := tree.ProvePointQueryV1(query)
	proof := TemporalFrontierQueryProofV1{
		Manifest: manifest, Query: query,
		OwnerProofs: []OwnerIntervalQueryProofV1{{OwnerRef: "owner:A", Proof: intervalProof}},
	}
	result, err := VerifyTemporalFrontierQueryProofV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:A"}, query, proof, root,
	)
	if err != nil {
		t.Fatal(err)
	}
	if !result.HasOwnerGaps {
		t.Fatal("owner gap was hidden")
	}
	if result.ProjectionCompleteness != ProjectionCompletenessNotEvaluatedV1 {
		t.Fatal("structural interval proof was promoted to receipt-projection completeness")
	}
}

func TestFrontierForestRootBindsOwnerHistoryAndIndexRoot(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("x", 1, 9)})
	if err != nil {
		t.Fatal(err)
	}
	base := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 5, 3, tree.RootHash(), nil),
		},
	}
	first, err := TemporalFrontierForestRootV1(base)
	if err != nil {
		t.Fatal(err)
	}
	changedHistory := base
	changedHistory.Owners = append([]OwnerIntervalIndexBindingV1(nil), base.Owners...)
	changedHistory.Owners[0].OwnerHistoryRef = "history:B"
	second, err := TemporalFrontierForestRootV1(changedHistory)
	if err != nil {
		t.Fatal(err)
	}
	if first == second {
		t.Fatal("forest root did not bind owner history identity")
	}
	changedIndex := base
	changedIndex.Owners = append([]OwnerIntervalIndexBindingV1(nil), base.Owners...)
	changedIndex.Owners[0].IntervalIndexRoot = sha256.Sum256([]byte("different index"))
	third, err := TemporalFrontierForestRootV1(changedIndex)
	if err != nil {
		t.Fatal(err)
	}
	if first == third {
		t.Fatal("forest root did not bind interval index root")
	}
}

func TestFrontierForestKeepsLogTreeSizeIndependentFromOwnerSequence(t *testing.T) {
	scope := intervalScopeFixtureV1()
	base := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 5, 6, [32]byte{1}, nil),
		},
	}
	if _, err := TemporalFrontierForestRootV1(base); err != nil {
		t.Fatalf("independent log tree size was compared to owner sequence: %v", err)
	}
	base.Owners[0] = forestOwnerBindingV1("owner:A", "history:A", 5, 3, [32]byte{1}, []OwnerGapRangeV1{
		{First: 3, Last: 4}, {First: 4, Last: 5},
	})
	if _, err := TemporalFrontierForestRootV1(base); err == nil {
		t.Fatal("overlapping/adjacent gap ranges passed")
	}
}

func TestFrontierForestRejectsFrontierAndRootSubstitution(t *testing.T) {
	scope := intervalScopeFixtureV1()
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{intervalRecordFixtureV1("x", 1, 9)})
	if err != nil {
		t.Fatal(err)
	}
	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 1, 1, tree.RootHash(), nil),
		},
	}
	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		t.Fatal(err)
	}
	query := intervalPointV1(5, 0)
	intervalProof, _ := tree.ProvePointQueryV1(query)
	proof := TemporalFrontierQueryProofV1{
		Manifest: manifest, Query: query,
		OwnerProofs: []OwnerIntervalQueryProofV1{{OwnerRef: "owner:A", Proof: intervalProof}},
	}
	if _, err := VerifyTemporalFrontierQueryProofV1(
		scope.VerifierRef, scope.PolicyEpochRef, "sha256:"+fmt.Sprintf("%064x", 123),
		scope, []string{"owner:A"}, query, proof, root,
	); err != ErrForestFrontierMismatch {
		t.Fatalf("frontier substitution: %v", err)
	}
	wrongRoot := sha256.Sum256([]byte("wrong forest root"))
	if _, err := VerifyTemporalFrontierQueryProofV1(
		scope.VerifierRef, scope.PolicyEpochRef, scope.SourceFrontierDigest,
		scope, []string{"owner:A"}, query, proof, wrongRoot,
	); err != ErrForestRootMismatch {
		t.Fatalf("forest root substitution: %v", err)
	}
}


func TestFrontierForestMatchesCanonicalKnownAnswer(t *testing.T) {
	scope := intervalScopeFixtureV1()
	manifest := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 100, 2, [32]byte{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, nil),
			forestOwnerBindingV1("owner:B", "history:B", 2, 2, [32]byte{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, nil),
		},
	}
	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		t.Fatal(err)
	}
	if got := fmt.Sprintf("%x", root); got != "244d1cfcc40aba335e96e6de1382fbde2ccf499d3a8cdb1b8c794dd6f4076705" {
		t.Fatalf("forest known-answer mismatch: %s", got)
	}
}

func TestFrontierForestRejectsInvalidUTF8AndNonCanonicalOwnerOrder(t *testing.T) {
	scope := intervalScopeFixtureV1()
	base := TemporalFrontierIndexManifestV1{
		VerifierRef: scope.VerifierRef, PolicyEpochRef: scope.PolicyEpochRef,
		FrontierDigest: scope.SourceFrontierDigest, IntervalProfile: "MYCELIX-INTERVAL-TREE-V0.1",
		QueryScope: scope,
		Owners: []OwnerIntervalIndexBindingV1{
			forestOwnerBindingV1("owner:A", "history:A", 5, 3, [32]byte{1}, nil),
			forestOwnerBindingV1("owner:B", "history:B", 5, 3, [32]byte{2}, nil),
		},
	}
	reversed := base
	reversed.Owners = append([]OwnerIntervalIndexBindingV1(nil), base.Owners...)
	reversed.Owners[0], reversed.Owners[1] = reversed.Owners[1], reversed.Owners[0]
	if _, err := TemporalFrontierForestRootV1(reversed); err == nil {
		t.Fatal("non-canonical owner order passed")
	}
	invalidUTF8 := base
	invalidUTF8.Owners = append([]OwnerIntervalIndexBindingV1(nil), base.Owners...)
	invalidUTF8.Owners[0].OwnerRef = string([]byte{0xff})
	if _, err := TemporalFrontierForestRootV1(invalidUTF8); err == nil {
		t.Fatal("invalid UTF-8 owner reference passed")
	}
}
