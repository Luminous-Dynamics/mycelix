package temporalrange

import (
	"crypto/sha256"
	"fmt"
	"testing"
)

func projectionInventoryFixtureV1(scope IntervalScopeV1) (OwnerIntervalIndexBindingV1, AcceptedReceiptInventoryV1) {
	record := IntervalRecordV1{
		AssertionRef: "assertion:grant-42",
		ValidFrom: intervalPointV1(10, 125_000_000),
		ValidTo: intervalFiniteV1(30, 750_000_000),
		ReceiptDigest: intervalDigestV1(41),
	}
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{record})
	if err != nil {
		panic(err)
	}
	binding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:grants",
		OwnerHistoryRef: "history:grants:v1",
		OwnerSequence: 9,
		TreeSize: 2,
		OwnerLogRootDigest: intervalDigestV1(42),
		IntervalIndexRoot: tree.RootHash(),
		Gaps: []OwnerGapRangeV1{{First: 4, Last: 5}},
	}
	inventory := AcceptedReceiptInventoryV1{
		OwnerRef: binding.OwnerRef,
		OwnerHistoryRef: binding.OwnerHistoryRef,
		OwnerSequenceCut: binding.OwnerSequence,
		TreeSize: binding.TreeSize,
		OwnerLogRootDigest: binding.OwnerLogRootDigest,
		Receipts: []AcceptedReceiptProjectionV1{
			{
				ReceiptLogLeafIndex: 0, OwnerSequence: 2,
				ReceiptDigest: record.ReceiptDigest,
				Class: ReceiptProjectsIntervalAssertionV1,
				AssertionRef: record.AssertionRef,
				ValidFrom: record.ValidFrom, ValidTo: record.ValidTo,
			},
			{
				ReceiptLogLeafIndex: 1, OwnerSequence: 8,
				ReceiptDigest: intervalDigestV1(43),
				Class: ReceiptIsNonTemporalV1,
			},
		},
	}
	return binding, inventory
}

func TestProjectionReplayMatchesExactSuppliedInventory(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	computed, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		t.Fatal(err)
	}
	if computed.Disposition != ProjectionMatchesSuppliedAcceptedInventoryV1 {
		t.Fatalf("wrong disposition: %s", computed.Disposition)
	}
	if computed.ReceiptInventoryAuthentication != ReceiptInventoryAuthenticationNotEvaluatedV1 {
		t.Fatalf("receipt inventory auth was promoted: %s", computed.ReceiptInventoryAuthentication)
	}
	if computed.ReceiptCount != 2 || computed.IntervalAssertionCount != 1 || computed.NonTemporalReceiptCount != 1 {
		t.Fatalf("wrong projection counts: %+v", computed)
	}
	if !computed.HasOwnerGaps {
		t.Fatal("explicit owner gaps were not preserved")
	}
	binding.ProjectionAuditDigest = fmt.Sprintf("sha256:%x", computed.AuditDigest)
	verified, err := VerifyOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		t.Fatal(err)
	}
	if verified.AuditDigest != computed.AuditDigest || verified.InventoryDigest != computed.InventoryDigest {
		t.Fatal("replay audit was not deterministic")
	}
}

func TestProjectionReplayRejectsMissingOrReorderedLogLeaves(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	missing := inventory
	missing.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts[:1]...)
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, missing); err != ErrProjectionLeafCoverage {
		t.Fatalf("missing receipt did not fail closed: %v", err)
	}
	reordered := inventory
	reordered.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	reordered.Receipts[0], reordered.Receipts[1] = reordered.Receipts[1], reordered.Receipts[0]
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, reordered); err != ErrProjectionLeafCoverage {
		t.Fatalf("reordered receipt leaf did not fail closed: %v", err)
	}
	duplicate := inventory
	duplicate.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	duplicate.Receipts[1].ReceiptLogLeafIndex = 0
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, duplicate); err != ErrProjectionLeafCoverage {
		t.Fatalf("duplicate receipt leaf did not fail closed: %v", err)
	}
}

func TestProjectionReplayDetectsOmittedOrAlteredIntervalAssertion(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	omitted := inventory
	omitted.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	omitted.Receipts[0].Class = ReceiptIsNonTemporalV1
	omitted.Receipts[0].AssertionRef = ""
	omitted.Receipts[0].ValidFrom = IntervalInstantV1{}
	omitted.Receipts[0].ValidTo = IntervalEndV1{}
	// Reclassifying the receipt changes the derived projection root even though the
	// caller still supplies the original committed interval-index root.
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, omitted); err != ErrProjectionIndexRootMismatch {
		t.Fatalf("omitted interval assertion was accepted: %v", err)
	}
	altered := inventory
	altered.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	altered.Receipts[0].ValidTo = intervalFiniteV1(31, 0)
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, altered); err != ErrProjectionIndexRootMismatch {
		t.Fatalf("altered interval endpoint was accepted: %v", err)
	}
}

func TestProjectionReplayRejectsWrongOwnerHistoryCutAndReceiptSequence(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	wrongOwner := inventory
	wrongOwner.OwnerRef = "owner:other"
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, wrongOwner); err != ErrProjectionInventoryMismatch {
		t.Fatalf("owner substitution: %v", err)
	}
	wrongCut := inventory
	wrongCut.OwnerSequenceCut--
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, wrongCut); err != ErrProjectionInventoryMismatch {
		t.Fatalf("owner cut substitution: %v", err)
	}
	regressed := inventory
	regressed.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	regressed.Receipts[1].OwnerSequence = regressed.Receipts[0].OwnerSequence
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, regressed); err != ErrProjectionSequenceOrder {
		t.Fatalf("non-increasing receipt sequence: %v", err)
	}
	inGap := inventory
	inGap.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	inGap.Receipts[1].OwnerSequence = 4
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inGap); err != ErrProjectionSequenceOrder {
		t.Fatalf("receipt in unresolved gap: %v", err)
	}
}

func TestProjectionReplayRejectsMalformedNonTemporalAndDuplicateAssertions(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	smuggled := inventory
	smuggled.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	smuggled.Receipts[1].AssertionRef = "hidden-interval"
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, smuggled); err != ErrProjectionReceiptClass {
		t.Fatalf("non-temporal interval fields were accepted: %v", err)
	}
	duplicate := inventory
	duplicate.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	duplicate.Receipts[1] = AcceptedReceiptProjectionV1{
		ReceiptLogLeafIndex: 1, OwnerSequence: 8,
		ReceiptDigest: intervalDigestV1(44),
		Class: ReceiptProjectsIntervalAssertionV1,
		AssertionRef: "assertion:grant-42",
		ValidFrom: intervalPointV1(11, 0), ValidTo: intervalFiniteV1(40, 0),
	}
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, duplicate); err != ErrIntervalMalformed {
		t.Fatalf("duplicate projected assertion was accepted: %v", err)
	}
}

func TestProjectionReplayRequiresCommittedAuditDigest(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	audit, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		t.Fatal(err)
	}
	binding.ProjectionAuditDigest = "sha256:" + fmt.Sprintf("%064x", 999)
	if _, err := VerifyOwnerProjectionReplayAuditV1(scope, binding, inventory); err != ErrProjectionAuditMismatch {
		t.Fatalf("wrong audit digest accepted: %v", err)
	}
	binding.ProjectionAuditDigest = fmt.Sprintf("sha256:%x", audit.AuditDigest)
	if _, err := VerifyOwnerProjectionReplayAuditV1(scope, binding, inventory); err != nil {
		t.Fatal(err)
	}
}

func TestProjectionReplaySupportsEmptyAndAllNonTemporalInventory(t *testing.T) {
	scope := intervalScopeFixtureV1()
	emptyTree, err := NewIntervalTreeV1(scope, nil)
	if err != nil {
		t.Fatal(err)
	}
	emptyBinding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:empty", OwnerHistoryRef: "history:empty",
		OwnerSequence: 0, TreeSize: 0,
		OwnerLogRootDigest: intervalDigestV1(50),
		IntervalIndexRoot: emptyTree.RootHash(),
	}
	emptyInventory := AcceptedReceiptInventoryV1{
		OwnerRef: emptyBinding.OwnerRef, OwnerHistoryRef: emptyBinding.OwnerHistoryRef,
		OwnerLogRootDigest: emptyBinding.OwnerLogRootDigest,
	}
	emptyAudit, err := ComputeOwnerProjectionReplayAuditV1(scope, emptyBinding, emptyInventory)
	if err != nil {
		t.Fatal(err)
	}
	emptyBinding.ProjectionAuditDigest = fmt.Sprintf("sha256:%x", emptyAudit.AuditDigest)
	if _, err := VerifyOwnerProjectionReplayAuditV1(scope, emptyBinding, emptyInventory); err != nil {
		t.Fatal(err)
	}

	allNonTemporalTree, err := NewIntervalTreeV1(scope, nil)
	if err != nil {
		t.Fatal(err)
	}
	allBinding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:metadata", OwnerHistoryRef: "history:metadata",
		OwnerSequence: 1, TreeSize: 1,
		OwnerLogRootDigest: intervalDigestV1(51),
		IntervalIndexRoot: allNonTemporalTree.RootHash(),
	}
	allInventory := AcceptedReceiptInventoryV1{
		OwnerRef: allBinding.OwnerRef, OwnerHistoryRef: allBinding.OwnerHistoryRef,
		OwnerSequenceCut: 1, TreeSize: 1, OwnerLogRootDigest: allBinding.OwnerLogRootDigest,
		Receipts: []AcceptedReceiptProjectionV1{{
			ReceiptLogLeafIndex: 0, OwnerSequence: 1,
			ReceiptDigest: intervalDigestV1(52), Class: ReceiptIsNonTemporalV1,
		}},
	}
	allAudit, err := ComputeOwnerProjectionReplayAuditV1(scope, allBinding, allInventory)
	if err != nil {
		t.Fatal(err)
	}
	if allAudit.NonTemporalReceiptCount != 1 || allAudit.IntervalAssertionCount != 0 {
		t.Fatalf("wrong all-non-temporal counts: %+v", allAudit)
	}
	_ = sha256.Sum256([]byte("compile-time sha256 dependency"))
}
