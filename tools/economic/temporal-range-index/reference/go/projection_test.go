package temporalrange

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"testing"
)

func appendReceiptFixtureStringV1(out *bytes.Buffer, value string) {
	_ = binary.Write(out, binary.BigEndian, uint32(len(value)))
	_, _ = out.WriteString(value)
}

func canonicalReceiptFixtureV1(scope IntervalScopeV1, receiptID, assertionID, assertionDigest, owner string, sequence uint64) []byte {
	var out bytes.Buffer
	out.WriteString(canonicalKnowledgeReceiptMagicV1)
	for _, value := range []string{receiptID, scope.VerifierRef, scope.PolicyEpochRef, assertionID, assertionDigest, owner} {
		appendReceiptFixtureStringV1(&out, value)
	}
	_ = binary.Write(&out, binary.BigEndian, sequence)
	_ = binary.Write(&out, binary.BigEndian, uint32(0))
	return out.Bytes()
}

func receiptDigestFixtureV1(entry []byte) string {
	sum := sha256.Sum256(entry)
	return fmt.Sprintf("sha256:%x", sum)
}

func projectionInventoryFixtureV1(scope IntervalScopeV1) (OwnerIntervalIndexBindingV1, AcceptedReceiptInventoryV1) {
	assertionRef := "assertion:grant-42"
	assertionDigest := intervalDigestV1(70)
	entry0 := canonicalReceiptFixtureV1(scope, "receipt:2", assertionRef, assertionDigest, "owner:grants", 2)
	entry1 := canonicalReceiptFixtureV1(scope, "receipt:8", "metadata:receipt-8", intervalDigestV1(71), "owner:grants", 8)
	receiptDigest := receiptDigestFixtureV1(entry0)
	record := IntervalRecordV1{
		AssertionRef: assertionRef,
		ValidFrom: intervalPointV1(10, 125_000_000),
		ValidTo: intervalFiniteV1(30, 750_000_000),
		ReceiptDigest: receiptDigest,
	}
	tree, err := NewIntervalTreeV1(scope, []IntervalRecordV1{record})
	if err != nil {
		panic(err)
	}
	logRoot := rfc9162TreeHashV1([][]byte{entry0, entry1})
	gaps := []OwnerGapRangeV1{{First: 1, Last: 1}, {First: 3, Last: 7}, {First: 9, Last: 9}}
	binding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:grants",
		OwnerHistoryRef: "history:grants:v1",
		OwnerSequence: 9,
		TreeSize: 2,
		OwnerLogRootDigest: fmt.Sprintf("sha256:%x", logRoot),
		IntervalIndexRoot: tree.RootHash(),
		Gaps: gaps,
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
				ReceiptDigest: receiptDigest, CanonicalReceiptEntryBytes: entry0,
				AssertionID: assertionRef, Class: ReceiptProjectsIntervalAssertionV1,
				AssertionRef: record.AssertionRef,
				ValidFrom: record.ValidFrom, ValidTo: record.ValidTo,
			},
			{
				ReceiptLogLeafIndex: 1, OwnerSequence: 8,
				ReceiptDigest: receiptDigestFixtureV1(entry1), CanonicalReceiptEntryBytes: entry1,
				AssertionID: "metadata:receipt-8", Class: ReceiptIsNonTemporalV1,
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
	if computed.ReceiptInventoryAuthentication != ReceiptLogRootRecomputedButTrustExternalV1 || !computed.OwnerLogRootMatches {
		t.Fatalf("owner log root was not independently reconstructed: %+v", computed)
	}
	if computed.ProjectionMappingAuthentication != ProjectionMappingAuthenticationNotEvaluatedV1 {
		t.Fatalf("projection semantics were overclaimed: %s", computed.ProjectionMappingAuthentication)
	}
	if computed.ReceiptCount != 2 || computed.IntervalAssertionCount != 1 || computed.NonTemporalReceiptCount != 1 {
		t.Fatalf("wrong projection counts: %+v", computed)
	}
	if got := fmt.Sprintf("%x", computed.InventoryDigest); got != "ce77089d983075fd2b23bdf6d7c67ef3568e56ab8040be5eabe4d2a13c87bac4" {
		t.Fatalf("accepted inventory known-answer mismatch: %s", got)
	}
	if got := fmt.Sprintf("%x", computed.DerivedIntervalIndexRoot); got != "2caeb7b8577945eeff7350d19d5824723e48cafa7a816a4812a3b5bbe652cca8" {
		t.Fatalf("derived interval root known-answer mismatch: %s", got)
	}
	if got := fmt.Sprintf("%x", computed.AuditDigest); got != "612cdec4914808234bea8719d744e84f09b0a1e3cf0f91371d03d96058dd9851" {
		t.Fatalf("projection audit known-answer mismatch: %s", got)
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
	regressed.Receipts[1].CanonicalReceiptEntryBytes = canonicalReceiptFixtureV1(
		scope, "receipt:8", "metadata:receipt-8", intervalDigestV1(71), "owner:grants", 2,
	)
	regressed.Receipts[1].ReceiptDigest = receiptDigestFixtureV1(regressed.Receipts[1].CanonicalReceiptEntryBytes)
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
	duplicateBytes := canonicalReceiptFixtureV1(scope, "receipt:8", "assertion:grant-42", intervalDigestV1(44), "owner:grants", 8)
	duplicate.Receipts[1] = AcceptedReceiptProjectionV1{
		ReceiptLogLeafIndex: 1, OwnerSequence: 8,
		ReceiptDigest: receiptDigestFixtureV1(duplicateBytes),
		CanonicalReceiptEntryBytes: duplicateBytes,
		AssertionID: "assertion:grant-42",
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
		OwnerLogRootDigest: fmt.Sprintf("sha256:%x", sha256.Sum256(nil)),
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
		OwnerLogRootDigest: "",
		IntervalIndexRoot: allNonTemporalTree.RootHash(),
	}
	allBytes := canonicalReceiptFixtureV1(scope, "receipt:1", "metadata:1", intervalDigestV1(52), "owner:metadata", 1)
	allBinding.OwnerLogRootDigest = fmt.Sprintf("sha256:%x", rfc9162TreeHashV1([][]byte{allBytes}))
	allInventory := AcceptedReceiptInventoryV1{
		OwnerRef: allBinding.OwnerRef, OwnerHistoryRef: allBinding.OwnerHistoryRef,
		OwnerSequenceCut: 1, TreeSize: 1, OwnerLogRootDigest: allBinding.OwnerLogRootDigest,
		Receipts: []AcceptedReceiptProjectionV1{{
			ReceiptLogLeafIndex: 0, OwnerSequence: 1,
			ReceiptDigest: receiptDigestFixtureV1(allBytes),
			CanonicalReceiptEntryBytes: allBytes,
			AssertionID: "metadata:1",
			Class: ReceiptIsNonTemporalV1,
		}},
	}
	allAudit, err := ComputeOwnerProjectionReplayAuditV1(scope, allBinding, allInventory)
	if err != nil {
		t.Fatal(err)
	}
	if allAudit.NonTemporalReceiptCount != 1 || allAudit.IntervalAssertionCount != 0 {
		t.Fatalf("wrong all-non-temporal counts: %+v", allAudit)
	}
}
