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
	assertionPayload := []byte(`{"profile":"MYCELIX-INTERVAL-ASSERTION-SYNTHETIC-V1","assertion_id":"assertion:grant-42","subject_ref":"grant:42","claim_id":"revoked","valid_from":"2026-11-20T10:00:00.125Z","valid_to":"2026-11-20T10:00:30.750Z"}`)
	assertionDigest := receiptDigestFixtureV1(assertionPayload)
	metadataPayload := []byte(`{"profile":"MYCELIX-METADATA-ASSERTION-SYNTHETIC-V1","assertion_id":"metadata:receipt-9","kind":"receipt-metadata"}`)
	metadataDigest := receiptDigestFixtureV1(metadataPayload)
	entry0 := canonicalReceiptFixtureV1(scope, "receipt:1", assertionRef, assertionDigest, "owner:grants", 1)
	ordinaryOwnerEvent := []byte("MYCELIX-TEST-OWNER-LOG:owner:grants:4")
	entry2 := canonicalReceiptFixtureV1(scope, "receipt:9", "metadata:receipt-9", metadataDigest, "owner:grants", 9)

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

	logEntries := [][]byte{entry0, ordinaryOwnerEvent, entry2}
	logRoot := rfc9162TreeHashV1(logEntries)
	// The owner cut reaches sequence 9. Six positions are explicit gaps,
	// leaving 3 authenticated owner-log leaves at sequences 1, 4 and 9.
	gaps := []OwnerGapRangeV1{{First: 2, Last: 3}, {First: 5, Last: 8}}
	binding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:grants",
		OwnerHistoryRef: "history:grants:v1",
		OwnerSequence: 9,
		TreeSize: 3,
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
		LogLeaves: []OwnerLogLeafV1{
			{LeafIndex: 0, OwnerSequence: 1, EntryBytes: entry0},
			{LeafIndex: 1, OwnerSequence: 4, EntryBytes: ordinaryOwnerEvent},
			{LeafIndex: 2, OwnerSequence: 9, EntryBytes: entry2},
		},
		Receipts: []AcceptedReceiptProjectionV1{
			{
				ReceiptLogLeafIndex: 0, OwnerSequence: 1,
				ReceiptDigest: receiptDigest, AssertionDigest: assertionDigest, AssertionPayloadBytes: assertionPayload, CanonicalReceiptEntryBytes: entry0,
				AssertionID: assertionRef, Class: ReceiptProjectsIntervalAssertionV1,
				AssertionRef: record.AssertionRef,
				ValidFrom: record.ValidFrom, ValidTo: record.ValidTo,
			},
			{
				ReceiptLogLeafIndex: 2, OwnerSequence: 9,
				ReceiptDigest: receiptDigestFixtureV1(entry2), AssertionDigest: metadataDigest, AssertionPayloadBytes: metadataPayload, CanonicalReceiptEntryBytes: entry2,
				AssertionID: "metadata:receipt-9", Class: ReceiptIsNonTemporalV1,
			},
		},
	}
	return binding, inventory
}

func TestProjectionReplayMatchesCompleteMixedOwnerLogInventory(t *testing.T) {
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
	if computed.ReceiptCount != 2 || computed.NonKnowledgeReceiptLeafCount != 1 ||
		computed.IntervalAssertionCount != 1 || computed.NonTemporalReceiptCount != 1 {
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
	if verified.RecomputedOwnerLogRoot != computed.RecomputedOwnerLogRoot {
		t.Fatal("owner log root changed during deterministic replay")
	}
}

func TestProjectionReplayMixedLogKnownAnswerVectors(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	if got := binding.OwnerLogRootDigest; got != "sha256:2f54b60be7b27a704b31b4fb2d2da50cd06f3e6843f966c97934c65574ef6b40" {
		t.Fatalf("full owner-log root vector mismatch: %s", got)
	}
	if got := inventory.Receipts[0].ReceiptDigest; got != "sha256:aee54246a474eb7cc88db961fc86d33507dc174971606058e852f7500b72566c" {
		t.Fatalf("first canonical receipt digest mismatch: %s", got)
	}
	if got := inventory.Receipts[1].ReceiptDigest; got != "sha256:0e3c882a46133141e13daa00f9b25cf4edc818ebc649419721cd3af363c004ff" {
		t.Fatalf("second canonical receipt digest mismatch: %s", got)
	}
	audit, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		t.Fatal(err)
	}
	if got := fmt.Sprintf("%x", audit.DerivedIntervalIndexRoot); got != "c399a222836b9cae71b80bae4fa920b3a2a86690f03baab8c207e28a3bf869ef" {
		t.Fatalf("derived interval root vector mismatch: %s", got)
	}
	if got := fmt.Sprintf("%x", audit.InventoryDigest); got != "634d09acd50f6fb2059ae43fa40d15a7c1c039ec82ce5d9f365d381d3c02f9da" {
		t.Fatalf("full inventory digest vector mismatch: %s", got)
	}
	if got := fmt.Sprintf("%x", audit.AuditDigest); got != "7ebf5680d430c78caff59fb059408c96459a03d621d6e274372ba5de7bccd515" {
		t.Fatalf("projection audit digest vector mismatch: %s", got)
	}
}

func TestProjectionReplayRequiresEveryLogLeafAndEveryReceiptDisposition(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)

	missingLogLeaf := inventory
	missingLogLeaf.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves[:2]...)
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, missingLogLeaf); err != ErrProjectionLeafCoverage {
		t.Fatalf("missing owner-log leaf did not fail closed: %v", err)
	}

	reordered := inventory
	reordered.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	reordered.LogLeaves[0], reordered.LogLeaves[1] = reordered.LogLeaves[1], reordered.LogLeaves[0]
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, reordered); err != ErrProjectionLeafCoverage {
		t.Fatalf("reordered owner-log leaves did not fail closed: %v", err)
	}

	duplicateLeaf := inventory
	duplicateLeaf.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	duplicateLeaf.LogLeaves[1].LeafIndex = 0
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, duplicateLeaf); err != ErrProjectionLeafCoverage {
		t.Fatalf("duplicate owner-log leaf index did not fail closed: %v", err)
	}

	omittedReceiptDisposition := inventory
	omittedReceiptDisposition.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts[:1]...)
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, omittedReceiptDisposition); err != ErrProjectionReceiptOmitted {
		t.Fatalf("receipt leaf without projection disposition passed: %v", err)
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
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, omitted); err != ErrProjectionIndexRootMismatch {
		t.Fatalf("reclassified interval receipt was accepted: %v", err)
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
	regressed.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	regressed.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	regressed.LogLeaves[2].OwnerSequence = 1
	regressed.Receipts[1].OwnerSequence = 1
	regressed.Receipts[1].CanonicalReceiptEntryBytes = canonicalReceiptFixtureV1(
		scope, "receipt:9", "metadata:receipt-9", intervalDigestV1(71), "owner:grants", 1,
	)
	regressed.Receipts[1].ReceiptDigest = receiptDigestFixtureV1(regressed.Receipts[1].CanonicalReceiptEntryBytes)
	regressed.LogLeaves[2].EntryBytes = regressed.Receipts[1].CanonicalReceiptEntryBytes
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, regressed); err != ErrProjectionSequenceOrder {
		t.Fatalf("non-increasing owner sequence: %v", err)
	}

	inGap := inventory
	inGap.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	inGap.LogLeaves[2].OwnerSequence = 5
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inGap); err != ErrProjectionSequenceOrder {
		t.Fatalf("owner log leaf inside unresolved gap: %v", err)
	}
}

func TestProjectionReplayRejectsNonTemporalSmugglingAndDuplicateAssertions(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)

	smuggled := inventory
	smuggled.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	smuggled.Receipts[1].AssertionRef = "hidden-interval"
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, smuggled); err != ErrProjectionReceiptClass {
		t.Fatalf("non-temporal interval fields were accepted: %v", err)
	}

	duplicate := inventory
	duplicate.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	duplicate.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	duplicatePayload := []byte(`{"profile":"MYCELIX-INTERVAL-ASSERTION-SYNTHETIC-V1","assertion_id":"assertion:grant-42","subject_ref":"grant:42","claim_id":"revoked","valid_from":"2026-11-21T11:00:00Z","valid_to":"2026-11-21T11:00:40Z"}`)
	duplicateAssertionDigest := receiptDigestFixtureV1(duplicatePayload)
	duplicateBytes := canonicalReceiptFixtureV1(
		scope, "receipt:9", "assertion:grant-42", duplicateAssertionDigest, "owner:grants", 9,
	)
	duplicate.LogLeaves[2].EntryBytes = duplicateBytes
	duplicate.Receipts[1] = AcceptedReceiptProjectionV1{
		ReceiptLogLeafIndex: 2,
		OwnerSequence: 9,
		ReceiptDigest: receiptDigestFixtureV1(duplicateBytes),
		AssertionDigest: duplicateAssertionDigest,
		AssertionPayloadBytes: duplicatePayload,
		CanonicalReceiptEntryBytes: duplicateBytes,
		AssertionID: "assertion:grant-42",
		Class: ReceiptProjectsIntervalAssertionV1,
		AssertionRef: "assertion:grant-42",
		ValidFrom: intervalPointV1(11, 0),
		ValidTo: intervalFiniteV1(40, 0),
	}
	newRoot := rfc9162TreeHashV1([][]byte{
		duplicate.LogLeaves[0].EntryBytes,
		duplicate.LogLeaves[1].EntryBytes,
		duplicate.LogLeaves[2].EntryBytes,
	})
	binding.OwnerLogRootDigest = fmt.Sprintf("sha256:%x", newRoot)
	duplicate.OwnerLogRootDigest = binding.OwnerLogRootDigest
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
	emptyLogRoot := fmt.Sprintf("sha256:%x", sha256.Sum256(nil))
	emptyBinding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:empty",
		OwnerHistoryRef: "history:empty",
		OwnerSequence: 0,
		TreeSize: 0,
		OwnerLogRootDigest: emptyLogRoot,
		IntervalIndexRoot: emptyTree.RootHash(),
	}
	emptyInventory := AcceptedReceiptInventoryV1{
		OwnerRef: emptyBinding.OwnerRef,
		OwnerHistoryRef: emptyBinding.OwnerHistoryRef,
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
	allPayload := []byte(`{"profile":"MYCELIX-METADATA-ASSERTION-SYNTHETIC-V1","assertion_id":"metadata:1","kind":"receipt-metadata"}`)
	allAssertionDigest := receiptDigestFixtureV1(allPayload)
	allBytes := canonicalReceiptFixtureV1(
		scope, "receipt:1", "metadata:1", allAssertionDigest, "owner:metadata", 1,
	)
	allBinding := OwnerIntervalIndexBindingV1{
		OwnerRef: "owner:metadata",
		OwnerHistoryRef: "history:metadata",
		OwnerSequence: 1,
		TreeSize: 1,
		OwnerLogRootDigest: fmt.Sprintf("sha256:%x", rfc9162TreeHashV1([][]byte{allBytes})),
		IntervalIndexRoot: allNonTemporalTree.RootHash(),
	}
	allInventory := AcceptedReceiptInventoryV1{
		OwnerRef: allBinding.OwnerRef,
		OwnerHistoryRef: allBinding.OwnerHistoryRef,
		OwnerSequenceCut: 1,
		TreeSize: 1,
		OwnerLogRootDigest: allBinding.OwnerLogRootDigest,
		LogLeaves: []OwnerLogLeafV1{{LeafIndex: 0, OwnerSequence: 1, EntryBytes: allBytes}},
		Receipts: []AcceptedReceiptProjectionV1{{
			ReceiptLogLeafIndex: 0,
			OwnerSequence: 1,
			ReceiptDigest: receiptDigestFixtureV1(allBytes),
			AssertionDigest: allAssertionDigest,
			AssertionPayloadBytes: allPayload,
			CanonicalReceiptEntryBytes: allBytes,
			AssertionID: "metadata:1",
			Class: ReceiptIsNonTemporalV1,
		}},
	}
	allAudit, err := ComputeOwnerProjectionReplayAuditV1(scope, allBinding, allInventory)
	if err != nil {
		t.Fatal(err)
	}
	if allAudit.NonTemporalReceiptCount != 1 || allAudit.IntervalAssertionCount != 0 || allAudit.NonKnowledgeReceiptLeafCount != 0 {
		t.Fatalf("wrong all-non-temporal counts: %+v", allAudit)
	}
}

func TestProjectionReplayRejectsMalformedKnowledgeReceiptPrefix(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	malformed := inventory
	malformed.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	malformed.LogLeaves[1].EntryBytes = []byte("MYCELIX-KNOWLEDGE-RECEIPT-V1-missing-nul")
	// The leaf is now a malformed receipt-envelope candidate, not an ordinary event.
	malformed.LogLeaves[1].OwnerSequence = 4
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, malformed); err != ErrProjectionReceiptClass {
		t.Fatalf("malformed knowledge receipt prefix was treated as an ordinary owner event: %v", err)
	}
}

func TestProjectionReplayRecomputesReceiptDigestsAndFullRFC9162OwnerLogRoot(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)
	audit, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, inventory)
	if err != nil {
		t.Fatal(err)
	}
	if !audit.OwnerLogRootMatches || audit.RecomputedOwnerLogRoot == ([32]byte{}) {
		t.Fatalf("owner log root not recomputed: %+v", audit)
	}
	if audit.NonKnowledgeReceiptLeafCount != 1 {
		t.Fatalf("ordinary non-receipt log leaf was omitted: %+v", audit)
	}

	staleDigest := inventory
	staleDigest.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	staleDigest.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	staleDigest.LogLeaves[0].EntryBytes = append([]byte(nil), inventory.LogLeaves[0].EntryBytes...)
	staleDigest.LogLeaves[0].EntryBytes[len(staleDigest.LogLeaves[0].EntryBytes)-1] ^= 0x01
	staleDigest.Receipts[0].CanonicalReceiptEntryBytes = append([]byte(nil), staleDigest.LogLeaves[0].EntryBytes...)
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, staleDigest); err != ErrProjectionReceiptClass {
		t.Fatalf("mutated canonical receipt with stale digest escaped: %v", err)
	}

	changedOrdinaryEvent := inventory
	changedOrdinaryEvent.LogLeaves = append([]OwnerLogLeafV1(nil), inventory.LogLeaves...)
	changedOrdinaryEvent.LogLeaves[1].EntryBytes = []byte("MYCELIX-TEST-OWNER-LOG:owner:grants:tampered")
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, changedOrdinaryEvent); err != ErrProjectionInventoryMismatch {
		t.Fatalf("mutated non-receipt log leaf escaped full root check: %v", err)
	}
}


func TestProjectionReplayRejectsMissingOrTamperedAssertionPayloadBytes(t *testing.T) {
	scope := intervalScopeFixtureV1()
	binding, inventory := projectionInventoryFixtureV1(scope)

	missing := inventory
	missing.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	missing.Receipts[0].AssertionPayloadBytes = nil
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, missing); err != ErrProjectionReceiptClass {
		t.Fatalf("missing assertion payload was accepted: %v", err)
	}

	tampered := inventory
	tampered.Receipts = append([]AcceptedReceiptProjectionV1(nil), inventory.Receipts...)
	tampered.Receipts[0].AssertionPayloadBytes = append([]byte(nil), inventory.Receipts[0].AssertionPayloadBytes...)
	tampered.Receipts[0].AssertionPayloadBytes[len(tampered.Receipts[0].AssertionPayloadBytes)-1] ^= 0x01
	if _, err := ComputeOwnerProjectionReplayAuditV1(scope, binding, tampered); err != ErrProjectionReceiptClass {
		t.Fatalf("assertion payload bytes with stale assertion digest passed: %v", err)
	}
}
