package oraclereceipts

import (
	"bytes"
	"strings"
	"testing"
)

const (
	testDigestA = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
	testDigestB = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
	testDigestC = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"
	testDigestD = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
)

func testScope() OracleScopeV1 {
	return OracleScopeV1{
		SourceRef: "source:registry",
		SnapshotRef: "snapshot:77",
		RegistryRef: "registry:main",
		RegistryEpoch: "epoch:77",
		QueryProfileRef: "query-profile:v1",
		QueryDigest: testDigestA,
		InputCommitmentDigest: testDigestB,
		EvidenceFrontierDigest: testDigestC,
	}
}

func testReceipt(t testing.TB, scope OracleScopeV1, sequence uint64, receiptID, previous string, disposition AnswerDispositionV1, payload string) OracleReceiptV1 {
	t.Helper()
	to := int64(20_000_000_000)
	receipt := OracleReceiptV1{
		Sequence: sequence,
		ReceiptID: receiptID,
		PreviousReceiptDigest: previous,
		OracleRef: "oracle:expert-1",
		KeyEpoch: "key-epoch:4",
		AuthorityGrantRef: "grant:licence-4",
		AuthorityScopeDigest: testDigestA,
		QueryDigest: scope.QueryDigest,
		InputCommitmentDigest: scope.InputCommitmentDigest,
		EvidenceFrontierDigest: scope.EvidenceFrontierDigest,
		AnswerSchemaRef: "answer-schema:legal-assessment-v1",
		AnswerDisposition: disposition,
		AnswerPayload: []byte(payload),
		AnswerDigest: AnswerDigestV1([]byte(payload)),
		EvidenceReferences: []EvidenceReferenceV1{
			{Reference: "evidence:case-file-1", Kind: "case-record", Digest: testDigestB},
		},
		RecordedAtUnixNanos: int64(sequence) * 100_000_000_000,
		ValidTime: ValidTimeV1{FromUnixNanos: 10_000_000_000, ToUnixNanos: &to},
		SignatureScheme: "ed25519",
		SignatureBytes: []byte("structural-test-signature"),
	}
	var err error
	receipt.SignedMessageDigest, err = SignableDigestV1(scope, receipt)
	if err != nil {
		t.Fatal(err)
	}
	receipt.ReceiptDigest, err = ReceiptDigestV1(scope, receipt)
	if err != nil {
		t.Fatal(err)
	}
	return receipt
}

func testGenesis(t testing.TB, scope OracleScopeV1) string {
	t.Helper()
	digest, err := ScopeDigestV1(scope)
	if err != nil {
		t.Fatal(err)
	}
	return genesisDigest(digest)
}

func appendReceipt(t testing.TB, scope OracleScopeV1, receipts []OracleReceiptV1, receiptID string, disposition AnswerDispositionV1, payload string) []OracleReceiptV1 {
	t.Helper()
	previous := testGenesis(t, scope)
	if len(receipts) > 0 {
		previous = receipts[len(receipts)-1].ReceiptDigest
	}
	receipt := testReceipt(t, scope, uint64(len(receipts)+1), receiptID, previous, disposition, payload)
	return append(receipts, receipt)
}

func verify(t testing.TB, scope OracleScopeV1, receipts []OracleReceiptV1) ChainResultV1 {
	t.Helper()
	tail := testGenesis(t, scope)
	if len(receipts) > 0 {
		tail = receipts[len(receipts)-1].ReceiptDigest
	}
	return VerifyOracleReceiptChainV1(scope, receipts, uint64(len(receipts)), tail)
}

func TestReceiptChainIsStructuralAndNeverPromotesTruth(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:1", DispositionAffirm, "assessment: satisfied")
	result := verify(t, scope, receipts)
	if result.Disposition != ChainStructurallyValid {
		t.Fatalf("valid structure rejected: %+v", result)
	}
	if result.OracleSignatureAuthentication != NotEvaluated ||
		result.OracleKeyEpochAuthentication != NotEvaluated ||
		result.AuthorityGrantAuthentication != NotEvaluated ||
		result.SourceUniverseCompleteness != NotEvaluated ||
		result.AnswerTruth != NotEvaluated {
		t.Fatalf("structural validation promoted a trust claim: %+v", result)
	}
}

func TestAnswerSubstitutionIsDetected(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:1", DispositionAffirm, "assessment: satisfied")
	receipts[0].AnswerPayload = []byte("assessment: rejected")
	if _, err := ReceiptDigestV1(scope, receipts[0]); err == nil {
		t.Fatal("answer payload substitution retained a valid receipt digest")
	}
	if got := verify(t, scope, receipts); got.Disposition != ChainIndeterminate {
		t.Fatalf("substituted answer accepted: %+v", got)
	}
}

func TestQueryInputAndFrontierSubstitutionFailClosed(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	mutations := []func(*OracleReceiptV1){
		func(r *OracleReceiptV1) { r.QueryDigest = testDigestC },
		func(r *OracleReceiptV1) { r.InputCommitmentDigest = testDigestC },
		func(r *OracleReceiptV1) { r.EvidenceFrontierDigest = testDigestD },
	}
	for i, mutate := range mutations {
		candidate := receipt
		mutate(&candidate)
		if _, err := ReceiptDigestV1(scope, candidate); err == nil {
			t.Fatalf("scope-bound field substitution %d accepted", i)
		}
	}
}

func TestNoResponseAbstainAndUnableToAssessAreDistinct(t *testing.T) {
	scope := testScope()
	cases := []struct {
		name string
		disposition AnswerDispositionV1
		payload string
	}{
		{"no-response", DispositionNoResponse, ""},
		{"abstain", DispositionAbstain, "expert declined to express an opinion"},
		{"unable-to-assess", DispositionUnableToAssess, "required evidence unavailable"},
	}
	digests := make(map[string]string)
	for _, test := range cases {
		t.Run(test.name, func(t *testing.T) {
			receipt := testReceipt(t, scope, 1, "receipt:"+test.name, testGenesis(t, scope), test.disposition, test.payload)
			digests[test.name] = receipt.ReceiptDigest
		})
	}
	if digests["no-response"] == digests["abstain"] ||
		digests["no-response"] == digests["unable-to-assess"] ||
		digests["abstain"] == digests["unable-to-assess"] {
		t.Fatal("different epistemic dispositions collapsed into the same receipt")
	}
}

func TestNoResponseCannotCarryAnswerPayload(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionNoResponse, "")
	receipt.AnswerPayload = []byte("invented answer")
	receipt.AnswerDigest = AnswerDigestV1(receipt.AnswerPayload)
	if _, err := SignableDigestV1(scope, receipt); err == nil {
		t.Fatal("NoResponse with an answer payload accepted")
	}
}

func TestAffirmRejectAbstainAndUnableToAssessRequirePayload(t *testing.T) {
	scope := testScope()
	for _, disposition := range []AnswerDispositionV1{DispositionAffirm, DispositionReject, DispositionAbstain, DispositionUnableToAssess} {
		receipt := testReceipt(t, scope, 1, "receipt:"+string(disposition), testGenesis(t, scope), disposition, "reason")
		receipt.AnswerPayload = nil
		receipt.AnswerDigest = AnswerDigestV1(nil)
		if _, err := SignableDigestV1(scope, receipt); err == nil {
			t.Fatalf("%s without payload accepted", disposition)
		}
	}
}

func TestBackdatedCorrectionAppendsAndPreservesPriorReceipt(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:original", DispositionAffirm, "valid from 2025-01-01")
	originalDigest := receipts[0].ReceiptDigest
	correction := testReceipt(t, scope, 2, "receipt:correction", receipts[0].ReceiptDigest, DispositionReject, "valid from 2024-12-01")
	correction.RecordedAtUnixNanos = 300_000_000_000
	backdated := int64(5_000_000_000)
	correction.ValidTime = ValidTimeV1{FromUnixNanos: backdated, OpenEnded: true}
	correction.SupersedesReceiptID = receipts[0].ReceiptID
	correction.SignedMessageDigest, _ = SignableDigestV1(scope, correction)
	correction.ReceiptDigest, _ = ReceiptDigestV1(scope, correction)
	receipts = append(receipts, correction)
	if receipts[0].ReceiptDigest != originalDigest {
		t.Fatal("backdated correction mutated its predecessor")
	}
	result := verify(t, scope, receipts)
	if result.Disposition != ChainStructurallyValid {
		t.Fatalf("append-only backdated correction rejected: %+v", result)
	}
}

func TestSupersessionMustReferenceAnEarlierReceipt(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:1", DispositionAffirm, "first")
	second := testReceipt(t, scope, 2, "receipt:2", receipts[0].ReceiptDigest, DispositionReject, "second")
	second.SupersedesReceiptID = "receipt:not-present"
	second.SignedMessageDigest, _ = SignableDigestV1(scope, second)
	second.ReceiptDigest, _ = ReceiptDigestV1(scope, second)
	receipts = append(receipts, second)
	if got := verify(t, scope, receipts); got.Disposition != ChainIndeterminate {
		t.Fatalf("unknown supersession target accepted: %+v", got)
	}
}

func TestConflictingOracleAnswersRemainPlural(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:affirm", DispositionAffirm, "assessment: yes")
	receipts = appendReceipt(t, scope, receipts, "receipt:reject", DispositionReject, "assessment: no")
	result := verify(t, scope, receipts)
	if result.Disposition != ChainStructurallyValid || result.ObservedReceiptCount != 2 {
		t.Fatalf("conflicting answers were not retained: %+v", result)
	}
	if receipts[0].AnswerDisposition == receipts[1].AnswerDisposition {
		t.Fatal("test requires conflicting dispositions")
	}
}

func TestEvidenceReferenceOrderingIsCanonical(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	receipt.EvidenceReferences = []EvidenceReferenceV1{
		{Reference: "evidence:B", Kind: "document", Digest: testDigestB},
		{Reference: "evidence:A", Kind: "document", Digest: testDigestA},
	}
	first, err := SignableDigestV1(scope, receipt)
	if err != nil { t.Fatal(err) }
	receipt.EvidenceReferences[0], receipt.EvidenceReferences[1] = receipt.EvidenceReferences[1], receipt.EvidenceReferences[0]
	second, err := SignableDigestV1(scope, receipt)
	if err != nil { t.Fatal(err) }
	if first != second { t.Fatal("evidence reference order changed signable digest") }
}

func TestDuplicateEvidenceReferencesRejected(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	receipt.EvidenceReferences = append(receipt.EvidenceReferences, receipt.EvidenceReferences[0])
	if _, err := SignableDigestV1(scope, receipt); err == nil {
		t.Fatal("duplicate evidence references accepted")
	}
}

func TestSignatureEnvelopeMustCommitToExactAnswer(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	receipt.SignatureBytes = []byte("different-signature")
	if _, err := ReceiptDigestV1(scope, receipt); err != nil {
		t.Fatalf("signature envelope content may differ structurally: %v", err)
	}
	receipt.SignedMessageDigest = testDigestC
	if _, err := ReceiptDigestV1(scope, receipt); err == nil {
		t.Fatal("signature message digest not bound to canonical receipt")
	}
}

func TestSignatureBytesAreBoundIntoReceiptDigest(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	first := receipt.ReceiptDigest
	receipt.SignatureBytes = []byte("other-signature")
	receipt.ReceiptDigest, _ = ReceiptDigestV1(scope, receipt)
	if first == receipt.ReceiptDigest {
		t.Fatal("signature byte substitution did not change receipt digest")
	}
}

func TestValidTimeUsesExplicitHalfOpenAndOpenEndSemantics(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	to := receipt.ValidTime.FromUnixNanos
	receipt.ValidTime.ToUnixNanos = &to
	if _, err := SignableDigestV1(scope, receipt); err == nil {
		t.Fatal("empty interval accepted")
	}
	receipt.ValidTime = ValidTimeV1{FromUnixNanos: 10_000_000_000, OpenEnded: true}
	if _, err := SignableDigestV1(scope, receipt); err != nil {
		t.Fatalf("explicit open-ended interval rejected: %v", err)
	}
}

func TestDuplicateReceiptIDsAndSequenceGapsRejected(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:1", DispositionAffirm, "one")
	receipts = appendReceipt(t, scope, receipts, "receipt:1", DispositionReject, "two")
	if got := verify(t, scope, receipts); got.Disposition != ChainIndeterminate {
		t.Fatalf("duplicate receipt ID accepted: %+v", got)
	}
	receipts[1].ReceiptID = "receipt:2"
	receipts[1].Sequence = 3
	receipts[1].PreviousReceiptDigest = receipts[0].ReceiptDigest
	receipts[1].SignedMessageDigest, _ = SignableDigestV1(scope, receipts[1])
	receipts[1].ReceiptDigest, _ = ReceiptDigestV1(scope, receipts[1])
	if got := verify(t, scope, receipts); got.Disposition != ChainIndeterminate {
		t.Fatalf("sequence gap accepted: %+v", got)
	}
}

func TestReceiptChainTailAndCountAreBound(t *testing.T) {
	scope := testScope()
	receipts := appendReceipt(t, scope, nil, "receipt:1", DispositionAffirm, "one")
	tail := testGenesis(t, scope)
	wrongTail := VerifyOracleReceiptChainV1(scope, receipts, 1, tail)
	wrongCount := VerifyOracleReceiptChainV1(scope, receipts, 2, receipts[0].ReceiptDigest)
	if wrongTail.Disposition != ChainIndeterminate || wrongCount.Disposition != ChainIndeterminate {
		t.Fatal("wrong tail or count accepted")
	}
}

func TestCanonicalEvidenceReferenceSortDoesNotMutateCallerSlice(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	receipt.EvidenceReferences = []EvidenceReferenceV1{
		{Reference: "evidence:B", Kind: "document", Digest: testDigestB},
		{Reference: "evidence:A", Kind: "document", Digest: testDigestA},
	}
	before := append([]EvidenceReferenceV1(nil), receipt.EvidenceReferences...)
	if _, err := SignableDigestV1(scope, receipt); err != nil { t.Fatal(err) }
	if !bytes.Equal([]byte(receipt.EvidenceReferences[0].Reference), []byte(before[0].Reference)) {
		t.Fatal("canonicalization mutated caller-owned evidence references")
	}
}

func TestFailureCertificatesBindSubmittedReceiptsAndExpectedTail(t *testing.T) {
	scope := testScope()
	first := appendReceipt(t, scope, nil, "receipt:first", DispositionAffirm, "answer:first")
	second := appendReceipt(t, scope, nil, "receipt:second", DispositionAffirm, "answer:second")
	wrongTailA := VerifyOracleReceiptChainV1(scope, first, 1, testDigestA)
	wrongTailB := VerifyOracleReceiptChainV1(scope, second, 1, testDigestA)
	if wrongTailA.Disposition != ChainIndeterminate || wrongTailB.Disposition != ChainIndeterminate {
		t.Fatal("wrong tail unexpectedly accepted")
	}
	if wrongTailA.SubmittedReceiptsDigest == wrongTailB.SubmittedReceiptsDigest ||
		wrongTailA.CertificateDigest == wrongTailB.CertificateDigest {
		t.Fatal("different submitted receipts collapsed into one failure certificate")
	}
	otherTail := VerifyOracleReceiptChainV1(scope, first, 1, testDigestB)
	if wrongTailA.ExpectedTailDigest == otherTail.ExpectedTailDigest ||
		wrongTailA.CertificateDigest == otherTail.CertificateDigest {
		t.Fatal("expected tail substitution was not bound into failure certificate")
	}
}

func TestReceiptEncodingPreflightRejectsOversizedFields(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:one", testGenesis(t, scope), DispositionAffirm, "answer")
	receipt.ReceiptID = strings.Repeat("r", MaxIdentifierBytes+1)
	result := VerifyOracleReceiptChainV1(scope, []OracleReceiptV1{receipt}, 1, receipt.ReceiptDigest)
	if result.Disposition != ChainIndeterminate ||
		!strings.Contains(result.Reason, "receipt_id") ||
		result.SubmittedReceiptsDigest == "" {
		t.Fatalf("oversized receipt field did not fail in bounded preflight: %+v", result)
	}
}

func TestPreEpochValidTimeIsRepresentable(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:pre-epoch", testGenesis(t, scope), DispositionAffirm, "historical assessment")
	end := int64(0)
	receipt.ValidTime = ValidTimeV1{FromUnixNanos: -10_000_000_000, ToUnixNanos: &end}
	if _, err := SignableDigestV1(scope, receipt); err != nil {
		t.Fatalf("pre-epoch valid time was rejected: %v", err)
	}
}

func TestReceiptDigestDeterministic(t *testing.T) {
	scope := testScope()
	receipt := testReceipt(t, scope, 1, "receipt:1", testGenesis(t, scope), DispositionAffirm, "answer")
	for i:=0;i<100;i++ {
		next, err := ReceiptDigestV1(scope, receipt)
		if err != nil { t.Fatal(err) }
		if next != receipt.ReceiptDigest { t.Fatal("receipt digest is nondeterministic") }
	}
}
