package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

func exposureTestDigest(value byte) string {
	const hex = "0123456789abcdef"
	result := make([]byte, 71)
	copy(result, "sha256:")
	for i := 7; i < len(result); i++ {
		result[i] = hex[value%16]
	}
	return string(result)
}

func exposureTestScope() ExposureScopeV1 {
	return ExposureScopeV1{
		OwnerID:       "owner:coop-a",
		PoolID:        "pool:treasury",
		DestinationID: "dest:provider-a",
		Currency:      "SAP",
	}
}

func exposureTestPolicy(maximum uint64) ExposurePolicyV1 {
	return ExposurePolicyV1{
		Scope:                 exposureTestScope(),
		Sequence:              1,
		MaximumTotalMinor:     maximum,
		MaximumPerEffectMinor: minExposureTest(maximum, 800),
		PolicyDigest:          exposureTestDigest(13),
	}
}

func exposureTestIntent(effectID string, maximum uint64) ExposureIntentV1 {
	return ExposureIntentV1{
		Scope:                exposureTestScope(),
		OperationID:          "op:1",
		EffectID:             effectID,
		IntentDigest:         exposureTestDigest(10),
		PayloadDigest:        exposureTestDigest(11),
		MaximumExposureMinor: maximum,
	}
}

func exposureTestResolution(effectID string, kind ExposureResolutionKind) DestinationResolutionV1 {
	return DestinationResolutionV1{
		Kind:           kind,
		EffectID:       effectID,
		DestinationID:  "dest:provider-a",
		PayloadDigest:  exposureTestDigest(11),
		ProfileDigest:  exposureTestDigest(14),
		EvidenceDigest: exposureTestDigest(15),
	}
}

func TestExposureReservationsAreScopedAndIdempotent(t *testing.T) {
	book, err := NewExposureBookV1(exposureTestPolicy(1_000))
	if err != nil {
		t.Fatal(err)
	}
	first, err := book.Reserve(exposureTestIntent("effect:1", 600))
	if err != nil || first != ExposureWasReserved {
		t.Fatalf("first reserve = %v, %v", first, err)
	}
	second, err := book.Reserve(exposureTestIntent("effect:1", 600))
	if err != nil || second != ExposureAlreadyReserved {
		t.Fatalf("duplicate reserve = %v, %v", second, err)
	}
	if total, _ := book.TotalExposureMinor(); total != 600 {
		t.Fatalf("total exposure = %d, want 600", total)
	}
	if headroom, _ := book.HeadroomMinor(); headroom != 400 {
		t.Fatalf("headroom = %d, want 400", headroom)
	}
	if _, err := book.Reserve(exposureTestIntent("effect:2", 500)); err != ErrExposureLimitExceeded {
		t.Fatalf("over-limit reserve = %v", err)
	}
}

func TestExposureChangedIntentConflictsWithoutChangingReservation(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	changed := exposureTestIntent("effect:1", 601)
	changed.PayloadDigest = exposureTestDigest(12)
	if _, err := book.Reserve(changed); err != ErrExposureReservationConflict {
		t.Fatalf("changed retry = %v", err)
	}
	if total, _ := book.TotalExposureMinor(); total != 600 {
		t.Fatalf("total exposure changed to %d", total)
	}
}

func TestIndeterminateExposureRemainsReserved(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	if err := book.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }
	if err := book.MarkIndeterminate("effect:1"); err != nil { t.Fatal(err) }
	if total, _ := book.TotalExposureMinor(); total != 600 {
		t.Fatalf("indeterminate exposure = %d, want 600", total)
	}
	if _, err := book.Reserve(exposureTestIntent("effect:2", 500)); err != ErrExposureLimitExceeded {
		t.Fatalf("new reserve under unresolved exposure = %v", err)
	}
}

func TestObservedUnsettledRequiresEvidenceAndPersistsReceipt(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	if err := book.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }

	wrongKind := exposureTestResolution("effect:1", ExposureFinalSettlementEvidence)
	if err := book.MarkCommittedUnsettled("effect:1", wrongKind); err != ErrExposureInvalidResolution {
		t.Fatalf("final settlement used as observation evidence: %v", err)
	}
	wrongSubject := exposureTestResolution("effect:1", ExposureObservedUnsettledEffect)
	wrongSubject.PayloadDigest = exposureTestDigest(12)
	if err := book.MarkCommittedUnsettled("effect:1", wrongSubject); err != ErrExposureInvalidResolution {
		t.Fatalf("wrong payload observation accepted: %v", err)
	}
	if status, _ := book.Status("effect:1"); status != ExposureDispatchAuthorized {
		t.Fatalf("invalid observation changed state to %v", status)
	}

	valid := exposureTestResolution("effect:1", ExposureObservedUnsettledEffect)
	if err := book.MarkCommittedUnsettled("effect:1", valid); err != nil { t.Fatal(err) }
	if err := book.MarkCommittedUnsettled("effect:1", valid); err != nil { t.Fatal(err) }
	changed := valid
	changed.EvidenceDigest = exposureTestDigest(2)
	if err := book.MarkCommittedUnsettled("effect:1", changed); err != ErrExposureResolutionConflict {
		t.Fatalf("changed observation receipt did not conflict: %v", err)
	}
	if total, _ := book.TotalExposureMinor(); total != 600 {
		t.Fatalf("observed unsettled exposure = %d", total)
	}
}

func TestSettlementReleasesOnlyMatchingReceiptBoundExposure(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	_ = book.MarkDispatchAuthorized("effect:1")
	_ = book.MarkCommittedUnsettled("effect:1", exposureTestResolution("effect:1", ExposureObservedUnsettledEffect))
	wrong := exposureTestResolution("effect:1", ExposureFinalSettlementEvidence)
	wrong.PayloadDigest = exposureTestDigest(12)
	if err := book.MarkSettled("effect:1", wrong); err != ErrExposureInvalidResolution {
		t.Fatalf("wrong receipt accepted: %v", err)
	}
	if err := book.MarkSettled("effect:1", exposureTestResolution("effect:1", ExposureFinalSettlementEvidence)); err != nil {
		t.Fatal(err)
	}
	if total, _ := book.TotalExposureMinor(); total != 0 {
		t.Fatalf("settled exposure = %d", total)
	}
}

func TestAuthoritativeAbsenceMustBeTypedAndSubjectBound(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	_ = book.MarkDispatchAuthorized("effect:1")
	_ = book.MarkIndeterminate("effect:1")
	wrong := exposureTestResolution("effect:1", ExposureFinalSettlementEvidence)
	if err := book.ReleaseAuthoritativeAbsence("effect:1", wrong); err != ErrExposureInvalidResolution {
		t.Fatalf("settlement receipt used as absence proof: %v", err)
	}
	wrong = exposureTestResolution("effect:1", ExposureAuthoritativeAbsence)
	wrong.DestinationID = "dest:other"
	if err := book.ReleaseAuthoritativeAbsence("effect:1", wrong); err != ErrExposureInvalidResolution {
		t.Fatalf("wrong destination absence accepted: %v", err)
	}
	if err := book.ReleaseAuthoritativeAbsence("effect:1", exposureTestResolution("effect:1", ExposureAuthoritativeAbsence)); err != nil {
		t.Fatal(err)
	}
	if total, _ := book.TotalExposureMinor(); total != 0 {
		t.Fatalf("released exposure = %d", total)
	}
}

func TestLowerPolicyRetainsExistingReservations(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	lower := exposureTestPolicy(500)
	lower.Sequence = 2
	if err := book.ReplacePolicy(lower); err != nil { t.Fatal(err) }
	if disposition, err := book.Reserve(exposureTestIntent("effect:1", 600)); err != nil || disposition != ExposureAlreadyReserved {
		t.Fatalf("exact retry after per-effect limit reduction = %v, %v", disposition, err)
	}
	if total, _ := book.TotalExposureMinor(); total != 600 {
		t.Fatalf("existing reservation was erased: %d", total)
	}
	if headroom, _ := book.HeadroomMinor(); headroom != 0 {
		t.Fatalf("headroom = %d, want 0", headroom)
	}
	if _, err := book.Reserve(exposureTestIntent("effect:2", 100)); err != ErrExposureLimitExceeded {
		t.Fatalf("new reserve after limit reduction = %v", err)
	}
}

func TestCancellationOnlyBeforeDispatch(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	if err := book.CancelBeforeDispatch("effect:1"); err != nil { t.Fatal(err) }
	if total, _ := book.TotalExposureMinor(); total != 0 {
		t.Fatalf("cancelled reservation = %d", total)
	}
	if err := book.MarkDispatchAuthorized("effect:1"); err != ErrExposureInvalidTransition {
		t.Fatalf("cancelled reservation dispatched: %v", err)
	}
}

func TestExposureSnapshotMatchesSharedKnownAnswer(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-payment-exposure-v1.kat")
	if err != nil { t.Fatal(err) }
	var expected string
	for _, line := range strings.Split(string(raw), "\n") {
		if strings.HasPrefix(line, "snapshot_sha256=") {
			expected = strings.TrimPrefix(line, "snapshot_sha256=")
			break
		}
	}
	if expected == "" { t.Fatal("missing exposure snapshot known answer") }
	book, err := NewExposureBookV1(exposureTestPolicy(1_000))
	if err != nil { t.Fatal(err) }
	intent := exposureTestIntent("effect:1", 600)
	if _, err := book.Reserve(intent); err != nil { t.Fatal(err) }
	got, err := book.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	const want = "sha256:" + expected
	if got != want { t.Fatalf("snapshot digest = %s, want %s", got, want) }
}

func TestSnapshotOrderingIsIndependentOfInsertionOrder(t *testing.T) {
	first, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	second, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = first.Reserve(exposureTestIntent("effect:1", 100))
	_, _ = first.Reserve(exposureTestIntent("effect:2", 200))
	_, _ = second.Reserve(exposureTestIntent("effect:2", 200))
	_, _ = second.Reserve(exposureTestIntent("effect:1", 100))
	firstDigest, err := first.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	secondDigest, err := second.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	if firstDigest != secondDigest { t.Fatalf("insertion order changed snapshot: %s != %s", firstDigest, secondDigest) }
}

func minExposureTest(left, right uint64) uint64 {
	if left < right { return left }
	return right
}

func TestExposureScopeMustMatchPolicyExactly(t *testing.T) {
	book, err := NewExposureBookV1(exposureTestPolicy(1_000))
	if err != nil { t.Fatal(err) }
	wrongOwner := exposureTestIntent("effect:1", 100)
	wrongOwner.Scope.OwnerID = "owner:other"
	if _, err := book.Reserve(wrongOwner); err != ErrExposureScopeMismatch {
		t.Fatalf("wrong owner reserve = %v", err)
	}
	wrongCurrency := exposureTestIntent("effect:2", 100)
	wrongCurrency.Scope.Currency = "USD"
	if _, err := book.Reserve(wrongCurrency); err != ErrExposureScopeMismatch {
		t.Fatalf("cross-currency reserve = %v", err)
	}
}

func TestInvalidUTF8IsRejectedForCrossLanguageCanonicalParity(t *testing.T) {
	scope := exposureTestScope()
	scope.OwnerID = string([]byte{0xff})
	policy := exposureTestPolicy(1_000)
	policy.Scope = scope
	if _, err := NewExposureBookV1(policy); err != ErrExposureInvalidText {
		t.Fatalf("invalid UTF-8 owner accepted: %v", err)
	}
}

func TestDispatchAuthorizationIsIdempotent(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(1_000))
	_, _ = book.Reserve(exposureTestIntent("effect:1", 600))
	if err := book.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }
	if err := book.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }
	status, ok := book.Status("effect:1")
	if !ok || status != ExposureDispatchAuthorized { t.Fatalf("status = %v, %v", status, ok) }
	if total, _ := book.TotalExposureMinor(); total != 600 { t.Fatalf("total = %d", total) }
}
func TestExposureBreakdownTracksEachAtRiskState(t *testing.T) {
	book, _ := NewExposureBookV1(exposureTestPolicy(2_000))
	_, _ = book.Reserve(exposureTestIntent("reserved", 100))
	_, _ = book.Reserve(exposureTestIntent("dispatch", 200))
	_, _ = book.Reserve(exposureTestIntent("indeterminate", 300))
	_, _ = book.Reserve(exposureTestIntent("committed", 400))
	if err := book.MarkDispatchAuthorized("dispatch"); err != nil {
		t.Fatal(err)
	}
	if err := book.MarkDispatchAuthorized("indeterminate"); err != nil {
		t.Fatal(err)
	}
	if err := book.MarkIndeterminate("indeterminate"); err != nil {
		t.Fatal(err)
	}
	if err := book.MarkDispatchAuthorized("committed"); err != nil {
		t.Fatal(err)
	}
	if err := book.MarkCommittedUnsettled("committed"); err != nil {
		t.Fatal(err)
	}
	got, err := book.BreakdownMinor()
	if err != nil {
		t.Fatal(err)
	}
	want := ExposureBreakdownV1{
		ReservedMinor:           100,
		DispatchAuthorizedMinor: 200,
		IndeterminateMinor:      300,
		CommittedUnsettledMinor: 400,
	}
	if got != want {
		t.Fatalf("breakdown = %+v, want %+v", got, want)
	}
	if total, err := got.TotalMinor(); err != nil || total != 1_000 {
		t.Fatalf("breakdown total = %d, %v; want 1000", total, err)
	}
}

