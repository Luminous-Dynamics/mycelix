package sagaverifier

import (
	"os"
	"strconv"
	"strings"
	"testing"
)

func outboxTestPolicy() BackoffPolicy {
	return BackoffPolicy{
		BaseDelayMS: 100, MaxDelayMS: 1000,
		JitterWindowMS: 25, MaxDispatchAttempts: 5,
		MaxReconciliationAttempts: 5, MaxReconciliationAgeMS: 60000,
	}
}

func outboxTestEffect() NewOutboxEffect {
	return NewOutboxEffect{
		OperationID: "op-001", EffectID: "effect:treasury-fee:op-001",
		DestinationID: "treasury:global",
		PayloadDigest: "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
		CreatedAtMS: 1000,
	}
}

func outboxTestReceipt(effect NewOutboxEffect) OutboxReceipt {
	return OutboxReceipt{
		OperationID: effect.OperationID, EffectID: effect.EffectID,
		DestinationID: effect.DestinationID, PayloadDigest: effect.PayloadDigest,
		ReceiptDigest: "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
	}
}

func ptrOutboxReceipt(value OutboxReceipt) *OutboxReceipt { return &value }

func addOutboxTestEffect(t *testing.T, book *OutboxBook) NewOutboxEffect {
	t.Helper()
	effect := outboxTestEffect()
	disposition, err := book.Enqueue(effect)
	if err != nil || disposition != OutboxInserted {
		t.Fatalf("enqueue = %v, %v", disposition, err)
	}
	return effect
}

func authorizeOutboxDispatch(t *testing.T, book *OutboxBook, now uint64) OutboxLease {
	t.Helper()
	leases, err := book.ClaimDue("worker-a", now, 100, 1, outboxTestPolicy())
	if err != nil || len(leases) != 1 {
		t.Fatalf("claim preflight = %v, %v", leases, err)
	}
	lease := leases[0]
	err = book.RecordPreflight(lease, OutboxQueryResult{
		Kind: OutboxQueryAuthoritativeAbsence,
		ProofDigest: "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
	}, now+1, outboxTestPolicy())
	if err != nil {
		t.Fatalf("record authoritative absence = %v", err)
	}
	return lease
}

func TestOutboxExactEnqueueIsIdempotentAndImmutable(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	disposition, err := book.Enqueue(effect)
	if err != nil || disposition != OutboxExistingSameEffect {
		t.Fatalf("exact enqueue = %v, %v", disposition, err)
	}
	changed := effect
	changed.PayloadDigest = "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"
	if _, err := book.Enqueue(changed); err != ErrOutboxImmutableConflict {
		t.Fatalf("changed payload error = %v, want immutable conflict", err)
	}
}

func TestOutboxDispatchRequiresPreflightAndLeaseIsSingleClaim(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	leases, err := book.ClaimDue("worker-a", 1000, 100, 1, outboxTestPolicy())
	if err != nil || len(leases) != 1 {
		t.Fatalf("claim = %v, %v", leases, err)
	}
	lease := leases[0]
	second, err := book.ClaimDue("worker-b", 1000, 100, 1, outboxTestPolicy())
	if err != nil || len(second) != 0 {
		t.Fatalf("second worker claimed already-leased item: %v, %v", second, err)
	}
	err = book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchDefinitelyNoEffectTransient, ErrorCode: "timeout",
	}, 1001, outboxTestPolicy())
	if err != ErrOutboxLeaseMismatch {
		t.Fatalf("dispatch before preflight error = %v", err)
	}
	if err := book.RecordPreflight(lease, OutboxQueryResult{
		Kind: OutboxQueryAuthoritativeAbsence,
		ProofDigest: "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
	}, 1001, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchCommitted, Receipt: ptrOutboxReceipt(outboxTestReceipt(effect)),
	}, 1002, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	item, ok := book.Get(effect.EffectID)
	if !ok || item.Status != OutboxDelivered {
		t.Fatalf("final status = %v, found=%v", item.Status, ok)
	}
	if !item.HasLastAttempt || item.LastAttemptAtMS != 1002 {
		t.Fatalf("last attempt = %d (present=%v), want 1002", item.LastAttemptAtMS, item.HasLastAttempt)
	}
}

func TestOutboxExpiredPreflightLeaseCanBeReclaimedButStaleLeaseCannotCommit(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	leases, err := book.ClaimDue("worker-a", 1000, 10, 1, outboxTestPolicy())
	if err != nil || len(leases) != 1 {
		t.Fatalf("first claim = %v, %v", leases, err)
	}
	old := leases[0]
	if got := book.ExpireLeases(1010); got != 1 {
		t.Fatalf("expired leases = %d, want 1", got)
	}
	newLeases, err := book.ClaimDue("worker-b", 1010, 100, 1, outboxTestPolicy())
	if err != nil || len(newLeases) != 1 {
		t.Fatalf("reclaimed preflight = %v, %v", newLeases, err)
	}
	if newLeases[0].Generation <= old.Generation {
		t.Fatalf("lease generation did not advance: old=%d new=%d", old.Generation, newLeases[0].Generation)
	}
	if err := book.RecordPreflight(old, OutboxQueryResult{
		Kind: OutboxQueryAuthoritativeAbsence,
		ProofDigest: "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
	}, 1011, outboxTestPolicy()); err != ErrOutboxLeaseMismatch {
		t.Fatalf("stale lease error = %v", err)
	}
}

func TestOutboxExpiredDispatchAuthorizationRequiresReconciliation(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	leases, err := book.ClaimDue("worker-a", 1000, 10, 1, outboxTestPolicy())
	if err != nil || len(leases) != 1 {
		t.Fatalf("claim = %v, %v", leases, err)
	}
	lease := leases[0]
	if err := book.RecordPreflight(lease, OutboxQueryResult{
		Kind: OutboxQueryAuthoritativeAbsence,
		ProofDigest: "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
	}, 1001, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	if got := book.ExpireLeases(1010); got != 1 {
		t.Fatalf("expired dispatch lease = %d, want 1", got)
	}
	item, _ := book.Get(effect.EffectID)
	if item.Status != OutboxAwaitingReconciliation {
		t.Fatalf("expired dispatch status = %v, want awaiting reconciliation", item.Status)
	}
	if claimed, err := book.ClaimDue("worker-b", 2000, 100, 1, outboxTestPolicy()); err != nil || len(claimed) != 0 {
		t.Fatalf("dispatch claims while ambiguous = %v, %v", claimed, err)
	}
	reconcile, err := book.ClaimReconciliation("worker-b", 1010, 100, 1, outboxTestPolicy())
	if err != nil || len(reconcile) != 1 {
		t.Fatalf("reconciliation claim = %v, %v", reconcile, err)
	}
	if err := book.RecordReconciliation(reconcile[0], OutboxQueryResult{
		Kind: OutboxQueryUncertain,
	}, 1011, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	if claimed, err := book.ClaimDue("worker-c", 10000, 100, 1, outboxTestPolicy()); err != nil || len(claimed) != 0 {
		t.Fatalf("dispatch claims after uncertain reconcile = %v, %v", claimed, err)
	}
}

func TestOutboxIndeterminateOutcomeMustResolveOriginalStableEffect(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	lease := authorizeOutboxDispatch(t, book, 1000)
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchIndeterminate,
		EvidenceDigest: "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
	}, 1002, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	item, _ := book.Get(effect.EffectID)
	if item.Status != OutboxAwaitingReconciliation {
		t.Fatalf("status = %v, want reconciliation", item.Status)
	}
	if claimed, err := book.ClaimDue("worker-b", 10000, 100, 1, outboxTestPolicy()); err != nil || len(claimed) != 0 {
		t.Fatalf("same effect must not be blindly redispatched: %v, %v", claimed, err)
	}
	reconcile, err := book.ClaimReconciliation("worker-b", item.NextReconciliationAtMS, 100, 1, outboxTestPolicy())
	if err != nil || len(reconcile) != 1 {
		t.Fatalf("claim reconciliation = %v, %v", reconcile, err)
	}
	if err := book.RecordReconciliation(reconcile[0], OutboxQueryResult{
		Kind: OutboxQueryReceipt, Receipt: ptrOutboxReceipt(outboxTestReceipt(effect)),
	}, item.NextReconciliationAtMS+1, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	final, _ := book.Get(effect.EffectID)
	if final.Status != OutboxDelivered {
		t.Fatalf("reconciled status = %v, want delivered", final.Status)
	}
}

func TestOutboxMismatchedDestinationReceiptRequiresManualReview(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	leases, err := book.ClaimDue("worker-a", 1000, 100, 1, outboxTestPolicy())
	if err != nil || len(leases) != 1 {
		t.Fatalf("claim = %v, %v", leases, err)
	}
	mismatch := outboxTestReceipt(effect)
	mismatch.DestinationID = "treasury:other"
	if err := book.RecordPreflight(leases[0], OutboxQueryResult{
		Kind: OutboxQueryReceipt, Receipt: ptrOutboxReceipt(mismatch),
	}, 1001, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	item, _ := book.Get(effect.EffectID)
	if item.Status != OutboxManualReview || item.LastErrorCode != "receipt_subject_mismatch" {
		t.Fatalf("mismatch disposition = %v / %s", item.Status, item.LastErrorCode)
	}
}

func TestOutboxDefinitelyNoEffectTransientUsesBackoffAndSameEffectID(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	lease := authorizeOutboxDispatch(t, book, 1000)
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchDefinitelyNoEffectTransient, ErrorCode: "rate_limited",
	}, 1002, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	item, _ := book.Get(effect.EffectID)
	if item.Status != OutboxReady || item.NextAttemptAtMS != 1107 {
		t.Fatalf("retry state/time = %v/%d, want ready/1107", item.Status, item.NextAttemptAtMS)
	}
	if due, err := book.ClaimDue("worker-b", 1106, 100, 1, outboxTestPolicy()); err != nil || len(due) != 0 {
		t.Fatalf("claimed before due = %v, %v", due, err)
	}
	next, err := book.ClaimDue("worker-b", 1107, 100, 1, outboxTestPolicy())
	if err != nil || len(next) != 1 || next[0].EffectID != effect.EffectID {
		t.Fatalf("retry identity = %v, %v", next, err)
	}
}

func TestOutboxPermanentRejectionDeadLetters(t *testing.T) {
	book := NewOutboxBook()
	lease := authorizeOutboxDispatch(t, book, 1000)
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchPermanentRejection, ErrorCode: "invalid_destination",
	}, 1002, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	item, _ := book.Get(lease.EffectID)
	if item.Status != OutboxDeadLetter {
		t.Fatalf("status = %v, want dead letter", item.Status)
	}
}

func TestOutboxBackoffMatchesSharedKnownAnswer(t *testing.T) {
	data, err := os.ReadFile("fixtures/sap-treasury-outbox-recovery-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	vectors := make(map[string]string)
	for _, line := range strings.Split(string(data), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		key, value, ok := strings.Cut(line, "=")
		if !ok {
			t.Fatalf("malformed KAT line %q", line)
		}
		vectors[key] = value
	}
	attempt64, err := strconv.ParseUint(vectors["attempt"], 10, 32)
	if err != nil {
		t.Fatal(err)
	}
	delay, err := OutboxBackoffDelayMS(vectors["effect-id"], uint32(attempt64), outboxTestPolicy())
	if err != nil {
		t.Fatal(err)
	}
	if strconv.FormatUint(delay, 10) != vectors["expected-delay-ms"] {
		t.Fatalf("backoff = %d, want %s", delay, vectors["expected-delay-ms"])
	}
}

func TestOutboxReconciliationAttemptBudgetStopsBlindRetry(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	policy := outboxTestPolicy()
	policy.MaxReconciliationAttempts = 1

	lease := authorizeOutboxDispatch(t, book, 1000)
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchIndeterminate,
		EvidenceDigest: "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
	}, 1002, policy); err != nil {
		t.Fatal(err)
	}
	item, _ := book.Get(effect.EffectID)
	due := item.NextReconciliationAtMS
	reconcile, err := book.ClaimReconciliation("worker-b", due, 100, 1, policy)
	if err != nil || len(reconcile) != 1 {
		t.Fatalf("first reconciliation = %v, %v", reconcile, err)
	}
	if err := book.RecordReconciliation(reconcile[0], OutboxQueryResult{
		Kind: OutboxQueryUncertain,
	}, due+1, policy); err != nil {
		t.Fatal(err)
	}
	item, _ = book.Get(effect.EffectID)
	if item.Status != OutboxManualReview || item.LastErrorCode != "reconciliation_budget_exhausted" {
		t.Fatalf("budget disposition = %v / %s", item.Status, item.LastErrorCode)
	}
	if claimed, err := book.ClaimDue("worker-c", due+10000, 100, 1, policy); err != nil || len(claimed) != 0 {
		t.Fatalf("exhausted reconciliation must not dispatch: %v, %v", claimed, err)
	}
}

func TestOutboxReconciliationAgeBudgetMovesEffectToManualReview(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	policy := outboxTestPolicy()
	policy.MaxReconciliationAgeMS = 10

	lease := authorizeOutboxDispatch(t, book, 1000)
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchIndeterminate,
		EvidenceDigest: "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
	}, 1002, policy); err != nil {
		t.Fatal(err)
	}
	item, _ := book.Get(effect.EffectID)
	due := item.NextReconciliationAtMS
	claimed, err := book.ClaimReconciliation("worker-b", due, 100, 1, policy)
	if err != nil || len(claimed) != 0 {
		t.Fatalf("expired reconciliation budget unexpectedly claimed: %v, %v", claimed, err)
	}
	item, _ = book.Get(effect.EffectID)
	if item.Status != OutboxManualReview || item.LastErrorCode != "reconciliation_budget_exhausted" {
		t.Fatalf("age budget disposition = %v / %s", item.Status, item.LastErrorCode)
	}
}

func TestOutboxMetricsExposeAgeAndDispositionCounts(t *testing.T) {
	book := NewOutboxBook()
	effect := addOutboxTestEffect(t, book)
	metrics := book.Metrics(1500)
	if metrics.Ready != 1 || metrics.OldestUnresolvedAgeMS != 500 {
		t.Fatalf("metrics = %+v", metrics)
	}
	lease := authorizeOutboxDispatch(t, book, 1500)
	if err := book.RecordDispatch(lease, OutboxDispatchResult{
		Kind: OutboxDispatchCommitted, Receipt: ptrOutboxReceipt(outboxTestReceipt(effect)),
	}, 1502, outboxTestPolicy()); err != nil {
		t.Fatal(err)
	}
	metrics = book.Metrics(1600)
	if metrics.Delivered != 1 || metrics.Ready != 0 {
		t.Fatalf("delivered metrics = %+v", metrics)
	}
}
