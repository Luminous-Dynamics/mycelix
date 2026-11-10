package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"sort"
	"strings"
	"unicode/utf8"
)

const (
	outboxBackoffDomain = "MXSAP-OUTBOX-BACKOFF-V1\x00"
	outboxMaxBatch      = 64
	outboxMaxTextBytes  = 4096
	outboxMaxErrorBytes = 128
)

type OutboxStatus uint8

const (
	OutboxReady OutboxStatus = iota + 1
	OutboxLeasedForPreflight
	OutboxDispatchAuthorized
	OutboxAwaitingReconciliation
	OutboxLeasedForReconciliation
	OutboxDelivered
	OutboxDeadLetter
	OutboxManualReview
)

var (
	ErrOutboxInvalidInput       = errors.New("outbox rejected: invalid input")
	ErrOutboxInvalidPolicy      = errors.New("outbox rejected: invalid backoff policy")
	ErrOutboxBatchLimit         = errors.New("outbox rejected: batch limit exceeded")
	ErrOutboxImmutableConflict  = errors.New("outbox rejected: immutable effect conflict")
	ErrOutboxUnknownEffect      = errors.New("outbox rejected: unknown effect")
	ErrOutboxLeaseMismatch      = errors.New("outbox rejected: lease mismatch")
	ErrOutboxLeaseExpired       = errors.New("outbox rejected: lease expired")
	ErrOutboxInvalidTime        = errors.New("outbox rejected: time overflow")
	ErrOutboxCounterOverflow    = errors.New("outbox rejected: counter overflow")
)

type NewOutboxEffect struct {
	OperationID   string
	EffectID      string
	DestinationID string
	PayloadDigest string
	CreatedAtMS   uint64
}

type OutboxReceipt struct {
	OperationID   string
	EffectID      string
	DestinationID string
	PayloadDigest string
	ReceiptDigest string
}

type OutboxQueryKind uint8

const (
	OutboxQueryAuthoritativeAbsence OutboxQueryKind = iota + 1
	OutboxQueryReceipt
	OutboxQueryUncertain
)

type OutboxQueryResult struct {
	Kind        OutboxQueryKind
	ProofDigest string
	Receipt     *OutboxReceipt
}

type OutboxDispatchKind uint8

const (
	OutboxDispatchCommitted OutboxDispatchKind = iota + 1
	OutboxDispatchDefinitelyNoEffectTransient
	OutboxDispatchIndeterminate
	OutboxDispatchPermanentRejection
	OutboxDispatchSourceEvidenceInvalid
)

type OutboxDispatchResult struct {
	Kind           OutboxDispatchKind
	Receipt        *OutboxReceipt
	ErrorCode      string
	EvidenceDigest string
}

type BackoffPolicy struct {
	BaseDelayMS               uint64
	MaxDelayMS                uint64
	JitterWindowMS            uint64
	MaxDispatchAttempts       uint32
	MaxReconciliationAttempts uint32
	MaxReconciliationAgeMS    uint64
}

func (p BackoffPolicy) Validate() error {
	if p.BaseDelayMS == 0 || p.MaxDelayMS < p.BaseDelayMS ||
		p.JitterWindowMS >= p.MaxDelayMS || p.MaxDispatchAttempts == 0 ||
		p.MaxReconciliationAttempts == 0 || p.MaxReconciliationAgeMS == 0 {
		return ErrOutboxInvalidPolicy
	}
	return nil
}

type OutboxLease struct {
	EffectID    string
	WorkerID    string
	Generation  uint64
	ExpiresAtMS uint64
}

type outboxLeaseState struct {
	workerID    string
	generation  uint64
	expiresAtMS uint64
}

func leaseState(lease OutboxLease) outboxLeaseState {
	return outboxLeaseState{
		workerID: lease.WorkerID, generation: lease.Generation, expiresAtMS: lease.ExpiresAtMS,
	}
}

type OutboxItem struct {
	OperationID               string
	EffectID                  string
	DestinationID             string
	PayloadDigest             string
	CreatedAtMS               uint64
	NextAttemptAtMS           uint64
	NextReconciliationAtMS    uint64
	DispatchAttempts          uint32
	ReconciliationAttempts    uint32
	ReconciliationStartedAtMS uint64
	HasReconciliationStarted  bool
	LastAttemptAtMS           uint64
	HasLastAttempt            bool
	LeaseGeneration           uint64
	LastErrorCode             string
	LastEvidenceDigest        string
	LastAbsenceProofDigest    string
	Status                    OutboxStatus
	lease                     outboxLeaseState
	hasLease                  bool
	absenceProofDigest        string
	receiptDigest             string
}

type EnqueueDisposition uint8

const (
	OutboxInserted EnqueueDisposition = iota + 1
	OutboxExistingSameEffect
)

type OutboxQueueMetrics struct {
	Ready                   int
	Leased                  int
	AwaitingReconciliation  int
	Delivered               int
	DeadLetter              int
	ManualReview            int
	OldestUnresolvedAgeMS   uint64
}

type OutboxBook struct {
	items map[string]*OutboxItem
}

func NewOutboxBook() *OutboxBook {
	return &OutboxBook{items: make(map[string]*OutboxItem)}
}

func (b *OutboxBook) Enqueue(effect NewOutboxEffect) (EnqueueDisposition, error) {
	if err := validateNewOutboxEffect(effect); err != nil {
		return 0, err
	}
	if b.items == nil {
		b.items = make(map[string]*OutboxItem)
	}
	if existing, ok := b.items[effect.EffectID]; ok {
		if existing.OperationID == effect.OperationID &&
			existing.DestinationID == effect.DestinationID &&
			existing.PayloadDigest == effect.PayloadDigest {
			return OutboxExistingSameEffect, nil
		}
		return 0, ErrOutboxImmutableConflict
	}
	b.items[effect.EffectID] = &OutboxItem{
		OperationID: effect.OperationID, EffectID: effect.EffectID,
		DestinationID: effect.DestinationID, PayloadDigest: effect.PayloadDigest,
		CreatedAtMS: effect.CreatedAtMS, NextAttemptAtMS: effect.CreatedAtMS,
		NextReconciliationAtMS: effect.CreatedAtMS, Status: OutboxReady,
	}
	return OutboxInserted, nil
}

// ClaimDue claims at most 64 ready effects for a destination preflight query.
// It never authorizes or performs the external dispatch itself.
func (b *OutboxBook) ClaimDue(workerID string, nowMS, leaseDurationMS uint64, limit int, policy BackoffPolicy) ([]OutboxLease, error) {
	if err := validateOutboxWorkerLease(workerID, leaseDurationMS, limit, policy); err != nil {
		return nil, err
	}
	b.ExpireLeases(nowMS)
	candidates := make([]*OutboxItem, 0)
	for _, item := range b.items {
		if item.Status == OutboxReady && item.NextAttemptAtMS <= nowMS {
			candidates = append(candidates, item)
		}
	}
	sort.Slice(candidates, func(i, j int) bool {
		a, c := candidates[i], candidates[j]
		if a.NextAttemptAtMS != c.NextAttemptAtMS {
			return a.NextAttemptAtMS < c.NextAttemptAtMS
		}
		if a.CreatedAtMS != c.CreatedAtMS {
			return a.CreatedAtMS < c.CreatedAtMS
		}
		return a.EffectID < c.EffectID
	})
	leases := make([]OutboxLease, 0, limit)
	for i, item := range candidates {
		if i >= limit {
			break
		}
		if item.DispatchAttempts >= policy.MaxDispatchAttempts {
			item.Status = OutboxDeadLetter
			item.LastErrorCode = "dispatch_attempt_limit"
			continue
		}
		lease, err := newOutboxLease(item, workerID, nowMS, leaseDurationMS)
		if err != nil {
			return nil, err
		}
		item.Status = OutboxLeasedForPreflight
		item.lease = leaseState(lease)
		item.hasLease = true
		leases = append(leases, lease)
	}
	return leases, nil
}

// ClaimReconciliation claims unresolved effects for a read-only destination query.
func (b *OutboxBook) ClaimReconciliation(workerID string, nowMS, leaseDurationMS uint64, limit int, policy BackoffPolicy) ([]OutboxLease, error) {
	if err := validateOutboxWorkerLease(workerID, leaseDurationMS, limit, policy); err != nil {
		return nil, err
	}
	b.ExpireLeases(nowMS)
	candidates := make([]*OutboxItem, 0)
	for _, item := range b.items {
		if item.Status == OutboxAwaitingReconciliation && item.NextReconciliationAtMS <= nowMS {
			candidates = append(candidates, item)
		}
	}
	sort.Slice(candidates, func(i, j int) bool {
		a, c := candidates[i], candidates[j]
		if a.NextReconciliationAtMS != c.NextReconciliationAtMS {
			return a.NextReconciliationAtMS < c.NextReconciliationAtMS
		}
		if a.CreatedAtMS != c.CreatedAtMS {
			return a.CreatedAtMS < c.CreatedAtMS
		}
		return a.EffectID < c.EffectID
	})
	leases := make([]OutboxLease, 0, limit)
	for i, item := range candidates {
		if i >= limit {
			break
		}
		if outboxReconciliationBudgetExhausted(item, nowMS, policy) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "reconciliation_budget_exhausted"
			continue
		}
		lease, err := newOutboxLease(item, workerID, nowMS, leaseDurationMS)
		if err != nil {
			return nil, err
		}
		if !item.HasReconciliationStarted {
			item.ReconciliationStartedAtMS = nowMS
			item.HasReconciliationStarted = true
		}
		if item.ReconciliationAttempts == ^uint32(0) {
			return nil, ErrOutboxCounterOverflow
		}
		item.ReconciliationAttempts++
		item.Status = OutboxLeasedForReconciliation
		item.lease = leaseState(lease)
		item.hasLease = true
		leases = append(leases, lease)
	}
	return leases, nil
}

func (b *OutboxBook) RecordPreflight(lease OutboxLease, outcome OutboxQueryResult, nowMS uint64, policy BackoffPolicy) error {
	if err := policy.Validate(); err != nil {
		return err
	}
	item, err := b.requireOutboxLease(lease, OutboxLeasedForPreflight, nowMS)
	if err != nil {
		return err
	}
	item.LastAttemptAtMS = nowMS
	item.HasLastAttempt = true
	switch outcome.Kind {
	case OutboxQueryReceipt:
		if outcome.Receipt == nil {
			item.Status = OutboxManualReview
			item.hasLease = false
			return nil
		}
		applyOutboxReceipt(item, *outcome.Receipt)
	case OutboxQueryAuthoritativeAbsence:
		if !isSHA256Digest(outcome.ProofDigest) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "invalid_absence_proof"
			item.hasLease = false
			return nil
		}
		if item.DispatchAttempts >= policy.MaxDispatchAttempts {
			item.Status = OutboxDeadLetter
			item.LastErrorCode = "dispatch_attempt_limit"
			item.hasLease = false
			return nil
		}
		if item.DispatchAttempts == ^uint32(0) {
			return ErrOutboxCounterOverflow
		}
		item.DispatchAttempts++
		item.LastAbsenceProofDigest = outcome.ProofDigest
		item.absenceProofDigest = outcome.ProofDigest
		item.Status = OutboxDispatchAuthorized
	case OutboxQueryUncertain:
		return b.scheduleOutboxReconciliation(item, nowMS, policy)
	default:
		item.Status = OutboxManualReview
		item.LastErrorCode = "unknown_query_disposition"
		item.hasLease = false
	}
	return nil
}

func (b *OutboxBook) RecordDispatch(lease OutboxLease, outcome OutboxDispatchResult, nowMS uint64, policy BackoffPolicy) error {
	if err := policy.Validate(); err != nil {
		return err
	}
	item, err := b.requireOutboxLease(lease, OutboxDispatchAuthorized, nowMS)
	if err != nil {
		return err
	}
	item.LastAttemptAtMS = nowMS
	item.HasLastAttempt = true
	switch outcome.Kind {
	case OutboxDispatchCommitted:
		if outcome.Receipt == nil {
			item.Status = OutboxManualReview
			item.LastErrorCode = "missing_committed_receipt"
			item.hasLease = false
			return nil
		}
		applyOutboxReceipt(item, *outcome.Receipt)
	case OutboxDispatchDefinitelyNoEffectTransient:
		if !validOutboxErrorCode(outcome.ErrorCode) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "invalid_error_code"
			item.hasLease = false
			return nil
		}
		item.LastErrorCode = outcome.ErrorCode
		item.hasLease = false
		if item.DispatchAttempts >= policy.MaxDispatchAttempts {
			item.Status = OutboxDeadLetter
			return nil
		}
		delay, err := OutboxBackoffDelayMS(item.EffectID, item.DispatchAttempts, policy)
		if err != nil {
			return err
		}
		next, ok := addOutboxTime(nowMS, delay)
		if !ok {
			return ErrOutboxInvalidTime
		}
		item.NextAttemptAtMS = next
		item.Status = OutboxReady
	case OutboxDispatchIndeterminate:
		if !isSHA256Digest(outcome.EvidenceDigest) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "invalid_indeterminate_evidence"
			item.hasLease = false
			return nil
		}
		item.LastEvidenceDigest = outcome.EvidenceDigest
		return b.scheduleOutboxReconciliation(item, nowMS, policy)
	case OutboxDispatchPermanentRejection:
		if !validOutboxErrorCode(outcome.ErrorCode) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "invalid_rejection_code"
		} else {
			item.Status = OutboxDeadLetter
			item.LastErrorCode = outcome.ErrorCode
		}
		item.hasLease = false
	case OutboxDispatchSourceEvidenceInvalid:
		if isSHA256Digest(outcome.EvidenceDigest) {
			item.LastEvidenceDigest = outcome.EvidenceDigest
		}
		item.Status = OutboxManualReview
		item.LastErrorCode = "source_evidence_invalid"
		item.hasLease = false
	default:
		item.Status = OutboxManualReview
		item.LastErrorCode = "unknown_dispatch_disposition"
		item.hasLease = false
	}
	return nil
}

func (b *OutboxBook) RecordReconciliation(lease OutboxLease, outcome OutboxQueryResult, nowMS uint64, policy BackoffPolicy) error {
	if err := policy.Validate(); err != nil {
		return err
	}
	item, err := b.requireOutboxLease(lease, OutboxLeasedForReconciliation, nowMS)
	if err != nil {
		return err
	}
	item.LastAttemptAtMS = nowMS
	item.HasLastAttempt = true
	switch outcome.Kind {
	case OutboxQueryReceipt:
		if outcome.Receipt == nil {
			item.Status = OutboxManualReview
			item.hasLease = false
			return nil
		}
		applyOutboxReceipt(item, *outcome.Receipt)
	case OutboxQueryAuthoritativeAbsence:
		if !isSHA256Digest(outcome.ProofDigest) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "invalid_absence_proof"
			item.hasLease = false
			return nil
		}
		if outboxReconciliationAgeExceeded(item, nowMS, policy) {
			item.Status = OutboxManualReview
			item.LastErrorCode = "reconciliation_budget_exhausted"
			item.hasLease = false
			return nil
		}
		item.LastAbsenceProofDigest = outcome.ProofDigest
		item.NextAttemptAtMS = nowMS
		item.Status = OutboxReady
		item.hasLease = false
	case OutboxQueryUncertain:
		return b.scheduleOutboxReconciliation(item, nowMS, policy)
	default:
		item.Status = OutboxManualReview
		item.LastErrorCode = "unknown_query_disposition"
		item.hasLease = false
	}
	return nil
}

// ExpireLeases re-queues a preflight-only lease, but sends any expired
// dispatch-authorized lease to reconciliation because the effect may have happened.
func (b *OutboxBook) ExpireLeases(nowMS uint64) int {
	expired := 0
	for _, item := range b.items {
		if !item.hasLease || nowMS < item.lease.expiresAtMS {
			continue
		}
		switch item.Status {
		case OutboxLeasedForPreflight:
			item.Status = OutboxReady
			item.NextAttemptAtMS = nowMS
			item.hasLease = false
			expired++
		case OutboxDispatchAuthorized, OutboxLeasedForReconciliation:
			item.Status = OutboxAwaitingReconciliation
			item.NextReconciliationAtMS = nowMS
			item.hasLease = false
			expired++
		}
	}
	return expired
}

func (b *OutboxBook) Get(effectID string) (OutboxItem, bool) {
	item, ok := b.items[effectID]
	if !ok {
		return OutboxItem{}, false
	}
	return *item, true
}

func (b *OutboxBook) Metrics(nowMS uint64) OutboxQueueMetrics {
	var metrics OutboxQueueMetrics
	for _, item := range b.items {
		switch item.Status {
		case OutboxReady:
			metrics.Ready++
		case OutboxLeasedForPreflight, OutboxDispatchAuthorized, OutboxLeasedForReconciliation:
			metrics.Leased++
		case OutboxAwaitingReconciliation:
			metrics.AwaitingReconciliation++
		case OutboxDelivered:
			metrics.Delivered++
		case OutboxDeadLetter:
			metrics.DeadLetter++
		case OutboxManualReview:
			metrics.ManualReview++
		}
		if item.Status != OutboxDelivered {
			age := nowMS - item.CreatedAtMS
			if nowMS < item.CreatedAtMS {
				age = 0
			}
			if age > metrics.OldestUnresolvedAgeMS {
				metrics.OldestUnresolvedAgeMS = age
			}
		}
	}
	return metrics
}

func (b *OutboxBook) requireOutboxLease(lease OutboxLease, status OutboxStatus, nowMS uint64) (*OutboxItem, error) {
	item, ok := b.items[lease.EffectID]
	if !ok {
		return nil, ErrOutboxUnknownEffect
	}
	if item.Status != status || !item.hasLease || item.lease != leaseState(lease) {
		return nil, ErrOutboxLeaseMismatch
	}
	if nowMS >= lease.ExpiresAtMS {
		return nil, ErrOutboxLeaseExpired
	}
	return item, nil
}

func outboxReconciliationAgeExceeded(item *OutboxItem, nowMS uint64, policy BackoffPolicy) bool {
	return item.HasReconciliationStarted &&
		nowMS >= item.ReconciliationStartedAtMS &&
		nowMS-item.ReconciliationStartedAtMS >= policy.MaxReconciliationAgeMS
}

func outboxReconciliationBudgetExhausted(item *OutboxItem, nowMS uint64, policy BackoffPolicy) bool {
	return item.ReconciliationAttempts >= policy.MaxReconciliationAttempts ||
		outboxReconciliationAgeExceeded(item, nowMS, policy)
}

func (b *OutboxBook) scheduleOutboxReconciliation(item *OutboxItem, nowMS uint64, policy BackoffPolicy) error {
	if !item.HasReconciliationStarted {
		item.ReconciliationStartedAtMS = nowMS
		item.HasReconciliationStarted = true
	}
	if item.ReconciliationAttempts >= policy.MaxReconciliationAttempts ||
		outboxReconciliationAgeExceeded(item, nowMS, policy) {
		item.Status = OutboxManualReview
		item.LastErrorCode = "reconciliation_budget_exhausted"
		item.hasLease = false
		return nil
	}
	backoffAttempt := item.ReconciliationAttempts + 1
	if backoffAttempt == 0 {
		backoffAttempt = 1
	}
	delay, err := OutboxBackoffDelayMS(item.EffectID, backoffAttempt, policy)
	if err != nil {
		return err
	}
	next, ok := addOutboxTime(nowMS, delay)
	if !ok {
		return ErrOutboxInvalidTime
	}
	item.NextReconciliationAtMS = next
	item.Status = OutboxAwaitingReconciliation
	item.hasLease = false
	return nil
}

func newOutboxLease(item *OutboxItem, workerID string, nowMS, durationMS uint64) (OutboxLease, error) {
	expiresAt, ok := addOutboxTime(nowMS, durationMS)
	if !ok {
		return OutboxLease{}, ErrOutboxInvalidTime
	}
	if item.LeaseGeneration == ^uint64(0) {
		return OutboxLease{}, ErrOutboxCounterOverflow
	}
	item.LeaseGeneration++
	return OutboxLease{
		EffectID: item.EffectID, WorkerID: workerID,
		Generation: item.LeaseGeneration, ExpiresAtMS: expiresAt,
	}, nil
}

func applyOutboxReceipt(item *OutboxItem, receipt OutboxReceipt) {
	if receipt.OperationID == item.OperationID &&
		receipt.EffectID == item.EffectID &&
		receipt.DestinationID == item.DestinationID &&
		receipt.PayloadDigest == item.PayloadDigest &&
		isSHA256Digest(receipt.ReceiptDigest) {
		item.Status = OutboxDelivered
		item.receiptDigest = receipt.ReceiptDigest
	} else {
		item.Status = OutboxManualReview
		if isSHA256Digest(receipt.ReceiptDigest) {
			item.LastEvidenceDigest = receipt.ReceiptDigest
		}
		item.LastErrorCode = "receipt_subject_mismatch"
	}
	item.hasLease = false
}

func validateNewOutboxEffect(effect NewOutboxEffect) error {
	for _, value := range []string{effect.OperationID, effect.EffectID, effect.DestinationID} {
		if strings.TrimSpace(value) == "" || len(value) > outboxMaxTextBytes || !utf8.ValidString(value) {
			return ErrOutboxInvalidInput
		}
	}
	if !isSHA256Digest(effect.PayloadDigest) {
		return ErrOutboxInvalidInput
	}
	return nil
}

func validateOutboxWorkerLease(workerID string, duration uint64, limit int, policy BackoffPolicy) error {
	if err := policy.Validate(); err != nil {
		return err
	}
	if strings.TrimSpace(workerID) == "" || len(workerID) > 128 || duration == 0 {
		return ErrOutboxInvalidInput
	}
	if limit < 0 || limit > outboxMaxBatch {
		return ErrOutboxBatchLimit
	}
	return nil
}

func addOutboxTime(left, right uint64) (uint64, bool) {
	if right > ^uint64(0)-left {
		return 0, false
	}
	return left + right, true
}

func OutboxBackoffDelayMS(effectID string, attempt uint32, policy BackoffPolicy) (uint64, error) {
	if err := policy.Validate(); err != nil {
		return 0, err
	}
	if strings.TrimSpace(effectID) == "" || len(effectID) > outboxMaxTextBytes || attempt == 0 {
		return 0, ErrOutboxInvalidInput
	}
	shift := attempt - 1
	if shift > 63 {
		shift = 63
	}
	multiplier := uint64(1) << shift
	exponential := policy.BaseDelayMS
	if multiplier > ^uint64(0)/policy.BaseDelayMS {
		exponential = policy.MaxDelayMS
	} else {
		exponential *= multiplier
	}
	if exponential > policy.MaxDelayMS {
		exponential = policy.MaxDelayMS
	}
	var encoded []byte
	encoded = append(encoded, []byte(outboxBackoffDomain)...)
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(effectID)))
	encoded = append(encoded, length[:]...)
	encoded = append(encoded, []byte(effectID)...)
	var attemptBytes [4]byte
	binary.BigEndian.PutUint32(attemptBytes[:], attempt)
	encoded = append(encoded, attemptBytes[:]...)
	digest := sha256.Sum256(encoded)
	random := binary.BigEndian.Uint64(digest[:8])
	jitter := random % (policy.JitterWindowMS + 1)
	delay, ok := addOutboxTime(exponential, jitter)
	if !ok || delay > policy.MaxDelayMS {
		delay = policy.MaxDelayMS
	}
	return delay, nil
}

func validOutboxErrorCode(value string) bool {
	if strings.TrimSpace(value) == "" || len(value) > outboxMaxErrorBytes {
		return false
	}
	for _, char := range []byte(value) {
		if !(char >= 'a' && char <= 'z') && !(char >= 'A' && char <= 'Z') &&
			!(char >= '0' && char <= '9') && char != '_' && char != '-' && char != '.' {
			return false
		}
	}
	return true
}
