// Package sagaverifier independently models the effects-disabled SAP payment
// journal. It is a separate implementation, not a proof of Holochain or rail
// settlement semantics.
package sagaverifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"reflect"
	"strings"
	"sync"
	"unicode/utf8"
)

const ReceiptProfileV1 = "mycelix.finance.payment-receipt.v1"

type Leg string

const (
	LocalPaymentTransaction Leg = "local-payment-transaction"
	TreasuryFee             Leg = "treasury-fee"
)

func (l Leg) suffix() string { return string(l) }

type Intent struct {
	SenderDID            string
	RecipientDID         string
	ClientIdempotencyKey string
	PayloadFingerprint   string
	AmountMinor          uint64
	FeeMinor             uint64
	Currency             string
}

func (i Intent) validate() error {
	if i.SenderDID == "" || i.RecipientDID == "" ||
		i.ClientIdempotencyKey == "" || i.Currency == "" ||
		i.SenderDID == i.RecipientDID {
		return ErrInvalidIntent
	}
	if !isSHA256Digest(i.PayloadFingerprint) {
		return ErrInvalidIntent
	}
	if i.AmountMinor == 0 {
		return ErrInvalidIntent
	}
	if i.FeeMinor > ^uint64(0)-i.AmountMinor {
		return ErrAmountOverflow
	}
	return nil
}

func checkedTotal(amount, fee uint64) (uint64, error) {
	if fee > ^uint64(0)-amount {
		return 0, ErrAmountOverflow
	}
	return amount + fee, nil
}

func OperationID(senderDID, clientKey string) string {
	// Byte lengths make this identity collision-resistant with respect to
	// embedded separators, matching the Rust reference model's framing.
	return fmt.Sprintf(
		"sap-payment:%d:%s:%d:%s",
		len(senderDID), senderDID, len(clientKey), clientKey,
	)
}

type CreateDisposition string

const (
	Created            CreateDisposition = "created"
	ExistingSameIntent CreateDisposition = "existing-same-intent"
)

type EffectStatus string

const (
	NotStarted    EffectStatus = "not-started"
	NotRequired   EffectStatus = "not-required"
	Prepared      EffectStatus = "prepared"
	Indeterminate EffectStatus = "indeterminate"
	Committed     EffectStatus = "committed"
)

type OperationStatus string

const (
	Proposed               OperationStatus = "proposed"
	AuthorizedPending      OperationStatus = "local-payment-pending"
	FeeDistributionPending OperationStatus = "fee-distribution-pending"
	OperationIndeterminate OperationStatus = "indeterminate"
	Rejected               OperationStatus = "rejected"
	Completed              OperationStatus = "completed"
)

type Transition string

const (
	TransitionPrepared          Transition = "prepared"
	TransitionAlreadyPrepared   Transition = "already-prepared"
	TransitionResumedSameEffect Transition = "resumed-same-effect"
	TransitionCommitted         Transition = "committed"
	TransitionAlreadyCommitted  Transition = "already-committed"
	TransitionNotRequired       Transition = "not-required"
)

var (
	ErrInvalidIntent           = errors.New("invalid payment intent")
	ErrAmountOverflow          = errors.New("principal plus fee overflows amount unit")
	ErrIdempotencyConflict     = errors.New("sender/key reused with a different immutable intent")
	ErrNotAuthorized           = errors.New("payment intent has not been authorized")
	ErrRejected                = errors.New("payment operation has been rejected")
	ErrDependencyNotSatisfied  = errors.New("payment leg dependency not satisfied")
	ErrEffectIdentityMismatch  = errors.New("payment effect identity changed")
	ErrInvalidTransition       = errors.New("invalid payment effect transition")
	ErrEvidenceInvalid         = errors.New("effect evidence does not match immutable intent")
	ErrEvidenceConflict        = errors.New("same effect identity has conflicting commit evidence")
	ErrCannotRejectAfterEffect = errors.New("cannot reject after an economic effect may have occurred")
)

type LocalPaymentEvidence struct {
	SenderAccountID            string
	RecipientAccountID         string
	SenderDebitMinor           uint64
	RecipientCreditMinor       uint64
	SenderPredecessorAction    string
	SenderDebitAction          string
	RecipientPredecessorAction string
	RecipientCreditAction      string
	PaymentActionHash          string
	PaymentStatus              string
	ReceiptActionHash          string
	ReceiptSignerDID           string
	SigningProfile             string
	SignatureDigest            string
	TreasuryFeeOutboxID        string
}

type TreasuryFeeEvidence struct {
	PoolID            string
	SourceMemberDID   string
	AmountMinor       uint64
	PaymentActionHash string
	IdempotencyKey    string
	PredecessorAction string
	TreasuryAction    string
}

type Evidence struct {
	OperationID    string
	EffectID       string
	Leg            Leg
	SenderDID      string
	RecipientDID   string
	PrincipalMinor uint64
	FeeMinor       uint64
	Currency       string
	EvidenceDigest string
	Local          *LocalPaymentEvidence
	Treasury       *TreasuryFeeEvidence
}

func (e Evidence) clone() Evidence {
	out := e
	if e.Local != nil {
		local := *e.Local
		out.Local = &local
	}
	if e.Treasury != nil {
		treasury := *e.Treasury
		out.Treasury = &treasury
	}
	return out
}

type EffectState struct {
	Status   EffectStatus
	EffectID string
	Evidence *Evidence
}

func (s EffectState) clone() EffectState {
	out := s
	if s.Evidence != nil {
		e := s.Evidence.clone()
		out.Evidence = &e
	}
	return out
}

type Journal struct {
	intent      Intent
	operationID string
	authorized  bool
	rejected    bool
	legs        map[Leg]EffectState
}

func newJournal(intent Intent) *Journal {
	j := &Journal{
		intent:      intent,
		operationID: OperationID(intent.SenderDID, intent.ClientIdempotencyKey),
		legs: map[Leg]EffectState{
			LocalPaymentTransaction: {Status: NotStarted},
			TreasuryFee:             {Status: NotStarted},
		},
	}
	if intent.FeeMinor == 0 {
		j.legs[TreasuryFee] = EffectState{Status: NotRequired}
	}
	return j
}

func (j *Journal) OperationID() string { return j.operationID }
func (j *Journal) Intent() Intent      { return j.intent }

func (j *Journal) EffectID(leg Leg) string {
	return j.operationID + ":" + leg.suffix()
}

func (j *Journal) EffectState(leg Leg) EffectState {
	state, ok := j.legs[leg]
	if !ok {
		return EffectState{Status: NotStarted}
	}
	return state.clone()
}

func (j *Journal) Authorize() error {
	if j.rejected {
		return ErrRejected
	}
	j.authorized = true
	return nil
}

func (j *Journal) hasEffectAttempted() bool {
	for _, state := range j.legs {
		switch state.Status {
		case Prepared, Indeterminate, Committed:
			return true
		}
	}
	return false
}

func (j *Journal) RejectBeforeEffects() error {
	if j.hasEffectAttempted() {
		return ErrCannotRejectAfterEffect
	}
	j.rejected = true
	return nil
}

func (j *Journal) localCommitted() bool {
	return j.legs[LocalPaymentTransaction].Status == Committed
}

func (j *Journal) localPaymentActionHash() string {
	state := j.legs[LocalPaymentTransaction]
	if state.Status != Committed || state.Evidence == nil || state.Evidence.Local == nil {
		return ""
	}
	return state.Evidence.Local.PaymentActionHash
}

func (j *Journal) dependenciesSatisfied(leg Leg) bool {
	switch leg {
	case LocalPaymentTransaction:
		return true
	case TreasuryFee:
		return j.localCommitted()
	default:
		return false
	}
}

func (j *Journal) PrepareLeg(leg Leg, effectID string) (Transition, error) {
	if j.rejected {
		return "", ErrRejected
	}
	if !j.authorized {
		return "", ErrNotAuthorized
	}
	if effectID != j.EffectID(leg) {
		return "", ErrEffectIdentityMismatch
	}
	if !j.dependenciesSatisfied(leg) {
		return "", ErrDependencyNotSatisfied
	}
	if leg == TreasuryFee && j.intent.FeeMinor == 0 {
		return TransitionNotRequired, nil
	}

	state, ok := j.legs[leg]
	if !ok {
		return "", ErrInvalidTransition
	}
	switch state.Status {
	case NotStarted:
		j.legs[leg] = EffectState{Status: Prepared, EffectID: effectID}
		return TransitionPrepared, nil
	case Prepared:
		if state.EffectID != effectID {
			return "", ErrEffectIdentityMismatch
		}
		return TransitionAlreadyPrepared, nil
	case Indeterminate:
		if state.EffectID != effectID {
			return "", ErrEffectIdentityMismatch
		}
		j.legs[leg] = EffectState{Status: Prepared, EffectID: effectID}
		return TransitionResumedSameEffect, nil
	case Committed:
		if state.EffectID != effectID {
			return "", ErrEffectIdentityMismatch
		}
		return TransitionAlreadyCommitted, nil
	case NotRequired:
		return TransitionNotRequired, nil
	default:
		return "", ErrInvalidTransition
	}
}

func (j *Journal) MarkIndeterminate(leg Leg, effectID string) error {
	state, ok := j.legs[leg]
	if !ok || state.EffectID != effectID {
		return ErrInvalidTransition
	}
	switch state.Status {
	case Prepared, Indeterminate:
		j.legs[leg] = EffectState{Status: Indeterminate, EffectID: effectID}
		return nil
	case Committed:
		return nil
	default:
		return ErrInvalidTransition
	}
}

func (j *Journal) validateEvidence(leg Leg, evidence Evidence) error {
	expectedDigest, digestErr := canonicalEvidenceDigest(evidence)
	commonValid := evidence.OperationID == j.operationID &&
		evidence.EffectID == j.EffectID(leg) &&
		evidence.Leg == leg &&
		evidence.SenderDID == j.intent.SenderDID &&
		evidence.RecipientDID == j.intent.RecipientDID &&
		evidence.PrincipalMinor == j.intent.AmountMinor &&
		evidence.FeeMinor == j.intent.FeeMinor &&
		evidence.Currency == j.intent.Currency &&
		digestErr == nil &&
		evidence.EvidenceDigest == expectedDigest
	if !commonValid {
		return ErrEvidenceInvalid
	}
	totalDebit, err := checkedTotal(j.intent.AmountMinor, j.intent.FeeMinor)
	if err != nil {
		return err
	}

	switch leg {
	case LocalPaymentTransaction:
		if evidence.Local == nil || evidence.Treasury != nil {
			return ErrEvidenceInvalid
		}
		local := evidence.Local
		outboxValid := false
		if j.intent.FeeMinor == 0 {
			outboxValid = local.TreasuryFeeOutboxID == ""
		} else {
			outboxValid = local.TreasuryFeeOutboxID == j.EffectID(TreasuryFee)
		}
		valid := local.SenderAccountID == j.intent.SenderDID &&
			local.RecipientAccountID == j.intent.RecipientDID &&
			local.SenderDebitMinor == totalDebit &&
			local.RecipientCreditMinor == j.intent.AmountMinor &&
			isActionReference(local.SenderPredecessorAction) &&
			isActionReference(local.SenderDebitAction) &&
			local.SenderPredecessorAction != local.SenderDebitAction &&
			isActionReference(local.RecipientPredecessorAction) &&
			isActionReference(local.RecipientCreditAction) &&
			local.RecipientPredecessorAction != local.RecipientCreditAction &&
			isActionReference(local.PaymentActionHash) &&
			local.PaymentStatus == "Completed" &&
			isActionReference(local.ReceiptActionHash) &&
			local.ReceiptActionHash != local.PaymentActionHash &&
			local.ReceiptSignerDID == j.intent.SenderDID &&
			local.SigningProfile == ReceiptProfileV1 &&
			isSHA256Digest(local.SignatureDigest) &&
			outboxValid
		if !valid {
			return ErrEvidenceInvalid
		}
		return nil

	case TreasuryFee:
		if evidence.Treasury == nil || evidence.Local != nil || j.intent.FeeMinor == 0 {
			return ErrEvidenceInvalid
		}
		treasury := evidence.Treasury
		valid := treasury.PoolID == "treasury:global-fee-pool" &&
			treasury.SourceMemberDID == j.intent.SenderDID &&
			treasury.AmountMinor == j.intent.FeeMinor &&
			treasury.PaymentActionHash == j.localPaymentActionHash() &&
			treasury.IdempotencyKey == j.EffectID(TreasuryFee) &&
			isActionReference(treasury.PaymentActionHash) &&
			isActionReference(treasury.PredecessorAction) &&
			isActionReference(treasury.TreasuryAction) &&
			treasury.PredecessorAction != treasury.TreasuryAction
		if !valid {
			return ErrEvidenceInvalid
		}
		return nil
	default:
		return ErrEvidenceInvalid
	}
}

func (j *Journal) CommitLeg(leg Leg, evidence Evidence) (Transition, error) {
	if j.rejected {
		return "", ErrRejected
	}
	if !j.dependenciesSatisfied(leg) {
		return "", ErrDependencyNotSatisfied
	}
	if leg == TreasuryFee && j.intent.FeeMinor == 0 {
		return TransitionNotRequired, nil
	}
	if err := j.validateEvidence(leg, evidence); err != nil {
		return "", err
	}
	state, ok := j.legs[leg]
	if !ok {
		return "", ErrInvalidTransition
	}
	switch state.Status {
	case Prepared, Indeterminate:
		if state.EffectID != evidence.EffectID {
			return "", ErrInvalidTransition
		}
		copyEvidence := evidence.clone()
		j.legs[leg] = EffectState{
			Status:   Committed,
			EffectID: evidence.EffectID,
			Evidence: &copyEvidence,
		}
		return TransitionCommitted, nil
	case Committed:
		if state.EffectID != evidence.EffectID {
			return "", ErrInvalidTransition
		}
		if state.Evidence != nil && reflect.DeepEqual(*state.Evidence, evidence) {
			return TransitionAlreadyCommitted, nil
		}
		return "", ErrEvidenceConflict
	default:
		return "", ErrInvalidTransition
	}
}

func (j *Journal) Status() OperationStatus {
	if j.rejected {
		return Rejected
	}
	for _, state := range j.legs {
		if state.Status == Indeterminate {
			return OperationIndeterminate
		}
	}
	if !j.localCommitted() {
		if j.authorized {
			return AuthorizedPending
		}
		return Proposed
	}
	feeState := j.legs[TreasuryFee].Status
	if feeState == Committed || feeState == NotRequired {
		return Completed
	}
	return FeeDistributionPending
}

type registryKey struct {
	sender string
	key    string
}

type Registry struct {
	mu         sync.Mutex
	operations map[registryKey]*Journal
}

func NewRegistry() *Registry {
	return &Registry{operations: make(map[registryKey]*Journal)}
}

func (r *Registry) CreateOrGet(intent Intent) (*Journal, CreateDisposition, error) {
	if err := intent.validate(); err != nil {
		return nil, "", err
	}
	key := registryKey{sender: intent.SenderDID, key: intent.ClientIdempotencyKey}
	r.mu.Lock()
	defer r.mu.Unlock()
	if existing, ok := r.operations[key]; ok {
		if existing.intent != intent {
			return nil, "", ErrIdempotencyConflict
		}
		return existing, ExistingSameIntent, nil
	}
	journal := newJournal(intent)
	r.operations[key] = journal
	return journal, Created, nil
}

func (r *Registry) Len() int {
	r.mu.Lock()
	defer r.mu.Unlock()
	return len(r.operations)
}

const maxCanonicalEvidenceFieldBytes = 4096

func appendCanonicalEvidenceString(output *bytes.Buffer, value string) error {
	if len(value) > maxCanonicalEvidenceFieldBytes || !utf8.ValidString(value) {
		return ErrEvidenceInvalid
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	output.Write(length[:])
	output.WriteString(value)
	return nil
}

func appendCanonicalEvidenceU64(output *bytes.Buffer, value uint64) {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	output.Write(encoded[:])
}

// canonicalEvidenceDigest uses the fixed-field MXSAP-EFFECT-EVIDENCE-V1
// profile: UTF-8 strings are u32-length-prefixed, integer units are u64
// big-endian, and effect variants have explicit one-byte tags. The digest
// field itself is intentionally excluded.
func canonicalEvidenceDigest(evidence Evidence) (string, error) {
	var output bytes.Buffer
	output.WriteString("MXSAP-EFFECT-EVIDENCE-V1\x00")

	for _, value := range []string{
		evidence.OperationID,
		evidence.EffectID,
		evidence.Leg.suffix(),
		evidence.SenderDID,
		evidence.RecipientDID,
	} {
		if err := appendCanonicalEvidenceString(&output, value); err != nil {
			return "", err
		}
	}
	appendCanonicalEvidenceU64(&output, evidence.PrincipalMinor)
	appendCanonicalEvidenceU64(&output, evidence.FeeMinor)
	if err := appendCanonicalEvidenceString(&output, evidence.Currency); err != nil {
		return "", err
	}

	switch evidence.Leg {
	case LocalPaymentTransaction:
		if evidence.Local == nil || evidence.Treasury != nil {
			return "", ErrEvidenceInvalid
		}
		local := evidence.Local
		output.WriteByte(1)
		for _, value := range []string{
			local.SenderAccountID,
			local.RecipientAccountID,
		} {
			if err := appendCanonicalEvidenceString(&output, value); err != nil {
				return "", err
			}
		}
		appendCanonicalEvidenceU64(&output, local.SenderDebitMinor)
		appendCanonicalEvidenceU64(&output, local.RecipientCreditMinor)
		for _, value := range []string{
			local.SenderPredecessorAction,
			local.SenderDebitAction,
			local.RecipientPredecessorAction,
			local.RecipientCreditAction,
			local.PaymentActionHash,
		} {
			if err := appendCanonicalEvidenceString(&output, value); err != nil {
				return "", err
			}
		}
		switch local.PaymentStatus {
		case "Completed":
			output.WriteByte(1)
		case "Pending":
			output.WriteByte(2)
		case "Failed":
			output.WriteByte(3)
		default:
			return "", ErrEvidenceInvalid
		}
		for _, value := range []string{
			local.ReceiptActionHash,
			local.ReceiptSignerDID,
			local.SigningProfile,
			local.SignatureDigest,
		} {
			if err := appendCanonicalEvidenceString(&output, value); err != nil {
				return "", err
			}
		}
		if local.TreasuryFeeOutboxID == "" {
			output.WriteByte(0)
		} else {
			output.WriteByte(1)
			if err := appendCanonicalEvidenceString(&output, local.TreasuryFeeOutboxID); err != nil {
				return "", err
			}
		}

	case TreasuryFee:
		if evidence.Treasury == nil || evidence.Local != nil {
			return "", ErrEvidenceInvalid
		}
		treasury := evidence.Treasury
		output.WriteByte(2)
		for _, value := range []string{
			treasury.PoolID,
			treasury.SourceMemberDID,
		} {
			if err := appendCanonicalEvidenceString(&output, value); err != nil {
				return "", err
			}
		}
		appendCanonicalEvidenceU64(&output, treasury.AmountMinor)
		for _, value := range []string{
			treasury.PaymentActionHash,
			treasury.IdempotencyKey,
			treasury.PredecessorAction,
			treasury.TreasuryAction,
		} {
			if err := appendCanonicalEvidenceString(&output, value); err != nil {
				return "", err
			}
		}
	default:
		return "", ErrEvidenceInvalid
	}

	digest := sha256.Sum256(output.Bytes())
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func isSHA256Digest(value string) bool {
	if !strings.HasPrefix(value, "sha256:") || len(value) != len("sha256:")+64 {
		return false
	}
	for i := len("sha256:"); i < len(value); i++ {
		c := value[i]
		if !((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) {
			return false
		}
	}
	return true
}

func isActionReference(value string) bool {
	if !strings.HasPrefix(value, "action:") || len(value) == len("action:") {
		return false
	}
	for i := len("action:"); i < len(value); i++ {
		c := value[i]
		if !((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
			(c >= '0' && c <= '9') || c == '-' || c == '_') {
			return false
		}
	}
	return true
}
