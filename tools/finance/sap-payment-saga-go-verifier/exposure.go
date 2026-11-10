package sagaverifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"sort"
	"strings"
)

const (
	exposureSnapshotDomain = "MXSAP-EXPOSURE-SNAPSHOT-V1\x00"
	maxExposureFieldBytes  = 4096
)

type ExposureScopeV1 struct {
	OwnerID       string
	PoolID        string
	DestinationID string
	Currency      string
}

func (s ExposureScopeV1) validate() error {
	for _, value := range []string{s.OwnerID, s.PoolID, s.DestinationID, s.Currency} {
		if err := validateExposureText(value); err != nil {
			return err
		}
	}
	return nil
}

type ExposurePolicyV1 struct {
	Scope                ExposureScopeV1
	Sequence             uint64
	MaximumTotalMinor    uint64
	MaximumPerEffectMinor uint64
	PolicyDigest         string
}

func (p ExposurePolicyV1) Validate() error {
	if err := p.Scope.validate(); err != nil {
		return err
	}
	if p.Sequence == 0 || p.MaximumTotalMinor == 0 ||
		p.MaximumPerEffectMinor == 0 || p.MaximumPerEffectMinor > p.MaximumTotalMinor {
		return ErrExposureInvalidPolicy
	}
	return validateExposureDigest(p.PolicyDigest)
}

type ExposureIntentV1 struct {
	Scope                 ExposureScopeV1
	OperationID           string
	EffectID              string
	IntentDigest          string
	PayloadDigest         string
	MaximumExposureMinor  uint64
}

func (i ExposureIntentV1) validate() error {
	if err := i.Scope.validate(); err != nil {
		return err
	}
	for _, value := range []string{i.OperationID, i.EffectID} {
		if err := validateExposureText(value); err != nil {
			return err
		}
	}
	if err := validateExposureDigest(i.IntentDigest); err != nil {
		return err
	}
	if err := validateExposureDigest(i.PayloadDigest); err != nil {
		return err
	}
	if i.MaximumExposureMinor == 0 {
		return ErrExposureInvalidAmount
	}
	return nil
}

type ExposureStatus uint8

const (
	ExposureReserved ExposureStatus = iota + 1
	ExposureDispatchAuthorized
	ExposureIndeterminate
	ExposureCommittedUnsettled
	ExposureSettled
	ExposureAuthoritativelyAbsent
	ExposureCancelledBeforeDispatch
)

func (s ExposureStatus) contributesToLimit() bool {
	switch s {
	case ExposureReserved, ExposureDispatchAuthorized, ExposureIndeterminate, ExposureCommittedUnsettled:
		return true
	default:
		return false
	}
}

type ExposureReservationV1 struct {
	Intent ExposureIntentV1
	Status ExposureStatus
}

type ExposureDisposition uint8

const (
	ExposureWasReserved ExposureDisposition = iota + 1
	ExposureAlreadyReserved
)

type ExposureResolutionKind uint8

const (
	ExposureSettlementReceipt ExposureResolutionKind = iota + 1
	ExposureAuthoritativeAbsence
)

type DestinationResolutionV1 struct {
	Kind          ExposureResolutionKind
	EffectID      string
	DestinationID string
	PayloadDigest string
	ProfileDigest string
	EvidenceDigest string
}

func (e DestinationResolutionV1) validateAgainst(r ExposureReservationV1, expected ExposureResolutionKind) error {
	if e.Kind != expected ||
		!isExposureDigest(e.PayloadDigest) ||
		!isExposureDigest(e.ProfileDigest) ||
		!isExposureDigest(e.EvidenceDigest) ||
		e.EffectID != r.Intent.EffectID ||
		e.DestinationID != r.Intent.Scope.DestinationID ||
		e.PayloadDigest != r.Intent.PayloadDigest {
		return ErrExposureInvalidResolution
	}
	return nil
}

var (
	ErrExposureInvalidText         = errors.New("exposure field is empty or oversized")
	ErrExposureInvalidDigest       = errors.New("exposure digest is not canonical")
	ErrExposureInvalidPolicy       = errors.New("exposure policy is invalid")
	ErrExposureInvalidAmount       = errors.New("exposure amount must be positive")
	ErrExposureScopeMismatch       = errors.New("exposure scope does not match owner policy")
	ErrExposureReservationConflict = errors.New("effect identity reused with different exposure intent")
	ErrExposureNotFound            = errors.New("exposure reservation not found")
	ErrExposureLimitExceeded       = errors.New("owner-authorized exposure limit exceeded")
	ErrExposureAmountOverflow      = errors.New("exposure amount overflow")
	ErrExposureInvalidTransition   = errors.New("invalid exposure state transition")
	ErrExposureInvalidResolution   = errors.New("resolution evidence does not match exact effect subject")
)

type ExposureBookV1 struct {
	policy       ExposurePolicyV1
	reservations map[string]ExposureReservationV1
}

func NewExposureBookV1(policy ExposurePolicyV1) (*ExposureBookV1, error) {
	if err := policy.Validate(); err != nil {
		return nil, err
	}
	return &ExposureBookV1{
		policy: policy,
		reservations: make(map[string]ExposureReservationV1),
	}, nil
}

func (b *ExposureBookV1) Policy() ExposurePolicyV1 {
	return b.policy
}

// ReplacePolicy never removes old reservations. A lower limit can leave the
// book temporarily over-limit; new risk-increasing reservations then fail.
func (b *ExposureBookV1) ReplacePolicy(policy ExposurePolicyV1) error {
	if err := policy.Validate(); err != nil {
		return err
	}
	if policy.Scope != b.policy.Scope || policy.Sequence <= b.policy.Sequence {
		return ErrExposureInvalidPolicy
	}
	b.policy = policy
	return nil
}

func (b *ExposureBookV1) Reserve(intent ExposureIntentV1) (ExposureDisposition, error) {
	if err := intent.validate(); err != nil {
		return 0, err
	}
	if intent.Scope != b.policy.Scope {
		return 0, ErrExposureScopeMismatch
	}
	if intent.MaximumExposureMinor > b.policy.MaximumPerEffectMinor {
		return 0, ErrExposureLimitExceeded
	}
	if existing, ok := b.reservations[intent.EffectID]; ok {
		if existing.Intent == intent {
			return ExposureAlreadyReserved, nil
		}
		return 0, ErrExposureReservationConflict
	}
	current, err := b.TotalExposureMinor()
	if err != nil {
		return 0, err
	}
	if intent.MaximumExposureMinor > ^uint64(0)-current {
		return 0, ErrExposureAmountOverflow
	}
	projected := current + intent.MaximumExposureMinor
	if projected > b.policy.MaximumTotalMinor {
		return 0, ErrExposureLimitExceeded
	}
	b.reservations[intent.EffectID] = ExposureReservationV1{
		Intent: intent,
		Status: ExposureReserved,
	}
	return ExposureWasReserved, nil
}

func (b *ExposureBookV1) MarkDispatchAuthorized(effectID string) error {
	return b.transition(effectID, []ExposureStatus{ExposureReserved}, ExposureDispatchAuthorized)
}

func (b *ExposureBookV1) MarkIndeterminate(effectID string) error {
	return b.transition(effectID, []ExposureStatus{ExposureDispatchAuthorized, ExposureIndeterminate}, ExposureIndeterminate)
}

func (b *ExposureBookV1) MarkCommittedUnsettled(effectID string) error {
	return b.transition(effectID, []ExposureStatus{ExposureDispatchAuthorized, ExposureIndeterminate, ExposureCommittedUnsettled}, ExposureCommittedUnsettled)
}

func (b *ExposureBookV1) MarkSettled(effectID string, evidence DestinationResolutionV1) error {
	reservation, ok := b.reservations[effectID]
	if !ok {
		return ErrExposureNotFound
	}
	if err := evidence.validateAgainst(reservation, ExposureSettlementReceipt); err != nil {
		return err
	}
	if reservation.Status == ExposureSettled {
		return nil
	}
	if reservation.Status != ExposureCommittedUnsettled {
		return ErrExposureInvalidTransition
	}
	reservation.Status = ExposureSettled
	b.reservations[effectID] = reservation
	return nil
}

func (b *ExposureBookV1) ReleaseAuthoritativeAbsence(effectID string, evidence DestinationResolutionV1) error {
	reservation, ok := b.reservations[effectID]
	if !ok {
		return ErrExposureNotFound
	}
	if err := evidence.validateAgainst(reservation, ExposureAuthoritativeAbsence); err != nil {
		return err
	}
	if reservation.Status == ExposureAuthoritativelyAbsent {
		return nil
	}
	if reservation.Status != ExposureDispatchAuthorized && reservation.Status != ExposureIndeterminate {
		return ErrExposureInvalidTransition
	}
	reservation.Status = ExposureAuthoritativelyAbsent
	b.reservations[effectID] = reservation
	return nil
}

func (b *ExposureBookV1) CancelBeforeDispatch(effectID string) error {
	return b.transition(effectID, []ExposureStatus{ExposureReserved}, ExposureCancelledBeforeDispatch)
}

func (b *ExposureBookV1) Status(effectID string) (ExposureStatus, bool) {
	reservation, ok := b.reservations[effectID]
	return reservation.Status, ok
}

func (b *ExposureBookV1) ReservationCount() int {
	return len(b.reservations)
}

func (b *ExposureBookV1) TotalExposureMinor() (uint64, error) {
	var total uint64
	for _, reservation := range b.reservations {
		if !reservation.Status.contributesToLimit() {
			continue
		}
		amount := reservation.Intent.MaximumExposureMinor
		if amount > ^uint64(0)-total {
			return 0, ErrExposureAmountOverflow
		}
		total += amount
	}
	return total, nil
}

func (b *ExposureBookV1) HeadroomMinor() (uint64, error) {
	total, err := b.TotalExposureMinor()
	if err != nil {
		return 0, err
	}
	if total >= b.policy.MaximumTotalMinor {
		return 0, nil
	}
	return b.policy.MaximumTotalMinor - total, nil
}

func (b *ExposureBookV1) CanonicalSnapshotBytes() ([]byte, error) {
	var output bytes.Buffer
	output.WriteString(exposureSnapshotDomain)
	for _, value := range []string{
		b.policy.Scope.OwnerID,
		b.policy.Scope.PoolID,
		b.policy.Scope.DestinationID,
		b.policy.Scope.Currency,
	} {
		if err := appendExposureText(&output, value); err != nil {
			return nil, err
		}
	}
	appendExposureU64(&output, b.policy.Sequence)
	appendExposureU64(&output, b.policy.MaximumTotalMinor)
	appendExposureU64(&output, b.policy.MaximumPerEffectMinor)
	if err := appendExposureText(&output, b.policy.PolicyDigest); err != nil {
		return nil, err
	}

	keys := make([]string, 0, len(b.reservations))
	for key := range b.reservations {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	if uint64(len(keys)) > uint64(^uint32(0)) {
		return nil, ErrExposureInvalidAmount
	}
	var count [4]byte
	binary.BigEndian.PutUint32(count[:], uint32(len(keys)))
	output.Write(count[:])

	for _, key := range keys {
		reservation := b.reservations[key]
		intent := reservation.Intent
		for _, value := range []string{
			intent.OperationID, intent.EffectID, intent.IntentDigest, intent.PayloadDigest,
		} {
			if err := appendExposureText(&output, value); err != nil {
				return nil, err
			}
		}
		appendExposureU64(&output, intent.MaximumExposureMinor)
		output.WriteByte(byte(reservation.Status))
	}
	return output.Bytes(), nil
}

func (b *ExposureBookV1) CanonicalSnapshotDigest() (string, error) {
	encoded, err := b.CanonicalSnapshotBytes()
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func (b *ExposureBookV1) transition(effectID string, allowed []ExposureStatus, next ExposureStatus) error {
	reservation, ok := b.reservations[effectID]
	if !ok {
		return ErrExposureNotFound
	}
	for _, state := range allowed {
		if reservation.Status == state {
			reservation.Status = next
			b.reservations[effectID] = reservation
			return nil
		}
	}
	return ErrExposureInvalidTransition
}

func validateExposureText(value string) error {
	if value == "" || len(value) > maxExposureFieldBytes {
		return ErrExposureInvalidText
	}
	return nil
}

func isExposureDigest(value string) bool {
	if len(value) != 71 || !strings.HasPrefix(value, "sha256:") {
		return false
	}
	for i := 7; i < len(value); i++ {
		c := value[i]
		if !((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) {
			return false
		}
	}
	return true
}

func validateExposureDigest(value string) error {
	if !isExposureDigest(value) {
		return ErrExposureInvalidDigest
	}
	return nil
}

func appendExposureText(output *bytes.Buffer, value string) error {
	if err := validateExposureText(value); err != nil {
		return err
	}
	if uint64(len(value)) > uint64(^uint32(0)) {
		return ErrExposureInvalidText
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	output.Write(length[:])
	output.WriteString(value)
	return nil
}

func appendExposureU64(output *bytes.Buffer, value uint64) {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	output.Write(encoded[:])
}

