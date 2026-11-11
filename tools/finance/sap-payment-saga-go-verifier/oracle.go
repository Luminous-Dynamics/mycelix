package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"math/big"
)

const traceOracleDomain = "MXSAP-SAGA-ORACLE-V1\x00"

type TraceDispositionV1 uint8

const (
	TraceApplied TraceDispositionV1 = iota + 1
	TraceNoOp
	TraceRejected
	TraceIndeterminate
)

type TraceReasonV1 uint8

const (
	TraceReasonNone TraceReasonV1 = iota
	TraceReasonOwnerMismatch
	TraceReasonWrongSubject
	TraceReasonWrongAmount
	TraceReasonMissingIntent
	TraceReasonIntentConflict
	TraceReasonMissingAuthorization
	TraceReasonMissingPrepare
	TraceReasonMissingLocalCommit
	TraceReasonAlreadyCommitted
	TraceReasonNotRequired
	TraceReasonMissingTreasuryPrepare
	TraceReasonInvalidTreasuryState
	TraceReasonEffectOutcomeUnknown
	TraceReasonLookupUncertain
	TraceReasonAuthoritativeAbsenceContradiction
	TraceReasonReceiptMismatch
	TraceReasonMissingDependency
	TraceReasonStaleFrontier
	TraceReasonWrongOwner
	TraceReasonForkedCheckpoint
	TraceReasonDuplicateEffectSuppressed
	TraceReasonImmutableIntent
	TraceReasonMutatedEvent
	TraceReasonInvalidFault
	TraceReasonCompensationIdentity
	TraceReasonCompensationAmount
	TraceReasonCompensationUnavailable
	TraceReasonCheckpointConflict
	TraceReasonMissingDispatch
	TraceReasonUnknownEffect
	TraceReasonNotAuthoritativeAbsence
)

type TreasuryOracleStateV1 uint8

const (
	TreasuryNotPrepared TreasuryOracleStateV1 = iota
	TreasuryNotRequired
	TreasuryPrepared
	TreasuryDispatchAuthorized
	TreasuryIndeterminate
	TreasuryAuthoritativelyAbsent
	TreasuryObservedUnsettled
	TreasuryCommitted
)

type TraceStepResultV1 struct {
	CommandIndex      uint8
	Disposition       TraceDispositionV1
	Reason            TraceReasonV1
	ActualEffectCount uint8
}

type TraceEvaluationV1 struct {
	TraceDigest                       [32]byte
	Steps                             []TraceStepResultV1
	PrincipalMinor                    uint64
	FeeMinor                          uint64
	PayerDebitMinor                   uint64
	RecipientCreditMinor              uint64
	FeeHeldMinor                      uint64
	TreasuryCreditMinor               uint64
	CompensationSourceDebitMinor      uint64
	CompensationRecipientCreditMinor  uint64
	UnresolvedTreasuryExposureMinor   uint64
	ActualTreasuryEffectCount         uint8
	CompensationCount                 uint8
	LocalCommitted                    bool
	TreasuryState                     TreasuryOracleStateV1
	ConservationDeltaMinor            *big.Int
	InvariantFailureMask              uint32
}

func (e TraceEvaluationV1) IsSafe() bool {
	return e.InvariantFailureMask == 0 &&
		e.ConservationDeltaMinor != nil && e.ConservationDeltaMinor.Sign() == 0 &&
		e.ActualTreasuryEffectCount <= 1 && e.CompensationCount <= 1
}

func (e TraceEvaluationV1) CanonicalBytes() ([]byte, error) {
	if len(e.Steps) > MaxTraceCommands {
		return nil, ErrOracleTooManySteps
	}
	if e.ConservationDeltaMinor == nil {
		return nil, ErrOracleInvalidDelta
	}
	output := make([]byte, 0, 160+len(e.Steps)*4)
	output = append(output, []byte(traceOracleDomain)...)
	output = append(output, e.TraceDigest[:]...)
	var count [4]byte
	binary.BigEndian.PutUint32(count[:], uint32(len(e.Steps)))
	output = append(output, count[:]...)
	for _, step := range e.Steps {
		output = append(output,
			step.CommandIndex,
			byte(step.Disposition),
			byte(step.Reason),
			step.ActualEffectCount,
		)
	}
	for _, value := range []uint64{
		e.PrincipalMinor,
		e.FeeMinor,
		e.PayerDebitMinor,
		e.RecipientCreditMinor,
		e.FeeHeldMinor,
		e.TreasuryCreditMinor,
		e.CompensationSourceDebitMinor,
		e.CompensationRecipientCreditMinor,
		e.UnresolvedTreasuryExposureMinor,
	} {
		output = appendOracleU64(output, value)
	}
	if e.LocalCommitted {
		output = append(output, 1)
	} else {
		output = append(output, 0)
	}
	output = append(output,
		byte(e.TreasuryState),
		e.ActualTreasuryEffectCount,
		e.CompensationCount,
	)
	delta, err := encodeOracleSigned128(e.ConservationDeltaMinor)
	if err != nil {
		return nil, err
	}
	output = append(output, delta[:]...)
	var mask [4]byte
	binary.BigEndian.PutUint32(mask[:], e.InvariantFailureMask)
	output = append(output, mask[:]...)
	return output, nil
}

func (e TraceEvaluationV1) CanonicalDigest() (string, error) {
	encoded, err := e.CanonicalBytes()
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

var (
	ErrOracleInvalidTrace   = errors.New("trace failed canonical validation")
	ErrOracleTooManySteps   = errors.New("trace oracle step count exceeds canonical bound")
	ErrOracleInvalidDelta   = errors.New("trace oracle conservation delta is nil")
	ErrOracleDeltaOverflow  = errors.New("trace oracle conservation delta exceeds signed 128-bit range")
)

type oracleStateV1 struct {
	operationNonce           uint64
	intentCreated            bool
	authorized               bool
	localPrepared            bool
	localCommitted           bool
	principalMinor           uint64
	feeMinor                 uint64
	totalMinor               uint64
	treasuryEffectID         uint64
	hasTreasuryEffectID      bool
	treasuryState            TreasuryOracleStateV1
	actualTreasuryEffectCount uint8
	paymentCheckpoint        uint64
	hasPaymentCheckpoint     bool
	treasuryCheckpoint       uint64
	hasTreasuryCheckpoint    bool
	compensationEffectID     uint64
	compensationAmount       uint64
	hasCompensation          bool
}

func newOracleStateV1(operationNonce uint64) oracleStateV1 {
	return oracleStateV1{
		operationNonce: operationNonce,
		treasuryState:  TreasuryNotPrepared,
	}
}

func (s *oracleStateV1) process(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	expectedOwner := TraceSender
	switch command.Opcode {
	case TraceCreateIntent, TraceAuthorize, TraceCompensate:
		expectedOwner = TraceSender
	case TracePrepareLocal, TraceCommitLocal, TraceCheckpointPayments:
		expectedOwner = TracePayments
	case TracePrepareTreasury, TraceDispatchTreasury, TraceMarkIndeterminate,
		TraceReconcileTreasury, TraceRetrySameEffect, TraceCommitTreasury,
		TraceCheckpointTreasury:
		expectedOwner = TraceTreasury
	case TraceMutateIntent, TraceDuplicateDelivery, TraceMutateEvent,
		TraceStaleCheckpoint, TraceForkCheckpoint:
		expectedOwner = TraceAdversary
	}
	if command.Owner != expectedOwner {
		return TraceRejected, TraceReasonOwnerMismatch
	}

	switch command.Opcode {
	case TraceCreateIntent:
		return s.createIntent(command)
	case TraceAuthorize:
		return s.authorize(command)
	case TracePrepareLocal:
		return s.prepareLocal(command)
	case TraceCommitLocal:
		return s.commitLocal(command)
	case TracePrepareTreasury:
		return s.prepareTreasury(command)
	case TraceDispatchTreasury:
		return s.dispatchTreasury(command)
	case TraceMarkIndeterminate:
		return s.markIndeterminate(command)
	case TraceReconcileTreasury:
		return s.reconcileTreasury(command)
	case TraceRetrySameEffect:
		return s.retrySameEffect(command)
	case TraceCommitTreasury:
		return s.commitTreasury(command)
	case TraceCheckpointPayments:
		return s.checkpointPayments(command)
	case TraceCheckpointTreasury:
		return s.checkpointTreasury(command)
	case TraceMutateIntent:
		if command.Fault == TracePayloadMutation {
			return TraceRejected, TraceReasonImmutableIntent
		}
		return TraceRejected, TraceReasonInvalidFault
	case TraceDuplicateDelivery:
		return s.duplicateDelivery(command)
	case TraceMutateEvent:
		if command.Fault == TracePayloadMutation {
			return TraceRejected, TraceReasonMutatedEvent
		}
		return TraceRejected, TraceReasonInvalidFault
	case TraceStaleCheckpoint:
		if command.Fault == TraceWrongOwner {
			return TraceRejected, TraceReasonWrongOwner
		}
		return TraceRejected, TraceReasonInvalidFault
	case TraceForkCheckpoint:
		if command.Fault == TraceForkedCheckpoint {
			return TraceRejected, TraceReasonForkedCheckpoint
		}
		return TraceRejected, TraceReasonInvalidFault
	case TraceCompensate:
		return s.compensate(command)
	default:
		return TraceRejected, TraceReasonInvalidFault
	}
}

func (s *oracleStateV1) createIntent(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if command.Subject != s.operationNonce {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value == 0 {
		return TraceRejected, TraceReasonWrongAmount
	}
	if s.intentCreated {
		if s.principalMinor == command.Value {
			return TraceNoOp, TraceReasonAlreadyCommitted
		}
		return TraceRejected, TraceReasonIntentConflict
	}
	s.intentCreated = true
	s.principalMinor = command.Value
	return TraceApplied, TraceReasonNone
}

func (s *oracleStateV1) authorize(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if !s.intentCreated {
		return TraceRejected, TraceReasonMissingIntent
	}
	if command.Subject != s.operationNonce {
		return TraceRejected, TraceReasonWrongSubject
	}
	if s.authorized {
		if s.feeMinor == command.Value {
			return TraceNoOp, TraceReasonAlreadyCommitted
		}
		return TraceRejected, TraceReasonIntentConflict
	}
	s.feeMinor = command.Value
	s.authorized = true
	if s.feeMinor == 0 {
		s.treasuryState = TreasuryNotRequired
	}
	return TraceApplied, TraceReasonNone
}

func (s *oracleStateV1) prepareLocal(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if !s.intentCreated {
		return TraceRejected, TraceReasonMissingIntent
	}
	if !s.authorized {
		return TraceRejected, TraceReasonMissingAuthorization
	}
	if command.Subject != s.operationNonce {
		return TraceRejected, TraceReasonWrongSubject
	}
	if ^uint64(0)-s.principalMinor < s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	total := s.principalMinor + s.feeMinor
	if command.Value != total {
		return TraceRejected, TraceReasonWrongAmount
	}
	if s.localPrepared {
		return TraceNoOp, TraceReasonAlreadyCommitted
	}
	s.totalMinor = total
	s.localPrepared = true
	return TraceApplied, TraceReasonNone
}

func (s *oracleStateV1) commitLocal(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if !s.localPrepared {
		return TraceRejected, TraceReasonMissingPrepare
	}
	if command.Subject != s.operationNonce {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.totalMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	if s.localCommitted {
		return TraceNoOp, TraceReasonAlreadyCommitted
	}
	s.localCommitted = true
	return TraceApplied, TraceReasonNone
}

func (s *oracleStateV1) prepareTreasury(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if !s.localCommitted {
		return TraceRejected, TraceReasonMissingLocalCommit
	}
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	switch s.treasuryState {
	case TreasuryNotPrepared:
		s.treasuryEffectID = command.Subject
		s.hasTreasuryEffectID = true
		s.treasuryState = TreasuryPrepared
		return TraceApplied, TraceReasonNone
	case TreasuryPrepared:
		if s.hasTreasuryEffectID && s.treasuryEffectID == command.Subject {
			return TraceNoOp, TraceReasonAlreadyCommitted
		}
	}
	return TraceRejected, TraceReasonInvalidTreasuryState
}

func (s *oracleStateV1) dispatchTreasury(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if !s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	switch s.treasuryState {
	case TreasuryPrepared:
		s.treasuryState = TreasuryDispatchAuthorized
		return TraceApplied, TraceReasonNone
	case TreasuryDispatchAuthorized:
		return TraceNoOp, TraceReasonAlreadyCommitted
	default:
		return TraceRejected, TraceReasonInvalidTreasuryState
	}
}

func (s *oracleStateV1) markIndeterminate(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if !s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	if s.treasuryState == TreasuryIndeterminate {
		return TraceNoOp, TraceReasonEffectOutcomeUnknown
	}
	if s.treasuryState != TreasuryDispatchAuthorized {
		return TraceRejected, TraceReasonMissingDispatch
	}
	switch command.Fault {
	case TraceResponseLostBeforeEffect:
		s.actualTreasuryEffectCount = 0
	case TraceResponseLostAfterEffect:
		s.actualTreasuryEffectCount = 1
	default:
		return TraceRejected, TraceReasonInvalidFault
	}
	s.treasuryState = TreasuryIndeterminate
	return TraceIndeterminate, TraceReasonEffectOutcomeUnknown
}

func (s *oracleStateV1) reconcileTreasury(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if !s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	switch {
	case s.treasuryState == TreasuryIndeterminate && command.Fault == TraceAuthoritativeAbsence:
		if s.actualTreasuryEffectCount != 0 {
			return TraceRejected, TraceReasonAuthoritativeAbsenceContradiction
		}
		s.treasuryState = TreasuryAuthoritativelyAbsent
		return TraceApplied, TraceReasonNone
	case s.treasuryState == TreasuryIndeterminate && command.Fault == TraceLookupUncertain:
		return TraceIndeterminate, TraceReasonLookupUncertain
	case s.treasuryState == TreasuryIndeterminate && command.Fault == TraceReceiptMismatch:
		return TraceRejected, TraceReasonReceiptMismatch
	case s.treasuryState == TreasuryIndeterminate && command.Fault == TraceNoFault:
		if s.actualTreasuryEffectCount == 1 {
			s.treasuryState = TreasuryObservedUnsettled
			return TraceApplied, TraceReasonNone
		}
		return TraceIndeterminate, TraceReasonNotAuthoritativeAbsence
	case s.treasuryState == TreasuryAuthoritativelyAbsent && command.Fault == TraceAuthoritativeAbsence:
		return TraceNoOp, TraceReasonAlreadyCommitted
	case s.treasuryState == TreasuryObservedUnsettled && command.Fault == TraceNoFault:
		return TraceNoOp, TraceReasonAlreadyCommitted
	case s.treasuryState == TreasuryCommitted && command.Fault == TraceNoFault:
		return TraceNoOp, TraceReasonAlreadyCommitted
	default:
		return TraceRejected, TraceReasonInvalidTreasuryState
	}
}

func (s *oracleStateV1) retrySameEffect(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if !s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	switch s.treasuryState {
	case TreasuryAuthoritativelyAbsent:
		s.treasuryState = TreasuryPrepared
		return TraceApplied, TraceReasonNone
	case TreasuryPrepared:
		return TraceNoOp, TraceReasonAlreadyCommitted
	default:
		return TraceRejected, TraceReasonNotAuthoritativeAbsence
	}
}

func (s *oracleStateV1) commitTreasury(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if !s.localCommitted {
		return TraceRejected, TraceReasonMissingLocalCommit
	}
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if !s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	switch s.treasuryState {
	case TreasuryDispatchAuthorized:
		s.actualTreasuryEffectCount = 1
		s.treasuryState = TreasuryCommitted
		return TraceApplied, TraceReasonNone
	case TreasuryObservedUnsettled:
		if s.actualTreasuryEffectCount != 1 {
			return TraceRejected, TraceReasonUnknownEffect
		}
		s.treasuryState = TreasuryCommitted
		return TraceApplied, TraceReasonNone
	case TreasuryCommitted:
		return TraceNoOp, TraceReasonAlreadyCommitted
	case TreasuryIndeterminate:
		return TraceRejected, TraceReasonEffectOutcomeUnknown
	case TreasuryAuthoritativelyAbsent:
		return TraceRejected, TraceReasonNotAuthoritativeAbsence
	default:
		return TraceRejected, TraceReasonInvalidTreasuryState
	}
}

func (s *oracleStateV1) checkpointPayments(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if command.Subject != s.operationNonce {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Fault == TraceMissingDependency {
		return TraceRejected, TraceReasonMissingDependency
	}
	if command.Fault != TraceNoFault {
		return TraceRejected, TraceReasonInvalidFault
	}
	if !s.localCommitted {
		return TraceRejected, TraceReasonMissingDependency
	}
	if s.hasPaymentCheckpoint {
		if s.paymentCheckpoint == command.Value {
			return TraceNoOp, TraceReasonAlreadyCommitted
		}
		return TraceRejected, TraceReasonCheckpointConflict
	}
	s.paymentCheckpoint = command.Value
	s.hasPaymentCheckpoint = true
	return TraceApplied, TraceReasonNone
}

func (s *oracleStateV1) checkpointTreasury(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if command.Fault == TraceStaleFrontier {
		return TraceRejected, TraceReasonStaleFrontier
	}
	if command.Fault != TraceNoFault {
		return TraceRejected, TraceReasonInvalidFault
	}
	if s.feeMinor > 0 && (!s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject) {
		return TraceRejected, TraceReasonWrongSubject
	}
	if s.feeMinor > 0 && s.treasuryState != TreasuryCommitted && s.treasuryState != TreasuryNotRequired {
		return TraceRejected, TraceReasonMissingDependency
	}
	if s.feeMinor == 0 && s.treasuryState != TreasuryNotRequired {
		return TraceRejected, TraceReasonMissingDependency
	}
	if s.hasTreasuryCheckpoint {
		if s.treasuryCheckpoint == command.Value {
			return TraceNoOp, TraceReasonAlreadyCommitted
		}
		return TraceRejected, TraceReasonCheckpointConflict
	}
	s.treasuryCheckpoint = command.Value
	s.hasTreasuryCheckpoint = true
	return TraceApplied, TraceReasonNone
}

func (s *oracleStateV1) duplicateDelivery(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if command.Fault != TraceDuplicateDeliveryFault {
		return TraceRejected, TraceReasonInvalidFault
	}
	if s.feeMinor == 0 || s.treasuryState == TreasuryNotRequired {
		return TraceNoOp, TraceReasonNotRequired
	}
	if !s.hasTreasuryEffectID || s.treasuryEffectID != command.Subject {
		return TraceRejected, TraceReasonWrongSubject
	}
	if command.Value != s.feeMinor {
		return TraceRejected, TraceReasonWrongAmount
	}
	switch s.treasuryState {
	case TreasuryDispatchAuthorized:
		s.actualTreasuryEffectCount = 1
		s.treasuryState = TreasuryIndeterminate
		return TraceIndeterminate, TraceReasonDuplicateEffectSuppressed
	case TreasuryIndeterminate:
		return TraceIndeterminate, TraceReasonDuplicateEffectSuppressed
	case TreasuryCommitted:
		return TraceNoOp, TraceReasonAlreadyCommitted
	default:
		return TraceRejected, TraceReasonInvalidTreasuryState
	}
}

func (s *oracleStateV1) compensate(command TraceCommandV1) (TraceDispositionV1, TraceReasonV1) {
	if !s.localCommitted || (s.feeMinor > 0 && s.treasuryState != TreasuryCommitted) {
		return TraceRejected, TraceReasonCompensationUnavailable
	}
	if command.Subject == s.operationNonce {
		return TraceRejected, TraceReasonCompensationIdentity
	}
	if command.Value != s.totalMinor {
		return TraceRejected, TraceReasonCompensationAmount
	}
	if s.hasCompensation {
		if s.compensationEffectID == command.Subject && s.compensationAmount == command.Value {
			return TraceNoOp, TraceReasonAlreadyCommitted
		}
		return TraceRejected, TraceReasonCompensationIdentity
	}
	s.compensationEffectID = command.Subject
	s.compensationAmount = command.Value
	s.hasCompensation = true
	return TraceApplied, TraceReasonNone
}

type TraceOracleV1 struct{}

func (TraceOracleV1) Evaluate(trace TraceV1) (TraceEvaluationV1, error) {
	encoded, err := trace.CanonicalBytes()
	if err != nil {
		return TraceEvaluationV1{}, ErrOracleInvalidTrace
	}
	traceDigest := sha256.Sum256(encoded)
	state := newOracleStateV1(trace.OperationNonce)
	steps := make([]TraceStepResultV1, 0, len(trace.Commands))
	for index, command := range trace.Commands {
		disposition, reason := state.process(command)
		steps = append(steps, TraceStepResultV1{
			CommandIndex: uint8(index), Disposition: disposition,
			Reason: reason, ActualEffectCount: state.actualTreasuryEffectCount,
		})
	}
	payerDebit := uint64(0)
	recipientCredit := uint64(0)
	if state.localCommitted {
		payerDebit = state.totalMinor
		recipientCredit = state.principalMinor
	}
	treasuryCredit := uint64(0)
	if state.treasuryState == TreasuryCommitted {
		treasuryCredit = state.feeMinor
	}
	feeHeld := uint64(0)
	if state.localCommitted && treasuryCredit == 0 {
		feeHeld = state.feeMinor
	}
	compensationDebit := uint64(0)
	if state.hasCompensation {
		compensationDebit = state.compensationAmount
	}
	compensationCredit := compensationDebit
	unresolved := uint64(0)
	switch state.treasuryState {
	case TreasuryPrepared, TreasuryDispatchAuthorized, TreasuryIndeterminate, TreasuryObservedUnsettled:
		unresolved = state.feeMinor
	}
	delta := new(big.Int).SetUint64(payerDebit)
	delta.Sub(delta, new(big.Int).SetUint64(recipientCredit))
	delta.Sub(delta, new(big.Int).SetUint64(feeHeld))
	delta.Sub(delta, new(big.Int).SetUint64(treasuryCredit))
	delta.Add(delta, new(big.Int).SetUint64(compensationDebit))
	delta.Sub(delta, new(big.Int).SetUint64(compensationCredit))

	var failureMask uint32
	if delta.Sign() != 0 {
		failureMask |= 1
	}
	if state.actualTreasuryEffectCount > 1 {
		failureMask |= 1 << 1
	}
	if state.hasCompensation && state.compensationEffectID == trace.OperationNonce {
		failureMask |= 1 << 2
	}
	if state.treasuryState == TreasuryCommitted && state.feeMinor > 0 && state.actualTreasuryEffectCount != 1 {
		failureMask |= 1 << 3
	}
	if state.treasuryState == TreasuryAuthoritativelyAbsent && state.actualTreasuryEffectCount != 0 {
		failureMask |= 1 << 4
	}
	return TraceEvaluationV1{
		TraceDigest: traceDigest,
		Steps: steps,
		PrincipalMinor: state.principalMinor,
		FeeMinor: state.feeMinor,
		PayerDebitMinor: payerDebit,
		RecipientCreditMinor: recipientCredit,
		FeeHeldMinor: feeHeld,
		TreasuryCreditMinor: treasuryCredit,
		CompensationSourceDebitMinor: compensationDebit,
		CompensationRecipientCreditMinor: compensationCredit,
		UnresolvedTreasuryExposureMinor: unresolved,
		ActualTreasuryEffectCount: state.actualTreasuryEffectCount,
		CompensationCount: boolToU8(state.hasCompensation),
		LocalCommitted: state.localCommitted,
		TreasuryState: state.treasuryState,
		ConservationDeltaMinor: delta,
		InvariantFailureMask: failureMask,
	}, nil
}

func boolToU8(value bool) uint8 {
	if value { return 1 }
	return 0
}

func appendOracleU64(output []byte, value uint64) []byte {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	return append(output, encoded[:]...)
}

func encodeOracleSigned128(value *big.Int) ([16]byte, error) {
	var output [16]byte
	if value == nil { return output, ErrOracleInvalidDelta }
	limit := new(big.Int).Lsh(big.NewInt(1), 127)
	minimum := new(big.Int).Neg(new(big.Int).Set(limit))
	maximum := new(big.Int).Sub(new(big.Int).Set(limit), big.NewInt(1))
	if value.Cmp(minimum) < 0 || value.Cmp(maximum) > 0 {
		return output, ErrOracleDeltaOverflow
	}
	encoded := new(big.Int).Set(value)
	if encoded.Sign() < 0 {
		encoded.Add(encoded, new(big.Int).Lsh(big.NewInt(1), 128))
	}
	bytes := encoded.Bytes()
	copy(output[len(output)-len(bytes):], bytes)
	return output, nil
}
