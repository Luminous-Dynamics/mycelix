package sagaverifier

import (
	"fmt"
)

type TraceDispositionV1 string

const (
	TraceApplied                    TraceDispositionV1 = "applied"
	TraceIdempotent                 TraceDispositionV1 = "idempotent"
	TraceNotRequired                TraceDispositionV1 = "not-required"
	TraceIndeterminateDisposition   TraceDispositionV1 = "indeterminate"
	TraceEffectObserved             TraceDispositionV1 = "effect-observed"
	TraceAuthoritativeAbsenceSeen   TraceDispositionV1 = "authoritative-absence-observed"
	TraceUnresolved                 TraceDispositionV1 = "unresolved"
	TraceRejected                   TraceDispositionV1 = "rejected"
	TraceDuplicateSuppressed        TraceDispositionV1 = "duplicate-suppressed"
	TraceRequiresSeparateEffect     TraceDispositionV1 = "requires-separate-effect"
)

type TraceInvariantV1 string

const (
	TraceInvariantStableOperation   TraceInvariantV1 = "stable-operation-identity"
	TraceInvariantStableEffect      TraceInvariantV1 = "stable-effect-identity"
	TraceInvariantLocalConservation TraceInvariantV1 = "local-conservation"
	TraceInvariantTreasuryConserve  TraceInvariantV1 = "treasury-fee-conservation"
	TraceInvariantDependency        TraceInvariantV1 = "dependency-ordering"
	TraceInvariantIndeterminate     TraceInvariantV1 = "indeterminate-preserved"
	TraceInvariantIntentImmutable   TraceInvariantV1 = "intent-immutability"
	TraceInvariantCompensation      TraceInvariantV1 = "compensation-separate-effect"
	TraceInvariantCheckpoint        TraceInvariantV1 = "checkpoint-authority"
	TraceInvariantOracleAgreement   TraceInvariantV1 = "oracle-disposition-agreement"
)

type TraceStepResultV1 struct {
	CommandIndex int
	Opcode       TraceOpcode
	Expected     TraceDispositionV1
	Observed     TraceDispositionV1
	Status       OperationStatus
}

type TraceInvariantFailureV1 struct {
	CommandIndex int
	Invariant    TraceInvariantV1
	Detail       string
}

type TraceEvaluationV1 struct {
	Seed             uint64
	TraceIndex       uint64
	Scenario         uint8
	TraceDigest      string
	OperationID      string
	LocalEffectID    string
	TreasuryEffectID string
	FinalStatus      OperationStatus
	Steps            []TraceStepResultV1
	Failures         []TraceInvariantFailureV1
}

func (e TraceEvaluationV1) Passed() bool {
	if len(e.Failures) != 0 {
		return false
	}
	for _, step := range e.Steps {
		if step.Expected != step.Observed {
			return false
		}
	}
	return true
}

type traceExecutionV1 struct {
	registry                    *Registry
	journal                     *Journal
	sender                      string
	idempotencyKey              string
	originalIntent              Intent
	operationID                 string
	localEffectID               string
	treasuryEffectID            string
	operationNonce              uint64
	effectNonce                 uint64
	hasEffectNonce              bool
	amountMinor                 uint64
	feeMinor                    uint64
	localCommitted              bool
	treasuryDispatched          bool
	remoteEffectPresent         bool
	authoritativeAbsenceSeen    bool
	receiptObserved             bool
	treasuryEvidence            *Evidence
	paymentsCheckpoint          uint64
	hasPaymentsCheckpoint       bool
	treasuryCheckpoint          uint64
	hasTreasuryCheckpoint       bool
	compensationSeen            bool
}

func expectedTraceOwner(opcode TraceOpcode) TraceOwner {
	switch opcode {
	case TraceCreateIntent, TraceAuthorize, TraceCompensate:
		return TraceSender
	case TracePrepareLocal, TraceCommitLocal, TraceCheckpointPayments:
		return TracePayments
	case TracePrepareTreasury, TraceDispatchTreasury, TraceMarkIndeterminate,
		TraceReconcileTreasury, TraceRetrySameEffect, TraceCommitTreasury,
		TraceCheckpointTreasury:
		return TraceTreasury
	default:
		return TraceAdversary
	}
}

// ExpectedTraceDispositionV1 is intentionally independent of Journal
// transitions. It is the frozen opcode/fault expectation table.
func ExpectedTraceDispositionV1(command TraceCommandV1, feeMinor uint64) TraceDispositionV1 {
	if command.Owner != expectedTraceOwner(command.Opcode) {
		return TraceRejected
	}
	if feeMinor == 0 {
		switch command.Opcode {
		case TracePrepareTreasury, TraceDispatchTreasury, TraceMarkIndeterminate,
			TraceReconcileTreasury, TraceRetrySameEffect, TraceCommitTreasury,
			TraceDuplicateDelivery, TraceCheckpointTreasury:
			return TraceNotRequired
		}
	}
	switch command.Opcode {
	case TraceMarkIndeterminate:
		return TraceIndeterminateDisposition
	case TraceReconcileTreasury:
		switch command.Fault {
		case TraceAuthoritativeAbsence:
			return TraceAuthoritativeAbsenceSeen
		case TraceLookupUncertain:
			return TraceUnresolved
		case TraceReceiptMismatch:
			return TraceRejected
		case TraceNoFault:
			return TraceEffectObserved
		default:
			return TraceRejected
		}
	case TraceDuplicateDelivery:
		return TraceDuplicateSuppressed
	case TraceMutateIntent, TraceMutateEvent, TraceStaleCheckpoint, TraceForkCheckpoint:
		return TraceRejected
	case TraceCompensate:
		return TraceRequiresSeparateEffect
	case TraceCheckpointPayments, TraceCheckpointTreasury:
		if command.Fault != TraceNoFault {
			return TraceRejected
		}
	}
	return TraceApplied
}

func traceDigest(seed uint64) string {
	return fmt.Sprintf("sha256:%064x", seed)
}

func traceLocalEvidence(journal *Journal, label string, seed uint64) Evidence {
	outboxID := ""
	if journal.intent.FeeMinor > 0 {
		outboxID = journal.EffectID(TreasuryFee)
	}
	evidence := Evidence{
		OperationID:    journal.operationID,
		EffectID:       journal.EffectID(LocalPaymentTransaction),
		Leg:            LocalPaymentTransaction,
		SenderDID:      journal.intent.SenderDID,
		RecipientDID:   journal.intent.RecipientDID,
		PrincipalMinor: journal.intent.AmountMinor,
		FeeMinor:       journal.intent.FeeMinor,
		Currency:       journal.intent.Currency,
		Local: &LocalPaymentEvidence{
			SenderAccountID:            journal.intent.SenderDID,
			RecipientAccountID:         journal.intent.RecipientDID,
			SenderDebitMinor:           journal.intent.AmountMinor + journal.intent.FeeMinor,
			RecipientCreditMinor:       journal.intent.AmountMinor,
			SenderPredecessorAction:    "action:trace-sender-before-" + label,
			SenderDebitAction:          "action:trace-sender-debit-" + label,
			RecipientPredecessorAction: "action:trace-recipient-before-" + label,
			RecipientCreditAction:      "action:trace-recipient-credit-" + label,
			PaymentActionHash:          "action:trace-payment-" + label,
			PaymentStatus:              "Completed",
			ReceiptActionHash:          "action:trace-receipt-" + label,
			ReceiptSignerDID:           journal.intent.SenderDID,
			SigningProfile:             ReceiptProfileV1,
			SignatureDigest:            traceDigest(seed + 10_000),
			TreasuryFeeOutboxID:         outboxID,
		},
	}
	digestValue, err := canonicalEvidenceDigest(evidence)
	if err != nil {
		panic(err)
	}
	evidence.EvidenceDigest = digestValue
	return evidence
}

func traceTreasuryEvidence(journal *Journal, effectID, label string) Evidence {
	evidence := Evidence{
		OperationID:    journal.operationID,
		EffectID:       effectID,
		Leg:            TreasuryFee,
		SenderDID:      journal.intent.SenderDID,
		RecipientDID:   journal.intent.RecipientDID,
		PrincipalMinor: journal.intent.AmountMinor,
		FeeMinor:       journal.intent.FeeMinor,
		Currency:       journal.intent.Currency,
		Treasury: &TreasuryFeeEvidence{
			PoolID:            "treasury:global-fee-pool",
			SourceMemberDID:   journal.intent.SenderDID,
			AmountMinor:       journal.intent.FeeMinor,
			PaymentActionHash: journal.localPaymentActionHash(),
			IdempotencyKey:    effectID,
			PredecessorAction: "action:trace-treasury-before-" + label,
			TreasuryAction:    "action:trace-treasury-effect-" + label,
		},
	}
	digestValue, err := canonicalEvidenceDigest(evidence)
	if err != nil {
		panic(err)
	}
	evidence.EvidenceDigest = digestValue
	return evidence
}

func mutateTraceTreasuryEvidence(evidence Evidence, opcode TraceOpcode) Evidence {
	mutated := evidence.clone()
	if mutated.Treasury == nil {
		return mutated
	}
	switch opcode {
	case TraceReconcileTreasury:
		mutated.Treasury.PaymentActionHash = "action:unlinked-payment"
	case TraceMutateEvent:
		mutated.Treasury.TreasuryAction = "action:mutated-treasury-event"
		mutated.Treasury.AmountMinor++
	}
	digestValue, err := canonicalEvidenceDigest(mutated)
	if err != nil {
		panic(err)
	}
	mutated.EvidenceDigest = digestValue
	return mutated
}

func (x *traceExecutionV1) label() string {
	return fmt.Sprintf("%016x-%016x", x.operationNonce, x.effectNonce)
}

func (x *traceExecutionV1) getTreasuryEvidence() Evidence {
	if x.treasuryEvidence != nil {
		return x.treasuryEvidence.clone()
	}
	evidence := traceTreasuryEvidence(x.journal, x.treasuryEffectID, x.label())
	x.treasuryEvidence = &evidence
	return evidence.clone()
}

func (x *traceExecutionV1) observe(command TraceCommandV1) TraceDispositionV1 {
	if command.Owner != expectedTraceOwner(command.Opcode) {
		return TraceRejected
	}
	feeNotRequired := x.feeMinor == 0
	switch command.Opcode {
	case TraceCreateIntent:
		if command.Subject != x.operationNonce || command.Value != x.amountMinor {
			return TraceRejected
		}
		journal, disposition, err := x.registry.CreateOrGet(x.originalIntent)
		if err != nil || journal.OperationID() != x.operationID {
			return TraceRejected
		}
		x.journal = journal
		if disposition == Created {
			return TraceApplied
		}
		return TraceIdempotent
	case TraceAuthorize:
		if command.Subject != x.operationNonce || command.Value != x.feeMinor || x.journal == nil {
			return TraceRejected
		}
		if err := x.journal.Authorize(); err != nil {
			return TraceRejected
		}
		return TraceApplied
	case TracePrepareLocal:
		if x.journal == nil || command.Subject != x.operationNonce ||
			command.Value != x.amountMinor+x.feeMinor {
			return TraceRejected
		}
		transition, err := x.journal.PrepareLeg(LocalPaymentTransaction, x.localEffectID)
		if err != nil {
			return TraceRejected
		}
		if transition == TransitionPrepared || transition == TransitionResumedSameEffect {
			return TraceApplied
		}
		return TraceIdempotent
	case TraceCommitLocal:
		if x.journal == nil || command.Subject != x.operationNonce ||
			command.Value != x.amountMinor+x.feeMinor {
			return TraceRejected
		}
		evidence := traceLocalEvidence(x.journal, "local-"+x.label(), x.operationNonce^x.effectNonce)
		transition, err := x.journal.CommitLeg(LocalPaymentTransaction, evidence)
		if err != nil {
			return TraceRejected
		}
		x.localCommitted = true
		if transition == TransitionCommitted {
			return TraceApplied
		}
		return TraceIdempotent
	case TracePrepareTreasury:
		if feeNotRequired {
			return TraceNotRequired
		}
		if x.journal == nil || (x.hasEffectNonce && command.Subject != x.effectNonce) ||
			command.Value != x.feeMinor {
			return TraceRejected
		}
		x.effectNonce = command.Subject
		x.hasEffectNonce = true
		transition, err := x.journal.PrepareLeg(TreasuryFee, x.treasuryEffectID)
		if err != nil {
			return TraceRejected
		}
		if transition == TransitionPrepared || transition == TransitionResumedSameEffect {
			return TraceApplied
		}
		return TraceIdempotent
	case TraceDispatchTreasury:
		if feeNotRequired {
			return TraceNotRequired
		}
		if x.journal == nil || !x.hasEffectNonce || command.Subject != x.effectNonce ||
			command.Value != x.feeMinor || x.journal.EffectState(TreasuryFee).Status != Prepared {
			return TraceRejected
		}
		x.treasuryDispatched = true
		if command.Fault == TraceNoFault {
			x.remoteEffectPresent = true
		}
		return TraceApplied
	case TraceMarkIndeterminate:
		if feeNotRequired {
			return TraceNotRequired
		}
		if !x.treasuryDispatched || command.Subject != x.effectNonce {
			return TraceRejected
		}
		x.remoteEffectPresent = command.Fault == TraceResponseLostAfterEffect
		x.authoritativeAbsenceSeen = false
		x.receiptObserved = false
		if err := x.journal.MarkIndeterminate(TreasuryFee, x.treasuryEffectID); err != nil {
			return TraceRejected
		}
		return TraceIndeterminateDisposition
	case TraceReconcileTreasury:
		if feeNotRequired {
			return TraceNotRequired
		}
		if !x.treasuryDispatched || !x.hasEffectNonce || command.Subject != x.effectNonce {
			return TraceRejected
		}
		switch command.Fault {
		case TraceAuthoritativeAbsence:
			if x.journal.Status() != OperationIndeterminate || x.remoteEffectPresent {
				return TraceRejected
			}
			x.authoritativeAbsenceSeen = true
			return TraceAuthoritativeAbsenceSeen
		case TraceLookupUncertain:
			if x.journal.Status() != OperationIndeterminate {
				return TraceRejected
			}
			return TraceUnresolved
		case TraceReceiptMismatch:
			mutated := mutateTraceTreasuryEvidence(x.getTreasuryEvidence(), TraceReconcileTreasury)
			if _, err := x.journal.CommitLeg(TreasuryFee, mutated); err != nil {
				return TraceRejected
			}
			return TraceApplied
		case TraceNoFault:
			if !x.remoteEffectPresent {
				return TraceRejected
			}
			x.receiptObserved = true
			return TraceEffectObserved
		default:
			return TraceRejected
		}
	case TraceRetrySameEffect:
		if feeNotRequired {
			return TraceNotRequired
		}
		if !x.authoritativeAbsenceSeen || x.remoteEffectPresent || command.Subject != x.effectNonce {
			return TraceRejected
		}
		transition, err := x.journal.PrepareLeg(TreasuryFee, x.treasuryEffectID)
		if err != nil {
			return TraceRejected
		}
		x.treasuryDispatched = false
		x.authoritativeAbsenceSeen = false
		if transition == TransitionPrepared || transition == TransitionResumedSameEffect {
			return TraceApplied
		}
		return TraceIdempotent
	case TraceCommitTreasury:
		if feeNotRequired {
			return TraceNotRequired
		}
		if !x.remoteEffectPresent ||
			(!x.receiptObserved && x.journal.Status() == OperationIndeterminate) {
			return TraceRejected
		}
		transition, err := x.journal.CommitLeg(TreasuryFee, x.getTreasuryEvidence())
		if err != nil {
			return TraceRejected
		}
		if transition == TransitionCommitted {
			return TraceApplied
		}
		return TraceIdempotent
	case TraceCheckpointPayments:
		if command.Fault != TraceNoFault || command.Subject != x.operationNonce {
			return TraceRejected
		}
		x.paymentsCheckpoint = command.Value
		x.hasPaymentsCheckpoint = true
		return TraceApplied
	case TraceCheckpointTreasury:
		if feeNotRequired {
			return TraceNotRequired
		}
		if command.Fault != TraceNoFault || !x.hasEffectNonce ||
			command.Subject != x.effectNonce || !x.hasPaymentsCheckpoint ||
			command.Value != x.paymentsCheckpoint {
			return TraceRejected
		}
		x.treasuryCheckpoint = command.Value
		x.hasTreasuryCheckpoint = true
		return TraceApplied
	case TraceMutateIntent:
		if x.journal == nil {
			return TraceRejected
		}
		changed := x.originalIntent
		changed.RecipientDID = "did:mycelix:trace-adversary"
		changed.PayloadFingerprint = traceDigest(x.operationNonce + 1)
		changed.AmountMinor = command.Value
		if changed.AmountMinor == 0 {
			changed.AmountMinor = 1
		}
		_, _, err := x.registry.CreateOrGet(changed)
		if err != ErrIdempotencyConflict || x.journal.OperationID() != x.operationID ||
			x.journal.Intent() != x.originalIntent {
			return TraceApplied
		}
		return TraceRejected
	case TraceDuplicateDelivery:
		if feeNotRequired {
			return TraceNotRequired
		}
		if !x.hasEffectNonce || command.Subject != x.effectNonce {
			return TraceRejected
		}
		if x.journal.EffectState(TreasuryFee).Status == Committed {
			transition, err := x.journal.CommitLeg(TreasuryFee, x.getTreasuryEvidence())
			if err == nil && transition == TransitionAlreadyCommitted {
				return TraceDuplicateSuppressed
			}
			return TraceRejected
		}
		_, err := x.journal.PrepareLeg(TreasuryFee, x.treasuryEffectID)
		if err != nil {
			return TraceRejected
		}
		return TraceDuplicateSuppressed
	case TraceMutateEvent:
		if feeNotRequired {
			return TraceRejected
		}
		_, err := x.journal.CommitLeg(
			TreasuryFee,
			mutateTraceTreasuryEvidence(x.getTreasuryEvidence(), TraceMutateEvent),
		)
		if err != nil {
			return TraceRejected
		}
		return TraceApplied
	case TraceStaleCheckpoint:
		return TraceRejected
	case TraceForkCheckpoint:
		return TraceRejected
	case TraceCompensate:
		x.compensationSeen = true
		return TraceRequiresSeparateEffect
	default:
		return TraceRejected
	}
}

func (x *traceExecutionV1) checkInvariants(commandIndex int, failures *[]TraceInvariantFailureV1) {
	if x.journal == nil {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantStableOperation,
			Detail: "journal is absent after CreateIntent",
		})
		return
	}
	if x.journal.OperationID() != x.operationID {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantStableOperation,
			Detail: "operation ID changed during trace execution",
		})
	}
	if x.journal.EffectID(LocalPaymentTransaction) != x.localEffectID ||
		x.journal.EffectID(TreasuryFee) != x.treasuryEffectID {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantStableEffect,
			Detail: "retry changed a stable effect identity",
		})
	}
	if x.journal.Intent() != x.originalIntent {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantIntentImmutable,
			Detail: "original intent changed after creation",
		})
	}
	localState := x.journal.EffectState(LocalPaymentTransaction)
	if localState.Status == Committed {
		if localState.Evidence == nil || localState.Evidence.Local == nil ||
			localState.Evidence.Local.SenderDebitMinor != x.amountMinor+x.feeMinor ||
			localState.Evidence.Local.RecipientCreditMinor != x.amountMinor {
			*failures = append(*failures, TraceInvariantFailureV1{
				CommandIndex: commandIndex, Invariant: TraceInvariantLocalConservation,
				Detail: "local evidence does not conserve principal and fee",
			})
		}
	}
	treasuryState := x.journal.EffectState(TreasuryFee)
	if treasuryState.Status == Committed {
		if treasuryState.Evidence == nil || treasuryState.Evidence.Treasury == nil ||
			treasuryState.Evidence.Treasury.AmountMinor != x.feeMinor ||
			localState.Status != Committed || !x.remoteEffectPresent {
			*failures = append(*failures, TraceInvariantFailureV1{
				CommandIndex: commandIndex, Invariant: TraceInvariantTreasuryConserve,
				Detail: "Treasury commit lacks local dependency or exact fee observation",
			})
		}
	}
	if x.journal.Status() == Completed &&
		(localState.Status != Committed ||
			(x.feeMinor > 0 && treasuryState.Status != Committed)) {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantDependency,
			Detail: "Completed while a required effect is unresolved",
		})
	}
	if treasuryState.Status == Indeterminate && x.journal.Status() != OperationIndeterminate {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantIndeterminate,
			Detail: "indeterminate Treasury effect not reflected by operation status",
		})
	}
	if x.authoritativeAbsenceSeen && x.receiptObserved {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantIndeterminate,
			Detail: "both authoritative absence and positive destination receipt observed",
		})
	}
	if x.compensationSeen && localState.Status != Committed {
		*failures = append(*failures, TraceInvariantFailureV1{
			CommandIndex: commandIndex, Invariant: TraceInvariantCompensation,
			Detail: "compensation rewrote the original local commit",
		})
	}
}

func EvaluateTraceV1(trace TraceV1) (TraceEvaluationV1, error) {
	if _, err := trace.CanonicalBytes(); err != nil {
		return TraceEvaluationV1{}, err
	}
	var create, authorize, prepareLocal *TraceCommandV1
	for index := range trace.Commands {
		command := &trace.Commands[index]
		switch command.Opcode {
		case TraceCreateIntent:
			create = command
		case TraceAuthorize:
			authorize = command
		case TracePrepareLocal:
			prepareLocal = command
		}
	}
	if create == nil || authorize == nil || prepareLocal == nil {
		return TraceEvaluationV1{}, ErrTraceEmptyCommands
	}
	total, err := checkedTotal(create.Value, authorize.Value)
	if err != nil || prepareLocal.Subject != create.Subject || prepareLocal.Value != total {
		return TraceEvaluationV1{}, ErrTraceInvalidScenario
	}
	sender := "did:mycelix:trace-sender"
	idempotencyKey := fmt.Sprintf("trace-%016x-%016x", trace.Seed, trace.TraceIndex)
	originalIntent := Intent{
		SenderDID: sender, RecipientDID: "did:mycelix:trace-recipient",
		ClientIdempotencyKey: idempotencyKey,
		PayloadFingerprint: traceDigest(trace.OperationNonce),
		AmountMinor: create.Value, FeeMinor: authorize.Value, Currency: "SAP",
	}
	operationID := OperationID(sender, idempotencyKey)
	context := &traceExecutionV1{
		registry: NewRegistry(),
		sender: sender,
		idempotencyKey: idempotencyKey,
		originalIntent: originalIntent,
		operationID: operationID,
		localEffectID: operationID + ":" + LocalPaymentTransaction.suffix(),
		treasuryEffectID: operationID + ":" + TreasuryFee.suffix(),
		operationNonce: trace.OperationNonce,
		amountMinor: create.Value,
		feeMinor: authorize.Value,
	}
	traceDigestValue, err := trace.CanonicalDigest()
	if err != nil {
		return TraceEvaluationV1{}, err
	}
	evaluation := TraceEvaluationV1{
		Seed: trace.Seed, TraceIndex: trace.TraceIndex, Scenario: trace.Scenario,
		TraceDigest: traceDigestValue, OperationID: operationID,
		LocalEffectID: context.localEffectID, TreasuryEffectID: context.treasuryEffectID,
		Steps: make([]TraceStepResultV1, 0, len(trace.Commands)),
	}
	for index, command := range trace.Commands {
		expected := ExpectedTraceDispositionV1(command, context.feeMinor)
		observed := context.observe(command)
		if expected != observed {
			evaluation.Failures = append(evaluation.Failures, TraceInvariantFailureV1{
				CommandIndex: index, Invariant: TraceInvariantOracleAgreement,
				Detail: fmt.Sprintf("%s expected %s observed %s", command.Opcode, expected, observed),
			})
		}
		status := Proposed
		if context.journal != nil {
			status = context.journal.Status()
		}
		evaluation.Steps = append(evaluation.Steps, TraceStepResultV1{
			CommandIndex: index, Opcode: command.Opcode,
			Expected: expected, Observed: observed, Status: status,
		})
		context.checkInvariants(index, &evaluation.Failures)
	}
	if context.journal != nil {
		evaluation.FinalStatus = context.journal.Status()
	}
	return evaluation, nil
}

func EvaluateGeneratedTraceV1(seed, traceIndex uint64) (TraceEvaluationV1, error) {
	return EvaluateTraceV1(GenerateTraceV1(seed, traceIndex))
}
