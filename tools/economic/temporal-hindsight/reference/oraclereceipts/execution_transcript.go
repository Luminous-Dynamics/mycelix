package oraclereceipts

import (
	"bytes"
	"errors"
	"fmt"
	"sort"
)

const (
	MaxExecutionEvents = 100000
	MaxExecutionEncodingBytes uint64 = 64 << 20

	StepApply ExecutionStepKindV1 = "Apply"
	StepRecomputeAfterCorrection ExecutionStepKindV1 = "RecomputeAfterCorrection"
	StepCompensate ExecutionStepKindV1 = "Compensate"
	StepRevokeDerivedState ExecutionStepKindV1 = "RevokeDerivedState"

	ExecutionCommitted ExecutionEventOutcomeV1 = "Committed"
	ExecutionUnknown ExecutionEventOutcomeV1 = "OutcomeUnknown"
	ExecutionReconciledApplied ExecutionEventOutcomeV1 = "ReconciledApplied"
	ExecutionReconciledNotApplied ExecutionEventOutcomeV1 = "ReconciledNotApplied"

	ExecutionChainStructurallyValid ExecutionDispositionV1 = "ExecutionTranscriptStructurallyValid"
	ExecutionChainIndeterminate ExecutionDispositionV1 = "Indeterminate"
)

type ExecutionStepKindV1 string
type ExecutionEventOutcomeV1 string
type ExecutionDispositionV1 string

// IVCUpdateEnvelopeV1 commits to the alleged proof update and public inputs.
// The structural transcript never verifies the proof or its setup.
type IVCUpdateEnvelopeV1 struct {
	ProtocolRef string
	ProtocolVersion string
	ParametersDigest string
	VerificationKeyDigest string
	PriorProofDigest string
	NextProofDigest string
	ProofArtifactDigest string
	PublicInputDigest string
}

// ExecutionTransitionV1 is an immutable computation step. Correction and
// revocation steps append new state; they never rewrite earlier transcript
// events, oracle receipts, state heads or proof heads.
type ExecutionTransitionV1 struct {
	Kind ExecutionStepKindV1
	StepID string
	ProgramDigest string
	EvaluatorVersionDigest string
	PolicyVersionDigest string
	QueryPlanDigest string
	InputStateDigest string
	OutputStateDigest string
	OutputDigest string
	ReceiptFrontierSequence uint64
	ReceiptFrontierDigest string
	ConsumedReceiptDigests []string
	SupersedesStepIDs []string
	IVCUpdate IVCUpdateEnvelopeV1
}

type ExecutionEventV1 struct {
	LogSequence uint64
	EventID string
	PreviousEventDigest string
	Outcome ExecutionEventOutcomeV1
	Transition *ExecutionTransitionV1
	ReconcilesEventID string
	EventDigest string
}

type ExecutionReplayResultV1 struct {
	Disposition ExecutionDispositionV1
	Reason string
	ScopeDigest string
	ReceiptChainDigest string
	ExpectedReceiptCount uint64
	ObservedReceiptCount uint64
	ExpectedReceiptTailDigest string
	SubmittedReceiptsDigest string
	ExpectedEventCount uint64
	ObservedEventCount uint64
	GenesisEventDigest string
	ObservedTailDigest string
	StepChainDigest string
	StateHeadDigest string
	ProofHeadDigest string
	PendingEventID string
	PendingStepID string
	MutationBlocked bool
	AppliedStepIDs []string
	ProofVerification string
	OracleSignatureAuthentication string
	OracleKeyEpochAuthentication string
	AuthorityGrantAuthentication string
	SourceUniverseCompleteness string
	AnswerTruth string
	SubmittedEventsDigest string
	CertificateDigest string
	replaySeal string
}

type executionActiveState struct {
	stateDigest string
	proofDigest string
	stepHeadDigest string
	receiptFrontierSequence uint64
}

type executionPendingState struct {
	event CapabilityEventV1
	transition ExecutionTransitionV1
	digest string
}

func executionGenesisEventDigest(scopeDigest string) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-EXECUTION-GENESIS-V1")
	writeString(&b, scopeDigest)
	return digestBytes(b.Bytes())
}

func executionGenesisStateDigest(scopeDigest string) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-EXECUTION-STATE-GENESIS-V1")
	writeString(&b, scopeDigest)
	return digestBytes(b.Bytes())
}

func executionGenesisProofDigest(scopeDigest string) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-IVC-PROOF-GENESIS-V1")
	writeString(&b, scopeDigest)
	return digestBytes(b.Bytes())
}

func canonicalDigestSet(values []string, limit int, label string) ([]string, error) {
	if len(values) > limit {
		return nil, fmt.Errorf("%s count exceeds %d", label, limit)
	}
	canonical := append([]string(nil), values...)
	for _, value := range canonical {
		if !validDigest(value) {
			return nil, fmt.Errorf("%s contains a malformed digest", label)
		}
	}
	sort.Strings(canonical)
	for i := 1; i < len(canonical); i++ {
		if canonical[i] == canonical[i-1] {
			return nil, fmt.Errorf("%s contains a duplicate digest", label)
		}
	}
	return canonical, nil
}

func canonicalStepIDs(values []string) ([]string, error) {
	if len(values) > MaxEvidenceReferences {
		return nil, errors.New("superseded step count exceeds configured limit")
	}
	canonical := append([]string(nil), values...)
	for _, value := range canonical {
		if err := requireIdentifier("superseded step ID", value); err != nil {
			return nil, err
		}
	}
	sort.Strings(canonical)
	for i := 1; i < len(canonical); i++ {
		if canonical[i] == canonical[i-1] {
			return nil, errors.New("duplicate superseded step ID")
		}
	}
	return canonical, nil
}

func ExecutionPublicInputDigestV1(scope OracleScopeV1, transition ExecutionTransitionV1) (string, error) {
	scopeDigest, err := ScopeDigestV1(scope)
	if err != nil {
		return "", err
	}
	if err := requireIdentifier("execution step ID", transition.StepID); err != nil {
		return "", err
	}
	for _, value := range []string{
		transition.ProgramDigest, transition.EvaluatorVersionDigest,
		transition.PolicyVersionDigest, transition.QueryPlanDigest,
		transition.InputStateDigest, transition.OutputStateDigest,
		transition.OutputDigest, transition.ReceiptFrontierDigest,
	} {
		if !validDigest(value) {
			return "", errors.New("execution public input contains a malformed digest")
		}
	}
	receipts, err := canonicalDigestSet(transition.ConsumedReceiptDigests, MaxEvidenceReferences, "consumed oracle receipt set")
	if err != nil {
		return "", err
	}
	supersedes, err := canonicalStepIDs(transition.SupersedesStepIDs)
	if err != nil {
		return "", err
	}
	return executionPublicInputDigestFromScopeDigestV1(scopeDigest, transition, receipts, supersedes)
}

func executionPublicInputDigestFromScopeDigestV1(scopeDigest string, transition ExecutionTransitionV1, receipts, supersedes []string) (string, error) {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-IVC-PUBLIC-INPUTS-V1")
	writeString(&b, scopeDigest)
	for _, value := range []string{
		string(transition.Kind),
		transition.StepID,
		transition.ProgramDigest,
		transition.EvaluatorVersionDigest,
		transition.PolicyVersionDigest,
		transition.QueryPlanDigest,
		transition.InputStateDigest,
		transition.OutputStateDigest,
		transition.OutputDigest,
	} {
		writeString(&b, value)
	}
	writeUint64(&b, transition.ReceiptFrontierSequence)
	writeString(&b, transition.ReceiptFrontierDigest)
	writeUint32(&b, uint32(len(receipts)))
	for _, digest := range receipts {
		writeString(&b, digest)
	}
	writeUint32(&b, uint32(len(supersedes)))
	for _, stepID := range supersedes {
		writeString(&b, stepID)
	}
	return digestBytes(b.Bytes()), nil
}

func executionTransitionDigestV1(scopeDigest string, transition ExecutionTransitionV1) (string, error) {
	receipts, err := canonicalDigestSet(transition.ConsumedReceiptDigests, MaxEvidenceReferences, "consumed oracle receipt set")
	if err != nil {
		return "", err
	}
	supersedes, err := canonicalStepIDs(transition.SupersedesStepIDs)
	if err != nil {
		return "", err
	}
	publicFields, err := executionPublicInputDigestFromScopeDigestV1(scopeDigest, transition, receipts, supersedes)
	if err != nil {
		return "", err
	}
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-EXECUTION-TRANSITION-V1")
	writeString(&b, publicFields)
	for _, value := range []string{
		transition.IVCUpdate.ProtocolRef,
		transition.IVCUpdate.ProtocolVersion,
		transition.IVCUpdate.ParametersDigest,
		transition.IVCUpdate.VerificationKeyDigest,
		transition.IVCUpdate.PriorProofDigest,
		transition.IVCUpdate.NextProofDigest,
		transition.IVCUpdate.ProofArtifactDigest,
		transition.IVCUpdate.PublicInputDigest,
	} {
		writeString(&b, value)
	}
	return digestBytes(b.Bytes()), nil
}


func validateExecutionTransition(
	scopeDigest string,
	scope OracleScopeV1,
	transition ExecutionTransitionV1,
	stateHead string,
	proofHead string,
	receiptSequence map[string]uint64,
	receiptFrontiers []string,
	appliedStepIDs map[string]struct{},
	appliedStepFrontier map[string]uint64,
	currentReceiptFrontier uint64,
	reservedStepIDs map[string]struct{},
	checkHeads bool,
) error {
	if err := requireIdentifier("execution step ID", transition.StepID); err != nil {
		return err
	}
	for _, field := range []struct{ label, value string }{
		{"program digest", transition.ProgramDigest},
		{"evaluator version digest", transition.EvaluatorVersionDigest},
		{"policy version digest", transition.PolicyVersionDigest},
		{"query-plan digest", transition.QueryPlanDigest},
		{"input state digest", transition.InputStateDigest},
		{"output state digest", transition.OutputStateDigest},
		{"output digest", transition.OutputDigest},
	} {
		if !validDigest(field.value) {
			return fmt.Errorf("%s is malformed", field.label)
		}
	}
	if !validDigest(transition.ReceiptFrontierDigest) {
		return errors.New("receipt frontier digest is malformed")
	}
	if transition.ReceiptFrontierSequence >= uint64(len(receiptFrontiers)) {
		return errors.New("receipt frontier sequence exceeds verified receipt chain")
	}
	if transition.ReceiptFrontierSequence < currentReceiptFrontier {
		return errors.New("execution receipt frontier regresses relative to the current state/proof head")
	}
	if transition.ReceiptFrontierDigest != receiptFrontiers[transition.ReceiptFrontierSequence] {
		return errors.New("receipt frontier digest does not match the selected verified receipt-chain prefix")
	}
	if transition.IVCUpdate.ProtocolRef == "" || transition.IVCUpdate.ProtocolVersion == "" {
		return errors.New("IVC protocol reference/version is incomplete")
	}
	if len(transition.IVCUpdate.ProtocolRef) > MaxIdentifierBytes ||
		len(transition.IVCUpdate.ProtocolVersion) > MaxIdentifierBytes {
		return errors.New("IVC protocol reference/version exceeds byte limit")
	}
	for _, field := range []struct{ label, value string }{
		{"IVC parameters digest", transition.IVCUpdate.ParametersDigest},
		{"IVC verification-key digest", transition.IVCUpdate.VerificationKeyDigest},
		{"prior IVC proof digest", transition.IVCUpdate.PriorProofDigest},
		{"next IVC proof digest", transition.IVCUpdate.NextProofDigest},
		{"IVC proof artifact digest", transition.IVCUpdate.ProofArtifactDigest},
		{"IVC public-input digest", transition.IVCUpdate.PublicInputDigest},
	} {
		if !validDigest(field.value) {
			return fmt.Errorf("%s is malformed", field.label)
		}
	}

	switch transition.Kind {
	case StepApply:
		if len(transition.SupersedesStepIDs) != 0 {
			return errors.New("Apply step must not supersede prior steps")
		}
	case StepRecomputeAfterCorrection, StepCompensate, StepRevokeDerivedState:
		if len(transition.SupersedesStepIDs) == 0 {
			return errors.New("correction/compensation/revocation step must name superseded steps")
		}
	default:
		return errors.New("unknown execution step kind")
	}
	receipts, err := canonicalDigestSet(transition.ConsumedReceiptDigests, MaxEvidenceReferences, "consumed oracle receipt set")
	if err != nil {
		return err
	}
	if len(receipts) == 0 {
		return errors.New("execution step must consume at least one oracle receipt")
	}
	for _, receiptDigest := range receipts {
		sequence, exists := receiptSequence[receiptDigest]
		if !exists {
			return errors.New("execution step consumes a receipt outside the verified receipt chain")
		}
		if sequence > transition.ReceiptFrontierSequence {
			return errors.New("execution step consumes a receipt beyond its declared receipt frontier")
		}
	}
	supersedes, err := canonicalStepIDs(transition.SupersedesStepIDs)
	if err != nil {
		return err
	}
	publicInputsDigest, err := executionPublicInputDigestFromScopeDigestV1(scopeDigest, transition, receipts, supersedes)
	if err != nil {
		return err
	}
	if transition.IVCUpdate.PublicInputDigest != publicInputsDigest {
		return errors.New("IVC public-input digest does not match the exact execution step")
	}
	for _, stepID := range supersedes {
		if _, exists := appliedStepIDs[stepID]; !exists {
			return errors.New("execution step supersedes an unknown or unapplied prior step")
		}
		if transition.Kind != StepApply && transition.ReceiptFrontierSequence <= appliedStepFrontier[stepID] {
			return errors.New("correction/compensation/revocation must consume a later oracle receipt frontier than the superseded step")
		}
	}
	if checkHeads {
		if transition.InputStateDigest != stateHead {
			return errors.New("execution input-state digest does not match the current state head")
		}
		if transition.IVCUpdate.PriorProofDigest != proofHead {
			return errors.New("IVC prior-proof digest does not match the current proof head")
		}
	}
	_ = scope
	_ = reservedStepIDs
	return nil
}

func ExecutionEventDigestV1(scopeDigest string, event ExecutionEventV1) (string, error) {
	if event.LogSequence == 0 {
		return "", errors.New("execution event sequence must start at one")
	}
	if err := requireIdentifier("execution event ID", event.EventID); err != nil {
		return "", err
	}
	if !validDigest(scopeDigest) || !validDigest(event.PreviousEventDigest) {
		return "", errors.New("execution scope or previous-event digest is malformed")
	}
	if len(event.ReconcilesEventID) > MaxIdentifierBytes {
		return "", errors.New("execution reconciliation reference exceeds byte limit")
	}
	var transitionDigest string
	if event.Transition == nil {
		transitionDigest = ""
	} else {
		var err error
		transitionDigest, err = executionTransitionDigestV1(scopeDigest, *event.Transition)
		if err != nil {
			return "", err
		}
	}
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-EXECUTION-EVENT-V1")
	writeString(&b, scopeDigest)
	writeUint64(&b, event.LogSequence)
	writeString(&b, event.EventID)
	writeString(&b, event.PreviousEventDigest)
	writeString(&b, string(event.Outcome))
	writeString(&b, transitionDigest)
	writeString(&b, event.ReconcilesEventID)
	return digestBytes(b.Bytes()), nil
}

func rawExecutionSubmissionDigest(events []ExecutionEventV1) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-EXECUTION-SUBMISSION-V1")
	writeUint64(&b, uint64(len(events)))
	limit := len(events)
	if limit > MaxExecutionEvents {
		limit = MaxExecutionEvents
	}
	for _, event := range events[:limit] {
		writeUint64(&b, event.LogSequence)
		writeString(&b, event.EventID)
		writeString(&b, event.PreviousEventDigest)
		writeString(&b, string(event.Outcome))
		writeString(&b, event.ReconcilesEventID)
		writeString(&b, event.EventDigest)
		if event.Transition != nil {
			digest, err := executionTransitionDigestV1("raw", *event.Transition)
			if err == nil {
				writeString(&b, digest)
			} else {
				writeString(&b, "INVALID-TRANSITION")
			}
		}
	}
	return digestBytes(b.Bytes())
}

func preflightExecutionEvents(events []ExecutionEventV1) (string, string) {
	if len(events) > MaxExecutionEvents {
		return "execution event count exceeds configured limit", digestBytes([]byte(fmt.Sprintf("MYCELIX-EXECUTION-COUNT-OVERLIMIT|%d", len(events))))
	}
	var total uint64
	add := func(value string) bool {
		if len(value) > MaxIdentifierBytes || uint64(len(value)) > MaxExecutionEncodingBytes-total {
			return false
		}
		total += uint64(len(value))
		return true
	}
	for index, event := range events {
		for _, value := range []string{event.EventID, event.PreviousEventDigest, string(event.Outcome), event.ReconcilesEventID, event.EventDigest} {
			if !add(value) {
				return fmt.Sprintf("execution event %d exceeds encoded-field budget", index), digestBytes([]byte(fmt.Sprintf("MYCELIX-EXECUTION-OVERSIZE|%d|%d", len(events), index)))
			}
		}
		if event.Transition == nil {
			continue
		}
		transition := event.Transition
		for _, value := range []string{
			string(transition.Kind), transition.StepID,
			transition.ProgramDigest, transition.EvaluatorVersionDigest,
			transition.PolicyVersionDigest, transition.QueryPlanDigest,
			transition.InputStateDigest, transition.OutputStateDigest,
			transition.OutputDigest, transition.ReceiptFrontierDigest,
			transition.IVCUpdate.ProtocolRef, transition.IVCUpdate.ProtocolVersion,
			transition.IVCUpdate.ParametersDigest, transition.IVCUpdate.VerificationKeyDigest,
			transition.IVCUpdate.PriorProofDigest, transition.IVCUpdate.NextProofDigest,
			transition.IVCUpdate.ProofArtifactDigest, transition.IVCUpdate.PublicInputDigest,
		} {
			if !add(value) {
				return fmt.Sprintf("execution transition %d exceeds encoded-field budget", index), digestBytes([]byte(fmt.Sprintf("MYCELIX-EXECUTION-TRANSITION-OVERSIZE|%d|%d", len(events), index)))
			}
		}
		if len(transition.ConsumedReceiptDigests) > MaxEvidenceReferences || len(transition.SupersedesStepIDs) > MaxEvidenceReferences {
			return fmt.Sprintf("execution transition %d collection count exceeds configured limit", index), digestBytes([]byte(fmt.Sprintf("MYCELIX-EXECUTION-COLLECTION-OVERLIMIT|%d|%d", len(events), index)))
		}
		for _, value := range transition.ConsumedReceiptDigests {
			if !add(value) {
				return fmt.Sprintf("execution transition %d receipt set exceeds encoded-field budget", index), digestBytes([]byte(fmt.Sprintf("MYCELIX-EXECUTION-RECEIPTS-OVERSIZE|%d|%d", len(events), index)))
			}
		}
		for _, value := range transition.SupersedesStepIDs {
			if !add(value) {
				return fmt.Sprintf("execution transition %d supersession set exceeds encoded-field budget", index), digestBytes([]byte(fmt.Sprintf("MYCELIX-EXECUTION-SUPERSEDES-OVERSIZE|%d|%d", len(events), index)))
			}
		}
	}
	return "", rawExecutionSubmissionDigest(events)
}

func applyExecutionTransition(state *executionActiveState, transition ExecutionTransitionV1, digest string) {
	state.stateDigest = transition.OutputStateDigest
	state.proofDigest = transition.IVCUpdate.NextProofDigest
	state.stepHeadDigest = digest
	state.receiptFrontierSequence = transition.ReceiptFrontierSequence
}

func executionReplayCertificateDigest(result ExecutionReplayResultV1) string {
	var b bytes.Buffer
	writeString(&b, "MYCELIX-ORACLE-EXECUTION-REPLAY-RESULT-V1")
	writeString(&b, string(result.Disposition))
	writeString(&b, result.Reason)
	writeString(&b, result.ScopeDigest)
	writeString(&b, result.ReceiptChainDigest)
	writeUint64(&b, result.ExpectedReceiptCount)
	writeUint64(&b, result.ObservedReceiptCount)
	writeString(&b, result.ExpectedReceiptTailDigest)
	writeString(&b, result.SubmittedReceiptsDigest)
	writeUint64(&b, result.ExpectedEventCount)
	writeUint64(&b, result.ObservedEventCount)
	writeString(&b, result.GenesisEventDigest)
	writeString(&b, result.ObservedTailDigest)
	writeString(&b, result.StepChainDigest)
	writeString(&b, result.StateHeadDigest)
	writeString(&b, result.ProofHeadDigest)
	writeString(&b, result.PendingEventID)
	writeString(&b, result.PendingStepID)
	if result.MutationBlocked { b.WriteByte(1) } else { b.WriteByte(0) }
	writeString(&b, result.ProofVerification)
	writeString(&b, result.OracleSignatureAuthentication)
	writeString(&b, result.OracleKeyEpochAuthentication)
	writeString(&b, result.AuthorityGrantAuthentication)
	writeString(&b, result.SourceUniverseCompleteness)
	writeString(&b, result.AnswerTruth)
	writeString(&b, result.SubmittedEventsDigest)
	writeUint32(&b, uint32(len(result.AppliedStepIDs)))
	for _, stepID := range result.AppliedStepIDs { writeString(&b, stepID) }
	return digestBytes(b.Bytes())
}

func finalizeExecutionReplay(result ExecutionReplayResultV1) ExecutionReplayResultV1 {
	result.CertificateDigest = executionReplayCertificateDigest(result)
	result.replaySeal = result.CertificateDigest
	return result
}

func executionReplayIntegrityValid(result ExecutionReplayResultV1) bool {
	return result.replaySeal != "" &&
		result.CertificateDigest == result.replaySeal &&
		result.CertificateDigest == executionReplayCertificateDigest(result)
}

// ReplayOracleExecutionTranscriptV1 reconstructs the exact state and IVC proof
// heads from an append-only execution-event chain. Receipt and IVC cryptographic
// verification remain NotEvaluated; this verifies transcript structure only.
func ReplayOracleExecutionTranscriptV1(
	scope OracleScopeV1,
	receipts []OracleReceiptV1,
	expectedReceiptCount uint64,
	expectedReceiptTail string,
	events []ExecutionEventV1,
	expectedEventCount uint64,
	expectedEventTail string,
) ExecutionReplayResultV1 {
	result := ExecutionReplayResultV1{
		Disposition: ExecutionChainIndeterminate,
		Reason: "execution transcript is indeterminate",
		ExpectedEventCount: expectedEventCount,
		ObservedEventCount: uint64(len(events)),
		ProofVerification: NotEvaluated,
		OracleSignatureAuthentication: NotEvaluated,
		OracleKeyEpochAuthentication: NotEvaluated,
		AuthorityGrantAuthentication: NotEvaluated,
		SourceUniverseCompleteness: NotEvaluated,
		AnswerTruth: NotEvaluated,
	}
	result.ExpectedReceiptCount = expectedReceiptCount
	result.ObservedReceiptCount = uint64(len(receipts))
	result.ExpectedReceiptTailDigest = expectedReceiptTail
	if reason, submissionDigest := preflightExecutionEvents(events); reason != "" {
		result.Reason = reason
		result.SubmittedEventsDigest = submissionDigest
		return finalizeExecutionReplay(result)
	}
	result.SubmittedEventsDigest = rawExecutionSubmissionDigest(events)
	receiptResult := VerifyOracleReceiptChainV1(scope, receipts, expectedReceiptCount, expectedReceiptTail)
	result.SubmittedReceiptsDigest = receiptResult.SubmittedReceiptsDigest
	if receiptResult.Disposition != ChainStructurallyValid {
		result.Reason = "oracle receipt chain is not structurally valid: " + receiptResult.Reason
		return finalizeExecutionReplay(result)
	}
	result.ScopeDigest = receiptResult.ScopeDigest
	result.ReceiptChainDigest = receiptResult.ReceiptChainDigest
	if expectedEventCount > MaxExecutionEvents || uint64(len(events)) > MaxExecutionEvents {
		result.Reason = "execution event count exceeds configured limit"
		return finalizeExecutionReplay(result)
	}
	if !validDigest(expectedEventTail) {
		result.Reason = "expected execution event tail digest is malformed"
		return finalizeExecutionReplay(result)
	}
	if uint64(len(events)) != expectedEventCount {
		result.Reason = "observed execution event count differs from expected count"
		return finalizeExecutionReplay(result)
	}

	scopeDigest, err := ScopeDigestV1(scope)
	if err != nil || scopeDigest != result.ScopeDigest {
		result.Reason = "execution oracle scope failed canonical verification"
		return finalizeExecutionReplay(result)
	}
	receiptFrontiers := make([]string, len(receipts)+1)
	receiptSequence := make(map[string]uint64, len(receipts))
	receiptFrontiers[0] = genesisDigest(scopeDigest)
	for index, receipt := range receipts {
		sequence := uint64(index + 1)
		receiptFrontiers[sequence] = receipt.ReceiptDigest
		receiptSequence[receipt.ReceiptDigest] = sequence
	}
	state := executionActiveState{
		stateDigest: executionGenesisStateDigest(scopeDigest),
		proofDigest: executionGenesisProofDigest(scopeDigest),
		stepHeadDigest: executionGenesisEventDigest(scopeDigest),
	}
	result.GenesisEventDigest = state.stepHeadDigest
	result.StateHeadDigest = state.stateDigest
	result.ProofHeadDigest = state.proofDigest
	running := state.stepHeadDigest
	seenEventIDs := make(map[string]struct{}, len(events))
	reservedStepIDs := make(map[string]struct{}, len(events))
	appliedStepIDs := make(map[string]struct{}, len(events))
	appliedStepFrontier := make(map[string]uint64, len(events))
	var pending *executionPendingState
	stepDigests := make([]string, 0, len(events))

	for index, event := range events {
		expectedSequence := uint64(index + 1)
		if event.LogSequence != expectedSequence {
			result.Reason = fmt.Sprintf("execution event sequence discontinuity at index %d", index)
			return finalizeExecutionReplay(result)
		}
		if _, exists := seenEventIDs[event.EventID]; exists {
			result.Reason = "duplicate execution event ID"
			return finalizeExecutionReplay(result)
		}
		if event.PreviousEventDigest != running {
			result.Reason = fmt.Sprintf("execution previous-event digest mismatch at sequence %d", expectedSequence)
			return finalizeExecutionReplay(result)
		}
		calculated, err := ExecutionEventDigestV1(scopeDigest, event)
		if err != nil {
			result.Reason = fmt.Sprintf("execution event %d invalid: %v", expectedSequence, err)
			return finalizeExecutionReplay(result)
		}
		if calculated != event.EventDigest {
			result.Reason = fmt.Sprintf("execution event digest mismatch at sequence %d", expectedSequence)
			return finalizeExecutionReplay(result)
		}
		if pending != nil && event.Outcome != ExecutionReconciledApplied && event.Outcome != ExecutionReconciledNotApplied {
			result.Reason = "execution mutation attempted while outcome is unknown; reconcile first"
			return finalizeExecutionReplay(result)
		}

		switch event.Outcome {
		case ExecutionCommitted:
			if event.Transition == nil || event.ReconcilesEventID != "" || pending != nil {
				result.Reason = "malformed committed execution event"
				return finalizeExecutionReplay(result)
			}
			if _, exists := reservedStepIDs[event.Transition.StepID]; exists {
				result.Reason = "duplicate execution step ID"
				return finalizeExecutionReplay(result)
			}
			if err := validateExecutionTransition(scopeDigest, scope, *event.Transition, state.stateDigest, state.proofDigest, receiptSequence, receiptFrontiers, appliedStepIDs, appliedStepFrontier, state.receiptFrontierSequence, reservedStepIDs, true); err != nil {
				result.Reason = err.Error()
				return finalizeExecutionReplay(result)
			}
			stepDigest, _ := executionTransitionDigestV1(scopeDigest, *event.Transition)
			reservedStepIDs[event.Transition.StepID] = struct{}{}
			appliedStepIDs[event.Transition.StepID] = struct{}{}
			appliedStepFrontier[event.Transition.StepID] = event.Transition.ReceiptFrontierSequence
			applyExecutionTransition(&state, *event.Transition, stepDigest)
			result.AppliedStepIDs = append(result.AppliedStepIDs, event.Transition.StepID)
			stepDigests = append(stepDigests, stepDigest)

		case ExecutionUnknown:
			if event.Transition == nil || event.ReconcilesEventID != "" || pending != nil {
				result.Reason = "malformed or nested unknown execution outcome"
				return finalizeExecutionReplay(result)
			}
			if _, exists := reservedStepIDs[event.Transition.StepID]; exists {
				result.Reason = "duplicate execution step ID"
				return finalizeExecutionReplay(result)
			}
			if err := validateExecutionTransition(scopeDigest, scope, *event.Transition, state.stateDigest, state.proofDigest, receiptSequence, receiptFrontiers, appliedStepIDs, appliedStepFrontier, state.receiptFrontierSequence, reservedStepIDs, true); err != nil {
				result.Reason = "unknown execution intent is invalid: " + err.Error()
				return finalizeExecutionReplay(result)
			}
			stepDigest, _ := executionTransitionDigestV1(scopeDigest, *event.Transition)
			reservedStepIDs[event.Transition.StepID] = struct{}{}
			pending = &executionPendingState{
				event: CapabilityEventV1{EventID: event.EventID},
				transition: *event.Transition,
				digest: stepDigest,
			}
			stepDigests = append(stepDigests, stepDigest)

		case ExecutionReconciledApplied, ExecutionReconciledNotApplied:
			if event.Transition != nil || pending == nil || event.ReconcilesEventID != pending.event.EventID {
				result.Reason = "execution reconciliation does not match the pending unknown event"
				return finalizeExecutionReplay(result)
			}
			if event.Outcome == ExecutionReconciledApplied {
				if err := validateExecutionTransition(scopeDigest, scope, pending.transition, state.stateDigest, state.proofDigest, receiptSequence, receiptFrontiers, appliedStepIDs, appliedStepFrontier, state.receiptFrontierSequence, reservedStepIDs, true); err != nil {
					result.Reason = "reconciled execution transition is no longer valid: " + err.Error()
					return finalizeExecutionReplay(result)
				}
				applyExecutionTransition(&state, pending.transition, pending.digest)
				appliedStepIDs[pending.transition.StepID] = struct{}{}
				appliedStepFrontier[pending.transition.StepID] = pending.transition.ReceiptFrontierSequence
				result.AppliedStepIDs = append(result.AppliedStepIDs, pending.transition.StepID)
			}
			pending = nil
		default:
			result.Reason = "unknown execution event outcome"
			return finalizeExecutionReplay(result)
		}
		seenEventIDs[event.EventID] = struct{}{}
		running = calculated
	}
	result.ObservedTailDigest = running
	if running != expectedEventTail {
		result.Reason = "observed execution event tail differs from expected anchor"
		return finalizeExecutionReplay(result)
	}
	result.StateHeadDigest = state.stateDigest
	result.ProofHeadDigest = state.proofDigest
	if pending != nil {
		result.PendingEventID = pending.event.EventID
		result.PendingStepID = pending.transition.StepID
		result.MutationBlocked = true
		result.Reason = "execution/proof update outcome is unknown; no further step or proof mutation until reconciliation"
	} else {
		result.Disposition = ExecutionChainStructurallyValid
		result.Reason = "execution transcript structurally replays; oracle signatures, source completeness and IVC proof validity remain NotEvaluated"
	}
	var chain bytes.Buffer
	writeString(&chain, "MYCELIX-ORACLE-EXECUTION-CHAIN-V1")
	writeString(&chain, scopeDigest)
	writeString(&chain, result.ReceiptChainDigest)
	for _, digest := range stepDigests { writeString(&chain, digest) }
	result.StepChainDigest = digestBytes(chain.Bytes())
	return finalizeExecutionReplay(result)
}
