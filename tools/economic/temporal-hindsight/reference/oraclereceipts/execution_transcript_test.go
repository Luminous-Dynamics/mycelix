package oraclereceipts

import (
	"fmt"
	"strings"
	"testing"
)

func executionFixtureScope() OracleScopeV1 {
	return OracleScopeV1{
		SourceRef: "source:execution",
		SnapshotRef: "snapshot:execution",
		RegistryRef: "registry:execution",
		RegistryEpoch: "epoch:execution",
		QueryProfileRef: "query:execution",
		QueryDigest: testDigestA,
		InputCommitmentDigest: testDigestB,
		EvidenceFrontierDigest: testDigestC,
	}
}

func executionGenesis(t testing.TB, scope OracleScopeV1) (string, string, string) {
	t.Helper()
	scopeDigest, err := ScopeDigestV1(scope)
	if err != nil {
		t.Fatal(err)
	}
	return scopeDigest, executionGenesisEventDigest(scopeDigest), executionGenesisStateDigest(scopeDigest)
}

func executionTransition(t testing.TB, scope OracleScopeV1, scopeDigest, receiptDigest string, receiptSequence uint64, inputState, priorProof, stepID string, kind ExecutionStepKindV1, outputState, nextProof string, supersedes ...string) ExecutionTransitionV1 {
	t.Helper()
	transition := ExecutionTransitionV1{
		Kind: kind,
		StepID: stepID,
		ProgramDigest: testDigestA,
		EvaluatorVersionDigest: testDigestB,
		PolicyVersionDigest: testDigestC,
		QueryPlanDigest: testDigestA,
		InputStateDigest: inputState,
		OutputStateDigest: outputState,
		OutputDigest: testDigestB,
		ReceiptFrontierSequence: receiptSequence,
		ReceiptFrontierDigest: receiptDigest,
		ConsumedReceiptDigests: []string{receiptDigest},
		SupersedesStepIDs: append([]string(nil), supersedes...),
		IVCUpdate: IVCUpdateEnvelopeV1{
			ProtocolRef: "ivc:incremental",
			ProtocolVersion: "v1",
			ParametersDigest: testDigestA,
			VerificationKeyDigest: testDigestB,
			PriorProofDigest: priorProof,
			NextProofDigest: nextProof,
			ProofArtifactDigest: testDigestC,
		},
	}
	var err error
	transition.IVCUpdate.PublicInputDigest, err = ExecutionPublicInputDigestV1(scope, transition)
	if err != nil {
		t.Fatal(err)
	}
	_ = scopeDigest
	return transition
}

func sealExecutionEvent(t testing.TB, scope OracleScopeV1, sequence uint64, eventID, previous string, outcome ExecutionEventOutcomeV1, transition *ExecutionTransitionV1, reconciles string) ExecutionEventV1 {
	t.Helper()
	scopeDigest, err := ScopeDigestV1(scope)
	if err != nil {
		t.Fatal(err)
	}
	event := ExecutionEventV1{
		LogSequence: sequence,
		EventID: eventID,
		PreviousEventDigest: previous,
		Outcome: outcome,
		Transition: transition,
		ReconcilesEventID: reconciles,
	}
	event.EventDigest, err = ExecutionEventDigestV1(scopeDigest, event)
	if err != nil {
		t.Fatal(err)
	}
	return event
}

func appendExecutionEvent(t testing.TB, scope OracleScopeV1, events []ExecutionEventV1, id string, outcome ExecutionEventOutcomeV1, transition *ExecutionTransitionV1, reconciles string) []ExecutionEventV1 {
	t.Helper()
	_, genesis, _ := executionGenesis(t, scope)
	previous := genesis
	if len(events) > 0 {
		previous = events[len(events)-1].EventDigest
	}
	return append(events, sealExecutionEvent(t, scope, uint64(len(events)+1), id, previous, outcome, transition, reconciles))
}

func executionReceipts(t *testing.T, scope OracleScopeV1, n int) []OracleReceiptV1 {
	t.Helper()
	var receipts []OracleReceiptV1
	for i := 0; i < n; i++ {
		disposition := DispositionAffirm
		payload := fmt.Sprintf("answer:%d", i)
		receipts = appendReceipt(t, scope, receipts, fmt.Sprintf("receipt:%d", i+1), disposition, payload)
		if i == 1 {
			receipts[i].AnswerDisposition = DispositionReject
			receipts[i].SupersedesReceiptID = receipts[0].ReceiptID
			receipts[i].SignedMessageDigest, _ = SignableDigestV1(scope, receipts[i])
			receipts[i].ReceiptDigest, _ = ReceiptDigestV1(scope, receipts[i])
		}
	}
	return receipts
}

func replayExecution(t *testing.T, scope OracleScopeV1, receipts []OracleReceiptV1, events []ExecutionEventV1) ExecutionReplayResultV1 {
	t.Helper()
	receiptTail := testGenesis(t, scope)
	if len(receipts) > 0 {
		receiptTail = receipts[len(receipts)-1].ReceiptDigest
	}
	_, eventGenesis, _ := executionGenesis(t, scope)
	eventTail := eventGenesis
	if len(events) > 0 {
		eventTail = events[len(events)-1].EventDigest
	}
	return ReplayOracleExecutionTranscriptV1(scope, receipts, uint64(len(receipts)), receiptTail, events, uint64(len(events)), eventTail)
}

func buildExecutionChain(t testing.TB, scope OracleScopeV1, receipts []OracleReceiptV1, count int) []ExecutionEventV1 {
	t.Helper()
	scopeDigest, _, stateHead := executionGenesis(t, scope)
	proofHead := executionGenesisProofDigest(scopeDigest)
	receiptTail := receipts[0].ReceiptDigest
	var events []ExecutionEventV1
	for i := 0; i < count; i++ {
		outputState := digestBytes([]byte(fmt.Sprintf("state:%d", i+1)))
		nextProof := digestBytes([]byte(fmt.Sprintf("proof:%d", i+1)))
		transition := executionTransition(t, scope, scopeDigest, receiptTail, 1, stateHead, proofHead,
			fmt.Sprintf("step:%d", i+1), StepApply, outputState, nextProof)
		events = appendExecutionEvent(t, scope, events, fmt.Sprintf("event:%d", i+1), ExecutionCommitted, &transition, "")
		stateHead = outputState
		proofHead = nextProof
	}
	return events
}

func TestExecutionTranscriptBindsStateAndIVCProofHeads(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, eventGenesis, stateGenesis := executionGenesis(t, scope)
	proofGenesis := executionGenesisProofDigest(scopeDigest)
	outputState := testDigestB
	nextProof := testDigestC
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		stateGenesis, proofGenesis, "step:1", StepApply, outputState, nextProof)
	events := appendExecutionEvent(t, scope, nil, "event:1", ExecutionCommitted, &transition, "")
	result := replayExecution(t, scope, receipts, events)
	if result.Disposition != ExecutionChainStructurallyValid {
		t.Fatalf("valid execution transcript rejected: %+v", result)
	}
	if result.StateHeadDigest != outputState || result.ProofHeadDigest != nextProof {
		t.Fatalf("state/proof heads not reconstructed: %+v", result)
	}
	if result.ProofVerification != NotEvaluated ||
		result.OracleSignatureAuthentication != NotEvaluated ||
		result.SourceUniverseCompleteness != NotEvaluated ||
		result.AnswerTruth != NotEvaluated {
		t.Fatalf("structural transcript promoted a trust claim: %+v", result)
	}
	if result.ObservedTailDigest != events[0].EventDigest || result.AppliedStepIDs[0] != "step:1" {
		t.Fatalf("execution event/step head omitted: %+v", result)
	}
	_ = eventGenesis
}

func TestExecutionStepRejectsReceiptOutsideDeclaredPrefix(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:bad-prefix", StepApply, testDigestB, testDigestC)
	transition.ConsumedReceiptDigests = []string{receipts[1].ReceiptDigest}
	transition.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, transition)
	events := appendExecutionEvent(t, scope, nil, "event:bad-prefix", ExecutionCommitted, &transition, "")
	if got := replayExecution(t, scope, receipts, events); got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "beyond its declared receipt frontier") {
		t.Fatalf("receipt beyond declared frontier accepted: %+v", got)
	}
}

func TestExecutionTranscriptRequiresComputedIVCPublicInputs(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:1", StepApply, testDigestB, testDigestC)
	transition.PolicyVersionDigest = testDigestA
	events := appendExecutionEvent(t, scope, nil, "event:1", ExecutionCommitted, &transition, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "public-input digest") {
		t.Fatalf("public-input substitution accepted: %+v", got)
	}
}

func TestExecutionStepRejectsStateOrProofHeadRollback(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		testDigestA, proof, "step:bad-state", StepApply, testDigestB, testDigestC)
	transition.IVCUpdate.PriorProofDigest = testDigestB
	transition.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, transition)
	events := appendExecutionEvent(t, scope, nil, "event:bad-state", ExecutionCommitted, &transition, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate {
		t.Fatalf("state/proof rollback accepted: %+v", got)
	}
	_ = state
}

func TestCorrectionAppendsNewStepAndPreservesPriorStep(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state0 := executionGenesis(t, scope)
	proof0 := executionGenesisProofDigest(scopeDigest)

	original := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state0, proof0, "step:original", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:original", ExecutionCommitted, &original, "")
	originalEventDigest := events[0].EventDigest

	correction := executionTransition(t, scope, scopeDigest, receipts[1].ReceiptDigest, 2,
		testDigestB, testDigestC, "step:recompute", StepRecomputeAfterCorrection, testDigestA, testDigestB, "step:original")
	events = appendExecutionEvent(t, scope, events, "event:recompute", ExecutionCommitted, &correction, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainStructurallyValid ||
		got.StateHeadDigest != testDigestA || got.ProofHeadDigest != testDigestB {
		t.Fatalf("backdated correction recomputation failed: %+v", got)
	}
	if events[0].EventDigest != originalEventDigest || len(got.AppliedStepIDs) != 2 {
		t.Fatal("correction rewrote prior step or dropped step history")
	}
}

func TestCorrectionMustSupersedeAnEarlierAppliedStep(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:correction", StepRecomputeAfterCorrection, testDigestB, testDigestC, "step:missing")
	events := appendExecutionEvent(t, scope, nil, "event:correction", ExecutionCommitted, &transition, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "unknown or unapplied") {
		t.Fatalf("unknown supersession target accepted: %+v", got)
	}
}

func TestUnknownExecutionOutcomeBlocksMutationUntilReconciled(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	pending := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:pending", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:unknown", ExecutionUnknown, &pending, "")
	blocked := replayExecution(t, scope, receipts, events)
	if blocked.Disposition != ExecutionChainIndeterminate || !blocked.MutationBlocked ||
		blocked.PendingEventID != "event:unknown" || blocked.StateHeadDigest != state || blocked.ProofHeadDigest != proof {
		t.Fatalf("unknown execution outcome advanced state or proof: %+v", blocked)
	}
	illegal := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:illegal", StepApply, testDigestC, testDigestA)
	events = appendExecutionEvent(t, scope, events, "event:illegal", ExecutionCommitted, &illegal, "")
	if got := replayExecution(t, scope, receipts, events); got.Disposition != ExecutionChainIndeterminate {
		t.Fatalf("mutation while outcome unknown accepted: %+v", got)
	}
}

func TestAppliedReconciliationCommitsPendingStateAndProof(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	pending := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:pending", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:unknown", ExecutionUnknown, &pending, "")
	events = appendExecutionEvent(t, scope, events, "event:reconcile-applied", ExecutionReconciledApplied, nil, "event:unknown")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainStructurallyValid || got.MutationBlocked ||
		got.StateHeadDigest != testDigestB || got.ProofHeadDigest != testDigestC ||
		len(got.AppliedStepIDs) != 1 || got.AppliedStepIDs[0] != "step:pending" {
		t.Fatalf("applied reconciliation failed: %+v", got)
	}
}

func TestNotAppliedReconciliationDoesNotAdvanceStateOrProof(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	pending := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:pending", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:unknown", ExecutionUnknown, &pending, "")
	events = appendExecutionEvent(t, scope, events, "event:reconcile-not-applied", ExecutionReconciledNotApplied, nil, "event:unknown")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainStructurallyValid || got.MutationBlocked ||
		got.StateHeadDigest != state || got.ProofHeadDigest != proof ||
		len(got.AppliedStepIDs) != 0 {
		t.Fatalf("not-applied reconciliation mutated execution state: %+v", got)
	}
}

func TestForgedOrMutatedExecutionReplayResultRejected(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	events := buildExecutionChain(t, scope, receipts, 1)
	got := replayExecution(t, scope, receipts, events)
	if !executionReplayIntegrityValid(got) {
		t.Fatal("genuine execution replay result failed its own seal")
	}
	got.ProofHeadDigest = testDigestC
	if executionReplayIntegrityValid(got) {
		t.Fatal("mutated proof head retained a valid replay seal")
	}
}

func TestExecutionFailureReceiptBindsSubmittedEvents(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	makeEvents := func(stepID string) []ExecutionEventV1 {
		transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
			state, proof, stepID, StepApply, testDigestB, testDigestC)
		events := appendExecutionEvent(t, scope, nil, "event:same", ExecutionCommitted, &transition, "")
		events[0].LogSequence = 2
		return events
	}
	first := replayExecution(t, scope, receipts, makeEvents("step:A"))
	second := replayExecution(t, scope, receipts, makeEvents("step:B"))
	if first.Disposition != ExecutionChainIndeterminate || second.Disposition != ExecutionChainIndeterminate ||
		first.SubmittedEventsDigest == second.SubmittedEventsDigest ||
		first.CertificateDigest == second.CertificateDigest {
		t.Fatalf("distinct malformed execution submissions collapsed: first=%+v second=%+v", first, second)
	}
}

func TestExecutionTranscriptDeterministic(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	events := buildExecutionChain(t, scope, receipts, 3)
	first := replayExecution(t, scope, receipts, events)
	for i := 0; i < 100; i++ {
		next := replayExecution(t, scope, receipts, events)
		if next.CertificateDigest != first.CertificateDigest {
			t.Fatal("execution replay certificate is nondeterministic")
		}
	}
}

func BenchmarkReplayOracleExecutionTranscript(b *testing.B) {
	for _, count := range []int{1, 10, 100, 1000, 10000} {
		b.Run(fmt.Sprintf("steps=%d", count), func(b *testing.B) {
			scope := executionFixtureScope()
			receipts := []OracleReceiptV1{
				{
					Sequence: 1, ReceiptID: "receipt:benchmark",
					PreviousReceiptDigest: testGenesisBenchmark(scope),
					OracleRef: "oracle:benchmark", KeyEpoch: "key:benchmark",
					AuthorityGrantRef: "grant:benchmark", AuthorityScopeDigest: testDigestA,
					QueryDigest: scope.QueryDigest, InputCommitmentDigest: scope.InputCommitmentDigest,
					EvidenceFrontierDigest: scope.EvidenceFrontierDigest,
					AnswerSchemaRef: "answer:v1", AnswerDisposition: DispositionAffirm,
					AnswerPayload: []byte("benchmark"), AnswerDigest: AnswerDigestV1([]byte("benchmark")),
					RecordedAtUnixNanos: 100, ValidTime: ValidTimeV1{FromUnixNanos: 1, OpenEnded: true},
					EvidenceReferences: []EvidenceReferenceV1{{Reference:"evidence:1",Kind:"source",Digest:testDigestB}},
					SignatureScheme: "ed25519", SignatureBytes: []byte("signature"),
				},
			}
			receipts[0].SignedMessageDigest, _ = SignableDigestV1(scope, receipts[0])
			receipts[0].ReceiptDigest, _ = ReceiptDigestV1(scope, receipts[0])
			events := buildExecutionChain(b, scope, receipts, count)
			receiptTail := receipts[0].ReceiptDigest
			eventTail := events[len(events)-1].EventDigest
			b.ReportAllocs()
			b.ReportMetric(float64(count), "steps/op")
			b.ResetTimer()
			for i:=0;i<b.N;i++ {
				result := ReplayOracleExecutionTranscriptV1(scope, receipts, 1, receiptTail, events, uint64(count), eventTail)
				if result.Disposition != ExecutionChainStructurallyValid {
					b.Fatalf("execution transcript replay failed: %s", result.Reason)
				}
			}
		})
	}
}
