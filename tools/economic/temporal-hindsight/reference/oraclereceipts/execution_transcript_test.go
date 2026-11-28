package oraclereceipts

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
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
			PriorAdviceCommitmentDigest: executionGenesisAdviceCommitmentDigest(scopeDigest),
			ParametersDigest: testDigestA,
			VerificationKeyDigest: testDigestB,
			PriorProofDigest: priorProof,
			NextProofDigest: nextProof,
			ProofArtifactDigest: testDigestC,
		},
	}
	var err error
	transition.IVCUpdate.AdviceDeltaCommitmentDigest, err = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, transition.ReceiptFrontierSequence, transition.ReceiptFrontierDigest, transition.ConsumedReceiptDigests)
	if err != nil {
		t.Fatal(err)
	}
	transition.IVCUpdate.NextAdviceCommitmentDigest, err = executionNextAdviceCommitmentDigestV1(
		scopeDigest, transition.IVCUpdate.PriorAdviceCommitmentDigest, transition.IVCUpdate.AdviceDeltaCommitmentDigest)
	if err != nil {
		t.Fatal(err)
	}
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
	adviceHead := executionGenesisAdviceCommitmentDigest(scopeDigest)
	receiptTail := receipts[0].ReceiptDigest
	var events []ExecutionEventV1
	for i := 0; i < count; i++ {
		outputState := digestBytes([]byte(fmt.Sprintf("state:%d", i+1)))
		nextProof := digestBytes([]byte(fmt.Sprintf("proof:%d", i+1)))
		transition := executionTransition(t, scope, scopeDigest, receiptTail, 1, stateHead, proofHead,
			fmt.Sprintf("step:%d", i+1), StepApply, outputState, nextProof)
		transition.IVCUpdate.PriorAdviceCommitmentDigest = adviceHead
		transition.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
			scopeDigest, transition.ReceiptFrontierSequence, transition.ReceiptFrontierDigest, transition.ConsumedReceiptDigests)
		transition.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
			scopeDigest, adviceHead, transition.IVCUpdate.AdviceDeltaCommitmentDigest)
		transition.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, transition)
		adviceHead = transition.IVCUpdate.NextAdviceCommitmentDigest
		events = appendExecutionEvent(t, scope, events, fmt.Sprintf("event:%d", i+1), ExecutionCommitted, &transition, "")
		stateHead = outputState
		proofHead = nextProof
	}
	return events
}

func TestExecutionOracleAdviceSharedVectorsV1(t *testing.T) {
	file, err := os.Open("ORACLE_IVC_ADVICE_VECTORS_V1.txt")
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	line := 0
	for scanner.Scan() {
		line++
		fields := strings.Fields(scanner.Text())
		if len(fields) == 0 || strings.HasPrefix(fields[0], "#") {
			continue
		}
		switch fields[0] {
		case "GENESIS":
			if len(fields) != 3 {
				t.Fatalf("invalid GENESIS vector at line %d", line)
			}
			if got := executionGenesisAdviceCommitmentDigest(fields[1]); got != fields[2] {
				t.Fatalf("GENESIS vector mismatch at line %d: %s != %s", line, got, fields[2])
			}
		case "DELTA":
			if len(fields) < 6 {
				t.Fatalf("invalid DELTA vector at line %d", line)
			}
			sequence, err := strconv.ParseUint(fields[2], 10, 64)
			if err != nil {
				t.Fatalf("invalid DELTA sequence at line %d: %v", line, err)
			}
			receiptDigests := fields[4 : len(fields)-1]
			got, err := executionAdviceDeltaCommitmentDigestV1(fields[1], sequence, fields[3], receiptDigests)
			if err != nil {
				t.Fatalf("DELTA vector failed at line %d: %v", line, err)
			}
			if got != fields[len(fields)-1] {
				t.Fatalf("DELTA vector mismatch at line %d: %s != %s", line, got, fields[len(fields)-1])
			}
		case "NEXT":
			if len(fields) != 5 {
				t.Fatalf("invalid NEXT vector at line %d", line)
			}
			got, err := executionNextAdviceCommitmentDigestV1(fields[1], fields[2], fields[3])
			if err != nil {
				t.Fatalf("NEXT vector failed at line %d: %v", line, err)
			}
			if got != fields[4] {
				t.Fatalf("NEXT vector mismatch at line %d: %s != %s", line, got, fields[4])
			}
		default:
			t.Fatalf("unknown advice vector kind at line %d: %s", line, fields[0])
		}
	}
	if err := scanner.Err(); err != nil {
		t.Fatal(err)
	}
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
	if result.AdviceCommitmentDigest != transition.IVCUpdate.NextAdviceCommitmentDigest ||
		result.ReceiptFrontierSequence != 1 {
		t.Fatalf("cumulative advice or receipt frontier not reconstructed: %+v", result)
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
	transition.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, transition.ReceiptFrontierSequence, transition.ReceiptFrontierDigest, transition.ConsumedReceiptDigests)
	transition.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, transition.IVCUpdate.PriorAdviceCommitmentDigest, transition.IVCUpdate.AdviceDeltaCommitmentDigest)
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

func TestExecutionReceiptFrontierCannotRegressAcrossSteps(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)

	first := executionTransition(t, scope, scopeDigest, receipts[1].ReceiptDigest, 2,
		state, proof, "step:frontier-2", StepApply, testDigestB, testDigestC)
	first.ConsumedReceiptDigests = []string{receipts[0].ReceiptDigest, receipts[1].ReceiptDigest}
	first.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, first.ReceiptFrontierSequence, first.ReceiptFrontierDigest, first.ConsumedReceiptDigests)
	first.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, first.IVCUpdate.PriorAdviceCommitmentDigest, first.IVCUpdate.AdviceDeltaCommitmentDigest)
	first.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, first)
	events := appendExecutionEvent(t, scope, nil, "event:frontier-2", ExecutionCommitted, &first, "")

	second := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		testDigestB, testDigestC, "step:frontier-1", StepApply, testDigestA, testDigestB)
	events = appendExecutionEvent(t, scope, events, "event:frontier-1", ExecutionCommitted, &second, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "receipt frontier regresses") {
		t.Fatalf("receipt frontier rollback accepted: %+v", got)
	}
}

func TestCorrectionMustAdvanceBeyondSupersededStepReceiptFrontier(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	original := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:original", StepApply, testDigestB, testDigestC)
	original.ReceiptFrontierSequence = 2
	original.ReceiptFrontierDigest = receipts[1].ReceiptDigest
	original.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, 2, receipts[1].ReceiptDigest, original.ConsumedReceiptDigests)
	original.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, original.IVCUpdate.PriorAdviceCommitmentDigest, original.IVCUpdate.AdviceDeltaCommitmentDigest)
	original.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, original)
	events := appendExecutionEvent(t, scope, nil, "event:original", ExecutionCommitted, &original, "")
	correction := executionTransition(t, scope, scopeDigest, receipts[1].ReceiptDigest, 2,
		testDigestB, testDigestC, "step:correction", StepRecomputeAfterCorrection, testDigestA, testDigestB, "step:original")
	correction.IVCUpdate.PriorAdviceCommitmentDigest = original.IVCUpdate.NextAdviceCommitmentDigest
	correction.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, 2, receipts[1].ReceiptDigest, correction.ConsumedReceiptDigests)
	correction.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, correction.IVCUpdate.PriorAdviceCommitmentDigest, correction.IVCUpdate.AdviceDeltaCommitmentDigest)
	correction.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, correction)
	events = appendExecutionEvent(t, scope, events, "event:correction", ExecutionCommitted, &correction, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "later oracle receipt frontier") {
		t.Fatalf("correction at the same receipt frontier accepted: %+v", got)
	}
}

func TestReceiptChainFailureStillBindsBothSubmissions(t *testing.T) {
	scope := executionFixtureScope()
	receiptsA := executionReceipts(t, scope, 1)
	receiptsB := executionReceipts(t, scope, 1)
	receiptsB[0].AnswerPayload = []byte("different malformed payload")
	_, _, state := executionGenesis(t, scope)
	scopeDigest, _, _ := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	makeEvent := func(stepID string) []ExecutionEventV1 {
		transition := executionTransition(t, scope, scopeDigest, receiptsA[0].ReceiptDigest, 1,
			state, proof, stepID, StepApply, testDigestB, testDigestC)
		return appendExecutionEvent(t, scope, nil, "event:"+stepID, ExecutionCommitted, &transition, "")
	}
	eventsA := makeEvent("step:A")
	eventsB := makeEvent("step:B")
	tailA := receiptsA[0].ReceiptDigest
	tailB := receiptsB[0].ReceiptDigest
	first := ReplayOracleExecutionTranscriptV1(scope, receiptsA, 1, tailA, eventsA, 1, eventsA[0].EventDigest)
	second := ReplayOracleExecutionTranscriptV1(scope, receiptsB, 1, tailB, eventsB, 1, eventsB[0].EventDigest)
	if first.Disposition != ExecutionChainIndeterminate || second.Disposition != ExecutionChainIndeterminate {
		t.Fatal("malformed receipt submission unexpectedly passed")
	}
	if first.SubmittedReceiptsDigest == second.SubmittedReceiptsDigest ||
		first.SubmittedEventsDigest == second.SubmittedEventsDigest ||
		first.CertificateDigest == second.CertificateDigest {
		t.Fatal("different receipt/event submissions collapsed to one failure receipt")
	}
}

func TestExecutionReceiptFrontierCannotRegressAcrossSteps(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)

	first := executionTransition(t, scope, scopeDigest, receipts[1].ReceiptDigest, 2,
		state, proof, "step:frontier-2", StepApply, testDigestB, testDigestC)
	first.ConsumedReceiptDigests = []string{receipts[0].ReceiptDigest, receipts[1].ReceiptDigest}
	first.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, first)
	events := appendExecutionEvent(t, scope, nil, "event:frontier-2", ExecutionCommitted, &first, "")

	second := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		testDigestB, testDigestC, "step:frontier-1", StepApply, testDigestA, testDigestB)
	events = appendExecutionEvent(t, scope, events, "event:frontier-1", ExecutionCommitted, &second, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "receipt frontier regresses") {
		t.Fatalf("receipt frontier rollback accepted: %+v", got)
	}
}

func TestCorrectionMustAdvanceBeyondSupersededStepReceiptFrontier(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	original := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:original", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:original", ExecutionCommitted, &original, "")
	correction := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		testDigestB, testDigestC, "step:correction", StepRecomputeAfterCorrection, testDigestA, testDigestB, "step:original")
	events = appendExecutionEvent(t, scope, events, "event:correction", ExecutionCommitted, &correction, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "later oracle receipt frontier") {
		t.Fatalf("correction at the same receipt frontier accepted: %+v", got)
	}
}

func TestReceiptChainFailureStillBindsBothSubmissions(t *testing.T) {
	scope := executionFixtureScope()
	receiptsA := executionReceipts(t, scope, 1)
	receiptsB := executionReceipts(t, scope, 1)
	receiptsB[0].AnswerPayload = []byte("different malformed payload")
	receiptsB[0].AnswerDigest = AnswerDigestV1(receiptsB[0].AnswerPayload)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	makeEvents := func(stepID string) []ExecutionEventV1 {
		transition := executionTransition(t, scope, scopeDigest, receiptsA[0].ReceiptDigest, 1,
			state, proof, stepID, StepApply, testDigestB, testDigestC)
		return appendExecutionEvent(t, scope, nil, "event:"+stepID, ExecutionCommitted, &transition, "")
	}
	eventsA := makeEvents("step:A")
	eventsB := makeEvents("step:B")
	first := ReplayOracleExecutionTranscriptV1(scope, receiptsA, 1, receiptsA[0].ReceiptDigest, eventsA, 1, eventsA[0].EventDigest)
	second := ReplayOracleExecutionTranscriptV1(scope, receiptsB, 1, receiptsB[0].ReceiptDigest, eventsB, 1, eventsB[0].EventDigest)
	if first.Disposition != ExecutionChainIndeterminate || second.Disposition != ExecutionChainIndeterminate {
		t.Fatal("malformed receipt submission unexpectedly passed")
	}
	if first.SubmittedReceiptsDigest == second.SubmittedReceiptsDigest ||
		first.SubmittedEventsDigest == second.SubmittedEventsDigest ||
		first.CertificateDigest == second.CertificateDigest {
		t.Fatal("different receipt/event submissions collapsed to one failure receipt")
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
	correction.IVCUpdate.PriorAdviceCommitmentDigest = original.IVCUpdate.NextAdviceCommitmentDigest
	correction.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, correction.ReceiptFrontierSequence, correction.ReceiptFrontierDigest, correction.ConsumedReceiptDigests)
	correction.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, correction.IVCUpdate.PriorAdviceCommitmentDigest, correction.IVCUpdate.AdviceDeltaCommitmentDigest)
	correction.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, correction)
	events = appendExecutionEvent(t, scope, events, "event:recompute", ExecutionCommitted, &correction, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainStructurallyValid ||
		got.StateHeadDigest != testDigestA || got.ProofHeadDigest != testDigestB ||
		got.AdviceCommitmentDigest != correction.IVCUpdate.NextAdviceCommitmentDigest ||
		got.ReceiptFrontierSequence != 2 {
		t.Fatalf("backdated correction recomputation failed: %+v", got)
	}
	if events[0].EventDigest != originalEventDigest || len(got.AppliedStepIDs) != 2 {
		t.Fatal("correction rewrote prior step or dropped step history")
	}
}

func TestCorrectionMustSupersedeAnEarlierAppliedStep(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	original := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:original", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:original", ExecutionCommitted, &original, "")

	transition := executionTransition(t, scope, scopeDigest, receipts[1].ReceiptDigest, 2,
		testDigestB, testDigestC, "step:correction", StepRecomputeAfterCorrection, testDigestA, testDigestB, "step:missing")
	transition.IVCUpdate.PriorAdviceCommitmentDigest = original.IVCUpdate.NextAdviceCommitmentDigest
	transition.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, transition.ReceiptFrontierSequence, transition.ReceiptFrontierDigest, transition.ConsumedReceiptDigests)
	transition.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, transition.IVCUpdate.PriorAdviceCommitmentDigest, transition.IVCUpdate.AdviceDeltaCommitmentDigest)
	transition.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, transition)
	events = appendExecutionEvent(t, scope, events, "event:correction", ExecutionCommitted, &transition, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "impact cone mismatch") {
		t.Fatalf("incorrect correction impact cone accepted: %+v", got)
	}
}

func buildTwoStepCorrectionConeFixture(t testing.TB) (OracleScopeV1, []OracleReceiptV1, []ExecutionEventV1, string) {
	t.Helper()
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 2)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)

	first := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:one", StepApply, testDigestB, testDigestC)
	events := appendExecutionEvent(t, scope, nil, "event:one", ExecutionCommitted, &first, "")

	second := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		testDigestB, testDigestC, "step:two", StepApply, testDigestA, testDigestB)
	second.IVCUpdate.PriorAdviceCommitmentDigest = first.IVCUpdate.NextAdviceCommitmentDigest
	second.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, 1, receipts[0].ReceiptDigest, second.ConsumedReceiptDigests)
	second.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, second.IVCUpdate.PriorAdviceCommitmentDigest, second.IVCUpdate.AdviceDeltaCommitmentDigest)
	second.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, second)
	events = appendExecutionEvent(t, scope, events, "event:two", ExecutionCommitted, &second, "")
	return scope, receipts, events, second.IVCUpdate.NextAdviceCommitmentDigest
}

func correctionTransitionForCone(t testing.TB, scope OracleScopeV1, receipts []OracleReceiptV1, priorState, priorProof, priorAdvice string, supersedes ...string) ExecutionTransitionV1 {
	t.Helper()
	scopeDigest, _, _ := executionGenesis(t, scope)
	correction := executionTransition(t, scope, scopeDigest, receipts[1].ReceiptDigest, 2,
		priorState, priorProof, "step:correction", StepRecomputeAfterCorrection, testDigestC, testDigestA, supersedes...)
	correction.IVCUpdate.PriorAdviceCommitmentDigest = priorAdvice
	correction.IVCUpdate.AdviceDeltaCommitmentDigest, _ = executionAdviceDeltaCommitmentDigestV1(
		scopeDigest, 2, receipts[1].ReceiptDigest, correction.ConsumedReceiptDigests)
	correction.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, priorAdvice, correction.IVCUpdate.AdviceDeltaCommitmentDigest)
	correction.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, correction)
	return correction
}

func TestCorrectionMustSupersedeWholeDownstreamLinearSuffix(t *testing.T) {
	scope, receipts, baseEvents, adviceHead := buildTwoStepCorrectionConeFixture(t)

	t.Run("partial cone rejected", func(t *testing.T) {
		partial := correctionTransitionForCone(t, scope, receipts, testDigestA, testDigestB, adviceHead, "step:one")
		events := appendExecutionEvent(t, scope, append([]ExecutionEventV1(nil), baseEvents...), "event:correction", ExecutionCommitted, &partial, "")
		got := replayExecution(t, scope, receipts, events)
		if got.Disposition != ExecutionChainIndeterminate ||
			!strings.Contains(got.Reason, "impact cone incomplete") {
			t.Fatalf("partial downstream correction cone accepted: %+v", got)
		}
	})

	t.Run("complete suffix accepted", func(t *testing.T) {
		complete := correctionTransitionForCone(t, scope, receipts, testDigestA, testDigestB, adviceHead, "step:one", "step:two")
		events := appendExecutionEvent(t, scope, append([]ExecutionEventV1(nil), baseEvents...), "event:correction", ExecutionCommitted, &complete, "")
		got := replayExecution(t, scope, receipts, events)
		if got.Disposition != ExecutionChainStructurallyValid ||
			got.StateHeadDigest != testDigestC ||
			got.ProofHeadDigest != testDigestA ||
			got.AdviceCommitmentDigest != complete.IVCUpdate.NextAdviceCommitmentDigest {
			t.Fatalf("complete downstream correction cone rejected: %+v", got)
		}
		if len(got.AppliedStepIDs) != 3 ||
			got.AppliedStepIDs[0] != "step:one" ||
			got.AppliedStepIDs[1] != "step:two" ||
			got.AppliedStepIDs[2] != "step:correction" {
			t.Fatalf("correction cone did not preserve ordered historical steps: %+v", got.AppliedStepIDs)
		}
	})
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
		blocked.PendingEventID != "event:unknown" || blocked.StateHeadDigest != state || blocked.ProofHeadDigest != proof ||
		blocked.AdviceCommitmentDigest != executionGenesisAdviceCommitmentDigest(scopeDigest) {
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
		got.AdviceCommitmentDigest != pending.IVCUpdate.NextAdviceCommitmentDigest ||
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
		got.AdviceCommitmentDigest != executionGenesisAdviceCommitmentDigest(scopeDigest) ||
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
	got = replayExecution(t, scope, receipts, events)
	got.AdviceCommitmentDigest = testDigestA
	if executionReplayIntegrityValid(got) {
		t.Fatal("mutated advice commitment retained a valid replay seal")
	}
	got = replayExecution(t, scope, receipts, events)
	got.ReceiptFrontierSequence = 0
	if executionReplayIntegrityValid(got) {
		t.Fatal("mutated receipt frontier retained a valid replay seal")
	}
}

func TestOversizedExecutionFieldFailureDigestBindsBoundedValueSamples(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	_, _, state := executionGenesis(t, scope)
	scopeDigest, _, _ := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	makeSubmission := func(prefix string) ExecutionEventV1 {
		transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
			state, proof, "step:oversized", StepApply, testDigestB, testDigestC)
		return ExecutionEventV1{
			LogSequence: 1,
			EventID: prefix + strings.Repeat("x", MaxIdentifierBytes),
			PreviousEventDigest: executionGenesisEventDigest(scopeDigest),
			Outcome: ExecutionCommitted,
			Transition: &transition,
		}
	}
	first := ReplayOracleExecutionTranscriptV1(scope, receipts, 1, receipts[0].ReceiptDigest,
		[]ExecutionEventV1{makeSubmission("a")}, 1, testDigestA)
	second := ReplayOracleExecutionTranscriptV1(scope, receipts, 1, receipts[0].ReceiptDigest,
		[]ExecutionEventV1{makeSubmission("b")}, 1, testDigestA)
	if first.Disposition != ExecutionChainIndeterminate || second.Disposition != ExecutionChainIndeterminate ||
		first.SubmittedEventsDigest == second.SubmittedEventsDigest ||
		first.CertificateDigest == second.CertificateDigest {
		t.Fatalf("distinct oversized event submissions collapsed: first=%+v second=%+v", first, second)
	}
}

func TestOversizedExecutionCollectionFailureDigestBindsSamples(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	makeSubmission := func(last string) ExecutionEventV1 {
		transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
			state, proof, "step:collections", StepApply, testDigestB, testDigestC)
		transition.ConsumedReceiptDigests = make([]string, MaxEvidenceReferences+1)
		for i := range transition.ConsumedReceiptDigests {
			transition.ConsumedReceiptDigests[i] = testDigestA
		}
		transition.ConsumedReceiptDigests[len(transition.ConsumedReceiptDigests)-1] = last
		transition.IVCUpdate.PublicInputDigest = testDigestB
		return ExecutionEventV1{
			LogSequence: 1,
			EventID: "event:collections",
			PreviousEventDigest: executionGenesisEventDigest(scopeDigest),
			Outcome: ExecutionCommitted,
			Transition: &transition,
		}
	}
	first := ReplayOracleExecutionTranscriptV1(scope, receipts, 1, receipts[0].ReceiptDigest,
		[]ExecutionEventV1{makeSubmission(testDigestB)}, 1, testDigestA)
	second := ReplayOracleExecutionTranscriptV1(scope, receipts, 1, receipts[0].ReceiptDigest,
		[]ExecutionEventV1{makeSubmission(testDigestC)}, 1, testDigestA)
	if first.Disposition != ExecutionChainIndeterminate || second.Disposition != ExecutionChainIndeterminate ||
		first.SubmittedEventsDigest == second.SubmittedEventsDigest ||
		first.CertificateDigest == second.CertificateDigest {
		t.Fatalf("distinct oversized collection submissions collapsed: first=%+v second=%+v", first, second)
	}
}

func TestExecutionTranscriptAccumulatesOracleAdviceCommitments(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	events := buildExecutionChain(t, scope, receipts, 3)
	for i := 1; i < len(events); i++ {
		prior := events[i-1].Transition.IVCUpdate.NextAdviceCommitmentDigest
		if events[i].Transition.IVCUpdate.PriorAdviceCommitmentDigest != prior {
			t.Fatalf("step %d does not extend prior advice commitment", i+1)
		}
	}
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainStructurallyValid ||
		got.AdviceCommitmentDigest != events[len(events)-1].Transition.IVCUpdate.NextAdviceCommitmentDigest {
		t.Fatalf("advice commitment chain was not reconstructed: %+v", got)
	}
}

func TestExecutionTranscriptRejectsAlteredAdviceDeltaCommitment(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:bad-advice", StepApply, testDigestB, testDigestC)
	transition.IVCUpdate.AdviceDeltaCommitmentDigest = testDigestC
	transition.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, transition.IVCUpdate.PriorAdviceCommitmentDigest, transition.IVCUpdate.AdviceDeltaCommitmentDigest)
	transition.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, transition)
	events := appendExecutionEvent(t, scope, nil, "event:bad-advice", ExecutionCommitted, &transition, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "advice-delta commitment") {
		t.Fatalf("altered advice delta commitment accepted: %+v", got)
	}
}

func TestExecutionTranscriptRejectsAdviceCommitmentFork(t *testing.T) {
	scope := executionFixtureScope()
	receipts := executionReceipts(t, scope, 1)
	scopeDigest, _, state := executionGenesis(t, scope)
	proof := executionGenesisProofDigest(scopeDigest)
	transition := executionTransition(t, scope, scopeDigest, receipts[0].ReceiptDigest, 1,
		state, proof, "step:forked-advice", StepApply, testDigestB, testDigestC)
	transition.IVCUpdate.PriorAdviceCommitmentDigest = testDigestC
	transition.IVCUpdate.NextAdviceCommitmentDigest, _ = executionNextAdviceCommitmentDigestV1(
		scopeDigest, transition.IVCUpdate.PriorAdviceCommitmentDigest, transition.IVCUpdate.AdviceDeltaCommitmentDigest)
	transition.IVCUpdate.PublicInputDigest, _ = ExecutionPublicInputDigestV1(scope, transition)
	events := appendExecutionEvent(t, scope, nil, "event:forked-advice", ExecutionCommitted, &transition, "")
	got := replayExecution(t, scope, receipts, events)
	if got.Disposition != ExecutionChainIndeterminate ||
		!strings.Contains(got.Reason, "prior advice commitment") {
		t.Fatalf("advice commitment fork accepted: %+v", got)
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
