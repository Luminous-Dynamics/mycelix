package sagaverifier

import "testing"

func TestTraceV1EvaluatorMatchesIndependentOracleAcross4096Traces(t *testing.T) {
	var scenarios [8]bool
	for index := uint64(0); index < 4096; index++ {
		trace := GenerateTraceV1(0x5eed2026, index)
		scenarios[trace.Scenario-1] = true
		evaluation, err := EvaluateTraceV1(trace)
		if err != nil {
			t.Fatalf("index %d scenario %d: %v", index, trace.Scenario, err)
		}
		if !evaluation.Passed() {
			t.Fatalf("index %d scenario %d digest %s failures=%+v",
				index, trace.Scenario, evaluation.TraceDigest, evaluation.Failures)
		}
	}
	for scenario, seen := range scenarios {
		if !seen {
			t.Fatalf("scenario %d not covered by deterministic corpus", scenario+1)
		}
	}
}

func TestTraceV1MutatedIntentDoesNotRewriteLocalCommit(t *testing.T) {
	for index := uint64(0); index < 4096; index++ {
		trace := GenerateTraceV1(0x5eed2026, index)
		if trace.Scenario != 5 {
			continue
		}
		evaluation, err := EvaluateTraceV1(trace)
		if err != nil { t.Fatal(err) }
		if !evaluation.Passed() { t.Fatalf("%+v", evaluation.Failures) }
		if evaluation.FinalStatus != FeeDistributionPending {
			t.Fatalf("final status = %s, want fee pending", evaluation.FinalStatus)
		}
		if got := evaluation.Steps[len(evaluation.Steps)-1].Observed; got != TraceRejected {
			t.Fatalf("mutate intent disposition = %s", got)
		}
		return
	}
	t.Fatal("scenario 5 missing")
}

func TestTraceV1UncertainLookupRemainsIndeterminate(t *testing.T) {
	for index := uint64(0); index < 4096; index++ {
		trace := GenerateTraceV1(0x5eed2026, index)
		if trace.Scenario != 4 { continue }
		evaluation, err := EvaluateTraceV1(trace)
		if err != nil { t.Fatal(err) }
		if !evaluation.Passed() { t.Fatalf("%+v", evaluation.Failures) }
		if evaluation.FinalStatus != OperationIndeterminate {
			t.Fatalf("final status = %s, want indeterminate", evaluation.FinalStatus)
		}
		return
	}
	t.Fatal("scenario 4 missing")
}

func TestTraceV1CompensationDoesNotUndoHistoricalCommit(t *testing.T) {
	for index := uint64(0); index < 4096; index++ {
		trace := GenerateTraceV1(0x5eed2026, index)
		if trace.Scenario != 8 { continue }
		evaluation, err := EvaluateTraceV1(trace)
		if err != nil { t.Fatal(err) }
		if !evaluation.Passed() { t.Fatalf("%+v", evaluation.Failures) }
		if evaluation.FinalStatus != Completed {
			t.Fatalf("final status = %s, want completed", evaluation.FinalStatus)
		}
		if got := evaluation.Steps[len(evaluation.Steps)-1].Observed; got != TraceRequiresSeparateEffect {
			t.Fatalf("compensation disposition = %s", got)
		}
		return
	}
	t.Fatal("scenario 8 missing")
}
