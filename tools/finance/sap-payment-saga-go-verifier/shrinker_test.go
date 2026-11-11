package sagaverifier

import (
	"testing"
)

func ownerMismatchSignatureV1(trace TraceV1) string {
	if len(trace.Commands) > 0 &&
		trace.Commands[0].Opcode == TraceCreateIntent &&
		trace.Commands[0].Owner == TraceAdversary {
		return "owner-mismatch:command-0"
	}
	return ""
}

func failingOwnerTraceV1() TraceV1 {
	trace := GenerateTraceV1(0x0123456789abcdef, 0)
	trace.Commands[0].Owner = TraceAdversary
	return trace
}

func TestTraceShrinkerMinimizesFirstOwnerFailure(t *testing.T) {
	input := failingOwnerTraceV1()
	result, err := MinimizeTraceV1(input, "owner-mismatch:command-0", ownerMismatchSignatureV1)
	if err != nil {
		t.Fatal(err)
	}
	if len(result.Trace.Commands) != 1 {
		t.Fatalf("minimized command count = %d", len(result.Trace.Commands))
	}
	if result.Trace.Seed != input.Seed ||
		result.Trace.TraceIndex != input.TraceIndex ||
		result.Trace.Scenario != input.Scenario {
		t.Fatal("trace profile metadata changed during shrinking")
	}
	command := result.Trace.Commands[0]
	if command.Opcode != input.Commands[0].Opcode ||
		command.Owner != TraceAdversary ||
		command.Fault != input.Commands[0].Fault ||
		command.Subject != 0 || command.Value != 0 {
		t.Fatalf("unexpected minimized command: %+v", command)
	}
	if ownerMismatchSignatureV1(result.Trace) != "owner-mismatch:command-0" {
		t.Fatal("target failure signature was not preserved")
	}
	digest, err := result.Trace.CanonicalDigest()
	if err != nil {
		t.Fatal(err)
	}
	if digest != result.MinimizedDigest {
		t.Fatalf("minimized digest = %s, recomputed = %s", result.MinimizedDigest, digest)
	}
	if result.RemovedCommands == 0 || result.ChangedNumericFields == 0 || result.Attempts == 0 {
		t.Fatalf("shrinker reported no reductions: %+v", result)
	}
}

func TestTraceShrinkerRejectsMissingFailureSignature(t *testing.T) {
	input := GenerateTraceV1(7, 9)
	if _, err := MinimizeTraceV1(input, "not-present", ownerMismatchSignatureV1); err != ErrTraceShrinkTargetMissing {
		t.Fatalf("missing failure result = %v", err)
	}
}

func TestTraceShrinkerPreservesOwnerAndFaultWhenReducing(t *testing.T) {
	input := failingOwnerTraceV1()
	input.Commands[0].Fault = TracePayloadMutation
	result, err := MinimizeTraceV1(input, "owner-mismatch:command-0", ownerMismatchSignatureV1)
	if err != nil {
		t.Fatal(err)
	}
	if result.Trace.Commands[0].Owner != TraceAdversary ||
		result.Trace.Commands[0].Fault != TracePayloadMutation {
		t.Fatalf("shrinker changed owner/fault: %+v", result.Trace.Commands[0])
	}
}

func TestTraceShrinkerIsDeterministic(t *testing.T) {
	input := failingOwnerTraceV1()
	first, err := MinimizeTraceV1(input, "owner-mismatch:command-0", ownerMismatchSignatureV1)
	if err != nil {
		t.Fatal(err)
	}
	second, err := MinimizeTraceV1(input, "owner-mismatch:command-0", ownerMismatchSignatureV1)
	if err != nil {
		t.Fatal(err)
	}
	if first.MinimizedDigest != second.MinimizedDigest ||
		first.Attempts != second.Attempts ||
		first.RemovedCommands != second.RemovedCommands ||
		first.ChangedNumericFields != second.ChangedNumericFields {
		t.Fatalf("shrink result diverged: first=%+v second=%+v", first, second)
	}
}
