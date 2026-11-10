package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

func readTraceKAT(t *testing.T) map[string]string {
	t.Helper()
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/saga-trace-v1.kat")
	if err != nil {
		t.Fatalf("read shared trace KAT: %v", err)
	}
	fields := make(map[string]string)
	for _, line := range strings.Split(string(raw), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		pair := strings.SplitN(line, "=", 2)
		if len(pair) == 2 {
			fields[pair[0]] = pair[1]
		}
	}
	return fields
}

func TestTraceV1MatchesSharedKnownAnswerFixture(t *testing.T) {
	kat := readTraceKAT(t)
	seed, err := parseHexUint64(kat["seed"])
	if err != nil {
		t.Fatalf("parse seed: %v", err)
	}
	traceIndex, err := parseDecimalUint64(kat["trace_index"])
	if err != nil {
		t.Fatalf("parse trace index: %v", err)
	}
	trace := GenerateTraceV1(seed, traceIndex)
	if got := trace.Scenario; got != 1 {
		t.Fatalf("scenario = %d, want %s", got, kat["scenario"])
	}
	if got := formatHexUint64(trace.OperationNonce); got != kat["operation_nonce"] {
		t.Fatalf("operation nonce = %s, want %s", got, kat["operation_nonce"])
	}
	if got := len(trace.Commands); got != 9 {
		t.Fatalf("command count = %d, want %s", got, kat["command_count"])
	}
	encoded, err := trace.CanonicalBytes()
	if err != nil {
		t.Fatalf("encode trace: %v", err)
	}
	if got := strings.ToLower(encodeHex(encoded)); got != kat["canonical_bytes_hex"] {
		t.Fatalf("canonical bytes differ from shared fixture")
	}
	digest, err := trace.CanonicalDigest()
	if err != nil {
		t.Fatalf("digest trace: %v", err)
	}
	if got := strings.TrimPrefix(digest, "sha256:"); got != kat["sha256"] {
		t.Fatalf("digest = %s, want %s", got, kat["sha256"])
	}
}

func TestTraceV1IsDeterministicAndCoversAllScenarios(t *testing.T) {
	var scenarios [8]bool
	for seed := uint64(0); seed < 4096; seed++ {
		index := (seed << 13) | (seed >> 51)
		first := GenerateTraceV1(seed, index)
		second := GenerateTraceV1(seed, index)
		if !equalTraceV1(first, second) {
			t.Fatalf("non-deterministic trace for seed=%d index=%d", seed, index)
		}
		if _, err := first.CanonicalBytes(); err != nil {
			t.Fatalf("invalid generated trace for seed=%d: %v", seed, err)
		}
		scenarios[first.Scenario-1] = true
	}
	for scenario, observed := range scenarios {
		if !observed {
			t.Fatalf("scenario %d was not generated", scenario+1)
		}
	}
}

func TestTraceV1EnforcesScenarioAndCommandBounds(t *testing.T) {
	trace := GenerateTraceV1(1, 0)
	trace.Scenario = 0
	if _, err := trace.CanonicalBytes(); err != ErrTraceInvalidScenario {
		t.Fatalf("invalid scenario error = %v", err)
	}

	trace = GenerateTraceV1(1, 0)
	for len(trace.Commands) <= MaxTraceCommands {
		trace.Commands = append(trace.Commands, trace.Commands[0])
	}
	if _, err := trace.CanonicalBytes(); err != ErrTraceTooManyCommands {
		t.Fatalf("command bound error = %v", err)
	}

	trace = GenerateTraceV1(1, 0)
	trace.Commands = nil
	if _, err := trace.CanonicalBytes(); err != ErrTraceEmptyCommands {
		t.Fatalf("empty trace error = %v", err)
	}
}

func equalTraceV1(left, right TraceV1) bool {
	if left.Seed != right.Seed ||
		left.TraceIndex != right.TraceIndex ||
		left.Scenario != right.Scenario ||
		left.OperationNonce != right.OperationNonce ||
		len(left.Commands) != len(right.Commands) {
		return false
	}
	for index := range left.Commands {
		if left.Commands[index] != right.Commands[index] {
			return false
		}
	}
	return true
}
package sagaverifier

import (
	"encoding/hex"
	"strconv"
)

func parseHexUint64(value string) (uint64, error) {
	parsed, err := strconv.ParseUint(value, 16, 64)
	return parsed, err
}

func parseDecimalUint64(value string) (uint64, error) {
	parsed, err := strconv.ParseUint(value, 10, 64)
	return parsed, err
}

func formatHexUint64(value uint64) string {
	const digits = "0123456789abcdef"
	var output [16]byte
	for index := 15; index >= 0; index-- {
		output[index] = digits[value&0x0f]
		value >>= 4
	}
	return string(output[:])
}

func encodeHex(value []byte) string {
	return hex.EncodeToString(value)
}
