package sagaverifier

import (
	"encoding/hex"
	"os"
	"strconv"
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
	seed, err := strconv.ParseUint(kat["seed"], 16, 64)
	if err != nil {
		t.Fatalf("parse seed: %v", err)
	}
	traceIndex, err := strconv.ParseUint(kat["trace_index"], 10, 64)
	if err != nil {
		t.Fatalf("parse trace index: %v", err)
	}
	trace := GenerateTraceV1(seed, traceIndex)

	scenario, err := strconv.ParseUint(kat["scenario"], 10, 8)
	if err != nil {
		t.Fatalf("parse scenario: %v", err)
	}
	if got := uint64(trace.Scenario); got != scenario {
		t.Fatalf("scenario = %d, want %d", got, scenario)
	}
	if got := formatTraceHex64(trace.OperationNonce); got != kat["operation_nonce"] {
		t.Fatalf("operation nonce = %s, want %s", got, kat["operation_nonce"])
	}
	commandCount, err := strconv.ParseUint(kat["command_count"], 10, 32)
	if err != nil {
		t.Fatalf("parse command count: %v", err)
	}
	if got := uint64(len(trace.Commands)); got != commandCount {
		t.Fatalf("command count = %d, want %d", got, commandCount)
	}
	encoded, err := trace.CanonicalBytes()
	if err != nil {
		t.Fatalf("encode trace: %v", err)
	}
	if got := hex.EncodeToString(encoded); got != kat["canonical_bytes_hex"] {
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

func TestAllEightTraceScenariosMatchIndependentKnownAnswers(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/saga-trace-v1.kat")
	if err != nil {
		t.Fatalf("read shared trace KAT: %v", err)
	}
	seen := make(map[uint8]bool)
	for _, line := range strings.Split(string(raw), "\n") {
		if !strings.HasPrefix(line, "vector=") {
			continue
		}
		fields := strings.Split(strings.TrimPrefix(line, "vector="), ",")
		if len(fields) != 6 {
			t.Fatalf("invalid vector row %q", line)
		}
		seed, err := strconv.ParseUint(fields[0], 16, 64)
		if err != nil {
			t.Fatalf("parse vector seed: %v", err)
		}
		traceIndex, err := strconv.ParseUint(fields[1], 10, 64)
		if err != nil {
			t.Fatalf("parse vector index: %v", err)
		}
		trace := GenerateTraceV1(seed, traceIndex)
		scenario, err := strconv.ParseUint(fields[2], 10, 8)
		if err != nil || uint64(trace.Scenario) != scenario {
			t.Fatalf("scenario mismatch for vector %q", line)
		}
		if formatTraceHex64(trace.OperationNonce) != fields[3] {
			t.Fatalf("operation nonce mismatch for vector %q", line)
		}
		commandCount, err := strconv.ParseUint(fields[4], 10, 32)
		if err != nil || uint64(len(trace.Commands)) != commandCount {
			t.Fatalf("command count mismatch for vector %q", line)
		}
		digest, err := trace.CanonicalDigest()
		if err != nil || strings.TrimPrefix(digest, "sha256:") != fields[5] {
			t.Fatalf("digest mismatch for vector %q: %v", line, err)
		}
		seen[trace.Scenario] = true
	}
	if len(seen) != 8 {
		t.Fatalf("known-answer scenario coverage = %d, want 8", len(seen))
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

func TestTraceV1DecoderRoundTripsAndRejectsNoncanonicalBytes(t *testing.T) {
	trace := GenerateTraceV1(0x0123456789abcdef, 7)
	encoded, err := trace.CanonicalBytes()
	if err != nil {
		t.Fatalf("encode trace: %v", err)
	}
	decoded, err := DecodeTraceV1(encoded)
	if err != nil || !equalTraceV1(decoded, trace) {
		t.Fatalf("decode round trip failed: err=%v", err)
	}

	badReserved := append([]byte(nil), encoded...)
	badReserved[20+8+8+1+8+4+3] = 1
	if _, err := DecodeTraceV1(badReserved); err != ErrTraceNonzeroReserved {
		t.Fatalf("reserved byte error = %v", err)
	}
	badOpcode := append([]byte(nil), encoded...)
	badOpcode[20+8+8+1+8+4] = 255
	if _, err := DecodeTraceV1(badOpcode); err != ErrTraceInvalidOpcode {
		t.Fatalf("opcode error = %v", err)
	}
	badOwner := append([]byte(nil), encoded...)
	badOwner[20+8+8+1+8+4+1] = 255
	if _, err := DecodeTraceV1(badOwner); err != ErrTraceInvalidOwner {
		t.Fatalf("owner error = %v", err)
	}
	badFault := append([]byte(nil), encoded...)
	badFault[20+8+8+1+8+4+2] = 255
	if _, err := DecodeTraceV1(badFault); err != ErrTraceInvalidFault {
		t.Fatalf("fault error = %v", err)
	}
	badMagic := append([]byte(nil), encoded...)
	badMagic[0] ^= 1
	if _, err := DecodeTraceV1(badMagic); err != ErrTraceInvalidMagic {
		t.Fatalf("magic error = %v", err)
	}
	if _, err := DecodeTraceV1(encoded[:len(encoded)-1]); err != ErrTraceTruncatedInput {
		t.Fatalf("truncated input error = %v", err)
	}
	if _, err := DecodeTraceV1(append(encoded, 0)); err != ErrTraceTrailingBytes {
		t.Fatalf("trailing bytes error = %v", err)
	}
}

func TestTraceV1EnforcesScenarioAndCommandBounds(t *testing.T) {
	trace := GenerateTraceV1(1, 0)
	trace.Scenario = 0
	if _, err := trace.CanonicalBytes(); err != ErrTraceInvalidScenario {
		t.Fatalf("invalid scenario error = %v", err)
	}

	trace = GenerateTraceV1(1, 0)
	template := trace.Commands[0]
	for len(trace.Commands) <= MaxTraceCommands {
		trace.Commands = append(trace.Commands, template)
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

func formatTraceHex64(value uint64) string {
	const digits = "0123456789abcdef"
	var output [16]byte
	for index := 15; index >= 0; index-- {
		output[index] = digits[value&0x0f]
		value >>= 4
	}
	return string(output[:])
}
