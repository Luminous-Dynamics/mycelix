package sagaverifier

import (
	"os"
	"strconv"
	"strings"
	"testing"
)

func findTraceScenario(t *testing.T, scenario uint8) TraceV1 {
	t.Helper()
	const seed = uint64(0x0123456789abcdef)
	for index := uint64(0); index < 100_000; index++ {
		trace := GenerateTraceV1(seed, index)
		if trace.Scenario == scenario {
			return trace
		}
	}
	t.Fatalf("could not find scenario %d", scenario)
	return TraceV1{}
}

func TestOracleV1MatchesSharedIndependentKnownAnswers(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/saga-oracle-v1.kat")
	if err != nil {
		t.Fatalf("read oracle KAT: %v", err)
	}
	vectorsSeen := 0
	for _, line := range strings.Split(string(raw), "\n") {
		if !strings.HasPrefix(line, "vector=") {
			continue
		}
		fields := strings.Split(strings.TrimPrefix(line, "vector="), ",")
		if len(fields) != 5 {
			t.Fatalf("invalid oracle KAT vector: %q", line)
		}
		seed, err := strconv.ParseUint(fields[0], 16, 64)
		if err != nil { t.Fatalf("parse seed: %v", err) }
		traceIndex, err := strconv.ParseUint(fields[1], 10, 64)
		if err != nil { t.Fatalf("parse index: %v", err) }
		scenario, err := strconv.ParseUint(fields[2], 10, 8)
		if err != nil { t.Fatalf("parse scenario: %v", err) }
		commandCount, err := strconv.ParseUint(fields[3], 10, 32)
		if err != nil { t.Fatalf("parse command count: %v", err) }

		trace := GenerateTraceV1(seed, traceIndex)
		if uint64(trace.Scenario) != scenario {
			t.Fatalf("scenario = %d, want %d", trace.Scenario, scenario)
		}
		if uint64(len(trace.Commands)) != commandCount {
			t.Fatalf("command count = %d, want %d", len(trace.Commands), commandCount)
		}
		evaluation, err := (TraceOracleV1{}).Evaluate(trace)
		if err != nil { t.Fatalf("evaluate oracle vector: %v", err) }
		if !evaluation.IsSafe() {
			t.Fatalf("unsafe oracle KAT seed=%s mask=%d delta=%v", fields[0], evaluation.InvariantFailureMask, evaluation.ConservationDeltaMinor)
		}
		digest, err := evaluation.CanonicalDigest()
		if err != nil { t.Fatalf("canonical oracle digest: %v", err) }
		if digest != "sha256:"+fields[4] {
			t.Fatalf("oracle digest = %s, want sha256:%s", digest, fields[4])
		}
		vectorsSeen++
	}
	if vectorsSeen != 8 { t.Fatalf("oracle KAT vectors = %d, want 8", vectorsSeen) }
}

func TestGeneratedTraceCorpusEvaluatesSafelyAndConservesMinorUnits(t *testing.T) {
	seen := [8]bool{}
	oracle := TraceOracleV1{}
	for seed := uint64(0); seed < 4096; seed++ {
		trace := GenerateTraceV1(seed, rotateLeft64(seed, 13))
		seen[trace.Scenario-1] = true
		evaluation, err := oracle.Evaluate(trace)
		if err != nil { t.Fatal(err) }
		if !evaluation.IsSafe() {
			t.Fatalf("unsafe trace scenario=%d seed=%d delta=%v mask=%d",
				trace.Scenario, seed, evaluation.ConservationDeltaMinor, evaluation.InvariantFailureMask)
		}
		if !evaluation.LocalCommitted { t.Fatalf("local payment did not commit in scenario %d", trace.Scenario) }
		if evaluation.ConservationDeltaMinor.Sign() != 0 { t.Fatalf("nonzero conservation delta: %v", evaluation.ConservationDeltaMinor) }
		if evaluation.ActualTreasuryEffectCount > 1 || evaluation.CompensationCount > 1 {
			t.Fatalf("duplicate effect count in scenario %d", trace.Scenario)
		}
	}
	for i, value := range seen {
		if !value { t.Fatalf("scenario %d not covered", i+1) }
	}
}

func TestScenarioTwoRequiresAuthoritativeAbsenceBeforeSameEffectRetry(t *testing.T) {
	evaluation, err := (TraceOracleV1{}).Evaluate(findTraceScenario(t, 2))
	if err != nil { t.Fatal(err) }
	if evaluation.TreasuryState != TreasuryCommitted { t.Fatalf("treasury state = %v", evaluation.TreasuryState) }
	if evaluation.ActualTreasuryEffectCount != 1 { t.Fatalf("effect count = %d", evaluation.ActualTreasuryEffectCount) }
	if evaluation.UnresolvedTreasuryExposureMinor != 0 { t.Fatalf("unresolved exposure = %d", evaluation.UnresolvedTreasuryExposureMinor) }
}

func TestUncertainLookupNeverBecomesCommitOrAbsence(t *testing.T) {
	evaluation, err := (TraceOracleV1{}).Evaluate(findTraceScenario(t, 4))
	if err != nil { t.Fatal(err) }
	if evaluation.TreasuryState != TreasuryIndeterminate { t.Fatalf("treasury state = %v", evaluation.TreasuryState) }
	if evaluation.UnresolvedTreasuryExposureMinor != evaluation.FeeMinor {
		t.Fatalf("unresolved exposure = %d, fee=%d", evaluation.UnresolvedTreasuryExposureMinor, evaluation.FeeMinor)
	}
	if evaluation.ActualTreasuryEffectCount != 1 { t.Fatalf("effect count = %d", evaluation.ActualTreasuryEffectCount) }
}

func TestDuplicateDeliveryDoesNotCreateSecondEffect(t *testing.T) {
	evaluation, err := (TraceOracleV1{}).Evaluate(findTraceScenario(t, 6))
	if err != nil { t.Fatal(err) }
	if evaluation.ActualTreasuryEffectCount != 1 { t.Fatalf("effect count = %d", evaluation.ActualTreasuryEffectCount) }
	if evaluation.TreasuryState != TreasuryIndeterminate { t.Fatalf("treasury state = %v", evaluation.TreasuryState) }
	found := false
	for _, step := range evaluation.Steps {
		if step.Disposition == TraceIndeterminate && step.Reason == TraceReasonDuplicateEffectSuppressed { found = true }
	}
	if !found { t.Fatal("duplicate delivery was not explicitly suppressed") }
}

func TestCompensationIsSeparateAndDoesNotEraseOriginalCommit(t *testing.T) {
	evaluation, err := (TraceOracleV1{}).Evaluate(findTraceScenario(t, 8))
	if err != nil { t.Fatal(err) }
	if !evaluation.LocalCommitted { t.Fatal("original local commit was erased") }
	if evaluation.CompensationCount != 1 { t.Fatalf("compensation count = %d", evaluation.CompensationCount) }
	if evaluation.CompensationSourceDebitMinor != evaluation.CompensationRecipientCreditMinor {
		t.Fatalf("compensation does not conserve: %d != %d",
			evaluation.CompensationSourceDebitMinor, evaluation.CompensationRecipientCreditMinor)
	}
	if evaluation.PayerDebitMinor != evaluation.PrincipalMinor+evaluation.FeeMinor {
		t.Fatalf("original payer debit changed: %d", evaluation.PayerDebitMinor)
	}
}

func TestOracleCanonicalDigestIsDeterministic(t *testing.T) {
	trace := findTraceScenario(t, 3)
	first, err := (TraceOracleV1{}).Evaluate(trace)
	if err != nil { t.Fatal(err) }
	second, err := (TraceOracleV1{}).Evaluate(trace)
	if err != nil { t.Fatal(err) }
	firstDigest, err := first.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	secondDigest, err := second.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	if firstDigest != secondDigest { t.Fatalf("oracle digest differs: %s != %s", firstDigest, secondDigest) }
}

func rotateLeft64(value uint64, amount uint) uint64 {
	return (value << amount) | (value >> (64 - amount))
}
