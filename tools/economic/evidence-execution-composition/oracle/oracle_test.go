package oracle

import (
	"testing"
)

func TestSeedManifestMatchesIndependentOracle(t *testing.T) {
	manifest, err := LoadManifest("../DJCC_V0_1_SEEDS.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := ValidateManifest(manifest); err != nil {
		t.Fatalf("DJCC seed manifest failed independent validation: %v", err)
	}
}

func TestAllRegisteredCasesArePresentExactlyOnce(t *testing.T) {
	manifest, err := LoadManifest("../DJCC_V0_1_SEEDS.json")
	if err != nil {
		t.Fatal(err)
	}
	if len(manifest.Cases) != len(CaseIDs()) {
		t.Fatalf("got %d cases, oracle expects %d", len(manifest.Cases), len(CaseIDs()))
	}
	seen := make(map[string]bool, len(manifest.Cases))
	for _, scenario := range manifest.Cases {
		if seen[scenario.ID] {
			t.Fatalf("duplicate case %s", scenario.ID)
		}
		seen[scenario.ID] = true
		if _, ok := ExpectedFor(scenario.ID); !ok {
			t.Fatalf("case %s has no independent expected result", scenario.ID)
		}
	}
	for _, id := range CaseIDs() {
		if !seen[id] {
			t.Errorf("manifest is missing oracle case %s", id)
		}
	}
}

func TestExpectedForReturnsDefensiveStageMap(t *testing.T) {
	first, ok := ExpectedFor("DJCC-001")
	if !ok {
		t.Fatal("DJCC-001 missing from oracle")
	}
	first.Stages[StagePolicy] = Rejected
	second, ok := ExpectedFor("DJCC-001")
	if !ok {
		t.Fatal("DJCC-001 missing from oracle on second read")
	}
	if second.Stages[StagePolicy] != VerifiedForDeclaredClaim {
		t.Fatal("caller mutation changed the oracle's expected result")
	}
}

func TestUnauthenticatedRootNeverComposesAsJustified(t *testing.T) {
	result, ok := ExpectedFor("DJCC-002")
	if !ok {
		t.Fatal("DJCC-002 missing from oracle")
	}
	if result.Stages[StageSourceTrust] != Indeterminate ||
		result.Composition != CompositionUnknown ||
		result.ClaimCeiling.Authentication != ClaimUnknown {
		t.Fatalf("unauthenticated root was promoted: %+v", result)
	}
}

func TestHiddenOwnerNeverBecomesFalseAbsence(t *testing.T) {
	result, ok := ExpectedFor("DJCC-003")
	if !ok {
		t.Fatal("DJCC-003 missing from oracle")
	}
	if result.Stages[StageVerifierFrontier] != Indeterminate ||
		result.ClaimCeiling.Completeness != ClaimUnknown ||
		result.Composition != CompositionUnknown {
		t.Fatalf("incomplete owner coverage was promoted: %+v", result)
	}
}

func TestHistoricalFrontierDoesNotImportLaterCorrection(t *testing.T) {
	result, ok := ExpectedFor("DJCC-004")
	if !ok {
		t.Fatal("DJCC-004 missing from oracle")
	}
	if result.Composition != JustifiedAtFrontier ||
		result.Stages[StageSemantic] != VerifiedForDeclaredClaim ||
		result.Stages[StageExecution] != NotApplicable {
		t.Fatalf("historical noninterference expectation changed: %+v", result)
	}
}

func TestOutcomeUnknownBlocksFurtherMutation(t *testing.T) {
	result, ok := ExpectedFor("DJCC-007")
	if !ok {
		t.Fatal("DJCC-007 missing from oracle")
	}
	if result.Stages[StageExecution] != Rejected ||
		result.ClaimCeiling.Effect != ClaimUnknown ||
		result.Composition != CompositionUnknown {
		t.Fatalf("unknown outcome was treated as safe to mutate: %+v", result)
	}
}

func TestProviderCompletionIsNotIndependentOutcome(t *testing.T) {
	result, ok := ExpectedFor("DJCC-008")
	if !ok {
		t.Fatal("DJCC-008 missing from oracle")
	}
	if result.Stages[StageOutcome] != Indeterminate ||
		result.ClaimCeiling.Effect != ClaimUnknown ||
		result.Composition != CompositionUnknown {
		t.Fatalf("provider completion was promoted to an observed outcome: %+v", result)
	}
}
