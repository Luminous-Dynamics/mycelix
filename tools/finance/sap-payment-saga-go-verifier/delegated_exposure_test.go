package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

func parentExposureTestPolicy(maximum uint64) DelegatedExposurePolicyV1 {
	childMaximum := maximum
	if childMaximum > 800 { childMaximum = 800 }
	return DelegatedExposurePolicyV1{
		Scope: ParentExposureScopeV1{
			OwnerID: "owner:treasury", PoolID: "pool:one", Currency: "SAP",
		},
		Sequence: 1, MaximumTotalMinor: maximum,
		MaximumChildBudgetMinor: childMaximum,
		PolicyDigest: exposureTestDigest(13),
	}
}

func parentExposureChildScope(owner, destination string) ExposureScopeV1 {
	return ExposureScopeV1{
		OwnerID: owner, PoolID: "pool:one",
		DestinationID: destination, Currency: "SAP",
	}
}

func parentExposureIntent(scope ExposureScopeV1, effectID string, amount uint64) ExposureIntentV1 {
	return ExposureIntentV1{
		Scope: scope, OperationID: "operation:" + effectID, EffectID: effectID,
		IntentDigest: exposureTestDigest(10), PayloadDigest: exposureTestDigest(11),
		MaximumExposureMinor: amount,
	}
}

func parentObservedUnsettled(effectID, destination string) DestinationResolutionV1 {
	return DestinationResolutionV1{
		Kind: ExposureObservedUnsettledEffect, EffectID: effectID, DestinationID: destination,
		PayloadDigest: exposureTestDigest(11), ProfileDigest: exposureTestDigest(14),
		EvidenceDigest: exposureTestDigest(1),
	}
}

func parentExposureResolution(effectID, destination string, kind ExposureResolutionKind) DestinationResolutionV1 {
	return DestinationResolutionV1{
		Kind: kind, EffectID: effectID, DestinationID: destination,
		PayloadDigest: exposureTestDigest(11), ProfileDigest: exposureTestDigest(14),
		EvidenceDigest: exposureTestDigest(15),
	}
}

func TestParentEscrowPreventsCrossDestinationOversubscription(t *testing.T) {
	parent, err := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	if err != nil { t.Fatal(err) }
	first, err := parent.AllocateChildBudget("provider-a", parentExposureChildScope("operator:a", "dest:a"), 700, 1, exposureTestDigest(1))
	if err != nil || first != DelegatedBudgetAllocated { t.Fatalf("first grant = %v, %v", first, err) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 300 { t.Fatalf("available = %d", available) }
	if _, err := parent.AllocateChildBudget("provider-b", parentExposureChildScope("operator:b", "dest:b"), 700, 2, exposureTestDigest(2)); err != ErrExposureLimitExceeded {
		t.Fatalf("second 700 grant exceeded parent cap: %v", err)
	}
	if allocated, _ := parent.AllocatedBudgetMinor(); allocated != 700 { t.Fatalf("allocated = %d", allocated) }
}

func TestChildCannotExceedDelegatedSublimit(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	scope := parentExposureChildScope("operator:a", "dest:a")
	_, err := parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1))
	if err != nil { t.Fatal(err) }
	if _, err := parent.Reserve("provider-a", parentExposureIntent(scope, "effect:1", 500)); err != nil { t.Fatal(err) }
	if _, err := parent.Reserve("provider-a", parentExposureIntent(scope, "effect:2", 250)); err != ErrExposureLimitExceeded {
		t.Fatalf("child exceeded 700 budget: %v", err)
	}
	if total, _ := parent.ChildExposureMinor("provider-a"); total != 500 { t.Fatalf("child exposure = %d", total) }
}

func TestRevocationStopsNewRiskButAllowsExistingResolution(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	scope := parentExposureChildScope("operator:a", "dest:a")
	_, _ = parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1))
	intent := parentExposureIntent(scope, "effect:1", 400)
	if _, err := parent.Reserve("provider-a", intent); err != nil { t.Fatal(err) }
	if err := parent.MarkDispatchAuthorized("provider-a", "effect:1"); err != nil { t.Fatal(err) }
	if err := parent.BeginRevocation("provider-a"); err != nil { t.Fatal(err) }
	if _, err := parent.Reserve("provider-a", parentExposureIntent(scope, "effect:2", 100)); err != ErrExposureInvalidTransition {
		t.Fatalf("new reservation during revocation = %v", err)
	}
	if disposition, err := parent.Reserve("provider-a", intent); err != nil || disposition != ExposureAlreadyReserved {
		t.Fatalf("exact retry during revocation = %v, %v", disposition, err)
	}
	if err := parent.MarkCommittedUnsettled("provider-a", "effect:1", parentObservedUnsettled("effect:1", "dest:a")); err != nil { t.Fatal(err) }
	evidence := parentExposureResolution("effect:1", "dest:a", ExposureFinalSettlementEvidence)
	if err := parent.MarkSettled("provider-a", "effect:1", evidence); err != nil { t.Fatal(err) }
	if err := parent.CloseGrant("provider-a"); err != nil { t.Fatal(err) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 1_000 { t.Fatalf("available after close = %d", available) }
}

func TestRevocationBlocksDispatchOfPreviouslyReservedButUndispatchedEffect(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	scope := parentExposureChildScope("operator:a", "dest:a")
	_, _ = parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1))
	_, _ = parent.Reserve("provider-a", parentExposureIntent(scope, "effect:reserved", 100))
	if err := parent.BeginRevocation("provider-a"); err != nil { t.Fatal(err) }
	if err := parent.MarkDispatchAuthorized("provider-a", "effect:reserved"); err != ErrExposureInvalidTransition {
		t.Fatalf("reserved effect dispatched after revocation: %v", err)
	}
	if err := parent.CancelBeforeDispatch("provider-a", "effect:reserved"); err != nil { t.Fatal(err) }
	if err := parent.CloseGrant("provider-a"); err != nil { t.Fatal(err) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 1_000 {
		t.Fatalf("available after safe cancellation = %d", available)
	}
}

func TestUnresolvedChildEffectPreventsParentBudgetReclamation(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	scope := parentExposureChildScope("operator:a", "dest:a")
	_, _ = parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1))
	_, _ = parent.Reserve("provider-a", parentExposureIntent(scope, "effect:1", 400))
	_ = parent.MarkDispatchAuthorized("provider-a", "effect:1")
	_ = parent.MarkIndeterminate("provider-a", "effect:1")
	_ = parent.BeginRevocation("provider-a")
	if err := parent.CloseGrant("provider-a"); err != ErrExposureInvalidTransition {
		t.Fatalf("unresolved grant closed: %v", err)
	}
	if available, _ := parent.AvailableParentBudgetMinor(); available != 300 { t.Fatalf("available while unresolved = %d", available) }
	absence := parentExposureResolution("effect:1", "dest:a", ExposureAuthoritativeAbsence)
	if err := parent.ReleaseAuthoritativeAbsence("provider-a", "effect:1", absence); err != nil { t.Fatal(err) }
	if err := parent.CloseGrant("provider-a"); err != nil { t.Fatal(err) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 1_000 { t.Fatalf("available after absence + close = %d", available) }
}

func TestGrantIdentityScopeAndSequenceAreBound(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	scope := parentExposureChildScope("operator:a", "dest:a")
	_, _ = parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1))
	if _, err := parent.AllocateChildBudget("provider-a", scope, 600, 1, exposureTestDigest(1)); err != ErrExposureReservationConflict {
		t.Fatalf("changed grant terms = %v", err)
	}
	if _, err := parent.AllocateChildBudget("provider-b", parentExposureChildScope("operator:b", "dest:b"), 200, 2, exposureTestDigest(2)); err != nil {
		t.Fatalf("valid second grant = %v", err)
	}
	wrongCurrency := parentExposureChildScope("operator:c", "dest:c")
	wrongCurrency.Currency = "USD"
	if _, err := parent.AllocateChildBudget("provider-c", wrongCurrency, 100, 3, exposureTestDigest(3)); err != ErrExposureScopeMismatch {
		t.Fatalf("cross-currency grant = %v", err)
	}
}

func TestParentPolicyReductionFreezesNewChildRiskUntilBudgetIsReclaimed(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	firstScope := parentExposureChildScope("operator:a", "dest:a")
	secondScope := parentExposureChildScope("operator:b", "dest:b")
	_, _ = parent.AllocateChildBudget("provider-a", firstScope, 600, 1, exposureTestDigest(1))
	existingIntent := parentExposureIntent(firstScope, "effect:existing", 200)
	if _, err := parent.Reserve("provider-a", existingIntent); err != nil { t.Fatal(err) }
	if err := parent.MarkDispatchAuthorized("provider-a", "effect:existing"); err != nil { t.Fatal(err) }
	_, _ = parent.AllocateChildBudget("provider-b", secondScope, 300, 2, exposureTestDigest(2))

	lower := parentExposureTestPolicy(800)
	lower.Sequence = 2
	lower.MaximumChildBudgetMinor = 600
	lower.PolicyDigest = exposureTestDigest(14)
	if err := parent.ReplacePolicy(lower); err != nil { t.Fatal(err) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 0 { t.Fatalf("available = %d", available) }
	if disposition, err := parent.Reserve("provider-a", existingIntent); err != nil || disposition != ExposureAlreadyReserved {
		t.Fatalf("existing exact retry while over cap = %v, %v", disposition, err)
	}
	if _, err := parent.Reserve("provider-a", parentExposureIntent(firstScope, "effect:new", 50)); err != ErrExposureLimitExceeded {
		t.Fatalf("new effect after cap reduction = %v", err)
	}

	// Existing effects remain resolvable while new risk is frozen.
	if err := parent.MarkCommittedUnsettled("provider-a", "effect:existing", parentObservedUnsettled("effect:existing", "dest:a")); err != nil { t.Fatal(err) }
	evidence := parentExposureResolution("effect:existing", "dest:a", ExposureFinalSettlementEvidence)
	if err := parent.MarkSettled("provider-a", "effect:existing", evidence); err != nil { t.Fatal(err) }
	if err := parent.BeginRevocation("provider-b"); err != nil { t.Fatal(err) }
	if err := parent.CloseGrant("provider-b"); err != nil { t.Fatal(err) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 200 { t.Fatalf("available after close = %d", available) }
	if _, err := parent.Reserve("provider-a", parentExposureIntent(firstScope, "effect:new", 50)); err != nil {
		t.Fatalf("new effect after encumbrance returned within cap = %v", err)
	}
}

func TestReducedChildCapBlocksNewEffectsButPreservesExistingRetry(t *testing.T) {
	parent, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	scope := parentExposureChildScope("operator:a", "dest:a")
	existing := parentExposureIntent(scope, "effect:existing", 100)
	_, _ = parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1))
	if _, err := parent.Reserve("provider-a", existing); err != nil { t.Fatal(err) }

	lower := parentExposureTestPolicy(800)
	lower.Sequence = 2
	lower.MaximumChildBudgetMinor = 600
	lower.PolicyDigest = exposureTestDigest(14)
	if err := parent.ReplacePolicy(lower); err != nil { t.Fatal(err) }

	// Exact retry remains idempotent, but this 700-unit grant can no longer
	// authorize a new effect under the reduced 600-unit child ceiling.
	if disposition, err := parent.Reserve("provider-a", existing); err != nil || disposition != ExposureAlreadyReserved {
		t.Fatalf("existing retry = %v, %v", disposition, err)
	}
	if disposition, err := parent.AllocateChildBudget("provider-a", scope, 700, 1, exposureTestDigest(1)); err != nil || disposition != DelegatedBudgetAlreadyAllocated {
		t.Fatalf("exact grant retry after policy reduction = %v, %v", disposition, err)
	}
	if err := parent.MarkDispatchAuthorized("provider-a", "effect:existing"); err != ErrExposureLimitExceeded {
		t.Fatalf("stale child budget dispatched after cap reduction: %v", err)
	}
	if err := parent.CancelBeforeDispatch("provider-a", "effect:existing"); err != nil { t.Fatal(err) }
	if _, err := parent.Reserve("provider-a", parentExposureIntent(scope, "effect:new", 50)); err != ErrExposureLimitExceeded {
		t.Fatalf("new effect under reduced child cap = %v", err)
	}
}

func TestDelegatedSnapshotMatchesSharedKnownAnswer(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-payment-delegated-exposure-v1.kat")
	if err != nil { t.Fatal(err) }
	var expected string
	for _, line := range strings.Split(string(raw), "\n") {
		if strings.HasPrefix(line, "parent_snapshot_sha256=") {
			expected = strings.TrimPrefix(line, "parent_snapshot_sha256=")
			break
		}
	}
	if expected == "" { t.Fatal("missing delegated exposure KAT") }
	parent, err := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	if err != nil { t.Fatal(err) }
	_, err = parent.AllocateChildBudget(
		"provider-a", parentExposureChildScope("operator:a", "dest:a"),
		700, 1, exposureTestDigest(1),
	)
	if err != nil { t.Fatal(err) }
	actual, err := parent.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	if actual != "sha256:"+expected {
		t.Fatalf("delegated snapshot = %s, want sha256:%s", actual, expected)
	}
}

func TestDelegatedSnapshotIsInsertionOrderIndependent(t *testing.T) {
	first, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	second, _ := NewParentDelegatedExposureBookV1(parentExposureTestPolicy(1_000))
	_, _ = first.AllocateChildBudget("provider-a", parentExposureChildScope("operator:a", "dest:a"), 300, 1, exposureTestDigest(1))
	_, _ = first.AllocateChildBudget("provider-b", parentExposureChildScope("operator:b", "dest:b"), 400, 2, exposureTestDigest(2))
	_, _ = second.AllocateChildBudget("provider-b", parentExposureChildScope("operator:b", "dest:b"), 400, 2, exposureTestDigest(2))
	_, _ = second.AllocateChildBudget("provider-a", parentExposureChildScope("operator:a", "dest:a"), 300, 1, exposureTestDigest(1))
	one, err := first.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	two, err := second.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	if one != two { t.Fatalf("snapshot differs by insertion order: %s != %s", one, two) }
}
