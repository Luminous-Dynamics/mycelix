package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

func handoffTestDigest(value byte) string {
	return "sha256:" + strings.Repeat(string(value), 64)
}

func handoffTestPolicy() DelegatedExposurePolicyV1 {
	return DelegatedExposurePolicyV1{
		Scope: ParentExposureScopeV1{
			OwnerID: "owner:treasury", PoolID: "pool:one", Currency: "SAP",
		},
		Sequence: 1, MaximumTotalMinor: 1_000,
		MaximumChildBudgetMinor: 800,
		PolicyDigest: handoffTestDigest('d'),
	}
}

func handoffTestTerms() GrantTermsV1 {
	return GrantTermsV1{
		GrantID: "grant:one",
		ParentScope: handoffTestPolicy().Scope,
		ChildScope: ExposureScopeV1{
			OwnerID: "operator:a", PoolID: "pool:one",
			DestinationID: "dest:a", Currency: "SAP",
		},
		BudgetMinor: 700, GrantSequence: 1,
		ParentPolicySequence: 1, ParentPolicyDigest: handoffTestDigest('d'),
	}
}

func handoffTestIntent(effectID string, amount uint64) ExposureIntentV1 {
	return ExposureIntentV1{
		Scope: handoffTestTerms().ChildScope,
		OperationID: "operation:" + effectID,
		EffectID: effectID,
		IntentDigest: handoffTestDigest('a'),
		PayloadDigest: handoffTestDigest('b'),
		MaximumExposureMinor: amount,
	}
}

func handoffObserved(effectID string) DestinationResolutionV1 {
	return DestinationResolutionV1{
		Kind: ExposureObservedUnsettledEffect,
		EffectID: effectID, DestinationID: "dest:a",
		PayloadDigest: handoffTestDigest('b'),
		ProfileDigest: handoffTestDigest('e'),
		EvidenceDigest: handoffTestDigest('f'),
	}
}

func handoffSettled(effectID string) DestinationResolutionV1 {
	return DestinationResolutionV1{
		Kind: ExposureFinalSettlementEvidence,
		EffectID: effectID, DestinationID: "dest:a",
		PayloadDigest: handoffTestDigest('b'),
		ProfileDigest: handoffTestDigest('c'),
		EvidenceDigest: handoffTestDigest('9'),
	}
}

func handoffAbsence(effectID string) DestinationResolutionV1 {
	return DestinationResolutionV1{
		Kind: ExposureAuthoritativeAbsence,
		EffectID: effectID, DestinationID: "dest:a",
		PayloadDigest: handoffTestDigest('b'),
		ProfileDigest: handoffTestDigest('c'),
		EvidenceDigest: handoffTestDigest('8'),
	}
}

func handoffParentHash() string { return handoffTestDigest('1') }
func handoffAcceptHash() string { return handoffTestDigest('2') }
func handoffRevokeHash() string { return handoffTestDigest('3') }
func handoffObserveHash() string { return handoffTestDigest('4') }
func handoffFrontier() string { return handoffTestDigest('5') }
func handoffCloseHash() string { return handoffTestDigest('6') }

func newHandoffBooks(t *testing.T) (*ParentHandoffBookV1, *ChildHandoffBookV1) {
	t.Helper()
	terms := handoffTestTerms()
	parent, err := NewParentHandoffBookV1(handoffTestPolicy())
	if err != nil { t.Fatal(err) }
	if _, err := parent.IssueGrant(terms, handoffParentHash()); err != nil { t.Fatal(err) }
	child, err := NewChildHandoffBookV1(terms, handoffParentHash())
	if err != nil { t.Fatal(err) }
	return parent, child
}

func TestChildCannotUseGrantBeforeAcceptanceAndParentEscrowIsAtomic(t *testing.T) {
	parent, child := newHandoffBooks(t)
	if allocated, _ := parent.AllocatedBudgetMinor(); allocated != 700 { t.Fatalf("allocated = %d", allocated) }
	if available, _ := parent.AvailableParentBudgetMinor(); available != 300 { t.Fatalf("available = %d", available) }
	if _, err := child.Reserve(handoffTestIntent("effect:1", 100)); err != ErrHandoffInvalidTransition {
		t.Fatalf("reservation before acceptance = %v", err)
	}
}

func TestLostAcceptanceAckKeepsParentEscrowAndRetriesStableDelivery(t *testing.T) {
	parent, child := newHandoffBooks(t)
	pending := parent.PendingDeliveries()
	if len(pending) != 1 || pending[0].StableDeliveryID != "grant:grant:one" {
		t.Fatalf("initial delivery = %+v", pending)
	}
	_, acceptance, err := child.AcceptGrant(handoffParentHash(), handoffAcceptHash())
	if err != nil { t.Fatal(err) }

	// Simulate the ACK being lost: parent outbox still contains the same item.
	restartedParent := parent.Clone()
	if len(restartedParent.PendingDeliveries()) != 1 { t.Fatal("grant outbox was lost on restart") }
	if allocated, _ := restartedParent.AllocatedBudgetMinor(); allocated != 700 {
		t.Fatalf("escrow changed after lost ACK: %d", allocated)
	}

	// Child restart + duplicate delivery returns the original acceptance action.
	restartedChild := child.Clone()
	disposition, replayed, err := restartedChild.AcceptGrant(handoffParentHash(), handoffTestDigest('7'))
	if err != nil || disposition != HandoffAlreadyObserved || replayed != acceptance {
		t.Fatalf("accept replay = %v, %v, %+v", disposition, err, replayed)
	}
	if _, err := restartedChild.Reserve(handoffTestIntent("effect:1", 400)); err != nil { t.Fatal(err) }
	if err := restartedChild.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }

	if disposition, err := restartedParent.ObserveAcceptance(acceptance); err != nil || disposition != HandoffObserved {
		t.Fatalf("acceptance observation = %v, %v", disposition, err)
	}
	if len(restartedParent.PendingDeliveries()) != 0 { t.Fatal("grant outbox not cleared after ACK") }
	if allocated, _ := restartedParent.AllocatedBudgetMinor(); allocated != 700 {
		t.Fatalf("acceptance released parent escrow: %d", allocated)
	}
}

func TestRevocationDeliveryDelayKeepsParentEscrowAndChildUseBounded(t *testing.T) {
	parent, child := newHandoffBooks(t)
	_, acceptance, _ := child.AcceptGrant(handoffParentHash(), handoffAcceptHash())
	_, _ = parent.ObserveAcceptance(acceptance)
	if _, err := parent.RequestRevocation("grant:one", handoffRevokeHash()); err != nil { t.Fatal(err) }
	if pending := parent.PendingDeliveries(); len(pending) != 1 || pending[0].Kind != DeliveryRevocation {
		t.Fatalf("revocation delivery = %+v", pending)
	}
	if _, err := child.Reserve(handoffTestIntent("effect:1", 500)); err != nil { t.Fatal(err) }
	if _, err := child.Reserve(handoffTestIntent("effect:2", 250)); err != ErrExposureLimitExceeded {
		t.Fatalf("child exceeded delegated budget during delayed revoke: %v", err)
	}
	if allocated, _ := parent.AllocatedBudgetMinor(); allocated != 700 {
		t.Fatalf("parent escrow freed before revoke observed: %d", allocated)
	}

	_, observed, err := child.ObserveRevocation(handoffRevokeHash(), handoffObserveHash())
	if err != nil { t.Fatal(err) }
	if _, err := parent.ObserveRevocation(observed); err != nil { t.Fatal(err) }
	if len(parent.PendingDeliveries()) != 0 { t.Fatal("revocation outbox not cleared") }
	if _, err := child.Reserve(handoffTestIntent("effect:3", 10)); err != ErrHandoffInvalidTransition {
		t.Fatalf("new child reservation after local revoke barrier = %v", err)
	}
}

func TestClosureWaitsForResolutionAndParentReleaseBindsChildCertificate(t *testing.T) {
	parent, child := newHandoffBooks(t)
	_, acceptance, _ := child.AcceptGrant(handoffParentHash(), handoffAcceptHash())
	_, _ = parent.ObserveAcceptance(acceptance)
	_, _ = parent.RequestRevocation("grant:one", handoffRevokeHash())
	_, observed, _ := child.ObserveRevocation(handoffRevokeHash(), handoffObserveHash())
	_, _ = parent.ObserveRevocation(observed)

	if _, err := child.Reserve(handoffTestIntent("effect:1", 500)); err != nil { t.Fatal(err) }
	if err := child.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }
	if err := child.MarkIndeterminate("effect:1"); err != nil { t.Fatal(err) }
	if _, _, err := child.Close(handoffFrontier(), handoffCloseHash()); err != ErrHandoffUnresolved {
		t.Fatalf("closed with unresolved exposure: %v", err)
	}
	if err := child.ReleaseAuthoritativeAbsence("effect:1", handoffAbsence("effect:1")); err != nil { t.Fatal(err) }
	_, closure, err := child.Close(handoffFrontier(), handoffCloseHash())
	if err != nil { t.Fatal(err) }

	wrong := closure
	wrong.ChildOwnerID = "operator:other"
	if _, err := parent.ReleaseGrant(wrong); err != ErrHandoffInvalidGrant {
		t.Fatalf("wrong child released parent budget: %v", err)
	}
	if allocated, _ := parent.AllocatedBudgetMinor(); allocated != 700 { t.Fatalf("wrong receipt freed escrow: %d", allocated) }
	if disposition, err := parent.ReleaseGrant(closure); err != nil || disposition != HandoffReleased {
		t.Fatalf("parent release = %v, %v", disposition, err)
	}
	if allocated, _ := parent.AllocatedBudgetMinor(); allocated != 0 { t.Fatalf("released budget = %d", allocated) }
	clone := parent.Clone()
	if disposition, err := clone.ReleaseGrant(closure); err != nil || disposition != HandoffAlreadyReleased {
		t.Fatalf("duplicate parent release = %v, %v", disposition, err)
	}
}

func TestUnacceptedGrantCanBeRevokedAndClosedWithoutChildSpend(t *testing.T) {
	parent, child := newHandoffBooks(t)
	_, err := parent.RequestRevocation("grant:one", handoffRevokeHash())
	if err != nil { t.Fatal(err) }
	_, observed, err := child.ObserveRevocation(handoffRevokeHash(), handoffObserveHash())
	if err != nil { t.Fatal(err) }
	if _, err := parent.ObserveRevocation(observed); err != nil { t.Fatal(err) }
	_, closure, err := child.Close(handoffFrontier(), handoffCloseHash())
	if err != nil { t.Fatal(err) }
	if closure.ChildAcceptActionHash != nil { t.Fatal("unaccepted grant claimed an acceptance action") }
	if disposition, err := parent.ReleaseGrant(closure); err != nil || disposition != HandoffReleased {
		t.Fatalf("release unaccepted grant = %v, %v", disposition, err)
	}
	if allocated, _ := parent.AllocatedBudgetMinor(); allocated != 0 { t.Fatalf("allocated = %d", allocated) }
}

func TestAlreadyDispatchedEffectCanResolveAfterRevocation(t *testing.T) {
	parent, child := newHandoffBooks(t)
	_, acceptance, _ := child.AcceptGrant(handoffParentHash(), handoffAcceptHash())
	_, _ = parent.ObserveAcceptance(acceptance)
	_, _ = child.Reserve(handoffTestIntent("effect:1", 400))
	if err := child.MarkDispatchAuthorized("effect:1"); err != nil { t.Fatal(err) }
	_, _ = parent.RequestRevocation("grant:one", handoffRevokeHash())
	_, observed, _ := child.ObserveRevocation(handoffRevokeHash(), handoffObserveHash())
	_, _ = parent.ObserveRevocation(observed)
	if err := child.MarkCommittedUnsettled("effect:1", handoffObserved("effect:1")); err != nil { t.Fatal(err) }
	if err := child.MarkSettled("effect:1", handoffSettled("effect:1")); err != nil { t.Fatal(err) }
	_, closure, err := child.Close(handoffFrontier(), handoffCloseHash())
	if err != nil { t.Fatal(err) }
	if _, err := parent.ReleaseGrant(closure); err != nil { t.Fatal(err) }
}

func TestLateAcceptanceAndRevocationAcksRemainIdempotentAfterRelease(t *testing.T) {
	parent, child := newHandoffBooks(t)
	_, acceptance, _ := child.AcceptGrant(handoffParentHash(), handoffAcceptHash())
	_, _ = parent.RequestRevocation("grant:one", handoffRevokeHash())
	_, observed, _ := child.ObserveRevocation(handoffRevokeHash(), handoffObserveHash())
	_, closure, _ := child.Close(handoffFrontier(), handoffCloseHash())
	if _, err := parent.ReleaseGrant(closure); err != nil { t.Fatal(err) }
	if disposition, err := parent.ObserveAcceptance(acceptance); err != nil || disposition != HandoffAlreadyObserved {
		t.Fatalf("late acceptance = %v, %v", disposition, err)
	}
	if disposition, err := parent.ObserveRevocation(observed); err != nil || disposition != HandoffAlreadyObserved {
		t.Fatalf("late revoke ACK = %v, %v", disposition, err)
	}
}

func TestParentAndChildRestartKeepDurableOutboxAndReceiptIdentities(t *testing.T) {
	parent, child := newHandoffBooks(t)
	parent = parent.Clone()
	child = child.Clone()
	if len(parent.PendingDeliveries()) != 1 { t.Fatal("parent grant outbox not restored") }
	_, acceptance, err := child.AcceptGrant(handoffParentHash(), handoffAcceptHash())
	if err != nil { t.Fatal(err) }
	parent.ObserveAcceptance(acceptance)
	_, _ = parent.RequestRevocation("grant:one", handoffRevokeHash())
	_, observed, _ := child.ObserveRevocation(handoffRevokeHash(), handoffObserveHash())
	parent.ObserveRevocation(observed)
	_, closure, err := child.Close(handoffFrontier(), handoffCloseHash())
	if err != nil { t.Fatal(err) }

	parent = parent.Clone()
	child = child.Clone()
	if disposition, err := parent.ReleaseGrant(closure); err != nil || disposition != HandoffReleased {
		t.Fatalf("release after restart = %v, %v", disposition, err)
	}
}

func TestHandoffKnownAnswerMatchesSharedFixture(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-payment-handoff-v1.kat")
	if err != nil { t.Fatal(err) }
	values := make(map[string]string)
	for _, line := range strings.Split(string(raw), "\n") {
		if strings.HasPrefix(line, "#") || line == "" { continue }
		parts := strings.SplitN(line, "=", 2)
		if len(parts) == 2 { values[parts[0]] = parts[1] }
	}
	terms := handoffTestTerms()
	grantDigest, err := terms.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	if grantDigest != values["grant_digest"] { t.Fatalf("grant digest = %s", grantDigest) }
	parent, _ := NewParentHandoffBookV1(handoffTestPolicy())
	if _, err := parent.IssueGrant(terms, handoffParentHash()); err != nil { t.Fatal(err) }
	parentDigest, err := parent.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	if parentDigest != values["parent_snapshot_sha256"] { t.Fatalf("parent snapshot = %s", parentDigest) }
	child, _ := NewChildHandoffBookV1(terms, handoffParentHash())
	exposureDigest, err := child.exposure.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	if exposureDigest != values["child_exposure_snapshot_sha256"] { t.Fatalf("child exposure snapshot = %s", exposureDigest) }
	childDigest, err := child.CanonicalSnapshotDigest()
	if err != nil { t.Fatal(err) }
	if childDigest != values["child_handoff_snapshot_sha256"] { t.Fatalf("child snapshot = %s", childDigest) }
}
