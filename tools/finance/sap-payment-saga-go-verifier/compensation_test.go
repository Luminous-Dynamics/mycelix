package sagaverifier

import (
	"fmt"
	"os"
	"strings"
	"sync"
	"testing"
)

const compensationTestNowV2 uint64 = 1_000

func compensationTestDigestV2(ch byte) string {
	return "sha256:" + stringsRepeatCompensationV2(ch, 64)
}

func compensationTestForwardV2() ForwardEffectV2 {
	return ForwardEffectV2{
		OperationID: "operation:merchant-01",
		EffectID: "effect:payment-01",
		SourceOwnerID: "owner:merchant",
		SourceAccountID: "account:merchant",
		DestinationOwnerID: "owner:customer",
		DestinationAccountID: "account:customer",
		Asset: "USD",
		AmountMinor: 100,
		Finality: ForwardSettledV2,
		ReversibleCeilingMinor: 100,
	}
}

func compensationTestPlanV2(forward ForwardEffectV2, maximum uint64) CompensationPlanV2 {
	digest, _ := forward.CanonicalDigest()
	return CompensationPlanV2{
		PlanID: "plan:refund-01",
		ForwardEffectID: forward.EffectID,
		ForwardEffectDigest: digest,
		FundingOwnerID: "owner:refund-treasury",
		FundingAccountID: "account:refund-treasury",
		BeneficiaryOwnerID: forward.SourceOwnerID,
		BeneficiaryAccountID: forward.SourceAccountID,
		Asset: forward.Asset,
		MaximumCompensationMinor: maximum,
		PolicyDigest: compensationTestDigestV2('d'),
		RecoveryReason: "customer-refund",
	}
}

func compensationTestAuthorizationV2(plan CompensationPlanV2, maximum uint64) CompensationAuthorizationV2 {
	planDigest, _ := plan.CanonicalDigest(compensationTestForwardV2())
	return CompensationAuthorizationV2{
		CapabilityID: "capability:refund-01",
		IssuerOwnerID: plan.FundingOwnerID,
		GranteeOwnerID: "owner:refund-operator",
		PlanDigest: planDigest,
		FundingOwnerID: plan.FundingOwnerID,
		FundingAccountID: plan.FundingAccountID,
		BeneficiaryOwnerID: plan.BeneficiaryOwnerID,
		BeneficiaryAccountID: plan.BeneficiaryAccountID,
		Asset: plan.Asset,
		MaximumMinor: maximum,
		Nonce: 1,
		ExpiresAtUnixMS: 10_000,
		Revoked: false,
	}
}

func newCompensationTestBookV2(t *testing.T, liquidity, planMaximum uint64) *CompensationBookV2 {
	t.Helper()
	forward := compensationTestForwardV2()
	plan := compensationTestPlanV2(forward, planMaximum)
	book, err := NewCompensationBookV2(forward, plan, liquidity)
	if err != nil { t.Fatal(err) }
	return book
}

func compensationTestRequestV2(
	book *CompensationBookV2,
	id string,
	amount uint64,
	reservationID string,
	authMaximum uint64,
) CompensationRequestV2 {
	plan := book.Plan()
	return CompensationRequestV2{
		CompensationID: id,
		ActorOwnerID: "owner:refund-operator",
		AmountMinor: amount,
		Authorization: compensationTestAuthorizationV2(plan, authMaximum),
		LiquidityReservation: LiquidityReservationV2{
			ReservationID: reservationID,
			SourceOwnerID: plan.FundingOwnerID,
			SourceAccountID: plan.FundingAccountID,
			Asset: plan.Asset,
			AmountMinor: amount,
		},
	}
}

func compensationTestOutcomeV2(
	t *testing.T,
	book *CompensationBookV2,
	request CompensationRequestV2,
	kind CompensationOutcomeKindV2,
	evidenceReference string,
) CompensationOutcomeV2 {
	t.Helper()
	forward := book.Forward()
	forwardDigest, err := forward.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	plan := book.Plan()
	authDigest, err := request.Authorization.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	reservationDigest, err := request.LiquidityReservation.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	return CompensationOutcomeV2{
		Kind: kind,
		CompensationID: request.CompensationID,
		CompensationEffectID: "mx-compensation:" + request.CompensationID,
		ForwardEffectID: forward.EffectID,
		ForwardEffectDigest: forwardDigest,
		PlanDigest: book.PlanDigest(),
		AmountMinor: request.AmountMinor,
		AuthorizationDigest: authDigest,
		LiquidityReservationDigest: reservationDigest,
		FundingOwnerID: plan.FundingOwnerID,
		FundingAccountID: plan.FundingAccountID,
		BeneficiaryOwnerID: plan.BeneficiaryOwnerID,
		BeneficiaryAccountID: plan.BeneficiaryAccountID,
		Asset: plan.Asset,
		EvidenceReference: evidenceReference,
	}
}

func stringsRepeatCompensationV2(value byte, count int) string {
	result := make([]byte, count)
	for i := range result { result[i] = value }
	return string(result)
}


func TestCompensationSharedV2KnownAnswers(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-compensation-v2.kat")
	if err != nil { t.Fatal(err) }
	values := map[string]string{}
	for _, line := range strings.Split(string(raw), "\n") {
		if line == "" || strings.HasPrefix(line, "#") { continue }
		parts := strings.SplitN(line, "=", 2)
		if len(parts) != 2 { t.Fatalf("invalid KAT line: %q", line) }
		values[parts[0]] = parts[1]
	}
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:kat-01", 60, "liquidity:kat-01", 100)
	outcome := compensationTestOutcomeV2(t, book, request, CompensationFinalSettlementV2, "evidence:kat-settlement")
	for name, actual := range map[string]string{
		"forward": mustCompensationDigestV2(t, book.Forward().CanonicalDigest()),
		"plan": book.PlanDigest(),
		"authorization_scope": mustCompensationDigestV2(t, request.Authorization.CanonicalScopeDigest()),
		"authorization": mustCompensationDigestV2(t, request.Authorization.CanonicalDigest()),
		"liquidity": mustCompensationDigestV2(t, request.LiquidityReservation.CanonicalDigest()),
		"request": mustCompensationDigestV2(t, request.CanonicalDigest()),
		"outcome": mustCompensationDigestV2(t, outcome.CanonicalDigest()),
	} {
		if values[name] != actual { t.Fatalf("%s digest = %s, want %s", name, actual, values[name]) }
	}
}

func mustCompensationDigestV2(t *testing.T, value string, err error) string {
	t.Helper()
	if err != nil { t.Fatal(err) }
	return value
}

func TestCompensationCapabilityIDCannotChangeScopeToResetLimitV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	first := compensationTestRequestV2(book, "refund:cap-scope-1", 40, "liquidity:cap-scope-1", 70)
	if _, err := book.Reserve(first, compensationTestNowV2); err != nil { t.Fatal(err) }
	second := compensationTestRequestV2(book, "refund:cap-scope-2", 40, "liquidity:cap-scope-2", 100)
	second.Authorization.CapabilityID = "capability:refund-01"
	if _, err := book.Reserve(second, compensationTestNowV2); err != ErrCompensationCapabilityIdentityConflictV2 {
		t.Fatalf("capability ID scope change = %v", err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 40 { t.Fatalf("encumbered = %d", encumbered) }
}

func TestCompensationRejectsInvalidForwardFinalityDiscriminantV2(t *testing.T) {
	forward := compensationTestForwardV2()
	forward.Finality = 0
	if err := forward.Validate(); err != ErrCompensationInvalidFinalityV2 {
		t.Fatalf("invalid finality = %v", err)
	}
}

func TestCompensationForwardMustBeSettledV2(t *testing.T) {
	unsettled := compensationTestForwardV2()
	unsettled.Finality = ForwardObservedUnsettledV2
	unsettled.ReversibleCeilingMinor = 0
	plan := compensationTestPlanV2(unsettled, 0)
	if _, err := NewCompensationBookV2(unsettled, plan, 100); err != ErrCompensationForwardNotSettledV2 {
		t.Fatalf("unsettled forward was accepted: %v", err)
	}

	absent := compensationTestForwardV2()
	absent.Finality = ForwardAuthoritativelyAbsentV2
	absent.ReversibleCeilingMinor = 0
	plan = compensationTestPlanV2(absent, 0)
	if _, err := NewCompensationBookV2(absent, plan, 100); err != ErrCompensationForwardNotSettledV2 {
		t.Fatalf("absent forward was accepted: %v", err)
	}
}

func TestCompensationPartialRefundCannotExceedReversibleCeilingV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	originalForward := book.Forward()
	first := compensationTestRequestV2(book, "refund:1", 60, "liquidity:1", 100)
	if disposition, err := book.Reserve(first, compensationTestNowV2); err != nil || disposition != CompensationCreatedV2 {
		t.Fatalf("reserve first = %v, %v", disposition, err)
	}
	if _, err := book.MarkDispatchAuthorized("refund:1", "owner:refund-operator", first.Authorization, compensationTestNowV2); err != nil {
		t.Fatal(err)
	}
	outcome := compensationTestOutcomeV2(t, book, first, CompensationFinalSettlementV2, "evidence:refund-1-settled")
	if disposition, err := book.Settle(outcome); err != nil || disposition != CompensationIsSettledV2 {
		t.Fatalf("settle first = %v, %v", disposition, err)
	}
	if settled, _ := book.SettledMinor(); settled != 60 { t.Fatalf("settled = %d", settled) }
	if encumbered, _ := book.EncumberedMinor(); encumbered != 0 { t.Fatalf("encumbered = %d", encumbered) }
	if remaining, _ := book.RemainingReversibleMinor(); remaining != 40 { t.Fatalf("remaining = %d", remaining) }

	second := compensationTestRequestV2(book, "refund:2", 41, "liquidity:2", 100)
	if _, err := book.Reserve(second, compensationTestNowV2); err != ErrCompensationReversibleCeilingV2 {
		t.Fatalf("over-ceiling refund result = %v", err)
	}
	if book.Forward() != originalForward { t.Fatal("compensation mutated forward effect") }
}

func TestCompensationIndeterminateKeepsEncumbranceAndRejectsFalseAbsenceV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:uncertain", 35, "liquidity:uncertain", 100)
	_, err := book.Reserve(request, compensationTestNowV2)
	if err != nil { t.Fatal(err) }
	if _, err = book.MarkDispatchAuthorized("refund:uncertain", "owner:refund-operator", request.Authorization, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err = book.MarkIndeterminate("refund:uncertain", 1); err != nil { t.Fatal(err) }
	if encumbered, _ := book.EncumberedMinor(); encumbered != 35 { t.Fatalf("encumbered = %d", encumbered) }

	absence := compensationTestOutcomeV2(t, book, request, CompensationAuthoritativeAbsenceV2, "evidence:absence-contradictory")
	if _, err = book.ResolveAuthoritativeAbsence(absence); err != ErrCompensationOutcomeContradictionV2 {
		t.Fatalf("contradictory absence = %v", err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 35 { t.Fatalf("absence released encumbrance = %d", encumbered) }

	observed := compensationTestOutcomeV2(t, book, request, CompensationObservedUnsettledV2, "evidence:refund-observed")
	if disposition, err := book.ObserveUnsettled(observed); err != nil || disposition != CompensationAppliedV2 {
		t.Fatalf("observe unsettled = %v, %v", disposition, err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 35 { t.Fatalf("observed unsettled encumbrance = %d", encumbered) }

	settlement := compensationTestOutcomeV2(t, book, request, CompensationFinalSettlementV2, "evidence:refund-settled")
	if disposition, err := book.Settle(settlement); err != nil || disposition != CompensationIsSettledV2 {
		t.Fatalf("settle = %v, %v", disposition, err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 0 { t.Fatalf("encumbered after settlement = %d", encumbered) }
	if settled, _ := book.SettledMinor(); settled != 35 { t.Fatalf("settled = %d", settled) }
}

func TestCompensationAuthoritativeAbsencePermitsSameEffectRetryV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:retry", 30, "liquidity:retry", 100)
	if _, err := book.Reserve(request, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.MarkDispatchAuthorized("refund:retry", request.ActorOwnerID, request.Authorization, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.MarkIndeterminate("refund:retry", 0); err != nil { t.Fatal(err) }
	absence := compensationTestOutcomeV2(t, book, request, CompensationAuthoritativeAbsenceV2, "evidence:refund-absent")
	if _, err := book.ResolveAuthoritativeAbsence(absence); err != nil { t.Fatal(err) }
	if disposition, err := book.RetrySameEffect("refund:retry", request.ActorOwnerID, request.Authorization, compensationTestNowV2); err != nil || disposition != CompensationAppliedV2 {
		t.Fatalf("retry = %v, %v", disposition, err)
	}
	effectID, ok := book.CompensationEffectID("refund:retry")
	if !ok || effectID != "mx-compensation:refund:retry" { t.Fatalf("effect identity = %s", effectID) }
	if status, _ := book.Status("refund:retry"); status != CompensationReservedV2 { t.Fatalf("status = %v", status) }
	if encumbered, _ := book.EncumberedMinor(); encumbered != 30 { t.Fatalf("encumbered = %d", encumbered) }
}

func TestCompensationExactRetryIdempotentAfterExpiryV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:replay", 20, "liquidity:replay", 100)
	if disposition, err := book.Reserve(request, compensationTestNowV2); err != nil || disposition != CompensationCreatedV2 {
		t.Fatalf("initial reserve = %v, %v", disposition, err)
	}
	if disposition, err := book.Reserve(request, 20_000); err != nil || disposition != CompensationAlreadyExistsV2 {
		t.Fatalf("expired exact replay = %v, %v", disposition, err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 20 { t.Fatalf("encumbered = %d", encumbered) }
}

func TestCompensationReservationIDCannotBeReusedV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	first := compensationTestRequestV2(book, "refund:first", 20, "liquidity:shared", 100)
	second := compensationTestRequestV2(book, "refund:second", 20, "liquidity:shared", 100)
	if _, err := book.Reserve(first, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.Reserve(second, compensationTestNowV2); err != ErrCompensationLiquidityReservationConflictV2 {
		t.Fatalf("duplicate reservation = %v", err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 20 { t.Fatalf("encumbered = %d", encumbered) }
}

func TestCompensationAuthorizationCumulativeCeilingV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	first := compensationTestRequestV2(book, "refund:cap-1", 40, "liquidity:cap-1", 70)
	second := compensationTestRequestV2(book, "refund:cap-2", 31, "liquidity:cap-2", 70)
	if _, err := book.Reserve(first, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.Reserve(second, compensationTestNowV2); err != ErrCompensationAuthorizationLimitV2 {
		t.Fatalf("capability cumulative overuse = %v", err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 40 { t.Fatalf("encumbered = %d", encumbered) }
}

func TestCompensationActorAndDispatchGranteeAreBoundV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:actor", 20, "liquidity:actor", 100)
	request.ActorOwnerID = "owner:mallory"
	if _, err := book.Reserve(request, compensationTestNowV2); err != ErrCompensationAuthorizationScopeV2 {
		t.Fatalf("wrong reserve actor = %v", err)
	}

	request = compensationTestRequestV2(book, "refund:dispatch-actor", 20, "liquidity:dispatch-actor", 100)
	if _, err := book.Reserve(request, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.MarkDispatchAuthorized("refund:dispatch-actor", "owner:mallory", request.Authorization, compensationTestNowV2); err != ErrCompensationCallerNotAuthorizedV2 {
		t.Fatalf("wrong dispatch actor = %v", err)
	}
	if status, _ := book.Status("refund:dispatch-actor"); status != CompensationReservedV2 { t.Fatalf("status = %v", status) }
}

func TestCompensationRevokedAndExpiredCapabilityCannotDispatchV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:revoked", 20, "liquidity:revoked", 100)
	if _, err := book.Reserve(request, compensationTestNowV2); err != nil { t.Fatal(err) }
	revoked := request.Authorization
	revoked.Revoked = true
	if _, err := book.MarkDispatchAuthorized("refund:revoked", request.ActorOwnerID, revoked, compensationTestNowV2); err != ErrCompensationAuthorizationRevokedV2 {
		t.Fatalf("revoked capability = %v", err)
	}
	if _, err := book.MarkDispatchAuthorized("refund:revoked", request.ActorOwnerID, request.Authorization, request.Authorization.ExpiresAtUnixMS); err != ErrCompensationAuthorizationExpiredV2 {
		t.Fatalf("expired capability = %v", err)
	}
	if status, _ := book.Status("refund:revoked"); status != CompensationReservedV2 { t.Fatalf("status = %v", status) }
}

func TestCompensationLiquidityAndOutcomeBindingV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 50, 100)
	tooLarge := compensationTestRequestV2(book, "refund:large", 51, "liquidity:large", 100)
	if _, err := book.Reserve(tooLarge, compensationTestNowV2); err != ErrCompensationInsufficientLiquidityV2 {
		t.Fatalf("insufficient liquidity = %v", err)
	}
	request := compensationTestRequestV2(book, "refund:bound", 20, "liquidity:bound", 100)
	if _, err := book.Reserve(request, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.MarkDispatchAuthorized("refund:bound", request.ActorOwnerID, request.Authorization, compensationTestNowV2); err != nil { t.Fatal(err) }

	outcome := compensationTestOutcomeV2(t, book, request, CompensationFinalSettlementV2, "evidence:comp-settled")
	outcome.AuthorizationDigest = compensationTestDigestV2('a')
	if _, err := book.Settle(outcome); err != ErrCompensationInvalidOutcomeV2 {
		t.Fatalf("wrong auth receipt = %v", err)
	}
	outcome = compensationTestOutcomeV2(t, book, request, CompensationFinalSettlementV2, "evidence:comp-settled")
	outcome.ForwardEffectID = "effect:other"
	if _, err := book.Settle(outcome); err != ErrCompensationInvalidOutcomeV2 {
		t.Fatalf("wrong forward subject = %v", err)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 20 { t.Fatalf("encumbered = %d", encumbered) }
}

func TestCompensationCancellationReleasesOnlyBeforeDispatchV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	request := compensationTestRequestV2(book, "refund:cancel", 25, "liquidity:cancel", 100)
	if _, err := book.Reserve(request, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.CancelBeforeDispatch("refund:cancel", request.ActorOwnerID); err != nil { t.Fatal(err) }
	if encumbered, _ := book.EncumberedMinor(); encumbered != 0 { t.Fatalf("encumbered after cancel = %d", encumbered) }

	dispatched := compensationTestRequestV2(book, "refund:dispatched", 25, "liquidity:dispatched", 100)
	if _, err := book.Reserve(dispatched, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.MarkDispatchAuthorized("refund:dispatched", dispatched.ActorOwnerID, dispatched.Authorization, compensationTestNowV2); err != nil { t.Fatal(err) }
	if _, err := book.CancelBeforeDispatch("refund:dispatched", dispatched.ActorOwnerID); err != ErrCompensationInvalidTransitionV2 {
		t.Fatalf("cancel after dispatch = %v", err)
	}
}

func TestConcurrentCompensationReservationsRespectSharedCeilingsV2(t *testing.T) {
	book := newCompensationTestBookV2(t, 100, 100)
	const workers = 50
	var wg sync.WaitGroup
	var mu sync.Mutex
	created := 0
	rejected := 0
	for index := 0; index < workers; index++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			id := fmt.Sprintf("refund:concurrent:%02d", i)
			reservation := fmt.Sprintf("liquidity:concurrent:%02d", i)
			request := compensationTestRequestV2(book, id, 3, reservation, 100)
			_, err := book.Reserve(request, compensationTestNowV2)
			mu.Lock()
			defer mu.Unlock()
			if err == nil { created++ } else { rejected++ }
		}(index)
	}
	wg.Wait()
	if created != 33 || rejected != workers-created {
		t.Fatalf("created=%d rejected=%d", created, rejected)
	}
	if encumbered, _ := book.EncumberedMinor(); encumbered != 99 { t.Fatalf("encumbered = %d", encumbered) }
}
