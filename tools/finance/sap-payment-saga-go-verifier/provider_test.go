package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

const providerTestNowMSV1 uint64 = 100_000_000

func providerTestDigestV1(ch byte) string {
	return "sha256:" + strings.Repeat(string(ch), 64)
}

func providerTestAdyenContractV1() ProviderContractV1 {
	return AdyenDocumentationBaselineV1("company:test-account")
}

func providerTestOperationKeyV1(contract ProviderContractV1) ProviderOperationKeyV1 {
	contractDigest, _ := contract.CanonicalDigest()
	return ProviderOperationKeyV1{
		ProviderID: contract.ProviderID,
		ProfileID: contract.ProfileID,
		APIVersion: contract.APIVersion,
		AccountScope: contract.AccountScope,
		Region: "eu",
		IdempotencyKey: "idem-00000000-0000-4000-8000-000000000001",
		RequestDigest: providerTestDigestV1('a'),
		CreatedAtUnixMS: 1_000,
		ContractDigest: contractDigest,
	}
}

func providerTestObservationV1(key ProviderOperationKeyV1) ProviderObservationV1 {
	return ProviderObservationV1{
		ProviderID: key.ProviderID,
		ProfileID: key.ProfileID,
		APIVersion: key.APIVersion,
		AccountScope: key.AccountScope,
		Region: key.Region,
		IdempotencyKey: key.IdempotencyKey,
		RequestDigest: key.RequestDigest,
		Status: ProviderAcceptedV1,
		ProviderObjectID: "psp:123",
		EventID: "event:456",
		ObservedAtUnixMS: 2_000,
		EvidenceReference: "provider-event:456",
		EvidenceDigest: providerTestDigestV1('b'),
	}
}

func providerTestAbsenceV1(key ProviderOperationKeyV1) ProviderAbsenceEvidenceV1 {
	return ProviderAbsenceEvidenceV1{
		ProviderID: key.ProviderID,
		ProfileID: key.ProfileID,
		APIVersion: key.APIVersion,
		AccountScope: key.AccountScope,
		Region: key.Region,
		IdempotencyKey: key.IdempotencyKey,
		RequestDigest: key.RequestDigest,
		LookupReference: "lookup:123",
		CompleteThroughUnixMS: 3_000,
		ObservedAtUnixMS: 4_000,
		EvidenceDigest: providerTestDigestV1('c'),
	}
}

func TestProviderV1SharedKnownAnswers(t *testing.T) {
	raw, err := os.ReadFile("../sap-payment-saga-model/fixtures/sap-provider-contract-v1.kat")
	if err != nil { t.Fatal(err) }
	expected := map[string]string{}
	for _, line := range strings.Split(string(raw), "\n") {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") { continue }
		fields := strings.SplitN(line, "=", 2)
		if len(fields) != 2 { t.Fatalf("invalid KAT line %q", line) }
		expected[fields[0]] = fields[1]
	}
	contract := providerTestAdyenContractV1()
	key := providerTestOperationKeyV1(contract)
	observation := providerTestObservationV1(key)
	absence := providerTestAbsenceV1(key)
	contractDigest, err := contract.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	keyDigest, err := key.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	observationDigest, err := observation.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	absenceDigest, err := absence.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	permit, err := key.QualifyRetry(
		contract, "company:test-account", "eu", key.RequestDigest,
		ProviderTransportTimeoutUnknownV1, 2_000,
	)
	if err != nil { t.Fatal(err) }
	permitDigest, err := permit.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	actual := map[string]string{
		"contract": contractDigest,
		"operation_key": keyDigest,
		"observation": observationDigest,
		"absence_subject": absenceDigest,
		"retry_permit": permitDigest,
	}
	for name, value := range actual {
		if expected[name] != value {
			t.Fatalf("%s digest=%s want=%s", name, value, expected[name])
		}
	}
}

func TestAdyenDocumentationBaselineIsNotLiveQualifiedV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	if err := contract.Validate(); err != nil { t.Fatal(err) }
	if err := contract.RequireLiveDispatch(); err != ErrProviderProfileNotQualifiedV1 {
		t.Fatalf("documentation profile live dispatch result = %v", err)
	}
	if contract.KeyScope != ProviderCompanyAccountScopeV1 ||
		contract.RegionScope != ProviderPerRegionScopeV1 ||
		contract.MaximumKeyCharacters != 64 ||
		contract.PartialRefundPolicy != ProviderPartialRefundPaymentMethodDependentV1 ||
		contract.RetentionMinimumMS != 7*providerDayMSV1 ||
		contract.RetentionMaximumMS != 14*providerDayMSV1 {
		t.Fatalf("unexpected Adyen documented constraints: %+v", contract)
	}
}

func TestProviderRetryIsBoundToAccountRegionRequestAndMinimumRetentionV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	key := providerTestOperationKeyV1(contract)
	cases := []struct {
		name string
		account string
		region string
		request string
		attempt ProviderAttemptClassV1
		now uint64
		want error
	}{
		{"timeout-within-retention", "company:test-account", "eu", key.RequestDigest, ProviderTransportTimeoutUnknownV1, key.CreatedAtUnixMS + 1, nil},
		{"transient-header-true", "company:test-account", "eu", key.RequestDigest, ProviderInProgressWithTransientSignalV1, key.CreatedAtUnixMS + 1, nil},
		{"transient-header-false", "company:test-account", "eu", key.RequestDigest, ProviderInProgressWithoutTransientSignalV1, key.CreatedAtUnixMS + 1, ErrProviderInProgressNotRetryableV1},
		{"wrong-region", "company:test-account", "us", key.RequestDigest, ProviderTransportTimeoutUnknownV1, key.CreatedAtUnixMS + 1, ErrProviderRegionMismatchV1},
		{"wrong-account", "company:other", "eu", key.RequestDigest, ProviderTransportTimeoutUnknownV1, key.CreatedAtUnixMS + 1, ErrProviderScopeMismatchV1},
		{"changed-body", "company:test-account", "eu", providerTestDigestV1('d'), ProviderTransportTimeoutUnknownV1, key.CreatedAtUnixMS + 1, ErrProviderChangedRequestDigestV1},
		{"retention-minimum", "company:test-account", "eu", key.RequestDigest, ProviderTransportTimeoutUnknownV1, key.CreatedAtUnixMS + 7*providerDayMSV1, ErrProviderRetentionGuaranteeElapsedV1},
		{"clock-reversal", "company:test-account", "eu", key.RequestDigest, ProviderTransportTimeoutUnknownV1, key.CreatedAtUnixMS - 1, ErrProviderClockMovedBackwardsV1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			err := key.RetrySameRequest(contract, tc.account, tc.region, tc.request, tc.attempt, tc.now)
			if err != tc.want { t.Fatalf("retry error=%v want=%v", err, tc.want) }
		})
	}
}

func TestProviderOperationKeyUsesUnicodeCharacterCountV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	contract.MaximumKeyCharacters = 2
	contractDigest, err := contract.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	key := providerTestOperationKeyV1(contract)
	key.IdempotencyKey = "éé"
	key.ContractDigest = contractDigest
	if err := key.Validate(contract); err != nil { t.Fatalf("two Unicode characters rejected: %v", err) }
	key.IdempotencyKey = "ééé"
	if err := key.Validate(contract); err != ErrProviderInvalidOperationKeyV1 {
		t.Fatalf("three Unicode characters result = %v", err)
	}
}

func TestProviderAbsenceRequiresSupportedCompleteLookupAndHorizonV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	key := providerTestOperationKeyV1(contract)
	evidence := providerTestAbsenceV1(key)
	if err := evidence.ValidateAsAuthoritativeAbsence(contract, key, providerTestNowMSV1); err != ErrProviderAbsenceUnsupportedV1 {
		t.Fatalf("Adyen absence result = %v", err)
	}

	contract.ProfileID = "test-provider-complete-lookup-v1"
	contract.AbsencePolicy = ProviderCompleteLookupAfterHorizonV1
	contract.AbsenceVisibilityHorizonMS = 2_000
	key = providerTestOperationKeyV1(contract)
	evidence = providerTestAbsenceV1(key)
	evidence.CompleteThroughUnixMS = 2_999
	if err := evidence.ValidateAsAuthoritativeAbsence(contract, key, providerTestNowMSV1); err != ErrProviderAbsenceHorizonNotElapsedV1 {
		t.Fatalf("early absence result = %v", err)
	}
	evidence.CompleteThroughUnixMS = 3_000
	if err := evidence.ValidateAsAuthoritativeAbsence(contract, key, providerTestNowMSV1); err != nil {
		t.Fatalf("complete absence rejected: %v", err)
	}
	evidence.Region = "us"
	if err := evidence.ValidateAsAuthoritativeAbsence(contract, key, providerTestNowMSV1); err != ErrProviderObservationScopeMismatchV1 {
		t.Fatalf("wrong-region absence result = %v", err)
	}
}

func TestProviderObservationCannotClaimAuthoritativeAbsenceV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	key := providerTestOperationKeyV1(contract)
	observation := providerTestObservationV1(key)
	observation.Status = ProviderAuthoritativelyAbsentV1
	if err := observation.ValidateAgainstKey(contract, key, providerTestNowMSV1); err != ErrProviderAbsenceRequiresDedicatedEvidenceV1 {
		t.Fatalf("ordinary absence observation result = %v", err)
	}
	if _, err := observation.CanonicalDigest(); err != ErrProviderObservationInvalidSubjectV1 {
		t.Fatalf("canonical ordinary absence digest result = %v", err)
	}
}

func TestProviderStatusTransitionsRequireSameDigestForReplaysV1(t *testing.T) {
	cases := []struct {
		previous ProviderStatusV1
		next ProviderStatusV1
		sameDigest bool
		want error
	}{
		{ProviderInProgressV1, ProviderInProgressV1, true, nil},
		{ProviderInProgressV1, ProviderInProgressV1, false, ErrProviderTransitionInvalidV1},
		{ProviderInProgressV1, ProviderSettledV1, false, nil},
		{ProviderSettledV1, ProviderInProgressV1, false, ErrProviderTransitionInvalidV1},
		{ProviderRejectedV1, ProviderSettledV1, false, ErrProviderTransitionContradictionV1},
		{ProviderSettledV1, ProviderReturnedV1, false, nil},
	}
	for _, tc := range cases {
		if err := ValidateProviderTransitionV1(tc.previous, tc.next, tc.sameDigest); err != tc.want {
			t.Fatalf("transition %d->%d error=%v want=%v", tc.previous, tc.next, err, tc.want)
		}
	}
}

func TestProviderContractDigestChangesWhenSemanticsChangeV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	before, err := contract.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	contract.RegionScope = ProviderGlobalRegionScopeV1
	after, err := contract.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	if before == after { t.Fatal("provider contract digest did not bind region scope") }
}


func TestProviderRetryPermitIsDigestBoundAndExpiresAtGuaranteeBoundaryV1(t *testing.T) {
	contract := providerTestAdyenContractV1()
	key := providerTestOperationKeyV1(contract)
	permit, err := key.QualifyRetry(
		contract,
		"company:test-account",
		"eu",
		key.RequestDigest,
		ProviderTransportTimeoutUnknownV1,
		2_000,
	)
	if err != nil { t.Fatal(err) }
	if permit.ValidUntilUnixMS != 1_000+7*providerDayMSV1 {
		t.Fatalf("permit expiry=%d", permit.ValidUntilUnixMS)
	}
	digest, err := permit.CanonicalDigest()
	if err != nil { t.Fatal(err) }
	if digest != "sha256:b90b3840d91826f4f79949b8d5872b20de988a677c3be0d62e1b3e86f54c0352" {
		t.Fatalf("permit digest=%s", digest)
	}
	if err := permit.ValidateAgainst(key, contract, 2_000); err != nil { t.Fatal(err) }

	changedRegion := permit
	changedRegion.Region = "us"
	if err := changedRegion.ValidateAgainst(key, contract, 2_000); err != ErrProviderRetryPermitMismatchV1 {
		t.Fatalf("changed region permit=%v", err)
	}
	if err := permit.ValidateAgainst(key, contract, 1_000+7*providerDayMSV1); err != ErrProviderRetryPermitExpiredV1 {
		t.Fatalf("expired permit=%v", err)
	}
}
