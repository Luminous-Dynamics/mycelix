package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"unicode/utf8"
)

const (
	providerContractDomainV1 = "MXSAP-PROVIDER-CONTRACT-V1\x00"
	providerOperationKeyDomainV1 = "MXSAP-PROVIDER-OPERATION-KEY-V1\x00"
	providerObservationDomainV1 = "MXSAP-PROVIDER-OBSERVATION-V1\x00"
	providerAbsenceDomainV1 = "MXSAP-PROVIDER-ABSENCE-EVIDENCE-V1\x00"
	providerMaxFieldBytesV1 = 4096
	providerDayMSV1 uint64 = 24 * 60 * 60 * 1000
)

type ProviderKeyScopeV1 uint8

const (
	ProviderCompanyAccountScopeV1 ProviderKeyScopeV1 = iota + 1
	ProviderMerchantAccountScopeV1
	ProviderPaymentOperationScopeV1
)

type ProviderRegionScopeV1 uint8

const (
	ProviderGlobalRegionScopeV1 ProviderRegionScopeV1 = iota + 1
	ProviderPerRegionScopeV1
)

type ProviderProfileMaturityV1 uint8

const (
	ProviderDocumentationBaselineV1 ProviderProfileMaturityV1 = iota + 1
	ProviderSandboxQualifiedV1
	ProviderProductionQualifiedV1
)

type ProviderChangedPayloadBehaviorV1 uint8

const (
	ProviderRejectChangedPayloadV1 ProviderChangedPayloadBehaviorV1 = iota + 1
	ProviderReplayOriginalResponseV1
	ProviderChangedPayloadUnspecifiedV1
)

type ProviderInProgressBehaviorV1 uint8

const (
	ProviderRetryOnlyWithTransientSignalV1 ProviderInProgressBehaviorV1 = iota + 1
	ProviderNeverRetryWhileInProgressV1
	ProviderInProgressUnspecifiedV1
)

type ProviderPartialRefundPolicyV1 uint8

const (
	ProviderPartialRefundUnsupportedV1 ProviderPartialRefundPolicyV1 = iota + 1
	ProviderPartialRefundSupportedV1
	ProviderPartialRefundPaymentMethodDependentV1
)

type ProviderRefundCeilingV1 uint8

const (
	ProviderCapturedAmountAggregateV1 ProviderRefundCeilingV1 = iota + 1
	ProviderProviderReportedRemainingV1
	ProviderRefundCeilingUnknownV1
)

type ProviderAbsencePolicyV1 uint8

const (
	ProviderAbsenceUnsupportedV1 ProviderAbsencePolicyV1 = iota + 1
	ProviderCompleteLookupAfterHorizonV1
)

type ProviderAttemptClassV1 uint8

const (
	ProviderTransportTimeoutUnknownV1 ProviderAttemptClassV1 = iota + 1
	ProviderInProgressWithTransientSignalV1
	ProviderInProgressWithoutTransientSignalV1
	ProviderDefinitiveNoEffectV1
	ProviderPermanentErrorV1
	ProviderAcceptedAttemptV1
)

type ProviderStatusV1 uint8

const (
	ProviderUnknownV1 ProviderStatusV1 = iota
	ProviderInProgressV1
	ProviderAcceptedV1
	ProviderSettledV1
	ProviderRejectedV1
	ProviderReturnedV1
	ProviderReversedV1
	ProviderAuthoritativelyAbsentV1
)

var (
	ErrProviderInvalidContractV1 = errors.New("provider contract V1 is invalid")
	ErrProviderInvalidOperationKeyV1 = errors.New("provider operation key V1 is invalid")
	ErrProviderScopeMismatchV1 = errors.New("provider account scope mismatch")
	ErrProviderRegionMismatchV1 = errors.New("provider region mismatch")
	ErrProviderChangedRequestDigestV1 = errors.New("provider retry changed request digest")
	ErrProviderClockMovedBackwardsV1 = errors.New("provider retry clock moved backwards")
	ErrProviderRetentionGuaranteeElapsedV1 = errors.New("provider minimum retention guarantee elapsed")
	ErrProviderIdempotencyUnsupportedV1 = errors.New("provider does not support idempotent POST")
	ErrProviderInProgressNotRetryableV1 = errors.New("provider in-progress outcome is not retryable")
	ErrProviderAttemptNotRetryableV1 = errors.New("provider attempt class is not retryable")
	ErrProviderProfileNotQualifiedV1 = errors.New("provider profile is not production-qualified")
	ErrProviderObservationInvalidSubjectV1 = errors.New("provider observation subject is invalid")
	ErrProviderObservationInvalidDigestV1 = errors.New("provider observation digest is invalid")
	ErrProviderObservationScopeMismatchV1 = errors.New("provider observation scope mismatch")
	ErrProviderObservationInvalidTimestampV1 = errors.New("provider observation timestamp is invalid")
	ErrProviderAbsenceUnsupportedV1 = errors.New("provider authoritative absence is not supported")
	ErrProviderAbsenceRequiresDedicatedEvidenceV1 = errors.New("authoritative absence requires dedicated lookup evidence")
	ErrProviderAbsenceHorizonNotElapsedV1 = errors.New("provider absence visibility horizon not elapsed")
	ErrProviderObservationAfterNowV1 = errors.New("provider observation is after current time")
	ErrProviderTransitionInvalidV1 = errors.New("provider status transition is invalid")
	ErrProviderTransitionContradictionV1 = errors.New("provider terminal status contradiction")
	ErrProviderObservationChangedV1 = errors.New("provider observation changed under same status")
)

type ProviderContractV1 struct {
	ProviderID string
	ProfileID string
	APIVersion string
	AccountScope string
	KeyScope ProviderKeyScopeV1
	RegionScope ProviderRegionScopeV1
	MaximumKeyCharacters uint16
	IdempotentPOST bool
	RetentionMinimumMS uint64
	RetentionMaximumMS uint64
	InProgressBehavior ProviderInProgressBehaviorV1
	ChangedPayloadBehavior ProviderChangedPayloadBehaviorV1
	PartialRefundPolicy ProviderPartialRefundPolicyV1
	RefundCeiling ProviderRefundCeilingV1
	AbsencePolicy ProviderAbsencePolicyV1
	AbsenceVisibilityHorizonMS uint64
	ProfileMaturity ProviderProfileMaturityV1
	WebhookAuthProfile string
}

func (c ProviderContractV1) Validate() error {
	for _, value := range []string{
		c.ProviderID, c.ProfileID, c.APIVersion, c.AccountScope, c.WebhookAuthProfile,
	} {
		if err := validateProviderTextV1(value); err != nil { return err }
	}
	if c.KeyScope < ProviderCompanyAccountScopeV1 || c.KeyScope > ProviderPaymentOperationScopeV1 ||
		c.RegionScope < ProviderGlobalRegionScopeV1 || c.RegionScope > ProviderPerRegionScopeV1 ||
		c.InProgressBehavior < ProviderRetryOnlyWithTransientSignalV1 || c.InProgressBehavior > ProviderInProgressUnspecifiedV1 ||
		c.ChangedPayloadBehavior < ProviderRejectChangedPayloadV1 || c.ChangedPayloadBehavior > ProviderChangedPayloadUnspecifiedV1 ||
		c.PartialRefundPolicy < ProviderPartialRefundUnsupportedV1 || c.PartialRefundPolicy > ProviderPartialRefundPaymentMethodDependentV1 ||
		c.RefundCeiling < ProviderCapturedAmountAggregateV1 || c.RefundCeiling > ProviderRefundCeilingUnknownV1 ||
		c.AbsencePolicy < ProviderAbsenceUnsupportedV1 || c.AbsencePolicy > ProviderCompleteLookupAfterHorizonV1 ||
		c.ProfileMaturity < ProviderDocumentationBaselineV1 || c.ProfileMaturity > ProviderProductionQualifiedV1 {
		return ErrProviderInvalidContractV1
	}
	if c.IdempotentPOST {
		if c.MaximumKeyCharacters == 0 || c.RetentionMinimumMS == 0 ||
			c.RetentionMaximumMS < c.RetentionMinimumMS {
			return ErrProviderInvalidContractV1
		}
	} else if c.RetentionMinimumMS != 0 || c.RetentionMaximumMS != 0 || c.MaximumKeyCharacters != 0 {
		return ErrProviderInvalidContractV1
	}
	switch c.AbsencePolicy {
	case ProviderAbsenceUnsupportedV1:
		if c.AbsenceVisibilityHorizonMS != 0 { return ErrProviderInvalidContractV1 }
	case ProviderCompleteLookupAfterHorizonV1:
		if c.AbsenceVisibilityHorizonMS == 0 { return ErrProviderInvalidContractV1 }
	}
	return nil
}

func (c ProviderContractV1) CanonicalDigest() (string, error) {
	if err := c.Validate(); err != nil { return "", err }
	output := []byte(providerContractDomainV1)
	var err error
	for _, value := range []string{c.ProviderID, c.ProfileID, c.APIVersion, c.AccountScope} {
		output, err = appendProviderTextV1(output, value)
		if err != nil { return "", err }
	}
	output = append(output, byte(c.KeyScope), byte(c.RegionScope))
	output = appendProviderU64V1(output, uint64(c.MaximumKeyCharacters))
	if c.IdempotentPOST { output = append(output, 1) } else { output = append(output, 0) }
	output = appendProviderU64V1(output, c.RetentionMinimumMS)
	output = appendProviderU64V1(output, c.RetentionMaximumMS)
	output = append(output, byte(c.InProgressBehavior), byte(c.ChangedPayloadBehavior))
	output = append(output, byte(c.PartialRefundPolicy))
	output = append(output, byte(c.RefundCeiling), byte(c.AbsencePolicy))
	output = appendProviderU64V1(output, c.AbsenceVisibilityHorizonMS)
	output = append(output, byte(c.ProfileMaturity))
	output, err = appendProviderTextV1(output, c.WebhookAuthProfile)
	if err != nil { return "", err }
	return providerSHA256V1(output), nil
}

// AdyenDocumentationBaselineV1 is intentionally documentation-only. The API
// version, webhook signature profile and authoritative-absence capability
// must be explicitly pinned before it can authorize production dispatch.
func AdyenDocumentationBaselineV1(accountScope string) ProviderContractV1 {
	return ProviderContractV1{
		ProviderID: "adyen",
		ProfileID: "adyen-idempotency-docs-2026-11-11",
		APIVersion: "operator-pinned-required",
		AccountScope: accountScope,
		KeyScope: ProviderCompanyAccountScopeV1,
		RegionScope: ProviderPerRegionScopeV1,
		MaximumKeyCharacters: 64,
		IdempotentPOST: true,
		RetentionMinimumMS: 7 * providerDayMSV1,
		RetentionMaximumMS: 14 * providerDayMSV1,
		InProgressBehavior: ProviderRetryOnlyWithTransientSignalV1,
		ChangedPayloadBehavior: ProviderChangedPayloadUnspecifiedV1,
		PartialRefundPolicy: ProviderPartialRefundPaymentMethodDependentV1,
		RefundCeiling: ProviderCapturedAmountAggregateV1,
		AbsencePolicy: ProviderAbsenceUnsupportedV1,
		ProfileMaturity: ProviderDocumentationBaselineV1,
		WebhookAuthProfile: "operator-pinned-required",
	}
}

func (c ProviderContractV1) RequireLiveDispatch() error {
	if err := c.Validate(); err != nil { return err }
	if c.ProfileMaturity != ProviderProductionQualifiedV1 ||
		c.APIVersion == "operator-pinned-required" ||
		c.WebhookAuthProfile == "operator-pinned-required" {
		return ErrProviderProfileNotQualifiedV1
	}
	return nil
}

type ProviderOperationKeyV1 struct {
	ProviderID string
	ProfileID string
	APIVersion string
	AccountScope string
	Region string
	IdempotencyKey string
	RequestDigest string
	CreatedAtUnixMS uint64
	ContractDigest string
}

func (k ProviderOperationKeyV1) Validate(contract ProviderContractV1) error {
	if err := contract.Validate(); err != nil { return ErrProviderInvalidContractV1 }
	if !contract.IdempotentPOST { return ErrProviderIdempotencyUnsupportedV1 }
	if k.ProviderID != contract.ProviderID || k.ProfileID != contract.ProfileID ||
		k.APIVersion != contract.APIVersion || k.AccountScope != contract.AccountScope {
		return ErrProviderScopeMismatchV1
	}
	regionShapeOK := (contract.RegionScope == ProviderGlobalRegionScopeV1 && k.Region == "global") ||
		(contract.RegionScope == ProviderPerRegionScopeV1 && k.Region != "")
	if !regionShapeOK { return ErrProviderRegionMismatchV1 }
	contractDigest, err := contract.CanonicalDigest()
	if err != nil { return ErrProviderInvalidContractV1 }
	if k.IdempotencyKey == "" || utf8.RuneCountInString(k.IdempotencyKey) > int(contract.MaximumKeyCharacters) ||
		k.CreatedAtUnixMS == 0 || !validProviderDigestV1(k.RequestDigest) || k.ContractDigest != contractDigest {
		return ErrProviderInvalidOperationKeyV1
	}
	return nil
}

func (k ProviderOperationKeyV1) CanonicalDigest() (string, error) {
	for _, value := range []string{
		k.ProviderID, k.ProfileID, k.APIVersion, k.AccountScope, k.Region, k.IdempotencyKey,
	} {
		if err := validateProviderTextV1(value); err != nil { return "", err }
	}
	if !validProviderDigestV1(k.RequestDigest) || !validProviderDigestV1(k.ContractDigest) || k.CreatedAtUnixMS == 0 {
		return "", ErrProviderInvalidOperationKeyV1
	}
	output := []byte(providerOperationKeyDomainV1)
	var err error
	for _, value := range []string{
		k.ProviderID, k.ProfileID, k.APIVersion, k.AccountScope, k.Region,
		k.IdempotencyKey, k.RequestDigest, k.ContractDigest,
	} {
		output, err = appendProviderTextV1(output, value)
		if err != nil { return "", err }
	}
	output = appendProviderU64V1(output, k.CreatedAtUnixMS)
	return providerSHA256V1(output), nil
}

func (k ProviderOperationKeyV1) RetrySameRequest(
	contract ProviderContractV1,
	currentAccountScope string,
	currentRegion string,
	currentRequestDigest string,
	attempt ProviderAttemptClassV1,
	nowUnixMS uint64,
) error {
	if err := contract.Validate(); err != nil { return ErrProviderInvalidContractV1 }
	if !contract.IdempotentPOST { return ErrProviderIdempotencyUnsupportedV1 }
	if err := k.Validate(contract); err != nil { return err }
	if currentAccountScope != k.AccountScope { return ErrProviderScopeMismatchV1 }
	if contract.RegionScope == ProviderPerRegionScopeV1 && currentRegion != k.Region {
		return ErrProviderRegionMismatchV1
	}
	if currentRequestDigest != k.RequestDigest { return ErrProviderChangedRequestDigestV1 }
	if nowUnixMS < k.CreatedAtUnixMS { return ErrProviderClockMovedBackwardsV1 }
	if nowUnixMS-k.CreatedAtUnixMS >= contract.RetentionMinimumMS {
		return ErrProviderRetentionGuaranteeElapsedV1
	}
	switch attempt {
	case ProviderTransportTimeoutUnknownV1:
		return nil
	case ProviderInProgressWithTransientSignalV1:
		if contract.InProgressBehavior == ProviderRetryOnlyWithTransientSignalV1 { return nil }
		return ErrProviderInProgressNotRetryableV1
	case ProviderInProgressWithoutTransientSignalV1:
		return ErrProviderInProgressNotRetryableV1
	default:
		return ErrProviderAttemptNotRetryableV1
	}
}

type ProviderObservationV1 struct {
	ProviderID string
	ProfileID string
	APIVersion string
	AccountScope string
	Region string
	IdempotencyKey string
	RequestDigest string
	Status ProviderStatusV1
	ProviderObjectID string
	EventID string
	ObservedAtUnixMS uint64
	EvidenceReference string
	EvidenceDigest string
}

func (o ProviderObservationV1) ValidateAgainstKey(
	contract ProviderContractV1,
	key ProviderOperationKeyV1,
	nowUnixMS uint64,
) error {
	if err := key.Validate(contract); err != nil { return ErrProviderObservationScopeMismatchV1 }
	if o.ProviderID != key.ProviderID || o.ProfileID != key.ProfileID ||
		o.APIVersion != key.APIVersion || o.AccountScope != key.AccountScope ||
		o.Region != key.Region || o.IdempotencyKey != key.IdempotencyKey ||
		o.RequestDigest != key.RequestDigest {
		return ErrProviderObservationScopeMismatchV1
	}
	if o.ObservedAtUnixMS == 0 || o.ObservedAtUnixMS > nowUnixMS {
		return ErrProviderObservationInvalidTimestampV1
	}
	if o.Status == ProviderAuthoritativelyAbsentV1 {
		return ErrProviderAbsenceRequiresDedicatedEvidenceV1
	}
	if !validProviderDigestV1(o.RequestDigest) || !validProviderDigestV1(o.EvidenceDigest) {
		return ErrProviderObservationInvalidDigestV1
	}
	for _, value := range []string{o.ProviderObjectID, o.EventID, o.EvidenceReference} {
		if err := validateProviderTextV1(value); err != nil { return err }
	}
	return nil
}

func (o ProviderObservationV1) CanonicalDigest() (string, error) {
	for _, value := range []string{
		o.ProviderID, o.ProfileID, o.APIVersion, o.AccountScope, o.Region,
		o.IdempotencyKey, o.ProviderObjectID, o.EventID, o.EvidenceReference,
	} {
		if err := validateProviderTextV1(value); err != nil { return "", err }
	}
	if !validProviderDigestV1(o.RequestDigest) || !validProviderDigestV1(o.EvidenceDigest) ||
		o.ObservedAtUnixMS == 0 || o.Status > ProviderReversedV1 {
		return "", ErrProviderObservationInvalidSubjectV1
	}
	output := []byte(providerObservationDomainV1)
	var err error
	for _, value := range []string{
		o.ProviderID, o.ProfileID, o.APIVersion, o.AccountScope, o.Region,
		o.IdempotencyKey, o.RequestDigest, o.ProviderObjectID, o.EventID,
		o.EvidenceReference, o.EvidenceDigest,
	} {
		output, err = appendProviderTextV1(output, value)
		if err != nil { return "", err }
	}
	output = append(output, byte(o.Status))
	output = appendProviderU64V1(output, o.ObservedAtUnixMS)
	return providerSHA256V1(output), nil
}

type ProviderAbsenceEvidenceV1 struct {
	ProviderID string
	ProfileID string
	APIVersion string
	AccountScope string
	Region string
	IdempotencyKey string
	RequestDigest string
	LookupReference string
	CompleteThroughUnixMS uint64
	ObservedAtUnixMS uint64
	EvidenceDigest string
}

func (a ProviderAbsenceEvidenceV1) CanonicalDigest() (string, error) {
	for _, value := range []string{
		a.ProviderID, a.ProfileID, a.APIVersion, a.AccountScope, a.Region,
		a.IdempotencyKey, a.LookupReference,
	} {
		if err := validateProviderTextV1(value); err != nil { return "", err }
	}
	if !validProviderDigestV1(a.RequestDigest) || !validProviderDigestV1(a.EvidenceDigest) ||
		a.CompleteThroughUnixMS == 0 || a.ObservedAtUnixMS == 0 {
		return "", ErrProviderObservationInvalidSubjectV1
	}
	output := []byte(providerAbsenceDomainV1)
	var err error
	for _, value := range []string{
		a.ProviderID, a.ProfileID, a.APIVersion, a.AccountScope, a.Region,
		a.IdempotencyKey, a.RequestDigest, a.LookupReference, a.EvidenceDigest,
	} {
		output, err = appendProviderTextV1(output, value)
		if err != nil { return "", err }
	}
	output = appendProviderU64V1(output, a.CompleteThroughUnixMS)
	output = appendProviderU64V1(output, a.ObservedAtUnixMS)
	return providerSHA256V1(output), nil
}

func (a ProviderAbsenceEvidenceV1) ValidateAsAuthoritativeAbsence(
	contract ProviderContractV1,
	key ProviderOperationKeyV1,
	nowUnixMS uint64,
) error {
	if err := key.Validate(contract); err != nil { return ErrProviderObservationScopeMismatchV1 }
	if contract.AbsencePolicy != ProviderCompleteLookupAfterHorizonV1 {
		return ErrProviderAbsenceUnsupportedV1
	}
	if a.ProviderID != key.ProviderID || a.ProfileID != key.ProfileID ||
		a.APIVersion != key.APIVersion || a.AccountScope != key.AccountScope ||
		a.Region != key.Region || a.IdempotencyKey != key.IdempotencyKey ||
		a.RequestDigest != key.RequestDigest {
		return ErrProviderObservationScopeMismatchV1
	}
	if a.ObservedAtUnixMS == 0 || a.ObservedAtUnixMS > nowUnixMS ||
		a.CompleteThroughUnixMS > a.ObservedAtUnixMS {
		return ErrProviderObservationAfterNowV1
	}
	requiredThrough, ok := checkedProviderAddV1(key.CreatedAtUnixMS, contract.AbsenceVisibilityHorizonMS)
	if !ok { return ErrProviderObservationInvalidTimestampV1 }
	if a.CompleteThroughUnixMS < requiredThrough {
		return ErrProviderAbsenceHorizonNotElapsedV1
	}
	if _, err := a.CanonicalDigest(); err != nil { return err }
	return nil
}

func ValidateProviderTransitionV1(
	previous ProviderStatusV1,
	next ProviderStatusV1,
	sameObservationDigest bool,
) error {
	if previous > ProviderAuthoritativelyAbsentV1 || next > ProviderAuthoritativelyAbsentV1 {
		return ErrProviderTransitionInvalidV1
	}
	if previous == next && sameObservationDigest { return nil }
	allowed := false
	switch previous {
	case ProviderUnknownV1:
		allowed = next == ProviderInProgressV1 || next == ProviderAcceptedV1 ||
			next == ProviderSettledV1 || next == ProviderRejectedV1
	case ProviderInProgressV1:
		allowed = next == ProviderAcceptedV1 || next == ProviderSettledV1 ||
			next == ProviderRejectedV1
	case ProviderAcceptedV1:
		allowed = next == ProviderSettledV1 || next == ProviderRejectedV1
	case ProviderSettledV1:
		allowed = next == ProviderReturnedV1 || next == ProviderReversedV1
	}
	if allowed { return nil }
	switch previous {
	case ProviderRejectedV1, ProviderReturnedV1, ProviderReversedV1, ProviderAuthoritativelyAbsentV1:
		return ErrProviderTransitionContradictionV1
	default:
		return ErrProviderTransitionInvalidV1
	}
}

func validateProviderTextV1(value string) error {
	if value == "" || len(value) > providerMaxFieldBytesV1 || !utf8.ValidString(value) {
		return ErrProviderObservationInvalidSubjectV1
	}
	return nil
}

func validProviderDigestV1(value string) bool {
	if len(value) != 71 || value[:7] != "sha256:" { return false }
	for i := 7; i < len(value); i++ {
		c := value[i]
		if !((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) { return false }
	}
	return true
}

func appendProviderTextV1(output []byte, value string) ([]byte, error) {
	if err := validateProviderTextV1(value); err != nil { return nil, err }
	var size [4]byte
	binary.BigEndian.PutUint32(size[:], uint32(len(value)))
	output = append(output, size[:]...)
	output = append(output, value...)
	return output, nil
}

func appendProviderU64V1(output []byte, value uint64) []byte {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	return append(output, encoded[:]...)
}

func checkedProviderAddV1(left, right uint64) (uint64, bool) {
	if right > ^uint64(0)-left { return 0, false }
	return left + right, true
}

func providerSHA256V1(data []byte) string {
	digest := sha256.Sum256(data)
	return "sha256:" + hex.EncodeToString(digest[:])
}
