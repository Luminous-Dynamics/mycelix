package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"sort"
	"sync"
	"unicode/utf8"
)

const (
	compensationForwardDomainV2   = "MXSAP-FORWARD-EFFECT-V2\x00"
	compensationPlanDomainV2      = "MXSAP-COMPENSATION-PLAN-V2\x00"
	compensationAuthDomainV2      = "MXSAP-COMPENSATION-AUTH-V2\x00"
	compensationLiquidityDomainV2 = "MXSAP-COMPENSATION-LIQUIDITY-V2\x00"
	compensationRequestDomainV2   = "MXSAP-COMPENSATION-REQUEST-V2\x00"
	compensationOutcomeDomainV2   = "MXSAP-COMPENSATION-OUTCOME-V2\x00"
	maxCompensationFieldBytesV2   = 4096
)

type ForwardFinalityV2 uint8

const (
	ForwardIndeterminateV2 ForwardFinalityV2 = iota + 1
	ForwardObservedUnsettledV2
	ForwardSettledV2
	ForwardAuthoritativelyAbsentV2
)

type ForwardEffectV2 struct {
	OperationID            string
	EffectID               string
	SourceOwnerID          string
	SourceAccountID        string
	DestinationOwnerID     string
	DestinationAccountID   string
	Asset                  string
	AmountMinor            uint64
	Finality               ForwardFinalityV2
	ReversibleCeilingMinor uint64
}

func (f ForwardEffectV2) Validate() error {
	if f.Finality < ForwardIndeterminateV2 || f.Finality > ForwardAuthoritativelyAbsentV2 {
		return ErrCompensationInvalidFinalityV2
	}
	for _, value := range []string{
		f.OperationID, f.EffectID, f.SourceOwnerID, f.SourceAccountID,
		f.DestinationOwnerID, f.DestinationAccountID, f.Asset,
	} {
		if err := validateCompensationTextV2(value); err != nil {
			return err
		}
	}
	if f.SourceOwnerID == f.DestinationOwnerID && f.SourceAccountID == f.DestinationAccountID {
		return ErrCompensationInvalidForwardEffectV2
	}
	if f.AmountMinor == 0 || f.ReversibleCeilingMinor > f.AmountMinor {
		return ErrCompensationInvalidForwardEffectV2
	}
	if f.Finality != ForwardSettledV2 && f.ReversibleCeilingMinor != 0 {
		return ErrCompensationForwardNotSettledV2
	}
	return nil
}

func (f ForwardEffectV2) CanonicalDigest() (string, error) {
	if err := f.Validate(); err != nil {
		return "", err
	}
	output := []byte(compensationForwardDomainV2)
	var err error
	for _, value := range []string{
		f.OperationID, f.EffectID, f.SourceOwnerID, f.SourceAccountID,
		f.DestinationOwnerID, f.DestinationAccountID, f.Asset,
	} {
		output, err = appendCompensationTextV2(output, value)
		if err != nil {
			return "", err
		}
	}
	output = appendCompensationU64V2(output, f.AmountMinor)
	output = append(output, byte(f.Finality))
	output = appendCompensationU64V2(output, f.ReversibleCeilingMinor)
	return compensationSHA256V2(output), nil
}

type CompensationPlanV2 struct {
	PlanID                   string
	ForwardEffectID          string
	ForwardEffectDigest      string
	FundingOwnerID           string
	FundingAccountID         string
	BeneficiaryOwnerID       string
	BeneficiaryAccountID     string
	Asset                    string
	MaximumCompensationMinor uint64
	PolicyDigest             string
	RecoveryReason           string
}

func (p CompensationPlanV2) Validate(forward ForwardEffectV2) error {
	if err := forward.Validate(); err != nil {
		return err
	}
	for _, value := range []string{
		p.PlanID, p.ForwardEffectID, p.FundingOwnerID, p.FundingAccountID,
		p.BeneficiaryOwnerID, p.BeneficiaryAccountID, p.Asset, p.RecoveryReason,
	} {
		if err := validateCompensationTextV2(value); err != nil {
			return err
		}
	}
	if err := validateCompensationDigestV2(p.ForwardEffectDigest); err != nil {
		return err
	}
	if err := validateCompensationDigestV2(p.PolicyDigest); err != nil {
		return err
	}
	if forward.Finality != ForwardSettledV2 {
		return ErrCompensationForwardNotSettledV2
	}
	forwardDigest, err := forward.CanonicalDigest()
	if err != nil {
		return err
	}
	if p.ForwardEffectID != forward.EffectID ||
		p.ForwardEffectDigest != forwardDigest ||
		p.BeneficiaryOwnerID != forward.SourceOwnerID ||
		p.BeneficiaryAccountID != forward.SourceAccountID ||
		p.Asset != forward.Asset ||
		p.MaximumCompensationMinor == 0 ||
		p.MaximumCompensationMinor > forward.ReversibleCeilingMinor {
		return ErrCompensationInvalidPlanV2
	}
	return nil
}

func (p CompensationPlanV2) CanonicalDigest(forward ForwardEffectV2) (string, error) {
	if err := p.Validate(forward); err != nil {
		return "", err
	}
	output := []byte(compensationPlanDomainV2)
	var err error
	for _, value := range []string{
		p.PlanID, p.ForwardEffectID, p.ForwardEffectDigest,
		p.FundingOwnerID, p.FundingAccountID, p.BeneficiaryOwnerID,
		p.BeneficiaryAccountID, p.Asset, p.PolicyDigest, p.RecoveryReason,
	} {
		output, err = appendCompensationTextV2(output, value)
		if err != nil {
			return "", err
		}
	}
	output = appendCompensationU64V2(output, p.MaximumCompensationMinor)
	return compensationSHA256V2(output), nil
}

type CompensationAuthorizationV2 struct {
	CapabilityID         string
	IssuerOwnerID        string
	GranteeOwnerID       string
	PlanDigest           string
	FundingOwnerID       string
	FundingAccountID     string
	BeneficiaryOwnerID   string
	BeneficiaryAccountID string
	Asset                string
	MaximumMinor         uint64
	Nonce                uint64
	ExpiresAtUnixMS      uint64
	Revoked              bool
}

func (a CompensationAuthorizationV2) Validate(
	plan CompensationPlanV2,
	planDigest string,
	amountMinor uint64,
	actorOwnerID string,
	nowUnixMS uint64,
) error {
	for _, value := range []string{
		a.CapabilityID, a.IssuerOwnerID, a.GranteeOwnerID, a.FundingOwnerID,
		a.FundingAccountID, a.BeneficiaryOwnerID, a.BeneficiaryAccountID, a.Asset,
	} {
		if err := validateCompensationTextV2(value); err != nil {
			return err
		}
	}
	if err := validateCompensationDigestV2(a.PlanDigest); err != nil {
		return err
	}
	if a.Revoked {
		return ErrCompensationAuthorizationRevokedV2
	}
	if a.ExpiresAtUnixMS <= nowUnixMS {
		return ErrCompensationAuthorizationExpiredV2
	}
	if a.GranteeOwnerID != actorOwnerID ||
		a.Nonce == 0 || a.MaximumMinor == 0 || amountMinor == 0 ||
		amountMinor > a.MaximumMinor || a.PlanDigest != planDigest ||
		a.IssuerOwnerID != plan.FundingOwnerID ||
		a.FundingOwnerID != plan.FundingOwnerID ||
		a.FundingAccountID != plan.FundingAccountID ||
		a.BeneficiaryOwnerID != plan.BeneficiaryOwnerID ||
		a.BeneficiaryAccountID != plan.BeneficiaryAccountID ||
		a.Asset != plan.Asset ||
		a.MaximumMinor > plan.MaximumCompensationMinor {
		return ErrCompensationAuthorizationScopeV2
	}
	return nil
}

// CanonicalScopeDigest excludes only the revocation bit so a revoked copy of
// the same capability is recognized as revoked, not as a different capability.
func (a CompensationAuthorizationV2) CanonicalScopeDigest() (string, error) {
	for _, value := range []string{
		a.CapabilityID, a.IssuerOwnerID, a.GranteeOwnerID, a.PlanDigest,
		a.FundingOwnerID, a.FundingAccountID, a.BeneficiaryOwnerID,
		a.BeneficiaryAccountID, a.Asset,
	} {
		if err := validateCompensationTextV2(value); err != nil {
			return "", err
		}
	}
	if err := validateCompensationDigestV2(a.PlanDigest); err != nil {
		return "", err
	}
	output := []byte(compensationAuthDomainV2)
	var err error
	for _, value := range []string{
		a.CapabilityID, a.IssuerOwnerID, a.GranteeOwnerID, a.PlanDigest,
		a.FundingOwnerID, a.FundingAccountID, a.BeneficiaryOwnerID,
		a.BeneficiaryAccountID, a.Asset,
	} {
		output, err = appendCompensationTextV2(output, value)
		if err != nil {
			return "", err
		}
	}
	output = appendCompensationU64V2(output, a.MaximumMinor)
	output = appendCompensationU64V2(output, a.Nonce)
	output = appendCompensationU64V2(output, a.ExpiresAtUnixMS)
	return compensationSHA256V2(output), nil
}

func (a CompensationAuthorizationV2) CanonicalDigest() (string, error) {
	for _, value := range []string{
		a.CapabilityID, a.IssuerOwnerID, a.GranteeOwnerID, a.PlanDigest,
		a.FundingOwnerID, a.FundingAccountID, a.BeneficiaryOwnerID,
		a.BeneficiaryAccountID, a.Asset,
	} {
		if err := validateCompensationTextV2(value); err != nil {
			return "", err
		}
	}
	if err := validateCompensationDigestV2(a.PlanDigest); err != nil {
		return "", err
	}
	output := []byte(compensationAuthDomainV2)
	var err error
	for _, value := range []string{
		a.CapabilityID, a.IssuerOwnerID, a.GranteeOwnerID, a.PlanDigest,
		a.FundingOwnerID, a.FundingAccountID, a.BeneficiaryOwnerID,
		a.BeneficiaryAccountID, a.Asset,
	} {
		output, err = appendCompensationTextV2(output, value)
		if err != nil {
			return "", err
		}
	}
	output = appendCompensationU64V2(output, a.MaximumMinor)
	output = appendCompensationU64V2(output, a.Nonce)
	output = appendCompensationU64V2(output, a.ExpiresAtUnixMS)
	if a.Revoked {
		output = append(output, 1)
	} else {
		output = append(output, 0)
	}
	return compensationSHA256V2(output), nil
}

type LiquidityReservationV2 struct {
	ReservationID   string
	SourceOwnerID   string
	SourceAccountID string
	Asset           string
	AmountMinor     uint64
}

func (r LiquidityReservationV2) Validate(plan CompensationPlanV2, amountMinor uint64) error {
	for _, value := range []string{r.ReservationID, r.SourceOwnerID, r.SourceAccountID, r.Asset} {
		if err := validateCompensationTextV2(value); err != nil {
			return err
		}
	}
	if r.SourceOwnerID != plan.FundingOwnerID ||
		r.SourceAccountID != plan.FundingAccountID ||
		r.Asset != plan.Asset ||
		r.AmountMinor != amountMinor ||
		r.AmountMinor == 0 {
		return ErrCompensationLiquidityReservationMismatchV2
	}
	return nil
}

func (r LiquidityReservationV2) CanonicalDigest() (string, error) {
	for _, value := range []string{r.ReservationID, r.SourceOwnerID, r.SourceAccountID, r.Asset} {
		if err := validateCompensationTextV2(value); err != nil {
			return "", err
		}
	}
	output := []byte(compensationLiquidityDomainV2)
	var err error
	for _, value := range []string{r.ReservationID, r.SourceOwnerID, r.SourceAccountID, r.Asset} {
		output, err = appendCompensationTextV2(output, value)
		if err != nil {
			return "", err
		}
	}
	output = appendCompensationU64V2(output, r.AmountMinor)
	return compensationSHA256V2(output), nil
}

type CompensationRequestV2 struct {
	CompensationID       string
	ActorOwnerID         string
	AmountMinor          uint64
	Authorization        CompensationAuthorizationV2
	LiquidityReservation LiquidityReservationV2
}

func (r CompensationRequestV2) CanonicalDigest() (string, error) {
	if err := validateCompensationTextV2(r.CompensationID); err != nil {
		return "", err
	}
	if err := validateCompensationTextV2(r.ActorOwnerID); err != nil {
		return "", err
	}
	authDigest, err := r.Authorization.CanonicalDigest()
	if err != nil {
		return "", err
	}
	reservationDigest, err := r.LiquidityReservation.CanonicalDigest()
	if err != nil {
		return "", err
	}
	output := []byte(compensationRequestDomainV2)
	output, err = appendCompensationTextV2(output, r.CompensationID)
	if err != nil {
		return "", err
	}
	output, err = appendCompensationTextV2(output, r.ActorOwnerID)
	if err != nil {
		return "", err
	}
	output = appendCompensationU64V2(output, r.AmountMinor)
	output, err = appendCompensationTextV2(output, authDigest)
	if err != nil {
		return "", err
	}
	output, err = appendCompensationTextV2(output, reservationDigest)
	if err != nil {
		return "", err
	}
	return compensationSHA256V2(output), nil
}

type CompensationStatusV2 uint8

const (
	CompensationReservedV2 CompensationStatusV2 = iota + 1
	CompensationDispatchAuthorizedV2
	CompensationIndeterminateV2
	CompensationAuthoritativelyAbsentV2
	CompensationObservedUnsettledV2
	CompensationSettledV2
	CompensationCancelledV2
)

type CompensationDispositionV2 uint8

const (
	CompensationCreatedV2 CompensationDispositionV2 = iota + 1
	CompensationAlreadyExistsV2
	CompensationAppliedV2
	CompensationAlreadyAppliedV2
	CompensationIsSettledV2
	CompensationAlreadySettledV2
	CompensationWasCancelledV2
)

type CompensationOutcomeKindV2 uint8

const (
	CompensationAuthoritativeAbsenceV2 CompensationOutcomeKindV2 = iota + 1
	CompensationObservedUnsettledV2
	CompensationFinalSettlementV2
)

type CompensationOutcomeV2 struct {
	Kind                       CompensationOutcomeKindV2
	CompensationID             string
	CompensationEffectID       string
	ForwardEffectID            string
	ForwardEffectDigest        string
	PlanDigest                 string
	AmountMinor                uint64
	AuthorizationDigest        string
	LiquidityReservationDigest string
	FundingOwnerID             string
	FundingAccountID           string
	BeneficiaryOwnerID         string
	BeneficiaryAccountID       string
	Asset                      string
	EvidenceReference          string
}

func (o CompensationOutcomeV2) CanonicalDigest() (string, error) {
	for _, value := range []string{
		o.CompensationID, o.CompensationEffectID, o.ForwardEffectID,
		o.FundingOwnerID, o.FundingAccountID, o.BeneficiaryOwnerID,
		o.BeneficiaryAccountID, o.Asset, o.EvidenceReference,
	} {
		if err := validateCompensationTextV2(value); err != nil {
			return "", err
		}
	}
	for _, value := range []string{
		o.ForwardEffectDigest, o.PlanDigest, o.AuthorizationDigest, o.LiquidityReservationDigest,
	} {
		if err := validateCompensationDigestV2(value); err != nil {
			return "", err
		}
	}
	if o.AmountMinor == 0 {
		return "", ErrCompensationInvalidOutcomeV2
	}
	output := []byte(compensationOutcomeDomainV2)
	output = append(output, byte(o.Kind))
	var err error
	for _, value := range []string{
		o.CompensationID, o.CompensationEffectID, o.ForwardEffectID,
		o.ForwardEffectDigest, o.PlanDigest, o.AuthorizationDigest,
		o.LiquidityReservationDigest, o.FundingOwnerID, o.FundingAccountID,
		o.BeneficiaryOwnerID, o.BeneficiaryAccountID, o.Asset, o.EvidenceReference,
	} {
		output, err = appendCompensationTextV2(output, value)
		if err != nil {
			return "", err
		}
	}
	output = appendCompensationU64V2(output, o.AmountMinor)
	return compensationSHA256V2(output), nil
}

type compensationRecordV2 struct {
	Request             CompensationRequestV2
	RequestDigest       string
	Status              CompensationStatusV2
	PossibleEffectCount uint8
	LastOutcomeDigest   string
	HasOutcomeDigest    bool
}

type CompensationBookV2 struct {
	mu                    sync.Mutex
	forward               ForwardEffectV2
	plan                  CompensationPlanV2
	planDigest            string
	initialLiquidityMinor uint64
	settledMinor          uint64
	encumberedMinor       uint64
	reservationIDs        map[string]struct{}
	capabilityScopes      map[string]string
	capabilityUsage       map[string]uint64
	records               map[string]*compensationRecordV2
}

func NewCompensationBookV2(
	forward ForwardEffectV2,
	plan CompensationPlanV2,
	initialLiquidityMinor uint64,
) (*CompensationBookV2, error) {
	if err := plan.Validate(forward); err != nil {
		return nil, err
	}
	planDigest, err := plan.CanonicalDigest(forward)
	if err != nil {
		return nil, err
	}
	return &CompensationBookV2{
		forward: forward, plan: plan, planDigest: planDigest,
		initialLiquidityMinor: initialLiquidityMinor,
		reservationIDs:        make(map[string]struct{}),
		capabilityScopes:      make(map[string]string),
		capabilityUsage:       make(map[string]uint64),
		records:               make(map[string]*compensationRecordV2),
	}, nil
}

func (b *CompensationBookV2) Forward() ForwardEffectV2 { return b.forward }
func (b *CompensationBookV2) Plan() CompensationPlanV2 { return b.plan }
func (b *CompensationBookV2) PlanDigest() string       { return b.planDigest }

func (b *CompensationBookV2) Reserve(
	request CompensationRequestV2,
	nowUnixMS uint64,
) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.forward.Finality != ForwardSettledV2 {
		return 0, ErrCompensationForwardNotSettledV2
	}
	requestDigest, err := request.CanonicalDigest()
	if err != nil {
		return 0, err
	}
	if existing, ok := b.records[request.CompensationID]; ok {
		if existing.RequestDigest == requestDigest {
			return CompensationAlreadyExistsV2, nil
		}
		return 0, ErrCompensationConflictV2
	}
	if err := request.Authorization.Validate(
		b.plan, b.planDigest, request.AmountMinor, request.ActorOwnerID, nowUnixMS,
	); err != nil {
		return 0, err
	}
	if err := request.LiquidityReservation.Validate(b.plan, request.AmountMinor); err != nil {
		return 0, err
	}
	reservationID := request.LiquidityReservation.ReservationID
	if _, used := b.reservationIDs[reservationID]; used {
		return 0, ErrCompensationLiquidityReservationConflictV2
	}
	capabilityID := request.Authorization.CapabilityID
	capabilityDigest, err := request.Authorization.CanonicalScopeDigest()
	if err != nil {
		return 0, err
	}
	if existingScope, ok := b.capabilityScopes[capabilityID]; ok && existingScope != capabilityDigest {
		return 0, ErrCompensationCapabilityIdentityConflictV2
	}
	capabilityProjected, err := checkedCompensationAddV2(b.capabilityUsage[capabilityID], request.AmountMinor)
	if err != nil {
		return 0, err
	}
	if capabilityProjected > request.Authorization.MaximumMinor {
		return 0, ErrCompensationAuthorizationLimitV2
	}
	used, err := checkedCompensationAddV2(b.settledMinor, b.encumberedMinor)
	if err != nil {
		return 0, err
	}
	projected, err := checkedCompensationAddV2(used, request.AmountMinor)
	if err != nil {
		return 0, err
	}
	if projected > b.plan.MaximumCompensationMinor {
		return 0, ErrCompensationReversibleCeilingV2
	}
	if projected > b.initialLiquidityMinor {
		return 0, ErrCompensationInsufficientLiquidityV2
	}
	projectedEncumbered, err := checkedCompensationAddV2(b.encumberedMinor, request.AmountMinor)
	if err != nil {
		return 0, err
	}
	b.reservationIDs[reservationID] = struct{}{}
	b.capabilityScopes[capabilityID] = capabilityDigest
	b.capabilityUsage[capabilityID] = capabilityProjected
	b.encumberedMinor = projectedEncumbered
	b.records[request.CompensationID] = &compensationRecordV2{
		Request: request, RequestDigest: requestDigest, Status: CompensationReservedV2,
	}
	return CompensationCreatedV2, nil
}

func (b *CompensationBookV2) MarkDispatchAuthorized(
	compensationID string,
	actorOwnerID string,
	currentAuthorization CompensationAuthorizationV2,
	nowUnixMS uint64,
) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	record, ok := b.records[compensationID]
	if !ok {
		return 0, ErrCompensationNotFoundV2
	}
	if actorOwnerID != record.Request.ActorOwnerID || actorOwnerID != currentAuthorization.GranteeOwnerID {
		return 0, ErrCompensationCallerNotAuthorizedV2
	}
	storedScope, err := record.Request.Authorization.CanonicalScopeDigest()
	if err != nil {
		return 0, err
	}
	currentScope, err := currentAuthorization.CanonicalScopeDigest()
	if err != nil {
		return 0, err
	}
	if currentScope != storedScope {
		return 0, ErrCompensationAuthorizationScopeV2
	}
	if err := currentAuthorization.Validate(b.plan, b.planDigest, record.Request.AmountMinor, actorOwnerID, nowUnixMS); err != nil {
		return 0, err
	}
	switch record.Status {
	case CompensationReservedV2:
		record.Status = CompensationDispatchAuthorizedV2
		return CompensationAppliedV2, nil
	case CompensationDispatchAuthorizedV2:
		return CompensationAlreadyAppliedV2, nil
	default:
		return 0, ErrCompensationInvalidTransitionV2
	}
}

func (b *CompensationBookV2) MarkIndeterminate(compensationID string, possibleEffectCount uint8) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if possibleEffectCount > 1 {
		return 0, ErrCompensationOutcomeContradictionV2
	}
	record, ok := b.records[compensationID]
	if !ok {
		return 0, ErrCompensationNotFoundV2
	}
	switch record.Status {
	case CompensationDispatchAuthorizedV2:
		record.Status = CompensationIndeterminateV2
		record.PossibleEffectCount = possibleEffectCount
		return CompensationAppliedV2, nil
	case CompensationIndeterminateV2:
		if record.PossibleEffectCount == possibleEffectCount {
			return CompensationAlreadyAppliedV2, nil
		}
		return 0, ErrCompensationOutcomeConflictV2
	default:
		return 0, ErrCompensationInvalidTransitionV2
	}
}

func (b *CompensationBookV2) ResolveAuthoritativeAbsence(outcome CompensationOutcomeV2) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if err := b.validateOutcomeLocked(outcome, CompensationAuthoritativeAbsenceV2); err != nil {
		return 0, err
	}
	digest, err := outcome.CanonicalDigest()
	if err != nil {
		return 0, err
	}
	record := b.records[outcome.CompensationID]
	if record.Status == CompensationAuthoritativelyAbsentV2 {
		if record.HasOutcomeDigest && record.LastOutcomeDigest == digest {
			return CompensationAlreadyAppliedV2, nil
		}
		return 0, ErrCompensationOutcomeConflictV2
	}
	if record.Status != CompensationIndeterminateV2 {
		return 0, ErrCompensationInvalidTransitionV2
	}
	if record.PossibleEffectCount != 0 {
		return 0, ErrCompensationOutcomeContradictionV2
	}
	record.Status = CompensationAuthoritativelyAbsentV2
	record.LastOutcomeDigest = digest
	record.HasOutcomeDigest = true
	return CompensationAppliedV2, nil
}

func (b *CompensationBookV2) RetrySameEffect(
	compensationID string,
	actorOwnerID string,
	currentAuthorization CompensationAuthorizationV2,
	nowUnixMS uint64,
) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	record, ok := b.records[compensationID]
	if !ok {
		return 0, ErrCompensationNotFoundV2
	}
	if record.Status != CompensationAuthoritativelyAbsentV2 {
		return 0, ErrCompensationInvalidTransitionV2
	}
	if actorOwnerID != record.Request.ActorOwnerID || actorOwnerID != currentAuthorization.GranteeOwnerID {
		return 0, ErrCompensationCallerNotAuthorizedV2
	}
	storedScope, err := record.Request.Authorization.CanonicalScopeDigest()
	if err != nil {
		return 0, err
	}
	currentScope, err := currentAuthorization.CanonicalScopeDigest()
	if err != nil {
		return 0, err
	}
	if currentScope != storedScope {
		return 0, ErrCompensationAuthorizationScopeV2
	}
	if err := currentAuthorization.Validate(b.plan, b.planDigest, record.Request.AmountMinor, actorOwnerID, nowUnixMS); err != nil {
		return 0, err
	}
	record.Status = CompensationReservedV2
	record.PossibleEffectCount = 0
	record.LastOutcomeDigest = ""
	record.HasOutcomeDigest = false
	return CompensationAppliedV2, nil
}

func (b *CompensationBookV2) ObserveUnsettled(outcome CompensationOutcomeV2) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if err := b.validateOutcomeLocked(outcome, CompensationObservedUnsettledV2); err != nil {
		return 0, err
	}
	digest, err := outcome.CanonicalDigest()
	if err != nil {
		return 0, err
	}
	record := b.records[outcome.CompensationID]
	if record.Status == CompensationObservedUnsettledV2 {
		if record.HasOutcomeDigest && record.LastOutcomeDigest == digest {
			return CompensationAlreadyAppliedV2, nil
		}
		return 0, ErrCompensationOutcomeConflictV2
	}
	if record.Status != CompensationIndeterminateV2 || record.PossibleEffectCount != 1 {
		return 0, ErrCompensationOutcomeContradictionV2
	}
	record.Status = CompensationObservedUnsettledV2
	record.LastOutcomeDigest = digest
	record.HasOutcomeDigest = true
	return CompensationAppliedV2, nil
}

func (b *CompensationBookV2) Settle(outcome CompensationOutcomeV2) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if err := b.validateOutcomeLocked(outcome, CompensationFinalSettlementV2); err != nil {
		return 0, err
	}
	digest, err := outcome.CanonicalDigest()
	if err != nil {
		return 0, err
	}
	record := b.records[outcome.CompensationID]
	if record.Status == CompensationSettledV2 {
		if record.HasOutcomeDigest && record.LastOutcomeDigest == digest {
			return CompensationAlreadySettledV2, nil
		}
		return 0, ErrCompensationOutcomeConflictV2
	}
	switch record.Status {
	case CompensationDispatchAuthorizedV2, CompensationObservedUnsettledV2:
	case CompensationIndeterminateV2:
		if record.PossibleEffectCount != 1 {
			return 0, ErrCompensationOutcomeContradictionV2
		}
	default:
		return 0, ErrCompensationInvalidTransitionV2
	}
	if b.encumberedMinor < record.Request.AmountMinor {
		return 0, ErrCompensationAmountOverflowV2
	}
	nextEncumbered := b.encumberedMinor - record.Request.AmountMinor
	nextSettled, err := checkedCompensationAddV2(b.settledMinor, record.Request.AmountMinor)
	if err != nil {
		return 0, err
	}
	record.Status = CompensationSettledV2
	record.PossibleEffectCount = 1
	record.LastOutcomeDigest = digest
	record.HasOutcomeDigest = true
	b.encumberedMinor = nextEncumbered
	b.settledMinor = nextSettled
	return CompensationIsSettledV2, nil
}

func (b *CompensationBookV2) CancelBeforeDispatch(compensationID, actorOwnerID string) (CompensationDispositionV2, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	record, ok := b.records[compensationID]
	if !ok {
		return 0, ErrCompensationNotFoundV2
	}
	if actorOwnerID != record.Request.ActorOwnerID && actorOwnerID != b.plan.FundingOwnerID {
		return 0, ErrCompensationCallerNotAuthorizedV2
	}
	switch record.Status {
	case CompensationReservedV2, CompensationAuthoritativelyAbsentV2:
		amount := record.Request.AmountMinor
		capabilityID := record.Request.Authorization.CapabilityID
		capabilityUsage, ok := b.capabilityUsage[capabilityID]
		if !ok || capabilityUsage < amount || b.encumberedMinor < amount {
			return 0, ErrCompensationAmountOverflowV2
		}
		b.capabilityUsage[capabilityID] = capabilityUsage - amount
		b.encumberedMinor -= amount
		record.Status = CompensationCancelledV2
		return CompensationWasCancelledV2, nil
	case CompensationCancelledV2:
		return CompensationWasCancelledV2, nil
	default:
		return 0, ErrCompensationInvalidTransitionV2
	}
}

func (b *CompensationBookV2) CompensationEffectID(compensationID string) (string, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if _, ok := b.records[compensationID]; !ok {
		return "", false
	}
	return "mx-compensation:" + compensationID, true
}

func (b *CompensationBookV2) Status(compensationID string) (CompensationStatusV2, bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	record, ok := b.records[compensationID]
	if !ok {
		return 0, false
	}
	return record.Status, true
}

func (b *CompensationBookV2) SettledMinor() (uint64, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	settled, _, err := b.totalsLocked()
	return settled, err
}

func (b *CompensationBookV2) EncumberedMinor() (uint64, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	_, encumbered, err := b.totalsLocked()
	return encumbered, err
}

func (b *CompensationBookV2) RemainingReversibleMinor() (uint64, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	settled, encumbered, err := b.totalsLocked()
	if err != nil {
		return 0, err
	}
	used, err := checkedCompensationAddV2(settled, encumbered)
	if err != nil {
		return 0, err
	}
	if used >= b.plan.MaximumCompensationMinor {
		return 0, nil
	}
	return b.plan.MaximumCompensationMinor - used, nil
}

func (b *CompensationBookV2) AvailableLiquidityMinor() (uint64, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	settled, encumbered, err := b.totalsLocked()
	if err != nil {
		return 0, err
	}
	used, err := checkedCompensationAddV2(settled, encumbered)
	if err != nil {
		return 0, err
	}
	if used >= b.initialLiquidityMinor {
		return 0, nil
	}
	return b.initialLiquidityMinor - used, nil
}

func (b *CompensationBookV2) validateOutcomeLocked(outcome CompensationOutcomeV2, expectedKind CompensationOutcomeKindV2) error {
	if _, err := outcome.CanonicalDigest(); err != nil {
		return err
	}
	record, ok := b.records[outcome.CompensationID]
	if !ok {
		return ErrCompensationNotFoundV2
	}
	expectedEffectID := "mx-compensation:" + outcome.CompensationID
	forwardDigest, err := b.forward.CanonicalDigest()
	if err != nil {
		return err
	}
	authDigest, err := record.Request.Authorization.CanonicalDigest()
	if err != nil {
		return err
	}
	reservationDigest, err := record.Request.LiquidityReservation.CanonicalDigest()
	if err != nil {
		return err
	}
	if outcome.Kind != expectedKind ||
		outcome.AuthorizationDigest != authDigest ||
		outcome.LiquidityReservationDigest != reservationDigest ||
		outcome.CompensationEffectID != expectedEffectID ||
		outcome.ForwardEffectID != b.forward.EffectID ||
		outcome.ForwardEffectDigest != forwardDigest ||
		outcome.PlanDigest != b.planDigest ||
		outcome.AmountMinor != record.Request.AmountMinor ||
		outcome.FundingOwnerID != b.plan.FundingOwnerID ||
		outcome.FundingAccountID != b.plan.FundingAccountID ||
		outcome.BeneficiaryOwnerID != b.plan.BeneficiaryOwnerID ||
		outcome.BeneficiaryAccountID != b.plan.BeneficiaryAccountID ||
		outcome.Asset != b.plan.Asset {
		return ErrCompensationInvalidOutcomeV2
	}
	return nil
}

func (b *CompensationBookV2) totalsLocked() (uint64, uint64, error) {
	return b.settledMinor, b.encumberedMinor, nil
}

var (
	ErrCompensationInvalidDigestV2                = errors.New("compensation V2 digest is invalid")
	ErrCompensationInvalidTextV2                  = errors.New("compensation V2 text is empty or oversized")
	ErrCompensationInvalidForwardEffectV2         = errors.New("forward effect V2 is invalid")
	ErrCompensationForwardNotSettledV2            = errors.New("forward effect is not settled")
	ErrCompensationInvalidPlanV2                  = errors.New("compensation plan V2 is invalid")
	ErrCompensationAuthorizationRevokedV2         = errors.New("compensation authorization is revoked")
	ErrCompensationAuthorizationExpiredV2         = errors.New("compensation authorization is expired")
	ErrCompensationAuthorizationScopeV2           = errors.New("compensation authorization scope mismatch")
	ErrCompensationCapabilityIdentityConflictV2   = errors.New("capability ID was reused with changed scope")
	ErrCompensationCallerNotAuthorizedV2          = errors.New("acting owner is not authorized")
	ErrCompensationAuthorizationLimitV2           = errors.New("compensation authorization cumulative limit exceeded")
	ErrCompensationLiquidityReservationMismatchV2 = errors.New("liquidity reservation does not match funding scope")
	ErrCompensationLiquidityReservationConflictV2 = errors.New("liquidity reservation ID is already used")
	ErrCompensationInsufficientLiquidityV2        = errors.New("compensation source liquidity is insufficient")
	ErrCompensationReversibleCeilingV2            = errors.New("compensation exceeds the reversible ceiling")
	ErrCompensationConflictV2                     = errors.New("compensation identity was reused with changed terms")
	ErrCompensationNotFoundV2                     = errors.New("compensation was not found")
	ErrCompensationInvalidTransitionV2            = errors.New("invalid compensation transition")
	ErrCompensationInvalidOutcomeV2               = errors.New("compensation outcome is not bound to the request")
	ErrCompensationOutcomeConflictV2              = errors.New("compensation outcome conflicts with prior evidence")
	ErrCompensationOutcomeContradictionV2         = errors.New("compensation outcome contradicts possible effect count")
	ErrCompensationAmountOverflowV2               = errors.New("compensation amount arithmetic overflow")
)

func validateCompensationTextV2(value string) error {
	if value == "" || len(value) > maxCompensationFieldBytesV2 || !utf8.ValidString(value) {
		return ErrCompensationInvalidTextV2
	}
	return nil
}

func validateCompensationDigestV2(value string) error {
	if len(value) != 71 || value[:7] != "sha256:" {
		return ErrCompensationInvalidDigestV2
	}
	for i := 7; i < len(value); i++ {
		c := value[i]
		if !((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) {
			return ErrCompensationInvalidDigestV2
		}
	}
	return nil
}

func appendCompensationTextV2(output []byte, value string) ([]byte, error) {
	if err := validateCompensationTextV2(value); err != nil {
		return nil, err
	}
	var size [4]byte
	binary.BigEndian.PutUint32(size[:], uint32(len(value)))
	output = append(output, size[:]...)
	output = append(output, value...)
	return output, nil
}

func appendCompensationU64V2(output []byte, value uint64) []byte {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	return append(output, encoded[:]...)
}

func checkedCompensationAddV2(left, right uint64) (uint64, error) {
	if right > ^uint64(0)-left {
		return 0, ErrCompensationAmountOverflowV2
	}
	return left + right, nil
}

func compensationSHA256V2(data []byte) string {
	digest := sha256.Sum256(data)
	return "sha256:" + hex.EncodeToString(digest[:])
}

// CanonicalCompensationDomainListV2 returns the exact profile labels exposed
// for independent corpus and protocol documentation checks.
func CanonicalCompensationDomainListV2() []string {
	values := []string{
		compensationForwardDomainV2, compensationPlanDomainV2,
		compensationAuthDomainV2, compensationLiquidityDomainV2,
		compensationRequestDomainV2, compensationOutcomeDomainV2,
	}
	sort.Strings(values)
	return values
}
