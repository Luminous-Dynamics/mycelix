package sagaverifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"sort"
)

const delegatedExposureDomain = "MXSAP-DELEGATED-EXPOSURE-V1\x00"

type ParentExposureScopeV1 struct {
	OwnerID  string
	PoolID   string
	Currency string
}

func (s ParentExposureScopeV1) validate() error {
	for _, value := range []string{s.OwnerID, s.PoolID, s.Currency} {
		if err := validateExposureText(value); err != nil {
			return err
		}
	}
	return nil
}

type DelegatedExposurePolicyV1 struct {
	Scope                   ParentExposureScopeV1
	Sequence                uint64
	MaximumTotalMinor       uint64
	MaximumChildBudgetMinor uint64
	PolicyDigest            string
}

func (p DelegatedExposurePolicyV1) validate() error {
	if err := p.Scope.validate(); err != nil {
		return err
	}
	if p.Sequence == 0 || p.MaximumTotalMinor == 0 ||
		p.MaximumChildBudgetMinor == 0 || p.MaximumChildBudgetMinor > p.MaximumTotalMinor {
		return ErrExposureInvalidPolicy
	}
	return validateExposureDigest(p.PolicyDigest)
}

type DelegatedBudgetStatus uint8

const (
	DelegatedBudgetActive DelegatedBudgetStatus = iota + 1
	DelegatedBudgetRevoking
	DelegatedBudgetClosed
)

type DelegatedBudgetDisposition uint8

const (
	DelegatedBudgetAllocated DelegatedBudgetDisposition = iota + 1
	DelegatedBudgetAlreadyAllocated
)

type delegatedBudgetGrant struct {
	GrantID      string
	ChildScope   ExposureScopeV1
	BudgetMinor  uint64
	Sequence     uint64
	GrantDigest  string
	Status       DelegatedBudgetStatus
	ChildBook    *ExposureBookV1
}

func (g delegatedBudgetGrant) sameTerms(
	childScope ExposureScopeV1,
	budgetMinor uint64,
	sequence uint64,
	grantDigest string,
) bool {
	return g.ChildScope == childScope &&
		g.BudgetMinor == budgetMinor &&
		g.Sequence == sequence &&
		g.GrantDigest == grantDigest
}

func (g delegatedBudgetGrant) countsAgainstParent() bool {
	return g.Status == DelegatedBudgetActive || g.Status == DelegatedBudgetRevoking
}

type ParentDelegatedExposureBookV1 struct {
	policy DelegatedExposurePolicyV1
	grants map[string]delegatedBudgetGrant
}

func NewParentDelegatedExposureBookV1(policy DelegatedExposurePolicyV1) (*ParentDelegatedExposureBookV1, error) {
	if err := policy.validate(); err != nil {
		return nil, err
	}
	return &ParentDelegatedExposureBookV1{
		policy: policy,
		grants: make(map[string]delegatedBudgetGrant),
	}, nil
}

// AllocateChildBudget encumbers the whole sublimit at the parent until the
// child grant is closed. Child books can only reserve within that delegation.
// ReplacePolicy preserves every existing delegation. If active/revoking
// encumbrance exceeds the lower cap, new child reservations pause until grants
// are safely closed; resolution of existing effects remains available.
func (b *ParentDelegatedExposureBookV1) ReplacePolicy(policy DelegatedExposurePolicyV1) error {
	if err := policy.validate(); err != nil { return err }
	if policy.Scope != b.policy.Scope || policy.Sequence <= b.policy.Sequence {
		return ErrExposureInvalidPolicy
	}
	b.policy = policy
	return nil
}

func (b *ParentDelegatedExposureBookV1) AllocateChildBudget(
	grantID string,
	childScope ExposureScopeV1,
	budgetMinor uint64,
	grantSequence uint64,
	grantDigest string,
) (DelegatedBudgetDisposition, error) {
	if err := validateExposureText(grantID); err != nil {
		return 0, err
	}
	if err := validateExposureDigest(grantDigest); err != nil {
		return 0, err
	}
	if childScope.PoolID != b.policy.Scope.PoolID ||
		childScope.Currency != b.policy.Scope.Currency {
		return 0, ErrExposureScopeMismatch
	}
	childPolicy := ExposurePolicyV1{
		Scope:                 childScope,
		Sequence:              grantSequence,
		MaximumTotalMinor:     budgetMinor,
		MaximumPerEffectMinor: budgetMinor,
		PolicyDigest:          grantDigest,
	}
	if err := childPolicy.Validate(); err != nil {
		return 0, err
	}
	if budgetMinor > b.policy.MaximumChildBudgetMinor {
		return 0, ErrExposureLimitExceeded
	}
	if existing, ok := b.grants[grantID]; ok {
		if existing.sameTerms(childScope, budgetMinor, grantSequence, grantDigest) {
			return DelegatedBudgetAlreadyAllocated, nil
		}
		return 0, ErrExposureReservationConflict
	}
	var latestSequence uint64
	for _, existing := range b.grants {
		if existing.Sequence > latestSequence {
			latestSequence = existing.Sequence
		}
	}
	if grantSequence <= latestSequence {
		return 0, ErrExposureInvalidPolicy
	}
	allocated, err := b.AllocatedBudgetMinor()
	if err != nil {
		return 0, err
	}
	if budgetMinor > ^uint64(0)-allocated {
		return 0, ErrExposureAmountOverflow
	}
	if allocated+budgetMinor > b.policy.MaximumTotalMinor {
		return 0, ErrExposureLimitExceeded
	}
	childBook, err := NewExposureBookV1(childPolicy)
	if err != nil {
		return 0, err
	}
	b.grants[grantID] = delegatedBudgetGrant{
		GrantID: grantID,
		ChildScope: childScope,
		BudgetMinor: budgetMinor,
		Sequence: grantSequence,
		GrantDigest: grantDigest,
		Status: DelegatedBudgetActive,
		ChildBook: childBook,
	}
	return DelegatedBudgetAllocated, nil
}

// Reserve admits new risk only under an active grant. Exact retries of an
// existing effect continue to resolve during revocation.
func (b *ParentDelegatedExposureBookV1) Reserve(
	grantID string,
	intent ExposureIntentV1,
) (ExposureDisposition, error) {
	grant, ok := b.grants[grantID]
	if !ok {
		return 0, ErrExposureNotFound
	}
	if intent.Scope != grant.ChildScope {
		return 0, ErrExposureScopeMismatch
	}
	_, exists := grant.ChildBook.Status(intent.EffectID)
	if exists {
		return grant.ChildBook.Reserve(intent)
	}
	if grant.Status != DelegatedBudgetActive {
		return 0, ErrExposureInvalidTransition
	}
	allocated, err := b.AllocatedBudgetMinor()
	if err != nil { return 0, err }
	if grant.BudgetMinor > b.policy.MaximumChildBudgetMinor ||
		allocated > b.policy.MaximumTotalMinor {
		return 0, ErrExposureLimitExceeded
	}
	return grant.ChildBook.Reserve(intent)
}

func (b *ParentDelegatedExposureBookV1) MarkDispatchAuthorized(grantID, effectID string) error {
	grant, err := b.childGrant(grantID)
	if err != nil { return err }
	return grant.ChildBook.MarkDispatchAuthorized(effectID)
}

func (b *ParentDelegatedExposureBookV1) MarkIndeterminate(grantID, effectID string) error {
	grant, err := b.childGrant(grantID)
	if err != nil { return err }
	return grant.ChildBook.MarkIndeterminate(effectID)
}

func (b *ParentDelegatedExposureBookV1) MarkCommittedUnsettled(grantID, effectID string) error {
	grant, err := b.childGrant(grantID)
	if err != nil { return err }
	return grant.ChildBook.MarkCommittedUnsettled(effectID)
}

func (b *ParentDelegatedExposureBookV1) MarkSettled(
	grantID, effectID string,
	evidence DestinationResolutionV1,
) error {
	grant, err := b.childGrant(grantID)
	if err != nil { return err }
	return grant.ChildBook.MarkSettled(effectID, evidence)
}

func (b *ParentDelegatedExposureBookV1) ReleaseAuthoritativeAbsence(
	grantID, effectID string,
	evidence DestinationResolutionV1,
) error {
	grant, err := b.childGrant(grantID)
	if err != nil { return err }
	return grant.ChildBook.ReleaseAuthoritativeAbsence(effectID, evidence)
}

// BeginRevocation blocks new reservations but preserves all current child
// reservations and keeps their entire sublimit encumbered at the parent.
func (b *ParentDelegatedExposureBookV1) BeginRevocation(grantID string) error {
	grant, ok := b.grants[grantID]
	if !ok { return ErrExposureNotFound }
	switch grant.Status {
	case DelegatedBudgetActive:
		grant.Status = DelegatedBudgetRevoking
		b.grants[grantID] = grant
		return nil
	case DelegatedBudgetRevoking:
		return nil
	default:
		return ErrExposureInvalidTransition
	}
}

// CloseGrant releases the parent escrow only after every child exposure has
// been resolved. Production must replace this in-memory observation with an
// authenticated child close report and an owner-native parent action.
func (b *ParentDelegatedExposureBookV1) CloseGrant(grantID string) error {
	grant, ok := b.grants[grantID]
	if !ok { return ErrExposureNotFound }
	if grant.Status == DelegatedBudgetClosed { return nil }
	if grant.Status != DelegatedBudgetRevoking {
		return ErrExposureInvalidTransition
	}
	total, err := grant.ChildBook.TotalExposureMinor()
	if err != nil { return err }
	if total != 0 { return ErrExposureInvalidTransition }
	grant.Status = DelegatedBudgetClosed
	b.grants[grantID] = grant
	return nil
}

func (b *ParentDelegatedExposureBookV1) GrantStatus(grantID string) (DelegatedBudgetStatus, bool) {
	grant, ok := b.grants[grantID]
	return grant.Status, ok
}

func (b *ParentDelegatedExposureBookV1) ChildExposureMinor(grantID string) (uint64, error) {
	grant, ok := b.grants[grantID]
	if !ok { return 0, ErrExposureNotFound }
	return grant.ChildBook.TotalExposureMinor()
}

func (b *ParentDelegatedExposureBookV1) AllocatedBudgetMinor() (uint64, error) {
	var total uint64
	for _, grant := range b.grants {
		if !grant.countsAgainstParent() { continue }
		if grant.BudgetMinor > ^uint64(0)-total {
			return 0, ErrExposureAmountOverflow
		}
		total += grant.BudgetMinor
	}
	return total, nil
}

func (b *ParentDelegatedExposureBookV1) AvailableParentBudgetMinor() (uint64, error) {
	allocated, err := b.AllocatedBudgetMinor()
	if err != nil { return 0, err }
	if allocated >= b.policy.MaximumTotalMinor { return 0, nil }
	return b.policy.MaximumTotalMinor - allocated, nil
}

func (b *ParentDelegatedExposureBookV1) CanonicalSnapshotDigest() (string, error) {
	var output bytes.Buffer
	output.WriteString(delegatedExposureDomain)
	for _, value := range []string{
		b.policy.Scope.OwnerID, b.policy.Scope.PoolID, b.policy.Scope.Currency,
	} {
		if err := appendExposureText(&output, value); err != nil { return "", err }
	}
	appendExposureU64(&output, b.policy.Sequence)
	appendExposureU64(&output, b.policy.MaximumTotalMinor)
	appendExposureU64(&output, b.policy.MaximumChildBudgetMinor)
	if err := appendExposureText(&output, b.policy.PolicyDigest); err != nil { return "", err }
	keys := make([]string, 0, len(b.grants))
	for key := range b.grants { keys = append(keys, key) }
	sort.Strings(keys)
	if uint64(len(keys)) > uint64(^uint32(0)) { return "", ErrExposureInvalidAmount }
	var count [4]byte
	binary.BigEndian.PutUint32(count[:], uint32(len(keys)))
	output.Write(count[:])
	for _, key := range keys {
		grant := b.grants[key]
		for _, value := range []string{
			grant.GrantID, grant.ChildScope.OwnerID, grant.ChildScope.PoolID,
			grant.ChildScope.DestinationID, grant.ChildScope.Currency, grant.GrantDigest,
		} {
			if err := appendExposureText(&output, value); err != nil { return "", err }
		}
		appendExposureU64(&output, grant.BudgetMinor)
		appendExposureU64(&output, grant.Sequence)
		output.WriteByte(byte(grant.Status))
		childDigest, err := grant.ChildBook.CanonicalSnapshotDigest()
		if err != nil { return "", err }
		if err := appendExposureText(&output, childDigest); err != nil { return "", err }
	}
	digest := sha256.Sum256(output.Bytes())
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func (b *ParentDelegatedExposureBookV1) childGrant(grantID string) (delegatedBudgetGrant, error) {
	grant, ok := b.grants[grantID]
	if !ok { return delegatedBudgetGrant{}, ErrExposureNotFound }
	return grant, nil
}

