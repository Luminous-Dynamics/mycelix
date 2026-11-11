package sagaverifier

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"sort"
)

const (
	grantTermsDomain = "MXSAP-GRANT-TERMS-V1\x00"
	handoffParentDomain = "MXSAP-HANDOFF-PARENT-V1\x00"
	handoffChildDomain = "MXSAP-HANDOFF-CHILD-V1\x00"
)

type HandoffDisposition uint8

const (
	HandoffCommitted HandoffDisposition = iota + 1
	HandoffAlreadyCommitted
	HandoffObserved
	HandoffAlreadyObserved
	HandoffReleased
	HandoffAlreadyReleased
)

type ParentGrantStatus uint8

const (
	ParentGrantIssued ParentGrantStatus = iota + 1
	ParentGrantRevocationRequested
	ParentGrantReleased
)

type DeliveryKind uint8

const (
	DeliveryGrant DeliveryKind = iota + 1
	DeliveryRevocation
)

type GrantTermsV1 struct {
	GrantID             string
	ParentScope         ParentExposureScopeV1
	ChildScope          ExposureScopeV1
	BudgetMinor         uint64
	GrantSequence       uint64
	ParentPolicySequence uint64
	ParentPolicyDigest  string
}

func (t GrantTermsV1) Validate() error {
	for _, value := range []string{
		t.GrantID,
		t.ParentScope.OwnerID, t.ParentScope.PoolID, t.ParentScope.Currency,
		t.ChildScope.OwnerID, t.ChildScope.PoolID, t.ChildScope.DestinationID, t.ChildScope.Currency,
	} {
		if err := validateExposureText(value); err != nil {
			return err
		}
	}
	if err := validateExposureDigest(t.ParentPolicyDigest); err != nil {
		return err
	}
	if t.BudgetMinor == 0 || t.GrantSequence == 0 || t.ParentPolicySequence == 0 ||
		t.ParentScope.PoolID != t.ChildScope.PoolID ||
		t.ParentScope.Currency != t.ChildScope.Currency {
		return ErrHandoffInvalidGrant
	}
	return nil
}

func (t GrantTermsV1) CanonicalDigest() (string, error) {
	if err := t.Validate(); err != nil {
		return "", err
	}
	var output bytes.Buffer
	output.WriteString(grantTermsDomain)
	for _, value := range []string{
		t.GrantID,
		t.ParentScope.OwnerID, t.ParentScope.PoolID, t.ParentScope.Currency,
		t.ChildScope.OwnerID, t.ChildScope.PoolID, t.ChildScope.DestinationID, t.ChildScope.Currency,
		t.ParentPolicyDigest,
	} {
		if err := appendExposureText(&output, value); err != nil {
			return "", err
		}
	}
	appendExposureU64(&output, t.BudgetMinor)
	appendExposureU64(&output, t.GrantSequence)
	appendExposureU64(&output, t.ParentPolicySequence)
	return handoffSHA256(output.Bytes()), nil
}

type PendingDeliveryV1 struct {
	Kind             DeliveryKind
	StableDeliveryID string
	GrantID          string
	ParentActionHash string
}

type GrantAcceptanceReceiptV1 struct {
	GrantID                string
	GrantDigest             string
	ParentActionHash        string
	ChildOwnerID            string
	ChildAcceptActionHash   string
}

type GrantRevocationReceiptV1 struct {
	GrantID                  string
	GrantDigest               string
	ParentRevokeActionHash    string
	ChildOwnerID              string
	ChildObservedActionHash   string
}

type GrantClosureReceiptV1 struct {
	GrantID                         string
	GrantDigest                      string
	ParentActionHash                 string
	ParentRevokeActionHash           string
	ChildOwnerID                     string
	ChildAcceptActionHash            *string
	ChildObservedRevokeActionHash    string
	ChildSourceChainFrontier         string
	ChildSnapshotDigest              string
	ChildCloseActionHash             string
	ZeroUnresolved                   bool
}

var (
	ErrHandoffInvalidText       = errors.New("handoff text is empty or oversized")
	ErrHandoffInvalidDigest     = errors.New("handoff digest is not canonical")
	ErrHandoffInvalidGrant      = errors.New("handoff grant terms or authority do not match")
	ErrHandoffGrantConflict     = errors.New("grant identity reused with changed terms")
	ErrHandoffGrantNotFound     = errors.New("handoff grant not found")
	ErrHandoffAcceptanceConflict = errors.New("grant acceptance receipt conflicts")
	ErrHandoffRevocationConflict = errors.New("grant revocation receipt conflicts")
	ErrHandoffClosureConflict   = errors.New("grant closure receipt conflicts")
	ErrHandoffParentLimit       = errors.New("delegated parent exposure limit exceeded")
	ErrHandoffInvalidTransition = errors.New("invalid grant handoff transition")
	ErrHandoffUnresolved        = errors.New("child still has unresolved exposure")
)

type parentGrantRecordV1 struct {
	Terms            GrantTermsV1
	GrantDigest      string
	ParentActionHash string
	Status           ParentGrantStatus
	Acceptance       *GrantAcceptanceReceiptV1
	RevokeActionHash *string
	Revocation       *GrantRevocationReceiptV1
	Closure          *GrantClosureReceiptV1
}

type ParentHandoffBookV1 struct {
	policy  DelegatedExposurePolicyV1
	grants  map[string]parentGrantRecordV1
}

func NewParentHandoffBookV1(policy DelegatedExposurePolicyV1) (*ParentHandoffBookV1, error) {
	if err := policy.validate(); err != nil {
		return nil, err
	}
	return &ParentHandoffBookV1{policy: policy, grants: make(map[string]parentGrantRecordV1)}, nil
}

// Clone models a process restart from a fully persisted parent-chain snapshot.
// Pointer fields are copied so later mutations do not alias the old process.
func (b *ParentHandoffBookV1) Clone() *ParentHandoffBookV1 {
	clone := &ParentHandoffBookV1{policy: b.policy, grants: make(map[string]parentGrantRecordV1, len(b.grants))}
	for key, record := range b.grants {
		if record.Acceptance != nil {
			value := *record.Acceptance
			record.Acceptance = &value
		}
		if record.RevokeActionHash != nil {
			value := *record.RevokeActionHash
			record.RevokeActionHash = &value
		}
		if record.Revocation != nil {
			value := *record.Revocation
			record.Revocation = &value
		}
		if record.Closure != nil {
			value := *record.Closure
			if value.ChildAcceptActionHash != nil {
				hash := *value.ChildAcceptActionHash
				value.ChildAcceptActionHash = &hash
			}
			record.Closure = &value
		}
		clone.grants[key] = record
	}
	return clone
}

func (b *ParentHandoffBookV1) Policy() DelegatedExposurePolicyV1 { return b.policy }

func (b *ParentHandoffBookV1) IssueGrant(
	terms GrantTermsV1,
	parentActionHash string,
) (HandoffDisposition, error) {
	if err := terms.Validate(); err != nil {
		return 0, err
	}
	if err := validateExposureDigest(parentActionHash); err != nil {
		return 0, err
	}
	grantDigest, err := terms.CanonicalDigest()
	if err != nil {
		return 0, err
	}
	if existing, ok := b.grants[terms.GrantID]; ok {
		if existing.Terms == terms && existing.GrantDigest == grantDigest {
			// Recovered retries return the existing committed grant and action
			// identity; they never create a second parent-chain grant.
			return HandoffAlreadyCommitted, nil
		}
		return 0, ErrHandoffGrantConflict
	}
	if terms.ParentScope != b.policy.Scope ||
		terms.ParentPolicySequence != b.policy.Sequence ||
		terms.ParentPolicyDigest != b.policy.PolicyDigest {
		return 0, ErrHandoffInvalidGrant
	}
	if terms.BudgetMinor > b.policy.MaximumChildBudgetMinor {
		return 0, ErrHandoffParentLimit
	}
	var latestSequence uint64
	for _, record := range b.grants {
		if record.Terms.GrantSequence > latestSequence {
			latestSequence = record.Terms.GrantSequence
		}
	}
	if terms.GrantSequence <= latestSequence {
		return 0, ErrHandoffInvalidGrant
	}
	allocated, err := b.AllocatedBudgetMinor()
	if err != nil {
		return 0, err
	}
	if terms.BudgetMinor > ^uint64(0)-allocated ||
		allocated+terms.BudgetMinor > b.policy.MaximumTotalMinor {
		return 0, ErrHandoffParentLimit
	}
	b.grants[terms.GrantID] = parentGrantRecordV1{
		Terms: terms, GrantDigest: grantDigest, ParentActionHash: parentActionHash,
		Status: ParentGrantIssued,
	}
	return HandoffCommitted, nil
}

func (b *ParentHandoffBookV1) PendingDeliveries() []PendingDeliveryV1 {
	output := make([]PendingDeliveryV1, 0)
	for _, record := range b.grants {
		if record.Status == ParentGrantReleased {
			continue
		}
		if record.Acceptance == nil && record.Closure == nil {
			output = append(output, PendingDeliveryV1{
				Kind: DeliveryGrant, StableDeliveryID: "grant:" + record.Terms.GrantID,
				GrantID: record.Terms.GrantID, ParentActionHash: record.ParentActionHash,
			})
		}
		if record.RevokeActionHash != nil && record.Revocation == nil && record.Closure == nil {
			output = append(output, PendingDeliveryV1{
				Kind: DeliveryRevocation, StableDeliveryID: "revoke:" + record.Terms.GrantID,
				GrantID: record.Terms.GrantID, ParentActionHash: *record.RevokeActionHash,
			})
		}
	}
	sort.Slice(output, func(i, j int) bool {
		if output[i].GrantID != output[j].GrantID { return output[i].GrantID < output[j].GrantID }
		return output[i].Kind < output[j].Kind
	})
	return output
}

func (b *ParentHandoffBookV1) ObserveAcceptance(receipt GrantAcceptanceReceiptV1) (HandoffDisposition, error) {
	if err := validateAcceptanceReceipt(receipt); err != nil {
		return 0, err
	}
	record, ok := b.grants[receipt.GrantID]
	if !ok { return 0, ErrHandoffGrantNotFound }
	if receipt.GrantDigest != record.GrantDigest ||
		receipt.ParentActionHash != record.ParentActionHash ||
		receipt.ChildOwnerID != record.Terms.ChildScope.OwnerID {
		return 0, ErrHandoffInvalidGrant
	}
	if record.Acceptance != nil {
		if *record.Acceptance == receipt { return HandoffAlreadyObserved, nil }
		return 0, ErrHandoffAcceptanceConflict
	}
	if record.Status == ParentGrantReleased {
		if record.Closure != nil && record.Closure.ChildAcceptActionHash != nil &&
			*record.Closure.ChildAcceptActionHash == receipt.ChildAcceptActionHash {
			return HandoffAlreadyObserved, nil
		}
		return 0, ErrHandoffAcceptanceConflict
	}
	record.Acceptance = &receipt
	b.grants[receipt.GrantID] = record
	return HandoffObserved, nil
}

func (b *ParentHandoffBookV1) RequestRevocation(grantID, revokeActionHash string) (HandoffDisposition, error) {
	if err := validateExposureDigest(revokeActionHash); err != nil { return 0, err }
	record, ok := b.grants[grantID]
	if !ok { return 0, ErrHandoffGrantNotFound }
	switch record.Status {
	case ParentGrantIssued:
		record.Status = ParentGrantRevocationRequested
		record.RevokeActionHash = &revokeActionHash
		b.grants[grantID] = record
		return HandoffCommitted, nil
	case ParentGrantRevocationRequested, ParentGrantReleased:
		if record.RevokeActionHash != nil && *record.RevokeActionHash == revokeActionHash {
			return HandoffAlreadyCommitted, nil
		}
		return 0, ErrHandoffRevocationConflict
	default:
		return 0, ErrHandoffInvalidTransition
	}
}

func (b *ParentHandoffBookV1) ObserveRevocation(receipt GrantRevocationReceiptV1) (HandoffDisposition, error) {
	if err := validateRevocationReceipt(receipt); err != nil { return 0, err }
	record, ok := b.grants[receipt.GrantID]
	if !ok { return 0, ErrHandoffGrantNotFound }
	if record.GrantDigest != receipt.GrantDigest ||
		record.RevokeActionHash == nil ||
		*record.RevokeActionHash != receipt.ParentRevokeActionHash ||
		record.Terms.ChildScope.OwnerID != receipt.ChildOwnerID {
		return 0, ErrHandoffInvalidGrant
	}
	if record.Revocation != nil {
		if *record.Revocation == receipt { return HandoffAlreadyObserved, nil }
		return 0, ErrHandoffRevocationConflict
	}
	if record.Status == ParentGrantReleased {
		if record.Closure != nil &&
			record.Closure.ChildObservedRevokeActionHash == receipt.ChildObservedActionHash {
			return HandoffAlreadyObserved, nil
		}
		return 0, ErrHandoffRevocationConflict
	}
	if record.Status != ParentGrantRevocationRequested {
		return 0, ErrHandoffInvalidTransition
	}
	record.Revocation = &receipt
	b.grants[receipt.GrantID] = record
	return HandoffObserved, nil
}

func (b *ParentHandoffBookV1) ReleaseGrant(receipt GrantClosureReceiptV1) (HandoffDisposition, error) {
	if err := validateClosureReceipt(receipt); err != nil { return 0, err }
	record, ok := b.grants[receipt.GrantID]
	if !ok { return 0, ErrHandoffGrantNotFound }
	if record.Status == ParentGrantReleased {
		if record.Closure != nil && sameHandoffClosure(*record.Closure, receipt) {
			return HandoffAlreadyReleased, nil
		}
		return 0, ErrHandoffClosureConflict
	}
	if record.Status != ParentGrantRevocationRequested ||
		!receipt.ZeroUnresolved ||
		record.GrantDigest != receipt.GrantDigest ||
		record.ParentActionHash != receipt.ParentActionHash ||
		record.RevokeActionHash == nil ||
		*record.RevokeActionHash != receipt.ParentRevokeActionHash ||
		record.Terms.ChildScope.OwnerID != receipt.ChildOwnerID {
		return 0, ErrHandoffInvalidGrant
	}
	if record.Acceptance != nil {
		if receipt.ChildAcceptActionHash == nil ||
			*receipt.ChildAcceptActionHash != record.Acceptance.ChildAcceptActionHash {
			return 0, ErrHandoffAcceptanceConflict
		}
	}
	if record.Revocation != nil &&
		record.Revocation.ChildObservedActionHash != receipt.ChildObservedRevokeActionHash {
		return 0, ErrHandoffRevocationConflict
	}
	record.Status = ParentGrantReleased
	record.Closure = &receipt
	b.grants[receipt.GrantID] = record
	return HandoffReleased, nil
}

func (b *ParentHandoffBookV1) GrantStatus(grantID string) (ParentGrantStatus, bool) {
	record, ok := b.grants[grantID]
	return record.Status, ok
}

func (b *ParentHandoffBookV1) AllocatedBudgetMinor() (uint64, error) {
	var total uint64
	for _, record := range b.grants {
		if record.Status == ParentGrantReleased { continue }
		if record.Terms.BudgetMinor > ^uint64(0)-total { return 0, ErrHandoffParentLimit }
		total += record.Terms.BudgetMinor
	}
	return total, nil
}

func (b *ParentHandoffBookV1) AvailableParentBudgetMinor() (uint64, error) {
	allocated, err := b.AllocatedBudgetMinor()
	if err != nil { return 0, err }
	if allocated >= b.policy.MaximumTotalMinor { return 0, nil }
	return b.policy.MaximumTotalMinor - allocated, nil
}

func (b *ParentHandoffBookV1) CanonicalSnapshotDigest() (string, error) {
	var output bytes.Buffer
	output.WriteString(handoffParentDomain)
	for _, value := range []string{b.policy.Scope.OwnerID, b.policy.Scope.PoolID, b.policy.Scope.Currency} {
		if err := appendExposureText(&output, value); err != nil { return "", err }
	}
	appendExposureU64(&output, b.policy.Sequence)
	appendExposureU64(&output, b.policy.MaximumTotalMinor)
	appendExposureU64(&output, b.policy.MaximumChildBudgetMinor)
	if err := appendExposureText(&output, b.policy.PolicyDigest); err != nil { return "", err }
	keys := make([]string, 0, len(b.grants))
	for key := range b.grants { keys = append(keys, key) }
	sort.Strings(keys)
	if uint64(len(keys)) > uint64(^uint32(0)) { return "", ErrHandoffInvalidGrant }
	var count [4]byte
	binary.BigEndian.PutUint32(count[:], uint32(len(keys)))
	output.Write(count[:])
	for _, key := range keys {
		record := b.grants[key]
		for _, value := range []string{record.Terms.GrantID, record.GrantDigest, record.ParentActionHash} {
			if err := appendExposureText(&output, value); err != nil { return "", err }
		}
		output.WriteByte(byte(record.Status))
		appendExposureU64(&output, record.Terms.GrantSequence)
		if record.Acceptance == nil {
			output.WriteByte(0)
		} else {
			output.WriteByte(1)
			if err := appendExposureText(&output, record.Acceptance.ChildAcceptActionHash); err != nil { return "", err }
		}
		if record.RevokeActionHash == nil {
			output.WriteByte(0)
		} else {
			output.WriteByte(1)
			if err := appendExposureText(&output, *record.RevokeActionHash); err != nil { return "", err }
		}
		if record.Revocation == nil {
			output.WriteByte(0)
		} else {
			output.WriteByte(1)
			if err := appendExposureText(&output, record.Revocation.ChildObservedActionHash); err != nil { return "", err }
		}
		if record.Closure == nil {
			output.WriteByte(0)
		} else {
			output.WriteByte(1)
			if record.Closure.ChildAcceptActionHash == nil { output.WriteByte(0) } else {
				output.WriteByte(1)
				if err := appendExposureText(&output, *record.Closure.ChildAcceptActionHash); err != nil { return "", err }
			}
			if err := appendExposureText(&output, record.Closure.ChildCloseActionHash); err != nil { return "", err }
			if err := appendExposureText(&output, record.Closure.ChildSnapshotDigest); err != nil { return "", err }
		}
	}
	return handoffSHA256(output.Bytes()), nil
}

type ChildHandoffBookV1 struct {
	terms           GrantTermsV1
	grantDigest     string
	parentActionHash string
	childOwnerID    string
	acceptance      *GrantAcceptanceReceiptV1
	revocation      *GrantRevocationReceiptV1
	exposure        *ExposureBookV1
	closure         *GrantClosureReceiptV1
}

func NewChildHandoffBookV1(terms GrantTermsV1, parentActionHash string) (*ChildHandoffBookV1, error) {
	if err := terms.Validate(); err != nil { return nil, err }
	if err := validateExposureDigest(parentActionHash); err != nil { return nil, err }
	grantDigest, err := terms.CanonicalDigest()
	if err != nil { return nil, err }
	exposurePolicy := ExposurePolicyV1{
		Scope: terms.ChildScope, Sequence: terms.GrantSequence,
		MaximumTotalMinor: terms.BudgetMinor, MaximumPerEffectMinor: terms.BudgetMinor,
		PolicyDigest: grantDigest,
	}
	book, err := NewExposureBookV1(exposurePolicy)
	if err != nil { return nil, err }
	return &ChildHandoffBookV1{
		terms: terms, grantDigest: grantDigest, parentActionHash: parentActionHash,
		childOwnerID: terms.ChildScope.OwnerID, exposure: book,
	}, nil
}

// Clone models restoring child-local durable state after a conductor restart.
func (b *ChildHandoffBookV1) Clone() *ChildHandoffBookV1 {
	clone := *b
	clone.exposure = cloneExposureBookForHandoff(b.exposure)
	if b.acceptance != nil {
		value := *b.acceptance
		clone.acceptance = &value
	}
	if b.revocation != nil {
		value := *b.revocation
		clone.revocation = &value
	}
	if b.closure != nil {
		value := *b.closure
		if value.ChildAcceptActionHash != nil {
			hash := *value.ChildAcceptActionHash
			value.ChildAcceptActionHash = &hash
		}
		clone.closure = &value
	}
	return &clone
}

func cloneExposureBookForHandoff(source *ExposureBookV1) *ExposureBookV1 {
	clone := *source
	clone.reservations = make(map[string]ExposureReservationV1, len(source.reservations))
	for key, reservation := range source.reservations {
		if reservation.Resolution != nil {
			receipt := *reservation.Resolution
			reservation.Resolution = &receipt
		}
		clone.reservations[key] = reservation
	}
	return &clone
}

func (b *ChildHandoffBookV1) AcceptGrant(parentActionHash, childAcceptActionHash string) (HandoffDisposition, GrantAcceptanceReceiptV1, error) {
	if err := validateExposureDigest(parentActionHash); err != nil { return 0, GrantAcceptanceReceiptV1{}, err }
	if err := validateExposureDigest(childAcceptActionHash); err != nil { return 0, GrantAcceptanceReceiptV1{}, err }
	if parentActionHash != b.parentActionHash { return 0, GrantAcceptanceReceiptV1{}, ErrHandoffInvalidGrant }
	if b.acceptance != nil { return HandoffAlreadyObserved, *b.acceptance, nil }
	if b.revocation != nil || b.closure != nil { return 0, GrantAcceptanceReceiptV1{}, ErrHandoffInvalidTransition }
	receipt := GrantAcceptanceReceiptV1{
		GrantID: b.terms.GrantID, GrantDigest: b.grantDigest,
		ParentActionHash: b.parentActionHash, ChildOwnerID: b.childOwnerID,
		ChildAcceptActionHash: childAcceptActionHash,
	}
	b.acceptance = &receipt
	return HandoffObserved, receipt, nil
}

func (b *ChildHandoffBookV1) Reserve(intent ExposureIntentV1) (ExposureDisposition, error) {
	if b.acceptance == nil || b.revocation != nil || b.closure != nil {
		return 0, ErrHandoffInvalidTransition
	}
	return b.exposure.Reserve(intent)
}

func (b *ChildHandoffBookV1) MarkDispatchAuthorized(effectID string) error {
	if b.acceptance == nil || b.revocation != nil || b.closure != nil {
		return ErrHandoffInvalidTransition
	}
	return b.exposure.MarkDispatchAuthorized(effectID)
}

func (b *ChildHandoffBookV1) MarkIndeterminate(effectID string) error {
	return b.exposure.MarkIndeterminate(effectID)
}

func (b *ChildHandoffBookV1) MarkCommittedUnsettled(effectID string, evidence DestinationResolutionV1) error {
	return b.exposure.MarkCommittedUnsettled(effectID, evidence)
}

func (b *ChildHandoffBookV1) MarkSettled(effectID string, evidence DestinationResolutionV1) error {
	return b.exposure.MarkSettled(effectID, evidence)
}

func (b *ChildHandoffBookV1) ReleaseAuthoritativeAbsence(effectID string, evidence DestinationResolutionV1) error {
	return b.exposure.ReleaseAuthoritativeAbsence(effectID, evidence)
}

func (b *ChildHandoffBookV1) CancelBeforeDispatch(effectID string) error {
	return b.exposure.CancelBeforeDispatch(effectID)
}

func (b *ChildHandoffBookV1) ObserveRevocation(
	parentRevokeActionHash, childObservedActionHash string,
) (HandoffDisposition, GrantRevocationReceiptV1, error) {
	if err := validateExposureDigest(parentRevokeActionHash); err != nil {
		return 0, GrantRevocationReceiptV1{}, err
	}
	if err := validateExposureDigest(childObservedActionHash); err != nil {
		return 0, GrantRevocationReceiptV1{}, err
	}
	if b.revocation != nil {
		if b.revocation.ParentRevokeActionHash != parentRevokeActionHash {
			return 0, GrantRevocationReceiptV1{}, ErrHandoffRevocationConflict
		}
		return HandoffAlreadyObserved, *b.revocation, nil
	}
	if b.closure != nil { return 0, GrantRevocationReceiptV1{}, ErrHandoffInvalidTransition }
	receipt := GrantRevocationReceiptV1{
		GrantID: b.terms.GrantID, GrantDigest: b.grantDigest,
		ParentRevokeActionHash: parentRevokeActionHash,
		ChildOwnerID: b.childOwnerID, ChildObservedActionHash: childObservedActionHash,
	}
	b.revocation = &receipt
	return HandoffObserved, receipt, nil
}

func (b *ChildHandoffBookV1) ExposureMinor() (uint64, error) {
	return b.exposure.TotalExposureMinor()
}

func (b *ChildHandoffBookV1) Close(
	childSourceChainFrontier, childCloseActionHash string,
) (HandoffDisposition, GrantClosureReceiptV1, error) {
	if b.closure != nil { return HandoffAlreadyCommitted, *b.closure, nil }
	if b.revocation == nil { return 0, GrantClosureReceiptV1{}, ErrHandoffInvalidTransition }
	if total, err := b.exposure.TotalExposureMinor(); err != nil {
		return 0, GrantClosureReceiptV1{}, err
	} else if total != 0 {
		return 0, GrantClosureReceiptV1{}, ErrHandoffUnresolved
	}
	if err := validateExposureDigest(childSourceChainFrontier); err != nil { return 0, GrantClosureReceiptV1{}, err }
	if err := validateExposureDigest(childCloseActionHash); err != nil { return 0, GrantClosureReceiptV1{}, err }
	var acceptanceHash *string
	if b.acceptance != nil {
		value := b.acceptance.ChildAcceptActionHash
		acceptanceHash = &value
	}
	snapshot, err := b.exposure.CanonicalSnapshotDigest()
	if err != nil { return 0, GrantClosureReceiptV1{}, err }
	receipt := GrantClosureReceiptV1{
		GrantID: b.terms.GrantID, GrantDigest: b.grantDigest,
		ParentActionHash: b.parentActionHash,
		ParentRevokeActionHash: b.revocation.ParentRevokeActionHash,
		ChildOwnerID: b.childOwnerID, ChildAcceptActionHash: acceptanceHash,
		ChildObservedRevokeActionHash: b.revocation.ChildObservedActionHash,
		ChildSourceChainFrontier: childSourceChainFrontier, ChildSnapshotDigest: snapshot,
		ChildCloseActionHash: childCloseActionHash, ZeroUnresolved: true,
	}
	b.closure = &receipt
	return HandoffCommitted, receipt, nil
}

func (b *ChildHandoffBookV1) CanonicalSnapshotDigest() (string, error) {
	var output bytes.Buffer
	output.WriteString(handoffChildDomain)
	for _, value := range []string{b.terms.GrantID, b.grantDigest, b.parentActionHash, b.childOwnerID} {
		if err := appendExposureText(&output, value); err != nil { return "", err }
	}
	if b.acceptance == nil { output.WriteByte(0) } else {
		output.WriteByte(1)
		if err := appendExposureText(&output, b.acceptance.ChildAcceptActionHash); err != nil { return "", err }
	}
	if b.revocation == nil { output.WriteByte(0) } else {
		output.WriteByte(1)
		if err := appendExposureText(&output, b.revocation.ParentRevokeActionHash); err != nil { return "", err }
		if err := appendExposureText(&output, b.revocation.ChildObservedActionHash); err != nil { return "", err }
	}
	exposureDigest, err := b.exposure.CanonicalSnapshotDigest()
	if err != nil { return "", err }
	if err := appendExposureText(&output, exposureDigest); err != nil { return "", err }
	if b.closure == nil { output.WriteByte(0) } else {
		output.WriteByte(1)
		if b.closure.ChildAcceptActionHash == nil { output.WriteByte(0) } else {
			output.WriteByte(1)
			if err := appendExposureText(&output, *b.closure.ChildAcceptActionHash); err != nil { return "", err }
		}
		if err := appendExposureText(&output, b.closure.ChildCloseActionHash); err != nil { return "", err }
		if err := appendExposureText(&output, b.closure.ChildSourceChainFrontier); err != nil { return "", err }
	}
	return handoffSHA256(output.Bytes()), nil
}

func sameHandoffClosure(left, right GrantClosureReceiptV1) bool {
	if left.GrantID != right.GrantID ||
		left.GrantDigest != right.GrantDigest ||
		left.ParentActionHash != right.ParentActionHash ||
		left.ParentRevokeActionHash != right.ParentRevokeActionHash ||
		left.ChildOwnerID != right.ChildOwnerID ||
		left.ChildObservedRevokeActionHash != right.ChildObservedRevokeActionHash ||
		left.ChildSourceChainFrontier != right.ChildSourceChainFrontier ||
		left.ChildSnapshotDigest != right.ChildSnapshotDigest ||
		left.ChildCloseActionHash != right.ChildCloseActionHash ||
		left.ZeroUnresolved != right.ZeroUnresolved {
		return false
	}
	if left.ChildAcceptActionHash == nil || right.ChildAcceptActionHash == nil {
		return left.ChildAcceptActionHash == nil && right.ChildAcceptActionHash == nil
	}
	return *left.ChildAcceptActionHash == *right.ChildAcceptActionHash
}

func validateAcceptanceReceipt(r GrantAcceptanceReceiptV1) error {
	if err := validateExposureText(r.GrantID); err != nil { return err }
	if err := validateExposureText(r.ChildOwnerID); err != nil { return err }
	for _, value := range []string{r.GrantDigest, r.ParentActionHash, r.ChildAcceptActionHash} {
		if err := validateExposureDigest(value); err != nil { return err }
	}
	return nil
}

func validateRevocationReceipt(r GrantRevocationReceiptV1) error {
	if err := validateExposureText(r.GrantID); err != nil { return err }
	if err := validateExposureText(r.ChildOwnerID); err != nil { return err }
	for _, value := range []string{r.GrantDigest, r.ParentRevokeActionHash, r.ChildObservedActionHash} {
		if err := validateExposureDigest(value); err != nil { return err }
	}
	return nil
}

func validateClosureReceipt(r GrantClosureReceiptV1) error {
	if err := validateExposureText(r.GrantID); err != nil { return err }
	if err := validateExposureText(r.ChildOwnerID); err != nil { return err }
	for _, value := range []string{
		r.GrantDigest, r.ParentActionHash, r.ParentRevokeActionHash,
		r.ChildObservedRevokeActionHash, r.ChildSourceChainFrontier,
		r.ChildSnapshotDigest, r.ChildCloseActionHash,
	} {
		if err := validateExposureDigest(value); err != nil { return err }
	}
	if r.ChildAcceptActionHash != nil {
		if err := validateExposureDigest(*r.ChildAcceptActionHash); err != nil { return err }
	}
	if !r.ZeroUnresolved { return ErrHandoffUnresolved }
	return nil
}

func handoffSHA256(bytes []byte) string {
	digest := sha256.Sum256(bytes)
	return "sha256:" + hex.EncodeToString(digest[:])
}
