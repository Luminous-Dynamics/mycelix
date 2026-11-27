package temporalrange

import (
	"errors"
	"sort"
)

type TemporalFrontierProjectionDispositionV1 string

const (
	ProjectionMatchesAllSuppliedOwnerInventoriesV1 TemporalFrontierProjectionDispositionV1 = "ProjectionMatchesAllSuppliedOwnerInventories"
)

type TemporalFrontierProjectionReplayResultV1 struct {
	Disposition                    TemporalFrontierProjectionDispositionV1
	ReceiptInventoryAuthentication ReceiptInventoryAuthenticationV1
	OwnerCount                     int
	HasOwnerGaps                   bool
	OwnerAudits                    []OwnerProjectionReplayAuditV1
}

var (
	ErrForestProjectionOwnerSetMismatch = errors.New("projection inventory owner set does not match accepted frontier")
	ErrForestProjectionInventoryMismatch = errors.New("projection inventories do not match forest owner bindings")
)

// VerifyTemporalFrontierProjectionReplayV1 verifies that every declared owner index root in the
// exact forest manifest is reproducible from the supplied owner-local accepted receipt inventory.
//
// The expected forest root and owner set must come from a separately verified checkpoint/frontier.
// This function proves only that each supplied inventory deterministically projects to its
// committed per-owner interval-index root and projection-audit digest. It deliberately returns
// ReceiptInventoryAuthenticationNotEvaluated: the caller must still verify that each supplied
// inventory is the complete accepted receipt-log prefix represented by the authenticated frontier.
func VerifyTemporalFrontierProjectionReplayV1(
	expectedVerifier string,
	expectedPolicyEpoch string,
	expectedFrontierDigest string,
	expectedScope IntervalScopeV1,
	expectedOwnerRefs []string,
	expectedForestRoot [32]byte,
	manifest TemporalFrontierIndexManifestV1,
	inventories []AcceptedReceiptInventoryV1,
) (TemporalFrontierProjectionReplayResultV1, error) {
	if expectedScope.VerifierRef != expectedVerifier ||
		expectedScope.PolicyEpochRef != expectedPolicyEpoch ||
		expectedScope.SourceFrontierDigest != expectedFrontierDigest ||
		validateIntervalScopeV1(expectedScope) != nil {
		return TemporalFrontierProjectionReplayResultV1{}, ErrForestScopeMismatch
	}
	if manifest.VerifierRef != expectedVerifier ||
		manifest.PolicyEpochRef != expectedPolicyEpoch ||
		manifest.FrontierDigest != expectedFrontierDigest ||
		manifest.QueryScope != expectedScope {
		return TemporalFrontierProjectionReplayResultV1{}, ErrForestFrontierMismatch
	}
	if err := validateTemporalFrontierManifestV1(manifest); err != nil {
		return TemporalFrontierProjectionReplayResultV1{}, err
	}

	expected := append([]string(nil), expectedOwnerRefs...)
	sort.Strings(expected)
	if len(expected) != len(manifest.Owners) || len(inventories) != len(manifest.Owners) {
		return TemporalFrontierProjectionReplayResultV1{}, ErrForestProjectionOwnerSetMismatch
	}
	for i, owner := range expected {
		if validateForestStringV1(owner) != nil || (i > 0 && expected[i-1] == owner) {
			return TemporalFrontierProjectionReplayResultV1{}, ErrForestMalformed
		}
		if manifest.Owners[i].OwnerRef != owner {
			return TemporalFrontierProjectionReplayResultV1{}, ErrForestProjectionOwnerSetMismatch
		}
	}

	root, err := TemporalFrontierForestRootV1(manifest)
	if err != nil {
		return TemporalFrontierProjectionReplayResultV1{}, err
	}
	if root != expectedForestRoot {
		return TemporalFrontierProjectionReplayResultV1{}, ErrForestRootMismatch
	}

	result := TemporalFrontierProjectionReplayResultV1{
		Disposition: ProjectionMatchesAllSuppliedOwnerInventoriesV1,
		ReceiptInventoryAuthentication: ReceiptInventoryAuthenticationNotEvaluatedV1,
		OwnerCount: len(manifest.Owners),
		OwnerAudits: make([]OwnerProjectionReplayAuditV1, 0, len(manifest.Owners)),
	}
	for i, binding := range manifest.Owners {
		if inventories[i].OwnerRef != binding.OwnerRef {
			return TemporalFrontierProjectionReplayResultV1{}, ErrForestProjectionInventoryMismatch
		}
		audit, err := VerifyOwnerProjectionReplayAuditV1(expectedScope, binding, inventories[i])
		if err != nil {
			return TemporalFrontierProjectionReplayResultV1{}, err
		}
		result.OwnerAudits = append(result.OwnerAudits, audit)
		if audit.HasOwnerGaps {
			result.HasOwnerGaps = true
		}
	}
	return result, nil
}
