package temporalrange

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"sort"
	"strings"
	"unicode/utf8"
)

const frontierForestDomainV1 = "MYCELIX-TEMPORAL-FRONTIER-INDEX-FOREST-V0.1\x00"
const intervalProfileV1 = "MYCELIX-INTERVAL-TREE-V0.1"

const (
	maxForestOwnersV1 = 100_000
	maxOwnerGapsV1 = 100_000
)

type OwnerGapRangeV1 struct {
	First uint64
	Last  uint64
}

// OwnerIntervalIndexBindingV1 binds one owner's accepted receipt cut to its temporal index root.
// OwnerSequence and TreeSize are intentionally distinct quantities.
type OwnerIntervalIndexBindingV1 struct {
	OwnerRef              string
	OwnerHistoryRef       string
	OwnerSequence         uint64
	TreeSize              uint64
	OwnerLogRootDigest    string
	IntervalIndexRoot     [32]byte
	ProjectionAuditDigest string
	Gaps                  []OwnerGapRangeV1
}

// TemporalFrontierIndexManifestV1 binds all per-owner interval roots to one verifier frontier.
type TemporalFrontierIndexManifestV1 struct {
	VerifierRef     string
	PolicyEpochRef  string
	FrontierDigest  string
	IntervalProfile string
	QueryScope      IntervalScopeV1
	Owners          []OwnerIntervalIndexBindingV1
}

type OwnerIntervalQueryProofV1 struct {
	OwnerRef string
	Proof    IntervalPointProofV1
}

type TemporalFrontierQueryProofV1 struct {
	Manifest    TemporalFrontierIndexManifestV1
	Query       IntervalInstantV1
	OwnerProofs []OwnerIntervalQueryProofV1
}

type FrontierForestDispositionV1 string

const (
	CompleteForDeclaredIndexForestV1 FrontierForestDispositionV1 = "CompleteForDeclaredIndexForest"
)

type ProjectionCompletenessV1 string

const (
	ProjectionCompletenessNotEvaluatedV1 ProjectionCompletenessV1 = "NotEvaluated"
)

type FrontierIntervalResultV1 struct {
	OwnerRef        string
	OwnerHistoryRef string
	OwnerSequence   uint64
	TreeSize        uint64
	Record          IntervalRecordV1
}

type TemporalFrontierQueryResultV1 struct {
	Disposition            FrontierForestDispositionV1
	ProjectionCompleteness ProjectionCompletenessV1
	HasOwnerGaps           bool
	OwnerCount             int
	Records                []FrontierIntervalResultV1
}

var (
	ErrForestMalformed          = errors.New("malformed temporal frontier forest")
	ErrForestRootMismatch       = errors.New("temporal frontier forest root mismatch")
	ErrForestOwnerSetMismatch   = errors.New("temporal frontier forest owner set mismatch")
	ErrForestOwnerProofMismatch = errors.New("temporal frontier forest owner proof mismatch")
	ErrForestScopeMismatch      = errors.New("temporal frontier forest scope mismatch")
	ErrForestFrontierMismatch   = errors.New("temporal frontier forest frontier mismatch")
)

// TemporalFrontierForestRootV1 canonically commits the ordered owner-to-index bindings.
// The caller must authenticate this root through the separately qualified checkpoint layer.
func TemporalFrontierForestRootV1(manifest TemporalFrontierIndexManifestV1) ([32]byte, error) {
	if err := validateTemporalFrontierManifestV1(manifest); err != nil {
		return [32]byte{}, err
	}
	var preimage bytes.Buffer
	preimage.WriteByte(0x40)
	preimage.WriteString(frontierForestDomainV1)
	appendForestStringV1(&preimage, manifest.VerifierRef)
	appendForestStringV1(&preimage, manifest.PolicyEpochRef)
	appendForestStringV1(&preimage, manifest.FrontierDigest)
	appendForestStringV1(&preimage, manifest.IntervalProfile)
	appendForestStringV1(&preimage, manifest.QueryScope.NamespaceRef)
	appendForestStringV1(&preimage, manifest.QueryScope.SubjectRef)
	appendForestStringV1(&preimage, manifest.QueryScope.ClaimID)
	_ = binary.Write(&preimage, binary.BigEndian, uint32(len(manifest.Owners)))
	for _, owner := range manifest.Owners {
		appendForestStringV1(&preimage, owner.OwnerRef)
		appendForestStringV1(&preimage, owner.OwnerHistoryRef)
		_ = binary.Write(&preimage, binary.BigEndian, owner.OwnerSequence)
		_ = binary.Write(&preimage, binary.BigEndian, owner.TreeSize)
		appendForestStringV1(&preimage, owner.OwnerLogRootDigest)
		preimage.Write(owner.IntervalIndexRoot[:])
		appendForestStringV1(&preimage, owner.ProjectionAuditDigest)
		_ = binary.Write(&preimage, binary.BigEndian, uint32(len(owner.Gaps)))
		for _, gap := range owner.Gaps {
			_ = binary.Write(&preimage, binary.BigEndian, gap.First)
			_ = binary.Write(&preimage, binary.BigEndian, gap.Last)
		}
	}
	return sha256.Sum256(preimage.Bytes()), nil
}

// VerifyTemporalFrontierQueryProofV1 verifies every per-owner interval proof against an exact
// owner set and forest root supplied by the caller's separately verified frontier/checkpoint.
// This establishes completeness relative to the declared index forest only. Source-receipt
// projection completeness is deliberately NotEvaluated by this function.
func VerifyTemporalFrontierQueryProofV1(
	expectedVerifier string,
	expectedPolicyEpoch string,
	expectedFrontierDigest string,
	expectedScope IntervalScopeV1,
	expectedOwnerRefs []string,
	expectedQuery IntervalInstantV1,
	proof TemporalFrontierQueryProofV1,
	expectedForestRoot [32]byte,
) (TemporalFrontierQueryResultV1, error) {
	if err := validateForestStringV1(expectedVerifier); err != nil {
		return TemporalFrontierQueryResultV1{}, err
	}
	if err := validateForestStringV1(expectedPolicyEpoch); err != nil {
		return TemporalFrontierQueryResultV1{}, err
	}
	if err := validateForestDigestV1(expectedFrontierDigest); err != nil {
		return TemporalFrontierQueryResultV1{}, err
	}
	if validateInstantV1(expectedQuery) != nil {
		return TemporalFrontierQueryResultV1{}, ErrForestMalformed
	}
	if proof.Query != expectedQuery {
		return TemporalFrontierQueryResultV1{}, ErrIntervalQueryMismatch
	}
	if proof.Manifest.VerifierRef != expectedVerifier || proof.Manifest.PolicyEpochRef != expectedPolicyEpoch {
		return TemporalFrontierQueryResultV1{}, ErrForestScopeMismatch
	}
	if proof.Manifest.FrontierDigest != expectedFrontierDigest {
		return TemporalFrontierQueryResultV1{}, ErrForestFrontierMismatch
	}
	if proof.Manifest.IntervalProfile != intervalProfileV1 {
		return TemporalFrontierQueryResultV1{}, ErrForestMalformed
	}
	if proof.Manifest.QueryScope != expectedScope ||
		expectedScope.VerifierRef != expectedVerifier ||
		expectedScope.PolicyEpochRef != expectedPolicyEpoch ||
		expectedScope.SourceFrontierDigest != expectedFrontierDigest {
		return TemporalFrontierQueryResultV1{}, ErrForestScopeMismatch
	}
	if len(proof.OwnerProofs) != len(proof.Manifest.Owners) {
		return TemporalFrontierQueryResultV1{}, ErrForestOwnerProofMismatch
	}

	expectedOwners := append([]string(nil), expectedOwnerRefs...)
	sort.Strings(expectedOwners)
	for i, owner := range expectedOwners {
		if err := validateForestStringV1(owner); err != nil {
			return TemporalFrontierQueryResultV1{}, err
		}
		if i > 0 && expectedOwners[i-1] == owner {
			return TemporalFrontierQueryResultV1{}, ErrForestMalformed
		}
	}
	if len(expectedOwners) != len(proof.Manifest.Owners) {
		return TemporalFrontierQueryResultV1{}, ErrForestOwnerSetMismatch
	}
	for i, owner := range proof.Manifest.Owners {
		if owner.OwnerRef != expectedOwners[i] {
			return TemporalFrontierQueryResultV1{}, ErrForestOwnerSetMismatch
		}
		if proof.OwnerProofs[i].OwnerRef != owner.OwnerRef {
			return TemporalFrontierQueryResultV1{}, ErrForestOwnerProofMismatch
		}
		if proof.OwnerProofs[i].Proof.Query != expectedQuery ||
			proof.OwnerProofs[i].Proof.Scope != expectedScope {
			return TemporalFrontierQueryResultV1{}, ErrForestOwnerProofMismatch
		}
	}

	root, err := TemporalFrontierForestRootV1(proof.Manifest)
	if err != nil {
		return TemporalFrontierQueryResultV1{}, err
	}
	if root != expectedForestRoot {
		return TemporalFrontierQueryResultV1{}, ErrForestRootMismatch
	}

	result := TemporalFrontierQueryResultV1{
		Disposition:            CompleteForDeclaredIndexForestV1,
		ProjectionCompleteness: ProjectionCompletenessNotEvaluatedV1,
		OwnerCount:             len(proof.Manifest.Owners),
		Records:                make([]FrontierIntervalResultV1, 0),
	}
	for i, owner := range proof.Manifest.Owners {
		if len(owner.Gaps) > 0 {
			result.HasOwnerGaps = true
		}
		verified, err := VerifyIntervalPointProofV1(
			expectedScope, expectedQuery, proof.OwnerProofs[i].Proof, owner.IntervalIndexRoot,
		)
		if err != nil {
			return TemporalFrontierQueryResultV1{}, err
		}
		for _, record := range verified.Records {
			result.Records = append(result.Records, FrontierIntervalResultV1{
				OwnerRef: owner.OwnerRef, OwnerHistoryRef: owner.OwnerHistoryRef,
				OwnerSequence: owner.OwnerSequence, TreeSize: owner.TreeSize, Record: record,
			})
		}
	}
	return result, nil
}

func validateTemporalFrontierManifestV1(manifest TemporalFrontierIndexManifestV1) error {
	if validateForestStringV1(manifest.VerifierRef) != nil ||
		validateForestStringV1(manifest.PolicyEpochRef) != nil ||
		validateForestDigestV1(manifest.FrontierDigest) != nil ||
		validateForestStringV1(manifest.IntervalProfile) != nil {
		return ErrForestMalformed
	}
	if manifest.QueryScope.VerifierRef != manifest.VerifierRef ||
		manifest.QueryScope.PolicyEpochRef != manifest.PolicyEpochRef ||
		manifest.QueryScope.SourceFrontierDigest != manifest.FrontierDigest ||
		validateIntervalScopeV1(manifest.QueryScope) != nil {
		return ErrForestScopeMismatch
	}
	if len(manifest.Owners) > maxForestOwnersV1 {
		return ErrForestMalformed
	}
	for i, owner := range manifest.Owners {
		if len(owner.Gaps) > maxOwnerGapsV1 {
			return ErrForestMalformed
		}
		if validateForestStringV1(owner.OwnerRef) != nil ||
			validateForestStringV1(owner.OwnerHistoryRef) != nil ||
			validateForestDigestV1(owner.OwnerLogRootDigest) != nil ||
			(owner.ProjectionAuditDigest != "" && validateForestDigestV1(owner.ProjectionAuditDigest) != nil) {
			return ErrForestMalformed
		}
		if i > 0 && manifest.Owners[i-1].OwnerRef >= owner.OwnerRef {
			return ErrForestMalformed
		}
		var previousLast uint64
		for gapIndex, gap := range owner.Gaps {
			if gap.First == 0 || gap.Last < gap.First || gap.Last > owner.OwnerSequence {
				return ErrForestMalformed
			}
			if gapIndex > 0 &&
				(gap.First <= previousLast || (previousLast != ^uint64(0) && gap.First == previousLast+1)) {
				return ErrForestMalformed
			}
			previousLast = gap.Last
		}
	}
	return nil
}

func appendForestStringV1(dst *bytes.Buffer, value string) {
	_ = binary.Write(dst, binary.BigEndian, uint32(len(value)))
	_, _ = dst.WriteString(value)
}

func validateForestStringV1(value string) error {
	if value == "" || strings.TrimSpace(value) == "" || len(value) > 1_024 || !utf8.ValidString(value) {
		return ErrForestMalformed
	}
	return nil
}

func validateForestDigestV1(value string) error {
	if len(value) != len("sha256:")+64 || !strings.HasPrefix(value, "sha256:") {
		return ErrForestMalformed
	}
	for _, ch := range value[len("sha256:"):] {
		if !((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')) {
			return ErrForestMalformed
		}
	}
	return nil
}
