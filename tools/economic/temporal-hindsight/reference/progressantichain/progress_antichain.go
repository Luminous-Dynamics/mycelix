package progressantichain

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sort"
	"strings"
)

const (
	ProgressProvisional ProgressDispositionV1 = "Provisional"
	ProgressStableRelativeToDeclared ProgressDispositionV1 = "StableRelativeToDeclaredProgress"
	ProgressIndeterminate ProgressDispositionV1 = "Indeterminate"

	AdvanceMonotone ProgressAdvanceDispositionV1 = "Monotone"
	AdvanceIndeterminate ProgressAdvanceDispositionV1 = "Indeterminate"

	NotEvaluated = "NotEvaluated"
)

type ProgressDispositionV1 string
type ProgressAdvanceDispositionV1 string

// ProgressScopeV1 fixes the owner universe and interpretation domain for
// one progress antichain. OwnerRefs are canonicalized as a set for hashing.
type ProgressScopeV1 struct {
	SourceRef        string
	RegistryRef      string
	RegistryEpoch    string
	ProfileRef       string
	QueryScopeDigest string
	OwnerRefs        []string
}

// OwnerSequenceV1 is one coordinate in a product-ordered causal cut.
type OwnerSequenceV1 struct {
	OwnerRef string
	Sequence uint64
}

// ProgressCutV1 is a vector of owner-local sequence positions. It is not a
// global timestamp and two such cuts may be incomparable.
type ProgressCutV1 struct {
	Owners []OwnerSequenceV1
}

// ProgressAntichainV1 is the set of minimal progress points that may still
// arrive. The empty set means no future progress point is declared.
type ProgressAntichainV1 struct {
	Scope  ProgressScopeV1
	Points []ProgressCutV1
}

type ProgressResultV1 struct {
	Disposition                ProgressDispositionV1
	Reason                     string
	ScopeDigest                string
	ProgressAntichainDigest    string
	QueryCutDigest             string
	ProgressAuthentication     string
	SourceUniverseCompleteness string
	CertificateDigest          string
}

type ProgressAdvanceResultV1 struct {
	Disposition                ProgressAdvanceDispositionV1
	Reason                     string
	ScopeDigest                string
	PreviousAntichainDigest    string
	NextAntichainDigest        string
	ProgressAuthentication     string
	SourceUniverseCompleteness string
	CertificateDigest          string
}

type pointVectorV1 struct {
	digest string
	values []uint64
}

func writeStringV1(b *bytes.Buffer, value string) {
	_ = binary.Write(b, binary.BigEndian, uint32(len(value)))
	_, _ = b.WriteString(value)
}

func validDigestV1(value string) bool {
	if len(value) != 64 || strings.ToLower(value) != value {
		return false
	}
	_, err := hex.DecodeString(value)
	return err == nil
}

func canonicalOwnersV1(scope ProgressScopeV1) ([]string, error) {
	if strings.TrimSpace(scope.SourceRef) == "" ||
		strings.TrimSpace(scope.RegistryRef) == "" ||
		strings.TrimSpace(scope.RegistryEpoch) == "" ||
		strings.TrimSpace(scope.ProfileRef) == "" {
		return nil, errors.New("progress scope identity is incomplete")
	}
	if !validDigestV1(scope.QueryScopeDigest) {
		return nil, errors.New("query scope digest is malformed")
	}
	owners := append([]string(nil), scope.OwnerRefs...)
	sort.Strings(owners)
	for i, owner := range owners {
		if strings.TrimSpace(owner) == "" {
			return nil, errors.New("progress scope contains an empty owner reference")
		}
		if i > 0 && owners[i-1] == owner {
			return nil, errors.New("progress scope contains a duplicate owner reference")
		}
	}
	return owners, nil
}

func ProgressScopeDigestV1(scope ProgressScopeV1) (string, error) {
	owners, err := canonicalOwnersV1(scope)
	if err != nil {
		return "", err
	}
	var b bytes.Buffer
	writeStringV1(&b, "MYCELIX-PROGRESS-SCOPE-V1")
	for _, value := range []string{
		scope.SourceRef, scope.RegistryRef, scope.RegistryEpoch,
		scope.ProfileRef, scope.QueryScopeDigest,
	} {
		writeStringV1(&b, value)
	}
	_ = binary.Write(&b, binary.BigEndian, uint32(len(owners)))
	for _, owner := range owners {
		writeStringV1(&b, owner)
	}
	digest := sha256.Sum256(b.Bytes())
	return hex.EncodeToString(digest[:]), nil
}

func progressVectorV1(cut ProgressCutV1, owners []string) ([]uint64, error) {
	if len(cut.Owners) != len(owners) {
		return nil, errors.New("progress cut owner coordinates do not match the declared universe")
	}
	coordinates := make(map[string]uint64, len(cut.Owners))
	for _, coordinate := range cut.Owners {
		if strings.TrimSpace(coordinate.OwnerRef) == "" {
			return nil, errors.New("progress cut contains an empty owner coordinate")
		}
		if _, exists := coordinates[coordinate.OwnerRef]; exists {
			return nil, errors.New("progress cut contains a duplicate owner coordinate")
		}
		coordinates[coordinate.OwnerRef] = coordinate.Sequence
	}
	values := make([]uint64, len(owners))
	for i, owner := range owners {
		sequence, exists := coordinates[owner]
		if !exists {
			return nil, errors.New("progress cut omits a declared owner coordinate")
		}
		values[i] = sequence
	}
	if len(coordinates) != len(owners) {
		return nil, errors.New("progress cut contains an owner outside the declared universe")
	}
	return values, nil
}

func progressPointDigestV1(scopeDigest string, owners []string, values []uint64) string {
	var b bytes.Buffer
	writeStringV1(&b, "MYCELIX-PROGRESS-POINT-V1")
	writeStringV1(&b, scopeDigest)
	for i, owner := range owners {
		writeStringV1(&b, owner)
		_ = binary.Write(&b, binary.BigEndian, values[i])
	}
	digest := sha256.Sum256(b.Bytes())
	return hex.EncodeToString(digest[:])
}

func lessOrEqualV1(left, right []uint64) bool {
	if len(left) != len(right) {
		return false
	}
	for i := range left {
		if left[i] > right[i] {
			return false
		}
	}
	return true
}

func validateProgressAntichainV1(antichain ProgressAntichainV1) (string, string, []pointVectorV1, error) {
	owners, err := canonicalOwnersV1(antichain.Scope)
	if err != nil {
		return "", "", nil, err
	}
	scopeDigest, err := ProgressScopeDigestV1(antichain.Scope)
	if err != nil {
		return "", "", nil, err
	}
	points := make([]pointVectorV1, len(antichain.Points))
	for i, point := range antichain.Points {
		values, err := progressVectorV1(point, owners)
		if err != nil {
			return scopeDigest, "", nil, fmt.Errorf("progress point %d: %w", i, err)
		}
		points[i] = pointVectorV1{
			digest: progressPointDigestV1(scopeDigest, owners, values),
			values: values,
		}
	}
	sort.Slice(points, func(i, j int) bool { return points[i].digest < points[j].digest })
	for i := 0; i < len(points); i++ {
		if i > 0 && points[i-1].digest == points[i].digest {
			return scopeDigest, "", nil, errors.New("progress antichain contains a duplicate point")
		}
		for j := i + 1; j < len(points); j++ {
			if lessOrEqualV1(points[i].values, points[j].values) ||
				lessOrEqualV1(points[j].values, points[i].values) {
				return scopeDigest, "", nil, errors.New("progress points are not pairwise incomparable")
			}
		}
	}
	var b bytes.Buffer
	writeStringV1(&b, "MYCELIX-PROGRESS-ANTICHAIN-V1")
	writeStringV1(&b, scopeDigest)
	_ = binary.Write(&b, binary.BigEndian, uint32(len(points)))
	for _, point := range points {
		writeStringV1(&b, point.digest)
	}
	digest := sha256.Sum256(b.Bytes())
	return scopeDigest, hex.EncodeToString(digest[:]), points, nil
}

// ValidateProgressAntichainV1 checks the fixed owner universe, point shape,
// duplicate/cross-scope coordinates and pairwise incomparability. A valid
// antichain is structural input, not authenticated source progress.
func ValidateProgressAntichainV1(antichain ProgressAntichainV1) (string, error) {
	_, digest, _, err := validateProgressAntichainV1(antichain)
	if err != nil {
		return "", err
	}
	return digest, nil
}

func finalizeProgressResultV1(result ProgressResultV1) ProgressResultV1 {
	var b bytes.Buffer
	writeStringV1(&b, "MYCELIX-PROGRESS-RESULT-V1")
	writeStringV1(&b, string(result.Disposition))
	writeStringV1(&b, result.Reason)
	writeStringV1(&b, result.ScopeDigest)
	writeStringV1(&b, result.ProgressAntichainDigest)
	writeStringV1(&b, result.QueryCutDigest)
	writeStringV1(&b, result.ProgressAuthentication)
	writeStringV1(&b, result.SourceUniverseCompleteness)
	digest := sha256.Sum256(b.Bytes())
	result.CertificateDigest = hex.EncodeToString(digest[:])
	return result
}

// EvaluateProgressAtV1 is conservative: if any minimal possible future point
// is <= the query cut, the query remains provisional. Otherwise it is only
// stable relative to the declared progress antichain. Authentication and
// source-universe completeness remain NotEvaluated.
func EvaluateProgressAtV1(antichain ProgressAntichainV1, queryCut ProgressCutV1) ProgressResultV1 {
	result := ProgressResultV1{
		Disposition: ProgressIndeterminate,
		Reason: "progress evaluation is indeterminate",
		ProgressAuthentication: NotEvaluated,
		SourceUniverseCompleteness: NotEvaluated,
	}
	scopeDigest, antichainDigest, points, err := validateProgressAntichainV1(antichain)
	result.ScopeDigest = scopeDigest
	result.ProgressAntichainDigest = antichainDigest
	if err != nil {
		result.Reason = err.Error()
		return finalizeProgressResultV1(result)
	}
	owners, _ := canonicalOwnersV1(antichain.Scope)
	queryValues, err := progressVectorV1(queryCut, owners)
	if err != nil {
		result.Reason = "query cut: " + err.Error()
		return finalizeProgressResultV1(result)
	}
	result.QueryCutDigest = progressPointDigestV1(scopeDigest, owners, queryValues)
	for _, point := range points {
		if lessOrEqualV1(point.values, queryValues) {
			result.Disposition = ProgressProvisional
			result.Reason = "a declared future progress point can still affect the query cut"
			return finalizeProgressResultV1(result)
		}
	}
	result.Disposition = ProgressStableRelativeToDeclared
	if len(points) == 0 {
		result.Reason = "the declared progress antichain is empty"
	} else {
		result.Reason = "no declared future progress point can affect the query cut"
	}
	return finalizeProgressResultV1(result)
}

func finalizeProgressAdvanceV1(result ProgressAdvanceResultV1) ProgressAdvanceResultV1 {
	var b bytes.Buffer
	writeStringV1(&b, "MYCELIX-PROGRESS-ADVANCE-RESULT-V1")
	writeStringV1(&b, string(result.Disposition))
	writeStringV1(&b, result.Reason)
	writeStringV1(&b, result.ScopeDigest)
	writeStringV1(&b, result.PreviousAntichainDigest)
	writeStringV1(&b, result.NextAntichainDigest)
	writeStringV1(&b, result.ProgressAuthentication)
	writeStringV1(&b, result.SourceUniverseCompleteness)
	digest := sha256.Sum256(b.Bytes())
	result.CertificateDigest = hex.EncodeToString(digest[:])
	return result
}

// AdvanceProgressAntichainV1 checks monotone progress under a fixed scope.
// For every new frontier point b there must be an old point a with a <= b.
// An empty new antichain is a valid completion declaration; it is not proof
// that the source is authentic or that its owner universe is complete.
func AdvanceProgressAntichainV1(previous, next ProgressAntichainV1) ProgressAdvanceResultV1 {
	result := ProgressAdvanceResultV1{
		Disposition: AdvanceIndeterminate,
		Reason: "progress advancement is indeterminate",
		ProgressAuthentication: NotEvaluated,
		SourceUniverseCompleteness: NotEvaluated,
	}
	oldScope, oldDigest, oldPoints, oldErr := validateProgressAntichainV1(previous)
	newScope, newDigest, newPoints, newErr := validateProgressAntichainV1(next)
	result.ScopeDigest = oldScope
	result.PreviousAntichainDigest = oldDigest
	result.NextAntichainDigest = newDigest
	if oldErr != nil {
		result.Reason = "previous antichain: " + oldErr.Error()
		return finalizeProgressAdvanceV1(result)
	}
	if newErr != nil {
		result.Reason = "next antichain: " + newErr.Error()
		return finalizeProgressAdvanceV1(result)
	}
	if oldScope != newScope {
		result.Reason = "progress advancement crosses source, registry epoch, profile, query scope or owner-universe identity"
		return finalizeProgressAdvanceV1(result)
	}
	for _, nextPoint := range newPoints {
		dominatedByPrevious := false
		for _, previousPoint := range oldPoints {
			if lessOrEqualV1(previousPoint.values, nextPoint.values) {
				dominatedByPrevious = true
				break
			}
		}
		if !dominatedByPrevious {
			result.Reason = "next progress frontier regresses or introduces an ungrounded future point"
			return finalizeProgressAdvanceV1(result)
		}
	}
	result.Disposition = AdvanceMonotone
	result.Reason = "every next progress point is above a previous progress point under the product partial order"
	return finalizeProgressAdvanceV1(result)
}
