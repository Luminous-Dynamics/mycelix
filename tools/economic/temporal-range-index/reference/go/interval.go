package temporalrange

import (
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"math"
	"sort"
	"strings"
	"unicode/utf8"
)

const intervalQueryDomainV1 = "MYCELIX-AUTHENTICATED-INTERVAL-QUERY-V0.1\x00"

// IntervalInstantV1 is an exact UTC instant. Nanos must be less than 1e9.
type IntervalInstantV1 struct {
	Seconds int64
	Nanos   uint32
}

// IntervalEndV1 uses Open=true for +infinity. At must be zero when Open=true.
type IntervalEndV1 struct {
	Open bool
	At   IntervalInstantV1
}

// IntervalRecordV1 is one assertion's half-open validity interval [ValidFrom, ValidTo).
type IntervalRecordV1 struct {
	AssertionRef  string
	ValidFrom     IntervalInstantV1
	ValidTo       IntervalEndV1
	ReceiptDigest string
}

// IntervalScopeV1 binds one index to one verifier, policy, namespace, claim and source frontier.
type IntervalScopeV1 struct {
	VerifierRef          string
	PolicyEpochRef       string
	NamespaceRef         string
	SubjectRef           string
	ClaimID              string
	SourceFrontierDigest string
}

// IntervalSummaryV1 is committed into every internal-node hash.
type IntervalSummaryV1 struct {
	MinValidFrom IntervalInstantV1
	MaxValidTo   IntervalEndV1
}

type intervalNodeV1 struct {
	count   uint64
	summary IntervalSummaryV1
	hash    [32]byte
	left    *intervalNodeV1
	right   *intervalNodeV1
	record  *IntervalRecordV1
}

// IntervalTreeV1 is a deterministic authenticated interval index for one exact scope.
type IntervalTreeV1 struct {
	scope    IntervalScopeV1
	count    uint64
	root     *intervalNodeV1
	rootHash [32]byte
}

const (
	IntervalProofPrunedV1 uint8 = 1
	IntervalProofBranchV1 uint8 = 2
	IntervalProofLeafV1   uint8 = 3
)

// IntervalProofNodeV1 is either a safe-pruned subtree, an expanded branch, or a returned matching leaf.
type IntervalProofNodeV1 struct {
	Kind    uint8
	Count   uint64
	Summary IntervalSummaryV1
	Hash    [32]byte
	Record  *IntervalRecordV1
	Left    *IntervalProofNodeV1
	Right   *IntervalProofNodeV1
}

// IntervalPointProofV1 binds a proof to an exact scope and query instant.
type IntervalPointProofV1 struct {
	Scope IntervalScopeV1
	Query IntervalInstantV1
	Count uint64
	Tree  *IntervalProofNodeV1
}

type IntervalDispositionV1 string

const (
	CompleteIntervalResultForRoot      IntervalDispositionV1 = "CompleteIntervalResultForRoot"
	CompleteEmptyIntervalResultForRoot IntervalDispositionV1 = "CompleteEmptyIntervalResultForRoot"
)

type IntervalVerificationResultV1 struct {
	Disposition IntervalDispositionV1
	Records     []IntervalRecordV1
}

var (
	ErrIntervalMalformed     = errors.New("malformed interval index or proof")
	ErrIntervalScopeMismatch = errors.New("interval proof scope mismatch")
	ErrIntervalQueryMismatch = errors.New("interval proof query mismatch")
	ErrIntervalRootMismatch  = errors.New("interval proof root mismatch")
	ErrIntervalUnsafePrune   = errors.New("interval proof prunes a subtree that may contain a match")
)

// NewIntervalTreeV1 validates and indexes the exact records for one scope.
func NewIntervalTreeV1(scope IntervalScopeV1, records []IntervalRecordV1) (*IntervalTreeV1, error) {
	if err := validateIntervalScopeV1(scope); err != nil {
		return nil, err
	}
	ordered := append([]IntervalRecordV1(nil), records...)
	seen := make(map[string]struct{}, len(ordered))
	for _, record := range ordered {
		if err := validateIntervalRecordV1(record); err != nil {
			return nil, err
		}
		if _, exists := seen[record.AssertionRef]; exists {
			return nil, ErrIntervalMalformed
		}
		seen[record.AssertionRef] = struct{}{}
	}
	sort.Slice(ordered, func(i, j int) bool {
		if c := compareInstantV1(ordered[i].ValidFrom, ordered[j].ValidFrom); c != 0 {
			return c < 0
		}
		return ordered[i].AssertionRef < ordered[j].AssertionRef
	})
	root := buildIntervalNodeV1(ordered)
	var count uint64
	var treeHash [32]byte
	if root == nil {
		treeHash = hashIntervalEmptyV1()
	} else {
		count = root.count
		treeHash = root.hash
	}
	return &IntervalTreeV1{
		scope: scope, count: count, root: root,
		rootHash: hashIntervalScopedRootV1(scope, count, treeHash),
	}, nil
}

func (tree *IntervalTreeV1) RootHash() [32]byte { return tree.rootHash }
func (tree *IntervalTreeV1) Count() uint64      { return tree.count }

// ProvePointQueryV1 constructs a complete proof for one exact point query.
func (tree *IntervalTreeV1) ProvePointQueryV1(query IntervalInstantV1) (IntervalPointProofV1, error) {
	if tree == nil || validateInstantV1(query) != nil {
		return IntervalPointProofV1{}, ErrIntervalMalformed
	}
	return IntervalPointProofV1{
		Scope: tree.scope, Query: query, Count: tree.count,
		Tree: buildIntervalProofNodeV1(tree.root, query),
	}, nil
}

// VerifyIntervalPointProofV1 verifies completeness relative to expectedRoot and expectedScope.
// It does not authenticate the root or establish source/index projection completeness.
func VerifyIntervalPointProofV1(
	expectedScope IntervalScopeV1,
	expectedQuery IntervalInstantV1,
	proof IntervalPointProofV1,
	expectedRoot [32]byte,
) (IntervalVerificationResultV1, error) {
	if err := validateIntervalScopeV1(expectedScope); err != nil || validateInstantV1(expectedQuery) != nil {
		return IntervalVerificationResultV1{}, ErrIntervalMalformed
	}
	if proof.Scope != expectedScope {
		return IntervalVerificationResultV1{}, ErrIntervalScopeMismatch
	}
	if proof.Query != expectedQuery {
		return IntervalVerificationResultV1{}, ErrIntervalQueryMismatch
	}
	if proof.Count == 0 {
		if proof.Tree != nil {
			return IntervalVerificationResultV1{}, ErrIntervalMalformed
		}
		emptyRoot := hashIntervalScopedRootV1(expectedScope, 0, hashIntervalEmptyV1())
		if emptyRoot != expectedRoot {
			return IntervalVerificationResultV1{}, ErrIntervalRootMismatch
		}
		return IntervalVerificationResultV1{
			Disposition: CompleteEmptyIntervalResultForRoot,
			Records:     []IntervalRecordV1{},
		}, nil
	}
	if proof.Tree == nil {
		return IntervalVerificationResultV1{}, ErrIntervalMalformed
	}
	seen := make(map[string]struct{})
	var records []IntervalRecordV1
	verified, err := verifyIntervalProofNodeV1(proof.Tree, expectedQuery, seen, &records)
	if err != nil {
		return IntervalVerificationResultV1{}, err
	}
	if verified.count != proof.Count {
		return IntervalVerificationResultV1{}, ErrIntervalMalformed
	}
	root := hashIntervalScopedRootV1(expectedScope, verified.count, verified.hash)
	if root != expectedRoot {
		return IntervalVerificationResultV1{}, ErrIntervalRootMismatch
	}
	disposition := CompleteIntervalResultForRoot
	if len(records) == 0 {
		disposition = CompleteEmptyIntervalResultForRoot
	}
	return IntervalVerificationResultV1{Disposition: disposition, Records: records}, nil
}

func buildIntervalNodeV1(records []IntervalRecordV1) *intervalNodeV1 {
	if len(records) == 0 {
		return nil
	}
	if len(records) == 1 {
		record := records[0]
		copyRecord := record
		summary := IntervalSummaryV1{MinValidFrom: record.ValidFrom, MaxValidTo: record.ValidTo}
		return &intervalNodeV1{
			count: 1, summary: summary, hash: hashIntervalLeafV1(record), record: &copyRecord,
		}
	}
	mid := len(records) / 2
	left := buildIntervalNodeV1(records[:mid])
	right := buildIntervalNodeV1(records[mid:])
	count := left.count + right.count
	summary := combineIntervalSummaryV1(left.summary, right.summary)
	return &intervalNodeV1{
		count: count, summary: summary,
		hash: hashIntervalBranchV1(count, summary, left.hash, right.hash),
		left: left, right: right,
	}
}

func buildIntervalProofNodeV1(node *intervalNodeV1, query IntervalInstantV1) *IntervalProofNodeV1 {
	if node == nil {
		return nil
	}
	if intervalCanPruneV1(node.summary, query) {
		return &IntervalProofNodeV1{
			Kind: IntervalProofPrunedV1, Count: node.count, Summary: node.summary, Hash: node.hash,
		}
	}
	if node.record != nil {
		record := *node.record
		return &IntervalProofNodeV1{
			Kind: IntervalProofLeafV1, Count: 1, Summary: node.summary, Hash: node.hash, Record: &record,
		}
	}
	return &IntervalProofNodeV1{
		Kind: IntervalProofBranchV1,
		Left: buildIntervalProofNodeV1(node.left, query),
		Right: buildIntervalProofNodeV1(node.right, query),
	}
}

type verifiedIntervalNodeV1 struct {
	count   uint64
	summary IntervalSummaryV1
	hash    [32]byte
}

func verifyIntervalProofNodeV1(
	node *IntervalProofNodeV1,
	query IntervalInstantV1,
	seen map[string]struct{},
	records *[]IntervalRecordV1,
) (verifiedIntervalNodeV1, error) {
	if node == nil {
		return verifiedIntervalNodeV1{}, ErrIntervalMalformed
	}
	switch node.Kind {
	case IntervalProofPrunedV1:
		if node.Count == 0 || node.Record != nil || node.Left != nil || node.Right != nil ||
			!validIntervalSummaryV1(node.Summary) {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		if !intervalCanPruneV1(node.Summary, query) {
			return verifiedIntervalNodeV1{}, ErrIntervalUnsafePrune
		}
		return verifiedIntervalNodeV1{count: node.Count, summary: node.Summary, hash: node.Hash}, nil
	case IntervalProofBranchV1:
		if node.Record != nil || node.Count != 0 || node.Summary != (IntervalSummaryV1{}) ||
			node.Hash != ([32]byte{}) || node.Left == nil || node.Right == nil {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		left, err := verifyIntervalProofNodeV1(node.Left, query, seen, records)
		if err != nil {
			return verifiedIntervalNodeV1{}, err
		}
		right, err := verifyIntervalProofNodeV1(node.Right, query, seen, records)
		if err != nil {
			return verifiedIntervalNodeV1{}, err
		}
		if left.count > math.MaxUint64-right.count {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		count := left.count + right.count
		if count < 2 || left.count != count/2 || right.count != count-count/2 {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		summary := combineIntervalSummaryV1(left.summary, right.summary)
		return verifiedIntervalNodeV1{
			count: count, summary: summary,
			hash: hashIntervalBranchV1(count, summary, left.hash, right.hash),
		}, nil
	case IntervalProofLeafV1:
		if node.Record == nil || node.Left != nil || node.Right != nil || node.Count != 1 {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		if err := validateIntervalRecordV1(*node.Record); err != nil || !intervalMatchesV1(*node.Record, query) {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		if _, exists := seen[node.Record.AssertionRef]; exists {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		seen[node.Record.AssertionRef] = struct{}{}
		*records = append(*records, *node.Record)
		summary := IntervalSummaryV1{MinValidFrom: node.Record.ValidFrom, MaxValidTo: node.Record.ValidTo}
		hash := hashIntervalLeafV1(*node.Record)
		if node.Summary != summary || node.Hash != hash {
			return verifiedIntervalNodeV1{}, ErrIntervalMalformed
		}
		return verifiedIntervalNodeV1{count: 1, summary: summary, hash: hash}, nil
	default:
		return verifiedIntervalNodeV1{}, ErrIntervalMalformed
	}
}

func intervalMatchesV1(record IntervalRecordV1, query IntervalInstantV1) bool {
	if compareInstantV1(record.ValidFrom, query) > 0 {
		return false
	}
	return record.ValidTo.Open || compareInstantV1(query, record.ValidTo.At) < 0
}

func intervalCanPruneV1(summary IntervalSummaryV1, query IntervalInstantV1) bool {
	if compareInstantV1(summary.MinValidFrom, query) > 0 {
		return true
	}
	return !summary.MaxValidTo.Open && compareInstantV1(summary.MaxValidTo.At, query) <= 0
}

func combineIntervalSummaryV1(left, right IntervalSummaryV1) IntervalSummaryV1 {
	minStart := left.MinValidFrom
	if compareInstantV1(right.MinValidFrom, minStart) < 0 {
		minStart = right.MinValidFrom
	}
	maxEnd := left.MaxValidTo
	if right.MaxValidTo.Open {
		maxEnd = right.MaxValidTo
	} else if !maxEnd.Open && compareInstantV1(right.MaxValidTo.At, maxEnd.At) > 0 {
		maxEnd = right.MaxValidTo
	}
	return IntervalSummaryV1{MinValidFrom: minStart, MaxValidTo: maxEnd}
}

func validIntervalSummaryV1(summary IntervalSummaryV1) bool {
	if validateInstantV1(summary.MinValidFrom) != nil {
		return false
	}
	if summary.MaxValidTo.Open {
		return summary.MaxValidTo.At == (IntervalInstantV1{})
	}
	return validateInstantV1(summary.MaxValidTo.At) == nil &&
		compareInstantV1(summary.MaxValidTo.At, summary.MinValidFrom) > 0
}

func validateIntervalScopeV1(scope IntervalScopeV1) error {
	for _, value := range []string{
		scope.VerifierRef, scope.PolicyEpochRef, scope.NamespaceRef, scope.SubjectRef, scope.ClaimID,
	} {
		if err := validateIntervalStringV1(value); err != nil {
			return err
		}
	}
	return validateIntervalDigestV1(scope.SourceFrontierDigest)
}

func validateIntervalRecordV1(record IntervalRecordV1) error {
	if err := validateIntervalStringV1(record.AssertionRef); err != nil {
		return err
	}
	if err := validateInstantV1(record.ValidFrom); err != nil {
		return err
	}
	if record.ValidTo.Open {
		if record.ValidTo.At != (IntervalInstantV1{}) {
			return ErrIntervalMalformed
		}
	} else if validateInstantV1(record.ValidTo.At) != nil ||
		compareInstantV1(record.ValidTo.At, record.ValidFrom) <= 0 {
		return ErrIntervalMalformed
	}
	return validateIntervalDigestV1(record.ReceiptDigest)
}

func validateInstantV1(value IntervalInstantV1) error {
	if value.Nanos >= 1_000_000_000 {
		return ErrIntervalMalformed
	}
	return nil
}

func validateIntervalStringV1(value string) error {
	if value == "" || strings.TrimSpace(value) == "" || len(value) > 1_024 || !utf8.ValidString(value) {
		return ErrIntervalMalformed
	}
	return nil
}

func validateIntervalDigestV1(value string) error {
	if len(value) != len("sha256:")+64 || !strings.HasPrefix(value, "sha256:") {
		return ErrIntervalMalformed
	}
	for _, ch := range value[len("sha256:"):] {
		if !((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')) {
			return ErrIntervalMalformed
		}
	}
	return nil
}

func compareInstantV1(a, b IntervalInstantV1) int {
	if a.Seconds < b.Seconds {
		return -1
	}
	if a.Seconds > b.Seconds {
		return 1
	}
	if a.Nanos < b.Nanos {
		return -1
	}
	if a.Nanos > b.Nanos {
		return 1
	}
	return 0
}

func appendIntervalStringV1(dst []byte, value string) []byte {
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	dst = append(dst, length[:]...)
	dst = append(dst, value...)
	return dst
}

func appendIntervalInstantV1(dst []byte, value IntervalInstantV1) []byte {
	var instant [12]byte
	binary.BigEndian.PutUint64(instant[:8], uint64(value.Seconds))
	binary.BigEndian.PutUint32(instant[8:], value.Nanos)
	return append(dst, instant[:]...)
}

func appendIntervalEndV1(dst []byte, value IntervalEndV1) []byte {
	if value.Open {
		return append(dst, 1)
	}
	dst = append(dst, 0)
	return appendIntervalInstantV1(dst, value.At)
}

func appendIntervalScopeV1(dst []byte, scope IntervalScopeV1) []byte {
	dst = appendIntervalStringV1(dst, scope.VerifierRef)
	dst = appendIntervalStringV1(dst, scope.PolicyEpochRef)
	dst = appendIntervalStringV1(dst, scope.NamespaceRef)
	dst = appendIntervalStringV1(dst, scope.SubjectRef)
	dst = appendIntervalStringV1(dst, scope.ClaimID)
	dst = appendIntervalStringV1(dst, scope.SourceFrontierDigest)
	return dst
}

func appendIntervalRecordV1(dst []byte, record IntervalRecordV1) []byte {
	dst = appendIntervalStringV1(dst, record.AssertionRef)
	dst = appendIntervalInstantV1(dst, record.ValidFrom)
	dst = appendIntervalEndV1(dst, record.ValidTo)
	dst = appendIntervalStringV1(dst, record.ReceiptDigest)
	return dst
}

func hashIntervalEmptyV1() [32]byte {
	var preimage [1 + len(intervalQueryDomainV1)]byte
	preimage[0] = 0x32
	copy(preimage[1:], intervalQueryDomainV1)
	return sha256.Sum256(preimage[:])
}

func hashIntervalLeafV1(record IntervalRecordV1) [32]byte {
	preimage := make([]byte, 0, 1+len(intervalQueryDomainV1)+4+len(record.AssertionRef)+12+1+12+4+len(record.ReceiptDigest))
	preimage = append(preimage, 0x30)
	preimage = append(preimage, intervalQueryDomainV1...)
	preimage = appendIntervalRecordV1(preimage, record)
	return sha256.Sum256(preimage)
}

func hashIntervalBranchV1(count uint64, summary IntervalSummaryV1, left, right [32]byte) [32]byte {
	const summarySize = 12 + 1 + 12
	var preimage [1 + len(intervalQueryDomainV1) + 8 + summarySize + 64]byte
	i := 0
	preimage[i] = 0x31
	i++
	i += copy(preimage[i:], intervalQueryDomainV1)
	binary.BigEndian.PutUint64(preimage[i:i+8], count)
	i += 8
	binary.BigEndian.PutUint64(preimage[i:i+8], uint64(summary.MinValidFrom.Seconds))
	i += 8
	binary.BigEndian.PutUint32(preimage[i:i+4], summary.MinValidFrom.Nanos)
	i += 4
	if summary.MaxValidTo.Open {
		preimage[i] = 1
		i++
	} else {
		preimage[i] = 0
		i++
		binary.BigEndian.PutUint64(preimage[i:i+8], uint64(summary.MaxValidTo.At.Seconds))
		i += 8
		binary.BigEndian.PutUint32(preimage[i:i+4], summary.MaxValidTo.At.Nanos)
		i += 4
	}
	i += copy(preimage[i:], left[:])
	i += copy(preimage[i:], right[:])
	return sha256.Sum256(preimage[:i])
}

func hashIntervalScopedRootV1(scope IntervalScopeV1, count uint64, treeHash [32]byte) [32]byte {
	capacity := 1 + len(intervalQueryDomainV1) + 8 + 32
	for _, value := range []string{scope.VerifierRef, scope.PolicyEpochRef, scope.NamespaceRef, scope.SubjectRef, scope.ClaimID, scope.SourceFrontierDigest} {
		capacity += 4 + len(value)
	}
	preimage := make([]byte, 0, capacity)
	preimage = append(preimage, 0x33)
	preimage = append(preimage, intervalQueryDomainV1...)
	preimage = appendIntervalScopeV1(preimage, scope)
	var countBytes [8]byte
	binary.BigEndian.PutUint64(countBytes[:], count)
	preimage = append(preimage, countBytes[:]...)
	preimage = append(preimage, treeHash[:]...)
	return sha256.Sum256(preimage)
}
