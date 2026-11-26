package conformance

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"errors"
	"sort"
	"strings"
	"unicode/utf8"
)

const SparseMerkleDepthV1 = 256

var (
	sparseDomain      = []byte("MYCELIX-EXPERIMENTAL-SMT-SHA256-V1\x00")
	sparseKeyDomain   = []byte("MYCELIX-ASSERTION-INDEX-KEY-V1\x00")
	sparseValueDomain = []byte("MYCELIX-ASSERTION-INDEX-VALUE-V1\x00")
)

type SparseKey [32]byte
type SparseValue [32]byte

type SparseProof struct {
	// Siblings in leaf-to-root order (bit 255 toward bit 0).
	SiblingsLeafToRoot []SparseValue
}

type SparseEntry struct {
	Key   SparseKey
	Value SparseValue
}

type SparseTree struct {
	entries map[SparseKey]SparseValue
}

func NewSparseTree() *SparseTree {
	return &SparseTree{entries: make(map[SparseKey]SparseValue)}
}

func (tree *SparseTree) Insert(key SparseKey, value SparseValue) error {
	if _, exists := tree.entries[key]; exists {
		return errors.New("duplicate sparse-map key")
	}
	tree.entries[key] = value
	return nil
}

func (tree *SparseTree) Replace(key SparseKey, value SparseValue) (SparseValue, bool) {
	previous, exists := tree.entries[key]
	if !exists {
		return SparseValue{}, false
	}
	tree.entries[key] = value
	return previous, true
}

func (tree *SparseTree) Len() int {
	return len(tree.entries)
}

func (tree *SparseTree) entriesSorted() []SparseEntry {
	entries := make([]SparseEntry, 0, len(tree.entries))
	for key, value := range tree.entries {
		entries = append(entries, SparseEntry{Key: key, Value: value})
	}
	sort.Slice(entries, func(i, j int) bool {
		return bytes.Compare(entries[i].Key[:], entries[j].Key[:]) < 0
	})
	return entries
}

func SparseEmptyLeafHash() SparseValue {
	buf := make([]byte, 1+len(sparseDomain))
	buf[0] = 0x12
	copy(buf[1:], sparseDomain)
	return sha256.Sum256(buf)
}

func SparsePresentLeafHash(key SparseKey, value SparseValue) SparseValue {
	buf := make([]byte, 1+len(sparseDomain)+64)
	buf[0] = 0x10
	copy(buf[1:], sparseDomain)
	copy(buf[1+len(sparseDomain):], key[:])
	copy(buf[1+len(sparseDomain)+32:], value[:])
	return sha256.Sum256(buf)
}

func SparseBranchHash(left, right SparseValue) SparseValue {
	var buf [65]byte
	buf[0] = 0x11
	copy(buf[1:33], left[:])
	copy(buf[33:], right[:])
	return sha256.Sum256(buf[:])
}

func sparseDefaultHashes() [SparseMerkleDepthV1 + 1]SparseValue {
	var defaults [SparseMerkleDepthV1 + 1]SparseValue
	defaults[SparseMerkleDepthV1] = SparseEmptyLeafHash()
	for depth := SparseMerkleDepthV1 - 1; depth >= 0; depth-- {
		defaults[depth] = SparseBranchHash(defaults[depth+1], defaults[depth+1])
	}
	return defaults
}

func sparseKeyBit(key SparseKey, depth int) bool {
	return ((key[depth/8] >> (7 - (depth % 8))) & 1) == 1
}

func sparseSplit(entries []SparseEntry, depth int) int {
	return sort.Search(len(entries), func(i int) bool {
		return sparseKeyBit(entries[i].Key, depth)
	})
}

func sparseSubtreeHash(entries []SparseEntry, depth int, defaults *[SparseMerkleDepthV1 + 1]SparseValue) SparseValue {
	if len(entries) == 0 {
		return defaults[depth]
	}
	if depth == SparseMerkleDepthV1 {
		return SparsePresentLeafHash(entries[0].Key, entries[0].Value)
	}
	split := sparseSplit(entries, depth)
	left := sparseSubtreeHash(entries[:split], depth+1, defaults)
	right := sparseSubtreeHash(entries[split:], depth+1, defaults)
	return SparseBranchHash(left, right)
}

func (tree *SparseTree) RootHash() SparseValue {
	defaults := sparseDefaultHashes()
	return sparseSubtreeHash(tree.entriesSorted(), 0, &defaults)
}

func sparseBuildProof(entries []SparseEntry, key SparseKey, depth int, defaults *[SparseMerkleDepthV1 + 1]SparseValue, output *[]SparseValue) {
	if depth == SparseMerkleDepthV1 {
		return
	}
	split := sparseSplit(entries, depth)
	if sparseKeyBit(key, depth) {
		sibling := sparseSubtreeHash(entries[:split], depth+1, defaults)
		sparseBuildProof(entries[split:], key, depth+1, defaults, output)
		*output = append(*output, sibling)
	} else {
		sibling := sparseSubtreeHash(entries[split:], depth+1, defaults)
		sparseBuildProof(entries[:split], key, depth+1, defaults, output)
		*output = append(*output, sibling)
	}
}

func (tree *SparseTree) Prove(key SparseKey) SparseProof {
	defaults := sparseDefaultHashes()
	proof := SparseProof{SiblingsLeafToRoot: make([]SparseValue, 0, SparseMerkleDepthV1)}
	sparseBuildProof(tree.entriesSorted(), key, 0, &defaults, &proof.SiblingsLeafToRoot)
	return proof
}

func verifySparsePath(key SparseKey, current SparseValue, proof SparseProof, root SparseValue) error {
	if len(proof.SiblingsLeafToRoot) != SparseMerkleDepthV1 {
		return errors.New("invalid sparse proof length")
	}
	for offset, sibling := range proof.SiblingsLeafToRoot {
		depth := SparseMerkleDepthV1 - 1 - offset
		if sparseKeyBit(key, depth) {
			current = SparseBranchHash(sibling, current)
		} else {
			current = SparseBranchHash(current, sibling)
		}
	}
	if current != root {
		return errors.New("sparse root mismatch")
	}
	return nil
}

func VerifySparseMembership(key SparseKey, value SparseValue, proof SparseProof, root SparseValue) error {
	return verifySparsePath(key, SparsePresentLeafHash(key, value), proof, root)
}

func VerifySparseNonMembership(key SparseKey, proof SparseProof, root SparseValue) error {
	return verifySparsePath(key, SparseEmptyLeafHash(), proof, root)
}

func AppendCanonicalString(dst *bytes.Buffer, value string) error {
	if value == "" {
		return errors.New("empty identifier")
	}
	if !utf8.ValidString(value) {
		return errors.New("invalid UTF-8")
	}
	if uint64(len(value)) > uint64(^uint32(0)) {
		return errors.New("string too long")
	}
	_ = binary.Write(dst, binary.BigEndian, uint32(len(value)))
	_, _ = dst.WriteString(value)
	return nil
}

func appendSparseString(dst *bytes.Buffer, value string) error {
	if value == "" {
		return errors.New("empty sparse-index identifier")
	}
	if !utf8.ValidString(value) {
		return errors.New("invalid UTF-8")
	}
	if len(value) > 1_024 {
		return errors.New("sparse-index identifier exceeds 1024 bytes")
	}
	_ = binary.Write(dst, binary.BigEndian, uint32(len(value)))
	_, _ = dst.WriteString(value)
	return nil
}

func AssertionIndexKeyV1(verifier, policy, subject, claim, dimension, assertionRef string) (SparseKey, error) {
	var buf bytes.Buffer
	_, _ = buf.Write(sparseKeyDomain)
	for _, part := range []string{verifier, policy, subject, claim, dimension, assertionRef} {
		if err := appendSparseString(&buf, part); err != nil {
			return SparseKey{}, err
		}
	}
	return SparseKey(sha256.Sum256(buf.Bytes())), nil
}

func AssertionIndexValueV1(receiptDigest, owner string, sequence uint64) (SparseValue, error) {
	if !strings.HasPrefix(receiptDigest, "sha256:") || len(receiptDigest) != len("sha256:")+64 {
		return SparseValue{}, errors.New("invalid receipt digest")
	}
	for _, ch := range receiptDigest[len("sha256:"):] {
		if !((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')) {
			return SparseValue{}, errors.New("invalid receipt digest")
		}
	}
	if sequence == 0 {
		return SparseValue{}, errors.New("invalid owner sequence")
	}
	var buf bytes.Buffer
	_, _ = buf.Write(sparseValueDomain)
	for _, part := range []string{receiptDigest, owner} {
		if err := appendSparseString(&buf, part); err != nil {
			return SparseValue{}, err
		}
	}
	_ = binary.Write(&buf, binary.BigEndian, sequence)
	return SparseValue(sha256.Sum256(buf.Bytes())), nil
}
