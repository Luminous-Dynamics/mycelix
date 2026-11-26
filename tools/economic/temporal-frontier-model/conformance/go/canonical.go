package conformance

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sort"
	"unicode/utf8"
)

type SequenceRange struct {
	First uint64
	Last  uint64
}

type OwnerPosition struct {
	OwnerRef string
	Sequence uint64
}

type OwnerCut struct {
	VerifierRef    string
	PolicyEpochRef string
	OwnerRef       string
	HistoryRef     string
	Sequence       uint64
	TreeSize       uint64
	RootDigest     string
	Gaps           []SequenceRange
}

type KnowledgeReceipt struct {
	ReceiptID        string
	VerifierRef      string
	PolicyEpochRef   string
	AssertionID      string
	AssertionDigest  string
	OwnerRef         string
	OwnerSequence    uint64
	CausalDependency []OwnerPosition
}

func appendString(dst *bytes.Buffer, value string) error {
	if value == "" {
		return errors.New("empty identifier")
	}
	if uint64(len(value)) > uint64(^uint32(0)) {
		return errors.New("string exceeds u32 length prefix")
	}
	if !utf8.ValidString(value) {
		return errors.New("invalid UTF-8")
	}
	_ = binary.Write(dst, binary.BigEndian, uint32(len(value)))
	_, _ = dst.WriteString(value)
	return nil
}

func validateDigest(value string) error {
	if len(value) != len("sha256:")+64 || value[:7] != "sha256:" {
		return errors.New("invalid sha256 digest shape")
	}
	decoded, err := hex.DecodeString(value[7:])
	if err != nil || len(decoded) != sha256.Size {
		return errors.New("invalid sha256 digest")
	}
	if value[7:] != string(bytes.ToLower([]byte(value[7:]))) {
		return errors.New("digest must use lower-case hex")
	}
	return nil
}

func CanonicalOwnerCutEntryV1(cut OwnerCut) ([]byte, error) {
	for _, value := range []string{
		cut.VerifierRef, cut.PolicyEpochRef, cut.OwnerRef, cut.HistoryRef,
	} {
		if value == "" {
			return nil, errors.New("empty owner-cut identifier")
		}
	}
	if err := validateDigest(cut.RootDigest); err != nil {
		return nil, err
	}

	var previous uint64
	missing := uint64(0)
	for i, gap := range cut.Gaps {
		if gap.First == 0 || gap.First > gap.Last || gap.Last > cut.Sequence {
			return nil, errors.New("invalid gap range")
		}
		if i > 0 && (gap.First <= previous || (previous < ^uint64(0) && gap.First == previous+1)) {
			return nil, errors.New("gaps overlap, are adjacent, or are not canonical")
		}
		width := gap.Last - gap.First + 1
		if ^uint64(0)-missing < width {
			return nil, errors.New("gap count overflow")
		}
		missing += width
		previous = gap.Last
	}
	if ^uint64(0)-missing < cut.TreeSize || cut.TreeSize+missing != cut.Sequence {
		return nil, errors.New("tree size plus gaps must equal owner sequence")
	}
	if cut.TreeSize == 0 && SHA256Digest(nil) != cut.RootDigest {
		return nil, errors.New("empty owner log must use the empty-tree hash")
	}

	var out bytes.Buffer
	_, _ = out.Write([]byte("MYCELIX-OWNER-CUT-V1\x00"))
	for _, value := range []string{
		cut.VerifierRef, cut.PolicyEpochRef, cut.OwnerRef, cut.HistoryRef,
	} {
		if err := appendString(&out, value); err != nil {
			return nil, err
		}
	}
	_ = binary.Write(&out, binary.BigEndian, cut.Sequence)
	_ = binary.Write(&out, binary.BigEndian, cut.TreeSize)
	if err := appendString(&out, cut.RootDigest); err != nil {
		return nil, err
	}
	_ = binary.Write(&out, binary.BigEndian, uint32(len(cut.Gaps)))
	for _, gap := range cut.Gaps {
		_ = binary.Write(&out, binary.BigEndian, gap.First)
		_ = binary.Write(&out, binary.BigEndian, gap.Last)
	}
	return out.Bytes(), nil
}

func CanonicalKnowledgeReceiptEntryV1(receipt KnowledgeReceipt) ([]byte, error) {
	for _, value := range []string{
		receipt.ReceiptID,
		receipt.VerifierRef,
		receipt.PolicyEpochRef,
		receipt.AssertionID,
		receipt.OwnerRef,
	} {
		if value == "" {
			return nil, errors.New("empty receipt identifier")
		}
	}
	if err := validateDigest(receipt.AssertionDigest); err != nil {
		return nil, err
	}
	if receipt.OwnerSequence == 0 {
		return nil, errors.New("owner sequence is zero")
	}

	deps := append([]OwnerPosition(nil), receipt.CausalDependency...)
	for _, dep := range deps {
		if dep.OwnerRef == "" || dep.Sequence == 0 {
			return nil, errors.New("invalid causal dependency")
		}
		if dep.OwnerRef == receipt.OwnerRef && dep.Sequence >= receipt.OwnerSequence {
			return nil, errors.New("same-owner dependency is not a predecessor")
		}
	}
	sort.Slice(deps, func(i, j int) bool {
		if deps[i].OwnerRef != deps[j].OwnerRef {
			return deps[i].OwnerRef < deps[j].OwnerRef
		}
		return deps[i].Sequence < deps[j].Sequence
	})
	for i := 1; i < len(deps); i++ {
		if deps[i] == deps[i-1] {
			return nil, errors.New("duplicate causal dependency")
		}
	}

	var out bytes.Buffer
	_, _ = out.Write([]byte("MYCELIX-KNOWLEDGE-RECEIPT-V1\x00"))
	for _, value := range []string{
		receipt.ReceiptID,
		receipt.VerifierRef,
		receipt.PolicyEpochRef,
		receipt.AssertionID,
		receipt.AssertionDigest,
		receipt.OwnerRef,
	} {
		if err := appendString(&out, value); err != nil {
			return nil, err
		}
	}
	_ = binary.Write(&out, binary.BigEndian, receipt.OwnerSequence)
	_ = binary.Write(&out, binary.BigEndian, uint32(len(deps)))
	for _, dep := range deps {
		if err := appendString(&out, dep.OwnerRef); err != nil {
			return nil, err
		}
		_ = binary.Write(&out, binary.BigEndian, dep.Sequence)
	}
	return out.Bytes(), nil
}

func SHA256Digest(value []byte) string {
	sum := sha256.Sum256(value)
	return "sha256:" + hex.EncodeToString(sum[:])
}

func VerifyGoldenDigest(value []byte, expected string) error {
	actual := SHA256Digest(value)
	if actual != expected {
		return fmt.Errorf("digest mismatch: got %s want %s", actual, expected)
	}
	return nil
}
