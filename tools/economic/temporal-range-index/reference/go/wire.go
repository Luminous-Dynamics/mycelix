package temporalrange

import (
	"bytes"
	"encoding/binary"
	"errors"
	"io"
	"unicode/utf8"
)

const (
	intervalProofWireMagicV1 = "MYCITPQ1"
	maxIntervalProofWireBytesV1 = 64 * 1024 * 1024
	maxIntervalProofWireNodesV1 = 2_000_001
	maxIntervalProofWireDepthV1 = 64
	maxIntervalProofEntriesV1 = 1_000_000
)

var ErrIntervalProofEncoding = errors.New("invalid canonical interval proof encoding")

// MarshalIntervalPointProofV1 encodes a semantically valid interval point proof using the
// canonical bounded V1 wire format. The trusted interval root remains external to the proof.
func MarshalIntervalPointProofV1(proof IntervalPointProofV1) ([]byte, error) {
	if validateIntervalScopeV1(proof.Scope) != nil ||
		validateInstantV1(proof.Query) != nil ||
		proof.Count > maxIntervalProofEntriesV1 {
		return nil, ErrIntervalProofEncoding
	}
	if proof.Count == 0 {
		if proof.Tree != nil {
			return nil, ErrIntervalProofEncoding
		}
	} else {
		if proof.Tree == nil {
			return nil, ErrIntervalProofEncoding
		}
		seen := make(map[string]struct{})
		var records []IntervalRecordV1
		verified, err := verifyIntervalProofNodeV1(proof.Tree, proof.Query, seen, &records)
		if err != nil || verified.count != proof.Count {
			return nil, ErrIntervalProofEncoding
		}
	}

	var out bytes.Buffer
	out.Grow(128 + len(proof.Scope.VerifierRef) + len(proof.Scope.PolicyEpochRef) +
		len(proof.Scope.NamespaceRef) + len(proof.Scope.SubjectRef) +
		len(proof.Scope.ClaimID) + len(proof.Scope.SourceFrontierDigest))
	out.WriteString(intervalProofWireMagicV1)
	writeWireStringV1(&out, proof.Scope.VerifierRef)
	writeWireStringV1(&out, proof.Scope.PolicyEpochRef)
	writeWireStringV1(&out, proof.Scope.NamespaceRef)
	writeWireStringV1(&out, proof.Scope.SubjectRef)
	writeWireStringV1(&out, proof.Scope.ClaimID)
	writeWireStringV1(&out, proof.Scope.SourceFrontierDigest)
	writeWireInstantV1(&out, proof.Query)
	_ = binary.Write(&out, binary.BigEndian, proof.Count)
	if proof.Count > 0 {
		if err := writeWireNodeV1(&out, proof.Tree, 0); err != nil {
			return nil, err
		}
	}
	if out.Len() > maxIntervalProofWireBytesV1 {
		return nil, ErrIntervalProofEncoding
	}
	return out.Bytes(), nil
}

// UnmarshalIntervalPointProofV1 decodes and structurally validates a canonical V1 proof.
// It does not authenticate the external root; call VerifyIntervalPointProofV1 afterwards.
func UnmarshalIntervalPointProofV1(data []byte) (IntervalPointProofV1, error) {
	if len(data) < len(intervalProofWireMagicV1) || len(data) > maxIntervalProofWireBytesV1 {
		return IntervalPointProofV1{}, ErrIntervalProofEncoding
	}
	reader := bytes.NewReader(data)
	magic := make([]byte, len(intervalProofWireMagicV1))
	if _, err := io.ReadFull(reader, magic); err != nil || string(magic) != intervalProofWireMagicV1 {
		return IntervalPointProofV1{}, ErrIntervalProofEncoding
	}

	var proof IntervalPointProofV1
	var err error
	fields := []*string{
		&proof.Scope.VerifierRef,
		&proof.Scope.PolicyEpochRef,
		&proof.Scope.NamespaceRef,
		&proof.Scope.SubjectRef,
		&proof.Scope.ClaimID,
		&proof.Scope.SourceFrontierDigest,
	}
	for _, field := range fields {
		*field, err = readWireStringV1(reader)
		if err != nil {
			return IntervalPointProofV1{}, err
		}
	}
	proof.Query, err = readWireInstantV1(reader)
	if err != nil {
		return IntervalPointProofV1{}, err
	}
	if err := binary.Read(reader, binary.BigEndian, &proof.Count); err != nil ||
		proof.Count > maxIntervalProofEntriesV1 {
		return IntervalPointProofV1{}, ErrIntervalProofEncoding
	}
	if proof.Count > 0 {
		nodes := 0
		proof.Tree, err = readWireNodeV1(reader, 0, &nodes, proof.Query)
		if err != nil {
			return IntervalPointProofV1{}, err
		}
	}
	if reader.Len() != 0 {
		return IntervalPointProofV1{}, ErrIntervalProofEncoding
	}
	if validateIntervalScopeV1(proof.Scope) != nil || validateInstantV1(proof.Query) != nil {
		return IntervalPointProofV1{}, ErrIntervalProofEncoding
	}
	if proof.Count == 0 {
		if proof.Tree != nil {
			return IntervalPointProofV1{}, ErrIntervalProofEncoding
		}
		return proof, nil
	}
	seen := make(map[string]struct{})
	var records []IntervalRecordV1
	verified, err := verifyIntervalProofNodeV1(proof.Tree, proof.Query, seen, &records)
	if err != nil || verified.count != proof.Count {
		return IntervalPointProofV1{}, ErrIntervalProofEncoding
	}
	return proof, nil
}

func writeWireNodeV1(out *bytes.Buffer, node *IntervalProofNodeV1, depth int) error {
	if node == nil || depth > maxIntervalProofWireDepthV1 {
		return ErrIntervalProofEncoding
	}
	out.WriteByte(node.Kind)
	switch node.Kind {
	case IntervalProofPrunedV1:
		if node.Count == 0 || node.Record != nil || node.Left != nil || node.Right != nil ||
			!validIntervalSummaryV1(node.Summary) {
			return ErrIntervalProofEncoding
		}
		_ = binary.Write(out, binary.BigEndian, node.Count)
		writeWireSummaryV1(out, node.Summary)
		out.Write(node.Hash[:])
	case IntervalProofBranchV1:
		if node.Record != nil || node.Count != 0 || node.Summary != (IntervalSummaryV1{}) ||
			node.Hash != ([32]byte{}) || node.Left == nil || node.Right == nil {
			return ErrIntervalProofEncoding
		}
		if err := writeWireNodeV1(out, node.Left, depth+1); err != nil {
			return err
		}
		if err := writeWireNodeV1(out, node.Right, depth+1); err != nil {
			return err
		}
	case IntervalProofLeafV1:
		if node.Record == nil || node.Count != 1 || node.Left != nil || node.Right != nil {
			return ErrIntervalProofEncoding
		}
		writeWireRecordV1(out, *node.Record)
	default:
		return ErrIntervalProofEncoding
	}
	if out.Len() > maxIntervalProofWireBytesV1 {
		return ErrIntervalProofEncoding
	}
	return nil
}

func readWireNodeV1(reader *bytes.Reader, depth int, nodes *int, query IntervalInstantV1) (*IntervalProofNodeV1, error) {
	if depth > maxIntervalProofWireDepthV1 || *nodes >= maxIntervalProofWireNodesV1 {
		return nil, ErrIntervalProofEncoding
	}
	*nodes++
	kind, err := reader.ReadByte()
	if err != nil {
		return nil, ErrIntervalProofEncoding
	}
	switch kind {
	case IntervalProofPrunedV1:
		node := &IntervalProofNodeV1{Kind: kind}
		if binary.Read(reader, binary.BigEndian, &node.Count) != nil || node.Count == 0 {
			return nil, ErrIntervalProofEncoding
		}
		node.Summary, err = readWireSummaryV1(reader)
		if err != nil || !validIntervalSummaryV1(node.Summary) || !intervalCanPruneV1(node.Summary, query) {
			return nil, ErrIntervalProofEncoding
		}
		if _, err := io.ReadFull(reader, node.Hash[:]); err != nil {
			return nil, ErrIntervalProofEncoding
		}
		return node, nil
	case IntervalProofBranchV1:
		left, err := readWireNodeV1(reader, depth+1, nodes, query)
		if err != nil {
			return nil, err
		}
		right, err := readWireNodeV1(reader, depth+1, nodes, query)
		if err != nil {
			return nil, err
		}
		return &IntervalProofNodeV1{Kind: kind, Left: left, Right: right}, nil
	case IntervalProofLeafV1:
		record, err := readWireRecordV1(reader)
		if err != nil || validateIntervalRecordV1(record) != nil || !intervalMatchesV1(record, query) {
			return nil, ErrIntervalProofEncoding
		}
		summary := IntervalSummaryV1{MinValidFrom: record.ValidFrom, MaxValidTo: record.ValidTo}
		return &IntervalProofNodeV1{
			Kind: kind, Count: 1, Summary: summary, Hash: hashIntervalLeafV1(record), Record: &record,
		}, nil
	default:
		return nil, ErrIntervalProofEncoding
	}
}

func writeWireStringV1(out *bytes.Buffer, value string) {
	_ = binary.Write(out, binary.BigEndian, uint32(len(value)))
	_, _ = out.WriteString(value)
}

func readWireStringV1(reader *bytes.Reader) (string, error) {
	var length uint32
	if binary.Read(reader, binary.BigEndian, &length) != nil ||
		length == 0 || length > 1_024 || uint64(length) > uint64(reader.Len()) {
		return "", ErrIntervalProofEncoding
	}
	value := make([]byte, int(length))
	if _, err := io.ReadFull(reader, value); err != nil || !utf8.Valid(value) {
		return "", ErrIntervalProofEncoding
	}
	return string(value), nil
}

func writeWireInstantV1(out *bytes.Buffer, instant IntervalInstantV1) {
	_ = binary.Write(out, binary.BigEndian, instant.Seconds)
	_ = binary.Write(out, binary.BigEndian, instant.Nanos)
}

func readWireInstantV1(reader *bytes.Reader) (IntervalInstantV1, error) {
	var instant IntervalInstantV1
	if binary.Read(reader, binary.BigEndian, &instant.Seconds) != nil ||
		binary.Read(reader, binary.BigEndian, &instant.Nanos) != nil ||
		validateInstantV1(instant) != nil {
		return IntervalInstantV1{}, ErrIntervalProofEncoding
	}
	return instant, nil
}

func writeWireEndV1(out *bytes.Buffer, end IntervalEndV1) {
	if end.Open {
		out.WriteByte(1)
		return
	}
	out.WriteByte(0)
	writeWireInstantV1(out, end.At)
}

func readWireEndV1(reader *bytes.Reader) (IntervalEndV1, error) {
	tag, err := reader.ReadByte()
	if err != nil {
		return IntervalEndV1{}, ErrIntervalProofEncoding
	}
	switch tag {
	case 0:
		instant, err := readWireInstantV1(reader)
		if err != nil {
			return IntervalEndV1{}, err
		}
		return IntervalEndV1{At: instant}, nil
	case 1:
		return IntervalEndV1{Open: true}, nil
	default:
		return IntervalEndV1{}, ErrIntervalProofEncoding
	}
}

func writeWireSummaryV1(out *bytes.Buffer, summary IntervalSummaryV1) {
	writeWireInstantV1(out, summary.MinValidFrom)
	writeWireEndV1(out, summary.MaxValidTo)
}

func readWireSummaryV1(reader *bytes.Reader) (IntervalSummaryV1, error) {
	minimum, err := readWireInstantV1(reader)
	if err != nil {
		return IntervalSummaryV1{}, err
	}
	maximum, err := readWireEndV1(reader)
	if err != nil {
		return IntervalSummaryV1{}, err
	}
	return IntervalSummaryV1{MinValidFrom: minimum, MaxValidTo: maximum}, nil
}

func writeWireRecordV1(out *bytes.Buffer, record IntervalRecordV1) {
	writeWireStringV1(out, record.AssertionRef)
	writeWireInstantV1(out, record.ValidFrom)
	writeWireEndV1(out, record.ValidTo)
	writeWireStringV1(out, record.ReceiptDigest)
}

func readWireRecordV1(reader *bytes.Reader) (IntervalRecordV1, error) {
	var record IntervalRecordV1
	var err error
	record.AssertionRef, err = readWireStringV1(reader)
	if err != nil {
		return IntervalRecordV1{}, err
	}
	record.ValidFrom, err = readWireInstantV1(reader)
	if err != nil {
		return IntervalRecordV1{}, err
	}
	record.ValidTo, err = readWireEndV1(reader)
	if err != nil {
		return IntervalRecordV1{}, err
	}
	record.ReceiptDigest, err = readWireStringV1(reader)
	if err != nil {
		return IntervalRecordV1{}, err
	}
	return record, nil
}
