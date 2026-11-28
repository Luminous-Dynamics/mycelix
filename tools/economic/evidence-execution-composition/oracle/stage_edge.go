package oracle

import (
	"bytes"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"strings"
	"unicode/utf8"
)

const (
	stageEdgeDomainV1 = "MYCELIX-DJCC-STAGE-EDGE-V1"
	maxDJCCIdentifierBytes = 4096
)

type StageEdgeV1 struct {
	ProducerStage   string
	ConsumerStage   string
	InputName       string
	OutputSchemaRef string
	OutputDigest    [32]byte
}

func validDJCCStageID(value string) bool {
	switch value {
	case "verifier_frontier", "source_trust", "semantic_projection", "query",
		"policy", "authorization", "execution", "outcome":
		return true
	default:
		return false
	}
}

func writeCanonicalStringV1(buffer *bytes.Buffer, value string) error {
	if !utf8.ValidString(value) {
		return errors.New("canonical DJCC string is not valid UTF-8")
	}
	if strings.TrimSpace(value) == "" {
		return errors.New("canonical DJCC identifier is empty")
	}
	if len([]byte(value)) > maxDJCCIdentifierBytes {
		return fmt.Errorf("canonical DJCC identifier exceeds %d UTF-8 bytes", maxDJCCIdentifierBytes)
	}
	var size [4]byte
	binary.BigEndian.PutUint32(size[:], uint32(len([]byte(value))))
	_, _ = buffer.Write(size[:])
	_, _ = buffer.Write([]byte(value))
	return nil
}

// CanonicalStageEdgeBytesV1 returns the byte-level domain-separated encoding
// shared with the Rust candidate. Strings are UTF-8 with big-endian u32 byte
// lengths; the 32-byte output digest is appended as raw bytes.
func CanonicalStageEdgeBytesV1(edge StageEdgeV1) ([]byte, error) {
	if !validDJCCStageID(edge.ProducerStage) || !validDJCCStageID(edge.ConsumerStage) {
		return nil, errors.New("stage edge contains an unknown producer or consumer stage")
	}
	var buffer bytes.Buffer
	if err := writeCanonicalStringV1(&buffer, stageEdgeDomainV1); err != nil {
		return nil, err
	}
	for _, value := range []string{
		edge.ProducerStage,
		edge.ConsumerStage,
		edge.InputName,
		edge.OutputSchemaRef,
	} {
		if err := writeCanonicalStringV1(&buffer, value); err != nil {
			return nil, err
		}
	}
	_, _ = buffer.Write(edge.OutputDigest[:])
	return buffer.Bytes(), nil
}

func parseStageEdgeVectorV1(fields []string) (StageEdgeV1, []byte, error) {
	if len(fields) != 7 {
		return StageEdgeV1{}, nil, fmt.Errorf("stage-edge vector needs 7 tab-separated fields, got %d", len(fields))
	}
	decoded, err := hex.DecodeString(fields[5])
	if err != nil {
		return StageEdgeV1{}, nil, fmt.Errorf("decode output digest: %w", err)
	}
	if len(decoded) != 32 {
		return StageEdgeV1{}, nil, fmt.Errorf("output digest must be 32 bytes, got %d", len(decoded))
	}
	var digest [32]byte
	copy(digest[:], decoded)
	expected, err := hex.DecodeString(fields[6])
	if err != nil {
		return StageEdgeV1{}, nil, fmt.Errorf("decode expected preimage: %w", err)
	}
	return StageEdgeV1{
		ProducerStage: fields[1], ConsumerStage: fields[2],
		InputName: fields[3], OutputSchemaRef: fields[4],
		OutputDigest: digest,
	}, expected, nil
}
