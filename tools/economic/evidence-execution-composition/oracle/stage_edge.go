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
	stageEdgeDomainV1      = "MYCELIX-DJCC-STAGE-EDGE-V1"
	maxDJCCIdentifierBytes = 4096
)

type StageEdgeV1 struct {
	ProducerStage   string
	ConsumerStage   string
	InputName       string
	OutputSchemaRef string
	RequiredClaims  []string
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

func validDJCCClaimDimension(value string) bool {
	switch value {
	case "authentication", "authority", "completeness", "effect", "independence", "truth":
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

// CanonicalStageEdgeBytesV1 encodes one producer-output to consumer-input edge.
// Strings are UTF-8 with big-endian u32 byte lengths. Required claim dimensions
// are encoded as a u32 count followed by their canonical strings. The 32-byte
// output digest is appended as raw bytes.
func CanonicalStageEdgeBytesV1(edge StageEdgeV1) ([]byte, error) {
	if !validDJCCStageID(edge.ProducerStage) || !validDJCCStageID(edge.ConsumerStage) {
		return nil, errors.New("stage edge contains an unknown producer or consumer stage")
	}
	if len(edge.RequiredClaims) == 0 {
		return nil, errors.New("stage edge must require at least one claim dimension")
	}
	claims := append([]string(nil), edge.RequiredClaims...)
	for i, claim := range claims {
		if !validDJCCClaimDimension(claim) {
			return nil, fmt.Errorf("unknown required claim dimension %q", claim)
		}
		if i > 0 && claims[i-1] >= claim {
			return nil, errors.New("required claim dimensions must be strictly sorted and unique")
		}
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
	var count [4]byte
	binary.BigEndian.PutUint32(count[:], uint32(len(claims)))
	_, _ = buffer.Write(count[:])
	for _, claim := range claims {
		if err := writeCanonicalStringV1(&buffer, claim); err != nil {
			return nil, err
		}
	}
	_, _ = buffer.Write(edge.OutputDigest[:])
	return buffer.Bytes(), nil
}

func parseStageEdgeVectorV1(fields []string) (StageEdgeV1, []byte, error) {
	if len(fields) != 8 {
		return StageEdgeV1{}, nil, fmt.Errorf("stage-edge vector needs 8 tab-separated fields, got %d", len(fields))
	}
	decoded, err := hex.DecodeString(fields[6])
	if err != nil {
		return StageEdgeV1{}, nil, fmt.Errorf("decode output digest: %w", err)
	}
	if len(decoded) != 32 {
		return StageEdgeV1{}, nil, fmt.Errorf("output digest must be 32 bytes, got %d", len(decoded))
	}
	var digest [32]byte
	copy(digest[:], decoded)
	expected, err := hex.DecodeString(fields[7])
	if err != nil {
		return StageEdgeV1{}, nil, fmt.Errorf("decode expected preimage: %w", err)
	}
	claims := strings.Split(fields[5], ",")
	if len(claims) == 1 && claims[0] == "" {
		return StageEdgeV1{}, nil, errors.New("required claim list is empty")
	}
	for i, claim := range claims {
		if !validDJCCClaimDimension(claim) {
			return StageEdgeV1{}, nil, fmt.Errorf("unknown required claim dimension %q", claim)
		}
		if i > 0 && claims[i-1] >= claim {
			return StageEdgeV1{}, nil, errors.New("required claim dimensions must be strictly sorted and unique")
		}
	}
	return StageEdgeV1{
		ProducerStage: fields[1], ConsumerStage: fields[2],
		InputName: fields[3], OutputSchemaRef: fields[4],
		RequiredClaims: claims, OutputDigest: digest,
	}, expected, nil
}
