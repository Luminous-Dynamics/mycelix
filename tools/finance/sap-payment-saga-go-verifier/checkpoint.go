package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"strings"
	"unicode/utf8"
)

const (
	checkpointSubjectDomain = "MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1\x00"
	maxCheckpointFieldBytes = 4096
)

type CheckpointOwner uint8

const (
	CheckpointPayments CheckpointOwner = 1
	CheckpointTreasury CheckpointOwner = 2
)

// CheckpointSubjectV1 is an owner-scoped attestation subject. Its digest
// identifies these bytes; it does not authenticate an issuer or source action.
type CheckpointSubjectV1 struct {
	OperationID           string
	IntentDigest          string
	Owner                 CheckpointOwner
	EventLogID            string
	EventCount            uint64
	TailDigest            string
	CheckpointSequence    uint64
	PriorCheckpointDigest string
	SourceActionHash      string
	SourceChainFrontier   string
	IssuerDID             string
	VerificationMethod    string
	AuthorityGrantRef     string
	IssuedAtUnixMS        uint64
}

var (
	ErrCheckpointInvalidSubject  = errors.New("checkpoint rejected: invalid subject")
	ErrCheckpointInvalidDigest   = errors.New("checkpoint rejected: invalid digest")
	ErrCheckpointInvalidSequence = errors.New("checkpoint rejected: invalid sequence")
	ErrCheckpointInvalidTimestamp = errors.New("checkpoint rejected: invalid timestamp")
)

func (s CheckpointSubjectV1) Validate() error {
	for _, value := range []string{
		s.OperationID,
		s.EventLogID,
		s.SourceActionHash,
		s.SourceChainFrontier,
		s.IssuerDID,
		s.VerificationMethod,
		s.AuthorityGrantRef,
	} {
		if value == "" || !utf8.ValidString(value) || len(value) > maxCheckpointFieldBytes {
			return ErrCheckpointInvalidSubject
		}
	}
	if !isCheckpointDigest(s.IntentDigest) || !isCheckpointDigest(s.TailDigest) {
		return ErrCheckpointInvalidDigest
	}
	if s.PriorCheckpointDigest != "" && !isCheckpointDigest(s.PriorCheckpointDigest) {
		return ErrCheckpointInvalidDigest
	}
	if s.Owner != CheckpointPayments && s.Owner != CheckpointTreasury {
		return ErrCheckpointInvalidSubject
	}
	if s.EventCount == 0 || s.CheckpointSequence == 0 {
		return ErrCheckpointInvalidSequence
	}
	if s.IssuedAtUnixMS == 0 {
		return ErrCheckpointInvalidTimestamp
	}
	if (s.CheckpointSequence == 1) != (s.PriorCheckpointDigest == "") {
		return ErrCheckpointInvalidSequence
	}
	return nil
}

func (s CheckpointSubjectV1) CanonicalBytes() ([]byte, error) {
	if err := s.Validate(); err != nil {
		return nil, err
	}
	encoded := make([]byte, 0, 512)
	encoded = append(encoded, []byte(checkpointSubjectDomain)...)
	var err error
	for _, value := range []string{s.OperationID, s.IntentDigest} {
		encoded, err = appendCheckpointText(encoded, value)
		if err != nil {
			return nil, err
		}
	}
	encoded = append(encoded, byte(s.Owner))
	encoded, err = appendCheckpointText(encoded, s.EventLogID)
	if err != nil {
		return nil, err
	}
	encoded = appendCheckpointU64(encoded, s.EventCount)
	encoded, err = appendCheckpointText(encoded, s.TailDigest)
	if err != nil {
		return nil, err
	}
	encoded = appendCheckpointU64(encoded, s.CheckpointSequence)
	if s.PriorCheckpointDigest == "" {
		encoded = append(encoded, 0)
	} else {
		encoded = append(encoded, 1)
		encoded, err = appendCheckpointText(encoded, s.PriorCheckpointDigest)
		if err != nil {
			return nil, err
		}
	}
	for _, value := range []string{
		s.SourceActionHash,
		s.SourceChainFrontier,
		s.IssuerDID,
		s.VerificationMethod,
		s.AuthorityGrantRef,
	} {
		encoded, err = appendCheckpointText(encoded, value)
		if err != nil {
			return nil, err
		}
	}
	encoded = appendCheckpointU64(encoded, s.IssuedAtUnixMS)
	return encoded, nil
}

func (s CheckpointSubjectV1) CanonicalDigest() (string, error) {
	encoded, err := s.CanonicalBytes()
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func appendCheckpointText(output []byte, value string) ([]byte, error) {
	if !utf8.ValidString(value) || len(value) > maxCheckpointFieldBytes {
		return nil, ErrCheckpointInvalidSubject
	}
	var size [4]byte
	binary.BigEndian.PutUint32(size[:], uint32(len(value)))
	output = append(output, size[:]...)
	output = append(output, []byte(value)...)
	return output, nil
}

func appendCheckpointU64(output []byte, value uint64) []byte {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	return append(output, encoded[:]...)
}

func isCheckpointDigest(value string) bool {
	if len(value) != 71 || !strings.HasPrefix(value, "sha256:") {
		return false
	}
	for _, char := range value[7:] {
		if !((char >= '0' && char <= '9') || (char >= 'a' && char <= 'f')) {
			return false
		}
	}
	return true
}
