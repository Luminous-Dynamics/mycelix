package main

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"strings"
	"unicode/utf8"
)

const (
	ownerCheckpointDomain                                  = "MXSAP-OWNER-CHECKPOINT-SUBJECT-V1\x00"
	maxOwnerCheckpointTextBytes                            = 4 * 1024
	holochainActionHashBytes                             = 39
	ownerCheckpointKeyProfileHolochainActionEd25519V1 byte = 1
)

type CheckpointOwnerDomain uint8

const (
	CheckpointPaymentsLocalTransaction CheckpointOwnerDomain = 1
	CheckpointTreasuryFeeEffect        CheckpointOwnerDomain = 2
)

var (
	ErrCheckpointInvalid         = errors.New("owner checkpoint subject is invalid")
	ErrCheckpointInvalidDigest   = errors.New("owner checkpoint digest is not canonical sha256")
	ErrCheckpointInvalidSequence = errors.New("owner checkpoint sequence/prior link is invalid")
	ErrCheckpointJournalIdentity = errors.New("owner checkpoint journal identity does not match operation and intent")
	ErrCheckpointAuthorityAction = errors.New("owner checkpoint authority ActionHash is invalid")
)

type OwnerCheckpointSubjectV1 struct {
	OperationID                string
	IntentDigest               string
	OwnerDomain                CheckpointOwnerDomain
	JournalIdentityDigest     string
	EventCount                 uint64
	TailDigest                 string
	CheckpointSequence         uint64
	PriorCheckpointDigest      *string
	SourceActionHash           []byte
	OwnerAuthorityActionHash   []byte
	IssuerDID                  string
	IssuerKeyFingerprint       string
	OwnerAuthorityDigest       string
	IssuedAtMicros             int64
	SupersedesCheckpointDigest *string
}

func (s OwnerCheckpointSubjectV1) Validate() error {
	if s.OwnerDomain != CheckpointPaymentsLocalTransaction && s.OwnerDomain != CheckpointTreasuryFeeEffect {
		return ErrCheckpointInvalid
	}
	for _, value := range []string{s.OperationID, s.IssuerDID} {
		if err := validateOwnerCheckpointText(value); err != nil {
			return err
		}
	}
	for _, value := range []string{s.IntentDigest, s.JournalIdentityDigest, s.TailDigest, s.IssuerKeyFingerprint, s.OwnerAuthorityDigest} {
		if err := validateOwnerCheckpointDigest(value); err != nil {
			return err
		}
	}
	expectedLogDigest, err := ownerCheckpointJournalIdentityDigest(s.OperationID, s.IntentDigest)
	if err != nil {
		return err
	}
	if s.JournalIdentityDigest != expectedLogDigest {
		return ErrCheckpointJournalIdentity
	}
	if s.EventCount == 0 {
		return ErrCheckpointInvalid
	}
	if s.CheckpointSequence == 0 {
		return ErrCheckpointInvalidSequence
	}
	if s.CheckpointSequence == 1 {
		if s.PriorCheckpointDigest != nil {
			return ErrCheckpointInvalidSequence
		}
	} else {
		if s.PriorCheckpointDigest == nil {
			return ErrCheckpointInvalidSequence
		}
		if err := validateOwnerCheckpointDigest(*s.PriorCheckpointDigest); err != nil {
			return err
		}
	}
	if len(s.SourceActionHash) != holochainActionHashBytes {
		return ErrCheckpointInvalid
	}
	if len(s.OwnerAuthorityActionHash) != holochainActionHashBytes {
		return ErrCheckpointAuthorityAction
	}
	if !strings.HasPrefix(s.IssuerDID, "did:mycelix:") || strings.TrimSpace(s.IssuerDID) != s.IssuerDID {
		return ErrCheckpointInvalid
	}
	if s.SupersedesCheckpointDigest != nil {
		if err := validateOwnerCheckpointDigest(*s.SupersedesCheckpointDigest); err != nil {
			return err
		}
	}
	return nil
}

func (s OwnerCheckpointSubjectV1) CanonicalBytes() ([]byte, error) {
	if err := s.Validate(); err != nil {
		return nil, err
	}
	encoded := make([]byte, 0, 512+len(s.SourceActionHash))
	encoded = append(encoded, []byte(ownerCheckpointDomain)...)
	encoded = append(encoded, byte(s.OwnerDomain))
	var err error
	for _, value := range []string{s.OperationID, s.IntentDigest, s.EventLogDigest} {
		encoded, err = appendOwnerCheckpointText(encoded, value)
		if err != nil {
			return nil, err
		}
	}
	encoded = appendOwnerCheckpointU64(encoded, s.EventCount)
	encoded, err = appendOwnerCheckpointText(encoded, s.TailDigest)
	if err != nil {
		return nil, err
	}
	encoded = appendOwnerCheckpointU64(encoded, s.CheckpointSequence)
	encoded, err = appendOwnerCheckpointOptionalDigest(encoded, s.PriorCheckpointDigest)
	if err != nil {
		return nil, err
	}
	encoded, err = appendOwnerCheckpointBytes(encoded, s.SourceActionHash)
	if err != nil {
		return nil, err
	}
	encoded, err = appendOwnerCheckpointBytes(encoded, s.OwnerAuthorityActionHash)
	if err != nil {
		return nil, err
	}
	encoded, err = appendOwnerCheckpointText(encoded, s.IssuerDID)
	if err != nil {
		return nil, err
	}
	encoded = append(encoded, ownerCheckpointKeyProfileHolochainActionEd25519V1)
	for _, value := range []string{s.IssuerKeyFingerprint, s.OwnerAuthorityDigest} {
		encoded, err = appendOwnerCheckpointText(encoded, value)
		if err != nil {
			return nil, err
		}
	}
	var timestamp [8]byte
	binary.BigEndian.PutUint64(timestamp[:], uint64(s.IssuedAtMicros))
	encoded = append(encoded, timestamp[:]...)
	encoded, err = appendOwnerCheckpointOptionalDigest(encoded, s.SupersedesCheckpointDigest)
	if err != nil {
		return nil, err
	}
	return encoded, nil
}

func (s OwnerCheckpointSubjectV1) Digest() (string, error) {
	encoded, err := s.CanonicalBytes()
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func ownerCheckpointJournalIdentityDigest(operationID, intentDigest string) (string, error) {
	encoded := []byte("MXSAP-JOURNAL-IDENTITY-V1\x00")
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(operationID)))
	encoded = append(encoded, length[:]...)
	encoded = append(encoded, []byte(operationID)...)
	binary.BigEndian.PutUint32(length[:], uint32(len(intentDigest)))
	encoded = append(encoded, length[:]...)
	encoded = append(encoded, []byte(intentDigest)...)
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func validateOwnerCheckpointText(value string) error {
	if value == "" || len(value) > maxOwnerCheckpointTextBytes || !utf8.ValidString(value) {
		return ErrCheckpointInvalid
	}
	for _, b := range []byte(value) {
		if b >= 0x80 {
			return ErrCheckpointInvalid
		}
	}
	return nil
}

func validateOwnerCheckpointDigest(value string) error {
	if len(value) != len("sha256:")+64 || !strings.HasPrefix(value, "sha256:") {
		return ErrCheckpointInvalidDigest
	}
	if _, err := hex.DecodeString(strings.TrimPrefix(value, "sha256:")); err != nil {
		return ErrCheckpointInvalidDigest
	}
	for _, b := range []byte(value[len("sha256:"):]) {
		if b >= 'A' && b <= 'F' {
			return ErrCheckpointInvalidDigest
		}
	}
	return nil
}

func appendOwnerCheckpointText(output []byte, value string) ([]byte, error) {
	if err := validateOwnerCheckpointText(value); err != nil {
		return nil, err
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	output = append(output, length[:]...)
	output = append(output, []byte(value)...)
	return output, nil
}

func appendOwnerCheckpointOptionalDigest(output []byte, value *string) ([]byte, error) {
	if value == nil {
		return append(output, 0), nil
	}
	output = append(output, 1)
	return appendOwnerCheckpointText(output, *value)
}

func appendOwnerCheckpointBytes(output []byte, value []byte) ([]byte, error) {
	if len(value) != holochainActionHashBytes {
		return nil, ErrCheckpointInvalid
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	output = append(output, length[:]...)
	output = append(output, value...)
	return output, nil
}

func appendOwnerCheckpointU64(output []byte, value uint64) []byte {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	return append(output, encoded[:]...)
}

func (d CheckpointOwnerDomain) String() string {
	switch d {
	case CheckpointPaymentsLocalTransaction:
		return "payments_local_transaction"
	case CheckpointTreasuryFeeEffect:
		return "treasury_fee_effect"
	default:
		return fmt.Sprintf("unknown_owner_domain_%d", d)
	}
}
