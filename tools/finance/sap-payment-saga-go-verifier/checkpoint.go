package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"unicode/utf8"
)

const (
	checkpointIntentDomain = "MXSAP-PAYMENT-INTENT-V1\x00"
	checkpointSubjectDomain = "MXSAP-JOURNAL-CHECKPOINT-V1\x00"
	maxCheckpointTextBytes = 16 * 1024
)

type CheckpointOwner uint8

const (
	CheckpointPayments CheckpointOwner = iota + 1
	CheckpointTreasury
)

type CheckpointLifecycle uint8

const (
	CheckpointCurrent CheckpointLifecycle = iota + 1
	CheckpointSupersedes
	CheckpointRetracts
)

// JournalCheckpointSubjectV1 is one source owner's attestation subject.
// It does not combine Payments and Treasury into a synthetic global order.
type JournalCheckpointSubjectV1 struct {
	Owner                       CheckpointOwner
	Lifecycle                   CheckpointLifecycle
	OperationID                 string
	IntentDigest                string
	EventLogID                  string
	EventCount                  uint64
	TailDigest                  string
	CheckpointSequence          uint64
	PreviousCheckpointDigest    *string
	SourceActionHash            string
	IssuerDID                   string
	VerificationMethod          string
	AuthorityProfileDigest      string
	IssuedAtUnixMicros          uint64
	SupersedesCheckpointDigest  *string
}

var ErrCheckpointInvalid = errors.New("journal checkpoint subject is invalid")

// CanonicalDigest validates the complete subject against its immutable intent,
// then returns its deterministic, versioned SHA-256 identity.
func (s JournalCheckpointSubjectV1) CanonicalDigest(intent Intent) (string, error) {
	encoded, err := s.CanonicalBytes(intent)
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

// CanonicalBytes is exported so independent implementations can reproduce the
// exact profile. It is not a signature, action-authenticity or settlement proof.
func (s JournalCheckpointSubjectV1) CanonicalBytes(intent Intent) ([]byte, error) {
	if err := intent.validate(); err != nil {
		return nil, ErrCheckpointInvalid
	}
	if s.OperationID != OperationID(intent.SenderDID, intent.ClientIdempotencyKey) {
		return nil, ErrCheckpointInvalid
	}
	intentDigest, err := CanonicalPaymentIntentDigest(intent)
	if err != nil || s.IntentDigest != intentDigest {
		return nil, ErrCheckpointInvalid
	}
	if s.Owner != CheckpointPayments && s.Owner != CheckpointTreasury {
		return nil, ErrCheckpointInvalid
	}
	if s.Lifecycle < CheckpointCurrent || s.Lifecycle > CheckpointRetracts {
		return nil, ErrCheckpointInvalid
	}
	if s.EventCount == 0 || s.CheckpointSequence == 0 || s.IssuedAtUnixMicros == 0 {
		return nil, ErrCheckpointInvalid
	}
	if !isSHA256Digest(s.TailDigest) || !isSHA256Digest(s.IntentDigest) ||
		!isSHA256Digest(s.AuthorityProfileDigest) {
		return nil, ErrCheckpointInvalid
	}
	if s.CheckpointSequence == 1 {
		if s.PreviousCheckpointDigest != nil {
			return nil, ErrCheckpointInvalid
		}
	} else {
		if s.PreviousCheckpointDigest == nil || !isSHA256Digest(*s.PreviousCheckpointDigest) {
			return nil, ErrCheckpointInvalid
		}
	}
	switch s.Lifecycle {
	case CheckpointCurrent:
		if s.SupersedesCheckpointDigest != nil {
			return nil, ErrCheckpointInvalid
		}
	case CheckpointSupersedes, CheckpointRetracts:
		if s.SupersedesCheckpointDigest == nil || !isSHA256Digest(*s.SupersedesCheckpointDigest) {
			return nil, ErrCheckpointInvalid
		}
	}

	encoded := make([]byte, 0, 512)
	encoded = append(encoded, []byte(checkpointSubjectDomain)...)
	encoded = append(encoded, 1, byte(s.Owner), byte(s.Lifecycle))
	for _, value := range []string{s.OperationID, s.IntentDigest, s.EventLogID} {
		var err error
		encoded, err = appendCheckpointText(encoded, value)
		if err != nil {
			return nil, err
		}
	}
	encoded = appendCheckpointU64(encoded, s.EventCount)
	var appendErr error
	encoded, appendErr = appendCheckpointText(encoded, s.TailDigest)
	if appendErr != nil {
		return nil, appendErr
	}
	encoded = appendCheckpointU64(encoded, s.CheckpointSequence)
	encoded, appendErr = appendCheckpointOptionalText(encoded, s.PreviousCheckpointDigest)
	if appendErr != nil {
		return nil, appendErr
	}
	for _, value := range []string{s.SourceActionHash, s.IssuerDID, s.VerificationMethod, s.AuthorityProfileDigest} {
		encoded, appendErr = appendCheckpointText(encoded, value)
		if appendErr != nil {
			return nil, appendErr
		}
	}
	encoded = appendCheckpointU64(encoded, s.IssuedAtUnixMicros)
	encoded, appendErr = appendCheckpointOptionalText(encoded, s.SupersedesCheckpointDigest)
	if appendErr != nil {
		return nil, appendErr
	}
	return encoded, nil
}

// CanonicalPaymentIntentDigest hashes the exact immutable intent using fixed
// field order, length-prefixed UTF-8 strings and big-endian integer minor units.
func CanonicalPaymentIntentDigest(intent Intent) (string, error) {
	if err := intent.validate(); err != nil {
		return "", ErrCheckpointInvalid
	}
	encoded := append([]byte(checkpointIntentDomain), 1)
	var err error
	for _, value := range []string{
		intent.SenderDID,
		intent.RecipientDID,
		intent.ClientIdempotencyKey,
		intent.PayloadFingerprint,
	} {
		encoded, err = appendCheckpointText(encoded, value)
		if err != nil {
			return "", err
		}
	}
	encoded = appendCheckpointU64(encoded, intent.AmountMinor)
	encoded = appendCheckpointU64(encoded, intent.FeeMinor)
	encoded, err = appendCheckpointText(encoded, intent.Currency)
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func appendCheckpointText(output []byte, value string) ([]byte, error) {
	if value == "" || len(value) > maxCheckpointTextBytes || !utf8.ValidString(value) ||
		uint64(len(value)) > uint64(^uint32(0)) {
		return nil, ErrCheckpointInvalid
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	output = append(output, length[:]...)
	output = append(output, value...)
	return output, nil
}

func appendCheckpointOptionalText(output []byte, value *string) ([]byte, error) {
	if value == nil {
		return append(output, 0), nil
	}
	output = append(output, 1)
	return appendCheckpointText(output, *value)
}

func appendCheckpointU64(output []byte, value uint64) []byte {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	return append(output, encoded[:]...)
}
