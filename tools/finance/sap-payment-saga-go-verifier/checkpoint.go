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
	checkpointDomainV1 = "MXSAP-JOURNAL-CHECKPOINT-V1\x00"
	intentDomainV1     = "MXSAP-PAYMENT-INTENT-V1\x00"
	maxCheckpointTextBytes = 4096
	holochainActionHashSize = 39
)

type CheckpointOwnerDomain uint8

const (
	CheckpointPayments CheckpointOwnerDomain = 1
	CheckpointTreasury CheckpointOwnerDomain = 2
)

var (
	ErrCheckpointInvalidSubject       = errors.New("checkpoint rejected: invalid subject")
	ErrCheckpointInvalidDigest        = errors.New("checkpoint rejected: invalid digest")
	ErrCheckpointInvalidActionHash    = errors.New("checkpoint rejected: invalid ActionHash")
	ErrCheckpointInvalidSequence      = errors.New("checkpoint rejected: invalid checkpoint sequence")
	ErrCheckpointInvalidSourceFrontier = errors.New("checkpoint rejected: invalid source-chain frontier")
	ErrCheckpointInvalidIntent        = errors.New("checkpoint rejected: invalid payment intent")
)

// JournalCheckpointSubject is an effects-disabled, owner-scoped statement about
// one exact journal prefix. It is not a signature, source-action proof, or
// settlement claim. Raw ActionHash values use Holochain's 39-byte typed form.
type JournalCheckpointSubject struct {
	OwnerDomain                CheckpointOwnerDomain
	OperationID                string
	IntentDigest               string
	OwnerDID                   string
	EventLogID                 string
	EventCount                 uint64
	TailDigest                 string
	CheckpointSequence         uint64
	PreviousCheckpointDigest   string
	HasPreviousCheckpoint      bool
	SourceActionHash           []byte
	SourceActionSequence       uint32
	SourcePreviousActionHash   []byte
	HasSourcePreviousAction    bool
}

// CanonicalPaymentIntentDigest hashes every immutable intent field under a
// separate fixed-field profile. OperationID is bound separately in the subject.
func CanonicalPaymentIntentDigest(intent Intent) (string, error) {
	if err := intent.validate(); err != nil {
		return "", ErrCheckpointInvalidIntent
	}
	encoded := append([]byte{}, []byte(intentDomainV1)...)
	for _, value := range []string{
		intent.SenderDID,
		intent.RecipientDID,
		intent.ClientIdempotencyKey,
		intent.PayloadFingerprint,
	} {
		if err := appendCheckpointText(&encoded, value); err != nil {
			return "", err
		}
	}
	appendCheckpointU64(&encoded, intent.AmountMinor)
	appendCheckpointU64(&encoded, intent.FeeMinor)
	if err := appendCheckpointText(&encoded, intent.Currency); err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

// CanonicalBytes returns the versioned fixed-field checkpoint preimage.
func (s JournalCheckpointSubject) CanonicalBytes() ([]byte, error) {
	if err := s.Validate(); err != nil {
		return nil, err
	}
	encoded := append([]byte{}, []byte(checkpointDomainV1)...)
	encoded = append(encoded, byte(s.OwnerDomain))
	for _, value := range []string{
		s.OperationID,
		s.IntentDigest,
		s.OwnerDID,
		s.EventLogID,
	} {
		if err := appendCheckpointText(&encoded, value); err != nil {
			return nil, err
		}
	}
	appendCheckpointU64(&encoded, s.EventCount)
	if err := appendCheckpointText(&encoded, s.TailDigest); err != nil {
		return nil, err
	}
	appendCheckpointU64(&encoded, s.CheckpointSequence)

	if s.HasPreviousCheckpoint {
		encoded = append(encoded, 1)
		if err := appendCheckpointText(&encoded, s.PreviousCheckpointDigest); err != nil {
			return nil, err
		}
	} else {
		encoded = append(encoded, 0)
	}
	encoded = append(encoded, s.SourceActionHash...)
	var sourceSequence [4]byte
	binary.BigEndian.PutUint32(sourceSequence[:], s.SourceActionSequence)
	encoded = append(encoded, sourceSequence[:]...)
	if s.HasSourcePreviousAction {
		encoded = append(encoded, 1)
		encoded = append(encoded, s.SourcePreviousActionHash...)
	} else {
		encoded = append(encoded, 0)
	}
	return encoded, nil
}

// CanonicalDigest returns sha256 over CanonicalBytes, in lowercase tagged form.
func (s JournalCheckpointSubject) CanonicalDigest() (string, error) {
	encoded, err := s.CanonicalBytes()
	if err != nil {
		return "", err
	}
	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func (s JournalCheckpointSubject) Validate() error {
	if s.OwnerDomain != CheckpointPayments && s.OwnerDomain != CheckpointTreasury {
		return ErrCheckpointInvalidSubject
	}
	for _, value := range []string{s.OperationID, s.OwnerDID, s.EventLogID} {
		if strings.TrimSpace(value) == "" || strings.ContainsRune(value, '\x00') ||
			!utf8.ValidString(value) || len(value) > maxCheckpointTextBytes {
			return ErrCheckpointInvalidSubject
		}
	}
	if !strings.HasPrefix(s.OwnerDID, "did:") {
		return ErrCheckpointInvalidSubject
	}
	if !isSHA256Digest(s.IntentDigest) || !isSHA256Digest(s.TailDigest) {
		return ErrCheckpointInvalidDigest
	}
	if s.EventCount == 0 || s.CheckpointSequence == 0 {
		return ErrCheckpointInvalidSequence
	}
	if s.CheckpointSequence == 1 {
		if s.HasPreviousCheckpoint || s.PreviousCheckpointDigest != "" {
			return ErrCheckpointInvalidSequence
		}
	} else if !s.HasPreviousCheckpoint || !isSHA256Digest(s.PreviousCheckpointDigest) {
		return ErrCheckpointInvalidSequence
	}
	if err := validateCheckpointActionHash(s.SourceActionHash); err != nil {
		return err
	}
	if s.SourceActionSequence == 0 {
		if s.HasSourcePreviousAction || len(s.SourcePreviousActionHash) != 0 {
			return ErrCheckpointInvalidSourceFrontier
		}
	} else {
		if !s.HasSourcePreviousAction {
			return ErrCheckpointInvalidSourceFrontier
		}
		if err := validateCheckpointActionHash(s.SourcePreviousActionHash); err != nil {
			return ErrCheckpointInvalidSourceFrontier
		}
	}
	return nil
}

func validateCheckpointActionHash(hash []byte) error {
	if len(hash) != holochainActionHashSize ||
		hash[0] != 0x84 || hash[1] != 0x29 || hash[2] != 0x24 {
		return ErrCheckpointInvalidActionHash
	}
	return nil
}

func appendCheckpointText(output *[]byte, value string) error {
	if !utf8.ValidString(value) || len(value) > maxCheckpointTextBytes ||
		uint64(len(value)) > uint64(^uint32(0)) {
		return ErrCheckpointInvalidSubject
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	*output = append(*output, length[:]...)
	*output = append(*output, []byte(value)...)
	return nil
}

func appendCheckpointU64(output *[]byte, value uint64) {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	*output = append(*output, encoded[:]...)
}
