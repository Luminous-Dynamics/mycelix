package main

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"unicode/utf8"
)

const (
	journalEventDomain = "MXSAP-JOURNAL-EVENT-V1\x00"
	journalGenesisHash = "sha256:0000000000000000000000000000000000000000000000000000000000000000"
	maxJournalEventFieldBytes = 16 * 1024
)

type JournalEventKind uint8

const (
	JournalCreated JournalEventKind = iota + 1
	JournalAuthorized
	JournalRejected
	JournalEffectPrepared
	JournalEffectIndeterminate
	JournalEffectCommitted
)

type JournalEvent struct {
	Kind        JournalEventKind
	OperationID string
	Intent      Intent
	Leg         Leg
	EffectID    string
	Evidence    *Evidence
}

func (e JournalEvent) clone() JournalEvent {
	out := e
	if e.Evidence != nil {
		copyEvidence := e.Evidence.clone()
		out.Evidence = &copyEvidence
	}
	return out
}

type JournalEventFrame struct {
	Sequence       uint64
	PreviousDigest string
	Event          JournalEvent
	EventDigest    string
}

type ReplayAnchor struct {
	EventCount uint64
	TailDigest string
}

type JournalEventLog struct {
	frames []JournalEventFrame
}

func NewJournalEventLog(intent Intent) (*JournalEventLog, error) {
	if err := intent.validate(); err != nil {
		return nil, ErrReplayInvalidIntent
	}
	journal := newJournal(intent)
	log := &JournalEventLog{}
	if err := log.append(JournalEvent{
		Kind:        JournalCreated,
		OperationID: journal.OperationID(),
		Intent:      intent,
	}); err != nil {
		return nil, err
	}
	return log, nil
}

func (l *JournalEventLog) Frames() []JournalEventFrame {
	out := make([]JournalEventFrame, len(l.frames))
	for index, frame := range l.frames {
		out[index] = frame
		out[index].Event = frame.Event.clone()
	}
	return out
}

func (l *JournalEventLog) Anchor() ReplayAnchor {
	tail := journalGenesisHash
	if len(l.frames) > 0 {
		tail = l.frames[len(l.frames)-1].EventDigest
	}
	return ReplayAnchor{EventCount: uint64(len(l.frames)), TailDigest: tail}
}

func (l *JournalEventLog) RecordAuthorized() error {
	return l.append(JournalEvent{Kind: JournalAuthorized})
}

func (l *JournalEventLog) RecordRejected() error {
	return l.append(JournalEvent{Kind: JournalRejected})
}

func (l *JournalEventLog) RecordEffectPrepared(leg Leg, effectID string) error {
	return l.append(JournalEvent{
		Kind:     JournalEffectPrepared,
		Leg:      leg,
		EffectID: effectID,
	})
}

func (l *JournalEventLog) RecordEffectIndeterminate(leg Leg, effectID string) error {
	return l.append(JournalEvent{
		Kind:     JournalEffectIndeterminate,
		Leg:      leg,
		EffectID: effectID,
	})
}

func (l *JournalEventLog) RecordEffectCommitted(evidence Evidence) error {
	copyEvidence := evidence.clone()
	return l.append(JournalEvent{
		Kind:     JournalEffectCommitted,
		Leg:      evidence.Leg,
		Evidence: &copyEvidence,
	})
}

func (l *JournalEventLog) append(event JournalEvent) error {
	if uint64(len(l.frames)) == ^uint64(0) {
		return ErrReplayInvalidEvent
	}
	sequence := uint64(len(l.frames))
	previous := journalGenesisHash
	if len(l.frames) > 0 {
		previous = l.frames[len(l.frames)-1].EventDigest
	}
	digest, err := journalEventDigest(sequence, previous, event)
	if err != nil {
		return err
	}
	l.frames = append(l.frames, JournalEventFrame{
		Sequence:       sequence,
		PreviousDigest: previous,
		Event:          event.clone(),
		EventDigest:    digest,
	})
	return nil
}

// ReplayJournal reconstructs a fresh journal from the exact event stream.
// The anchor must be retained outside the stream to detect tail truncation.
// It checks byte integrity and model transitions, not issuer or source-record
// authenticity.
func ReplayJournal(frames []JournalEventFrame, anchor ReplayAnchor) (*Journal, error) {
	if uint64(len(frames)) != anchor.EventCount {
		return nil, ErrReplayAnchorMismatch
	}
	if len(frames) == 0 {
		return nil, ErrReplayMissingCreate
	}

	previous := journalGenesisHash
	var journal *Journal
	for index, frame := range frames {
		if frame.Sequence != uint64(index) {
			return nil, ErrReplaySequenceMismatch
		}
		if frame.PreviousDigest != previous {
			return nil, ErrReplayPreviousDigestMismatch
		}
		expected, err := journalEventDigest(frame.Sequence, frame.PreviousDigest, frame.Event)
		if err != nil {
			return nil, err
		}
		if frame.EventDigest != expected {
			return nil, ErrReplayEventDigestMismatch
		}

		if index == 0 {
			if frame.Event.Kind != JournalCreated {
				return nil, ErrReplayMissingCreate
			}
			if err := frame.Event.Intent.validate(); err != nil {
				return nil, ErrReplayInvalidIntent
			}
			created := newJournal(frame.Event.Intent)
			if created.OperationID() != frame.Event.OperationID {
				return nil, ErrReplayOperationIdentityMismatch
			}
			journal = created
		} else {
			if journal == nil {
				return nil, ErrReplayMissingCreate
			}
			if err := applyJournalEvent(journal, frame.Event); err != nil {
				return nil, err
			}
		}
		previous = frame.EventDigest
	}

	if previous != anchor.TailDigest {
		return nil, ErrReplayAnchorMismatch
	}
	if journal == nil {
		return nil, ErrReplayMissingCreate
	}
	return journal, nil
}

func applyJournalEvent(journal *Journal, event JournalEvent) error {
	switch event.Kind {
	case JournalCreated:
		return ErrReplayDuplicateCreate
	case JournalAuthorized:
		if err := journal.Authorize(); err != nil {
			return ErrReplayInvalidTransition
		}
		return nil
	case JournalRejected:
		if err := journal.RejectBeforeEffects(); err != nil {
			return ErrReplayInvalidTransition
		}
		return nil
	case JournalEffectPrepared:
		if _, err := journal.PrepareLeg(event.Leg, event.EffectID); err != nil {
			return ErrReplayInvalidTransition
		}
		return nil
	case JournalEffectIndeterminate:
		if err := journal.MarkIndeterminate(event.Leg, event.EffectID); err != nil {
			return ErrReplayInvalidTransition
		}
		return nil
	case JournalEffectCommitted:
		if event.Evidence == nil || event.Evidence.Leg != event.Leg {
			return ErrReplayInvalidEvidence
		}
		if _, err := journal.CommitLeg(event.Leg, *event.Evidence); err != nil {
			if errors.Is(err, ErrEvidenceInvalid) {
				return ErrReplayInvalidEvidence
			}
			return ErrReplayInvalidTransition
		}
		return nil
	default:
		return ErrReplayInvalidEvent
	}
}

func journalEventDigest(sequence uint64, previous string, event JournalEvent) (string, error) {
	var encoded []byte
	encoded = append(encoded, []byte(journalEventDomain)...)
	var number [8]byte
	binary.BigEndian.PutUint64(number[:], sequence)
	encoded = append(encoded, number[:]...)
	if err := appendJournalEventText(&encoded, previous); err != nil {
		return "", err
	}

	switch event.Kind {
	case JournalCreated:
		encoded = append(encoded, byte(JournalCreated))
		if err := appendJournalEventText(&encoded, event.OperationID); err != nil {
			return "", err
		}
		for _, value := range []string{
			event.Intent.SenderDID,
			event.Intent.RecipientDID,
			event.Intent.ClientIdempotencyKey,
			event.Intent.PayloadFingerprint,
		} {
			if err := appendJournalEventText(&encoded, value); err != nil {
				return "", err
			}
		}
		appendJournalEventU64(&encoded, event.Intent.AmountMinor)
		appendJournalEventU64(&encoded, event.Intent.FeeMinor)
		if err := appendJournalEventText(&encoded, event.Intent.Currency); err != nil {
			return "", err
		}
	case JournalAuthorized, JournalRejected:
		encoded = append(encoded, byte(event.Kind))
	case JournalEffectPrepared, JournalEffectIndeterminate:
		encoded = append(encoded, byte(event.Kind), journalLegTag(event.Leg))
		if err := appendJournalEventText(&encoded, event.EffectID); err != nil {
			return "", err
		}
	case JournalEffectCommitted:
		if event.Evidence == nil || event.Evidence.Leg != event.Leg {
			return "", ErrReplayInvalidEvidence
		}
		canonical, err := canonicalEvidenceDigest(*event.Evidence)
		if err != nil {
			return "", ErrReplayInvalidEvidence
		}
		encoded = append(encoded, byte(JournalEffectCommitted), journalLegTag(event.Leg))
		if err := appendJournalEventText(&encoded, canonical); err != nil {
			return "", err
		}
		if err := appendJournalEventText(&encoded, event.Evidence.EvidenceDigest); err != nil {
			return "", err
		}
	default:
		return "", ErrReplayInvalidEvent
	}

	digest := sha256.Sum256(encoded)
	return "sha256:" + hex.EncodeToString(digest[:]), nil
}

func appendJournalEventText(output *[]byte, value string) error {
	if !utf8.ValidString(value) || len(value) > maxJournalEventFieldBytes || uint64(len(value)) > uint64(^uint32(0)) {
		return ErrReplayInvalidEvent
	}
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(value)))
	*output = append(*output, length[:]...)
	*output = append(*output, []byte(value)...)
	return nil
}

func appendJournalEventU64(output *[]byte, value uint64) {
	var encoded [8]byte
	binary.BigEndian.PutUint64(encoded[:], value)
	*output = append(*output, encoded[:]...)
}

func journalLegTag(leg Leg) byte {
	switch leg {
	case LocalPaymentTransaction:
		return 1
	case TreasuryFee:
		return 2
	default:
		return 0
	}
}

var (
	ErrReplayInvalidIntent             = errors.New("replay rejected: invalid intent")
	ErrReplayInvalidEvent              = errors.New("replay rejected: invalid event")
	ErrReplayMissingCreate              = errors.New("replay rejected: missing create event")
	ErrReplayDuplicateCreate            = errors.New("replay rejected: duplicate create event")
	ErrReplayOperationIdentityMismatch = errors.New("replay rejected: operation identity mismatch")
	ErrReplaySequenceMismatch           = errors.New("replay rejected: sequence mismatch")
	ErrReplayPreviousDigestMismatch    = errors.New("replay rejected: previous digest mismatch")
	ErrReplayEventDigestMismatch       = errors.New("replay rejected: event digest mismatch")
	ErrReplayAnchorMismatch             = errors.New("replay rejected: checkpoint mismatch")
	ErrReplayInvalidTransition          = errors.New("replay rejected: invalid state transition")
	ErrReplayInvalidEvidence            = errors.New("replay rejected: invalid effect evidence")
)

