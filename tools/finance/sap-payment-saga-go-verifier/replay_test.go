package main

import (
	"os"
	"reflect"
	"strings"
	"testing"
)

func replayTestIntent() Intent {
	return Intent{
		SenderDID:            "did:sender",
		RecipientDID:         "did:recipient",
		ClientIdempotencyKey: "client-key-001",
		PayloadFingerprint:   "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
		AmountMinor:          100,
		FeeMinor:             3,
		Currency:             "SAP",
	}
}

func replayTestLocalEvidence(journal *Journal) Evidence {
	evidence := Evidence{
		OperationID:    journal.OperationID(),
		EffectID:       journal.EffectID(LocalPaymentTransaction),
		Leg:            LocalPaymentTransaction,
		SenderDID:      "did:sender",
		RecipientDID:   "did:recipient",
		PrincipalMinor: 100,
		FeeMinor:       3,
		Currency:       "SAP",
		Local: &LocalPaymentEvidence{
			SenderAccountID:            "did:sender",
			RecipientAccountID:         "did:recipient",
			SenderDebitMinor:           103,
			RecipientCreditMinor:       100,
			SenderPredecessorAction:    "action:sender-before",
			SenderDebitAction:          "action:sender-debit",
			RecipientPredecessorAction: "action:recipient-before",
			RecipientCreditAction:      "action:recipient-credit",
			PaymentActionHash:          "action:payment",
			PaymentStatus:              "Completed",
			ReceiptActionHash:          "action:receipt",
			ReceiptSignerDID:           "did:sender",
			SigningProfile:             ReceiptProfileV1,
			SignatureDigest:            "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
			TreasuryFeeOutboxID:        journal.EffectID(TreasuryFee),
		},
	}
	evidence.EvidenceDigest, _ = canonicalEvidenceDigest(evidence)
	return evidence
}

func replayTestTreasuryEvidence(journal *Journal) Evidence {
	evidence := Evidence{
		OperationID:    journal.OperationID(),
		EffectID:       journal.EffectID(TreasuryFee),
		Leg:            TreasuryFee,
		SenderDID:      "did:sender",
		RecipientDID:   "did:recipient",
		PrincipalMinor: 100,
		FeeMinor:       3,
		Currency:       "SAP",
		Treasury: &TreasuryFeeEvidence{
			PoolID:            "treasury:global-fee-pool",
			SourceMemberDID:   "did:sender",
			AmountMinor:       3,
			PaymentActionHash: "action:payment",
			IdempotencyKey:    journal.EffectID(TreasuryFee),
			PredecessorAction: "action:treasury-before",
			TreasuryAction:    "action:treasury-credit",
		},
	}
	evidence.EvidenceDigest, _ = canonicalEvidenceDigest(evidence)
	return evidence
}

func TestJournalEventFrameHashesMatchSharedCrossLanguageVectors(t *testing.T) {
	data, err := os.ReadFile("fixtures/sap-payment-journal-replay-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	vectors := make(map[string]string)
	for _, line := range strings.Split(string(data), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		name, value, ok := strings.Cut(line, "=")
		if !ok {
			t.Fatalf("invalid known-answer line: %q", line)
		}
		vectors[name] = value
	}

	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[0].EventDigest; got != vectors["created-basic"] {
		t.Fatalf("created frame digest = %s, want %s", got, vectors["created-basic"])
	}
	if err := log.RecordAuthorized(); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[1].EventDigest; got != vectors["authorized-after-created"] {
		t.Fatalf("authorized frame digest = %s, want %s", got, vectors["authorized-after-created"])
	}
	journal := newJournal(replayTestIntent())
	if err := log.RecordEffectPrepared(
		LocalPaymentTransaction,
		journal.EffectID(LocalPaymentTransaction),
	); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[2].EventDigest; got != vectors["local-prepared-after-authorized"] {
		t.Fatalf("prepared frame digest = %s, want %s", got, vectors["local-prepared-after-authorized"])
	}
}

func TestJournalEventReplayReconstructsLocalAndTreasuryRecovery(t *testing.T) {
	data, err := os.ReadFile("fixtures/sap-payment-journal-replay-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	vectors := make(map[string]string)
	for _, line := range strings.Split(string(data), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		name, value, ok := strings.Cut(line, "=")
		if !ok {
			t.Fatalf("invalid known-answer line: %q", line)
		}
		vectors[name] = value
	}

	journal := newJournal(replayTestIntent())
	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[0].EventDigest; got != vectors["history-frame-00"] {
		t.Fatalf("created event digest = %s, want %s", got, vectors["history-frame-00"])
	}

	if err := journal.Authorize(); err != nil {
		t.Fatal(err)
	}
	if err := log.RecordAuthorized(); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[1].EventDigest; got != vectors["history-frame-01"] {
		t.Fatalf("authorized event digest = %s, want %s", got, vectors["history-frame-01"])
	}

	localID := journal.EffectID(LocalPaymentTransaction)
	if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
		t.Fatal(err)
	}
	if err := log.RecordEffectPrepared(LocalPaymentTransaction, localID); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[2].EventDigest; got != vectors["history-frame-02"] {
		t.Fatalf("local prepare digest = %s, want %s", got, vectors["history-frame-02"])
	}
	local := replayTestLocalEvidence(journal)
	if local.EvidenceDigest != vectors["local-effect-evidence"] {
		t.Fatalf("local effect evidence digest = %s, want %s", local.EvidenceDigest, vectors["local-effect-evidence"])
	}
	if _, err := journal.CommitLeg(LocalPaymentTransaction, local); err != nil {
		t.Fatal(err)
	}
	if err := log.RecordEffectCommitted(local); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[3].EventDigest; got != vectors["history-frame-03"] {
		t.Fatalf("local commit digest = %s, want %s", got, vectors["history-frame-03"])
	}

	treasuryID := journal.EffectID(TreasuryFee)
	if _, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil {
		t.Fatal(err)
	}
	if err := log.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[4].EventDigest; got != vectors["history-frame-04"] {
		t.Fatalf("Treasury prepare digest = %s, want %s", got, vectors["history-frame-04"])
	}
	if err := journal.MarkIndeterminate(TreasuryFee, treasuryID); err != nil {
		t.Fatal(err)
	}
	if err := log.RecordEffectIndeterminate(TreasuryFee, treasuryID); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[5].EventDigest; got != vectors["history-frame-05"] {
		t.Fatalf("Treasury indeterminate digest = %s, want %s", got, vectors["history-frame-05"])
	}
	if transition, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil || transition != TransitionResumedSameEffect {
		t.Fatalf("same-effect recovery = %q, %v", transition, err)
	}
	if err := log.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[6].EventDigest; got != vectors["history-frame-06"] {
		t.Fatalf("Treasury retry prepare digest = %s, want %s", got, vectors["history-frame-06"])
	}
	treasury := replayTestTreasuryEvidence(journal)
	if treasury.EvidenceDigest != vectors["treasury-effect-evidence"] {
		t.Fatalf("Treasury effect evidence digest = %s, want %s", treasury.EvidenceDigest, vectors["treasury-effect-evidence"])
	}
	if _, err := journal.CommitLeg(TreasuryFee, treasury); err != nil {
		t.Fatal(err)
	}
	if err := log.RecordEffectCommitted(treasury); err != nil {
		t.Fatal(err)
	}
	if got := log.Frames()[7].EventDigest; got != vectors["history-frame-07"] {
		t.Fatalf("Treasury commit digest = %s, want %s", got, vectors["history-frame-07"])
	}
	if got := log.Anchor().TailDigest; got != vectors["history-tail-anchor"] {
		t.Fatalf("history tail anchor = %s, want %s", got, vectors["history-tail-anchor"])
	}

	if got := journal.Status(); got != Completed {
		t.Fatalf("live journal status = %q, want %q", got, Completed)
	}
	replayed, err := ReplayJournal(log.Frames(), log.Anchor())
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(replayed, journal) {
		t.Fatalf("replayed journal differs from live projection:\nreplayed=%#v\nlive=%#v", replayed, journal)
	}
}

func TestJournalReplayRejectsTailTruncationAgainstExternalAnchor(t *testing.T) {
	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	if err := log.RecordAuthorized(); err != nil {
		t.Fatal(err)
	}
	anchor := log.Anchor()
	frames := log.Frames()
	frames = frames[:len(frames)-1]
	if _, err := ReplayJournal(frames, anchor); err != ErrReplayAnchorMismatch {
		t.Fatalf("truncated journal error = %v, want %v", err, ErrReplayAnchorMismatch)
	}
}

func TestJournalReplayRejectsReorderedEvents(t *testing.T) {
	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	_ = log.RecordAuthorized()
	_ = log.RecordRejected()
	anchor := log.Anchor()
	frames := log.Frames()
	frames[1], frames[2] = frames[2], frames[1]
	if _, err := ReplayJournal(frames, anchor); err != ErrReplaySequenceMismatch {
		t.Fatalf("reordered journal error = %v, want %v", err, ErrReplaySequenceMismatch)
	}
}

func TestJournalReplayRejectsEventPayloadTampering(t *testing.T) {
	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	_ = log.RecordAuthorized()
	anchor := log.Anchor()
	frames := log.Frames()
	frames[1].Event.Kind = JournalRejected
	if _, err := ReplayJournal(frames, anchor); err != ErrReplayEventDigestMismatch {
		t.Fatalf("tampered journal error = %v, want %v", err, ErrReplayEventDigestMismatch)
	}
}

func TestJournalReplayRejectsImpossibleTransitionOrder(t *testing.T) {
	journal := newJournal(replayTestIntent())
	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	if err := log.RecordEffectPrepared(
		LocalPaymentTransaction,
		journal.EffectID(LocalPaymentTransaction),
	); err != nil {
		t.Fatal(err)
	}
	if _, err := ReplayJournal(log.Frames(), log.Anchor()); err != ErrReplayInvalidTransition {
		t.Fatalf("invalid transition error = %v, want %v", err, ErrReplayInvalidTransition)
	}
}

func TestJournalReplayRejectsInvalidCommittedEvidenceDigest(t *testing.T) {
	journal := newJournal(replayTestIntent())
	if err := journal.Authorize(); err != nil {
		t.Fatal(err)
	}
	localID := journal.EffectID(LocalPaymentTransaction)
	if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
		t.Fatal(err)
	}
	evidence := replayTestLocalEvidence(journal)
	evidence.EvidenceDigest = "sha256:bad"

	log, err := NewJournalEventLog(replayTestIntent())
	if err != nil {
		t.Fatal(err)
	}
	_ = log.RecordAuthorized()
	_ = log.RecordEffectPrepared(LocalPaymentTransaction, localID)
	if err := log.RecordEffectCommitted(evidence); err != nil {
		t.Fatal(err)
	}
	if _, err := ReplayJournal(log.Frames(), log.Anchor()); err != ErrReplayInvalidEvidence {
		t.Fatalf("invalid evidence error = %v, want %v", err, ErrReplayInvalidEvidence)
	}
}
