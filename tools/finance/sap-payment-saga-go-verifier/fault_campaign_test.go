package sagaverifier

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"os"
	"strings"
	"strconv"
	"testing"
)

const (
	faultCampaignRootSeedV1   uint64 = 0x4d594843414d5031
	faultCampaignSequenceV1   uint64 = 0x9e3779b97f4a7c15
	faultCampaignDomainV1     uint64 = 0xd1b54a32d192ed03
	faultCampaignLocalAckLost uint8  = 0b01
	faultCampaignTreasuryAck  uint8  = 0b10
	faultCampaignPresubmit    uint64 = 10_000
	faultCampaignFull         uint64 = 1_000_000
)

type faultCampaignPlanV1 struct {
	Sequence                       uint64
	Seed                           uint64
	AmountMinor                    uint64
	FeeMinor                       uint64
	LocalIndeterminateRetries      uint8
	TreasuryIndeterminateRetries   uint8
	LostCommitAckMask              uint8
}

type faultCampaignOutcomeV1 struct {
	OperationID     string
	LocalEffectID   string
	TreasuryEffectID string
	Status          OperationStatus
	EventCount      uint64
	TailDigest      string
}

func faultCampaignMix64V1(value uint64) uint64 {
	value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9
	value = (value ^ (value >> 27)) * 0x94d049bb133111eb
	return value ^ (value >> 31)
}

// fieldDomain is fixed by v1: 0=seed, 1=principal, 2=fee mode,
// 3=fee amount, 4=local retries, 5=Treasury retries, 6=lost-ack mask.
func faultCampaignWordV1(sequence, fieldDomain uint64) uint64 {
	return faultCampaignMix64V1(
		faultCampaignRootSeedV1 ^
			sequence*faultCampaignSequenceV1 ^
			fieldDomain*faultCampaignDomainV1,
	)
}

func faultCampaignPlanForSequenceV1(sequence uint64) faultCampaignPlanV1 {
	seed := faultCampaignWordV1(sequence, 0)
	amount := uint64(1) + faultCampaignWordV1(sequence, 1)%1_000_000
	fee := uint64(0)
	if faultCampaignWordV1(sequence, 2)%5 != 0 {
		fee = uint64(1) + faultCampaignWordV1(sequence, 3)%500
	}
	localRetries := uint8(faultCampaignWordV1(sequence, 4) % 4)
	treasuryRetries := uint8(0)
	if fee > 0 {
		treasuryRetries = uint8(faultCampaignWordV1(sequence, 5) % 4)
	}
	ackMask := uint8(faultCampaignWordV1(sequence, 6) & 0b11)
	if fee == 0 {
		ackMask &^= faultCampaignTreasuryAck
	}
	return faultCampaignPlanV1{
		Sequence:                     sequence,
		Seed:                         seed,
		AmountMinor:                  amount,
		FeeMinor:                     fee,
		LocalIndeterminateRetries:    localRetries,
		TreasuryIndeterminateRetries: treasuryRetries,
		LostCommitAckMask:            ackMask,
	}
}

func (p faultCampaignPlanV1) katLine() string {
	return fmt.Sprintf(
		"%d %016x %d %d %d %d %d",
		p.Sequence,
		p.Seed,
		p.AmountMinor,
		p.FeeMinor,
		p.LocalIndeterminateRetries,
		p.TreasuryIndeterminateRetries,
		p.LostCommitAckMask,
	)
}

func faultCampaignDigest(seed uint64) string {
	return fmt.Sprintf("sha256:%064x", seed)
}

func faultCampaignLocalEvidence(journal *Journal, plan faultCampaignPlanV1) Evidence {
	label := fmt.Sprintf("campaign-%d-local", plan.Sequence)
	outboxID := ""
	if journal.intent.FeeMinor > 0 {
		outboxID = journal.EffectID(TreasuryFee)
	}
	evidence := Evidence{
		OperationID:    journal.operationID,
		EffectID:       journal.EffectID(LocalPaymentTransaction),
		Leg:            LocalPaymentTransaction,
		SenderDID:      journal.intent.SenderDID,
		RecipientDID:   journal.intent.RecipientDID,
		PrincipalMinor: journal.intent.AmountMinor,
		FeeMinor:       journal.intent.FeeMinor,
		Currency:       journal.intent.Currency,
		Local: &LocalPaymentEvidence{
			SenderAccountID:            journal.intent.SenderDID,
			RecipientAccountID:         journal.intent.RecipientDID,
			SenderDebitMinor:           journal.intent.AmountMinor + journal.intent.FeeMinor,
			RecipientCreditMinor:       journal.intent.AmountMinor,
			SenderPredecessorAction:    "action:sender-before-" + label,
			SenderDebitAction:          "action:sender-debit-" + label,
			RecipientPredecessorAction: "action:recipient-before-" + label,
			RecipientCreditAction:      "action:recipient-credit-" + label,
			PaymentActionHash:          "action:payment-" + label,
			PaymentStatus:              "Completed",
			ReceiptActionHash:          "action:receipt-" + label,
			ReceiptSignerDID:           journal.intent.SenderDID,
			SigningProfile:             ReceiptProfileV1,
			SignatureDigest:            faultCampaignDigest(plan.Seed ^ 0x4c4f43414c31),
			TreasuryFeeOutboxID:         outboxID,
		},
	}
	digest, err := canonicalEvidenceDigest(evidence)
	if err != nil {
		panic(err)
	}
	evidence.EvidenceDigest = digest
	return evidence
}

func faultCampaignTreasuryEvidence(journal *Journal, plan faultCampaignPlanV1) Evidence {
	label := fmt.Sprintf("campaign-%d-treasury", plan.Sequence)
	evidence := Evidence{
		OperationID:    journal.operationID,
		EffectID:       journal.EffectID(TreasuryFee),
		Leg:            TreasuryFee,
		SenderDID:      journal.intent.SenderDID,
		RecipientDID:   journal.intent.RecipientDID,
		PrincipalMinor: journal.intent.AmountMinor,
		FeeMinor:       journal.intent.FeeMinor,
		Currency:       journal.intent.Currency,
		Treasury: &TreasuryFeeEvidence{
			PoolID:            "treasury:global-fee-pool",
			SourceMemberDID:   journal.intent.SenderDID,
			AmountMinor:       journal.intent.FeeMinor,
			PaymentActionHash: journal.localPaymentActionHash(),
			IdempotencyKey:    journal.EffectID(TreasuryFee),
			PredecessorAction: "action:treasury-before-" + label,
			TreasuryAction:    "action:treasury-fee-" + label,
		},
	}
	digest, err := canonicalEvidenceDigest(evidence)
	if err != nil {
		panic(err)
	}
	evidence.EvidenceDigest = digest
	return evidence
}

func runFaultCampaignPlanV1(t *testing.T, plan faultCampaignPlanV1) faultCampaignOutcomeV1 {
	t.Helper()
	sender := "did:mycelix:campaign-sender"
	recipient := "did:mycelix:campaign-recipient"
	idempotencyKey := fmt.Sprintf("fault-campaign-v1-%d", plan.Sequence)
	intent := Intent{
		SenderDID:            sender,
		RecipientDID:         recipient,
		ClientIdempotencyKey: idempotencyKey,
		PayloadFingerprint:   faultCampaignDigest(plan.Seed),
		AmountMinor:          plan.AmountMinor,
		FeeMinor:             plan.FeeMinor,
		Currency:             "SAP",
	}

	registry := NewRegistry()
	journal, _, err := registry.CreateOrGet(intent)
	if err != nil {
		t.Fatalf("seed %d create: %v", plan.Sequence, err)
	}
	eventLog, err := NewJournalEventLog(intent)
	if err != nil {
		t.Fatalf("seed %d create event log: %v", plan.Sequence, err)
	}

	if err := journal.Authorize(); err != nil {
		t.Fatalf("seed %d authorize: %v", plan.Sequence, err)
	}
	if err := eventLog.RecordAuthorized(); err != nil {
		t.Fatalf("seed %d record authorize: %v", plan.Sequence, err)
	}

	localID := journal.EffectID(LocalPaymentTransaction)
	if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
		t.Fatalf("seed %d prepare local: %v", plan.Sequence, err)
	}
	if err := eventLog.RecordEffectPrepared(LocalPaymentTransaction, localID); err != nil {
		t.Fatalf("seed %d log local prepare: %v", plan.Sequence, err)
	}

	for retry := uint8(0); retry < plan.LocalIndeterminateRetries; retry++ {
		if err := journal.MarkIndeterminate(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d local indeterminate %d: %v", plan.Sequence, retry, err)
		}
		if err := eventLog.RecordEffectIndeterminate(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d log local indeterminate %d: %v", plan.Sequence, retry, err)
		}
		if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d local same-effect retry %d: %v", plan.Sequence, retry, err)
		}
		if err := eventLog.RecordEffectPrepared(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d log local same-effect retry %d: %v", plan.Sequence, retry, err)
		}
	}

	localEvidence := faultCampaignLocalEvidence(journal, plan)
	if _, err := journal.CommitLeg(LocalPaymentTransaction, localEvidence); err != nil {
		t.Fatalf("seed %d local commit: %v", plan.Sequence, err)
	}
	if err := eventLog.RecordEffectCommitted(localEvidence); err != nil {
		t.Fatalf("seed %d log local commit: %v", plan.Sequence, err)
	}

	// Independent integer-unit conservation oracle. A fee of zero has no
	// Treasury leg; otherwise the local debit must equal principal + fee.
	if localEvidence.Local.SenderDebitMinor !=
		localEvidence.Local.RecipientCreditMinor+plan.FeeMinor {
		t.Fatalf("seed %d unexplained local conservation delta", plan.Sequence)
	}
	if localEvidence.Local.SenderDebitMinor != plan.AmountMinor+plan.FeeMinor {
		t.Fatalf("seed %d local debit does not equal principal plus fee", plan.Sequence)
	}

	if plan.LostCommitAckMask&faultCampaignLocalAckLost != 0 {
		if err := journal.MarkIndeterminate(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d lost local ack: %v", plan.Sequence, err)
		}
		if err := eventLog.RecordEffectIndeterminate(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d log lost local ack: %v", plan.Sequence, err)
		}
		if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d observe committed local effect: %v", plan.Sequence, err)
		}
		if err := eventLog.RecordEffectPrepared(LocalPaymentTransaction, localID); err != nil {
			t.Fatalf("seed %d log committed local observation: %v", plan.Sequence, err)
		}
		if _, err := journal.CommitLeg(LocalPaymentTransaction, localEvidence); err != nil {
			t.Fatalf("seed %d local exact retry: %v", plan.Sequence, err)
		}
		if err := eventLog.RecordEffectCommitted(localEvidence); err != nil {
			t.Fatalf("seed %d log local exact retry: %v", plan.Sequence, err)
		}
	}

	treasuryID := journal.EffectID(TreasuryFee)
	if plan.FeeMinor > 0 {
		if _, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil {
			t.Fatalf("seed %d prepare Treasury: %v", plan.Sequence, err)
		}
		if err := eventLog.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
			t.Fatalf("seed %d log Treasury prepare: %v", plan.Sequence, err)
		}
		for retry := uint8(0); retry < plan.TreasuryIndeterminateRetries; retry++ {
			if err := journal.MarkIndeterminate(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d Treasury indeterminate %d: %v", plan.Sequence, retry, err)
			}
			if err := eventLog.RecordEffectIndeterminate(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d log Treasury indeterminate %d: %v", plan.Sequence, retry, err)
			}
			if _, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d Treasury same-effect retry %d: %v", plan.Sequence, retry, err)
			}
			if err := eventLog.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d log Treasury same-effect retry %d: %v", plan.Sequence, retry, err)
			}
		}

		treasuryEvidence := faultCampaignTreasuryEvidence(journal, plan)
		if _, err := journal.CommitLeg(TreasuryFee, treasuryEvidence); err != nil {
			t.Fatalf("seed %d Treasury commit: %v", plan.Sequence, err)
		}
		if err := eventLog.RecordEffectCommitted(treasuryEvidence); err != nil {
			t.Fatalf("seed %d log Treasury commit: %v", plan.Sequence, err)
		}
		if treasuryEvidence.Treasury.AmountMinor != plan.FeeMinor {
			t.Fatalf("seed %d Treasury credit does not match exact fee", plan.Sequence)
		}
		if localEvidence.Local.RecipientCreditMinor+treasuryEvidence.Treasury.AmountMinor !=
			localEvidence.Local.SenderDebitMinor {
			t.Fatalf("seed %d principal + Treasury fee do not conserve integer units", plan.Sequence)
		}

		if plan.LostCommitAckMask&faultCampaignTreasuryAck != 0 {
			if err := journal.MarkIndeterminate(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d lost Treasury ack: %v", plan.Sequence, err)
			}
			if err := eventLog.RecordEffectIndeterminate(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d log lost Treasury ack: %v", plan.Sequence, err)
			}
			if _, err := journal.PrepareLeg(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d observe committed Treasury effect: %v", plan.Sequence, err)
			}
			if err := eventLog.RecordEffectPrepared(TreasuryFee, treasuryID); err != nil {
				t.Fatalf("seed %d log committed Treasury observation: %v", plan.Sequence, err)
			}
			if _, err := journal.CommitLeg(TreasuryFee, treasuryEvidence); err != nil {
				t.Fatalf("seed %d Treasury exact retry: %v", plan.Sequence, err)
			}
			if err := eventLog.RecordEffectCommitted(treasuryEvidence); err != nil {
				t.Fatalf("seed %d log Treasury exact retry: %v", plan.Sequence, err)
			}
		}
	}

	if got := journal.Status(); got != Completed {
		t.Fatalf("seed %d expected Completed after all required effects, got %s", plan.Sequence, got)
	}

	snapshot := eventLog.Frames()
	anchor := eventLog.Anchor()
	replayed, err := ReplayJournal(snapshot, anchor)
	if err != nil {
		t.Fatalf("seed %d replay: %v", plan.Sequence, err)
	}
	if !reflectDeepEqualFrames(snapshot, eventLog.Frames()) {
		t.Fatalf("seed %d replay mutated stored frames", plan.Sequence)
	}
	if replayed.Status() != journal.Status() {
		t.Fatalf("seed %d replay status differs: %s / %s", plan.Sequence, replayed.Status(), journal.Status())
	}
	if replayed.OperationID() != journal.OperationID() {
		t.Fatalf("seed %d replay operation identity differs", plan.Sequence)
	}
	if !reflectDeepEqualEffectState(
		replayed.EffectState(LocalPaymentTransaction),
		journal.EffectState(LocalPaymentTransaction),
	) {
		t.Fatalf("seed %d replay local effect state differs", plan.Sequence)
	}
	if !reflectDeepEqualEffectState(
		replayed.EffectState(TreasuryFee),
		journal.EffectState(TreasuryFee),
	) {
		t.Fatalf("seed %d replay Treasury effect state differs", plan.Sequence)
	}

	outcome := faultCampaignOutcomeV1{
		OperationID:      journal.OperationID(),
		LocalEffectID:    localID,
		Status:           journal.Status(),
		EventCount:       anchor.EventCount,
		TailDigest:       anchor.TailDigest,
	}
	if plan.FeeMinor > 0 {
		outcome.TreasuryEffectID = treasuryID
	}
	return outcome
}

func reflectDeepEqualFrames(left, right []JournalEventFrame) bool {
	if len(left) != len(right) {
		return false
	}
	for index := range left {
		if left[index].Sequence != right[index].Sequence ||
			left[index].PreviousDigest != right[index].PreviousDigest ||
			left[index].EventDigest != right[index].EventDigest {
			return false
		}
	}
	return true
}

func reflectDeepEqualEffectState(left, right EffectState) bool {
	if left.Status != right.Status || left.EffectID != right.EffectID {
		return false
	}
	if left.Evidence == nil || right.Evidence == nil {
		return left.Evidence == nil && right.Evidence == nil
	}
	leftDigest, leftErr := canonicalEvidenceDigest(*left.Evidence)
	rightDigest, rightErr := canonicalEvidenceDigest(*right.Evidence)
	return leftErr == nil && rightErr == nil && leftDigest == rightDigest
}

func faultCampaignSummaryText(hasher interface{ Write([]byte) (int, error) }, value string) {
	encoded := []byte(value)
	var length [4]byte
	binary.BigEndian.PutUint32(length[:], uint32(len(encoded)))
	_, _ = hasher.Write(length[:])
	_, _ = hasher.Write(encoded)
}

func faultCampaignSummaryV1(traceCount uint64, t *testing.T) string {
	t.Helper()
	hasher := sha256.New()
	_, _ = hasher.Write([]byte("MXSAP-FAULT-CAMPAIGN-RESULT-V1\x00"))

	for sequence := uint64(0); sequence < traceCount; sequence++ {
		plan := faultCampaignPlanForSequenceV1(sequence)
		outcome := runFaultCampaignPlanV1(t, plan)
		var word [8]byte

		for _, value := range []uint64{
			plan.Sequence,
			plan.Seed,
			plan.AmountMinor,
			plan.FeeMinor,
		} {
			binary.BigEndian.PutUint64(word[:], value)
			_, _ = hasher.Write(word[:])
		}
		_, _ = hasher.Write([]byte{
			plan.LocalIndeterminateRetries,
			plan.TreasuryIndeterminateRetries,
			plan.LostCommitAckMask,
		})
		faultCampaignSummaryText(hasher, outcome.OperationID)
		faultCampaignSummaryText(hasher, outcome.LocalEffectID)
		faultCampaignSummaryText(hasher, outcome.TreasuryEffectID)
		faultCampaignSummaryText(hasher, string(outcome.Status))
		binary.BigEndian.PutUint64(word[:], outcome.EventCount)
		_, _ = hasher.Write(word[:])
		faultCampaignSummaryText(hasher, outcome.TailDigest)
	}
	return hex.EncodeToString(hasher.Sum(nil))
}

func TestFaultCampaignPlanKnownAnswerVectors(t *testing.T) {
	data, err := os.ReadFile("fixtures/sap-payment-saga-fault-campaign-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	found := 0
	for _, rawLine := range strings.Split(string(data), "\n") {
		line := strings.TrimSpace(rawLine)
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		fields := strings.Fields(line)
		if len(fields) != 7 {
			t.Fatalf("KAT line must have 7 fields, got %d: %q", len(fields), line)
		}
		sequence, err := strconv.ParseUint(fields[0], 10, 64)
		if err != nil {
			t.Fatalf("invalid KAT sequence: %v", err)
		}
		if got := faultCampaignPlanForSequenceV1(sequence).katLine(); got != line {
			t.Fatalf("sequence %d KAT mismatch:\n got %s\nwant %s", sequence, got, line)
		}
		found++
	}
	if found < 8 {
		t.Fatalf("expected at least eight shared KAT vectors, found %d", found)
	}
}

func TestDeterministicPresubmitFaultCampaignReplaysTenThousandTraces(t *testing.T) {
	summary := faultCampaignSummaryV1(faultCampaignPresubmit, t)
	fmt.Printf("fault_campaign_go_summary_10000=%s\n", summary)
}

func TestDeterministicFullMillionFaultCampaign(t *testing.T) {
	if os.Getenv("MYCELIX_FULL_CAMPAIGN") != "1" {
		t.Skip("set MYCELIX_FULL_CAMPAIGN=1 for the one-million-trace campaign")
	}
	summary := faultCampaignSummaryV1(faultCampaignFull, t)
	fmt.Printf("fault_campaign_go_summary_1000000=%s\n", summary)
}
