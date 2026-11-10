package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

func checkpointTestIntent() Intent {
	return Intent{
		SenderDID:            "did:sender",
		RecipientDID:         "did:recipient",
		ClientIdempotencyKey: "client-key-001",
		PayloadFingerprint:   "sha256:" + strings.Repeat("a", 64),
		AmountMinor:          100,
		FeeMinor:             3,
		Currency:             "SAP",
	}
}

func checkpointTestSubject() CheckpointSubjectV1 {
	return CheckpointSubjectV1{
		OperationID:           "sap-payment:10:did:sender:14:client-key-001",
		IntentDigest:          "sha256:5a6e4c29cb95cee151d9f41ac9a82b6b91f3f4b443f2902a6026c769466f5b07",
		Owner:                 CheckpointPayments,
		EventLogID:            "journal:did:sender:payment:001",
		EventCount:            8,
		TailDigest:            "sha256:a1e5b966be2b6aab453c65370639d702e6d83b3097c9a542a0d85d28bf6ed899",
		CheckpointSequence:    1,
		SourceActionHash:      "uhCkk-source-payment-action",
		SourceChainFrontier:   "uhCkk-source-chain-frontier",
		IssuerDID:             "did:mycelix:agent:sender",
		VerificationMethod:    "did:mycelix:agent:sender#holochain-key-1",
		AuthorityGrantRef:     "uhCkk-payments-authority-grant",
		IssuedAtUnixMS:        1794316800000,
	}
}

func checkpointVectors(t *testing.T) map[string]string {
	t.Helper()
	raw, err := os.ReadFile("fixtures/sap-payment-journal-checkpoint-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	vectors := make(map[string]string)
	for lineNumber, line := range strings.Split(string(raw), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		key, value, ok := strings.Cut(line, "=")
		if !ok || key == "" || value == "" {
			t.Fatalf("invalid checkpoint fixture line %d", lineNumber+1)
		}
		if _, exists := vectors[key]; exists {
			t.Fatalf("duplicate checkpoint fixture key %q", key)
		}
		vectors[key] = value
	}
	if len(vectors) != 6 {
		t.Fatalf("fixture has %d values, want exactly 6", len(vectors))
	}
	return vectors
}

func TestCheckpointSubjectMatchesSharedKnownAnswer(t *testing.T) {
	vectors := checkpointVectors(t)
	if vectors["profile"] != "MXSAP-JOURNAL-CHECKPOINT-SUBJECT-V1" {
		t.Fatalf("unexpected checkpoint profile %q", vectors["profile"])
	}
	intent := checkpointTestIntent()
	if vectors["intent-profile"] != "MXSAP-PAYMENT-INTENT-V1" {
		t.Fatalf("unexpected intent profile %q", vectors["intent-profile"])
	}
	intentBytes, err := CanonicalIntentBytes(intent)
	if err != nil {
		t.Fatal(err)
	}
	if got := hexString(intentBytes); got != vectors["intent-bytes-hex"] {
		t.Fatalf("intent bytes mismatch:\n got %s\nwant %s", got, vectors["intent-bytes-hex"])
	}
	intentDigest, err := CanonicalIntentDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	if intentDigest != vectors["intent-digest"] {
		t.Fatalf("intent digest = %s, want %s", intentDigest, vectors["intent-digest"])
	}

	subject := checkpointTestSubject()
	if err := subject.ValidateAgainstIntent(intent); err != nil {
		t.Fatalf("subject does not bind canonical intent: %v", err)
	}
	encoded, err := subject.CanonicalBytes()
	if err != nil {
		t.Fatal(err)
	}
	if got := hexString(encoded); got != vectors["subject-bytes-hex"] {
		t.Fatalf("canonical bytes mismatch:\n got %s\nwant %s", got, vectors["subject-bytes-hex"])
	}
	digest, err := subject.CanonicalDigest()
	if err != nil {
		t.Fatal(err)
	}
	if digest != vectors["subject-digest"] {
		t.Fatalf("canonical digest = %s, want %s", digest, vectors["subject-digest"])
	}
}

func TestCheckpointRejectsSubjectIntentSubstitution(t *testing.T) {
	intent := checkpointTestIntent()
	subject := checkpointTestSubject()

	subject.OperationID += "x"
	if err := subject.ValidateAgainstIntent(intent); err != ErrCheckpointIntentMismatch {
		t.Fatalf("changed operation ID returned %v", err)
	}

	subject = checkpointTestSubject()
	subject.IntentDigest = "sha256:" + strings.Repeat("2", 64)
	if err := subject.ValidateAgainstIntent(intent); err != ErrCheckpointIntentMismatch {
		t.Fatalf("changed intent digest returned %v", err)
	}
}

func TestCheckpointRejectsDifferentImmutableIntent(t *testing.T) {
	subject := checkpointTestSubject()
	mutations := []func(*Intent){
		func(i *Intent) { i.RecipientDID = "did:other-recipient" },
		func(i *Intent) { i.ClientIdempotencyKey = "client-key-002" },
		func(i *Intent) { i.PayloadFingerprint = "sha256:" + strings.Repeat("b", 64) },
		func(i *Intent) { i.AmountMinor++ },
		func(i *Intent) { i.FeeMinor++ },
		func(i *Intent) { i.Currency = "EUR" },
	}
	for index, mutate := range mutations {
		intent := checkpointTestIntent()
		mutate(&intent)
		if err := subject.ValidateAgainstIntent(intent); err != ErrCheckpointIntentMismatch {
			t.Fatalf("intent mutation %d returned %v", index, err)
		}
	}
}

func TestEveryCheckpointSubjectDimensionChangesDigest(t *testing.T) {
	original, err := checkpointTestSubject().CanonicalDigest()
	if err != nil {
		t.Fatal(err)
	}
	mutations := []func(*CheckpointSubjectV1){
		func(s *CheckpointSubjectV1) { s.OperationID += "x" },
		func(s *CheckpointSubjectV1) { s.IntentDigest = "sha256:" + strings.Repeat("22", 32) },
		func(s *CheckpointSubjectV1) { s.Owner = CheckpointTreasury },
		func(s *CheckpointSubjectV1) { s.EventLogID += "x" },
		func(s *CheckpointSubjectV1) { s.EventCount++ },
		func(s *CheckpointSubjectV1) { s.TailDigest = "sha256:" + strings.Repeat("22", 32) },
		func(s *CheckpointSubjectV1) {
			s.CheckpointSequence = 2
			s.PriorCheckpointDigest = "sha256:" + strings.Repeat("33", 32)
		},
		func(s *CheckpointSubjectV1) {
			s.CheckpointSequence = 2
			s.PriorCheckpointDigest = "sha256:" + strings.Repeat("44", 32)
		},
		func(s *CheckpointSubjectV1) { s.SourceActionHash += "x" },
		func(s *CheckpointSubjectV1) { s.SourceChainFrontier += "x" },
		func(s *CheckpointSubjectV1) { s.IssuerDID += "x" },
		func(s *CheckpointSubjectV1) { s.VerificationMethod += "x" },
		func(s *CheckpointSubjectV1) { s.AuthorityGrantRef += "x" },
		func(s *CheckpointSubjectV1) { s.IssuedAtUnixMS++ },
	}
	for index, mutate := range mutations {
		subject := checkpointTestSubject()
		mutate(&subject)
		got, err := subject.CanonicalDigest()
		if err != nil {
			t.Fatalf("mutation %d invalid: %v", index, err)
		}
		if got == original {
			t.Fatalf("mutation %d did not change digest", index)
		}
	}
}

func TestCheckpointSubjectRejectsInvalidInputs(t *testing.T) {
	subject := checkpointTestSubject()
	subject.IntentDigest = "sha256:ABC"
	if err := subject.Validate(); err != ErrCheckpointInvalidDigest {
		t.Fatalf("invalid digest error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.CheckpointSequence = 2
	if err := subject.Validate(); err != ErrCheckpointInvalidSequence {
		t.Fatalf("missing prior digest error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.PriorCheckpointDigest = "sha256:" + strings.Repeat("22", 32)
	if err := subject.Validate(); err != ErrCheckpointInvalidSequence {
		t.Fatalf("unexpected prior digest error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.EventCount = 0
	if err := subject.Validate(); err != ErrCheckpointInvalidSequence {
		t.Fatalf("zero event count error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.AuthorityGrantRef = ""
	if err := subject.Validate(); err != ErrCheckpointInvalidSubject {
		t.Fatalf("empty authority grant error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.SourceActionHash = strings.Repeat("x", maxCheckpointFieldBytes+1)
	if err := subject.Validate(); err != ErrCheckpointInvalidSubject {
		t.Fatalf("oversized action hash error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.Owner = CheckpointOwner(9)
	if err := subject.Validate(); err != ErrCheckpointInvalidSubject {
		t.Fatalf("unknown owner error = %v", err)
	}

	subject = checkpointTestSubject()
	subject.IssuedAtUnixMS = 0
	if err := subject.Validate(); err != ErrCheckpointInvalidTimestamp {
		t.Fatalf("zero issued-at error = %v", err)
	}
}

func TestCheckpointSequenceRequiresContiguousPriorReference(t *testing.T) {
	subject := checkpointTestSubject()
	subject.CheckpointSequence = 2
	subject.PriorCheckpointDigest = "sha256:" + strings.Repeat("44", 32)
	if err := subject.Validate(); err != nil {
		t.Fatal(err)
	}
	subject.CheckpointSequence = 1
	if err := subject.Validate(); err != ErrCheckpointInvalidSequence {
		t.Fatalf("sequence one with prior reference error = %v", err)
	}
}

func hexString(value []byte) string {
	const digits = "0123456789abcdef"
	output := make([]byte, len(value)*2)
	for index, byteValue := range value {
		output[index*2] = digits[byteValue>>4]
		output[index*2+1] = digits[byteValue&0x0f]
	}
	return string(output)
}
