package sagaverifier

import (
	"bytes"
	"encoding/hex"
	"os"
	"strings"
	"testing"
)

func checkpointTestActionHash(core, location byte) []byte {
	hash := []byte{0x84, 0x29, 0x24}
	for i := 0; i < 32; i++ {
		hash = append(hash, core)
	}
	for i := 0; i < 4; i++ {
		hash = append(hash, location)
	}
	return hash
}

func checkpointTestSubject(intent Intent, owner CheckpointOwnerDomain) JournalCheckpointSubject {
	operationID := OperationID(intent.SenderDID, intent.ClientIdempotencyKey)
	intentDigest, err := CanonicalPaymentIntentDigest(intent)
	if err != nil {
		panic(err)
	}
	subject := JournalCheckpointSubject{
		OwnerDomain:              owner,
		OperationID:              operationID,
		IntentDigest:             intentDigest,
		EventCount:               8,
		TailDigest:               "sha256:a1e5b966be2b6aab453c65370639d702e6d83b3097c9a542a0d85d28bf6ed899",
		CheckpointSequence:       1,
		SourceActionHash:         checkpointTestActionHash(0x11, 0x22),
		SourceActionSequence:     42,
		SourcePreviousActionHash: checkpointTestActionHash(0x33, 0x44),
		HasSourcePreviousAction:  true,
	}
	if owner == CheckpointPayments {
		subject.OwnerDID = "did:mycelix:payments:alice"
		subject.EventLogID = "sap-journal:payments:op-001"
	} else {
		subject.OwnerDID = "did:mycelix:treasury:community"
		subject.EventLogID = "sap-journal:treasury:op-001"
		subject.EventCount = 2
		subject.TailDigest = "sha256:98bea9e9428e50e2fb522a6d8e7800d3cbc9e32ea068c5b23313f459d9e13f27"
		subject.SourceActionHash = checkpointTestActionHash(0x55, 0x66)
		subject.SourceActionSequence = 101
		subject.SourcePreviousActionHash = checkpointTestActionHash(0x77, 0x88)
	}
	return subject
}

func checkpointTestVectors(t *testing.T) map[string]string {
	t.Helper()
	data, err := os.ReadFile("fixtures/sap-payment-journal-checkpoint-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	vectors := make(map[string]string)
	for _, line := range strings.Split(string(data), "\n") {
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		name, value, ok := strings.Cut(line, "=")
		if !ok || name == "" || value == "" {
			t.Fatalf("invalid checkpoint known-answer line %q", line)
		}
		if _, exists := vectors[name]; exists {
			t.Fatalf("duplicate checkpoint known-answer key %q", name)
		}
		vectors[name] = value
	}
	for _, key := range []string{
		"intent-digest",
		"payments-subject-hex", "payments-subject-digest",
		"treasury-subject-hex", "treasury-subject-digest",
		"treasury-next-subject-hex", "treasury-next-subject-digest",
	} {
		if vectors[key] == "" {
			t.Fatalf("missing checkpoint known-answer key %q", key)
		}
	}
	return vectors
}

func TestCheckpointKnownAnswerVectorsMatchCanonicalIntentAndSubjects(t *testing.T) {
	vectors := checkpointTestVectors(t)
	intent := replayTestIntent()
	intentDigest, err := CanonicalPaymentIntentDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	if intentDigest != vectors["intent-digest"] {
		t.Fatalf("intent digest = %s, want %s", intentDigest, vectors["intent-digest"])
	}

	for name, owner := range map[string]CheckpointOwnerDomain{
		"payments": CheckpointPayments,
		"treasury": CheckpointTreasury,
	} {
		subject := checkpointTestSubject(intent, owner)
		actual, err := subject.CanonicalBytes()
		if err != nil {
			t.Fatalf("%s canonical bytes: %v", name, err)
		}
		expected, err := hex.DecodeString(vectors[name+"-subject-hex"])
		if err != nil {
			t.Fatal(err)
		}
		if !bytes.Equal(actual, expected) {
			t.Fatalf("%s subject bytes differ from shared vector", name)
		}
		digest, err := subject.CanonicalDigest()
		if err != nil {
			t.Fatal(err)
		}
		if digest != vectors[name+"-subject-digest"] {
			t.Fatalf("%s digest = %s, want %s", name, digest, vectors[name+"-subject-digest"])
		}
	}
}

func TestTreasuryCheckpointSequenceBindsPriorCheckpointAndSourceFrontier(t *testing.T) {
	vectors := checkpointTestVectors(t)
	intent := replayTestIntent()
	first := checkpointTestSubject(intent, CheckpointTreasury)
	firstDigest, err := first.CanonicalDigest()
	if err != nil {
		t.Fatal(err)
	}
	if firstDigest != vectors["treasury-subject-digest"] {
		t.Fatalf("first Treasury digest = %s, want %s", firstDigest, vectors["treasury-subject-digest"])
	}

	next := first
	next.EventCount = 3
	next.CheckpointSequence = 2
	next.PreviousCheckpointDigest = firstDigest
	next.HasPreviousCheckpoint = true
	next.SourceActionHash = checkpointTestActionHash(0x99, 0xaa)
	next.SourceActionSequence = 102
	next.SourcePreviousActionHash = append([]byte(nil), first.SourceActionHash...)
	next.HasSourcePreviousAction = true

	actual, err := next.CanonicalBytes()
	if err != nil {
		t.Fatal(err)
	}
	expected, err := hex.DecodeString(vectors["treasury-next-subject-hex"])
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(actual, expected) {
		t.Fatal("continued Treasury checkpoint bytes differ from shared vector")
	}
	digest, err := next.CanonicalDigest()
	if err != nil {
		t.Fatal(err)
	}
	if digest != vectors["treasury-next-subject-digest"] {
		t.Fatalf("continued Treasury digest = %s, want %s", digest, vectors["treasury-next-subject-digest"])
	}
}

func TestCheckpointOwnerAndEvidenceMutationsChangeTheSubjectDigest(t *testing.T) {
	intent := replayTestIntent()
	original := checkpointTestSubject(intent, CheckpointPayments)
	baseline, err := original.CanonicalDigest()
	if err != nil {
		t.Fatal(err)
	}

	mutations := []func(*JournalCheckpointSubject){
		func(s *JournalCheckpointSubject) { s.OwnerDomain = CheckpointTreasury },
		func(s *JournalCheckpointSubject) { s.OwnerDID += ":substituted" },
		func(s *JournalCheckpointSubject) { s.OperationID += ":other" },
		func(s *JournalCheckpointSubject) { s.EventCount++ },
		func(s *JournalCheckpointSubject) {
			s.TailDigest = "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
		},
		func(s *JournalCheckpointSubject) { s.SourceActionHash[10] ^= 1 },
	}
	for index, mutate := range mutations {
		changed := original
		changed.SourceActionHash = append([]byte(nil), original.SourceActionHash...)
		mutate(&changed)
		digest, err := changed.CanonicalDigest()
		if err != nil {
			t.Fatalf("mutation %d rejected unexpectedly: %v", index, err)
		}
		if digest == baseline {
			t.Fatalf("mutation %d did not change the subject commitment", index)
		}
	}
}

func TestCheckpointRejectsMalformedOwnerDigestAndActionFrontiers(t *testing.T) {
	intent := replayTestIntent()
	base := checkpointTestSubject(intent, CheckpointPayments)
	tests := []struct {
		name string
		edit func(*JournalCheckpointSubject)
		want error
	}{
		{"unknown-owner", func(s *JournalCheckpointSubject) { s.OwnerDomain = 0 }, ErrCheckpointInvalidSubject},
		{"non-DID-owner", func(s *JournalCheckpointSubject) { s.OwnerDID = "alice" }, ErrCheckpointInvalidSubject},
		{"empty-history", func(s *JournalCheckpointSubject) { s.EventCount = 0 }, ErrCheckpointInvalidSequence},
		{"uppercase-digest", func(s *JournalCheckpointSubject) { s.IntentDigest = "SHA256:" + strings.Repeat("a", 64) }, ErrCheckpointInvalidDigest},
		{"missing-prior-checkpoint", func(s *JournalCheckpointSubject) { s.CheckpointSequence = 2 }, ErrCheckpointInvalidSequence},
		{"genesis-with-prior-checkpoint", func(s *JournalCheckpointSubject) {
			s.PreviousCheckpointDigest = "sha256:" + strings.Repeat("b", 64)
			s.HasPreviousCheckpoint = true
		}, ErrCheckpointInvalidSequence},
		{"wrong-action-hash-type", func(s *JournalCheckpointSubject) { s.SourceActionHash[0] = 0 }, ErrCheckpointInvalidActionHash},
		{"missing-source-predecessor", func(s *JournalCheckpointSubject) { s.SourceActionSequence = 0 }, ErrCheckpointInvalidSourceFrontier},
		{"overlong-event-log-id", func(s *JournalCheckpointSubject) { s.EventLogID = strings.Repeat("x", maxCheckpointTextBytes+1) }, ErrCheckpointInvalidSubject},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			subject := base
			subject.SourceActionHash = append([]byte(nil), base.SourceActionHash...)
			test.edit(&subject)
			if err := subject.Validate(); err != test.want {
				t.Fatalf("Validate() error = %v, want %v", err, test.want)
			}
		})
	}
}
