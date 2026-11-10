package sagaverifier

import (
	"os"
	"strings"
	"testing"
)

func checkpointVector(name string) string {
	data, err := os.ReadFile("fixtures/sap-payment-journal-checkpoint-v1.kat")
	if err != nil {
		panic(err)
	}
	for _, line := range strings.Split(string(data), "\\n") {
		if strings.HasPrefix(line, name+"=") {
			return strings.TrimPrefix(line, name+"=")
		}
	}
	panic("missing checkpoint known-answer vector: " + name)
}

func checkpointTestIntent() Intent {
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

func checkpointTestSubject() JournalCheckpointSubjectV1 {
	return JournalCheckpointSubjectV1{
		Owner:                    CheckpointPayments,
		Lifecycle:                CheckpointCurrent,
		OperationID:              "sap-payment:10:did:sender:14:client-key-001",
		IntentDigest:             "sha256:4226d08540bfc19a3dfd9249480cfe21d1358c2b6153919ec137f691ebd46348",
		EventLogID:               "sap-payment-journal:payments:operation-1",
		EventCount:               4,
		TailDigest:               "sha256:5c34988af074f1a2f8a387a16e58d1121bc888426b2d5989cd8904f7f279684f",
		CheckpointSequence:       1,
		SourceActionHash:         "uhCkkPaymentsActionHashExample",
		IssuerDID:                "did:mycelix:payments-operator",
		VerificationMethod:       "did:mycelix:payments-operator#agent-key-1",
		AuthorityProfileDigest:   "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
		IssuedAtUnixMicros:       1794326400000000,
	}
}

func TestCanonicalCheckpointMatchesSharedCrossLanguageVector(t *testing.T) {
	intent := checkpointTestIntent()
	intentDigest, err := CanonicalPaymentIntentDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	if want := checkpointVector("intent-digest"); intentDigest != want {
		t.Fatalf("intent digest = %s, want %s", intentDigest, want)
	}
	got, err := checkpointTestSubject().CanonicalDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	want := checkpointVector("payments-checkpoint-digest")
	if got != want {
		t.Fatalf("checkpoint digest = %s, want %s", got, want)
	}
}

func TestCheckpointOwnerAndSubjectFieldsAreBound(t *testing.T) {
	intent := checkpointTestIntent()
	original, err := checkpointTestSubject().CanonicalDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	changed := checkpointTestSubject()
	changed.Owner = CheckpointTreasury
	got, err := changed.CanonicalDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	if got == original {
		t.Fatal("owner domain was not bound into checkpoint digest")
	}
	changed = checkpointTestSubject()
	changed.IssuerDID += "-changed"
	got, err = changed.CanonicalDigest(intent)
	if err != nil {
		t.Fatal(err)
	}
	if got == original {
		t.Fatal("issuer identity was not bound into checkpoint digest")
	}
}

func TestCheckpointRejectsWrongOperationOrIntent(t *testing.T) {
	intent := checkpointTestIntent()
	changed := checkpointTestSubject()
	changed.OperationID += "-other"
	if _, err := changed.CanonicalDigest(intent); err == nil {
		t.Fatal("accepted wrong operation identity")
	}
	changed = checkpointTestSubject()
	changed.IntentDigest = "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
	if _, err := changed.CanonicalDigest(intent); err == nil {
		t.Fatal("accepted wrong intent digest")
	}
}

func TestCheckpointRequiresMonotonicOwnerSequence(t *testing.T) {
	intent := checkpointTestIntent()
	changed := checkpointTestSubject()
	changed.CheckpointSequence = 2
	if _, err := changed.CanonicalDigest(intent); err == nil {
		t.Fatal("accepted sequence > 1 without prior checkpoint")
	}
	prior := "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"
	changed.PreviousCheckpointDigest = &prior
	if _, err := changed.CanonicalDigest(intent); err != nil {
		t.Fatalf("valid second checkpoint rejected: %v", err)
	}
}

func TestCheckpointRequiresCorrectionTarget(t *testing.T) {
	intent := checkpointTestIntent()
	changed := checkpointTestSubject()
	changed.Lifecycle = CheckpointSupersedes
	if _, err := changed.CanonicalDigest(intent); err == nil {
		t.Fatal("accepted supersession without target")
	}
	target := "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
	changed.SupersedesCheckpointDigest = &target
	if _, err := changed.CanonicalDigest(intent); err != nil {
		t.Fatalf("valid supersession rejected: %v", err)
	}
	changed.Lifecycle = CheckpointCurrent
	if _, err := changed.CanonicalDigest(intent); err == nil {
		t.Fatal("accepted a supersession target under current lifecycle")
	}
}
