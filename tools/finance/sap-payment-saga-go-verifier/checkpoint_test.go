package main

import (
	"bufio"
	"encoding/hex"
	"errors"
	"os"
	"strconv"
	"strings"
	"testing"
)

const ownerCheckpointKnownAnswerFixture = "../sap-payment-saga-model/fixtures/sap-payment-owner-checkpoint-v1.kat"

func readOwnerCheckpointFixture(t *testing.T) map[string]string {
	t.Helper()
	file, err := os.Open(ownerCheckpointKnownAnswerFixture)
	if err != nil {
		t.Fatalf("open checkpoint fixture: %v", err)
	}
	defer file.Close()

	fields := make(map[string]string)
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		key, value, ok := strings.Cut(line, "=")
		if !ok || key == "" {
			t.Fatalf("invalid checkpoint fixture line %q", line)
		}
		if _, exists := fields[key]; exists {
			t.Fatalf("duplicate checkpoint fixture key %q", key)
		}
		fields[key] = value
	}
	if err := scanner.Err(); err != nil {
		t.Fatalf("read checkpoint fixture: %v", err)
	}
	return fields
}

func checkpointFixtureSubject(t *testing.T, fields map[string]string) OwnerCheckpointSubjectV1 {
	t.Helper()
	var owner CheckpointOwnerDomain
	switch fields["owner_domain"] {
	case "payments_local_transaction":
		owner = CheckpointPaymentsLocalTransaction
	case "treasury_fee_effect":
		owner = CheckpointTreasuryFeeEffect
	default:
		t.Fatalf("unknown fixture owner domain %q", fields["owner_domain"])
	}
	eventCount, err := strconv.ParseUint(fields["event_count"], 10, 64)
	if err != nil {
		t.Fatalf("parse event count: %v", err)
	}
	sequence, err := strconv.ParseUint(fields["checkpoint_sequence"], 10, 64)
	if err != nil {
		t.Fatalf("parse checkpoint sequence: %v", err)
	}
	issuedAt, err := strconv.ParseInt(fields["issued_at_micros"], 10, 64)
	if err != nil {
		t.Fatalf("parse issued_at_micros: %v", err)
	}
	actionHash, err := hex.DecodeString(fields["source_action_hash_hex"])
	if err != nil {
		t.Fatalf("decode source action hash: %v", err)
	}
	authorityActionHash, err := hex.DecodeString(fields["owner_authority_action_hash_hex"])
	if err != nil {
		t.Fatalf("decode owner authority action hash: %v", err)
	}
	optional := func(name string) *string {
		if fields[name] == "-" {
			return nil
		}
		value := fields[name]
		return &value
	}
	return OwnerCheckpointSubjectV1{
		OperationID:                fields["operation_id"],
		IntentDigest:               fields["intent_digest"],
		OwnerDomain:                owner,
		JournalIdentityDigest:      fields["journal_identity_digest"],
		EventCount:                 eventCount,
		TailDigest:                 fields["tail_digest"],
		CheckpointSequence:         sequence,
		PriorCheckpointDigest:      optional("prior_checkpoint_digest"),
		SourceActionHash:           actionHash,
		OwnerAuthorityActionHash:   authorityActionHash,
		IssuerDID:                  fields["issuer_did"],
		IssuerKeyFingerprint:       fields["issuer_key_fingerprint"],
		OwnerAuthorityDigest:       fields["owner_authority_digest"],
		IssuedAtMicros:             issuedAt,
		SupersedesCheckpointDigest: optional("supersedes_checkpoint_digest"),
	}
}

func TestOwnerCheckpointSubjectKnownAnswer(t *testing.T) {
	fields := readOwnerCheckpointFixture(t)
	if fields["schema"] != "mycelix.sap.owner-checkpoint-subject.v1" {
		t.Fatalf("unexpected checkpoint schema %q", fields["schema"])
	}
	if fields["key_profile"] != "holochain.action-signature.ed25519.v1" {
		t.Fatalf("unexpected Holochain key profile %q", fields["key_profile"])
	}
	subject := checkpointFixtureSubject(t, fields)
	canonical, err := subject.CanonicalBytes()
	if err != nil {
		t.Fatalf("canonicalize checkpoint: %v", err)
	}
	if got := hex.EncodeToString(canonical); got != fields["canonical_hex"] {
		t.Fatalf("canonical bytes = %s, want %s", got, fields["canonical_hex"])
	}
	digest, err := subject.Digest()
	if err != nil {
		t.Fatalf("digest checkpoint: %v", err)
	}
	if digest != fields["digest"] {
		t.Fatalf("checkpoint digest = %s, want %s", digest, fields["digest"])
	}
}

func TestOwnerCheckpointJournalIdentityIsDerivedFromOperationAndIntent(t *testing.T) {
	fields := readOwnerCheckpointFixture(t)
	subject := checkpointFixtureSubject(t, fields)
	subject.OperationID += "-substituted"
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointJournalIdentity) {
		t.Fatalf("substituted journal identity = %v, want %v", err, ErrCheckpointJournalIdentity)
	}
}

func TestOwnerCheckpointDomainAndSourceActionAreBound(t *testing.T) {
	fields := readOwnerCheckpointFixture(t)
	subject := checkpointFixtureSubject(t, fields)
	first, err := subject.Digest()
	if err != nil {
		t.Fatal(err)
	}
	subject.OwnerDomain = CheckpointTreasuryFeeEffect
	second, err := subject.Digest()
	if err != nil {
		t.Fatal(err)
	}
	if first == second {
		t.Fatal("owner-domain substitution did not change checkpoint digest")
	}

	subject = checkpointFixtureSubject(t, fields)
	subject.SourceActionHash = append([]byte(nil), subject.SourceActionHash...)
	subject.SourceActionHash[0] ^= 1
	third, err := subject.Digest()
	if err != nil {
		t.Fatal(err)
	}
	if first == third {
		t.Fatal("source ActionHash substitution did not change checkpoint digest")
	}

	subject = checkpointFixtureSubject(t, fields)
	subject.OwnerAuthorityActionHash = append([]byte(nil), subject.OwnerAuthorityActionHash...)
	subject.OwnerAuthorityActionHash[0] ^= 1
	fourth, err := subject.Digest()
	if err != nil {
		t.Fatal(err)
	}
	if first == fourth {
		t.Fatal("owner-authority ActionHash substitution did not change checkpoint digest")
	}
}

func TestOwnerCheckpointRequiresOwnerScopedSequenceLink(t *testing.T) {
	fields := readOwnerCheckpointFixture(t)
	subject := checkpointFixtureSubject(t, fields)
	subject.CheckpointSequence = 2
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointInvalidSequence) {
		t.Fatalf("sequence without prior checkpoint = %v, want %v", err, ErrCheckpointInvalidSequence)
	}
	prior := "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
	subject.PriorCheckpointDigest = &prior
	if err := subject.Validate(); err != nil {
		t.Fatalf("valid second checkpoint rejected: %v", err)
	}
}

func TestOwnerCheckpointRejectsNoncanonicalAuthorityFields(t *testing.T) {
	fields := readOwnerCheckpointFixture(t)
	subject := checkpointFixtureSubject(t, fields)
	subject.OwnerAuthorityDigest = "sha256:" + strings.Repeat("A", 64)
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointInvalidDigest) {
		t.Fatalf("uppercase authority digest = %v, want %v", err, ErrCheckpointInvalidDigest)
	}

	subject = checkpointFixtureSubject(t, fields)
	subject.IssuerDID = "did:example:alice"
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointInvalid) {
		t.Fatalf("foreign DID namespace = %v, want %v", err, ErrCheckpointInvalid)
	}

	subject = checkpointFixtureSubject(t, fields)
	subject.SourceActionHash = nil
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointInvalid) {
		t.Fatalf("empty source action hash = %v, want %v", err, ErrCheckpointInvalid)
	}

	subject = checkpointFixtureSubject(t, fields)
	subject.SourceActionHash = subject.SourceActionHash[:len(subject.SourceActionHash)-1]
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointInvalid) {
		t.Fatalf("short source ActionHash = %v, want %v", err, ErrCheckpointInvalid)
	}

		subject = checkpointFixtureSubject(t, fields)
	subject.OwnerAuthorityActionHash = nil
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointAuthorityAction) {
		t.Fatalf("empty authority action hash = %v, want %v", err, ErrCheckpointAuthorityAction)
	}

	subject = checkpointFixtureSubject(t, fields)
	subject.EventCount = 0
	if err := subject.Validate(); !errors.Is(err, ErrCheckpointInvalid) {
		t.Fatalf("zero event frontier = %v, want %v", err, ErrCheckpointInvalid)
	}
}

func TestOwnerCheckpointSupersessionIsBound(t *testing.T) {
	fields := readOwnerCheckpointFixture(t)
	subject := checkpointFixtureSubject(t, fields)
	first, err := subject.Digest()
	if err != nil {
		t.Fatal(err)
	}
	supersedes := "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
	subject.SupersedesCheckpointDigest = &supersedes
	second, err := subject.Digest()
	if err != nil {
		t.Fatal(err)
	}
	if first == second {
		t.Fatal("supersession reference did not change checkpoint digest")
	}
}
