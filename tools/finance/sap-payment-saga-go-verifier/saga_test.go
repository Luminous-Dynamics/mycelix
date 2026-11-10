package sagaverifier

import (
	"bytes"
	"encoding/json"
	"fmt"
	"os"
	"reflect"
	"strings"
	"sync"
	"testing"
)

func testIntent() Intent {
	return Intent{
		SenderDID:            "did:mycelix:alice",
		RecipientDID:         "did:mycelix:bob",
		ClientIdempotencyKey: "client-op-001",
		PayloadFingerprint:   "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
		AmountMinor:          10_000,
		FeeMinor:             100,
		Currency:             "SAP",
	}
}

func digest(seed uint64) string {
	return fmt.Sprintf("sha256:%064x", seed)
}

func refreshEvidenceDigest(t *testing.T, evidence *Evidence) {
	t.Helper()
	digest, err := canonicalEvidenceDigest(*evidence)
	if err != nil {
		t.Fatal(err)
	}
	evidence.EvidenceDigest = digest
}

func localEvidence(j *Journal, label string, seed uint64) Evidence {
	outboxID := ""
	if j.intent.FeeMinor > 0 {
		outboxID = j.EffectID(TreasuryFee)
	}
	evidence := Evidence{
		OperationID:    j.operationID,
		EffectID:       j.EffectID(LocalPaymentTransaction),
		Leg:            LocalPaymentTransaction,
		SenderDID:      j.intent.SenderDID,
		RecipientDID:   j.intent.RecipientDID,
		PrincipalMinor: j.intent.AmountMinor,
		FeeMinor:       j.intent.FeeMinor,
		Currency:       j.intent.Currency,
		Local: &LocalPaymentEvidence{
			SenderAccountID:            j.intent.SenderDID,
			RecipientAccountID:         j.intent.RecipientDID,
			SenderDebitMinor:           j.intent.AmountMinor + j.intent.FeeMinor,
			RecipientCreditMinor:       j.intent.AmountMinor,
			SenderPredecessorAction:    "action:sender-before-" + label,
			SenderDebitAction:          "action:sender-debit-" + label,
			RecipientPredecessorAction: "action:recipient-before-" + label,
			RecipientCreditAction:      "action:recipient-credit-" + label,
			PaymentActionHash:          "action:payment-" + label,
			PaymentStatus:              "Completed",
			ReceiptActionHash:          "action:receipt-" + label,
			ReceiptSignerDID:           j.intent.SenderDID,
			SigningProfile:             ReceiptProfileV1,
			SignatureDigest:            digest(seed + 100_000),
			TreasuryFeeOutboxID:         outboxID,
		},
	}
	digestValue, err := canonicalEvidenceDigest(evidence)
	if err != nil {
		panic(err)
	}
	evidence.EvidenceDigest = digestValue
	return evidence
}

func treasuryEvidence(j *Journal, label string, _seed uint64) Evidence {
	evidence := Evidence{
		OperationID:    j.operationID,
		EffectID:       j.EffectID(TreasuryFee),
		Leg:            TreasuryFee,
		SenderDID:      j.intent.SenderDID,
		RecipientDID:   j.intent.RecipientDID,
		PrincipalMinor: j.intent.AmountMinor,
		FeeMinor:       j.intent.FeeMinor,
		Currency:       j.intent.Currency,
		Treasury: &TreasuryFeeEvidence{
			PoolID:            "treasury:global-fee-pool",
			SourceMemberDID:   j.intent.SenderDID,
			AmountMinor:       j.intent.FeeMinor,
			PaymentActionHash: j.localPaymentActionHash(),
			IdempotencyKey:    j.EffectID(TreasuryFee),
			PredecessorAction: "action:treasury-before-" + label,
			TreasuryAction:    "action:treasury-fee-" + label,
		},
	}
	digestValue, err := canonicalEvidenceDigest(evidence)
	if err != nil {
		panic(err)
	}
	evidence.EvidenceDigest = digestValue
	return evidence
}

func readyLocalJournal(t *testing.T, intent Intent) *Journal {
	t.Helper()
	registry := NewRegistry()
	journal, _, err := registry.CreateOrGet(intent)
	if err != nil {
		t.Fatal(err)
	}
	if err := journal.Authorize(); err != nil {
		t.Fatal(err)
	}
	localID := journal.EffectID(LocalPaymentTransaction)
	if _, err := journal.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
		t.Fatal(err)
	}
	if _, err := journal.CommitLeg(LocalPaymentTransaction, localEvidence(journal, "local", 1)); err != nil {
		t.Fatal(err)
	}
	return journal
}

func TestPublicConformancePack(t *testing.T) {
	data, err := os.ReadFile("fixtures/sap-payment-saga-conformance-v1.json")
	if err != nil {
		t.Fatal(err)
	}
	var pack struct {
		Schema       string `json:"schema"`
		Version      string `json:"version"`
		State        string `json:"state"`
		CaseCount    int `json:"case_count"`
		ClaimCeiling string `json:"claim_ceiling"`
		Cases        []struct {
			ID       string `json:"id"`
			Scenario string `json:"scenario"`
			Expected string `json:"expected"`
		} `json:"cases"`
	}
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&pack); err != nil {
		t.Fatal(err)
	}
	if pack.Schema != "mycelix.sap-payment-saga-conformance/1" ||
		pack.Version != "1.0.0" ||
		pack.State != "FROZEN_PUBLIC_SYNTHETIC_CONFORMANCE" ||
		pack.CaseCount != len(pack.Cases) ||
		pack.ClaimCeiling == "" {
		t.Fatal("conformance pack header is invalid")
	}
	handlers := scenarioHandlers()
	if len(handlers) != pack.CaseCount {
		t.Fatalf("scenario implementation count=%d pack count=%d", len(handlers), pack.CaseCount)
	}
	seen := make(map[string]bool)
	for _, item := range pack.Cases {
		item := item
		if item.ID == "" || item.Expected == "" || seen[item.ID] {
			t.Fatalf("invalid or duplicate scenario ID %q", item.ID)
		}
		seen[item.ID] = true
		handler, ok := handlers[item.Scenario]
		if !ok {
			t.Fatalf("scenario %q has no executable test", item.Scenario)
		}
		t.Run(item.ID+"_"+item.Scenario, handler)
	}
}

func scenarioHandlers() map[string]func(*testing.T) {
	return map[string]func(*testing.T){
		"exact_retry": func(t *testing.T) {
			registry := NewRegistry()
			first, disposition, err := registry.CreateOrGet(testIntent())
			if err != nil || disposition != Created {
				t.Fatalf("first create: disposition=%s err=%v", disposition, err)
			}
			second, disposition, err := registry.CreateOrGet(testIntent())
			if err != nil || disposition != ExistingSameIntent || first.OperationID() != second.OperationID() || registry.Len() != 1 {
				t.Fatalf("exact retry did not resolve the same operation: %s %v", disposition, err)
			}
		},
		"same_key_changed_intent": func(t *testing.T) {
			registry := NewRegistry()
			_, _, err := registry.CreateOrGet(testIntent())
			if err != nil {
				t.Fatal(err)
			}
			changed := testIntent()
			changed.RecipientDID = "did:mycelix:mallory"
			if _, _, err := registry.CreateOrGet(changed); err != ErrIdempotencyConflict {
				t.Fatalf("changed intent error=%v", err)
			}
		},
		"sender_scoped_key": func(t *testing.T) {
			registry := NewRegistry()
			firstIntent := testIntent()
			secondIntent := testIntent()
			secondIntent.SenderDID = "did:mycelix:carol"
			secondIntent.RecipientDID = "did:mycelix:dave"
			first, _, err := registry.CreateOrGet(firstIntent)
			if err != nil {
				t.Fatal(err)
			}
			second, _, err := registry.CreateOrGet(secondIntent)
			if err != nil {
				t.Fatal(err)
			}
			if first.OperationID() == second.OperationID() || registry.Len() != 2 {
				t.Fatal("same raw key across senders was not sender-scoped")
			}
		},
		"amount_overflow": func(t *testing.T) {
			registry := NewRegistry()
			invalid := testIntent()
			invalid.AmountMinor = ^uint64(0)
			invalid.FeeMinor = 1
			if _, _, err := registry.CreateOrGet(invalid); err != ErrAmountOverflow || registry.Len() != 0 {
				t.Fatalf("overflow was not rejected before claim creation: %v", err)
			}
		},
		"fee_before_local_commit": func(t *testing.T) {
			registry := NewRegistry()
			j, _, err := registry.CreateOrGet(testIntent())
			if err != nil {
				t.Fatal(err)
			}
			if err := j.Authorize(); err != nil {
				t.Fatal(err)
			}
			if _, err := j.PrepareLeg(TreasuryFee, j.EffectID(TreasuryFee)); err != ErrDependencyNotSatisfied {
				t.Fatalf("fee leg before local payment error=%v", err)
			}
		},
		"wrong_sender_debit": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "wrong-debit", 2)
			e.Local.SenderDebitMinor--
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong sender debit error=%v", err)
			}
		},
		"wrong_recipient_credit": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "wrong-credit", 3)
			e.Local.RecipientCreditMinor++
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong recipient credit error=%v", err)
			}
		},
		"pending_payment_record": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "pending", 4)
			e.Local.PaymentStatus = "Pending"
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("pending Payment status error=%v", err)
			}
		},
		"missing_fee_outbox": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "missing-outbox", 5)
			e.Local.TreasuryFeeOutboxID = ""
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("missing outbox error=%v", err)
			}
		},
		"wrong_receipt_signer": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "wrong-signer", 6)
			e.Local.ReceiptSignerDID = "did:mycelix:mallory"
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong signer error=%v", err)
			}
		},
		"wrong_receipt_profile": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "wrong-profile", 7)
			e.Local.SigningProfile = "legacy-unversioned"
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong profile error=%v", err)
			}
		},
		"wrong_treasury_pool": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			feeID := j.EffectID(TreasuryFee)
			if _, err := j.PrepareLeg(TreasuryFee, feeID); err != nil {
				t.Fatal(err)
			}
			e := treasuryEvidence(j, "wrong-pool", 8)
			e.Treasury.PoolID = "treasury:other-pool"
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(TreasuryFee, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong Treasury pool error=%v", err)
			}
		},
		"wrong_treasury_amount": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			if _, err := j.PrepareLeg(TreasuryFee, j.EffectID(TreasuryFee)); err != nil {
				t.Fatal(err)
			}
			e := treasuryEvidence(j, "wrong-fee", 9)
			e.Treasury.AmountMinor++
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(TreasuryFee, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong Treasury amount error=%v", err)
			}
		},
		"wrong_treasury_payment_link": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			if _, err := j.PrepareLeg(TreasuryFee, j.EffectID(TreasuryFee)); err != nil {
				t.Fatal(err)
			}
			e := treasuryEvidence(j, "wrong-payment", 10)
			e.Treasury.PaymentActionHash = "action:unrelated-payment"
			refreshEvidenceDigest(t, &e)
			if err := j.validateEvidence(TreasuryFee, e); err != ErrEvidenceInvalid {
				t.Fatalf("wrong Payment link error=%v", err)
			}
		},
		"local_response_loss": func(t *testing.T) {
			registry := NewRegistry()
			j, _, err := registry.CreateOrGet(testIntent())
			if err != nil {
				t.Fatal(err)
			}
			if err := j.Authorize(); err != nil {
				t.Fatal(err)
			}
			localID := j.EffectID(LocalPaymentTransaction)
			if _, err := j.PrepareLeg(LocalPaymentTransaction, localID); err != nil {
				t.Fatal(err)
			}
			if err := j.MarkIndeterminate(LocalPaymentTransaction, localID); err != nil {
				t.Fatal(err)
			}
			if j.Status() != OperationIndeterminate {
				t.Fatal("lost response did not remain indeterminate")
			}
			if got, err := j.PrepareLeg(LocalPaymentTransaction, localID); err != nil || got != TransitionResumedSameEffect {
				t.Fatalf("retry transition=%s err=%v", got, err)
			}
			e := localEvidence(j, "local-recovered", 11)
			if got, err := j.CommitLeg(LocalPaymentTransaction, e); err != nil || got != TransitionCommitted {
				t.Fatalf("commit transition=%s err=%v", got, err)
			}
			if j.Status() != FeeDistributionPending || j.EffectID(LocalPaymentTransaction) != localID {
				t.Fatal("local retry changed identity or overstated overall completion")
			}
			if got, err := j.CommitLeg(LocalPaymentTransaction, e); err != nil || got != TransitionAlreadyCommitted {
				t.Fatalf("duplicate local evidence transition=%s err=%v", got, err)
			}
		},
		"treasury_response_loss": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			feeID := j.EffectID(TreasuryFee)
			if _, err := j.PrepareLeg(TreasuryFee, feeID); err != nil {
				t.Fatal(err)
			}
			if err := j.MarkIndeterminate(TreasuryFee, feeID); err != nil {
				t.Fatal(err)
			}
			if got, err := j.PrepareLeg(TreasuryFee, feeID); err != nil || got != TransitionResumedSameEffect {
				t.Fatalf("fee resume transition=%s err=%v", got, err)
			}
			e := treasuryEvidence(j, "fee-recovered", 12)
			if got, err := j.CommitLeg(TreasuryFee, e); err != nil || got != TransitionCommitted {
				t.Fatalf("fee commit transition=%s err=%v", got, err)
			}
			if j.Status() != Completed {
				t.Fatal("saga not completed after confirmed fee receipt")
			}
			if got, err := j.CommitLeg(TreasuryFee, e); err != nil || got != TransitionAlreadyCommitted {
				t.Fatalf("duplicate fee evidence transition=%s err=%v", got, err)
			}
			changed := treasuryEvidence(j, "fee-recovered", 13)
			changed.Treasury.TreasuryAction = "action:treasury-fee-conflict"
			refreshEvidenceDigest(t, &changed)
			if _, err := j.CommitLeg(TreasuryFee, changed); err != ErrEvidenceConflict {
				t.Fatalf("changed evidence error=%v", err)
			}
		},
		"zero_fee": func(t *testing.T) {
			intent := testIntent()
			intent.FeeMinor = 0
			j := readyLocalJournal(t, intent)
			if j.EffectState(TreasuryFee).Status != NotRequired || j.Status() != Completed {
				t.Fatal("zero-fee transaction must not require a Treasury effect")
			}
		},
		"cannot_reject_after_effect": func(t *testing.T) {
			registry := NewRegistry()
			j, _, err := registry.CreateOrGet(testIntent())
			if err != nil {
				t.Fatal(err)
			}
			if err := j.Authorize(); err != nil {
				t.Fatal(err)
			}
			if _, err := j.PrepareLeg(LocalPaymentTransaction, j.EffectID(LocalPaymentTransaction)); err != nil {
				t.Fatal(err)
			}
			if err := j.RejectBeforeEffects(); err != ErrCannotRejectAfterEffect {
				t.Fatalf("post-effect rejection error=%v", err)
			}
		},
		"concurrent_same_key": func(t *testing.T) {
			registry := NewRegistry()
			const workers = 32
			var wg sync.WaitGroup
			ids := make(chan string, workers)
			dispositions := make(chan CreateDisposition, workers)
			errors := make(chan error, workers)
			for i := 0; i < workers; i++ {
				wg.Add(1)
				go func() {
					defer wg.Done()
					j, disposition, err := registry.CreateOrGet(testIntent())
					if err != nil {
						errors <- err
						return
					}
					ids <- j.OperationID()
					dispositions <- disposition
				}()
			}
			wg.Wait()
			close(ids)
			close(dispositions)
			close(errors)
			created := 0
			var first string
			for id := range ids {
				if first == "" {
					first = id
				} else if id != first {
					t.Fatal("concurrent exact retries produced different operation IDs")
				}
			}
			for disposition := range dispositions {
				if disposition == Created {
					created++
				}
			}
			for err := range errors {
				t.Fatal(err)
			}
			if created != 1 || registry.Len() != 1 {
				t.Fatalf("created=%d registry length=%d", created, registry.Len())
			}
		},
		"noncanonical_digest": func(t *testing.T) {
			j := readyLocalJournal(t, testIntent())
			e := localEvidence(j, "bad-digest", 14)
			e.EvidenceDigest = "SHA256:ABC"
			if err := j.validateEvidence(LocalPaymentTransaction, e); err != ErrEvidenceInvalid {
				t.Fatalf("noncanonical digest error=%v", err)
			}
		},
	}
}

func treasuryDigestVector(senderDID string) Evidence {
	return Evidence{
		OperationID:    "op:1",
		EffectID:       "op:1:treasury-fee",
		Leg:            TreasuryFee,
		SenderDID:      senderDID,
		RecipientDID:   "did:b",
		PrincipalMinor: 42,
		FeeMinor:       3,
		Currency:       "SAP",
		Treasury: &TreasuryFeeEvidence{
			PoolID:            "treasury:global-fee-pool",
			SourceMemberDID:   "did:a",
			AmountMinor:       3,
			PaymentActionHash: "action:p",
			IdempotencyKey:    "op:1:treasury-fee",
			PredecessorAction: "action:before",
			TreasuryAction:    "action:fee",
		},
	}
}

func canonicalDigestFixtureValue(t *testing.T, key string) string {
	t.Helper()
	data, err := os.ReadFile("fixtures/sap-payment-evidence-v1.kat")
	if err != nil {
		t.Fatal(err)
	}
	for _, line := range strings.Split(string(data), "\n") {
		name, value, ok := strings.Cut(line, "=")
		if ok && name == key {
			return strings.TrimSpace(value)
		}
	}
	t.Fatalf("canonical digest fixture has no %q vector", key)
	return ""
}

func TestCanonicalTreasuryFeeDigestMatchesSharedV1KnownAnswers(t *testing.T) {
	for _, vector := range []struct {
		name   string
		sender string
	}{
		{name: "treasury-basic", sender: "did:a"},
		{name: "treasury-unicode-sender", sender: "did:é"},
	} {
		got, err := canonicalEvidenceDigest(treasuryDigestVector(vector.sender))
		if err != nil {
			t.Fatal(err)
		}
		if want := canonicalDigestFixtureValue(t, vector.name); got != want {
			t.Fatalf("%s digest=%s, want %s", vector.name, got, want)
		}
	}
}

func TestCanonicalEvidenceStringFieldsEnforce4096UTF8ByteLimit(t *testing.T) {
	evidence := treasuryDigestVector("did:a")
	evidence.Currency = strings.Repeat("x", maxCanonicalEvidenceFieldBytes)
	if _, err := canonicalEvidenceDigest(evidence); err != nil {
		t.Fatalf("4,096-byte field rejected: %v", err)
	}
	evidence.Currency += "x"
	if _, err := canonicalEvidenceDigest(evidence); err != ErrEvidenceInvalid {
		t.Fatalf("4,097-byte field error=%v", err)
	}

	evidence = treasuryDigestVector("did:a")
	evidence.SenderDID = string([]byte{0xff})
	if _, err := canonicalEvidenceDigest(evidence); err != ErrEvidenceInvalid {
		t.Fatalf("invalid UTF-8 field error=%v", err)
	}
}

func TestCanonicalDigestBindsTypedFields(t *testing.T) {
	j := readyLocalJournal(t, testIntent())
	original := localEvidence(j, "canonical", 31)
	changed := original
	local := *original.Local
	local.RecipientCreditMinor++
	changed.Local = &local
	refreshEvidenceDigest(t, &changed)
	if changed.EvidenceDigest == original.EvidenceDigest {
		t.Fatal("mutated typed evidence retained the same canonical digest")
	}
	if err := j.validateEvidence(LocalPaymentTransaction, changed); err != ErrEvidenceInvalid {
		t.Fatalf("semantically invalid but freshly digested evidence error=%v", err)
	}
}

func TestRegistryReturnsIndependentJournalPerSenderAndKey(t *testing.T) {
	registry := NewRegistry()
	first, _, err := registry.CreateOrGet(testIntent())
	if err != nil {
		t.Fatal(err)
	}
	secondIntent := testIntent()
	secondIntent.SenderDID = "did:mycelix:carol"
	secondIntent.RecipientDID = "did:mycelix:dave"
	second, _, err := registry.CreateOrGet(secondIntent)
	if err != nil {
		t.Fatal(err)
	}
	if reflect.DeepEqual(first.OperationID(), second.OperationID()) {
		t.Fatal("sender-scoped operation identities unexpectedly alias")
	}
}
