//! Deterministic, effects-disabled fault campaign for the SAP payment reference model.
//!
//! The generator is counter-based: each trace field is derived from
//! (root_seed, sequence, field_domain). It has no mutable PRNG state, so adding
//! a draw to one field cannot perturb later fields or later traces.

use super::*;
use crate::replay::{JournalEventLog, replay_journal};
use sha2::{Digest, Sha256};

const ROOT_SEED_V1: u64 = 0x4d59_4843_414d_5031; // "MYHCAMP1"
const SEQUENCE_GAMMA_V1: u64 = 0x9e37_79b9_7f4a_7c15;
const DOMAIN_GAMMA_V1: u64 = 0xd1b5_4a32_d192_ed03;
const LOCAL_ACK_LOST: u8 = 0b01;
const TREASURY_ACK_LOST: u8 = 0b10;
const PRESUBMIT_TRACE_COUNT: u64 = 10_000;
const FULL_TRACE_COUNT: u64 = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct FaultCampaignPlanV1 {
    sequence: u64,
    seed: u64,
    amount_minor: u64,
    fee_minor: u64,
    local_indeterminate_retries: u8,
    treasury_indeterminate_retries: u8,
    lost_commit_ack_mask: u8,
}

impl FaultCampaignPlanV1 {
    fn for_sequence(sequence: u64) -> Self {
        let seed = derive_word_v1(sequence, 0);
        let amount_minor = 1 + derive_word_v1(sequence, 1) % 1_000_000;
        let fee_minor = if derive_word_v1(sequence, 2).is_multiple_of(5) {
            0
        } else {
            1 + derive_word_v1(sequence, 3) % 500
        };
        let local_indeterminate_retries = (derive_word_v1(sequence, 4) % 4) as u8;
        let treasury_indeterminate_retries = if fee_minor == 0 {
            0
        } else {
            (derive_word_v1(sequence, 5) % 4) as u8
        };
        let mut lost_commit_ack_mask = (derive_word_v1(sequence, 6) & 0b11) as u8;
        if fee_minor == 0 {
            lost_commit_ack_mask &= !TREASURY_ACK_LOST;
        }

        Self {
            sequence,
            seed,
            amount_minor,
            fee_minor,
            local_indeterminate_retries,
            treasury_indeterminate_retries,
            lost_commit_ack_mask,
        }
    }

    fn kat_line(self) -> String {
        format!(
            "{} {:016x} {} {} {} {} {}",
            self.sequence,
            self.seed,
            self.amount_minor,
            self.fee_minor,
            self.local_indeterminate_retries,
            self.treasury_indeterminate_retries,
            self.lost_commit_ack_mask
        )
    }
}

/// SplitMix64 finalizer over a counter/domain-separated input.
fn mix64_v1(mut value: u64) -> u64 {
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

/// Counter-based schedule derivation. field_domain is fixed by the v1 grammar:
/// 0=trace seed, 1=principal, 2=fee mode, 3=fee amount, 4=local retries,
/// 5=Treasury retries, 6=lost-ack mask.
fn derive_word_v1(sequence: u64, field_domain: u64) -> u64 {
    mix64_v1(
        ROOT_SEED_V1
            ^ sequence.wrapping_mul(SEQUENCE_GAMMA_V1)
            ^ field_domain.wrapping_mul(DOMAIN_GAMMA_V1),
    )
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct CampaignOutcomeV1 {
    operation_id: String,
    local_effect_id: String,
    treasury_effect_id: String,
    status: OperationStatus,
    event_count: u64,
    tail_digest: String,
}

fn local_evidence_for_plan(journal: &PaymentJournal, plan: FaultCampaignPlanV1) -> EffectEvidence {
    let intent = journal.intent();
    let label = format!("campaign-{}-local", plan.sequence);
    let mut evidence = EffectEvidence {
        operation_id: journal.operation_id().to_owned(),
        effect_id: journal.effect_id(PaymentLeg::LocalPaymentTransaction),
        leg: PaymentLeg::LocalPaymentTransaction,
        sender_did: intent.sender_did.clone(),
        recipient_did: intent.recipient_did.clone(),
        principal_minor: intent.amount_minor,
        fee_minor: intent.fee_minor,
        currency: intent.currency.clone(),
        evidence_digest: String::new(),
        kind: EffectEvidenceKind::LocalPaymentTransaction {
            sender_account_id: intent.sender_did.clone(),
            recipient_account_id: intent.recipient_did.clone(),
            sender_debit_minor: intent.amount_minor + intent.fee_minor,
            recipient_credit_minor: intent.amount_minor,
            sender_predecessor_action: format!("action:sender-before-{label}"),
            sender_debit_action: format!("action:sender-debit-{label}"),
            recipient_predecessor_action: format!("action:recipient-before-{label}"),
            recipient_credit_action: format!("action:recipient-credit-{label}"),
            payment_action_hash: format!("action:payment-{label}"),
            payment_status: PaymentRecordStatus::Completed,
            receipt_action_hash: format!("action:receipt-{label}"),
            receipt_signer_did: intent.sender_did.clone(),
            signing_profile: "mycelix.finance.payment-receipt.v1".into(),
            signature_digest: format!("sha256:{:064x}", plan.seed ^ 0x4c4f_4341_4c31),
            treasury_fee_outbox_id: if intent.fee_minor == 0 {
                None
            } else {
                Some(journal.effect_id(PaymentLeg::TreasuryFee))
            },
        },
    };
    evidence.evidence_digest = evidence
        .canonical_digest()
        .expect("bounded synthetic local evidence has a canonical digest");
    evidence
}

fn treasury_evidence_for_plan(
    journal: &PaymentJournal,
    plan: FaultCampaignPlanV1,
) -> EffectEvidence {
    let intent = journal.intent();
    let payment_action_hash = journal
        .committed_payment_action_hash()
        .expect("Treasury delivery depends on the committed local payment")
        .to_owned();
    let label = format!("campaign-{}-treasury", plan.sequence);
    let mut evidence = EffectEvidence {
        operation_id: journal.operation_id().to_owned(),
        effect_id: journal.effect_id(PaymentLeg::TreasuryFee),
        leg: PaymentLeg::TreasuryFee,
        sender_did: intent.sender_did.clone(),
        recipient_did: intent.recipient_did.clone(),
        principal_minor: intent.amount_minor,
        fee_minor: intent.fee_minor,
        currency: intent.currency.clone(),
        evidence_digest: String::new(),
        kind: EffectEvidenceKind::TreasuryFee {
            pool_id: "treasury:global-fee-pool".into(),
            source_member_did: intent.sender_did.clone(),
            amount_minor: intent.fee_minor,
            payment_action_hash,
            idempotency_key: journal.effect_id(PaymentLeg::TreasuryFee),
            predecessor_action: format!("action:treasury-before-{label}"),
            treasury_action: format!("action:treasury-fee-{label}"),
        },
    };
    evidence.evidence_digest = evidence
        .canonical_digest()
        .expect("bounded synthetic Treasury evidence has a canonical digest");
    evidence
}

fn run_fault_plan_v1(plan: FaultCampaignPlanV1) -> CampaignOutcomeV1 {
    let sender = "did:mycelix:campaign-sender";
    let recipient = "did:mycelix:campaign-recipient";
    let idempotency_key = format!("fault-campaign-v1-{}", plan.sequence);
    let intent = PaymentIntent::new(
        sender,
        recipient,
        idempotency_key.clone(),
        format!("sha256:{:064x}", plan.seed),
        plan.amount_minor,
        plan.fee_minor,
        "SAP",
    );

    let mut registry = JournalRegistry::default();
    registry
        .create_or_get(intent.clone())
        .expect("generated immutable intent must be valid");
    let mut event_log =
        JournalEventLog::new(intent).expect("generated intent must seed a replay log");
    let journal = registry
        .get_mut(sender, &idempotency_key)
        .expect("new operation must exist in the reference registry");

    journal
        .authorize()
        .expect("new operation can be authorized");
    event_log
        .record_authorized()
        .expect("authorization event is valid");

    let local_effect_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
    journal
        .prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
        .expect("local effect can be prepared");
    event_log
        .record_effect_prepared(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
        .expect("local preparation event is valid");

    for _ in 0..plan.local_indeterminate_retries {
        journal
            .mark_indeterminate(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("prepared local effect can become indeterminate");
        event_log
            .record_effect_indeterminate(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("local indeterminate event is valid");
        journal
            .prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("local retry must resume the same effect");
        event_log
            .record_effect_prepared(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("local same-effect resume event is valid");
    }

    let local_evidence = local_evidence_for_plan(journal, plan);
    let local_evidence_for_replay = local_evidence.clone();
    assert_eq!(
        journal
            .commit_leg(PaymentLeg::LocalPaymentTransaction, local_evidence.clone())
            .expect("local effect evidence must bind to the intent"),
        Transition::Committed
    );
    event_log
        .record_effect_committed(local_evidence_for_replay.clone())
        .expect("local commit event is valid");

    if plan.lost_commit_ack_mask & LOCAL_ACK_LOST != 0 {
        journal
            .mark_indeterminate(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("lost local commit acknowledgement does not undo a commit");
        event_log
            .record_effect_indeterminate(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("post-commit local ambiguity is recorded");
        assert_eq!(
            journal
                .prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
                .expect("same committed local effect is observed"),
            Transition::AlreadyCommitted
        );
        event_log
            .record_effect_prepared(PaymentLeg::LocalPaymentTransaction, &local_effect_id)
            .expect("same committed local effect can be replayed");
        assert_eq!(
            journal
                .commit_leg(PaymentLeg::LocalPaymentTransaction, local_evidence.clone())
                .expect("same local evidence is idempotent"),
            Transition::AlreadyCommitted
        );
        event_log
            .record_effect_committed(local_evidence.clone())
            .expect("same local commit evidence can be replayed");
    }

    // Independent integer-unit conservation oracle. Local sender debit must
    // equal beneficiary principal credit plus the separately observed fee.
    if let EffectEvidenceKind::LocalPaymentTransaction {
        sender_debit_minor,
        recipient_credit_minor,
        ..
    } = &local_evidence.kind
    {
        assert_eq!(*sender_debit_minor, plan.amount_minor + plan.fee_minor);
        assert_eq!(*recipient_credit_minor, plan.amount_minor);
    } else {
        unreachable!("local effect has local transaction evidence kind");
    }

    let treasury_effect_id = journal.effect_id(PaymentLeg::TreasuryFee);
    if plan.fee_minor > 0 {
        journal
            .prepare_leg(PaymentLeg::TreasuryFee, &treasury_effect_id)
            .expect("Treasury effect may be prepared after local commit");
        event_log
            .record_effect_prepared(PaymentLeg::TreasuryFee, &treasury_effect_id)
            .expect("Treasury preparation event is valid");

        for _ in 0..plan.treasury_indeterminate_retries {
            journal
                .mark_indeterminate(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("prepared Treasury effect can become indeterminate");
            event_log
                .record_effect_indeterminate(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("Treasury indeterminate event is valid");
            journal
                .prepare_leg(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("Treasury retry must resume the same effect");
            event_log
                .record_effect_prepared(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("Treasury same-effect resume event is valid");
        }

        let treasury_evidence = treasury_evidence_for_plan(journal, plan);
        let treasury_fee_minor = match &treasury_evidence.kind {
            EffectEvidenceKind::TreasuryFee { amount_minor, .. } => *amount_minor,
            _ => unreachable!("Treasury effect has Treasury evidence kind"),
        };
        assert_eq!(treasury_fee_minor, plan.fee_minor);
        let (recipient_credit_minor, sender_debit_minor) = match &local_evidence.kind {
            EffectEvidenceKind::LocalPaymentTransaction {
                recipient_credit_minor,
                sender_debit_minor,
                ..
            } => (*recipient_credit_minor, *sender_debit_minor),
            _ => unreachable!("local effect has local transaction evidence kind"),
        };
        assert_eq!(
            recipient_credit_minor + treasury_fee_minor,
            sender_debit_minor
        );
        assert_eq!(
            journal
                .commit_leg(PaymentLeg::TreasuryFee, treasury_evidence.clone())
                .expect("Treasury evidence must bind to local Payment and outbox ID"),
            Transition::Committed
        );
        event_log
            .record_effect_committed(treasury_evidence.clone())
            .expect("Treasury commit event is valid");

        if plan.lost_commit_ack_mask & TREASURY_ACK_LOST != 0 {
            journal
                .mark_indeterminate(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("lost Treasury acknowledgement does not undo a commit");
            event_log
                .record_effect_indeterminate(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("post-commit Treasury ambiguity is recorded");
            assert_eq!(
                journal
                    .prepare_leg(PaymentLeg::TreasuryFee, &treasury_effect_id)
                    .expect("same committed Treasury effect is observed"),
                Transition::AlreadyCommitted
            );
            event_log
                .record_effect_prepared(PaymentLeg::TreasuryFee, &treasury_effect_id)
                .expect("same committed Treasury effect can be replayed");
            assert_eq!(
                journal
                    .commit_leg(PaymentLeg::TreasuryFee, treasury_evidence.clone())
                    .expect("same Treasury evidence is idempotent"),
                Transition::AlreadyCommitted
            );
            event_log
                .record_effect_committed(treasury_evidence)
                .expect("same Treasury commit evidence can be replayed");
        }
    }

    assert_eq!(journal.status(), OperationStatus::Completed);

    let snapshot = event_log.frames().to_vec();
    let anchor = event_log.anchor();
    let replayed =
        replay_journal(&snapshot, &anchor).expect("generated valid campaign history must replay");
    assert_eq!(event_log.frames(), snapshot.as_slice());
    assert_eq!(replayed.status(), journal.status());
    assert_eq!(
        replayed.effect_state(PaymentLeg::LocalPaymentTransaction),
        journal.effect_state(PaymentLeg::LocalPaymentTransaction)
    );
    assert_eq!(
        replayed.effect_state(PaymentLeg::TreasuryFee),
        journal.effect_state(PaymentLeg::TreasuryFee)
    );
    assert_eq!(replayed.operation_id(), journal.operation_id());

    CampaignOutcomeV1 {
        operation_id: journal.operation_id().to_owned(),
        local_effect_id,
        treasury_effect_id: if plan.fee_minor > 0 {
            treasury_effect_id
        } else {
            String::new()
        },
        status: journal.status(),
        event_count: anchor.event_count,
        tail_digest: anchor.tail_digest,
    }
}

fn update_summary_text(hasher: &mut Sha256, value: &str) {
    let bytes = value.as_bytes();
    let length = u32::try_from(bytes.len()).expect("campaign summary fields are bounded");
    hasher.update(length.to_be_bytes());
    hasher.update(bytes);
}

fn campaign_summary_v1(trace_count: u64) -> String {
    let mut hasher = Sha256::new();
    hasher.update(b"MXSAP-FAULT-CAMPAIGN-RESULT-V1\0");

    for sequence in 0..trace_count {
        let plan = FaultCampaignPlanV1::for_sequence(sequence);
        let outcome = run_fault_plan_v1(plan);

        hasher.update(plan.sequence.to_be_bytes());
        hasher.update(plan.seed.to_be_bytes());
        hasher.update(plan.amount_minor.to_be_bytes());
        hasher.update(plan.fee_minor.to_be_bytes());
        hasher.update([plan.local_indeterminate_retries]);
        hasher.update([plan.treasury_indeterminate_retries]);
        hasher.update([plan.lost_commit_ack_mask]);
        update_summary_text(&mut hasher, &outcome.operation_id);
        update_summary_text(&mut hasher, &outcome.local_effect_id);
        update_summary_text(&mut hasher, &outcome.treasury_effect_id);
        update_summary_text(&mut hasher, &format!("{:?}", outcome.status));
        hasher.update(outcome.event_count.to_be_bytes());
        update_summary_text(&mut hasher, &outcome.tail_digest);
    }

    format!("{:x}", hasher.finalize())
}

#[test]
fn fault_campaign_plan_known_answer_vectors_match_shared_fixture() {
    let fixture = include_str!(
        "../../sap-payment-saga-go-verifier/fixtures/sap-payment-saga-fault-campaign-v1.kat"
    );

    let mut found = 0usize;
    for line in fixture
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
    {
        let sequence = line
            .split_whitespace()
            .next()
            .expect("non-empty vector line has a sequence")
            .parse::<u64>()
            .expect("known-answer sequence parses");
        assert_eq!(FaultCampaignPlanV1::for_sequence(sequence).kat_line(), line);
        found += 1;
    }
    assert!(
        found >= 8,
        "shared fixture must pin at least eight sequences"
    );
}

#[test]
fn deterministic_presubmit_fault_campaign_replays_ten_thousand_traces() {
    let summary = campaign_summary_v1(PRESUBMIT_TRACE_COUNT);
    println!("fault_campaign_rust_summary_10000={summary}");
    assert_eq!(PRESUBMIT_TRACE_COUNT, 10_000);
}

#[test]
fn deterministic_full_million_fault_campaign() {
    if std::env::var("MYCELIX_FULL_CAMPAIGN").as_deref() != Ok("1") {
        return;
    }
    let summary = campaign_summary_v1(FULL_TRACE_COUNT);
    println!("fault_campaign_rust_summary_1000000={summary}");
    assert_eq!(FULL_TRACE_COUNT, 1_000_000);
}
