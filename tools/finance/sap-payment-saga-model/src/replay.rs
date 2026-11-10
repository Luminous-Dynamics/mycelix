use super::{
    EffectEvidence, PaymentIntent, PaymentJournal, PaymentLeg, SagaError,
};
use sha2::{Digest, Sha256};

const EVENT_DOMAIN: &[u8] = b"MXSAP-JOURNAL-EVENT-V1\0";
const GENESIS_DIGEST: &str =
    "sha256:0000000000000000000000000000000000000000000000000000000000000000";

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum JournalEvent {
    Created {
        operation_id: String,
        intent: PaymentIntent,
    },
    Authorized,
    Rejected,
    EffectPrepared {
        leg: PaymentLeg,
        effect_id: String,
    },
    EffectIndeterminate {
        leg: PaymentLeg,
        effect_id: String,
    },
    EffectCommitted {
        evidence: EffectEvidence,
    },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JournalEventFrame {
    pub sequence: u64,
    pub previous_digest: String,
    pub event: JournalEvent,
    pub event_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReplayAnchor {
    pub event_count: u64,
    pub tail_digest: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReplayError {
    InvalidIntent,
    InvalidEvent,
    MissingCreateEvent,
    DuplicateCreateEvent,
    OperationIdentityMismatch,
    SequenceMismatch,
    PreviousDigestMismatch,
    EventDigestMismatch,
    AnchorMismatch,
    InvalidTransition,
    InvalidEvidence,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JournalEventLog {
    frames: Vec<JournalEventFrame>,
}

impl JournalEventLog {
    pub fn new(intent: PaymentIntent) -> Result<Self, ReplayError> {
        intent.validate().map_err(|_| ReplayError::InvalidIntent)?;
        let journal = PaymentJournal::new(intent.clone());
        let mut log = Self { frames: Vec::new() };
        log.append(JournalEvent::Created {
            operation_id: journal.operation_id().to_owned(),
            intent,
        })?;
        Ok(log)
    }

    pub fn frames(&self) -> &[JournalEventFrame] {
        &self.frames
    }

    pub fn anchor(&self) -> ReplayAnchor {
        ReplayAnchor {
            event_count: self.frames.len() as u64,
            tail_digest: self
                .frames
                .last()
                .map(|frame| frame.event_digest.clone())
                .unwrap_or_else(|| GENESIS_DIGEST.to_owned()),
        }
    }

    pub fn record_authorized(&mut self) -> Result<(), ReplayError> {
        self.append(JournalEvent::Authorized)
    }

    pub fn record_rejected(&mut self) -> Result<(), ReplayError> {
        self.append(JournalEvent::Rejected)
    }

    pub fn record_effect_prepared(
        &mut self,
        leg: PaymentLeg,
        effect_id: impl Into<String>,
    ) -> Result<(), ReplayError> {
        self.append(JournalEvent::EffectPrepared {
            leg,
            effect_id: effect_id.into(),
        })
    }

    pub fn record_effect_indeterminate(
        &mut self,
        leg: PaymentLeg,
        effect_id: impl Into<String>,
    ) -> Result<(), ReplayError> {
        self.append(JournalEvent::EffectIndeterminate {
            leg,
            effect_id: effect_id.into(),
        })
    }

    pub fn record_effect_committed(
        &mut self,
        evidence: EffectEvidence,
    ) -> Result<(), ReplayError> {
        self.append(JournalEvent::EffectCommitted { evidence })
    }

    fn append(&mut self, event: JournalEvent) -> Result<(), ReplayError> {
        let sequence =
            u64::try_from(self.frames.len()).map_err(|_| ReplayError::InvalidEvent)?;
        let previous_digest = self
            .frames
            .last()
            .map(|frame| frame.event_digest.clone())
            .unwrap_or_else(|| GENESIS_DIGEST.to_owned());
        let event_digest = event_digest(sequence, &previous_digest, &event)?;
        self.frames.push(JournalEventFrame {
            sequence,
            previous_digest,
            event,
            event_digest,
        });
        Ok(())
    }
}

/// Reconstructs a fresh journal from an immutable event stream.
///
/// The caller must retain the anchor independently of the replayed stream to
/// detect tail truncation. This function checks byte integrity and legal model
/// transitions; it does not authenticate the anchor, issuer or source records.
pub fn replay_journal(
    frames: &[JournalEventFrame],
    anchor: &ReplayAnchor,
) -> Result<PaymentJournal, ReplayError> {
    if u64::try_from(frames.len()).map_err(|_| ReplayError::AnchorMismatch)? != anchor.event_count {
        return Err(ReplayError::AnchorMismatch);
    }
    if frames.is_empty() {
        return Err(ReplayError::MissingCreateEvent);
    }

    let mut previous_digest = GENESIS_DIGEST.to_owned();
    let mut journal: Option<PaymentJournal> = None;

    for (index, frame) in frames.iter().enumerate() {
        let expected_sequence =
            u64::try_from(index).map_err(|_| ReplayError::SequenceMismatch)?;
        if frame.sequence != expected_sequence {
            return Err(ReplayError::SequenceMismatch);
        }
        if frame.previous_digest != previous_digest {
            return Err(ReplayError::PreviousDigestMismatch);
        }

        let expected_digest = event_digest(
            frame.sequence,
            &frame.previous_digest,
            &frame.event,
        )?;
        if frame.event_digest != expected_digest {
            return Err(ReplayError::EventDigestMismatch);
        }

        if index == 0 {
            match &frame.event {
                JournalEvent::Created {
                    operation_id,
                    intent,
                } => {
                    intent.validate().map_err(|_| ReplayError::InvalidIntent)?;
                    let created = PaymentJournal::new(intent.clone());
                    if created.operation_id() != operation_id {
                        return Err(ReplayError::OperationIdentityMismatch);
                    }
                    journal = Some(created);
                }
                _ => return Err(ReplayError::MissingCreateEvent),
            }
        } else {
            let current = journal
                .as_mut()
                .ok_or(ReplayError::MissingCreateEvent)?;
            apply_event(current, &frame.event)?;
        }

        previous_digest.clone_from(&frame.event_digest);
    }

    if previous_digest != anchor.tail_digest {
        return Err(ReplayError::AnchorMismatch);
    }

    journal.ok_or(ReplayError::MissingCreateEvent)
}

fn apply_event(journal: &mut PaymentJournal, event: &JournalEvent) -> Result<(), ReplayError> {
    match event {
        JournalEvent::Created { .. } => Err(ReplayError::DuplicateCreateEvent),
        JournalEvent::Authorized => journal
            .authorize()
            .map(|_| ())
            .map_err(|_| ReplayError::InvalidTransition),
        JournalEvent::Rejected => journal
            .reject_before_effects()
            .map_err(|_| ReplayError::InvalidTransition),
        JournalEvent::EffectPrepared { leg, effect_id } => journal
            .prepare_leg(*leg, effect_id)
            .map(|_| ())
            .map_err(|_| ReplayError::InvalidTransition),
        JournalEvent::EffectIndeterminate { leg, effect_id } => journal
            .mark_indeterminate(*leg, effect_id)
            .map_err(|_| ReplayError::InvalidTransition),
        JournalEvent::EffectCommitted { evidence } => {
            let leg = evidence.leg;
            journal
                .commit_leg(leg, evidence.clone())
                .map(|_| ())
                .map_err(|error| match error {
                    SagaError::InvalidEffectEvidence(_) => ReplayError::InvalidEvidence,
                    _ => ReplayError::InvalidTransition,
                })
        }
    }
}

fn event_digest(
    sequence: u64,
    previous_digest: &str,
    event: &JournalEvent,
) -> Result<String, ReplayError> {
    let mut bytes = Vec::with_capacity(1024);
    bytes.extend_from_slice(EVENT_DOMAIN);
    bytes.extend_from_slice(&sequence.to_be_bytes());
    append_text(&mut bytes, previous_digest)?;
    match event {
        JournalEvent::Created {
            operation_id,
            intent,
        } => {
            bytes.push(1);
            append_text(&mut bytes, operation_id)?;
            append_text(&mut bytes, &intent.sender_did)?;
            append_text(&mut bytes, &intent.recipient_did)?;
            append_text(&mut bytes, &intent.client_idempotency_key)?;
            append_text(&mut bytes, &intent.payload_fingerprint)?;
            bytes.extend_from_slice(&intent.amount_minor.to_be_bytes());
            bytes.extend_from_slice(&intent.fee_minor.to_be_bytes());
            append_text(&mut bytes, &intent.currency)?;
        }
        JournalEvent::Authorized => bytes.push(2),
        JournalEvent::Rejected => bytes.push(3),
        JournalEvent::EffectPrepared { leg, effect_id } => {
            bytes.push(4);
            bytes.push(leg_tag(*leg));
            append_text(&mut bytes, effect_id)?;
        }
        JournalEvent::EffectIndeterminate { leg, effect_id } => {
            bytes.push(5);
            bytes.push(leg_tag(*leg));
            append_text(&mut bytes, effect_id)?;
        }
        JournalEvent::EffectCommitted { evidence } => {
            bytes.push(6);
            bytes.push(leg_tag(evidence.leg));
            let canonical = evidence
                .canonical_digest()
                .map_err(|_| ReplayError::InvalidEvidence)?;
            append_text(&mut bytes, &canonical)?;
            // Preserve the caller-supplied digest as part of the event bytes.
            append_text(&mut bytes, &evidence.evidence_digest)?;
        }
    }

    let digest = Sha256::digest(&bytes);
    let mut output = String::with_capacity(71);
    output.push_str("sha256:");
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").map_err(|_| ReplayError::InvalidEvent)?;
    }
    Ok(output)
}

fn append_text(output: &mut Vec<u8>, value: &str) -> Result<(), ReplayError> {
    let bytes = value.as_bytes();
    let length = u32::try_from(bytes.len()).map_err(|_| ReplayError::InvalidEvent)?;
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(bytes);
    Ok(())
}

fn leg_tag(leg: PaymentLeg) -> u8 {
    match leg {
        PaymentLeg::LocalPaymentTransaction => 1,
        PaymentLeg::TreasuryFee => 2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intent() -> PaymentIntent {
        PaymentIntent::new(
            "did:sender",
            "did:recipient",
            "client-key-001",
            format!("sha256:{}", "a".repeat(64)),
            100,
            3,
            "SAP",
        )
    }

    fn local_evidence(journal: &PaymentJournal) -> EffectEvidence {
        let mut evidence = EffectEvidence {
            operation_id: journal.operation_id().to_owned(),
            effect_id: journal.effect_id(PaymentLeg::LocalPaymentTransaction),
            leg: PaymentLeg::LocalPaymentTransaction,
            sender_did: "did:sender".into(),
            recipient_did: "did:recipient".into(),
            principal_minor: 100,
            fee_minor: 3,
            currency: "SAP".into(),
            evidence_digest: String::new(),
            kind: super::super::EffectEvidenceKind::LocalPaymentTransaction {
                sender_account_id: "did:sender".into(),
                recipient_account_id: "did:recipient".into(),
                sender_debit_minor: 103,
                recipient_credit_minor: 100,
                sender_predecessor_action: "action:sender-before".into(),
                sender_debit_action: "action:sender-debit".into(),
                recipient_predecessor_action: "action:recipient-before".into(),
                recipient_credit_action: "action:recipient-credit".into(),
                payment_action_hash: "action:payment".into(),
                payment_status: super::super::PaymentRecordStatus::Completed,
                receipt_action_hash: "action:receipt".into(),
                receipt_signer_did: "did:sender".into(),
                signing_profile: "mycelix.finance.payment-receipt.v1".into(),
                signature_digest: format!("sha256:{}", "b".repeat(64)),
                treasury_fee_outbox_id: Some(
                    journal.effect_id(PaymentLeg::TreasuryFee),
                ),
            },
        };
        evidence.evidence_digest = evidence.canonical_digest().expect("canonical digest");
        evidence
    }

    fn treasury_evidence(journal: &PaymentJournal) -> EffectEvidence {
        let mut evidence = EffectEvidence {
            operation_id: journal.operation_id().to_owned(),
            effect_id: journal.effect_id(PaymentLeg::TreasuryFee),
            leg: PaymentLeg::TreasuryFee,
            sender_did: "did:sender".into(),
            recipient_did: "did:recipient".into(),
            principal_minor: 100,
            fee_minor: 3,
            currency: "SAP".into(),
            evidence_digest: String::new(),
            kind: super::super::EffectEvidenceKind::TreasuryFee {
                pool_id: "treasury:global-fee-pool".into(),
                source_member_did: "did:sender".into(),
                amount_minor: 3,
                payment_action_hash: "action:payment".into(),
                idempotency_key: journal.effect_id(PaymentLeg::TreasuryFee),
                predecessor_action: "action:treasury-before".into(),
                treasury_action: "action:treasury-credit".into(),
            },
        };
        evidence.evidence_digest = evidence.canonical_digest().expect("canonical digest");
        evidence
    }

    #[test]
    fn replays_local_payment_and_treasury_recovery_history() {
        let mut journal = PaymentJournal::new(intent());
        let mut log = JournalEventLog::new(intent()).expect("event log");

        assert_eq!(journal.authorize(), Ok(super::super::Transition::Prepared));
        log.record_authorized().expect("record authorization");

        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id),
            Ok(super::super::Transition::Prepared)
        );
        log.record_effect_prepared(PaymentLeg::LocalPaymentTransaction, local_id)
            .expect("record local prepare");

        let local = local_evidence(&journal);
        assert_eq!(
            journal.commit_leg(PaymentLeg::LocalPaymentTransaction, local.clone()),
            Ok(super::super::Transition::Committed)
        );
        log.record_effect_committed(local)
            .expect("record local commit");

        let treasury_id = journal.effect_id(PaymentLeg::TreasuryFee);
        assert_eq!(
            journal.prepare_leg(PaymentLeg::TreasuryFee, &treasury_id),
            Ok(super::super::Transition::Prepared)
        );
        log.record_effect_prepared(PaymentLeg::TreasuryFee, treasury_id.clone())
            .expect("record Treasury prepare");

        journal
            .mark_indeterminate(PaymentLeg::TreasuryFee, &treasury_id)
            .expect("mark indeterminate");
        log.record_effect_indeterminate(PaymentLeg::TreasuryFee, treasury_id.clone())
            .expect("record indeterminate outcome");

        assert_eq!(
            journal.prepare_leg(PaymentLeg::TreasuryFee, &treasury_id),
            Ok(super::super::Transition::ResumedSameEffect)
        );
        log.record_effect_prepared(PaymentLeg::TreasuryFee, treasury_id)
            .expect("record same-effect recovery");

        let treasury = treasury_evidence(&journal);
        assert_eq!(
            journal.commit_leg(PaymentLeg::TreasuryFee, treasury.clone()),
            Ok(super::super::Transition::Committed)
        );
        log.record_effect_committed(treasury)
            .expect("record Treasury commit");

        assert_eq!(journal.status(), super::super::OperationStatus::Completed);
        let replayed = replay_journal(log.frames(), &log.anchor()).expect("replay");
        assert_eq!(replayed, journal);
        assert_eq!(replayed.status(), super::super::OperationStatus::Completed);
    }

    #[test]
    fn replay_rejects_tail_truncation_against_external_anchor() {
        let mut log = JournalEventLog::new(intent()).expect("event log");
        log.record_authorized().expect("authorization");
        let anchor = log.anchor();
        let mut truncated = log.frames().to_vec();
        truncated.pop();

        assert_eq!(
            replay_journal(&truncated, &anchor),
            Err(ReplayError::AnchorMismatch)
        );
    }

    #[test]
    fn replay_rejects_reordered_events() {
        let mut log = JournalEventLog::new(intent()).expect("event log");
        log.record_authorized().expect("authorization");
        log.record_rejected().expect("rejection");
        let anchor = log.anchor();
        let mut reordered = log.frames().to_vec();
        reordered.swap(1, 2);

        assert_eq!(
            replay_journal(&reordered, &anchor),
            Err(ReplayError::SequenceMismatch)
        );
    }

    #[test]
    fn replay_rejects_event_payload_tampering() {
        let mut log = JournalEventLog::new(intent()).expect("event log");
        log.record_authorized().expect("authorization");
        let anchor = log.anchor();
        let mut tampered = log.frames().to_vec();
        tampered[1].event = JournalEvent::Rejected;

        assert_eq!(
            replay_journal(&tampered, &anchor),
            Err(ReplayError::EventDigestMismatch)
        );
    }

    #[test]
    fn replay_rejects_transitions_that_could_not_have_been_applied() {
        let journal = PaymentJournal::new(intent());
        let mut log = JournalEventLog::new(intent()).expect("event log");
        log.record_effect_prepared(
            PaymentLeg::LocalPaymentTransaction,
            journal.effect_id(PaymentLeg::LocalPaymentTransaction),
        )
        .expect("record invalid order");
        let anchor = log.anchor();

        assert_eq!(
            replay_journal(log.frames(), &anchor),
            Err(ReplayError::InvalidTransition)
        );
    }

    #[test]
    fn replay_rejects_committed_evidence_without_a_valid_canonical_digest() {
        let mut journal = PaymentJournal::new(intent());
        journal.authorize().expect("authorize");
        let local_id = journal.effect_id(PaymentLeg::LocalPaymentTransaction);
        journal
            .prepare_leg(PaymentLeg::LocalPaymentTransaction, &local_id)
            .expect("prepare");
        let mut evidence = local_evidence(&journal);
        evidence.evidence_digest = "sha256:bad".into();

        let mut log = JournalEventLog::new(intent()).expect("event log");
        log.record_authorized().expect("authorization");
        log.record_effect_prepared(PaymentLeg::LocalPaymentTransaction, local_id)
            .expect("prepare");
        log.record_effect_committed(evidence)
            .expect("record evidence");
        let anchor = log.anchor();

        assert_eq!(
            replay_journal(log.frames(), &anchor),
            Err(ReplayError::InvalidEvidence)
        );
    }
}
