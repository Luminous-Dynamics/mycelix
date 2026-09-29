//! D6J transparency receipt reference model.
//!
//! Models append-only registration and a structurally bound inclusion receipt.
//! It does not implement signatures, Merkle proofs, cryptographic verification,
//! trusted timestamps, or a production transparency service.
//!
//! Claim ceiling: ReferenceModelOnly. A receipt is evidence of registration
//! within this model only; it is not evidence that a statement is true, valid,
//! legitimate, consented to, endorsed, or authorized.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatementOrigin {
    Local,
    Foreign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransparencyStatement {
    pub statement_id: &'static str,
    pub digest: &'static str,
    pub origin: StatementOrigin,
    pub source_ref: &'static str,
    pub evidence_ref: &'static str,
    pub schema_generation: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransparencyRecord {
    pub record_id: &'static str,
    pub statement_id: &'static str,
    pub statement_digest: &'static str,
    pub origin: StatementOrigin,
    pub source_ref: &'static str,
    pub evidence_ref: &'static str,
    pub schema_generation: u32,
    pub log_generation: u64,
    pub index: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransparencyReceipt {
    pub record_id: &'static str,
    pub statement_id: &'static str,
    pub statement_digest: &'static str,
    pub origin: StatementOrigin,
    pub source_ref: &'static str,
    pub evidence_ref: &'static str,
    pub schema_generation: u32,
    pub log_generation: u64,
    pub index: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistrationDecision {
    Registered,
    Replayed,
    RejectedInvalidStatement,
    RejectedDuplicateMutation,
    RejectedGeneration,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct TransparencyLog {
    pub records: Vec<TransparencyRecord>,
    pub log_generation: u64,
}

impl TransparencyLog {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register exactly the supplied statement at the next append position.
    /// Existing IDs are idempotent only when all bound statement fields match.
    pub fn register(
        &mut self,
        statement: TransparencyStatement,
        current_schema_generation: u32,
    ) -> RegistrationDecision {
        if statement.statement_id.trim().is_empty()
            || statement.digest.trim().is_empty()
            || statement.source_ref.trim().is_empty()
            || statement.evidence_ref.trim().is_empty()
        {
            return RegistrationDecision::RejectedInvalidStatement;
        }
        if statement.schema_generation != current_schema_generation {
            return RegistrationDecision::RejectedGeneration;
        }
        if let Some(existing) = self.records.iter().find(|r| r.statement_id == statement.statement_id) {
            return if existing.statement_digest == statement.digest
                && existing.origin == statement.origin
                && existing.source_ref == statement.source_ref
                && existing.evidence_ref == statement.evidence_ref
                && existing.schema_generation == statement.schema_generation
            {
                RegistrationDecision::Replayed
            } else {
                RegistrationDecision::RejectedDuplicateMutation
            };
        }

        let index = self.records.len() as u64;
        self.log_generation = self.log_generation.saturating_add(1);
        self.records.push(TransparencyRecord {
            record_id: statement.statement_id,
            statement_id: statement.statement_id,
            statement_digest: statement.digest,
            origin: statement.origin,
            source_ref: statement.source_ref,
            evidence_ref: statement.evidence_ref,
            schema_generation: statement.schema_generation,
            log_generation: self.log_generation,
            index,
        });
        RegistrationDecision::Registered
    }

    /// Issue a model receipt only for an exact record already in this log.
    pub fn receipt_for(&self, statement_id: &str) -> Option<TransparencyReceipt> {
        let record = self.records.iter().find(|r| r.statement_id == statement_id)?;
        Some(TransparencyReceipt {
            record_id: record.record_id,
            statement_id: record.statement_id,
            statement_digest: record.statement_digest,
            origin: record.origin,
            source_ref: record.source_ref,
            evidence_ref: record.evidence_ref,
            schema_generation: record.schema_generation,
            log_generation: record.log_generation,
            index: record.index,
        })
    }

    /// Structural binding only. This does not verify a cryptographic inclusion proof.
    pub fn verify_receipt(&self, receipt: &TransparencyReceipt) -> bool {
        self.records.iter().any(|record| {
            record.record_id == receipt.record_id
                && record.statement_id == receipt.statement_id
                && record.statement_digest == receipt.statement_digest
                && record.origin == receipt.origin
                && record.source_ref == receipt.source_ref
                && record.evidence_ref == receipt.evidence_ref
                && record.schema_generation == receipt.schema_generation
                && record.log_generation == receipt.log_generation
                && record.index == receipt.index
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn statement() -> TransparencyStatement {
        TransparencyStatement {
            statement_id: "statement-1",
            digest: "digest:opaque-001",
            origin: StatementOrigin::Foreign,
            source_ref: "source://node-b",
            evidence_ref: "evidence://node-b/17",
            schema_generation: 4,
        }
    }

    #[test]
    fn registration_receipt_binds_exact_record_and_preserves_foreign_origin() {
        let mut log = TransparencyLog::new();
        assert_eq!(log.register(statement(), 4), RegistrationDecision::Registered);
        let receipt = log.receipt_for("statement-1").unwrap();
        assert!(log.verify_receipt(&receipt));
        assert_eq!(receipt.origin, StatementOrigin::Foreign);
        assert_eq!(receipt.source_ref, "source://node-b");
        assert_eq!(receipt.evidence_ref, "evidence://node-b/17");
        assert_eq!(receipt.index, 0);
        assert_eq!(receipt.log_generation, 1);
    }

    #[test]
    fn exact_registration_replay_is_idempotent_without_new_log_entry() {
        let mut log = TransparencyLog::new();
        assert_eq!(log.register(statement(), 4), RegistrationDecision::Registered);
        assert_eq!(log.register(statement(), 4), RegistrationDecision::Replayed);
        assert_eq!(log.records.len(), 1);
        assert_eq!(log.log_generation, 1);
    }

    #[test]
    fn same_identity_with_changed_statement_fields_fails_closed() {
        let mut log = TransparencyLog::new();
        assert_eq!(log.register(statement(), 4), RegistrationDecision::Registered);
        let mut changed = statement();
        changed.digest = "digest:mutated";
        assert_eq!(log.register(changed, 4), RegistrationDecision::RejectedDuplicateMutation);
        let mut changed_origin = statement();
        changed_origin.origin = StatementOrigin::Local;
        assert_eq!(log.register(changed_origin, 4), RegistrationDecision::RejectedDuplicateMutation);
        let mut changed_source = statement();
        changed_source.source_ref = "source://other";
        assert_eq!(log.register(changed_source, 4), RegistrationDecision::RejectedDuplicateMutation);
        let mut changed_evidence = statement();
        changed_evidence.evidence_ref = "evidence://other";
        assert_eq!(log.register(changed_evidence, 4), RegistrationDecision::RejectedDuplicateMutation);
        assert_eq!(log.records.len(), 1);
    }

    #[test]
    fn receipt_mutation_is_rejected_independently() {
        let mut log = TransparencyLog::new();
        log.register(statement(), 4);
        let receipt = log.receipt_for("statement-1").unwrap();
        let mut changed = receipt;
        changed.statement_digest = "digest:other";
        assert!(!log.verify_receipt(&changed));
        let mut changed = receipt;
        changed.origin = StatementOrigin::Local;
        assert!(!log.verify_receipt(&changed));
        let mut changed = receipt;
        changed.source_ref = "source://other";
        assert!(!log.verify_receipt(&changed));
        let mut changed = receipt;
        changed.evidence_ref = "evidence://other";
        assert!(!log.verify_receipt(&changed));
        let mut changed = receipt;
        changed.index = 99;
        assert!(!log.verify_receipt(&changed));
        let mut changed = receipt;
        changed.log_generation = 99;
        assert!(!log.verify_receipt(&changed));
    }

    #[test]
    fn invalid_or_stale_statements_are_not_registered() {
        let mut log = TransparencyLog::new();
        assert_eq!(log.register(statement(), 5), RegistrationDecision::RejectedGeneration);
        let mut invalid = statement();
        invalid.evidence_ref = "";
        assert_eq!(log.register(invalid, 4), RegistrationDecision::RejectedInvalidStatement);
        assert!(log.records.is_empty());
    }

    #[test]
    fn receipt_is_not_an_authority_or_truth_claim() {
        let mut log = TransparencyLog::new();
        log.register(statement(), 4);
        let receipt = log.receipt_for("statement-1").unwrap();
        // Receipt schema contains registration bindings only: no authority,
        // endorsement, consent, legitimacy, or truth field exists.
        assert!(log.verify_receipt(&receipt));
    }
}
