// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Administrative record classification and retention primitives.
//!
//! These types describe policy-relevant metadata without deciding jurisdiction-specific
//! disclosure law. Domain zomes can bind local rules to the stable shared vocabulary.

use hdk::prelude::Timestamp;
use serde::{Deserialize, Serialize};

use crate::AuthorityBasis;

/// Disclosure posture for an administrative record.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RecordClassification {
    /// May be disclosed as stored.
    Public,
    /// Is a public record, but a disclosure copy requires redaction first.
    PublicRedacted,
    /// Is non-public under an applicable policy or authority basis.
    Confidential,
    /// Access is limited to explicitly authorized actors or conditions.
    Restricted,
}

impl RecordClassification {
    /// True only when the record may be released without a redaction step.
    pub fn is_publicly_disclosable(&self) -> bool {
        matches!(self, Self::Public)
    }

    /// True when a public disclosure path exists but requires redaction.
    pub fn requires_redaction(&self) -> bool {
        matches!(self, Self::PublicRedacted)
    }
}

/// Action to consider after a record's required retention period ends.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum RecordDisposition {
    Review,
    Archive,
    Destroy,
}

/// Retention schedule attached to a record.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RetentionPolicy {
    /// Stable identifier for the governing retention schedule.
    pub schedule_id: String,
    /// Earliest timestamp at which disposition may occur. `None` means no computed
    /// disposition date (for example, indefinite retention or a trigger not yet met).
    pub retain_until: Option<Timestamp>,
    pub disposition: RecordDisposition,
}

/// Shared metadata that makes an administrative record auditable across civic domains.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AdministrativeRecordMetadata {
    pub record_id: String,
    /// Domain-defined record kind, e.g. `permit.decision` or `procurement.award`.
    pub record_type: String,
    pub created_by: String,
    pub created_at: Timestamp,
    pub classification: RecordClassification,
    /// Legal or policy source for a non-public/redacted classification when applicable.
    pub classification_basis: Option<AuthorityBasis>,
    /// Authority grant exercised to create or approve this record, when applicable.
    pub authority_grant_id: Option<String>,
    /// Hash references to supporting evidence without duplicating evidence payloads.
    pub evidence_hashes: Vec<String>,
    pub retention: Option<RetentionPolicy>,
    /// A legal hold always blocks disposition regardless of the retention deadline.
    pub legal_hold: bool,
}

impl AdministrativeRecordMetadata {
    /// True only when the record can be disclosed as stored, without redaction.
    pub fn is_publicly_disclosable(&self) -> bool {
        self.classification.is_publicly_disclosable()
    }

    /// True when a public disclosure copy must pass through redaction first.
    pub fn requires_redaction(&self) -> bool {
        self.classification.requires_redaction()
    }

    /// Returns true when policy permits the record to enter its configured disposition
    /// workflow at `at`. This does not itself archive or destroy anything.
    pub fn eligible_for_disposition_at(&self, at: Timestamp) -> bool {
        if self.legal_hold {
            return false;
        }

        let Some(retention) = &self.retention else {
            return false;
        };
        let Some(retain_until) = &retention.retain_until else {
            return false;
        };

        at.as_micros() >= retain_until.as_micros()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(classification: RecordClassification) -> AdministrativeRecordMetadata {
        AdministrativeRecordMetadata {
            record_id: "record-1".into(),
            record_type: "permit.decision".into(),
            created_by: "did:mycelix:clerk".into(),
            created_at: Timestamp::from_micros(100),
            classification,
            classification_basis: None,
            authority_grant_id: Some("grant-1".into()),
            evidence_hashes: vec!["sha256:evidence".into()],
            retention: Some(RetentionPolicy {
                schedule_id: "permits-7y".into(),
                retain_until: Some(Timestamp::from_micros(200)),
                disposition: RecordDisposition::Review,
            }),
            legal_hold: false,
        }
    }

    #[test]
    fn public_and_redacted_disclosure_paths_are_distinct() {
        let public = record(RecordClassification::Public);
        assert!(public.is_publicly_disclosable());
        assert!(!public.requires_redaction());

        let redacted = record(RecordClassification::PublicRedacted);
        assert!(!redacted.is_publicly_disclosable());
        assert!(redacted.requires_redaction());

        let confidential = record(RecordClassification::Confidential);
        assert!(!confidential.is_publicly_disclosable());
        assert!(!confidential.requires_redaction());
    }

    #[test]
    fn retention_deadline_only_makes_record_eligible_for_workflow() {
        let record = record(RecordClassification::Public);
        assert!(!record.eligible_for_disposition_at(Timestamp::from_micros(199)));
        assert!(record.eligible_for_disposition_at(Timestamp::from_micros(200)));
    }

    #[test]
    fn legal_hold_blocks_disposition() {
        let mut record = record(RecordClassification::Public);
        record.legal_hold = true;
        assert!(!record.eligible_for_disposition_at(Timestamp::from_micros(500)));
    }

    #[test]
    fn missing_or_unresolved_retention_never_auto_qualifies() {
        let mut record = record(RecordClassification::Public);
        record.retention = None;
        assert!(!record.eligible_for_disposition_at(Timestamp::from_micros(500)));

        record.retention = Some(RetentionPolicy {
            schedule_id: "event-triggered".into(),
            retain_until: None,
            disposition: RecordDisposition::Review,
        });
        assert!(!record.eligible_for_disposition_at(Timestamp::from_micros(500)));
    }
}
