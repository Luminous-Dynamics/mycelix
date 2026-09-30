// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Vertical-neutral relationship identity primitives.
//!
//! This module intentionally models identity and lineage, not CRM records.
//! A relationship identity is an opaque logical root that remains stable while
//! revisions, source-chain actions, UI records, and external-system references change.

use std::fmt;

pub const RELATIONSHIP_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct RelationshipId([u8; 32]);

impl RelationshipId {
    pub const fn from_seed(seed: [u8; 32]) -> Self {
        Self(seed)
    }

    pub fn derive(namespace: &str, seed: &[u8]) -> Self {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix.relationship-id.v1\0");
        hasher.update(&(namespace.len() as u64).to_le_bytes());
        hasher.update(namespace.as_bytes());
        hasher.update(&(seed.len() as u64).to_le_bytes());
        hasher.update(seed);
        Self(*hasher.finalize().as_bytes())
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Display for RelationshipId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct RelationshipRevisionId([u8; 32]);

impl RelationshipRevisionId {
    pub fn derive(
        relationship_id: RelationshipId,
        revision: u64,
        canonical_payload: &[u8],
    ) -> Self {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix.relationship-revision.v1\0");
        hasher.update(relationship_id.as_bytes());
        hasher.update(&revision.to_le_bytes());
        hasher.update(&(canonical_payload.len() as u64).to_le_bytes());
        hasher.update(canonical_payload);
        Self(*hasher.finalize().as_bytes())
    }

    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ParticipantRef {
    pub namespace: String,
    pub identifier: String,
}

impl ParticipantRef {
    pub fn new(namespace: impl Into<String>, identifier: impl Into<String>) -> Self {
        Self {
            namespace: namespace.into(),
            identifier: identifier.into(),
        }
    }

    fn canonical_bytes(&self, out: &mut Vec<u8>) {
        write_len_prefixed(out, self.namespace.as_bytes());
        write_len_prefixed(out, self.identifier.as_bytes());
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationshipRecord {
    pub schema_version: u16,
    pub relationship_id: RelationshipId,
    pub revision: u64,
    pub revision_id: RelationshipRevisionId,
    pub created_by: ParticipantRef,
    pub participants: Vec<ParticipantRef>,
}

impl RelationshipRecord {
    pub fn create(
        relationship_id: RelationshipId,
        created_by: ParticipantRef,
        mut participants: Vec<ParticipantRef>,
    ) -> Result<Self, RelationshipError> {
        if participants.is_empty() {
            return Err(RelationshipError::EmptyParticipants);
        }

        participants.sort();
        if participants.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(RelationshipError::DuplicateParticipant);
        }

        let revision = 0;
        let canonical = canonical_record_bytes(
            RELATIONSHIP_SCHEMA_VERSION,
            relationship_id,
            revision,
            &created_by,
            &participants,
        );
        let revision_id = RelationshipRevisionId::derive(relationship_id, revision, &canonical);

        Ok(Self {
            schema_version: RELATIONSHIP_SCHEMA_VERSION,
            relationship_id,
            revision,
            revision_id,
            created_by,
            participants,
        })
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        canonical_record_bytes(
            self.schema_version,
            self.relationship_id,
            self.revision,
            &self.created_by,
            &self.participants,
        )
    }

    pub fn revise(
        &self,
        revision: u64,
        mut participants: Vec<ParticipantRef>,
    ) -> Result<Self, RelationshipError> {
        if self.schema_version != RELATIONSHIP_SCHEMA_VERSION {
            return Err(RelationshipError::UnsupportedSchema(self.schema_version));
        }
        if revision <= self.revision {
            return Err(RelationshipError::NonMonotonicRevision);
        }
        if participants.is_empty() {
            return Err(RelationshipError::EmptyParticipants);
        }

        participants.sort();
        if participants.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(RelationshipError::DuplicateParticipant);
        }

        let canonical = canonical_record_bytes(
            self.schema_version,
            self.relationship_id,
            revision,
            &self.created_by,
            &participants,
        );
        let revision_id = RelationshipRevisionId::derive(self.relationship_id, revision, &canonical);

        Ok(Self {
            schema_version: self.schema_version,
            relationship_id: self.relationship_id,
            revision,
            revision_id,
            created_by: self.created_by.clone(),
            participants,
        })
    }

    pub fn validate_schema(&self) -> Result<(), RelationshipError> {
        if self.schema_version == RELATIONSHIP_SCHEMA_VERSION {
            Ok(())
        } else {
            Err(RelationshipError::UnsupportedSchema(self.schema_version))
        }
    }
}

fn canonical_record_bytes(
    schema_version: u16,
    relationship_id: RelationshipId,
    revision: u64,
    created_by: &ParticipantRef,
    participants: &[ParticipantRef],
) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(b"mycelix.relationship-record.v1\0");
    out.extend_from_slice(&schema_version.to_le_bytes());
    out.extend_from_slice(relationship_id.as_bytes());
    out.extend_from_slice(&revision.to_le_bytes());
    created_by.canonical_bytes(&mut out);
    out.extend_from_slice(&(participants.len() as u64).to_le_bytes());
    for participant in participants {
        participant.canonical_bytes(&mut out);
    }
    out
}

fn write_len_prefixed(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    out.extend_from_slice(bytes);
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RelationshipError {
    EmptyParticipants,
    DuplicateParticipant,
    NonMonotonicRevision,
    UnsupportedSchema(u16),
}

impl fmt::Display for RelationshipError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyParticipants => write!(f, "relationship requires at least one participant"),
            Self::DuplicateParticipant => write!(f, "relationship participants must be unique"),
            Self::NonMonotonicRevision => write!(f, "revision must increase monotonically"),
            Self::UnsupportedSchema(version) => {
                write!(f, "unsupported relationship schema version {version}")
            }
        }
    }
}

impl std::error::Error for RelationshipError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(namespace: &str, identifier: &str) -> ParticipantRef {
        ParticipantRef::new(namespace, identifier)
    }

    #[test]
    fn identity_is_stable_across_revisions() {
        let id = RelationshipId::derive("test", b"relationship-a");
        let record = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        )
        .unwrap();
        let revised = record.revise(1, vec![p("did", "alice"), p("org", "acme")]).unwrap();

        assert_eq!(record.relationship_id, revised.relationship_id);
        assert_ne!(record.revision_id, revised.revision_id);
    }

    #[test]
    fn same_participants_can_have_distinct_relationships() {
        let a = RelationshipId::derive("test", b"relationship-a");
        let b = RelationshipId::derive("test", b"relationship-b");
        assert_ne!(a, b);
    }

    #[test]
    fn participant_order_is_canonicalized() {
        let id = RelationshipId::derive("test", b"order");
        let a = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        )
        .unwrap();
        let b = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("org", "acme"), p("did", "alice")],
        )
        .unwrap();

        assert_eq!(a.canonical_bytes(), b.canonical_bytes());
        assert_eq!(a.revision_id, b.revision_id);
    }

    #[test]
    fn duplicate_participants_fail_closed() {
        let id = RelationshipId::derive("test", b"duplicates");
        let result = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("did", "alice"), p("did", "alice")],
        );
        assert_eq!(result, Err(RelationshipError::DuplicateParticipant));
    }

    #[test]
    fn empty_participants_fail_closed() {
        let id = RelationshipId::derive("test", b"empty");
        let result = RelationshipRecord::create(id, p("did", "alice"), vec![]);
        assert_eq!(result, Err(RelationshipError::EmptyParticipants));
    }

    #[test]
    fn non_monotonic_revision_fails_closed() {
        let id = RelationshipId::derive("test", b"revision");
        let record = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        )
        .unwrap();

        assert_eq!(
            record.revise(0, record.participants.clone()),
            Err(RelationshipError::NonMonotonicRevision)
        );
    }

    #[test]
    fn unknown_schema_fails_closed() {
        let id = RelationshipId::derive("test", b"schema");
        let mut record = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        )
        .unwrap();
        record.schema_version = RELATIONSHIP_SCHEMA_VERSION + 1;

        assert_eq!(
            record.validate_schema(),
            Err(RelationshipError::UnsupportedSchema(RELATIONSHIP_SCHEMA_VERSION + 1))
        );
    }

    #[test]
    fn canonical_bytes_are_deterministic() {
        let id = RelationshipId::derive("test", b"canonical");
        let record = RelationshipRecord::create(
            id,
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        )
        .unwrap();

        assert_eq!(record.canonical_bytes(), record.canonical_bytes());
        assert_eq!(record.revision_id.as_bytes().len(), 32);
    }

    #[test]
    fn relationship_identity_is_not_action_hash() {
        let id = RelationshipId::derive("test", b"identity");
        let fake_action_hash = [0xabu8; 32];
        assert_ne!(id.as_bytes(), &fake_action_hash);
    }

    #[test]
    fn external_identifier_remains_typed_reference() {
        let participant = p("salesforce-account", "001-example");
        assert_eq!(participant.namespace, "salesforce-account");
        assert_eq!(participant.identifier, "001-example");
    }
}
