#![forbid(unsafe_code)]

//! Provenance-carrying temporal relationship claims (experimental V1).
//!
//! OCEL remains the interoperable event-log representation. This module
//! projects relationship assertion and mutation records into a two-frontier
//! snapshot. A relationship is a claim about an edge, not proof that the edge
//! is true or authorized. Source references, digests and authority scopes are
//! carried for later independent verification.
//!
//! Validity intervals are half-open: [valid_from, valid_to). The accepted
//! sequence is an append-order knowledge frontier, not a wall-clock timestamp.
//! source_recorded_at is preserved as source metadata and is not treated as a
//! platform transaction-time attestation.

use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalRelationCardinalityV1 {
    OneToOne,
    OneToMany,
    ManyToOne,
    ManyToMany,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalRelationshipConflictSideV1 {
    Subject,
    Object,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationClaimV1 {
    pub relation_id: String,
    pub subject_id: String,
    pub predicate: String,
    pub object_id: String,
    /// Explicit endpoint cardinality policy for this predicate.
    pub cardinality: TemporalRelationCardinalityV1,
    pub valid_from: String,
    pub valid_to: Option<String>,
    pub source_recorded_at: String,
    /// Pre-admitted harness frontier only. When decoded from OCEL, this value
    /// is source-supplied data, not proof of host acceptance. Production
    /// adapters must bind it to a verified host append-log receipt.
    pub accepted_sequence: u64,
    pub source_ref: String,
    pub source_digest: [u8; 32],
    pub asserting_principal: String,
    pub authority_scope: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalRelationMutationKindV1 {
    Supersede,
    Revoke,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationMutationV1 {
    pub event_id: String,
    pub target_relation_id: String,
    pub target_claim_digest: [u8; 32],
    pub kind: TemporalRelationMutationKindV1,
    /// When the mutation is effective in the modeled domain.
    pub effective_at: String,
    pub source_recorded_at: String,
    /// Pre-admitted harness frontier only. When decoded from OCEL, this value
    /// is source-supplied data, not proof of host acceptance. Production
    /// adapters must bind it to a verified host append-log receipt.
    pub accepted_sequence: u64,
    pub source_ref: String,
    pub source_digest: [u8; 32],
    pub asserting_principal: String,
    pub authority_scope: String,
    /// Required for Supersede; absent for Revoke.
    pub replacement_relation_id: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationshipCutV1 {
    pub valid_at: String,
    pub known_through_sequence: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationshipConflictV1 {
    pub side: TemporalRelationshipConflictSideV1,
    pub endpoint_id: String,
    pub predicate: String,
    pub relation_ids: Vec<String>,
    pub subject_ids: Vec<String>,
    pub object_ids: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalRelationshipReceiptV1 {
    pub valid_at: String,
    pub known_through_sequence: u64,
    pub known_relation_ids: Vec<String>,
    pub visible_relation_ids: Vec<String>,
    pub pending_relation_ids: Vec<String>,
    pub expired_relation_ids: Vec<String>,
    pub active_relations: Vec<TemporalRelationClaimV1>,
    pub conflicts: Vec<TemporalRelationshipConflictV1>,
    pub known_mutation_ids: Vec<String>,
    pub pending_mutation_ids: Vec<String>,
    pub applied_mutation_ids: Vec<String>,
    pub superseded_relation_ids: Vec<String>,
    pub revoked_relation_ids: Vec<String>,
    pub snapshot_sha256: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalRelationshipErrorV1 {
    EmptyIdentity,
    InvalidUtcTimestamp,
    InvalidValidityInterval,
    InvalidSequence,
    ZeroSourceDigest,
    DuplicateRelationId,
    DuplicateEventId,
    DuplicateAcceptedSequence,
    UnknownMutationTarget,
    UnknownReplacementRelation,
    TargetClaimDigestMismatch,
    NonCausalMutation,
    MutationScopeMismatch,
    InvalidReplacement,
    ConflictingMutations,
    CardinalityPolicyMismatch,
    InvalidOcel20,
    MissingOcelAttribute,
    DuplicateOcelAttribute,
    InvalidOcelRelationship,
    InvalidDigestEncoding,
    DuplicateOcelIdentity,
    UnknownOcelObject,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value
}

/// Validate canonical fixed-width UTC seconds. Lexical ordering is chronological
/// only after this representation has been checked.
fn valid_utc_second(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 20
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes[10] != b'T'
        || bytes[13] != b':'
        || bytes[16] != b':'
        || bytes[19] != b'Z'
    {
        return false;
    }
    for range in [0..4, 5..7, 8..10, 11..13, 14..16, 17..19] {
        if !bytes[range].iter().all(u8::is_ascii_digit) {
            return false;
        }
    }

    let number = |start: usize, end: usize| {
        std::str::from_utf8(&bytes[start..end])
            .ok()
            .and_then(|part| part.parse::<u32>().ok())
    };
    let year = number(0, 4).unwrap_or(0);
    let month = number(5, 7).unwrap_or(0);
    let day = number(8, 10).unwrap_or(0);
    let hour = number(11, 13).unwrap_or(u32::MAX);
    let minute = number(14, 16).unwrap_or(u32::MAX);
    let second = number(17, 19).unwrap_or(u32::MAX);
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => 0,
    };

    year > 0 && day > 0 && day <= days_in_month && hour <= 23 && minute <= 59 && second <= 59
}

fn validate_claim(claim: &TemporalRelationClaimV1) -> Result<(), TemporalRelationshipErrorV1> {
    if !non_empty(&claim.relation_id)
        || !non_empty(&claim.subject_id)
        || !non_empty(&claim.predicate)
        || !non_empty(&claim.object_id)
        || !non_empty(&claim.source_ref)
        || !non_empty(&claim.asserting_principal)
        || !non_empty(&claim.authority_scope)
    {
        return Err(TemporalRelationshipErrorV1::EmptyIdentity);
    }
    if claim.accepted_sequence == 0 {
        return Err(TemporalRelationshipErrorV1::InvalidSequence);
    }
    if claim.source_digest == [0; 32] {
        return Err(TemporalRelationshipErrorV1::ZeroSourceDigest);
    }
    if !valid_utc_second(&claim.valid_from) || !valid_utc_second(&claim.source_recorded_at) {
        return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
    }
    if let Some(valid_to) = claim.valid_to.as_deref() {
        if !valid_utc_second(valid_to) {
            return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
        }
        if valid_to <= claim.valid_from.as_str() {
            return Err(TemporalRelationshipErrorV1::InvalidValidityInterval);
        }
    }
    Ok(())
}

fn validate_mutation(
    mutation: &TemporalRelationMutationV1,
) -> Result<(), TemporalRelationshipErrorV1> {
    if !non_empty(&mutation.event_id)
        || !non_empty(&mutation.target_relation_id)
        || !non_empty(&mutation.source_ref)
        || !non_empty(&mutation.asserting_principal)
        || !non_empty(&mutation.authority_scope)
    {
        return Err(TemporalRelationshipErrorV1::EmptyIdentity);
    }
    if mutation.accepted_sequence == 0 {
        return Err(TemporalRelationshipErrorV1::InvalidSequence);
    }
    if mutation.source_digest == [0; 32] || mutation.target_claim_digest == [0; 32] {
        return Err(TemporalRelationshipErrorV1::ZeroSourceDigest);
    }
    if !valid_utc_second(&mutation.effective_at) || !valid_utc_second(&mutation.source_recorded_at)
    {
        return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
    }
    match mutation.kind {
        TemporalRelationMutationKindV1::Supersede => {
            if mutation
                .replacement_relation_id
                .as_deref()
                .is_none_or(|value| !non_empty(value))
            {
                return Err(TemporalRelationshipErrorV1::InvalidReplacement);
            }
        }
        TemporalRelationMutationKindV1::Revoke => {
            if mutation.replacement_relation_id.is_some() {
                return Err(TemporalRelationshipErrorV1::InvalidReplacement);
            }
        }
    }
    Ok(())
}

/// Stable SHA-256 commitment to the complete declared relation assertion.
/// Length-prefixing prevents ambiguous concatenations of string fields.
pub fn temporal_relation_claim_digest(claim: &TemporalRelationClaimV1) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hash_field(&mut hasher, "MYCELIX-TEMPORAL-RELATION-CLAIM-V1");
    hash_field(&mut hasher, &claim.relation_id);
    hash_field(&mut hasher, &claim.subject_id);
    hash_field(&mut hasher, &claim.predicate);
    hash_field(&mut hasher, &claim.object_id);
    hasher.update([match claim.cardinality {
        TemporalRelationCardinalityV1::OneToOne => 1,
        TemporalRelationCardinalityV1::OneToMany => 2,
        TemporalRelationCardinalityV1::ManyToOne => 3,
        TemporalRelationCardinalityV1::ManyToMany => 4,
    }]);
    hash_field(&mut hasher, &claim.valid_from);
    match claim.valid_to.as_deref() {
        Some(value) => {
            hasher.update([1]);
            hash_field(&mut hasher, value);
        }
        None => {
            hasher.update([0]);
        }
    }
    hash_field(&mut hasher, &claim.source_recorded_at);
    hasher.update(claim.accepted_sequence.to_be_bytes());
    hash_field(&mut hasher, &claim.source_ref);
    hasher.update(claim.source_digest);
    hash_field(&mut hasher, &claim.asserting_principal);
    hash_field(&mut hasher, &claim.authority_scope);
    finish_digest(hasher)
}

fn hash_field(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> [u8; 32] {
    let bytes = hasher.finalize();
    let mut digest = [0; 32];
    digest.copy_from_slice(&bytes);
    digest
}

fn digest_hex(digest: &[u8; 32]) -> String {
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[derive(Clone, Copy)]
struct SnapshotDigestInput<'a> {
    known_relation_ids: &'a [String],
    visible_relation_ids: &'a [String],
    pending_relation_ids: &'a [String],
    expired_relation_ids: &'a [String],
    known_claims: &'a [&'a TemporalRelationClaimV1],
    active: &'a [TemporalRelationClaimV1],
    conflicts: &'a [TemporalRelationshipConflictV1],
    known_mutations: &'a [&'a TemporalRelationMutationV1],
    superseded: &'a [String],
    revoked: &'a [String],
}

fn snapshot_digest(cut: &TemporalRelationshipCutV1, input: &SnapshotDigestInput<'_>) -> [u8; 32] {
    let SnapshotDigestInput {
        known_relation_ids,
        visible_relation_ids,
        pending_relation_ids,
        expired_relation_ids,
        known_claims,
        active,
        conflicts,
        known_mutations,
        superseded,
        revoked,
    } = *input;
    let mut hasher = Sha256::new();
    hash_field(&mut hasher, "MYCELIX-TEMPORAL-RELATION-SNAPSHOT-V1");
    hash_field(&mut hasher, &cut.valid_at);
    hasher.update(cut.known_through_sequence.to_be_bytes());

    hasher.update((known_relation_ids.len() as u64).to_be_bytes());
    for id in known_relation_ids {
        hash_field(&mut hasher, id);
    }
    hasher.update((visible_relation_ids.len() as u64).to_be_bytes());
    for id in visible_relation_ids {
        hash_field(&mut hasher, id);
    }
    hasher.update((pending_relation_ids.len() as u64).to_be_bytes());
    for id in pending_relation_ids {
        hash_field(&mut hasher, id);
    }
    hasher.update((expired_relation_ids.len() as u64).to_be_bytes());
    for id in expired_relation_ids {
        hash_field(&mut hasher, id);
    }
    hasher.update((known_claims.len() as u64).to_be_bytes());
    for claim in known_claims {
        hasher.update(temporal_relation_claim_digest(claim));
        let validity_state = if claim.valid_from > cut.valid_at {
            1
        } else if claim
            .valid_to
            .as_deref()
            .is_some_and(|valid_to| valid_to <= cut.valid_at.as_str())
        {
            2
        } else {
            3
        };
        hasher.update([validity_state]);
    }
    hasher.update((active.len() as u64).to_be_bytes());
    for claim in active {
        hasher.update(temporal_relation_claim_digest(claim));
    }
    hasher.update((conflicts.len() as u64).to_be_bytes());
    for conflict in conflicts {
        hasher.update([match conflict.side {
            TemporalRelationshipConflictSideV1::Subject => 1,
            TemporalRelationshipConflictSideV1::Object => 2,
        }]);
        hash_field(&mut hasher, &conflict.endpoint_id);
        hash_field(&mut hasher, &conflict.predicate);
        hasher.update((conflict.subject_ids.len() as u64).to_be_bytes());
        for subject_id in &conflict.subject_ids {
            hash_field(&mut hasher, subject_id);
        }
        hasher.update((conflict.relation_ids.len() as u64).to_be_bytes());
        for relation_id in &conflict.relation_ids {
            hash_field(&mut hasher, relation_id);
        }
        hasher.update((conflict.object_ids.len() as u64).to_be_bytes());
        for object_id in &conflict.object_ids {
            hash_field(&mut hasher, object_id);
        }
    }
    hasher.update((known_mutations.len() as u64).to_be_bytes());
    for mutation in known_mutations {
        hash_field(&mut hasher, "MYCELIX-TEMPORAL-RELATION-MUTATION-V1");
        hasher.update([u8::from(mutation.effective_at <= cut.valid_at)]);
        hash_field(&mut hasher, &mutation.event_id);
        hash_field(&mut hasher, &mutation.target_relation_id);
        hasher.update(mutation.target_claim_digest);
        match mutation.kind {
            TemporalRelationMutationKindV1::Supersede => hasher.update([1]),
            TemporalRelationMutationKindV1::Revoke => hasher.update([2]),
        }
        hash_field(&mut hasher, &mutation.effective_at);
        hash_field(&mut hasher, &mutation.source_recorded_at);
        hasher.update(mutation.accepted_sequence.to_be_bytes());
        hash_field(&mut hasher, &mutation.source_ref);
        hasher.update(mutation.source_digest);
        hash_field(&mut hasher, &mutation.asserting_principal);
        hash_field(&mut hasher, &mutation.authority_scope);
        match mutation.replacement_relation_id.as_deref() {
            Some(value) => {
                hasher.update([1]);
                hash_field(&mut hasher, value);
            }
            None => {
                hasher.update([0]);
            }
        }
    }
    hasher.update((superseded.len() as u64).to_be_bytes());
    for id in superseded {
        hash_field(&mut hasher, id);
    }
    hasher.update((revoked.len() as u64).to_be_bytes());
    for id in revoked {
        hash_field(&mut hasher, id);
    }
    finish_digest(hasher)
}

fn declared_ocel_attribute_type<'a>(
    root: &'a Value,
    collection: &str,
    type_name: &str,
    attribute_name: &str,
) -> Result<Option<&'a str>, TemporalRelationshipErrorV1> {
    let declarations = root
        .get(collection)
        .and_then(Value::as_array)
        .ok_or(TemporalRelationshipErrorV1::InvalidOcel20)?;
    let mut type_matches = declarations
        .iter()
        .filter(|declaration| declaration.get("name").and_then(Value::as_str) == Some(type_name));
    let Some(declaration) = type_matches.next() else {
        return Ok(None);
    };
    if type_matches.next().is_some() {
        return Err(TemporalRelationshipErrorV1::InvalidOcel20);
    }

    let attributes = declaration
        .get("attributes")
        .and_then(Value::as_array)
        .ok_or(TemporalRelationshipErrorV1::InvalidOcel20)?;
    let mut attribute_matches = attributes
        .iter()
        .filter(|attribute| attribute.get("name").and_then(Value::as_str) == Some(attribute_name));
    let Some(attribute) = attribute_matches.next() else {
        return Ok(None);
    };
    if attribute_matches.next().is_some() {
        return Err(TemporalRelationshipErrorV1::InvalidOcel20);
    }

    Ok(attribute.get("type").and_then(Value::as_str))
}

fn require_ocel_attribute_declaration(
    root: &Value,
    collection: &str,
    type_name: &str,
    attribute_name: &str,
    expected_type: &str,
) -> Result<(), TemporalRelationshipErrorV1> {
    if declared_ocel_attribute_type(root, collection, type_name, attribute_name)?
        != Some(expected_type)
    {
        return Err(TemporalRelationshipErrorV1::InvalidOcel20);
    }
    Ok(())
}

fn validate_relation_claim_declaration(root: &Value) -> Result<(), TemporalRelationshipErrorV1> {
    for (name, kind) in [
        ("predicate", "string"),
        ("cardinality", "string"),
        ("valid_from", "time"),
        ("source_recorded_at", "time"),
        ("accepted_sequence", "integer"),
        ("source_ref", "string"),
        ("source_digest", "string"),
        ("asserting_principal", "string"),
        ("authority_scope", "string"),
    ] {
        require_ocel_attribute_declaration(
            root,
            "objectTypes",
            "temporal_relation_claim",
            name,
            kind,
        )?;
    }
    match declared_ocel_attribute_type(root, "objectTypes", "temporal_relation_claim", "valid_to")?
    {
        None | Some("time") => {}
        Some(_) => return Err(TemporalRelationshipErrorV1::InvalidOcel20),
    }
    Ok(())
}

fn validate_relation_mutation_declaration(
    root: &Value,
    event_type: &str,
) -> Result<(), TemporalRelationshipErrorV1> {
    for (name, kind) in [
        ("target_claim_digest", "string"),
        ("effective_at", "time"),
        ("source_recorded_at", "time"),
        ("accepted_sequence", "integer"),
        ("source_ref", "string"),
        ("source_digest", "string"),
        ("asserting_principal", "string"),
        ("authority_scope", "string"),
    ] {
        require_ocel_attribute_declaration(root, "eventTypes", event_type, name, kind)?;
    }
    Ok(())
}

fn ocel_attribute<'a>(
    record: &'a Value,
    name: &str,
) -> Result<Option<&'a Value>, TemporalRelationshipErrorV1> {
    let Some(attributes) = record.get("attributes") else {
        return Ok(None);
    };
    let attributes = attributes
        .as_array()
        .ok_or(TemporalRelationshipErrorV1::InvalidOcel20)?;
    let mut matches = attributes
        .iter()
        .filter(|attribute| attribute.get("name").and_then(Value::as_str) == Some(name));
    let Some(first) = matches.next() else {
        return Ok(None);
    };
    if matches.next().is_some() {
        return Err(TemporalRelationshipErrorV1::DuplicateOcelAttribute);
    }
    first
        .get("value")
        .map(Some)
        .ok_or(TemporalRelationshipErrorV1::InvalidOcel20)
}

fn required_ocel_text(record: &Value, name: &str) -> Result<String, TemporalRelationshipErrorV1> {
    ocel_attribute(record, name)?
        .and_then(Value::as_str)
        .filter(|value| non_empty(value))
        .map(str::to_owned)
        .ok_or(TemporalRelationshipErrorV1::MissingOcelAttribute)
}

fn required_ocel_u64(record: &Value, name: &str) -> Result<u64, TemporalRelationshipErrorV1> {
    ocel_attribute(record, name)?
        .and_then(Value::as_u64)
        .ok_or(TemporalRelationshipErrorV1::MissingOcelAttribute)
}

fn optional_ocel_text(
    record: &Value,
    name: &str,
) -> Result<Option<String>, TemporalRelationshipErrorV1> {
    match ocel_attribute(record, name)? {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(value)) if non_empty(value) => Ok(Some(value.clone())),
        Some(_) => Err(TemporalRelationshipErrorV1::InvalidOcel20),
    }
}

fn ocel_relation_target(
    record: &Value,
    qualifier: &str,
) -> Result<String, TemporalRelationshipErrorV1> {
    let relationships = record
        .get("relationships")
        .and_then(Value::as_array)
        .ok_or(TemporalRelationshipErrorV1::InvalidOcelRelationship)?;
    let mut targets = Vec::new();
    for relationship in relationships {
        if relationship.get("qualifier").and_then(Value::as_str) == Some(qualifier) {
            let target = relationship
                .get("objectId")
                .and_then(Value::as_str)
                .filter(|target| non_empty(target))
                .ok_or(TemporalRelationshipErrorV1::InvalidOcelRelationship)?;
            targets.push(target);
        }
    }
    if targets.len() != 1 {
        return Err(TemporalRelationshipErrorV1::InvalidOcelRelationship);
    }
    Ok(targets[0].to_owned())
}

fn decode_digest(value: &str) -> Result<[u8; 32], TemporalRelationshipErrorV1> {
    if value.len() != 64 {
        return Err(TemporalRelationshipErrorV1::InvalidDigestEncoding);
    }
    let mut digest = [0; 32];
    for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        let hex = std::str::from_utf8(pair)
            .map_err(|_| TemporalRelationshipErrorV1::InvalidDigestEncoding)?;
        digest[index] = u8::from_str_radix(hex, 16)
            .map_err(|_| TemporalRelationshipErrorV1::InvalidDigestEncoding)?;
    }
    if digest == [0; 32] {
        return Err(TemporalRelationshipErrorV1::ZeroSourceDigest);
    }
    Ok(digest)
}

fn parse_cardinality(
    value: &str,
) -> Result<TemporalRelationCardinalityV1, TemporalRelationshipErrorV1> {
    match value {
        "one_to_one" => Ok(TemporalRelationCardinalityV1::OneToOne),
        "one_to_many" => Ok(TemporalRelationCardinalityV1::OneToMany),
        "many_to_one" => Ok(TemporalRelationCardinalityV1::ManyToOne),
        "many_to_many" => Ok(TemporalRelationCardinalityV1::ManyToMany),
        _ => Err(TemporalRelationshipErrorV1::InvalidOcel20),
    }
}

/// Parse a restricted OCEL 2.0 JSON extension into temporal relationship claims
/// and append-only mutations. Only temporal_relation_claim objects and
/// temporal_relation_superseded / temporal_relation_revoked events are consumed;
/// unrelated OCEL records are ignored.
///
/// Relation claim objects require exactly one subject and one object O2O
/// relationship. Mutation events require one target_claim relationship;
/// supersession events also require one replacement_claim relationship.
/// This adapter does not authenticate acceptance order: accepted_sequence
/// decoded from an OCEL attribute is pre-admitted fixture input only. Production
/// callers must bind the projection to a verified host append-log receipt.
pub fn parse_temporal_relationships_ocel20(
    input: &str,
) -> Result<
    (
        Vec<TemporalRelationClaimV1>,
        Vec<TemporalRelationMutationV1>,
    ),
    TemporalRelationshipErrorV1,
> {
    let root: Value =
        serde_json::from_str(input).map_err(|_| TemporalRelationshipErrorV1::InvalidOcel20)?;
    for key in ["objectTypes", "eventTypes"] {
        if root.get(key).and_then(Value::as_array).is_none() {
            return Err(TemporalRelationshipErrorV1::InvalidOcel20);
        }
    }
    let objects = root
        .get("objects")
        .and_then(Value::as_array)
        .ok_or(TemporalRelationshipErrorV1::InvalidOcel20)?;
    let events = root
        .get("events")
        .and_then(Value::as_array)
        .ok_or(TemporalRelationshipErrorV1::InvalidOcel20)?;

    let mut object_ids = BTreeSet::new();
    for object in objects {
        let object_id = object
            .get("id")
            .and_then(Value::as_str)
            .filter(|value| non_empty(value))
            .ok_or(TemporalRelationshipErrorV1::EmptyIdentity)?;
        if !object_ids.insert(object_id) {
            return Err(TemporalRelationshipErrorV1::DuplicateOcelIdentity);
        }
    }
    let mut event_ids = BTreeSet::new();
    for event in events {
        let event_id = event
            .get("id")
            .and_then(Value::as_str)
            .filter(|value| non_empty(value))
            .ok_or(TemporalRelationshipErrorV1::EmptyIdentity)?;
        if !event_ids.insert(event_id) {
            return Err(TemporalRelationshipErrorV1::DuplicateOcelIdentity);
        }
    }

    let has_relation_claims = objects.iter().any(|object| {
        object.get("type").and_then(Value::as_str) == Some("temporal_relation_claim")
    });
    if has_relation_claims {
        validate_relation_claim_declaration(&root)?;
    }

    let mut claims = Vec::new();
    for object in objects {
        if object.get("type").and_then(Value::as_str) != Some("temporal_relation_claim") {
            continue;
        }
        let relation_id = object
            .get("id")
            .and_then(Value::as_str)
            .filter(|value| non_empty(value))
            .ok_or(TemporalRelationshipErrorV1::EmptyIdentity)?
            .to_owned();
        if ocel_attribute(object, "valid_to")?.is_some() {
            require_ocel_attribute_declaration(
                &root,
                "objectTypes",
                "temporal_relation_claim",
                "valid_to",
                "time",
            )?;
        }
        let subject_id = ocel_relation_target(object, "subject")?;
        let object_id = ocel_relation_target(object, "object")?;
        if !object_ids.contains(subject_id.as_str())
            || !object_ids.contains(object_id.as_str())
            || subject_id == relation_id
            || object_id == relation_id
        {
            return Err(TemporalRelationshipErrorV1::UnknownOcelObject);
        }
        let claim = TemporalRelationClaimV1 {
            relation_id,
            subject_id,
            predicate: required_ocel_text(object, "predicate")?,
            object_id,
            cardinality: parse_cardinality(&required_ocel_text(object, "cardinality")?)?,
            valid_from: required_ocel_text(object, "valid_from")?,
            valid_to: optional_ocel_text(object, "valid_to")?,
            source_recorded_at: required_ocel_text(object, "source_recorded_at")?,
            accepted_sequence: required_ocel_u64(object, "accepted_sequence")?,
            source_ref: required_ocel_text(object, "source_ref")?,
            source_digest: decode_digest(&required_ocel_text(object, "source_digest")?)?,
            asserting_principal: required_ocel_text(object, "asserting_principal")?,
            authority_scope: required_ocel_text(object, "authority_scope")?,
        };
        claims.push(claim);
    }

    let mut mutations = Vec::new();
    for event in events {
        let event_type = event
            .get("type")
            .and_then(Value::as_str)
            .unwrap_or_default();
        let kind = match event_type {
            "temporal_relation_superseded" => {
                validate_relation_mutation_declaration(&root, event_type)?;
                TemporalRelationMutationKindV1::Supersede
            }
            "temporal_relation_revoked" => {
                validate_relation_mutation_declaration(&root, event_type)?;
                TemporalRelationMutationKindV1::Revoke
            }
            _ => continue,
        };
        let event_id = event
            .get("id")
            .and_then(Value::as_str)
            .filter(|value| non_empty(value))
            .ok_or(TemporalRelationshipErrorV1::EmptyIdentity)?
            .to_owned();
        let replacement_relation_id = match kind {
            TemporalRelationMutationKindV1::Supersede => {
                Some(ocel_relation_target(event, "replacement_claim")?)
            }
            TemporalRelationMutationKindV1::Revoke => {
                let relationships = event
                    .get("relationships")
                    .and_then(Value::as_array)
                    .ok_or(TemporalRelationshipErrorV1::InvalidOcelRelationship)?;
                if relationships.iter().any(|relationship| {
                    relationship.get("qualifier").and_then(Value::as_str)
                        == Some("replacement_claim")
                }) {
                    return Err(TemporalRelationshipErrorV1::InvalidOcelRelationship);
                }
                None
            }
        };
        mutations.push(TemporalRelationMutationV1 {
            event_id,
            target_relation_id: ocel_relation_target(event, "target_claim")?,
            target_claim_digest: decode_digest(&required_ocel_text(event, "target_claim_digest")?)?,
            kind,
            effective_at: required_ocel_text(event, "effective_at")?,
            source_recorded_at: required_ocel_text(event, "source_recorded_at")?,
            accepted_sequence: required_ocel_u64(event, "accepted_sequence")?,
            source_ref: required_ocel_text(event, "source_ref")?,
            source_digest: decode_digest(&required_ocel_text(event, "source_digest")?)?,
            asserting_principal: required_ocel_text(event, "asserting_principal")?,
            authority_scope: required_ocel_text(event, "authority_scope")?,
            replacement_relation_id,
        });
    }

    Ok((claims, mutations))
}

/// Derive a deterministic temporal relationship snapshot from append-only
/// assertion and mutation records. The function does not authenticate sources,
/// verify legal authority, or infer that an asserted relationship is true.
pub fn project_temporal_relationships(
    claims: &[TemporalRelationClaimV1],
    mutations: &[TemporalRelationMutationV1],
    cut: &TemporalRelationshipCutV1,
) -> Result<TemporalRelationshipReceiptV1, TemporalRelationshipErrorV1> {
    if !valid_utc_second(&cut.valid_at) {
        return Err(TemporalRelationshipErrorV1::InvalidUtcTimestamp);
    }

    // An as-known-at query validates only records accepted by its knowledge
    // frontier. A malformed future record must not retroactively poison a
    // historical snapshot that could not yet have known about it.
    let known_claims: Vec<_> = claims
        .iter()
        .filter(|claim| claim.accepted_sequence <= cut.known_through_sequence)
        .collect();
    let mut claim_by_id = BTreeMap::new();
    let mut accepted_sequences = BTreeSet::new();
    let mut predicate_cardinalities = BTreeMap::new();
    for claim in &known_claims {
        validate_claim(claim)?;
        if !accepted_sequences.insert(claim.accepted_sequence) {
            return Err(TemporalRelationshipErrorV1::DuplicateAcceptedSequence);
        }
        if claim_by_id
            .insert(claim.relation_id.as_str(), *claim)
            .is_some()
        {
            return Err(TemporalRelationshipErrorV1::DuplicateRelationId);
        }
        if predicate_cardinalities
            .insert(claim.predicate.as_str(), claim.cardinality)
            .is_some_and(|previous| previous != claim.cardinality)
        {
            return Err(TemporalRelationshipErrorV1::CardinalityPolicyMismatch);
        }
    }

    let known_mutation_records: Vec<_> = mutations
        .iter()
        .filter(|mutation| mutation.accepted_sequence <= cut.known_through_sequence)
        .collect();
    let mut event_ids = BTreeSet::new();
    for mutation in &known_mutation_records {
        validate_mutation(mutation)?;
        if !event_ids.insert(mutation.event_id.as_str()) {
            return Err(TemporalRelationshipErrorV1::DuplicateEventId);
        }
        if !accepted_sequences.insert(mutation.accepted_sequence) {
            return Err(TemporalRelationshipErrorV1::DuplicateAcceptedSequence);
        }
    }

    let mut ordered_mutations: Vec<_> = known_mutation_records.to_vec();
    ordered_mutations.sort_by_key(|mutation| mutation.accepted_sequence);
    let mut mutated_targets = BTreeSet::new();
    for mutation in &ordered_mutations {
        let target = claim_by_id
            .get(mutation.target_relation_id.as_str())
            .ok_or(TemporalRelationshipErrorV1::UnknownMutationTarget)?;

        if mutation.accepted_sequence <= target.accepted_sequence
            || mutation.effective_at < target.valid_from
        {
            return Err(TemporalRelationshipErrorV1::NonCausalMutation);
        }
        if mutation.authority_scope != target.authority_scope {
            return Err(TemporalRelationshipErrorV1::MutationScopeMismatch);
        }
        if mutation.target_claim_digest != temporal_relation_claim_digest(target) {
            return Err(TemporalRelationshipErrorV1::TargetClaimDigestMismatch);
        }
        if !mutated_targets.insert(mutation.target_relation_id.as_str()) {
            return Err(TemporalRelationshipErrorV1::ConflictingMutations);
        }

        if mutation.kind == TemporalRelationMutationKindV1::Supersede {
            let replacement_id = mutation
                .replacement_relation_id
                .as_deref()
                .ok_or(TemporalRelationshipErrorV1::InvalidReplacement)?;
            let replacement = claim_by_id
                .get(replacement_id)
                .ok_or(TemporalRelationshipErrorV1::UnknownReplacementRelation)?;
            if mutated_targets.contains(replacement_id)
                || replacement.relation_id == target.relation_id
                || replacement.accepted_sequence >= mutation.accepted_sequence
                || replacement.valid_from > mutation.effective_at
                || replacement.subject_id != target.subject_id
                || replacement.predicate != target.predicate
                || replacement.cardinality != target.cardinality
                || replacement.authority_scope != target.authority_scope
                || replacement
                    .valid_to
                    .as_deref()
                    .is_some_and(|valid_to| mutation.effective_at.as_str() >= valid_to)
            {
                return Err(TemporalRelationshipErrorV1::InvalidReplacement);
            }
        }
    }

    let mut ordered_known_claims: Vec<_> = known_claims.to_vec();
    ordered_known_claims.sort_by(|left, right| left.relation_id.cmp(&right.relation_id));
    let known_claim_refs: Vec<_> = ordered_known_claims.to_vec();
    let known_relation_ids: Vec<_> = known_claim_refs
        .iter()
        .map(|claim| claim.relation_id.clone())
        .collect();
    let visible_claims: Vec<_> = known_claims
        .iter()
        .copied()
        .filter(|claim| {
            claim.valid_from <= cut.valid_at
                && claim
                    .valid_to
                    .as_deref()
                    .is_none_or(|valid_to| cut.valid_at.as_str() < valid_to)
        })
        .collect();
    let visible_relation_ids: Vec<_> = visible_claims
        .iter()
        .map(|claim| claim.relation_id.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let pending_relation_ids: Vec<_> = known_claims
        .iter()
        .filter(|claim| claim.valid_from > cut.valid_at)
        .map(|claim| claim.relation_id.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let expired_relation_ids: Vec<_> = known_claims
        .iter()
        .filter(|claim| {
            claim
                .valid_to
                .as_deref()
                .is_some_and(|valid_to| valid_to <= cut.valid_at.as_str())
        })
        .map(|claim| claim.relation_id.clone())
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();

    let mut known_mutations: Vec<_> = known_mutation_records.to_vec();
    known_mutations.sort_by_key(|mutation| mutation.accepted_sequence);
    let pending_mutation_ids: Vec<_> = known_mutations
        .iter()
        .filter(|mutation| mutation.effective_at > cut.valid_at)
        .map(|mutation| mutation.event_id.clone())
        .collect();
    let applied: Vec<_> = known_mutations
        .iter()
        .copied()
        .filter(|mutation| mutation.effective_at <= cut.valid_at)
        .collect();
    let known_mutation_ids: Vec<_> = known_mutations
        .iter()
        .map(|mutation| mutation.event_id.clone())
        .collect();
    let mut superseded = BTreeSet::new();
    let mut revoked = BTreeSet::new();
    for mutation in &applied {
        match mutation.kind {
            TemporalRelationMutationKindV1::Supersede => {
                superseded.insert(mutation.target_relation_id.clone());
            }
            TemporalRelationMutationKindV1::Revoke => {
                revoked.insert(mutation.target_relation_id.clone());
            }
        }
    }
    let retired: BTreeSet<_> = superseded.union(&revoked).cloned().collect();
    let mut active_relations: Vec<_> = visible_claims
        .into_iter()
        .filter(|claim| !retired.contains(&claim.relation_id))
        .cloned()
        .collect();
    active_relations.sort_by(|left, right| left.relation_id.cmp(&right.relation_id));

    let mut by_subject_predicate: BTreeMap<(String, String), Vec<&TemporalRelationClaimV1>> =
        BTreeMap::new();
    let mut by_object_predicate: BTreeMap<(String, String), Vec<&TemporalRelationClaimV1>> =
        BTreeMap::new();
    for claim in &active_relations {
        by_subject_predicate
            .entry((claim.subject_id.clone(), claim.predicate.clone()))
            .or_default()
            .push(claim);
        by_object_predicate
            .entry((claim.object_id.clone(), claim.predicate.clone()))
            .or_default()
            .push(claim);
    }

    let mut conflicts = Vec::new();
    for ((subject_id, predicate), group) in by_subject_predicate {
        let cardinality = group[0].cardinality;
        if matches!(
            cardinality,
            TemporalRelationCardinalityV1::OneToOne | TemporalRelationCardinalityV1::ManyToOne
        ) {
            let object_ids: BTreeSet<_> =
                group.iter().map(|claim| claim.object_id.clone()).collect();
            if object_ids.len() > 1 {
                let relation_ids = group
                    .iter()
                    .map(|claim| claim.relation_id.clone())
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect();
                conflicts.push(TemporalRelationshipConflictV1 {
                    side: TemporalRelationshipConflictSideV1::Subject,
                    endpoint_id: subject_id.clone(),
                    predicate,
                    relation_ids,
                    subject_ids: vec![subject_id],
                    object_ids: object_ids.into_iter().collect(),
                });
            }
        }
    }
    for ((object_id, predicate), group) in by_object_predicate {
        let cardinality = group[0].cardinality;
        if matches!(
            cardinality,
            TemporalRelationCardinalityV1::OneToOne | TemporalRelationCardinalityV1::OneToMany
        ) {
            let subject_ids: BTreeSet<_> =
                group.iter().map(|claim| claim.subject_id.clone()).collect();
            if subject_ids.len() > 1 {
                let relation_ids = group
                    .iter()
                    .map(|claim| claim.relation_id.clone())
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect();
                conflicts.push(TemporalRelationshipConflictV1 {
                    side: TemporalRelationshipConflictSideV1::Object,
                    endpoint_id: object_id.clone(),
                    predicate,
                    relation_ids,
                    subject_ids: subject_ids.into_iter().collect(),
                    object_ids: vec![object_id],
                });
            }
        }
    }

    let applied_mutation_ids: Vec<_> = applied
        .iter()
        .map(|mutation| mutation.event_id.clone())
        .collect();
    let known_mutation_refs: Vec<_> = known_mutations.to_vec();
    let superseded_relation_ids: Vec<_> = superseded.into_iter().collect();
    let revoked_relation_ids: Vec<_> = revoked.into_iter().collect();
    let digest = snapshot_digest(
        cut,
        &SnapshotDigestInput {
            known_relation_ids: &known_relation_ids,
            visible_relation_ids: &visible_relation_ids,
            pending_relation_ids: &pending_relation_ids,
            expired_relation_ids: &expired_relation_ids,
            known_claims: &known_claim_refs,
            active: &active_relations,
            conflicts: &conflicts,
            known_mutations: &known_mutation_refs,
            superseded: &superseded_relation_ids,
            revoked: &revoked_relation_ids,
        },
    );

    Ok(TemporalRelationshipReceiptV1 {
        valid_at: cut.valid_at.clone(),
        known_through_sequence: cut.known_through_sequence,
        known_relation_ids,
        visible_relation_ids,
        pending_relation_ids,
        expired_relation_ids,
        active_relations,
        conflicts,
        known_mutation_ids,
        pending_mutation_ids,
        applied_mutation_ids,
        superseded_relation_ids,
        revoked_relation_ids,
        snapshot_sha256: digest_hex(&digest),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const T08: &str = "2026-11-27T08:00:00Z";
    const T10: &str = "2026-11-27T10:00:00Z";
    const T11: &str = "2026-11-27T11:00:00Z";
    const T12: &str = "2026-11-27T12:00:00Z";
    const T13: &str = "2026-11-27T13:00:00Z";

    fn claim(
        relation_id: &str,
        subject_id: &str,
        predicate: &str,
        object_id: &str,
        valid_from: &str,
        sequence: u64,
    ) -> TemporalRelationClaimV1 {
        TemporalRelationClaimV1 {
            relation_id: relation_id.to_owned(),
            subject_id: subject_id.to_owned(),
            predicate: predicate.to_owned(),
            object_id: object_id.to_owned(),
            cardinality: TemporalRelationCardinalityV1::OneToOne,
            valid_from: valid_from.to_owned(),
            valid_to: None,
            source_recorded_at: T08.to_owned(),
            accepted_sequence: sequence,
            source_ref: format!("source:{relation_id}"),
            source_digest: [sequence as u8; 32],
            asserting_principal: "principal:synthetic-operator".to_owned(),
            authority_scope: "scope:water-zone-A".to_owned(),
        }
    }

    fn mutation(
        event_id: &str,
        target: &TemporalRelationClaimV1,
        kind: TemporalRelationMutationKindV1,
        effective_at: &str,
        sequence: u64,
        replacement_relation_id: Option<&str>,
    ) -> TemporalRelationMutationV1 {
        TemporalRelationMutationV1 {
            event_id: event_id.to_owned(),
            target_relation_id: target.relation_id.clone(),
            target_claim_digest: temporal_relation_claim_digest(target),
            kind,
            effective_at: effective_at.to_owned(),
            source_recorded_at: T13.to_owned(),
            accepted_sequence: sequence,
            source_ref: format!("source:{event_id}"),
            source_digest: [sequence as u8; 32],
            asserting_principal: "principal:synthetic-reviewer".to_owned(),
            authority_scope: target.authority_scope.clone(),
            replacement_relation_id: replacement_relation_id.map(str::to_owned),
        }
    }

    fn cut(valid_at: &str, known_through_sequence: u64) -> TemporalRelationshipCutV1 {
        TemporalRelationshipCutV1 {
            valid_at: valid_at.to_owned(),
            known_through_sequence,
        }
    }

    fn object_fixture_attribute(
        name: &str,
        value: impl Into<serde_json::Value>,
    ) -> serde_json::Value {
        serde_json::json!({
            "name": name,
            "time": T08,
            "value": value.into()
        })
    }

    fn event_fixture_attribute(
        name: &str,
        value: impl Into<serde_json::Value>,
    ) -> serde_json::Value {
        serde_json::json!({
            "name": name,
            "value": value.into()
        })
    }

    fn temporal_relationship_ocel_fixture(
        target_digest: &str,
        source_digest: &str,
    ) -> serde_json::Value {
        serde_json::json!({
            "objectTypes": [
                {"name": "asset", "attributes": []},
                {"name": "zone", "attributes": []},
                {
                    "name": "temporal_relation_claim",
                    "attributes": [
                        {"name": "predicate", "type": "string"},
                        {"name": "cardinality", "type": "string"},
                        {"name": "valid_from", "type": "time"},
                        {"name": "source_recorded_at", "type": "time"},
                        {"name": "accepted_sequence", "type": "integer"},
                        {"name": "source_ref", "type": "string"},
                        {"name": "source_digest", "type": "string"},
                        {"name": "asserting_principal", "type": "string"},
                        {"name": "authority_scope", "type": "string"}
                    ]
                }
            ],
            "eventTypes": [{
                "name": "temporal_relation_revoked",
                "attributes": [
                    {"name": "target_claim_digest", "type": "string"},
                    {"name": "effective_at", "type": "time"},
                    {"name": "source_recorded_at", "type": "time"},
                    {"name": "accepted_sequence", "type": "integer"},
                    {"name": "source_ref", "type": "string"},
                    {"name": "source_digest", "type": "string"},
                    {"name": "asserting_principal", "type": "string"},
                    {"name": "authority_scope", "type": "string"}
                ]
            }],
            "objects": [
                {"id": "asset-1", "type": "asset", "attributes": [], "relationships": []},
                {"id": "zone-1", "type": "zone", "attributes": [], "relationships": []},
                {
                    "id": "rel-asset-zone",
                    "type": "temporal_relation_claim",
                    "attributes": [
                        object_fixture_attribute("predicate", "member_of_zone"),
                        object_fixture_attribute("cardinality", "many_to_one"),
                        object_fixture_attribute("valid_from", T08),
                        object_fixture_attribute("source_recorded_at", T08),
                        object_fixture_attribute("accepted_sequence", 1),
                        object_fixture_attribute("source_ref", "source:asset-zone"),
                        object_fixture_attribute("source_digest", source_digest),
                        object_fixture_attribute("asserting_principal", "principal:operator"),
                        object_fixture_attribute("authority_scope", "scope:zone-A")
                    ],
                    "relationships": [
                        {"objectId": "asset-1", "qualifier": "subject"},
                        {"objectId": "zone-1", "qualifier": "object"}
                    ]
                }
            ],
            "events": [{
                "id": "event-zone-revoked",
                "type": "temporal_relation_revoked",
                "time": T13,
                "attributes": [
                    event_fixture_attribute("target_claim_digest", target_digest),
                    event_fixture_attribute("effective_at", T10),
                    event_fixture_attribute("source_recorded_at", T13),
                    event_fixture_attribute("accepted_sequence", 2),
                    event_fixture_attribute("source_ref", "source:zone-revocation"),
                    event_fixture_attribute("source_digest", digest_hex(&[0x22; 32])),
                    event_fixture_attribute("asserting_principal", "principal:reviewer"),
                    event_fixture_attribute("authority_scope", "scope:zone-A")
                ],
                "relationships": [{
                    "objectId": "rel-asset-zone",
                    "qualifier": "target_claim"
                }]
            }]
        })
    }

    #[test]
    fn parses_ocel20_claim_objects_and_revocation_events_into_temporal_projection() {
        let mut expected_claim = claim(
            "rel-asset-zone",
            "asset-1",
            "member_of_zone",
            "zone-1",
            T08,
            1,
        );
        expected_claim.cardinality = TemporalRelationCardinalityV1::ManyToOne;
        expected_claim.source_ref = "source:asset-zone".to_owned();
        expected_claim.source_digest = [0x11; 32];
        expected_claim.asserting_principal = "principal:operator".to_owned();
        expected_claim.authority_scope = "scope:zone-A".to_owned();

        let target_digest = digest_hex(&temporal_relation_claim_digest(&expected_claim));
        let source_digest = digest_hex(&expected_claim.source_digest);
        let fixture = temporal_relationship_ocel_fixture(&target_digest, &source_digest);
        let input = serde_json::to_string(&fixture).unwrap();

        let (claims, mutations) = parse_temporal_relationships_ocel20(&input).unwrap();
        assert_eq!(claims, vec![expected_claim]);
        assert_eq!(mutations.len(), 1);
        assert_eq!(mutations[0].target_relation_id, "rel-asset-zone");
        assert_eq!(mutations[0].kind, TemporalRelationMutationKindV1::Revoke);

        let receipt = project_temporal_relationships(&claims, &mutations, &cut(T11, 2)).unwrap();
        assert!(receipt.active_relations.is_empty());
        assert_eq!(receipt.revoked_relation_ids, vec!["rel-asset-zone"]);
    }

    #[test]
    fn rejects_ocel_relation_attribute_with_wrong_declared_type() {
        let expected_claim = claim(
            "rel-asset-zone",
            "asset-1",
            "member_of_zone",
            "zone-1",
            T08,
            1,
        );
        let target_digest = digest_hex(&temporal_relation_claim_digest(&expected_claim));
        let source_digest = digest_hex(&expected_claim.source_digest);
        let mut fixture = temporal_relationship_ocel_fixture(&target_digest, &source_digest);
        let declarations = fixture["objectTypes"][2]["attributes"]
            .as_array_mut()
            .unwrap();
        let cardinality = declarations
            .iter_mut()
            .find(|attribute| attribute["name"] == "cardinality")
            .unwrap();
        cardinality["type"] = serde_json::Value::String("integer".to_owned());
        let input = serde_json::to_string(&fixture).unwrap();

        assert_eq!(
            parse_temporal_relationships_ocel20(&input),
            Err(TemporalRelationshipErrorV1::InvalidOcel20)
        );
    }

    #[test]
    fn rejects_duplicate_consumed_ocel_type_declaration_in_either_order() {
        let digest = digest_hex(&[0x11; 32]);
        let fixture = temporal_relationship_ocel_fixture(&digest, &digest);
        let mut duplicate = fixture["objectTypes"][2].clone();
        duplicate["attributes"][0]["type"] = serde_json::Value::String("integer".to_owned());

        for duplicate_first in [false, true] {
            let mut candidate = fixture.clone();
            let declarations = candidate["objectTypes"].as_array_mut().unwrap();
            declarations.push(duplicate.clone());
            if duplicate_first {
                let last = declarations.len() - 1;
                declarations.swap(2, last);
            }
            let input = serde_json::to_string(&candidate).unwrap();
            assert_eq!(
                parse_temporal_relationships_ocel20(&input),
                Err(TemporalRelationshipErrorV1::InvalidOcel20)
            );
        }
    }

    #[test]
    fn rejects_duplicate_consumed_ocel_attribute_declaration_in_either_order() {
        let digest = digest_hex(&[0x11; 32]);
        let fixture = temporal_relationship_ocel_fixture(&digest, &digest);
        for duplicate_first in [false, true] {
            let mut candidate = fixture.clone();
            let attributes = candidate["objectTypes"][2]["attributes"]
                .as_array_mut()
                .unwrap();
            let duplicate = serde_json::json!({"name": "predicate", "type": "integer"});
            if duplicate_first {
                attributes.insert(0, duplicate);
            } else {
                attributes.push(duplicate);
            }
            let input = serde_json::to_string(&candidate).unwrap();
            assert_eq!(
                parse_temporal_relationships_ocel20(&input),
                Err(TemporalRelationshipErrorV1::InvalidOcel20)
            );
        }
    }

    #[test]
    fn rejects_duplicate_optional_valid_to_schema_even_when_no_claim_uses_it() {
        let digest = digest_hex(&[0x11; 32]);
        let fixture = temporal_relationship_ocel_fixture(&digest, &digest);
        for duplicate_first in [false, true] {
            let mut candidate = fixture.clone();
            let attributes = candidate["objectTypes"][2]["attributes"]
                .as_array_mut()
                .unwrap();
            let declaration = serde_json::json!({"name": "valid_to", "type": "time"});
            let conflicting = serde_json::json!({"name": "valid_to", "type": "string"});
            if duplicate_first {
                attributes.push(conflicting);
                attributes.push(declaration);
            } else {
                attributes.push(declaration);
                attributes.push(conflicting);
            }
            let input = serde_json::to_string(&candidate).unwrap();
            assert_eq!(
                parse_temporal_relationships_ocel20(&input),
                Err(TemporalRelationshipErrorV1::InvalidOcel20)
            );
        }
    }

    #[test]
    fn rejects_duplicate_consumed_ocel_attribute() {
        let expected_claim = claim(
            "rel-asset-zone",
            "asset-1",
            "member_of_zone",
            "zone-1",
            T08,
            1,
        );
        let target_digest = digest_hex(&temporal_relation_claim_digest(&expected_claim));
        let source_digest = digest_hex(&expected_claim.source_digest);
        let mut fixture = temporal_relationship_ocel_fixture(&target_digest, &source_digest);
        let duplicate = fixture["objects"][2]["attributes"][0].clone();
        fixture["objects"][2]["attributes"]
            .as_array_mut()
            .unwrap()
            .push(duplicate);
        let input = serde_json::to_string(&fixture).unwrap();

        assert_eq!(
            parse_temporal_relationships_ocel20(&input),
            Err(TemporalRelationshipErrorV1::DuplicateOcelAttribute)
        );
    }

    #[test]
    fn rejects_ambiguous_ocel_relation_endpoint_qualifier() {
        let mut expected_claim = claim(
            "rel-asset-zone",
            "asset-1",
            "member_of_zone",
            "zone-1",
            T08,
            1,
        );
        expected_claim.cardinality = TemporalRelationCardinalityV1::ManyToOne;
        let target_digest = digest_hex(&temporal_relation_claim_digest(&expected_claim));
        let source_digest = digest_hex(&expected_claim.source_digest);
        let mut fixture = temporal_relationship_ocel_fixture(&target_digest, &source_digest);
        fixture["objects"][2]["relationships"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::json!({
                "objectId": "asset-1",
                "qualifier": "subject"
            }));
        let input = serde_json::to_string(&fixture).unwrap();

        assert_eq!(
            parse_temporal_relationships_ocel20(&input),
            Err(TemporalRelationshipErrorV1::InvalidOcelRelationship)
        );
    }

    #[test]
    fn rejects_ocel_relationship_endpoint_that_does_not_exist() {
        let mut expected_claim = claim(
            "rel-asset-zone",
            "asset-1",
            "member_of_zone",
            "zone-1",
            T08,
            1,
        );
        expected_claim.cardinality = TemporalRelationCardinalityV1::ManyToOne;
        let target_digest = digest_hex(&temporal_relation_claim_digest(&expected_claim));
        let source_digest = digest_hex(&expected_claim.source_digest);
        let mut fixture = temporal_relationship_ocel_fixture(&target_digest, &source_digest);
        fixture["objects"][2]["relationships"][1]["objectId"] =
            serde_json::Value::String("zone-missing".to_owned());
        let input = serde_json::to_string(&fixture).unwrap();

        assert_eq!(
            parse_temporal_relationships_ocel20(&input),
            Err(TemporalRelationshipErrorV1::UnknownOcelObject)
        );
    }

    #[test]
    fn rejects_duplicate_object_identity_even_when_unrelated_to_claim() {
        let expected_claim = claim(
            "rel-asset-zone",
            "asset-1",
            "member_of_zone",
            "zone-1",
            T08,
            1,
        );
        let target_digest = digest_hex(&temporal_relation_claim_digest(&expected_claim));
        let source_digest = digest_hex(&expected_claim.source_digest);
        let mut fixture = temporal_relationship_ocel_fixture(&target_digest, &source_digest);
        fixture["objects"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::json!({
                "id": "asset-1",
                "type": "asset",
                "attributes": [],
                "relationships": []
            }));
        let input = serde_json::to_string(&fixture).unwrap();

        assert_eq!(
            parse_temporal_relationships_ocel20(&input),
            Err(TemporalRelationshipErrorV1::DuplicateOcelIdentity)
        );
    }

    #[test]
    fn late_recorded_relation_is_invisible_before_accepted_sequence() {
        let mut late = claim(
            "rel-crew-wo",
            "crew-7",
            "assigned_to",
            "work-order-9",
            T08,
            2,
        );
        late.source_recorded_at = T13.to_owned();

        let before = project_temporal_relationships(&[late.clone()], &[], &cut(T11, 1)).unwrap();
        assert!(before.active_relations.is_empty());

        let after = project_temporal_relationships(&[late], &[], &cut(T11, 2)).unwrap();
        assert_eq!(after.active_relations.len(), 1);
        assert_eq!(after.active_relations[0].relation_id, "rel-crew-wo");
    }

    #[test]
    fn supersession_surfaces_conflict_until_the_mutation_is_known() {
        let original = claim(
            "rel-asset-zone-old",
            "asset-17",
            "member_of_zone",
            "zone-A",
            T08,
            1,
        );
        let replacement = claim(
            "rel-asset-zone-new",
            "asset-17",
            "member_of_zone",
            "zone-B",
            T10,
            2,
        );
        let correction = mutation(
            "event-rel-correction",
            &original,
            TemporalRelationMutationKindV1::Supersede,
            T10,
            3,
            Some("rel-asset-zone-new"),
        );

        let known_before_correction = project_temporal_relationships(
            &[original.clone(), replacement.clone()],
            std::slice::from_ref(&correction),
            &cut(T11, 2),
        )
        .unwrap();
        assert_eq!(known_before_correction.active_relations.len(), 2);
        assert_eq!(known_before_correction.conflicts.len(), 1);

        let known_after_correction =
            project_temporal_relationships(&[original, replacement], &[correction], &cut(T11, 3))
                .unwrap();
        assert_eq!(known_after_correction.active_relations.len(), 1);
        assert_eq!(
            known_after_correction.active_relations[0].object_id,
            "zone-B"
        );
        assert!(known_after_correction.conflicts.is_empty());
        assert_eq!(
            known_after_correction.superseded_relation_ids,
            vec!["rel-asset-zone-old"]
        );
    }

    #[test]
    fn supersession_chains_are_sequence_ordered_and_permutation_invariant() {
        let original = claim(
            "rel-zone-original",
            "asset-17",
            "member_of_zone",
            "zone-A",
            T08,
            1,
        );
        let replacement = claim(
            "rel-zone-replacement",
            "asset-17",
            "member_of_zone",
            "zone-B",
            T10,
            2,
        );
        let final_relation = claim(
            "rel-zone-final",
            "asset-17",
            "member_of_zone",
            "zone-C",
            T12,
            4,
        );
        let first_mutation = mutation(
            "event-zone-correction-1",
            &original,
            TemporalRelationMutationKindV1::Supersede,
            T10,
            3,
            Some("rel-zone-replacement"),
        );
        let second_mutation = mutation(
            "event-zone-correction-2",
            &replacement,
            TemporalRelationMutationKindV1::Supersede,
            T12,
            5,
            Some("rel-zone-final"),
        );

        let ordered_claims = vec![
            original.clone(),
            replacement.clone(),
            final_relation.clone(),
        ];
        let ordered_mutations = vec![first_mutation.clone(), second_mutation.clone()];
        let canonical =
            project_temporal_relationships(&ordered_claims, &ordered_mutations, &cut(T13, 5))
                .unwrap();

        let mut reversed_claims = ordered_claims;
        reversed_claims.reverse();
        let reversed_mutations = vec![second_mutation, first_mutation];
        let permuted =
            project_temporal_relationships(&reversed_claims, &reversed_mutations, &cut(T13, 5))
                .unwrap();

        assert_eq!(canonical, permuted);
        assert_eq!(canonical.active_relations.len(), 1);
        assert_eq!(canonical.active_relations[0].relation_id, "rel-zone-final");
        assert_eq!(
            canonical.superseded_relation_ids,
            vec!["rel-zone-original", "rel-zone-replacement"]
        );
    }

    #[test]
    fn supersession_cannot_reactivate_an_already_retired_relation() {
        let original = claim(
            "rel-zone-original",
            "asset-17",
            "member_of_zone",
            "zone-A",
            T08,
            1,
        );
        let replacement = claim(
            "rel-zone-replacement",
            "asset-17",
            "member_of_zone",
            "zone-B",
            T10,
            2,
        );
        let first_mutation = mutation(
            "event-zone-correction-1",
            &original,
            TemporalRelationMutationKindV1::Supersede,
            T10,
            3,
            Some("rel-zone-replacement"),
        );
        let attempted_reactivation = mutation(
            "event-zone-correction-2",
            &replacement,
            TemporalRelationMutationKindV1::Supersede,
            T12,
            4,
            Some("rel-zone-original"),
        );

        assert_eq!(
            project_temporal_relationships(
                &[original, replacement],
                &[attempted_reactivation, first_mutation],
                &cut(T13, 4),
            ),
            Err(TemporalRelationshipErrorV1::InvalidReplacement)
        );
    }

    #[test]
    fn revocation_requires_both_valid_time_and_knowledge_frontier() {
        let original = claim(
            "rel-permit-crew",
            "permit-2",
            "authorizes",
            "crew-7",
            T08,
            1,
        );
        let revocation = mutation(
            "event-permit-revoked",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T12,
            2,
            None,
        );

        let before_effective_time = project_temporal_relationships(
            std::slice::from_ref(&original),
            std::slice::from_ref(&revocation),
            &cut(T11, 2),
        )
        .unwrap();
        assert_eq!(before_effective_time.active_relations.len(), 1);
        assert_eq!(
            before_effective_time.known_mutation_ids,
            vec!["event-permit-revoked"]
        );
        assert_eq!(
            before_effective_time.pending_mutation_ids,
            vec!["event-permit-revoked"]
        );
        assert!(before_effective_time.applied_mutation_ids.is_empty());

        let after_effective_before_knowledge = project_temporal_relationships(
            std::slice::from_ref(&original),
            std::slice::from_ref(&revocation),
            &cut(T13, 1),
        )
        .unwrap();
        assert_eq!(after_effective_before_knowledge.active_relations.len(), 1);

        let after_effective_and_knowledge =
            project_temporal_relationships(&[original], &[revocation], &cut(T13, 2)).unwrap();
        assert!(after_effective_and_knowledge.active_relations.is_empty());
        assert_eq!(
            after_effective_and_knowledge.revoked_relation_ids,
            vec!["rel-permit-crew"]
        );
    }

    #[test]
    fn pending_mutation_is_visible_and_changes_snapshot_receipt() {
        let original = claim(
            "rel-permit-crew",
            "permit-2",
            "authorizes",
            "crew-7",
            T08,
            1,
        );
        let future_revocation = mutation(
            "event-permit-revoked",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T12,
            2,
            None,
        );
        let cut = cut(T11, 2);

        let no_known_mutation =
            project_temporal_relationships(std::slice::from_ref(&original), &[], &cut).unwrap();
        let known_but_pending =
            project_temporal_relationships(&[original], &[future_revocation], &cut).unwrap();

        assert_eq!(known_but_pending.active_relations.len(), 1);
        assert_eq!(
            known_but_pending.pending_mutation_ids,
            vec!["event-permit-revoked"]
        );
        assert!(known_but_pending.applied_mutation_ids.is_empty());
        assert_ne!(
            no_known_mutation.snapshot_sha256,
            known_but_pending.snapshot_sha256
        );
    }

    #[test]
    fn future_malformed_records_do_not_poison_an_earlier_knowledge_frontier() {
        let known = claim("rel-known", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut future = claim("rel-future", "asset-2", "in_zone", "zone-2", T08, 99);
        future.valid_from = "not-a-time".to_owned();
        future.source_digest = [0; 32];
        let mut future_mutation = mutation(
            "event-future-invalid",
            &known,
            TemporalRelationMutationKindV1::Revoke,
            T10,
            100,
            None,
        );
        future_mutation.source_recorded_at = "not-a-time".to_owned();
        future_mutation.target_claim_digest = [0; 32];

        let receipt =
            project_temporal_relationships(&[known, future], &[future_mutation], &cut(T11, 1))
                .unwrap();
        assert_eq!(receipt.known_relation_ids, vec!["rel-known"]);
        assert_eq!(receipt.active_relations.len(), 1);
        assert!(receipt.known_mutation_ids.is_empty());
    }

    #[test]
    fn known_future_relation_is_reported_separately_from_active_edges() {
        let mut future = claim(
            "rel-future",
            "crew-1",
            "assigned_to",
            "work-order-1",
            T12,
            1,
        );
        future.source_recorded_at = T08.to_owned();
        let receipt = project_temporal_relationships(&[future], &[], &cut(T11, 1)).unwrap();

        assert_eq!(receipt.known_relation_ids, vec!["rel-future"]);
        assert!(receipt.visible_relation_ids.is_empty());
        assert_eq!(receipt.pending_relation_ids, vec!["rel-future"]);
        assert!(receipt.expired_relation_ids.is_empty());
        assert!(receipt.active_relations.is_empty());
    }

    #[test]
    fn known_but_not_active_claim_content_is_bound_into_snapshot_digest() {
        let mut future_a = claim("rel-future", "asset-1", "member_of_zone", "zone-A", T12, 1);
        future_a.source_recorded_at = T08.to_owned();
        let mut future_b = future_a.clone();
        future_b.object_id = "zone-B".to_owned();

        let first = project_temporal_relationships(&[future_a], &[], &cut(T11, 1)).unwrap();
        let second = project_temporal_relationships(&[future_b], &[], &cut(T11, 1)).unwrap();

        assert!(first.active_relations.is_empty());
        assert_eq!(first.pending_relation_ids, vec!["rel-future"]);
        assert_ne!(first.snapshot_sha256, second.snapshot_sha256);
    }

    #[test]
    fn many_to_many_predicates_allow_multiple_active_endpoint_pairs() {
        let mut first = claim("rel-1", "crew-1", "collaborates_with", "crew-2", T08, 1);
        let mut second = claim("rel-2", "crew-1", "collaborates_with", "crew-3", T08, 2);
        let mut third = claim("rel-3", "crew-4", "collaborates_with", "crew-2", T08, 3);
        for relation in [&mut first, &mut second, &mut third] {
            relation.cardinality = TemporalRelationCardinalityV1::ManyToMany;
        }

        let receipt =
            project_temporal_relationships(&[first, second, third], &[], &cut(T11, 3)).unwrap();
        assert_eq!(receipt.active_relations.len(), 3);
        assert!(receipt.conflicts.is_empty());
    }

    #[test]
    fn one_to_many_and_many_to_one_enforce_the_correct_endpoint() {
        let mut one_to_many_a = claim(
            "rel-otm-a",
            "crew-1",
            "has_assignment",
            "work-order-1",
            T08,
            1,
        );
        let mut one_to_many_b = claim(
            "rel-otm-b",
            "crew-2",
            "has_assignment",
            "work-order-1",
            T08,
            2,
        );
        one_to_many_a.cardinality = TemporalRelationCardinalityV1::OneToMany;
        one_to_many_b.cardinality = TemporalRelationCardinalityV1::OneToMany;
        let one_to_many =
            project_temporal_relationships(&[one_to_many_a, one_to_many_b], &[], &cut(T11, 2))
                .unwrap();
        assert_eq!(one_to_many.conflicts.len(), 1);
        assert_eq!(
            one_to_many.conflicts[0].side,
            TemporalRelationshipConflictSideV1::Object
        );

        let mut many_to_one_a = claim("rel-mto-a", "asset-1", "member_of_zone", "zone-1", T08, 1);
        let mut many_to_one_b = claim("rel-mto-b", "asset-1", "member_of_zone", "zone-2", T08, 2);
        many_to_one_a.cardinality = TemporalRelationCardinalityV1::ManyToOne;
        many_to_one_b.cardinality = TemporalRelationCardinalityV1::ManyToOne;
        let many_to_one =
            project_temporal_relationships(&[many_to_one_a, many_to_one_b], &[], &cut(T11, 2))
                .unwrap();
        assert_eq!(many_to_one.conflicts.len(), 1);
        assert_eq!(
            many_to_one.conflicts[0].side,
            TemporalRelationshipConflictSideV1::Subject
        );
    }

    #[test]
    fn conflicting_cardinality_policy_for_one_predicate_fails_closed() {
        let one = claim("rel-one", "asset-1", "member_of_zone", "zone-1", T08, 1);
        let mut many = claim("rel-many", "asset-2", "member_of_zone", "zone-2", T08, 2);
        many.cardinality = TemporalRelationCardinalityV1::ManyToMany;

        assert_eq!(
            project_temporal_relationships(&[one, many], &[], &cut(T11, 2)),
            Err(TemporalRelationshipErrorV1::CardinalityPolicyMismatch)
        );
    }

    #[test]
    fn event_array_permutation_preserves_snapshot_digest() {
        let first = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let second = claim("rel-2", "crew-1", "assigned_to", "work-order-1", T08, 2);
        let original = vec![first.clone(), second.clone()];
        let reordered = vec![second, first];

        let a = project_temporal_relationships(&original, &[], &cut(T11, 2)).unwrap();
        let b = project_temporal_relationships(&reordered, &[], &cut(T11, 2)).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn target_claim_digest_prevents_relation_id_rebinding() {
        let original = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut altered = original.clone();
        altered.object_id = "zone-2".to_owned();
        let mut attempted = mutation(
            "event-rel-revoke",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T10,
            2,
            None,
        );
        attempted.target_claim_digest = temporal_relation_claim_digest(&altered);

        assert_eq!(
            project_temporal_relationships(&[original], &[attempted], &cut(T11, 2)),
            Err(TemporalRelationshipErrorV1::TargetClaimDigestMismatch)
        );
    }

    #[test]
    fn authority_scope_mismatch_fails_closed() {
        let original = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut attempted = mutation(
            "event-rel-revoke",
            &original,
            TemporalRelationMutationKindV1::Revoke,
            T10,
            2,
            None,
        );
        attempted.authority_scope = "scope:other-zone".to_owned();

        assert_eq!(
            project_temporal_relationships(&[original], &[attempted], &cut(T11, 2)),
            Err(TemporalRelationshipErrorV1::MutationScopeMismatch)
        );
    }

    #[test]
    fn duplicate_accepted_sequence_fails_closed_across_claims_and_events() {
        let first = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        let mut second = claim("rel-2", "crew-1", "assigned_to", "work-order-1", T08, 1);
        second.accepted_sequence = 1;

        assert_eq!(
            project_temporal_relationships(&[first, second], &[], &cut(T11, 1)),
            Err(TemporalRelationshipErrorV1::DuplicateAcceptedSequence)
        );
    }

    #[test]
    fn half_open_validity_interval_excludes_edge_at_expiry() {
        let mut expiring = claim("rel-1", "asset-1", "in_zone", "zone-1", T08, 1);
        expiring.valid_to = Some(T10.to_owned());

        let before =
            project_temporal_relationships(&[expiring.clone()], &[], &cut(T08, 1)).unwrap();
        let at_expiry = project_temporal_relationships(&[expiring], &[], &cut(T10, 1)).unwrap();
        assert_eq!(before.active_relations.len(), 1);
        assert!(at_expiry.active_relations.is_empty());
    }
}
