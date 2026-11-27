#![forbid(unsafe_code)]

//! Dual-frontier replay for the synthetic OCEL water-service log.
//!
//! event.time is the event's valid-time coordinate. host_sequence is an
//! append-order knowledge frontier; it is not a trusted wall-clock timestamp.
//! source_recorded_at is retained and validated as source metadata, but is
//! not silently promoted to platform transaction time. This module therefore
//! implements valid-time plus accepted-sequence replay, not a full bitemporal
//! database or an authenticity/authority verifier.

use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const FIXTURE: &str = include_str!("../../ocel/ocel-water-service-v1.json");

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalReplayErrorV1 {
    InvalidJson,
    MissingEvents,
    InvalidEvent,
    EmptyIdentity,
    InvalidUtcTimestamp,
    InvalidHostSequence,
    DuplicateEventId,
    DuplicateHostSequence,
    MissingMutationTarget,
    UnknownMutationTarget,
    NonCausalMutation,
    MutationScopeMismatch,
    ConflictingMutations,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalReplayCutV1 {
    /// Inclusive valid-time cutoff. V1 requires canonical UTC seconds ending in Z.
    pub valid_time_as_of: String,
    /// Inclusive host-assigned append sequence; this is not a wall-clock time.
    pub known_through_sequence: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalReplayReceiptV1 {
    pub valid_time_as_of: String,
    pub known_through_sequence: u64,
    pub visible_event_ids: Vec<String>,
    /// Evidence-bearing events after applying visible corrections/revocations.
    /// A correction event replaces its target; a revocation event is metadata.
    pub effective_event_ids: Vec<String>,
    pub superseded_event_ids: Vec<String>,
    pub revoked_event_ids: Vec<String>,
    pub applied_mutation_event_ids: Vec<String>,
    pub replay_sha256: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum EventKind {
    Assertion,
    Correction,
    Revocation,
}

#[derive(Clone, Debug)]
struct ReplayEvent {
    id: String,
    valid_time: String,
    host_sequence: u64,
    kind: EventKind,
    mutation_target: Option<String>,
    related_object_ids: BTreeSet<String>,
}

fn non_empty_text(value: Option<&Value>) -> Option<&str> {
    value
        .and_then(Value::as_str)
        .filter(|text| !text.trim().is_empty() && text.trim() == *text)
}

fn attribute_value<'a>(event: &'a Value, name: &str) -> Option<&'a Value> {
    event
        .get("attributes")?
        .as_array()?
        .iter()
        .find(|attribute| attribute.get("name").and_then(Value::as_str) == Some(name))?
        .get("value")
}

/// V1 replay uses fixed-width UTC timestamps so lexical ordering is chronological.
/// General RFC3339 offsets and fractional precision remain profile-validator concerns.
fn valid_utc_second(value: &str) -> bool {
    let bytes = value.as_bytes();
    bytes.len() == 20
        && bytes[4] == b'-'
        && bytes[7] == b'-'
        && bytes[10] == b'T'
        && bytes[13] == b':'
        && bytes[16] == b':'
        && bytes[19] == b'Z'
        && [0..4, 5..7, 8..10, 11..13, 14..16, 17..19]
            .iter()
            .all(|range| {
                bytes
                    .get(range.clone())
                    .is_some_and(|slice| slice.iter().all(u8::is_ascii_digit))
            })
        && {
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
            let days = match month {
                1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
                4 | 6 | 9 | 11 => 30,
                2 if leap => 29,
                2 => 28,
                _ => 0,
            };
            year > 0 && day > 0 && day <= days && hour <= 23 && minute <= 59 && second <= 60
        }
}

fn parse_events(input: &str) -> Result<Vec<ReplayEvent>, TemporalReplayErrorV1> {
    let root: Value =
        serde_json::from_str(input).map_err(|_| TemporalReplayErrorV1::InvalidJson)?;
    let events = root
        .get("events")
        .and_then(Value::as_array)
        .ok_or(TemporalReplayErrorV1::MissingEvents)?;

    let mut parsed = Vec::with_capacity(events.len());
    let mut ids = BTreeSet::new();
    let mut sequences = BTreeSet::new();

    for event in events {
        let id = non_empty_text(event.get("id")).ok_or(TemporalReplayErrorV1::EmptyIdentity)?;
        let event_type =
            non_empty_text(event.get("type")).ok_or(TemporalReplayErrorV1::InvalidEvent)?;
        let valid_time =
            non_empty_text(event.get("time")).ok_or(TemporalReplayErrorV1::InvalidEvent)?;
        let recorded_at = non_empty_text(attribute_value(event, "source_recorded_at"));
        let source_ref = non_empty_text(attribute_value(event, "source_ref"));
        let source_digest = non_empty_text(attribute_value(event, "source_digest"));
        let sequence = attribute_value(event, "host_sequence")
            .and_then(Value::as_u64)
            .filter(|sequence| *sequence > 0)
            .ok_or(TemporalReplayErrorV1::InvalidHostSequence)?;

        if !valid_utc_second(valid_time) || !recorded_at.is_some_and(valid_utc_second) {
            return Err(TemporalReplayErrorV1::InvalidUtcTimestamp);
        }
        if source_ref.is_none() || source_digest.is_none() {
            return Err(TemporalReplayErrorV1::InvalidEvent);
        }
        if !ids.insert(id.to_owned()) {
            return Err(TemporalReplayErrorV1::DuplicateEventId);
        }
        if !sequences.insert(sequence) {
            return Err(TemporalReplayErrorV1::DuplicateHostSequence);
        }

        let (kind, target_name) = match event_type {
            "source_assertion_corrected" => (EventKind::Correction, "supersedes_event_id"),
            "source_assertion_revoked" => (EventKind::Revocation, "revokes_event_id"),
            _ => (EventKind::Assertion, ""),
        };
        let mutation_target = if target_name.is_empty() {
            None
        } else {
            Some(
                non_empty_text(attribute_value(event, target_name))
                    .ok_or(TemporalReplayErrorV1::MissingMutationTarget)?
                    .to_owned(),
            )
        };
        let relationships = event
            .get("relationships")
            .and_then(Value::as_array)
            .filter(|relationships| !relationships.is_empty())
            .ok_or(TemporalReplayErrorV1::InvalidEvent)?;
        let mut related_object_ids = BTreeSet::new();
        for relationship in relationships {
            let object_id = non_empty_text(relationship.get("objectId"))
                .ok_or(TemporalReplayErrorV1::InvalidEvent)?;
            related_object_ids.insert(object_id.to_owned());
        }

        parsed.push(ReplayEvent {
            id: id.to_owned(),
            valid_time: valid_time.to_owned(),
            host_sequence: sequence,
            kind,
            mutation_target,
            related_object_ids,
        });
    }

    parsed.sort_by_key(|event| event.host_sequence);
    let by_id: BTreeMap<_, _> = parsed
        .iter()
        .map(|event| (event.id.as_str(), event))
        .collect();
    let mut mutated_targets = BTreeSet::new();

    for event in &parsed {
        let Some(target_id) = event.mutation_target.as_deref() else {
            continue;
        };
        let target = by_id
            .get(target_id)
            .ok_or(TemporalReplayErrorV1::UnknownMutationTarget)?;
        if target.host_sequence >= event.host_sequence
            || target.valid_time > event.valid_time
            || target.kind != EventKind::Assertion
        {
            return Err(TemporalReplayErrorV1::NonCausalMutation);
        }
        if !target
            .related_object_ids
            .iter()
            .any(|object_id| event.related_object_ids.contains(object_id))
        {
            return Err(TemporalReplayErrorV1::MutationScopeMismatch);
        }
        if !mutated_targets.insert(target_id.to_owned()) {
            return Err(TemporalReplayErrorV1::ConflictingMutations);
        }
    }

    Ok(parsed)
}

/// Reconstruct a log view using an inclusive valid-time cutoff and an inclusive
/// accepted-sequence cutoff. Results do not depend on input array order.
pub fn replay_temporal_frontier(
    input: &str,
    cut: &TemporalReplayCutV1,
) -> Result<TemporalReplayReceiptV1, TemporalReplayErrorV1> {
    if !valid_utc_second(&cut.valid_time_as_of) {
        return Err(TemporalReplayErrorV1::InvalidUtcTimestamp);
    }

    let events = parse_events(input)?;
    let visible: Vec<_> = events
        .iter()
        .filter(|event| {
            event.host_sequence <= cut.known_through_sequence
                && event.valid_time <= cut.valid_time_as_of
        })
        .collect();
    let visible_ids: BTreeSet<_> = visible.iter().map(|event| event.id.as_str()).collect();

    let mut superseded = BTreeSet::new();
    let mut revoked = BTreeSet::new();
    let mut applied_mutations = Vec::new();

    for event in &visible {
        if let Some(target) = event.mutation_target.as_deref() {
            if !visible_ids.contains(target) {
                return Err(TemporalReplayErrorV1::NonCausalMutation);
            }
            match event.kind {
                EventKind::Correction => {
                    superseded.insert(target.to_owned());
                    applied_mutations.push(event.id.clone());
                }
                EventKind::Revocation => {
                    revoked.insert(target.to_owned());
                    applied_mutations.push(event.id.clone());
                }
                EventKind::Assertion => {}
            }
        }
    }

    let retired: BTreeSet<_> = superseded.union(&revoked).cloned().collect();
    let effective_event_ids = visible
        .iter()
        .filter(|event| event.kind != EventKind::Revocation && !retired.contains(&event.id))
        .map(|event| event.id.clone())
        .collect::<Vec<_>>();
    let visible_event_ids = visible.iter().map(|event| event.id.clone()).collect();

    let canonical = format!(
        "valid_time_as_of={}\nknown_through_sequence={}\nvisible={}\neffective={}\nsuperseded={}\nrevoked={}\nmutations={}\n",
        cut.valid_time_as_of,
        cut.known_through_sequence,
        visible_event_ids.join(","),
        effective_event_ids.join(","),
        superseded.iter().cloned().collect::<Vec<_>>().join(","),
        revoked.iter().cloned().collect::<Vec<_>>().join(","),
        applied_mutations.join(","),
    );
    let digest = Sha256::digest(canonical.as_bytes());
    let replay_sha256 = digest.iter().map(|byte| format!("{byte:02x}")).collect();

    Ok(TemporalReplayReceiptV1 {
        valid_time_as_of: cut.valid_time_as_of.clone(),
        known_through_sequence: cut.known_through_sequence,
        visible_event_ids,
        effective_event_ids,
        superseded_event_ids: superseded.into_iter().collect(),
        revoked_event_ids: revoked.into_iter().collect(),
        applied_mutation_event_ids: applied_mutations,
        replay_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("synthetic OCEL fixture parses")
    }

    fn input(value: &Value) -> String {
        serde_json::to_string(value).expect("fixture serializes")
    }

    fn cut(valid_time_as_of: &str, known_through_sequence: u64) -> TemporalReplayCutV1 {
        TemporalReplayCutV1 {
            valid_time_as_of: valid_time_as_of.to_owned(),
            known_through_sequence,
        }
    }

    #[test]
    fn accepted_sequence_frontier_preserves_what_was_known_then() {
        let source = FIXTURE;
        let before_correction =
            replay_temporal_frontier(source, &cut("2026-11-27T16:00:00Z", 10)).unwrap();
        assert!(before_correction
            .effective_event_ids
            .iter()
            .any(|id| id == "event-006"));
        assert!(before_correction
            .effective_event_ids
            .iter()
            .any(|id| id == "event-008"));
        assert!(!before_correction
            .visible_event_ids
            .iter()
            .any(|id| id == "event-011"));

        let after_correction =
            replay_temporal_frontier(source, &cut("2026-11-27T16:00:00Z", 11)).unwrap();
        assert!(after_correction
            .superseded_event_ids
            .iter()
            .any(|id| id == "event-006"));
        assert!(after_correction
            .effective_event_ids
            .iter()
            .any(|id| id == "event-011"));
        assert!(after_correction
            .effective_event_ids
            .iter()
            .any(|id| id == "event-008"));

        let after_revocation =
            replay_temporal_frontier(source, &cut("2026-11-27T16:00:00Z", 12)).unwrap();
        assert!(after_revocation
            .revoked_event_ids
            .iter()
            .any(|id| id == "event-008"));
        assert!(!after_revocation
            .effective_event_ids
            .iter()
            .any(|id| id == "event-008"));
        assert!(after_revocation
            .effective_event_ids
            .iter()
            .any(|id| id == "event-011"));
    }

    #[test]
    fn appending_later_mutations_does_not_rewrite_an_earlier_frontier_receipt() {
        let mut prefix = fixture();
        prefix["events"]
            .as_array_mut()
            .unwrap()
            .retain(|event| {
                !matches!(
                    event.get("id").and_then(Value::as_str),
                    Some("event-011" | "event-012")
                )
            });
        let cut = cut("2026-11-27T16:00:00Z", 10);
        let prefix_receipt = replay_temporal_frontier(&input(&prefix), &cut).unwrap();
        let extended_receipt = replay_temporal_frontier(FIXTURE, &cut).unwrap();
        assert_eq!(prefix_receipt, extended_receipt);
    }

    #[test]
    fn valid_time_frontier_excludes_a_correction_before_its_effective_time() {
        let receipt = replay_temporal_frontier(FIXTURE, &cut("2026-11-27T11:40:00Z", 11)).unwrap();
        assert!(!receipt.visible_event_ids.iter().any(|id| id == "event-011"));
        assert!(receipt
            .effective_event_ids
            .iter()
            .any(|id| id == "event-006"));
        assert!(receipt
            .effective_event_ids
            .iter()
            .any(|id| id == "event-008"));
    }

    #[test]
    fn late_recorded_correction_uses_separate_valid_and_knowledge_frontiers() {
        let valid_time_cut = "2026-11-27T12:00:00Z";
        let before_acceptance =
            replay_temporal_frontier(FIXTURE, &cut(valid_time_cut, 10)).unwrap();
        let after_acceptance =
            replay_temporal_frontier(FIXTURE, &cut(valid_time_cut, 11)).unwrap();

        assert!(!before_acceptance.visible_event_ids.iter().any(|id| id == "event-011"));
        assert!(before_acceptance.effective_event_ids.iter().any(|id| id == "event-006"));
        assert!(after_acceptance.visible_event_ids.iter().any(|id| id == "event-011"));
        assert!(after_acceptance.superseded_event_ids.iter().any(|id| id == "event-006"));
    }

    #[test]
    fn mutation_cannot_cross_service_object_scope() {
        let mut value = fixture();
        for relationship in value["events"][10]["relationships"].as_array_mut().unwrap() {
            relationship["objectId"] = Value::String("unrelated-service-object".to_owned());
        }
        assert_eq!(
            replay_temporal_frontier(&input(&value), &cut("2026-11-27T16:00:00Z", 12)),
            Err(TemporalReplayErrorV1::MutationScopeMismatch)
        );
    }

    #[test]
    fn event_array_permutation_does_not_change_replay_receipt() {
        let original = fixture();
        let mut reordered = original;
        reordered["events"].as_array_mut().unwrap().reverse();
        let first =
            replay_temporal_frontier(&input(&fixture()), &cut("2026-11-27T16:00:00Z", 12)).unwrap();
        let second =
            replay_temporal_frontier(&input(&reordered), &cut("2026-11-27T16:00:00Z", 12)).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn duplicate_host_sequence_fails_closed() {
        let mut value = fixture();
        value["events"][1]["attributes"][1]["value"] = Value::from(1);
        assert_eq!(
            replay_temporal_frontier(&input(&value), &cut("2026-11-27T16:00:00Z", 12)),
            Err(TemporalReplayErrorV1::DuplicateHostSequence)
        );
    }

    #[test]
    fn unknown_mutation_target_fails_closed() {
        let mut value = fixture();
        value["events"][10]["attributes"][4]["value"] = Value::String("event-999".to_owned());
        assert_eq!(
            replay_temporal_frontier(&input(&value), &cut("2026-11-27T16:00:00Z", 12)),
            Err(TemporalReplayErrorV1::UnknownMutationTarget)
        );
    }

    #[test]
    fn source_recorded_time_is_validated_but_not_used_as_host_transaction_time() {
        let mut value = fixture();
        value["events"][10]["attributes"][0]["value"] = Value::String("not-a-time".to_owned());
        assert_eq!(
            replay_temporal_frontier(&input(&value), &cut("2026-11-27T16:00:00Z", 12)),
            Err(TemporalReplayErrorV1::InvalidUtcTimestamp)
        );
    }
}
