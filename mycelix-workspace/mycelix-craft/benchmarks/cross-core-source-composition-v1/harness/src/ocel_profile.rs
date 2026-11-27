#![forbid(unsafe_code)]

//! Restricted OCEL 2.0 JSON profile for the synthetic water-service benchmark.
//!
//! This module validates the profile used by the frozen WSR fixture. It is not
//! a complete OCEL 2.0 schema implementation and does not verify source truth,
//! evidence independence, temporal authority, or the semantics of a service
//! conclusion. Owner-native Craft packets remain the assurance layer.

use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const FIXTURE: &str = include_str!("../../ocel/ocel-water-service-v1.json");

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OcelProfileErrorV1 {
    InvalidJson,
    MissingTopLevelCollection,
    InvalidTypeDeclaration,
    DuplicateTypeName,
    DuplicateAttributeDeclaration,
    UnsupportedAttributeType,
    InvalidObject,
    DuplicateObjectId,
    UnknownObjectType,
    InvalidEvent,
    DuplicateEventId,
    UnknownEventType,
    UnknownObjectReference,
    InvalidAttribute,
    DuplicateAttribute,
    AttributeTypeMismatch,
    InvalidTimestamp,
    InvalidRelationship,
    DuplicateRelationship,
    EventWithoutObjectRelationship,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OcelProfileReceiptV1 {
    pub object_type_count: usize,
    pub event_type_count: usize,
    pub object_count: usize,
    pub event_count: usize,
    pub event_to_object_relationship_count: usize,
    pub object_to_object_relationship_count: usize,
    pub canonical_sha256: String,
    pub canonical_json: String,
}

type AttributeSchema = BTreeMap<String, String>;

fn non_empty_text(value: &Value) -> Option<&str> {
    value
        .as_str()
        .filter(|text| !text.trim().is_empty() && text.trim() == *text)
}

fn array<'a>(root: &'a Value, key: &str) -> Result<&'a [Value], OcelProfileErrorV1> {
    root.get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or(OcelProfileErrorV1::MissingTopLevelCollection)
}

fn attribute_schema(
    declaration: &Value,
) -> Result<AttributeSchema, OcelProfileErrorV1> {
    let attributes = declaration
        .get("attributes")
        .and_then(Value::as_array)
        .ok_or(OcelProfileErrorV1::InvalidTypeDeclaration)?;
    let mut schema = AttributeSchema::new();

    for attribute in attributes {
        let name = attribute
            .get("name")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidTypeDeclaration)?;
        let kind = attribute
            .get("type")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidTypeDeclaration)?;

        if !matches!(kind, "string" | "time" | "integer" | "float" | "boolean") {
            return Err(OcelProfileErrorV1::UnsupportedAttributeType);
        }
        if schema.insert(name.to_owned(), kind.to_owned()).is_some() {
            return Err(OcelProfileErrorV1::DuplicateAttributeDeclaration);
        }
    }

    Ok(schema)
}

fn type_registry(
    declarations: &[Value],
) -> Result<BTreeMap<String, AttributeSchema>, OcelProfileErrorV1> {
    let mut registry = BTreeMap::new();
    for declaration in declarations {
        let name = declaration
            .get("name")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidTypeDeclaration)?;
        let schema = attribute_schema(declaration)?;
        if registry.insert(name.to_owned(), schema).is_some() {
            return Err(OcelProfileErrorV1::DuplicateTypeName);
        }
    }
    Ok(registry)
}

/// Validate the timestamp subset used by the WSR profile.
///
/// The profile accepts RFC3339-style full timestamps with an explicit timezone.
/// It deliberately rejects naive/local timestamps rather than guessing a zone.
fn valid_timestamp(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() < 20
        || bytes.get(4) != Some(&b'-')
        || bytes.get(7) != Some(&b'-')
        || bytes.get(10) != Some(&b'T')
        || bytes.get(13) != Some(&b':')
        || bytes.get(16) != Some(&b':')
    {
        return false;
    }

    let digits = |range: std::ops::Range<usize>| {
        bytes
            .get(range)
            .is_some_and(|slice| slice.iter().all(u8::is_ascii_digit))
    };
    if !digits(0..4)
        || !digits(5..7)
        || !digits(8..10)
        || !digits(11..13)
        || !digits(14..16)
        || !digits(17..19)
    {
        return false;
    }

    let number = |start: usize, end: usize| -> Option<u32> {
        std::str::from_utf8(bytes.get(start..end)?)
            .ok()?
            .parse()
            .ok()
    };

    let month = number(5, 7).unwrap_or(0);
    let day = number(8, 10).unwrap_or(0);
    let hour = number(11, 13).unwrap_or(u32::MAX);
    let minute = number(14, 16).unwrap_or(u32::MAX);
    let second = number(17, 19).unwrap_or(u32::MAX);

    if !(1..=12).contains(&month)
        || !(1..=31).contains(&day)
        || hour > 23
        || minute > 59
        || second > 60
    {
        return false;
    }

    let mut cursor = 19;
    if bytes.get(cursor) == Some(&b'.') {
        cursor += 1;
        let start = cursor;
        while bytes.get(cursor).is_some_and(u8::is_ascii_digit) {
            cursor += 1;
        }
        if cursor == start {
            return false;
        }
    }

    match bytes.get(cursor) {
        Some(&b'Z') => cursor + 1 == bytes.len(),
        Some(&b'+') | Some(&b'-') => {
            if cursor + 6 != bytes.len()
                || bytes.get(cursor + 3) != Some(&b':')
                || !bytes
                    .get(cursor + 1..cursor + 3)
                    .is_some_and(|slice| slice.iter().all(u8::is_ascii_digit))
                || !bytes
                    .get(cursor + 4..cursor + 6)
                    .is_some_and(|slice| slice.iter().all(u8::is_ascii_digit))
            {
                return false;
            }
            let offset_hour = number(cursor + 1, cursor + 3).unwrap_or(u32::MAX);
            let offset_minute = number(cursor + 4, cursor + 6).unwrap_or(u32::MAX);
            offset_hour <= 23 && offset_minute <= 59
        }
        _ => false,
    }
}

fn value_matches(kind: &str, value: &Value) -> bool {
    match kind {
        "string" => value.is_string(),
        "time" => value.as_str().is_some_and(valid_timestamp),
        "integer" => value.as_i64().is_some() || value.as_u64().is_some(),
        "float" => value.as_f64().is_some_and(f64::is_finite),
        "boolean" => value.is_boolean(),
        _ => false,
    }
}

fn validate_attributes(
    values: &[Value],
    schema: &AttributeSchema,
    object_attributes: bool,
) -> Result<(), OcelProfileErrorV1> {
    let mut seen = BTreeSet::new();

    for attribute in values {
        let name = attribute
            .get("name")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidAttribute)?;
        let kind = schema
            .get(name)
            .ok_or(OcelProfileErrorV1::InvalidAttribute)?;
        let value = attribute
            .get("value")
            .ok_or(OcelProfileErrorV1::InvalidAttribute)?;

        if !value_matches(kind, value) {
            return Err(if kind.as_str() == "time" {
                OcelProfileErrorV1::InvalidTimestamp
            } else {
                OcelProfileErrorV1::AttributeTypeMismatch
            });
        }

        let identity = if object_attributes {
            let time = attribute
                .get("time")
                .and_then(non_empty_text)
                .filter(|time| valid_timestamp(time))
                .ok_or(OcelProfileErrorV1::InvalidTimestamp)?;
            format!("{name}\u{1f}{time}")
        } else {
            name.to_owned()
        };

        if !seen.insert(identity) {
            return Err(OcelProfileErrorV1::DuplicateAttribute);
        }
    }

    Ok(())
}

fn relationships(
    value: &Value,
    key: &str,
) -> Result<&[Value], OcelProfileErrorV1> {
    match value.get(key) {
        None => Ok(&[]),
        Some(value) => value
            .as_array()
            .map(Vec::as_slice)
            .ok_or(OcelProfileErrorV1::InvalidRelationship),
    }
}

fn validate_relationships(
    source: &Value,
    key: &str,
    object_ids: &BTreeSet<String>,
) -> Result<usize, OcelProfileErrorV1> {
    let mut seen = BTreeSet::new();
    let relations = relationships(source, key)?;

    for relation in relations {
        let target = relation
            .get("objectId")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidRelationship)?;
        let qualifier = relation
            .get("qualifier")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidRelationship)?;

        if !object_ids.contains(target) {
            return Err(OcelProfileErrorV1::UnknownObjectReference);
        }
        if !seen.insert((target.to_owned(), qualifier.to_owned())) {
            return Err(OcelProfileErrorV1::DuplicateRelationship);
        }
    }

    Ok(relations.len())
}

fn validate_profile(value: &Value) -> Result<OcelProfileReceiptV1, OcelProfileErrorV1> {
    let object_types = type_registry(array(value, "objectTypes")?)?;
    let event_types = type_registry(array(value, "eventTypes")?)?;
    let objects = array(value, "objects")?;
    let events = array(value, "events")?;

    if object_types.is_empty() || event_types.is_empty() {
        return Err(OcelProfileErrorV1::InvalidTypeDeclaration);
    }

    let mut object_ids = BTreeSet::new();
    for object in objects {
        let object_id = object
            .get("id")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidObject)?;
        let type_name = object
            .get("type")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidObject)?;
        let schema = object_types
            .get(type_name)
            .ok_or(OcelProfileErrorV1::UnknownObjectType)?;
        if !object_ids.insert(object_id.to_owned()) {
            return Err(OcelProfileErrorV1::DuplicateObjectId);
        }

        let attributes = object
            .get("attributes")
            .and_then(Value::as_array)
            .ok_or(OcelProfileErrorV1::InvalidObject)?;
        validate_attributes(attributes, schema, true)?;
    }

    let mut event_ids = BTreeSet::new();
    let mut event_to_object_count = 0usize;
    for event in events {
        let event_id = event
            .get("id")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidEvent)?;
        let type_name = event
            .get("type")
            .and_then(non_empty_text)
            .ok_or(OcelProfileErrorV1::InvalidEvent)?;
        let event_time = event
            .get("time")
            .and_then(non_empty_text)
            .filter(|time| valid_timestamp(time))
            .ok_or(OcelProfileErrorV1::InvalidTimestamp)?;
        let _ = event_time;

        if !event_ids.insert(event_id.to_owned()) {
            return Err(OcelProfileErrorV1::DuplicateEventId);
        }

        let schema = event_types
            .get(type_name)
            .ok_or(OcelProfileErrorV1::UnknownEventType)?;
        let attributes = event
            .get("attributes")
            .and_then(Value::as_array)
            .ok_or(OcelProfileErrorV1::InvalidEvent)?;
        validate_attributes(attributes, schema, false)?;

        let relation_count = validate_relationships(event, "relationships", &object_ids)?;
        if relation_count == 0 {
            return Err(OcelProfileErrorV1::EventWithoutObjectRelationship);
        }
        event_to_object_count += relation_count;
    }

    let mut object_to_object_count = 0usize;
    for object in objects {
        object_to_object_count +=
            validate_relationships(object, "relationships", &object_ids)?;
    }

    let canonical_json = canonical_json(value)?;
    let digest = Sha256::digest(canonical_json.as_bytes());
    let canonical_sha256 = digest.iter().map(|byte| format!("{byte:02x}")).collect();

    Ok(OcelProfileReceiptV1 {
        object_type_count: object_types.len(),
        event_type_count: event_types.len(),
        object_count: objects.len(),
        event_count: events.len(),
        event_to_object_relationship_count: event_to_object_count,
        object_to_object_relationship_count: object_to_object_count,
        canonical_sha256,
        canonical_json,
    })
}

/// Validate a restricted water-service profile of OCEL 2.0 JSON.
///
/// The profile checks the four OCEL collections, event/object type references,
/// typed attributes, timestamp syntax, and qualified event-to-object and
/// object-to-object references. Extra JSON fields are preserved by the
/// canonicalizer; this is not a full standard-schema implementation.
pub fn validate_ocel20_water_profile(
    input: &str,
) -> Result<OcelProfileReceiptV1, OcelProfileErrorV1> {
    let value: Value =
        serde_json::from_str(input).map_err(|_| OcelProfileErrorV1::InvalidJson)?;
    validate_profile(&value)
}

/// Canonical JSON for the restricted profile.
///
/// Set-like collections and relation/attribute arrays are sorted by stable
/// semantic keys. Unknown extension values are retained, while their object
/// keys are serialized deterministically by serde_json's ordered map.
pub fn canonical_json(input: &Value) -> Result<String, OcelProfileErrorV1> {
    let mut value = input.clone();

    for key in ["objectTypes", "eventTypes"] {
        let collection = value
            .get_mut(key)
            .and_then(Value::as_array_mut)
            .ok_or(OcelProfileErrorV1::MissingTopLevelCollection)?;
        for declaration in collection.iter_mut() {
            if let Some(attributes) = declaration
                .get_mut("attributes")
                .and_then(Value::as_array_mut)
            {
                attributes.sort_by(|left, right| {
                    left.get("name")
                        .and_then(Value::as_str)
                        .cmp(&right.get("name").and_then(Value::as_str))
                });
            }
        }
        collection.sort_by(|left, right| {
            left.get("name")
                .and_then(Value::as_str)
                .cmp(&right.get("name").and_then(Value::as_str))
        });
    }

    for object in value
        .get_mut("objects")
        .and_then(Value::as_array_mut)
        .ok_or(OcelProfileErrorV1::MissingTopLevelCollection)?
    {
        if let Some(attributes) = object
            .get_mut("attributes")
            .and_then(Value::as_array_mut)
        {
            attributes.sort_by(|left, right| {
                left.get("name")
                    .and_then(Value::as_str)
                    .cmp(&right.get("name").and_then(Value::as_str))
                    .then_with(|| {
                        left.get("time")
                            .and_then(Value::as_str)
                            .cmp(&right.get("time").and_then(Value::as_str))
                    })
            });
        }
        if let Some(relations) = object
            .get_mut("relationships")
            .and_then(Value::as_array_mut)
        {
            relations.sort_by(|left, right| {
                left.get("objectId")
                    .and_then(Value::as_str)
                    .cmp(&right.get("objectId").and_then(Value::as_str))
                    .then_with(|| {
                        left.get("qualifier")
                            .and_then(Value::as_str)
                            .cmp(&right.get("qualifier").and_then(Value::as_str))
                    })
            });
        }
    }
    value
        .get_mut("objects")
        .and_then(Value::as_array_mut)
        .ok_or(OcelProfileErrorV1::MissingTopLevelCollection)?
        .sort_by(|left, right| {
            left.get("id")
                .and_then(Value::as_str)
                .cmp(&right.get("id").and_then(Value::as_str))
        });

    for event in value
        .get_mut("events")
        .and_then(Value::as_array_mut)
        .ok_or(OcelProfileErrorV1::MissingTopLevelCollection)?
    {
        if let Some(attributes) = event
            .get_mut("attributes")
            .and_then(Value::as_array_mut)
        {
            attributes.sort_by(|left, right| {
                left.get("name")
                    .and_then(Value::as_str)
                    .cmp(&right.get("name").and_then(Value::as_str))
            });
        }
        if let Some(relations) = event
            .get_mut("relationships")
            .and_then(Value::as_array_mut)
        {
            relations.sort_by(|left, right| {
                left.get("objectId")
                    .and_then(Value::as_str)
                    .cmp(&right.get("objectId").and_then(Value::as_str))
                    .then_with(|| {
                        left.get("qualifier")
                            .and_then(Value::as_str)
                            .cmp(&right.get("qualifier").and_then(Value::as_str))
                    })
            });
        }
    }
    value
        .get_mut("events")
        .and_then(Value::as_array_mut)
        .ok_or(OcelProfileErrorV1::MissingTopLevelCollection)?
        .sort_by(|left, right| {
            left.get("id")
                .and_then(Value::as_str)
                .cmp(&right.get("id").and_then(Value::as_str))
        });

    serde_json::to_string(&value).map_err(|_| OcelProfileErrorV1::InvalidJson)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Value {
        serde_json::from_str(FIXTURE).expect("synthetic WSR OCEL fixture parses as JSON")
    }

    #[test]
    fn ocel20_water_fixture_has_expected_object_centric_shape() {
        let receipt = validate_ocel20_water_profile(FIXTURE)
            .expect("restricted OCEL 2.0 water profile is valid");
        assert_eq!(receipt.object_type_count, 13);
        assert_eq!(receipt.event_type_count, 9);
        assert_eq!(receipt.object_count, 17);
        assert_eq!(receipt.event_count, 10);
        assert!(receipt.event_to_object_relationship_count >= 20);
        assert!(receipt.object_to_object_relationship_count >= 5);
        assert_eq!(receipt.canonical_sha256.len(), 64);
    }

    #[test]
    fn canonicalization_is_invariant_to_unordered_collection_permutations() {
        let original = fixture();
        let original_canonical = canonical_json(&original).unwrap();

        let mut reordered = original;
        for key in ["objects", "events", "objectTypes", "eventTypes"] {
            reordered[key].as_array_mut().unwrap().reverse();
        }
        reordered["events"][0]["attributes"]
            .as_array_mut()
            .unwrap()
            .reverse();
        reordered["objects"][0]["relationships"]
            .as_array_mut()
            .unwrap()
            .reverse();

        let reordered_canonical = canonical_json(&reordered).unwrap();
        assert_eq!(original_canonical, reordered_canonical);
    }

    #[test]
    fn every_event_reference_must_resolve_to_an_existing_object() {
        let mut value = fixture();
        value["events"][0]["relationships"][0]["objectId"] =
            Value::String("missing-object".to_owned());
        let input = serde_json::to_string(&value).unwrap();
        assert_eq!(
            validate_ocel20_water_profile(&input),
            Err(OcelProfileErrorV1::UnknownObjectReference)
        );
    }

    #[test]
    fn duplicate_event_and_object_ids_fail_closed() {
        let mut duplicate_event = fixture();
        duplicate_event["events"][1]["id"] = duplicate_event["events"][0]["id"].clone();
        let input = serde_json::to_string(&duplicate_event).unwrap();
        assert_eq!(
            validate_ocel20_water_profile(&input),
            Err(OcelProfileErrorV1::DuplicateEventId)
        );

        let mut duplicate_object = fixture();
        duplicate_object["objects"][1]["id"] = duplicate_object["objects"][0]["id"].clone();
        let input = serde_json::to_string(&duplicate_object).unwrap();
        assert_eq!(
            validate_ocel20_water_profile(&input),
            Err(OcelProfileErrorV1::DuplicateObjectId)
        );
    }

    #[test]
    fn same_fault_complaints_share_one_callout_but_repeat_incident_is_new_episode() {
        let value = fixture();
        let complaint_relationships = ["complaint-1", "complaint-2"].map(|id| {
            value["objects"]
                .as_array()
                .unwrap()
                .iter()
                .find(|object| object["id"].as_str() == Some(id))
                .unwrap()["relationships"][0]["objectId"]
                .as_str()
                .unwrap()
                .to_owned()
        });
        assert_eq!(complaint_relationships, ["callout-1", "callout-1"]);

        let episodes: BTreeSet<_> = value["objects"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|object| object["type"] == "ServiceEpisode")
            .map(|object| object["id"].as_str().unwrap())
            .collect();
        assert_eq!(episodes, BTreeSet::from(["episode-1", "episode-2"]));
        assert_ne!(value["objects"][3]["id"], value["objects"][16]["id"]);
    }

    #[test]
    fn knowledge_time_and_host_sequence_survive_canonical_round_trip() {
        let mut value = fixture();
        value["metadata"] = serde_json::json!({
            "profile_extensions": {
                "valid_time_vs_recorded_time": true,
                "host_sequence_is_authoritative_order": true
            }
        });
        let encoded = canonical_json(&value).unwrap();
        let decoded: Value = serde_json::from_str(&encoded).unwrap();
        assert_eq!(
            decoded["metadata"]["profile_extensions"]["valid_time_vs_recorded_time"],
            true
        );
        assert_eq!(decoded["events"][0]["attributes"][0]["name"], "source_recorded_at");
        assert_eq!(
            decoded["events"][0]["attributes"].as_array().unwrap().len(),
            5
        );
    }
}
