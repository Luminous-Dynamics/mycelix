#![forbid(unsafe_code)]

//! Deterministic, loss-aware OCEL 2.0 JSON projection.
//!
//! The projection is a derived analytical representation. It is not a
//! journal, execution authority, legal ownership record, settlement proof,
//! or replacement for FIN-EFFECT owner records.
//!
//! All event and object-attribute timestamps require explicit source-time
//! evidence. Record time, ingestion time, owner sequence and exporter clocks
//! are rejected as substitutes. Event/object causal ordering is intentionally
//! kept outside the OCEL document in a separate owner-scoped sidecar.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

const MAX_IDENTIFIER_BYTES: usize = 1_024;
const MAX_TEXT_BYTES: usize = 16_384;
/// Keep JSON integer values exactly representable by common IEEE-754 consumers.
pub const MAX_INTEROPERABLE_INTEGER: i64 = 9_007_199_254_740_991;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ValueKind {
    String,
    Integer,
    Boolean,
}

impl ValueKind {
    fn as_ocel_type(self) -> &'static str {
        match self {
            Self::String => "string",
            Self::Integer => "integer",
            Self::Boolean => "boolean",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum OcelValue {
    String(String),
    Integer(i64),
    Boolean(bool),
}

impl OcelValue {
    fn kind(&self) -> ValueKind {
        match self {
            Self::String(_) => ValueKind::String,
            Self::Integer(_) => ValueKind::Integer,
            Self::Boolean(_) => ValueKind::Boolean,
        }
    }

    fn validate(&self) -> Result<(), ProjectionError> {
        match self {
            Self::String(value) if value.len() > MAX_TEXT_BYTES => Err(ProjectionError::TextTooLong),
            Self::Integer(value)
                if !(-MAX_INTEROPERABLE_INTEGER..=MAX_INTEROPERABLE_INTEGER).contains(value) =>
            {
                Err(ProjectionError::IntegerNotExactlyInteroperable)
            }
            _ => Ok(()),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AttributeDeclaration {
    pub name: String,
    pub kind: ValueKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EventType {
    pub name: String,
    pub attributes: Vec<AttributeDeclaration>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObjectType {
    pub name: String,
    pub attributes: Vec<AttributeDeclaration>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimeBasis {
    SourceEventTime,
    SourceAttributeTime,
    RecordTime,
    IngestionTime,
    OwnerSequence,
    Unknown,
}

impl TimeBasis {
    fn as_str(self) -> &'static str {
        match self {
            Self::SourceEventTime => "source_record_event_time",
            Self::SourceAttributeTime => "source_record_attribute_time",
            Self::RecordTime => "record_time",
            Self::IngestionTime => "ingestion_time",
            Self::OwnerSequence => "owner_sequence",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimePrecision {
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
    Minute,
    Day,
}

impl TimePrecision {
    fn as_str(self) -> &'static str {
        match self {
            Self::Nanosecond => "NANOSECOND",
            Self::Microsecond => "MICROSECOND",
            Self::Millisecond => "MILLISECOND",
            Self::Second => "SECOND",
            Self::Minute => "MINUTE",
            Self::Day => "DAY",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimeQuality {
    SourceAsserted,
    Estimated,
    Uncertain,
    Contested,
}

impl TimeQuality {
    fn as_str(self) -> &'static str {
        match self {
            Self::SourceAsserted => "SOURCE_ASSERTED",
            Self::Estimated => "ESTIMATED",
            Self::Uncertain => "UNCERTAIN",
            Self::Contested => "CONTESTED",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimeUncertaintyWindow {
    pub earliest: String,
    pub latest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimeAssertionMetadata {
    pub precision: TimePrecision,
    pub quality: TimeQuality,
    pub clock_profile: String,
    pub source_authority_ref: String,
    pub uncertainty_window: Option<TimeUncertaintyWindow>,
}

impl TimeAssertionMetadata {
    pub fn source_asserted(
        clock_profile: impl Into<String>,
        source_authority_ref: impl Into<String>,
        precision: TimePrecision,
    ) -> Self {
        Self {
            precision,
            quality: TimeQuality::SourceAsserted,
            clock_profile: clock_profile.into(),
            source_authority_ref: source_authority_ref.into(),
            uncertainty_window: None,
        }
    }

    pub fn uncertain(
        clock_profile: impl Into<String>,
        source_authority_ref: impl Into<String>,
        precision: TimePrecision,
        uncertainty_window: TimeUncertaintyWindow,
    ) -> Self {
        Self {
            precision,
            quality: TimeQuality::Uncertain,
            clock_profile: clock_profile.into(),
            source_authority_ref: source_authority_ref.into(),
            uncertainty_window: Some(uncertainty_window),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QualifiedTime {
    pub iso8601: String,
    pub basis: TimeBasis,
    pub evidence_ref: String,
    pub metadata: TimeAssertionMetadata,
}

impl QualifiedTime {
    pub fn source_event_time(
        iso8601: impl Into<String>,
        evidence_ref: impl Into<String>,
        metadata: TimeAssertionMetadata,
    ) -> Self {
        Self {
            iso8601: iso8601.into(),
            basis: TimeBasis::SourceEventTime,
            evidence_ref: evidence_ref.into(),
            metadata,
        }
    }

    pub fn source_attribute_time(
        iso8601: impl Into<String>,
        evidence_ref: impl Into<String>,
        metadata: TimeAssertionMetadata,
    ) -> Self {
        Self {
            iso8601: iso8601.into(),
            basis: TimeBasis::SourceAttributeTime,
            evidence_ref: evidence_ref.into(),
            metadata,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct QualifiedRelation {
    pub object_id: String,
    pub qualifier: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EventRecord {
    pub id: String,
    pub event_type: String,
    pub time: QualifiedTime,
    /// Domain-specific attributes; source/time metadata are added by the exporter.
    pub attributes: BTreeMap<String, OcelValue>,
    pub source_event_ref: String,
    pub relationships: Vec<QualifiedRelation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimedObjectAttribute {
    pub name: String,
    pub time: QualifiedTime,
    pub value: OcelValue,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ObjectRecord {
    pub id: String,
    pub object_type: String,
    pub attributes: Vec<TimedObjectAttribute>,
    pub relationships: Vec<QualifiedRelation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProjectionInput {
    pub mapping_profile_id: String,
    pub event_types: Vec<EventType>,
    pub object_types: Vec<ObjectType>,
    pub events: Vec<EventRecord>,
    pub objects: Vec<ObjectRecord>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProjectionArtifacts {
    /// Strict OCEL 2.0 JSON with only its four standard root collections.
    pub ocel_json: String,
    /// Separate sidecar for source-time provenance of events and object attributes.
    pub time_provenance_json: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProjectionError {
    EmptyIdentifier,
    IdentifierTooLong,
    TextTooLong,
    DuplicateEventType,
    DuplicateObjectType,
    DuplicateAttributeDeclaration,
    MissingMandatoryTimeMetadata,
    DuplicateEventId,
    DuplicateObjectId,
    UnknownEventType,
    UnknownObjectType,
    UnknownAttribute,
    AttributeTypeMismatch,
    MissingSourceEventReference,
    InvalidTimestamp,
    MissingTimeEvidence,
    MissingClockProfile,
    MissingTimeAuthority,
    ContestedTimeCannotExport,
    UncertaintyWindowRequired,
    InvalidUncertaintyTimestamp,
    InvalidUncertaintyWindow,
    TimestampPrecisionMismatch,
    InvalidEventTimeBasis,
    InvalidObjectAttributeTimeBasis,
    EventWithoutObjectRelationship,
    DanglingEventObjectReference,
    DanglingObjectObjectReference,
    EmptyRelationshipQualifier,
    DuplicateRelationship,
    IntegerNotExactlyInteroperable,
    NoEvents,
    NoObjects,
}

impl ProjectionInput {
    pub fn export(&self) -> Result<ProjectionArtifacts, ProjectionError> {
        self.validate()?;
        Ok(ProjectionArtifacts {
            ocel_json: self.encode_ocel_json(),
            time_provenance_json: self.encode_time_provenance_json(),
        })
    }

    pub fn validate(&self) -> Result<(), ProjectionError> {
        validate_identifier(&self.mapping_profile_id)?;

        if self.events.is_empty() {
            return Err(ProjectionError::NoEvents);
        }
        if self.objects.is_empty() {
            return Err(ProjectionError::NoObjects);
        }

        let event_types = index_event_types(&self.event_types)?;
        let object_types = index_object_types(&self.object_types)?;
        let object_ids: BTreeSet<&str> =
            self.objects.iter().map(|object| object.id.as_str()).collect();
        let event_ids: BTreeSet<&str> =
            self.events.iter().map(|event| event.id.as_str()).collect();

        if object_ids.len() != self.objects.len() {
            return Err(ProjectionError::DuplicateObjectId);
        }
        if event_ids.len() != self.events.len() {
            return Err(ProjectionError::DuplicateEventId);
        }

        for object in &self.objects {
            validate_identifier(&object.id)?;
            let declarations = object_types
                .get(object.object_type.as_str())
                .ok_or(ProjectionError::UnknownObjectType)?;
            let declared: BTreeMap<&str, ValueKind> = declarations
                .iter()
                .map(|attribute| (attribute.name.as_str(), attribute.kind))
                .collect();
            let mut seen = BTreeSet::new();
            for attribute in &object.attributes {
                validate_identifier(&attribute.name)?;
                attribute.value.validate()?;
                if !seen.insert((&attribute.name, &attribute.time.iso8601)) {
                    return Err(ProjectionError::DuplicateAttributeDeclaration);
                }
                match declared.get(attribute.name.as_str()) {
                    None => return Err(ProjectionError::UnknownAttribute),
                    Some(kind) if *kind != attribute.value.kind() => {
                        return Err(ProjectionError::AttributeTypeMismatch);
                    }
                    Some(_) => {}
                }
                validate_time(&attribute.time, TimeBasis::SourceAttributeTime)?;
            }
            validate_relations(&object.relationships, &object_ids, Some(&object.id))?;
        }

        for event in &self.events {
            validate_identifier(&event.id)?;
            validate_identifier(&event.event_type)?;
            if event.source_event_ref.trim().is_empty() {
                return Err(ProjectionError::MissingSourceEventReference);
            }
            validate_identifier(&event.source_event_ref)?;
            validate_time(&event.time, TimeBasis::SourceEventTime)?;
            let declarations = event_types
                .get(event.event_type.as_str())
                .ok_or(ProjectionError::UnknownEventType)?;
            let declared: BTreeMap<&str, ValueKind> = declarations
                .iter()
                .map(|attribute| (attribute.name.as_str(), attribute.kind))
                .collect();
            for mandatory in [
                "time_basis",
                "time_evidence_ref",
                "time_precision",
                "time_quality",
                "source_event_ref",
            ] {
                if declared.get(mandatory) != Some(&ValueKind::String) {
                    return Err(ProjectionError::MissingMandatoryTimeMetadata);
                }
            }
            for (name, value) in &event.attributes {
                validate_identifier(name)?;
                value.validate()?;
                if matches!(
                    name.as_str(),
                    "time_basis" | "time_evidence_ref" | "time_precision" | "time_quality" | "source_event_ref"
                ) {
                    return Err(ProjectionError::UnknownAttribute);
                }
                match declared.get(name.as_str()) {
                    None => return Err(ProjectionError::UnknownAttribute),
                    Some(kind) if *kind != value.kind() => {
                        return Err(ProjectionError::AttributeTypeMismatch);
                    }
                    Some(_) => {}
                }
            }
            if event.relationships.is_empty() {
                return Err(ProjectionError::EventWithoutObjectRelationship);
            }
            validate_relations(&event.relationships, &object_ids, None)?;
        }

        Ok(())
    }

    fn encode_ocel_json(&self) -> String {
        let mut output = String::new();
        output.push('{');

        let mut event_types = self.event_types.clone();
        event_types.sort_by(|a, b| a.name.cmp(&b.name));
        output.push_str("\"eventTypes\":[");
        for (index, event_type) in event_types.iter_mut().enumerate() {
            if index > 0 {
                output.push(',');
            }
            event_type.attributes.sort_by(|a, b| a.name.cmp(&b.name));
            output.push('{');
            output.push_str("\"name\":");
            push_json_string(&mut output, &event_type.name);
            output.push_str(",\"attributes\":");
            encode_declarations(&mut output, &event_type.attributes);
            output.push('}');
        }

        let mut events = self.events.clone();
        events.sort_by(|a, b| a.id.cmp(&b.id));
        output.push_str("],\"events\":[");
        for (index, event) in events.iter().enumerate() {
            if index > 0 {
                output.push(',');
            }
            encode_event(&mut output, event);
        }

        let mut object_types = self.object_types.clone();
        object_types.sort_by(|a, b| a.name.cmp(&b.name));
        output.push_str("],\"objectTypes\":[");
        for (index, object_type) in object_types.iter_mut().enumerate() {
            if index > 0 {
                output.push(',');
            }
            object_type.attributes.sort_by(|a, b| a.name.cmp(&b.name));
            output.push('{');
            output.push_str("\"name\":");
            push_json_string(&mut output, &object_type.name);
            output.push_str(",\"attributes\":");
            encode_declarations(&mut output, &object_type.attributes);
            output.push('}');
        }

        let mut objects = self.objects.clone();
        objects.sort_by(|a, b| a.id.cmp(&b.id));
        output.push_str("],\"objects\":[");
        for (index, object) in objects.iter_mut().enumerate() {
            if index > 0 {
                output.push(',');
            }
            encode_object(&mut output, object);
        }
        output.push_str("]}");
        output
    }

    fn encode_time_provenance_json(&self) -> String {
        let mut output = String::new();
        output.push('{');
        output.push_str("\"profile\":\"MYCELIX-OCEL-TIME-PROVENANCE-V1\",");
        output.push_str("\"mapping_profile_id\":");
        push_json_string(&mut output, &self.mapping_profile_id);
        output.push_str(",\"event_time_evidence\":[");

        let mut events = self.events.iter().collect::<Vec<_>>();
        events.sort_by(|a, b| a.id.cmp(&b.id));
        for (index, event) in events.iter().enumerate() {
            if index > 0 {
                output.push(',');
            }
            output.push('{');
            output.push_str("\"ocel_event_id\":");
            push_json_string(&mut output, &event.id);
            output.push_str(",\"source_event_ref\":");
            push_json_string(&mut output, &event.source_event_ref);
            output.push_str(",\"event_time\":");
            push_json_string(&mut output, &event.time.iso8601);
            output.push_str(",\"time_basis\":");
            push_json_string(&mut output, event.time.basis.as_str());
            output.push_str(",\"time_evidence_ref\":");
            push_json_string(&mut output, &event.time.evidence_ref);
            encode_temporal_provenance_fields(&mut output, &event.time);
            output.push('}');
        }

        output.push_str("],\"object_attribute_time_evidence\":[");
        let mut objects = self.objects.iter().collect::<Vec<_>>();
        objects.sort_by(|a, b| a.id.cmp(&b.id));
        let mut first = true;
        for object in objects {
            let mut attributes = object.attributes.iter().collect::<Vec<_>>();
            attributes.sort_by(|a, b| {
                (&a.name, &a.time.iso8601).cmp(&(&b.name, &b.time.iso8601))
            });
            for attribute in attributes {
                if !first {
                    output.push(',');
                }
                first = false;
                output.push('{');
                output.push_str("\"object_id\":");
                push_json_string(&mut output, &object.id);
                output.push_str(",\"attribute_name\":");
                push_json_string(&mut output, &attribute.name);
                output.push_str(",\"attribute_time\":");
                push_json_string(&mut output, &attribute.time.iso8601);
                output.push_str(",\"time_basis\":");
                push_json_string(&mut output, attribute.time.basis.as_str());
                output.push_str(",\"time_evidence_ref\":");
                push_json_string(&mut output, &attribute.time.evidence_ref);
                encode_temporal_provenance_fields(&mut output, &attribute.time);
                output.push('}');
            }
        }
        output.push_str("]}");
        output
    }
}

fn index_event_types<'a>(
    definitions: &'a [EventType],
) -> Result<BTreeMap<&'a str, &'a Vec<AttributeDeclaration>>, ProjectionError> {
    let mut result = BTreeMap::new();
    for definition in definitions {
        validate_identifier(&definition.name)?;
        validate_declarations(&definition.attributes)?;
        if result
            .insert(definition.name.as_str(), &definition.attributes)
            .is_some()
        {
            return Err(ProjectionError::DuplicateEventType);
        }
    }
    Ok(result)
}

fn index_object_types<'a>(
    definitions: &'a [ObjectType],
) -> Result<BTreeMap<&'a str, &'a Vec<AttributeDeclaration>>, ProjectionError> {
    let mut result = BTreeMap::new();
    for definition in definitions {
        validate_identifier(&definition.name)?;
        validate_declarations(&definition.attributes)?;
        if result
            .insert(definition.name.as_str(), &definition.attributes)
            .is_some()
        {
            return Err(ProjectionError::DuplicateObjectType);
        }
    }
    Ok(result)
}

fn validate_declarations(declarations: &[AttributeDeclaration]) -> Result<(), ProjectionError> {
    let mut names = BTreeSet::new();
    for declaration in declarations {
        validate_identifier(&declaration.name)?;
        if !names.insert(declaration.name.as_str()) {
            return Err(ProjectionError::DuplicateAttributeDeclaration);
        }
    }
    Ok(())
}

fn validate_relations(
    relations: &[QualifiedRelation],
    object_ids: &BTreeSet<&str>,
    source_object_id: Option<&str>,
) -> Result<(), ProjectionError> {
    let mut seen = BTreeSet::new();
    for relation in relations {
        validate_identifier(&relation.object_id)?;
        if relation.qualifier.trim().is_empty() {
            return Err(ProjectionError::EmptyRelationshipQualifier);
        }
        validate_identifier(&relation.qualifier)?;
        if !object_ids.contains(relation.object_id.as_str()) {
            return Err(if source_object_id.is_some() {
                ProjectionError::DanglingObjectObjectReference
            } else {
                ProjectionError::DanglingEventObjectReference
            });
        }
        if source_object_id == Some(relation.object_id.as_str()) {
            return Err(ProjectionError::DanglingObjectObjectReference);
        }
        if !seen.insert((relation.object_id.as_str(), relation.qualifier.as_str())) {
            return Err(ProjectionError::DuplicateRelationship);
        }
    }
    Ok(())
}

fn validate_identifier(value: &str) -> Result<(), ProjectionError> {
    if value.trim().is_empty() {
        return Err(ProjectionError::EmptyIdentifier);
    }
    if value.len() > MAX_IDENTIFIER_BYTES {
        return Err(ProjectionError::IdentifierTooLong);
    }
    Ok(())
}

fn validate_time(time: &QualifiedTime, expected_basis: TimeBasis) -> Result<(), ProjectionError> {
    if time.basis != expected_basis {
        return Err(match expected_basis {
            TimeBasis::SourceEventTime => ProjectionError::InvalidEventTimeBasis,
            TimeBasis::SourceAttributeTime => ProjectionError::InvalidObjectAttributeTimeBasis,
            _ => ProjectionError::InvalidTimestamp,
        });
    }
    validate_identifier(&time.evidence_ref).map_err(|_| ProjectionError::MissingTimeEvidence)?;
    validate_identifier(&time.metadata.clock_profile)
        .map_err(|_| ProjectionError::MissingClockProfile)?;
    validate_identifier(&time.metadata.source_authority_ref)
        .map_err(|_| ProjectionError::MissingTimeAuthority)?;
    if !is_rfc3339_timestamp(&time.iso8601) {
        return Err(ProjectionError::InvalidTimestamp);
    }
    if !timestamp_matches_precision(&time.iso8601, time.metadata.precision) {
        return Err(ProjectionError::TimestampPrecisionMismatch);
    }
    if time.metadata.quality == TimeQuality::Contested {
        return Err(ProjectionError::ContestedTimeCannotExport);
    }
    match (&time.metadata.quality, &time.metadata.uncertainty_window) {
        (TimeQuality::Uncertain, None) => {
            return Err(ProjectionError::UncertaintyWindowRequired);
        }
        (_, Some(window)) => {
            if !is_rfc3339_timestamp(&window.earliest)
                || !is_rfc3339_timestamp(&window.latest)
            {
                return Err(ProjectionError::InvalidUncertaintyTimestamp);
            }
            if timezone_suffix(&window.earliest) != timezone_suffix(&window.latest)
                || window.earliest > window.latest
            {
                return Err(ProjectionError::InvalidUncertaintyWindow);
            }
        }
        _ => {}
    }
    Ok(())
}

fn is_rfc3339_timestamp(value: &str) -> bool {
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
    let Some(year) = parse_digits(bytes, 0, 4) else {
        return false;
    };
    let Some(month) = parse_digits(bytes, 5, 2) else {
        return false;
    };
    let Some(day) = parse_digits(bytes, 8, 2) else {
        return false;
    };
    let Some(hour) = parse_digits(bytes, 11, 2) else {
        return false;
    };
    let Some(minute) = parse_digits(bytes, 14, 2) else {
        return false;
    };
    let Some(second) = parse_digits(bytes, 17, 2) else {
        return false;
    };

    if year == 0
        || !(1..=12).contains(&month)
        || !(0..=23).contains(&hour)
        || !(0..=59).contains(&minute)
        || !(0..=59).contains(&second)
    {
        return false;
    }
    let leap_year = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let max_day = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap_year => 29,
        2 => 28,
        _ => return false,
    };
    if day < 1 || day > max_day {
        return false;
    }

    let mut index = 19;
    if bytes.get(index) == Some(&b'.') {
        index += 1;
        let start = index;
        while bytes.get(index).is_some_and(u8::is_ascii_digit) {
            index += 1;
        }
        if index == start {
            return false;
        }
    }
    match bytes.get(index) {
        Some(&b'Z') => index + 1 == bytes.len(),
        Some(&b'+') | Some(&b'-') => {
            let Some(offset_hour) = parse_digits(bytes, index + 1, 2) else {
                return false;
            };
            if bytes.get(index + 3) != Some(&b':') {
                return false;
            }
            let Some(offset_minute) = parse_digits(bytes, index + 4, 2) else {
                return false;
            };
            index + 6 == bytes.len() && offset_hour <= 23 && offset_minute <= 59
        }
        _ => false,
    }
}

fn timestamp_fraction_digits(value: &str) -> Option<usize> {
    let bytes = value.as_bytes();
    let mut index = 19;
    if bytes.get(index) != Some(&b'.') {
        return Some(0);
    }
    index += 1;
    let start = index;
    while bytes.get(index).is_some_and(u8::is_ascii_digit) {
        index += 1;
    }
    (index > start).then_some(index - start)
}

fn timezone_suffix(value: &str) -> &str {
    let bytes = value.as_bytes();
    let start = bytes.iter().enumerate().skip(19).find_map(|(index, byte)| {
        matches!(*byte, b'Z' | b'+' | b'-').then_some(index)
    }).unwrap_or(value.len());
    &value[start..]
}

fn timestamp_matches_precision(value: &str, precision: TimePrecision) -> bool {
    let bytes = value.as_bytes();
    let Some(hour) = parse_digits(bytes, 11, 2) else { return false; };
    let Some(minute) = parse_digits(bytes, 14, 2) else { return false; };
    let Some(second) = parse_digits(bytes, 17, 2) else { return false; };
    let Some(fraction_digits) = timestamp_fraction_digits(value) else { return false; };

    match precision {
        TimePrecision::Day => hour == 0 && minute == 0 && second == 0 && fraction_digits == 0,
        TimePrecision::Minute => second == 0 && fraction_digits == 0,
        TimePrecision::Second => fraction_digits == 0,
        TimePrecision::Millisecond => fraction_digits == 3,
        TimePrecision::Microsecond => fraction_digits == 6,
        TimePrecision::Nanosecond => fraction_digits == 9,
    }
}

fn parse_digits(bytes: &[u8], start: usize, count: usize) -> Option<u32> {
    let slice = bytes.get(start..start + count)?;
    if !slice.iter().all(u8::is_ascii_digit) {
        return None;
    }
    slice.iter().try_fold(0_u32, |value, byte| {
        value.checked_mul(10)?.checked_add((*byte - b'0') as u32)
    })
}

fn encode_temporal_provenance_fields(output: &mut String, time: &QualifiedTime) {
    output.push_str(",\"precision\":");
    push_json_string(output, time.metadata.precision.as_str());
    output.push_str(",\"quality\":");
    push_json_string(output, time.metadata.quality.as_str());
    output.push_str(",\"clock_profile\":");
    push_json_string(output, &time.metadata.clock_profile);
    output.push_str(",\"source_authority_ref\":");
    push_json_string(output, &time.metadata.source_authority_ref);
    if let Some(window) = &time.metadata.uncertainty_window {
        output.push_str(",\"uncertainty_window\":{");
        output.push_str("\"earliest\":");
        push_json_string(output, &window.earliest);
        output.push_str(",\"latest\":");
        push_json_string(output, &window.latest);
        output.push('}');
    }
}

fn encode_declarations(output: &mut String, declarations: &[AttributeDeclaration]) {
    output.push('[');
    for (index, declaration) in declarations.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push('{');
        output.push_str("\"name\":");
        push_json_string(output, &declaration.name);
        output.push_str(",\"type\":");
        push_json_string(output, declaration.kind.as_ocel_type());
        output.push('}');
    }
    output.push(']');
}

fn encode_event(output: &mut String, event: &EventRecord) {
    output.push('{');
    output.push_str("\"id\":");
    push_json_string(output, &event.id);
    output.push_str(",\"type\":");
    push_json_string(output, &event.event_type);
    output.push_str(",\"time\":");
    push_json_string(output, &event.time.iso8601);
    output.push_str(",\"attributes\":");

    let mut values: Vec<(String, OcelValue)> = event
        .attributes
        .iter()
        .map(|(name, value)| (name.clone(), value.clone()))
        .collect();
    values.push((
        "time_basis".into(),
        OcelValue::String(event.time.basis.as_str().into()),
    ));
    values.push((
        "time_evidence_ref".into(),
        OcelValue::String(event.time.evidence_ref.clone()),
    ));
    values.push((
        "source_event_ref".into(),
        OcelValue::String(event.source_event_ref.clone()),
    ));
    values.push((
        "time_precision".into(),
        OcelValue::String(event.time.metadata.precision.as_str().into()),
    ));
    values.push((
        "time_quality".into(),
        OcelValue::String(event.time.metadata.quality.as_str().into()),
    ));
    values.sort_by(|a, b| a.0.cmp(&b.0));

    output.push('[');
    for (index, (name, value)) in values.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push('{');
        output.push_str("\"name\":");
        push_json_string(output, name);
        output.push_str(",\"value\":");
        encode_value(output, value);
        output.push('}');
    }
    output.push(']');
    output.push_str(",\"relationships\":");
    encode_relations(output, &event.relationships);
    output.push('}');
}

fn encode_object(output: &mut String, object: &ObjectRecord) {
    output.push('{');
    output.push_str("\"id\":");
    push_json_string(output, &object.id);
    output.push_str(",\"type\":");
    push_json_string(output, &object.object_type);
    output.push_str(",\"attributes\":");
    let mut attributes = object.attributes.iter().collect::<Vec<_>>();
    attributes.sort_by(|a, b| (&a.name, &a.time.iso8601).cmp(&(&b.name, &b.time.iso8601)));
    output.push('[');
    for (index, attribute) in attributes.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push('{');
        output.push_str("\"name\":");
        push_json_string(output, &attribute.name);
        output.push_str(",\"time\":");
        push_json_string(output, &attribute.time.iso8601);
        output.push_str(",\"value\":");
        encode_value(output, &attribute.value);
        output.push('}');
    }
    output.push(']');
    output.push_str(",\"relationships\":");
    encode_relations(output, &object.relationships);
    output.push('}');
}

fn encode_relations(output: &mut String, relations: &[QualifiedRelation]) {
    let mut relations = relations.to_vec();
    relations.sort();
    output.push('[');
    for (index, relation) in relations.iter().enumerate() {
        if index > 0 {
            output.push(',');
        }
        output.push('{');
        output.push_str("\"objectId\":");
        push_json_string(output, &relation.object_id);
        output.push_str(",\"qualifier\":");
        push_json_string(output, &relation.qualifier);
        output.push('}');
    }
    output.push(']');
}

fn encode_value(output: &mut String, value: &OcelValue) {
    match value {
        OcelValue::String(value) => push_json_string(output, value),
        OcelValue::Integer(value) => {
            let _ = write!(output, "{value}");
        }
        OcelValue::Boolean(value) => output.push_str(if *value { "true" } else { "false" }),
    }
}

fn push_json_string(output: &mut String, value: &str) {
    output.push('"');
    for character in value.chars() {
        match character {
            '"' => output.push_str("\\\""),
            '\\' => output.push_str("\\\\"),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            '\u{0008}' => output.push_str("\\b"),
            '\u{000C}' => output.push_str("\\f"),
            value if value <= '\u{001F}' => {
                let _ = write!(output, "\\u{:04x}", value as u32);
            }
            value => output.push(value),
        }
    }
    output.push('"');
}

#[cfg(test)]
mod tests {
    use super::*;

    fn declaration(name: &str, kind: ValueKind) -> AttributeDeclaration {
        AttributeDeclaration { name: name.into(), kind }
    }

    fn sample() -> ProjectionInput {
        let event_type = EventType {
            name: "goods_received".into(),
            attributes: vec![
                declaration("time_basis", ValueKind::String),
                declaration("time_evidence_ref", ValueKind::String),
                declaration("time_precision", ValueKind::String),
                declaration("time_quality", ValueKind::String),
                declaration("source_event_ref", ValueKind::String),
                declaration("quantity", ValueKind::Integer),
                declaration("unit_profile", ValueKind::String),
            ],
        };
        let object_type = ObjectType {
            name: "DeliveryLot".into(),
            attributes: vec![
                declaration("quantity", ValueKind::Integer),
                declaration("unit_profile", ValueKind::String),
            ],
        };
        let mut attributes = BTreeMap::new();
        attributes.insert("quantity".into(), OcelValue::Integer(250));
        attributes.insert("unit_profile".into(), OcelValue::String("filter_cartridge".into()));
        ProjectionInput {
            mapping_profile_id: "economic-os-ocel-crosswalk-v0.1".into(),
            event_types: vec![event_type],
            object_types: vec![object_type],
            events: vec![EventRecord {
                id: "e-receipt".into(),
                event_type: "goods_received".into(),
                time: QualifiedTime::source_event_time(
                    "2026-11-21T13:00:00Z",
                    "source:receipt:time",
                    TimeAssertionMetadata::source_asserted(
                        "receiver-device-utc-v1",
                        "warehouse:receiver-1:goods-receipt-authority",
                        TimePrecision::Second,
                    ),
                ),
                attributes,
                source_event_ref: "source:goods-receipt:1".into(),
                relationships: vec![QualifiedRelation {
                    object_id: "LOT-001".into(),
                    qualifier: "received_lot".into(),
                }],
            }],
            objects: vec![ObjectRecord {
                id: "LOT-001".into(),
                object_type: "DeliveryLot".into(),
                attributes: vec![
                    TimedObjectAttribute {
                        name: "quantity".into(),
                        time: QualifiedTime::source_attribute_time(
                            "2026-11-21T10:00:00Z",
                            "source:lot:quantity-time",
                            TimeAssertionMetadata::source_asserted(
                                "supplier-device-utc-v1",
                                "supplier:sup-001:delivery-lot-authority",
                                TimePrecision::Second,
                            ),
                        ),
                        value: OcelValue::Integer(250),
                    },
                    TimedObjectAttribute {
                        name: "unit_profile".into(),
                        time: QualifiedTime::source_attribute_time(
                            "2026-11-21T10:00:00Z",
                            "source:lot:unit-time",
                            TimeAssertionMetadata::source_asserted(
                                "supplier-device-utc-v1",
                                "supplier:sup-001:delivery-lot-authority",
                                TimePrecision::Second,
                            ),
                        ),
                        value: OcelValue::String("filter_cartridge".into()),
                    },
                ],
                relationships: vec![],
            }],
        }
    }

    #[test]
    fn export_has_only_the_four_standard_ocel_root_collections() {
        let artifacts = sample().export().unwrap();
        assert!(artifacts.ocel_json.starts_with("{\"eventTypes\":["));
        assert!(artifacts.ocel_json.contains("\"events\":["));
        assert!(artifacts.ocel_json.contains("\"objectTypes\":["));
        assert!(artifacts.ocel_json.contains("\"objects\":["));
        assert!(!artifacts.ocel_json.contains("time_provenance"));
        assert!(artifacts.ocel_json.ends_with("]}"));
    }

    #[test]
    fn event_and_object_attribute_time_provenance_are_separate_and_complete() {
        let artifacts = sample().export().unwrap();
        assert!(artifacts.ocel_json.contains("\"name\":\"time_basis\",\"value\":\"source_record_event_time\""));
        assert!(artifacts.time_provenance_json.contains("source:receipt:time"));
        assert!(artifacts.time_provenance_json.contains("source:lot:quantity-time"));
        assert!(artifacts.time_provenance_json.contains("source:lot:unit-time"));
    }

    #[test]
    fn precision_quality_clock_and_authority_are_preserved() {
        let artifacts = sample().export().unwrap();
        assert!(artifacts.ocel_json.contains("\"name\":\"time_precision\",\"value\":\"SECOND\""));
        assert!(artifacts.ocel_json.contains("\"name\":\"time_quality\",\"value\":\"SOURCE_ASSERTED\""));
        for expected in [
            "\"precision\":\"SECOND\"",
            "\"quality\":\"SOURCE_ASSERTED\"",
            "\"clock_profile\":\"receiver-device-utc-v1\"",
            "\"source_authority_ref\":\"warehouse:receiver-1:goods-receipt-authority\"",
            "\"clock_profile\":\"supplier-device-utc-v1\"",
        ] {
            assert!(artifacts.time_provenance_json.contains(expected), "missing {expected}");
        }
    }

    #[test]
    fn uncertain_time_requires_explicit_uncertainty_window() {
        let mut input = sample();
        input.events[0].time.metadata.quality = TimeQuality::Uncertain;
        assert_eq!(input.export(), Err(ProjectionError::UncertaintyWindowRequired));
        input.events[0].time.metadata.uncertainty_window = Some(TimeUncertaintyWindow {
            earliest: "2026-11-21T12:59:58Z".into(),
            latest: "2026-11-21T13:00:04Z".into(),
        });
        let artifacts = input.export().unwrap();
        assert!(artifacts.time_provenance_json.contains("\"uncertainty_window\""));
        assert!(artifacts.time_provenance_json.contains("2026-11-21T12:59:58Z"));
    }

    #[test]
    fn contested_temporal_assertion_is_not_exported() {
        let mut input = sample();
        input.events[0].time.metadata.quality = TimeQuality::Contested;
        assert_eq!(input.export(), Err(ProjectionError::ContestedTimeCannotExport));
    }

    #[test]
    fn timestamp_precision_must_match_serialized_timestamp() {
        let mut input = sample();
        input.events[0].time.metadata.precision = TimePrecision::Millisecond;
        assert_eq!(input.export(), Err(ProjectionError::TimestampPrecisionMismatch));

        let mut input = sample();
        input.events[0].time.iso8601 = "2026-11-21T13:00:00.125Z".into();
        input.events[0].time.metadata.precision = TimePrecision::Millisecond;
        assert!(input.export().is_ok());

        let mut input = sample();
        input.events[0].time.metadata.precision = TimePrecision::Day;
        assert_eq!(input.export(), Err(ProjectionError::TimestampPrecisionMismatch));
    }

    #[test]
    fn uncertainty_window_must_be_ordered_and_share_timezone() {
        let mut input = sample();
        input.events[0].time.metadata = TimeAssertionMetadata::uncertain(
            "provider-rail-clock-v1",
            "provider:rail-1:status-authority",
            TimePrecision::Second,
            TimeUncertaintyWindow {
                earliest: "2026-11-21T13:00:02Z".into(),
                latest: "2026-11-21T13:00:01Z".into(),
            },
        );
        assert_eq!(input.export(), Err(ProjectionError::InvalidUncertaintyWindow));

        let mut input = sample();
        input.events[0].time.metadata = TimeAssertionMetadata::uncertain(
            "provider-rail-clock-v1",
            "provider:rail-1:status-authority",
            TimePrecision::Second,
            TimeUncertaintyWindow {
                earliest: "2026-11-21T13:00:01Z".into(),
                latest: "2026-11-21T15:00:02+02:00".into(),
            },
        );
        assert_eq!(input.export(), Err(ProjectionError::InvalidUncertaintyWindow));
    }

    #[test]
    fn missing_clock_profile_and_source_authority_are_rejected() {
        let mut input = sample();
        input.events[0].time.metadata.clock_profile.clear();
        assert_eq!(input.export(), Err(ProjectionError::MissingClockProfile));

        let mut input = sample();
        input.events[0].time.metadata.source_authority_ref.clear();
        assert_eq!(input.export(), Err(ProjectionError::MissingTimeAuthority));
    }

    #[test]
    fn input_permutation_does_not_change_export_bytes() {
        let first = sample().export().unwrap();
        let mut permuted = sample();
        permuted.event_types.reverse();
        permuted.object_types.reverse();
        permuted.events.reverse();
        permuted.objects.reverse();
        for object in &mut permuted.objects {
            object.attributes.reverse();
            object.relationships.reverse();
        }
        let second = permuted.export().unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn ingestion_time_is_not_accepted_as_event_time() {
        let mut input = sample();
        input.events[0].time.basis = TimeBasis::IngestionTime;
        assert_eq!(input.export(), Err(ProjectionError::InvalidEventTimeBasis));
    }

    #[test]
    fn owner_sequence_is_not_accepted_as_event_time() {
        let mut input = sample();
        input.events[0].time.basis = TimeBasis::OwnerSequence;
        assert_eq!(input.export(), Err(ProjectionError::InvalidEventTimeBasis));
    }

    #[test]
    fn missing_time_evidence_is_held() {
        let mut input = sample();
        input.events[0].time.evidence_ref.clear();
        assert_eq!(input.export(), Err(ProjectionError::MissingTimeEvidence));
    }

    #[test]
    fn invalid_calendar_dates_are_rejected() {
        let mut input = sample();
        input.events[0].time.iso8601 = "2026-02-30T13:00:00Z".into();
        assert_eq!(input.export(), Err(ProjectionError::InvalidTimestamp));
    }

    #[test]
    fn dangling_event_object_relation_is_rejected() {
        let mut input = sample();
        input.events[0].relationships[0].object_id = "missing".into();
        assert_eq!(input.export(), Err(ProjectionError::DanglingEventObjectReference));
    }

    #[test]
    fn dangling_object_object_relation_is_rejected() {
        let mut input = sample();
        input.objects[0].relationships.push(QualifiedRelation {
            object_id: "missing".into(),
            qualifier: "related".into(),
        });
        assert_eq!(input.export(), Err(ProjectionError::DanglingObjectObjectReference));
    }

    #[test]
    fn unknown_attributes_are_rejected() {
        let mut input = sample();
        input.events[0].attributes.insert("settled".into(), OcelValue::Boolean(true));
        assert_eq!(input.export(), Err(ProjectionError::UnknownAttribute));
    }

    #[test]
    fn type_mismatched_attributes_are_rejected() {
        let mut input = sample();
        input.events[0].attributes.insert("quantity".into(), OcelValue::String("250".into()));
        assert_eq!(input.export(), Err(ProjectionError::AttributeTypeMismatch));
    }

    #[test]
    fn integers_beyond_exact_interoperability_range_are_rejected() {
        let mut input = sample();
        input.events[0].attributes.insert("quantity".into(), OcelValue::Integer(MAX_INTEROPERABLE_INTEGER + 1));
        assert_eq!(input.export(), Err(ProjectionError::IntegerNotExactlyInteroperable));
    }

    #[test]
    fn event_without_object_relationship_is_rejected() {
        let mut input = sample();
        input.events[0].relationships.clear();
        assert_eq!(input.export(), Err(ProjectionError::EventWithoutObjectRelationship));
    }

    #[test]
    fn duplicate_event_and_object_ids_are_rejected() {
        let mut duplicate_event = sample();
        let repeated_event = duplicate_event.events[0].clone();
        duplicate_event.events.push(repeated_event);
        assert_eq!(duplicate_event.export(), Err(ProjectionError::DuplicateEventId));

        let mut duplicate_object = sample();
        let repeated_object = duplicate_object.objects[0].clone();
        duplicate_object.objects.push(repeated_object);
        assert_eq!(duplicate_object.export(), Err(ProjectionError::DuplicateObjectId));
    }

    #[test]
    fn source_attribute_time_is_required_for_object_attribute_updates() {
        let mut input = sample();
        input.objects[0].attributes[0].time.basis = TimeBasis::RecordTime;
        assert_eq!(input.export(), Err(ProjectionError::InvalidObjectAttributeTimeBasis));
    }

    #[test]
    fn json_string_escaping_preserves_quotes_controls_and_unicode() {
        let mut input = sample();
        input.event_types[0].attributes.push(declaration("label", ValueKind::String));
        input.events[0].attributes.insert(
            "label".into(),
            OcelValue::String("lot \"A\"\nfilter\tcartridge — café".into()),
        );
        let artifacts = input.export().unwrap();
        assert!(artifacts.ocel_json.contains("lot \\\"A\\\"\\nfilter\\tcartridge — café"));
    }

    #[test]
    fn duplicate_event_object_relationship_is_rejected() {
        let mut input = sample();
        let duplicate = input.events[0].relationships[0].clone();
        input.events[0].relationships.push(duplicate);
        assert_eq!(input.export(), Err(ProjectionError::DuplicateRelationship));
    }

    #[test]
    fn non_utc_rfc3339_source_event_time_is_preserved_not_normalized() {
        let mut input = sample();
        input.events[0].time.iso8601 = "2026-11-21T15:00:00+02:00".into();
        let artifacts = input.export().unwrap();
        assert!(artifacts.ocel_json.contains("2026-11-21T15:00:00+02:00"));
        assert!(artifacts.time_provenance_json.contains("2026-11-21T15:00:00+02:00"));
    }

    #[test]
    fn timestamp_without_timezone_is_rejected() {
        let mut input = sample();
        input.events[0].time.iso8601 = "2026-11-21T13:00:00".into();
        assert_eq!(input.export(), Err(ProjectionError::InvalidTimestamp));
    }

    #[test]
    fn object_attribute_time_evidence_is_required() {
        let mut input = sample();
        input.objects[0].attributes[0].time.evidence_ref.clear();
        assert_eq!(input.export(), Err(ProjectionError::MissingTimeEvidence));
    }

    #[test]
    fn duplicate_object_attribute_at_same_time_is_rejected() {
        let mut input = sample();
        let repeated_attribute = input.objects[0].attributes[0].clone();
        input.objects[0].attributes.push(repeated_attribute);
        assert_eq!(input.export(), Err(ProjectionError::DuplicateAttributeDeclaration));
    }
}
