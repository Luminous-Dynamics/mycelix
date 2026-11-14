#![forbid(unsafe_code)]

// This reference compares an observed frame with a declared reference frame.
// It does not authenticate sources or prove that the reference frame is exhaustive.

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "record_type|record_id|scope_status|reference_record_id|decision|evidence_owner|evidence_ref|valid_from_seconds|valid_until_seconds";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuditError(pub String);

impl std::fmt::Display for AuditError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for AuditError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ScopeStatus {
    InScope,
    OutOfScope,
    Unknown,
}
impl ScopeStatus {
    fn parse(value: &str) -> Result<Self, AuditError> {
        match value {
            "IN_SCOPE" => Ok(Self::InScope),
            "OUT_OF_SCOPE" => Ok(Self::OutOfScope),
            "UNKNOWN" => Ok(Self::Unknown),
            _ => Err(AuditError(format!("invalid scope status {value}"))),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Interval {
    from: u64,
    until: u64,
}
impl Interval {
    fn covers(self, start: u64, end: u64) -> bool {
        self.from <= start && self.until >= end && self.until > self.from
    }
}

#[derive(Clone, Debug)]
struct ReferenceRecord {
    interval: Interval,
}
#[derive(Clone, Debug)]
struct ObservedRecord {
    scope: ScopeStatus,
    interval: Interval,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Decision {
    Match,
    Unresolved,
    NoMatchInScope,
    OutOfScope,
}
impl Decision {
    fn parse(value: &str) -> Result<Self, AuditError> {
        match value {
            "MATCH" => Ok(Self::Match),
            "UNRESOLVED" => Ok(Self::Unresolved),
            "NO_MATCH_IN_SCOPE" => Ok(Self::NoMatchInScope),
            "OUT_OF_SCOPE" => Ok(Self::OutOfScope),
            _ => Err(AuditError(format!("invalid link decision {value}"))),
        }
    }
}

#[derive(Clone, Debug)]
struct Link {
    observed_id: String,
    scope: ScopeStatus,
    reference_id: Option<String>,
    decision: Decision,
    _evidence_owner: String,
    _evidence_ref: String,
    interval: Interval,
}

#[derive(Clone, Debug)]
struct Metadata {
    audit_id: String,
    _scope_id: String,
    _unit_type: String,
    study_start: u64,
    study_end: u64,
    _reference_frame_id: String,
    _reference_owner: String,
    _reference_ref: String,
    _reference_snapshot_id: String,
    _reference_interval: Interval,
    reference_coverage_basis: String,
    _observed_frame_id: String,
    _observed_owner: String,
    _observed_ref: String,
    _observed_snapshot_id: String,
    _observed_interval: Interval,
    _crosswalk_owner: String,
    _crosswalk_ref: String,
    _crosswalk_snapshot_id: String,
    _crosswalk_interval: Interval,
    crosswalk_independence_assessment: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AuditReport {
    pub audit_id: String,
    pub disposition: String,
    pub reference_units: usize,
    pub observed_units: usize,
    pub matched_reference_units: usize,
    pub omitted_reference_units: usize,
    pub duplicate_reference_units: usize,
    pub unresolved_reference_units: usize,
    pub unrepresented_in_scope_observed: usize,
    pub out_of_scope_observed: usize,
    pub unknown_scope_observed: usize,
    pub unresolved_observed: usize,
    pub match_edges: usize,
    pub unresolved_edges: usize,
    pub reference_coverage_basis: String,
    pub crosswalk_independence_assessment: String,
}

impl AuditReport {
    /// Stable, line-oriented output suitable for exact golden comparison.
    pub fn render(&self) -> String {
        format!(
            concat!(
                "audit_id={}\n",
                "disposition={}\n",
                "reference_units={}\n",
                "observed_units={}\n",
                "matched_reference_units={}\n",
                "omitted_reference_units={}\n",
                "duplicate_reference_units={}\n",
                "unresolved_reference_units={}\n",
                "unrepresented_in_scope_observed={}\n",
                "out_of_scope_observed={}\n",
                "unknown_scope_observed={}\n",
                "unresolved_observed={}\n",
                "match_edges={}\n",
                "unresolved_edges={}\n",
                "reference_coverage_basis={}\n",
                "crosswalk_independence_assessment={}\n",
                "reference_population_completeness=NOT_ESTABLISHED\n",
                "population_coverage_estimate=NOT_COMPUTED\n",
                "source_authentication=ASSERTED_NOT_AUTHENTICATED\n"
            ),
            self.audit_id,
            self.disposition,
            self.reference_units,
            self.observed_units,
            self.matched_reference_units,
            self.omitted_reference_units,
            self.duplicate_reference_units,
            self.unresolved_reference_units,
            self.unrepresented_in_scope_observed,
            self.out_of_scope_observed,
            self.unknown_scope_observed,
            self.unresolved_observed,
            self.match_edges,
            self.unresolved_edges,
            self.reference_coverage_basis,
            self.crosswalk_independence_assessment
        )
    }
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric()
                || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}
fn required<'a>(metadata: &'a BTreeMap<String, String>, key: &str) -> Result<&'a str, AuditError> {
    let value = metadata
        .get(key)
        .ok_or_else(|| AuditError(format!("missing metadata key {key}")))?;
    if value.is_empty() {
        return Err(AuditError(format!("empty metadata key {key}")));
    }
    Ok(value)
}
fn parse_u64(value: &str, key: &str) -> Result<u64, AuditError> {
    value
        .parse::<u64>()
        .map_err(|error| AuditError(format!("invalid {key}: {error}")))
}
fn parse_interval(from: &str, until: &str, label: &str) -> Result<Interval, AuditError> {
    let interval = Interval {
        from: parse_u64(from, &format!("{label}_valid_from_seconds"))?,
        until: parse_u64(until, &format!("{label}_valid_until_seconds"))?,
    };
    if interval.until <= interval.from {
        return Err(AuditError(format!("invalid {label} half-open interval")));
    }
    Ok(interval)
}
fn checked_id(value: &str, key: &str) -> Result<String, AuditError> {
    if !valid_id(value) || value == "-" {
        return Err(AuditError(format!("invalid identifier for {key}")));
    }
    Ok(value.to_owned())
}

fn parse_metadata(metadata: &BTreeMap<String, String>) -> Result<Metadata, AuditError> {
    const KEYS: &[&str] = &[
        "audit_id", "scope_id", "unit_type", "study_start_seconds", "study_end_seconds",
        "reference_frame_id", "reference_owner", "reference_ref", "reference_snapshot_id",
        "reference_valid_from_seconds", "reference_valid_until_seconds", "reference_coverage_basis",
        "observed_frame_id", "observed_owner", "observed_ref", "observed_snapshot_id",
        "observed_valid_from_seconds", "observed_valid_until_seconds", "crosswalk_owner",
        "crosswalk_ref", "crosswalk_snapshot_id", "crosswalk_valid_from_seconds",
        "crosswalk_valid_until_seconds", "crosswalk_independence_assessment",
    ];
    if let Some(key) = metadata.keys().find(|key| !KEYS.contains(&key.as_str())) {
        return Err(AuditError(format!("unknown metadata key {key}")));
    }
    let id = |key: &str| -> Result<String, AuditError> {
        checked_id(required(metadata, key)?, key)
    };
    let study_start = parse_u64(required(metadata, "study_start_seconds")?, "study_start_seconds")?;
    let study_end = parse_u64(required(metadata, "study_end_seconds")?, "study_end_seconds")?;
    if study_end <= study_start {
        return Err(AuditError("study horizon must be a non-empty half-open interval".to_owned()));
    }

    let reference_interval = parse_interval(
        required(metadata, "reference_valid_from_seconds")?,
        required(metadata, "reference_valid_until_seconds")?, "reference_frame",
    )?;
    let observed_interval = parse_interval(
        required(metadata, "observed_valid_from_seconds")?,
        required(metadata, "observed_valid_until_seconds")?, "observed_frame",
    )?;
    let crosswalk_interval = parse_interval(
        required(metadata, "crosswalk_valid_from_seconds")?,
        required(metadata, "crosswalk_valid_until_seconds")?, "crosswalk",
    )?;
    for (label, interval) in [
        ("reference frame", reference_interval),
        ("observed frame", observed_interval),
        ("crosswalk", crosswalk_interval),
    ] {
        if !interval.covers(study_start, study_end) {
            return Err(AuditError(format!(
                "{label} validity does not cover the complete study horizon"
            )));
        }
    }

    let reference_coverage_basis = required(metadata, "reference_coverage_basis")?.to_owned();
    if !matches!(
        reference_coverage_basis.as_str(),
        "DECLARED_ENUMERATED_REGISTER" | "SAMPLE" | "UNKNOWN"
    ) {
        return Err(AuditError("unsupported reference_coverage_basis".to_owned()));
    }
    let crosswalk_independence_assessment =
        required(metadata, "crosswalk_independence_assessment")?.to_owned();
    if !matches!(
        crosswalk_independence_assessment.as_str(),
        "REVIEWED_INDEPENDENT" | "KNOWN_DEPENDENT" | "UNKNOWN"
    ) {
        return Err(AuditError("invalid crosswalk_independence_assessment".to_owned()));
    }

    Ok(Metadata {
        audit_id: id("audit_id")?,
        _scope_id: id("scope_id")?,
        _unit_type: id("unit_type")?,
        study_start,
        study_end,
        _reference_frame_id: id("reference_frame_id")?,
        _reference_owner: id("reference_owner")?,
        _reference_ref: id("reference_ref")?,
        _reference_snapshot_id: id("reference_snapshot_id")?,
        _reference_interval: reference_interval,
        reference_coverage_basis,
        _observed_frame_id: id("observed_frame_id")?,
        _observed_owner: id("observed_owner")?,
        _observed_ref: id("observed_ref")?,
        _observed_snapshot_id: id("observed_snapshot_id")?,
        _observed_interval: observed_interval,
        _crosswalk_owner: id("crosswalk_owner")?,
        _crosswalk_ref: id("crosswalk_ref")?,
        _crosswalk_snapshot_id: id("crosswalk_snapshot_id")?,
        _crosswalk_interval: crosswalk_interval,
        crosswalk_independence_assessment,
    })
}
