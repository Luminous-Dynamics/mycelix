#![forbid(unsafe_code)]

//! Pure bitemporal evidence selection for the Economic OS.
//!
//! The kernel answers: "What was valid at V, according to the assertions
//! verifier X had accepted by knowledge frontier K?".
//!
//! It never substitutes source record time, receiver observation time, or
//! owner-local sequence for occurrence/valid time. Corrections are append-only
//! assertions linked to their predecessor. A correction not yet known to the
//! queried verifier does not rewrite the verifier's earlier as-known result.
//!
//! V1 compares canonical UTC RFC3339 strings (Z suffix only) to avoid silently
//! normalizing offsets. Offset-to-UTC conversion must be performed by a
//! separately qualified, evidence-preserving clock profile.

use std::collections::{BTreeMap, BTreeSet};

const MAX_ID_BYTES: usize = 1_024;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum TemporalDimensionV1 {
    OccurredAt,
    ValidFrom,
    ValidTo,
    SourceRecordedAt,
    ReceiverObservedAt,
    VerifierKnowledgeAt,
}

impl TemporalDimensionV1 {
    fn is_fact(self) -> bool {
        matches!(self, Self::OccurredAt | Self::ValidFrom | Self::ValidTo)
    }

    fn requires_assertion_ref(self) -> bool {
        matches!(
            self,
            Self::SourceRecordedAt | Self::ReceiverObservedAt | Self::VerifierKnowledgeAt
        )
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalPrecisionV1 {
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
    Minute,
    Day,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalQualityV1 {
    SourceAsserted,
    Estimated,
    Uncertain,
    Contested,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct UncertaintyWindowV1 {
    pub earliest: String,
    pub latest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalAssertionV1 {
    pub assertion_id: String,
    pub claim_id: String,
    pub subject_ref: String,
    pub dimension: TemporalDimensionV1,
    pub time_value: String,
    pub precision: TemporalPrecisionV1,
    pub quality: TemporalQualityV1,
    pub clock_profile: String,
    pub source_authority_ref: String,
    pub evidence_ref: String,
    /// SHA-256 digest of the canonical evidence payload, lower-case hex.
    pub content_digest: String,
    /// For record/observation/knowledge metadata, the exact fact assertion
    /// whose time/evidence lifecycle this metadata qualifies.
    pub assertion_ref: Option<String>,
    /// Required for VERIFIER_KNOWLEDGE_AT; knowledge is verifier-specific.
    pub verifier_ref: Option<String>,
    pub knowledge_frontier_ref: Option<String>,
    /// Append-only correction link for OCCURRED_AT / VALID_FROM / VALID_TO.
    pub supersedes_assertion_id: Option<String>,
    pub uncertainty_window: Option<UncertaintyWindowV1>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueryDispositionV1 {
    ValidAtQuery,
    NotValidAtQuery,
    Indeterminate,
    Conflict,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BitemporalValidityResultV1 {
    pub disposition: QueryDispositionV1,
    pub claim_id: String,
    pub subject_ref: String,
    pub verifier_ref: String,
    pub valid_at: String,
    pub known_at: String,
    pub valid_from_assertion: Option<String>,
    pub valid_to_assertion: Option<String>,
    pub knowledge_assertions: Vec<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OccurrenceQueryDispositionV1 {
    Known,
    NotKnown,
    Conflict,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OccurrenceAsKnownResultV1 {
    pub disposition: OccurrenceQueryDispositionV1,
    pub assertion_id: Option<String>,
    pub claim_id: String,
    pub subject_ref: String,
    pub occurred_at: Option<String>,
    pub quality: Option<TemporalQualityV1>,
    pub precision: Option<TemporalPrecisionV1>,
    pub uncertainty_window: Option<UncertaintyWindowV1>,
    pub knowledge_assertion_ids: Vec<String>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TemporalKernelErrorV1 {
    EmptyInput,
    EmptyIdentifier,
    IdentifierTooLong,
    TextTooLong,
    DuplicateAssertionId,
    ClaimSubjectMismatch,
    InvalidTimestamp,
    NonUtcTimestampUnsupported,
    TimestampPrecisionMismatch,
    InvalidContentDigest,
    InvalidValidityInterval,
    MissingEvidenceReference,
    MissingClockProfile,
    MissingSourceAuthority,
    MissingAssertionReference,
    MissingVerifierReference,
    MissingKnowledgeFrontier,
    UncertaintyWindowRequired,
    InvalidUncertaintyWindow,
    InvalidAssertionReference,
    MetadataReferencesMetadata,
    SupersessionOnMetadata,
    InvalidSupersessionTarget,
    SupersessionCycle,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TemporalEvidenceSetV1 {
    pub assertions: Vec<TemporalAssertionV1>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct NormalizedTime {
    sortable: String,
    fractional_digits: usize,
}

impl TemporalEvidenceSetV1 {
    pub fn validate(&self) -> Result<(), TemporalKernelErrorV1> {
        if self.assertions.is_empty() {
            return Err(TemporalKernelErrorV1::EmptyInput);
        }

        let mut by_id = BTreeMap::<&str, &TemporalAssertionV1>::new();
        let mut claim_subjects = BTreeMap::<&str, &str>::new();
        for assertion in &self.assertions {
            validate_identifier(&assertion.assertion_id)?;
            validate_identifier(&assertion.claim_id)?;
            validate_identifier(&assertion.subject_ref)?;
            if let Some(previous_subject) = claim_subjects.insert(
                assertion.claim_id.as_str(),
                assertion.subject_ref.as_str(),
            ) {
                if previous_subject != assertion.subject_ref {
                    return Err(TemporalKernelErrorV1::ClaimSubjectMismatch);
                }
            }
            validate_identifier(&assertion.clock_profile)
                .map_err(|_| TemporalKernelErrorV1::MissingClockProfile)?;
            validate_identifier(&assertion.source_authority_ref)
                .map_err(|_| TemporalKernelErrorV1::MissingSourceAuthority)?;
            validate_identifier(&assertion.evidence_ref)
                .map_err(|_| TemporalKernelErrorV1::MissingEvidenceReference)?;
            validate_content_digest(&assertion.content_digest)?;

            if by_id
                .insert(assertion.assertion_id.as_str(), assertion)
                .is_some()
            {
                return Err(TemporalKernelErrorV1::DuplicateAssertionId);
            }

            let time = normalize_utc(&assertion.time_value)?;
            if !timestamp_matches_precision(&time, assertion.precision) {
                return Err(TemporalKernelErrorV1::TimestampPrecisionMismatch);
            }

            match (&assertion.dimension, &assertion.assertion_ref) {
                (dimension, None) if dimension.requires_assertion_ref() => {
                    return Err(TemporalKernelErrorV1::MissingAssertionReference);
                }
                (dimension, Some(_)) if dimension.is_fact() => {
                    return Err(TemporalKernelErrorV1::InvalidAssertionReference);
                }
                _ => {}
            }

            if assertion.dimension == TemporalDimensionV1::VerifierKnowledgeAt {
                validate_optional_identifier(assertion.verifier_ref.as_deref())
                    .map_err(|_| TemporalKernelErrorV1::MissingVerifierReference)?;
                validate_optional_identifier(assertion.knowledge_frontier_ref.as_deref())
                    .map_err(|_| TemporalKernelErrorV1::MissingKnowledgeFrontier)?;
            }

            if assertion.supersedes_assertion_id.is_some() && !assertion.dimension.is_fact() {
                return Err(TemporalKernelErrorV1::SupersessionOnMetadata);
            }

            match (&assertion.quality, &assertion.uncertainty_window) {
                (TemporalQualityV1::Uncertain, None) => {
                    return Err(TemporalKernelErrorV1::UncertaintyWindowRequired);
                }
                (_, Some(window)) => {
                    let earliest = normalize_utc(&window.earliest)?;
                    let latest = normalize_utc(&window.latest)?;
                    if earliest.sortable > latest.sortable {
                        return Err(TemporalKernelErrorV1::InvalidUncertaintyWindow);
                    }
                }
                _ => {}
            }
        }

        for assertion in &self.assertions {
            if assertion.dimension.requires_assertion_ref() {
                let reference = assertion
                    .assertion_ref
                    .as_deref()
                    .ok_or(TemporalKernelErrorV1::MissingAssertionReference)?;
                let Some(target) = by_id.get(reference) else {
                    return Err(TemporalKernelErrorV1::InvalidAssertionReference);
                };
                if !target.dimension.is_fact()
                    || target.claim_id != assertion.claim_id
                    || target.subject_ref != assertion.subject_ref
                {
                    return Err(TemporalKernelErrorV1::InvalidAssertionReference);
                }
                if assertion.dimension == TemporalDimensionV1::VerifierKnowledgeAt
                    && (assertion.verifier_ref.is_none() || assertion.knowledge_frontier_ref.is_none())
                {
                    return Err(TemporalKernelErrorV1::MissingVerifierReference);
                }
            }

            if let Some(previous_id) = &assertion.supersedes_assertion_id {
                let Some(previous) = by_id.get(previous_id.as_str()) else {
                    return Err(TemporalKernelErrorV1::InvalidSupersessionTarget);
                };
                if !previous.dimension.is_fact()
                    || previous.claim_id != assertion.claim_id
                    || previous.subject_ref != assertion.subject_ref
                    || previous.dimension != assertion.dimension
                {
                    return Err(TemporalKernelErrorV1::InvalidSupersessionTarget);
                }
            }
        }

        self.validate_supersession_acyclic(&by_id)
    }

    fn validate_supersession_acyclic(
        &self,
        by_id: &BTreeMap<&str, &TemporalAssertionV1>,
    ) -> Result<(), TemporalKernelErrorV1> {
        for assertion in &self.assertions {
            let mut visited = BTreeSet::new();
            let mut current = assertion;
            while let Some(previous_id) = current.supersedes_assertion_id.as_deref() {
                if !visited.insert(previous_id) {
                    return Err(TemporalKernelErrorV1::SupersessionCycle);
                }
                current = by_id
                    .get(previous_id)
                    .copied()
                    .ok_or(TemporalKernelErrorV1::InvalidSupersessionTarget)?;
            }
        }
        Ok(())
    }

    /// Evaluate a valid-time interval at V using only assertions accepted by
    /// the named verifier by K. The interval is half-open: [valid_from, valid_to).
    /// An absent VALID_TO is open-ended only when the caller explicitly allows it.
    pub fn evaluate_valid_at(
        &self,
        claim_id: &str,
        verifier_ref: &str,
        valid_at: &str,
        known_at: &str,
        allow_open_ended_valid_to: bool,
    ) -> Result<BitemporalValidityResultV1, TemporalKernelErrorV1> {
        self.validate()?;
        validate_identifier(claim_id)?;
        validate_identifier(verifier_ref)?;
        let valid_query = normalize_utc(valid_at)?;
        let knowledge_query = normalize_utc(known_at)?;

        let claim_assertions: Vec<&TemporalAssertionV1> = self
            .assertions
            .iter()
            .filter(|assertion| assertion.claim_id == claim_id && assertion.dimension.is_fact())
            .collect();

        let Some(subject_ref) = claim_assertions.first().map(|assertion| assertion.subject_ref.clone()) else {
            return Ok(BitemporalValidityResultV1 {
                disposition: QueryDispositionV1::Indeterminate,
                claim_id: claim_id.into(),
                subject_ref: String::new(),
                verifier_ref: verifier_ref.into(),
                valid_at: valid_at.into(),
                known_at: known_at.into(),
                valid_from_assertion: None,
                valid_to_assertion: None,
                knowledge_assertions: Vec::new(),
            });
        };

        let from = self.select_as_known(
            claim_id,
            TemporalDimensionV1::ValidFrom,
            verifier_ref,
            &knowledge_query,
        )?;
        let to = self.select_as_known(
            claim_id,
            TemporalDimensionV1::ValidTo,
            verifier_ref,
            &knowledge_query,
        )?;

        let (from, from_knowledge) = match from {
            Selection::Selected(assertion, knowledge) => (Some(assertion), knowledge),
            Selection::Missing => (None, Vec::new()),
            Selection::Conflict => {
                return Ok(self.validity_result(
                    QueryDispositionV1::Conflict,
                    claim_id,
                    &subject_ref,
                    verifier_ref,
                    valid_at,
                    known_at,
                    None,
                    None,
                    Vec::new(),
                ));
            }
        };
        let Some(from) = from else {
            return Ok(self.validity_result(
                QueryDispositionV1::Indeterminate,
                claim_id,
                &subject_ref,
                verifier_ref,
                valid_at,
                known_at,
                None,
                None,
                Vec::new(),
            ));
        };

        let (to, to_knowledge) = match to {
            Selection::Selected(assertion, knowledge) => (Some(assertion), knowledge),
            Selection::Missing if allow_open_ended_valid_to => (None, Vec::new()),
            Selection::Missing => {
                return Ok(self.validity_result(
                    QueryDispositionV1::Indeterminate,
                    claim_id,
                    &subject_ref,
                    verifier_ref,
                    valid_at,
                    known_at,
                    Some(from),
                    None,
                    from_knowledge,
                ));
            }
            Selection::Conflict => {
                return Ok(self.validity_result(
                    QueryDispositionV1::Conflict,
                    claim_id,
                    &subject_ref,
                    verifier_ref,
                    valid_at,
                    known_at,
                    Some(from),
                    None,
                    from_knowledge,
                ));
            }
        };

        if let Some(to_assertion) = to {
            let from_time = normalize_utc(&from.time_value)?;
            let to_time = normalize_utc(&to_assertion.time_value)?;
            if from_time.sortable > to_time.sortable {
                return Err(TemporalKernelErrorV1::InvalidValidityInterval);
            }
        }

        let mut knowledge_ids = from_knowledge;
        knowledge_ids.extend(to_knowledge);
        knowledge_ids.sort();
        knowledge_ids.dedup();

        let disposition = if from.quality == TemporalQualityV1::Contested
            || to.is_some_and(|assertion| assertion.quality == TemporalQualityV1::Contested)
        {
            QueryDispositionV1::Conflict
        } else if boundary_result(from, &valid_query, true) == BoundaryResult::Uncertain
            || to.is_some_and(|assertion| {
                boundary_result(assertion, &valid_query, false) == BoundaryResult::Uncertain
            })
        {
            QueryDispositionV1::Indeterminate
        } else if boundary_result(from, &valid_query, true) == BoundaryResult::Outside
            || to.is_some_and(|assertion| {
                boundary_result(assertion, &valid_query, false) == BoundaryResult::Outside
            })
        {
            QueryDispositionV1::NotValidAtQuery
        } else {
            QueryDispositionV1::ValidAtQuery
        };

        Ok(self.validity_result(
            disposition,
            claim_id,
            &subject_ref,
            verifier_ref,
            valid_at,
            known_at,
            Some(from),
            to,
            knowledge_ids,
        ))
    }

    /// Return the occurrence-time assertion visible to verifier X by K.
    /// If a later correction was not yet known at K, the earlier assertion
    /// remains the historical answer.
    pub fn occurred_at_as_known(
        &self,
        claim_id: &str,
        verifier_ref: &str,
        known_at: &str,
    ) -> Result<OccurrenceAsKnownResultV1, TemporalKernelErrorV1> {
        self.validate()?;
        validate_identifier(claim_id)?;
        validate_identifier(verifier_ref)?;
        let knowledge_query = normalize_utc(known_at)?;
        let result = match self.select_as_known(
            claim_id,
            TemporalDimensionV1::OccurredAt,
            verifier_ref,
            &knowledge_query,
        )? {
            Selection::Missing => OccurrenceAsKnownResultV1 {
                disposition: OccurrenceQueryDispositionV1::NotKnown,
                assertion_id: None,
                claim_id: claim_id.into(),
                subject_ref: String::new(),
                occurred_at: None,
                quality: None,
                precision: None,
                uncertainty_window: None,
                knowledge_assertion_ids: Vec::new(),
            },
            Selection::Conflict => OccurrenceAsKnownResultV1 {
                disposition: OccurrenceQueryDispositionV1::Conflict,
                assertion_id: None,
                claim_id: claim_id.into(),
                subject_ref: String::new(),
                occurred_at: None,
                quality: None,
                precision: None,
                uncertainty_window: None,
                knowledge_assertion_ids: Vec::new(),
            },
            Selection::Selected(assertion, knowledge_ids) => OccurrenceAsKnownResultV1 {
                disposition: OccurrenceQueryDispositionV1::Known,
                assertion_id: Some(assertion.assertion_id.clone()),
                claim_id: assertion.claim_id.clone(),
                subject_ref: assertion.subject_ref.clone(),
                occurred_at: Some(assertion.time_value.clone()),
                quality: Some(assertion.quality),
                precision: Some(assertion.precision),
                uncertainty_window: assertion.uncertainty_window.clone(),
                knowledge_assertion_ids: knowledge_ids,
            },
        };
        Ok(result)
    }

    fn select_as_known(
        &self,
        claim_id: &str,
        dimension: TemporalDimensionV1,
        verifier_ref: &str,
        known_at: &NormalizedTime,
    ) -> Result<Selection<'_>, TemporalKernelErrorV1> {
        let by_id: BTreeMap<&str, &TemporalAssertionV1> = self
            .assertions
            .iter()
            .map(|assertion| (assertion.assertion_id.as_str(), assertion))
            .collect();

        let mut eligible: Vec<(&TemporalAssertionV1, Vec<String>)> = Vec::new();
        for assertion in self.assertions.iter().filter(|assertion| {
            assertion.claim_id == claim_id && assertion.dimension == dimension
        }) {
            let mut knowledge_ids = Vec::new();
            let mut contested_knowledge = false;
            let mut uncertain_knowledge = false;
            for knowledge in self.assertions.iter().filter(|knowledge| {
                knowledge.dimension == TemporalDimensionV1::VerifierKnowledgeAt
                    && knowledge.assertion_ref.as_deref() == Some(assertion.assertion_id.as_str())
                    && knowledge.verifier_ref.as_deref() == Some(verifier_ref)
            }) {
                let time = normalize_utc(&knowledge.time_value)?;
                if time.sortable <= known_at.sortable {
                    match knowledge.quality {
                        TemporalQualityV1::SourceAsserted => {
                            knowledge_ids.push(knowledge.assertion_id.clone());
                        }
                        TemporalQualityV1::Contested => contested_knowledge = true,
                        TemporalQualityV1::Estimated | TemporalQualityV1::Uncertain => {
                            uncertain_knowledge = true;
                        }
                    }
                }
            }
            if contested_knowledge || (uncertain_knowledge && knowledge_ids.is_empty()) {
                return Ok(Selection::Conflict);
            }
            if !knowledge_ids.is_empty() {
                knowledge_ids.sort();
                eligible.push((assertion, knowledge_ids));
            }
        }

        if eligible.is_empty() {
            return Ok(Selection::Missing);
        }

        let mut maximal: Vec<&TemporalAssertionV1> = Vec::new();
        for (candidate, _) in &eligible {
            let mut shadowed = false;
            for (other, _) in &eligible {
                if candidate.assertion_id == other.assertion_id {
                    continue;
                }
                let mut current = *other;
                while let Some(parent_id) = current.supersedes_assertion_id.as_deref() {
                    if parent_id == candidate.assertion_id {
                        shadowed = true;
                        break;
                    }
                    let Some(parent) = by_id.get(parent_id) else {
                        break;
                    };
                    current = parent;
                }
                if shadowed {
                    break;
                }
            }
            if !shadowed {
                maximal.push(candidate);
            }
        }

        if maximal.len() != 1 {
            return Ok(Selection::Conflict);
        }

        let selected = maximal[0];
        let knowledge_ids = eligible
            .into_iter()
            .find(|(assertion, _)| assertion.assertion_id == selected.assertion_id)
            .map(|(_, knowledge)| knowledge)
            .unwrap_or_default();
        Ok(Selection::Selected(selected, knowledge_ids))
    }

    fn validity_result(
        &self,
        disposition: QueryDispositionV1,
        claim_id: &str,
        subject_ref: &str,
        verifier_ref: &str,
        valid_at: &str,
        known_at: &str,
        valid_from: Option<&TemporalAssertionV1>,
        valid_to: Option<&TemporalAssertionV1>,
        knowledge_assertions: Vec<String>,
    ) -> BitemporalValidityResultV1 {
        BitemporalValidityResultV1 {
            disposition,
            claim_id: claim_id.into(),
            subject_ref: subject_ref.into(),
            verifier_ref: verifier_ref.into(),
            valid_at: valid_at.into(),
            known_at: known_at.into(),
            valid_from_assertion: valid_from.map(|assertion| assertion.assertion_id.clone()),
            valid_to_assertion: valid_to.map(|assertion| assertion.assertion_id.clone()),
            knowledge_assertions,
        }
    }
}

enum Selection<'a> {
    Selected(&'a TemporalAssertionV1, Vec<String>),
    Missing,
    Conflict,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundaryResult {
    Inside,
    Outside,
    Uncertain,
}

fn boundary_result(
    assertion: &TemporalAssertionV1,
    valid_at: &NormalizedTime,
    is_start: bool,
) -> BoundaryResult {
    if let Some(window) = &assertion.uncertainty_window {
        let Ok(earliest) = normalize_utc(&window.earliest) else {
            return BoundaryResult::Uncertain;
        };
        let Ok(latest) = normalize_utc(&window.latest) else {
            return BoundaryResult::Uncertain;
        };
        if valid_at.sortable < earliest.sortable {
            return if is_start {
                BoundaryResult::Outside
            } else {
                BoundaryResult::Inside
            };
        }
        if valid_at.sortable >= latest.sortable {
            return if is_start {
                BoundaryResult::Inside
            } else {
                BoundaryResult::Outside
            };
        }
        return BoundaryResult::Uncertain;
    }

    let Ok(boundary) = normalize_utc(&assertion.time_value) else {
        return BoundaryResult::Uncertain;
    };
    if is_start {
        if valid_at.sortable < boundary.sortable {
            BoundaryResult::Outside
        } else {
            BoundaryResult::Inside
        }
    } else if valid_at.sortable >= boundary.sortable {
        BoundaryResult::Outside
    } else {
        BoundaryResult::Inside
    }
}

fn validate_optional_identifier(value: Option<&str>) -> Result<(), TemporalKernelErrorV1> {
    let value = value.ok_or(TemporalKernelErrorV1::EmptyIdentifier)?;
    validate_identifier(value)
}

fn validate_identifier(value: &str) -> Result<(), TemporalKernelErrorV1> {
    if value.trim().is_empty() {
        return Err(TemporalKernelErrorV1::EmptyIdentifier);
    }
    if value.len() > MAX_ID_BYTES {
        return Err(TemporalKernelErrorV1::IdentifierTooLong);
    }
    Ok(())
}

fn normalize_utc(value: &str) -> Result<NormalizedTime, TemporalKernelErrorV1> {
    if !value.ends_with('Z') {
        if value.contains('+') || value.get(19..).is_some_and(|suffix| suffix.contains('-')) {
            return Err(TemporalKernelErrorV1::NonUtcTimestampUnsupported);
        }
        return Err(TemporalKernelErrorV1::InvalidTimestamp);
    }
    let body = &value[..value.len() - 1];
    let (whole_seconds, fraction) = match body.split_once('.') {
        Some((whole, fraction)) => {
            if fraction.is_empty()
                || fraction.len() > 9
                || !fraction.bytes().all(|byte| byte.is_ascii_digit())
            {
                return Err(TemporalKernelErrorV1::InvalidTimestamp);
            }
            (whole, fraction)
        }
        None => (body, ""),
    };
    if whole_seconds.len() != 19 {
        return Err(TemporalKernelErrorV1::InvalidTimestamp);
    }
    let bytes = whole_seconds.as_bytes();
    if bytes[4] != b'-' || bytes[7] != b'-' || bytes[10] != b'T'
        || bytes[13] != b':' || bytes[16] != b':'
    {
        return Err(TemporalKernelErrorV1::InvalidTimestamp);
    }

    let year = parse_digits(bytes, 0, 4).ok_or(TemporalKernelErrorV1::InvalidTimestamp)?;
    let month = parse_digits(bytes, 5, 2).ok_or(TemporalKernelErrorV1::InvalidTimestamp)?;
    let day = parse_digits(bytes, 8, 2).ok_or(TemporalKernelErrorV1::InvalidTimestamp)?;
    let hour = parse_digits(bytes, 11, 2).ok_or(TemporalKernelErrorV1::InvalidTimestamp)?;
    let minute = parse_digits(bytes, 14, 2).ok_or(TemporalKernelErrorV1::InvalidTimestamp)?;
    let second = parse_digits(bytes, 17, 2).ok_or(TemporalKernelErrorV1::InvalidTimestamp)?;
    if year == 0 || !(1..=12).contains(&month) || !(0..=23).contains(&hour)
        || !(0..=59).contains(&minute) || !(0..=59).contains(&second)
    {
        return Err(TemporalKernelErrorV1::InvalidTimestamp);
    }
    let leap_year = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let month_days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap_year => 29,
        2 => 28,
        _ => return Err(TemporalKernelErrorV1::InvalidTimestamp),
    };
    if day == 0 || day > month_days {
        return Err(TemporalKernelErrorV1::InvalidTimestamp);
    }

    // RFC3339 permits 1-9 fractional digits; right-padding preserves exact
    // temporal ordering without claiming greater source precision.
    let sortable_fraction = format!("{fraction:0<9}");
    let normalized = format!(
        "{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}.{sortable_fraction}Z"
    );
    Ok(NormalizedTime {
        sortable: normalized,
        fractional_digits: fraction.len(),
    })
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

fn timestamp_matches_precision(value: &NormalizedTime, precision: TemporalPrecisionV1) -> bool {
    let bytes = value.sortable.as_bytes();
    let Some(hour) = parse_digits(bytes, 11, 2) else { return false; };
    let Some(minute) = parse_digits(bytes, 14, 2) else { return false; };
    let Some(second) = parse_digits(bytes, 17, 2) else { return false; };
    match precision {
        TemporalPrecisionV1::Day => hour == 0 && minute == 0 && second == 0
            && value.fractional_digits == 0,
        TemporalPrecisionV1::Minute => second == 0 && value.fractional_digits == 0,
        TemporalPrecisionV1::Second => value.fractional_digits == 0,
        TemporalPrecisionV1::Millisecond => value.fractional_digits == 3,
        TemporalPrecisionV1::Microsecond => value.fractional_digits == 6,
        TemporalPrecisionV1::Nanosecond => value.fractional_digits == 9,
    }
}

fn validate_content_digest(value: &str) -> Result<(), TemporalKernelErrorV1> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(TemporalKernelErrorV1::InvalidContentDigest);
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(TemporalKernelErrorV1::InvalidContentDigest);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const VERIFIER: &str = "did:mycelix:verifier:finance-v1";

    fn fact(
        id: &str,
        claim: &str,
        subject: &str,
        dimension: TemporalDimensionV1,
        value: &str,
        supersedes: Option<&str>,
    ) -> TemporalAssertionV1 {
        TemporalAssertionV1 {
            assertion_id: id.into(),
            claim_id: claim.into(),
            subject_ref: subject.into(),
            dimension,
            time_value: value.into(),
            precision: TemporalPrecisionV1::Second,
            quality: TemporalQualityV1::SourceAsserted,
            clock_profile: "source-clock-v1".into(),
            source_authority_ref: "source-authority-v1".into(),
            evidence_ref: format!("evidence:{id}"),
            content_digest: format!("sha256:{}", "a".repeat(64)),
            assertion_ref: None,
            verifier_ref: None,
            knowledge_frontier_ref: None,
            supersedes_assertion_id: supersedes.map(str::to_owned),
            uncertainty_window: None,
        }
    }

    fn knowledge(
        id: &str,
        claim: &str,
        subject: &str,
        assertion_ref: &str,
        verifier: &str,
        known_at: &str,
    ) -> TemporalAssertionV1 {
        TemporalAssertionV1 {
            assertion_id: id.into(),
            claim_id: claim.into(),
            subject_ref: subject.into(),
            dimension: TemporalDimensionV1::VerifierKnowledgeAt,
            time_value: known_at.into(),
            precision: TemporalPrecisionV1::Second,
            quality: TemporalQualityV1::SourceAsserted,
            clock_profile: "verifier-clock-v1".into(),
            source_authority_ref: "verifier-profile-v1".into(),
            evidence_ref: format!("evidence:{id}"),
            content_digest: format!("sha256:{}", "b".repeat(64)),
            assertion_ref: Some(assertion_ref.into()),
            verifier_ref: Some(verifier.into()),
            knowledge_frontier_ref: Some(format!("frontier:{id}")),
            supersedes_assertion_id: None,
            uncertainty_window: None,
        }
    }

    fn valid_fixture() -> TemporalEvidenceSetV1 {
        TemporalEvidenceSetV1 {
            assertions: vec![
                fact(
                    "from-v1",
                    "grant-1",
                    "grant:1",
                    TemporalDimensionV1::ValidFrom,
                    "2026-11-18T00:00:00Z",
                    None,
                ),
                fact(
                    "to-v1",
                    "grant-1",
                    "grant:1",
                    TemporalDimensionV1::ValidTo,
                    "2026-11-22T00:00:00Z",
                    None,
                ),
                knowledge(
                    "from-known",
                    "grant-1",
                    "grant:1",
                    "from-v1",
                    VERIFIER,
                    "2026-11-20T08:00:20Z",
                ),
                knowledge(
                    "to-known",
                    "grant-1",
                    "grant:1",
                    "to-v1",
                    VERIFIER,
                    "2026-11-20T08:00:20Z",
                ),
            ],
        }
    }

    #[test]
    fn query_uses_valid_time_and_verifier_knowledge_time_independently() {
        let assertions = valid_fixture();
        let before_revocation_known = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-19T00:00:00Z",
                "2026-11-19T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(
            before_revocation_known.disposition,
            QueryDispositionV1::Indeterminate
        );

        let after_revocation_known = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-19T00:00:00Z",
                "2026-11-21T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(
            after_revocation_known.disposition,
            QueryDispositionV1::ValidAtQuery
        );
        assert_eq!(
            after_revocation_known.valid_from_assertion.as_deref(),
            Some("from-v1")
        );
    }

    #[test]
    fn valid_interval_is_half_open() {
        let assertions = valid_fixture();
        let at_start = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-18T00:00:00Z",
                "2026-11-21T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(at_start.disposition, QueryDispositionV1::ValidAtQuery);

        let at_end = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-22T00:00:00Z",
                "2026-11-21T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(at_end.disposition, QueryDispositionV1::NotValidAtQuery);
    }

    #[test]
    fn occurrence_correction_is_selected_only_after_verifier_knows_it() {
        let mut assertions = TemporalEvidenceSetV1 {
            assertions: vec![
                fact(
                    "occurred-v1",
                    "delivery-1",
                    "delivery:1",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:00Z",
                    None,
                ),
                fact(
                    "occurred-v2",
                    "delivery-1",
                    "delivery:1",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:02Z",
                    Some("occurred-v1"),
                ),
                knowledge(
                    "v1-known",
                    "delivery-1",
                    "delivery:1",
                    "occurred-v1",
                    VERIFIER,
                    "2026-11-21T10:00:10Z",
                ),
                knowledge(
                    "v2-known",
                    "delivery-1",
                    "delivery:1",
                    "occurred-v2",
                    VERIFIER,
                    "2026-11-21T10:00:30Z",
                ),
            ],
        };
        let before_correction = assertions
            .occurred_at_as_known("delivery-1", VERIFIER, "2026-11-21T10:00:20Z")
            .unwrap()
            .unwrap();
        assert_eq!(before_correction.assertion_id.as_deref(), Some("occurred-v1"));
        assert_eq!(before_correction.occurred_at.as_deref(), Some("2026-11-21T10:00:00Z"));

        let after_correction = assertions
            .occurred_at_as_known("delivery-1", VERIFIER, "2026-11-21T10:00:40Z")
            .unwrap()
            .unwrap();
        assert_eq!(after_correction.assertion_id.as_deref(), Some("occurred-v2"));
        assert_eq!(after_correction.occurred_at.as_deref(), Some("2026-11-21T10:00:02Z"));
        assertions.validate().unwrap();
    }

    #[test]
    fn a_different_verifier_does_not_inherit_another_verifiers_knowledge() {
        let assertions = valid_fixture();
        let result = assertions
            .evaluate_valid_at(
                "grant-1",
                "did:mycelix:verifier:other-v1",
                "2026-11-19T00:00:00Z",
                "2026-11-21T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(result.disposition, QueryDispositionV1::Indeterminate);
    }

    #[test]
    fn competing_known_correction_branches_are_a_conflict() {
        let assertions = TemporalEvidenceSetV1 {
            assertions: vec![
                fact(
                    "base",
                    "claim",
                    "subject",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:00Z",
                    None,
                ),
                fact(
                    "branch-a",
                    "claim",
                    "subject",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:01Z",
                    Some("base"),
                ),
                fact(
                    "branch-b",
                    "claim",
                    "subject",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:02Z",
                    Some("base"),
                ),
                knowledge(
                    "base-known",
                    "claim",
                    "subject",
                    "base",
                    VERIFIER,
                    "2026-11-21T10:00:10Z",
                ),
                knowledge(
                    "branch-a-known",
                    "claim",
                    "subject",
                    "branch-a",
                    VERIFIER,
                    "2026-11-21T10:00:20Z",
                ),
                knowledge(
                    "branch-b-known",
                    "claim",
                    "subject",
                    "branch-b",
                    VERIFIER,
                    "2026-11-21T10:00:20Z",
                ),
            ],
        };
        let result = assertions
            .occurred_at_as_known("claim", VERIFIER, "2026-11-21T10:00:30Z")
            .unwrap();
        assert_eq!(result.disposition, OccurrenceQueryDispositionV1::Conflict);
        assert_eq!(result.assertion_id, None);
    }

    #[test]
    fn missing_knowledge_frontier_is_a_missing_fact_not_a_default() {
        let assertions = TemporalEvidenceSetV1 {
            assertions: vec![fact(
                "from",
                "claim",
                "subject",
                TemporalDimensionV1::ValidFrom,
                "2026-11-18T00:00:00Z",
                None,
            )],
        };
        let result = assertions
            .evaluate_valid_at(
                "claim",
                VERIFIER,
                "2026-11-19T00:00:00Z",
                "2026-11-21T00:00:00Z",
                true,
            )
            .unwrap();
        assert_eq!(result.disposition, QueryDispositionV1::Indeterminate);
    }

    #[test]
    fn open_ended_valid_to_requires_explicit_caller_policy() {
        let mut assertions = valid_fixture();
        assertions.assertions.retain(|assertion| {
            assertion.assertion_id != "to-v1" && assertion.assertion_id != "to-known"
        });
        let strict = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-21T00:00:00Z",
                "2026-11-21T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(strict.disposition, QueryDispositionV1::Indeterminate);

        let open = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-21T00:00:00Z",
                "2026-11-21T00:00:00Z",
                true,
            )
            .unwrap();
        assert_eq!(open.disposition, QueryDispositionV1::ValidAtQuery);
    }

    #[test]
    fn uncertain_boundary_returns_indeterminate_inside_window() {
        let mut assertions = valid_fixture();
        let from = assertions
            .assertions
            .iter_mut()
            .find(|assertion| assertion.assertion_id == "from-v1")
            .unwrap();
        from.quality = TemporalQualityV1::Uncertain;
        from.uncertainty_window = Some(UncertaintyWindowV1 {
            earliest: "2026-11-18T00:00:00Z".into(),
            latest: "2026-11-20T00:00:00Z".into(),
        });
        let result = assertions
            .evaluate_valid_at(
                "grant-1",
                VERIFIER,
                "2026-11-19T00:00:00Z",
                "2026-11-21T00:00:00Z",
                false,
            )
            .unwrap();
        assert_eq!(result.disposition, QueryDispositionV1::Indeterminate);
    }

    #[test]
    fn invalid_metadata_target_and_supersession_cycle_are_rejected() {
        let mut assertions = valid_fixture();
        assertions.assertions[2].assertion_ref = Some("missing".into());
        assert_eq!(
            assertions.validate(),
            Err(TemporalKernelErrorV1::InvalidAssertionReference)
        );

        let cycle = TemporalEvidenceSetV1 {
            assertions: vec![
                fact(
                    "a",
                    "claim",
                    "subject",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:00Z",
                    Some("b"),
                ),
                fact(
                    "b",
                    "claim",
                    "subject",
                    TemporalDimensionV1::OccurredAt,
                    "2026-11-21T10:00:01Z",
                    Some("a"),
                ),
            ],
        };
        assert_eq!(
            cycle.validate(),
            Err(TemporalKernelErrorV1::SupersessionCycle)
        );
    }

    #[test]
    fn offsets_are_rejected_until_a_pinned_normalization_profile_exists() {
        let result = normalize_utc("2026-11-21T10:00:00+02:00");
        assert_eq!(result, Err(TemporalKernelErrorV1::NonUtcTimestampUnsupported));
    }

    #[test]
    fn timestamps_with_one_to_nine_fractional_digits_are_ordered_exactly() {
        let millis = normalize_utc("2026-11-21T10:00:00.123Z").unwrap();
        let nanos = normalize_utc("2026-11-21T10:00:00.123000001Z").unwrap();
        assert_eq!(millis.fractional_digits, 3);
        assert_eq!(nanos.fractional_digits, 9);
        assert!(millis.sortable < nanos.sortable);
        assert_eq!(
            normalize_utc("2026-11-21T10:00:00.1Z").unwrap().sortable,
            "2026-11-21T10:00:00.100000000Z"
        );
    }

    #[test]
    fn declared_fractional_precision_must_match_encoded_digits() {
        let millis = normalize_utc("2026-11-21T10:00:00.123Z").unwrap();
        assert!(timestamp_matches_precision(&millis, TemporalPrecisionV1::Millisecond));
        assert!(!timestamp_matches_precision(&millis, TemporalPrecisionV1::Microsecond));
        let whole_second = normalize_utc("2026-11-21T10:00:00Z").unwrap();
        assert!(timestamp_matches_precision(&whole_second, TemporalPrecisionV1::Second));
        assert!(!timestamp_matches_precision(&whole_second, TemporalPrecisionV1::Nanosecond));
    }

    #[test]
    fn invalid_digest_and_reversed_validity_interval_are_rejected() {
        let mut invalid_digest = valid_fixture();
        invalid_digest.assertions[0].content_digest = "sha256:abc".into();
        assert_eq!(
            invalid_digest.validate(),
            Err(TemporalKernelErrorV1::InvalidContentDigest)
        );

        let mut reversed = valid_fixture();
        let to = reversed.assertions.iter_mut()
            .find(|assertion| assertion.assertion_id == "to-v1")
            .unwrap();
        to.time_value = "2026-11-17T00:00:00Z".into();
        assert_eq!(
            reversed.evaluate_valid_at(
                "grant-1", VERIFIER, "2026-11-19T00:00:00Z",
                "2026-11-21T00:00:00Z", false
            ),
            Err(TemporalKernelErrorV1::InvalidValidityInterval)
        );
    }
}
