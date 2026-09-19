use serde::{Deserialize, Serialize};

pub const CARE_LEDGER_SCHEMA_VERSION: u16 = 1;
const MAX_ID_LEN: usize = 512;
const MAX_EVIDENCE_REFS: usize = 64;
const MAX_MINUTES: u32 = 7 * 24 * 60;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ScheduleId(pub String);
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct OccurrenceId(pub String);
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct MemberId(pub String);
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct RecordId(pub String);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TemplateState {
    Active,
    Paused,
    Retired,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceWindow {
    pub start_micros: i64,
    pub end_micros: i64,
}
impl OccurrenceWindow {
    pub fn validate(&self) -> Result<(), LedgerError> {
        if self.start_micros >= self.end_micros {
            return Err(LedgerError::InvalidWindow);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareOccurrence {
    pub schema_version: u16,
    pub id: OccurrenceId,
    pub schedule_id: ScheduleId,
    pub assigned_to: MemberId,
    pub window: OccurrenceWindow,
    /// Explicit household/planner estimate. Unknown remains None.
    pub estimated_minutes: Option<u32>,
}

impl CareOccurrence {
    pub fn new(
        schedule_id: ScheduleId,
        assigned_to: MemberId,
        window: OccurrenceWindow,
        estimated_minutes: Option<u32>,
    ) -> Result<Self, LedgerError> {
        validate_id("schedule_id", &schedule_id.0)?;
        validate_id("assigned_to", &assigned_to.0)?;
        window.validate()?;
        validate_minutes(estimated_minutes)?;
        let id = derive_occurrence_id(&schedule_id, &window)?;
        Ok(Self {
            schema_version: CARE_LEDGER_SCHEMA_VERSION,
            id,
            schedule_id,
            assigned_to,
            window,
            estimated_minutes,
        })
    }

    pub fn validate(&self) -> Result<(), LedgerError> {
        if self.schema_version != CARE_LEDGER_SCHEMA_VERSION {
            return Err(LedgerError::UnsupportedSchema(self.schema_version));
        }
        validate_id("schedule_id", &self.schedule_id.0)?;
        validate_id("assigned_to", &self.assigned_to.0)?;
        validate_id("occurrence_id", &self.id.0)?;
        self.window.validate()?;
        validate_minutes(self.estimated_minutes)?;
        if derive_occurrence_id(&self.schedule_id, &self.window)? != self.id {
            return Err(LedgerError::OccurrenceIdentityMismatch);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceRecord {
    /// DHT ActionHash / local record id / equivalent stable record identity.
    pub record_id: RecordId,
    pub occurrence: CareOccurrence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareCompletion {
    pub schema_version: u16,
    pub occurrence_id: OccurrenceId,
    /// Member who actually performed the work. Workload/fairness aggregates use this field.
    pub performed_by: MemberId,
    /// Member/agent who recorded or attested the completion. On DHT this should bind to author.
    pub recorded_by: MemberId,
    pub completed_at_micros: i64,
    /// Measured/reported duration. None means unknown; never infer one hour.
    pub actual_minutes: Option<u32>,
    pub evidence_refs: Vec<String>,
}
impl CareCompletion {
    pub fn validate(&self) -> Result<(), LedgerError> {
        if self.schema_version != CARE_LEDGER_SCHEMA_VERSION {
            return Err(LedgerError::UnsupportedSchema(self.schema_version));
        }
        validate_id("occurrence_id", &self.occurrence_id.0)?;
        validate_id("performed_by", &self.performed_by.0)?;
        validate_id("recorded_by", &self.recorded_by.0)?;
        validate_minutes(self.actual_minutes)?;
        if self.evidence_refs.len() > MAX_EVIDENCE_REFS {
            return Err(LedgerError::TooManyEvidenceRefs(self.evidence_refs.len()));
        }
        for evidence in &self.evidence_refs {
            validate_id("evidence_ref", evidence)?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompletionRecord {
    pub record_id: RecordId,
    pub completion: CareCompletion,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MaterializeDecision {
    Create(CareOccurrence),
    Existing(OccurrenceRecord),
    InhibitedTemplateState(TemplateState),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LedgerError {
    EmptyId { field: &'static str },
    IdTooLong { field: &'static str, len: usize },
    InvalidWindow,
    InvalidMinutes(u32),
    UnsupportedSchema(u16),
    OccurrenceIdentityMismatch,
    OccurrenceConflict { occurrence_id: OccurrenceId },
    TooManyEvidenceRefs(usize),
}

/// Stable occurrence identity from schedule + concrete window.
/// Length-prefixing avoids separator ambiguity and runtime hashing differences.
pub fn derive_occurrence_id(
    schedule_id: &ScheduleId,
    window: &OccurrenceWindow,
) -> Result<OccurrenceId, LedgerError> {
    validate_id("schedule_id", &schedule_id.0)?;
    window.validate()?;
    Ok(OccurrenceId(format!(
        "care-occ-v1|{}|{}|{}|{}",
        schedule_id.0.len(), schedule_id.0, window.start_micros, window.end_micros
    )))
}

pub(crate) fn validate_id(field: &'static str, value: &str) -> Result<(), LedgerError> {
    if value.is_empty() {
        return Err(LedgerError::EmptyId { field });
    }
    if value.len() > MAX_ID_LEN {
        return Err(LedgerError::IdTooLong { field, len: value.len() });
    }
    Ok(())
}

fn validate_minutes(value: Option<u32>) -> Result<(), LedgerError> {
    if let Some(minutes) = value {
        if minutes == 0 || minutes > MAX_MINUTES {
            return Err(LedgerError::InvalidMinutes(minutes));
        }
    }
    Ok(())
}
