use crate::model::validate_id;
use crate::{
    CareOccurrence, CompletionRecord, LedgerError, OccurrenceId, OccurrenceRecord, RecordId,
    TemplateState,
};
use std::collections::{BTreeMap, BTreeSet};

/// Decide whether a concrete occurrence should be created or reused.
pub fn decide_materialization(
    template_state: TemplateState,
    proposed: CareOccurrence,
    existing: &[OccurrenceRecord],
) -> Result<crate::MaterializeDecision, LedgerError> {
    proposed.validate()?;
    if template_state != TemplateState::Active {
        return Ok(crate::MaterializeDecision::InhibitedTemplateState(template_state));
    }
    let canonical = canonicalize_occurrences(existing)?;
    if let Some(record) = canonical.get(&proposed.id) {
        if record.occurrence == proposed {
            return Ok(crate::MaterializeDecision::Existing(record.clone()));
        }
        return Err(LedgerError::OccurrenceConflict {
            occurrence_id: proposed.id,
        });
    }
    Ok(crate::MaterializeDecision::Create(proposed))
}

/// Collapse identical concurrent materializations using stable record id.
/// The same occurrence id with different semantics is unsafe and fails closed.
pub fn canonicalize_occurrences(
    records: &[OccurrenceRecord],
) -> Result<BTreeMap<OccurrenceId, OccurrenceRecord>, LedgerError> {
    let mut out = BTreeMap::new();
    let mut seen_records = BTreeSet::new();
    for record in records {
        validate_id("record_id", &record.record_id.0)?;
        record.occurrence.validate()?;
        if !seen_records.insert(record.record_id.clone()) {
            continue;
        }
        match out.get(&record.occurrence.id) {
            None => {
                out.insert(record.occurrence.id.clone(), record.clone());
            }
            Some(existing) if existing.occurrence != record.occurrence => {
                return Err(LedgerError::OccurrenceConflict {
                    occurrence_id: record.occurrence.id.clone(),
                });
            }
            Some(existing) if record.record_id < existing.record_id => {
                out.insert(record.occurrence.id.clone(), record.clone());
            }
            Some(_) => {}
        }
    }
    Ok(out)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanonicalCompletion {
    pub canonical: CompletionRecord,
    pub duplicate_record_ids: Vec<RecordId>,
    /// Distinct records disagree on performer or actual duration.
    /// Different recorders may independently attest the same underlying work.
    pub semantic_conflict: bool,
}

/// At most one logical completion is counted per occurrence.
/// The smallest stable record id is canonical; disagreement over the actual
/// work is surfaced, while different recorders may agree on one completion.
pub fn canonicalize_completions(
    records: &[CompletionRecord],
) -> Result<BTreeMap<OccurrenceId, CanonicalCompletion>, LedgerError> {
    let mut grouped: BTreeMap<OccurrenceId, Vec<CompletionRecord>> = BTreeMap::new();
    let mut seen_records = BTreeSet::new();
    for record in records {
        validate_id("record_id", &record.record_id.0)?;
        record.completion.validate()?;
        if !seen_records.insert(record.record_id.clone()) {
            continue;
        }
        grouped
            .entry(record.completion.occurrence_id.clone())
            .or_default()
            .push(record.clone());
    }

    let mut out = BTreeMap::new();
    for (occurrence_id, mut group) in grouped {
        group.sort_by(|a, b| a.record_id.cmp(&b.record_id));
        let canonical = group[0].clone();
        let semantic_conflict = group.iter().skip(1).any(|other| {
            other.completion.performed_by != canonical.completion.performed_by
                || other.completion.actual_minutes != canonical.completion.actual_minutes
        });
        let duplicate_record_ids = group
            .iter()
            .skip(1)
            .map(|record| record.record_id.clone())
            .collect();
        out.insert(
            occurrence_id,
            CanonicalCompletion {
                canonical,
                duplicate_record_ids,
                semantic_conflict,
            },
        );
    }
    Ok(out)
}
