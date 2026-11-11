//! Deterministic delta-debugging shrinker for frozen Trace V1 inputs.
//!
//! The caller supplies the failure predicate. This keeps failure identity
//! independent from the shrinker and prevents it from declaring a new failure
//! equivalent merely because a different invariant also fails.

use crate::trace::{TraceError, TraceV1};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceShrinkResultV1 {
    pub minimized: TraceV1,
    pub attempts: u64,
    pub original_digest: String,
    pub minimized_digest: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceShrinkError {
    InvalidInput(TraceError),
    PredicateDoesNotHold,
    MandatoryPrefixMissing,
}

pub fn shrink_trace_v1<F>(
    original: &TraceV1,
    mut preserves_failure: F,
) -> Result<TraceShrinkResultV1, TraceShrinkError>
where
    F: FnMut(&TraceV1) -> bool,
{
    original
        .canonical_bytes()
        .map_err(TraceShrinkError::InvalidInput)?;
    if original.commands.len() < 4 {
        return Err(TraceShrinkError::MandatoryPrefixMissing);
    }
    if !preserves_failure(original) {
        return Err(TraceShrinkError::PredicateDoesNotHold);
    }

    let original_digest = original
        .canonical_digest()
        .map_err(TraceShrinkError::InvalidInput)?;
    let mut minimized = original.clone();
    let mut attempts = 0_u64;

    // Trace V1's common prefix establishes intent, authorization, local
    // preparation and local commit. Keep it intact while minimizing suffixes.
    let mandatory_prefix = 4_usize;
    let mut granularity = 2_usize;

    while minimized.commands.len() > mandatory_prefix {
        let removable = minimized.commands.len() - mandatory_prefix;
        let chunk_size = removable.div_ceil(granularity).max(1);
        let mut cursor = mandatory_prefix;
        let mut reduced = false;

        while cursor < minimized.commands.len() {
            let end = (cursor + chunk_size).min(minimized.commands.len());
            let mut candidate = minimized.clone();
            candidate.commands.drain(cursor..end);
            attempts = attempts.saturating_add(1);

            if candidate.canonical_bytes().is_ok() && preserves_failure(&candidate) {
                minimized = candidate;
                granularity = granularity.saturating_sub(1).max(2);
                reduced = true;
                break;
            }
            cursor = end;
        }

        if !reduced {
            if granularity >= removable {
                break;
            }
            granularity = (granularity * 2).min(removable);
        }
    }

    // Deterministically shrink suffix command numeric fields. The caller's
    // predicate must preserve the same invariant/failure class.
    for command_index in mandatory_prefix..minimized.commands.len() {
        for field_is_subject in [true, false] {
            let current = if field_is_subject {
                minimized.commands[command_index].subject
            } else {
                minimized.commands[command_index].value
            };
            let candidates = [0_u64, 1, 2, current / 2];
            for replacement in candidates {
                if replacement == current {
                    continue;
                }
                let mut candidate = minimized.clone();
                if field_is_subject {
                    candidate.commands[command_index].subject = replacement;
                } else {
                    candidate.commands[command_index].value = replacement;
                }
                attempts = attempts.saturating_add(1);
                if candidate.canonical_bytes().is_ok() && preserves_failure(&candidate) {
                    minimized = candidate;
                    break;
                }
            }
        }
    }

    let minimized_digest = minimized
        .canonical_digest()
        .map_err(TraceShrinkError::InvalidInput)?;

    Ok(TraceShrinkResultV1 {
        minimized,
        attempts,
        original_digest,
        minimized_digest,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::{TraceFault, TraceGeneratorV1, TraceOpcode};

    #[test]
    fn shrinker_preserves_failure_predicate_and_frozen_trace_encoding() {
        let original = (0..4096)
            .map(|index| TraceGeneratorV1::generate(0x5eed_2026, index))
            .find(|trace| trace.scenario == 6)
            .expect("scenario six appears in fixed corpus");

        let predicate = |trace: &TraceV1| {
            trace
                .commands
                .iter()
                .any(|command| command.opcode == TraceOpcode::MutateEvent)
        };
        let result = shrink_trace_v1(&original, predicate).unwrap();

        assert!(predicate(&result.minimized));
        assert_eq!(result.minimized.commands.len(), 5);
        assert_eq!(result.minimized.commands[0].opcode, TraceOpcode::CreateIntent);
        assert_eq!(result.minimized.commands[3].opcode, TraceOpcode::CommitLocal);
        assert_eq!(
            result.minimized.commands[4].opcode,
            TraceOpcode::MutateEvent
        );
        assert!(result.minimized.canonical_bytes().is_ok());
        assert_ne!(result.original_digest, result.minimized_digest);
        assert!(result.attempts > 0);
    }

    #[test]
    fn shrinker_rejects_a_nonfailing_input() {
        let trace = TraceGeneratorV1::generate(0x5eed_2026, 0);
        assert_eq!(
            shrink_trace_v1(&trace, |_| false),
            Err(TraceShrinkError::PredicateDoesNotHold)
        );
    }

    #[test]
    fn shrinker_does_not_edit_profile_version_or_scenario() {
        let original = (0..4096)
            .map(|index| TraceGeneratorV1::generate(0x5eed_2026, index))
            .find(|trace| trace.scenario == 7)
            .expect("scenario seven appears in fixed corpus);
        let scenario = original.scenario;
        let seed = original.seed;
        let index = original.trace_index;
        let result = shrink_trace_v1(&original, |trace| {
            trace.commands.iter().any(|command| {
                command.fault == TraceFault::ForkedCheckpoint
                    || command.opcode == TraceOpcode::ForkCheckpoint
            })
        })
        .unwrap();
        assert_eq!(result.minimized.scenario, scenario);
        assert_eq!(result.minimized.seed, seed);
        assert_eq!(result.minimized.trace_index, index);
    }
}
