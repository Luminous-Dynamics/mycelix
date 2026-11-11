//! Deterministic delta-debugging for frozen Saga Trace V1 failures.
//!
//! The shrinker preserves a caller-supplied failure signature. It never changes
//! opcodes, owners, fault codes, trace metadata or the V1 encoding profile.

use crate::trace::{TraceError, TraceV1};

const MAX_SHRINK_ATTEMPTS: u32 = 100_000;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TraceShrinkResultV1 {
    pub trace: TraceV1,
    pub original_digest: String,
    pub minimized_digest: String,
    pub attempts: u32,
    pub removed_commands: u8,
    pub changed_numeric_fields: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TraceShrinkError {
    InvalidInput,
    TargetFailureNotPresent,
    AttemptLimitExceeded,
}

impl std::fmt::Display for TraceShrinkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::InvalidInput => "input trace is not canonical Trace V1",
            Self::TargetFailureNotPresent => "target failure signature is absent from input",
            Self::AttemptLimitExceeded => "trace shrink attempt limit exceeded",
        })
    }
}

impl std::error::Error for TraceShrinkError {}

impl From<TraceError> for TraceShrinkError {
    fn from(_: TraceError) -> Self {
        Self::InvalidInput
    }
}

/// Minimize a trace while preserving exactly target_failure.
///
/// The supplied callback owns failure classification. The shrinker does not
/// decide whether a trace is safe; it only preserves the caller's signature.
pub fn minimize_trace_v1<F>(
    input: &TraceV1,
    target_failure: &str,
    failure_signature: F,
) -> Result<TraceShrinkResultV1, TraceShrinkError>
where
    F: Fn(&TraceV1) -> Option<String>,
{
    let original_digest = input.canonical_digest()?;
    if failure_signature(input).as_deref() != Some(target_failure) {
        return Err(TraceShrinkError::TargetFailureNotPresent);
    }

    let mut current = input.clone();
    let mut attempts = 0_u32;
    let mut removed_commands = 0_u8;
    let mut changed_numeric_fields = 0_u16;

    // Deterministic ddmin over contiguous command chunks.
    let mut granularity = 2_usize;
    loop {
        let length = current.commands.len();
        if length <= 1 {
            break;
        }
        let chunk_size = length.div_ceil(granularity);
        let mut reduced = false;

        for start in (0..length).step_by(chunk_size) {
            let end = (start + chunk_size).min(length);
            if end - start == length {
                continue;
            }
            let mut candidate = current.clone();
            candidate.commands.drain(start..end);
            attempts = next_attempt(attempts)?;
            if preserves(&candidate, target_failure, &failure_signature) {
                removed_commands = removed_commands.saturating_add(
                    u8::try_from(end - start).unwrap_or(u8::MAX),
                );
                current = candidate;
                granularity = granularity.saturating_sub(1).max(2);
                reduced = true;
                break;
            }
        }

        if reduced {
            continue;
        }
        if granularity >= length {
            break;
        }
        granularity = granularity.saturating_mul(2).min(length);
    }

    // Lower numeric metadata, subjects and amounts toward zero. Opcode, owner,
    // fault identity and trace metadata remain immutable.
    loop {
        let mut changed_this_pass = false;

        if try_numeric_reduction(
            &mut current,
            NumericField::OperationNonce,
            0,
            current.operation_nonce,
            target_failure,
            &failure_signature,
            &mut attempts,
        )? {
            changed_numeric_fields = changed_numeric_fields.saturating_add(1);
            changed_this_pass = true;
        }

        for command_index in 0..current.commands.len() {
            let subject = current.commands[command_index].subject;
            if try_numeric_reduction(
                &mut current,
                NumericField::Subject,
                command_index,
                subject,
                target_failure,
                &failure_signature,
                &mut attempts,
            )? {
                changed_numeric_fields = changed_numeric_fields.saturating_add(1);
                changed_this_pass = true;
            }

            let value = current.commands[command_index].value;
            if try_numeric_reduction(
                &mut current,
                NumericField::Value,
                command_index,
                value,
                target_failure,
                &failure_signature,
                &mut attempts,
            )? {
                changed_numeric_fields = changed_numeric_fields.saturating_add(1);
                changed_this_pass = true;
            }
        }

        if !changed_this_pass {
            break;
        }
    }

    let minimized_digest = current.canonical_digest()?;
    Ok(TraceShrinkResultV1 {
        trace: current,
        original_digest,
        minimized_digest,
        attempts,
        removed_commands,
        changed_numeric_fields,
    })
}

#[derive(Clone, Copy)]
enum NumericField {
    OperationNonce,
    Subject,
    Value,
}

fn preserves<F>(trace: &TraceV1, target: &str, signature: &F) -> bool
where
    F: Fn(&TraceV1) -> Option<String>,
{
    trace.canonical_bytes().is_ok()
        && signature(trace).as_deref() == Some(target)
}

fn next_attempt(attempts: u32) -> Result<u32, TraceShrinkError> {
    let next = attempts.saturating_add(1);
    if next > MAX_SHRINK_ATTEMPTS {
        Err(TraceShrinkError::AttemptLimitExceeded)
    } else {
        Ok(next)
    }
}

fn try_numeric_reduction<F>(
    trace: &mut TraceV1,
    field: NumericField,
    command_index: usize,
    original: u64,
    target: &str,
    signature: &F,
    attempts: &mut u32,
) -> Result<bool, TraceShrinkError>
where
    F: Fn(&TraceV1) -> Option<String>,
{
    let mut candidates = [0_u64, 1, original / 2, original.saturating_sub(1)];
    candidates.sort_unstable();
    let mut previous = None;

    for candidate_value in candidates {
        if candidate_value >= original || previous == Some(candidate_value) {
            previous = Some(candidate_value);
            continue;
        }
        previous = Some(candidate_value);
        let mut candidate = trace.clone();
        match field {
            NumericField::OperationNonce => candidate.operation_nonce = candidate_value,
            NumericField::Subject => candidate.commands[command_index].subject = candidate_value,
            NumericField::Value => candidate.commands[command_index].value = candidate_value,
        }
        *attempts = next_attempt(*attempts)?;
        if preserves(&candidate, target, signature) {
            *trace = candidate;
            return Ok(true);
        }
    }
    Ok(false)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::{TraceFault, TraceGeneratorV1, TraceOpcode, TraceOwner};

    fn failing_trace() -> TraceV1 {
        let mut trace = TraceGeneratorV1::generate(0x0123_4567_89ab_cdef, 0);
        trace.commands[0].owner = TraceOwner::Adversary;
        trace
    }

    fn owner_failure(trace: &TraceV1) -> Option<String> {
        trace.commands.first().and_then(|command| {
            (command.opcode == TraceOpcode::CreateIntent
                && command.owner == TraceOwner::Adversary)
                .then(|| "owner-mismatch:command-0".to_owned())
        })
    }

    #[test]
    fn shrinker_reduces_to_minimal_first_owner_failure() {
        let input = failing_trace();
        let seed = input.seed;
        let trace_index = input.trace_index;
        let scenario = input.scenario;
        let original_command = input.commands[0];

        let result = minimize_trace_v1(
            &input,
            "owner-mismatch:command-0",
            owner_failure,
        )
        .unwrap();

        assert_eq!(result.trace.commands.len(), 1);
        assert_eq!(result.trace.seed, seed);
        assert_eq!(result.trace.trace_index, trace_index);
        assert_eq!(result.trace.scenario, scenario);
        assert_eq!(result.trace.operation_nonce, 0);
        assert_eq!(result.trace.commands[0].opcode, original_command.opcode);
        assert_eq!(result.trace.commands[0].owner, TraceOwner::Adversary);
        assert_eq!(result.trace.commands[0].fault, original_command.fault);
        assert_eq!(result.trace.commands[0].subject, 0);
        assert_eq!(result.trace.commands[0].value, 0);
        assert_eq!(owner_failure(&result.trace).as_deref(), Some("owner-mismatch:command-0"));
        assert_eq!(result.minimized_digest, result.trace.canonical_digest().unwrap());
        assert!(result.removed_commands > 0);
        assert!(result.changed_numeric_fields > 0);
        assert!(result.attempts > 0);
    }

    #[test]
    fn shrinker_rejects_missing_failure_signature() {
        let input = TraceGeneratorV1::generate(7, 9);
        assert_eq!(
            minimize_trace_v1(&input, "not-present", owner_failure),
            Err(TraceShrinkError::TargetFailureNotPresent)
        );
    }

    #[test]
    fn shrinker_does_not_change_fault_or_owner_to_hide_failure() {
        let mut input = failing_trace();
        input.commands[0].fault = TraceFault::PayloadMutation;
        let original_fault = input.commands[0].fault;
        let result = minimize_trace_v1(
            &input,
            "owner-mismatch:command-0",
            owner_failure,
        )
        .unwrap();
        assert_eq!(result.trace.commands[0].owner, TraceOwner::Adversary);
        assert_eq!(result.trace.commands[0].fault, original_fault);
    }
}
