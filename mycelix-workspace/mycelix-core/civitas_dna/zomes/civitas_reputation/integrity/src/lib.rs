// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Civitas Reputation Integrity Zome
//!
//! Defines entry types and validation rules for the Civitas reputation system
//! on Holochain 0.6. This zome tracks agent reputation scores based on their
//! causal contributions to the federated learning network.
//!
//! # Security redesign (2026-07-27, MASTER_ROADMAP.md P0-#1)
//!
//! Previously `update_causal_reputation` accepted a raw client-supplied
//! `agent`+`score` with zero authentication of the caller and zero
//! verification the score derived from any real contribution -- any agent
//! could set any other agent's reputation to any value in [0,1]. Worse, this
//! zome's own `validate()` only ever handled `Op::StoreEntry`: updates and
//! deletes were completely unvalidated, so even bypassing the coordinator
//! entirely, anyone could forge an update to any agent's reputation with no
//! author binding at all. Since `get_trust_threshold` uses this score to
//! gate scrutiny (higher reputation = less scrutiny), this was a real
//! privilege-escalation path, not just a data-integrity gap.
//!
//! Fixed by requiring every `CivitasReputationScore` to reference the exact
//! `CausalContributionRecord` (in the sibling `causal_contribution` zome)
//! that justifies it, via `justifying_record`, and independently
//! recomputing the expected reputation transition from that record instead
//! of trusting the coordinator's math. `must_get_valid_record` only ever
//! returns a record that has ALREADY passed its own zome's validation, so
//! referencing a record this way transitively inherits
//! `causal_contribution`'s own author-binding fix (only a receipt's
//! attesters may create a contribution record referencing it -- see that
//! zome's coordinator).
//!
//! Residual limitation, disclosed rather than silently assumed solved: this
//! does not by itself guarantee a given `CausalContributionRecord` is
//! applied to reputation *at most once* -- true replay-uniqueness
//! enforcement needs a DHT-wide existence check, which isn't reliably
//! deterministic inside a validation callback (the same class of residual
//! limitation already disclosed for `mycelix-pulse`'s `TrustScore`
//! transitive-trust component). The `RegisterUpdate` check here does reject
//! reusing the *exact same* record consecutively, and the coordinator adds a
//! best-effort link-based guard against reapplying any record at all; both
//! are meaningful defenses, but neither is airtight against a byzantine
//! coordinator implementation choosing to route around them.

use hdi::prelude::*;

/// Mirrors `causal_contribution_integrity::CausalContributionRecord` field-
/// for-field (agent, round_id, contribution_score, proof_hash, timestamp, in
/// that exact order). Deliberately duplicated rather than imported: each
/// Holochain zome's `#[hdk_entry_types]`/`#[hdk_link_types]` macros generate
/// crate-global symbols (`__num_entry_types` etc.) that must be unique per
/// compiled WASM module. Depending on `causal_contribution_integrity` as a
/// regular Cargo dependency pulls its own `#[hdk_entry_types]`-decorated
/// `EntryTypes` enum into this crate's compiled unit too, producing a
/// duplicate-symbol link error (confirmed while building this fix) -- this
/// isn't a workaround, it's the standard Holochain pattern for one zome
/// reading another zome's entry shape. Keep this in sync with the real
/// struct by hand if it ever changes.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CausalContributionRecordMirror {
    pub agent: AgentPubKey,
    pub round_id: u64,
    pub contribution_score: f64,
    pub proof_hash: EntryHash,
    pub timestamp: Timestamp,
}

/// The causal reputation score for an agent in the Civitas system.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CivitasReputationScore {
    /// The agent whose reputation is being tracked.
    pub agent: AgentPubKey,
    /// The agent's causal reputation score, in the range [0, 1].
    pub reputation: f64,
    /// The number of rounds the agent has participated in.
    pub rounds_participated: u64,
    /// The last time the score was updated.
    pub last_updated: Timestamp,
    /// The `CausalContributionRecord` (in the sibling `causal_contribution`
    /// zome) whose contribution this exact value was derived from. Required
    /// so validation can independently recompute the expected transition --
    /// see module doc.
    pub justifying_record: ActionHash,
}

/// Entry types for the Civitas reputation system
#[hdk_entry_types]
#[unit_enum(EntryTypesUnit)]
pub enum EntryTypes {
    CivitasReputationScore(CivitasReputationScore),
}

/// Link types for the Civitas reputation system
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from an agent to their reputation score entry
    AgentToReputationScore,
    /// Links from a `CausalContributionRecord`'s ActionHash to the
    /// `CivitasReputationScore` action it was applied to. Existence of any
    /// such link is the coordinator's best-effort replay guard against
    /// applying the same contribution record to reputation twice -- see
    /// module doc for why this isn't airtight.
    ContributionRecordApplied,
}

// === Shared transition math ===
//
// Used both here (to independently recompute and content-restrict updates)
// and by the coordinator (to perform the same transition when creating an
// entry) -- defined once so the two can never drift apart.

/// Sigmoid-normalize a raw contribution score to (0,1), then apply an
/// exponential-moving-average update against the previous reputation.
pub fn apply_contribution(
    prev_reputation: f64,
    prev_rounds: u64,
    contribution_score: f64,
) -> (f64, u64) {
    let normalized = 1.0 / (1.0 + (-contribution_score).exp());
    let alpha = 0.1;
    let reputation = (1.0 - alpha) * prev_reputation + alpha * normalized;
    (reputation, prev_rounds + 1)
}

/// Initial reputation state for an agent's first-ever score entry.
pub const INITIAL_REPUTATION: f64 = 0.5;

// === Validation Functions ===

/// Basic field-level checks shared by create and update, factored out so it
/// can be unit-tested without a live HDI (`must_get_valid_record` has no
/// test mock in this codebase).
fn validate_score_fields(score: &CivitasReputationScore) -> ValidateCallbackResult {
    if score.reputation.is_nan() {
        return ValidateCallbackResult::Invalid("Reputation score must not be NaN".to_string());
    }
    if !(0.0..=1.0).contains(&score.reputation) {
        return ValidateCallbackResult::Invalid(
            "Reputation score must be between 0.0 and 1.0".to_string(),
        );
    }
    ValidateCallbackResult::Valid
}

/// Fetches and deserializes the `CausalContributionRecord` a
/// `CivitasReputationScore` claims to be justified by. `must_get_valid_record`
/// only returns successfully once the referenced record has already passed
/// `causal_contribution`'s own validation -- this is the actual security
/// boundary this whole redesign rests on, not just a convenience fetch.
fn fetch_justifying_record(
    justifying_record: &ActionHash,
) -> ExternResult<Option<CausalContributionRecordMirror>> {
    let record = must_get_valid_record(justifying_record.clone())?;
    record
        .entry()
        .to_app_option::<CausalContributionRecordMirror>()
        .map_err(|e| wasm_error!(e))
}

fn validate_create_score(
    action: Create,
    score: CivitasReputationScore,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_score_fields(&score);
    if fields != ValidateCallbackResult::Valid {
        return Ok(fields);
    }

    let Some(contribution) = fetch_justifying_record(&score.justifying_record)? else {
        return Ok(ValidateCallbackResult::Invalid(
            "justifying_record does not resolve to a CausalContributionRecord".to_string(),
        ));
    };

    if contribution.agent != score.agent {
        return Ok(ValidateCallbackResult::Invalid(
            "justifying_record's agent does not match this score's agent".to_string(),
        ));
    }

    let (expected_reputation, expected_rounds) =
        apply_contribution(INITIAL_REPUTATION, 0, contribution.contribution_score);
    if (score.reputation - expected_reputation).abs() > f64::EPSILON
        || score.rounds_participated != expected_rounds
    {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "First reputation entry must equal apply_contribution(INITIAL_REPUTATION, 0, {}), \
             got reputation={} rounds={} expected reputation={} rounds={}",
            contribution.contribution_score,
            score.reputation,
            score.rounds_participated,
            expected_reputation,
            expected_rounds
        )));
    }

    // The action author does not have to equal the scored agent -- a
    // reputation entry is a third-party attestation about someone else's
    // contribution, same as the underlying CausalContributionRecord. What
    // matters is that the referenced record is real and already-validated,
    // checked above.
    let _ = action;

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_score(
    action: Update,
    score: CivitasReputationScore,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_score_fields(&score);
    if fields != ValidateCallbackResult::Valid {
        return Ok(fields);
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: CivitasReputationScore = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original CivitasReputationScore not found".to_string()
        )))?;

    if score.agent != original.agent {
        return Ok(ValidateCallbackResult::Invalid(
            "agent cannot change on update".to_string(),
        ));
    }

    if score.justifying_record == original.justifying_record {
        return Ok(ValidateCallbackResult::Invalid(
            "Update must reference a different justifying_record than the original \
             (reusing the same record to update reputation twice is rejected)"
                .to_string(),
        ));
    }

    let Some(contribution) = fetch_justifying_record(&score.justifying_record)? else {
        return Ok(ValidateCallbackResult::Invalid(
            "justifying_record does not resolve to a CausalContributionRecord".to_string(),
        ));
    };

    if contribution.agent != score.agent {
        return Ok(ValidateCallbackResult::Invalid(
            "justifying_record's agent does not match this score's agent".to_string(),
        ));
    }

    let (expected_reputation, expected_rounds) = apply_contribution(
        original.reputation,
        original.rounds_participated,
        contribution.contribution_score,
    );
    if (score.reputation - expected_reputation).abs() > f64::EPSILON
        || score.rounds_participated != expected_rounds
    {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Updated reputation must equal apply_contribution(original.reputation, \
             original.rounds_participated, {}), got reputation={} rounds={} expected \
             reputation={} rounds={}",
            contribution.contribution_score,
            score.reputation,
            score.rounds_participated,
            expected_reputation,
            expected_rounds
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn check_author_match(
    original_author: &AgentPubKey,
    action_author: &AgentPubKey,
    operation: &str,
) -> ValidateCallbackResult {
    if action_author != original_author {
        ValidateCallbackResult::Invalid(format!(
            "Only the original entry author can {} their entries",
            operation,
        ))
    } else {
        ValidateCallbackResult::Valid
    }
}

/// Main validation dispatcher
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CivitasReputationScore(score) => validate_create_score(action, score),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::CivitasReputationScore(score) => validate_update_score(action, score),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "update",
            ))
        }
        // No legitimate reason to delete a reputation record -- it should
        // only ever be superseded by an update. Rejecting outright removes
        // an attack surface that previously had zero validation at all.
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "CivitasReputationScore entries cannot be deleted".to_string(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_agent_pub_key(seed: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![seed; 36])
    }

    fn fake_action_hash(seed: u8) -> ActionHash {
        ActionHash::from_raw_36(vec![seed; 36])
    }

    fn fake_timestamp() -> Timestamp {
        Timestamp::from_micros(0)
    }

    fn is_valid(result: ValidateCallbackResult) -> bool {
        matches!(result, ValidateCallbackResult::Valid)
    }

    fn is_invalid(result: ValidateCallbackResult) -> bool {
        matches!(result, ValidateCallbackResult::Invalid(_))
    }

    fn valid_score() -> CivitasReputationScore {
        CivitasReputationScore {
            agent: fake_agent_pub_key(1),
            reputation: 0.5,
            rounds_participated: 0,
            last_updated: fake_timestamp(),
            justifying_record: fake_action_hash(9),
        }
    }

    // --- apply_contribution (the shared transition math) ---

    #[test]
    fn apply_contribution_from_initial_state_matches_sigmoid_ema() {
        let (reputation, rounds) = apply_contribution(INITIAL_REPUTATION, 0, 0.0);
        // sigmoid(0.0) = 0.5, EMA(0.5, 0.5, alpha=0.1) = 0.5
        assert!((reputation - 0.5).abs() < 1e-9);
        assert_eq!(rounds, 1);
    }

    #[test]
    fn apply_contribution_increments_rounds_each_time() {
        let (_, rounds1) = apply_contribution(0.5, 0, 1.0);
        let (_, rounds2) = apply_contribution(0.5, rounds1, 1.0);
        assert_eq!(rounds1, 1);
        assert_eq!(rounds2, 2);
    }

    #[test]
    fn apply_contribution_stays_in_unit_range_for_extreme_scores() {
        let (high, _) = apply_contribution(0.5, 0, 1_000_000.0);
        let (low, _) = apply_contribution(0.5, 0, -1_000_000.0);
        assert!((0.0..=1.0).contains(&high));
        assert!((0.0..=1.0).contains(&low));
    }

    // --- validate_score_fields ---

    #[test]
    fn score_fields_reject_out_of_range_reputation() {
        let mut score = valid_score();
        score.reputation = 1.5;
        assert!(is_invalid(validate_score_fields(&score)));
    }

    #[test]
    fn score_fields_reject_negative_reputation() {
        let mut score = valid_score();
        score.reputation = -0.1;
        assert!(is_invalid(validate_score_fields(&score)));
    }

    #[test]
    fn score_fields_accept_boundary_values() {
        let mut score = valid_score();
        score.reputation = 0.0;
        assert!(is_valid(validate_score_fields(&score)));
        score.reputation = 1.0;
        assert!(is_valid(validate_score_fields(&score)));
    }

    #[test]
    fn score_fields_reject_nan() {
        let mut score = valid_score();
        score.reputation = f64::NAN;
        assert!(is_invalid(validate_score_fields(&score)));
    }

    // --- check_author_match ---

    #[test]
    fn author_match_accepts_same_agent() {
        let agent = fake_agent_pub_key(1);
        assert!(is_valid(check_author_match(&agent, &agent, "update")));
    }

    #[test]
    fn author_match_rejects_different_agent() {
        let original = fake_agent_pub_key(1);
        let forged = fake_agent_pub_key(2);
        assert!(is_invalid(check_author_match(&original, &forged, "update")));
    }
}
