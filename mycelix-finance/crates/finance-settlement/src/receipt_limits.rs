use crate::limits::{
    SettlementVerificationBudgetError, SettlementVerificationBudgetV1,
    SettlementVerificationResource, SettlementVerificationUsage,
    assess_settlement_verification_budget_v1,
};
use crate::{
    FinalityProfile, SettlementEvaluationContext, SettlementObservation, SettlementSubject,
};

/// Account the strongest public FIN-ECO-002 v1 qualification path without
/// hashing or allocating canonical byte buffers.
///
/// `assess_settlement_verification_budget_v1` bounds the inner
/// `qualify_settlement(...)` implementation. The receipt path subsequently
/// re-hashes the selected current observations and reconstructs the evidence
/// frontier. Rather than duplicate the private canonical-length model, this
/// wrapper conservatively charges one additional full physical canonical-work
/// envelope. That envelope strictly dominates the receipt-only surcharge:
///
/// - at most one additional hash of each selected observation body;
/// - therefore at most one additional hash of each evidence body reachable
///   from those selected observations;
/// - one additional frontier hash.
///
/// The full charged envelope also contains profile/context bytes that the
/// receipt path does not re-hash, so the resulting bound intentionally errs on
/// the conservative side.
pub fn assess_settlement_receipt_verification_budget_v1(
    subject: &SettlementSubject,
    profile: &FinalityProfile,
    observations: &[SettlementObservation],
    context: &SettlementEvaluationContext,
    budget: SettlementVerificationBudgetV1,
) -> Result<SettlementVerificationUsage, SettlementVerificationBudgetError> {
    // The core assessor must not reject on its narrower hash-work ceiling before
    // the receipt surcharge has been added. Every other resource ceiling remains
    // exactly the caller's requested budget.
    let mut core_budget = budget;
    core_budget.max_total_hash_input_bytes = u64::MAX;

    let mut usage = assess_settlement_verification_budget_v1(
        subject,
        profile,
        observations,
        context,
        core_budget,
    )?;

    let receipt_hash_upper_bound = receipt_hash_input_upper_bound(
        usage.total_hash_input_bytes_upper_bound,
        usage.total_charged_canonical_bytes,
    )?;

    if receipt_hash_upper_bound > budget.max_total_hash_input_bytes {
        return Err(SettlementVerificationBudgetError::LimitExceeded {
            resource: SettlementVerificationResource::TotalHashInputBytes,
            limit: budget.max_total_hash_input_bytes,
            actual: receipt_hash_upper_bound,
        });
    }

    usage.total_hash_input_bytes_upper_bound = receipt_hash_upper_bound;
    Ok(usage)
}

fn receipt_hash_input_upper_bound(
    core_hash_input_bytes_upper_bound: u64,
    charged_canonical_bytes: u64,
) -> Result<u64, SettlementVerificationBudgetError> {
    core_hash_input_bytes_upper_bound
        .checked_add(charged_canonical_bytes)
        .ok_or(SettlementVerificationBudgetError::AccountingOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn receipt_hash_surcharge_is_one_full_charged_envelope() {
        assert_eq!(
            receipt_hash_input_upper_bound(1_000, 400),
            Ok(1_400)
        );
    }

    #[test]
    fn receipt_hash_surcharge_overflow_fails_closed() {
        assert_eq!(
            receipt_hash_input_upper_bound(u64::MAX, 1),
            Err(SettlementVerificationBudgetError::AccountingOverflow)
        );
    }
}
