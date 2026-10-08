        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

/// Validate fund allocation update (status transitions)
fn validate_update_fund_allocation(
    action: Update,
    alloc: FundAllocation,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: FundAllocation = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original fund allocation not found".into()
        )))?;

    if action.author != *original_record.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the fund allocation creator can update the allocation".into(),
        ));
    }

    match check_update_fund_allocation(&original, &alloc) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

#[cfg(test)]