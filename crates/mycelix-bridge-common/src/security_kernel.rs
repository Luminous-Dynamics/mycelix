        ));
    }
    if !c.actions.contains(&request.action) {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::ActionNotGranted,
        ));
    }
    if c.policy_version != request.policy_version {
        return Err(AuthorizationDecision::Deny(
            AuthorizationDenial::PolicyVersionMismatch,
        ));
    }

    let permit_lifetime_us = now_us.saturating_add(MAX_AUTHORIZATION_PERMIT_LIFETIME_US);
    let valid_until_us = c
        .expires_at_us
        .min(verified.verification_valid_until_us)
        .min(permit_lifetime_us);

    Ok(AuthorizationPermit {
        request: request.clone(),
        issued_at_us: now_us,
        valid_until_us,
        capability_binding: c.binding_digest(),
        authority_binding: verified.authority_binding,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn capability() -> Capability {
        Capability::new(
            "did:mycelix:alice",
            "did:mycelix:issuer",
            "resource:ledger",
            vec![CapabilityAction::Read],
            100,
            200,
            7,