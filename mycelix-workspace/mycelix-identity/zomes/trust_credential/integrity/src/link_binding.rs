// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! DHT-level binding rules for trust-credential indexes.
//!
//! Coordinator-created links are discovery hints, not authority by themselves.
//! Every index link must therefore prove that its base anchor is derived from
//! the valid creation record it targets and that the link author is the same
//! agent who authored that target record.

use super::*;

/// Validate one trust-cluster index link.
///
/// All supported indexes point at creation actions. Updates/deletes are resolved
/// from that canonical root through CRUD metadata rather than being indexed as
/// independent credential roots.
pub(crate) fn validate_create_link_binding(
    action: CreateLink,
    base_address: AnyLinkableHash,
    target_address: AnyLinkableHash,
    link_type: LinkTypes,
) -> ExternResult<ValidateCallbackResult> {
    let target_action = match ActionHash::try_from(target_address) {
        Ok(hash) => hash,
        Err(_) => {
            return Ok(ValidateCallbackResult::Invalid(
                "Trust index links must target an action hash".into(),
            ));
        }
    };

    let target_record = must_get_valid_record(target_action)?;

    if !matches!(target_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust index links must target canonical creation actions".into(),
        ));
    }

    if *target_record.action().author() != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust index link author must match target record author".into(),
        ));
    }

    let expected_anchor = match link_type {
        LinkTypes::SubjectToCredential => {
            let cred = match target_record.entry().to_app_option::<TrustCredential>() {
                Ok(Some(cred)) => cred,
                _ => return invalid_target("SubjectToCredential", "TrustCredential"),
            };
            index_anchor(&format!("subject:{}", cred.subject_did))
        }
        LinkTypes::IssuerToCredential => {
            let cred = match target_record.entry().to_app_option::<TrustCredential>() {
                Ok(Some(cred)) => cred,
                _ => return invalid_target("IssuerToCredential", "TrustCredential"),
            };
            index_anchor(&format!("issuer:{}", cred.issuer_did))
        }
        LinkTypes::TierToCredential => {
            let cred = match target_record.entry().to_app_option::<TrustCredential>() {
                Ok(Some(cred)) => cred,
                _ => return invalid_target("TierToCredential", "TrustCredential"),
            };
            index_anchor(&format!("tier:{:?}", cred.trust_tier))
        }
        LinkTypes::SubjectToRequest => {
            let request = match target_record.entry().to_app_option::<AttestationRequest>() {
                Ok(Some(request)) => request,
                _ => return invalid_target("SubjectToRequest", "AttestationRequest"),
            };
            index_anchor(&format!("requests:{}", request.subject_did))
        }
        LinkTypes::CredentialToPresentation => {
            let presentation = match target_record.entry().to_app_option::<TrustPresentation>() {
                Ok(Some(presentation)) => presentation,
                _ => return invalid_target("CredentialToPresentation", "TrustPresentation"),
            };
            index_anchor(&format!("credential:{}", presentation.credential_id))
        }
    };

    let expected_base: AnyLinkableHash = expected_anchor.into();
    if base_address != expected_base {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust index link base does not match the target record".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn invalid_target(
    link_name: &str,
    expected_entry: &str,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Invalid(format!(
        "{} link target must decode as {}",
        link_name, expected_entry
    )))
}

/// Reproduce the coordinator's deterministic anchor derivation exactly.
fn index_anchor(anchor: &str) -> EntryHash {
    let digest = holo_hash::blake2b_256(anchor.as_bytes());
    EntryHash::from_raw_32(digest.to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn anchor_derivation_is_domain_separated() {
        let subject = index_anchor("subject:did:mycelix:alice");
        let issuer = index_anchor("issuer:did:mycelix:alice");
        let requests = index_anchor("requests:did:mycelix:alice");

        assert_ne!(subject, issuer);
        assert_ne!(subject, requests);
        assert_ne!(issuer, requests);
    }

    #[test]
    fn tier_anchor_depends_on_exact_tier_name() {
        assert_ne!(index_anchor("tier:Standard"), index_anchor("tier:Guardian"));
    }

    #[test]
    fn anchor_derivation_is_deterministic() {
        let a = index_anchor("credential:cred-123");
        let b = index_anchor("credential:cred-123");
        assert_eq!(a, b);
    }
}
