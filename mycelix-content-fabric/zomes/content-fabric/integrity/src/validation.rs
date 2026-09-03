use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use hdi::prelude::*;

use crate::{
    canonical::{
        algorithm_anchor_v1, all_providers_anchor_v1, digest_anchor_v1, is_valid_anchor_v1,
        provider_binding_bytes_v1,
    },
    types::*,
};

fn invalid(message: impl Into<String>) -> ValidateCallbackResult {
    ValidateCallbackResult::Invalid(message.into())
}

fn guest_error(message: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(message.into()))
}

fn validate_schema(schema_version: u16) -> Result<(), String> {
    if schema_version == SCHEMA_VERSION_V1 {
        Ok(())
    } else {
        Err("unsupported Content Fabric schema version".into())
    }
}

fn validate_domain_value(value: &str) -> Result<(), String> {
    let bytes = value.as_bytes();
    if bytes.is_empty() || bytes.len() > 64 {
        return Err("failure-domain value must be 1-64 bytes".into());
    }
    if !bytes.iter().all(|byte| {
        byte.is_ascii_lowercase()
            || byte.is_ascii_digit()
            || matches!(*byte, b'-' | b'_' | b'.' | b':')
    }) {
        return Err("failure-domain value must use canonical lowercase ASCII tokens".into());
    }
    Ok(())
}

fn validate_advertisement_shape(ad: &ProviderAdvertisementV1) -> Result<(), String> {
    validate_schema(ad.schema_version)?;
    if ad.protocol != IROH_ALPN_V1 {
        return Err("provider advertisement must use the Content Fabric v1 ALPN".into());
    }
    if ad.max_blob_size_bytes == 0 || ad.max_blob_size_bytes > MAX_BLOB_SIZE_V1 {
        return Err("invalid provider maximum blob size".into());
    }
    if !(60..=MAX_AD_TTL_SECONDS_V1).contains(&ad.ttl_seconds) {
        return Err("provider advertisement TTL must be between 60s and 24h".into());
    }
    if ad.supported_algorithms.is_empty() || ad.supported_algorithms.len() > 2 {
        return Err("provider must advertise one or two supported digest algorithms".into());
    }
    if ad
        .supported_algorithms
        .windows(2)
        .any(|window| window[0] >= window[1])
    {
        return Err("supported algorithms must be unique and canonically sorted".into());
    }
    if ad.failure_domains.len() > MAX_FAILURE_DOMAINS_V1 {
        return Err("too many failure-domain claims".into());
    }
    for claim in &ad.failure_domains {
        validate_domain_value(&claim.value)?;
    }
    if ad
        .failure_domains
        .windows(2)
        .any(|window| window[0].kind >= window[1].kind)
    {
        return Err("failure-domain dimensions must be unique and canonically sorted".into());
    }

    let verifying_key = VerifyingKey::from_bytes(&ad.iroh_endpoint_id)
        .map_err(|_| "invalid Iroh endpoint public key".to_string())?;
    let signature = Signature::from_bytes(&ad.endpoint_binding_signature);
    verifying_key
        .verify(&provider_binding_bytes_v1(ad), &signature)
        .map_err(|_| "Iroh endpoint binding signature is invalid".to_string())?;
    Ok(())
}

fn load_advertisement(hash: &ActionHash) -> ExternResult<ProviderAdvertisementV1> {
    let record = must_get_valid_record(hash.clone())?;
    record
        .entry()
        .to_app_option::<ProviderAdvertisementV1>()
        .map_err(|error| guest_error(format!("failed to decode provider advertisement: {error}")))?
        .ok_or_else(|| guest_error("referenced action is not a provider advertisement"))
}

fn load_availability(hash: &ActionHash) -> ExternResult<ContentAvailabilityClaimV1> {
    let record = must_get_valid_record(hash.clone())?;
    record
        .entry()
        .to_app_option::<ContentAvailabilityClaimV1>()
        .map_err(|error| guest_error(format!("failed to decode availability claim: {error}")))?
        .ok_or_else(|| guest_error("referenced action is not an availability claim"))
}

fn load_withdrawal(hash: &ActionHash) -> ExternResult<ProviderWithdrawalV1> {
    let record = must_get_valid_record(hash.clone())?;
    record
        .entry()
        .to_app_option::<ProviderWithdrawalV1>()
        .map_err(|error| guest_error(format!("failed to decode provider withdrawal: {error}")))?
        .ok_or_else(|| guest_error("referenced action is not a provider withdrawal"))
}

fn load_observation(hash: &ActionHash) -> ExternResult<ReplicaObservationV1> {
    let record = must_get_valid_record(hash.clone())?;
    record
        .entry()
        .to_app_option::<ReplicaObservationV1>()
        .map_err(|error| guest_error(format!("failed to decode replica observation: {error}")))?
        .ok_or_else(|| guest_error("referenced action is not a replica observation"))
}

fn validate_availability_shape(
    claim: &ContentAvailabilityClaimV1,
    ad: &ProviderAdvertisementV1,
) -> Result<(), String> {
    validate_schema(claim.schema_version)?;
    if claim.provider != ad.provider {
        return Err("availability provider does not match advertisement provider".into());
    }
    if !ad.supported_algorithms.contains(&claim.digest.algorithm) {
        return Err("availability digest algorithm is not advertised by provider".into());
    }
    if claim.size_bytes == 0 || claim.size_bytes > ad.max_blob_size_bytes {
        return Err("availability size exceeds provider advertisement".into());
    }
    if claim.ttl_seconds < 15 || claim.ttl_seconds > ad.ttl_seconds {
        return Err("availability TTL must be within the parent advertisement TTL".into());
    }
    Ok(())
}

fn validate_observation_shape(
    observation: &ReplicaObservationV1,
    ad: &ProviderAdvertisementV1,
) -> Result<(), String> {
    validate_schema(observation.schema_version)?;
    if observation.provider != ad.provider {
        return Err("observation provider does not match advertisement provider".into());
    }
    if !ad.supported_algorithms.contains(&observation.digest.algorithm) {
        return Err("observation digest algorithm is not supported by advertisement".into());
    }
    if observation.latency_ms.is_some_and(|value| value > 86_400_000) {
        return Err("observation latency exceeds v1 limit".into());
    }
    match &observation.outcome {
        ObservationOutcomeV1::VerifiedComplete { size_bytes } => {
            if *size_bytes == 0 || *size_bytes > ad.max_blob_size_bytes {
                return Err("verified observation size is invalid".into());
            }
        }
        ObservationOutcomeV1::DigestMismatch {
            observed_digest,
            size_bytes,
        } => {
            if observed_digest.algorithm != observation.digest.algorithm {
                return Err("mismatch observation must use the requested digest algorithm".into());
            }
            if observed_digest == &observation.digest {
                return Err("digest-mismatch observation must actually differ".into());
            }
            if *size_bytes == 0 || *size_bytes > ad.max_blob_size_bytes {
                return Err("mismatch observation size is invalid".into());
            }
        }
        ObservationOutcomeV1::UnavailableOrHidden
        | ObservationOutcomeV1::Busy
        | ObservationOutcomeV1::TransferFailed
        | ObservationOutcomeV1::ProviderReportedIntegrityFailure => {}
    }
    Ok(())
}

fn expected_anchor_base(anchor: &str) -> ExternResult<AnyLinkableHash> {
    Ok(hash_entry(&EntryTypes::Anchor(Anchor(anchor.to_string())))?.into())
}

fn target_action(target: &AnyLinkableHash) -> Result<ActionHash, ValidateCallbackResult> {
    target
        .clone()
        .into_action_hash()
        .ok_or_else(|| invalid("Content Fabric index links must target action hashes"))
}

fn validate_create_link(
    base: AnyLinkableHash,
    target: AnyLinkableHash,
    tag: LinkTag,
    link_type: LinkTypes,
    author: AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    if !tag.0.is_empty() {
        return Ok(invalid("Content Fabric v1 index links must use an empty tag"));
    }
    let target = match target_action(&target) {
        Ok(target) => target,
        Err(result) => return Ok(result),
    };

    let valid = match link_type {
        LinkTypes::AllProviders => {
            let ad = load_advertisement(&target)?;
            author == ad.provider && base == expected_anchor_base(all_providers_anchor_v1())?
        }
        LinkTypes::ProviderAdsByAgent => {
            let ad = load_advertisement(&target)?;
            let expected: AnyLinkableHash = ad.provider.clone().into();
            author == ad.provider && base == expected
        }
        LinkTypes::ProvidersByAlgorithm => {
            let ad = load_advertisement(&target)?;
            if author != ad.provider {
                false
            } else {
                let mut matched = false;
                for algorithm in ad.supported_algorithms {
                    if base == expected_anchor_base(&algorithm_anchor_v1(algorithm))? {
                        matched = true;
                        break;
                    }
                }
                matched
            }
        }
        LinkTypes::AvailabilityByDigest => {
            let claim = load_availability(&target)?;
            author == claim.provider
                && base == expected_anchor_base(&digest_anchor_v1(&claim.digest))?
        }
        LinkTypes::AvailabilityByAdvertisement => {
            let claim = load_availability(&target)?;
            let expected: AnyLinkableHash = claim.advertisement.clone().into();
            author == claim.provider && base == expected
        }
        LinkTypes::ObservationsByDigest => {
            let observation = load_observation(&target)?;
            author == observation.observer
                && base == expected_anchor_base(&digest_anchor_v1(&observation.digest))?
        }
        LinkTypes::ObservationsByAdvertisement => {
            let observation = load_observation(&target)?;
            let expected: AnyLinkableHash = observation.advertisement.clone().into();
            author == observation.observer && base == expected
        }
        LinkTypes::WithdrawalsByAdvertisement => {
            let withdrawal = load_withdrawal(&target)?;
            let expected: AnyLinkableHash = withdrawal.advertisement.clone().into();
            author == withdrawal.provider && base == expected
        }
    };
    Ok(if valid {
        ValidateCallbackResult::Valid
    } else {
        invalid("Content Fabric index link does not match its target record")
    })
}

pub fn validate_content_fabric_op(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::ProviderAdvertisementV1(ad) => {
                    if ad.provider != action.author {
                        return Ok(invalid("provider advertisement author must equal provider"));
                    }
                    Ok(match validate_advertisement_shape(&ad) {
                        Ok(()) => ValidateCallbackResult::Valid,
                        Err(message) => invalid(message),
                    })
                }
                EntryTypes::ContentAvailabilityClaimV1(claim) => {
                    if claim.provider != action.author {
                        return Ok(invalid("availability author must equal provider"));
                    }
                    let ad = load_advertisement(&claim.advertisement)?;
                    Ok(match validate_availability_shape(&claim, &ad) {
                        Ok(()) => ValidateCallbackResult::Valid,
                        Err(message) => invalid(message),
                    })
                }
                EntryTypes::ProviderWithdrawalV1(withdrawal) => {
                    if withdrawal.provider != action.author {
                        return Ok(invalid("withdrawal author must equal provider"));
                    }
                    if let Err(message) = validate_schema(withdrawal.schema_version) {
                        return Ok(invalid(message));
                    }
                    if withdrawal.note.as_ref().is_some_and(|note| note.len() > 256) {
                        return Ok(invalid("withdrawal note exceeds 256 bytes"));
                    }
                    let ad = load_advertisement(&withdrawal.advertisement)?;
                    if ad.provider != withdrawal.provider {
                        return Ok(invalid("withdrawal provider does not own advertisement"));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                EntryTypes::ReplicaObservationV1(observation) => {
                    if observation.observer != action.author {
                        return Ok(invalid("observation author must equal observer"));
                    }
                    let ad = load_advertisement(&observation.advertisement)?;
                    Ok(match validate_observation_shape(&observation, &ad) {
                        Ok(()) => ValidateCallbackResult::Valid,
                        Err(message) => invalid(message),
                    })
                }
                EntryTypes::Anchor(anchor) => Ok(if is_valid_anchor_v1(&anchor.0) {
                    ValidateCallbackResult::Valid
                } else {
                    invalid("invalid Content Fabric anchor")
                }),
            },
            OpEntry::UpdateEntry { .. } => Ok(invalid(
                "Content Fabric evidence entries are append-only and cannot be updated",
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            tag,
            link_type,
            action,
        } => validate_create_link(
            base_address,
            target_address,
            tag,
            link_type,
            action.author,
        ),
        FlatOp::RegisterDeleteLink { .. } => Ok(invalid(
            "Content Fabric index links are append-only and cannot be deleted",
        )),
        FlatOp::RegisterUpdate(_) => Ok(invalid(
            "Content Fabric evidence entries are append-only and cannot be updated",
        )),
        FlatOp::RegisterDelete(_) => Ok(invalid(
            "Content Fabric evidence entries are append-only and cannot be deleted",
        )),
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
    }
}
