// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact Holochain Record provenance adapter for Identity V2 historical verifier observation.
//!
//! This unprovisioned coordinator owns the concrete #356 boundary:
//!
//! `#346 qualified action set -> exact ActionHash fetch -> exact Record/action/app-entry type
//!  -> committed entry bytes -> internally constructed #353 normalized observation`.
//!
//! It deliberately does not own runtime DNA identity (#382/#385), trusted activation (#337),
//! signature authenticity, policy currentness, or positive evidence.

#![forbid(unsafe_code)]

use did_registry_v2_integrity::{DidDeactivationV2, DidDocumentV2};
use hdk::prelude::holo_hash::{ActionHashB64, AgentPubKeyB64};
use hdk::prelude::*;
use mycelix_agent_activity_coverage_policy::QualifiedAgentActivityCoverageV2;
use mycelix_authority_scoped_kvector_verification_record_policy::{
    derive_authority_scoped_kvector_verification_record_signing_digest_v2,
    AuthorityScopedKVectorProofVerificationRecordBodyV2,
};
use mycelix_historical_generation_observer_policy::{
    assemble_normalized_historical_generation_observation_v2,
    FetchedDidDeactivationRecordV2, FetchedDidDocumentRecordV2,
    FetchedDidVerificationMethodV2, FetchedIdentitySecurityRecordV2,
    FetchedVerifierKeyGenerationRecordV2, QualifiedNormalizedHistoricalGenerationObservationV2,
};
use sha2::{Digest, Sha256};
use verifier_key_generation_integrity::VerifierKeyGenerationRecord;

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const EXACT_HISTORICAL_RECORD_OBSERVATION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:exact-historical-record-observation:v2\0";
const DID_PREFIX_V2: &str = "did:mycelix:";

/// The dependency order is security-significant. `hdk_dependent_entry_types` resolves
/// these dependency slots through the executing coordinator's `zome_info()` scope.
/// Future DNA provisioning must therefore declare these integrity dependencies in this
/// exact order rather than hard-coding raw zome/entry indices here.
#[hdk_dependent_entry_types]
enum IdentitySecurityEntryTypes {
    DidRegistryV2(did_registry_v2_integrity::EntryTypes),
    VerifierKeyGeneration(verifier_key_generation_integrity::EntryTypes),
}

#[derive(Debug)]
pub struct QualifiedExactHistoricalGenerationObservationV2 {
    normalized_observation: QualifiedNormalizedHistoricalGenerationObservationV2,
    record_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    exact_observation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    exact_record_count: usize,
}

impl QualifiedExactHistoricalGenerationObservationV2 {
    pub fn normalized_observation(&self) -> &QualifiedNormalizedHistoricalGenerationObservationV2 {
        &self.normalized_observation
    }

    pub fn record_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.record_signing_digest_sha256
    }

    pub fn exact_observation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.exact_observation_digest_sha256
    }

    pub fn exact_record_count(&self) -> usize {
        self.exact_record_count
    }
}

enum ExactIdentitySecurityRecordV2 {
    DidDocument {
        action_seq: u32,
        action_id: String,
        action_hash: ActionHash,
        previous_action_id: Option<String>,
        document: DidDocumentV2,
    },
    DidDeactivation {
        action_seq: u32,
        action_id: String,
        action_hash: ActionHash,
        deactivation: DidDeactivationV2,
    },
    VerifierKeyGeneration {
        action_seq: u32,
        action_id: String,
        action_hash: ActionHash,
        action_timestamp_micros: i64,
        did_document_action_id: String,
        previous_generation_action_id: Option<String>,
        generation: VerifierKeyGenerationRecord,
    },
}

impl ExactIdentitySecurityRecordV2 {
    fn action_seq(&self) -> u32 {
        match self {
            Self::DidDocument { action_seq, .. }
            | Self::DidDeactivation { action_seq, .. }
            | Self::VerifierKeyGeneration { action_seq, .. } => *action_seq,
        }
    }

    fn action_hash(&self) -> &ActionHash {
        match self {
            Self::DidDocument { action_hash, .. }
            | Self::DidDeactivation { action_hash, .. }
            | Self::VerifierKeyGeneration { action_hash, .. } => action_hash,
        }
    }
}

fn guest(message: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(message.into()))
}

fn parse_canonical_action_hash_v2(value: &str) -> ExternResult<ActionHash> {
    let encoded: ActionHashB64 = value
        .parse()
        .map_err(|error| guest(format!("invalid qualified ActionHash: {error}")))?;
    let action_hash: ActionHash = encoded.into();
    let canonical = ActionHashB64::from(action_hash.clone()).to_string();
    if canonical != value {
        return Err(guest("qualified ActionHash is not in canonical base64 form"));
    }
    Ok(action_hash)
}

fn verifier_agent_from_did_v2(verifier_did: &str) -> ExternResult<AgentPubKey> {
    let encoded = verifier_did
        .strip_prefix(DID_PREFIX_V2)
        .filter(|suffix| !suffix.is_empty())
        .ok_or_else(|| guest("verifier DID is not canonical did:mycelix:<AgentPubKey>"))?;
    let agent_b64: AgentPubKeyB64 = encoded
        .parse()
        .map_err(|error| guest(format!("verifier DID AgentPubKey is invalid: {error}")))?;
    let agent: AgentPubKey = agent_b64.into();
    if AgentPubKeyB64::from(agent.clone()).to_string() != encoded {
        return Err(guest("verifier DID AgentPubKey is not in canonical base64 form"));
    }
    Ok(agent)
}

fn exact_present_app_entry_v2<'a>(record: &'a Record) -> ExternResult<&'a Entry> {
    let entry = match record.entry() {
        RecordEntry::Present(entry) => entry,
        RecordEntry::Hidden => return Err(guest("security Record entry is hidden")),
        RecordEntry::NA => return Err(guest("security Record has no entry")),
        RecordEntry::NotStored => return Err(guest("security Record entry was not stored")),
    };
    if !matches!(entry, Entry::App(_)) {
        return Err(guest("security Record substituted a system entry for an app entry"));
    }
    Ok(entry)
}

fn fetch_exact_security_record_v2(
    qualified_action_seq: u32,
    qualified_action_id: &str,
    verifier_agent: &AgentPubKey,
) -> ExternResult<ExactIdentitySecurityRecordV2> {
    let requested_action_hash = parse_canonical_action_hash_v2(qualified_action_id)?;
    let record = get(requested_action_hash.clone(), GetOptions::default())?
        .ok_or_else(|| guest("qualified security action was not retrievable by exact ActionHash"))?;

    if record.action_address() != &requested_action_hash {
        return Err(guest("retrieved Record action hash differs from the requested ActionHash"));
    }

    let action = record.action();
    if action.action_seq() != qualified_action_seq {
        return Err(guest("retrieved Record action sequence differs from #346 qualification"));
    }
    if action.author() != verifier_agent {
        return Err(guest("retrieved security Record author differs from verifier DID agent"));
    }

    let entry = exact_present_app_entry_v2(&record)?;
    let committed_entry_hash = action
        .entry_hash()
        .ok_or_else(|| guest("qualified security action does not commit an entry hash"))?;
    let recomputed_entry_hash = hash_entry(entry.clone())?;
    if &recomputed_entry_hash != committed_entry_hash {
        return Err(guest("present app-entry bytes do not hash to the Action's committed EntryHash"));
    }

    let app_entry_def = match action.entry_type() {
        Some(EntryType::App(app_entry_def)) => app_entry_def,
        _ => return Err(guest("qualified security action is not an application entry action")),
    };
    if app_entry_def.visibility() != &EntryVisibility::Public {
        return Err(guest("Identity V2 security authority entries must be public"));
    }

    let typed = IdentitySecurityEntryTypes::deserialize_from_type(
        app_entry_def.zome_index(),
        app_entry_def.entry_index(),
        entry,
    )?
    .ok_or_else(|| guest("qualified security action has an unknown scoped application entry type"))?;

    let action_id = ActionHashB64::from(requested_action_hash.clone()).to_string();
    match (action, typed) {
        (
            Action::Create(_),
            IdentitySecurityEntryTypes::DidRegistryV2(
                did_registry_v2_integrity::EntryTypes::DidDocumentV2(document),
            ),
        ) => Ok(ExactIdentitySecurityRecordV2::DidDocument {
            action_seq: qualified_action_seq,
            action_id,
            action_hash: requested_action_hash,
            previous_action_id: None,
            document,
        }),
        (
            Action::Update(update),
            IdentitySecurityEntryTypes::DidRegistryV2(
                did_registry_v2_integrity::EntryTypes::DidDocumentV2(document),
            ),
        ) => Ok(ExactIdentitySecurityRecordV2::DidDocument {
            action_seq: qualified_action_seq,
            action_id,
            action_hash: requested_action_hash,
            previous_action_id: Some(update.original_action_address.to_string()),
            document,
        }),
        (
            Action::Create(_),
            IdentitySecurityEntryTypes::DidRegistryV2(
                did_registry_v2_integrity::EntryTypes::DidDeactivationV2(deactivation),
            ),
        ) => Ok(ExactIdentitySecurityRecordV2::DidDeactivation {
            action_seq: qualified_action_seq,
            action_id,
            action_hash: requested_action_hash,
            deactivation,
        }),
        (
            Action::Create(create),
            IdentitySecurityEntryTypes::VerifierKeyGeneration(
                verifier_key_generation_integrity::EntryTypes::VerifierKeyGeneration(generation),
            ),
        ) => {
            let did_document_action_id = generation.did_document_action.to_string();
            let previous_generation_action_id = generation
                .previous_generation_action
                .as_ref()
                .map(ToString::to_string);
            Ok(ExactIdentitySecurityRecordV2::VerifierKeyGeneration {
                action_seq: qualified_action_seq,
                action_id,
                action_hash: requested_action_hash,
                action_timestamp_micros: create.timestamp.as_micros(),
                did_document_action_id,
                previous_generation_action_id,
                generation,
            })
        }
        (
            Action::Update(_),
            IdentitySecurityEntryTypes::DidRegistryV2(
                did_registry_v2_integrity::EntryTypes::DidDeactivationV2(_),
            ),
        ) => Err(guest("DID deactivation security records must be Create actions")),
        (
            Action::Update(_),
            IdentitySecurityEntryTypes::VerifierKeyGeneration(_),
        ) => Err(guest("verifier-key generation security records must be Create actions")),
        _ => Err(guest("security entry type is incompatible with its Holochain action variant")),
    }
}

fn derive_exact_observation_digest_v2(
    verifier_agent: &AgentPubKey,
    coverage: &QualifiedAgentActivityCoverageV2,
    records: &[ExactIdentitySecurityRecordV2],
) -> ExternResult<[u8; SHA256_DIGEST_LEN_V2]> {
    let head_hash = parse_canonical_action_hash_v2(coverage.valid_head_action_id())?;
    let record_count = u32::try_from(records.len())
        .map_err(|_| guest("exact security Record set is too large to digest"))?;

    let mut hasher = Sha256::new();
    hasher.update(EXACT_HISTORICAL_RECORD_OBSERVATION_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(verifier_agent.get_raw_39());
    hasher.update([0x02]);
    hasher.update(coverage.valid_head_action_seq().to_be_bytes());
    hasher.update([0x03]);
    hasher.update(head_hash.get_raw_39());
    hasher.update([0x04]);
    hasher.update(record_count.to_be_bytes());

    for record in records {
        hasher.update([0x10]);
        hasher.update(record.action_seq().to_be_bytes());
        hasher.update(record.action_hash().get_raw_39());
    }
    Ok(hasher.finalize().into())
}

/// Bind one authority-scoped signed record to an exact, complete Holochain security
/// observation before delegating semantic lineage/material reasoning to #353.
///
/// The caller may supply only already-qualified #346 coverage plus the exact #374 signed
/// record envelope being evaluated. All historical security records and their normalized
/// fields are fetched and constructed internally from Holochain Records.
pub fn qualify_exact_historical_generation_observation_v2<'a>(
    coverage: &QualifiedAgentActivityCoverageV2,
    authority_scoped_record: AuthorityScopedKVectorProofVerificationRecordBodyV2<'a>,
) -> ExternResult<QualifiedExactHistoricalGenerationObservationV2> {
    let verifier_agent = verifier_agent_from_did_v2(authority_scoped_record.base_record.verifier_did)?;

    let mut exact_records = Vec::with_capacity(coverage.matching_valid_action_count());
    for qualified in coverage.matching_valid_actions() {
        exact_records.push(fetch_exact_security_record_v2(
            qualified.action_seq(),
            qualified.action_id(),
            &verifier_agent,
        )?);
    }
    exact_records.sort_by_key(ExactIdentitySecurityRecordV2::action_seq);

    let method_views: Vec<Vec<FetchedDidVerificationMethodV2<'_>>> = exact_records
        .iter()
        .map(|record| match record {
            ExactIdentitySecurityRecordV2::DidDocument { document, .. } => document
                .verification_method
                .iter()
                .map(|method| FetchedDidVerificationMethodV2 {
                    id: &method.id,
                    type_: &method.type_,
                    controller: &method.controller,
                    public_key_multibase: &method.public_key_multibase,
                    algorithm: method.algorithm,
                })
                .collect(),
            _ => Vec::new(),
        })
        .collect();

    let authentication_views: Vec<Vec<&str>> = exact_records
        .iter()
        .map(|record| match record {
            ExactIdentitySecurityRecordV2::DidDocument { document, .. } => {
                document.authentication.iter().map(String::as_str).collect()
            }
            _ => Vec::new(),
        })
        .collect();

    let key_agreement_views: Vec<Vec<&str>> = exact_records
        .iter()
        .map(|record| match record {
            ExactIdentitySecurityRecordV2::DidDocument { document, .. } => {
                document.key_agreement.iter().map(String::as_str).collect()
            }
            _ => Vec::new(),
        })
        .collect();

    let fetched_records: Vec<FetchedIdentitySecurityRecordV2<'_>> = exact_records
        .iter()
        .enumerate()
        .map(|(index, record)| match record {
            ExactIdentitySecurityRecordV2::DidDocument {
                action_seq,
                action_id,
                previous_action_id,
                document,
                ..
            } => FetchedIdentitySecurityRecordV2::DidDocument(FetchedDidDocumentRecordV2 {
                action_seq: *action_seq,
                action_id,
                previous_action_id: previous_action_id.as_deref(),
                version: document.version,
                did: &document.id,
                verification_methods: &method_views[index],
                authentication: &authentication_views[index],
                key_agreement: &key_agreement_views[index],
            }),
            ExactIdentitySecurityRecordV2::DidDeactivation {
                action_seq,
                action_id,
                deactivation,
                ..
            } => FetchedIdentitySecurityRecordV2::DidDeactivation(
                FetchedDidDeactivationRecordV2 {
                    action_seq: *action_seq,
                    action_id,
                    did: &deactivation.did,
                },
            ),
            ExactIdentitySecurityRecordV2::VerifierKeyGeneration {
                action_seq,
                action_id,
                action_timestamp_micros,
                did_document_action_id,
                previous_generation_action_id,
                generation,
                ..
            } => FetchedIdentitySecurityRecordV2::VerifierKeyGeneration(
                FetchedVerifierKeyGenerationRecordV2 {
                    action_seq: *action_seq,
                    action_id,
                    action_timestamp_micros: *action_timestamp_micros,
                    verifier_did: &generation.verifier_did,
                    verifier_key_id: &generation.verifier_key_id,
                    did_document_action_id,
                    key_generation: generation.key_generation,
                    valid_from_micros: generation.valid_from_micros,
                    valid_until_micros: generation.valid_until_micros,
                    previous_generation_action_id: previous_generation_action_id.as_deref(),
                },
            ),
        })
        .collect();

    let normalized_observation = assemble_normalized_historical_generation_observation_v2(
        coverage.clone(),
        &fetched_records,
        authority_scoped_record.base_record,
    )
    .map_err(|error| guest(format!("#353 historical observation rejected exact Records: {error:?}")))?;

    let record_signing_digest_sha256 =
        derive_authority_scoped_kvector_verification_record_signing_digest_v2(
            authority_scoped_record,
        )
        .map_err(|error| guest(format!("#374 signed record envelope is invalid: {error:?}")))?;
    let exact_observation_digest_sha256 =
        derive_exact_observation_digest_v2(&verifier_agent, coverage, &exact_records)?;

    Ok(QualifiedExactHistoricalGenerationObservationV2 {
        normalized_observation,
        record_signing_digest_sha256,
        exact_observation_digest_sha256,
        exact_record_count: exact_records.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_action_hash_parser_round_trips() {
        let hash = ActionHash::from_raw_36(vec![0x42; 36]);
        let encoded = ActionHashB64::from(hash.clone()).to_string();
        assert_eq!(parse_canonical_action_hash_v2(&encoded).unwrap(), hash);
    }

    #[test]
    fn canonical_verifier_did_parser_round_trips() {
        let agent = AgentPubKey::from_raw_36(vec![0x24; 36]);
        let encoded = AgentPubKeyB64::from(agent.clone()).to_string();
        let did = format!("{DID_PREFIX_V2}{encoded}");
        assert_eq!(verifier_agent_from_did_v2(&did).unwrap(), agent);
    }

    #[test]
    fn qualified_result_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedExactHistoricalGenerationObservationV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedExactHistoricalGenerationObservationV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub normalized_observation:",
            "pub record_signing_digest_sha256:",
            "pub exact_observation_digest_sha256:",
            "pub exact_record_count:",
        ] {
            assert!(!body.contains(field));
        }
    }

    #[test]
    fn public_qualification_has_no_normalized_record_input() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub fn qualify_exact_historical_generation_observation_v2")
            .unwrap();
        let end = source[start..].index(") -> ExternResult").unwrap() + start;
        let signature = &source[start..end];
        assert!(signature.contains("coverage: &QualifiedAgentActivityCoverageV2"));
        assert!(signature.contains(
            "authority_scoped_record: AuthorityScopedKVectorProofVerificationRecordBodyV2"
        ));
        assert!(!signature.contains("FetchedIdentitySecurityRecordV2"));
        assert!(!signature.contains("records:"));
        assert!(!signature.contains("action_hash:"));
        assert!(!signature.contains("verifier_agent:"));
    }
}
