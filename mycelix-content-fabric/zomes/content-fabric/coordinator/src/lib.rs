// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Coordinator for Content Fabric discovery/evidence metadata.
//! No function here grants byte access, creates leases, moves bulk content, or settles payment.

use content_fabric_integrity::*;
use hdk::prelude::*;

fn guest_error(message: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(message.into()))
}

fn current_agent() -> ExternResult<AgentPubKey> {
    Ok(agent_info()?.agent_initial_pubkey)
}

fn require_agent(expected: &AgentPubKey) -> ExternResult<()> {
    let actual = current_agent()?;
    if &actual == expected {
        Ok(())
    } else {
        Err(guest_error("entry identity must match the calling Holochain agent"))
    }
}

fn anchor_hash(value: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(value.to_string())))
}

fn ensure_anchor(value: &str) -> ExternResult<EntryHash> {
    create_entry(&EntryTypes::Anchor(Anchor(value.to_string())))?;
    anchor_hash(value)
}

fn fetch_linked_records(
    base: impl Into<AnyLinkableHash>,
    link_type: LinkTypes,
    limit: usize,
) -> ExternResult<Vec<Record>> {
    let links = get_links(
        LinkQuery::try_new(base.into(), link_type)?,
        GetStrategy::default(),
    )?;
    let mut records = Vec::new();
    for link in links.into_iter().rev().take(limit.clamp(1, 256)) {
        if let Some(target) = link.target.into_action_hash() {
            if let Some(record) = get(target, GetOptions::default())? {
                records.push(record);
            }
        }
    }
    Ok(records)
}

#[hdk_extern]
pub fn publish_provider_advertisement(ad: ProviderAdvertisementV1) -> ExternResult<Record> {
    require_agent(&ad.provider)?;
    let action_hash = create_entry(&EntryTypes::ProviderAdvertisementV1(ad.clone()))?;

    let all = ensure_anchor(all_providers_anchor_v1())?;
    create_link(all, action_hash.clone(), LinkTypes::AllProviders, ())?;
    create_link(
        ad.provider.clone(),
        action_hash.clone(),
        LinkTypes::ProviderAdsByAgent,
        (),
    )?;
    for algorithm in ad.supported_algorithms {
        let base = ensure_anchor(&algorithm_anchor_v1(algorithm))?;
        create_link(
            base,
            action_hash.clone(),
            LinkTypes::ProvidersByAlgorithm,
            (),
        )?;
    }

    get(action_hash, GetOptions::default())?.ok_or_else(|| guest_error("new provider advertisement not found"))
}

#[hdk_extern]
pub fn publish_content_availability(
    claim: ContentAvailabilityClaimV1,
) -> ExternResult<Record> {
    require_agent(&claim.provider)?;
    let action_hash = create_entry(&EntryTypes::ContentAvailabilityClaimV1(claim.clone()))?;
    let digest_base = ensure_anchor(&digest_anchor_v1(&claim.digest))?;
    create_link(
        digest_base,
        action_hash.clone(),
        LinkTypes::AvailabilityByDigest,
        (),
    )?;
    create_link(
        claim.advertisement.clone(),
        action_hash.clone(),
        LinkTypes::AvailabilityByAdvertisement,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| guest_error("new availability claim not found"))
}

#[hdk_extern]
pub fn withdraw_provider(withdrawal: ProviderWithdrawalV1) -> ExternResult<Record> {
    require_agent(&withdrawal.provider)?;
    let action_hash = create_entry(&EntryTypes::ProviderWithdrawalV1(withdrawal.clone()))?;
    create_link(
        withdrawal.advertisement,
        action_hash.clone(),
        LinkTypes::WithdrawalsByAdvertisement,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| guest_error("new provider withdrawal not found"))
}

#[hdk_extern]
pub fn record_replica_observation(observation: ReplicaObservationV1) -> ExternResult<Record> {
    require_agent(&observation.observer)?;
    let action_hash = create_entry(&EntryTypes::ReplicaObservationV1(observation.clone()))?;
    let digest_base = ensure_anchor(&digest_anchor_v1(&observation.digest))?;
    create_link(
        digest_base,
        action_hash.clone(),
        LinkTypes::ObservationsByDigest,
        (),
    )?;
    create_link(
        observation.advertisement,
        action_hash.clone(),
        LinkTypes::ObservationsByAdvertisement,
        (),
    )?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| guest_error("new replica observation not found"))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct LimitInput {
    pub limit: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ProviderQueryInput {
    pub provider: AgentPubKey,
    pub limit: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AlgorithmQueryInput {
    pub algorithm: DigestAlgorithmV1,
    pub limit: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DigestQueryInput {
    pub digest: ContentDigestRefV1,
    pub limit: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AdvertisementQueryInput {
    pub advertisement: ActionHash,
    pub limit: usize,
}

#[hdk_extern]
pub fn list_provider_advertisements(input: LimitInput) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        anchor_hash(all_providers_anchor_v1())?,
        LinkTypes::AllProviders,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_provider_advertisements(input: ProviderQueryInput) -> ExternResult<Vec<Record>> {
    fetch_linked_records(input.provider, LinkTypes::ProviderAdsByAgent, input.limit)
}

#[hdk_extern]
pub fn get_providers_for_algorithm(input: AlgorithmQueryInput) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        anchor_hash(&algorithm_anchor_v1(input.algorithm))?,
        LinkTypes::ProvidersByAlgorithm,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_availability_for_digest(input: DigestQueryInput) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        anchor_hash(&digest_anchor_v1(&input.digest))?,
        LinkTypes::AvailabilityByDigest,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_observations_for_digest(input: DigestQueryInput) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        anchor_hash(&digest_anchor_v1(&input.digest))?,
        LinkTypes::ObservationsByDigest,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_availability_for_advertisement(
    input: AdvertisementQueryInput,
) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        input.advertisement,
        LinkTypes::AvailabilityByAdvertisement,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_observations_for_advertisement(
    input: AdvertisementQueryInput,
) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        input.advertisement,
        LinkTypes::ObservationsByAdvertisement,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_withdrawals_for_advertisement(
    input: AdvertisementQueryInput,
) -> ExternResult<Vec<Record>> {
    fetch_linked_records(
        input.advertisement,
        LinkTypes::WithdrawalsByAdvertisement,
        input.limit,
    )
}

#[hdk_extern]
pub fn get_provider_advertisement(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}
