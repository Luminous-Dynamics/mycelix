use serde::{Deserialize, Serialize};

use crate::{InfrastructureErrorV1, PartyIdV1, PayloadCommitmentV1, ResourceVectorV1, StableIdV1};

pub const INFRASTRUCTURE_SCHEMA_V1: u16 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct UnixMillisV1(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TimeWindowV1 {
    pub start: UnixMillisV1,
    pub end: UnixMillisV1,
}

impl TimeWindowV1 {
    pub fn new(start: u64, end: u64) -> Result<Self, InfrastructureErrorV1> {
        if start >= end {
            return Err(InfrastructureErrorV1::InvalidTimeWindow);
        }
        Ok(Self {
            start: UnixMillisV1(start),
            end: UnixMillisV1(end),
        })
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        if self.start >= self.end {
            return Err(InfrastructureErrorV1::InvalidTimeWindow);
        }
        Ok(())
    }

    pub fn contains(&self, child: &Self) -> bool {
        self.start <= child.start && child.end <= self.end
    }

    pub(crate) fn canonical_bytes(&self) -> [u8; 16] {
        let mut out = [0u8; 16];
        out[..8].copy_from_slice(&self.start.0.to_be_bytes());
        out[8..].copy_from_slice(&self.end.0.to_be_bytes());
        out
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CapabilityEnvelopeV1 {
    pub schema_version: u16,
    pub id: StableIdV1,
    pub provider: PartyIdV1,
    pub validity: TimeWindowV1,
    pub resources: ResourceVectorV1,
    pub payload: PayloadCommitmentV1,
}

impl CapabilityEnvelopeV1 {
    pub fn new(
        provider: PartyIdV1,
        validity: TimeWindowV1,
        resources: ResourceVectorV1,
        payload: PayloadCommitmentV1,
    ) -> Result<Self, InfrastructureErrorV1> {
        validate_common(INFRASTRUCTURE_SCHEMA_V1, &validity, &resources, &payload)?;
        let id = capability_id(provider, validity, &resources, &payload)?;
        Ok(Self {
            schema_version: INFRASTRUCTURE_SCHEMA_V1,
            id,
            provider,
            validity,
            resources,
            payload,
        })
    }

    pub fn recompute_id(&self) -> Result<StableIdV1, InfrastructureErrorV1> {
        capability_id(self.provider, self.validity, &self.resources, &self.payload)
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        validate_common(self.schema_version, &self.validity, &self.resources, &self.payload)?;
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OfferingEnvelopeV1 {
    pub schema_version: u16,
    pub id: StableIdV1,
    pub capability_id: StableIdV1,
    pub provider: PartyIdV1,
    pub validity: TimeWindowV1,
    pub resources: ResourceVectorV1,
    pub payload: PayloadCommitmentV1,
}

impl OfferingEnvelopeV1 {
    pub fn new(
        capability: &CapabilityEnvelopeV1,
        validity: TimeWindowV1,
        resources: ResourceVectorV1,
        payload: PayloadCommitmentV1,
    ) -> Result<Self, InfrastructureErrorV1> {
        capability.validate()?;
        validate_common(INFRASTRUCTURE_SCHEMA_V1, &validity, &resources, &payload)?;
        validate_child(&capability.validity, &capability.resources, &validity, &resources)?;
        let id = offering_id(capability.id, capability.provider, validity, &resources, &payload)?;
        Ok(Self {
            schema_version: INFRASTRUCTURE_SCHEMA_V1,
            id,
            capability_id: capability.id,
            provider: capability.provider,
            validity,
            resources,
            payload,
        })
    }

    pub fn validate_against(&self, capability: &CapabilityEnvelopeV1) -> Result<(), InfrastructureErrorV1> {
        capability.validate()?;
        validate_common(self.schema_version, &self.validity, &self.resources, &self.payload)?;
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        if self.capability_id != capability.id {
            return Err(InfrastructureErrorV1::ParentIdMismatch);
        }
        if self.provider != capability.provider {
            return Err(InfrastructureErrorV1::PartyMismatch);
        }
        validate_child(&capability.validity, &capability.resources, &self.validity, &self.resources)
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        validate_common(self.schema_version, &self.validity, &self.resources, &self.payload)?;
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> Result<StableIdV1, InfrastructureErrorV1> {
        offering_id(self.capability_id, self.provider, self.validity, &self.resources, &self.payload)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeaseEnvelopeV1 {
    pub schema_version: u16,
    pub id: StableIdV1,
    pub offering_id: StableIdV1,
    pub provider: PartyIdV1,
    pub consumer: PartyIdV1,
    pub validity: TimeWindowV1,
    pub resources: ResourceVectorV1,
    pub payload: PayloadCommitmentV1,
}

impl LeaseEnvelopeV1 {
    pub fn new(
        offering: &OfferingEnvelopeV1,
        consumer: PartyIdV1,
        validity: TimeWindowV1,
        resources: ResourceVectorV1,
        payload: PayloadCommitmentV1,
    ) -> Result<Self, InfrastructureErrorV1> {
        offering.validate()?;
        validate_common(INFRASTRUCTURE_SCHEMA_V1, &validity, &resources, &payload)?;
        validate_child(&offering.validity, &offering.resources, &validity, &resources)?;
        let id = lease_id(
            offering.id,
            offering.provider,
            consumer,
            validity,
            &resources,
            &payload,
        )?;
        Ok(Self {
            schema_version: INFRASTRUCTURE_SCHEMA_V1,
            id,
            offering_id: offering.id,
            provider: offering.provider,
            consumer,
            validity,
            resources,
            payload,
        })
    }

    pub fn validate_against(&self, offering: &OfferingEnvelopeV1) -> Result<(), InfrastructureErrorV1> {
        offering.validate()?;
        validate_common(self.schema_version, &self.validity, &self.resources, &self.payload)?;
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        if self.offering_id != offering.id {
            return Err(InfrastructureErrorV1::ParentIdMismatch);
        }
        if self.provider != offering.provider {
            return Err(InfrastructureErrorV1::PartyMismatch);
        }
        validate_child(&offering.validity, &offering.resources, &self.validity, &self.resources)
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        validate_common(self.schema_version, &self.validity, &self.resources, &self.payload)?;
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> Result<StableIdV1, InfrastructureErrorV1> {
        lease_id(
            self.offering_id,
            self.provider,
            self.consumer,
            self.validity,
            &self.resources,
            &self.payload,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReceiptEnvelopeV1 {
    pub schema_version: u16,
    pub id: StableIdV1,
    pub lease_id: StableIdV1,
    pub provider: PartyIdV1,
    pub consumer: PartyIdV1,
    pub service_window: TimeWindowV1,
    pub observed_at: UnixMillisV1,
    pub delivered: ResourceVectorV1,
    pub payload: PayloadCommitmentV1,
}

impl ReceiptEnvelopeV1 {
    pub fn new(
        lease: &LeaseEnvelopeV1,
        service_window: TimeWindowV1,
        observed_at: UnixMillisV1,
        delivered: ResourceVectorV1,
        payload: PayloadCommitmentV1,
    ) -> Result<Self, InfrastructureErrorV1> {
        lease.validate()?;
        service_window.validate()?;
        delivered.validate()?;
        payload.validate()?;
        validate_child(&lease.validity, &lease.resources, &service_window, &delivered)?;
        if observed_at < service_window.end {
            return Err(InfrastructureErrorV1::ReceiptBeforeServiceEnd);
        }
        let id = receipt_id(
            lease.id,
            lease.provider,
            lease.consumer,
            service_window,
            observed_at,
            &delivered,
            &payload,
        )?;
        Ok(Self {
            schema_version: INFRASTRUCTURE_SCHEMA_V1,
            id,
            lease_id: lease.id,
            provider: lease.provider,
            consumer: lease.consumer,
            service_window,
            observed_at,
            delivered,
            payload,
        })
    }

    pub fn validate_against(&self, lease: &LeaseEnvelopeV1) -> Result<(), InfrastructureErrorV1> {
        lease.validate()?;
        if self.schema_version != INFRASTRUCTURE_SCHEMA_V1 {
            return Err(InfrastructureErrorV1::UnsupportedSchemaVersion);
        }
        self.service_window.validate()?;
        self.delivered.validate()?;
        self.payload.validate()?;
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        if self.lease_id != lease.id {
            return Err(InfrastructureErrorV1::ParentIdMismatch);
        }
        if self.provider != lease.provider || self.consumer != lease.consumer {
            return Err(InfrastructureErrorV1::PartyMismatch);
        }
        validate_child(&lease.validity, &lease.resources, &self.service_window, &self.delivered)?;
        if self.observed_at < self.service_window.end {
            return Err(InfrastructureErrorV1::ReceiptBeforeServiceEnd);
        }
        Ok(())
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        if self.schema_version != INFRASTRUCTURE_SCHEMA_V1 {
            return Err(InfrastructureErrorV1::UnsupportedSchemaVersion);
        }
        self.service_window.validate()?;
        self.delivered.validate()?;
        self.payload.validate()?;
        if self.observed_at < self.service_window.end {
            return Err(InfrastructureErrorV1::ReceiptBeforeServiceEnd);
        }
        if self.id != self.recompute_id()? {
            return Err(InfrastructureErrorV1::IdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> Result<StableIdV1, InfrastructureErrorV1> {
        receipt_id(
            self.lease_id,
            self.provider,
            self.consumer,
            self.service_window,
            self.observed_at,
            &self.delivered,
            &self.payload,
        )
    }
}

fn validate_common(
    schema_version: u16,
    validity: &TimeWindowV1,
    resources: &ResourceVectorV1,
    payload: &PayloadCommitmentV1,
) -> Result<(), InfrastructureErrorV1> {
    if schema_version != INFRASTRUCTURE_SCHEMA_V1 {
        return Err(InfrastructureErrorV1::UnsupportedSchemaVersion);
    }
    validity.validate()?;
    resources.validate()?;
    payload.validate()?;
    Ok(())
}

fn validate_child(
    parent_window: &TimeWindowV1,
    parent_resources: &ResourceVectorV1,
    child_window: &TimeWindowV1,
    child_resources: &ResourceVectorV1,
) -> Result<(), InfrastructureErrorV1> {
    parent_window.validate()?;
    child_window.validate()?;
    parent_resources.validate()?;
    child_resources.validate()?;
    if !parent_window.contains(child_window) {
        return Err(InfrastructureErrorV1::WindowOutsideParent);
    }
    if !parent_resources.contains(child_resources) {
        return Err(InfrastructureErrorV1::ResourceExceedsParent);
    }
    Ok(())
}

fn capability_id(
    provider: PartyIdV1,
    validity: TimeWindowV1,
    resources: &ResourceVectorV1,
    payload: &PayloadCommitmentV1,
) -> Result<StableIdV1, InfrastructureErrorV1> {
    let time = validity.canonical_bytes();
    let resources = resources.canonical_bytes();
    let payload_fields = payload.canonical_fields();
    StableIdV1::derive(
        "capability",
        INFRASTRUCTURE_SCHEMA_V1,
        &[&provider.0, &time, &resources, payload_fields[0], payload_fields[1]],
    )
}

fn offering_id(
    capability_id: StableIdV1,
    provider: PartyIdV1,
    validity: TimeWindowV1,
    resources: &ResourceVectorV1,
    payload: &PayloadCommitmentV1,
) -> Result<StableIdV1, InfrastructureErrorV1> {
    let time = validity.canonical_bytes();
    let resources = resources.canonical_bytes();
    let payload_fields = payload.canonical_fields();
    StableIdV1::derive(
        "offering",
        INFRASTRUCTURE_SCHEMA_V1,
        &[
            &capability_id.0,
            &provider.0,
            &time,
            &resources,
            payload_fields[0],
            payload_fields[1],
        ],
    )
}

fn lease_id(
    offering_id: StableIdV1,
    provider: PartyIdV1,
    consumer: PartyIdV1,
    validity: TimeWindowV1,
    resources: &ResourceVectorV1,
    payload: &PayloadCommitmentV1,
) -> Result<StableIdV1, InfrastructureErrorV1> {
    let time = validity.canonical_bytes();
    let resources = resources.canonical_bytes();
    let payload_fields = payload.canonical_fields();
    StableIdV1::derive(
        "lease",
        INFRASTRUCTURE_SCHEMA_V1,
        &[
            &offering_id.0,
            &provider.0,
            &consumer.0,
            &time,
            &resources,
            payload_fields[0],
            payload_fields[1],
        ],
    )
}

fn receipt_id(
    lease_id: StableIdV1,
    provider: PartyIdV1,
    consumer: PartyIdV1,
    service_window: TimeWindowV1,
    observed_at: UnixMillisV1,
    delivered: &ResourceVectorV1,
    payload: &PayloadCommitmentV1,
) -> Result<StableIdV1, InfrastructureErrorV1> {
    let time = service_window.canonical_bytes();
    let observed_at = observed_at.0.to_be_bytes();
    let delivered = delivered.canonical_bytes();
    let payload_fields = payload.canonical_fields();
    StableIdV1::derive(
        "receipt",
        INFRASTRUCTURE_SCHEMA_V1,
        &[
            &lease_id.0,
            &provider.0,
            &consumer.0,
            &time,
            &observed_at,
            &delivered,
            payload_fields[0],
            payload_fields[1],
        ],
    )
}
