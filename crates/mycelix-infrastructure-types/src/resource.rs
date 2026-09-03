use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{id::is_valid_token, InfrastructureErrorV1};

const MAX_RESOURCE_DIMENSIONS_V1: usize = 64;

/// Namespaced resource dimension, e.g. `storage/bytes` or `bandwidth/bytes`.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ResourceKeyV1(String);

impl ResourceKeyV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, InfrastructureErrorV1> {
        let value = value.into();
        if !is_valid_token(&value) {
            return Err(InfrastructureErrorV1::InvalidResourceKey);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        if is_valid_token(&self.0) {
            Ok(())
        } else {
            Err(InfrastructureErrorV1::InvalidResourceKey)
        }
    }
}

/// Extensible resource vector with deterministic key ordering.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceVectorV1(BTreeMap<ResourceKeyV1, u128>);

impl ResourceVectorV1 {
    pub fn new<I>(entries: I) -> Result<Self, InfrastructureErrorV1>
    where
        I: IntoIterator<Item = (ResourceKeyV1, u128)>,
    {
        let mut values = BTreeMap::new();
        for (key, amount) in entries {
            key.validate()?;
            if amount == 0 {
                return Err(InfrastructureErrorV1::ZeroResourceAmount);
            }
            if values.insert(key, amount).is_some() {
                return Err(InfrastructureErrorV1::DuplicateResourceKey);
            }
            if values.len() > MAX_RESOURCE_DIMENSIONS_V1 {
                return Err(InfrastructureErrorV1::TooManyResourceDimensions);
            }
        }
        if values.is_empty() {
            return Err(InfrastructureErrorV1::EmptyResourceVector);
        }
        Ok(Self(values))
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        if self.0.is_empty() {
            return Err(InfrastructureErrorV1::EmptyResourceVector);
        }
        if self.0.len() > MAX_RESOURCE_DIMENSIONS_V1 {
            return Err(InfrastructureErrorV1::TooManyResourceDimensions);
        }
        for (key, amount) in &self.0 {
            key.validate()?;
            if *amount == 0 {
                return Err(InfrastructureErrorV1::ZeroResourceAmount);
            }
        }
        Ok(())
    }

    pub fn get(&self, key: &ResourceKeyV1) -> Option<u128> {
        self.0.get(key).copied()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&ResourceKeyV1, &u128)> {
        self.0.iter()
    }

    /// True when every requested dimension exists and is <= the corresponding capacity.
    pub fn contains(&self, requested: &Self) -> bool {
        requested.0.iter().all(|(key, amount)| {
            self.0
                .get(key)
                .is_some_and(|capacity| amount <= capacity)
        })
    }

    pub(crate) fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&(self.0.len() as u32).to_be_bytes());
        for (key, amount) in &self.0 {
            let bytes = key.as_str().as_bytes();
            out.extend_from_slice(&(bytes.len() as u64).to_be_bytes());
            out.extend_from_slice(bytes);
            out.extend_from_slice(&amount.to_be_bytes());
        }
        out
    }
}
