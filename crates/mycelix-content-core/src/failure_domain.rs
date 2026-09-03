use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::ContentErrorV1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum FailureDomainKindV1 {
    Operator,
    Machine,
    Site,
    NetworkAsn,
    Region,
    Jurisdiction,
    PowerDomain,
}

impl FailureDomainKindV1 {
    pub(crate) fn tag(self) -> u8 {
        match self {
            Self::Operator => 0,
            Self::Machine => 1,
            Self::Site => 2,
            Self::NetworkAsn => 3,
            Self::Region => 4,
            Self::Jurisdiction => 5,
            Self::PowerDomain => 6,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FailureDomainPolicyV1(BTreeMap<FailureDomainKindV1, u16>);

impl FailureDomainPolicyV1 {
    pub fn empty() -> Self {
        Self::default()
    }

    pub fn new<I>(requirements: I) -> Result<Self, ContentErrorV1>
    where
        I: IntoIterator<Item = (FailureDomainKindV1, u16)>,
    {
        let mut values = BTreeMap::new();
        for (kind, minimum_distinct) in requirements {
            if minimum_distinct == 0 {
                return Err(ContentErrorV1::ZeroFailureDomainMinimum);
            }
            if values.insert(kind, minimum_distinct).is_some() {
                return Err(ContentErrorV1::DuplicateFailureDomainRequirement);
            }
        }
        Ok(Self(values))
    }

    pub fn minimum_for(&self, kind: FailureDomainKindV1) -> Option<u16> {
        self.0.get(&kind).copied()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&FailureDomainKindV1, &u16)> {
        self.0.iter()
    }

    pub fn validate_against_replicas(&self, minimum_replicas: u16) -> Result<(), ContentErrorV1> {
        for minimum in self.0.values() {
            if *minimum == 0 {
                return Err(ContentErrorV1::ZeroFailureDomainMinimum);
            }
            if *minimum > minimum_replicas {
                return Err(ContentErrorV1::FailureDomainExceedsReplicas);
            }
        }
        Ok(())
    }

    pub(crate) fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&(self.0.len() as u32).to_be_bytes());
        for (kind, minimum) in &self.0 {
            out.push(kind.tag());
            out.extend_from_slice(&minimum.to_be_bytes());
        }
        out
    }
}
