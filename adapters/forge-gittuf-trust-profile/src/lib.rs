// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pinned local-key verifier trust profile for gittuf v0.16.0.
//!
//! This crate defines which verification methods are compatible with an
//! offline/local trust closure. It deliberately does not scan gittuf policy
//! metadata itself; FORGE-004D2C2 must produce the complete inventory.

use mycelix_forge_core::{Digest, DigestAlgorithm};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize};
use thiserror::Error;

const PROFILE_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-local-trust-profile/v1\0";
const INVENTORY_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-policy-trust-inventory/v1\0";
const QUALIFIED_DOMAIN_V1: &[u8] = b"mycelix-forge/gittuf-local-trust-qualified/v1\0";

pub const GITTUF_VERSION: &str = "0.16.0";
pub const GITTUF_TAG_OBJECT: &str = "5d4d7652bf84e347eefbad1a7e07fc88dede9b92";
pub const GITTUF_RELEASE_COMMIT: &str = "fa3c295e1e46c1cff10aec1194edc29f19677723";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum VerificationMethod {
    #[serde(rename = "gpg")]
    Gpg,
    #[serde(rename = "ssh")]
    Ssh,
    #[serde(rename = "sigstore")]
    Sigstore,
}

impl VerificationMethod {
    const fn code(self) -> u8 {
        match self {
            Self::Gpg => 1,
            Self::Ssh => 2,
            Self::Sigstore => 3,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct PolicyTrustInventory {
    policy_state: Digest,
    methods: Vec<VerificationMethod>,
}

impl PolicyTrustInventory {
    pub fn new(
        policy_state: Digest,
        mut methods: Vec<VerificationMethod>,
    ) -> Result<Self, TrustProfileError> {
        if methods.is_empty() {
            return Err(TrustProfileError::EmptyMethodInventory);
        }
        methods.sort();
        methods.dedup();
        Ok(Self {
            policy_state,
            methods,
        })
    }

    pub fn policy_state(&self) -> &Digest {
        &self.policy_state
    }

    pub fn methods(&self) -> &[VerificationMethod] {
        &self.methods
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, TrustProfileError> {
        let mut out = Vec::new();
        out.extend_from_slice(INVENTORY_DOMAIN_V1);
        push_digest(&mut out, &self.policy_state)?;
        let count = u16::try_from(self.methods.len())
            .map_err(|_| TrustProfileError::CanonicalFieldTooLarge("methods"))?;
        out.extend_from_slice(&count.to_be_bytes());
        for method in &self.methods {
            out.push(method.code());
        }
        Ok(out)
    }

    pub fn digest(&self, algorithm: DigestAlgorithm) -> Result<Digest, TrustProfileError> {
        Ok(Digest::of_bytes(algorithm, &self.canonical_bytes()?))
    }
}

impl<'de> Deserialize<'de> for PolicyTrustInventory {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            policy_state: Digest,
            methods: Vec<VerificationMethod>,
        }

        let wire = Wire::deserialize(deserializer)?;
        let mut canonical = wire.methods.clone();
        canonical.sort();
        canonical.dedup();
        if canonical != wire.methods {
            return Err(D::Error::custom(
                "verification-method inventory is not canonical",
            ));
        }
        Self::new(wire.policy_state, wire.methods).map_err(D::Error::custom)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LocalEmbeddedKeysV1 {
    profile_digest: Digest,
}

impl LocalEmbeddedKeysV1 {
    pub fn profile_digest(&self) -> &Digest {
        &self.profile_digest
    }

    pub const fn allows(method: VerificationMethod) -> bool {
        matches!(method, VerificationMethod::Gpg | VerificationMethod::Ssh)
    }

    pub const fn requires_network_denied(&self) -> bool {
        true
    }

    pub const fn allows_ambient_user_keyring(&self) -> bool {
        false
    }
}

pub fn local_embedded_keys_v1() -> LocalEmbeddedKeysV1 {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(PROFILE_DOMAIN_V1);
    for value in [GITTUF_VERSION, GITTUF_TAG_OBJECT, GITTUF_RELEASE_COMMIT] {
        let len = u16::try_from(value.len()).expect("profile constants fit v1 length prefix");
        bytes.extend_from_slice(&len.to_be_bytes());
        bytes.extend_from_slice(value.as_bytes());
    }
    // Exact permitted method set, in canonical order.
    bytes.extend_from_slice(&2u16.to_be_bytes());
    bytes.push(VerificationMethod::Gpg.code());
    bytes.push(VerificationMethod::Ssh.code());
    // Runtime composition requirements: network denied, ambient keyring denied.
    bytes.push(1);
    bytes.push(0);

    LocalEmbeddedKeysV1 {
        profile_digest: Digest::of_bytes(DigestAlgorithm::Sha256, &bytes),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QualifiedLocalKeyTrustProfile {
    profile_digest: Digest,
    policy_state: Digest,
    inventory_digest: Digest,
    evidence_digest: Digest,
}

impl QualifiedLocalKeyTrustProfile {
    pub fn profile_digest(&self) -> &Digest {
        &self.profile_digest
    }

    pub fn policy_state(&self) -> &Digest {
        &self.policy_state
    }

    pub fn inventory_digest(&self) -> &Digest {
        &self.inventory_digest
    }

    pub fn evidence_digest(&self) -> &Digest {
        &self.evidence_digest
    }
}

pub fn qualify_local_key_profile(
    inventory: &PolicyTrustInventory,
) -> Result<QualifiedLocalKeyTrustProfile, TrustProfileError> {
    let profile = local_embedded_keys_v1();
    if let Some(forbidden) = inventory
        .methods()
        .iter()
        .copied()
        .find(|method| !LocalEmbeddedKeysV1::allows(*method))
    {
        return Err(TrustProfileError::MethodNotAllowed(forbidden));
    }

    let inventory_digest = inventory.digest(DigestAlgorithm::Sha256)?;
    let mut evidence = Vec::new();
    evidence.extend_from_slice(QUALIFIED_DOMAIN_V1);
    push_digest(&mut evidence, profile.profile_digest())?;
    push_digest(&mut evidence, inventory.policy_state())?;
    push_digest(&mut evidence, &inventory_digest)?;
    let evidence_digest = Digest::of_bytes(DigestAlgorithm::Sha256, &evidence);

    Ok(QualifiedLocalKeyTrustProfile {
        profile_digest: profile.profile_digest,
        policy_state: inventory.policy_state.clone(),
        inventory_digest,
        evidence_digest,
    })
}

fn push_digest(out: &mut Vec<u8>, digest: &Digest) -> Result<(), TrustProfileError> {
    let algorithm = digest.algorithm().id().as_bytes();
    let algorithm_len = u16::try_from(algorithm.len())
        .map_err(|_| TrustProfileError::CanonicalFieldTooLarge("digest algorithm"))?;
    let digest_len = u16::try_from(digest.as_bytes().len())
        .map_err(|_| TrustProfileError::CanonicalFieldTooLarge("digest"))?;
    out.extend_from_slice(&algorithm_len.to_be_bytes());
    out.extend_from_slice(algorithm);
    out.extend_from_slice(&digest_len.to_be_bytes());
    out.extend_from_slice(digest.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TrustProfileError {
    #[error("policy trust inventory may not be empty")]
    EmptyMethodInventory,
    #[error("verification method is not permitted by LocalEmbeddedKeysV1: {0:?}")]
    MethodNotAllowed(VerificationMethod),
    #[error("canonical field is too large: {0}")]
    CanonicalFieldTooLarge(&'static str),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        Digest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    #[test]
    fn ssh_and_gpg_inventory_qualifies() {
        let inventory = PolicyTrustInventory::new(
            digest(1),
            vec![VerificationMethod::Ssh, VerificationMethod::Gpg],
        )
        .unwrap();
        let qualified = qualify_local_key_profile(&inventory).unwrap();
        assert_eq!(qualified.policy_state(), inventory.policy_state());
    }

    #[test]
    fn sigstore_invalidates_local_key_profile() {
        let inventory = PolicyTrustInventory::new(
            digest(1),
            vec![VerificationMethod::Ssh, VerificationMethod::Sigstore],
        )
        .unwrap();
        assert_eq!(
            qualify_local_key_profile(&inventory).unwrap_err(),
            TrustProfileError::MethodNotAllowed(VerificationMethod::Sigstore)
        );
    }

    #[test]
    fn inventory_is_canonicalized() {
        let a = PolicyTrustInventory::new(
            digest(1),
            vec![VerificationMethod::Ssh, VerificationMethod::Gpg],
        )
        .unwrap();
        let b = PolicyTrustInventory::new(
            digest(1),
            vec![VerificationMethod::Gpg, VerificationMethod::Ssh],
        )
        .unwrap();
        assert_eq!(a, b);
        assert_eq!(
            a.digest(DigestAlgorithm::Sha256).unwrap(),
            b.digest(DigestAlgorithm::Sha256).unwrap()
        );
    }

    #[test]
    fn duplicate_methods_canonicalize_to_one() {
        let inventory = PolicyTrustInventory::new(
            digest(1),
            vec![VerificationMethod::Ssh, VerificationMethod::Ssh],
        )
        .unwrap();
        assert_eq!(inventory.methods(), &[VerificationMethod::Ssh]);
    }

    #[test]
    fn serde_rejects_unknown_method() {
        let value = serde_json::json!({
            "policy_state": digest(1),
            "methods": ["ssh", "future-method"]
        });
        assert!(serde_json::from_value::<PolicyTrustInventory>(value).is_err());
    }

    #[test]
    fn serde_rejects_noncanonical_method_order() {
        let inventory = PolicyTrustInventory::new(
            digest(1),
            vec![VerificationMethod::Gpg, VerificationMethod::Ssh],
        )
        .unwrap();
        let mut value = serde_json::to_value(inventory).unwrap();
        value["methods"].as_array_mut().unwrap().reverse();
        assert!(serde_json::from_value::<PolicyTrustInventory>(value).is_err());
    }

    #[test]
    fn profile_is_network_denied_and_keyring_free() {
        let profile = local_embedded_keys_v1();
        assert!(profile.requires_network_denied());
        assert!(!profile.allows_ambient_user_keyring());
        assert!(LocalEmbeddedKeysV1::allows(VerificationMethod::Ssh));
        assert!(LocalEmbeddedKeysV1::allows(VerificationMethod::Gpg));
        assert!(!LocalEmbeddedKeysV1::allows(VerificationMethod::Sigstore));
    }
}
