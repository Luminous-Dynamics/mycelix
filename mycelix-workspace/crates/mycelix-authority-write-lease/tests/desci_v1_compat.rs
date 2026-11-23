//! Cross-implementation compatibility test against the actual DeSci v1
//! authority lease protocol source. The fixture contains the exact protocol
//! definitions plus their canonical byte helper functions; CI verifies both
//! source slices remain synchronized with the maintained DeSci source.

#![allow(dead_code, unused_imports)]

use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::SigningKey;
use uuid::Uuid;

#[derive(Debug)]
pub enum Error {
    Validation(String),
    VerificationFailed(String),
    Crypto(String),
}

pub type Result<T> = std::result::Result<T, Error>;

pub mod scientific_events {
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
    #[serde(transparent)]
    pub struct ContentHash(pub [u8; 32]);

    impl ContentHash {
        pub fn digest(bytes: &[u8]) -> Self {
            Self(*blake3::hash(bytes).as_bytes())
        }

        pub fn to_hex(self) -> String {
            hex::encode(self.0)
        }
    }
}

pub mod authority_signing {
    use crate::{Error, Result};
    use ed25519_dalek::{Signer, SigningKey, VerifyingKey};

    pub trait AuthoritySigner: Send + Sync {
        fn key_id(&self) -> &str;
        fn verifying_key(&self) -> VerifyingKey;
        fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>>;
    }

    impl AuthoritySigner for SigningKey {
        fn key_id(&self) -> &str {
            "software-ed25519"
        }

        fn verifying_key(&self) -> VerifyingKey {
            SigningKey::verifying_key(self)
        }

        fn sign_message(&self, message: &[u8]) -> Result<Vec<u8>> {
            Ok(self.sign(message).to_bytes().to_vec())
        }
    }

    impl From<std::io::Error> for Error {
        fn from(value: std::io::Error) -> Self {
            Self::Crypto(value.to_string())
        }
    }
}

#[path = "fixtures/desci_authority_fencing_v1.rs"]
mod desci_v1;

use mycelix_authority_write_lease as shared;
use scientific_events::ContentHash as LegacyContentHash;

fn legacy_signed_lease() -> desci_v1::SignedAuthorityWriteLease {
    let key = SigningKey::from_bytes(&[84; 32]);
    desci_v1::SignedAuthorityWriteLease::sign(
        desci_v1::AuthorityWriteLease {
            lease_id: Uuid::from_bytes([0x11; 16]),
            deployment_id: "deployment:test".into(),
            primary_id: "postgres-a".into(),
            database_system_identifier: "731245".into(),
            postgres_timeline: 2,
            generation: 7,
            phase: desci_v1::AuthorityWriteLeasePhase::Epoch,
            epoch_number: Some(3),
            epoch_hash: Some(LegacyContentHash([3; 32])),
            allowed_scopes: [desci_v1::AuthorityWriteScope::DirectoryFabricEffect]
                .into_iter()
                .collect(),
            issued_at: DateTime::from_timestamp(1_700_000_000, 0).unwrap(),
            not_before: DateTime::from_timestamp(1_700_000_001, 0).unwrap(),
            expires_at: DateTime::from_timestamp(1_700_000_301, 0).unwrap(),
            reason: "cross-implementation compatibility vector".into(),
        },
        &key,
    )
    .unwrap()
}

#[test]
fn actual_desci_v1_source_matches_shared_canonical_payload_signing_bytes_and_hash() {
    let legacy = legacy_signed_lease();
    let shared: shared::SignedAuthorityWriteLease =
        serde_json::from_value(serde_json::to_value(&legacy).unwrap()).unwrap();

    assert_eq!(
        legacy.lease.canonical_bytes().unwrap(),
        shared.lease.canonical_bytes().unwrap(),
        "DeSci and shared lease canonical payload bytes diverged"
    );
    assert_eq!(
        legacy.signing_bytes().unwrap(),
        shared.signing_bytes().unwrap(),
        "DeSci and shared signed-envelope bytes diverged"
    );
    assert_eq!(
        legacy.signed_lease_hash().unwrap().0,
        shared.signed_lease_hash().unwrap().0,
        "DeSci and shared lease hash diverged"
    );

    shared.verify_signature().unwrap();
    let trusted = [legacy.signer_public_key].into_iter().collect();
    let target = shared::AuthorityWriteLeaseTarget {
        deployment_id: "deployment:test".into(),
        primary_id: "postgres-a".into(),
        database_system_identifier: "731245".into(),
        postgres_timeline: 2,
        phase: shared::AuthorityWriteLeasePhase::Epoch,
        epoch_number: Some(3),
        epoch_hash: Some(shared::ContentHash([3; 32])),
    };
    shared
        .verify_for_target(
            &trusted,
            DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
            Duration::zero(),
            shared::AuthorityWriteScope::DirectoryFabricEffect,
            &target,
        )
        .unwrap();

    let mut wrong_target = target;
    wrong_target.postgres_timeline = 3;
    assert!(shared
        .verify_for_target(
            &trusted,
            DateTime::from_timestamp(1_700_000_100, 0).unwrap(),
            Duration::zero(),
            shared::AuthorityWriteScope::DirectoryFabricEffect,
            &wrong_target,
        )
        .is_err());
}
