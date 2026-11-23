//! Directory Fabric-specific use of the shared signed PostgreSQL lease protocol.
//!
//! This function verifies a lease for the exact Directory Fabric mutation
//! scope and target identity. It is a cryptographic admission primitive; it
//! does not by itself make a PostgreSQL transaction safe. The caller must
//! bind the returned lease hash/generation to a transactionally checked
//! database-side lease state and must not let application roles bypass that
//! checked mutation entry point.

use std::collections::BTreeSet;

use chrono::{DateTime, Duration, Utc};
use mycelix_authority_write_lease::{
    AuthorityWriteLeaseTarget, AuthorityWriteScope, ContentHash, SignedAuthorityWriteLease,
};

/// Verify a signed lease for a Directory Fabric protected effect.
///
/// The expected deployment/primary/database identity/timeline/epoch must be
/// obtained from the same trusted control-plane and PostgreSQL identity
/// evidence that will govern the mutation. A lease for another scope or
/// target is rejected even when its signature is otherwise valid.
pub fn verify_directory_fabric_write_lease(
    signed: &SignedAuthorityWriteLease,
    trusted_keys: &BTreeSet<[u8; 32]>,
    now: DateTime<Utc>,
    clock_skew: Duration,
    target: &AuthorityWriteLeaseTarget,
) -> Result<ContentHash, String> {
    signed.verify_for_target(
        trusted_keys,
        now,
        clock_skew,
        AuthorityWriteScope::DirectoryFabricEffect,
        target,
    )?;
    signed.signed_lease_hash()
}
