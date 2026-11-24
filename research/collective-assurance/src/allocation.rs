//! Commit/reveal support for reproducible, concealed team-level randomization.
//!
//! Use OS entropy to create a secret, publish the commitment before enrollment,
//! keep the seed and salt concealed through allocation, then reveal after the
//! enrollment and analysis freeze. The commitment is a hash, not a timestamp,
//! signature, or allocation-concealment service; publish it through an
//! independently timestamped channel.

use crate::{allocate_complete_blocks, Assignment, RandomizationUnit, RandomSource};
use rand_chacha::ChaCha20Rng;
use rand_core::{Rng, SeedableRng};
use sha2::{Digest, Sha256};
use zeroize::Zeroize;

const COMMITMENT_DOMAIN: &[u8] = b"MYCELIX-TEAM-ALLOCATION-SEED-COMMITMENT-V0.1\0";
const ROSTER_DOMAIN: &[u8] = b"MYCELIX-TEAM-ALLOCATION-ROSTER-V0.1\0";
const DESIGN_DOMAIN: &[u8] = b"MYCELIX-TEAM-ALLOCATION-DESIGN-V0.1\0";
const ALGORITHM_ID: &str = "complete-block-fisher-yates-chacha20-v0.1";

/// Secret material stays out of Debug output and is zeroized on drop.
pub struct AllocationSecret {
    seed: [u8; 32],
    salt: [u8; 32],
}

impl AllocationSecret {
    /// Generate 256-bit seed and salt independently from the operating system RNG.
    /// There is deliberately no weak-entropy fallback.
    pub fn generate() -> Result<Self, String> {
        let mut seed = [0u8; 32];
        let mut salt = [0u8; 32];
        if let Err(error) = getrandom::fill(&mut seed) {
            seed.zeroize();
            salt.zeroize();
            return Err(format!("OS entropy unavailable for allocation seed: {error}"));
        }
        if let Err(error) = getrandom::fill(&mut salt) {
            seed.zeroize();
            salt.zeroize();
            return Err(format!("OS entropy unavailable for allocation salt: {error}"));
        }
        Ok(Self { seed, salt })
    }

    /// Reconstruct secret material from a post-study reveal.
    /// Callers must validate the reveal against its previously published commitment.
    pub fn from_reveal(seed: [u8; 32], salt: [u8; 32]) -> Self {
        Self { seed, salt }
    }

    /// Export the secret only after the preregistered reveal condition is met.
    pub fn reveal_hex(&self) -> (String, String) {
        (hex(&self.seed), hex(&self.salt))
    }
}

impl Drop for AllocationSecret {
    fn drop(&mut self) {
        self.seed.zeroize();
        self.salt.zeroize();
    }
}

pub struct AllocationCommitment {
    pub schema_version: &'static str,
    pub algorithm_id: &'static str,
    pub run_id: String,
    pub design_digest: String,
    pub roster_digest: String,
    pub seed_commitment_sha256: String,
}

fn push_len(out: &mut Vec<u8>, length: usize) {
    out.extend_from_slice(&(length as u64).to_be_bytes());
}

fn push_bytes(out: &mut Vec<u8>, bytes: &[u8]) {
    push_len(out, bytes.len());
    out.extend_from_slice(bytes);
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_bytes(out, value.as_bytes());
}

fn hex(bytes: &[u8]) -> String {
    let mut result = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write;
        let _ = write!(&mut result, "{byte:02x}");
    }
    result
}

fn sha256(bytes: &[u8]) -> String {
    hex(&Sha256::digest(bytes))
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_arms(arms: &[String]) -> Result<(), String> {
    if arms.len() < 2 {
        return Err("at least two randomization arms are required".into());
    }
    let mut seen = std::collections::BTreeSet::new();
    if arms
        .iter()
        .any(|arm| arm.trim().is_empty() || !seen.insert(arm.as_str()))
    {
        return Err("arm names must be non-empty and unique".into());
    }
    Ok(())
}

/// Canonical digest over team pseudonyms and strata, independent of roster input order.
pub fn compute_roster_digest(roster: &[RandomizationUnit]) -> Result<String, String> {
    if roster.is_empty() {
        return Err("allocation roster must not be empty".into());
    }
    let mut sorted: Vec<&RandomizationUnit> = roster.iter().collect();
    sorted.sort_by(|left, right| left.unit_id.cmp(&right.unit_id));
    let mut seen = std::collections::BTreeSet::new();
    let mut bytes = Vec::new();
    bytes.extend_from_slice(ROSTER_DOMAIN);
    push_len(&mut bytes, sorted.len());
    for unit in sorted {
        if unit.unit_id.trim().is_empty()
            || unit.stratum.trim().is_empty()
            || !seen.insert(unit.unit_id.as_str())
        {
            return Err("roster contains an empty field or duplicate team identity".into());
        }
        push_str(&mut bytes, &unit.unit_id);
        push_str(&mut bytes, &unit.stratum);
    }
    Ok(sha256(&bytes))
}

/// Bind the experiment ID, algorithm, arm order, and frozen roster.
pub fn compute_design_digest(
    run_id: &str,
    roster_digest: &str,
    arms: &[String],
) -> Result<String, String> {
    if run_id.trim().is_empty() {
        return Err("run id must not be empty".into());
    }
    if !is_sha256_hex(roster_digest) {
        return Err("roster digest must be lowercase SHA-256 hex".into());
    }
    validate_arms(arms)?;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(DESIGN_DOMAIN);
    push_str(&mut bytes, run_id);
    push_str(&mut bytes, ALGORITHM_ID);
    push_str(&mut bytes, roster_digest);
    push_len(&mut bytes, arms.len());
    for arm in arms {
        push_str(&mut bytes, arm);
    }
    Ok(sha256(&bytes))
}

fn compute_seed_commitment(
    run_id: &str,
    design_digest: &str,
    roster_digest: &str,
    seed: &[u8; 32],
    salt: &[u8; 32],
) -> String {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(COMMITMENT_DOMAIN);
    push_str(&mut bytes, run_id);
    push_str(&mut bytes, ALGORITHM_ID);
    push_str(&mut bytes, design_digest);
    push_str(&mut bytes, roster_digest);
    push_bytes(&mut bytes, seed);
    push_bytes(&mut bytes, salt);
    sha256(&bytes)
}

pub fn create_allocation_commitment(
    run_id: &str,
    roster: &[RandomizationUnit],
    arms: &[String],
    secret: &AllocationSecret,
) -> Result<AllocationCommitment, String> {
    let roster_digest = compute_roster_digest(roster)?;
    let design_digest = compute_design_digest(run_id, &roster_digest, arms)?;
    let seed_commitment_sha256 = compute_seed_commitment(
        run_id,
        &design_digest,
        &roster_digest,
        &secret.seed,
        &secret.salt,
    );
    Ok(AllocationCommitment {
        schema_version: "team-allocation-commitment-v0.1",
        algorithm_id: ALGORITHM_ID,
        run_id: run_id.to_owned(),
        design_digest,
        roster_digest,
        seed_commitment_sha256,
    })
}

/// Fail-closed reveal verification against the commitment and exact frozen design.
pub fn verify_allocation_reveal(
    commitment: &AllocationCommitment,
    roster: &[RandomizationUnit],
    arms: &[String],
    secret: &AllocationSecret,
) -> Result<(), String> {
    if commitment.schema_version != "team-allocation-commitment-v0.1"
        || commitment.algorithm_id != ALGORITHM_ID
    {
        return Err("unsupported commitment schema or randomization algorithm".into());
    }
    let actual_roster = compute_roster_digest(roster)?;
    if actual_roster != commitment.roster_digest {
        return Err("revealed roster does not match committed roster digest".into());
    }
    let actual_design = compute_design_digest(&commitment.run_id, &actual_roster, arms)?;
    if actual_design != commitment.design_digest {
        return Err("revealed design does not match committed design digest".into());
    }
    let actual_commitment = compute_seed_commitment(
        &commitment.run_id,
        &actual_design,
        &actual_roster,
        &secret.seed,
        &secret.salt,
    );
    if actual_commitment != commitment.seed_commitment_sha256 {
        return Err("seed/salt reveal does not match published commitment".into());
    }
    Ok(())
}

impl RandomSource for ChaCha20Rng {
    fn next_u64(&mut self) -> u64 {
        <ChaCha20Rng as Rng>::next_u64(self)
    }
}

/// Recompute the exact team schedule after reveal using a pinned ChaCha20 stream.
pub fn allocate_committed_teams(
    commitment: &AllocationCommitment,
    roster: &[RandomizationUnit],
    arms: &[String],
    secret: &AllocationSecret,
) -> Result<Vec<Assignment>, String> {
    verify_allocation_reveal(commitment, roster, arms, secret)?;
    let mut rng = ChaCha20Rng::from_seed(secret.seed);
    allocate_complete_blocks(roster, arms, &mut rng)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roster() -> Vec<RandomizationUnit> {
        (0..8)
            .map(|index| RandomizationUnit {
                unit_id: format!("team-{index:02}"),
                stratum: if index < 4 { "north" } else { "south" }.into(),
            })
            .collect()
    }

    fn arms() -> Vec<String> {
        vec![
            "ai-first".into(),
            "independent-first".into(),
            "evidence-first".into(),
            "human-only".into(),
        ]
    }

    fn secret() -> AllocationSecret {
        AllocationSecret::from_reveal([7u8; 32], [9u8; 32])
    }

    #[test]
    fn commitment_reveal_reproduces_exact_allocation() {
        let roster = roster();
        let arms = arms();
        let secret = secret();
        let commitment =
            create_allocation_commitment("run-001", &roster, &arms, &secret).unwrap();
        verify_allocation_reveal(&commitment, &roster, &arms, &secret).unwrap();
        let first = allocate_committed_teams(&commitment, &roster, &arms, &secret).unwrap();
        let second = allocate_committed_teams(&commitment, &roster, &arms, &secret).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn roster_input_order_does_not_change_commitment_or_assignment() {
        let original = roster();
        let mut reversed = original.clone();
        reversed.reverse();
        let arms = arms();
        let secret = secret();
        let first_commit =
            create_allocation_commitment("run-001", &original, &arms, &secret).unwrap();
        let second_commit =
            create_allocation_commitment("run-001", &reversed, &arms, &secret).unwrap();
        assert_eq!(first_commit.roster_digest, second_commit.roster_digest);
        assert_eq!(first_commit.design_digest, second_commit.design_digest);
        let first = allocate_committed_teams(&first_commit, &original, &arms, &secret).unwrap();
        let second = allocate_committed_teams(&second_commit, &reversed, &arms, &secret).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn changed_seed_or_salt_fails_reveal() {
        let roster = roster();
        let arms = arms();
        let original = secret();
        let commitment =
            create_allocation_commitment("run-001", &roster, &arms, &original).unwrap();
        let altered = AllocationSecret::from_reveal([8u8; 32], [9u8; 32]);
        assert!(verify_allocation_reveal(&commitment, &roster, &arms, &altered).is_err());
        let altered = AllocationSecret::from_reveal([7u8; 32], [10u8; 32]);
        assert!(verify_allocation_reveal(&commitment, &roster, &arms, &altered).is_err());
    }

    #[test]
    fn changed_roster_or_arm_order_fails_reveal() {
        let roster = roster();
        let arms = arms();
        let secret = secret();
        let commitment =
            create_allocation_commitment("run-001", &roster, &arms, &secret).unwrap();
        let mut changed_roster = roster.clone();
        changed_roster[0].stratum = "west".into();
        assert!(verify_allocation_reveal(&commitment, &changed_roster, &arms, &secret).is_err());
        let mut changed_arms = arms.clone();
        changed_arms.swap(0, 1);
        assert!(verify_allocation_reveal(&commitment, &roster, &changed_arms, &secret).is_err());
    }

    #[test]
    fn commitment_is_domain_and_run_bound() {
        let roster = roster();
        let arms = arms();
        let secret = secret();
        let first = create_allocation_commitment("run-001", &roster, &arms, &secret).unwrap();
        let second = create_allocation_commitment("run-002", &roster, &arms, &secret).unwrap();
        assert_ne!(first.seed_commitment_sha256, second.seed_commitment_sha256);
    }
}
