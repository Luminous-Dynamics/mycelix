// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Domain separation tags for Mycelix proof protocols.
//!
//! Each cluster + proof type gets a unique domain tag to prevent
//! cross-protocol replay attacks. The canonical format is:
//! `ZTML:{Cluster}:{ProofType}:v{N}`.

use serde::{Deserialize, Serialize};

const MAX_COMPONENT_LEN: usize = 64;

/// Parsed components of a canonical domain tag.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DomainTagParts<'a> {
    pub cluster: &'a str,
    pub proof_type: &'a str,
    pub version: u32,
}

/// A domain separation tag that uniquely identifies a proof context.
///
/// Included in commitments and authenticated messages to prevent a proof from
/// being replayed in another cluster, purpose, or protocol version.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DomainTag {
    /// Raw canonical tag bytes (for example `ZTML:Hearth:GuardianAuthority:v1`).
    tag: Vec<u8>,
}

impl DomainTag {
    /// Create a canonical domain tag from trusted, static protocol components.
    ///
    /// This convenience constructor panics when a programmer supplies an
    /// invalid component. Use [`DomainTag::try_new`] for untrusted input.
    pub fn new(cluster: &str, proof_type: &str, version: u32) -> Self {
        Self::try_new(cluster, proof_type, version)
            .expect("domain-tag components must be canonical")
    }

    /// Create a canonical domain tag while validating all components.
    pub fn try_new(cluster: &str, proof_type: &str, version: u32) -> Result<Self, String> {
        validate_component(cluster, "cluster")?;
        validate_component(proof_type, "proof type")?;
        if version == 0 {
            return Err("domain-tag version must be greater than zero".to_string());
        }

        Ok(Self {
            tag: format!("ZTML:{cluster}:{proof_type}:v{version}").into_bytes(),
        })
    }

    /// Create from raw bytes without validating them.
    ///
    /// Kept for legacy deserialization. Call [`DomainTag::validate`] before
    /// using a value from an untrusted source.
    pub fn from_bytes(bytes: Vec<u8>) -> Self {
        Self { tag: bytes }
    }

    /// Create from untrusted raw bytes and require canonical encoding.
    pub fn try_from_bytes(bytes: Vec<u8>) -> Result<Self, String> {
        let tag = Self { tag: bytes };
        tag.validate()?;
        Ok(tag)
    }

    /// Get the tag as bytes for hashing.
    pub fn as_bytes(&self) -> &[u8] {
        &self.tag
    }

    /// Get the tag as a UTF-8 string (best effort for legacy values).
    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.tag).unwrap_or("<invalid utf8>")
    }

    /// Parse and validate the canonical tag structure.
    pub fn parse(&self) -> Result<DomainTagParts<'_>, String> {
        let value = std::str::from_utf8(&self.tag)
            .map_err(|_| "domain tag must be valid UTF-8".to_string())?;
        let mut parts = value.split(':');

        if parts.next() != Some("ZTML") {
            return Err("domain tag must start with 'ZTML:'".to_string());
        }
        let cluster = parts
            .next()
            .ok_or_else(|| "domain tag is missing cluster".to_string())?;
        let proof_type = parts
            .next()
            .ok_or_else(|| "domain tag is missing proof type".to_string())?;
        let version_part = parts
            .next()
            .ok_or_else(|| "domain tag is missing version".to_string())?;
        if parts.next().is_some() {
            return Err("domain tag contains unexpected extra components".to_string());
        }

        validate_component(cluster, "cluster")?;
        validate_component(proof_type, "proof type")?;
        let version = version_part
            .strip_prefix('v')
            .ok_or_else(|| "domain-tag version must start with 'v'".to_string())?
            .parse::<u32>()
            .map_err(|_| "domain-tag version must be an unsigned integer".to_string())?;
        if version == 0 {
            return Err("domain-tag version must be greater than zero".to_string());
        }

        Ok(DomainTagParts {
            cluster,
            proof_type,
            version,
        })
    }

    /// Validate that this value is a canonical domain tag.
    pub fn validate(&self) -> Result<(), String> {
        self.parse().map(|_| ())
    }

    /// Validate exact cluster and proof-type identity.
    ///
    /// Unlike the legacy prefix comparison, this rejects extra components such
    /// as `ZTML:Hearth:GuardianAuthority:Other:v1`.
    pub fn matches(&self, cluster: &str, proof_type: &str) -> bool {
        self.parse()
            .map(|parts| parts.cluster == cluster && parts.proof_type == proof_type)
            .unwrap_or(false)
    }

    /// Validate exact cluster, proof type, and version.
    pub fn matches_version(&self, cluster: &str, proof_type: &str, version: u32) -> bool {
        self.parse()
            .map(|parts| {
                parts.cluster == cluster
                    && parts.proof_type == proof_type
                    && parts.version == version
            })
            .unwrap_or(false)
    }
}

fn validate_component(value: &str, label: &str) -> Result<(), String> {
    if value.is_empty() {
        return Err(format!("domain-tag {label} cannot be empty"));
    }
    if value.len() > MAX_COMPONENT_LEN {
        return Err(format!(
            "domain-tag {label} exceeds {MAX_COMPONENT_LEN} bytes"
        ));
    }
    if !value
        .bytes()
        .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
    {
        return Err(format!(
            "domain-tag {label} may contain only ASCII letters, digits, '-' and '_'"
        ));
    }
    Ok(())
}

// --- Well-known domain tags ---

/// Existing: Federated learning gradient validation.
pub const TAG_FL_GRADIENT: &[u8] = b"ZTML:Gen7:AuthGradProof:v1";

/// Governance: Anonymous voting.
pub fn tag_governance_vote() -> DomainTag {
    DomainTag::new("Governance", "AnonVote", 1)
}

/// Identity: Selective credential disclosure.
pub fn tag_identity_disclosure() -> DomainTag {
    DomainTag::new("Identity", "SelectiveDisclosure", 1)
}

/// Consciousness: Tier verification (used by multiple clusters).
pub fn tag_consciousness_tier() -> DomainTag {
    DomainTag::new("Consciousness", "TierProof", 1)
}

/// Finance: Transaction privacy.
pub fn tag_finance_tx() -> DomainTag {
    DomainTag::new("Finance", "TxPrivacy", 1)
}

/// Health: Record attestation.
pub fn tag_health_attest() -> DomainTag {
    DomainTag::new("Health", "RecordAttest", 1)
}

/// Mail: Sender authentication.
pub fn tag_mail_sender() -> DomainTag {
    DomainTag::new("Mail", "SenderAuth", 1)
}

/// Personal: Vault access.
pub fn tag_personal_vault() -> DomainTag {
    DomainTag::new("Personal", "VaultAccess", 1)
}

/// Supply chain: Provenance.
pub fn tag_supply_provenance() -> DomainTag {
    DomainTag::new("Supply", "Provenance", 1)
}

/// Hearth: prove active household membership without disclosing membership data.
pub fn tag_hearth_member_eligibility() -> DomainTag {
    DomainTag::new("Hearth", "MemberEligibility", 1)
}

/// Hearth: prove guardian authority for a context-bound operation.
pub fn tag_hearth_guardian_authority() -> DomainTag {
    DomainTag::new("Hearth", "GuardianAuthority", 1)
}

/// Hearth: prove that an autonomy/capability threshold is satisfied.
pub fn tag_hearth_capability_threshold() -> DomainTag {
    DomainTag::new("Hearth", "CapabilityThreshold", 1)
}

/// Hearth: prove eligibility for a context-bound anonymous decision.
pub fn tag_hearth_anonymous_decision() -> DomainTag {
    DomainTag::new("Hearth", "AnonymousDecision", 1)
}

/// Hearth: authorize a narrowly scoped emergency break-glass operation.
pub fn tag_hearth_emergency_break_glass() -> DomainTag {
    DomainTag::new("Hearth", "EmergencyBreakGlass", 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_domain_tag_format() {
        let tag = DomainTag::new("Governance", "AnonVote", 1);
        assert_eq!(tag.as_str(), "ZTML:Governance:AnonVote:v1");
        assert_eq!(tag.as_bytes(), b"ZTML:Governance:AnonVote:v1");
    }

    #[test]
    fn test_domain_tag_matches_exact_components() {
        let tag = DomainTag::new("Health", "RecordAttest", 1);
        assert!(tag.matches("Health", "RecordAttest"));
        assert!(tag.matches_version("Health", "RecordAttest", 1));
        assert!(!tag.matches("Health", "WrongType"));
        assert!(!tag.matches("Finance", "RecordAttest"));
        assert!(!tag.matches_version("Health", "RecordAttest", 2));
    }

    #[test]
    fn test_prefix_smuggling_is_rejected() {
        let smuggled = DomainTag::from_bytes(
            b"ZTML:Hearth:GuardianAuthority:Other:v1".to_vec(),
        );
        assert!(smuggled.validate().is_err());
        assert!(!smuggled.matches("Hearth", "GuardianAuthority"));
    }

    #[test]
    fn test_invalid_components_are_rejected() {
        assert!(DomainTag::try_new("", "Proof", 1).is_err());
        assert!(DomainTag::try_new("Hearth:Other", "Proof", 1).is_err());
        assert!(DomainTag::try_new("Hearth", "Proof\nInjected", 1).is_err());
        assert!(DomainTag::try_new("Hearth", "Proof", 0).is_err());
    }

    #[test]
    fn test_all_tags_unique() {
        let tags: Vec<DomainTag> = vec![
            tag_governance_vote(),
            tag_identity_disclosure(),
            tag_consciousness_tier(),
            tag_finance_tx(),
            tag_health_attest(),
            tag_mail_sender(),
            tag_personal_vault(),
            tag_supply_provenance(),
            tag_hearth_member_eligibility(),
            tag_hearth_guardian_authority(),
            tag_hearth_capability_threshold(),
            tag_hearth_anonymous_decision(),
            tag_hearth_emergency_break_glass(),
        ];
        for i in 0..tags.len() {
            for j in (i + 1)..tags.len() {
                assert_ne!(
                    tags[i],
                    tags[j],
                    "Domain tags must be unique: {} vs {}",
                    tags[i].as_str(),
                    tags[j].as_str()
                );
            }
        }
    }

    #[test]
    fn test_legacy_tag_compat() {
        let legacy = DomainTag::from_bytes(TAG_FL_GRADIENT.to_vec());
        assert_eq!(legacy.as_str(), "ZTML:Gen7:AuthGradProof:v1");
        assert!(legacy.matches("Gen7", "AuthGradProof"));
    }
}
