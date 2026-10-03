use crate::identity_lineage::{IdentityKind, IdentityRef};
use serde::{Deserialize, Serialize};

/// Structural provenance witness explaining why a logical qualification
/// dependency is allowed to participate in a runtime binding.
///
/// This witness deliberately contains no protocol address. The Holochain
/// adapter binds the same logical identity to a concrete ActionHash/EntryHash
/// separately. The witness therefore explains provenance without becoming a
/// second interpretation of protocol addresses.
///
/// This is a structural provenance statement, not a cryptographic proof,
/// legal authority assertion, or global completeness claim.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationDependencyBindingProvenance {
    pub witness_identity: IdentityRef,
    pub logical_identity: IdentityRef,
    pub authority: IdentityRef,
    pub authority_scope: IdentityRef,
    pub authority_delegation: IdentityRef,
    pub basis: Vec<IdentityRef>,
}

impl QualificationDependencyBindingProvenance {
    pub fn validate(&self) -> Result<(), String> {
        self.witness_identity.validate()?;
        if self.witness_identity.kind != IdentityKind::ReconciliationWitness {
            return Err(
                "dependency binding provenance witness must use ReconciliationWitness identity kind"
                    .into(),
            );
        }

        self.logical_identity.validate()?;
        self.authority.validate()?;
        self.authority_scope.validate()?;
        self.authority_delegation.validate()?;

        if self.authority_scope.kind != IdentityKind::ReconciliationWitness {
            return Err("binding provenance authority scope must be a ReconciliationWitness".into());
        }
        if self.authority_delegation.kind != IdentityKind::ReconciliationWitness {
            return Err(
                "binding provenance authority delegation must be a ReconciliationWitness".into(),
            );
        }

        let identities = [
            &self.witness_identity,
            &self.logical_identity,
            &self.authority,
            &self.authority_scope,
            &self.authority_delegation,
        ];
        let mut distinct = std::collections::BTreeSet::new();
        for identity in identities {
            if !distinct.insert(identity) {
                return Err("binding provenance identities must be distinct".into());
            }
        }

        if self.basis.is_empty() {
            return Err("binding provenance requires at least one basis witness".into());
        }

        let mut seen = std::collections::BTreeSet::new();
        for basis in &self.basis {
            basis.validate()?;
            if *basis == self.witness_identity {
                return Err(
                    "binding provenance witness cannot include itself in its own basis".into(),
                );
            }
            if *basis == self.logical_identity {
                return Err(
                    "binding provenance basis cannot silently turn the bound dependency into its own justification"
                        .into(),
                );
            }
            if !seen.insert(basis) {
                return Err("binding provenance basis witnesses must be unique".into());
            }
        }

        for required in [
            &self.authority,
            &self.authority_scope,
            &self.authority_delegation,
        ] {
            if !seen.contains(required) {
                return Err(
                    "binding provenance basis must include every exact authority witness".into(),
                );
            }
        }

        Ok(())
    }

    pub fn matches_logical_identity(&self, logical_identity: &IdentityRef) -> bool {
        &self.logical_identity == logical_identity
    }
}

/// Structural provenance witness explaining why a domain authority identity
/// is eligible to be associated with a protocol AgentPubKey.
///
/// This deliberately differs from QualificationDependencyBindingProvenance:
/// the authority is the subject of this credential rather than the supporting
/// authority for another dependency.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QualificationAuthorityAgentBindingProvenance {
    pub witness_identity: IdentityRef,
    pub authority: IdentityRef,
    pub authority_scope: IdentityRef,
    pub authority_delegation: IdentityRef,
    pub basis: Vec<IdentityRef>,
}

impl QualificationAuthorityAgentBindingProvenance {
    pub fn validate(&self) -> Result<(), String> {
        self.witness_identity.validate()?;
        self.authority.validate()?;
        self.authority_scope.validate()?;
        self.authority_delegation.validate()?;

        if self.witness_identity.kind != IdentityKind::ReconciliationWitness {
            return Err("authority-agent provenance witness must use ReconciliationWitness identity kind".into());
        }
        if self.authority_scope.kind != IdentityKind::ReconciliationWitness {
            return Err("authority-agent provenance scope must use ReconciliationWitness identity kind".into());
        }
        if self.authority_delegation.kind != IdentityKind::ReconciliationWitness {
            return Err("authority-agent provenance delegation must use ReconciliationWitness identity kind".into());
        }

        let identities = [
            &self.witness_identity,
            &self.authority,
            &self.authority_scope,
            &self.authority_delegation,
        ];
        let mut distinct = std::collections::BTreeSet::new();
        for identity in identities {
            if !distinct.insert(identity) {
                return Err("authority-agent provenance identities must be distinct".into());
            }
        }

        if self.basis.is_empty() {
            return Err("authority-agent provenance requires at least one basis witness".into());
        }

        let mut seen = std::collections::BTreeSet::new();
        for basis in &self.basis {
            basis.validate()?;
            if *basis == self.witness_identity {
                return Err("authority-agent provenance witness cannot include itself in basis".into());
            }
            if *basis == self.authority {
                return Err("authority-agent provenance cannot use the authority identity as its own basis".into());
            }
            if !seen.insert(basis) {
                return Err("authority-agent provenance basis witnesses must be unique".into());
            }
        }

        for required in [&self.authority_scope, &self.authority_delegation] {
            if !seen.contains(required) {
                return Err("authority-agent provenance basis must include exact scope and delegation witnesses".into());
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "mobility".into(),
            id: value.into(),
        }
    }

    fn witness(value: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: value.into(),
        }
    }

    fn valid_provenance() -> QualificationDependencyBindingProvenance {
        let authority = id("authority");
        let authority_scope = witness("authority-scope");
        let authority_delegation = witness("authority-delegation");
        QualificationDependencyBindingProvenance {
            witness_identity: witness("binding-witness"),
            logical_identity: id("dependency"),
            authority: authority.clone(),
            authority_scope: authority_scope.clone(),
            authority_delegation: authority_delegation.clone(),
            basis: vec![
                authority,
                authority_scope,
                authority_delegation,
                id("basis"),
            ],
        }
    }

    #[test]
    fn accepts_well_formed_binding_provenance() {
        assert!(valid_provenance().validate().is_ok());
    }

    #[test]
    fn rejects_wrong_witness_kind() {
        let mut provenance = valid_provenance();
        provenance.witness_identity = id("not-a-witness");
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_empty_basis() {
        let mut provenance = valid_provenance();
        provenance.basis.clear();
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_duplicate_basis() {
        let mut provenance = valid_provenance();
        provenance.basis.push(provenance.basis[0].clone());
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_witness_identity_equal_to_authority() {
        let mut provenance = valid_provenance();
        provenance.witness_identity = provenance.authority.clone();
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_witness_identity_in_its_own_basis() {
        let mut provenance = valid_provenance();
        provenance.basis.push(provenance.witness_identity.clone());
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_authority_scope_without_reconciliation_witness_kind() {
        let mut provenance = valid_provenance();
        provenance.authority_scope = id("scope");
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_authority_delegation_without_reconciliation_witness_kind() {
        let mut provenance = valid_provenance();
        provenance.authority_delegation = id("delegation");
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_identity_reuse_across_provenance_roles() {
        let mut provenance = valid_provenance();
        provenance.authority_delegation = provenance.authority_scope.clone();
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_logical_dependency_as_its_own_basis() {
        let mut provenance = valid_provenance();
        provenance.basis.push(provenance.logical_identity.clone());
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn rejects_authority_omission_from_basis() {
        let mut provenance = valid_provenance();
        provenance.basis.retain(|basis| basis != &provenance.authority);
        assert!(provenance.validate().is_err());
    }

    #[test]
    fn provenance_roundtrips_as_closed_json() {
        let provenance = valid_provenance();
        let encoded = serde_json::to_string(&provenance).expect("provenance must serialize");
        let decoded: QualificationDependencyBindingProvenance =
            serde_json::from_str(&encoded).expect("provenance must deserialize");
        assert_eq!(decoded, provenance);

        let mut object: serde_json::Value =
            serde_json::from_str(&encoded).expect("serialized provenance must be JSON");
        object["unexpected"] = serde_json::json!("reject");
        assert!(serde_json::from_value::<QualificationDependencyBindingProvenance>(object).is_err());
    }

    #[test]
    fn rejects_logical_identity_mismatch_when_bound() {
        let provenance = valid_provenance();
        assert!(!provenance.matches_logical_identity(&id("other")));
        assert!(provenance.matches_logical_identity(&id("dependency")));
    }
}
