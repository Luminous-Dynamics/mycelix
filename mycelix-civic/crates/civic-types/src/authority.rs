// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Persistent, delegated civic authority primitives.
//!
//! Proceeding roles answer "what part does this party play here?". Authority grants
//! answer the separate administrative question "what may this actor do, where, and
//! under which legal or policy basis?".

use hdk::prelude::Timestamp;
use serde::{Deserialize, Serialize};

/// Source from which a civic authority grant derives its legitimacy.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum AuthorityBasisKind {
    Charter,
    Statute,
    Ordinance,
    Regulation,
    Policy,
    Delegation,
    EmergencyDeclaration,
    CourtOrder,
    Other,
}

/// Human- and machine-auditable reference to the source of authority.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AuthorityBasis {
    pub kind: AuthorityBasisKind,
    /// Stable external or local reference, such as an ordinance identifier.
    pub reference: String,
    /// Optional body that issued or adopted the authority source.
    pub issuing_body: Option<String>,
}

/// Boundaries within which a grant may be exercised.
///
/// Optional fields are wildcards when absent. A grant scoped to a department,
/// geography, or resource only matches requests carrying that exact scope value.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AuthorityScope {
    /// Municipality, district, authority, or other governing jurisdiction identifier.
    pub jurisdiction: String,
    pub department: Option<String>,
    pub geography: Option<String>,
    pub resource: Option<String>,
}

impl AuthorityScope {
    /// Returns true when the requested operation is within this scope.
    pub fn contains(
        &self,
        jurisdiction: &str,
        department: Option<&str>,
        geography: Option<&str>,
        resource: Option<&str>,
    ) -> bool {
        self.jurisdiction == jurisdiction
            && optional_scope_matches(&self.department, department)
            && optional_scope_matches(&self.geography, geography)
            && optional_scope_matches(&self.resource, resource)
    }
}

fn optional_scope_matches(granted: &Option<String>, requested: Option<&str>) -> bool {
    match granted.as_deref() {
        None => true,
        Some(value) => requested == Some(value),
    }
}

/// A persistent grant of civic authority to a DID.
///
/// Time validity uses a half-open interval: `valid_from <= t < valid_until`.
/// Revocation takes effect at `revoked_at`.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AuthorityGrant {
    pub id: String,
    pub grantor_did: String,
    pub grantee_did: String,
    pub basis: AuthorityBasis,
    /// Optional parent grant for auditable delegation chains.
    pub parent_grant_id: Option<String>,
    /// Capability identifiers intentionally remain domain-neutral strings so new civic
    /// domains can add capabilities without changing this shared crate.
    pub capabilities: Vec<String>,
    pub scope: AuthorityScope,
    pub valid_from: Timestamp,
    pub valid_until: Option<Timestamp>,
    pub revoked_at: Option<Timestamp>,
    /// Whether the grantee may issue narrower child grants from this authority.
    pub delegable: bool,
}

impl AuthorityGrant {
    /// Returns true when this grant is temporally valid and has not been revoked.
    pub fn is_active_at(&self, at: Timestamp) -> bool {
        let at = at.as_micros();

        if at < self.valid_from.as_micros() {
            return false;
        }

        if let Some(valid_until) = &self.valid_until {
            if at >= valid_until.as_micros() {
                return false;
            }
        }

        if let Some(revoked_at) = &self.revoked_at {
            if at >= revoked_at.as_micros() {
                return false;
            }
        }

        true
    }

    /// Checks capability, scope, and time in one fail-closed authorization predicate.
    pub fn authorizes(
        &self,
        capability: &str,
        jurisdiction: &str,
        department: Option<&str>,
        geography: Option<&str>,
        resource: Option<&str>,
        at: Timestamp,
    ) -> bool {
        self.is_active_at(at)
            && self.capabilities.iter().any(|item| item == capability)
            && self
                .scope
                .contains(jurisdiction, department, geography, resource)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            id: "grant-1".into(),
            grantor_did: "did:mycelix:city".into(),
            grantee_did: "did:mycelix:alice".into(),
            basis: AuthorityBasis {
                kind: AuthorityBasisKind::Ordinance,
                reference: "ORD-2026-17".into(),
                issuing_body: Some("city-council".into()),
            },
            parent_grant_id: None,
            capabilities: vec!["public-works.work-order.approve".into()],
            scope: AuthorityScope {
                jurisdiction: "reference-city".into(),
                department: Some("public-works".into()),
                geography: Some("district-4".into()),
                resource: None,
            },
            valid_from: Timestamp::from_micros(100),
            valid_until: Some(Timestamp::from_micros(200)),
            revoked_at: None,
            delegable: false,
        }
    }

    #[test]
    fn validity_interval_is_half_open() {
        let grant = grant();
        assert!(!grant.is_active_at(Timestamp::from_micros(99)));
        assert!(grant.is_active_at(Timestamp::from_micros(100)));
        assert!(grant.is_active_at(Timestamp::from_micros(199)));
        assert!(!grant.is_active_at(Timestamp::from_micros(200)));
    }

    #[test]
    fn revocation_is_effective_at_revocation_timestamp() {
        let mut grant = grant();
        grant.revoked_at = Some(Timestamp::from_micros(150));
        assert!(grant.is_active_at(Timestamp::from_micros(149)));
        assert!(!grant.is_active_at(Timestamp::from_micros(150)));
    }

    #[test]
    fn authorization_is_capability_and_scope_bound() {
        let grant = grant();
        let at = Timestamp::from_micros(120);

        assert!(grant.authorizes(
            "public-works.work-order.approve",
            "reference-city",
            Some("public-works"),
            Some("district-4"),
            Some("water-main-22"),
            at,
        ));
        assert!(!grant.authorizes(
            "public-works.work-order.approve",
            "reference-city",
            Some("public-works"),
            Some("district-5"),
            None,
            at,
        ));
        assert!(!grant.authorizes(
            "public-works.work-order.delete",
            "reference-city",
            Some("public-works"),
            Some("district-4"),
            None,
            at,
        ));
    }
}
