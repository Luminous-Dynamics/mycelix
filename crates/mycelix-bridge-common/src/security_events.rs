// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Structured, append-only-friendly security evidence vocabulary.
//!
//! These types describe what happened; they do not make an authorization true.
//! A SecurityEvent may therefore record Allow, Deny, or Indeterminate outcomes.
//! Provenance records origin and transformation context without asserting that
//! the underlying claim is true.

use serde::{
    Deserialize, Deserializer, Serialize,
    de::{Error as _, SeqAccess, Visitor},
};

use crate::security_kernel::{AuthorizationDecision, AuthorizationRequest, EnforcementRequest};

pub const MAX_PROVENANCE_IDENTIFIER_BYTES: usize = 512;
/// Resource bound for provenance edges accepted from a security-event wire envelope.
pub const MAX_SECURITY_EVENT_PROVENANCE_REFS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(try_from = "ProvenanceRefWire")]
pub struct ProvenanceRef {
    artifact_id: String,
    relation: ProvenanceRelation,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProvenanceRefWire {
    #[serde(deserialize_with = "deserialize_provenance_identifier")]
    artifact_id: String,
    relation: ProvenanceRelation,
}

impl TryFrom<ProvenanceRefWire> for ProvenanceRef {
    type Error = &'static str;

    fn try_from(wire: ProvenanceRefWire) -> Result<Self, Self::Error> {
        Self::new(wire.artifact_id, wire.relation)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProvenanceRelation {
    DerivedFrom,
    TransformedFrom,
    Summarizes,
    References,
    Supersedes,
}

impl ProvenanceRef {
    pub fn new(
        artifact_id: impl Into<String>,
        relation: ProvenanceRelation,
    ) -> Result<Self, &'static str> {
        let artifact_id = artifact_id.into();
        if artifact_id.is_empty() {
            return Err("provenance artifact identifier cannot be empty");
        }
        if artifact_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
            return Err("provenance artifact identifier exceeds size limit");
        }
        Ok(Self {
            artifact_id,
            relation,
        })
    }

    pub fn artifact_id(&self) -> &str {
        &self.artifact_id
    }

    pub fn relation(&self) -> ProvenanceRelation {
        self.relation
    }
}

/// Decision vocabulary used by security-event audit records.
///
/// This is separate from AuthorizationDecision: successful authorization is
/// represented by an opaque permit, while Allow here records live enforcement.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SecurityEventDecision {
    Allow,
    Deny(crate::security_kernel::AuthorizationDenial),
    Indeterminate(crate::security_kernel::AuthorizationIndeterminacy),
}

/// A structured record of an authorization-relevant security event.
///
/// This is evidence for reconstruction and audit. It is intentionally not an
/// authority primitive and must not be used as a substitute for a fresh
/// authorization decision.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct SecurityEvent {
    event_id: String,
    actor_id: String,
    capability_ref: String,
    request: AuthorizationRequest,
    decision: SecurityEventDecision,
    policy_version: u64,
    timestamp_us: u64,
    /// Kernel-derived commitment of the exact capability used for a successful
    /// enforcement request. Directly constructed events may leave this absent.
    #[serde(default)]
    capability_binding: Option<[u8; 32]>,
    /// Opaque commitment for the exact authority generation/freshness state
    /// that qualified a successful enforcement request.
    #[serde(default)]
    authority_binding: Option<[u8; 32]>,
    provenance: Vec<ProvenanceRef>,
    recovery_correlation: Option<String>,
}

/// Bound provenance/security identifiers before retaining attacker-controlled wire strings.
fn deserialize_provenance_identifier<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: Deserializer<'de>,
{
    struct ProvenanceIdentifierVisitor;

    impl<'de> Visitor<'de> for ProvenanceIdentifierVisitor {
        type Value = String;

        fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            formatter.write_str("a bounded provenance/security identifier string")
        }

        fn visit_borrowed_str<E>(self, value: &'de str) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            if value.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
                return Err(E::custom(
                    "provenance/security identifier exceeds size limit",
                ));
            }
            Ok(value.to_owned())
        }

        fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            if value.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
                return Err(E::custom(
                    "provenance/security identifier exceeds size limit",
                ));
            }
            Ok(value.to_owned())
        }

        fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            if value.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
                return Err(E::custom(
                    "provenance/security identifier exceeds size limit",
                ));
            }
            Ok(value)
        }
    }

    deserializer.deserialize_str(ProvenanceIdentifierVisitor)
}

/// Bound an optional provenance/security identifier during wire decoding.
fn deserialize_optional_provenance_identifier<'de, D>(
    deserializer: D,
) -> Result<Option<String>, D::Error>
where
    D: Deserializer<'de>,
{
    struct OptionalProvenanceIdentifierVisitor;

    impl<'de> Visitor<'de> for OptionalProvenanceIdentifierVisitor {
        type Value = Option<String>;

        fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            formatter.write_str("null or a bounded provenance/security identifier string")
        }

        fn visit_none<E>(self) -> Result<Self::Value, E>
        where
            E: serde::de::Error,
        {
            Ok(None)
        }

        fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
        where
            D: Deserializer<'de>,
        {
            deserialize_provenance_identifier(deserializer).map(Some)
        }
    }

    deserializer.deserialize_option(OptionalProvenanceIdentifierVisitor)
}

/// Bound provenance sequences before retaining an attacker-controlled
/// number of references from wire input.
fn deserialize_provenance_refs<'de, D>(deserializer: D) -> Result<Vec<ProvenanceRef>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    struct ProvenanceRefsVisitor;

    impl<'de> Visitor<'de> for ProvenanceRefsVisitor {
        type Value = Vec<ProvenanceRef>;

        fn expecting(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            formatter.write_str("a bounded provenance-reference sequence")
        }

        fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
        where
            A: SeqAccess<'de>,
        {
            let mut provenance = Vec::with_capacity(MAX_SECURITY_EVENT_PROVENANCE_REFS);
            while let Some(reference) = seq.next_element()? {
                if provenance.len() >= MAX_SECURITY_EVENT_PROVENANCE_REFS {
                    return Err(A::Error::custom(
                        "security event provenance exceeds size limit",
                    ));
                }
                provenance.push(reference);
            }
            Ok(provenance)
        }
    }

    deserializer.deserialize_seq(ProvenanceRefsVisitor)
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SecurityEventWire {
    #[serde(deserialize_with = "deserialize_provenance_identifier")]
    event_id: String,
    #[serde(deserialize_with = "deserialize_provenance_identifier")]
    actor_id: String,
    #[serde(deserialize_with = "deserialize_provenance_identifier")]
    capability_ref: String,
    request: AuthorizationRequest,
    decision: AuthorizationDecision,
    policy_version: u64,
    timestamp_us: u64,
    #[serde(default)]
    capability_binding: Option<[u8; 32]>,
    #[serde(default)]
    authority_binding: Option<[u8; 32]>,
    #[serde(deserialize_with = "deserialize_provenance_refs")]
    provenance: Vec<ProvenanceRef>,
    #[serde(
        default,
        deserialize_with = "deserialize_optional_provenance_identifier"
    )]
    recovery_correlation: Option<String>,
}

impl<'de> serde::Deserialize<'de> for SecurityEvent {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = SecurityEventWire::deserialize(deserializer)?;
        let mut event = Self::new_internal(
            wire.event_id,
            wire.actor_id,
            wire.capability_ref,
            wire.request,
            wire.decision.clone(),
            wire.policy_version,
            wire.timestamp_us,
        )
        .map_err(D::Error::custom)?;

        if matches!(&wire.decision, SecurityEventDecision::Allow) {
            return Err(D::Error::custom(
                "Allow security events must be created from a live EnforcementRequest",
            ));
        }
        if wire.capability_binding.is_some() || wire.authority_binding.is_some() {
            return Err(D::Error::custom(
                "non-Allow security events cannot carry enforcement bindings",
            ));
        }

        event.capability_binding = wire.capability_binding;
        event.authority_binding = wire.authority_binding;
        event.provenance = wire.provenance;
        if let Some(correlation) = wire.recovery_correlation {
            event = event
                .with_recovery_correlation(correlation)
                .map_err(D::Error::custom)?;
        }
        Ok(event)
    }
}

impl SecurityEvent {
    pub fn event_id(&self) -> &str {
        &self.event_id
    }

    pub fn actor_id(&self) -> &str {
        &self.actor_id
    }

    pub fn capability_ref(&self) -> &str {
        &self.capability_ref
    }

    pub fn request(&self) -> &AuthorizationRequest {
        &self.request
    }

    pub fn decision(&self) -> &SecurityEventDecision {
        &self.decision
    }

    pub fn policy_version(&self) -> u64 {
        self.policy_version
    }

    pub fn timestamp_us(&self) -> u64 {
        self.timestamp_us
    }

    pub fn capability_binding(&self) -> Option<[u8; 32]> {
        self.capability_binding
    }

    pub fn authority_binding(&self) -> Option<[u8; 32]> {
        self.authority_binding
    }

    pub fn provenance(&self) -> &[ProvenanceRef] {
        &self.provenance
    }

    pub fn recovery_correlation(&self) -> Option<&str> {
        self.recovery_correlation.as_deref()
    }

    fn new_internal(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        request: AuthorizationRequest,
        decision: SecurityEventDecision,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let event_id = event_id.into();
        let actor_id = actor_id.into();
        let capability_ref = capability_ref.into();

        if event_id.is_empty() || actor_id.is_empty() || capability_ref.is_empty() {
            return Err("security event identifiers cannot be empty");
        }
        if event_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
            || actor_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
            || capability_ref.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
        {
            return Err("security event identifier exceeds size limit");
        }
        if policy_version != request.policy_version() {
            return Err("security event policy version does not match request");
        }

        Ok(Self {
            event_id,
            actor_id,
            capability_ref,
            request,
            decision,
            policy_version,
            timestamp_us,
            capability_binding: None,
            authority_binding: None,
            provenance: Vec::new(),
            recovery_correlation: None,
        })
    }

    pub fn new(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        request: AuthorizationRequest,
        decision: AuthorizationDecision,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let decision = match decision {
            SecurityEventDecision::Deny(reason) => SecurityEventDecision::Deny(reason),
            AuthorizationDecision::Indeterminate(reason) => {
                SecurityEventDecision::Indeterminate(reason)
            }
        };
        Self::new_internal(
            event_id,
            actor_id,
            capability_ref,
            request,
            decision,
            policy_version,
            timestamp_us,
        )
    }

    pub fn with_provenance(mut self, provenance: Vec<ProvenanceRef>) -> Self {
        self.provenance = provenance;
        self
    }

    pub fn with_recovery_correlation(
        mut self,
        correlation: impl Into<String>,
    ) -> Result<Self, &'static str> {
        let correlation = correlation.into();
        if correlation.is_empty() {
            return Err("recovery correlation cannot be empty");
        }
        if correlation.len() > MAX_PROVENANCE_IDENTIFIER_BYTES {
            return Err("recovery correlation exceeds size limit");
        }
        self.recovery_correlation = Some(correlation);
        Ok(self)
    }

    /// Record the exact request carried by an enforcement permit.
    ///
    /// This does not re-authorize the operation; enforcement should already
    /// have received an EnforcementRequest derived from the permit. The event
    /// timestamp must equal the revalidation timestamp carried by that request.
    pub fn from_enforcement_request(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        enforcement: &EnforcementRequest,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let actor_id = actor_id.into();
        if actor_id != enforcement.request().subject() {
            return Err("security event actor does not match enforcement subject");
        }
        if policy_version != enforcement.request().policy_version() {
            return Err("security event policy version does not match enforcement request");
        }
        if timestamp_us != enforcement.revalidated_at_us() {
            return Err("security event timestamp does not match enforcement revalidation time");
        }
        let mut event = Self::new_internal(
            event_id,
            actor_id,
            capability_ref,
            enforcement.request().clone(),
            SecurityEventDecision::Allow,
            policy_version,
            timestamp_us,
        )?;
        event.capability_binding = Some(enforcement.capability_binding());
        event.authority_binding = Some(enforcement.authority_binding());
        Ok(event)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::security_kernel::CapabilityAction;

    #[test]
    fn provenance_ref_rejects_empty_and_oversized_ids() {
        assert!(ProvenanceRef::new("", ProvenanceRelation::References).is_err());
        assert!(
            ProvenanceRef::new(
                "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES + 1),
                ProvenanceRelation::References
            )
            .is_err()
        );
    }

    #[test]
    fn provenance_ref_deserialization_is_constructor_gated() {
        let valid: ProvenanceRef =
            serde_json::from_str(r#"{"artifact_id":"evidence:source-1","relation":"DerivedFrom"}"#)
                .unwrap();
        assert_eq!(valid.artifact_id, "evidence:source-1");

        let empty: Result<ProvenanceRef, _> =
            serde_json::from_str(r#"{"artifact_id":"","relation":"References"}"#);
        assert!(empty.is_err());

        let oversized: Result<ProvenanceRef, _> = serde_json::from_str(&format!(
            r#"{{"artifact_id":"{}","relation":"References"}}"#,
            "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES + 1)
        ));
        assert!(oversized.is_err());

        let unknown: Result<ProvenanceRef, _> = serde_json::from_str(
            r#"{"artifact_id":"evidence:source-1","relation":"References","authority":"ignored"}"#,
        );
        assert!(unknown.is_err());
    }

    #[test]
    fn security_event_deserialization_rejects_excessive_provenance() {
        let provenance: String = (0..=MAX_SECURITY_EVENT_PROVENANCE_REFS)
            .map(|index| format!(r#"{{"artifact_id":"evidence:{index}","relation":"References"}}"#))
            .collect::<Vec<_>>()
            .join(",");

        let json = format!(
            r#"{{
                "event_id":"event:provenance-limit",
                "actor_id":"did:mycelix:alice",
                "capability_ref":"capability:1",
                "request":{{
                    "subject":"did:mycelix:alice",
                    "resource":"resource:ledger",
                    "action":"Read",
                    "policy_version":7
                }},
                "decision":{{"Deny":"ActionNotGranted"}},
                "policy_version":7,
                "timestamp_us":151,
                "provenance":[{provenance}],
                "recovery_correlation":null
            }}"#
        );

        assert!(serde_json::from_str::<SecurityEvent>(&json).is_err());
    }

    #[test]
    fn security_event_rejects_unknown_wire_fields() {
        let json = r#"{
            "event_id":"event:unknown",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:1",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":{"Deny":"ActionNotGranted"},
            "policy_version":7,
            "timestamp_us":151,
            "provenance":[],
            "recovery_correlation":null,
            "authority":"ignored"
        }"#;
        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn security_event_deserialization_preserves_recovery_correlation() {
        let json = r#"{
            "event_id":"event:recovery",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:1",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":{"Deny":"ActionNotGranted"},
            "policy_version":7,
            "timestamp_us":151,
            "provenance":[],
            "recovery_correlation":"recovery:1"
        }"#;

        let event: SecurityEvent = serde_json::from_str(json).unwrap();
        assert_eq!(event.recovery_correlation(), Some("recovery:1"));
    }

    #[test]
    fn security_event_wire_identifier_exact_limit_is_accepted() {
        let boundary = "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES);
        let json = format!(
            r#"{{"event_id":"{id}","actor_id":"did:mycelix:alice","capability_ref":"capability:1","request":{{"subject":"did:mycelix:alice","resource":"resource:ledger","action":"Read","policy_version":7}},"decision":{{"Deny":"ActionNotGranted"}},"policy_version":7,"timestamp_us":151,"provenance":[],"recovery_correlation":null}}"#,
            id = boundary,
        );

        let event: SecurityEvent = serde_json::from_str(&json).unwrap();
        assert_eq!(event.event_id().len(), MAX_PROVENANCE_IDENTIFIER_BYTES);
    }

    #[test]
    fn provenance_ref_wire_identifier_exact_limit_is_accepted() {
        let boundary = "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES);
        let json = format!(
            r#"{{"artifact_id":"{id}","relation":"References"}}"#,
            id = boundary,
        );

        let decoded: ProvenanceRef = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.artifact_id().len(), MAX_PROVENANCE_IDENTIFIER_BYTES);
    }

    #[test]
    fn security_event_deserialization_rejects_invalid_recovery_correlation() {
        let base = |correlation: &str| {
            format!(
                r#"{{
                    "event_id":"event:recovery-invalid",
                    "actor_id":"did:mycelix:alice",
                    "capability_ref":"capability:1",
                    "request":{{
                        "subject":"did:mycelix:alice",
                        "resource":"resource:ledger",
                        "action":"Read",
                        "policy_version":7
                    }},
                    "decision":{{"Deny":"ActionNotGranted"}},
                    "policy_version":7,
                    "timestamp_us":151,
                    "provenance":[],
                    "recovery_correlation":{correlation}
                }}"#,
                correlation = serde_json::to_string(correlation).unwrap()
            )
        };

        assert!(serde_json::from_str::<SecurityEvent>(&base("")).is_err());

        let boundary = "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES);
        assert!(serde_json::from_str::<SecurityEvent>(&base(&boundary)).is_ok());

        let oversized = "x".repeat(MAX_PROVENANCE_IDENTIFIER_BYTES + 1);
        assert!(serde_json::from_str::<SecurityEvent>(&base(&oversized)).is_err());
    }

    #[test]
    fn security_event_preserves_allow_decision_and_lineage() {
        let enforcement = crate::security_kernel::test_enforcement_request();
        let source =
            ProvenanceRef::new("evidence:source-1", ProvenanceRelation::DerivedFrom).unwrap();

        let event = SecurityEvent::from_enforcement_request(
            "event:1",
            "did:mycelix:alice",
            "capability:1",
            &enforcement,
            7,
            150,
        )
        .unwrap()
        .with_provenance(vec![source])
        .with_recovery_correlation("recovery:1")
        .unwrap();

        assert_eq!(event.decision(), &SecurityEventDecision::Allow);
        assert_eq!(event.request(), enforcement.request());
        assert_eq!(event.provenance().len(), 1);
        assert_eq!(event.recovery_correlation(), Some("recovery:1"));
        assert_eq!(
            event.capability_binding(),
            Some(enforcement.capability_binding())
        );
        assert_eq!(
            event.authority_binding(),
            Some(enforcement.authority_binding())
        );
    }

    #[test]
    fn enforcement_event_rejects_actor_mismatch() {
        let enforcement = crate::security_kernel::test_enforcement_request();

        assert_eq!(
            SecurityEvent::from_enforcement_request(
                "event:actor-mismatch",
                "did:mycelix:bob",
                "capability:read",
                &enforcement,
                7,
                150,
            )
            .unwrap_err(),
            "security event actor does not match enforcement subject"
        );
    }

    #[test]
    fn legacy_security_event_without_binding_deserializes_as_deny() {
        let json = r#"{
            "event_id":"event:legacy",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:legacy",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":{
                "Deny":"ActionNotGranted"
            },
            "policy_version":7,
            "timestamp_us":151,
            "provenance":[],
            "recovery_correlation":null
        }"#;
        let event: SecurityEvent = serde_json::from_str(json).unwrap();
        assert_eq!(
            event.decision(),
            &SecurityEventDecision::Deny(
                crate::security_kernel::AuthorizationDenial::ActionNotGranted
            )
        );
        assert_eq!(event.capability_binding(), None);
        assert_eq!(event.authority_binding(), None);
    }

    #[test]
    fn deserialization_rejects_unbound_allow() {
        let json = r#"{
            "event_id":"event:forged-allow",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:forged",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":"Allow",
            "policy_version":7,
            "timestamp_us":151,
            "provenance":[],
            "recovery_correlation":null
        }"#;

        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn deserialization_rejects_non_allow_bindings() {
        let json = r#"{
            "event_id":"event:forged-deny-binding",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:forged",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":{
                "Deny":"ActionNotGranted"
            },
            "policy_version":7,
            "timestamp_us":151,
            "capability_binding":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            "authority_binding":[2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2],
            "provenance":[],
            "recovery_correlation":null
        }"#;

        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn deserialization_rejects_allow_even_with_bindings() {
        let json = r#"{
            "event_id":"event:forged-allow",
            "actor_id":"did:mycelix:alice",
            "capability_ref":"capability:forged",
            "request":{
                "subject":"did:mycelix:alice",
                "resource":"resource:ledger",
                "action":"Read",
                "policy_version":7
            },
            "decision":"Allow",
            "policy_version":7,
            "timestamp_us":151,
            "capability_binding":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
            "authority_binding":[2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2],
            "provenance":[],
            "recovery_correlation":null
        }"#;

        assert!(serde_json::from_str::<SecurityEvent>(json).is_err());
    }

    #[test]
    fn serialized_enforcement_event_is_not_deserializable_as_authority() {
        let enforcement = crate::security_kernel::test_enforcement_request();
        let event = SecurityEvent::from_enforcement_request(
            "event:serialized-allow",
            "did:mycelix:alice",
            "capability:1",
            &enforcement,
            7,
            150,
        )
        .unwrap();

        let encoded = serde_json::to_string(&event).unwrap();
        assert!(serde_json::from_str::<SecurityEvent>(&encoded).is_err());
    }

    #[test]
    fn serialized_deny_event_round_trips() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();
        let event = SecurityEvent::new(
            "event:deny-round-trip",
            "did:mycelix:alice",
            "capability:1",
            request,
            AuthorizationDecision::Deny(
                crate::security_kernel::AuthorizationDenial::ActionNotGranted,
            ),
            7,
            151,
        )
        .unwrap();

        let encoded = serde_json::to_string(&event).unwrap();
        let decoded: SecurityEvent = serde_json::from_str(&encoded).unwrap();

        assert_eq!(decoded, event);
        assert_eq!(
            decoded.decision(),
            &AuthorizationDecision::Deny(
                crate::security_kernel::AuthorizationDenial::ActionNotGranted
            )
        );
    }

    #[test]
    fn enforcement_event_rejects_policy_version_mismatch() {
        let enforcement = crate::security_kernel::test_enforcement_request();

        assert_eq!(
            SecurityEvent::from_enforcement_request(
                "event:mismatch",
                "did:mycelix:alice",
                "capability:1",
                &enforcement,
                8,
                151,
            )
            .unwrap_err(),
            "security event policy version does not match enforcement request"
        );
    }

    #[test]
    fn security_event_constructor_rejects_policy_version_mismatch() {
        let request = crate::security_kernel::AuthorizationRequest::new(
            "did:mycelix:alice",
            "resource:ledger",
            CapabilityAction::Read,
            7,
        )
        .unwrap();

        assert_eq!(
            SecurityEvent::new(
                "event:mismatch",
                "did:mycelix:alice",
                "capability:1",
                request,
                AuthorizationDecision::Deny(
                    crate::security_kernel::AuthorizationDenial::ActionNotGranted,
                ),
                8,
                150,
            )
            .unwrap_err(),
            "security event policy version does not match request"
        );
    }
}
