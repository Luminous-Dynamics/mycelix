//! Offline, fail-closed validation for civilian awareness evidence envelopes.
//!
//! The validator checks structure, field allowlists, canonical digests and
//! evidence-quality declarations. It does not authenticate an external source,
//! verify a Mycelix authority receipt, adjudicate physical truth, or authorize
//! an operational action. Even a structurally valid record remains blocked
//! pending the independently verified receipt path.

use std::collections::BTreeSet;

use chrono::DateTime;
use jsonschema::Validator;
use serde::de::{self, MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Number, Value};
use sha2::{Digest, Sha256};

pub mod observability;

pub const MAX_ENVELOPE_BYTES: usize = 2 * 1024 * 1024;
pub const SCHEMA_JSON: &str =
    include_str!("../../../docs/research/civil-awareness-evidence-envelope-v0.1.schema.json");
pub const FIELD_POLICY_JSON: &str =
    include_str!("../../../docs/research/civil-awareness-field-policy-v0.1.json");
pub const EXAMPLE_FIXTURES_JSON: &str =
    include_str!("../../../docs/research/civil-awareness-envelope-examples-v0.1.json");

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Disposition {
    /// The JSON Schema, digest, allowlist or immutable structural contract failed.
    Rejected,
    /// Rights, authorization or privacy state blocks admission.
    Quarantined,
    /// Evidence may be retained, but one or more quality/provenance conditions
    /// are insufficient for even offline replay comparison.
    Unresolved,
    /// Structure and declared quality gates pass, but an independently
    /// authenticated authority/evidence receipt is still required.
    StructurallyValidRequiresAuthorityReceipt,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ValidationReport {
    pub disposition: Disposition,
    /// Stable, value-free machine reason codes. Never contains source values.
    pub reason_codes: Vec<String>,
    pub schema_error_count: usize,
    pub field_policy_digest: String,
}

impl ValidationReport {
    fn new(
        disposition: Disposition,
        reasons: BTreeSet<String>,
        schema_error_count: usize,
        digest: String,
    ) -> Self {
        Self {
            disposition,
            reason_codes: reasons.into_iter().collect(),
            schema_error_count,
            field_policy_digest: digest,
        }
    }
}

#[derive(Debug)]
pub struct EnvelopeValidator {
    schema_validator: Validator,
    policy: Value,
    field_policy_digest: String,
}

impl EnvelopeValidator {
    /// Compile only the repository-pinned schema and policy. No URL resolution
    /// or network access is performed by this API.
    pub fn from_embedded() -> Result<Self, String> {
        Self::from_documents(SCHEMA_JSON, FIELD_POLICY_JSON)
    }

    pub fn from_documents(schema_text: &str, policy_text: &str) -> Result<Self, String> {
        let schema: Value = parse_strict_json(schema_text.as_bytes())
            .map_err(|_| "pinned schema JSON is invalid".to_string())?;
        let policy: Value = parse_strict_json(policy_text.as_bytes())
            .map_err(|_| "pinned policy JSON is invalid".to_string())?;

        if string_at(&policy, "/policy_version") != Some("0.1.0") {
            return Err("policy.version_unsupported".into());
        }
        if string_at(&schema, "/$id")
            != Some("urn:luminous-dynamics:gcpn:civil-awareness-evidence-envelope:0.1.0")
            || string_at(&schema, "/$schema")
                != Some("https://json-schema.org/draft/2020-12/schema")
        {
            return Err("schema.identity_or_draft_unsupported".into());
        }
        if string_at(&policy, "/canonicalization/encoding") != Some("UTF-8")
            || string_at(&policy, "/canonicalization/digest_algorithm") != Some("SHA-256")
            || string_at(&policy, "/canonicalization/json_payload_serialization")
                != Some("RFC 8785 JSON Canonicalization Scheme")
            || string_at(&policy, "/canonicalization/digest_format")
                != Some("sha256:<64 lowercase hexadecimal characters>")
            || string_at(
                &policy,
                "/canonicalization/digest_semantics/field_allowlist_digest",
            ) != Some("SHA-256 over the exact UTF-8 bytes of this field-policy artifact.")
            || string_at(&policy, "/canonicalization/digest_semantics/payload_digest")
                != Some("SHA-256 over RFC 8785 canonical JSON bytes of domain_extension.payload.")
            || string_at(
                &policy,
                "/canonicalization/digest_semantics/envelope_content_digest",
            ) != Some(
                "SHA-256 over RFC 8785 canonical JSON bytes of the entire envelope object after removing only lineage.envelope_content_digest; this avoids self-reference while binding payload_digest, correction/supersession links and every other field.",
            )
        {
            return Err("policy.digest_contract_unsupported".into());
        }
        if schema_has_external_refs(&schema) {
            return Err("schema.external_reference_forbidden".into());
        }
        if !jsonschema::meta::is_valid(&schema) {
            return Err("schema.meta_validation_failed".into());
        }
        let schema_validator = jsonschema::draft202012::options()
            .should_validate_formats(true)
            .build(&schema)
            .map_err(|_| "schema.compile_failed".to_string())?;

        let field_policy_digest = format!("sha256:{}", sha256_hex(policy_text.as_bytes()));
        Ok(Self {
            schema_validator,
            policy,
            field_policy_digest,
        })
    }

    pub fn field_policy_digest(&self) -> &str {
        &self.field_policy_digest
    }

    /// Parse with duplicate-property rejection, then validate. Oversized input
    /// is rejected before schema work. Error reports never echo raw values.
    pub fn validate_bytes(&self, bytes: &[u8]) -> ValidationReport {
        if bytes.len() > MAX_ENVELOPE_BYTES {
            return self.reject("input.too_large");
        }
        let value = match parse_strict_json(bytes) {
            Ok(value) => value,
            Err(_) => return self.reject("json.parse_or_duplicate_key_failure"),
        };
        self.validate_value(&value)
    }

    /// Validate a value that has already passed the strict parser. This stays
    /// private so external callers cannot bypass duplicate-key rejection by
    /// first parsing untrusted JSON with a last-key-wins deserializer.
    fn validate_value(&self, envelope: &Value) -> ValidationReport {
        let schema_error_count = self.schema_validator.iter_errors(envelope).count();
        if schema_error_count != 0 {
            return ValidationReport::new(
                Disposition::Rejected,
                ["schema.validation_failed".to_string()]
                    .into_iter()
                    .collect(),
                schema_error_count,
                self.field_policy_digest.clone(),
            );
        }

        let mut rejected = BTreeSet::new();
        let mut quarantined = BTreeSet::new();
        let mut unresolved = BTreeSet::new();

        let domain = string_at(envelope, "/domain").unwrap_or_default();
        let claim_type = string_at(envelope, "/claim/claim_type").unwrap_or_default();

        // Top-level allowlist is independently checked in addition to
        // additionalProperties:false in the JSON Schema.
        self.check_object_keys(
            Some(envelope),
            self.policy.pointer("/common_envelope_allowlist"),
            "envelope.field_not_allowlisted",
            &mut rejected,
        );

        // Domain extension identity and exact field allowlist.
        let expected_extension = self
            .policy
            .get("domain_extension_schemas")
            .and_then(|schemas| schemas.get(domain));
        let actual_uri = string_at(envelope, "/domain_extension/schema_uri");
        let actual_revision = string_at(envelope, "/domain_extension/schema_revision");
        if expected_extension.is_none()
            || actual_uri
                != expected_extension
                    .and_then(|v| v.get("schema_uri"))
                    .and_then(Value::as_str)
            || actual_revision
                != expected_extension
                    .and_then(|v| v.get("schema_revision"))
                    .and_then(Value::as_str)
        {
            rejected.insert("domain_extension.unknown_schema_or_revision".into());
        }
        self.check_object_keys(
            envelope.pointer("/domain_extension/payload"),
            self.policy
                .get("domain_extension_allowlists")
                .and_then(|allowlists| allowlists.get(domain)),
            "domain_extension.field_not_allowlisted",
            &mut rejected,
        );
        if envelope
            .pointer("/domain_extension/payload")
            .and_then(Value::as_object)
            .is_some_and(|object| {
                object
                    .values()
                    .any(|value| matches!(value, Value::Object(_) | Value::Array(_)))
            })
        {
            rejected.insert("domain_extension.nested_or_array_data_not_allowed_in_v0_1".into());
        }

        // claim.value is deliberately narrower than arbitrary JSON. v0.1
        // accepts a scalar or a flat object with domain+claim-specific keys.
        // Use map lookup rather than constructing a JSON Pointer from untrusted
        // claim_type text; slash/tilde characters must not alter the lookup path.
        let claim_allowlist = self
            .policy
            .get("claim_value_allowlists")
            .and_then(|domains| domains.get(domain))
            .and_then(|claims| claims.get(claim_type));
        if claim_allowlist.is_none() {
            rejected.insert("claim.unknown_domain_claim_type".into());
        }
        match envelope.pointer("/claim/value") {
            Some(Value::Object(value)) => {
                self.check_object_keys(
                    Some(&Value::Object(value.clone())),
                    claim_allowlist,
                    "claim.value_field_not_allowlisted",
                    &mut rejected,
                );
                if value
                    .values()
                    .any(|v| matches!(v, Value::Object(_) | Value::Array(_)))
                {
                    rejected.insert("claim.value_nested_or_array_data_not_allowed".into());
                }
            }
            Some(Value::Array(_)) => {
                rejected.insert("claim.value_array_not_allowed_in_v0_1".into());
            }
            Some(_) => {}
            None => {
                rejected.insert("claim.value_missing".into());
            }
        }

        // The field-policy digest binds the allowlists the validator actually
        // applies. The payload digest uses RFC 8785 JCS, not ordinary JSON text.
        if string_at(envelope, "/governance/field_allowlist_digest")
            != Some(self.field_policy_digest.as_str())
        {
            rejected.insert("digest.field_policy_mismatch".into());
        }

        if let Some(payload) = envelope.pointer("/domain_extension/payload") {
            match serde_json_canonicalizer::to_vec(payload) {
                Ok(bytes) => {
                    let expected = format!("sha256:{}", sha256_hex(&bytes));
                    if string_at(envelope, "/lineage/payload_digest") != Some(expected.as_str()) {
                        rejected.insert("digest.domain_payload_mismatch".into());
                    }
                }
                Err(_) => {
                    rejected.insert("digest.domain_payload_not_canonicalizable".into());
                }
            }
        }

        // Envelope digest covers every field, including correction and
        // supersession lineage, except its own digest field.
        let mut content_without_digest = envelope.clone();
        if let Some(lineage) = content_without_digest
            .get_mut("lineage")
            .and_then(Value::as_object_mut)
        {
            lineage.remove("envelope_content_digest");
        } else {
            rejected.insert("envelope.lineage_missing".into());
        }
        match serde_json_canonicalizer::to_vec(&content_without_digest) {
            Ok(bytes) => {
                let expected = format!("sha256:{}", sha256_hex(&bytes));
                if string_at(envelope, "/lineage/envelope_content_digest")
                    != Some(expected.as_str())
                {
                    rejected.insert("digest.envelope_content_mismatch".into());
                }
            }
            Err(_) => {
                rejected.insert("digest.envelope_not_canonicalizable".into());
            }
        }

        // Verify a valid event-time interval independently of JSON Schema.
        let start = string_at(envelope, "/timestamps/event_time/start")
            .and_then(|v| DateTime::parse_from_rfc3339(v).ok());
        let end = string_at(envelope, "/timestamps/event_time/end")
            .and_then(|v| DateTime::parse_from_rfc3339(v).ok());
        if matches!((start, end), (Some(a), Some(b)) if a > b) {
            rejected.insert("time.event_interval_reversed".into());
        }

        // Explicitly reject numeric values that fall outside the I-JSON /
        // IEEE-754 interoperability constraints used by RFC 8785.
        if !numbers_are_ijson_safe(envelope) {
            rejected.insert("json.number_outside_ijson_safe_range".into());
        }

        // Governance state is a gate, never just another weighted score.
        let authorization = string_at(envelope, "/governance/authorization_status");
        if authorization != Some("approved") {
            quarantined.insert("rights.authorization_not_approved".into());
        }
        if string_at(envelope, "/governance/privacy_state") == Some("quarantined") {
            quarantined.insert("privacy.quarantined".into());
        }
        let allowed_uses = envelope
            .pointer("/governance/allowed_uses")
            .and_then(Value::as_array);
        if allowed_uses.is_some_and(Vec::is_empty) {
            quarantined.insert("rights.allowed_uses_empty".into());
        }
        let declared_purpose = string_at(envelope, "/governance/purpose_code");
        if !allowed_uses.is_some_and(|uses| {
            declared_purpose.is_some_and(|purpose| {
                uses.iter()
                    .any(|use_code| use_code.as_str() == Some(purpose))
            })
        }) {
            quarantined.insert("rights.declared_purpose_not_allowed".into());
        }

        let retention_until = string_at(envelope, "/governance/retention_until")
            .and_then(|value| DateTime::parse_from_rfc3339(value).ok());
        let received_at = string_at(envelope, "/timestamps/received_at")
            .and_then(|value| DateTime::parse_from_rfc3339(value).ok());
        if matches!((retention_until, received_at), (Some(retention), Some(received)) if retention < received)
        {
            quarantined.insert("rights.retention_expired_at_ingest".into());
        }

        // Source and evidence quality. A self-declared "authenticated" value is
        // not cryptographic proof; the external receipt verifier remains needed.
        match string_at(envelope, "/source/integrity_status") {
            Some("failed") => {
                rejected.insert("source.integrity_failed".into());
            }
            Some("authenticated") => {
                if string_at(envelope, "/source/authentication_method").is_none() {
                    unresolved.insert("source.authentication_method_missing".into());
                }
            }
            _ => {
                unresolved.insert("source.authentication_unverified".into());
            }
        }
        if string_at(envelope, "/quality/source_health") != Some("healthy") {
            unresolved.insert("source.health_not_healthy".into());
        }
        if string_at(envelope, "/quality/freshness") != Some("current") {
            unresolved.insert("source.freshness_not_current".into());
        }
        if matches!(
            string_at(envelope, "/quality/calibration_status"),
            Some("stale" | "unknown")
        ) || matches!(
            string_at(envelope, "/claim/calibration_status"),
            Some("stale" | "unknown")
        ) {
            unresolved.insert("source.calibration_not_current".into());
        }
        if string_at(envelope, "/quality/missingness") != Some("complete") {
            unresolved.insert("evidence.missing_or_withheld".into());
        }
        if envelope.pointer("/quality/abstention/abstained") == Some(&Value::Bool(true)) {
            unresolved.insert("evidence.abstained".into());
        }
        match string_at(envelope, "/timestamps/clock_quality") {
            Some("disciplined") => {}
            Some("bounded") => {
                if envelope
                    .pointer("/timestamps/clock_uncertainty_ms")
                    .is_none()
                {
                    unresolved.insert("time.bounded_clock_uncertainty_missing".into());
                }
            }
            _ => {
                unresolved.insert("time.clock_not_trustworthy".into());
            }
        }

        let dependency_groups = envelope
            .pointer("/dependence/dependency_group_refs")
            .and_then(Value::as_array)
            .map_or(0, Vec::len);
        if dependency_groups == 0 {
            unresolved.insert("dependence.group_identity_missing".into());
        }
        if envelope.pointer("/dependence/lineage_complete") != Some(&Value::Bool(true)) {
            unresolved.insert("dependence.lineage_incomplete".into());
        }

        let spatial_precision = string_at(envelope, "/spatial_context/precision_class");
        let location_status = string_at(envelope, "/spatial_context/location_reference_status");
        if matches!(location_status, Some("present"))
            && envelope.pointer("/spatial_context/location_ref").is_none()
            && envelope
                .pointer("/spatial_context/coordinate_payload_ref")
                .is_none()
        {
            unresolved.insert("spatial.reference_missing".into());
        }
        if matches!(
            spatial_precision,
            Some("sector" | "site" | "operational_minimum")
        ) && envelope
            .pointer("/spatial_context/purpose_binding_ref")
            .is_none()
        {
            unresolved.insert("spatial.purpose_binding_missing".into());
        }

        if !rejected.is_empty() {
            return ValidationReport::new(
                Disposition::Rejected,
                rejected,
                schema_error_count,
                self.field_policy_digest.clone(),
            );
        }
        if !quarantined.is_empty() {
            return ValidationReport::new(
                Disposition::Quarantined,
                quarantined,
                schema_error_count,
                self.field_policy_digest.clone(),
            );
        }
        if !unresolved.is_empty() {
            return ValidationReport::new(
                Disposition::Unresolved,
                unresolved,
                schema_error_count,
                self.field_policy_digest.clone(),
            );
        }

        // No cryptographic source/authority receipt verifier is in this crate.
        // A valid self-asserted status can never become an admission decision.
        let mut receipt_gate = BTreeSet::new();
        receipt_gate.insert("authority.signed_receipt_verification_required".into());
        receipt_gate.insert("source.attestation_verification_required".into());
        ValidationReport::new(
            Disposition::StructurallyValidRequiresAuthorityReceipt,
            receipt_gate,
            schema_error_count,
            self.field_policy_digest.clone(),
        )
    }

    fn check_object_keys(
        &self,
        value: Option<&Value>,
        allowlist: Option<&Value>,
        reason: &str,
        rejected: &mut BTreeSet<String>,
    ) {
        let Some(object) = value.and_then(Value::as_object) else {
            rejected.insert(format!("{reason}.not_object"));
            return;
        };
        let Some(allowed) = allowlist.and_then(Value::as_array) else {
            rejected.insert(format!("{reason}.allowlist_missing"));
            return;
        };
        let allowed: BTreeSet<&str> = allowed.iter().filter_map(Value::as_str).collect();
        if object.keys().any(|key| !allowed.contains(key.as_str())) {
            rejected.insert(reason.into());
        }
    }

    fn reject(&self, reason: &str) -> ValidationReport {
        ValidationReport::new(
            Disposition::Rejected,
            [reason.to_string()].into_iter().collect(),
            0,
            self.field_policy_digest.clone(),
        )
    }
}

fn schema_has_external_refs(value: &Value) -> bool {
    match value {
        Value::Object(object) => object.iter().any(|(key, child)| {
            (key == "$ref" || key == "$dynamicRef")
                && child
                    .as_str()
                    .is_some_and(|reference| !reference.starts_with('#'))
                || schema_has_external_refs(child)
        }),
        Value::Array(values) => values.iter().any(schema_has_external_refs),
        _ => false,
    }
}

fn string_at<'a>(value: &'a Value, pointer: &str) -> Option<&'a str> {
    value.pointer(pointer).and_then(Value::as_str)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn numbers_are_ijson_safe(value: &Value) -> bool {
    const MAX_SAFE_INTEGER: u64 = 9_007_199_254_740_991;
    match value {
        Value::Number(number) => {
            if let Some(integer) = number.as_i64() {
                integer.unsigned_abs() <= MAX_SAFE_INTEGER
            } else if let Some(integer) = number.as_u64() {
                integer <= MAX_SAFE_INTEGER
            } else {
                number.as_f64().is_some_and(f64::is_finite)
            }
        }
        Value::Array(values) => values.iter().all(numbers_are_ijson_safe),
        Value::Object(values) => values.values().all(numbers_are_ijson_safe),
        _ => true,
    }
}

/// Deserialize JSON while rejecting duplicate object properties. serde_json's
/// ordinary Value parser keeps only the last duplicate; that behavior is not
/// acceptable for digest- and signature-bearing evidence.
#[derive(Debug)]
struct StrictValue(Value);

impl<'de> Deserialize<'de> for StrictValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct StrictVisitor;

        impl<'de> Visitor<'de> for StrictVisitor {
            type Value = StrictValue;

            fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                formatter.write_str("a JSON value with unique object property names")
            }

            fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Bool(value)))
            }
            fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Number(Number::from(value))))
            }
            fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Number(Number::from(value))))
            }
            fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Number::from_f64(value)
                    .map(|number| StrictValue(Value::Number(number)))
                    .ok_or_else(|| E::custom("non-finite JSON number"))
            }
            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::String(value.to_owned())))
            }
            fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::String(value)))
            }
            fn visit_unit<E>(self) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Null))
            }
            fn visit_none<E>(self) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Null))
            }
            fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
            where
                D: serde::Deserializer<'de>,
            {
                StrictValue::deserialize(deserializer)
            }
            fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut values = Vec::new();
                while let Some(value) = sequence.next_element::<StrictValue>()? {
                    values.push(value.0);
                }
                Ok(StrictValue(Value::Array(values)))
            }
            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut values = Map::new();
                while let Some(key) = map.next_key::<String>()? {
                    if values.contains_key(&key) {
                        return Err(de::Error::custom("duplicate JSON object property"));
                    }
                    let value = map.next_value::<StrictValue>()?;
                    values.insert(key, value.0);
                }
                Ok(StrictValue(Value::Object(values)))
            }
        }

        deserializer.deserialize_any(StrictVisitor)
    }
}

pub(crate) fn parse_strict_json(bytes: &[u8]) -> Result<Value, ()> {
    if bytes.len() > MAX_ENVELOPE_BYTES {
        return Err(());
    }
    let mut deserializer = serde_json::Deserializer::from_slice(bytes);
    let value = StrictValue::deserialize(&mut deserializer).map_err(|_| ())?;
    deserializer.end().map_err(|_| ())?;
    if !numbers_are_ijson_safe(&value.0) {
        return Err(());
    }
    Ok(value.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn validator() -> EnvelopeValidator {
        EnvelopeValidator::from_embedded().expect("pinned schema and policy compile")
    }

    fn examples() -> Vec<Value> {
        let fixtures: Value = parse_strict_json(EXAMPLE_FIXTURES_JSON.as_bytes()).unwrap();
        fixtures["examples"].as_array().unwrap().clone()
    }

    fn refresh_digests(value: &mut Value, validator: &EnvelopeValidator) {
        value["governance"]["field_allowlist_digest"] =
            Value::String(validator.field_policy_digest().to_string());
        let payload = &value["domain_extension"]["payload"];
        let bytes = serde_json_canonicalizer::to_vec(payload).unwrap();
        value["lineage"]["payload_digest"] =
            Value::String(format!("sha256:{}", sha256_hex(&bytes)));
        let mut content = value.clone();
        content["lineage"]
            .as_object_mut()
            .unwrap()
            .remove("envelope_content_digest");
        let bytes = serde_json_canonicalizer::to_vec(&content).unwrap();
        value["lineage"]["envelope_content_digest"] =
            Value::String(format!("sha256:{}", sha256_hex(&bytes)));
    }

    #[test]
    fn pinned_schema_and_policy_are_valid_and_compile_offline() {
        assert!(jsonschema::meta::is_valid(
            &parse_strict_json(SCHEMA_JSON.as_bytes()).unwrap()
        ));
        let v = validator();
        assert!(v.field_policy_digest().starts_with("sha256:"));
    }

    #[test]
    fn embedded_schema_contains_no_remote_references() {
        let schema = parse_strict_json(SCHEMA_JSON.as_bytes()).unwrap();
        assert!(!schema_has_external_refs(&schema));
    }

    #[test]
    fn external_schema_reference_is_rejected_without_network_resolution() {
        let mut schema: Value = parse_strict_json(SCHEMA_JSON.as_bytes()).unwrap();
        schema["properties"]["envelope_id"] =
            json!({"$ref": "https://untrusted.example/schema.json"});
        let result = EnvelopeValidator::from_documents(
            &serde_json::to_string(&schema).unwrap(),
            FIELD_POLICY_JSON,
        );
        assert!(
            result
                .unwrap_err()
                .contains("schema.external_reference_forbidden")
        );
    }

    #[test]
    fn unknown_claim_type_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["claim"]["claim_type"] = json!("unknown_or_unregistered_claim");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"claim.unknown_domain_claim_type".into())
        );
    }

    #[test]
    fn unrecognized_domain_schema_revision_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["domain_extension"]["schema_revision"] = json!("99.0.0");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"domain_extension.unknown_schema_or_revision".into())
        );
    }

    #[test]
    fn failed_source_integrity_is_rejected_not_merely_unresolved() {
        let v = validator();
        let mut example = examples().remove(0);
        example["source"]["integrity_status"] = json!("failed");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"source.integrity_failed".into())
        );
    }

    #[test]
    fn schema_identity_and_draft_are_pinned() {
        let schema: Value = parse_strict_json(SCHEMA_JSON.as_bytes()).unwrap();
        let mut changed = schema;
        changed["$id"] = json!("urn:attacker:replacement");
        let result = EnvelopeValidator::from_documents(
            &serde_json::to_string(&changed).unwrap(),
            FIELD_POLICY_JSON,
        );
        assert!(result.is_err());
    }

    #[test]
    fn policy_digest_semantics_are_pinned() {
        let policy: Value = parse_strict_json(FIELD_POLICY_JSON.as_bytes()).unwrap();
        let mut changed = policy;
        changed["canonicalization"]["digest_semantics"]["payload_digest"] =
            json!("SHA-256 over arbitrary JSON serialization.");
        let result = EnvelopeValidator::from_documents(
            SCHEMA_JSON,
            &serde_json::to_string(&changed).unwrap(),
        );
        assert!(result.is_err());
    }

    #[test]
    fn unknown_policy_version_is_rejected_during_initialization() {
        let policy: Value = parse_strict_json(FIELD_POLICY_JSON.as_bytes()).unwrap();
        let mut changed = policy;
        changed["policy_version"] = json!("9.9.9");
        let report = EnvelopeValidator::from_documents(
            SCHEMA_JSON,
            &serde_json::to_string(&changed).unwrap(),
        );
        assert!(report.is_err());
    }

    #[test]
    fn declared_purpose_must_be_in_allowed_uses() {
        let v = validator();
        let mut example = examples().remove(0);
        example["governance"]["allowed_uses"] = json!(["other_purpose"]);
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Quarantined);
        assert!(
            report
                .reason_codes
                .contains(&"rights.declared_purpose_not_allowed".into())
        );
    }

    #[test]
    fn expired_retention_at_ingest_is_quarantined() {
        let v = validator();
        let mut example = examples().remove(0);
        example["governance"]["retention_until"] = json!("2026-11-09T07:59:00Z");
        example["timestamps"]["received_at"] = json!("2026-11-09T08:00:00Z");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Quarantined);
        assert!(
            report
                .reason_codes
                .contains(&"rights.retention_expired_at_ingest".into())
        );
    }

    #[test]
    fn correction_lineage_is_covered_by_envelope_digest() {
        let v = validator();
        let mut example = examples().remove(0);
        example["lineage"]["corrects_refs"] = json!(["prior-envelope-001"]);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"digest.envelope_content_mismatch".into())
        );
    }

    #[test]
    fn synthetic_domain_examples_remain_unresolved_without_source_attestation() {
        let v = validator();
        let reports: Vec<_> = examples().iter().map(|e| v.validate_value(e)).collect();
        assert_eq!(reports.len(), 3);
        assert!(reports.iter().all(|r| {
            r.disposition == Disposition::Unresolved
                && r.reason_codes
                    .contains(&"source.authentication_unverified".into())
        }));
    }

    #[test]
    fn authenticated_source_declaration_still_requires_an_independent_receipt() {
        let v = validator();
        let mut example = examples().remove(0);
        example["source"]["integrity_status"] = json!("authenticated");
        example["source"]["authentication_method"] = json!("synthetic_test_attestation");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(
            report.disposition,
            Disposition::StructurallyValidRequiresAuthorityReceipt
        );
        assert!(
            report
                .reason_codes
                .contains(&"authority.signed_receipt_verification_required".into())
        );
        assert!(
            report
                .reason_codes
                .contains(&"source.attestation_verification_required".into())
        );
    }

    #[test]
    fn wrong_field_policy_digest_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["governance"]["field_allowlist_digest"] =
            json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        refresh_digests(&mut example, &v);
        // refresh_digests restores the correct digest, so deliberately corrupt it after binding.
        example["governance"]["field_allowlist_digest"] =
            json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"digest.field_policy_mismatch".into())
        );
    }

    #[test]
    fn unknown_domain_extension_field_is_rejected_even_with_recomputed_digests() {
        let v = validator();
        let mut example = examples().remove(0);
        example["domain_extension"]["payload"]["pilot_personal_location"] =
            json!("never-log-this-value");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"domain_extension.field_not_allowlisted".into())
        );
        assert!(
            !serde_json::to_string(&report)
                .unwrap()
                .contains("never-log-this-value")
        );
    }

    #[test]
    fn nested_domain_extension_payload_is_rejected_even_with_recomputed_digests() {
        let v = validator();
        let mut example = examples().remove(0);
        example["domain_extension"]["payload"]["fixture_id"] = json!({"nested": "not-allowed"});
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"domain_extension.nested_or_array_data_not_allowed_in_v0_1".into())
        );
    }

    #[test]
    fn array_domain_extension_payload_value_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["domain_extension"]["payload"]["fixture_id"] = json!(["not-allowed"]);
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"domain_extension.nested_or_array_data_not_allowed_in_v0_1".into())
        );
    }

    #[test]
    fn unknown_claim_value_field_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["claim"]["value"]["controller_location"] = json!("private-value");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"claim.value_field_not_allowlisted".into())
        );
        assert!(
            !serde_json::to_string(&report)
                .unwrap()
                .contains("private-value")
        );
    }

    #[test]
    fn nested_claim_value_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["claim"]["value"]["reported_class"] = json!({"nested":"not-allowed"});
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"claim.value_nested_or_array_data_not_allowed".into())
        );
    }

    #[test]
    fn pending_rights_are_quarantined() {
        let v = validator();
        let mut example = examples().remove(0);
        example["governance"]["authorization_status"] = json!("pending");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Quarantined);
        assert!(
            report
                .reason_codes
                .contains(&"rights.authorization_not_approved".into())
        );
    }

    #[test]
    fn unverified_source_is_unresolved_not_accepted() {
        let v = validator();
        let mut example = examples().remove(0);
        example["source"]["integrity_status"] = json!("unverified");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Unresolved);
        assert!(
            report
                .reason_codes
                .contains(&"source.authentication_unverified".into())
        );
    }

    #[test]
    fn stale_data_is_unresolved() {
        let v = validator();
        let mut example = examples().remove(0);
        example["quality"]["freshness"] = json!("stale");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Unresolved);
        assert!(
            report
                .reason_codes
                .contains(&"source.freshness_not_current".into())
        );
    }

    #[test]
    fn missing_dependence_groups_are_unresolved() {
        let v = validator();
        let mut example = examples().remove(0);
        example["dependence"]["dependency_group_refs"] = json!([]);
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Unresolved);
        assert!(
            report
                .reason_codes
                .contains(&"dependence.group_identity_missing".into())
        );
    }

    #[test]
    fn reversed_event_time_interval_is_rejected() {
        let v = validator();
        let mut example = examples().remove(0);
        example["timestamps"]["event_time"]["start"] = json!("2026-11-09T09:00:00Z");
        example["timestamps"]["event_time"]["end"] = json!("2026-11-09T08:00:00Z");
        refresh_digests(&mut example, &v);
        let report = v.validate_value(&example);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"time.event_interval_reversed".into())
        );
    }

    #[test]
    fn envelope_content_digest_binds_metadata_not_only_sensor_payload() {
        let v = validator();
        let example = examples().remove(0);
        let mut changed = example.clone();
        changed["source"]["source_class"] = json!("authority_feed");
        let report = v.validate_value(&changed);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"digest.envelope_content_mismatch".into())
        );
    }

    #[test]
    fn duplicate_json_object_properties_are_rejected() {
        let v = validator();
        let report = v.validate_bytes(br#"{"domain":"uas","domain":"maritime"}"#);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(
            report
                .reason_codes
                .contains(&"json.parse_or_duplicate_key_failure".into())
        );
    }

    #[test]
    fn oversized_input_is_rejected_before_parsing() {
        let v = validator();
        let bytes = vec![b' '; MAX_ENVELOPE_BYTES + 1];
        let report = v.validate_bytes(&bytes);
        assert_eq!(report.disposition, Disposition::Rejected);
        assert!(report.reason_codes.contains(&"input.too_large".into()));
    }

    #[test]
    fn rfc_8785_canonicalization_example_is_stable() {
        let value: Value = serde_json::from_str(
            r#"{"numbers":[333333333.33333329,1E30,4.50,2e-3,0.000000000000000000000000001],"string":"€$\u000f\nA'B\"\\\\\"\/","literals":[null,true,false]}"#
        ).unwrap();
        let canonical = serde_json_canonicalizer::to_string(&value).unwrap();
        assert_eq!(
            canonical,
            r#"{"literals":[null,true,false],"numbers":[333333333.3333333,1e+30,4.5,0.002,1e-27],"string":"€$\u000f\nA'B\"\\\\\"/"}"#
        );
    }
}
