use serde::{
    de::{self, MapAccess, SeqAccess, Visitor},
    Deserialize, Deserializer, Serialize,
};
use ed25519_dalek::{Signature, VerifyingKey};
use serde_json::{Map as JsonMap, Number, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fmt, fs, process,
};

const DEFAULT_CORPUS: &str =
    "mycelix-finance/tests/reference/valueflows-interop/mapping-corpus-v1.json";
const EMBEDDED_CORPUS: &str =
    include_str!("../../../tests/reference/valueflows-interop/mapping-corpus-v1.json");
const DEFAULT_PROVENANCE_FIXTURE: &str =
    "mycelix-finance/tests/reference/valueflows-interop/provenance-envelope-contract-v1.json";
const EMBEDDED_PROVENANCE_FIXTURE: &str = include_str!(
    "../../../tests/reference/valueflows-interop/provenance-envelope-contract-v1.json"
);

// Synthetic-fixture allowlist only; this is not a production issuer registry.
const SYNTHETIC_TRUSTED_ISSUER_ID: &str = "urn:mycelix:issuer:synthetic-01";
const SYNTHETIC_TRUSTED_KEY_ID: &str =
    "sha256:dfd6251d9330a21b322de85cdf49fa4730ce7d0e8890d677124c4c41e487e197";
const SYNTHETIC_TRUSTED_PUBLIC_KEY_HEX: &str =
    "376dbc1327c9e66b5a5eb33343ff5cbe86dc303b59ba0d2be413f7224e8b05a4";

// Deserialize through a map visitor so duplicate JSON object names are rejected
// before serde_json::Value could collapse them. JCS requires unique member names.
struct UniqueJson(Value);

impl<'de> Deserialize<'de> for UniqueJson {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(UniqueJsonVisitor)
    }
}

struct UniqueJsonVisitor;

impl<'de> Visitor<'de> for UniqueJsonVisitor {
    type Value = UniqueJson;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("JSON with unique object member names")
    }

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Bool(value)))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Number(Number::from(value))))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Number(Number::from(value))))
    }

    fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
    where
        E: de::Error,
    {
        Number::from_f64(value)
            .map(|number| UniqueJson(Value::Number(number)))
            .ok_or_else(|| E::custom("non-finite JSON number is not permitted"))
    }

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::String(value.to_owned())))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::String(value)))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_none<E>(self) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(value) = sequence.next_element::<UniqueJson>()? {
            values.push(value.0);
        }
        Ok(UniqueJson(Value::Array(values)))
    }

    fn visit_map<A>(self, mut object: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut seen = BTreeSet::new();
        let mut values = JsonMap::new();
        while let Some(key) = object.next_key::<String>()? {
            if !seen.insert(key.clone()) {
                return Err(de::Error::custom(format!(
                    "duplicate JSON object key: {key:?}"
                )));
            }
            let value = object.next_value::<UniqueJson>()?;
            values.insert(key, value.0);
        }
        Ok(UniqueJson(Value::Object(values)))
    }
}

fn parse_unique_json(input: &str) -> Result<Value, String> {
    serde_json::from_str::<UniqueJson>(input)
        .map(|parsed| parsed.0)
        .map_err(|error| error.to_string())
}

fn canonical_json_bytes(value: &Value) -> Result<Vec<u8>, String> {
    serde_json_canonicalizer::to_vec(value)
        .map_err(|error| format!("RFC 8785/JCS canonicalization failed: {error}"))
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    format!("{digest:x}")
}

fn canonical_payload_digest(payload: &Value) -> Result<String, String> {
    let canonical = canonical_json_bytes(payload)?;
    Ok(sha256_hex(&canonical))
}

fn canonical_envelope_digest(
    schema_id: &str,
    schema_version: &str,
    observation: &ProvenanceObservation,
) -> Result<String, String> {
    let preimage = EnvelopeDigestPreimage {
        schema_id,
        schema_version,
        observation,
    };
    let canonical = serde_json_canonicalizer::to_vec(&preimage)
        .map_err(|error| format!("RFC 8785/JCS envelope canonicalization failed: {error}"))?;
    Ok(sha256_hex(&canonical))
}

fn is_lowercase_sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|byte| byte.is_ascii_hexdigit())
        && value == value.to_ascii_lowercase()
}

fn decode_lowercase_hex<const N: usize>(value: &str, label: &str) -> Result<[u8; N], String> {
    if value.len() != N * 2
        || !value.bytes().all(|byte| byte.is_ascii_hexdigit())
        || value != value.to_ascii_lowercase()
    {
        return Err(format!("{label} must be lowercase hex encoding of exactly {N} bytes"));
    }
    let mut output = [0_u8; N];
    let bytes = value.as_bytes();
    for index in 0..N {
        let high = hex_nibble(bytes[index * 2])
            .ok_or_else(|| format!("{label} contains invalid hex"))?;
        let low = hex_nibble(bytes[index * 2 + 1])
            .ok_or_else(|| format!("{label} contains invalid hex"))?;
        output[index] = (high << 4) | low;
    }
    Ok(output)
}

fn hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        _ => None,
    }
}

fn signature_message_bytes(
    schema_id: &str,
    schema_version: &str,
    envelope_digest_sha256: &str,
    signature: &ProvenanceSignature,
) -> Result<Vec<u8>, String> {
    let preimage = SignaturePreimage {
        profile: &signature.profile,
        algorithm: &signature.algorithm,
        issuer_id: &signature.issuer_id,
        key_id: &signature.key_id,
        schema_id,
        schema_version,
        envelope_digest_sha256,
        signed_at: &signature.signed_at,
    };
    serde_json_canonicalizer::to_vec(&preimage)
        .map_err(|error| format!("signature preimage RFC 8785/JCS canonicalization failed: {error}"))
}

fn verify_provenance_signature(
    fixture: &ProvenanceFixture,
    key_status: TrustedKeyStatus,
) -> Result<(), String> {
    if key_status != TrustedKeyStatus::Active {
        return Err(match key_status {
            TrustedKeyStatus::Revoked =>
                "trusted issuer key is revoked; self-reported signed_at is not a trusted timestamp".into(),
            TrustedKeyStatus::Unknown =>
                "issuer key status is unknown; signature must fail closed".into(),
            TrustedKeyStatus::Active => unreachable!(),
        });
    }

    let signature = &fixture.signature;
    if signature.profile != "mycelix.provenance-envelope-signature/v1"
        || signature.algorithm != "Ed25519"
    {
        return Err("unsupported or unqualified provenance signature profile/algorithm".into());
    }
    if signature.issuer_id != SYNTHETIC_TRUSTED_ISSUER_ID
        || signature.key_id != SYNTHETIC_TRUSTED_KEY_ID
        || signature.public_key_hex != SYNTHETIC_TRUSTED_PUBLIC_KEY_HEX
    {
        return Err("signature issuer/key does not match the out-of-band synthetic test-key binding".into());
    }

    let public_key_bytes =
        decode_lowercase_hex::<32>(&signature.public_key_hex, "Ed25519 public key")?;
    let computed_key_id = format!("sha256:{}", sha256_hex(&public_key_bytes));
    if computed_key_id != signature.key_id {
        return Err("signature key_id does not match SHA-256 of the raw Ed25519 public-key bytes".into());
    }

    let signature_bytes = decode_lowercase_hex::<64>(&signature.signature_hex, "Ed25519 signature")?;
    let verifying_key = VerifyingKey::from_bytes(&public_key_bytes)
        .map_err(|_| "invalid Ed25519 verifying key".to_string())?;
    let parsed_signature = Signature::from_bytes(&signature_bytes);
    let message = signature_message_bytes(
        &fixture.schema_id,
        &fixture.schema_version,
        &fixture.example_envelope_digest_sha256,
        signature,
    )?;
    verifying_key
        .verify_strict(&message, &parsed_signature)
        .map_err(|_| "Ed25519 strict signature verification failed".to_string())
}



#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus {
    schema_version: String,
    status: String,
    authority_boundary: AuthorityBoundary,
    source_profile: SourceProfile,
    purpose: String,
    mapping_contract: MappingContract,
    mappings: Vec<Mapping>,
    negative_controls: Vec<NegativeControl>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AuthorityBoundary {
    external_facts_are_authoritative: bool,
    external_import_may_mutate_sap: bool,
    external_import_may_mutate_tend: bool,
    external_import_may_mutate_mycel: bool,
    external_import_may_mark_settlement_final: bool,
    unknown_or_conflicting_evidence_default: String,
    export_only: bool,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SourceProfile {
    normative_vocabulary_iri: String,
    published_ontology_version_iri: String,
    published_ontology_version: String,
    published_specification_release: String,
    published_specification_status_url: String,
    normative_specification_url: String,
    hrea_repository: String,
    hrea_source_commit: String,
    hrea_source_branch: String,
    hrea_source_describes_profile_as: String,
    hrea_source_commit_url: String,
    packaged_hrea_artifact_pinned: bool,
    hrea_runtime_qualified_with_mycelix: bool,
    version_note: String,
    release_evidence: ReleaseEvidence,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReleaseEvidence {
    release_tag: String,
    release_commit: String,
    published_at: String,
    release_url: String,
    claimed_valueflows_profile: String,
    claimed_holochain_requirement: String,
    claimed_client_requirement: String,
    dna_asset_sha256: String,
    happ_asset_sha256: String,
    upstream_license: String,
    license_file_url: String,
    license_review_approved_for_reuse: bool,
    migration_boundary: String,
    selected_for_mycelix_integration: bool,
    evidence_scope: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct MappingContract {
    direction: String,
    import_authority: String,
    required_envelope_fields: Vec<String>,
    quantity_policy: String,
    identity_policy: String,
    correction_policy: String,
    semantics_policy: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Mapping {
    id: String,
    mycelix_instrument: String,
    mycelix_identity: String,
    valueflows_entities: Vec<String>,
    quantity_encoding: String,
    unit_profile: String,
    preserves_asset_identity: bool,
    preserves_origin_and_source_event: bool,
    preserves_community_scope: bool,
    preserves_correction_lineage: bool,
    source_event_is_authority_for_ledger_mutation: bool,
    maps_to_authoritative_balance: bool,
    transferability: String,
    notes: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NegativeControl {
    id: String,
    input_condition: String,
    expected_disposition: String,
    required_guards: Vec<String>,
}

fn expected_disposition(id: &str) -> Option<&'static str> {
    match id {
        "VF-AUTH-001" => Some("reject_authority_promotion"),
        "VF-MYCEL-001" => Some("reject_semantic_loss"),
        "VF-TEND-001" => Some("reject_instrument_substitution"),
        "VF-AMOUNT-001" => Some("reject_lossy_quantity"),
        "VF-UNIT-001" => Some("reject_unit_mismatch"),
        "VF-SCOPE-001" => Some("reject_unbound_scope"),
        "VF-ORIGIN-001" => Some("reject_missing_provenance"),
        "VF-REPLAY-001" => Some("reject_duplicate_effect"),
        "VF-CORRECTION-001" => Some("preserve_append_only_correction_lineage"),
        "VF-UNKNOWN-001" => Some("indeterminate_or_policy_bounded_rejection"),
        "VF-VERSION-001" => Some("reject_unqualified_version_claim"),
        "VF-TEND-F32-001" => Some("block_unqualified_quantity_export"),
        "VF-TEND-BEARER-001" => Some("reject_bearer_asset_promotion"),
        _ => None,
    }
}

fn expected_mapping(
    instrument: &str,
) -> Option<(
    &'static str,
    &'static str,
    &'static str,
    &'static str,
    &'static [&'static str],
)> {
    match instrument {
        "SAP" => Some((
            "mycelix:SAP",
            "decimal_string_micro_sap",
            "mycelix:microSAP",
            "transferable_units_with_domain_rules",
            &["ResourceSpecification", "EconomicResource", "EconomicEvent"],
        )),
        "TEND" => Some((
            "mycelix:TEND",
            "blocked_legacy_f32_needs_quantization_profile",
            "mycelix:TEND-hour",
            "community_scoped_mutual_credit_position",
            &["Commitment", "EconomicEvent"],
        )),
        "MYCEL" => Some((
            "mycelix:MYCEL",
            "none_attestation_only",
            "not_applicable",
            "non_transferable_recognition_attestation",
            &[],
        )),
        _ => None,
    }
}

fn expected_input_condition(id: &str) -> Option<&'static str> {
    match id {
        "VF-AUTH-001" => Some(
            "Imported EconomicEvent is used directly to increase, decrease, or settle an authoritative Mycelix SAP/TEND balance.",
        ),
        "VF-MYCEL-001" => Some(
            "MYCEL recognition is exported as a ValueFlows Claim, currency, transferable balance, or spendable entitlement without a specifically reviewed semantic profile.",
        ),
        "VF-TEND-001" => Some(
            "TEND obligation is translated into SAP without an explicit, separately authorized exchange event and conversion profile.",
        ),
        "VF-AMOUNT-001" => Some(
            "An authoritative SAP micro-unit amount is parsed through binary floating point or loses its base-10 integer representation.",
        ),
        "VF-UNIT-001" => Some(
            "A quantity lacks its unit profile or a receiving adapter substitutes another unit.",
        ),
        "VF-SCOPE-001" => Some(
            "Community/issuer scope is omitted and records from separate economic networks become indistinguishable.",
        ),
        "VF-ORIGIN-001" => Some(
            "An exported record loses its source system, source record, source action/event, or revision identity.",
        ),
        "VF-REPLAY-001" => Some(
            "The same imported observation is replayed and counted as a second economic effect.",
        ),
        "VF-CORRECTION-001" => Some(
            "A correction overwrites the original event or removes its provenance from historical exports.",
        ),
        "VF-UNKNOWN-001" => Some(
            "Stale, missing, conflicting or unsupported evidence is represented as settled, valid, or definitively false without a policy basis.",
        ),
        "VF-VERSION-001" => Some(
            "hREA source's stated VF 1.0 profile is silently treated as identical to the published ontology page version 0.17, or a source branch is treated as a packaged compatible artifact.",
        ),
        "VF-TEND-F32-001" => Some(
            "TEND service hours sourced from the legacy f32 fields are exported as an exact decimal quantity without a versioned quantization/precision profile.",
        ),
        "VF-TEND-BEARER-001" => Some(
            "A TEND balance/obligation is exported as freely transferable bearer units without the original community, counterparties and service-obligation scope.",
        ),
        _ => None,
    }
}

fn expected_guards(id: &str) -> Option<&'static [&'static str]> {
    match id {
        "VF-AUTH-001" => Some(&[
            "external_facts_are_authoritative=false",
            "external_import_may_mutate_sap=false",
            "external_import_may_mutate_tend=false",
            "external_import_may_mark_settlement_final=false",
        ]),
        "VF-MYCEL-001" => Some(&[
            "MYCEL.valueflows_entities=empty_no_core_mapping",
            "MYCEL.quantity_encoding=none_attestation_only",
            "MYCEL.maps_to_authoritative_balance=false",
            "MYCEL.transferability=non_transferable_recognition_attestation",
        ]),
        "VF-TEND-001" => Some(&[
            "TEND.identity_preserved",
            "TEND.unit_preserved",
            "no_implicit_TEND_to_SAP_conversion",
            "TEND.transferability=community_scoped_mutual_credit_position",
        ]),
        "VF-AMOUNT-001" => Some(&[
            "canonical_integer_string",
            "unit_profile_preserved",
            "round_trip_exact",
        ]),
        "VF-UNIT-001" => Some(&["unit_profile_required", "unit_identity_preserved"]),
        "VF-SCOPE-001" => Some(&["scope_id_required", "issuer_or_community_scope_preserved"]),
        "VF-ORIGIN-001" => Some(&[
            "source_system_required",
            "source_record_id_required",
            "source_action_or_event_id_required",
            "source_revision_or_sequence_required",
        ]),
        "VF-REPLAY-001" => Some(&[
            "source_event_identity_is_idempotency_key",
            "import_has_no_ledger_mutation_authority",
        ]),
        "VF-CORRECTION-001" => Some(&[
            "original_event_immutable",
            "correction_references_original",
            "history_remains_auditable",
        ]),
        "VF-UNKNOWN-001" => Some(&[
            "unknown_is_not_false",
            "unknown_is_not_settled",
            "conflicts_are_preserved",
        ]),
        "VF-VERSION-001" => Some(&[
            "ontology_version_and_adapter_profile_separate",
            "exact_source_commit_pinned",
            "packaged_artifact_qualification_required",
        ]),
        "VF-TEND-F32-001" => Some(&[
            "legacy_f32_source_explicitly_blocked",
            "quantization_profile_required",
            "round_trip_error_bound_required",
        ]),
        "VF-TEND-BEARER-001" => Some(&[
            "TEND.transferability=community_scoped_mutual_credit_position",
            "community_scope_required",
            "service_obligation_provenance_required",
        ]),
        _ => None,
    }
}


#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProvenanceFixture {
    schema_id: String,
    schema_version: String,
    status: String,
    purpose: String,
    authority_boundary: ProvenanceAuthorityBoundary,
    canonicalization_policy: CanonicalizationPolicy,
    signature_policy: SignaturePolicy,
    example_observation: ProvenanceObservation,
    example_envelope_digest_sha256: String,
    signature: ProvenanceSignature,
    negative_controls: Vec<ProvenanceNegativeControl>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProvenanceAuthorityBoundary {
    external_observation_authority: String,
    external_import_may_mutate_sap: bool,
    external_import_may_mutate_tend: bool,
    external_import_may_mutate_mycel: bool,
    external_import_may_finalize_settlement: bool,
    external_import_may_finalize_governance: bool,
    signature_proves: Vec<String>,
    signature_proves_truth: bool,
    signature_grants_domain_authority: bool,
    corrections_are_append_only: bool,
    unknown_fields_disposition: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CanonicalizationPolicy {
    algorithm: String,
    digest_algorithm: String,
    digest_scope: String,
    envelope_digest_scope: String,
    payload_digest_verification_implemented: bool,
    envelope_digest_verification_implemented: bool,
    note: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SignaturePolicy {
    algorithm: String,
    profile: String,
    key_id_algorithm: String,
    trust_mode: String,
    signature_verification_implemented: bool,
    production_issuer_registry_implemented: bool,
    key_revocation_registry_implemented: bool,
    signed_at_is_trusted_timestamp: bool,
    claim_ceiling: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProvenanceSignature {
    profile: String,
    algorithm: String,
    issuer_id: String,
    key_id: String,
    public_key_hex: String,
    signed_at: String,
    signature_hex: String,
}

#[derive(Serialize)]
struct SignaturePreimage<'a> {
    profile: &'a str,
    algorithm: &'a str,
    issuer_id: &'a str,
    key_id: &'a str,
    schema_id: &'a str,
    schema_version: &'a str,
    envelope_digest_sha256: &'a str,
    signed_at: &'a str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TrustedKeyStatus {
    Active,
    Revoked,
    Unknown,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ProvenanceObservation {
    source_app: String,
    source_profile_id: String,
    source_record_id: String,
    subject_id: String,
    actor_id: String,
    scope_id: String,
    event_id: String,
    revision: u64,
    causal_predecessor: Option<String>,
    observed_at: String,
    recorded_at: String,
    payload: serde_json::Value,
    payload_canonicalization: String,
    payload_digest_sha256: String,
    evidence_class: String,
    correction_of: Option<String>,
    privacy_class: String,
    license_id: String,
    claim_ceiling: String,
}

#[derive(Serialize)]
struct EnvelopeDigestPreimage<'a> {
    schema_id: &'a str,
    schema_version: &'a str,
    observation: &'a ProvenanceObservation,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProvenanceNegativeControl {
    id: String,
    input_condition: String,
    expected_disposition: String,
    required_guards: Vec<String>,
}

fn expected_provenance_control(
    id: &str,
) -> Option<(&'static str, &'static str, &'static [&'static str])> {
    match id {
        "PE-AUTH-001" => Some((
            "A source hApp marks an observation as settled or uses it to mutate SAP, TEND, MYCEL, governance, or settlement without an independent Mycelix authorization decision.",
            "reject_authority_promotion",
            &[
                "external_import_authority=none",
                "signature_does_not_grant_domain_authority",
                "outcome_requires_separate_domain_authorization",
            ],
        )),
        "PE-SIG-001" => Some((
            "A valid signature is treated as proof that the observation is scientifically true or that the signer has domain authority.",
            "reject_claim_ceiling_escalation",
            &[
                "signature_proves_origin_and_integrity_only",
                "evidence_class_unchanged",
                "claim_ceiling_preserved",
            ],
        )),
        "PE-REPLAY-001" => Some((
            "An identical scoped event identity is received again with an identical canonical payload digest.",
            "accept_idempotently_without_second_effect",
            &[
                "idempotency_key_includes_source_scope_event_revision",
                "same_identity_same_digest_is_idempotent",
                "no_second_domain_effect",
            ],
        )),
        "PE-CONFLICT-001" => Some((
            "A previously observed scoped event identity reappears with a different payload digest.",
            "reject_identity_digest_conflict",
            &[
                "identity_digest_conflict_is_not_a_retry",
                "preserve_both_conflicting_receipts",
                "no_domain_mutation",
            ],
        )),
        "PE-SUBJECT-001" => Some((
            "The same payload digest is emitted for two distinct subjects or sensors.",
            "preserve_distinct_subject_observations",
            &[
                "subject_identity_part_of_idempotency_key",
                "payload_equality_does_not_merge_subjects",
                "source_scope_preserved",
            ],
        )),
        "PE-CORRECTION-001" => Some((
            "A correction overwrites or deletes the original event instead of appending a linked revision.",
            "preserve_append_only_correction",
            &[
                "original_record_immutable",
                "correction_references_original",
                "revision_monotonic",
            ],
        )),
        "PE-UNKNOWN-001" => Some((
            "A v1 record contains an unreviewed schema field that may change the meaning or authority of the record.",
            "reject_or_quarantine_unknown_schema",
            &[
                "unknown_fields_denied_in_v1",
                "no_default_authority_upgrade",
                "quarantine_is_non_authoritative",
            ],
        )),
        "PE-EVIDENCE-001" => Some((
            "A producer labels synthetic data as real or self-asserts that its own observation has independent review.",
            "reject_evidence_class_escalation",
            &[
                "evidence_class_not_self_promoted",
                "reviewer_attestation_separate_from_producer",
                "synthetic_never_admitted_as_real",
            ],
        )),
        "PE-SCOPE-001" => Some((
            "A record is re-used under another community or scope without an explicit evidence-bound crosswalk and authorization.",
            "reject_scope_rebinding",
            &[
                "scope_part_of_identity",
                "authorized_crosswalk_required",
                "out_of_scope_record_non_authoritative",
            ],
        )),
        "PE-SIG-KEY-001" => Some((
            "A source supplies a different, self-derived Ed25519 key and claims that key is an authorized issuer.",
            "reject_untrusted_issuer_key",
            &[
                "issuer_key_must_match_out_of_band_trust_binding",
                "public_key_fingerprint_recomputed",
                "signature_failure_non_authoritative",
            ],
        )),
        "PE-SIG-TAMPER-001" => Some((
            "An otherwise valid provenance signature or one of its signed metadata fields is altered after signing.",
            "reject_invalid_ed25519_signature",
            &[
                "ed25519_verify_strict",
                "signature_preimage_domain_separated",
                "signature_failure_non_authoritative",
            ],
        )),
        "PE-SIG-REVOKE-001" => Some((
            "A cryptographically valid signature is presented after the trusted key registry marks its key revoked, without independent trusted timestamp evidence.",
            "reject_revoked_key_without_trusted_timestamp",
            &[
                "revocation_state_external_to_signed_envelope",
                "self_asserted_signed_at_not_trusted",
                "revoked_key_rejects_by_default",
            ],
        )),
        "PE-REVOCATION-001" => Some((
            "Access revocation is reported as proof that plaintext already copied by another peer has been erased.",
            "reject_false_erasure_claim",
            &[
                "revocation_limits_future_authorized_access",
                "no_retroactive_copy_erasure_claim",
                "retraction_is_attributable_event",
            ],
        )),
        _ => None,
    }
}

fn validate_provenance(fixture: &ProvenanceFixture) -> Vec<String> {
    validate_provenance_with_key_status(fixture, TrustedKeyStatus::Active)
}

fn validate_provenance_with_key_status(
    fixture: &ProvenanceFixture,
    key_status: TrustedKeyStatus,
) -> Vec<String> {
    let mut errors = Vec::new();
    macro_rules! require {
        ($condition:expr, $message:expr) => {
            if !$condition {
                errors.push($message.to_string());
            }
        };
    }

    require!(
        fixture.schema_id == "urn:mycelix:provenance-envelope",
        "schema_id must remain urn:mycelix:provenance-envelope"
    );
    require!(
        fixture.schema_version == "mycelix.provenance-envelope/v1",
        "schema_version must remain mycelix.provenance-envelope/v1"
    );
    require!(
        fixture.status == "research-only-contract",
        "status must remain research-only-contract"
    );
    require!(!fixture.purpose.trim().is_empty(), "purpose must be explicit");

    let authority = &fixture.authority_boundary;
    require!(
        authority.external_observation_authority == "none",
        "external observations must have no domain authority"
    );
    require!(
        !authority.external_import_may_mutate_sap,
        "external import must not mutate SAP"
    );
    require!(
        !authority.external_import_may_mutate_tend,
        "external import must not mutate TEND"
    );
    require!(
        !authority.external_import_may_mutate_mycel,
        "external import must not mutate MYCEL"
    );
    require!(
        !authority.external_import_may_finalize_settlement,
        "external import must not finalize settlement"
    );
    require!(
        !authority.external_import_may_finalize_governance,
        "external import must not finalize governance"
    );
    require!(
        authority.signature_proves.iter().map(String::as_str).eq(["origin", "integrity"]),
        "signature semantics must be exactly origin and integrity"
    );
    require!(
        !authority.signature_proves_truth,
        "signature must not be treated as proof of truth"
    );
    require!(
        !authority.signature_grants_domain_authority,
        "signature must not grant domain authority"
    );
    require!(
        authority.corrections_are_append_only,
        "corrections must remain append-only"
    );
    require!(
        authority.unknown_fields_disposition == "reject_or_quarantine",
        "unknown v1 fields must be rejected or quarantined"
    );

    let canonicalization = &fixture.canonicalization_policy;
    require!(
        canonicalization.algorithm == "RFC8785-JCS",
        "canonicalization algorithm must remain RFC8785-JCS"
    );
    require!(
        canonicalization.digest_algorithm == "SHA-256",
        "payload digest algorithm must remain SHA-256"
    );
    require!(
        canonicalization.digest_scope == "canonicalized_payload_bytes",
        "payload digest scope must be canonicalized payload bytes"
    );
    require!(
        canonicalization.envelope_digest_scope == "canonicalized_envelope_preimage",
        "envelope digest scope must be the canonicalized, domain-separated preimage"
    );
    require!(
        canonicalization.payload_digest_verification_implemented,
        "fixture checker must recompute and compare the payload digest"
    );
    require!(
        canonicalization.envelope_digest_verification_implemented,
        "fixture checker must recompute and compare the envelope digest"
    );
    require!(
        canonicalization.note.contains("recomputes SHA-256 over RFC 8785/JCS canonical bytes for both payload and envelope preimage")
            && canonicalization.note.contains("Ed25519 signature verification")
            && canonicalization.note.contains("does not establish truth"),
        "canonicalization note must describe digest/signature verification and retained truth limits"
    );

    let signature_policy = &fixture.signature_policy;
    require!(
        signature_policy.algorithm == "Ed25519"
            && signature_policy.profile == "mycelix.provenance-envelope-signature/v1"
            && signature_policy.key_id_algorithm == "SHA-256 of raw Ed25519 public-key bytes",
        "signature algorithm/profile/key fingerprint must remain pinned"
    );
    require!(
        signature_policy.trust_mode == "pinned-synthetic-test-key-only"
            && signature_policy.signature_verification_implemented,
        "signature verification must use only the pinned synthetic fixture key"
    );
    require!(
        !signature_policy.production_issuer_registry_implemented
            && !signature_policy.key_revocation_registry_implemented
            && !signature_policy.signed_at_is_trusted_timestamp,
        "fixture must not imply production issuer trust, revocation, or trusted timestamp infrastructure"
    );
    require!(
        signature_policy.claim_ceiling.contains("does not establish truth")
            && signature_policy.claim_ceiling.contains("does not grant domain authority"),
        "signature claim ceiling must exclude truth and domain authority"
    );

    let example = &fixture.example_observation;
    for (value, field) in [
        (example.source_app.as_str(), "source_app"),
        (example.source_profile_id.as_str(), "source_profile_id"),
        (example.source_record_id.as_str(), "source_record_id"),
        (example.subject_id.as_str(), "subject_id"),
        (example.actor_id.as_str(), "actor_id"),
        (example.scope_id.as_str(), "scope_id"),
        (example.event_id.as_str(), "event_id"),
        (example.observed_at.as_str(), "observed_at"),
        (example.recorded_at.as_str(), "recorded_at"),
        (example.privacy_class.as_str(), "privacy_class"),
        (example.license_id.as_str(), "license_id"),
        (example.claim_ceiling.as_str(), "claim_ceiling"),
    ] {
        require!(!value.trim().is_empty(), format!("example observation requires {field}"));
    }
    require!(
        example.source_profile_id == "fixture:provenance-envelope/v1",
        "example must remain a local synthetic profile, not an implied upstream integration"
    );
    require!(example.revision > 0, "event revision must be positive");
    require!(
        example.payload_canonicalization == "RFC8785-JCS",
        "example payload canonicalization must agree with the declared algorithm"
    );
    require!(
        is_lowercase_sha256_hex(&example.payload_digest_sha256),
        "example payload digest must have lowercase 64-character SHA-256 syntax"
    );
    require!(
        is_lowercase_sha256_hex(&fixture.example_envelope_digest_sha256),
        "example envelope digest must have lowercase 64-character SHA-256 syntax"
    );
    match canonical_payload_digest(&example.payload) {
        Ok(actual_digest) => require!(
            actual_digest == example.payload_digest_sha256,
            "example payload digest must match SHA-256 of RFC 8785/JCS canonical payload bytes"
        ),
        Err(error) => errors.push(error),
    }
    match canonical_envelope_digest(
        &fixture.schema_id,
        &fixture.schema_version,
        example,
    ) {
        Ok(actual_digest) => require!(
            actual_digest == fixture.example_envelope_digest_sha256,
            "example envelope digest must bind schema, identity, scope, timestamps, evidence class, claim ceiling, and payload digest"
        ),
        Err(error) => errors.push(error),
    }

    if let Err(error) = verify_provenance_signature(fixture, key_status) {
        errors.push(error);
    }
    require!(
        example.evidence_class == "synthetic",
        "reference example must remain synthetic"
    );
    require!(
        example.payload.get("synthetic") == Some(&serde_json::Value::Bool(true)),
        "synthetic example payload must retain its explicit synthetic marker"
    );
    require!(
        example.correction_of.is_none() && example.causal_predecessor.is_none(),
        "base fixture observation must not claim a predecessor or correction"
    );

    let expected_ids = [
        "PE-AUTH-001",
        "PE-SIG-001",
        "PE-SIG-KEY-001",
        "PE-SIG-TAMPER-001",
        "PE-SIG-REVOKE-001",
        "PE-REPLAY-001",
        "PE-CONFLICT-001",
        "PE-SUBJECT-001",
        "PE-CORRECTION-001",
        "PE-UNKNOWN-001",
        "PE-EVIDENCE-001",
        "PE-SCOPE-001",
        "PE-REVOCATION-001",
    ];
    let mut seen_ids = BTreeSet::new();
    for control in &fixture.negative_controls {
        require!(
            seen_ids.insert(control.id.as_str()),
            format!("duplicate provenance control id: {}", control.id)
        );
        require!(
            !control.input_condition.trim().is_empty(),
            format!("provenance control {} requires an input condition", control.id)
        );
        require!(
            !control.required_guards.is_empty(),
            format!("provenance control {} requires guards", control.id)
        );
        match expected_provenance_control(&control.id) {
            Some((condition, disposition, guards)) => {
                require!(
                    control.input_condition == condition,
                    format!("provenance control {} has a drifted condition", control.id)
                );
                require!(
                    control.expected_disposition == disposition,
                    format!("provenance control {} has an unsafe disposition", control.id)
                );
                require!(
                    control.required_guards.iter().map(String::as_str).eq(guards.iter().copied()),
                    format!("provenance control {} must use exactly the registered ordered guards", control.id)
                );
            }
            None => errors.push(format!(
                "provenance control {} has no registered contract",
                control.id
            )),
        }
    }
    for id in expected_ids {
        require!(
            seen_ids.contains(id),
            format!("required provenance control is missing: {id}")
        );
    }
    require!(
        seen_ids.len() == expected_ids.len(),
        "provenance contract v1 must contain exactly the 13 registered controls"
    );

    errors
}

fn check_provenance_text(input: &str) -> Result<usize, Vec<String>> {
    let value = match parse_unique_json(input) {
        Ok(value) => value,
        Err(error) => {
            return Err(vec![format!(
                "invalid provenance fixture JSON/shape: {error}"
            )])
        }
    };
    let fixture: ProvenanceFixture = match serde_json::from_value(value) {
        Ok(fixture) => fixture,
        Err(error) => {
            return Err(vec![format!(
                "invalid provenance fixture JSON/shape: {error}"
            )])
        }
    };
    let errors = validate_provenance(&fixture);
    if errors.is_empty() {
        Ok(fixture.negative_controls.len())
    } else {
        Err(errors)
    }
}

fn validate(corpus: &Corpus) -> Vec<String> {
    let mut errors = Vec::new();
    let fail = |errors: &mut Vec<String>, condition: bool, message: String| {
        if !condition {
            errors.push(message);
        }
    };

    fail(
        &mut errors,
        corpus.schema_version == "mycelix.valueflows-interop.corpus/v1",
        "schema_version must be mycelix.valueflows-interop.corpus/v1".into(),
    );
    fail(
        &mut errors,
        corpus.status == "experimental-read-only-export",
        "status must remain experimental-read-only-export".into(),
    );
    fail(
        &mut errors,
        !corpus.purpose.trim().is_empty(),
        "purpose must be non-empty".into(),
    );

    let authority = &corpus.authority_boundary;
    fail(
        &mut errors,
        authority.export_only,
        "authority_boundary.export_only must be true".into(),
    );
    fail(
        &mut errors,
        !authority.external_facts_are_authoritative,
        "external facts must never be authoritative in this contract".into(),
    );
    fail(
        &mut errors,
        !authority.external_import_may_mutate_sap,
        "external import must not mutate SAP".into(),
    );
    fail(
        &mut errors,
        !authority.external_import_may_mutate_tend,
        "external import must not mutate TEND".into(),
    );
    fail(
        &mut errors,
        !authority.external_import_may_mutate_mycel,
        "external import must not mutate MYCEL".into(),
    );
    fail(
        &mut errors,
        !authority.external_import_may_mark_settlement_final,
        "external import must not mark settlement final".into(),
    );
    fail(
        &mut errors,
        authority.unknown_or_conflicting_evidence_default == "indeterminate",
        "unknown/conflicting evidence must default to indeterminate".into(),
    );

    let source = &corpus.source_profile;
    fail(
        &mut errors,
        source.normative_vocabulary_iri == "https://w3id.org/valueflows/ont/vf",
        "unexpected normative ValueFlows vocabulary IRI".into(),
    );
    fail(
        &mut errors,
        source.published_ontology_version == "0.17"
            && source.published_ontology_version_iri == "https://w3id.org/valueflows/ont/0.17/vf",
        "published ontology version and version IRI must remain explicitly pinned to 0.17".into(),
    );
    fail(
        &mut errors,
        source.published_specification_release == "1.0.0"
            && source.published_specification_status_url
                == "https://www.valueflo.ws/introduction/status/",
        "ValueFlows documentation/specification release label must remain separately pinned to 1.0.0"
            .into(),
    );
    fail(
        &mut errors,
        source.normative_specification_url == "https://www.valueflo.ws/specification/all_vf/",
        "normative specification URL must be present and pinned".into(),
    );
    fail(
        &mut errors,
        source.hrea_repository == "h-REA/hREA"
            && source.hrea_source_branch == "sprout"
            && source.hrea_source_describes_profile_as == "ValueFlows 1.0 + Holochain 0.7",
        "hREA source-branch profile claim must be recorded separately from published ontology version".into(),
    );
    fail(
        &mut errors,
        source.hrea_source_commit == "8c99160bbf6c4ffab0b4d05a3edf8ec629c09ab4",
        "hREA source commit must match the exact reviewed sprout snapshot".into(),
    );
    fail(
        &mut errors,
        source.hrea_source_commit_url
            == "https://github.com/h-REA/hREA/commit/8c99160bbf6c4ffab0b4d05a3edf8ec629c09ab4",
        "hREA source commit URL must match the exact reviewed source location".into(),
    );
    fail(
        &mut errors,
        !source.packaged_hrea_artifact_pinned,
        "this corpus must not imply a packaged hREA artifact is pinned".into(),
    );
    fail(
        &mut errors,
        !source.hrea_runtime_qualified_with_mycelix,
        "hREA must remain explicitly unqualified with Mycelix until tested".into(),
    );
    fail(
        &mut errors,
        source.version_note.contains("documentation/specification release 1.0.0")
            && source.version_note.contains("ontology metadata uses version info 0.17")
            && source.version_note.contains("not by themselves evidence of incompatibility")
            && source.version_note.contains("research evidence only"),
        "source profile must distinguish the documentation, ontology-metadata and hREA-profile version layers"
            .into(),
    );

    let release = &source.release_evidence;
    fail(
        &mut errors,
        release.release_tag == "happ-0.5.0-beta.1"
            && release.release_url
                == "https://github.com/h-REA/hREA/releases/tag/happ-0.5.0-beta.1",
        "hREA released artifact evidence must be pinned to happ-0.5.0-beta.1".into(),
    );
    fail(
        &mut errors,
        release.release_commit == "79c8d4fb98c4f7f9a8cf3f7dd69690d6d1334d85",
        "hREA release commit must match the exact published release tag target".into(),
    );
    fail(
        &mut errors,
        release.published_at == "2026-09-10T13:10:58Z",
        "hREA release publication timestamp must remain pinned to the reviewed release".into(),
    );
    fail(
        &mut errors,
        release.claimed_valueflows_profile == "1.0"
            && release.claimed_holochain_requirement == "0.7.x"
            && release.claimed_client_requirement == "^0.21.0",
        "hREA release profile must preserve its ValueFlows/Holochain/client requirements".into(),
    );
    fail(
        &mut errors,
        release.dna_asset_sha256
            == "18b9bb6847fea31bfed73d9a0f65221911f5a9f1b91bf9fb3f5273b2fda7acec",
        "hREA DNA artifact digest does not match the pinned release asset".into(),
    );
    fail(
        &mut errors,
        release.happ_asset_sha256
            == "b9a7c3f0dd81fa934ba69b5054b7cdf58008526ee77ce57351b30cc5900a43a7",
        "hREA hApp artifact digest does not match the pinned release asset".into(),
    );
    fail(
        &mut errors,
        release.upstream_license == "Apache-2.0"
            && release.license_file_url
                == "https://github.com/h-REA/hREA/blob/79c8d4fb98c4f7f9a8cf3f7dd69690d6d1334d85/LICENSE",
        "hREA upstream license evidence must remain pinned to the release commit".into(),
    );
    fail(
        &mut errors,
        !release.license_review_approved_for_reuse,
        "hREA license reuse must remain unapproved pending explicit review".into(),
    );
    fail(
        &mut errors,
        release.migration_boundary
            == "the happ-0.4.0-beta to happ-0.5.0-beta upgrade changes the DNA hash, has no in-place data migration, and must be treated as a new network",
        "hREA DNA migration boundary must remain explicit and fail closed".into(),
    );
    fail(
        &mut errors,
        !release.selected_for_mycelix_integration
            && release.evidence_scope == "research_evidence_only"
            && !source.packaged_hrea_artifact_pinned
            && !source.hrea_runtime_qualified_with_mycelix,
        "pinned release evidence must not be promoted to a selected or qualified Mycelix dependency"
            .into(),
    );

    let contract = &corpus.mapping_contract;
    fail(
        &mut errors,
        contract.direction == "Mycelix -> external ValueFlows-shaped export",
        "mapping direction must remain export-only".into(),
    );
    fail(
        &mut errors,
        contract.import_authority == "none",
        "external import authority must be none".into(),
    );
    let expected_envelope_fields = [
        "source_system",
        "source_record_id",
        "source_action_or_event_id",
        "source_revision_or_sequence",
        "observed_at",
        "scope_id",
        "asset_or_instrument_identity",
        "quantity_and_unit_profile",
        "claim_ceiling",
    ];
    for required in expected_envelope_fields {
        fail(
            &mut errors,
            contract
                .required_envelope_fields
                .iter()
                .any(|field| field == required),
            format!("required envelope field is missing: {required}"),
        );
    }
    fail(
        &mut errors,
        contract.required_envelope_fields.len() == expected_envelope_fields.len(),
        "envelope field set must contain exactly the nine v1 fields".into(),
    );
    fail(
        &mut errors,
        contract.quantity_policy
            == "Authoritative numeric quantities are emitted as canonical base-10 integer strings in the mapping envelope, not binary floating-point JSON numbers. A downstream serializer may emit a ValueFlows Measure only when its exact-decimal behavior is established; the original integer string and unit profile remain available for round-trip checks.",
        "quantity policy must remain the exact approved integer-string rule".into(),
    );
    fail(
        &mut errors,
        contract.identity_policy
            == "Every external identifier is namespaced by source system and scope. Equal bare IDs from different sources are not the same resource unless an explicit, evidence-bound crosswalk exists.",
        "identity policy must remain the exact approved namespacing rule".into(),
    );
    fail(
        &mut errors,
        contract.correction_policy
            == "Corrections append a new record referencing the corrected event. They do not mutate or erase the historical source event.",
        "correction policy must remain append-only and preserve the historical event".into(),
    );
    fail(
        &mut errors,
        contract.semantics_policy
            == "Economic facts, reputation attestations, eligibility, authorization, settlement and outcomes remain separate claim types.",
        "semantic policy must continue separating facts, authorization, settlement and outcomes".into(),
    );

    let mut mapping_ids = BTreeSet::new();
    let mut instruments = BTreeMap::new();
    for mapping in &corpus.mappings {
        fail(
            &mut errors,
            mapping_ids.insert(mapping.id.as_str()),
            format!("duplicate mapping id: {}", mapping.id),
        );
        fail(
            &mut errors,
            !mapping.notes.trim().is_empty(),
            format!("mapping {} requires a claim-ceiling note", mapping.id),
        );
        fail(
            &mut errors,
            !mapping.mycelix_identity.trim().is_empty() && !mapping.unit_profile.trim().is_empty(),
            format!(
                "mapping {} must identify an instrument and unit profile",
                mapping.id
            ),
        );
        fail(
            &mut errors,
            mapping.preserves_asset_identity,
            format!("mapping {} must preserve asset identity", mapping.id),
        );
        fail(
            &mut errors,
            mapping.preserves_origin_and_source_event,
            format!("mapping {} must preserve origin/source event", mapping.id),
        );
        fail(
            &mut errors,
            mapping.preserves_community_scope,
            format!("mapping {} must preserve community scope", mapping.id),
        );
        fail(
            &mut errors,
            mapping.preserves_correction_lineage,
            format!("mapping {} must preserve correction lineage", mapping.id),
        );
        fail(
            &mut errors,
            !mapping.source_event_is_authority_for_ledger_mutation,
            format!(
                "mapping {} must not promote source events to ledger authority",
                mapping.id
            ),
        );
        fail(
            &mut errors,
            !mapping.maps_to_authoritative_balance,
            format!(
                "mapping {} must not map directly to authoritative balances",
                mapping.id
            ),
        );

        if instruments
            .insert(mapping.mycelix_instrument.as_str(), mapping)
            .is_some()
        {
            errors.push(format!(
                "duplicate instrument mapping: {}",
                mapping.mycelix_instrument
            ));
        }

        match expected_mapping(&mapping.mycelix_instrument) {
            Some((identity, encoding, unit_profile, transferability, entities)) => {
                fail(
                    &mut errors,
                    mapping.mycelix_identity == identity,
                    format!("{} has unexpected instrument identity", mapping.id),
                );
                fail(
                    &mut errors,
                    mapping.quantity_encoding == encoding,
                    format!(
                        "{} has an unsafe or unexpected quantity encoding",
                        mapping.id
                    ),
                );
                fail(
                    &mut errors,
                    mapping.unit_profile == unit_profile,
                    format!("{} has an unexpected or lossy unit profile", mapping.id),
                );
                fail(
                    &mut errors,
                    mapping.transferability == transferability,
                    format!("{} has incorrect transferability semantics", mapping.id),
                );
                let has_exact_entities = mapping
                    .valueflows_entities
                    .iter()
                    .map(String::as_str)
                    .eq(entities.iter().copied());
                fail(
                    &mut errors,
                    has_exact_entities,
                    format!("{} has an unexpected ValueFlows entity set", mapping.id),
                );
                if mapping.mycelix_instrument == "MYCEL" {
                    fail(
                        &mut errors,
                        mapping.valueflows_entities.is_empty(),
                        "MYCEL has no direct ValueFlows economic entity mapping".into(),
                    );
                }
            }
            None => errors.push(format!(
                "unsupported Mycelix instrument mapping: {}",
                mapping.mycelix_instrument
            )),
        }
    }

    for instrument in ["SAP", "TEND", "MYCEL"] {
        fail(
            &mut errors,
            instruments.keys().any(|candidate| *candidate == instrument),
            format!("required mapping is missing for {instrument}"),
        );
    }
    fail(
        &mut errors,
        instruments.len() == 3,
        "exactly the SAP, TEND and MYCEL mappings are allowed in corpus v1".into(),
    );

    let expected_controls = [
        "VF-AUTH-001",
        "VF-MYCEL-001",
        "VF-TEND-001",
        "VF-AMOUNT-001",
        "VF-UNIT-001",
        "VF-SCOPE-001",
        "VF-ORIGIN-001",
        "VF-REPLAY-001",
        "VF-CORRECTION-001",
        "VF-UNKNOWN-001",
        "VF-VERSION-001",
        "VF-TEND-F32-001",
        "VF-TEND-BEARER-001",
    ];
    let mut control_ids = BTreeSet::new();
    for control in &corpus.negative_controls {
        fail(
            &mut errors,
            control_ids.insert(control.id.as_str()),
            format!("duplicate negative control id: {}", control.id),
        );
        fail(
            &mut errors,
            !control.input_condition.trim().is_empty(),
            format!("negative control {} needs an input condition", control.id),
        );
        fail(
            &mut errors,
            !control.required_guards.is_empty(),
            format!("negative control {} must name required guards", control.id),
        );
        match expected_input_condition(&control.id) {
            Some(expected) => fail(
                &mut errors,
                control.input_condition == expected,
                format!("negative control {} has drifted input condition", control.id),
            ),
            None => errors.push(format!(
                "negative control {} has no registered input-condition contract",
                control.id
            )),
        }
        match expected_disposition(&control.id) {
            Some(expected) => fail(
                &mut errors,
                control.expected_disposition == expected,
                format!(
                    "negative control {} has incorrect expected disposition",
                    control.id
                ),
            ),
            None => errors.push(format!("unknown negative control: {}", control.id)),
        }
        match expected_guards(&control.id) {
            Some(required) => {
                for guard in required {
                    fail(
                        &mut errors,
                        control
                            .required_guards
                            .iter()
                            .any(|candidate| candidate.as_str() == *guard),
                        format!(
                            "negative control {} is missing required guard: {guard}",
                            control.id
                        ),
                    );
                }
                fail(
                    &mut errors,
                    control
                        .required_guards
                        .iter()
                        .map(String::as_str)
                        .eq(required.iter().copied()),
                    format!(
                        "negative control {} must use exactly the registered ordered guard set",
                        control.id
                    ),
                );
            }
            None => errors.push(format!(
                "negative control {} has no registered guard contract",
                control.id
            )),
        }
    }
    for id in expected_controls {
        fail(
            &mut errors,
            control_ids.iter().any(|registered| *registered == id),
            format!("required negative control is missing: {id}"),
        );
    }
    fail(
        &mut errors,
        control_ids.len() == expected_controls.len(),
        "negative-control corpus v1 must contain exactly the 13 registered cases".into(),
    );

    errors
}

fn check_text(input: &str) -> Result<usize, Vec<String>> {
    let value = match parse_unique_json(input) {
        Ok(value) => value,
        Err(error) => return Err(vec![format!("invalid corpus JSON/shape: {error}")]),
    };
    let corpus: Corpus = match serde_json::from_value(value) {
        Ok(corpus) => corpus,
        Err(error) => return Err(vec![format!("invalid corpus JSON/shape: {error}")]),
    };
    let errors = validate(&corpus);
    if errors.is_empty() {
        Ok(corpus.negative_controls.len())
    } else {
        Err(errors)
    }
}

fn main() {
    let path = env::args().nth(1);
    let (label, raw) = match path {
        Some(path) => match fs::read_to_string(&path) {
            Ok(raw) => (path, raw),
            Err(error) => {
                eprintln!("FAIL: cannot read corpus: {error}");
                process::exit(2);
            }
        },
        None => (DEFAULT_CORPUS.to_owned(), EMBEDDED_CORPUS.to_owned()),
    };

    match check_text(&raw) {
        Ok(count) => println!(
            "PASS: {} has 3 instrument mappings and {} registered negative controls; export-only authority boundary intact (schema v1).",
            label, count
        ),
        Err(errors) => {
            eprintln!("FAIL: {} failed {} validation checks:", label, errors.len());
            for error in errors {
                eprintln!(" - {error}");
            }
            process::exit(1);
        }
    }

    match check_provenance_text(EMBEDDED_PROVENANCE_FIXTURE) {
        Ok(count) => println!(
            "PASS: {} has {} registered provenance boundary controls; no domain authority is granted (envelope v1).",
            DEFAULT_PROVENANCE_FIXTURE, count
        ),
        Err(errors) => {
            eprintln!(
                "FAIL: {} failed {} validation checks:",
                DEFAULT_PROVENANCE_FIXTURE,
                errors.len()
            );
            for error in errors {
                eprintln!(" - {error}");
            }
            process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> serde_json::Value {
        serde_json::from_str(EMBEDDED_CORPUS).expect("checked-in corpus JSON must parse")
    }

    #[test]
    fn checked_in_corpus_satisfies_fail_closed_contract() {
        let raw = EMBEDDED_CORPUS;
        let count = check_text(raw).expect("checked-in corpus must satisfy local contract");
        assert_eq!(count, 13);
    }

    #[test]
    fn external_authority_promotion_is_rejected() {
        let mut value = fixture();
        value["authority_boundary"]["external_facts_are_authoritative"] = true.into();
        let errors = check_text(&value.to_string()).expect_err("external authority must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("external facts must never be authoritative")));
    }

    #[test]
    fn mycel_cannot_become_transferable_money() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let mycel = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "MYCEL")
            .expect("MYCEL mapping");
        mycel["transferability"] = "transferable_units_with_domain_rules".into();
        let errors = check_text(&value.to_string()).expect_err("MYCEL transferability must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("incorrect transferability semantics")));
    }

    #[test]
    fn mycel_recognition_is_not_mapped_to_valueflows_claim() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let mycel = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "MYCEL")
            .expect("MYCEL mapping");
        mycel["valueflows_entities"] = serde_json::json!(["Claim"]);
        let errors = check_text(&value.to_string())
            .expect_err("a generic reputation attestation must not become a VF economic Claim");
        assert!(errors.iter().any(|error| {
            error.contains("no direct ValueFlows economic entity")
        }));
    }

    #[test]
    fn tend_mutual_credit_cannot_become_bearer_transferable() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let tend = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "TEND")
            .expect("TEND mapping");
        tend["transferability"] = "transferable_units_with_domain_rules".into();
        let errors = check_text(&value.to_string()).expect_err("TEND bearer promotion must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("incorrect transferability semantics")));
    }

    #[test]
    fn lossy_sap_quantity_encoding_is_rejected() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let sap = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "SAP")
            .expect("SAP mapping");
        sap["quantity_encoding"] = "binary_float".into();
        let errors = check_text(&value.to_string()).expect_err("binary float encoding must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("unsafe or unexpected quantity encoding")));
    }

    #[test]
    fn legacy_tend_f32_quantity_export_remains_blocked() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let tend = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "TEND")
            .expect("TEND mapping");
        tend["quantity_encoding"] = "canonical_integer_tend_units".into();
        let errors = check_text(&value.to_string())
            .expect_err("unqualified exact TEND amount export must remain blocked");
        assert!(errors
            .iter()
            .any(|error| error.contains("unsafe or unexpected quantity encoding")));
    }

    #[test]
    fn hrea_release_evidence_is_pinned_without_claiming_integration() {
        let mut value = fixture();
        value["source_profile"]["release_evidence"]["dna_asset_sha256"] =
            "not-a-sha256-digest".into();
        let errors =
            check_text(&value.to_string()).expect_err("malformed release digest must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("DNA artifact digest does not match")));

        let mut value = fixture();
        value["source_profile"]["release_evidence"]["selected_for_mycelix_integration"] =
            true.into();
        let errors = check_text(&value.to_string())
            .expect_err("release evidence must not imply Mycelix integration");
        assert!(errors.iter().any(|error| {
            error.contains("must not be promoted to a selected or qualified Mycelix dependency")
        }));
    }

    #[test]
    fn hrea_source_and_release_refs_cannot_drift_to_other_valid_shas() {
        let mut value = fixture();
        value["source_profile"]["hrea_source_commit"] =
            "9c99160bbf6c4ffab0b4d05a3edf8ec629c09ab4".into();
        let errors = check_text(&value.to_string())
            .expect_err("a different valid source SHA must not replace the inspected revision");
        assert!(errors
            .iter()
            .any(|error| error.contains("exact reviewed sprout snapshot")));

        let mut value = fixture();
        value["source_profile"]["release_evidence"]["release_commit"] =
            "89c8d4fb98c4f7f9a8cf3f7dd69690d6d1334d85".into();
        let errors = check_text(&value.to_string())
            .expect_err("a different valid release SHA must not replace the release target");
        assert!(errors
            .iter()
            .any(|error| error.contains("exact published release tag target")));
    }

    #[test]
    fn hrea_artifact_digests_cannot_drift_to_other_valid_hex_values() {
        let mut value = fixture();
        value["source_profile"]["release_evidence"]["dna_asset_sha256"] =
            "0000000000000000000000000000000000000000000000000000000000000000".into();
        let errors = check_text(&value.to_string())
            .expect_err("a valid-shaped but wrong DNA digest must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("DNA artifact digest does not match")));

        let mut value = fixture();
        value["source_profile"]["release_evidence"]["happ_asset_sha256"] =
            "1111111111111111111111111111111111111111111111111111111111111111".into();
        let errors = check_text(&value.to_string())
            .expect_err("a valid-shaped but wrong hApp digest must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("hApp artifact digest does not match")));
    }

    #[test]
    fn hrea_license_and_upgrade_boundary_remain_unqualified() {
        let mut value = fixture();
        value["source_profile"]["release_evidence"]["upstream_license"] = "MIT".into();
        let errors = check_text(&value.to_string())
            .expect_err("a different upstream license must not replace the pinned license");
        assert!(errors
            .iter()
            .any(|error| error.contains("license evidence must remain pinned")));

        let mut value = fixture();
        value["source_profile"]["release_evidence"]["license_review_approved_for_reuse"] =
            true.into();
        let errors = check_text(&value.to_string())
            .expect_err("recording the license does not constitute reuse approval");
        assert!(errors
            .iter()
            .any(|error| error.contains("license reuse must remain unapproved")));

        let mut value = fixture();
        value["source_profile"]["release_evidence"]["migration_boundary"] =
            "in-place migration is supported".into();
        let errors = check_text(&value.to_string())
            .expect_err("unsupported in-place migration must not be implied");
        assert!(errors
            .iter()
            .any(|error| error.contains("DNA migration boundary must remain explicit")));
    }

    #[test]
    fn mapping_contract_rules_and_entity_sets_cannot_be_widened() {
        for (field, changed) in [
            ("quantity_policy", "binary floating-point is acceptable"),
            ("identity_policy", "bare identifiers are globally unique"),
            ("correction_policy", "corrections overwrite history"),
            ("semantics_policy", "settlement means truth and authorization"),
        ] {
            let mut value = fixture();
            value["mapping_contract"][field] = changed.into();
            let errors = check_text(&value.to_string())
                .expect_err("a weakened mapping policy must fail");
            assert!(
                errors.iter().any(|error| error.contains("policy")),
                "{field}: {errors:?}"
            );
        }

        let mut value = fixture();
        value["mapping_contract"]["required_envelope_fields"]
            .as_array_mut()
            .expect("envelope field list")
            .push("unreviewed_extra_field".into());
        let errors = check_text(&value.to_string())
            .expect_err("v1 envelope must not silently grow extra fields");
        assert!(errors
            .iter()
            .any(|error| error.contains("exactly the nine v1 fields")));

        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let tend = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "TEND")
            .expect("TEND mapping");
        tend["valueflows_entities"]
            .as_array_mut()
            .expect("TEND ValueFlows entities")
            .push("Claim".into());
        let errors = check_text(&value.to_string())
            .expect_err("TEND's exact entity profile must not be silently widened");
        assert!(errors
            .iter()
            .any(|error| error.contains("unexpected ValueFlows entity set")));
    }

    #[test]
    fn valueflows_version_layers_remain_separate() {
        let mut value = fixture();
        value["source_profile"]["published_ontology_version"] = "1.0".into();
        let errors = check_text(&value.to_string())
            .expect_err("ontology metadata version must not be rewritten as the docs release");
        assert!(errors
            .iter()
            .any(|error| error.contains("published ontology version")));

        let mut value = fixture();
        value["source_profile"]["published_specification_release"] = "0.17".into();
        let errors = check_text(&value.to_string())
            .expect_err("the documentation release label must remain explicitly pinned");
        assert!(errors
            .iter()
            .any(|error| error.contains("documentation/specification release label")));
    }

    #[test]
    fn negative_control_dispositions_are_fixed_and_fail_closed() {
        let mut value = fixture();
        let controls = value["negative_controls"]
            .as_array_mut()
            .expect("controls array");
        let replay = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-REPLAY-001")
            .expect("replay control");
        replay["expected_disposition"] = "accept".into();
        let errors = check_text(&value.to_string()).expect_err("unsafe disposition must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("incorrect expected disposition")));
    }

    #[test]
    fn negative_control_conditions_cannot_drift_without_review() {
        let mut value = fixture();
        let controls = value["negative_controls"]
            .as_array_mut()
            .expect("controls array");
        let replay = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-REPLAY-001")
            .expect("replay control");
        replay["input_condition"] = "duplicate reads are always harmless".into();
        let errors = check_text(&value.to_string())
            .expect_err("a weakened negative-control condition must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("drifted input condition")));
    }

    #[test]
    fn negative_control_guard_sets_cannot_be_extended_or_reordered() {
        let mut value = fixture();
        let controls = value["negative_controls"]
            .as_array_mut()
            .expect("controls array");
        let amount = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-AMOUNT-001")
            .expect("amount control");
        amount["required_guards"]
            .as_array_mut()
            .expect("guard list")
            .push("unreviewed_guard".into());
        let errors = check_text(&value.to_string())
            .expect_err("unreviewed guards must fail closed");
        assert!(errors
            .iter()
            .any(|error| error.contains("exactly the registered ordered guard set")));

        let mut value = fixture();
        let controls = value["negative_controls"]
            .as_array_mut()
            .expect("controls array");
        let amount = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-AMOUNT-001")
            .expect("amount control");
        amount["required_guards"] = serde_json::json!([
            "round_trip_exact",
            "unit_profile_preserved",
            "canonical_integer_string"
        ]);
        let errors = check_text(&value.to_string())
            .expect_err("guard ordering must remain canonical for corpus v1");
        assert!(errors
            .iter()
            .any(|error| error.contains("exactly the registered ordered guard set")));
    }

    #[test]
    fn unknown_fields_fail_closed_for_corpus_v1() {
        let mut value = fixture();
        value["unreviewed_authority_override"] = true.into();
        let errors = check_text(&value.to_string()).expect_err("unknown fields must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("invalid corpus JSON/shape")));
    }

    #[test]
    fn required_negative_control_guards_cannot_be_removed() {
        let mut value = fixture();
        let controls = value["negative_controls"]
            .as_array_mut()
            .expect("controls array");
        let amount = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-AMOUNT-001")
            .expect("amount control");
        amount["required_guards"] = serde_json::json!(["canonical_integer_string"]);
        let errors =
            check_text(&value.to_string()).expect_err("dropping exact-round-trip guards must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("missing required guard")));
    }

    #[test]
    fn imported_external_fact_cannot_mark_settlement_final() {
        let mut value = fixture();
        value["authority_boundary"]["external_import_may_mark_settlement_final"] = true.into();
        let errors = check_text(&value.to_string())
            .expect_err("external import must not finalize settlement");
        assert!(errors
            .iter()
            .any(|error| error.contains("external import must not mark settlement final")));
    }

    #[test]
    fn provenance_scope_and_correction_lineage_are_required() {
        for (field, expected_error) in [
            (
                "preserves_origin_and_source_event",
                "must preserve origin/source event",
            ),
            ("preserves_community_scope", "must preserve community scope"),
            (
                "preserves_correction_lineage",
                "must preserve correction lineage",
            ),
        ] {
            let mut value = fixture();
            let mappings = value["mappings"].as_array_mut().expect("mappings array");
            let sap = mappings
                .iter_mut()
                .find(|mapping| mapping["mycelix_instrument"] == "SAP")
                .expect("SAP mapping");
            sap[field] = false.into();
            let errors = check_text(&value.to_string()).expect_err("lossy mapping must fail");
            assert!(
                errors.iter().any(|error| error.contains(expected_error)),
                "{field}: {errors:?}"
            );
        }
    }

    #[test]
    fn tend_and_sap_instrument_identities_cannot_be_substituted() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let tend = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "TEND")
            .expect("TEND mapping");
        tend["mycelix_identity"] = "mycelix:SAP".into();
        let errors =
            check_text(&value.to_string()).expect_err("TEND/SAP identity substitution must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("unexpected instrument identity")));
    }

    #[test]
    fn all_required_negative_controls_are_present_exactly_once() {
        let mut value = fixture();
        let controls = value["negative_controls"]
            .as_array_mut()
            .expect("controls array");
        controls.pop();
        let errors =
            check_text(&value.to_string()).expect_err("missing negative control must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("required negative control is missing")));
    }

    #[test]
    fn checked_in_provenance_fixture_satisfies_fail_closed_contract() {
        let count = check_provenance_text(EMBEDDED_PROVENANCE_FIXTURE)
            .expect("checked-in provenance fixture must satisfy its local contract");
        assert_eq!(count, 13);
    }

    #[test]
    fn checked_in_ed25519_signature_verifies_under_synthetic_allowlist() {
        let fixture: ProvenanceFixture =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        verify_provenance_signature(&fixture, TrustedKeyStatus::Active)
            .expect("checked-in synthetic Ed25519 signature must verify strictly");
    }

    fn published_w3c_eddsa_jcs_2022_vector() -> (Value, Value) {
        let credential = serde_json::json!({
            "@context": [
                "https://www.w3.org/ns/credentials/v2",
                "https://www.w3.org/ns/credentials/examples/v2"
            ],
            "id": "urn:uuid:58172aac-d8ba-11ed-83dd-0b3aef56cc33",
            "type": ["VerifiableCredential", "AlumniCredential"],
            "name": "Alumni Credential",
            "description": "A minimum viable example of an Alumni Credential.",
            "issuer": "https://vc.example/issuers/5678",
            "validFrom": "2023-01-01T00:00:00Z",
            "credentialSubject": {
                "id": "did:example:abcdefgh",
                "alumniOf": "The School of Examples"
            }
        });
        let proof_options = serde_json::json!({
            "type": "DataIntegrityProof",
            "cryptosuite": "eddsa-jcs-2022",
            "created": "2023-02-24T23:36:38Z",
            "verificationMethod": "did:key:z6MkrJVnaZkeFzdQyMZu1cgjg7k1pZZ6pvBQ7XJPt4swbTQ2#z6MkrJVnaZkeFzdQyMZu1cgjg7k1pZZ6pvBQ7XJPt4swbTQ2",
            "proofPurpose": "assertionMethod",
            "@context": [
                "https://www.w3.org/ns/credentials/v2",
                "https://www.w3.org/ns/credentials/examples/v2"
            ]
        });
        (credential, proof_options)
    }

    // W3C VC-DI-EDDSA Example 36 defines hashData; Example 37 gives the
    // signature bytes. The key bytes below
    // are the Ed25519 public key from Example 29's publicKeyMultibase after
    // removing its 0xed01 multicodec prefix.
    fn published_w3c_eddsa_jcs_2022_hash_data(
        credential: &Value,
        proof_options: &Value,
    ) -> ([u8; 64], [u8; 32], [u8; 32]) {
        let credential_digest =
            decode_lowercase_hex::<32>(
                &canonical_payload_digest(credential).expect("canonical credential"),
                "W3C credential digest",
            )
            .expect("W3C credential digest bytes");
        let proof_options_digest = decode_lowercase_hex::<32>(
            &sha256_hex(
                &canonical_json_bytes(proof_options).expect("canonical proof options"),
            ),
            "W3C proof configuration digest",
        )
        .expect("W3C proof configuration digest bytes");
        let mut hash_data = [0_u8; 64];
        hash_data[..32].copy_from_slice(&proof_options_digest);
        hash_data[32..].copy_from_slice(&credential_digest);
        (hash_data, proof_options_digest, credential_digest)
    }

    #[test]
    fn published_w3c_eddsa_jcs_2022_vector_verifies() {
        let (credential, proof_options) = published_w3c_eddsa_jcs_2022_vector();
        let (hash_data, proof_options_digest, credential_digest) =
            published_w3c_eddsa_jcs_2022_hash_data(&credential, &proof_options);

        assert_eq!(
            sha256_hex(&canonical_json_bytes(&credential).expect("canonical credential")),
            "59b7cb6251b8991add1ce0bc83107e3db9dbbab5bd2c28f687db1a03abc92f19"
        );
        assert_eq!(
            sha256_hex(&canonical_json_bytes(&proof_options).expect("canonical proof options")),
            "66ab154f5c2890a140cb8388a22a160454f80575f6eae09e5a097cabe539a1db"
        );
        assert_eq!(
            proof_options_digest,
            decode_lowercase_hex::<32>(
                "66ab154f5c2890a140cb8388a22a160454f80575f6eae09e5a097cabe539a1db",
                "W3C proof-options digest",
            )
            .expect("proof-options digest bytes")
        );
        assert_eq!(
            credential_digest,
            decode_lowercase_hex::<32>(
                "59b7cb6251b8991add1ce0bc83107e3db9dbbab5bd2c28f687db1a03abc92f19",
                "W3C credential digest",
            )
            .expect("credential digest bytes")
        );

        let public_key = decode_lowercase_hex::<32>(
            "b00d8d938e7f773d51565aad36a623f5344f7f5d1960f9cf3e8e12620ea2810f",
            "W3C example Ed25519 public key",
        )
        .expect("valid public key bytes");
        let signature = decode_lowercase_hex::<64>(
            "407cd12654b33d718ecbb99179a1506daaa849450bf3fc523cce3e1c96f8b80351da3f253d725c6f00b07c9e5448d50b3ef78012b9ab54255116d069c6dd2808",
            "W3C example Ed25519 signature",
        )
        .expect("valid signature bytes");
        let verifying_key = VerifyingKey::from_bytes(&public_key).expect("W3C public key");
        verifying_key
            .verify_strict(&hash_data, &Signature::from_bytes(&signature))
            .expect("the published W3C eddsa-jcs-2022 vector must verify");
    }

    #[test]
    fn published_w3c_eddsa_jcs_2022_vector_rejects_payload_and_proof_tampering() {
        let (credential, proof_options) = published_w3c_eddsa_jcs_2022_vector();
        let (hash_data, _, _) =
            published_w3c_eddsa_jcs_2022_hash_data(&credential, &proof_options);
        let public_key = decode_lowercase_hex::<32>(
            "b00d8d938e7f773d51565aad36a623f5344f7f5d1960f9cf3e8e12620ea2810f",
            "W3C example Ed25519 public key",
        )
        .expect("valid public key bytes");
        let signature = decode_lowercase_hex::<64>(
            "407cd12654b33d718ecbb99179a1506daaa849450bf3fc523cce3e1c96f8b80351da3f253d725c6f00b07c9e5448d50b3ef78012b9ab54255116d069c6dd2808",
            "W3C example Ed25519 signature",
        )
        .expect("valid signature bytes");
        let verifying_key = VerifyingKey::from_bytes(&public_key).expect("W3C public key");
        let parsed_signature = Signature::from_bytes(&signature);

        let mut changed_credential = credential.clone();
        changed_credential["name"] = "Altered Alumni Credential".into();
        let (changed_payload_hash_data, _, _) =
            published_w3c_eddsa_jcs_2022_hash_data(&changed_credential, &proof_options);
        assert!(verifying_key
            .verify_strict(&changed_payload_hash_data, &parsed_signature)
            .is_err());

        let mut changed_proof_options = proof_options.clone();
        changed_proof_options["proofPurpose"] = "authentication".into();
        let (changed_proof_hash_data, _, _) =
            published_w3c_eddsa_jcs_2022_hash_data(&credential, &changed_proof_options);
        assert!(verifying_key
            .verify_strict(&changed_proof_hash_data, &parsed_signature)
            .is_err());

        // Keep the baseline live so the test cannot pass solely because every
        // constructed hashData value is invalid.
        assert!(verifying_key.verify_strict(&hash_data, &parsed_signature).is_ok());
    }

    #[test]
    fn provenance_signature_bit_flip_is_rejected() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        let signature = value["signature"]["signature_hex"]
            .as_str().expect("signature hex").to_owned();
        let first = if signature.as_bytes()[0] == b'0' { '1' } else { '0' };
        value["signature"]["signature_hex"] = format!("{first}{}", &signature[1..]).into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("changed signature bytes must fail");
        assert!(errors.iter().any(|error| error.contains("Ed25519 strict signature verification failed")));
    }

    #[test]
    fn provenance_signature_cannot_choose_its_own_trust_root() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        value["signature"]["issuer_id"] = "urn:attacker:issuer".into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("issuer identity must be bound out of band");
        assert!(errors.iter().any(|error| error.contains("out-of-band synthetic test-key binding")));
    }

    #[test]
    fn revoked_trusted_key_fails_closed_without_a_trusted_timestamp() {
        let fixture: ProvenanceFixture =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        let errors = validate_provenance_with_key_status(&fixture, TrustedKeyStatus::Revoked);
        assert!(errors.iter().any(|error| {
            error.contains("self-reported signed_at is not a trusted timestamp")
        }));
        let errors = validate_provenance_with_key_status(&fixture, TrustedKeyStatus::Unknown);
        assert!(errors.iter().any(|error| error.contains("issuer key status is unknown")));
    }

    #[test]
    fn signed_at_tampering_invalidates_the_signature() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        value["signature"]["signed_at"] = "2026-10-15T17:55:00Z".into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("asserted signed_at is part of the signed preimage");
        assert!(errors.iter().any(|error| error.contains("Ed25519 strict signature verification failed")));
    }

    #[test]
    fn provenance_signature_cannot_promote_truth_or_authority() {
        for (field, expected) in [
            ("signature_proves_truth", "signature must not be treated as proof of truth"),
            ("signature_grants_domain_authority", "signature must not grant domain authority"),
        ] {
            let mut value: serde_json::Value =
                serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
            value["authority_boundary"][field] = true.into();
            let errors = check_provenance_text(&value.to_string())
                .expect_err("signature must not promote claims or authority");
            assert!(errors.iter().any(|error| error.contains(expected)), "{errors:?}");
        }
    }

    #[test]
    fn provenance_fixture_rejects_unknown_v1_fields() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        value["authority_override"] = true.into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("unknown fields must not silently change v1 semantics");
        assert!(errors
            .iter()
            .any(|error| error.contains("invalid provenance fixture JSON/shape")));
    }

    #[test]
    fn provenance_fixture_rejects_malformed_digest_syntax() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        value["example_observation"]["payload_digest_sha256"] = "abc".into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("malformed digest syntax must fail");
        assert!(errors
            .iter()
            .any(|error| error.contains("lowercase 64-character SHA-256 syntax")));
        assert!(errors.iter().any(|error| error.contains("canonical payload bytes")));
    }

    #[test]
    fn rfc8785_jcs_canonicalizes_member_order_and_number_formats() {
        let simple = parse_unique_json(r#"{"c":120,"b":false,"a":"Hello!"}"#)
            .expect("valid JCS input");
        let canonical = canonical_json_bytes(&simple).expect("canonicalization");
        assert_eq!(
            std::str::from_utf8(&canonical).expect("canonical JSON is UTF-8"),
            r#"{"a":"Hello!","b":false,"c":120}"#
        );

        let numbers = parse_unique_json(
            r#"{"numbers":[333333333.33333329,1E30,4.50,2e-3,0.000000000000000000000000001]}"#,
        )
        .expect("valid JCS number vector");
        let canonical = canonical_json_bytes(&numbers).expect("canonicalization");
        assert_eq!(
            std::str::from_utf8(&canonical).expect("canonical JSON is UTF-8"),
            r#"{"numbers":[333333333.3333333,1e+30,4.5,0.002,1e-27]}"#
        );

        // RFC 8785 Appendix C's representative key ordering, including UTF-16
        // ordering across a non-BMP emoji and BMP characters.
        let unicode = parse_unique_json(
            r#"{"€":"Euro Sign","\r":"Carriage Return","דּ":"Hebrew Letter Dalet With Dagesh","1":"One","😀":"Emoji: Grinning Face","\u0080":"Control","ö":"Latin Small Letter O With Diaeresis"}"#,
        )
        .expect("valid JCS Unicode key-order input");
        let canonical = String::from_utf8(
            canonical_json_bytes(&unicode).expect("canonicalization"),
        )
        .expect("canonical JSON is UTF-8");
        let control_key = format!("\"{}\":\"Control\"", '\u{0080}');
        let key_positions = [
            canonical.find(r#""\r":"Carriage Return""#).expect("CR key"),
            canonical.find(r#""1":"One""#).expect("numeric key"),
            canonical.find(&control_key).expect("C1 key"),
            canonical.find(r#""ö":"Latin Small Letter O With Diaeresis""#).expect("ö key"),
            canonical.find(r#""€":"Euro Sign""#).expect("euro key"),
            canonical.find(r#""😀":"Emoji: Grinning Face""#).expect("emoji key"),
            canonical.find(r#""דּ":"Hebrew Letter Dalet With Dagesh""#).expect("Hebrew key"),
        ];
        assert!(
            key_positions.windows(2).all(|pair| pair[0] < pair[1]),
            "JCS keys must use UTF-16 lexical order: {canonical}"
        );
    }

    #[test]
    fn provenance_payload_tampering_breaks_digest_verification() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        value["example_observation"]["payload"]["value"] = "2".into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("changed payload must no longer match the recorded digest");
        assert!(errors.iter().any(|error| {
            error.contains("payload digest must match SHA-256 of RFC 8785/JCS canonical payload bytes")
        }));
    }

    #[test]
    fn duplicate_json_member_names_are_rejected_even_when_escaped() {
        let duplicate = r#"{"schema_id":"urn:mycelix:provenance-envelope","schema_id":"urn:attacker:override"}"#;
        let errors = check_provenance_text(duplicate)
            .expect_err("duplicate object names must be rejected before value construction");
        assert!(errors.iter().any(|error| {
            error.contains("duplicate JSON object key")
        }));

        let escaped_duplicate = r#"{"schema_id":"first","\u0073chema_id":"second"}"#;
        let errors = check_provenance_text(escaped_duplicate)
            .expect_err("equivalent escaped names must also be duplicates");
        assert!(errors.iter().any(|error| {
            error.contains("duplicate JSON object key")
        }));
    }

    #[test]
    fn provenance_metadata_mutation_breaks_envelope_digest_even_when_payload_is_unchanged() {
        for (field, replacement) in [
            ("source_app", "mycelix.synthetic-fixture-altered"),
            ("source_record_id", "synthetic:observation-0002"),
            ("subject_id", "synthetic:subject-02"),
            ("actor_id", "synthetic:actor-02"),
            ("scope_id", "synthetic:scope-02"),
            ("event_id", "synthetic:event-0002"),
            ("observed_at", "2026-10-15T10:00:01Z"),
            ("recorded_at", "2026-10-15T10:00:04Z"),
            ("evidence_class", "reported"),
            ("privacy_class", "synthetic-private"),
            ("license_id", "CC-BY-4.0"),
            ("claim_ceiling", "This observation proves settlement."),
        ] {
            let mut value: serde_json::Value =
                serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
            value["example_observation"][field] = replacement.into();
            let errors = check_provenance_text(&value.to_string())
                .expect_err("changed envelope metadata must invalidate the envelope digest");
            assert!(
                errors.iter().any(|error| {
                    error.contains("example envelope digest must bind schema, identity, scope")
                }),
                "{field}: {errors:?}"
            );
        }
    }

    #[test]
    fn provenance_controls_cannot_drift_or_widen_guard_sets() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        let controls = value["negative_controls"].as_array_mut().expect("controls");
        let replay = controls
            .iter_mut()
            .find(|control| control["id"] == "PE-REPLAY-001")
            .expect("replay control");
        replay["required_guards"]
            .as_array_mut()
            .expect("guard list")
            .push("unreviewed_guard".into());
        let errors = check_provenance_text(&value.to_string())
            .expect_err("unreviewed guards must fail closed");
        assert!(errors.iter().any(|error| error.contains("registered ordered guards")));
    }

    #[test]
    fn synthetic_observations_cannot_be_promoted_to_real_evidence() {
        let mut value: serde_json::Value =
            serde_json::from_str(EMBEDDED_PROVENANCE_FIXTURE).expect("fixture JSON");
        value["example_observation"]["evidence_class"] = "independently_corroborated".into();
        let errors = check_provenance_text(&value.to_string())
            .expect_err("the sample must remain a synthetic-only fixture");
        assert!(errors
            .iter()
            .any(|error| error.contains("reference example must remain synthetic")));
    }

}
