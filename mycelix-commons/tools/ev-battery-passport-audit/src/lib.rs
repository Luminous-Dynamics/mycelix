use jsonschema::{draft202012, Validator};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

const SCHEMA_ID: &str =
    "https://luminousdynamics.io/schemas/transport/ev-battery-passport-projection/0.1";
const CORPUS_VERSION: &str = "ev-battery-passport-projection-corpus/0.1";
const PROTOCOL_VERSION: &str = "ev-battery-passport-projection/0.1";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Corpus {
    corpus_version: String,
    schema_id: String,
    cases: Vec<CorpusCase>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CorpusCase {
    case_id: String,
    expectation: String,
    manifest: Value,
}

#[derive(Debug, Serialize)]
struct CaseAudit {
    case_id: String,
    expectation: String,
    accepted: bool,
    errors: Vec<String>,
    passport_id: Option<String>,
    lifecycle_status: Option<String>,
    projected_field_count: usize,
    claim_ceiling: String,
}

#[derive(Debug, Serialize)]
struct ReportPayload {
    corpus_version: String,
    schema_id: String,
    schema_sha256: String,
    canonical_corpus_sha256: String,
    case_count: usize,
    accepted_count: usize,
    rejected_count: usize,
    cases: Vec<CaseAudit>,
}

#[derive(Debug, Serialize)]
pub struct CorpusReport {
    #[serde(flatten)]
    payload: ReportPayload,
    report_sha256: String,
}

fn sha256(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn is_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_evidence_class(value: &str) -> bool {
    matches!(
        value,
        "physical_observation"
            | "derived_from_physical"
            | "simulation"
            | "synthetic_fixture"
            | "vendor_claim"
            | "legal_record"
    )
}

fn opt_string(value: &Value, key: &str) -> Option<String> {
    value.get(key).and_then(Value::as_str).map(str::to_owned)
}

fn schema_validator(bytes: &[u8]) -> Result<Validator, String> {
    let schema: Value =
        serde_json::from_slice(bytes).map_err(|error| format!("schema JSON: {error}"))?;
    if schema.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        return Err("schema must declare JSON Schema Draft 2020-12".to_owned());
    }
    if schema.get("$id").and_then(Value::as_str) != Some(SCHEMA_ID) {
        return Err("schema ID does not match Battery Passport projection v0.1".to_owned());
    }
    if !draft202012::meta::is_valid(&schema) {
        return Err("schema fails Draft 2020-12 meta-schema validation".to_owned());
    }
    draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| format!("schema compilation: {error}"))
}

fn push_error(errors: &mut Vec<String>, message: impl Into<String>) {
    errors.push(message.into());
}

fn validate_finished_battery(manifest: &Value, errors: &mut Vec<String>) {
    let battery = &manifest["finished_battery"];
    let packs = battery["packs"].as_array().cloned().unwrap_or_default();
    let boundary = battery["boundary_kind"].as_str().unwrap_or_default();

    let mut pack_refs = BTreeSet::new();
    for pack in &packs {
        let reference = pack["pack_ref"].as_str().unwrap_or_default();
        let digest = pack["configuration_digest"].as_str().unwrap_or_default();
        if !nonempty(reference) || !pack_refs.insert(reference.to_owned()) || !is_digest(digest) {
            push_error(
                errors,
                "finished-battery pack inventory contains an invalid or duplicate pack reference",
            );
        }
    }

    match boundary {
        "single_pack_with_bms" | "independent_finished_pack" if packs.len() != 1 => {
            push_error(errors, "single-pack boundary must contain exactly one pack");
        }
        "multiple_packs_common_bms" if packs.len() < 2 => {
            push_error(
                errors,
                "multiple-packs/common-BMS boundary requires at least two packs",
            );
        }
        "single_pack_with_bms" | "multiple_packs_common_bms" | "independent_finished_pack" => {}
        _ => push_error(errors, "unknown finished-battery boundary kind"),
    }

    if !nonempty(battery["bms_ref"].as_str().unwrap_or_default())
        || !is_digest(battery["bms_digest"].as_str().unwrap_or_default())
        || !is_digest(battery["identity_digest"].as_str().unwrap_or_default())
        || !is_digest(
            battery["boundary_evidence_digest"]
                .as_str()
                .unwrap_or_default(),
        )
        || !nonempty(
            battery["boundary_rationale"]
                .as_str()
                .unwrap_or_default(),
        )
    {
        push_error(
            errors,
            "finished-battery identity, BMS, rationale or boundary evidence is incomplete",
        );
    }
    if !battery["nominal_energy_kwh"]
        .as_f64()
        .is_some_and(|value| value.is_finite() && value > 0.0)
    {
        push_error(
            errors,
            "finished-battery nominal energy must be finite and positive",
        );
    }
}

fn validate_responsibility(manifest: &Value, errors: &mut Vec<String>) {
    let responsibility = &manifest["responsibility"];
    let operator = responsibility["economic_operator_ref"]
        .as_str()
        .unwrap_or_default();
    let host = responsibility["passport_host_ref"].as_str().unwrap_or_default();
    let operator_digest = responsibility["economic_operator_identity_digest"]
        .as_str()
        .unwrap_or_default();
    let host_digest = responsibility["passport_host_identity_digest"]
        .as_str()
        .unwrap_or_default();
    let host_role = responsibility["host_role"].as_str().unwrap_or_default();
    let evaluation_time = manifest["evaluation_time_ms"].as_u64().unwrap_or_default();
    let delegation = &responsibility["delegation"];

    if !nonempty(operator)
        || !nonempty(host)
        || !is_digest(operator_digest)
        || !is_digest(host_digest)
    {
        push_error(
            errors,
            "economic operator or passport host identity is incomplete",
        );
    }

    match host_role {
        "responsible_operator" => {
            if host != operator || !delegation.is_null() {
                push_error(
                    errors,
                    "responsible-operator hosting must use the responsible operator identity and no delegation record",
                );
            }
        }
        "authorized_operator" => {
            let issuer = delegation["issuer_operator_ref"].as_str().unwrap_or_default();
            let delegate = delegation["delegate_operator_ref"].as_str().unwrap_or_default();
            let state = delegation["state"].as_str().unwrap_or_default();
            let from = delegation["valid_from_ms"].as_u64().unwrap_or_default();
            let to = delegation["valid_to_ms"].as_u64().unwrap_or_default();
            let revoked = !delegation["revoked_at_ms"].is_null();
            let scope: BTreeSet<&str> = delegation["scope"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
                .collect();

            if host == operator
                || issuer != operator
                || delegate != host
                || !is_digest(delegation["mandate_digest"].as_str().unwrap_or_default())
                || !nonempty(delegation["mandate_ref"].as_str().unwrap_or_default())
                || from == 0
                || to <= from
                || evaluation_time < from
                || evaluation_time > to
                || state != "active"
                || revoked
                || !scope.contains("maintain_passport")
                || !scope.contains("publish_projection")
            {
                push_error(
                    errors,
                    "authorized passport host lacks a current, correctly scoped written mandate from the responsible economic operator",
                );
            }
        }
        _ => push_error(errors, "unknown passport host role"),
    }
}

fn validate_event_delegation(
    event: &Value,
    responsibility: &Value,
    errors: &mut Vec<String>,
) {
    let role = event["actor_role"].as_str().unwrap_or_default();
    let actor = event["actor_ref"].as_str().unwrap_or_default();
    let operator = responsibility["economic_operator_ref"]
        .as_str()
        .unwrap_or_default();

    if role == "economic_operator" {
        if actor != operator {
            push_error(
                errors,
                "economic-operator lifecycle event actor does not match the responsible operator",
            );
        }
        if !event["mandate_ref"].is_null() || !event["mandate_digest"].is_null() {
            push_error(
                errors,
                "direct economic-operator event must not carry a delegated mandate",
            );
        }
        return;
    }

    if role != "authorized_operator" {
        push_error(
            errors,
            "passport-mutating lifecycle events must be authored by the economic operator or its authorized operator",
        );
        return;
    }

    let mandate = &responsibility["delegation"];
    let scope: BTreeSet<&str> = mandate["scope"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    let event_mandate_ref = event["mandate_ref"].as_str().unwrap_or_default();
    let event_mandate_digest = event["mandate_digest"].as_str().unwrap_or_default();
    if actor != mandate["delegate_operator_ref"].as_str().unwrap_or_default()
        || event_mandate_ref != mandate["mandate_ref"].as_str().unwrap_or_default()
        || event_mandate_digest != mandate["mandate_digest"].as_str().unwrap_or_default()
        || mandate["state"].as_str() != Some("active")
        || !mandate["revoked_at_ms"].is_null()
        || !scope.contains("maintain_passport")
    {
        push_error(
            errors,
            "delegated lifecycle event is not covered by the active mandate",
        );
    }

    let event_time = event["event_time_ms"].as_u64().unwrap_or_default();
    let from = mandate["valid_from_ms"].as_u64().unwrap_or_default();
    let to = mandate["valid_to_ms"].as_u64().unwrap_or_default();
    if event_time < from || event_time > to {
        push_error(
            errors,
            "delegated lifecycle event falls outside the mandate validity interval",
        );
    }
}

fn validate_lifecycle(manifest: &Value, errors: &mut Vec<String>) {
    let passport = &manifest["passport"];
    let responsibility = &manifest["responsibility"];
    let events = passport["events"].as_array().cloned().unwrap_or_default();

    let passport_id = passport["passport_id"].as_str().unwrap_or_default();
    let declared_status = passport["status"].as_str().unwrap_or_default();
    let evaluation_time = manifest["evaluation_time_ms"].as_u64().unwrap_or_default();
    let mut current_passport: Option<String> = None;
    let mut current_configuration: Option<String> = None;
    let mut current_operator: Option<String> = None;
    let mut last_timestamp = 0u64;
    let mut last_non_null_passport = None;
    let mut last_predecessor = None;
    let mut last_creation_time = None;
    let mut recycled = false;

    if !nonempty(passport_id)
        || !is_digest(
            passport["current_configuration_digest"]
                .as_str()
                .unwrap_or_default(),
        )
    {
        push_error(
            errors,
            "passport identity or current configuration digest is invalid",
        );
    }

    for (index, event) in events.iter().enumerate() {
        let event_id = event["event_id"].as_str().unwrap_or_default();
        let kind = event["event_kind"].as_str().unwrap_or_default();
        let time = event["event_time_ms"].as_u64().unwrap_or_default();
        let before = opt_string(event, "passport_id_before");
        let after = opt_string(event, "passport_id_after");
        let config_before = opt_string(event, "configuration_before_digest");
        let config_after = opt_string(event, "configuration_after_digest");
        let operator_before = opt_string(event, "responsible_operator_before_ref");
        let operator_after = opt_string(event, "responsible_operator_after_ref");

        if !nonempty(event_id)
            || time == 0
            || time <= last_timestamp
            || time > evaluation_time
            || !is_digest(event["actor_identity_digest"].as_str().unwrap_or_default())
            || !is_digest(event["evidence_digest"].as_str().unwrap_or_default())
            || !valid_evidence_class(event["evidence_class"].as_str().unwrap_or_default())
            || !nonempty(event["actor_ref"].as_str().unwrap_or_default())
        {
            push_error(
                errors,
                format!("lifecycle event {index} has invalid identity, time, actor or evidence"),
            );
        }
        last_timestamp = time;

        if recycled {
            push_error(errors, "no lifecycle event may follow recycling completion");
            continue;
        }
        if before != current_passport {
            push_error(
                errors,
                format!("lifecycle event {event_id} does not reference the current predecessor passport"),
            );
        }
        if let (Some(expected), Some(actual)) = (&current_configuration, &config_before) {
            if expected != actual {
                push_error(
                    errors,
                    format!("lifecycle event {event_id} configuration-before digest does not match the preceding event"),
                );
            }
        }
        if config_before.is_some() && !config_before.as_deref().is_some_and(is_digest) {
            push_error(
                errors,
                format!("lifecycle event {event_id} has invalid configuration-before digest"),
            );
        }
        if config_after.is_some() && !config_after.as_deref().is_some_and(is_digest) {
            push_error(
                errors,
                format!("lifecycle event {event_id} has invalid configuration-after digest"),
            );
        }
        if operator_before != current_operator {
            push_error(
                errors,
                format!("lifecycle event {event_id} responsible-operator predecessor does not match the event chain"),
            );
        }

        validate_event_delegation(event, responsibility, errors);

        match kind {
            "placed_on_market" => {
                if index != 0
                    || before.is_some()
                    || after.as_deref() != Some(passport_id)
                    || config_before.is_some()
                    || config_after.is_none()
                    || operator_before.is_some()
                    || operator_after.as_deref()
                        != responsibility["economic_operator_ref"].as_str()
                {
                    push_error(
                        errors,
                        "placed-on-market event must establish the initial passport, configuration and responsible operator",
                    );
                }
                current_passport = after.clone();
                current_configuration = config_after.clone();
                current_operator = operator_after.clone();
                last_creation_time = Some(time);
                last_non_null_passport = after;
            }
            "simple_repair" | "status_change" | "inspection" | "diagnostic_test" | "maintenance" => {
                if before.is_none()
                    || before != after
                    || config_after.is_none()
                    || operator_before != operator_after
                {
                    push_error(
                        errors,
                        format!("{kind} must preserve passport identity and responsible operator while recording a resulting configuration"),
                    );
                }
                current_configuration = config_after.clone().or(current_configuration);
            }
            "remanufacture" | "repurpose" | "waste_prepared_for_reuse" => {
                if before.is_none()
                    || after.is_none()
                    || before == after
                    || config_after.is_none()
                    || operator_after.is_none()
                {
                    push_error(
                        errors,
                        format!("{kind} must create a new passport linked to a predecessor and a resulting configuration"),
                    );
                } else {
                    last_predecessor = before.clone();
                    last_creation_time = Some(time);
                    current_passport = after.clone();
                    current_configuration = config_after.clone();
                    current_operator = operator_after.clone();
                    last_non_null_passport = after;
                }
            }
            "waste_transfer" => {
                if before.is_none()
                    || before != after
                    || operator_before.is_none()
                    || operator_after.is_none()
                    || operator_before == operator_after
                    || config_after != config_before
                {
                    push_error(
                        errors,
                        "waste transfer must retain passport/configuration identity and explicitly transfer responsible-operator lineage",
                    );
                } else {
                    current_operator = operator_after.clone();
                    current_configuration = config_after.clone().or(current_configuration);
                }
            }
            "recycling_completed" => {
                if before.is_none()
                    || after.is_some()
                    || config_before.is_none()
                    || config_after.is_some()
                    || operator_before != operator_after
                    || index + 1 != events.len()
                {
                    push_error(
                        errors,
                        "recycling completion must close the current passport as the terminal lifecycle event",
                    );
                } else {
                    recycled = true;
                    current_passport = None;
                    current_configuration = None;
                }
            }
            _ => push_error(errors, format!("unknown lifecycle event kind {kind}")),
        }
        current_operator = current_operator.or(operator_after);
    }

    if last_non_null_passport.as_deref() != Some(passport_id) {
        push_error(
            errors,
            "passport ID does not match the final non-cessation lifecycle event",
        );
    }
    if current_configuration.as_deref()
        != passport["current_configuration_digest"].as_str()
        && !recycled
    {
        push_error(
            errors,
            "passport current configuration does not match the final lifecycle event",
        );
    }
    if current_operator.as_deref()
        != responsibility["economic_operator_ref"].as_str()
    {
        push_error(
            errors,
            "responsible economic operator does not match the final lifecycle event",
        );
    }
    if recycled != (declared_status == "ceased") {
        push_error(
            errors,
            "passport status must be ceased exactly when recycling has completed",
        );
    }
    if !recycled && declared_status != "active" {
        push_error(errors, "non-recycled passport must remain active");
    }

    let declared_predecessor = opt_string(passport, "predecessor_passport_id");
    if declared_predecessor != last_predecessor {
        push_error(
            errors,
            "successor passport predecessor reference does not match the lifecycle transition",
        );
    }
    if let Some(created) = passport["created_at_ms"].as_u64() {
        if last_creation_time != Some(created) {
            push_error(
                errors,
                "passport creation time does not match placement or successor transition",
            );
        }
    } else {
        push_error(errors, "passport creation timestamp is missing");
    }
}

fn validate_applicability_and_access(manifest: &Value, errors: &mut Vec<String>) {
    let catalog = &manifest["applicability_catalog"];
    let model_version = manifest["data_model"]["version"].as_str().unwrap_or_default();
    let catalog_version = catalog["data_model_version"].as_str().unwrap_or_default();
    let evaluation_time = manifest["evaluation_time_ms"].as_u64().unwrap_or_default();
    let audience = manifest["audience"].as_str().unwrap_or_default();
    let policy = &manifest["access_policy"];
    let policy_status = policy["legal_status"].as_str().unwrap_or_default();
    let allowed: BTreeSet<&str> = policy["allowed_data_point_ids"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
        .collect();
    let points = catalog["points"].as_array().cloned().unwrap_or_default();
    let fields = manifest["projected_fields"].as_array().cloned().unwrap_or_default();

    if model_version != catalog_version
        || !is_digest(catalog["catalog_digest"].as_str().unwrap_or_default())
        || !is_digest(policy["policy_digest"].as_str().unwrap_or_default())
        || !nonempty(policy["policy_ref"].as_str().unwrap_or_default())
        || !nonempty(policy["legal_basis_ref"].as_str().unwrap_or_default())
    {
        push_error(
            errors,
            "applicability catalog, data model or access policy identity is inconsistent",
        );
    }
    if !is_digest(manifest["rulebook"]["source_digest"].as_str().unwrap_or_default())
        || !nonempty(manifest["rulebook"]["version"].as_str().unwrap_or_default())
    {
        push_error(errors, "rulebook version or source digest is missing");
    }

    let mut point_map = std::collections::BTreeMap::new();
    for point in &points {
        let id = point["data_point_id"].as_str().unwrap_or_default();
        let status = point["applicability"].as_str().unwrap_or_default();
        if !nonempty(id) || point_map.insert(id.to_owned(), point).is_some() {
            push_error(
                errors,
                "applicability catalog contains an empty or duplicate data-point ID",
            );
        }
        if !matches!(
            status,
            "required_at_launch"
                | "conditional_required"
                | "optional"
                | "deferred"
                | "not_applicable"
        ) {
            push_error(
                errors,
                format!("data point {id} has an unknown applicability status"),
            );
        }
    }

    let mut emitted = std::collections::BTreeMap::new();
    for field in &fields {
        let id = field["data_point_id"].as_str().unwrap_or_default();
        let value_digest = field["value_digest"].as_str().unwrap_or_default();
        let source_digest = field["source_evidence_digest"].as_str().unwrap_or_default();
        let class = field["evidence_class"].as_str().unwrap_or_default();

        if !nonempty(id)
            || emitted.insert(id.to_owned(), field).is_some()
            || !is_digest(value_digest)
            || !is_digest(source_digest)
            || !valid_evidence_class(class)
            || field["data_model_version"].as_str() != Some(model_version)
        {
            push_error(
                errors,
                format!("projected field {id} has duplicate identity, invalid value/source evidence or wrong schema version"),
            );
        }

        let Some(point) = point_map.get(id) else {
            push_error(
                errors,
                format!("projected field {id} is absent from the pinned applicability catalog"),
            );
            continue;
        };
        let status = point["applicability"].as_str().unwrap_or_default();
        let access = point["access_tier"].as_str().unwrap_or_default();
        let effective = point["effective_from_ms"].as_u64().unwrap_or_default();
        let condition = point["condition_met"].as_bool().unwrap_or(false);

        if status == "deferred" || status == "not_applicable" {
            push_error(
                errors,
                format!("deferred/non-applicable field {id} must be absent from the active projection"),
            );
        }
        if status == "conditional_required" && !condition {
            push_error(
                errors,
                format!("conditional data point {id} was emitted although its applicability condition is false"),
            );
        }
        if effective > evaluation_time {
            push_error(
                errors,
                format!("data point {id} is not yet effective at the projection time"),
            );
        }

        let permitted = match access {
            "public" => true,
            "authority_only" => {
                matches!(audience, "authority" | "economic_operator" | "delegated_host")
            }
            "restricted" => {
                matches!(audience, "economic_operator" | "delegated_host")
                    || (policy_status == "finalized" && allowed.contains(id))
            }
            "legitimate_interest_pending" => {
                audience == "legitimate_interest"
                    && policy_status == "finalized"
                    && allowed.contains(id)
            }
            _ => false,
        };
        if !permitted {
            push_error(
                errors,
                format!("audience {audience} is not authorized to receive data point {id} under the pinned access policy"),
            );
        }
        if access == "legitimate_interest_pending" && policy_status != "finalized" {
            push_error(
                errors,
                format!("legitimate-interest data point {id} is not publishable while its access policy is pending"),
            );
        }
    }

    for point in &points {
        let id = point["data_point_id"].as_str().unwrap_or_default();
        let status = point["applicability"].as_str().unwrap_or_default();
        let condition = point["condition_met"].as_bool().unwrap_or(false);
        let effective = point["effective_from_ms"].as_u64().unwrap_or_default();
        let required = status == "required_at_launch"
            || (status == "conditional_required" && condition);
        if required && effective <= evaluation_time && !emitted.contains_key(id) {
            push_error(
                errors,
                format!("required data point {id} is missing from the projection"),
            );
        }
        if (status == "deferred" || status == "not_applicable") && emitted.contains_key(id) {
            push_error(
                errors,
                format!("deferred/non-applicable data point {id} must not appear in the projection"),
            );
        }
    }
}

fn validate_registry_boundary(manifest: &Value, errors: &mut Vec<String>) {
    let registry = &manifest["registry_packet"];
    let passport = &manifest["passport"];
    let responsibility = &manifest["responsibility"];
    if registry["record_kind"].as_str() != Some("registration_metadata_only")
        || registry["includes_full_passport_payload"].as_bool() != Some(false)
        || !is_digest(
            registry["registration_metadata_digest"]
                .as_str()
                .unwrap_or_default(),
        )
        || registry["passport_id"].as_str() != passport["passport_id"].as_str()
        || registry["economic_operator_ref"].as_str()
            != responsibility["economic_operator_ref"].as_str()
        || registry["host_ref"].as_str() != responsibility["passport_host_ref"].as_str()
        || registry["rulebook_digest"].as_str()
            != manifest["rulebook"]["source_digest"].as_str()
    {
        push_error(
            errors,
            "registry packet must contain registration metadata only and match the current passport, operator, host and rulebook",
        );
    }
}

fn validate_manifest(manifest: &Value) -> Vec<String> {
    let mut errors = Vec::new();

    if manifest["protocol_version"].as_str() != Some(PROTOCOL_VERSION)
        || !nonempty(manifest["manifest_id"].as_str().unwrap_or_default())
        || manifest["evaluation_time_ms"].as_u64().unwrap_or_default() == 0
    {
        push_error(
            &mut errors,
            "protocol version, manifest ID or evaluation time is invalid",
        );
    }
    if manifest["claims"]["legal_compliance_claim"].as_bool() != Some(false)
        || manifest["claims"]["safety_qualification_claim"].as_bool() != Some(false)
    {
        push_error(
            &mut errors,
            "schema validation must not be promoted into legal compliance or safety qualification claims",
        );
    }

    validate_finished_battery(manifest, &mut errors);
    validate_responsibility(manifest, &mut errors);
    validate_lifecycle(manifest, &mut errors);
    validate_applicability_and_access(manifest, &mut errors);
    validate_registry_boundary(manifest, &mut errors);
    errors
}

fn canonicalize_corpus(value: &mut Value) {
    let Some(cases) = value.get_mut("cases").and_then(Value::as_array_mut) else {
        return;
    };
    cases.sort_by(|left, right| {
        left.get("case_id")
            .and_then(Value::as_str)
            .cmp(&right.get("case_id").and_then(Value::as_str))
    });
    for case in cases {
        if let Some(manifest) = case.get_mut("manifest") {
            if let Some(rows) = manifest
                .get_mut("finished_battery")
                .and_then(|battery| battery.get_mut("packs"))
                .and_then(Value::as_array_mut)
            {
                rows.sort_by(|left, right| {
                    left.get("pack_ref")
                        .and_then(Value::as_str)
                        .cmp(&right.get("pack_ref").and_then(Value::as_str))
                });
            }
            if let Some(rows) = manifest
                .get_mut("projected_fields")
                .and_then(Value::as_array_mut)
            {
                rows.sort_by(|left, right| {
                    left.get("data_point_id")
                        .and_then(Value::as_str)
                        .cmp(&right.get("data_point_id").and_then(Value::as_str))
                });
            }
            if let Some(rows) = manifest
                .get_mut("applicability_catalog")
                .and_then(|catalog| catalog.get_mut("points"))
                .and_then(Value::as_array_mut)
            {
                rows.sort_by(|left, right| {
                    left.get("data_point_id")
                        .and_then(Value::as_str)
                        .cmp(&right.get("data_point_id").and_then(Value::as_str))
                });
            }
            if let Some(rows) = manifest
                .get_mut("access_policy")
                .and_then(|policy| policy.get_mut("allowed_data_point_ids"))
                .and_then(Value::as_array_mut)
            {
                rows.sort_by(|left, right| left.as_str().cmp(&right.as_str()));
            }
        }
    }
}

pub fn audit_corpus(schema_bytes: &[u8], corpus_bytes: &[u8]) -> Result<CorpusReport, String> {
    let validator = schema_validator(schema_bytes)?;
    let mut corpus_value: Value =
        serde_json::from_slice(corpus_bytes).map_err(|error| format!("corpus JSON: {error}"))?;
    canonicalize_corpus(&mut corpus_value);
    let canonical = serde_json::to_vec(&corpus_value)
        .map_err(|error| format!("canonical corpus: {error}"))?;
    let corpus: Corpus =
        serde_json::from_slice(&canonical).map_err(|error| format!("typed corpus: {error}"))?;
    if corpus.corpus_version != CORPUS_VERSION || corpus.schema_id != SCHEMA_ID {
        return Err("corpus version or schema ID mismatch".to_owned());
    }

    let mut seen = BTreeSet::new();
    let mut cases = Vec::new();
    let mut accepted_count = 0usize;
    let mut rejected_count = 0usize;

    for case in corpus.cases {
        if !nonempty(&case.case_id) || !seen.insert(case.case_id.clone()) {
            return Err(format!("case identity is empty or duplicated: {}", case.case_id));
        }
        if !matches!(case.expectation.as_str(), "accept" | "reject") {
            return Err(format!("case {} has an unknown expectation", case.case_id));
        }

        let schema_errors: Vec<String> = validator
            .iter_errors(&case.manifest)
            .map(|error| error.to_string())
            .collect();
        let errors = if schema_errors.is_empty() {
            validate_manifest(&case.manifest)
        } else {
            schema_errors
        };
        let accepted = errors.is_empty();
        if accepted != (case.expectation == "accept") {
            let detail = if accepted {
                "unexpected acceptance".to_owned()
            } else {
                errors.join("; ")
            };
            return Err(format!(
                "case {} expectation mismatch: {}",
                case.case_id, detail
            ));
        }
        if accepted {
            accepted_count += 1;
        } else {
            rejected_count += 1;
        }
        cases.push(CaseAudit {
            case_id: case.case_id,
            expectation: case.expectation,
            accepted,
            errors,
            passport_id: opt_string(&case.manifest["passport"], "passport_id"),
            lifecycle_status: opt_string(&case.manifest["passport"], "status"),
            projected_field_count: case.manifest["projected_fields"]
                .as_array()
                .map_or(0, Vec::len),
            claim_ceiling: "projection_readiness_only_not_legal_or_safety_qualification".to_owned(),
        });
    }

    let payload = ReportPayload {
        corpus_version: CORPUS_VERSION.to_owned(),
        schema_id: SCHEMA_ID.to_owned(),
        schema_sha256: sha256(schema_bytes),
        canonical_corpus_sha256: sha256(&canonical),
        case_count: cases.len(),
        accepted_count,
        rejected_count,
        cases,
    };
    let bytes =
        serde_json::to_vec(&payload).map_err(|error| format!("report serialization: {error}"))?;
    Ok(CorpusReport {
        payload,
        report_sha256: sha256(&bytes),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    const SCHEMA: &[u8] =
        include_bytes!("../../../schemas/ev_battery_passport_projection_v0_1.schema.json");
    const CORPUS: &[u8] =
        include_bytes!("../../../fixtures/ev_battery/passport_projection_scenarios_v0_1.json");

    fn report() -> CorpusReport {
        audit_corpus(SCHEMA, CORPUS).expect("frozen Battery Passport projection corpus")
    }

    #[test]
    fn frozen_corpus_accepts_four_and_rejects_fourteen_controls() {
        let report = report();
        assert_eq!(report.payload.case_count, 18);
        assert_eq!(report.payload.accepted_count, 4);
        assert_eq!(report.payload.rejected_count, 14);
    }

    #[test]
    fn repair_retains_passport_identity_and_successor_events_link_predecessor() {
        let report = report();
        let repair = report.payload.cases.iter()
            .find(|case| case.case_id == "simple-repair-retains-passport")
            .expect("simple repair case");
        let repurpose = report.payload.cases.iter()
            .find(|case| case.case_id == "repurpose-creates-linked-successor")
            .expect("repurpose case");
        assert!(repair.accepted);
        assert!(repurpose.accepted);
        assert_eq!(repair.passport_id.as_deref(), Some("passport-ev-001"));
        assert_eq!(repurpose.passport_id.as_deref(), Some("passport-ev-002"));
    }

    #[test]
    fn authorized_host_does_not_replace_the_responsible_economic_operator() {
        let report = report();
        let allowed = report.payload.cases.iter()
            .find(|case| case.case_id == "authorized-host-with-active-mandate")
            .expect("authorized host case");
        let rejected = report.payload.cases.iter()
            .find(|case| case.case_id == "delegation-must-not-transfer-accountability")
            .expect("accountability control");
        assert!(allowed.accepted);
        assert!(!rejected.accepted);
    }

    #[test]
    fn applicability_and_access_are_enforced_before_projection() {
        let report = report();
        for case_id in [
            "required-data-point-missing",
            "not-applicable-field-emitted",
            "deferred-field-emitted",
            "legitimate-interest-policy-pending",
            "projected-field-needs-source-evidence",
            "unknown-data-point-rejected",
        ] {
            let case = report.payload.cases.iter()
                .find(|case| case.case_id == case_id)
                .expect("applicability/access control");
            assert!(!case.accepted);
        }
    }

    #[test]
    fn registry_is_metadata_only_and_recycling_ends_the_passport() {
        let report = report();
        for case_id in [
            "registry-must-not-host-full-payload",
            "recycling-completed-must-cease",
        ] {
            let case = report.payload.cases.iter()
                .find(|case| case.case_id == case_id)
                .expect("registry/lifecycle control");
            assert!(!case.accepted);
        }
    }

    #[test]
    fn expired_revoked_and_scope_incomplete_mandates_are_rejected() {
        let report = report();
        for case_id in [
            "expired-delegate-mandate",
            "revoked-delegate-mandate",
        ] {
            let case = report.payload.cases.iter()
                .find(|case| case.case_id == case_id)
                .expect("mandate control");
            assert!(!case.accepted);
        }
    }

    #[test]
    fn lifecycle_state_machine_rejects_invalid_repair_repurpose_and_event_time() {
        let report = report();
        for case_id in [
            "repair-must-not-create-new-passport",
            "repurpose-missing-predecessor",
            "lifecycle-events-must-be-chronological",
        ] {
            let case = report.payload.cases.iter()
                .find(|case| case.case_id == case_id)
                .expect("lifecycle control");
            assert!(!case.accepted);
        }
    }

    #[test]
    fn canonical_report_is_invariant_to_noncausal_array_order() {
        let original = report();
        let mut value: Value = serde_json::from_slice(CORPUS).expect("corpus JSON");
        value["cases"].as_array_mut().unwrap().reverse();
        for case in value["cases"].as_array_mut().unwrap() {
            case["manifest"]["projected_fields"].as_array_mut().unwrap().reverse();
            case["manifest"]["applicability_catalog"]["points"]
                .as_array_mut().unwrap().reverse();
            case["manifest"]["finished_battery"]["packs"]
                .as_array_mut().unwrap().reverse();
            case["manifest"]["access_policy"]["allowed_data_point_ids"]
                .as_array_mut().unwrap().reverse();
        }
        let bytes = serde_json::to_vec(&value).expect("permuted corpus");
        let replay = audit_corpus(SCHEMA, &bytes).expect("permuted projection audit");
        assert_eq!(original.report_sha256, replay.report_sha256);
        assert_eq!(
            original.payload.canonical_corpus_sha256,
            replay.payload.canonical_corpus_sha256
        );
    }
}
