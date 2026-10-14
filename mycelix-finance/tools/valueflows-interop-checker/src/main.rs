use serde::Deserialize;
use std::{collections::{BTreeMap, BTreeSet}, env, fs, process};

const DEFAULT_CORPUS: &str =
    "mycelix-finance/tests/reference/valueflows-interop/mapping-corpus-v1.json";
const EMBEDDED_CORPUS: &str =
    include_str!("../../../tests/reference/valueflows-interop/mapping-corpus-v1.json");

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
    normative_specification_url: String,
    hrea_repository: String,
    hrea_source_commit: String,
    hrea_source_branch: String,
    hrea_source_describes_profile_as: String,
    hrea_source_commit_url: String,
    packaged_hrea_artifact_pinned: bool,
    hrea_runtime_qualified_with_mycelix: bool,
    version_note: String,
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
    transferable: bool,
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
        _ => None,
    }
}

fn expected_mapping(
    instrument: &str,
) -> Option<(&'static str, &'static str, &'static str, bool, &'static [&'static str])> {
    match instrument {
        "SAP" => Some((
            "mycelix:SAP",
            "decimal_string_micro_sap",
            "mycelix:microSAP",
            true,
            &["ResourceSpecification", "EconomicResource", "EconomicEvent"],
        )),
        "TEND" => Some((
            "mycelix:TEND",
            "blocked_legacy_f32_needs_quantization_profile",
            "mycelix:TEND-hour",
            true,
            &["Commitment", "EconomicEvent"],
        )),
        "MYCEL" => Some((
            "mycelix:MYCEL",
            "none_attestation_only",
            "not_applicable",
            false,
            &["Claim"],
        )),
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
            "MYCEL.transferable=false",
            "MYCEL.quantity_encoding=none_attestation_only",
            "MYCEL.maps_to_authoritative_balance=false",
        ]),
        "VF-TEND-001" => Some(&[
            "TEND.identity_preserved",
            "TEND.unit_preserved",
            "no_implicit_TEND_to_SAP_conversion",
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
        _ => None,
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
    fail(&mut errors, authority.export_only, "authority_boundary.export_only must be true".into());
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
            && source.published_ontology_version_iri
                == "https://w3id.org/valueflows/ont/0.17/vf",
        "published ontology version and version IRI must remain explicitly pinned to 0.17".into(),
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
        source.hrea_source_commit.len() == 40
            && source.hrea_source_commit.chars().all(|ch| ch.is_ascii_hexdigit()),
        "hREA source commit must be a full 40-character hexadecimal SHA".into(),
    );
    fail(
        &mut errors,
        source.hrea_source_commit_url.ends_with(&source.hrea_source_commit),
        "hREA source commit URL must end with the pinned commit SHA".into(),
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
        !source.version_note.trim().is_empty(),
        "source profile must explain version/profile separation".into(),
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
    for required in [
        "source_system",
        "source_record_id",
        "source_action_or_event_id",
        "source_revision_or_sequence",
        "observed_at",
        "scope_id",
        "asset_or_instrument_identity",
        "quantity_and_unit_profile",
        "claim_ceiling",
    ] {
        fail(
            &mut errors,
            contract.required_envelope_fields.iter().any(|field| field == required),
            format!("required envelope field is missing: {required}"),
        );
    }
    fail(
        &mut errors,
        contract.quantity_policy.contains("base-10 integer strings")
            && contract.quantity_policy.contains("binary floating-point"),
        "quantity policy must preserve exact base-10 integer strings and reject float loss".into(),
    );
    fail(
        &mut errors,
        contract.identity_policy.contains("namespaced")
            && contract.identity_policy.contains("explicit"),
        "identity policy must require namespaced identifiers and explicit crosswalks".into(),
    );
    fail(
        &mut errors,
        contract.correction_policy.contains("append")
            && contract.correction_policy.contains("not mutate"),
        "correction policy must preserve append-only source history".into(),
    );
    fail(
        &mut errors,
        contract.semantics_policy.contains("authorization")
            && contract.semantics_policy.contains("settlement"),
        "semantic policy must distinguish facts, authorization and settlement".into(),
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
            !mapping.mycelix_identity.trim().is_empty()
                && !mapping.unit_profile.trim().is_empty(),
            format!("mapping {} must identify an instrument and unit profile", mapping.id),
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
            format!("mapping {} must not promote source events to ledger authority", mapping.id),
        );
        fail(
            &mut errors,
            !mapping.maps_to_authoritative_balance,
            format!("mapping {} must not map directly to authoritative balances", mapping.id),
        );

        if instruments.insert(mapping.mycelix_instrument.as_str(), mapping).is_some() {
            errors.push(format!("duplicate instrument mapping: {}", mapping.mycelix_instrument));
        }

        match expected_mapping(&mapping.mycelix_instrument) {
            Some((identity, encoding, unit_profile, transferable, entities)) => {
                fail(
                    &mut errors,
                    mapping.mycelix_identity == identity,
                    format!("{} has unexpected instrument identity", mapping.id),
                );
                fail(
                    &mut errors,
                    mapping.quantity_encoding == encoding,
                    format!("{} has an unsafe or unexpected quantity encoding", mapping.id),
                );
                fail(
                    &mut errors,
                    mapping.unit_profile == unit_profile,
                    format!("{} has an unexpected or lossy unit profile", mapping.id),
                );
                fail(
                    &mut errors,
                    mapping.transferable == transferable,
                    format!("{} has incorrect transferability semantics", mapping.id),
                );
                for entity in entities {
                    fail(
                        &mut errors,
                        mapping.valueflows_entities.iter().any(|candidate| candidate.as_str() == *entity),
                        format!("{} is missing required ValueFlows entity {entity}", mapping.id),
                    );
                }
                if mapping.mycelix_instrument == "MYCEL" {
                    fail(
                        &mut errors,
                        mapping.valueflows_entities.len() == 1
                            && mapping.valueflows_entities[0] == "Claim",
                        "MYCEL must map only to a scoped Claim/attestation in this profile".into(),
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
            instruments.contains_key(instrument),
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
        match expected_disposition(&control.id) {
            Some(expected) => fail(
                &mut errors,
                control.expected_disposition == expected,
                format!("negative control {} has incorrect expected disposition", control.id),
            ),
            None => errors.push(format!("unknown negative control: {}", control.id)),
        }
        match expected_guards(&control.id) {
            Some(required) => {
                for guard in required {
                    fail(
                        &mut errors,
                        control.required_guards.iter().any(|candidate| candidate.as_str() == *guard),
                        format!("negative control {} is missing required guard: {guard}", control.id),
                    );
                }
            }
            None => errors.push(format!("negative control {} has no registered guard contract", control.id)),
        }
    }
    for id in expected_controls {
        fail(
            &mut errors,
            control_ids.contains(id),
            format!("required negative control is missing: {id}"),
        );
    }
    fail(
        &mut errors,
        control_ids.len() == expected_controls.len(),
        "negative-control corpus v1 must contain exactly the 11 registered cases".into(),
    );

    errors
}

fn check_text(input: &str) -> Result<usize, Vec<String>> {
    let corpus: Corpus = match serde_json::from_str(input) {
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
        assert_eq!(count, 12);
    }

    #[test]
    fn external_authority_promotion_is_rejected() {
        let mut value = fixture();
        value["authority_boundary"]["external_facts_are_authoritative"] = true.into();
        let errors = check_text(&value.to_string()).expect_err("external authority must fail");
        assert!(errors.iter().any(|error| error.contains("external facts must never be authoritative")));
    }

    #[test]
    fn mycel_cannot_become_transferable_money() {
        let mut value = fixture();
        let mappings = value["mappings"].as_array_mut().expect("mappings array");
        let mycel = mappings
            .iter_mut()
            .find(|mapping| mapping["mycelix_instrument"] == "MYCEL")
            .expect("MYCEL mapping");
        mycel["transferable"] = true.into();
        let errors = check_text(&value.to_string()).expect_err("MYCEL transferability must fail");
        assert!(errors.iter().any(|error| error.contains("incorrect transferability semantics")));
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
        assert!(errors.iter().any(|error| error.contains("unsafe or unexpected quantity encoding")));
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
        assert!(errors.iter().any(|error| error.contains("unsafe or unexpected quantity encoding")));
    }

    #[test]
    fn valueflows_and_hrea_profile_versions_cannot_be_collapsed() {
        let mut value = fixture();
        value["source_profile"]["published_ontology_version"] = "1.0".into();
        let errors = check_text(&value.to_string()).expect_err("version conflation must fail");
        assert!(errors.iter().any(|error| error.contains("published ontology version")));
    }

    #[test]
    fn negative_control_dispositions_are_fixed_and_fail_closed() {
        let mut value = fixture();
        let controls = value["negative_controls"].as_array_mut().expect("controls array");
        let replay = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-REPLAY-001")
            .expect("replay control");
        replay["expected_disposition"] = "accept".into();
        let errors = check_text(&value.to_string()).expect_err("unsafe disposition must fail");
        assert!(errors.iter().any(|error| error.contains("incorrect expected disposition")));
    }

    #[test]
    fn unknown_fields_fail_closed_for_corpus_v1() {
        let mut value = fixture();
        value["unreviewed_authority_override"] = true.into();
        let errors = check_text(&value.to_string()).expect_err("unknown fields must fail");
        assert!(errors.iter().any(|error| error.contains("invalid corpus JSON/shape")));
    }

    #[test]
    fn required_negative_control_guards_cannot_be_removed() {
        let mut value = fixture();
        let controls = value["negative_controls"].as_array_mut().expect("controls array");
        let amount = controls
            .iter_mut()
            .find(|control| control["id"] == "VF-AMOUNT-001")
            .expect("amount control");
        amount["required_guards"] = serde_json::json!(["canonical_integer_string"]);
        let errors = check_text(&value.to_string()).expect_err("dropping exact-round-trip guards must fail");
        assert!(errors.iter().any(|error| error.contains("missing required guard")));
    }

    #[test]
    fn all_required_negative_controls_are_present_exactly_once() {
        let mut value = fixture();
        let controls = value["negative_controls"].as_array_mut().expect("controls array");
        controls.pop();
        let errors = check_text(&value.to_string()).expect_err("missing negative control must fail");
        assert!(errors.iter().any(|error| error.contains("required negative control is missing")));
    }
}
