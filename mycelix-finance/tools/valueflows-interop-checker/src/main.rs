use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs, process,
};

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
            "MYCEL.transferability=non_transferable_recognition_attestation",
            "MYCEL.quantity_encoding=none_attestation_only",
            "MYCEL.maps_to_authoritative_balance=false",
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
}
