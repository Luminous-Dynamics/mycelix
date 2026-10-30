//! Standalone source-pinned abstract input generator for Mycelix scientific policy.
//!
//! This tool does not import the broken mycelix-desci-core crate and does not
//! calculate expected allow/deny outcomes. It enumerates abstract contexts
//! derived from the pure DefaultScientificAuthorizationPolicy match space.
//! Every row remains pending evaluation by the authentic compiled Rust oracle.

use serde::Serialize;
use sha1::Sha1;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

pub const GENERATOR_ID: &str = "mycelix-scientific-policy-space";
pub const GENERATOR_VERSION: &str = "0.1.0";
pub const BASELINE_REVISION: &str = "f89b4bcdcfb901961e2ca11a577775acbf53a74d";
pub const BASELINE_SOURCE_PATH: &str = "mycelix-desci/src/core/src/scientific_governance.rs";
pub const BASELINE_SOURCE_BLOB_SHA: &str = "dd9ea9267cdae2e2c67244e8a90821defaacefb3";
pub const POLICY_ID: &str = "mycelix-default-scientific-authorization";
pub const POLICY_VERSION: &str = "1.0.0";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Action {
    ClaimProposed,
    LegacyClaimImported,
    EvidenceAttached,
    AttestationRecorded,
    AttestationCorrected,
    AttestationWithdrawn,
    ClaimSuperseded,
    ClaimRetracted,
}

impl Action {
    pub const ALL: [Self; 8] = [
        Self::ClaimProposed,
        Self::LegacyClaimImported,
        Self::EvidenceAttached,
        Self::AttestationRecorded,
        Self::AttestationCorrected,
        Self::AttestationWithdrawn,
        Self::ClaimSuperseded,
        Self::ClaimRetracted,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ClaimProposed => "claim_proposed",
            Self::LegacyClaimImported => "legacy_claim_imported",
            Self::EvidenceAttached => "evidence_attached",
            Self::AttestationRecorded => "attestation_recorded",
            Self::AttestationCorrected => "attestation_corrected",
            Self::AttestationWithdrawn => "attestation_withdrawn",
            Self::ClaimSuperseded => "claim_superseded",
            Self::ClaimRetracted => "claim_retracted",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Role {
    Contributor,
    Reviewer,
    Editor,
    Institution,
    Service,
    MigrationService,
    RegistryAdmin,
}

impl Role {
    pub const ALL: [Self; 7] = [
        Self::Contributor,
        Self::Reviewer,
        Self::Editor,
        Self::Institution,
        Self::Service,
        Self::MigrationService,
        Self::RegistryAdmin,
    ];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Contributor => "contributor",
            Self::Reviewer => "reviewer",
            Self::Editor => "editor",
            Self::Institution => "institution",
            Self::Service => "service",
            Self::MigrationService => "migration_service",
            Self::RegistryAdmin => "registry_admin",
        }
    }

    pub const fn bit(self) -> u8 {
        match self {
            Self::Contributor => 1 << 0,
            Self::Reviewer => 1 << 1,
            Self::Editor => 1 << 2,
            Self::Institution => 1 << 3,
            Self::Service => 1 << 4,
            Self::MigrationService => 1 << 5,
            Self::RegistryAdmin => 1 << 6,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AttestationKind {
    Ordinary,
    IndependentReplication,
}

impl AttestationKind {
    pub const ALL: [Self; 2] = [Self::Ordinary, Self::IndependentReplication];

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Ordinary => "ordinary",
            Self::IndependentReplication => "independent_replication",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetAttestationState {
    Absent,
    AuthoredByActor,
    AuthoredByOther,
}

impl TargetAttestationState {
    pub const ALL: [Self; 3] = [
        Self::Absent,
        Self::AuthoredByActor,
        Self::AuthoredByOther,
    ];
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ActingOrganizationRelation {
    Absent,
    EqualToCreator,
    DifferentFromCreator,
    PresentWithoutCreatorOrganization,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ActorInput {
    pub id: String,
    pub roles: Vec<Role>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ClaimInput {
    pub exists: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub creator_actor_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub creator_organization_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AttestationInput {
    pub kind: AttestationKind,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TargetAttestationInput {
    pub exists: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub author_actor_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionContext {
    pub action: Action,
    pub actor: ActorInput,
    pub claim: ClaimInput,
    pub attestation: AttestationInput,
    pub target_attestation: TargetAttestationInput,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acting_organization_id: Option<String>,
}

#[derive(Serialize)]
struct CaseIdentity<'a> {
    generator_id: &'static str,
    generator_version: &'static str,
    baseline_revision: &'static str,
    baseline_source_blob_sha: &'static str,
    policy_id: &'static str,
    policy_version: &'static str,
    context: &'a DecisionContext,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionCase {
    pub case_id: String,
    pub baseline_revision: String,
    pub baseline_source_path: String,
    pub baseline_source_blob_sha: String,
    pub policy_id: String,
    pub policy_version: String,
    pub generator_version: String,
    pub context: DecisionContext,
    /// Deliberately null until the authentic compiled Rust oracle executes.
    pub expected_decision: Option<String>,
    pub expected_decision_status: String,
    pub domain_annotations: DomainAnnotations,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct DomainAnnotations {
    pub role_mask: u8,
    pub actor_is_claim_creator: Option<bool>,
    pub acting_organization_relation: ActingOrganizationRelation,
    pub note: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CorpusManifest {
    pub generator_id: String,
    pub generator_version: String,
    pub baseline_revision: String,
    pub baseline_source_path: String,
    pub baseline_source_blob_sha: String,
    pub policy_id: String,
    pub policy_version: String,
    pub case_count: usize,
    pub empty_stream_cases: usize,
    pub existing_stream_cases: usize,
    pub distinct_role_subsets: usize,
    pub action_count: usize,
    pub expected_decision_status: String,
    pub claim_ceiling: String,
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn git_blob_oid(bytes: &[u8]) -> String {
    let mut hasher = Sha1::new();
    hasher.update(format!("blob {}\0", bytes.len()).as_bytes());
    hasher.update(bytes);
    hex::encode(hasher.finalize())
}

fn role_subset(mask: u8) -> Vec<Role> {
    Role::ALL
        .into_iter()
        .filter(|role| mask & role.bit() != 0)
        .collect()
}

fn empty_stream_case(action: Action, role_mask: u8) -> DecisionCase {
    let context = DecisionContext {
        action,
        actor: ActorInput {
            id: "actor:candidate".to_owned(),
            roles: role_subset(role_mask),
        },
        claim: ClaimInput {
            exists: false,
            creator_actor_id: None,
            creator_organization_id: None,
        },
        // These payload-side dimensions are held at neutral values for an empty
        // stream; the Rust match returns before consulting a current projection.
        attestation: AttestationInput {
            kind: AttestationKind::Ordinary,
        },
        target_attestation: TargetAttestationInput {
            exists: false,
            author_actor_id: None,
        },
        acting_organization_id: None,
    };
    make_case(
        context,
        role_mask,
        None,
        ActingOrganizationRelation::Absent,
        "current claim stream is absent; ownership, organization and target-attestation dimensions are not applicable to the source match branch",
    )
}

fn existing_stream_case(
    action: Action,
    role_mask: u8,
    actor_is_claim_creator: bool,
    creator_organization_present: bool,
    acting_organization_relation: ActingOrganizationRelation,
    attestation_kind: AttestationKind,
    target_attestation_state: TargetAttestationState,
) -> DecisionCase {
    let creator_actor_id = "actor:creator".to_owned();
    let actor_id = if actor_is_claim_creator {
        creator_actor_id.clone()
    } else {
        "actor:candidate".to_owned()
    };
    let creator_organization_id = creator_organization_present.then(|| "org:creator".to_owned());
    let acting_organization_id = match acting_organization_relation {
        ActingOrganizationRelation::Absent => None,
        ActingOrganizationRelation::EqualToCreator => Some("org:creator".to_owned()),
        ActingOrganizationRelation::DifferentFromCreator
        | ActingOrganizationRelation::PresentWithoutCreatorOrganization => Some("org:other".to_owned()),
    };
    let (target_exists, target_author) = match target_attestation_state {
        TargetAttestationState::Absent => (false, None),
        TargetAttestationState::AuthoredByActor => (true, Some(actor_id.clone())),
        TargetAttestationState::AuthoredByOther => (true, Some("actor:other".to_owned())),
    };
    let context = DecisionContext {
        action,
        actor: ActorInput {
            id: actor_id,
            roles: role_subset(role_mask),
        },
        claim: ClaimInput {
            exists: true,
            creator_actor_id: Some(creator_actor_id),
            creator_organization_id,
        },
        attestation: AttestationInput { kind: attestation_kind },
        target_attestation: TargetAttestationInput {
            exists: target_exists,
            author_actor_id: target_author,
        },
        acting_organization_id,
    };
    make_case(
        context,
        role_mask,
        Some(actor_is_claim_creator),
        acting_organization_relation,
        "current claim stream exists; all payload and projection dimensions are retained, including dimensions irrelevant to this action, to test noninterference",
    )
}

fn make_case(
    context: DecisionContext,
    role_mask: u8,
    actor_is_claim_creator: Option<bool>,
    acting_organization_relation: ActingOrganizationRelation,
    note: &str,
) -> DecisionCase {
    let identity = CaseIdentity {
        generator_id: GENERATOR_ID,
        generator_version: GENERATOR_VERSION,
        baseline_revision: BASELINE_REVISION,
        baseline_source_blob_sha: BASELINE_SOURCE_BLOB_SHA,
        policy_id: POLICY_ID,
        policy_version: POLICY_VERSION,
        context: &context,
    };
    let identity_bytes =
        serde_json::to_vec(&identity).expect("fixed abstract case identity serializes");
    let case_id = format!("AUTH-GEN-{}", sha256(&identity_bytes));
    DecisionCase {
        case_id,
        baseline_revision: BASELINE_REVISION.to_owned(),
        baseline_source_path: BASELINE_SOURCE_PATH.to_owned(),
        baseline_source_blob_sha: BASELINE_SOURCE_BLOB_SHA.to_owned(),
        policy_id: POLICY_ID.to_owned(),
        policy_version: POLICY_VERSION.to_owned(),
        generator_version: GENERATOR_VERSION.to_owned(),
        context,
        expected_decision: None,
        expected_decision_status: "pending_authentic_rust_oracle".to_owned(),
        domain_annotations: DomainAnnotations {
            role_mask,
            actor_is_claim_creator,
            acting_organization_relation,
            note: note.to_owned(),
        },
    }
}


#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RegoActor {
    pub id: String,
    pub roles: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RegoClaim {
    pub exists: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub creator_actor_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub creator_organization_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RegoAttestation {
    pub kind: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RegoTargetAttestation {
    pub exists: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub author_actor_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RegoInput {
    pub action: String,
    pub actor: RegoActor,
    pub claim: RegoClaim,
    pub attestation: RegoAttestation,
    pub target_attestation: RegoTargetAttestation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acting_organization_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct RegoCaseProjection {
    pub case_id: String,
    pub input: RegoInput,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CedarUid {
    #[serde(rename = "type")]
    pub entity_type: String,
    pub id: String,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CedarEntity {
    pub uid: CedarUid,
    pub attrs: serde_json::Value,
    pub parents: Vec<CedarUid>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct CedarContext {
    pub attestation_kind: String,
    pub target_attestation_exists: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_attestation_author_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acting_organization_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct CedarRequest {
    pub principal: String,
    pub action: String,
    pub resource: String,
    pub context: CedarContext,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CedarCaseProjection {
    pub case_id: String,
    pub request: CedarRequest,
    pub entities: Vec<CedarEntity>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct EngineInputProjection {
    pub case_id: String,
    pub cedar: CedarCaseProjection,
    pub rego: RegoCaseProjection,
}

fn cedar_entity_uid(entity_type: &str, id: &str) -> String {
    format!("{entity_type}::{}", serde_json::to_string(id).expect("generated Cedar entity ID serializes"))
}

/// Translate the exact same abstract case into the frozen Cedar PARC request
/// shape and the OPA/Rego input object. No expected authorization result is
/// added, and organization membership remains an outer Rust gate.
pub fn project_engine_inputs(case: &DecisionCase) -> EngineInputProjection {
    let context = &case.context;
    let role_names = context.actor.roles.iter().map(|role| role.as_str().to_owned()).collect::<Vec<_>>();
    let action = context.action.as_str().to_owned();
    let actor_id = context.actor.id.clone();
    let claim_id = case.case_id.clone();

    let mut actor_attrs = serde_json::Map::new();
    actor_attrs.insert("actorId".to_owned(), serde_json::Value::String(actor_id.clone()));
    actor_attrs.insert("roles".to_owned(), serde_json::json!(role_names));
    // Organization-membership authorization is enforced before the policy
    // kernel. No membership facts are inferred from a claimed acting org.
    actor_attrs.insert("organizations".to_owned(), serde_json::json!([]));

    let mut claim_attrs = serde_json::Map::new();
    claim_attrs.insert("exists".to_owned(), serde_json::Value::Bool(context.claim.exists));
    if let Some(creator) = &context.claim.creator_actor_id {
        claim_attrs.insert("creatorActorId".to_owned(), serde_json::Value::String(creator.clone()));
    }
    if let Some(organization) = &context.claim.creator_organization_id {
        claim_attrs.insert("creatorOrganizationId".to_owned(), serde_json::Value::String(organization.clone()));
    }

    let actor_uid = CedarUid {
        entity_type: "Mycelix::Actor".to_owned(),
        id: actor_id.clone(),
    };
    let claim_uid = CedarUid {
        entity_type: "Mycelix::Claim".to_owned(),
        id: claim_id.clone(),
    };
    let empty_parents = Vec::new();
    let entities = vec![
        CedarEntity {
            uid: actor_uid,
            attrs: serde_json::Value::Object(actor_attrs),
            parents: empty_parents.clone(),
        },
        CedarEntity {
            uid: claim_uid,
            attrs: serde_json::Value::Object(claim_attrs),
            parents: empty_parents,
        },
    ];
    let cedar = CedarCaseProjection {
        case_id: case.case_id.clone(),
        request: CedarRequest {
            principal: cedar_entity_uid("Mycelix::Actor", &actor_id),
            action: cedar_entity_uid("Mycelix::Action", &action),
            resource: cedar_entity_uid("Mycelix::Claim", &claim_id),
            context: CedarContext {
                attestation_kind: context.attestation.kind.as_str().to_owned(),
                target_attestation_exists: context.target_attestation.exists,
                target_attestation_author_id: context.target_attestation.author_actor_id.clone(),
                acting_organization_id: context.acting_organization_id.clone(),
            },
        },
        entities,
    };

    let rego = RegoCaseProjection {
        case_id: case.case_id.clone(),
        input: RegoInput {
            action,
            actor: RegoActor {
                id: actor_id,
                roles: role_names,
            },
            claim: RegoClaim {
                exists: context.claim.exists,
                creator_actor_id: context.claim.creator_actor_id.clone(),
                creator_organization_id: context.claim.creator_organization_id.clone(),
            },
            attestation: RegoAttestation {
                kind: context.attestation.kind.as_str().to_owned(),
            },
            target_attestation: RegoTargetAttestation {
                exists: context.target_attestation.exists,
                author_actor_id: context.target_attestation.author_actor_id.clone(),
            },
            acting_organization_id: context.acting_organization_id.clone(),
        },
    };
    EngineInputProjection {
        case_id: case.case_id.clone(),
        cedar,
        rego,
    }
}

/// Materialize the full bounded abstract domain without synthesizing expected
/// decisions. A future adapter can evaluate the same rows in Rust, Cedar, Rego
/// and CEL, while preserving the baseline source and domain identities.
pub fn enumerate_decision_space() -> Vec<DecisionCase> {
    let mut cases = Vec::with_capacity(62_464);

    // With no claim stream, the Rust match can only inspect action and roles:
    // proposal role, migration role, or the catch-all existing-stream denial.
    for action in Action::ALL {
        for role_mask in 0_u8..128 {
            cases.push(empty_stream_case(action, role_mask));
        }
    }

    // Existing stream: retain all dimensions for every action. Where a source
    // branch does not inspect a dimension, those values become metamorphic
    // noninterference controls instead of being silently pruned.
    for action in Action::ALL {
        for role_mask in 0_u8..128 {
            for actor_is_claim_creator in [false, true] {
                for creator_organization_present in [false, true] {
                    let relations: &[ActingOrganizationRelation] =
                        if creator_organization_present {
                            &[
                                ActingOrganizationRelation::Absent,
                                ActingOrganizationRelation::EqualToCreator,
                                ActingOrganizationRelation::DifferentFromCreator,
                            ]
                        } else {
                            &[
                                ActingOrganizationRelation::Absent,
                                ActingOrganizationRelation::PresentWithoutCreatorOrganization,
                            ]
                        };
                    for relation in relations {
                        for attestation_kind in AttestationKind::ALL {
                            for target_attestation in TargetAttestationState::ALL {
                                cases.push(existing_stream_case(
                                    action,
                                    role_mask,
                                    actor_is_claim_creator,
                                    creator_organization_present,
                                    *relation,
                                    attestation_kind,
                                    target_attestation,
                                ));
                            }
                        }
                    }
                }
            }
        }
    }
    cases
}

pub fn build_manifest(cases: &[DecisionCase]) -> CorpusManifest {
    let empty_stream_cases = cases.iter().filter(|case| !case.context.claim.exists).count();
    let existing_stream_cases = cases.len() - empty_stream_cases;
    let role_masks = cases
        .iter()
        .map(|case| case.domain_annotations.role_mask)
        .collect::<BTreeSet<_>>();
    CorpusManifest {
        generator_id: GENERATOR_ID.to_owned(),
        generator_version: GENERATOR_VERSION.to_owned(),
        baseline_revision: BASELINE_REVISION.to_owned(),
        baseline_source_path: BASELINE_SOURCE_PATH.to_owned(),
        baseline_source_blob_sha: BASELINE_SOURCE_BLOB_SHA.to_owned(),
        policy_id: POLICY_ID.to_owned(),
        policy_version: POLICY_VERSION.to_owned(),
        case_count: cases.len(),
        empty_stream_cases,
        existing_stream_cases,
        distinct_role_subsets: role_masks.len(),
        action_count: Action::ALL.len(),
        expected_decision_status: "pending_authentic_rust_oracle".to_owned(),
        claim_ceiling: concat!(
            "bounded abstract input corpus derived from the pinned Rust policy match structure; ",
            "no expected decision is asserted and no authentic Rust policy execution, ",
            "three-engine parity, formal equivalence or credential authority is claimed"
        )
        .to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_snapshot_matches_the_pinned_git_blob_oid() {
        const SOURCE: &[u8] =
            include_bytes!("../../../mycelix-desci/src/core/src/scientific_governance.rs");
        assert_eq!(git_blob_oid(SOURCE), BASELINE_SOURCE_BLOB_SHA);
    }

    #[test]
    fn emits_exact_expected_domain_denominators() {
        let cases = enumerate_decision_space();
        assert_eq!(cases.len(), 62_464);
        assert_eq!(
            cases.iter().filter(|case| !case.context.claim.exists).count(),
            1_024
        );
        assert_eq!(
            cases.iter().filter(|case| case.context.claim.exists).count(),
            61_440
        );
        let manifest = build_manifest(&cases);
        assert_eq!(manifest.distinct_role_subsets, 128);
        assert_eq!(manifest.action_count, 8);
    }

    #[test]
    fn identities_are_unique_and_source_bound() {
        let cases = enumerate_decision_space();
        let ids = cases.iter().map(|case| case.case_id.as_str()).collect::<BTreeSet<_>>();
        assert_eq!(ids.len(), cases.len());
        assert!(cases.iter().all(|case| {
            case.baseline_revision == BASELINE_REVISION
                && case.baseline_source_blob_sha == BASELINE_SOURCE_BLOB_SHA
                && case.policy_id == POLICY_ID
                && case.policy_version == POLICY_VERSION
        }));
    }

    #[test]
    fn generator_never_fabricates_expected_authorization_decisions() {
        let cases = enumerate_decision_space();
        assert!(cases.iter().all(|case| case.expected_decision.is_none()));
        assert!(cases.iter().all(|case| {
            case.expected_decision_status == "pending_authentic_rust_oracle"
        }));
    }

    #[test]
    fn cedar_and_rego_projections_preserve_the_same_policy_inputs() {
        let cases = enumerate_decision_space();
        let selected = cases.iter().find(|case| {
            case.context.action == Action::AttestationRecorded
                && case.context.claim.exists
                && case.context.attestation.kind == AttestationKind::IndependentReplication
                && case.context.actor.roles.contains(&Role::Reviewer)
                && case.domain_annotations.acting_organization_relation
                    == ActingOrganizationRelation::EqualToCreator
        }).expect("generated domain includes same-org independent replication control");
        let projection = project_engine_inputs(selected);

        assert_eq!(projection.case_id, selected.case_id);
        assert_eq!(projection.cedar.case_id, projection.rego.case_id);
        assert_eq!(projection.cedar.request.context.attestation_kind, "independent_replication");
        assert_eq!(
            projection.cedar.request.context.acting_organization_id.as_deref(),
            Some("org:creator")
        );
        assert_eq!(projection.rego.input.acting_organization_id.as_deref(), Some("org:creator"));
        assert_eq!(projection.rego.input.claim.creator_organization_id.as_deref(), Some("org:creator"));
        assert_eq!(projection.cedar.entities.len(), 2);
        let cedar_entities = serde_json::to_value(&projection.cedar.entities)
            .expect("Cedar entity list serializes");
        assert_eq!(cedar_entities[0]["uid"]["type"], "Mycelix::Actor");
        assert!(cedar_entities[0]["uid"].get("entity_type").is_none());
        assert_eq!(cedar_entities[1]["uid"]["type"], "Mycelix::Claim");
        assert_eq!(cedar_entities.len(), 2, "action entities are supplied canonically from the Cedar schema");
        assert_eq!(
            projection.cedar.entities[0].attrs["organizations"],
            serde_json::json!([]),
            "claimed acting organization must not imply authenticated membership"
        );
        assert_eq!(
            projection.cedar.request.action,
            format!(
                "Mycelix::Action::{}",
                serde_json::to_string("attestation_recorded").expect("action name serializes")
            )
        );
    }

    #[test]
    fn cedar_and_rego_projection_omit_missing_optional_attributes() {
        let cases = enumerate_decision_space();
        let selected = cases.iter().find(|case| {
            case.context.action == Action::AttestationRecorded
                && case.context.claim.exists
                && case.context.claim.creator_organization_id.is_none()
                && case.context.acting_organization_id.is_none()
        }).expect("generated domain includes absent optional organizations");
        let projection = project_engine_inputs(selected);
        let rego = serde_json::to_value(&projection.rego).expect("Rego projection serializes");
        let cedar = serde_json::to_value(&projection.cedar).expect("Cedar projection serializes");
        assert!(rego["input"]["claim"].get("creatorOrganizationId").is_none());
        assert!(rego["input"].get("actingOrganizationId").is_none());
        assert!(cedar["request"]["context"].get("actingOrganizationId").is_none());
        assert!(cedar["entities"][1]["attrs"].get("creatorOrganizationId").is_none());
    }

    #[test]
    fn equal_organization_relation_is_never_generated_without_creator_organization() {
        let cases = enumerate_decision_space();
        assert!(cases.iter().all(|case| {
            case.domain_annotations.acting_organization_relation
                != ActingOrganizationRelation::EqualToCreator
                || case.context.claim.creator_organization_id.is_some()
        }));
        assert!(cases.iter().all(|case| {
            case.domain_annotations.acting_organization_relation
                != ActingOrganizationRelation::DifferentFromCreator
                || case.context.claim.creator_organization_id.is_some()
        }));
        assert!(cases.iter().all(|case| {
            case.domain_annotations.acting_organization_relation
                != ActingOrganizationRelation::PresentWithoutCreatorOrganization
                || (case.context.claim.creator_organization_id.is_none()
                    && case.context.acting_organization_id.is_some())
        }));
    }

    #[test]
    fn source_irrelevant_dimensions_are_retained_for_noninterference() {
        let cases = enumerate_decision_space();
        assert!(cases.iter().any(|case| {
            case.context.action == Action::EvidenceAttached
                && case.context.attestation.kind == AttestationKind::IndependentReplication
                && case.context.target_attestation.exists
        }));
        assert!(cases.iter().any(|case| {
            case.context.action == Action::ClaimProposed
                && case.context.claim.exists
                && case.context.target_attestation.exists
        }));
    }
}
