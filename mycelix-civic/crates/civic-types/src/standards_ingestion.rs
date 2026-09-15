// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! AC-005 standards ingestion boundary.
//!
//! Projects a bounded subset of externally schema-validated OCDS 1.1 and BODS
//! 0.4 data into AC-003 *declared* graph assertions. This module is intentionally
//! not a full schema validator, identity resolver, corroboration engine, or
//! adjudicator.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use serde_json::{Number, Value};

use crate::capture_observation::ProvenanceRef;
use crate::institutional_graph::{
    AssertionStatus, DisclosureClass, InstitutionalEdge, InstitutionalGraphContract,
    InstitutionalGraphViolation, InstitutionalNodeKind, InstitutionalNodeRef,
    InstitutionalRelationKind, OwnershipInterest, ProcurementRole,
};

pub const OCDS_PACKAGE_VERSION: &str = "1.1";
pub const OCDS_SCHEMA_REVISION: &str = "1.1.5";
pub const BODS_VERSION: &str = "0.4";
pub const MAX_INGEST_BYTES: usize = 16 * 1024 * 1024;
pub const MAX_STANDARD_RECORDS: usize = 50_000;

/// Evidence supplied by an upstream validator.
///
/// AC-005 syntax-checks `content_hash` but does not recompute it. The referenced
/// validation receipt is expected to bind the exact source bytes to this digest
/// and to the external schema validation result.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct StandardsSourceEvidence {
    pub source_ref: String,
    pub content_hash: String,
    pub validation_receipt_ref: String,
    pub ingested_at: u64,
}

/// Versioned privacy/identifier policy used by a projection.
///
/// Identifier bindings are deny-by-default: only schemes explicitly named here
/// may leave AC-005 as public/legal-entity bindings. Natural-person identifiers
/// are never emitted, regardless of this allowlist.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct StandardsIngestionPolicy {
    pub policy_ref: String,
    pub public_entity_identifier_schemes: BTreeSet<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ExternalStandard {
    Ocds11SchemaRevision115,
    Bods04,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct EntityIdentifierBinding {
    pub node_id: String,
    pub scheme: String,
    pub identifier: String,
    pub standard: ExternalStandard,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum BodsRelationSide {
    Subject,
    InterestedParty,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum StandardsIngestionWarning {
    OcdsPartyWithoutIdentifier {
        ocid: String,
        party_id: String,
    },
    OcdsIdentifierSchemeSuppressed {
        ocid: String,
        party_id: String,
        scheme: Option<String>,
    },
    BodsPublisherScopeFallback {
        statement_index: usize,
    },
    BodsEntityIdentifierSchemeSuppressed {
        statement_index: usize,
        scheme: Option<String>,
    },
    BodsPrivateIdentifiersSuppressed {
        statement_index: usize,
        count: usize,
    },
    BodsShareRangeNotProjected {
        statement_index: usize,
        interest_index: usize,
    },
    BodsTemporalRangeNotProjected {
        statement_index: usize,
        interest_index: usize,
    },
    BodsUnsupportedInterest {
        statement_index: usize,
    },
    BodsUnspecifiedPartySkipped {
        statement_index: usize,
        side: BodsRelationSide,
    },
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct StandardsIngestionResult {
    pub source_evidence: StandardsSourceEvidence,
    pub policy_ref: String,
    pub nodes: Vec<InstitutionalNodeRef>,
    pub edges: Vec<InstitutionalEdge>,
    pub entity_identifiers: Vec<EntityIdentifierBinding>,
    pub warnings: Vec<StandardsIngestionWarning>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum StandardsIngestionViolation {
    SourceTooLarge,
    TooManyStandardRecords,
    MalformedJson,
    MissingSourceReference,
    InvalidContentHash,
    MissingValidationReceiptReference,
    MissingPolicyReference,
    InvalidIdentifierSchemePolicy,
    UnsupportedOcdsPackageVersion(String),
    UnsupportedOcdsExtension(String),
    MissingOcdsPackageUri,
    MissingOcdsRelease,
    MissingOcdsOcid,
    MissingOcdsReleaseId,
    MissingOcdsPartyId,
    MissingOcdsAwardId,
    UnknownOcdsPartyReference {
        ocid: String,
        party_id: String,
    },
    UnsupportedBodsVersion {
        statement_index: usize,
        version: String,
    },
    InvalidBodsStatementId {
        statement_index: usize,
    },
    MissingBodsRecordId {
        statement_index: usize,
    },
    UnsupportedBodsRecordType {
        statement_index: usize,
    },
    InvalidBodsRecordDetails {
        statement_index: usize,
    },
    DuplicateBodsRecordKind {
        statement_index: usize,
    },
    BodsRelationshipReferenceNotPrior {
        statement_index: usize,
        side: BodsRelationSide,
    },
    BodsRelationshipSubjectNotEntity {
        statement_index: usize,
    },
    BodsBeneficialOwnerNotPerson {
        statement_index: usize,
    },
    InvalidBodsExactShare {
        statement_index: usize,
        interest_index: usize,
    },
    GeneratedEdgeInvalid {
        edge_id: String,
        violations: Vec<InstitutionalGraphViolation>,
    },
}

#[derive(Debug, Default, Clone, Copy)]
pub struct StandardsIngestionContract;

impl StandardsIngestionContract {
    pub fn ingest_ocds_release_package_json(
        json: &str,
        source: StandardsSourceEvidence,
        policy: StandardsIngestionPolicy,
    ) -> Result<StandardsIngestionResult, Vec<StandardsIngestionViolation>> {
        let mut violations = validate_envelope(json, &source, &policy);
        if !violations.is_empty() {
            return Err(violations);
        }

        let package: OcdsReleasePackage = match serde_json::from_str(json) {
            Ok(package) => package,
            Err(_) => return Err(vec![StandardsIngestionViolation::MalformedJson]),
        };
        if package.releases.len() > MAX_STANDARD_RECORDS {
            return Err(vec![StandardsIngestionViolation::TooManyStandardRecords]);
        }
        if package.version != OCDS_PACKAGE_VERSION {
            violations.push(StandardsIngestionViolation::UnsupportedOcdsPackageVersion(
                package.version.clone(),
            ));
        }
        if package.uri.trim().is_empty() {
            violations.push(StandardsIngestionViolation::MissingOcdsPackageUri);
        }
        if package.releases.is_empty() {
            violations.push(StandardsIngestionViolation::MissingOcdsRelease);
        }
        for extension in &package.extensions {
            violations.push(StandardsIngestionViolation::UnsupportedOcdsExtension(
                extension.clone(),
            ));
        }
        if !violations.is_empty() {
            return Err(violations);
        }

        let mut nodes = BTreeMap::<String, InstitutionalNodeRef>::new();
        let mut edges = Vec::new();
        let mut bindings = BTreeMap::<String, EntityIdentifierBinding>::new();
        let mut warnings = Vec::new();

        for release in &package.releases {
            if release.ocid.trim().is_empty() {
                violations.push(StandardsIngestionViolation::MissingOcdsOcid);
                continue;
            }
            if release.id.trim().is_empty() {
                violations.push(StandardsIngestionViolation::MissingOcdsReleaseId);
                continue;
            }

            let procedure = InstitutionalNodeRef {
                id: stable_key("ocds-process", &[&release.ocid]),
                kind: InstitutionalNodeKind::ProcurementProcedure,
            };
            insert_node(&mut nodes, procedure.clone());

            let mut parties = BTreeMap::<String, InstitutionalNodeRef>::new();
            let mut roles = BTreeMap::<String, BTreeSet<String>>::new();
            for party in &release.parties {
                if party.id.trim().is_empty() {
                    violations.push(StandardsIngestionViolation::MissingOcdsPartyId);
                    continue;
                }
                let node = InstitutionalNodeRef {
                    id: stable_key("ocds-party", &[&release.ocid, &party.id]),
                    kind: InstitutionalNodeKind::Organization,
                };
                insert_node(&mut nodes, node.clone());
                parties.insert(party.id.clone(), node.clone());
                roles.insert(party.id.clone(), party.roles.iter().cloned().collect());

                project_ocds_identifier(
                    release,
                    party,
                    &node,
                    &policy,
                    &mut bindings,
                    &mut warnings,
                );
            }

            let provenance = provenance_for(
                &source,
                &stable_key(
                    "ocds-release",
                    &[&package.uri, &release.ocid, &release.id],
                ),
            );
            let mut authority_ids = BTreeSet::new();
            if let Some(buyer) = &release.buyer {
                authority_ids.insert(buyer.id.clone());
            }
            for (party_id, party_roles) in &roles {
                if party_roles.contains("buyer") || party_roles.contains("procuringEntity") {
                    authority_ids.insert(party_id.clone());
                }
            }
            for party_id in authority_ids {
                match parties.get(&party_id) {
                    Some(node) => push_validated_edge(
                        &mut edges,
                        procurement_edge(
                            stable_key(
                                "ocds-edge-authority",
                                &[&release.ocid, &release.id, &party_id],
                            ),
                            node.clone(),
                            procedure.clone(),
                            ProcurementRole::ProcuringAuthority,
                            provenance.clone(),
                            source.ingested_at,
                        ),
                        &mut violations,
                    ),
                    None => violations.push(
                        StandardsIngestionViolation::UnknownOcdsPartyReference {
                            ocid: release.ocid.clone(),
                            party_id,
                        },
                    ),
                }
            }

            for party in &release.parties {
                if party.roles.iter().any(|role| role == "tenderer") {
                    match parties.get(&party.id) {
                        Some(node) => push_validated_edge(
                            &mut edges,
                            procurement_edge(
                                stable_key(
                                    "ocds-edge-bidder",
                                    &[&release.ocid, &release.id, &party.id],
                                ),
                                node.clone(),
                                procedure.clone(),
                                ProcurementRole::Bidder,
                                provenance.clone(),
                                source.ingested_at,
                            ),
                            &mut violations,
                        ),
                        None => violations.push(
                            StandardsIngestionViolation::UnknownOcdsPartyReference {
                                ocid: release.ocid.clone(),
                                party_id: party.id.clone(),
                            },
                        ),
                    }
                }
            }

            for award in &release.awards {
                let award_id = award.id.as_text();
                if award_id.trim().is_empty() {
                    violations.push(StandardsIngestionViolation::MissingOcdsAwardId);
                    continue;
                }
                for supplier in &award.suppliers {
                    match parties.get(&supplier.id) {
                        Some(node) => push_validated_edge(
                            &mut edges,
                            procurement_edge(
                                stable_key(
                                    "ocds-edge-awardee",
                                    &[&release.ocid, &release.id, &award_id, &supplier.id],
                                ),
                                node.clone(),
                                procedure.clone(),
                                ProcurementRole::Awardee,
                                provenance.clone(),
                                source.ingested_at,
                            ),
                            &mut violations,
                        ),
                        None => violations.push(
                            StandardsIngestionViolation::UnknownOcdsPartyReference {
                                ocid: release.ocid.clone(),
                                party_id: supplier.id.clone(),
                            },
                        ),
                    }
                }
            }
        }

        finish(source, policy, nodes, edges, bindings, warnings, violations)
    }

    pub fn ingest_bods_json(
        json: &str,
        source: StandardsSourceEvidence,
        policy: StandardsIngestionPolicy,
    ) -> Result<StandardsIngestionResult, Vec<StandardsIngestionViolation>> {
        let violations = validate_envelope(json, &source, &policy);
        if !violations.is_empty() {
            return Err(violations);
        }
        let statements: Vec<BodsStatement> = match serde_json::from_str(json) {
            Ok(statements) => statements,
            Err(_) => return Err(vec![StandardsIngestionViolation::MalformedJson]),
        };
        if statements.len() > MAX_STANDARD_RECORDS {
            return Err(vec![StandardsIngestionViolation::TooManyStandardRecords]);
        }

        let mut violations = Vec::new();
        let mut nodes = BTreeMap::<String, InstitutionalNodeRef>::new();
        let mut records = BTreeMap::<String, InstitutionalNodeRef>::new();
        let mut bindings = BTreeMap::<String, EntityIdentifierBinding>::new();
        let mut edges = Vec::new();
        let mut warnings = Vec::new();

        for (statement_index, statement) in statements.iter().enumerate() {
            validate_bods_header(statement_index, statement, &mut violations);
            if statement.publication_details.bods_version != BODS_VERSION {
                violations.push(StandardsIngestionViolation::UnsupportedBodsVersion {
                    statement_index,
                    version: statement.publication_details.bods_version.clone(),
                });
                continue;
            }
            let publisher_scope = publisher_scope(statement_index, statement, &source, &mut warnings);
            let opaque_node_id = |namespace: &str| {
                stable_key(
                    namespace,
                    &[&source.content_hash, &statement_index.to_string()],
                )
            };

            match statement.record_type.as_str() {
                "entity" => {
                    let details: BodsEntityDetails = match serde_json::from_value(
                        statement.record_details.clone(),
                    ) {
                        Ok(details) => details,
                        Err(_) => {
                            violations.push(
                                StandardsIngestionViolation::InvalidBodsRecordDetails {
                                    statement_index,
                                },
                            );
                            continue;
                        }
                    };
                    let node = InstitutionalNodeRef {
                        id: opaque_node_id("bods-entity"),
                        kind: InstitutionalNodeKind::LegalEntity,
                    };
                    register_bods_record(
                        &mut records,
                        &mut nodes,
                        statement_index,
                        &publisher_scope,
                        &statement.record_id,
                        node.clone(),
                        &mut violations,
                    );
                    for identifier in &details.identifiers {
                        project_bods_entity_identifier(
                            statement_index,
                            identifier,
                            &node,
                            &policy,
                            &mut bindings,
                            &mut warnings,
                        );
                    }
                }
                "person" => {
                    let details: BodsPersonDetails = match serde_json::from_value(
                        statement.record_details.clone(),
                    ) {
                        Ok(details) => details,
                        Err(_) => {
                            violations.push(
                                StandardsIngestionViolation::InvalidBodsRecordDetails {
                                    statement_index,
                                },
                            );
                            continue;
                        }
                    };
                    let node = InstitutionalNodeRef {
                        id: opaque_node_id("bods-private-person"),
                        kind: InstitutionalNodeKind::PrivatePersonCredential,
                    };
                    register_bods_record(
                        &mut records,
                        &mut nodes,
                        statement_index,
                        &publisher_scope,
                        &statement.record_id,
                        node,
                        &mut violations,
                    );
                    if !details.identifiers.is_empty() {
                        warnings.push(
                            StandardsIngestionWarning::BodsPrivateIdentifiersSuppressed {
                                statement_index,
                                count: details.identifiers.len(),
                            },
                        );
                    }
                }
                "relationship" => {
                    let details: BodsRelationshipDetails = match serde_json::from_value(
                        statement.record_details.clone(),
                    ) {
                        Ok(details) => details,
                        Err(_) => {
                            violations.push(
                                StandardsIngestionViolation::InvalidBodsRecordDetails {
                                    statement_index,
                                },
                            );
                            continue;
                        }
                    };
                    project_bods_relationship(
                        statement_index,
                        &details,
                        &publisher_scope,
                        &source,
                        &records,
                        &mut edges,
                        &mut warnings,
                        &mut violations,
                    );
                }
                _ => violations.push(StandardsIngestionViolation::UnsupportedBodsRecordType {
                    statement_index,
                }),
            }
        }

        finish(source, policy, nodes, edges, bindings, warnings, violations)
    }
}

fn finish(
    source: StandardsSourceEvidence,
    policy: StandardsIngestionPolicy,
    nodes: BTreeMap<String, InstitutionalNodeRef>,
    edges: Vec<InstitutionalEdge>,
    bindings: BTreeMap<String, EntityIdentifierBinding>,
    warnings: Vec<StandardsIngestionWarning>,
    violations: Vec<StandardsIngestionViolation>,
) -> Result<StandardsIngestionResult, Vec<StandardsIngestionViolation>> {
    if !violations.is_empty() {
        return Err(violations);
    }
    Ok(StandardsIngestionResult {
        source_evidence: source,
        policy_ref: policy.policy_ref,
        nodes: nodes.into_values().collect(),
        edges,
        entity_identifiers: bindings.into_values().collect(),
        warnings,
    })
}

fn validate_envelope(
    json: &str,
    source: &StandardsSourceEvidence,
    policy: &StandardsIngestionPolicy,
) -> Vec<StandardsIngestionViolation> {
    let mut violations = Vec::new();
    if json.len() > MAX_INGEST_BYTES {
        violations.push(StandardsIngestionViolation::SourceTooLarge);
    }
    if source.source_ref.trim().is_empty() {
        violations.push(StandardsIngestionViolation::MissingSourceReference);
    }
    if !source
        .content_hash
        .split_once(':')
        .is_some_and(|(algorithm, digest)| !algorithm.trim().is_empty() && !digest.trim().is_empty())
    {
        violations.push(StandardsIngestionViolation::InvalidContentHash);
    }
    if source.validation_receipt_ref.trim().is_empty() {
        violations.push(StandardsIngestionViolation::MissingValidationReceiptReference);
    }
    if policy.policy_ref.trim().is_empty() {
        violations.push(StandardsIngestionViolation::MissingPolicyReference);
    }
    if policy
        .public_entity_identifier_schemes
        .iter()
        .any(|scheme| scheme.trim().is_empty())
    {
        violations.push(StandardsIngestionViolation::InvalidIdentifierSchemePolicy);
    }
    violations
}

fn project_ocds_identifier(
    release: &OcdsRelease,
    party: &OcdsParty,
    node: &InstitutionalNodeRef,
    policy: &StandardsIngestionPolicy,
    bindings: &mut BTreeMap<String, EntityIdentifierBinding>,
    warnings: &mut Vec<StandardsIngestionWarning>,
) {
    let Some(identifier) = &party.identifier else {
        warnings.push(StandardsIngestionWarning::OcdsPartyWithoutIdentifier {
            ocid: release.ocid.clone(),
            party_id: party.id.clone(),
        });
        return;
    };
    let scheme = identifier.scheme.as_deref().map(str::trim).filter(|v| !v.is_empty());
    let id = identifier
        .id
        .as_ref()
        .map(TextOrInteger::as_text)
        .filter(|value| !value.trim().is_empty());
    let (Some(scheme), Some(id)) = (scheme, id) else {
        warnings.push(StandardsIngestionWarning::OcdsIdentifierSchemeSuppressed {
            ocid: release.ocid.clone(),
            party_id: party.id.clone(),
            scheme: scheme.map(ToString::to_string),
        });
        return;
    };
    if !policy.public_entity_identifier_schemes.contains(scheme) {
        warnings.push(StandardsIngestionWarning::OcdsIdentifierSchemeSuppressed {
            ocid: release.ocid.clone(),
            party_id: party.id.clone(),
            scheme: Some(scheme.to_string()),
        });
        return;
    }
    insert_binding(
        bindings,
        EntityIdentifierBinding {
            node_id: node.id.clone(),
            scheme: scheme.to_string(),
            identifier: id,
            standard: ExternalStandard::Ocds11SchemaRevision115,
        },
    );
}

fn project_bods_entity_identifier(
    statement_index: usize,
    identifier: &BodsIdentifier,
    node: &InstitutionalNodeRef,
    policy: &StandardsIngestionPolicy,
    bindings: &mut BTreeMap<String, EntityIdentifierBinding>,
    warnings: &mut Vec<StandardsIngestionWarning>,
) {
    let scheme = identifier.scheme.as_deref().map(str::trim).filter(|v| !v.is_empty());
    let id = identifier.id.as_deref().map(str::trim).filter(|v| !v.is_empty());
    let (Some(scheme), Some(id)) = (scheme, id) else {
        warnings.push(
            StandardsIngestionWarning::BodsEntityIdentifierSchemeSuppressed {
                statement_index,
                scheme: scheme.map(ToString::to_string),
            },
        );
        return;
    };
    if !policy.public_entity_identifier_schemes.contains(scheme) {
        warnings.push(
            StandardsIngestionWarning::BodsEntityIdentifierSchemeSuppressed {
                statement_index,
                scheme: Some(scheme.to_string()),
            },
        );
        return;
    }
    insert_binding(
        bindings,
        EntityIdentifierBinding {
            node_id: node.id.clone(),
            scheme: scheme.to_string(),
            identifier: id.to_string(),
            standard: ExternalStandard::Bods04,
        },
    );
}

fn procurement_edge(
    id: String,
    from: InstitutionalNodeRef,
    to: InstitutionalNodeRef,
    role: ProcurementRole,
    provenance: ProvenanceRef,
    recorded_at: u64,
) -> InstitutionalEdge {
    InstitutionalEdge {
        id,
        from,
        to,
        relation: InstitutionalRelationKind::ProcurementParticipation { role },
        disclosure: DisclosureClass::PublicMetadata,
        assertion_status: AssertionStatus::Declared,
        provenance: vec![provenance],
        challenge_refs: vec![],
        recorded_at,
        valid_from: None,
        valid_until: None,
    }
}

fn push_validated_edge(
    edges: &mut Vec<InstitutionalEdge>,
    edge: InstitutionalEdge,
    violations: &mut Vec<StandardsIngestionViolation>,
) {
    match InstitutionalGraphContract::validate_edge(&edge) {
        Ok(()) => edges.push(edge),
        Err(graph_violations) => violations.push(
            StandardsIngestionViolation::GeneratedEdgeInvalid {
                edge_id: edge.id,
                violations: graph_violations,
            },
        ),
    }
}

fn provenance_for(source: &StandardsSourceEvidence, object_ref: &str) -> ProvenanceRef {
    ProvenanceRef {
        source_ref: stable_key("external-source-object", &[&source.source_ref, object_ref]),
        content_hash: Some(source.content_hash.clone()),
    }
}

fn stable_key(namespace: &str, parts: &[&str]) -> String {
    let mut output = format!("{namespace}|");
    for part in parts {
        output.push_str(&format!("{}:{}|", part.len(), part));
    }
    output
}

fn insert_node(nodes: &mut BTreeMap<String, InstitutionalNodeRef>, node: InstitutionalNodeRef) {
    nodes.entry(node.id.clone()).or_insert(node);
}

fn insert_binding(
    bindings: &mut BTreeMap<String, EntityIdentifierBinding>,
    binding: EntityIdentifierBinding,
) {
    let key = stable_key(
        "binding",
        &[&binding.node_id, &binding.scheme, &binding.identifier],
    );
    bindings.entry(key).or_insert(binding);
}

fn validate_bods_header(
    statement_index: usize,
    statement: &BodsStatement,
    violations: &mut Vec<StandardsIngestionViolation>,
) {
    if !(32..=64).contains(&statement.statement_id.chars().count()) {
        violations.push(StandardsIngestionViolation::InvalidBodsStatementId {
            statement_index,
        });
    }
    if statement.record_id.trim().is_empty() {
        violations.push(StandardsIngestionViolation::MissingBodsRecordId {
            statement_index,
        });
    }
}

fn publisher_scope(
    statement_index: usize,
    statement: &BodsStatement,
    source: &StandardsSourceEvidence,
    warnings: &mut Vec<StandardsIngestionWarning>,
) -> String {
    match statement
        .publication_details
        .publisher
        .url
        .as_deref()
        .filter(|value| !value.trim().is_empty())
    {
        Some(url) => url.to_string(),
        None => {
            warnings.push(StandardsIngestionWarning::BodsPublisherScopeFallback {
                statement_index,
            });
            source.source_ref.clone()
        }
    }
}

fn bods_record_key(publisher_scope: &str, record_id: &str) -> String {
    stable_key("bods-record-key", &[publisher_scope, record_id])
}

#[allow(clippy::too_many_arguments)]
fn register_bods_record(
    records: &mut BTreeMap<String, InstitutionalNodeRef>,
    nodes: &mut BTreeMap<String, InstitutionalNodeRef>,
    statement_index: usize,
    publisher_scope: &str,
    record_id: &str,
    node: InstitutionalNodeRef,
    violations: &mut Vec<StandardsIngestionViolation>,
) {
    let key = bods_record_key(publisher_scope, record_id);
    if let Some(existing) = records.get(&key) {
        if existing.kind != node.kind {
            violations.push(StandardsIngestionViolation::DuplicateBodsRecordKind {
                statement_index,
            });
            return;
        }
    }
    records.insert(key, node.clone());
    insert_node(nodes, node);
}

#[allow(clippy::too_many_arguments)]
fn project_bods_relationship(
    statement_index: usize,
    details: &BodsRelationshipDetails,
    publisher_scope: &str,
    source: &StandardsSourceEvidence,
    records: &BTreeMap<String, InstitutionalNodeRef>,
    edges: &mut Vec<InstitutionalEdge>,
    warnings: &mut Vec<StandardsIngestionWarning>,
    violations: &mut Vec<StandardsIngestionViolation>,
) {
    let subject_id = match &details.subject {
        BodsRecordPointer::RecordId(record_id) => record_id,
        BodsRecordPointer::Unspecified { .. } => {
            warnings.push(StandardsIngestionWarning::BodsUnspecifiedPartySkipped {
                statement_index,
                side: BodsRelationSide::Subject,
            });
            return;
        }
    };
    let interested_id = match &details.interested_party {
        BodsRecordPointer::RecordId(record_id) => record_id,
        BodsRecordPointer::Unspecified { .. } => {
            warnings.push(StandardsIngestionWarning::BodsUnspecifiedPartySkipped {
                statement_index,
                side: BodsRelationSide::InterestedParty,
            });
            return;
        }
    };

    let subject = match records.get(&bods_record_key(publisher_scope, subject_id)) {
        Some(node) => node.clone(),
        None => {
            violations.push(
                StandardsIngestionViolation::BodsRelationshipReferenceNotPrior {
                    statement_index,
                    side: BodsRelationSide::Subject,
                },
            );
            return;
        }
    };
    let interested = match records.get(&bods_record_key(publisher_scope, interested_id)) {
        Some(node) => node.clone(),
        None => {
            violations.push(
                StandardsIngestionViolation::BodsRelationshipReferenceNotPrior {
                    statement_index,
                    side: BodsRelationSide::InterestedParty,
                },
            );
            return;
        }
    };
    if subject.kind != InstitutionalNodeKind::LegalEntity {
        violations.push(StandardsIngestionViolation::BodsRelationshipSubjectNotEntity {
            statement_index,
        });
        return;
    }

    for (interest_index, interest) in details.interests.iter().enumerate() {
        if interest.interest_type.as_deref() != Some("shareholding") {
            warnings.push(StandardsIngestionWarning::BodsUnsupportedInterest {
                statement_index,
            });
            continue;
        }
        let beneficial = interest.beneficial_ownership_or_control.unwrap_or(false);
        if beneficial && interested.kind != InstitutionalNodeKind::PrivatePersonCredential {
            violations.push(StandardsIngestionViolation::BodsBeneficialOwnerNotPerson {
                statement_index,
            });
            continue;
        }
        let share_bps = match &interest.share {
            None => None,
            Some(share) => match &share.exact {
                Some(exact) => match percentage_to_basis_points(exact) {
                    Some(value) => Some(value),
                    None => {
                        violations.push(StandardsIngestionViolation::InvalidBodsExactShare {
                            statement_index,
                            interest_index,
                        });
                        continue;
                    }
                },
                None => {
                    if share.has_range() {
                        warnings.push(
                            StandardsIngestionWarning::BodsShareRangeNotProjected {
                                statement_index,
                                interest_index,
                            },
                        );
                    }
                    None
                }
            },
        };
        if interest.start_date.is_some() || interest.end_date.is_some() {
            warnings.push(StandardsIngestionWarning::BodsTemporalRangeNotProjected {
                statement_index,
                interest_index,
            });
        }

        push_validated_edge(
            edges,
            InstitutionalEdge {
                id: stable_key(
                    "bods-edge-shareholding",
                    &[
                        &source.content_hash,
                        &statement_index.to_string(),
                        &interest_index.to_string(),
                    ],
                ),
                from: interested.clone(),
                to: subject.clone(),
                relation: InstitutionalRelationKind::Ownership(OwnershipInterest {
                    share_bps,
                    beneficial,
                }),
                disclosure: if interested.kind == InstitutionalNodeKind::PrivatePersonCredential {
                    DisclosureClass::LegitimateInterest
                } else {
                    DisclosureClass::PublicMetadata
                },
                assertion_status: AssertionStatus::Declared,
                provenance: vec![provenance_for(
                    source,
                    &stable_key(
                        "bods-statement-index",
                        &[&statement_index.to_string()],
                    ),
                )],
                challenge_refs: vec![],
                recorded_at: source.ingested_at,
                valid_from: None,
                valid_until: None,
            },
            violations,
        );
    }
}

/// Exact percentage -> basis points without floating-point authority or rounding.
fn percentage_to_basis_points(number: &Number) -> Option<u16> {
    let text = number.to_string();
    if text.starts_with('-') || text.contains('e') || text.contains('E') {
        return None;
    }
    let mut parts = text.split('.');
    let whole = parts.next()?.parse::<u16>().ok()?;
    let fraction = parts.next().unwrap_or("");
    if parts.next().is_some() || whole > 100 {
        return None;
    }
    if fraction.len() > 2 && fraction[2..].chars().any(|character| character != '0') {
        return None;
    }
    let mut fractional = fraction.chars().take(2).collect::<String>();
    while fractional.len() < 2 {
        fractional.push('0');
    }
    let total = whole
        .checked_mul(100)?
        .checked_add(fractional.parse::<u16>().ok()?)?;
    (total <= 10_000).then_some(total)
}

#[derive(Deserialize, Debug)]
struct OcdsReleasePackage {
    uri: String,
    version: String,
    #[serde(default)]
    extensions: Vec<String>,
    releases: Vec<OcdsRelease>,
}

#[derive(Deserialize, Debug)]
struct OcdsRelease {
    ocid: String,
    id: String,
    #[serde(default)]
    parties: Vec<OcdsParty>,
    buyer: Option<OcdsOrganizationReference>,
    #[serde(default)]
    awards: Vec<OcdsAward>,
}

#[derive(Deserialize, Debug)]
struct OcdsParty {
    id: String,
    identifier: Option<OcdsIdentifier>,
    #[serde(default)]
    roles: Vec<String>,
}

#[derive(Deserialize, Debug)]
struct OcdsIdentifier {
    scheme: Option<String>,
    id: Option<TextOrInteger>,
}

#[derive(Deserialize, Debug)]
struct OcdsOrganizationReference {
    id: String,
}

#[derive(Deserialize, Debug)]
struct OcdsAward {
    id: TextOrInteger,
    #[serde(default)]
    suppliers: Vec<OcdsOrganizationReference>,
}

#[derive(Deserialize, Debug, Clone)]
#[serde(untagged)]
enum TextOrInteger {
    Text(String),
    Integer(i64),
}

impl TextOrInteger {
    fn as_text(&self) -> String {
        match self {
            Self::Text(value) => value.clone(),
            Self::Integer(value) => value.to_string(),
        }
    }
}

#[derive(Deserialize, Debug)]
struct BodsStatement {
    #[serde(rename = "statementId")]
    statement_id: String,
    #[serde(rename = "recordId")]
    record_id: String,
    #[serde(rename = "recordType")]
    record_type: String,
    #[serde(rename = "recordDetails")]
    record_details: Value,
    #[serde(rename = "publicationDetails")]
    publication_details: BodsPublicationDetails,
}

#[derive(Deserialize, Debug)]
struct BodsPublicationDetails {
    #[serde(rename = "bodsVersion")]
    bods_version: String,
    publisher: BodsPublisher,
}

#[derive(Deserialize, Debug)]
struct BodsPublisher {
    url: Option<String>,
}

#[derive(Deserialize, Debug)]
struct BodsIdentifier {
    id: Option<String>,
    scheme: Option<String>,
}

#[derive(Deserialize, Debug)]
struct BodsEntityDetails {
    #[serde(default)]
    identifiers: Vec<BodsIdentifier>,
}

#[derive(Deserialize, Debug)]
struct BodsPersonDetails {
    #[serde(default)]
    identifiers: Vec<BodsIdentifier>,
}

#[derive(Deserialize, Debug)]
struct BodsRelationshipDetails {
    subject: BodsRecordPointer,
    #[serde(rename = "interestedParty")]
    interested_party: BodsRecordPointer,
    #[serde(default)]
    interests: Vec<BodsInterest>,
}

#[derive(Deserialize, Debug)]
#[serde(untagged)]
enum BodsRecordPointer {
    RecordId(String),
    Unspecified { reason: String },
}

#[derive(Deserialize, Debug)]
struct BodsInterest {
    #[serde(rename = "type")]
    interest_type: Option<String>,
    #[serde(rename = "beneficialOwnershipOrControl")]
    beneficial_ownership_or_control: Option<bool>,
    share: Option<BodsShare>,
    #[serde(rename = "startDate")]
    start_date: Option<String>,
    #[serde(rename = "endDate")]
    end_date: Option<String>,
}

#[derive(Deserialize, Debug)]
struct BodsShare {
    exact: Option<Number>,
    minimum: Option<Number>,
    maximum: Option<Number>,
    #[serde(rename = "exclusiveMinimum")]
    exclusive_minimum: Option<Number>,
    #[serde(rename = "exclusiveMaximum")]
    exclusive_maximum: Option<Number>,
}

impl BodsShare {
    fn has_range(&self) -> bool {
        self.minimum.is_some()
            || self.maximum.is_some()
            || self.exclusive_minimum.is_some()
            || self.exclusive_maximum.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source() -> StandardsSourceEvidence {
        StandardsSourceEvidence {
            source_ref: "https://data.example.test/publication.json".into(),
            content_hash: "sha256:abc123".into(),
            validation_receipt_ref: "receipt:schema-and-hash-validation:1".into(),
            ingested_at: 1_789_100_000,
        }
    }

    fn policy() -> StandardsIngestionPolicy {
        StandardsIngestionPolicy {
            policy_ref: "policy:public-entity-identifiers:v1".into(),
            public_entity_identifier_schemes: ["ZA-CIPC".to_string()].into_iter().collect(),
        }
    }

    #[test]
    fn stable_keys_are_unambiguous() {
        assert_ne!(stable_key("x", &["ab", "c"]), stable_key("x", &["a", "bc"]));
    }

    #[test]
    fn ocid_scopes_parties_and_identifier_export_is_allowlisted() {
        let json = r#"{
          "uri":"https://data.example.test/ocds.json","version":"1.1",
          "releases":[
            {"ocid":"ocds-test-001","id":"r1","parties":[
              {"id":"buyer","roles":["buyer"],"identifier":{"scheme":"ZA-CIPC","id":"PUB001"}},
              {"id":"supplier","roles":["supplier"],"identifier":{"scheme":"PRIVATE-SCHEME","id":"DO-NOT-EXPORT"}}
            ],"buyer":{"id":"buyer"},"awards":[{"id":"a1","suppliers":[{"id":"supplier"}]}]},
            {"ocid":"ocds-test-002","id":"r2","parties":[{"id":"supplier","roles":["supplier"]}],"awards":[]}
          ]
        }"#;
        let result = StandardsIngestionContract::ingest_ocds_release_package_json(
            json,
            source(),
            policy(),
        )
        .expect("supported OCDS subset");
        assert_eq!(
            result.nodes.iter().filter(|node| node.id.contains("supplier")).count(),
            2
        );
        assert!(result.entity_identifiers.iter().any(|binding| {
            binding.scheme == "ZA-CIPC" && binding.identifier == "PUB001"
        }));
        assert!(result
            .entity_identifiers
            .iter()
            .all(|binding| binding.identifier != "DO-NOT-EXPORT"));
        assert!(result.edges.iter().all(|edge| {
            edge.assertion_status == AssertionStatus::Declared
                && edge.recorded_at == source().ingested_at
                && InstitutionalGraphContract::validate_edge(edge).is_ok()
        }));
    }

    #[test]
    fn unsupported_ocds_extensions_fail_closed() {
        let json = r#"{"uri":"https://data.example.test/ocds.json","version":"1.1","extensions":["https://example.test/ext.json"],"releases":[{"ocid":"ocds-test-1","id":"r1"}]}"#;
        let errors = StandardsIngestionContract::ingest_ocds_release_package_json(
            json,
            source(),
            policy(),
        )
        .expect_err("unsupported extension");
        assert!(errors.iter().any(|error| matches!(
            error,
            StandardsIngestionViolation::UnsupportedOcdsExtension(_)
        )));
    }

    #[test]
    fn bods_private_record_ids_and_identifiers_do_not_escape() {
        let secret_record_id = "PASSPORT-LIKE-RECORD-ID-123";
        let json = format!(
            r#"[
              {{"statementId":"11111111-1111-4111-8111-111111111111","recordId":"entity-1","recordType":"entity","publicationDetails":{{"bodsVersion":"0.4","publisher":{{"url":"https://registry.example.test"}}}},"recordDetails":{{"identifiers":[{{"scheme":"ZA-CIPC","id":"2026/001"}}]}}}},
              {{"statementId":"22222222-2222-4222-8222-222222222222","recordId":"{secret_record_id}","recordType":"person","publicationDetails":{{"bodsVersion":"0.4","publisher":{{"url":"https://registry.example.test"}}}},"recordDetails":{{"identifiers":[{{"scheme":"710-PASSPORT","id":"SECRET-PASSPORT"}}]}}}},
              {{"statementId":"33333333-3333-4333-8333-333333333333","recordId":"rel-1","recordType":"relationship","publicationDetails":{{"bodsVersion":"0.4","publisher":{{"url":"https://registry.example.test"}}}},"source":{{"type":["verified"]}},"recordDetails":{{"subject":"entity-1","interestedParty":"{secret_record_id}","interests":[{{"type":"shareholding","beneficialOwnershipOrControl":true,"share":{{"exact":25.5}}}}]}}}}
            ]"#
        );
        let result = StandardsIngestionContract::ingest_bods_json(&json, source(), policy())
            .expect("supported BODS subset");
        assert_eq!(result.edges.len(), 1);
        assert!(result.nodes.iter().all(|node| !node.id.contains(secret_record_id)));
        assert!(result.nodes.iter().all(|node| !node.id.contains("SECRET-PASSPORT")));
        assert!(result
            .entity_identifiers
            .iter()
            .all(|binding| binding.identifier != "SECRET-PASSPORT"));
        let serialized = serde_json::to_string(&result).unwrap();
        assert!(!serialized.contains(secret_record_id));
        assert!(!serialized.contains("SECRET-PASSPORT"));
        let edge = &result.edges[0];
        assert_eq!(edge.assertion_status, AssertionStatus::Declared);
        assert_eq!(edge.disclosure, DisclosureClass::LegitimateInterest);
        match &edge.relation {
            InstitutionalRelationKind::Ownership(interest) => {
                assert_eq!(interest.share_bps, Some(2550));
                assert!(interest.beneficial);
            }
            _ => panic!("expected ownership"),
        }
    }

    #[test]
    fn bods_entity_identifier_binding_requires_allowlisted_scheme() {
        let json = r#"[
          {"statementId":"11111111-1111-4111-8111-111111111111","recordId":"entity-a","recordType":"entity","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://registry.example"}},"recordDetails":{"identifiers":[{"scheme":"INTERNAL-TAX-ID","id":"NOPE"},{"scheme":"ZA-CIPC","id":"PUBLIC"}]}}
        ]"#;
        let result = StandardsIngestionContract::ingest_bods_json(json, source(), policy())
            .expect("entity record projects");
        assert!(result.entity_identifiers.iter().any(|binding| binding.identifier == "PUBLIC"));
        assert!(result.entity_identifiers.iter().all(|binding| binding.identifier != "NOPE"));
    }

    #[test]
    fn bods_relationship_requires_prior_records_in_same_publisher_scope() {
        let json = r#"[
          {"statementId":"11111111-1111-4111-8111-111111111111","recordId":"entity-1","recordType":"entity","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://publisher-a.example"}},"recordDetails":{"identifiers":[]}},
          {"statementId":"22222222-2222-4222-8222-222222222222","recordId":"person-1","recordType":"person","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://publisher-b.example"}},"recordDetails":{"identifiers":[]}},
          {"statementId":"33333333-3333-4333-8333-333333333333","recordId":"rel-1","recordType":"relationship","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://publisher-a.example"}},"recordDetails":{"subject":"entity-1","interestedParty":"person-1","interests":[{"type":"shareholding"}]}}
        ]"#;
        let errors = StandardsIngestionContract::ingest_bods_json(json, source(), policy())
            .expect_err("cross-publisher record collision must not resolve");
        assert!(errors.iter().any(|error| matches!(
            error,
            StandardsIngestionViolation::BodsRelationshipReferenceNotPrior {
                side: BodsRelationSide::InterestedParty,
                ..
            }
        )));
    }

    #[test]
    fn bods_share_ranges_are_not_midpointed() {
        let json = r#"[
          {"statementId":"11111111-1111-4111-8111-111111111111","recordId":"entity-1","recordType":"entity","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://registry.example"}},"recordDetails":{"identifiers":[]}},
          {"statementId":"22222222-2222-4222-8222-222222222222","recordId":"person-1","recordType":"person","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://registry.example"}},"recordDetails":{"identifiers":[]}},
          {"statementId":"33333333-3333-4333-8333-333333333333","recordId":"rel-1","recordType":"relationship","publicationDetails":{"bodsVersion":"0.4","publisher":{"url":"https://registry.example"}},"recordDetails":{"subject":"entity-1","interestedParty":"person-1","interests":[{"type":"shareholding","beneficialOwnershipOrControl":true,"share":{"minimum":25,"maximum":50}}]}}
        ]"#;
        let result = StandardsIngestionContract::ingest_bods_json(json, source(), policy())
            .expect("range projects with explicit loss warning");
        match &result.edges[0].relation {
            InstitutionalRelationKind::Ownership(interest) => assert_eq!(interest.share_bps, None),
            _ => panic!("expected ownership"),
        }
        assert!(result.warnings.iter().any(|warning| matches!(
            warning,
            StandardsIngestionWarning::BodsShareRangeNotProjected { .. }
        )));
    }

    #[test]
    fn exact_share_never_rounds_silently() {
        let exact: Number = serde_json::from_str("25.5").unwrap();
        let too_precise: Number = serde_json::from_str("25.555").unwrap();
        assert_eq!(percentage_to_basis_points(&exact), Some(2550));
        assert_eq!(percentage_to_basis_points(&too_precise), None);
    }
}
