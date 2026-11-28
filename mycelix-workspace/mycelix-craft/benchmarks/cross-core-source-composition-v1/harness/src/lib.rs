#![forbid(unsafe_code)]

use std::fmt::Write as _;

use craft_action_denominator_v1 as action_denominator;
use craft_claim_support_v1 as claim_support;
use craft_decision_closure_v1 as decision_closure;
use craft_evidence_lineage_v1 as evidence_lineage;
use craft_impact_qualification_v1 as impact;
use craft_need_intake_v1 as need_intake;
use craft_project_formation_v1 as project_formation;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExpectedDisposition {
    NotEligible,
    EligibleForLocalReview,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WaterEpisode {
    pub provider_reported_complete: bool,
    pub current_pressure_restoration: bool,
    pub current_water_quality: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CoreDisposition {
    pub need_ratification: need_intake::RatificationStateV1,
    pub project_requirement: project_formation::RequirementDispositionV1,
    pub pressure_support: claim_support::ClaimSupportDispositionV1,
    pub quality_support: claim_support::ClaimSupportDispositionV1,
    pub evidence_sufficiency: impact::SupportSufficiencyV1,
    pub decision_closure: decision_closure::DecisionClosureDispositionV1,
    pub lineage_source: evidence_lineage::ArtifactStateV1,
    pub lineage_claim: evidence_lineage::ArtifactStateV1,
    pub service_disposition: ExpectedDisposition,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CrossCoreReceipt {
    pub case_id: String,
    pub provider_reported_complete: bool,
    pub owner_dispositions: CoreDisposition,
    pub oracle_disposition: ExpectedDisposition,
    pub receipt_body: String,
}

pub fn independent_oracle(episode: WaterEpisode) -> ExpectedDisposition {
    // Deliberately independent of every owner evaluator: provider completion
    // alone is insufficient; both independent restoration and quality states
    // must be current before the outcome is eligible for local review.
    if episode.provider_reported_complete
        && episode.current_pressure_restoration
        && episode.current_water_quality
    {
        ExpectedDisposition::EligibleForLocalReview
    } else {
        ExpectedDisposition::NotEligible
    }
}
pub fn claim_packet(claim_id: &str) -> claim_support::ClaimSupportPacketV1 {
    let mut packet = claim_support_helpers::packet();
    packet.claim.claim_id = claim_id.to_owned();
    for assessment in &mut packet.assessments {
        assessment.claim_id = claim_id.to_owned();
    }
    packet
}

pub fn claim_result(
    claim_id: &str,
    supported: bool,
    now: u64,
) -> claim_support::ClaimSupportResultV1 {
    let mut packet = claim_packet(claim_id);
    if supported {
        packet.reviews.push(claim_support_helpers::review(
            claim_support::ReviewDecisionV1::ConfirmDirectSupport,
        ));
    } else {
        packet.assessments.clear();
    }
    claim_support::evaluate_claim_support_result(
        &packet,
        now,
        &format!("result:{claim_id}"),
        [73; 32],
    )
    .expect("synthetic claim support packet is structurally valid")
}

pub fn closure_support_ref(
    result: &claim_support::ClaimSupportResultV1,
) -> decision_closure::ClaimSupportResultRefV1 {
    let disposition = match result.disposition {
        claim_support::ClaimSupportDispositionV1::Supported => {
            decision_closure::SupportDispositionV1::DirectSupport
        }
        claim_support::ClaimSupportDispositionV1::BlockedByContradiction => {
            decision_closure::SupportDispositionV1::Contradiction
        }
        claim_support::ClaimSupportDispositionV1::HumanReviewRequired => {
            decision_closure::SupportDispositionV1::HumanReviewRequired
        }
        claim_support::ClaimSupportDispositionV1::InsufficientEvidence => {
            decision_closure::SupportDispositionV1::InsufficientEvidence
        }
    };
    decision_closure::ClaimSupportResultRefV1 {
        result_id: result.result_id.clone(),
        result_digest: result.result_digest,
        claim_id: result.claim_id.clone(),
        claim_digest: result.claim_digest,
        proposal_digest: result.proposal_digest,
        domain_id: result.domain_id.clone(),
        jurisdiction_id: result.jurisdiction_id.clone(),
        authorized_corpus_id: result.authorized_corpus_id.clone(),
        domain_standard_id: result.domain_standard_id.clone(),
        domain_standard_revision: result.domain_standard_revision,
        domain_standard_digest: result.domain_standard_digest,
        corpus_digest: result.authorized_corpus_digest,
        disposition,
        evaluated_at: result.evaluated_at,
        valid_until: result.valid_until,
    }
}

pub fn source_lineage(expired: bool) -> evidence_lineage::LineageEvaluationV1 {
    use evidence_lineage::{ArtifactKindV1, ChangeReasonV1, DependencyRelationV1, ImpactPolicyV1};
    let source = lineage_helpers::reference("wsr:pressure-source", 1, 1);
    let claim = lineage_helpers::reference("wsr:pressure-claim", 1, 2);
    let decision = lineage_helpers::reference("wsr:restoration-decision", 1, 3);
    let unaffected_quality_source =
        lineage_helpers::reference("wsr:quality-source-independent", 1, 4);
    let nodes = vec![
        evidence_lineage::ArtifactNodeV1 {
            reference: source.clone(),
            kind: ArtifactKindV1::Source,
            dependencies: vec![],
        },
        evidence_lineage::ArtifactNodeV1 {
            reference: unaffected_quality_source,
            kind: ArtifactKindV1::Source,
            dependencies: vec![],
        },
        evidence_lineage::ArtifactNodeV1 {
            reference: claim.clone(),
            kind: ArtifactKindV1::Claim,
            dependencies: vec![lineage_helpers::dependency(
                source.clone(),
                DependencyRelationV1::SourcedFrom,
                ImpactPolicyV1::RevalidateOnChange,
            )],
        },
        evidence_lineage::ArtifactNodeV1 {
            reference: decision.clone(),
            kind: ArtifactKindV1::Other,
            dependencies: vec![lineage_helpers::dependency(
                claim.clone(),
                DependencyRelationV1::SemanticallySupportedBy,
                ImpactPolicyV1::RevalidateOnChange,
            )],
        },
    ];
    let events = if expired {
        vec![lineage_helpers::event(
            "wsr:source-expired",
            1,
            source,
            ChangeReasonV1::Expired,
            None,
        )]
    } else {
        vec![]
    };
    evidence_lineage::evaluate_lineage(&nodes, &events)
        .expect("synthetic evidence lineage is acyclic and correctly scoped")
}

pub fn evaluate_episode(
    case_id: &str,
    episode: WaterEpisode,
    local_mapping_reviews: usize,
) -> CrossCoreReceipt {
    use decision_closure::DecisionClosureDispositionV1;
    use impact::{SupportEvidenceStateV1, SupportRuleV1};

    let proposal = need_helpers::valid_proposal();
    need_intake::validate_proposal(&proposal).expect("synthetic community need proposal is valid");
    let ratification = need_intake::evaluate_ratification(&proposal, 100)
        .expect("need ratification can be evaluated");
    assert_eq!(ratification.state, need_intake::RatificationStateV1::Draft);
    assert!(need_intake::ratified_need_envelope(&proposal, 100).is_err());

    let project = project_helpers::project(project_formation::RequirementKindV1::Required, 1);
    let team = project_formation::evaluate_team(
        &project,
        &project_helpers::proposal(vec![project_helpers::member("local-maintainer")], vec![]),
        project_helpers::NOW,
    )
    .expect("project team evaluates without promoting missing capability evidence");
    let project_requirement = team.requirements[0].disposition;
    assert_eq!(
        project_requirement,
        project_formation::RequirementDispositionV1::UnknownEvidence
    );

    let pressure = claim_result(
        "water:pressure-restoration",
        episode.current_pressure_restoration,
        100,
    );
    let quality = claim_result("water:quality-release", episode.current_water_quality, 100);
    let pressure_support = pressure.disposition;
    let quality_support = quality.disposition;

    let support_state = |disposition| match disposition {
        claim_support::ClaimSupportDispositionV1::Supported => {
            SupportEvidenceStateV1::DirectSupport
        }
        claim_support::ClaimSupportDispositionV1::BlockedByContradiction => {
            SupportEvidenceStateV1::Contradicted
        }
        claim_support::ClaimSupportDispositionV1::HumanReviewRequired
        | claim_support::ClaimSupportDispositionV1::InsufficientEvidence => {
            SupportEvidenceStateV1::NotAssessable
        }
    };
    let sufficiency = impact::evaluate_support_sufficiency(
        &SupportRuleV1::AllOf {
            required_evidence_ids: vec![
                "water:pressure-restoration".to_owned(),
                "water:quality-release".to_owned(),
            ],
        },
        &[
            impact_helpers::item(
                "water:pressure-restoration",
                support_state(pressure_support),
                Some("field-telemetry"),
                31,
            ),
            impact_helpers::item(
                "water:quality-release",
                support_state(quality_support),
                Some("laboratory-release"),
                32,
            ),
        ],
    )
    .expect("all-of impact support rule is structurally valid");

    let pressure_ref = closure_support_ref(&pressure);
    let quality_ref = closure_support_ref(&quality);
    let mapping = closure_helpers::mapping(
        case_id,
        vec![
            closure_helpers::element("pressure-restoration", vec![], vec![pressure_ref]),
            closure_helpers::element("water-quality-release", vec![], vec![quality_ref]),
        ],
    );
    let mut mapping = mapping;
    mapping.reviews.truncate(local_mapping_reviews);
    let mut closure_packet = closure_helpers::packet(vec![mapping]);
    // The adapter translates each Claim Support result into the exact domain,
    // jurisdiction, corpus and standard context expected by Decision Closure.
    closure_packet.decision.proposal_digest = pressure.proposal_digest;
    closure_packet.decision.domain_id = pressure.domain_id.clone();
    closure_packet.decision.jurisdiction_id = pressure.jurisdiction_id.clone();
    closure_packet.decision.authorized_corpus_id = pressure.authorized_corpus_id.clone();
    closure_packet.decision.authorized_corpus_digest = pressure.authorized_corpus_digest;
    closure_packet.decision.standard_id = pressure.domain_standard_id.clone();
    closure_packet.decision.standard_revision = pressure.domain_standard_revision;
    closure_packet.decision.standard_digest = pressure.domain_standard_digest;
    closure_packet.mappings[0].standard_digest = pressure.domain_standard_digest;
    let closure = decision_closure::evaluate_decision_closure(&closure_packet, 100)
        .expect("decision closure evaluates the owner support references");
    let closure_disposition = closure.disposition;

    let lineage = source_lineage(false);
    let source = lineage_helpers::reference("wsr:pressure-source", 1, 1);
    let claim = lineage_helpers::reference("wsr:pressure-claim", 1, 2);
    let lineage_source = lineage.state_for(&source).expect("source lineage state");
    let lineage_claim = lineage.state_for(&claim).expect("claim lineage state");

    let service_disposition = if sufficiency == impact::SupportSufficiencyV1::Supported
        && closure_disposition == DecisionClosureDispositionV1::MappingReviewRequired
    {
        ExpectedDisposition::EligibleForLocalReview
    } else {
        ExpectedDisposition::NotEligible
    };

    let oracle = independent_oracle(episode);
    assert_eq!(
        service_disposition, oracle,
        "owner outputs disagree with independent oracle for {case_id}"
    );
    assert_ne!(
        closure_disposition,
        DecisionClosureDispositionV1::Closed,
        "this slice must never auto-close the municipal service episode"
    );

    let owner_dispositions = CoreDisposition {
        need_ratification: ratification.state,
        project_requirement,
        pressure_support,
        quality_support,
        evidence_sufficiency: sufficiency,
        decision_closure: closure_disposition,
        lineage_source,
        lineage_claim,
        service_disposition,
    };
    let mut receipt_body = String::new();
    let _ = write!(
        receipt_body,
        "case={case_id}|provider_reported_complete={}|need={:?}|project_requirement={:?}|pressure={:?}|quality={:?}|support={:?}|closure={:?}|lineage_source={:?}|lineage_claim={:?}|service={:?}|oracle={:?}",
        episode.provider_reported_complete,
        owner_dispositions.need_ratification,
        owner_dispositions.project_requirement,
        owner_dispositions.pressure_support,
        owner_dispositions.quality_support,
        owner_dispositions.evidence_sufficiency,
        owner_dispositions.decision_closure,
        owner_dispositions.lineage_source,
        owner_dispositions.lineage_claim,
        owner_dispositions.service_disposition,
        oracle,
    );
    CrossCoreReceipt {
        case_id: case_id.to_owned(),
        provider_reported_complete: episode.provider_reported_complete,
        owner_dispositions,
        oracle_disposition: oracle,
        receipt_body,
    }
}

pub mod need_helpers {
    use super::need_intake::*;
    pub fn authority(id: &str) -> LocalAuthorityRefV1 {
        LocalAuthorityRefV1 {
            policy_id: format!("policy-{id}"),
            revision: 1,
            policy_digest: [1; 32],
            decision_id: format!("decision-{id}"),
            decision_digest: [2; 32],
        }
    }

    pub fn valid_proposal() -> NeedProposalV1 {
        let source = SourceRefV1 {
            source_id: "community-water-survey".to_owned(),
            source_digest: [3; 32],
            kind: SourceKindV1::ParticipantStatement,
            visibility_profile_id: "community-controlled".to_owned(),
        };
        let trace = vec![TraceabilityLinkV1 {
            relation: "supports the proposed service outcome".to_owned(),
            basis: TraceabilityBasisV1::Source {
                source_id: source.source_id.clone(),
                source_digest: source.source_digest,
            },
        }];
        NeedProposalV1 {
            schema_version: NEED_INTAKE_SCHEMA_VERSION,
            jurisdiction_id: "ZA-GP".to_owned(),
            need: CommunityNeedV1 {
                need_id: "community-water-access".to_owned(),
                community_id: "community-water-committee".to_owned(),
                statement_digest: [4; 32],
                submitter_id: "community-assembly".to_owned(),
                visibility_profile_id: "community-controlled".to_owned(),
            },
            revision: 1,
            previous_proposal_digest: None,
            proposal_digest: [5; 32],
            sources: vec![source],
            assumptions: vec![
                AssumptionV1 {
                    assumption_id: "existing-parts-cost".to_owned(),
                    statement: "Existing compatible parts are available at lower cost than a full replacement".to_owned(),
                    owner_id: "maintenance-team".to_owned(),
                    risk_if_false: "Repair may not be the lower-cost option".to_owned(),
                },
                AssumptionV1 {
                    assumption_id: "replacement-maintenance".to_owned(),
                    statement: "A correctly specified replacement pump may reduce recurring maintenance".to_owned(),
                    owner_id: "maintenance-team".to_owned(),
                    risk_if_false: "Replacement may not improve lifecycle cost".to_owned(),
                },
            ],
            outcomes: vec![ProposedOutcomeV1 {
                outcome_id: "reliable-water-service".to_owned(),
                statement: "Improve reliability of the community water service".to_owned(),
                indicator: "Hours of potable water service per day".to_owned(),
                measurement_ref: "water-service-hours-observation-v1".to_owned(),
                acceptance_policy: PolicyRefV1 {
                    policy_id: "water-service-acceptance".to_owned(),
                    revision: 1,
                    digest: [30; 32],
                },
                baseline: None,
                target: "At least 20 hours/day".to_owned(),
                time_horizon: "90 days after commissioning".to_owned(),
                outcome_owner_id: "water-committee".to_owned(),
                acceptance_authority: authority("water-committee"),
                traceability: trace.clone(),
            }],
            requirements: vec![ProposedRequirementV1 {
                requirement_id: "electrical-safety".to_owned(),
                capability: CapabilityProfileRefV1 {
                    vocabulary_id: "electrical-capabilities".to_owned(),
                    concept_id: "electrical-inspection".to_owned(),
                    revision: "1.0.0".to_owned(),
                    digest: [31; 32],
                },
                context_digest: [32; 32],
                jurisdiction_id: "ZA-GP".to_owned(),
                assessment_policy: PolicyRefV1 {
                    policy_id: "licensed-electrical-inspection".to_owned(),
                    revision: 1,
                    digest: [33; 32],
                },
                minimum_contributors: 1,
                max_status_age_seconds: 60,
                statement: "Qualified electrical inspection before energizing".to_owned(),
                kind: RequirementKindV1::SafetyCritical,
                traceability: trace.clone(),
            }],
            work_packages: vec![
                ProposedWorkPackageV1 {
                    work_package_id: "inspect".to_owned(),
                    description: "Inspect the pump and electrical installation".to_owned(),
                    scope_digest: [34; 32],
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    depends_on: Vec::new(),
                },
                ProposedWorkPackageV1 {
                    work_package_id: "repair".to_owned(),
                    description: "Complete the owner-approved repair".to_owned(),
                    scope_digest: [35; 32],
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    depends_on: vec!["inspect".to_owned()],
                },
            ],
            alternatives: vec![
                ProposedAlternativeV1 {
                    alternative_id: "incremental-repair".to_owned(),
                    label: "Repair existing pump".to_owned(),
                    description: "Inspect and repair the existing pump where technically feasible".to_owned(),
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    work_package_ids: vec!["inspect".to_owned(), "repair".to_owned()],
                    tradeoffs: vec![
                        ProposedTradeoffV1 {
                            tradeoff_id: "lower-initial-cost".to_owned(),
                            statement: "Potentially lower initial material cost".to_owned(),
                            traceability: vec![TraceabilityLinkV1 {
                                relation: "explicit assumption requiring field verification".to_owned(),
                                basis: TraceabilityBasisV1::Assumption {
                                    assumption_id: "existing-parts-cost".to_owned(),
                                },
                            }],
                        },
                        ProposedTradeoffV1 {
                            tradeoff_id: "existing-part-dependence".to_owned(),
                            statement: "Depends on condition and availability of existing parts".to_owned(),
                            traceability: vec![trace[0].clone()],
                        },
                    ],
                    traceability: vec![trace[0].clone()],
                },
                ProposedAlternativeV1 {
                    alternative_id: "replace-pump".to_owned(),
                    label: "Replace pump".to_owned(),
                    description: "Replace the existing pump after inspection confirms replacement is warranted".to_owned(),
                    outcome_ids: vec!["reliable-water-service".to_owned()],
                    requirement_ids: vec!["electrical-safety".to_owned()],
                    work_package_ids: vec!["inspect".to_owned(), "repair".to_owned()],
                    tradeoffs: vec![
                        ProposedTradeoffV1 {
                            tradeoff_id: "higher-capital-cost".to_owned(),
                            statement: "Potentially higher initial capital cost".to_owned(),
                            traceability: vec![TraceabilityLinkV1 {
                                relation: "explicit assumption requiring supplier quotation".to_owned(),
                                basis: TraceabilityBasisV1::Assumption {
                                    assumption_id: "replacement-maintenance".to_owned(),
                                },
                            }],
                        },
                        ProposedTradeoffV1 {
                            tradeoff_id: "maintenance-impact".to_owned(),
                            statement: "May reduce recurring maintenance if replacement is correctly specified".to_owned(),
                            traceability: vec![TraceabilityLinkV1 {
                                relation: "explicit lifecycle assumption requiring validation".to_owned(),
                                basis: TraceabilityBasisV1::Assumption {
                                    assumption_id: "replacement-maintenance".to_owned(),
                                },
                            }],
                        },
                    ],
                    traceability: vec![trace[0].clone()],
                },
            ],
            contradictions: Vec::new(),
            open_questions: Vec::new(),
            safety_review_required: true,
            reviews: Vec::new(),
        }
    }
}

pub mod project_helpers {
    use super::project_formation::*;

    pub const NOW: u64 = 10;

    pub fn profile() -> CapabilityProfileRefV1 {
        CapabilityProfileRefV1 {
            vocabulary_id: "local:community-capabilities".to_owned(),
            concept_id: "capability:electrical-inspection".to_owned(),
            revision: "2026.1".to_owned(),
            digest: [1; 32],
        }
    }

    pub fn policy() -> PolicyRefV1 {
        PolicyRefV1 {
            policy_id: "assessment:electrical-v1".to_owned(),
            revision: 1,
            digest: [2; 32],
        }
    }

    pub fn project(kind: RequirementKindV1, minimum: u16) -> NeedToProjectV1 {
        NeedToProjectV1 {
            schema_version: PROJECT_FORMATION_SCHEMA_VERSION,
            project_id: "project-water-pump-repair".to_owned(),
            community_id: "community-a".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            terms_digest: [3; 32],
            authority: ProjectAuthorityRefV1 {
                decision_id: "authority-decision-1".to_owned(),
                decision_digest: [4; 32],
            },
            outcomes: vec![ProjectOutcomeV1 {
                outcome_id: "water-service-restored".to_owned(),
                measurement_ref: "measurement:pump-availability".to_owned(),
                acceptance_policy: policy(),
            }],
            work_packages: vec![WorkPackageV1 {
                work_package_id: "inspect-and-repair".to_owned(),
                scope_digest: [5; 32],
                outcome_ids: vec!["water-service-restored".to_owned()],
                requirements: vec![CapabilityRequirementV1 {
                    requirement_id: "electrical-inspection".to_owned(),
                    capability: profile(),
                    context_digest: [6; 32],
                    jurisdiction_id: "ZA-GP".to_owned(),
                    assessment_policy: policy(),
                    minimum_contributors: minimum,
                    max_status_age_seconds: 5,
                    kind,
                }],
            }],
        }
    }

    pub fn member(id: &str) -> ParticipantConsentV1 {
        ParticipantConsentV1 {
            participant_id: id.to_owned(),
            accepted_terms_digest: [3; 32],
            accepted_at: 1,
            withdrawn_at: None,
        }
    }

    pub fn evidence(subject: &str, disposition: EvidenceDispositionV1) -> CapabilityEvidenceRefV1 {
        CapabilityEvidenceRefV1 {
            evidence_id: format!("evidence-{subject}"),
            evidence_digest: [7; 32],
            subject_id: subject.to_owned(),
            capability: profile(),
            context_digest: [6; 32],
            jurisdiction_id: "ZA-GP".to_owned(),
            assessment_policy: policy(),
            disposition,
            status_checked_at: 8,
            status_snapshot_digest: [12; 32],
            valid_from: 1,
            valid_until: Some(100),
        }
    }

    pub fn assignment(
        contributor: &str,
        evidence: Vec<CapabilityEvidenceRefV1>,
    ) -> RequirementAssignmentV1 {
        RequirementAssignmentV1 {
            work_package_id: "inspect-and-repair".to_owned(),
            requirement_id: "electrical-inspection".to_owned(),
            contributor_id: contributor.to_owned(),
            evidence,
            supervised_development: None,
        }
    }

    pub fn proposal(
        members: Vec<ParticipantConsentV1>,
        assignments: Vec<RequirementAssignmentV1>,
    ) -> TeamProposalV1 {
        TeamProposalV1 {
            project_id: "project-water-pump-repair".to_owned(),
            terms_digest: [3; 32],
            members,
            assignments,
        }
    }

    pub fn result(project: &NeedToProjectV1, proposal: &TeamProposalV1) -> RequirementCoverageV1 {
        evaluate_team(project, proposal, NOW).unwrap().requirements[0].clone()
    }
}

pub mod claim_support_helpers {
    use super::claim_support::*;
    pub const NOW: u64 = 100;

    pub fn authority() -> LocalAuthorityRefV1 {
        LocalAuthorityRefV1 {
            policy_id: "domain-review-policy".to_owned(),
            revision: 1,
            policy_digest: [1; 32],
            decision_id: "review-delegation".to_owned(),
            decision_digest: [2; 32],
        }
    }

    pub fn packet() -> ClaimSupportPacketV1 {
        let dimensions = vec![
            SemanticDimensionV1::Negation,
            SemanticDimensionV1::Quantity,
            SemanticDimensionV1::Unit,
            SemanticDimensionV1::TemporalScope,
            SemanticDimensionV1::Population,
            SemanticDimensionV1::Scope,
            SemanticDimensionV1::Jurisdiction,
            SemanticDimensionV1::Uncertainty,
            SemanticDimensionV1::CausalStrength,
            SemanticDimensionV1::Conditions,
        ];
        let source = SourceExcerptRefV1 {
            source_id: "pump-service-measurement".to_owned(),
            authorized_corpus_id: "ZA-water-standard-corpus".to_owned(),
            authorized_corpus_digest: [21; 32],
            source_digest: [3; 32],
            excerpt_digest: [4; 32],
            locator_digest: [5; 32],
            source_revision: 1,
        };
        let qualifiers = dimensions
            .iter()
            .enumerate()
            .map(|(index, dimension)| QualifierComparisonV1 {
                dimension: *dimension,
                claim_value_digest: Some([10 + index as u8; 32]),
                source_value_digest: Some([20 + index as u8; 32]),
                relation: QualifierRelationV1::Preserved,
            })
            .collect();
        let assessment = SupportAssessmentV1 {
            assessment_id: "assessment-1".to_owned(),
            claim_id: "water-service-hours".to_owned(),
            claim_digest: [6; 32],
            proposal_digest: [7; 32],
            domain_id: "water-service".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            authorized_corpus_id: "ZA-water-standard-corpus".to_owned(),
            authorized_corpus_digest: [21; 32],
            domain_standard_id: "water-service-standard".to_owned(),
            domain_standard_revision: 1,
            domain_standard_digest: [22; 32],
            source_id: source.source_id.clone(),
            source_revision: source.source_revision,
            source_digest: source.source_digest,
            excerpt_digest: source.excerpt_digest,
            locator_digest: source.locator_digest,
            relation: SupportRelationV1::DirectSupport,
            assessor_id: "symthaea-assessor".to_owned(),
            method_id: "claim-support-check".to_owned(),
            method_version: "1.0.0".to_owned(),
            assessor_kind: AssessorKindV1::Machine,
            model_digest: Some([8; 32]),
            assessed_at: 10,
            valid_until: Some(200),
            assessment_digest: [9; 32],
            qualifiers,
        };
        ClaimSupportPacketV1 {
            schema_version: CLAIM_SUPPORT_SCHEMA_VERSION,
            claim: ClaimRefV1 {
                claim_id: "water-service-hours".to_owned(),
                claim_digest: [6; 32],
                proposal_digest: [7; 32],
                domain_id: "water-service".to_owned(),
                jurisdiction_id: "ZA-GP".to_owned(),
                authorized_corpus_id: "ZA-water-standard-corpus".to_owned(),
                authorized_corpus_digest: [21; 32],
                domain_standard_id: "water-service-standard".to_owned(),
                domain_standard_revision: 1,
                domain_standard_digest: [22; 32],
                required_dimensions: dimensions,
                required_reviewer_roles: vec![ReviewerRoleV1::DomainExpert],
                max_assessment_age_seconds: 120,
            },
            sources: vec![source],
            assessments: vec![assessment],
            reviews: Vec::new(),
        }
    }

    pub fn review(decision: ReviewDecisionV1) -> SupportReviewV1 {
        SupportReviewV1 {
            review_id: "review-1".to_owned(),
            assessment_id: "assessment-1".to_owned(),
            assessment_digest: [9; 32],
            reviewer_id: "independent-engineer".to_owned(),
            role: ReviewerRoleV1::DomainExpert,
            authority: authority(),
            decision,
            reviewed_at: 20,
            valid_until: Some(180),
            withdrawn_at: None,
        }
    }
}

pub mod closure_helpers {
    use super::decision_closure::*;
    pub const NOW: u64 = 100;

    pub fn authority(id: &str) -> LocalAuthorityRefV1 {
        LocalAuthorityRefV1 {
            policy_id: format!("policy-{id}"),
            revision: 1,
            policy_digest: [1; 32],
            decision_id: format!("authority-decision-{id}"),
            decision_digest: [2; 32],
        }
    }

    pub fn authority_for(
        id: &str,
        decision_id: &str,
        decision_digest: [u8; 32],
    ) -> LocalAuthorityRefV1 {
        let mut reference = authority(id);
        reference.decision_id = decision_id.to_owned();
        reference.decision_digest = decision_digest;
        reference
    }

    pub fn decision() -> DecisionContextV1 {
        DecisionContextV1 {
            decision_id: "water-service-compliance".to_owned(),
            conclusion_digest: [3; 32],
            proposal_digest: [8; 32],
            domain_id: "water-service-assurance-v1".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            decision_time: NOW,
            authorized_corpus_id: "water-authority-corpus".to_owned(),
            authorized_corpus_digest: [4; 32],
            standard_id: "water-service-standard".to_owned(),
            standard_revision: 1,
            standard_digest: [5; 32],
            required_mapping_roles: vec![MappingReviewerRoleV1::DomainExpert],
            minimum_reviewers_per_role: 2,
            max_support_age_seconds: 120,
        }
    }

    pub fn support(id: &str, disposition: SupportDispositionV1) -> ClaimSupportResultRefV1 {
        ClaimSupportResultRefV1 {
            result_id: id.to_owned(),
            result_digest: [6; 32],
            claim_id: format!("claim-{id}"),
            claim_digest: [7; 32],
            proposal_digest: [8; 32],
            domain_id: "water-service-assurance-v1".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            authorized_corpus_id: "water-authority-corpus".to_owned(),
            domain_standard_id: "water-service-standard".to_owned(),
            domain_standard_revision: 1,
            domain_standard_digest: [5; 32],
            corpus_digest: [4; 32],
            disposition,
            evaluated_at: 10,
            valid_until: Some(200),
        }
    }

    pub fn element(
        id: &str,
        dependencies: Vec<String>,
        supports: Vec<ClaimSupportResultRefV1>,
    ) -> MaterialElementV1 {
        MaterialElementV1 {
            element_id: id.to_owned(),
            statement: format!("Required material element {id}"),
            kind: MaterialElementKindV1::DomainSpecific,
            applicability: ElementApplicabilityV1::Applicable,
            depends_on: dependencies,
            minimum_support_results: 1,
            support_results: supports,
        }
    }

    pub fn review(mapping_id: &str, digest: [u8; 32]) -> MappingReviewV1 {
        MappingReviewV1 {
            review_id: format!("review-{mapping_id}"),
            mapping_id: mapping_id.to_owned(),
            mapping_digest: digest,
            reviewer_id: "independent-domain-reviewer".to_owned(),
            role: MappingReviewerRoleV1::DomainExpert,
            authority: authority("domain"),
            decision: MappingReviewDecisionV1::Approve,
            reviewed_at: 20,
            valid_until: Some(180),
            withdrawn_at: None,
        }
    }

    pub fn mapping(id: &str, elements: Vec<MaterialElementV1>) -> MaterialElementMappingV1 {
        let digest = [9; 32];
        MaterialElementMappingV1 {
            mapping_id: id.to_owned(),
            revision: 1,
            previous_mapping_digest: None,
            mapping_digest: digest,
            decision_id: "water-service-compliance".to_owned(),
            conclusion_digest: [3; 32],
            standard_digest: [5; 32],
            elements,
            reviews: vec![
                review(id, digest),
                MappingReviewV1 {
                    review_id: format!("review-{id}-second"),
                    mapping_id: id.to_owned(),
                    mapping_digest: digest,
                    reviewer_id: "independent-domain-reviewer-2".to_owned(),
                    role: MappingReviewerRoleV1::DomainExpert,
                    authority: authority("domain-2"),
                    decision: MappingReviewDecisionV1::Approve,
                    reviewed_at: 21,
                    valid_until: Some(180),
                    withdrawn_at: None,
                },
            ],
        }
    }

    pub fn packet(mappings: Vec<MaterialElementMappingV1>) -> DecisionClosurePacketV1 {
        DecisionClosurePacketV1 {
            schema_version: DECISION_CLOSURE_SCHEMA_VERSION,
            decision: decision(),
            mappings,
        }
    }
}

pub mod lineage_helpers {
    use super::evidence_lineage::*;
    pub fn reference(id: &str, revision: u64, digest_byte: u8) -> ArtifactRefV1 {
        ArtifactRefV1 {
            artifact_id: id.to_owned(),
            revision,
            digest: [digest_byte; 32],
        }
    }

    pub fn dependency(
        reference: ArtifactRefV1,
        relation: DependencyRelationV1,
        impact_policy: ImpactPolicyV1,
    ) -> ArtifactDependencyV1 {
        ArtifactDependencyV1 {
            reference,
            relation,
            impact_policy,
        }
    }

    pub fn node(
        reference: ArtifactRefV1,
        dependencies: Vec<ArtifactDependencyV1>,
    ) -> ArtifactNodeV1 {
        ArtifactNodeV1 {
            reference,
            kind: ArtifactKindV1::Other,
            dependencies,
        }
    }

    pub fn event(
        event_id: &str,
        sequence: u64,
        target: ArtifactRefV1,
        reason: ChangeReasonV1,
        replacement: Option<ArtifactRefV1>,
    ) -> ChangeEventV1 {
        ChangeEventV1 {
            event_id: event_id.to_owned(),
            sequence,
            target,
            reason,
            replacement,
            occurred_at: 1_800_000_000 + sequence,
        }
    }

    pub fn state(evaluation: &LineageEvaluationV1, reference: &ArtifactRefV1) -> ArtifactStateV1 {
        evaluation
            .state_for(reference)
            .expect("artifact state exists")
    }
}

pub mod impact_helpers {
    use super::impact::*;
    pub const NOW: u64 = 100;
    pub fn digest(byte: u8) -> DigestV1 {
        [byte; 32]
    }

    pub fn artifact(id: &str, revision: u64, byte: u8) -> ArtifactVersionRefV1 {
        ArtifactVersionRefV1 {
            artifact_id: id.to_owned(),
            revision,
            digest: digest(byte),
        }
    }

    pub fn lease(id: &str) -> OwnerQualifiedLeaseRefV1 {
        OwnerQualifiedLeaseRefV1 {
            lease_id: id.to_owned(),
            digest: digest(90),
            verified_at: NOW - 50,
            valid_until: NOW + 50,
        }
    }

    pub fn expected_scope() -> ExpectedImpactScopeV1 {
        let source = artifact("source", 3, 3);
        let downstream = artifact("decision", 4, 4);
        ExpectedImpactScopeV1 {
            change_event: ChangeEventRefV1 {
                event_id: "event-7".to_owned(),
                digest: digest(6),
            },
            changed_artifact: source.clone(),
            dependent_artifact: downstream.clone(),
            dependency_edge: DependencyEdgeRefV1 {
                edge_id: "decision-source".to_owned(),
                digest: digest(5),
                upstream: source,
                downstream,
            },
            context: UseContextRefV1 {
                context_id: "project-scope".to_owned(),
                digest: digest(7),
                jurisdiction: "ZA".to_owned(),
                population_scope_digest: digest(8),
                use_scope_digest: digest(9),
            },
            domain_authority: artifact("domain-charter", 2, 12),
            safety_authority: Some(artifact("safety-charter", 1, 13)),
        }
    }

    pub fn determination(status: ImpactStatusV1) -> ImpactDeterminationV1 {
        let expected = expected_scope();
        let justification = match status {
            ImpactStatusV1::Affected => ImpactJustificationV1::DirectlyAffected,
            ImpactStatusV1::NotAffected => ImpactJustificationV1::NotInExecutionPath,
            ImpactStatusV1::UnderInvestigation => ImpactJustificationV1::InsufficientInformation,
        };
        ImpactDeterminationV1 {
            determination_id: "impact-1".to_owned(),
            digest: digest(10),
            change_event: expected.change_event.clone(),
            changed_artifact: expected.changed_artifact.clone(),
            dependent_artifact: expected.dependent_artifact.clone(),
            dependency_edge: expected.dependency_edge.clone(),
            context: expected.context.clone(),
            status,
            justification,
            evidence: vec![ImpactEvidenceRefV1 {
                evidence_id: "execution-map".to_owned(),
                digest: digest(11),
                context_digest: expected.context.digest,
            }],
            assessor_id: "assessor".to_owned(),
            issued_at: NOW - 20,
            lease: lease("impact-lease"),
            reviews: vec![
                ImpactReviewV1 {
                    reviewer_id: "domain-reviewer".to_owned(),
                    role: ImpactReviewerRoleV1::DomainAuthority,
                    approved_determination_digest: digest(10),
                    authority_ref: expected.domain_authority.clone(),
                    reviewed_at: NOW - 10,
                    lease: lease("domain-review-lease"),
                },
                ImpactReviewV1 {
                    reviewer_id: "safety-reviewer".to_owned(),
                    role: ImpactReviewerRoleV1::SafetyAuthority,
                    approved_determination_digest: digest(10),
                    authority_ref: expected
                        .safety_authority
                        .clone()
                        .expect("safety authority ref"),
                    reviewed_at: NOW - 9,
                    lease: lease("safety-review-lease"),
                },
            ],
        }
    }

    pub fn item(
        id: &str,
        state: SupportEvidenceStateV1,
        group: Option<&str>,
        byte: u8,
    ) -> SupportEvidenceV1 {
        SupportEvidenceV1 {
            evidence_id: id.to_owned(),
            digest: digest(byte),
            independence_group_id: group.map(str::to_owned),
            state,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wsr_action_denominator_action(
        action_id: &str,
        prerequisites: &[&str],
    ) -> action_denominator::AuthorizedActionV1 {
        action_denominator::AuthorizedActionV1 {
            action_id: action_id.to_owned(),
            action_type: format!("water-service:{action_id}"),
            risk: action_denominator::ActionRiskV1::Consequential,
            responsible_role: "municipal-water-maintainer".to_owned(),
            required_qualification_refs: vec!["qualification:current".to_owned()],
            required_permit_refs: vec!["permit:current".to_owned()],
            prerequisite_action_ids: prerequisites
                .iter()
                .map(|value| (*value).to_owned())
                .collect(),
            evidence_profile_ref: "water-service-action-evidence-v1".to_owned(),
            evidence_profile_digest: [0xa1; 32],
            minimum_independent_groups: 1,
        }
    }

    fn wsr_action_denominator_draft(
        revision: u64,
        supersedes: Option<action_denominator::DigestV1>,
        authority_decision_ref: &str,
        authority_decision_digest: [u8; 32],
        action_ids: &[&str],
    ) -> action_denominator::ActionPlanDraftV1 {
        action_denominator::ActionPlanDraftV1 {
            plan_ref: "wsr-plan:episode-010".to_owned(),
            episode_ref: "wsr-010-episode".to_owned(),
            plan_revision: revision,
            supersedes_plan_digest: supersedes,
            authority: action_denominator::AuthorityRefV1 {
                authority_ref: "authority:synthetic-local-water-office".to_owned(),
                decision_ref: authority_decision_ref.to_owned(),
                decision_digest: authority_decision_digest,
            },
            scope: action_denominator::ServiceScopeV1 {
                episode_ref: "wsr-010-episode".to_owned(),
                zone_ref: "zone-synthetic-a".to_owned(),
                jurisdiction_ref: "jurisdiction:synthetic".to_owned(),
                context_digest: [0xa2; 32],
            },
            policy_profile_ref: "water-service-action-policy-v1".to_owned(),
            policy_profile_digest: [0xa3; 32],
            issued_at: 10,
            valid_until: 1_000,
            actions: action_ids
                .iter()
                .map(|action_id| wsr_action_denominator_action(action_id, &[]))
                .collect(),
        }
    }

    fn wsr_action_denominator_observation(
        plan: &action_denominator::AuthorizedActionPlanV1,
        observation_id: &str,
        action_id: &str,
        independence_group: &str,
    ) -> action_denominator::ActionObservationV1 {
        let action = plan
            .actions()
            .iter()
            .find(|action| action.action_id == action_id)
            .expect("test observation references an action in the sealed plan");
        action_denominator::ActionObservationV1 {
            observation_id: observation_id.to_owned(),
            plan_digest: plan.digest(),
            episode_ref: plan.episode_ref().to_owned(),
            action_id: action_id.to_owned(),
            zone_ref: plan.scope().zone_ref.clone(),
            jurisdiction_ref: plan.scope().jurisdiction_ref.clone(),
            context_digest: plan.scope().context_digest,
            source_ref: format!("source:{independence_group}"),
            source_revision: 1,
            evidence_digest: [0xb1; 32],
            evidence_profile_digest: action.evidence_profile_digest,
            independence_group: independence_group.to_owned(),
            verifier_ref: "verifier:synthetic-water-review".to_owned(),
            observed_at: 100,
            valid_until: 900,
            status: action_denominator::ObservationStatusV1::VerifiedComplete,
        }
    }

    #[test]
    fn wsr001_provider_completion_alone_never_establishes_restoration_or_quality() {
        let receipt = evaluate_episode(
            "WSR-001",
            WaterEpisode {
                provider_reported_complete: true,
                current_pressure_restoration: false,
                current_water_quality: false,
            },
            2,
        );
        assert_eq!(receipt.oracle_disposition, ExpectedDisposition::NotEligible);
        assert_eq!(
            receipt.owner_dispositions.decision_closure,
            decision_closure::DecisionClosureDispositionV1::OpenMissingElements
        );
    }

    #[test]
    fn wsr004_current_independent_evidence_only_reaches_local_review() {
        let receipt = evaluate_episode(
            "WSR-004",
            WaterEpisode {
                provider_reported_complete: true,
                current_pressure_restoration: true,
                current_water_quality: true,
            },
            1,
        );
        assert_eq!(
            receipt.oracle_disposition,
            ExpectedDisposition::EligibleForLocalReview
        );
        assert_eq!(
            receipt.owner_dispositions.decision_closure,
            decision_closure::DecisionClosureDispositionV1::MappingReviewRequired
        );
    }

    #[test]
    fn expired_upstream_source_requires_targeted_revalidation() {
        let lineage = source_lineage(true);
        let claim = lineage_helpers::reference("wsr:pressure-claim", 1, 2);
        let decision = lineage_helpers::reference("wsr:restoration-decision", 1, 3);
        let unaffected_quality_source =
            lineage_helpers::reference("wsr:quality-source-independent", 1, 4);
        assert_eq!(
            lineage.state_for(&unaffected_quality_source),
            Some(evidence_lineage::ArtifactStateV1::Current)
        );
        assert_eq!(
            lineage.state_for(&claim),
            Some(evidence_lineage::ArtifactStateV1::NeedsRevalidation)
        );
        assert_eq!(
            lineage.state_for(&decision),
            Some(evidence_lineage::ArtifactStateV1::NeedsRevalidation)
        );
    }

    #[test]
    fn wsr006_wrong_zone_scope_qualifier_cannot_become_direct_support() {
        use claim_support::{
            ClaimSupportDispositionV1, QualifierRelationV1, ReviewDecisionV1, SemanticDimensionV1,
        };

        let mut packet = claim_packet("water:pressure-restoration");
        let scope = packet.assessments[0]
            .qualifiers
            .iter_mut()
            .find(|qualifier| qualifier.dimension == SemanticDimensionV1::Scope)
            .expect("scope is a required semantic dimension");
        // The target claim is scoped to ZONE-SYN-F, while the observation is
        // sourced from another zone. The two opaque fixture digests intentionally
        // differ; the core must not promote the assessment to direct support.
        scope.claim_value_digest = Some([0xf1; 32]);
        scope.source_value_digest = Some([0x66; 32]);
        scope.relation = QualifierRelationV1::Contradicted;
        packet.reviews.push(claim_support_helpers::review(
            ReviewDecisionV1::ConfirmDirectSupport,
        ));

        let result = claim_support::evaluate_claim_support_result(
            &packet,
            100,
            "result:wrong-zone-pressure",
            [74; 32],
        )
        .expect("wrong-zone packet is structurally valid but semantically unsupported");
        assert_eq!(
            result.disposition,
            ClaimSupportDispositionV1::HumanReviewRequired
        );
        assert_ne!(result.disposition, ClaimSupportDispositionV1::Supported);
    }

    #[test]
    fn wsr009_expired_permit_and_revoked_crew_block_safety_critical_work() {
        use evidence_lineage::{
            ArtifactKindV1, ArtifactStateV1, ChangeReasonV1, DependencyRelationV1, ImpactPolicyV1,
        };
        use project_formation::EvidenceDispositionV1;
        use project_formation::RequirementDispositionV1;
        use project_formation::RequirementKindV1;

        let project = project_helpers::project(RequirementKindV1::SafetyCritical, 1);
        let proposal = project_helpers::proposal(
            vec![project_helpers::member("crew:maintainer-1")],
            vec![project_helpers::assignment(
                "crew:maintainer-1",
                vec![project_helpers::evidence(
                    "crew:maintainer-1",
                    EvidenceDispositionV1::Revoked,
                )],
            )],
        );
        let team = project_formation::evaluate_team(&project, &proposal, 10)
            .expect("revoked crew evidence remains a typed safety blocker");
        assert_eq!(
            team.requirements[0].disposition,
            RequirementDispositionV1::BlockedBySafetyGate
        );

        let permit = lineage_helpers::reference("wsr:permit:worksite-7", 1, 51);
        let crew = lineage_helpers::reference("wsr:crew-qualification:maintainer-1", 1, 52);
        let authorization = lineage_helpers::reference("wsr:work-authorization:episode-11", 1, 53);
        let nodes = vec![
            evidence_lineage::ArtifactNodeV1 {
                reference: permit.clone(),
                kind: ArtifactKindV1::Source,
                dependencies: vec![],
            },
            evidence_lineage::ArtifactNodeV1 {
                reference: crew.clone(),
                kind: ArtifactKindV1::Source,
                dependencies: vec![],
            },
            evidence_lineage::ArtifactNodeV1 {
                reference: authorization.clone(),
                kind: ArtifactKindV1::Other,
                dependencies: vec![
                    lineage_helpers::dependency(
                        permit.clone(),
                        DependencyRelationV1::SourcedFrom,
                        ImpactPolicyV1::RevalidateOnChange,
                    ),
                    lineage_helpers::dependency(
                        crew.clone(),
                        DependencyRelationV1::SourcedFrom,
                        ImpactPolicyV1::RevalidateOnChange,
                    ),
                ],
            },
        ];
        let events = vec![
            lineage_helpers::event(
                "wsr:permit-expired",
                1,
                permit.clone(),
                ChangeReasonV1::Expired,
                None,
            ),
            lineage_helpers::event(
                "wsr:crew-revoked",
                2,
                crew.clone(),
                ChangeReasonV1::Revoked,
                None,
            ),
        ];
        let lineage = evidence_lineage::evaluate_lineage(&nodes, &events)
            .expect("permit and crew change events have unique monotone sequences");
        assert_eq!(lineage.state_for(&permit), Some(ArtifactStateV1::Expired));
        assert_eq!(lineage.state_for(&crew), Some(ArtifactStateV1::Revoked));
        assert_eq!(
            lineage.state_for(&authorization),
            Some(ArtifactStateV1::NeedsRevalidation)
        );
    }

    #[test]
    fn duplicated_common_cause_cannot_satisfy_independent_k_of_n() {
        use impact::{SupportEvidenceStateV1, SupportRuleV1, SupportSufficiencyV1};
        let result = impact::evaluate_support_sufficiency(
            &SupportRuleV1::AtLeastIndependent {
                candidate_evidence_ids: vec!["meter-a".into(), "meter-b".into()],
                minimum: 2,
            },
            &[
                impact_helpers::item(
                    "meter-a",
                    SupportEvidenceStateV1::DirectSupport,
                    Some("same-meter-cluster"),
                    41,
                ),
                impact_helpers::item(
                    "meter-b",
                    SupportEvidenceStateV1::DirectSupport,
                    Some("same-meter-cluster"),
                    42,
                ),
            ],
        )
        .unwrap();
        assert_eq!(result, SupportSufficiencyV1::InsufficientEvidence);
    }

    #[test]
    fn wsr010_action_coverage_uses_the_canonical_immutable_denominator_owner() {
        let plan = action_denominator::AuthorizedActionPlanV1::seal(wsr_action_denominator_draft(
            1,
            None,
            "authority-decision:wsr010:p1",
            [0x81; 32],
            &[
                "isolate",
                "repair",
                "pressure-check",
                "quality-sample",
                "site-safety-check",
                "handover-documentation",
            ],
        ))
        .expect("the initial six-action denominator is canonical and valid");

        let isolate = wsr_action_denominator_observation(
            &plan,
            "obs:isolate",
            "isolate",
            "independent:field-crew",
        );
        let repair = wsr_action_denominator_observation(
            &plan,
            "obs:repair",
            "repair",
            "independent:field-crew",
        );
        let pressure_check = wsr_action_denominator_observation(
            &plan,
            "obs:pressure-check",
            "pressure-check",
            "independent:pressure-team",
        );
        let quality_sample = wsr_action_denominator_observation(
            &plan,
            "obs:quality-sample",
            "quality-sample",
            "independent:laboratory",
        );
        let safety_check = wsr_action_denominator_observation(
            &plan,
            "obs:site-safety-check",
            "site-safety-check",
            "independent:safety-inspector",
        );
        let mut extra = wsr_action_denominator_observation(
            &plan,
            "obs:extra",
            "repair",
            "independent:field-crew",
        );
        // Start from a valid plan-bound observation, then mutate only its action
        // identity. This exercises the owner's UnknownAction rejection without
        // asking the fixture constructor to invent an action absent from the plan.
        extra.action_id = "unapproved-extra-work".to_owned();
        let mut observations = vec![
            isolate,
            repair.clone(),
            pressure_check,
            quality_sample,
            safety_check,
            extra,
        ];
        // One real action observation repeated 100 times must still contribute
        // exactly one action to the numerator.
        observations.extend(std::iter::repeat_n(repair, 99));
        let receipt = action_denominator::evaluate_action_coverage_v1(&plan, &observations, 200)
            .expect("owner evaluates a plan-bound observation set");
        assert_eq!(receipt.denominator_count, 6);
        assert_eq!(receipt.covered_count, 5);
        assert_eq!(receipt.coverage_basis_points, 8_333);
        assert_eq!(receipt.uncovered_action_ids, vec!["handover-documentation"]);
        assert_eq!(receipt.identical_duplicate_observations_ignored, 99);
        assert!(receipt.rejected_observations.iter().any(|rejected| {
            rejected.action_id == "unapproved-extra-work"
                && rejected.reason == action_denominator::RejectionReasonV1::UnknownAction
        }));

        let mut expanded_observations = observations;
        let mut second_extra = wsr_action_denominator_observation(
            &plan,
            "obs:another-extra",
            "repair",
            "independent:field-crew",
        );
        second_extra.action_id = "another-unapproved-action".to_owned();
        expanded_observations.push(second_extra);
        let expanded =
            action_denominator::evaluate_action_coverage_v1(&plan, &expanded_observations, 200)
                .unwrap();
        assert_eq!(expanded.covered_count, receipt.covered_count);
        assert_eq!(expanded.denominator_count, receipt.denominator_count);
    }

    #[test]
    fn wsr010_denominator_revision_cannot_reuse_observations_from_prior_authority() {
        let original =
            action_denominator::AuthorizedActionPlanV1::seal(wsr_action_denominator_draft(
                1,
                None,
                "authority-decision:wsr010:p1",
                [0x82; 32],
                &[
                    "isolate",
                    "repair",
                    "pressure-check",
                    "quality-sample",
                    "site-safety-check",
                    "handover-documentation",
                ],
            ))
            .unwrap();
        let original_observations = vec![
            wsr_action_denominator_observation(
                &original,
                "p1:obs:isolate",
                "isolate",
                "independent:field-crew",
            ),
            wsr_action_denominator_observation(
                &original,
                "p1:obs:repair",
                "repair",
                "independent:field-crew",
            ),
            wsr_action_denominator_observation(
                &original,
                "p1:obs:pressure-check",
                "pressure-check",
                "independent:pressure-team",
            ),
            wsr_action_denominator_observation(
                &original,
                "p1:obs:quality-sample",
                "quality-sample",
                "independent:laboratory",
            ),
            wsr_action_denominator_observation(
                &original,
                "p1:obs:site-safety-check",
                "site-safety-check",
                "independent:safety-inspector",
            ),
        ];
        let original_receipt =
            action_denominator::evaluate_action_coverage_v1(&original, &original_observations, 200)
                .unwrap();

        let amended = action_denominator::AuthorizedActionPlanV1::seal_amendment(
            &original,
            wsr_action_denominator_draft(
                2,
                Some(original.digest()),
                "authority-decision:wsr010:p2",
                [0x83; 32],
                &[
                    "isolate",
                    "repair",
                    "pressure-check",
                    "quality-sample",
                    "site-safety-check",
                ],
            ),
        )
        .expect("the amended plan is a new authority-bound denominator revision");

        let historical_replay =
            action_denominator::evaluate_action_coverage_v1(&original, &original_observations, 200)
                .unwrap();
        assert_eq!(historical_replay, original_receipt);
        assert_eq!(historical_replay.denominator_count, 6);
        assert_eq!(historical_replay.covered_count, 5);
        assert_eq!(
            historical_replay.uncovered_action_ids,
            vec!["handover-documentation"]
        );

        let prior_on_amended =
            action_denominator::evaluate_action_coverage_v1(&amended, &original_observations, 200)
                .unwrap();
        assert_eq!(prior_on_amended.covered_count, 0);
        assert_eq!(prior_on_amended.denominator_count, 5);
        assert!(prior_on_amended
            .rejected_observations
            .iter()
            .all(|rejected| {
                rejected.reason == action_denominator::RejectionReasonV1::WrongPlan
            }));

        let revised_observations: Vec<_> = amended
            .actions()
            .iter()
            .map(|action| {
                wsr_action_denominator_observation(
                    &amended,
                    &format!("p2:obs:{}", action.action_id),
                    &action.action_id,
                    &format!("independent:p2:{}", action.action_id),
                )
            })
            .collect();
        let revised_receipt =
            action_denominator::evaluate_action_coverage_v1(&amended, &revised_observations, 200)
                .unwrap();
        assert_eq!(revised_receipt.covered_count, 5);
        assert_eq!(revised_receipt.denominator_count, 5);
        assert_eq!(
            revised_receipt.disposition,
            action_denominator::CoverageDispositionV1::CompleteForAuthorizedPlan
        );
        assert_eq!(original_receipt.covered_count, 5);
        assert_eq!(original_receipt.denominator_count, 6);
    }

    #[test]
    fn wsr010_complete_action_coverage_does_not_auto_close_the_service_episode() {
        let plan = action_denominator::AuthorizedActionPlanV1::seal(wsr_action_denominator_draft(
            1,
            None,
            "authority-decision:wsr010:complete",
            [0x84; 32],
            &[
                "isolate",
                "repair",
                "pressure-check",
                "quality-sample",
                "site-safety-check",
                "handover-documentation",
            ],
        ))
        .unwrap();
        let observations: Vec<_> = plan
            .actions()
            .iter()
            .map(|action| {
                wsr_action_denominator_observation(
                    &plan,
                    &format!("obs:{}", action.action_id),
                    &action.action_id,
                    &format!("independent:{}", action.action_id),
                )
            })
            .collect();
        let coverage =
            action_denominator::evaluate_action_coverage_v1(&plan, &observations, 200).unwrap();
        assert_eq!(
            coverage.disposition,
            action_denominator::CoverageDispositionV1::CompleteForAuthorizedPlan
        );

        let service = evaluate_episode(
            "WSR-010",
            WaterEpisode {
                provider_reported_complete: true,
                current_pressure_restoration: true,
                current_water_quality: true,
            },
            1,
        );
        assert_eq!(
            service.owner_dispositions.decision_closure,
            decision_closure::DecisionClosureDispositionV1::MappingReviewRequired
        );
        assert_ne!(
            service.owner_dispositions.decision_closure,
            decision_closure::DecisionClosureDispositionV1::Closed
        );
    }

    #[test]
    fn wsr011_repeat_burst_is_a_new_episode_and_preserves_prior_closure() {
        use evidence_lineage::{
            ArtifactKindV1, ArtifactStateV1, ChangeReasonV1, DependencyRelationV1, ImpactPolicyV1,
        };

        let prior_episode = lineage_helpers::reference("wsr:episode:prior", 1, 61);
        let prior_closure = lineage_helpers::reference("wsr:closure:prior", 1, 62);
        let repeat_episode = lineage_helpers::reference("wsr:episode:repeat", 1, 63);
        let repeat_closure = lineage_helpers::reference("wsr:closure:repeat", 1, 64);
        let nodes = vec![
            evidence_lineage::ArtifactNodeV1 {
                reference: prior_episode.clone(),
                kind: ArtifactKindV1::Source,
                dependencies: vec![],
            },
            evidence_lineage::ArtifactNodeV1 {
                reference: prior_closure.clone(),
                kind: ArtifactKindV1::Other,
                dependencies: vec![lineage_helpers::dependency(
                    prior_episode.clone(),
                    DependencyRelationV1::Verifies,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            },
            evidence_lineage::ArtifactNodeV1 {
                reference: repeat_episode.clone(),
                kind: ArtifactKindV1::Source,
                dependencies: vec![],
            },
            evidence_lineage::ArtifactNodeV1 {
                reference: repeat_closure.clone(),
                kind: ArtifactKindV1::Other,
                dependencies: vec![lineage_helpers::dependency(
                    repeat_episode.clone(),
                    DependencyRelationV1::Verifies,
                    ImpactPolicyV1::RevalidateOnChange,
                )],
            },
        ];
        let events = vec![lineage_helpers::event(
            "wsr:repeat-episode-evidence-expired",
            1,
            repeat_episode.clone(),
            ChangeReasonV1::Expired,
            None,
        )];

        let lineage = evidence_lineage::evaluate_lineage(&nodes, &events)
            .expect("repeat episode has a distinct identity and dependency closure");
        assert_ne!(prior_episode, repeat_episode);
        assert_eq!(
            lineage.state_for(&prior_closure),
            Some(ArtifactStateV1::Current),
            "a new burst must not rewrite the historical closure for the prior episode"
        );
        assert_eq!(
            lineage.state_for(&repeat_closure),
            Some(ArtifactStateV1::NeedsRevalidation),
            "the new episode's expired evidence must affect its own dependent closure"
        );
    }

    #[test]
    fn wsr012_unresolved_high_consequence_water_quality_blocks_review_eligibility() {
        let receipt = evaluate_episode(
            "WSR-012",
            WaterEpisode {
                provider_reported_complete: true,
                current_pressure_restoration: true,
                current_water_quality: false,
            },
            2,
        );
        assert_eq!(
            receipt.owner_dispositions.quality_support,
            claim_support::ClaimSupportDispositionV1::InsufficientEvidence
        );
        assert_eq!(
            receipt.owner_dispositions.evidence_sufficiency,
            impact::SupportSufficiencyV1::InsufficientEvidence
        );
        assert_eq!(
            receipt.owner_dispositions.decision_closure,
            decision_closure::DecisionClosureDispositionV1::OpenMissingElements
        );
        assert_eq!(receipt.oracle_disposition, ExpectedDisposition::NotEligible);
    }
}
