//! Evidence-bounded construction provenance and approval trace checks.
//!
//! Accepted means only that this crate's v0.1 record-binding contract passed.
//! It is not a permit, engineering certification, safety determination or
//! professional acceptance of a constructed asset.

use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceStatus {
    Verified,
    Revoked,
    Disputed,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsePermission {
    Permitted,
    Prohibited,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityStatus {
    Active,
    Revoked,
    Disputed,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApprovalDecision {
    Approved,
    Rejected,
    Deferred,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InspectionOutcome {
    Conforming,
    Nonconforming,
    Inconclusive,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Disposition {
    Accepted,
    Rejected,
    Indeterminate,
    Disputed,
    Superseded,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceAssertion {
    pub reference: String,
    /// Exact record or claim scope this evidence is permitted to support.
    pub subject_ref: String,
    pub sha256: String,
    pub status: EvidenceStatus,
    pub use_permission: UsePermission,
    /// First time this relying party could use the evidence; not source creation time.
    pub available_at: u64,
    /// Domain-valid start, inclusive.
    pub valid_from: u64,
    /// Domain-valid end, inclusive; None means no declared end.
    pub valid_until: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExternalArtifactRef {
    /// Stable opaque reference in the owning system.
    pub reference: String,
    pub owner_system: String,
    /// Examples: design_revision, simulation_run, portfolio_alternative.
    pub artifact_role: String,
    pub revision_ref: String,
    pub sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityAssertion {
    pub reference: String,
    pub signer_id: String,
    pub role: String,
    /// Canonical scope reference issued by the external authority resolver.
    pub scope_ref: String,
    pub status: AuthorityStatus,
    /// First time this relying party could use the authority assertion.
    pub available_at: u64,
    pub valid_from: u64,
    pub valid_until: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DesignRevision {
    pub revision_id: String,
    pub source_artifact: ExternalArtifactRef,
    pub ifc_model_export: ExternalArtifactRef,
    pub drawing_package: ExternalArtifactRef,
    pub sha256: String,
    pub supersedes_revision_id: Option<String>,
    pub created_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimulationRun {
    pub run_id: String,
    pub execution_artifact: ExternalArtifactRef,
    pub design_revision_id: String,
    pub design_sha256: String,
    pub solver_id: String,
    pub solver_version: String,
    pub result_sha256: String,
    pub completed_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ApprovalCondition {
    pub condition_id: String,
    pub satisfied: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ApprovalReceipt {
    pub approval_id: String,
    pub design_revision_id: String,
    pub design_sha256: String,
    pub simulation_run_id: String,
    pub simulation_result_sha256: String,
    pub authority_ref: String,
    pub signer_id: String,
    pub role: String,
    pub scope_ref: String,
    pub supersedes_approval_id: Option<String>,
    pub decision: ApprovalDecision,
    pub conditions: Vec<ApprovalCondition>,
    pub evidence_refs: Vec<String>,
    /// Dissent is retained; its presence is not silently converted into consent.
    pub dissent_refs: Vec<String>,
    pub decided_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MaterialBatch {
    pub batch_id: String,
    pub supplier_party_id: String,
    pub receiving_site_party_id: String,
    pub material_description: String,
    /// Supplier statement only. It is not proof of the statement.
    pub supplier_assertion: Option<String>,
    /// Must resolve to an externally attested certification/evidence record.
    pub certification_evidence_ref: Option<String>,
    /// Exact supplier claim scope covered by the certification reference.
    pub certification_scope_ref: Option<String>,
    pub declared_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CustodyReceipt {
    pub receipt_id: String,
    pub batch_id: String,
    pub from_party_id: String,
    pub to_party_id: String,
    pub predecessor_receipt_id: Option<String>,
    pub evidence_ref: String,
    pub occurred_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InspectionReceipt {
    pub inspection_id: String,
    pub batch_id: String,
    pub design_revision_id: String,
    pub design_sha256: String,
    pub terminal_custody_receipt_id: String,
    pub authority_ref: String,
    pub inspector_id: String,
    pub role: String,
    pub scope_ref: String,
    pub evidence_refs: Vec<String>,
    pub outcome: InspectionOutcome,
    pub inspected_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConstructionProjectTrace {
    pub schema_version: u32,
    pub project_id: String,
    pub current_design_revision_id: String,
    /// Optional link to site context; never a second GIS record.
    pub site_context: Option<ExternalArtifactRef>,
    /// Optional link to a Sol-Atlas Foundry alternative; never a second GIS record.
    pub external_portfolio_context: Option<ExternalArtifactRef>,
    pub design_revisions: Vec<DesignRevision>,
    pub simulation_runs: Vec<SimulationRun>,
    pub authority_assertions: Vec<AuthorityAssertion>,
    pub evidence_assertions: Vec<EvidenceAssertion>,
    pub approvals: Vec<ApprovalReceipt>,
    pub material_batches: Vec<MaterialBatch>,
    pub custody_receipts: Vec<CustodyReceipt>,
    pub inspections: Vec<InspectionReceipt>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Finding {
    pub subject_type: String,
    pub subject_id: String,
    pub disposition: Disposition,
    pub code: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AuditReport {
    pub findings: Vec<Finding>,
}

impl AuditReport {
    fn push(
        &mut self,
        subject_type: &str,
        subject_id: &str,
        disposition: Disposition,
        code: &str,
    ) {
        self.findings.push(Finding {
            subject_type: subject_type.to_owned(),
            subject_id: subject_id.to_owned(),
            disposition,
            code: code.to_owned(),
        });
    }

    pub fn disposition_for(&self, subject_type: &str, subject_id: &str) -> Option<Disposition> {
        self.findings
            .iter()
            .find(|finding| {
                finding.subject_type == subject_type && finding.subject_id == subject_id
            })
            .map(|finding| finding.disposition)
    }
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn external_artifact_disposition(
    artifact: &ExternalArtifactRef,
    expected_owner: Option<&str>,
    expected_role: &str,
    expected_revision: &str,
    expected_sha256: Option<&str>,
) -> Disposition {
    if !nonempty(&artifact.reference)
        || !nonempty(&artifact.owner_system)
        || !nonempty(&artifact.artifact_role)
        || !nonempty(&artifact.revision_ref)
    {
        return Disposition::Indeterminate;
    }
    if expected_owner.is_some_and(|owner| artifact.owner_system != owner)
        || artifact.artifact_role != expected_role
        || artifact.revision_ref != expected_revision
        || expected_sha256.is_some_and(|digest| artifact.sha256 != digest)
        || !valid_sha256(&artifact.sha256)
    {
        return Disposition::Rejected;
    }
    Disposition::Accepted
}

fn design_artifact_disposition(design: &DesignRevision) -> Disposition {
    combine(
        combine(
            external_artifact_disposition(
                &design.source_artifact,
                None,
                "design_revision",
                &design.revision_id,
                Some(&design.sha256),
            ),
            external_artifact_disposition(
                &design.ifc_model_export,
                None,
                "ifc_model_export",
                &design.revision_id,
                None,
            ),
        ),
        external_artifact_disposition(
            &design.drawing_package,
            None,
            "drawing_package",
            &design.revision_id,
            None,
        ),
    )
}

fn simulation_artifact_disposition(run: &SimulationRun) -> Disposition {
    external_artifact_disposition(
        &run.execution_artifact,
        None,
        "simulation_run",
        &run.run_id,
        Some(&run.result_sha256),
    )
}

fn valid_at(from: u64, until: Option<u64>, available_at: u64, event_at: u64) -> bool {
    available_at <= event_at && from <= event_at && until.is_none_or(|end| event_at <= end)
}

fn combine(current: Disposition, next: Disposition) -> Disposition {
    use Disposition::{Accepted, Disputed, Indeterminate, Rejected, Superseded};
    match (current, next) {
        (Rejected, _) | (_, Rejected) => Rejected,
        (Disputed, _) | (_, Disputed) => Disputed,
        (Indeterminate, _) | (_, Indeterminate) => Indeterminate,
        (Superseded, _) | (_, Superseded) => Superseded,
        _ => Accepted,
    }
}

fn evidence_disposition(
    trace: &ConstructionProjectTrace,
    reference: &str,
    subject_ref: &str,
    event_at: u64,
) -> Disposition {
    if !nonempty(reference) || !nonempty(subject_ref) {
        return Disposition::Indeterminate;
    }
    let matching = trace.evidence_assertions.iter().filter(|item| item.reference == reference).collect::<Vec<_>>();
    let Some(evidence) = matching.first().copied() else {
        return Disposition::Indeterminate;
    };
    if matching.len() != 1 || evidence.subject_ref != subject_ref {
        return Disposition::Rejected;
    }
    if !valid_sha256(&evidence.sha256) {
        return Disposition::Rejected;
    }
    match evidence.status {
        EvidenceStatus::Revoked => return Disposition::Rejected,
        EvidenceStatus::Disputed => return Disposition::Disputed,
        EvidenceStatus::Unknown => return Disposition::Indeterminate,
        EvidenceStatus::Verified => {}
    }
    match evidence.use_permission {
        UsePermission::Prohibited => return Disposition::Rejected,
        UsePermission::Unknown => return Disposition::Indeterminate,
        UsePermission::Permitted => {}
    }
    if valid_at(
        evidence.valid_from,
        evidence.valid_until,
        evidence.available_at,
        event_at,
    ) {
        Disposition::Accepted
    } else {
        Disposition::Indeterminate
    }
}

fn authority_disposition(
    trace: &ConstructionProjectTrace,
    reference: &str,
    signer_id: &str,
    role: &str,
    scope_ref: &str,
    event_at: u64,
) -> Disposition {
    if !nonempty(reference) || !nonempty(signer_id) || !nonempty(role) || !nonempty(scope_ref) {
        return Disposition::Indeterminate;
    }
    let matching = trace.authority_assertions.iter().filter(|item| item.reference == reference).collect::<Vec<_>>();
    let Some(authority) = matching.first().copied() else {
        return Disposition::Indeterminate;
    };
    if matching.len() != 1 {
        return Disposition::Disputed;
    }
    if authority.signer_id != signer_id
        || authority.role != role
        || authority.scope_ref != scope_ref
    {
        return Disposition::Rejected;
    }
    match authority.status {
        AuthorityStatus::Revoked => return Disposition::Rejected,
        AuthorityStatus::Disputed => return Disposition::Disputed,
        AuthorityStatus::Unknown => return Disposition::Indeterminate,
        AuthorityStatus::Active => {}
    }
    if valid_at(
        authority.valid_from,
        authority.valid_until,
        authority.available_at,
        event_at,
    ) {
        Disposition::Accepted
    } else {
        Disposition::Indeterminate
    }
}

fn design_for(
    trace: &ConstructionProjectTrace,
    revision_id: &str,
) -> Option<&DesignRevision> {
    trace
        .design_revisions
        .iter()
        .find(|revision| revision.revision_id == revision_id)
}

fn simulation_for(
    trace: &ConstructionProjectTrace,
    run_id: &str,
) -> Option<&SimulationRun> {
    trace.simulation_runs.iter().find(|run| run.run_id == run_id)
}

fn revision_lineage_is_valid(
    trace: &ConstructionProjectTrace,
    revision_id: &str,
) -> bool {
    let mut visited = BTreeSet::new();
    let mut current_id = revision_id;

    loop {
        if !visited.insert(current_id.to_owned()) {
            return false;
        }
        let matching = trace
            .design_revisions
            .iter()
            .filter(|revision| revision.revision_id == current_id)
            .collect::<Vec<_>>();
        let Some(current) = matching.first().copied() else {
            return false;
        };
        if matching.len() != 1 {
            return false;
        }
        let Some(parent_id) = current.supersedes_revision_id.as_deref() else {
            return true;
        };
        let children = trace
            .design_revisions
            .iter()
            .filter(|revision| revision.supersedes_revision_id.as_deref() == Some(parent_id))
            .count();
        if children != 1 {
            return false;
        }
        let parents = trace
            .design_revisions
            .iter()
            .filter(|revision| revision.revision_id == parent_id)
            .collect::<Vec<_>>();
        let Some(parent) = parents.first() else {
            return false;
        };
        if parents.len() != 1 || parent.created_at >= current.created_at {
            return false;
        }
        current_id = parent_id;
    }
}

fn approval_disposition(
    trace: &ConstructionProjectTrace,
    approval: &ApprovalReceipt,
    superseded: bool,
) -> Disposition {
    if superseded || approval.design_revision_id != trace.current_design_revision_id {
        return Disposition::Superseded;
    }
    if !valid_sha256(&approval.design_sha256)
        || !valid_sha256(&approval.simulation_result_sha256)
        || approval.approval_id.trim().is_empty()
    {
        return Disposition::Rejected;
    }
    if trace.design_revisions.iter().filter(|item| item.revision_id == approval.design_revision_id).count() != 1 {
        return Disposition::Rejected;
    }
    let Some(design) = design_for(trace, &approval.design_revision_id) else {
        return Disposition::Indeterminate;
    };
    if design.sha256 != approval.design_sha256 || !valid_sha256(&design.sha256) {
        return Disposition::Rejected;
    }
    let has_child = trace.design_revisions.iter().any(|candidate| {
        candidate.supersedes_revision_id.as_deref() == Some(approval.design_revision_id.as_str())
    });
    if has_child || !revision_lineage_is_valid(trace, &approval.design_revision_id) {
        return Disposition::Rejected;
    }
    let design_artifacts = design_artifact_disposition(design);
    if design_artifacts != Disposition::Accepted {
        return design_artifacts;
    }
    if trace.simulation_runs.iter().filter(|item| item.run_id == approval.simulation_run_id).count() != 1 {
        return Disposition::Rejected;
    }
    let Some(simulation) = simulation_for(trace, &approval.simulation_run_id) else {
        return Disposition::Indeterminate;
    };
    let simulation_artifact = simulation_artifact_disposition(simulation);
    if simulation_artifact != Disposition::Accepted {
        return simulation_artifact;
    }
    if simulation.design_revision_id != approval.design_revision_id
        || simulation.design_sha256 != approval.design_sha256
        || simulation.result_sha256 != approval.simulation_result_sha256
        || simulation.completed_at > approval.decided_at
    {
        return Disposition::Rejected;
    }
    if !nonempty(&simulation.solver_id) || !nonempty(&simulation.solver_version) {
        return Disposition::Indeterminate;
    }

    let mut result = authority_disposition(
        trace,
        &approval.authority_ref,
        &approval.signer_id,
        &approval.role,
        &approval.scope_ref,
        approval.decided_at,
    );
    if approval.decision == ApprovalDecision::Rejected {
        return Disposition::Rejected;
    }
    if approval.decision == ApprovalDecision::Deferred {
        return combine(result, Disposition::Indeterminate);
    }
    if approval.conditions.iter().any(|condition| !condition.satisfied) {
        result = combine(result, Disposition::Indeterminate);
    }
    if approval.evidence_refs.is_empty() {
        result = combine(result, Disposition::Indeterminate);
    }
    for evidence_ref in &approval.evidence_refs {
        result = combine(
            result,
            evidence_disposition(trace, evidence_ref, &approval.scope_ref, approval.decided_at),
        );
    }
    result
}

fn material_disposition(
    trace: &ConstructionProjectTrace,
    batch: &MaterialBatch,
) -> Disposition {
    if !nonempty(&batch.batch_id)
        || !nonempty(&batch.supplier_party_id)
        || !nonempty(&batch.receiving_site_party_id)
        || !nonempty(&batch.material_description)
    {
        return Disposition::Rejected;
    }
    if batch
        .supplier_assertion
        .as_ref()
        .is_none_or(|assertion| !nonempty(assertion))
    {
        return Disposition::Indeterminate;
    }
    let Some(reference) = batch.certification_evidence_ref.as_deref() else {
        return Disposition::Indeterminate;
    };
    let Some(scope_ref) = batch.certification_scope_ref.as_deref() else {
        return Disposition::Indeterminate;
    };
    evidence_disposition(trace, reference, scope_ref, batch.declared_at)
}

fn custody_disposition(
    trace: &ConstructionProjectTrace,
    batch: &MaterialBatch,
) -> Disposition {
    let mut receipts = trace
        .custody_receipts
        .iter()
        .filter(|receipt| receipt.batch_id == batch.batch_id)
        .collect::<Vec<_>>();
    if receipts.is_empty() {
        return Disposition::Indeterminate;
    }
    receipts.sort_by(|left, right| {
        (left.occurred_at, left.receipt_id.as_str())
            .cmp(&(right.occurred_at, right.receipt_id.as_str()))
    });

    let mut result = Disposition::Accepted;
    let mut receipt_ids = BTreeSet::new();
    let mut previous_id: Option<&str> = None;
    let mut previous_to: Option<&str> = None;
    let mut previous_time = 0_u64;

    for (index, receipt) in receipts.iter().enumerate() {
        if !receipt_ids.insert(receipt.receipt_id.as_str()) {
            return Disposition::Rejected;
        }
        if !nonempty(&receipt.receipt_id)
            || !nonempty(&receipt.from_party_id)
            || !nonempty(&receipt.to_party_id)
            || !nonempty(&receipt.evidence_ref)
        {
            return Disposition::Rejected;
        }
        if index == 0 {
            if receipt.predecessor_receipt_id.is_some()
                || receipt.from_party_id != batch.supplier_party_id
            {
                return Disposition::Rejected;
            }
        } else if receipt.predecessor_receipt_id.as_deref() != previous_id
            || Some(receipt.from_party_id.as_str()) != previous_to
            || receipt.occurred_at < previous_time
        {
            return Disposition::Rejected;
        }
        if trace.custody_receipts
            .iter()
            .filter(|item| item.receipt_id == receipt.receipt_id)
            .count() != 1
            || receipt.occurred_at < batch.declared_at
        {
            return Disposition::Rejected;
        }
        result = combine(
            result,
            evidence_disposition(trace, &receipt.evidence_ref, &receipt.receipt_id, receipt.occurred_at),
        );
        previous_id = Some(receipt.receipt_id.as_str());
        previous_to = Some(receipt.to_party_id.as_str());
        previous_time = receipt.occurred_at;
    }
    if previous_to != Some(batch.receiving_site_party_id.as_str()) {
        return Disposition::Indeterminate;
    }
    result
}

/// Independently evaluate each record family without minting a project-wide
/// readiness or safety decision.
pub fn verify_trace(trace: &ConstructionProjectTrace) -> AuditReport {
    let mut report = AuditReport::default();

    if trace.schema_version != 1 || !nonempty(&trace.project_id) {
        report.push("project", &trace.project_id, Disposition::Rejected, "invalid_project_envelope");
    }

    if let Some(site) = &trace.site_context {
        let disposition = external_artifact_disposition(
            site,
            Some("sol-atlas-leptos"),
            "site_context",
            &site.revision_ref,
            None,
        );
        report.push(
            "external_context",
            &site.reference,
            disposition,
            match disposition {
                Disposition::Accepted => "sol_atlas_site_reference_well_formed",
                Disposition::Rejected => "sol_atlas_site_reference_rejected",
                Disposition::Indeterminate => "sol_atlas_site_reference_incomplete",
                Disposition::Disputed => "sol_atlas_site_reference_disputed",
                Disposition::Superseded => "sol_atlas_site_reference_superseded",
            },
        );
    }

    if let Some(context) = &trace.external_portfolio_context {
        let disposition = external_artifact_disposition(
            context,
            Some("sol-atlas-leptos"),
            "portfolio_alternative",
            &context.revision_ref,
            None,
        );
        report.push(
            "external_context",
            &context.reference,
            disposition,
            match disposition {
                Disposition::Accepted => "sol_atlas_portfolio_reference_well_formed",
                Disposition::Rejected => "sol_atlas_portfolio_reference_rejected",
                Disposition::Indeterminate => "sol_atlas_portfolio_reference_incomplete",
                Disposition::Disputed => "sol_atlas_portfolio_reference_disputed",
                Disposition::Superseded => "sol_atlas_portfolio_reference_superseded",
            },
        );
    }

    let mut revision_ids = BTreeSet::new();
    let mut duplicate_revision_ids = BTreeSet::new();
    for revision in &trace.design_revisions {
        if !revision_ids.insert(revision.revision_id.clone()) {
            duplicate_revision_ids.insert(revision.revision_id.clone());
        }
    }
    for revision in &trace.design_revisions {
        if duplicate_revision_ids.contains(&revision.revision_id) {
            report.push("design", &revision.revision_id, Disposition::Rejected, "duplicate_revision_id");
            continue;
        }
        if !valid_sha256(&revision.sha256) || !nonempty(&revision.revision_id) {
            report.push("design", &revision.revision_id, Disposition::Rejected, "invalid_revision_identity_or_digest");
        } else if revision.revision_id != trace.current_design_revision_id {
            report.push("design", &revision.revision_id, Disposition::Superseded, "not_current_design_revision");
        } else {
            let has_child = trace.design_revisions.iter().any(|candidate| {
                candidate.supersedes_revision_id.as_deref() == Some(revision.revision_id.as_str())
            });
            let lineage_valid =
                !has_child && revision_lineage_is_valid(trace, &revision.revision_id);
            let combined = if lineage_valid {
                design_artifact_disposition(revision)
            } else {
                Disposition::Rejected
            };
            report.push(
                "design",
                &revision.revision_id,
                combined,
                match combined {
                    Disposition::Accepted => "current_revision_ifc_drawing_and_source_bound",
                    Disposition::Rejected => "invalid_revision_lineage_or_external_artifact_binding",
                    Disposition::Indeterminate => "external_design_artifact_reference_missing",
                    Disposition::Disputed => "external_design_artifact_reference_disputed",
                    Disposition::Superseded => "design_revision_superseded",
                },
            );
        }
    }
    if design_for(trace, &trace.current_design_revision_id).is_none() {
        report.push("design", &trace.current_design_revision_id, Disposition::Indeterminate, "current_revision_missing");
    }

    let mut simulation_ids = BTreeSet::new();
    let mut duplicate_simulation_ids = BTreeSet::new();
    for run in &trace.simulation_runs {
        if !simulation_ids.insert(run.run_id.clone()) {
            duplicate_simulation_ids.insert(run.run_id.clone());
        }
    }
    for run in &trace.simulation_runs {
        if duplicate_simulation_ids.contains(&run.run_id) {
            report.push("simulation", &run.run_id, Disposition::Rejected, "duplicate_simulation_id");
            continue;
        }
        if trace
            .design_revisions
            .iter()
            .filter(|revision| revision.revision_id == run.design_revision_id)
            .count() > 1
        {
            report.push("simulation", &run.run_id, Disposition::Rejected, "ambiguous_design_revision_identity");
            continue;
        }
        let Some(design) = design_for(trace, &run.design_revision_id) else {
            report.push("simulation", &run.run_id, Disposition::Indeterminate, "design_revision_missing");
            continue;
        };
        if run.design_revision_id != trace.current_design_revision_id {
            report.push("simulation", &run.run_id, Disposition::Superseded, "simulation_targets_superseded_design");
        } else if run.design_sha256 != design.sha256
            || !valid_sha256(&run.design_sha256)
            || !valid_sha256(&run.result_sha256)
        {
            report.push("simulation", &run.run_id, Disposition::Rejected, "simulation_design_or_result_digest_mismatch");
        } else if run.completed_at < design.created_at
            || !nonempty(&run.solver_id)
            || !nonempty(&run.solver_version)
        {
            report.push("simulation", &run.run_id, Disposition::Rejected, "invalid_simulation_provenance");
        } else {
            let disposition = simulation_artifact_disposition(run);
            report.push(
                "simulation",
                &run.run_id,
                disposition,
                match disposition {
                    Disposition::Accepted => "exact_design_and_external_run_binding_valid",
                    Disposition::Rejected => "external_simulation_run_binding_rejected",
                    Disposition::Indeterminate => "external_simulation_run_reference_missing",
                    Disposition::Disputed => "external_simulation_run_reference_disputed",
                    Disposition::Superseded => "simulation_run_superseded",
                },
            );
        }
    }

    let mut approval_ids = BTreeSet::new();
    let mut duplicate_approval_ids = BTreeSet::new();
    for approval in &trace.approvals {
        if !approval_ids.insert(approval.approval_id.clone()) {
            duplicate_approval_ids.insert(approval.approval_id.clone());
        }
    }

    // Supersession is only effective when a unique, later, fully accepted
    // successor explicitly names a unique, previously accepted predecessor
    // on the same design revision. A rejected or unknown successor cannot
    // erase a valid earlier approval.
    let mut superseded_approvals = BTreeSet::new();
    for successor in &trace.approvals {
        if duplicate_approval_ids.contains(&successor.approval_id)
            || trace
                .approvals
                .iter()
                .filter(|item| item.approval_id == successor.approval_id)
                .count()
                != 1
        {
            continue;
        }
        let Some(predecessor_id) = successor.supersedes_approval_id.as_deref() else {
            continue;
        };
        let predecessors = trace
            .approvals
            .iter()
            .filter(|item| item.approval_id == predecessor_id)
            .collect::<Vec<_>>();
        let Some(predecessor) = predecessors.first().copied() else {
            continue;
        };
        if predecessors.len() == 1
            && predecessor.design_revision_id == successor.design_revision_id
            && predecessor.decided_at < successor.decided_at
            && approval_disposition(trace, predecessor, false) == Disposition::Accepted
            && approval_disposition(trace, successor, false) == Disposition::Accepted
        {
            superseded_approvals.insert(predecessor_id.to_owned());
        }
    }

    for approval in &trace.approvals {
        if duplicate_approval_ids.contains(&approval.approval_id) {
            report.push("approval", &approval.approval_id, Disposition::Rejected, "duplicate_approval_id");
            continue;
        }
        let disposition = approval_disposition(
            trace,
            approval,
            superseded_approvals.contains(&approval.approval_id),
        );
        let code = match disposition {
            Disposition::Accepted => "approval_bindings_valid",
            Disposition::Rejected => "approval_binding_rejected",
            Disposition::Indeterminate => "approval_evidence_or_authority_incomplete",
            Disposition::Disputed => "approval_depends_on_disputed_evidence",
            Disposition::Superseded => "approval_is_not_current",
        };
        report.push("approval", &approval.approval_id, disposition, code);
    }

    let mut batch_ids = BTreeSet::new();
    let mut duplicate_batch_ids = BTreeSet::new();
    for batch in &trace.material_batches {
        if !batch_ids.insert(batch.batch_id.clone()) {
            duplicate_batch_ids.insert(batch.batch_id.clone());
        }
    }
    for batch in &trace.material_batches {
        if duplicate_batch_ids.contains(&batch.batch_id) {
            report.push("material_batch", &batch.batch_id, Disposition::Rejected, "duplicate_batch_id");
            continue;
        }
        let disposition = material_disposition(trace, batch);
        let code = match disposition {
            Disposition::Accepted => "certification_reference_and_supplier_assertion_separated",
            Disposition::Rejected => "material_evidence_rejected",
            Disposition::Indeterminate => "material_certification_missing_or_unknown",
            Disposition::Disputed => "material_certification_disputed",
            Disposition::Superseded => "material_record_superseded",
        };
        report.push("material_batch", &batch.batch_id, disposition, code);
    }

    let mut custody_ids = BTreeSet::new();
    for receipt in &trace.custody_receipts {
        if !custody_ids.insert(receipt.receipt_id.clone()) {
            report.push("custody_receipt", &receipt.receipt_id, Disposition::Rejected, "duplicate_custody_receipt_id");
        }
    }
    for batch in &trace.material_batches {
        let disposition = custody_disposition(trace, batch);
        let code = match disposition {
            Disposition::Accepted => "custody_chain_contiguous_to_receiving_site",
            Disposition::Rejected => "custody_chain_discontinuous_or_malformed",
            Disposition::Indeterminate => "custody_chain_incomplete_or_evidence_unknown",
            Disposition::Disputed => "custody_chain_evidence_disputed",
            Disposition::Superseded => "custody_chain_superseded",
        };
        report.push("custody", &batch.batch_id, disposition, code);
    }

    let mut inspection_ids = BTreeSet::new();
    let mut duplicate_inspection_ids = BTreeSet::new();
    for inspection in &trace.inspections {
        if !inspection_ids.insert(inspection.inspection_id.clone()) {
            duplicate_inspection_ids.insert(inspection.inspection_id.clone());
        }
    }
    for inspection in &trace.inspections {
        if duplicate_inspection_ids.contains(&inspection.inspection_id) {
            report.push("inspection", &inspection.inspection_id, Disposition::Rejected, "duplicate_inspection_id");
            continue;
        }
        let disposition = inspection_disposition(trace, inspection, &report);
        let code = match disposition {
            Disposition::Accepted => "inspection_record_bindings_valid",
            Disposition::Rejected => "inspection_record_rejected",
            Disposition::Indeterminate => "inspection_record_incomplete",
            Disposition::Disputed => "inspection_record_disputed",
            Disposition::Superseded => "inspection_targets_superseded_design",
        };
        report.push("inspection", &inspection.inspection_id, disposition, code);
    }

    report.findings.sort_by(|left, right| {
        (&left.subject_type, &left.subject_id).cmp(&(&right.subject_type, &right.subject_id))
    });
    report
}

fn inspection_disposition(
    trace: &ConstructionProjectTrace,
    inspection: &InspectionReceipt,
    report: &AuditReport,
) -> Disposition {
    if inspection.design_revision_id != trace.current_design_revision_id {
        return Disposition::Superseded;
    }
    let Some(design) = design_for(trace, &inspection.design_revision_id) else {
        return Disposition::Indeterminate;
    };
    if inspection.design_sha256 != design.sha256 || !valid_sha256(&inspection.design_sha256) {
        return Disposition::Rejected;
    }
    let Some(batch) = trace
        .material_batches
        .iter()
        .find(|batch| batch.batch_id == inspection.batch_id)
    else {
        return Disposition::Indeterminate;
    };
    let terminal_matches = trace
        .custody_receipts
        .iter()
        .filter(|receipt| receipt.receipt_id == inspection.terminal_custody_receipt_id)
        .collect::<Vec<_>>();
    let Some(terminal_receipt) = terminal_matches.first().copied() else {
        return Disposition::Indeterminate;
    };
    if terminal_matches.len() != 1 {
        return Disposition::Rejected;
    }
    if terminal_receipt.batch_id != batch.batch_id
        || terminal_receipt.to_party_id != batch.receiving_site_party_id
    {
        return Disposition::Rejected;
    }
    let terminal_for_batch = trace
        .custody_receipts
        .iter()
        .filter(|receipt| receipt.batch_id == batch.batch_id)
        .max_by(|left, right| {
            (left.occurred_at, left.receipt_id.as_str())
                .cmp(&(right.occurred_at, right.receipt_id.as_str()))
        });
    if report.disposition_for("custody", &batch.batch_id) != Some(Disposition::Accepted)
        || report.disposition_for("material_batch", &batch.batch_id) != Some(Disposition::Accepted)
        || terminal_receipt.occurred_at > inspection.inspected_at
        || terminal_for_batch.is_none_or(|receipt| receipt.receipt_id != terminal_receipt.receipt_id)
    {
        return Disposition::Indeterminate;
    }
    let approved = trace.approvals.iter().any(|approval| {
        approval.design_revision_id == trace.current_design_revision_id
            && approval.decided_at <= inspection.inspected_at
            && report.disposition_for("approval", &approval.approval_id)
                == Some(Disposition::Accepted)
    });
    if !approved {
        return Disposition::Indeterminate;
    }

    let mut result = authority_disposition(
        trace,
        &inspection.authority_ref,
        &inspection.inspector_id,
        &inspection.role,
        &inspection.scope_ref,
        inspection.inspected_at,
    );
    if inspection.evidence_refs.is_empty() {
        result = combine(result, Disposition::Indeterminate);
    }
    for evidence_ref in &inspection.evidence_refs {
        result = combine(
            result,
            evidence_disposition(trace, evidence_ref, &inspection.inspection_id, inspection.inspected_at),
        );
    }
    match inspection.outcome {
        InspectionOutcome::Conforming => result,
        InspectionOutcome::Nonconforming => Disposition::Rejected,
        InspectionOutcome::Inconclusive => combine(result, Disposition::Indeterminate),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D1: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const D2: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const D3: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn authority(reference: &str, signer: &str, role: &str, scope: &str) -> AuthorityAssertion {
        AuthorityAssertion {
            reference: reference.into(),
            signer_id: signer.into(),
            role: role.into(),
            scope_ref: scope.into(),
            status: AuthorityStatus::Active,
            available_at: 90,
            valid_from: 90,
            valid_until: Some(500),
        }
    }

    fn evidence(reference: &str, digest: &str) -> EvidenceAssertion {
        EvidenceAssertion {
            reference: reference.into(),
            subject_ref: match reference {
                "evidence:design-review-r1" => "scope:design-r1",
                "evidence:design-review-r2" => "scope:design-r2",
                "evidence:batch-cert-01" => "claim:batch-01:recycled-content",
                "evidence:custody-01-a" => "custody-01-a",
                "evidence:custody-01-b" => "custody-01-b",
                "evidence:inspection-01" => "inspection-01",
                _ => "unknown",
            }.into(),
            sha256: digest.into(),
            status: EvidenceStatus::Verified,
            use_permission: UsePermission::Permitted,
            available_at: 90,
            valid_from: 90,
            valid_until: Some(500),
        }
    }

    fn sample_trace() -> ConstructionProjectTrace {
        ConstructionProjectTrace {
            schema_version: 1,
            project_id: "pavilion-demo-01".into(),
            current_design_revision_id: "design-r2".into(),
            site_context: Some(ExternalArtifactRef {
                reference: "sol-atlas:site:pavilion-01".into(),
                owner_system: "sol-atlas-leptos".into(),
                artifact_role: "site_context".into(),
                revision_ref: "site-r1".into(),
                sha256: D1.into(),
            }),
            external_portfolio_context: Some(ExternalArtifactRef {
                reference: "sol-atlas:foundry:study-01:alternative-01".into(),
                owner_system: "sol-atlas-leptos".into(),
                artifact_role: "portfolio_alternative".into(),
                revision_ref: "alternative-r1".into(),
                sha256: D3.into(),
            }),
            design_revisions: vec![
                DesignRevision {
                    revision_id: "design-r1".into(),
                    source_artifact: ExternalArtifactRef {
                        reference: "symthaea:design:pavilion-r1".into(),
                        owner_system: "symthaea".into(),
                        artifact_role: "design_revision".into(),
                        revision_ref: "design-r1".into(),
                        sha256: D1.into(),
                    },
                    ifc_model_export: ExternalArtifactRef {
                        reference: "symthaea:ifc:pavilion-r1".into(),
                        owner_system: "symthaea".into(),
                        artifact_role: "ifc_model_export".into(),
                        revision_ref: "design-r1".into(),
                        sha256: D2.into(),
                    },
                    drawing_package: ExternalArtifactRef {
                        reference: "symthaea:drawings:pavilion-r1".into(),
                        owner_system: "symthaea".into(),
                        artifact_role: "drawing_package".into(),
                        revision_ref: "design-r1".into(),
                        sha256: D3.into(),
                    },
                    sha256: D1.into(),
                    supersedes_revision_id: None,
                    created_at: 100,
                },
                DesignRevision {
                    revision_id: "design-r2".into(),
                    source_artifact: ExternalArtifactRef {
                        reference: "symthaea:design:pavilion-r2".into(),
                        owner_system: "symthaea".into(),
                        artifact_role: "design_revision".into(),
                        revision_ref: "design-r2".into(),
                        sha256: D2.into(),
                    },
                    ifc_model_export: ExternalArtifactRef {
                        reference: "symthaea:ifc:pavilion-r2".into(),
                        owner_system: "symthaea".into(),
                        artifact_role: "ifc_model_export".into(),
                        revision_ref: "design-r2".into(),
                        sha256: D1.into(),
                    },
                    drawing_package: ExternalArtifactRef {
                        reference: "symthaea:drawings:pavilion-r2".into(),
                        owner_system: "symthaea".into(),
                        artifact_role: "drawing_package".into(),
                        revision_ref: "design-r2".into(),
                        sha256: D3.into(),
                    },
                    sha256: D2.into(),
                    supersedes_revision_id: Some("design-r1".into()),
                    created_at: 180,
                },
            ],
            simulation_runs: vec![
                SimulationRun {
                    run_id: "sim-r1".into(),
                    execution_artifact: ExternalArtifactRef {
                        reference: "symtropy:run:sim-r1".into(),
                        owner_system: "symtropy".into(),
                        artifact_role: "simulation_run".into(),
                        revision_ref: "sim-r1".into(),
                        sha256: D3.into(),
                    },
                    design_revision_id: "design-r1".into(),
                    design_sha256: D1.into(),
                    solver_id: "symtropy-structural-adapter".into(),
                    solver_version: "0.1.0".into(),
                    result_sha256: D3.into(),
                    completed_at: 130,
                },
                SimulationRun {
                    run_id: "sim-r2".into(),
                    execution_artifact: ExternalArtifactRef {
                        reference: "symtropy:run:sim-r2".into(),
                        owner_system: "symtropy".into(),
                        artifact_role: "simulation_run".into(),
                        revision_ref: "sim-r2".into(),
                        sha256: D3.into(),
                    },
                    design_revision_id: "design-r2".into(),
                    design_sha256: D2.into(),
                    solver_id: "qualified-external-solver-ref".into(),
                    solver_version: "vendor-build-7".into(),
                    result_sha256: D3.into(),
                    completed_at: 220,
                },
            ],
            authority_assertions: vec![
                authority("authority:design-r1", "party:engineer-01", "design-approver", "scope:design-r1"),
                authority("authority:design-r2", "party:engineer-01", "design-approver", "scope:design-r2"),
                authority("authority:inspection", "party:inspector-01", "independent-inspector", "scope:inspection:batch-01:r2"),
            ],
            evidence_assertions: vec![
                evidence("evidence:design-review-r1", D1),
                evidence("evidence:design-review-r2", D2),
                evidence("evidence:batch-cert-01", D3),
                evidence("evidence:custody-01-a", D1),
                evidence("evidence:custody-01-b", D2),
                evidence("evidence:inspection-01", D3),
            ],
            approvals: vec![
                ApprovalReceipt {
                    approval_id: "approval-r1".into(),
                    design_revision_id: "design-r1".into(),
                    design_sha256: D1.into(),
                    simulation_run_id: "sim-r1".into(),
                    simulation_result_sha256: D3.into(),
                    authority_ref: "authority:design-r1".into(),
                    signer_id: "party:engineer-01".into(),
                    role: "design-approver".into(),
                    scope_ref: "scope:design-r1".into(),
                    supersedes_approval_id: None,
                    decision: ApprovalDecision::Approved,
                    conditions: vec![],
                    evidence_refs: vec!["evidence:design-review-r1".into()],
                    dissent_refs: vec![],
                    decided_at: 150,
                },
                ApprovalReceipt {
                    approval_id: "approval-r2".into(),
                    design_revision_id: "design-r2".into(),
                    design_sha256: D2.into(),
                    simulation_run_id: "sim-r2".into(),
                    simulation_result_sha256: D3.into(),
                    authority_ref: "authority:design-r2".into(),
                    signer_id: "party:engineer-01".into(),
                    role: "design-approver".into(),
                    scope_ref: "scope:design-r2".into(),
                    supersedes_approval_id: Some("approval-r1".into()),
                    decision: ApprovalDecision::Approved,
                    conditions: vec![ApprovalCondition {
                        condition_id: "condition:drawings-final".into(),
                        satisfied: true,
                    }],
                    evidence_refs: vec!["evidence:design-review-r2".into()],
                    dissent_refs: vec!["dissent:reviewer-02".into()],
                    decided_at: 240,
                },
            ],
            material_batches: vec![MaterialBatch {
                batch_id: "batch-01".into(),
                supplier_party_id: "party:supplier-01".into(),
                receiving_site_party_id: "site:pavilion-01".into(),
                material_description: "structural steel members".into(),
                supplier_assertion: Some("Supplier declares recycled content".into()),
                certification_evidence_ref: Some("evidence:batch-cert-01".into()),
                certification_scope_ref: Some("claim:batch-01:recycled-content".into()),
                declared_at: 230,
            }],
            custody_receipts: vec![
                CustodyReceipt {
                    receipt_id: "custody-01-a".into(),
                    batch_id: "batch-01".into(),
                    from_party_id: "party:supplier-01".into(),
                    to_party_id: "party:carrier-01".into(),
                    predecessor_receipt_id: None,
                    evidence_ref: "evidence:custody-01-a".into(),
                    occurred_at: 250,
                },
                CustodyReceipt {
                    receipt_id: "custody-01-b".into(),
                    batch_id: "batch-01".into(),
                    from_party_id: "party:carrier-01".into(),
                    to_party_id: "site:pavilion-01".into(),
                    predecessor_receipt_id: Some("custody-01-a".into()),
                    evidence_ref: "evidence:custody-01-b".into(),
                    occurred_at: 270,
                },
            ],
            inspections: vec![InspectionReceipt {
                inspection_id: "inspection-01".into(),
                batch_id: "batch-01".into(),
                design_revision_id: "design-r2".into(),
                design_sha256: D2.into(),
                terminal_custody_receipt_id: "custody-01-b".into(),
                authority_ref: "authority:inspection".into(),
                inspector_id: "party:inspector-01".into(),
                role: "independent-inspector".into(),
                scope_ref: "scope:inspection:batch-01:r2".into(),
                evidence_refs: vec!["evidence:inspection-01".into()],
                outcome: InspectionOutcome::Conforming,
                inspected_at: 300,
            }],
        }
    }

    fn disposition(
        trace: &ConstructionProjectTrace,
        subject_type: &str,
        subject_id: &str,
    ) -> Disposition {
        verify_trace(trace)
            .disposition_for(subject_type, subject_id)
            .expect("finding must exist")
    }

    #[test]
    fn valid_trace_keeps_supersession_and_current_records_distinct() {
        let trace = sample_trace();
        let report = verify_trace(&trace);
        assert_eq!(report.disposition_for("design", "design-r1"), Some(Disposition::Superseded));
        assert_eq!(report.disposition_for("design", "design-r2"), Some(Disposition::Accepted));
        assert_eq!(report.disposition_for("approval", "approval-r1"), Some(Disposition::Superseded));
        assert_eq!(report.disposition_for("approval", "approval-r2"), Some(Disposition::Accepted));
        assert_eq!(report.disposition_for("material_batch", "batch-01"), Some(Disposition::Accepted));
        assert_eq!(report.disposition_for("custody", "batch-01"), Some(Disposition::Accepted));
        assert_eq!(report.disposition_for("inspection", "inspection-01"), Some(Disposition::Accepted));
    }

    #[test]
    fn design_and_simulation_must_bind_to_external_owner_artifacts() {
        let mut trace = sample_trace();
        trace.design_revisions[1].source_artifact.sha256 = D1.into();
        assert_eq!(disposition(&trace, "design", "design-r2"), Disposition::Rejected);

        let mut trace = sample_trace();
        trace.simulation_runs[1].execution_artifact.revision_ref = "sim-r1".into();
        assert_eq!(disposition(&trace, "simulation", "sim-r2"), Disposition::Rejected);
    }

    #[test]
    fn portfolio_context_is_an_opaque_sol_atlas_reference_not_a_local_gis_record() {
        let mut trace = sample_trace();
        trace.external_portfolio_context.as_mut().unwrap().owner_system = "symtropy".into();
        assert_eq!(
            disposition(&trace, "external_context", "sol-atlas:foundry:study-01:alternative-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn ifc_and_drawing_package_refs_must_match_the_design_revision() {
        let mut trace = sample_trace();
        trace.design_revisions[1].ifc_model_export.artifact_role = "simulation_run".into();
        assert_eq!(disposition(&trace, "design", "design-r2"), Disposition::Rejected);

        let mut trace = sample_trace();
        trace.design_revisions[1].drawing_package.revision_ref = "design-r1".into();
        assert_eq!(disposition(&trace, "design", "design-r2"), Disposition::Rejected);
    }

    #[test]
    fn broken_ancestor_revision_link_invalidates_current_design() {
        let mut trace = sample_trace();
        trace.design_revisions[0].supersedes_revision_id = Some("missing-parent".into());
        assert_eq!(
            disposition(&trace, "design", "design-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn rejected_successor_cannot_supersede_a_valid_prior_approval() {
        let mut trace = sample_trace();
        let mut prior = trace.approvals[1].clone();
        prior.approval_id = "approval-prior-r2".into();
        prior.supersedes_approval_id = None;
        prior.decided_at = 230;
        trace.approvals.push(prior);
        trace.approvals[1].supersedes_approval_id = Some("approval-prior-r2".into());
        trace.approvals[1].signer_id = "party:unauthorized".into();

        assert_eq!(
            disposition(&trace, "approval", "approval-prior-r2"),
            Disposition::Accepted
        );
        assert_eq!(
            disposition(&trace, "approval", "approval-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn custody_receipt_identity_is_unique_across_all_batches() {
        let mut trace = sample_trace();
        trace.material_batches.push(MaterialBatch {
            batch_id: "batch-02".into(),
            supplier_party_id: "party:supplier-02".into(),
            receiving_site_party_id: "site:pavilion-01".into(),
            material_description: "structural steel members".into(),
            supplier_assertion: Some("Supplier declares recycled content".into()),
            certification_evidence_ref: Some("evidence:batch-cert-01".into()),
            certification_scope_ref: Some("claim:batch-01:recycled-content".into()),
            declared_at: 230,
        });
        trace.custody_receipts.push(CustodyReceipt {
            receipt_id: "custody-01-b".into(),
            batch_id: "batch-02".into(),
            from_party_id: "party:supplier-02".into(),
            to_party_id: "site:pavilion-01".into(),
            predecessor_receipt_id: None,
            evidence_ref: "evidence:custody-01-b".into(),
            occurred_at: 280,
        });
        assert_eq!(
            disposition(&trace, "custody", "batch-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn approval_requires_the_exact_design_ifc_and_drawing_references() {
        let mut trace = sample_trace();
        trace.design_revisions[1].ifc_model_export.artifact_role = "simulation_run".into();
        assert_eq!(
            disposition(&trace, "approval", "approval-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn approval_requires_the_exact_external_simulation_run_reference() {
        let mut trace = sample_trace();
        trace.simulation_runs[1].execution_artifact.owner_system = "unknown-engine".into();
        assert_eq!(
            disposition(&trace, "approval", "approval-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn approval_without_resolved_authority_is_indeterminate() {
        let mut trace = sample_trace();
        trace.authority_assertions.clear();
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Indeterminate);
    }

    #[test]
    fn mismatched_signer_or_scope_is_rejected() {
        let mut trace = sample_trace();
        trace.approvals[1].signer_id = "party:other-engineer".into();
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Rejected);
    }

    #[test]
    fn approval_cannot_bind_a_different_design_or_simulation_digest() {
        let mut trace = sample_trace();
        trace.approvals[1].design_sha256 = D1.into();
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Rejected);

        let mut trace = sample_trace();
        trace.approvals[1].simulation_result_sha256 = D1.into();
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Rejected);
    }

    #[test]
    fn revoked_material_certification_is_rejected() {
        let mut trace = sample_trace();
        trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:batch-cert-01")
            .unwrap()
            .status = EvidenceStatus::Revoked;
        assert_eq!(disposition(&trace, "material_batch", "batch-01"), Disposition::Rejected);
    }

    #[test]
    fn duplicate_batch_identity_is_rejected() {
        let mut trace = sample_trace();
        trace.material_batches.push(trace.material_batches[0].clone());
        assert_eq!(disposition(&trace, "material_batch", "batch-01"), Disposition::Rejected);
    }

    #[test]
    fn certification_must_be_bound_to_the_exact_supplier_claim_scope() {
        let mut trace = sample_trace();
        trace.material_batches[0].certification_scope_ref =
            Some("claim:another-batch:recycled-content".into());
        assert_eq!(disposition(&trace, "material_batch", "batch-01"), Disposition::Rejected);
    }

    #[test]
    fn stale_or_post_cutoff_evidence_is_indeterminate() {
        let mut trace = sample_trace();
        let evidence = trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:design-review-r2")
            .unwrap();
        evidence.available_at = 250;
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Indeterminate);

        let mut trace = sample_trace();
        let evidence = trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:batch-cert-01")
            .unwrap();
        evidence.valid_until = Some(200);
        assert_eq!(disposition(&trace, "material_batch", "batch-01"), Disposition::Indeterminate);
    }

    #[test]
    fn broken_custody_predecessor_is_rejected() {
        let mut trace = sample_trace();
        trace.custody_receipts[1].predecessor_receipt_id = Some("custody:missing".into());
        assert_eq!(disposition(&trace, "custody", "batch-01"), Disposition::Rejected);
    }

    #[test]
    fn approval_with_open_condition_is_not_accepted() {
        let mut trace = sample_trace();
        trace.approvals[1].conditions[0].satisfied = false;
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Indeterminate);
    }

    #[test]
    fn inspection_cannot_accept_a_superseded_design_or_wrong_batch() {
        let mut trace = sample_trace();
        trace.inspections[0].design_revision_id = "design-r1".into();
        assert_eq!(disposition(&trace, "inspection", "inspection-01"), Disposition::Superseded);

        let mut trace = sample_trace();
        trace.inspections[0].batch_id = "batch-other".into();
        assert_eq!(disposition(&trace, "inspection", "inspection-01"), Disposition::Indeterminate);
    }

    #[test]
    fn nonconforming_inspection_is_rejected() {
        let mut trace = sample_trace();
        trace.inspections[0].outcome = InspectionOutcome::Nonconforming;
        assert_eq!(disposition(&trace, "inspection", "inspection-01"), Disposition::Rejected);
    }

    #[test]
    fn future_simulation_cannot_support_past_approval() {
        let mut trace = sample_trace();
        trace.simulation_runs[1].completed_at = 250;
        assert_eq!(disposition(&trace, "approval", "approval-r2"), Disposition::Rejected);
    }
}
