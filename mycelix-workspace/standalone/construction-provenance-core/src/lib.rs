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
    pub sha256: String,
    pub supersedes_revision_id: Option<String>,
    pub created_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimulationRun {
    pub run_id: String,
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
    if trace.simulation_runs.iter().filter(|item| item.run_id == approval.simulation_run_id).count() != 1 {
        return Disposition::Rejected;
    }
    let Some(simulation) = simulation_for(trace, &approval.simulation_run_id) else {
        return Disposition::Indeterminate;
    };
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
            let lineage_valid = !has_child
                && match revision.supersedes_revision_id.as_deref() {
                    None => true,
                    Some(parent_id) => design_for(trace, parent_id)
                        .is_some_and(|parent| parent.created_at < revision.created_at),
                };
            report.push(
                "design",
                &revision.revision_id,
                if lineage_valid { Disposition::Accepted } else { Disposition::Rejected },
                if lineage_valid { "current_revision_identity_valid" } else { "invalid_or_nonterminal_revision_lineage" },
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
            report.push("simulation", &run.run_id, Disposition::Accepted, "exact_design_binding_valid");
        }
    }

    let superseded_approvals = trace
        .approvals
        .iter()
        .filter_map(|approval| approval.supersedes_approval_id.clone())
        .collect::<BTreeSet<_>>();
    let mut approval_ids = BTreeSet::new();
    let mut duplicate_approval_ids = BTreeSet::new();
    for approval in &trace.approvals {
        if !approval_ids.insert(approval.approval_id.clone()) {
            duplicate_approval_ids.insert(approval.approval_id.clone());
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
            design_revisions: vec![
                DesignRevision {
                    revision_id: "design-r1".into(),
                    sha256: D1.into(),
                    supersedes_revision_id: None,
                    created_at: 100,
                },
                DesignRevision {
                    revision_id: "design-r2".into(),
                    sha256: D2.into(),
                    supersedes_revision_id: Some("design-r1".into()),
                    created_at: 180,
                },
            ],
            simulation_runs: vec![
                SimulationRun {
                    run_id: "sim-r1".into(),
                    design_revision_id: "design-r1".into(),
                    design_sha256: D1.into(),
                    solver_id: "symtropy-structural-adapter".into(),
                    solver_version: "0.1.0".into(),
                    result_sha256: D3.into(),
                    completed_at: 130,
                },
                SimulationRun {
                    run_id: "sim-r2".into(),
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
