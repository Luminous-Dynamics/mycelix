//! Mycelix participant authority and material-custody trace checks.
//!
//! This crate records cross-party attestations and supply-chain custody while
//! referring to engineering artifacts owned by their native systems. It does
//! not reproduce the Sol-Atlas built-environment artifact graph or decide
//! engineering adequacy. "Accepted" means only that this Mycelix attestation
//! and its supplied authority/evidence bindings pass this narrow contract.

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
pub enum ParticipantDecision {
    Affirmed,
    Objected,
    Deferred,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InspectionClaim {
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

/// Opaque pointer to a record whose content and semantic validation remain
/// owned by the named source system. The digest is the source-declared digest;
/// this crate does not fetch the bytes or independently recompute it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExternalArtifactRef {
    pub reference: String,
    pub owner_system: String,
    pub artifact_role: String,
    pub revision_ref: String,
    pub sha256: String,
    /// Optional exact parent/subject artifact in the source-owned artifact graph.
    pub subject_ref: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EvidenceAssertion {
    pub reference: String,
    /// Exact approval, claim, custody receipt, manufacturing receipt or
    /// inspection receipt that this evidence is permitted to support.
    pub subject_ref: String,
    pub sha256: String,
    pub status: EvidenceStatus,
    pub use_permission: UsePermission,
    /// First time this relying party could use the evidence; not source creation time.
    pub available_at: u64,
    pub valid_from: u64,
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
    pub available_at: u64,
    pub valid_from: u64,
    pub valid_until: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ApprovalCondition {
    pub condition_id: String,
    pub satisfied: bool,
}

/// A participant's scoped commitment about an exact source-owned artifact.
/// This is an authority/evidence receipt, not an engineering approval or
/// authorization to fabricate, install, energize or operate an asset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParticipantApprovalReceipt {
    pub approval_id: String,
    pub subject_artifact_ref: String,
    /// Opaque simulation/engineering-review artifact references. Their
    /// physical and engineering semantics remain with the owning systems.
    pub supporting_artifact_refs: Vec<String>,
    pub authority_ref: String,
    pub signer_id: String,
    pub role: String,
    pub scope_ref: String,
    pub supersedes_approval_id: Option<String>,
    pub decision: ParticipantDecision,
    pub conditions: Vec<ApprovalCondition>,
    pub evidence_refs: Vec<String>,
    /// Dissent is retained; it is not silently converted into consent.
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
    /// Must resolve to independently attested evidence for this exact claim.
    pub certification_evidence_ref: Option<String>,
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

/// Manufacturer attestation joins a Mycelix material batch/custody record to
/// source-owned manufacturing and fabricated-component artifacts. It does
/// not certify the manufacturing process or machine capability.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ManufacturingReceipt {
    pub manufacturing_id: String,
    pub design_artifact_ref: String,
    pub material_batch_id: String,
    pub manufacturing_artifact_ref: String,
    pub component_artifact_ref: String,
    pub authority_ref: String,
    pub manufacturer_id: String,
    pub role: String,
    pub scope_ref: String,
    pub evidence_refs: Vec<String>,
    pub manufactured_at: u64,
}

/// Inspector attestation binds the manufactured component to source-owned
/// inspection/metrology, installation and as-built artifacts. The claimed
/// outcome is recorded as an attestation; professional sufficiency is not
/// evaluated here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InspectionReceipt {
    pub inspection_id: String,
    pub manufacturing_id: String,
    pub manufacturing_artifact_ref: String,
    pub component_artifact_ref: String,
    pub inspection_artifact_ref: String,
    pub installation_artifact_ref: String,
    pub as_built_artifact_ref: String,
    pub authority_ref: String,
    pub inspector_id: String,
    pub role: String,
    pub scope_ref: String,
    pub evidence_refs: Vec<String>,
    pub claimed_outcome: InspectionClaim,
    pub inspected_at: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConstructionProjectTrace {
    pub schema_version: u32,
    pub project_id: String,
    /// Points to the current design revision owned by Symthaea/Sol-Atlas or
    /// another named design authority. Revision history stays at that owner.
    pub current_design_artifact_ref: String,
    pub site_context_artifact_ref: Option<String>,
    pub portfolio_alternative_artifact_ref: Option<String>,
    pub artifact_references: Vec<ExternalArtifactRef>,
    pub authority_assertions: Vec<AuthorityAssertion>,
    pub evidence_assertions: Vec<EvidenceAssertion>,
    pub approvals: Vec<ParticipantApprovalReceipt>,
    pub material_batches: Vec<MaterialBatch>,
    pub custody_receipts: Vec<CustodyReceipt>,
    pub manufacturing_receipts: Vec<ManufacturingReceipt>,
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
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
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

fn artifact_disposition(
    trace: &ConstructionProjectTrace,
    reference: &str,
    expected_owner: Option<&str>,
    expected_role: Option<&str>,
) -> Disposition {
    if !nonempty(reference) {
        return Disposition::Indeterminate;
    }
    let matching = trace
        .artifact_references
        .iter()
        .filter(|artifact| artifact.reference == reference)
        .collect::<Vec<_>>();
    let Some(artifact) = matching.first().copied() else {
        return Disposition::Indeterminate;
    };
    if matching.len() != 1 {
        return Disposition::Rejected;
    }
    if !nonempty(&artifact.owner_system)
        || !nonempty(&artifact.artifact_role)
        || !nonempty(&artifact.revision_ref)
        || !valid_sha256(&artifact.sha256)
        || expected_owner.is_some_and(|owner| artifact.owner_system != owner)
        || expected_role.is_some_and(|role| artifact.artifact_role != role)
    {
        return Disposition::Rejected;
    }
    Disposition::Accepted
}

fn artifact_subject_disposition(
    trace: &ConstructionProjectTrace,
    reference: &str,
    expected_subject: &str,
) -> Disposition {
    let matching = trace
        .artifact_references
        .iter()
        .filter(|artifact| artifact.reference == reference)
        .collect::<Vec<_>>();
    let Some(artifact) = matching.first().copied() else {
        return Disposition::Indeterminate;
    };
    if matching.len() != 1 {
        return Disposition::Rejected;
    }
    if artifact.subject_ref.as_deref() == Some(expected_subject) {
        Disposition::Accepted
    } else {
        Disposition::Rejected
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
    let matching = trace
        .evidence_assertions
        .iter()
        .filter(|item| item.reference == reference)
        .collect::<Vec<_>>();
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
    if !nonempty(reference)
        || !nonempty(signer_id)
        || !nonempty(role)
        || !nonempty(scope_ref)
    {
        return Disposition::Indeterminate;
    }
    let matching = trace
        .authority_assertions
        .iter()
        .filter(|item| item.reference == reference)
        .collect::<Vec<_>>();
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

fn unique_material_batch<'a>(
    trace: &'a ConstructionProjectTrace,
    batch_id: &str,
) -> Option<&'a MaterialBatch> {
    let matches = trace
        .material_batches
        .iter()
        .filter(|batch| batch.batch_id == batch_id)
        .collect::<Vec<_>>();
    if matches.len() == 1 {
        matches.first().copied()
    } else {
        None
    }
}

fn material_disposition_at(
    trace: &ConstructionProjectTrace,
    batch: &MaterialBatch,
    event_at: u64,
) -> Disposition {
    if !nonempty(&batch.batch_id)
        || !nonempty(&batch.supplier_party_id)
        || !nonempty(&batch.receiving_site_party_id)
        || !nonempty(&batch.material_description)
    {
        return Disposition::Rejected;
    }
    if event_at < batch.declared_at {
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
    evidence_disposition(trace, reference, scope_ref, event_at)
}

fn custody_disposition(
    trace: &ConstructionProjectTrace,
    batch: &MaterialBatch,
) -> Disposition {
    let receipts = trace
        .custody_receipts
        .iter()
        .filter(|receipt| receipt.batch_id == batch.batch_id)
        .collect::<Vec<_>>();
    if receipts.is_empty() {
        return Disposition::Indeterminate;
    }

    // Follow the explicit predecessor graph instead of sorting timestamps.
    // Equal event timestamps are not sufficient to infer transfer order.
    let roots = receipts
        .iter()
        .copied()
        .filter(|receipt| receipt.predecessor_receipt_id.is_none())
        .collect::<Vec<_>>();
    let Some(mut current) = roots.first().copied() else {
        return Disposition::Rejected;
    };
    if roots.len() != 1 {
        return Disposition::Rejected;
    }

    let mut result = Disposition::Accepted;
    let mut visited = BTreeSet::new();
    let mut previous_to: Option<&str> = None;
    let mut previous_time = 0_u64;

    loop {
        if !nonempty(&current.receipt_id)
            || !nonempty(&current.from_party_id)
            || !nonempty(&current.to_party_id)
            || !nonempty(&current.evidence_ref)
        {
            return Disposition::Rejected;
        }
        if trace
            .custody_receipts
            .iter()
            .filter(|item| item.receipt_id == current.receipt_id)
            .count()
            != 1
            || !visited.insert(current.receipt_id.as_str())
            || current.occurred_at < batch.declared_at
            || current.occurred_at < previous_time
        {
            return Disposition::Rejected;
        }

        if previous_to.is_none() {
            if current.from_party_id != batch.supplier_party_id {
                return Disposition::Rejected;
            }
        } else if previous_to != Some(current.from_party_id.as_str()) {
            return Disposition::Rejected;
        }

        result = combine(
            result,
            evidence_disposition(
                trace,
                &current.evidence_ref,
                &current.receipt_id,
                current.occurred_at,
            ),
        );

        let children = receipts
            .iter()
            .copied()
            .filter(|receipt| {
                receipt.predecessor_receipt_id.as_deref() == Some(current.receipt_id.as_str())
            })
            .collect::<Vec<_>>();
        if children.len() > 1 {
            return Disposition::Rejected;
        }

        previous_to = Some(current.to_party_id.as_str());
        previous_time = current.occurred_at;

        let Some(next) = children.first().copied() else {
            break;
        };
        current = next;
    }

    if visited.len() != receipts.len() {
        return Disposition::Rejected;
    }
    if previous_to != Some(batch.receiving_site_party_id.as_str()) {
        return Disposition::Indeterminate;
    }
    result
}

fn participant_approval_record_disposition(
    trace: &ConstructionProjectTrace,
    approval: &ParticipantApprovalReceipt,
) -> Disposition {
    if !nonempty(&approval.approval_id) || !nonempty(&approval.scope_ref) {
        return Disposition::Rejected;
    }
    let mut result = artifact_disposition(
        trace,
        &approval.subject_artifact_ref,
        None,
        Some("design_revision"),
    );
    if approval.supporting_artifact_refs.is_empty() {
        result = combine(result, Disposition::Indeterminate);
    }
    let mut support_ids = BTreeSet::new();
    let mut has_engineering_receipt = false;
    for reference in &approval.supporting_artifact_refs {
        if !support_ids.insert(reference.as_str()) {
            result = combine(result, Disposition::Rejected);
            continue;
        }
        result = combine(
            result,
            artifact_disposition(trace, reference, None, None),
        );
        result = combine(
            result,
            artifact_subject_disposition(trace, reference, &approval.subject_artifact_ref),
        );
        has_engineering_receipt |= trace
            .artifact_references
            .iter()
            .any(|artifact| {
                artifact.reference == *reference
                    && artifact.artifact_role == "engineering_receipt"
                    && artifact.subject_ref.as_deref()
                        == Some(approval.subject_artifact_ref.as_str())
            });
    }
    if !has_engineering_receipt {
        result = combine(result, Disposition::Indeterminate);
    }
    result = combine(
        result,
        authority_disposition(
            trace,
            &approval.authority_ref,
            &approval.signer_id,
            &approval.role,
            &approval.scope_ref,
            approval.decided_at,
        ),
    );
    if approval.evidence_refs.is_empty() {
        result = combine(result, Disposition::Indeterminate);
    }
    for reference in &approval.evidence_refs {
        result = combine(
            result,
            evidence_disposition(
                trace,
                reference,
                &approval.scope_ref,
                approval.decided_at,
            ),
        );
    }
    if approval.conditions.iter().any(|condition| !condition.satisfied) {
        result = combine(result, Disposition::Indeterminate);
    }
    match approval.decision {
        ParticipantDecision::Affirmed => result,
        ParticipantDecision::Objected => Disposition::Rejected,
        ParticipantDecision::Deferred => combine(result, Disposition::Indeterminate),
    }
}

fn participant_approval_disposition(
    trace: &ConstructionProjectTrace,
    approval: &ParticipantApprovalReceipt,
    superseded: bool,
) -> Disposition {
    let record = participant_approval_record_disposition(trace, approval);
    if record != Disposition::Accepted {
        return record;
    }
    if superseded || approval.subject_artifact_ref != trace.current_design_artifact_ref {
        Disposition::Superseded
    } else {
        Disposition::Accepted
    }
}

fn manufacturing_disposition(
    trace: &ConstructionProjectTrace,
    manufacturing: &ManufacturingReceipt,
    report: &AuditReport,
) -> Disposition {
    if !nonempty(&manufacturing.manufacturing_id)
        || !nonempty(&manufacturing.manufacturer_id)
        || !nonempty(&manufacturing.scope_ref)
    {
        return Disposition::Rejected;
    }
    if manufacturing.design_artifact_ref != trace.current_design_artifact_ref {
        return Disposition::Superseded;
    }
    let participant_commitment_exists = trace.approvals.iter().any(|approval| {
        approval.subject_artifact_ref == manufacturing.design_artifact_ref
            && approval.decided_at <= manufacturing.manufactured_at
            && report.disposition_for("participant_approval", &approval.approval_id)
                == Some(Disposition::Accepted)
    });
    if !participant_commitment_exists {
        return Disposition::Indeterminate;
    }
    let mut result = artifact_disposition(
        trace,
        &manufacturing.design_artifact_ref,
        None,
        Some("design_revision"),
    );
    result = combine(
        result,
        artifact_disposition(
            trace,
            &manufacturing.manufacturing_artifact_ref,
            None,
            Some("manufacturing_record"),
        ),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &manufacturing.manufacturing_artifact_ref,
            &manufacturing.design_artifact_ref,
        ),
    );
    result = combine(
        result,
        artifact_disposition(
            trace,
            &manufacturing.component_artifact_ref,
            None,
            Some("fabricated_component"),
        ),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &manufacturing.component_artifact_ref,
            &manufacturing.manufacturing_artifact_ref,
        ),
    );

    let Some(batch) = unique_material_batch(trace, &manufacturing.material_batch_id) else {
        return combine(result, Disposition::Indeterminate);
    };
    result = combine(
        result,
        report
            .disposition_for("material_batch", &batch.batch_id)
            .unwrap_or(Disposition::Indeterminate),
    );
    // Re-check certification validity at the manufacturing event itself.
    // A certificate valid at batch declaration may expire before fabrication.
    result = combine(
        result,
        material_disposition_at(trace, batch, manufacturing.manufactured_at),
    );
    result = combine(
        result,
        report
            .disposition_for("custody", &batch.batch_id)
            .unwrap_or(Disposition::Indeterminate),
    );
    let last_custody_time = trace
        .custody_receipts
        .iter()
        .filter(|receipt| receipt.batch_id == batch.batch_id)
        .map(|receipt| receipt.occurred_at)
        .max();
    if last_custody_time.is_none_or(|time| time > manufacturing.manufactured_at) {
        result = combine(result, Disposition::Indeterminate);
    }
    result = combine(
        result,
        authority_disposition(
            trace,
            &manufacturing.authority_ref,
            &manufacturing.manufacturer_id,
            &manufacturing.role,
            &manufacturing.scope_ref,
            manufacturing.manufactured_at,
        ),
    );
    if manufacturing.evidence_refs.is_empty() {
        result = combine(result, Disposition::Indeterminate);
    }
    for reference in &manufacturing.evidence_refs {
        result = combine(
            result,
            evidence_disposition(
                trace,
                reference,
                &manufacturing.scope_ref,
                manufacturing.manufactured_at,
            ),
        );
    }
    result
}

fn inspection_disposition(
    trace: &ConstructionProjectTrace,
    inspection: &InspectionReceipt,
    report: &AuditReport,
) -> Disposition {
    if !nonempty(&inspection.inspection_id) || !nonempty(&inspection.scope_ref) {
        return Disposition::Rejected;
    }
    let manufacturing_matches = trace
        .manufacturing_receipts
        .iter()
        .filter(|item| item.manufacturing_id == inspection.manufacturing_id)
        .collect::<Vec<_>>();
    let Some(manufacturing) = manufacturing_matches.first().copied() else {
        return Disposition::Indeterminate;
    };
    if manufacturing_matches.len() != 1 {
        return Disposition::Rejected;
    }
    let manufacturing_status = report
        .disposition_for("manufacturing", &manufacturing.manufacturing_id)
        .unwrap_or(Disposition::Indeterminate);
    if manufacturing_status != Disposition::Accepted {
        return manufacturing_status;
    }
    if inspection.inspected_at < manufacturing.manufactured_at {
        return Disposition::Rejected;
    }

    if inspection.manufacturing_artifact_ref != manufacturing.manufacturing_artifact_ref
        || inspection.component_artifact_ref != manufacturing.component_artifact_ref
    {
        return Disposition::Rejected;
    }
    let mut result = artifact_disposition(
        trace,
        &inspection.manufacturing_artifact_ref,
        None,
        Some("manufacturing_record"),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &inspection.manufacturing_artifact_ref,
            &manufacturing.design_artifact_ref,
        ),
    );
    result = combine(
        result,
        artifact_disposition(
            trace,
            &inspection.component_artifact_ref,
            None,
            Some("fabricated_component"),
        ),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &inspection.component_artifact_ref,
            &inspection.manufacturing_artifact_ref,
        ),
    );
    result = combine(
        result,
        artifact_disposition(
            trace,
            &inspection.inspection_artifact_ref,
            None,
            Some("inspection_record"),
        ),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &inspection.inspection_artifact_ref,
            &inspection.component_artifact_ref,
        ),
    );
    result = combine(
        result,
        artifact_disposition(
            trace,
            &inspection.installation_artifact_ref,
            None,
            Some("installation_record"),
        ),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &inspection.installation_artifact_ref,
            &inspection.component_artifact_ref,
        ),
    );
    result = combine(
        result,
        artifact_disposition(
            trace,
            &inspection.as_built_artifact_ref,
            None,
            Some("as_built_asset"),
        ),
    );
    result = combine(
        result,
        artifact_subject_disposition(
            trace,
            &inspection.as_built_artifact_ref,
            &inspection.component_artifact_ref,
        ),
    );

    if inspection.evidence_refs.is_empty() {
        result = combine(result, Disposition::Indeterminate);
    }
    for reference in &inspection.evidence_refs {
        result = combine(
            result,
            evidence_disposition(
                trace,
                reference,
                &inspection.scope_ref,
                inspection.inspected_at,
            ),
        );
    }
    result = combine(
        result,
        authority_disposition(
            trace,
            &inspection.authority_ref,
            &inspection.inspector_id,
            &inspection.role,
            &inspection.scope_ref,
            inspection.inspected_at,
        ),
    );
    match inspection.claimed_outcome {
        InspectionClaim::Conforming => result,
        InspectionClaim::Nonconforming => Disposition::Rejected,
        InspectionClaim::Inconclusive => combine(result, Disposition::Indeterminate),
    }
}

/// Evaluate identity, authority and custody receipts independently. A valid
/// participant or inspection attestation is not a Sol-Atlas engineering
/// decision, statutory permit, professional certification or operating order.
pub fn verify_trace(trace: &ConstructionProjectTrace) -> AuditReport {
    let mut report = AuditReport::default();
    if trace.schema_version != 1 || !nonempty(&trace.project_id) {
        report.push(
            "project",
            &trace.project_id,
            Disposition::Rejected,
            "invalid_project_envelope",
        );
    }

    let mut artifact_ids = BTreeSet::new();
    let mut duplicate_artifact_ids = BTreeSet::new();
    for artifact in &trace.artifact_references {
        if !artifact_ids.insert(artifact.reference.clone()) {
            duplicate_artifact_ids.insert(artifact.reference.clone());
        }
    }
    for artifact in &trace.artifact_references {
        let disposition = if duplicate_artifact_ids.contains(&artifact.reference) {
            Disposition::Rejected
        } else {
            artifact_disposition(trace, &artifact.reference, None, None)
        };
        report.push(
            "artifact_reference",
            &artifact.reference,
            disposition,
            match disposition {
                Disposition::Accepted => "opaque_source_reference_well_formed",
                Disposition::Rejected => "duplicate_or_malformed_source_reference",
                Disposition::Indeterminate => "source_reference_missing_or_incomplete",
                Disposition::Disputed => "source_reference_disputed",
                Disposition::Superseded => "source_reference_superseded",
            },
        );
    }

    let current_design = artifact_disposition(
        trace,
        &trace.current_design_artifact_ref,
        None,
        Some("design_revision"),
    );
    report.push(
        "current_design",
        &trace.current_design_artifact_ref,
        current_design,
        if current_design == Disposition::Accepted {
            "current_design_source_reference_bound"
        } else {
            "current_design_source_reference_not_qualified"
        },
    );
    if let Some(site_ref) = trace.site_context_artifact_ref.as_deref() {
        let site = artifact_disposition(
            trace,
            site_ref,
            Some("sol-atlas-leptos"),
            Some("site_context"),
        );
        report.push(
            "site_context",
            site_ref,
            site,
            "sol_atlas_site_context_reference",
        );
    }
    if let Some(portfolio_ref) = trace.portfolio_alternative_artifact_ref.as_deref() {
        let portfolio = artifact_disposition(
            trace,
            portfolio_ref,
            Some("sol-atlas-leptos"),
            Some("portfolio_alternative"),
        );
        report.push(
            "portfolio_context",
            portfolio_ref,
            portfolio,
            "sol_atlas_portfolio_reference",
        );
    }

    let mut evidence_ids = BTreeSet::new();
    let mut duplicate_evidence_ids = BTreeSet::new();
    for evidence in &trace.evidence_assertions {
        if !evidence_ids.insert(evidence.reference.clone()) {
            duplicate_evidence_ids.insert(evidence.reference.clone());
        }
    }
    for reference in duplicate_evidence_ids {
        report.push(
            "evidence_reference",
            &reference,
            Disposition::Rejected,
            "duplicate_evidence_reference",
        );
    }

    let mut authority_ids = BTreeSet::new();
    let mut duplicate_authority_ids = BTreeSet::new();
    for authority in &trace.authority_assertions {
        if !authority_ids.insert(authority.reference.clone()) {
            duplicate_authority_ids.insert(authority.reference.clone());
        }
    }
    for reference in duplicate_authority_ids {
        report.push(
            "authority_reference",
            &reference,
            Disposition::Disputed,
            "ambiguous_authority_reference",
        );
    }

    let mut batch_ids = BTreeSet::new();
    let mut duplicate_batch_ids = BTreeSet::new();
    for batch in &trace.material_batches {
        if !batch_ids.insert(batch.batch_id.clone()) {
            duplicate_batch_ids.insert(batch.batch_id.clone());
        }
    }
    for batch in &trace.material_batches {
        let disposition = if duplicate_batch_ids.contains(&batch.batch_id) {
            Disposition::Rejected
        } else {
            material_disposition_at(trace, batch, batch.declared_at)
        };
        report.push(
            "material_batch",
            &batch.batch_id,
            disposition,
            match disposition {
                Disposition::Accepted => "supplier_claim_and_certificate_scope_bound",
                Disposition::Rejected => "material_claim_or_certificate_rejected",
                Disposition::Indeterminate => "material_certificate_missing_or_unknown",
                Disposition::Disputed => "material_certificate_disputed",
                Disposition::Superseded => "material_record_superseded",
            },
        );
    }

    let mut custody_ids = BTreeSet::new();
    let mut duplicate_custody_ids = BTreeSet::new();
    for receipt in &trace.custody_receipts {
        if !custody_ids.insert(receipt.receipt_id.clone()) {
            duplicate_custody_ids.insert(receipt.receipt_id.clone());
        }
    }
    for receipt_id in duplicate_custody_ids {
        report.push(
            "custody_receipt",
            &receipt_id,
            Disposition::Rejected,
            "duplicate_custody_receipt_id",
        );
    }
    for batch in &trace.material_batches {
        let disposition = if duplicate_batch_ids.contains(&batch.batch_id) {
            Disposition::Rejected
        } else {
            custody_disposition(trace, batch)
        };
        report.push(
            "custody",
            &batch.batch_id,
            disposition,
            match disposition {
                Disposition::Accepted => "custody_chain_contiguous_to_receiving_site",
                Disposition::Rejected => "custody_chain_discontinuous_or_malformed",
                Disposition::Indeterminate => "custody_chain_incomplete_or_evidence_unknown",
                Disposition::Disputed => "custody_chain_evidence_disputed",
                Disposition::Superseded => "custody_chain_superseded",
            },
        );
    }

    let mut approval_ids = BTreeSet::new();
    let mut duplicate_approval_ids = BTreeSet::new();
    for approval in &trace.approvals {
        if !approval_ids.insert(approval.approval_id.clone()) {
            duplicate_approval_ids.insert(approval.approval_id.clone());
        }
    }
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
            && predecessor.subject_artifact_ref == successor.subject_artifact_ref
            && predecessor.decided_at < successor.decided_at
            && participant_approval_record_disposition(trace, predecessor)
                == Disposition::Accepted
            && participant_approval_record_disposition(trace, successor)
                == Disposition::Accepted
        {
            superseded_approvals.insert(predecessor_id.to_owned());
        }
    }
    for approval in &trace.approvals {
        let disposition = if duplicate_approval_ids.contains(&approval.approval_id) {
            Disposition::Rejected
        } else {
            participant_approval_disposition(
                trace,
                approval,
                superseded_approvals.contains(&approval.approval_id),
            )
        };
        report.push(
            "participant_approval",
            &approval.approval_id,
            disposition,
            match disposition {
                Disposition::Accepted => "participant_authority_receipt_valid",
                Disposition::Rejected => "participant_attestation_rejected",
                Disposition::Indeterminate => "participant_authority_or_evidence_incomplete",
                Disposition::Disputed => "participant_attestation_disputed",
                Disposition::Superseded => "participant_attestation_not_current",
            },
        );
    }

    let mut manufacturing_ids = BTreeSet::new();
    let mut duplicate_manufacturing_ids = BTreeSet::new();
    for receipt in &trace.manufacturing_receipts {
        if !manufacturing_ids.insert(receipt.manufacturing_id.clone()) {
            duplicate_manufacturing_ids.insert(receipt.manufacturing_id.clone());
        }
    }
    let mut manufacturing_report = report.clone();
    for receipt in &trace.manufacturing_receipts {
        let disposition = if duplicate_manufacturing_ids.contains(&receipt.manufacturing_id) {
            Disposition::Rejected
        } else {
            manufacturing_disposition(trace, receipt, &manufacturing_report)
        };
        report.push(
            "manufacturing",
            &receipt.manufacturing_id,
            disposition,
            match disposition {
                Disposition::Accepted => "manufacturer_attestation_and_batch_link_valid",
                Disposition::Rejected => "manufacturer_attestation_rejected",
                Disposition::Indeterminate => "manufacturer_authority_or_input_evidence_incomplete",
                Disposition::Disputed => "manufacturer_attestation_disputed",
                Disposition::Superseded => "manufacturer_attestation_superseded",
            },
        );
        manufacturing_report.push(
            "manufacturing",
            &receipt.manufacturing_id,
            disposition,
            "manufacturing_dependency",
        );
    }

    let mut inspection_ids = BTreeSet::new();
    let mut duplicate_inspection_ids = BTreeSet::new();
    for inspection in &trace.inspections {
        if !inspection_ids.insert(inspection.inspection_id.clone()) {
            duplicate_inspection_ids.insert(inspection.inspection_id.clone());
        }
    }
    for inspection in &trace.inspections {
        let disposition = if duplicate_inspection_ids.contains(&inspection.inspection_id) {
            Disposition::Rejected
        } else {
            inspection_disposition(trace, inspection, &report)
        };
        report.push(
            "inspection",
            &inspection.inspection_id,
            disposition,
            match disposition {
                Disposition::Accepted => {
                    "inspection_attestation_bound_to_installation_and_as_built"
                }
                Disposition::Rejected => "inspection_attestation_or_claim_rejected",
                Disposition::Indeterminate => "inspection_chain_or_evidence_incomplete",
                Disposition::Disputed => "inspection_attestation_disputed",
                Disposition::Superseded => "inspection_attestation_superseded",
            },
        );
    }

    report.findings.sort_by(|left, right| {
        (&left.subject_type, &left.subject_id).cmp(&(&right.subject_type, &right.subject_id))
    });
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    const D1: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const D2: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const D3: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
    const D4: &str = "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd";
    const D5: &str = "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";

    fn artifact(
        reference: &str,
        owner: &str,
        role: &str,
        revision: &str,
        digest: &str,
        subject_ref: Option<&str>,
    ) -> ExternalArtifactRef {
        ExternalArtifactRef {
            reference: reference.into(),
            owner_system: owner.into(),
            artifact_role: role.into(),
            revision_ref: revision.into(),
            sha256: digest.into(),
            subject_ref: subject_ref.map(str::to_owned),
        }
    }

    fn evidence(reference: &str, subject: &str, digest: &str) -> EvidenceAssertion {
        EvidenceAssertion {
            reference: reference.into(),
            subject_ref: subject.into(),
            sha256: digest.into(),
            status: EvidenceStatus::Verified,
            use_permission: UsePermission::Permitted,
            available_at: 90,
            valid_from: 90,
            valid_until: Some(1_000),
        }
    }

    fn authority(reference: &str, signer: &str, role: &str, scope: &str) -> AuthorityAssertion {
        AuthorityAssertion {
            reference: reference.into(),
            signer_id: signer.into(),
            role: role.into(),
            scope_ref: scope.into(),
            status: AuthorityStatus::Active,
            available_at: 90,
            valid_from: 90,
            valid_until: Some(1_000),
        }
    }

    fn sample_trace() -> ConstructionProjectTrace {
        ConstructionProjectTrace {
            schema_version: 1,
            project_id: "pavilion-demo-01".into(),
            current_design_artifact_ref: "sol-atlas:artifact:design-r2".into(),
            site_context_artifact_ref: Some("sol-atlas:site:pavilion-01".into()),
            portfolio_alternative_artifact_ref: Some("sol-atlas:foundry:alternative-01".into()),
            artifact_references: vec![
                artifact(
                    "symthaea:design:pavilion-r1",
                    "symthaea",
                    "design_revision",
                    "design-r1",
                    D1,
                    None,
                ),
                artifact(
                    "symthaea:ifc:pavilion-r1",
                    "symthaea",
                    "ifc_model_export",
                    "design-r1",
                    D2,
                    Some("symthaea:design:pavilion-r1"),
                ),
                artifact(
                    "symthaea:drawings:pavilion-r1",
                    "symthaea",
                    "drawing_package",
                    "design-r1",
                    D3,
                    Some("symthaea:design:pavilion-r1"),
                ),
                artifact(
                    "sol-atlas:artifact:design-r1",
                    "sol-atlas-leptos",
                    "design_revision",
                    "design-r1",
                    D1,
                    None,
                ),
                artifact(
                    "symthaea:design:pavilion-r2",
                    "symthaea",
                    "design_revision",
                    "design-r2",
                    D2,
                    None,
                ),
                artifact(
                    "symthaea:ifc:pavilion-r2",
                    "symthaea",
                    "ifc_model_export",
                    "design-r2",
                    D3,
                    Some("symthaea:design:pavilion-r2"),
                ),
                artifact(
                    "symthaea:drawings:pavilion-r2",
                    "symthaea",
                    "drawing_package",
                    "design-r2",
                    D4,
                    Some("symthaea:design:pavilion-r2"),
                ),
                artifact(
                    "sol-atlas:artifact:design-r2",
                    "sol-atlas-leptos",
                    "design_revision",
                    "design-r2",
                    D2,
                    None,
                ),
                artifact(
                    "symtropy:run:sim-r1",
                    "symtropy",
                    "simulation_run",
                    "sim-r1",
                    D3,
                    Some("sol-atlas:artifact:design-r1"),
                ),
                artifact(
                    "symtropy:run:sim-r2",
                    "symtropy",
                    "simulation_run",
                    "sim-r2",
                    D4,
                    Some("sol-atlas:artifact:design-r2"),
                ),
                artifact(
                    "sol-atlas:engineering:receipt-r1",
                    "sol-atlas-leptos",
                    "engineering_receipt",
                    "receipt-r1",
                    D4,
                    Some("sol-atlas:artifact:design-r1"),
                ),
                artifact(
                    "sol-atlas:engineering:receipt-r2",
                    "sol-atlas-leptos",
                    "engineering_receipt",
                    "receipt-r2",
                    D5,
                    Some("sol-atlas:artifact:design-r2"),
                ),
                artifact(
                    "sol-atlas:site:pavilion-01",
                    "sol-atlas-leptos",
                    "site_context",
                    "site-r1",
                    D1,
                    None,
                ),
                artifact(
                    "sol-atlas:foundry:alternative-01",
                    "sol-atlas-leptos",
                    "portfolio_alternative",
                    "alternative-r1",
                    D3,
                    Some("sol-atlas:site:pavilion-01"),
                ),
                artifact(
                    "fabricator:manufacturing:batch-01",
                    "fabricator-system",
                    "manufacturing_record",
                    "mfg-r1",
                    D4,
                    Some("sol-atlas:artifact:design-r2"),
                ),
                artifact(
                    "fabricator:component:segment-01",
                    "fabricator-system",
                    "fabricated_component",
                    "component-r1",
                    D5,
                    Some("fabricator:manufacturing:batch-01"),
                ),
                artifact(
                    "metrology:inspection:segment-01",
                    "metrology-system",
                    "inspection_record",
                    "inspection-r1",
                    D3,
                    Some("fabricator:component:segment-01"),
                ),
                artifact(
                    "site:installation:segment-01",
                    "site-work-system",
                    "installation_record",
                    "install-r1",
                    D4,
                    Some("fabricator:component:segment-01"),
                ),
                artifact(
                    "site:asbuilt:segment-01",
                    "asbuilt-registry",
                    "as_built_asset",
                    "asbuilt-r1",
                    D5,
                    Some("fabricator:component:segment-01"),
                ),
            ],
            authority_assertions: vec![
                authority(
                    "authority:design-r1",
                    "party:engineer-01",
                    "design-representative",
                    "scope:design-r1",
                ),
                authority(
                    "authority:design-r2",
                    "party:engineer-01",
                    "design-representative",
                    "scope:design-r2",
                ),
                authority(
                    "authority:manufacturer",
                    "party:manufacturer-01",
                    "qualified-manufacturer",
                    "scope:manufacture:segment-01",
                ),
                authority(
                    "authority:inspector",
                    "party:inspector-01",
                    "independent-inspector",
                    "scope:inspection:segment-01",
                ),
            ],
            evidence_assertions: vec![
                evidence("evidence:design-r1", "scope:design-r1", D1),
                evidence("evidence:design-r2", "scope:design-r2", D2),
                evidence("evidence:batch-cert-01", "claim:batch-01:grade", D3),
                evidence("evidence:custody-01-a", "custody-01-a", D1),
                evidence("evidence:custody-01-b", "custody-01-b", D2),
                evidence("evidence:manufacturing-01", "scope:manufacture:segment-01", D4),
                evidence("evidence:inspection-01", "scope:inspection:segment-01", D5),
            ],
            approvals: vec![
                ParticipantApprovalReceipt {
                    approval_id: "approval-r1".into(),
                    subject_artifact_ref: "sol-atlas:artifact:design-r1".into(),
                    supporting_artifact_refs: vec![
                        "symtropy:run:sim-r1".into(),
                        "sol-atlas:engineering:receipt-r1".into(),
                    ],
                    authority_ref: "authority:design-r1".into(),
                    signer_id: "party:engineer-01".into(),
                    role: "design-representative".into(),
                    scope_ref: "scope:design-r1".into(),
                    supersedes_approval_id: None,
                    decision: ParticipantDecision::Affirmed,
                    conditions: vec![],
                    evidence_refs: vec!["evidence:design-r1".into()],
                    dissent_refs: vec![],
                    decided_at: 150,
                },
                ParticipantApprovalReceipt {
                    approval_id: "approval-r2".into(),
                    subject_artifact_ref: "sol-atlas:artifact:design-r2".into(),
                    supporting_artifact_refs: vec![
                        "symtropy:run:sim-r2".into(),
                        "sol-atlas:engineering:receipt-r2".into(),
                    ],
                    authority_ref: "authority:design-r2".into(),
                    signer_id: "party:engineer-01".into(),
                    role: "design-representative".into(),
                    scope_ref: "scope:design-r2".into(),
                    supersedes_approval_id: None,
                    decision: ParticipantDecision::Affirmed,
                    conditions: vec![ApprovalCondition {
                        condition_id: "condition:drawings-final".into(),
                        satisfied: true,
                    }],
                    evidence_refs: vec!["evidence:design-r2".into()],
                    dissent_refs: vec!["dissent:reviewer-02".into()],
                    decided_at: 240,
                },
            ],
            material_batches: vec![MaterialBatch {
                batch_id: "batch-01".into(),
                supplier_party_id: "party:supplier-01".into(),
                receiving_site_party_id: "site:pavilion-01".into(),
                material_description: "structural steel segment stock".into(),
                supplier_assertion: Some("Supplier declares certified grade".into()),
                certification_evidence_ref: Some("evidence:batch-cert-01".into()),
                certification_scope_ref: Some("claim:batch-01:grade".into()),
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
            manufacturing_receipts: vec![ManufacturingReceipt {
                manufacturing_id: "manufacture-segment-01".into(),
                design_artifact_ref: "sol-atlas:artifact:design-r2".into(),
                material_batch_id: "batch-01".into(),
                manufacturing_artifact_ref: "fabricator:manufacturing:batch-01".into(),
                component_artifact_ref: "fabricator:component:segment-01".into(),
                authority_ref: "authority:manufacturer".into(),
                manufacturer_id: "party:manufacturer-01".into(),
                role: "qualified-manufacturer".into(),
                scope_ref: "scope:manufacture:segment-01".into(),
                evidence_refs: vec!["evidence:manufacturing-01".into()],
                manufactured_at: 300,
            }],
            inspections: vec![InspectionReceipt {
                inspection_id: "inspection-segment-01".into(),
                manufacturing_id: "manufacture-segment-01".into(),
                manufacturing_artifact_ref: "fabricator:manufacturing:batch-01".into(),
                component_artifact_ref: "fabricator:component:segment-01".into(),
                inspection_artifact_ref: "metrology:inspection:segment-01".into(),
                installation_artifact_ref: "site:installation:segment-01".into(),
                as_built_artifact_ref: "site:asbuilt:segment-01".into(),
                authority_ref: "authority:inspector".into(),
                inspector_id: "party:inspector-01".into(),
                role: "independent-inspector".into(),
                scope_ref: "scope:inspection:segment-01".into(),
                evidence_refs: vec!["evidence:inspection-01".into()],
                claimed_outcome: InspectionClaim::Conforming,
                inspected_at: 400,
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
    fn complete_trace_keeps_external_engineering_ownership_and_material_custody_separate() {
        let trace = sample_trace();
        let report = verify_trace(&trace);
        assert_eq!(
            report.disposition_for("current_design", "sol-atlas:artifact:design-r2"),
            Some(Disposition::Accepted)
        );
        assert_eq!(
            report.disposition_for("participant_approval", "approval-r1"),
            Some(Disposition::Superseded)
        );
        assert_eq!(
            report.disposition_for("participant_approval", "approval-r2"),
            Some(Disposition::Accepted)
        );
        assert_eq!(
            report.disposition_for("material_batch", "batch-01"),
            Some(Disposition::Accepted)
        );
        assert_eq!(
            report.disposition_for("custody", "batch-01"),
            Some(Disposition::Accepted)
        );
        assert_eq!(
            report.disposition_for("manufacturing", "manufacture-segment-01"),
            Some(Disposition::Accepted)
        );
        assert_eq!(
            report.disposition_for("inspection", "inspection-segment-01"),
            Some(Disposition::Accepted)
        );
    }

    #[test]
    fn missing_current_design_reference_is_indeterminate() {
        let mut trace = sample_trace();
        trace.current_design_artifact_ref = "missing-design".into();
        assert_eq!(
            disposition(&trace, "current_design", "missing-design"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn external_artifact_reference_duplicate_or_digest_mismatch_is_rejected() {
        let mut trace = sample_trace();
        trace.artifact_references.push(trace.artifact_references[0].clone());
        assert_eq!(
            disposition(&trace, "artifact_reference", "symthaea:design:pavilion-r1"),
            Disposition::Rejected
        );

        let mut trace = sample_trace();
        trace.artifact_references[0].sha256 = "not-a-digest".into();
        assert_eq!(
            disposition(&trace, "artifact_reference", "symthaea:design:pavilion-r1"),
            Disposition::Rejected
        );
    }

    #[test]
    fn engineering_receipt_must_bind_to_the_exact_approved_design() {
        let mut trace = sample_trace();
        trace
            .artifact_references
            .iter_mut()
            .find(|item| item.reference == "sol-atlas:engineering:receipt-r2")
            .unwrap()
            .subject_ref = Some("sol-atlas:artifact:design-r1".into());
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn simulation_run_must_bind_to_the_same_design_as_its_approval() {
        let mut trace = sample_trace();
        trace
            .artifact_references
            .iter_mut()
            .find(|item| item.reference == "symtropy:run:sim-r2")
            .unwrap()
            .subject_ref = Some("sol-atlas:artifact:design-r1".into());
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn approval_requires_a_matching_authority_scope_and_engineering_reference() {
        let mut trace = sample_trace();
        trace.approvals[1].signer_id = "party:other-engineer".into();
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Rejected
        );

        let mut trace = sample_trace();
        trace.approvals[1].supporting_artifact_refs = vec!["missing-engineering-review".into()];
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn approval_cannot_use_disputed_revoked_or_post_cutoff_evidence() {
        let mut trace = sample_trace();
        trace.evidence_assertions[1].status = EvidenceStatus::Revoked;
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Rejected
        );

        let mut trace = sample_trace();
        trace.evidence_assertions[1].available_at = 250;
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn unresolved_approval_conditions_and_dissent_are_not_erased() {
        let mut trace = sample_trace();
        trace.approvals[1].conditions[0].satisfied = false;
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Indeterminate
        );
        assert_eq!(trace.approvals[1].dissent_refs, vec!["dissent:reviewer-02"]);
    }

    #[test]
    fn invalid_successor_does_not_supersede_valid_prior_participant_receipt() {
        let mut trace = sample_trace();
        let mut prior = trace.approvals[1].clone();
        prior.approval_id = "approval-prior-r2".into();
        prior.supersedes_approval_id = None;
        prior.decided_at = 230;
        trace.approvals.push(prior);
        trace.approvals[1].subject_artifact_ref = "sol-atlas:artifact:design-r2".into();
        trace.approvals[1].supersedes_approval_id = Some("approval-prior-r2".into());
        trace.approvals[1].signer_id = "party:unauthorized".into();

        assert_eq!(
            disposition(&trace, "participant_approval", "approval-prior-r2"),
            Disposition::Accepted
        );
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Rejected
        );
    }

    #[test]
    fn participant_receipt_on_previous_design_is_superseded_not_a_current_authorization() {
        let trace = sample_trace();
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r1"),
            Disposition::Superseded
        );
    }

    #[test]
    fn revoked_material_certification_is_rejected() {
        let mut trace = sample_trace();
        trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:batch-cert-01")
            .unwrap()
            .status = EvidenceStatus::Revoked;
        assert_eq!(
            disposition(&trace, "material_batch", "batch-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn certification_must_be_bound_to_the_exact_batch_claim_scope() {
        let mut trace = sample_trace();
        trace.material_batches[0].certification_scope_ref =
            Some("claim:another-batch:grade".into());
        assert_eq!(
            disposition(&trace, "material_batch", "batch-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn duplicate_batch_identity_is_rejected_for_every_occurrence() {
        let mut trace = sample_trace();
        trace.material_batches.push(trace.material_batches[0].clone());
        assert_eq!(
            disposition(&trace, "material_batch", "batch-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn custody_predecessor_and_cross_batch_identity_are_enforced() {
        let mut trace = sample_trace();
        trace.custody_receipts[1].predecessor_receipt_id = Some("missing-receipt".into());
        assert_eq!(
            disposition(&trace, "custody", "batch-01"),
            Disposition::Rejected
        );

        let mut trace = sample_trace();
        trace.custody_receipts.push(CustodyReceipt {
            receipt_id: "custody-01-b".into(),
            batch_id: "batch-other".into(),
            from_party_id: "party:supplier-other".into(),
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
    fn custody_predecessor_graph_handles_equal_timestamps_without_id_sorting() {
        let mut trace = sample_trace();
        trace.custody_receipts[0].receipt_id = "custody-z".into();
        trace.custody_receipts[1].receipt_id = "custody-a".into();
        trace.custody_receipts[1].predecessor_receipt_id = Some("custody-z".into());
        trace.custody_receipts[0].occurred_at = 250;
        trace.custody_receipts[1].occurred_at = 250;
        trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:custody-01-a")
            .unwrap()
            .subject_ref = "custody-z".into();
        trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:custody-01-b")
            .unwrap()
            .subject_ref = "custody-a".into();

        assert_eq!(
            disposition(&trace, "custody", "batch-01"),
            Disposition::Accepted
        );
    }

    #[test]
    fn custody_graph_rejects_branching_and_disconnected_receipts() {
        let mut branched = sample_trace();
        branched.custody_receipts.push(CustodyReceipt {
            receipt_id: "custody-01-c".into(),
            batch_id: "batch-01".into(),
            from_party_id: "party:carrier-01".into(),
            to_party_id: "site:pavilion-01".into(),
            predecessor_receipt_id: Some("custody-01-a".into()),
            evidence_ref: "evidence:custody-01-b".into(),
            occurred_at: 260,
        });
        assert_eq!(
            disposition(&branched, "custody", "batch-01"),
            Disposition::Rejected
        );

        let mut disconnected = sample_trace();
        disconnected.custody_receipts[1].predecessor_receipt_id = None;
        assert_eq!(
            disposition(&disconnected, "custody", "batch-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn custody_chain_must_terminate_at_declared_receiving_site() {
        let mut trace = sample_trace();
        trace.custody_receipts[1].to_party_id = "site:other".into();
        assert_eq!(
            disposition(&trace, "custody", "batch-01"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn manufacturing_requires_participant_commitment_before_manufacture() {
        let mut trace = sample_trace();
        trace.approvals[1].decided_at = 350;
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn material_certification_must_remain_valid_at_manufacturing_time() {
        let mut trace = sample_trace();
        let certificate = trace
            .evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:batch-cert-01")
            .unwrap();
        certificate.valid_until = Some(250);
        assert_eq!(
            disposition(&trace, "material_batch", "batch-01"),
            Disposition::Accepted
        );
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn manufacturing_requires_accepted_material_and_terminal_delivery() {
        let mut trace = sample_trace();
        trace.manufacturing_receipts[0].manufactured_at = 260;
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Indeterminate
        );

        let mut trace = sample_trace();
        trace.evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:batch-cert-01")
            .unwrap()
            .status = EvidenceStatus::Revoked;
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn manufacturing_artifacts_must_resolve_to_exact_roles() {
        let mut trace = sample_trace();
        let artifact = trace
            .artifact_references
            .iter_mut()
            .find(|item| item.reference == "fabricator:component:segment-01")
            .unwrap();
        artifact.artifact_role = "design_revision".into();
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn manufacturing_requires_a_current_design_and_accepted_participant_commitment() {
        let mut trace = sample_trace();
        trace.manufacturing_receipts[0].design_artifact_ref =
            "sol-atlas:artifact:design-r1".into();
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Superseded
        );

        let mut trace = sample_trace();
        trace.approvals[1].decision = ParticipantDecision::Deferred;
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn inspection_must_preserve_exact_manufacturing_and_component_identity() {
        let mut trace = sample_trace();
        trace.inspections[0].component_artifact_ref = "fabricator:component:other".into();
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Rejected
        );

        let mut trace = sample_trace();
        trace.inspections[0].manufacturing_artifact_ref = "fabricator:manufacturing:other".into();
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn inspection_requires_manufacturing_and_exact_installation_as_built_refs() {
        let mut trace = sample_trace();
        trace.inspections[0].installation_artifact_ref = "missing-installation".into();
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Indeterminate
        );

        let mut trace = sample_trace();
        trace.inspections[0].claimed_outcome = InspectionClaim::Nonconforming;
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn inspection_propagates_rejected_manufacturing_without_downgrading_it() {
        let mut trace = sample_trace();
        trace
            .evidence_assertions
            .iter_mut()
            .find(|item| item.reference == "evidence:batch-cert-01")
            .unwrap()
            .status = EvidenceStatus::Revoked;
        assert_eq!(
            disposition(&trace, "manufacturing", "manufacture-segment-01"),
            Disposition::Rejected
        );
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Rejected
        );
    }

    #[test]
    fn inspection_cannot_precede_manufacture_or_rely_on_unaccepted_manufacturing() {
        let mut trace = sample_trace();
        trace.inspections[0].inspected_at = 290;
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Indeterminate
        );

        let mut trace = sample_trace();
        trace.manufacturing_receipts[0].authority_ref = "missing-authority".into();
        assert_eq!(
            disposition(&trace, "inspection", "inspection-segment-01"),
            Disposition::Indeterminate
        );
    }

    #[test]
    fn unknown_and_disputed_authority_never_become_affirmation() {
        let mut trace = sample_trace();
        trace.authority_assertions[1].status = AuthorityStatus::Unknown;
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Indeterminate
        );

        let mut trace = sample_trace();
        trace.authority_assertions[1].status = AuthorityStatus::Disputed;
        assert_eq!(
            disposition(&trace, "participant_approval", "approval-r2"),
            Disposition::Disputed
        );
    }

    #[test]
    fn supplier_assertion_without_independent_certification_is_indeterminate() {
        let mut trace = sample_trace();
        trace.material_batches[0].certification_evidence_ref = None;
        trace.material_batches[0].certification_scope_ref = None;
        assert_eq!(
            disposition(&trace, "material_batch", "batch-01"),
            Disposition::Indeterminate
        );
    }
}
