//! Transitive scope, provenance and revocation qualification for derived memory artifacts.
//!
//! This module validates declared evidence records and receipts. It does not
//! implement storage, authorization, deletion, model unlearning or runtime
//! enforcement. Receipt digests are structural bindings, not authentication.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use sha2::{Digest, Sha256};

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn push_len(out: &mut Vec<u8>, value: usize) {
    out.extend_from_slice(&(value as u64).to_be_bytes());
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_len(out, value.len());
    out.extend_from_slice(value.as_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_optional_str(out: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            out.push(1);
            push_str(out, value);
        }
        None => out.push(0),
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

const SCOPE_DOMAIN: &[u8] = b"MYCELIX-MEMORY-SCOPE-V0.1\0";
const GRAPH_DOMAIN: &[u8] = b"MYCELIX-TRANSITIVE-MEMORY-GRAPH-V0.1\0";
const SNAPSHOT_DOMAIN: &[u8] = b"MYCELIX-TRANSITIVE-MEMORY-SNAPSHOT-V0.1\0";

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryScope {
    /// Exact principals; wildcard and empty-as-unbounded semantics are forbidden.
    pub principals: Vec<String>,
    /// Exact permitted purposes; purpose equivalence is outside this validator.
    pub purposes: Vec<String>,
    /// Exact entity-scope identifiers.
    pub entities: Vec<String>,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub retain_until_ms: u64,
}

fn validate_scope(scope: &MemoryScope) -> Result<(), String> {
    for (label, values) in [
        ("principal", &scope.principals),
        ("purpose", &scope.purposes),
        ("entity", &scope.entities),
    ] {
        let mut seen = BTreeSet::new();
        if values.is_empty()
            || values
                .iter()
                .any(|value| !nonempty(value) || !seen.insert(value.as_str()))
        {
            return Err(format!("{label} scope must be non-empty and unique"));
        }
    }
    if scope.valid_from_ms >= scope.valid_until_ms {
        return Err("memory scope validity interval is empty or reversed".into());
    }
    if scope.retain_until_ms == 0 {
        return Err("memory retention deadline must be explicit and positive".into());
    }
    Ok(())
}

/// Returns a deterministic, domain-separated digest of an exact scope envelope.
pub fn compute_memory_scope_digest(scope: &MemoryScope) -> Result<String, String> {
    validate_scope(scope)?;
    let mut bytes = Vec::new();
    bytes.extend_from_slice(SCOPE_DOMAIN);
    for values in [&scope.principals, &scope.purposes, &scope.entities] {
        let sorted: BTreeSet<&str> = values.iter().map(String::as_str).collect();
        push_len(&mut bytes, sorted.len());
        for value in sorted {
            push_str(&mut bytes, value);
        }
    }
    push_u64(&mut bytes, scope.valid_from_ms);
    push_u64(&mut bytes, scope.valid_until_ms);
    push_u64(&mut bytes, scope.retain_until_ms);
    Ok(sha256_hex(&bytes))
}

fn set_is_subset(inner: &[String], outer: &[String]) -> bool {
    let outer: BTreeSet<&str> = outer.iter().map(String::as_str).collect();
    inner.iter().all(|value| outer.contains(value.as_str()))
}

/// True only when every permission and temporal bound is no broader than the parent.
fn scope_is_within(child: &MemoryScope, parent: &MemoryScope) -> bool {
    set_is_subset(&child.principals, &parent.principals)
        && set_is_subset(&child.purposes, &parent.purposes)
        && set_is_subset(&child.entities, &parent.entities)
        && child.valid_from_ms >= parent.valid_from_ms
        && child.valid_until_ms <= parent.valid_until_ms
        && child.retain_until_ms <= parent.retain_until_ms
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryDeclassificationReceipt {
    pub receipt_id: String,
    /// Ordered exactly like the derived artifact's parent_artifact_ids.
    pub input_artifact_ids: Vec<String>,
    /// Ordered exactly like the derived artifact's parent_artifact_ids.
    pub input_payload_digests: Vec<String>,
    pub transform_digest: String,
    pub output_payload_digest: String,
    pub output_scope_digest: String,
    pub policy_digest: String,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    pub receipt_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryArtifact {
    pub artifact_id: String,
    pub payload_digest: String,
    /// Sorted unique parent IDs. An empty vector denotes a source artifact.
    pub parent_artifact_ids: Vec<String>,
    pub transform_digest: Option<String>,
    pub derivation_receipt_digest: Option<String>,
    pub declassification: Option<MemoryDeclassificationReceipt>,
    pub scope: MemoryScope,
    pub created_at_ms: u64,
    /// False means the producer cannot attest to complete parent lineage.
    pub lineage_complete: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryRevocationEvent {
    pub event_id: String,
    pub source_artifact_id: String,
    /// When the revocation became effective under the named policy.
    pub effective_at_ms: u64,
    /// When this evaluator/authority domain first observed the event.
    pub recorded_at_ms: u64,
    pub policy_epoch_digest: String,
    pub authority_receipt_digest: String,
    pub event_receipt_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryRevocationClosure {
    pub event_id: String,
    pub source_artifact_id: String,
    /// Sorted unique transitive descendants; the source itself is not included.
    pub descendant_artifact_ids: Vec<String>,
    pub graph_digest: String,
    pub complete: bool,
    pub closure_receipt_digest: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryUseDisposition {
    Consumed,
    BlockedByScope,
    BlockedByRetention,
    BlockedByRevocation,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryUseObservation {
    pub use_id: String,
    pub artifact_id: String,
    pub principal: String,
    pub purpose: String,
    pub entity: String,
    pub used_at_ms: u64,
    pub snapshot_digest: String,
    pub disposition: MemoryUseDisposition,
    pub use_receipt_digest: String,
    pub effect_receipt_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TransitiveMemorySnapshot {
    pub snapshot_id: String,
    /// Evidence-frontier time for this inventory.
    pub as_of_ms: u64,
    pub graph_digest: String,
    pub snapshot_digest: String,
    pub inventory_complete: bool,
    pub revocation_inventory_complete: bool,
    pub artifacts: Vec<MemoryArtifact>,
    pub revocations: Vec<MemoryRevocationEvent>,
    pub revocation_closures: Vec<MemoryRevocationClosure>,
    pub uses: Vec<MemoryUseObservation>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransitiveMemoryStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TransitiveMemoryReport {
    pub status: TransitiveMemoryStatus,
    pub snapshot_id: String,
    pub artifact_count: usize,
    pub derived_artifact_count: usize,
    pub revocation_count: usize,
    pub revoked_descendant_count: usize,
    pub consumed_use_count: usize,
    pub blocked_use_count: usize,
    pub unknown_use_count: usize,
    pub reasons: Vec<String>,
}

fn report(
    status: TransitiveMemoryStatus,
    snapshot: &TransitiveMemorySnapshot,
    reasons: Vec<String>,
) -> TransitiveMemoryReport {
    TransitiveMemoryReport {
        status,
        snapshot_id: snapshot.snapshot_id.clone(),
        artifact_count: snapshot.artifacts.len(),
        derived_artifact_count: snapshot
            .artifacts
            .iter()
            .filter(|artifact| !artifact.parent_artifact_ids.is_empty())
            .count(),
        revocation_count: snapshot.revocations.len(),
        revoked_descendant_count: 0,
        consumed_use_count: 0,
        blocked_use_count: 0,
        unknown_use_count: 0,
        reasons,
    }
}

/// Computes the artifact graph digest over sorted artifact identities and lineage edges.
pub fn compute_memory_graph_digest(artifacts: &[MemoryArtifact]) -> Result<String, String> {
    let mut sorted = artifacts.iter().collect::<Vec<_>>();
    sorted.sort_by(|left, right| left.artifact_id.cmp(&right.artifact_id));
    let mut ids = BTreeSet::new();
    let mut bytes = Vec::new();
    bytes.extend_from_slice(GRAPH_DOMAIN);
    push_len(&mut bytes, sorted.len());

    for artifact in sorted {
        if !nonempty(&artifact.artifact_id)
            || !valid_digest(&artifact.payload_digest)
            || !ids.insert(artifact.artifact_id.as_str())
        {
            return Err("memory artifact identity is invalid or duplicated".into());
        }
        validate_scope(&artifact.scope)?;
        if artifact.created_at_ms == 0 {
            return Err("memory artifact creation time must be positive".into());
        }
        if artifact.scope.retain_until_ms < artifact.created_at_ms {
            return Err("memory artifact retention deadline precedes its creation".into());
        }
        let mut parents = artifact.parent_artifact_ids.clone();
        parents.sort();
        parents.dedup();
        if parents != artifact.parent_artifact_ids
            || parents.iter().any(|parent| !nonempty(parent))
        {
            return Err("memory artifact parent IDs must be sorted, non-empty and unique".into());
        }
        push_str(&mut bytes, &artifact.artifact_id);
        push_str(&mut bytes, &artifact.payload_digest);
        push_len(&mut bytes, parents.len());
        for parent in parents {
            push_str(&mut bytes, &parent);
        }
        push_optional_str(&mut bytes, artifact.transform_digest.as_deref());
        push_optional_str(&mut bytes, artifact.derivation_receipt_digest.as_deref());
        match &artifact.declassification {
            Some(receipt) => {
                bytes.push(1);
                push_str(&mut bytes, &receipt.receipt_digest);
            }
            None => bytes.push(0),
        }
        push_str(&mut bytes, &compute_memory_scope_digest(&artifact.scope)?);
        push_u64(&mut bytes, artifact.created_at_ms);
        bytes.push(u8::from(artifact.lineage_complete));
    }
    Ok(sha256_hex(&bytes))
}

/// Computes a snapshot identity over the graph, revocation events and closure receipts.
/// Use observations bind this digest but are excluded to avoid a circular commitment.
pub fn compute_transitive_memory_snapshot_digest(
    snapshot: &TransitiveMemorySnapshot,
) -> Result<String, String> {
    let graph_digest = compute_memory_graph_digest(&snapshot.artifacts)?;
    if graph_digest != snapshot.graph_digest {
        return Err("snapshot graph digest does not bind the declared artifacts".into());
    }
    let mut revocations = snapshot.revocations.iter().collect::<Vec<_>>();
    revocations.sort_by(|left, right| left.event_id.cmp(&right.event_id));
    let mut closures = snapshot.revocation_closures.iter().collect::<Vec<_>>();
    closures.sort_by(|left, right| {
        (&left.event_id, &left.source_artifact_id)
            .cmp(&(&right.event_id, &right.source_artifact_id))
    });

    let mut bytes = Vec::new();
    if !nonempty(&snapshot.snapshot_id) || snapshot.as_of_ms == 0 {
        return Err("snapshot identity and as-of time must be explicit".into());
    }
    bytes.extend_from_slice(SNAPSHOT_DOMAIN);
    push_str(&mut bytes, &snapshot.snapshot_id);
    push_u64(&mut bytes, snapshot.as_of_ms);
    push_str(&mut bytes, &graph_digest);
    bytes.push(u8::from(snapshot.inventory_complete));
    bytes.push(u8::from(snapshot.revocation_inventory_complete));
    push_len(&mut bytes, revocations.len());
    for event in revocations {
        push_str(&mut bytes, &event.event_id);
        push_str(&mut bytes, &event.source_artifact_id);
        push_u64(&mut bytes, event.effective_at_ms);
        push_u64(&mut bytes, event.recorded_at_ms);
        push_str(&mut bytes, &event.policy_epoch_digest);
        push_str(&mut bytes, &event.authority_receipt_digest);
        push_str(&mut bytes, &event.event_receipt_digest);
    }
    push_len(&mut bytes, closures.len());
    for closure in closures {
        push_str(&mut bytes, &closure.event_id);
        push_str(&mut bytes, &closure.source_artifact_id);
        push_len(&mut bytes, closure.descendant_artifact_ids.len());
        for artifact_id in &closure.descendant_artifact_ids {
            push_str(&mut bytes, artifact_id);
        }
        push_str(&mut bytes, &closure.graph_digest);
        bytes.push(u8::from(closure.complete));
        push_str(&mut bytes, &closure.closure_receipt_digest);
    }
    Ok(sha256_hex(&bytes))
}

struct GraphIndex {
    artifacts: BTreeMap<String, MemoryArtifact>,
    ancestors: BTreeMap<String, BTreeSet<String>>,
    descendants: BTreeMap<String, BTreeSet<String>>,
    incomplete_lineage: bool,
}

fn build_graph_index(
    artifacts: &[MemoryArtifact],
) -> Result<GraphIndex, String> {
    let mut by_id = BTreeMap::new();
    for artifact in artifacts {
        if by_id
            .insert(artifact.artifact_id.clone(), artifact.clone())
            .is_some()
        {
            return Err("memory artifact IDs must be unique".into());
        }
    }

    let mut children: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut indegree: BTreeMap<String, usize> =
        by_id.keys().map(|id| (id.clone(), 0)).collect();
    let mut incomplete_lineage = false;

    for artifact in artifacts {
        if !artifact.lineage_complete {
            incomplete_lineage = true;
        }
        if artifact.parent_artifact_ids.is_empty() {
            if artifact.transform_digest.is_some()
                || artifact.derivation_receipt_digest.is_some()
                || artifact.declassification.is_some()
            {
                return Err("source artifacts cannot declare transform or derivation receipts".into());
            }
            continue;
        }
        if !artifact.transform_digest.as_deref().is_some_and(valid_digest)
            || !artifact.derivation_receipt_digest.as_deref().is_some_and(valid_digest)
        {
            return Err("derived artifacts require transform and derivation receipt digests".into());
        }
        for parent_id in &artifact.parent_artifact_ids {
            if !by_id.contains_key(parent_id) {
                if artifact.lineage_complete {
                    return Err(format!(
                        "complete artifact lineage references missing parent {parent_id}"
                    ));
                }
                incomplete_lineage = true;
                continue;
            }
            let parent = by_id.get(parent_id).expect("parent presence checked above");
            if parent.created_at_ms > artifact.created_at_ms {
                return Err("derived artifact predates one of its declared parents".into());
            }
            *indegree
                .get_mut(&artifact.artifact_id)
                .expect("artifact ID inserted above") += 1;
            children
                .entry(parent_id.clone())
                .or_default()
                .insert(artifact.artifact_id.clone());
        }
    }

    let mut queue: VecDeque<String> = indegree
        .iter()
        .filter_map(|(id, degree)| (*degree == 0).then_some(id.clone()))
        .collect();
    let mut topological = Vec::with_capacity(by_id.len());
    while let Some(id) = queue.pop_front() {
        topological.push(id.clone());
        if let Some(child_ids) = children.get(&id) {
            for child_id in child_ids {
                let degree = indegree
                    .get_mut(child_id)
                    .expect("child ID inserted above");
                *degree -= 1;
                if *degree == 0 {
                    queue.push_back(child_id.clone());
                }
            }
        }
    }
    if topological.len() != by_id.len() {
        return Err("memory artifact lineage contains a cycle".into());
    }

    let mut ancestors: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for id in &topological {
        let artifact = by_id.get(id).expect("topological ID must exist");
        let mut inherited = BTreeSet::new();
        for parent_id in &artifact.parent_artifact_ids {
            if let Some(parent) = ancestors.get(parent_id) {
                inherited.insert(parent_id.clone());
                inherited.extend(parent.iter().cloned());
            }
        }
        ancestors.insert(id.clone(), inherited);
    }

    let mut descendants: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for (artifact_id, ancestor_ids) in &ancestors {
        for ancestor_id in ancestor_ids {
            descendants
                .entry(ancestor_id.clone())
                .or_default()
                .insert(artifact_id.clone());
        }
    }

    Ok(GraphIndex {
        artifacts: by_id,
        ancestors,
        descendants,
        incomplete_lineage,
    })
}

fn validate_declassification(
    artifact: &MemoryArtifact,
    parents: &[&MemoryArtifact],
    receipt: &MemoryDeclassificationReceipt,
) -> Result<(), String> {
    if !nonempty(&receipt.receipt_id)
        || !valid_digest(&receipt.transform_digest)
        || !valid_digest(&receipt.output_payload_digest)
        || !valid_digest(&receipt.output_scope_digest)
        || !valid_digest(&receipt.policy_digest)
        || !valid_digest(&receipt.receipt_digest)
        || receipt.issued_at_ms > artifact.created_at_ms
        || artifact.created_at_ms >= receipt.expires_at_ms
    {
        return Err("declassification receipt identity, digest or validity is invalid".into());
    }
    let input_ids = parents
        .iter()
        .map(|parent| parent.artifact_id.clone())
        .collect::<Vec<_>>();
    let input_digests = parents
        .iter()
        .map(|parent| parent.payload_digest.clone())
        .collect::<Vec<_>>();
    if receipt.input_artifact_ids != input_ids
        || receipt.input_payload_digests != input_digests
        || receipt.transform_digest != artifact.transform_digest.as_deref().unwrap_or_default()
        || receipt.output_payload_digest != artifact.payload_digest
        || receipt.output_scope_digest != compute_memory_scope_digest(&artifact.scope)?
    {
        return Err("declassification receipt does not bind the exact inputs, transform, output or scope".into());
    }
    Ok(())
}

fn validate_scope_inheritance(
    artifact: &MemoryArtifact,
    index: &GraphIndex,
) -> Result<(), String> {
    if artifact.parent_artifact_ids.is_empty() {
        return Ok(());
    }
    let parents = artifact
        .parent_artifact_ids
        .iter()
        .filter_map(|id| index.artifacts.get(id))
        .collect::<Vec<_>>();
    if parents.len() != artifact.parent_artifact_ids.len() {
        // The missing parent is permitted only on an explicitly incomplete lineage.
        return if artifact.lineage_complete {
            Err("complete derivation is missing a parent artifact".into())
        } else {
            Ok(())
        };
    }

    let within_every_parent = parents
        .iter()
        .all(|parent| scope_is_within(&artifact.scope, &parent.scope));
    match (&artifact.declassification, within_every_parent) {
        (None, true) => Ok(()),
        (Some(receipt), _) => validate_declassification(artifact, &parents, receipt),
        (None, false) => Err(
            "derived artifact amplifies principal, purpose, entity, validity or retention scope without declassification"
                .into(),
        ),
    }
}

fn expected_use_disposition(
    artifact: &MemoryArtifact,
    source_revocations: &[&MemoryRevocationEvent],
    use_event: &MemoryUseObservation,
) -> Option<MemoryUseDisposition> {
    if source_revocations.iter().any(|event| {
        event.effective_at_ms <= use_event.used_at_ms
            && event.recorded_at_ms <= use_event.used_at_ms
    }) {
        return Some(MemoryUseDisposition::BlockedByRevocation);
    }
    if use_event.used_at_ms >= artifact.scope.retain_until_ms {
        return Some(MemoryUseDisposition::BlockedByRetention);
    }
    if use_event.used_at_ms < artifact.scope.valid_from_ms
        || use_event.used_at_ms >= artifact.scope.valid_until_ms
        || !artifact.scope.principals.contains(&use_event.principal)
        || !artifact.scope.purposes.contains(&use_event.purpose)
        || !artifact.scope.entities.contains(&use_event.entity)
    {
        return Some(MemoryUseDisposition::BlockedByScope);
    }
    None
}

/// Qualifies transitive scope inheritance, descendant closure and time-qualified uses.
///
/// A revocation blocks future use of the source and its known descendants. Uses
/// before the event's effective time remain historical facts and are not rewritten.
pub fn qualify_transitive_memory_snapshot(
    snapshot: &TransitiveMemorySnapshot,
) -> TransitiveMemoryReport {
    if !nonempty(&snapshot.snapshot_id) {
        return report(
            TransitiveMemoryStatus::Fail,
            snapshot,
            vec!["snapshot ID must be non-empty".into()],
        );
    }
    let graph_digest = match compute_memory_graph_digest(&snapshot.artifacts) {
        Ok(digest) => digest,
        Err(error) => {
            return report(TransitiveMemoryStatus::Fail, snapshot, vec![error]);
        }
    };
    if graph_digest != snapshot.graph_digest {
        return report(
            TransitiveMemoryStatus::Fail,
            snapshot,
            vec!["memory graph digest does not bind its artifact and lineage records".into()],
        );
    }
    if snapshot.as_of_ms == 0
        || snapshot.artifacts.iter().any(|artifact| artifact.created_at_ms > snapshot.as_of_ms)
    {
        return report(
            TransitiveMemoryStatus::Fail,
            snapshot,
            vec!["snapshot as-of time is invalid or precedes an included artifact".into()],
        );
    }
    let snapshot_digest = match compute_transitive_memory_snapshot_digest(snapshot) {
        Ok(digest) => digest,
        Err(error) => {
            return report(TransitiveMemoryStatus::Fail, snapshot, vec![error]);
        }
    };
    if snapshot_digest != snapshot.snapshot_digest {
        return report(
            TransitiveMemoryStatus::Fail,
            snapshot,
            vec!["snapshot digest does not bind graph, revocation and closure records".into()],
        );
    }
    let index = match build_graph_index(&snapshot.artifacts) {
        Ok(index) => index,
        Err(error) => {
            return report(TransitiveMemoryStatus::Fail, snapshot, vec![error]);
        }
    };

    let mut result = report(TransitiveMemoryStatus::Pass, snapshot, Vec::new());
    if !snapshot.inventory_complete || index.incomplete_lineage {
        result.status = TransitiveMemoryStatus::Indeterminate;
        result.reasons.push("artifact inventory or at least one lineage is incomplete".into());
    }
    if !snapshot.revocation_inventory_complete {
        result.status = TransitiveMemoryStatus::Indeterminate;
        result.reasons.push("revocation inventory is incomplete".into());
    }

    for artifact in &snapshot.artifacts {
        if let Err(error) = validate_scope_inheritance(artifact, &index) {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!("artifact {}: {error}", artifact.artifact_id));
            return result;
        }
    }

    let mut event_by_id = BTreeMap::new();
    for event in &snapshot.revocations {
        if !nonempty(&event.event_id)
            || !nonempty(&event.source_artifact_id)
            || event.effective_at_ms == 0
            || event.recorded_at_ms == 0
            || event.recorded_at_ms > snapshot.as_of_ms
            || !valid_digest(&event.policy_epoch_digest)
            || !valid_digest(&event.authority_receipt_digest)
            || !valid_digest(&event.event_receipt_digest)
            || event_by_id.insert(event.event_id.as_str(), event).is_some()
        {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push("revocation event identity, digest or uniqueness is invalid".into());
            return result;
        }
        if !index.artifacts.contains_key(&event.source_artifact_id) {
            if snapshot.revocation_inventory_complete {
                result.status = TransitiveMemoryStatus::Fail;
                result.reasons.push(format!(
                    "revocation {} references an unknown source artifact",
                    event.event_id
                ));
                return result;
            }
            result.status = TransitiveMemoryStatus::Indeterminate;
            result.reasons.push(format!(
                "revocation {} source artifact cannot be resolved",
                event.event_id
            ));
        }
    }

    // A derivative created after a contributing source was both effective-revoked
    // and known-revoked is itself an invalid post-revocation derivation.
    for artifact in &snapshot.artifacts {
        let mut contributing_sources = index
            .ancestors
            .get(&artifact.artifact_id)
            .cloned()
            .unwrap_or_default();
        contributing_sources.insert(artifact.artifact_id.clone());
        if snapshot.revocations.iter().any(|event| {
            contributing_sources.contains(&event.source_artifact_id)
                && event.effective_at_ms <= artifact.created_at_ms
                && event.recorded_at_ms <= artifact.created_at_ms
        }) {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "artifact {} was derived after a contributing source was effectively revoked and the revocation was known",
                artifact.artifact_id
            ));
            return result;
        }
    }

    let mut closure_by_event = BTreeMap::new();
    for closure in &snapshot.revocation_closures {
        if !event_by_id.contains_key(closure.event_id.as_str())
            || !valid_digest(&closure.graph_digest)
            || !valid_digest(&closure.closure_receipt_digest)
            || closure
                .descendant_artifact_ids
                .iter()
                .any(|id| !nonempty(id))
            || closure
                .descendant_artifact_ids
                .windows(2)
                .any(|pair| pair[0] >= pair[1])
            || closure_by_event.insert(closure.event_id.as_str(), closure).is_some()
        {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push("revocation closure identity, ordering or receipt is invalid".into());
            return result;
        }
        let Some(event) = event_by_id.get(closure.event_id.as_str()) else {
            unreachable!("event membership checked above");
        };
        if closure.source_artifact_id != event.source_artifact_id
            || closure.graph_digest != snapshot.graph_digest
        {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "revocation closure {} does not bind its event source and graph",
                closure.event_id
            ));
            return result;
        }
        if !closure.complete {
            result.status = TransitiveMemoryStatus::Indeterminate;
            result.reasons.push(format!(
                "revocation closure {} is explicitly incomplete",
                closure.event_id
            ));
            continue;
        }
        let expected = index
            .descendants
            .get(&closure.source_artifact_id)
            .cloned()
            .unwrap_or_default();
        let declared = closure
            .descendant_artifact_ids
            .iter()
            .cloned()
            .collect::<BTreeSet<_>>();
        if expected != declared {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "revocation closure {} omits or invents descendants",
                closure.event_id
            ));
            return result;
        }
        result.revoked_descendant_count += expected.len();
    }
    for event in &snapshot.revocations {
        if !closure_by_event.contains_key(event.event_id.as_str()) {
            result.status = TransitiveMemoryStatus::Indeterminate;
            result.reasons.push(format!(
                "revocation {} has no descendant-closure receipt",
                event.event_id
            ));
        }
    }

    let mut use_ids = BTreeSet::new();
    for use_event in &snapshot.uses {
        if !nonempty(&use_event.use_id)
            || !nonempty(&use_event.principal)
            || !nonempty(&use_event.purpose)
            || !nonempty(&use_event.entity)
            || !valid_digest(&use_event.snapshot_digest)
            || !valid_digest(&use_event.use_receipt_digest)
            || !use_ids.insert(use_event.use_id.as_str())
        {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push("memory use identity, scope or receipt is invalid".into());
            return result;
        }
        // A use points to the policy/evidence snapshot actually observed at its
        // event time. It may legitimately differ from the current snapshot after
        // revocation; resolving that historical digest is an external archive duty.
        if use_event.used_at_ms > snapshot.as_of_ms {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "use {} occurs after the declared snapshot as-of time",
                use_event.use_id
            ));
            return result;
        }
        if use_event
            .effect_receipt_digest
            .as_deref()
            .is_some_and(|digest| !valid_digest(digest))
            || (use_event.disposition == MemoryUseDisposition::Consumed
                && use_event.effect_receipt_digest.is_none())
        {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "use {} has an invalid or missing effect receipt",
                use_event.use_id
            ));
            return result;
        }
        let Some(artifact) = index.artifacts.get(&use_event.artifact_id) else {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "use {} references an unknown artifact",
                use_event.use_id
            ));
            return result;
        };
        if use_event.used_at_ms < artifact.created_at_ms {
            result.status = TransitiveMemoryStatus::Fail;
            result.reasons.push(format!(
                "use {} predates artifact creation",
                use_event.use_id
            ));
            return result;
        }
        let mut contributing_sources = index
            .ancestors
            .get(&artifact.artifact_id)
            .cloned()
            .unwrap_or_default();
        contributing_sources.insert(artifact.artifact_id.clone());
        let source_revocations = snapshot
            .revocations
            .iter()
            .filter(|event| contributing_sources.contains(&event.source_artifact_id))
            .collect::<Vec<_>>();
        let expected = expected_use_disposition(artifact, &source_revocations, use_event);
        match (expected, use_event.disposition) {
            (Some(expected), actual) if expected == actual => {
                result.blocked_use_count += 1;
            }
            (Some(_), MemoryUseDisposition::Unknown)
            | (None, MemoryUseDisposition::Unknown) => {
                result.status = TransitiveMemoryStatus::Indeterminate;
                result.unknown_use_count += 1;
                result.reasons.push(format!(
                    "use {} has an unknown consumed/blocked disposition",
                    use_event.use_id
                ));
            }
            (Some(expected), actual) => {
                result.status = TransitiveMemoryStatus::Fail;
                result.reasons.push(format!(
                    "use {} reports {:?}, but its scope and history require {:?}",
                    use_event.use_id, actual, expected
                ));
                return result;
            }
            (None, MemoryUseDisposition::Consumed) => {
                result.consumed_use_count += 1;
            }
            (None, actual) => {
                result.status = TransitiveMemoryStatus::Fail;
                result.reasons.push(format!(
                    "use {} reports {:?} although no modeled scope, retention or revocation block applies",
                    use_event.use_id, actual
                ));
                return result;
            }
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn scope(principal: &str, purpose: &str, entity: &str, until: u64) -> MemoryScope {
        MemoryScope {
            principals: vec![principal.into()],
            purposes: vec![purpose.into()],
            entities: vec![entity.into()],
            valid_from_ms: 10,
            valid_until_ms: until,
            retain_until_ms: until + 100,
        }
    }

    fn source() -> MemoryArtifact {
        MemoryArtifact {
            artifact_id: "source".into(),
            payload_digest: digest('a'),
            parent_artifact_ids: Vec::new(),
            transform_digest: None,
            derivation_receipt_digest: None,
            declassification: None,
            scope: scope("alice", "project-a", "entity-1", 1000),
            created_at_ms: 20,
            lineage_complete: true,
        }
    }

    fn derivative(parent: &MemoryArtifact) -> MemoryArtifact {
        MemoryArtifact {
            artifact_id: "summary".into(),
            payload_digest: digest('b'),
            parent_artifact_ids: vec![parent.artifact_id.clone()],
            transform_digest: Some(digest('c')),
            derivation_receipt_digest: Some(digest('d')),
            declassification: None,
            scope: scope("alice", "project-a", "entity-1", 900),
            created_at_ms: 30,
            lineage_complete: true,
        }
    }

    fn base_snapshot(
        artifacts: Vec<MemoryArtifact>,
        revocations: Vec<MemoryRevocationEvent>,
        closures: Vec<MemoryRevocationClosure>,
    ) -> TransitiveMemorySnapshot {
        let graph_digest = compute_memory_graph_digest(&artifacts).unwrap();
        let mut snapshot = TransitiveMemorySnapshot {
            snapshot_id: "snapshot-01".into(),
            as_of_ms: 1000,
            graph_digest,
            snapshot_digest: String::new(),
            inventory_complete: true,
            revocation_inventory_complete: true,
            artifacts,
            revocations,
            revocation_closures: closures,
            uses: Vec::new(),
        };
        snapshot.snapshot_digest =
            compute_transitive_memory_snapshot_digest(&snapshot).unwrap();
        snapshot
    }

    fn use_event(
        id: &str,
        artifact: &MemoryArtifact,
        snapshot: &TransitiveMemorySnapshot,
        time: u64,
        disposition: MemoryUseDisposition,
    ) -> MemoryUseObservation {
        MemoryUseObservation {
            use_id: id.into(),
            artifact_id: artifact.artifact_id.clone(),
            principal: "alice".into(),
            purpose: "project-a".into(),
            entity: "entity-1".into(),
            used_at_ms: time,
            snapshot_digest: snapshot.snapshot_digest.clone(),
            disposition,
            use_receipt_digest: digest('e'),
            effect_receipt_digest: (disposition == MemoryUseDisposition::Consumed)
                .then(|| digest('f')),
        }
    }

    fn seal_uses(snapshot: &mut TransitiveMemorySnapshot) {
        for use_event in &mut snapshot.uses {
            use_event.snapshot_digest = snapshot.snapshot_digest.clone();
        }
    }

    #[test]
    fn narrowed_derivative_and_authorized_use_pass() {
        let root = source();
        let child = derivative(&root);
        let mut snapshot = base_snapshot(vec![root.clone(), child.clone()], Vec::new(), Vec::new());
        snapshot.uses.push(use_event(
            "use-summary",
            &child,
            &snapshot,
            100,
            MemoryUseDisposition::Consumed,
        ));
        seal_uses(&mut snapshot);
        snapshot.snapshot_digest =
            compute_transitive_memory_snapshot_digest(&snapshot).unwrap();
        seal_uses(&mut snapshot);
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Pass
        );
    }

    #[test]
    fn scope_amplification_without_declassification_fails() {
        let root = source();
        let mut child = derivative(&root);
        child.scope.principals.push("bob".into());
        let snapshot = base_snapshot(vec![root, child], Vec::new(), Vec::new());
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Fail
        );
    }

    #[test]
    fn exact_declassification_receipt_qualifies_scope_expansion() {
        let root = source();
        let mut child = derivative(&root);
        child.scope.principals.push("bob".into());
        let output_scope_digest = compute_memory_scope_digest(&child.scope).unwrap();
        child.declassification = Some(MemoryDeclassificationReceipt {
            receipt_id: "declass-01".into(),
            input_artifact_ids: vec!["source".into()],
            input_payload_digests: vec![root.payload_digest.clone()],
            transform_digest: digest('c'),
            output_payload_digest: child.payload_digest.clone(),
            output_scope_digest,
            policy_digest: digest('1'),
            issued_at_ms: 25,
            expires_at_ms: 50,
            receipt_digest: digest('2'),
        });
        let snapshot = base_snapshot(vec![root, child], Vec::new(), Vec::new());
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Pass
        );
    }

    #[test]
    fn revocation_blocks_future_use_of_derived_descendant() {
        let root = source();
        let child = derivative(&root);
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 150,
            recorded_at_ms: 140,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let graph_digest =
            compute_memory_graph_digest(&[root.clone(), child.clone()]).unwrap();
        let closure = MemoryRevocationClosure {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            descendant_artifact_ids: vec!["summary".into()],
            graph_digest,
            complete: true,
            closure_receipt_digest: digest('6'),
        };
        let mut snapshot = base_snapshot(vec![root, child.clone()], vec![event], vec![closure]);
        snapshot.uses.push(use_event(
            "use-after-revoke",
            &child,
            &snapshot,
            200,
            MemoryUseDisposition::BlockedByRevocation,
        ));
        snapshot.snapshot_digest =
            compute_transitive_memory_snapshot_digest(&snapshot).unwrap();
        seal_uses(&mut snapshot);
        let report = qualify_transitive_memory_snapshot(&snapshot);
        assert_eq!(report.status, TransitiveMemoryStatus::Pass);
        assert_eq!(report.revoked_descendant_count, 1);
        assert_eq!(report.blocked_use_count, 1);
    }

    #[test]
    fn historical_use_before_revocation_remains_valid() {
        let root = source();
        let child = derivative(&root);
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 150,
            recorded_at_ms: 140,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let graph_digest =
            compute_memory_graph_digest(&[root.clone(), child.clone()]).unwrap();
        let closure = MemoryRevocationClosure {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            descendant_artifact_ids: vec!["summary".into()],
            graph_digest,
            complete: true,
            closure_receipt_digest: digest('6'),
        };
        let mut snapshot = base_snapshot(vec![root, child.clone()], vec![event], vec![closure]);
        snapshot.uses.push(use_event(
            "historical-use",
            &child,
            &snapshot,
            100,
            MemoryUseDisposition::Consumed,
        ));
        snapshot.snapshot_digest =
            compute_transitive_memory_snapshot_digest(&snapshot).unwrap();
        seal_uses(&mut snapshot);
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Pass
        );
    }

    #[test]
    fn historical_use_keeps_its_event_time_snapshot_reference() {
        let root = source();
        let child = derivative(&root);
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 150,
            recorded_at_ms: 140,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let graph_digest =
            compute_memory_graph_digest(&[root.clone(), child.clone()]).unwrap();
        let closure = MemoryRevocationClosure {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            descendant_artifact_ids: vec!["summary".into()],
            graph_digest,
            complete: true,
            closure_receipt_digest: digest('6'),
        };
        let mut snapshot = base_snapshot(vec![root, child.clone()], vec![event], vec![closure]);
        snapshot.uses.push(use_event(
            "historical-use",
            &child,
            &snapshot,
            100,
            MemoryUseDisposition::Consumed,
        ));
        // This is a separately archived pre-revocation policy snapshot.
        snapshot.uses[0].snapshot_digest = digest('a');
        let report = qualify_transitive_memory_snapshot(&snapshot);
        assert_eq!(report.status, TransitiveMemoryStatus::Pass);
        assert_eq!(report.consumed_use_count, 1);
    }

    #[test]
    fn revocation_effective_before_observation_does_not_rewrite_prior_use() {
        let root = source();
        let child = derivative(&root);
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 100,
            recorded_at_ms: 200,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let graph_digest =
            compute_memory_graph_digest(&[root.clone(), child.clone()]).unwrap();
        let closure = MemoryRevocationClosure {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            descendant_artifact_ids: vec!["summary".into()],
            graph_digest,
            complete: true,
            closure_receipt_digest: digest('6'),
        };
        let mut snapshot = base_snapshot(vec![root, child.clone()], vec![event], vec![closure]);
        snapshot.uses.push(use_event(
            "before-observation",
            &child,
            &snapshot,
            150,
            MemoryUseDisposition::Consumed,
        ));
        snapshot.uses.push(use_event(
            "after-observation",
            &child,
            &snapshot,
            250,
            MemoryUseDisposition::BlockedByRevocation,
        ));
        snapshot.uses[0].snapshot_digest = digest('a');
        snapshot.uses[1].snapshot_digest = digest('b');
        let report = qualify_transitive_memory_snapshot(&snapshot);
        assert_eq!(report.status, TransitiveMemoryStatus::Pass);
        assert_eq!(report.consumed_use_count, 1);
        assert_eq!(report.blocked_use_count, 1);
    }

    #[test]
    fn derivative_created_after_known_revocation_fails() {
        let root = source();
        let mut child = derivative(&root);
        child.created_at_ms = 200;
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 150,
            recorded_at_ms: 140,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let snapshot = base_snapshot(vec![root, child], vec![event], Vec::new());
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Fail
        );
    }

    #[test]
    fn multi_source_derivative_must_fit_every_parent_scope() {
        let root = source();
        let mut second = source();
        second.artifact_id = "source-2".into();
        second.payload_digest = digest('8');
        second.scope.entities = vec!["entity-2".into()];
        let mut child = derivative(&root);
        child.artifact_id = "composite".into();
        child.parent_artifact_ids = vec!["source".into(), "source-2".into()];
        child.payload_digest = digest('9');
        let snapshot = base_snapshot(vec![root, second, child], Vec::new(), Vec::new());
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Fail
        );
    }

    #[test]
    fn incomplete_revocation_closure_is_indeterminate() {
        let root = source();
        let child = derivative(&root);
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 150,
            recorded_at_ms: 140,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let graph_digest =
            compute_memory_graph_digest(&[root.clone(), child.clone()]).unwrap();
        let closure = MemoryRevocationClosure {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            descendant_artifact_ids: vec!["summary".into()],
            graph_digest,
            complete: false,
            closure_receipt_digest: digest('6'),
        };
        let snapshot = base_snapshot(vec![root, child], vec![event], vec![closure]);
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Indeterminate
        );
    }

    #[test]
    fn unknown_use_disposition_is_indeterminate() {
        let root = source();
        let child = derivative(&root);
        let mut snapshot = base_snapshot(vec![root.clone(), child.clone()], Vec::new(), Vec::new());
        snapshot.uses.push(use_event(
            "unknown-use",
            &child,
            &snapshot,
            100,
            MemoryUseDisposition::Unknown,
        ));
        snapshot.snapshot_digest =
            compute_transitive_memory_snapshot_digest(&snapshot).unwrap();
        seal_uses(&mut snapshot);
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Indeterminate
        );
    }

    #[test]
    fn revocation_closure_cannot_omit_a_descendant() {
        let root = source();
        let child = derivative(&root);
        let grandchild = MemoryArtifact {
            artifact_id: "report".into(),
            payload_digest: digest('7'),
            parent_artifact_ids: vec!["summary".into()],
            transform_digest: Some(digest('8')),
            derivation_receipt_digest: Some(digest('9')),
            declassification: None,
            scope: scope("alice", "project-a", "entity-1", 800),
            created_at_ms: 40,
            lineage_complete: true,
        };
        let event = MemoryRevocationEvent {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            effective_at_ms: 150,
            recorded_at_ms: 140,
            policy_epoch_digest: digest('3'),
            authority_receipt_digest: digest('4'),
            event_receipt_digest: digest('5'),
        };
        let graph_digest = compute_memory_graph_digest(&[
            root.clone(),
            child.clone(),
            grandchild.clone(),
        ])
        .unwrap();
        let closure = MemoryRevocationClosure {
            event_id: "revoke-01".into(),
            source_artifact_id: "source".into(),
            descendant_artifact_ids: vec!["summary".into()],
            graph_digest,
            complete: true,
            closure_receipt_digest: digest('6'),
        };
        let snapshot =
            base_snapshot(vec![root, child, grandchild], vec![event], vec![closure]);
        assert_eq!(
            qualify_transitive_memory_snapshot(&snapshot).status,
            TransitiveMemoryStatus::Fail
        );
    }
}
