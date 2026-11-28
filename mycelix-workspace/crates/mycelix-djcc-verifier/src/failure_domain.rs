//! Structural failure-domain lineage checks for DJCC independence evidence.
//!
//! This module checks whether a declared lineage graph is internally valid,
//! covers the failure classes required by a named profile, and contains shared
//! material ancestors. It does not assert real-world independence: graph
//! provenance, lineage completeness, and domain declarations must be
//! authenticated by their owning systems before a root claim is established.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum FailureDomainClass {
    OrganizationOperator,
    AdministrativeControlPlane,
    SigningKeyCustody,
    CollectionDevice,
    CalibrationChain,
    NetworkTransport,
    UpstreamSource,
    TransformationPipeline,
    PowerSupply,
    EconomicControl,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FailureDomainDisposition {
    StructurallyDisjointWithinDeclaredGraph,
    CommonCauseDetected,
    LineageCoverageIncomplete,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineageDisclosureStatus {
    CompleteForDeclaredModel,
    Incomplete,
    Withheld,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineageAttestationBasis {
    IssuerDeclared,
    IndependentlyAudited,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineageAttestationPrecheck {
    ReadyForExternalTrustVerification,
    IndeterminateDisclosure,
    IndeterminateAssuranceBasis,
}

/// Canonical statement metadata to be authenticated by an existing COSE/SCITT
/// issuer. Digest values are supplied by their owning systems; this type does
/// not calculate hashes, verify signatures, or authenticate the issuer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LineageAttestationV1 {
    pub attestation_ref: String,
    pub issuer_ref: String,
    pub issuer_key_ref: String,
    pub issuer_key_epoch: u64,
    pub issuer_ref: String,
    pub issuer_key_ref: String,
    pub issuer_key_epoch: u64,
    pub evidence_ref: String,
    pub scope_digest: [u8; 32],
    pub graph_digest: [u8; 32],
    pub profile_ref: String,
    pub profile_digest: [u8; 32],
    pub verifier_frontier_digest: [u8; 32],
    pub trust_policy_ref: String,
    pub trust_policy_digest: [u8; 32],
    pub identity_registry_digest: [u8; 32],
    pub assurance_basis_ref: String,
    pub statement_digest: [u8; 32],
    pub issued_at_unix_seconds: u64,
    pub valid_from_unix_seconds: u64,
    pub valid_until_unix_seconds: u64,
    pub disclosure_status: LineageDisclosureStatus,
    pub assurance_basis: LineageAttestationBasis,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LineageAttestationContextV1 {
    pub evidence_ref: String,
    pub scope_digest: [u8; 32],
    pub graph_digest: [u8; 32],
    pub profile_ref: String,
    pub profile_digest: [u8; 32],
    pub verifier_frontier_digest: [u8; 32],
    pub trust_policy_ref: String,
    pub trust_policy_digest: [u8; 32],
    pub identity_registry_digest: [u8; 32],
    pub assurance_basis_ref: String,
    /// Trusted evaluation time associated with the accepted verifier frontier.
    /// This is used for valid-time evaluation, not causal ordering.
    pub accepted_at_unix_seconds: u64,
    pub require_independent_audit: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FailureDomainNode {
    /// Globally stable identifier for this failure domain, not a display label.
    pub domain_ref: String,
    pub class: FailureDomainClass,
    /// Parent domains on which this domain materially depends.
    pub parent_refs: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EvidenceChannel {
    pub evidence_ref: String,
    /// Entry points into the lineage DAG for this evidence channel.
    pub root_domain_refs: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FailureDomainGraph {
    /// Nodes sorted by domain_ref; every node must be reachable from a channel.
    pub nodes: Vec<FailureDomainNode>,
    /// Channels sorted by evidence_ref; at least two are required.
    pub channels: Vec<EvidenceChannel>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IndependenceProfile {
    pub profile_ref: String,
    /// Each evidence channel must disclose an ancestor in every required class.
    pub required_classes: BTreeSet<FailureDomainClass>,
    /// A shared node of one of these classes is a material common cause.
    pub material_shared_classes: BTreeSet<FailureDomainClass>,
    /// Shared nodes in these classes are explicitly accepted as non-material
    /// under this profile. An unclassified shared class remains indeterminate.
    pub explicitly_non_material_shared_classes: BTreeSet<FailureDomainClass>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SharedDomainClassification {
    Material,
    ExplicitlyNonMaterial,
    Unclassified,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SharedLineageGroup {
    pub domain_ref: String,
    pub domain_class: FailureDomainClass,
    /// All evidence channels whose declared ancestry includes this domain.
    pub channel_refs: Vec<String>,
    pub classification: SharedDomainClassification,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IndependenceAssessment {
    pub disposition: FailureDomainDisposition,
    /// Shared domain IDs, not merely shared labels or classes.
    pub common_domain_refs: Vec<String>,
    /// Shared IDs whose class has not been classified by this profile.
    pub unclassified_shared_domain_refs: Vec<String>,
    /// Required classes absent from each channel's reachable lineage.
    pub missing_classes_by_channel: BTreeMap<String, Vec<FailureDomainClass>>,
    /// Compact groups identify every shared ancestor and all channels it affects.
    pub shared_lineage_groups: Vec<SharedLineageGroup>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FailureScenario {
    pub scenario_ref: String,
    /// Exact failure-domain identities assumed unavailable in this scenario.
    pub failed_domain_refs: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FailureScenarioOutcome {
    pub scenario_ref: String,
    pub affected_channels: Vec<String>,
    pub surviving_channels: Vec<String>,
    /// Survivors may still share common causes; this field preserves that fact.
    pub surviving_shared_lineage_groups: Vec<SharedLineageGroup>,
    pub minimum_survivor_floor: usize,
    pub meets_survivor_floor: bool,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FailureScenarioAssessment {
    /// Structural completeness/common-cause disposition of the selected profile.
    /// A passing survivor floor never promotes this disposition.
    pub profile_independence_disposition: FailureDomainDisposition,
    pub outcomes: Vec<FailureScenarioOutcome>,
    /// This applies only to the declared scenarios; it is not an exhaustive
    /// reliability or probability claim about unmodeled failures.
    pub all_declared_scenarios_meet_survivor_floor: bool,
}

fn valid_ref(value: &str) -> bool {
    !value.trim().is_empty() && value.len() <= 4096
}

fn write_attestation_string(buffer: &mut Vec<u8>, value: &str) -> Result<(), String> {
    if !valid_ref(value) {
        return Err("lineage attestation contains an empty or oversized identifier".to_owned());
    }
    let bytes = value.as_bytes();
    let length = u32::try_from(bytes.len())
        .map_err(|_| "lineage attestation string length exceeds u32".to_owned())?;
    buffer.extend_from_slice(&length.to_be_bytes());
    buffer.extend_from_slice(bytes);
    Ok(())
}

/// Encodes the DJCC lineage-attestation signing preimage V1.
///
/// Field order is fixed: domain separator; seven UTF-8 length-prefixed references;
/// key epoch; scope, graph, profile, frontier, trust-policy, identity-registry
/// and statement digests; issued/valid-from/valid-until times; disclosure and
/// assurance tags. All integers are unsigned big-endian. The resulting bytes
/// are a signing preimage only, not a verified COSE/SCITT receipt.
pub fn canonical_lineage_attestation_bytes_v1(
    attestation: &LineageAttestationV1,
) -> Result<Vec<u8>, String> {
    let mut output = Vec::new();
    write_attestation_string(&mut output, "MYCELIX-DJCC-LINEAGE-ATTESTATION-V1")?;
    for value in [
        attestation.attestation_ref.as_str(),
        attestation.issuer_ref.as_str(),
        attestation.issuer_key_ref.as_str(),
        attestation.evidence_ref.as_str(),
        context.issuer_ref.as_str(),
        context.issuer_key_ref.as_str(),
        context.assurance_basis_ref.as_str(),
        attestation.profile_ref.as_str(),
        attestation.trust_policy_ref.as_str(),
        attestation.assurance_basis_ref.as_str(),
    ] {
        write_attestation_string(&mut output, value)?;
    }
    output.extend_from_slice(&attestation.issuer_key_epoch.to_be_bytes());
    for digest in [
        &attestation.scope_digest,
        &attestation.graph_digest,
        &attestation.profile_digest,
        &attestation.verifier_frontier_digest,
        &attestation.trust_policy_digest,
        &attestation.identity_registry_digest,
        &attestation.statement_digest,
    ] {
        output.extend_from_slice(digest);
    }
    for timestamp in [
        attestation.issued_at_unix_seconds,
        attestation.valid_from_unix_seconds,
        attestation.valid_until_unix_seconds,
    ] {
        output.extend_from_slice(&timestamp.to_be_bytes());
    }
    let disclosure_tag = match attestation.disclosure_status {
        LineageDisclosureStatus::CompleteForDeclaredModel => 1,
        LineageDisclosureStatus::Incomplete => 2,
        LineageDisclosureStatus::Withheld => 3,
    };
    let assurance_tag = match attestation.assurance_basis {
        LineageAttestationBasis::IssuerDeclared => 1,
        LineageAttestationBasis::IndependentlyAudited => 2,
    };
    output.push(disclosure_tag);
    output.push(assurance_tag);
    Ok(output)
}

/// Checks structural binding and valid-time preconditions before an external
/// COSE/SCITT verifier is invoked.
///
/// ReadyForExternalTrustVerification means only that the canonical statement
/// is structurally bound to the expected issuer, key epoch, scope, graph,
/// profile, trust policy, identity-registry snapshot and accepted frontier,
/// and that its declared disclosure and assurance basis meet the profile.
/// It does not establish signature validity, issuer trust, log registration
/// or inclusion at the accepted frontier, audit authenticity, source-universe
/// completeness, or channel independence.
///
/// The self-reported issued_at timestamp is included in the signed statement
/// but is not causal evidence. Its inclusion by the named issuer/log at the
/// accepted verifier frontier must be proven by the external receipt verifier.
pub fn precheck_lineage_attestation_v1(
    attestation: &LineageAttestationV1,
    context: &LineageAttestationContextV1,
) -> Result<LineageAttestationPrecheck, String> {
    for value in [
        attestation.attestation_ref.as_str(),
        attestation.issuer_ref.as_str(),
        attestation.issuer_key_ref.as_str(),
        attestation.evidence_ref.as_str(),
        attestation.profile_ref.as_str(),
        attestation.trust_policy_ref.as_str(),
        attestation.assurance_basis_ref.as_str(),
        context.evidence_ref.as_str(),
        context.profile_ref.as_str(),
        context.trust_policy_ref.as_str(),
    ] {
        if !valid_ref(value) {
            return Err("lineage attestation context contains an empty or oversized reference".to_owned());
        }
    }
    if attestation.valid_from_unix_seconds >= attestation.valid_until_unix_seconds {
        return Err("lineage attestation validity interval is empty or reversed".to_owned());
    }
    if context.accepted_at_unix_seconds < attestation.valid_from_unix_seconds
        || context.accepted_at_unix_seconds >= attestation.valid_until_unix_seconds
    {
        return Err("lineage attestation is outside its validity interval at the accepted frontier".to_owned());
    }
    if attestation.issuer_ref != context.issuer_ref
        || attestation.issuer_key_ref != context.issuer_key_ref
        || attestation.issuer_key_epoch != context.issuer_key_epoch
        || attestation.assurance_basis_ref != context.assurance_basis_ref
        || attestation.evidence_ref != context.evidence_ref
        || attestation.scope_digest != context.scope_digest
        || attestation.graph_digest != context.graph_digest
        || attestation.profile_ref != context.profile_ref
        || attestation.profile_digest != context.profile_digest
        || attestation.verifier_frontier_digest != context.verifier_frontier_digest
        || attestation.trust_policy_ref != context.trust_policy_ref
        || attestation.trust_policy_digest != context.trust_policy_digest
        || attestation.identity_registry_digest != context.identity_registry_digest
    {
        return Err("lineage attestation does not bind the expected scope, graph, profile, frontier or trust context".to_owned());
    }
    if attestation.disclosure_status != LineageDisclosureStatus::CompleteForDeclaredModel {
        return Ok(LineageAttestationPrecheck::IndeterminateDisclosure);
    }
    if context.require_independent_audit
        && attestation.assurance_basis != LineageAttestationBasis::IndependentlyAudited
    {
        return Ok(LineageAttestationPrecheck::IndeterminateAssuranceBasis);
    }
    Ok(LineageAttestationPrecheck::ReadyForExternalTrustVerification)
}


fn strictly_sorted_unique<T: Ord>(values: &[T]) -> bool {
    values.windows(2).all(|pair| pair[0] < pair[1])
}

fn visit(
    domain_ref: &str,
    nodes: &BTreeMap<String, &FailureDomainNode>,
    active: &mut BTreeSet<String>,
    visited: &mut BTreeSet<String>,
) -> Result<(), String> {
    if visited.contains(domain_ref) {
        return Ok(());
    }
    if !active.insert(domain_ref.to_owned()) {
        return Err(format!("failure-domain lineage cycle at {domain_ref}"));
    }
    let node = nodes
        .get(domain_ref)
        .ok_or_else(|| format!("unknown failure-domain reference {domain_ref}"))?;
    for parent_ref in &node.parent_refs {
        visit(parent_ref, nodes, active, visited)?;
    }
    active.remove(domain_ref);
    visited.insert(domain_ref.to_owned());
    Ok(())
}

fn build_channel_lineages<'a>(
    graph: &'a FailureDomainGraph,
) -> Result<
    (
        BTreeMap<String, &'a FailureDomainNode>,
        BTreeMap<String, BTreeSet<String>>,
    ),
    String,
> {
    if graph.channels.len() < 2 {
        return Err("failure-domain graph requires at least two evidence channels".to_owned());
    }
    if graph.nodes.is_empty() {
        return Err("failure-domain graph is empty".to_owned());
    }

    let mut nodes = BTreeMap::new();
    let mut node_refs = Vec::with_capacity(graph.nodes.len());
    for node in &graph.nodes {
        if !valid_ref(&node.domain_ref) {
            return Err("failure-domain reference is empty or oversized".to_owned());
        }
        if !node.parent_refs.is_empty() && !strictly_sorted_unique(&node.parent_refs) {
            return Err(format!(
                "parent references for {} must be sorted and unique",
                node.domain_ref
            ));
        }
        if nodes.insert(node.domain_ref.clone(), node).is_some() {
            return Err(format!("duplicate failure-domain {}", node.domain_ref));
        }
        node_refs.push(node.domain_ref.as_str());
    }
    if !strictly_sorted_unique(&node_refs) {
        return Err("failure-domain nodes must be sorted by unique domain_ref".to_owned());
    }

    let mut channel_refs = Vec::with_capacity(graph.channels.len());
    for channel in &graph.channels {
        if !valid_ref(&channel.evidence_ref) || channel.root_domain_refs.is_empty() {
            return Err(
                "evidence channel requires an identifier and at least one lineage root".to_owned(),
            );
        }
        if !strictly_sorted_unique(&channel.root_domain_refs) {
            return Err(format!(
                "lineage roots for {} must be sorted and unique",
                channel.evidence_ref
            ));
        }
        channel_refs.push(channel.evidence_ref.as_str());
    }
    if !strictly_sorted_unique(&channel_refs) {
        return Err("evidence channels must be sorted by unique evidence_ref".to_owned());
    }

    for node in &graph.nodes {
        for parent_ref in &node.parent_refs {
            if !nodes.contains_key(parent_ref) {
                return Err(format!(
                    "{} references missing parent {}",
                    node.domain_ref, parent_ref
                ));
            }
        }
    }

    let mut channel_lineage = BTreeMap::new();
    for channel in &graph.channels {
        let mut lineage = BTreeSet::new();
        let mut active = BTreeSet::new();
        for root in &channel.root_domain_refs {
            visit(root, &nodes, &mut active, &mut lineage)?;
        }
        channel_lineage.insert(channel.evidence_ref.clone(), lineage);
    }

    let reachable: BTreeSet<_> = channel_lineage
        .values()
        .flat_map(|lineage| lineage.iter().cloned())
        .collect();
    let unreachable: Vec<_> = nodes
        .keys()
        .filter(|domain_ref| !reachable.contains(*domain_ref))
        .cloned()
        .collect();
    if !unreachable.is_empty() {
        return Err(format!(
            "failure-domain graph contains unreachable nodes: {}",
            unreachable.join(", ")
        ));
    }

    Ok((nodes, channel_lineage))
}

fn shared_lineage_groups(
    channels: &[EvidenceChannel],
    nodes: &BTreeMap<String, &FailureDomainNode>,
    channel_lineage: &BTreeMap<String, BTreeSet<String>>,
    profile: &IndependenceProfile,
) -> Result<Vec<SharedLineageGroup>, String> {
    // Invert the lineage relation once. This is O(total lineage references) for
    // disjoint graphs and avoids materializing an O(channel_count squared) matrix.
    let mut channels_by_domain: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for channel in channels {
        let lineage = channel_lineage
            .get(&channel.evidence_ref)
            .ok_or_else(|| format!("lineage for {} is missing", channel.evidence_ref))?;
        for domain_ref in lineage {
            channels_by_domain
                .entry(domain_ref.clone())
                .or_default()
                .push(channel.evidence_ref.clone());
        }
    }

    let mut groups = Vec::new();
    for (domain_ref, channel_refs) in channels_by_domain {
        if channel_refs.len() < 2 {
            continue;
        }
        let node = nodes
            .get(&domain_ref)
            .ok_or_else(|| format!("shared domain {domain_ref} is missing"))?;
        let classification = if profile.material_shared_classes.contains(&node.class) {
            SharedDomainClassification::Material
        } else if profile
            .explicitly_non_material_shared_classes
            .contains(&node.class)
        {
            SharedDomainClassification::ExplicitlyNonMaterial
        } else {
            SharedDomainClassification::Unclassified
        };
        groups.push(SharedLineageGroup {
            domain_ref,
            domain_class: node.class,
            channel_refs,
            classification,
        });
    }
    Ok(groups)
}

/// Evaluates an explicit deterministic fault-injection suite against the declared lineage DAG.
///
/// This reports which evidence channels are affected by each named failure scenario and
/// how many remain. It does not estimate failure probabilities, prove the scenario set is
/// exhaustive, or claim that the surviving channels are mutually independent.
pub fn assess_failure_scenarios(
    graph: &FailureDomainGraph,
    profile: &IndependenceProfile,
    scenarios: &[FailureScenario],
    minimum_survivor_floor: usize,
) -> Result<FailureScenarioAssessment, String> {
    let profile_assessment = assess_independence(graph, profile)?;
    let (nodes, channel_lineage) = build_channel_lineages(graph)?;
    if scenarios.is_empty() {
        return Err("failure scenario suite must not be empty".to_owned());
    }
    if minimum_survivor_floor == 0 || minimum_survivor_floor > graph.channels.len() {
        return Err("minimum survivor floor is outside the evidence-channel count".to_owned());
    }

    let mut scenario_refs = Vec::with_capacity(scenarios.len());
    for scenario in scenarios {
        if !valid_ref(&scenario.scenario_ref) || scenario.failed_domain_refs.is_empty() {
            return Err("failure scenario needs an identifier and at least one failed domain".to_owned());
        }
        if !strictly_sorted_unique(&scenario.failed_domain_refs) {
            return Err(format!(
                "failed domain references for {} must be sorted and unique",
                scenario.scenario_ref
            ));
        }
        for failed_ref in &scenario.failed_domain_refs {
            if !nodes.contains_key(failed_ref) {
                return Err(format!(
                    "failure scenario {} references unknown domain {}",
                    scenario.scenario_ref, failed_ref
                ));
            }
        }
        scenario_refs.push(scenario.scenario_ref.as_str());
    }
    if !strictly_sorted_unique(&scenario_refs) {
        return Err("failure scenarios must be sorted by unique scenario_ref".to_owned());
    }

    let mut outcomes = Vec::with_capacity(scenarios.len());
    for scenario in scenarios {
        let failed: BTreeSet<_> = scenario.failed_domain_refs.iter().cloned().collect();
        let mut affected_channels = Vec::new();
        let mut surviving_channels = Vec::new();
        for channel in &graph.channels {
            let lineage = channel_lineage
                .get(&channel.evidence_ref)
                .ok_or_else(|| format!("lineage for {} is missing", channel.evidence_ref))?;
            if lineage.iter().any(|domain_ref| failed.contains(domain_ref)) {
                affected_channels.push(channel.evidence_ref.clone());
            } else {
                surviving_channels.push(channel.evidence_ref.clone());
            }
        }
        let survivor_channels: Vec<_> = graph
            .channels
            .iter()
            .filter(|channel| surviving_channels.contains(&channel.evidence_ref))
            .cloned()
            .collect();
        let survivor_lineage: BTreeMap<_, _> = survivor_channels
            .iter()
            .filter_map(|channel| {
                channel_lineage
                    .get(&channel.evidence_ref)
                    .map(|lineage| (channel.evidence_ref.clone(), lineage.clone()))
            })
            .collect();
        let surviving_shared_lineage_groups = shared_lineage_groups(
            &survivor_channels,
            &nodes,
            &survivor_lineage,
            profile,
        )?;
        let meets_survivor_floor = surviving_channels.len() >= minimum_survivor_floor;
        outcomes.push(FailureScenarioOutcome {
            scenario_ref: scenario.scenario_ref.clone(),
            affected_channels,
            surviving_channels,
            surviving_shared_lineage_groups,
            minimum_survivor_floor,
            meets_survivor_floor,
        });
    }
    let all_declared_scenarios_meet_survivor_floor =
        outcomes.iter().all(|outcome| outcome.meets_survivor_floor);
    Ok(FailureScenarioAssessment {
        profile_independence_disposition: profile_assessment.disposition,
        outcomes,
        all_declared_scenarios_meet_survivor_floor,
    })
}

/// Checks structural disjointness under an explicit profile.
///
/// A StructurallyDisjointWithinDeclaredGraph result is not itself an
/// Independence=EstablishedWithinProfile receipt. It is only a structural
/// negative-control capable fact; trusted lineage attestations and threat-model
/// coverage must be checked separately by the source-trust owner.
pub fn assess_independence(
    graph: &FailureDomainGraph,
    profile: &IndependenceProfile,
) -> Result<IndependenceAssessment, String> {
    if !valid_ref(&profile.profile_ref) {
        return Err("independence profile reference is empty or oversized".to_owned());
    }
    if profile.required_classes.is_empty() {
        return Err(
            "independence profile must require at least one failure-domain class".to_owned(),
        );
    }
    if profile.material_shared_classes.is_empty() {
        return Err("material shared-domain classes must not be empty".to_owned());
    }
    if !profile
        .material_shared_classes
        .is_disjoint(&profile.explicitly_non_material_shared_classes)
    {
        return Err(
            "a shared-domain class cannot be both material and explicitly non-material"
                .to_owned(),
        );
    }
    let (nodes, channel_lineage) = build_channel_lineages(graph)?;
    let mut missing_classes_by_channel = BTreeMap::new();
    for channel in &graph.channels {
        let lineage = channel_lineage
            .get(&channel.evidence_ref)
            .ok_or_else(|| format!("lineage for {} is missing", channel.evidence_ref))?;
        let classes: BTreeSet<_> = lineage
            .iter()
            .filter_map(|domain_ref| nodes.get(domain_ref))
            .map(|node| node.class)
            .collect();
        let missing: Vec<_> = profile
            .required_classes
            .difference(&classes)
            .copied()
            .collect();
        if !missing.is_empty() {
            missing_classes_by_channel.insert(channel.evidence_ref.clone(), missing);
        }
    }

    let shared_lineage_groups =
        shared_lineage_groups(&graph.channels, &nodes, &channel_lineage, profile)?;
    let common_domain_refs: Vec<_> = shared_lineage_groups
        .iter()
        .filter(|group| group.classification == SharedDomainClassification::Material)
        .map(|group| group.domain_ref.clone())
        .collect();
    let unclassified_shared_domain_refs: Vec<_> = shared_lineage_groups
        .iter()
        .filter(|group| group.classification == SharedDomainClassification::Unclassified)
        .map(|group| group.domain_ref.clone())
        .collect();

    let disposition = if !common_domain_refs.is_empty() {
        FailureDomainDisposition::CommonCauseDetected
    } else if !unclassified_shared_domain_refs.is_empty()
        || !missing_classes_by_channel.is_empty()
    {
        FailureDomainDisposition::LineageCoverageIncomplete
    } else {
        FailureDomainDisposition::StructurallyDisjointWithinDeclaredGraph
    };

    Ok(IndependenceAssessment {
        disposition,
        common_domain_refs,
        unclassified_shared_domain_refs,
        missing_classes_by_channel,
        shared_lineage_groups,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> IndependenceProfile {
        IndependenceProfile {
            profile_ref: "profile:independence:operational-v1".to_owned(),
            required_classes: BTreeSet::from([
                FailureDomainClass::OrganizationOperator,
                FailureDomainClass::SigningKeyCustody,
                FailureDomainClass::UpstreamSource,
            ]),
            material_shared_classes: BTreeSet::from([
                FailureDomainClass::OrganizationOperator,
                FailureDomainClass::SigningKeyCustody,
                FailureDomainClass::UpstreamSource,
            ]),
            explicitly_non_material_shared_classes: BTreeSet::new(),
        }
    }

    fn node(
        domain_ref: &str,
        class: FailureDomainClass,
        parent_refs: &[&str],
    ) -> FailureDomainNode {
        FailureDomainNode {
            domain_ref: domain_ref.to_owned(),
            class,
            parent_refs: parent_refs.iter().map(|value| (*value).to_owned()).collect(),
        }
    }

    fn channel(evidence_ref: &str, root: &str) -> EvidenceChannel {
        EvidenceChannel {
            evidence_ref: evidence_ref.to_owned(),
            root_domain_refs: vec![root.to_owned()],
        }
    }

    fn disjoint_graph() -> FailureDomainGraph {
        FailureDomainGraph {
            nodes: vec![
                node(
                    "a-device",
                    FailureDomainClass::CollectionDevice,
                    &["a-feed", "a-key", "a-operator"],
                ),
                node("a-feed", FailureDomainClass::UpstreamSource, &[]),
                node("a-key", FailureDomainClass::SigningKeyCustody, &[]),
                node("a-operator", FailureDomainClass::OrganizationOperator, &[]),
                node(
                    "b-device",
                    FailureDomainClass::CollectionDevice,
                    &["b-feed", "b-key", "b-operator"],
                ),
                node("b-feed", FailureDomainClass::UpstreamSource, &[]),
                node("b-key", FailureDomainClass::SigningKeyCustody, &[]),
                node("b-operator", FailureDomainClass::OrganizationOperator, &[]),
            ],
            channels: vec![channel("evidence:a", "a-device"), channel("evidence:b", "b-device")],
        }
    }

    #[test]
    fn disjoint_declared_lineages_are_structurally_disjoint_only() {
        let result = assess_independence(&disjoint_graph(), &profile()).unwrap();
        assert_eq!(
            result.disposition,
            FailureDomainDisposition::StructurallyDisjointWithinDeclaredGraph
        );
        assert!(result.common_domain_refs.is_empty());
        assert!(result.missing_classes_by_channel.is_empty());
    }

    #[test]
    fn shared_upstream_source_is_detected_as_a_common_cause() {
        let mut graph = disjoint_graph();
        graph.nodes.retain(|node| node.domain_ref != "b-feed");
        graph
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "b-device")
            .unwrap()
            .parent_refs = vec![
            "a-feed".to_owned(),
            "b-key".to_owned(),
            "b-operator".to_owned(),
        ];
        let result = assess_independence(&graph, &profile()).unwrap();
        assert_eq!(result.disposition, FailureDomainDisposition::CommonCauseDetected);
        assert_eq!(result.common_domain_refs, vec!["a-feed".to_owned()]);
        assert_eq!(result.shared_lineage_groups.len(), 1);
        assert_eq!(
            result.shared_lineage_groups[0].domain_ref,
            "a-feed".to_owned()
        );
        assert_eq!(
            result.shared_lineage_groups[0].channel_refs,
            vec!["evidence:a".to_owned(), "evidence:b".to_owned()]
        );
        assert_eq!(
            result.shared_lineage_groups[0].classification,
            SharedDomainClassification::Material
        );
    }

    #[test]
    fn missing_failure_class_is_not_promoted_to_independence() {
        let mut graph = disjoint_graph();
        graph.nodes.retain(|node| node.domain_ref != "b-key");
        graph.nodes.iter_mut().find(|node| node.domain_ref == "b-device").unwrap().parent_refs =
            vec!["b-feed".to_owned(), "b-operator".to_owned()];
        let result = assess_independence(&graph, &profile()).unwrap();
        assert_eq!(result.disposition, FailureDomainDisposition::LineageCoverageIncomplete);
        assert!(result.missing_classes_by_channel.contains_key("evidence:b"));
    }

    #[test]
    fn shared_non_material_domain_class_is_not_automatically_a_common_cause() {
        let mut graph = disjoint_graph();
        graph.nodes.push(node("shared-network", FailureDomainClass::NetworkTransport, &[]));
        graph.nodes.sort_by(|a, b| a.domain_ref.cmp(&b.domain_ref));
        graph
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "a-device")
            .unwrap()
            .parent_refs
            .push("shared-network".to_owned());
        graph
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "a-device")
            .unwrap()
            .parent_refs
            .sort();
        graph
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "b-device")
            .unwrap()
            .parent_refs
            .push("shared-network".to_owned());
        graph
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "b-device")
            .unwrap()
            .parent_refs
            .sort();
        let mut selected_profile = profile();
        selected_profile.explicitly_non_material_shared_classes =
            BTreeSet::from([FailureDomainClass::NetworkTransport]);
        let result = assess_independence(&graph, &selected_profile).unwrap();
        assert_eq!(
            result.disposition,
            FailureDomainDisposition::StructurallyDisjointWithinDeclaredGraph
        );
    }

    #[test]
    fn material_shared_class_is_detected_even_when_not_required_on_every_channel() {
        let mut graph = disjoint_graph();
        graph.nodes.push(node("shared-network", FailureDomainClass::NetworkTransport, &[]));
        graph.nodes.sort_by(|a, b| a.domain_ref.cmp(&b.domain_ref));
        for evidence in ["a-device", "b-device"] {
            let device = graph.nodes.iter_mut().find(|item| item.domain_ref == evidence).unwrap();
            device.parent_refs.push("shared-network".to_owned());
            device.parent_refs.sort();
        }
        let mut selected_profile = profile();
        selected_profile
            .material_shared_classes
            .insert(FailureDomainClass::NetworkTransport);
        let result = assess_independence(&graph, &selected_profile).unwrap();
        assert_eq!(result.disposition, FailureDomainDisposition::CommonCauseDetected);
        assert_eq!(
            result.common_domain_refs,
            vec!["shared-network".to_owned()]
        );
    }

    #[test]
    fn unclassified_shared_domain_remains_incomplete() {
        let mut graph = disjoint_graph();
        graph.nodes.push(node("shared-network", FailureDomainClass::NetworkTransport, &[]));
        graph.nodes.sort_by(|a, b| a.domain_ref.cmp(&b.domain_ref));
        for evidence in ["a-device", "b-device"] {
            let device = graph.nodes.iter_mut().find(|item| item.domain_ref == evidence).unwrap();
            device.parent_refs.push("shared-network".to_owned());
            device.parent_refs.sort();
        }
        let result = assess_independence(&graph, &profile()).unwrap();
        assert_eq!(
            result.disposition,
            FailureDomainDisposition::LineageCoverageIncomplete
        );
        assert_eq!(
            result.unclassified_shared_domain_refs,
            vec!["shared-network".to_owned()]
        );
    }

    #[test]
    fn malformed_graphs_fail_closed_on_cycles_and_missing_parents() {
        let mut cycle = disjoint_graph();
        cycle
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "a-operator")
            .unwrap()
            .parent_refs = vec!["a-device".to_owned()];
        let result = assess_independence(&cycle, &profile());
        assert!(result.unwrap_err().contains("cycle"));

        let mut missing = disjoint_graph();
        missing
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "a-device")
            .unwrap()
            .parent_refs
            .push("not-registered".to_owned());
        missing
            .nodes
            .iter_mut()
            .find(|node| node.domain_ref == "a-device")
            .unwrap()
            .parent_refs
            .sort();
        let result = assess_independence(&missing, &profile());
        assert!(result.unwrap_err().contains("missing parent"));
    }

    #[test]
    fn profile_cannot_classify_one_shared_class_as_both_material_and_non_material() {
        let mut selected_profile = profile();
        selected_profile
            .explicitly_non_material_shared_classes
            .insert(FailureDomainClass::UpstreamSource);
        assert!(
            assess_independence(&disjoint_graph(), &selected_profile)
                .unwrap_err()
                .contains("both material and explicitly non-material")
        );
    }

    #[test]
    fn unreachable_domain_nodes_are_rejected_instead_of_ignored() {
        let mut graph = disjoint_graph();
        graph
            .nodes
            .push(node("unused-domain", FailureDomainClass::EconomicControl, &[]));
        graph.nodes.sort_by(|a, b| a.domain_ref.cmp(&b.domain_ref));
        assert!(
            assess_independence(&graph, &profile())
                .unwrap_err()
                .contains("unreachable nodes")
        );
    }

    fn three_channel_shared_feed_graph() -> FailureDomainGraph {
        FailureDomainGraph {
            nodes: vec![
                node(
                    "a-device",
                    FailureDomainClass::CollectionDevice,
                    &["a-feed", "a-key", "a-operator"],
                ),
                node("a-feed", FailureDomainClass::UpstreamSource, &[]),
                node("a-key", FailureDomainClass::SigningKeyCustody, &[]),
                node("a-operator", FailureDomainClass::OrganizationOperator, &[]),
                node(
                    "b-device",
                    FailureDomainClass::CollectionDevice,
                    &["a-feed", "b-key", "b-operator"],
                ),
                node("b-key", FailureDomainClass::SigningKeyCustody, &[]),
                node("b-operator", FailureDomainClass::OrganizationOperator, &[]),
                node(
                    "c-device",
                    FailureDomainClass::CollectionDevice,
                    &["c-feed", "c-key", "c-operator"],
                ),
                node("c-feed", FailureDomainClass::UpstreamSource, &[]),
                node("c-key", FailureDomainClass::SigningKeyCustody, &[]),
                node("c-operator", FailureDomainClass::OrganizationOperator, &[]),
            ],
            channels: vec![
                channel("evidence:a", "a-device"),
                channel("evidence:b", "b-device"),
                channel("evidence:c", "c-device"),
            ],
        }
    }

    #[test]
    fn explicit_failure_scenarios_reveal_quorum_fragility_from_shared_ancestors() {
        let graph = three_channel_shared_feed_graph();
        let scenarios = vec![
            FailureScenario {
                scenario_ref: "scenario:a-device-loss".to_owned(),
                failed_domain_refs: vec!["a-device".to_owned()],
            },
            FailureScenario {
                scenario_ref: "scenario:shared-feed-loss".to_owned(),
                failed_domain_refs: vec!["a-feed".to_owned()],
            },
        ];
        let result = assess_failure_scenarios(&graph, &profile(), &scenarios, 2).unwrap();
        assert!(!result.all_declared_scenarios_meet_survivor_floor);
        assert_eq!(
            result.outcomes[0].surviving_channels,
            vec!["evidence:b".to_owned(), "evidence:c".to_owned()]
        );
        assert!(result.outcomes[0].meets_survivor_floor);
        assert_eq!(
            result.outcomes[1].affected_channels,
            vec!["evidence:a".to_owned(), "evidence:b".to_owned()]
        );
        assert_eq!(
            result.outcomes[1].surviving_channels,
            vec!["evidence:c".to_owned()]
        );
        assert!(!result.outcomes[1].meets_survivor_floor);
    }

    #[test]
    fn survivor_floor_does_not_promote_incomplete_lineage() {
        let graph = disjoint_graph();
        let mut incomplete_profile = profile();
        incomplete_profile
            .required_classes
            .insert(FailureDomainClass::EconomicControl);
        let scenarios = vec![FailureScenario {
            scenario_ref: "scenario:single-local-loss".to_owned(),
            failed_domain_refs: vec!["a-device".to_owned()],
        }];
        let result =
            assess_failure_scenarios(&graph, &incomplete_profile, &scenarios, 1).unwrap();
        assert!(result.all_declared_scenarios_meet_survivor_floor);
        assert_eq!(
            result.profile_independence_disposition,
            FailureDomainDisposition::LineageCoverageIncomplete
        );
    }

    #[test]
    fn failure_scenarios_reject_unknown_domains_empty_suites_and_invalid_floors() {
        let graph = disjoint_graph();
        let unknown = vec![FailureScenario {
            scenario_ref: "scenario:unknown".to_owned(),
            failed_domain_refs: vec!["not-in-graph".to_owned()],
        }];
        assert!(
            assess_failure_scenarios(&graph, &profile(), &unknown, 1)
                .unwrap_err()
                .contains("unknown domain")
        );
        let valid = vec![FailureScenario {
            scenario_ref: "scenario:valid".to_owned(),
            failed_domain_refs: vec!["a-device".to_owned()],
        }];
        assert!(assess_failure_scenarios(&graph, &profile(), &[], 1).is_err());
        assert!(assess_failure_scenarios(&graph, &profile(), &valid, 0).is_err());
        assert!(assess_failure_scenarios(&graph, &profile(), &valid, 3).is_err());
    }

    #[test]
    fn surviving_channels_keep_their_pairwise_common_cause_details() {
        let graph = three_channel_shared_feed_graph();
        let scenarios = vec![FailureScenario {
            scenario_ref: "scenario:independent-local-loss".to_owned(),
            failed_domain_refs: vec!["c-device".to_owned()],
        }];
        let result = assess_failure_scenarios(&graph, &profile(), &scenarios, 2).unwrap();
        let survivors = &result.outcomes[0].surviving_shared_lineage_groups;
        assert_eq!(survivors.len(), 1);
        assert_eq!(survivors[0].domain_ref, "a-feed".to_owned());
        assert_eq!(
            survivors[0].channel_refs,
            vec!["evidence:a".to_owned(), "evidence:b".to_owned()]
        );
    }

    fn decode_attestation_hex(value: &str) -> Vec<u8> {
        assert_eq!(value.len() % 2, 0, "hex input has odd length");
        value
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let high = char::from(pair[0]).to_digit(16).expect("hex digit");
                let low = char::from(pair[1]).to_digit(16).expect("hex digit");
                ((high << 4) | low) as u8
            })
            .collect()
    }

    fn digest_from_hex(value: &str) -> [u8; 32] {
        decode_attestation_hex(value)
            .try_into()
            .expect("lineage attestation digest must be 32 bytes")
    }

    #[test]
    fn shared_go_rust_lineage_attestation_v1_known_answer_vector_matches() {
        const VECTORS: &str = include_str!(
            "../../../../tools/economic/evidence-execution-composition/LINEAGE_ATTESTATION_V1_KAT.tsv"
        );
        let mut seen = 0;
        for (line_index, line) in VECTORS.lines().enumerate() {
            let line = line.trim_end_matches('\r');
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields: Vec<_> = line.split('\t').collect();
            assert_eq!(fields.len(), 22, "lineage KAT line {}", line_index + 1);
            let parse_u64 = |index: usize| -> u64 {
                fields[index]
                    .parse()
                    .expect("lineage attestation integer must be u64")
            };
            let disclosure_status = match fields[19] {
                "complete" => LineageDisclosureStatus::CompleteForDeclaredModel,
                "incomplete" => LineageDisclosureStatus::Incomplete,
                "withheld" => LineageDisclosureStatus::Withheld,
                other => panic!("unknown lineage disclosure status {other}"),
            };
            let assurance_basis = match fields[20] {
                "issuer-declared" => LineageAttestationBasis::IssuerDeclared,
                "independently-audited" => LineageAttestationBasis::IndependentlyAudited,
                other => panic!("unknown lineage assurance basis {other}"),
            };
            let attestation = LineageAttestationV1 {
                attestation_ref: fields[1].to_owned(),
                issuer_ref: fields[2].to_owned(),
                issuer_key_ref: fields[3].to_owned(),
                issuer_key_epoch: parse_u64(4),
                evidence_ref: fields[5].to_owned(),
                scope_digest: digest_from_hex(fields[6]),
                graph_digest: digest_from_hex(fields[7]),
                profile_ref: fields[8].to_owned(),
                profile_digest: digest_from_hex(fields[9]),
                verifier_frontier_digest: digest_from_hex(fields[10]),
                trust_policy_ref: fields[11].to_owned(),
                trust_policy_digest: digest_from_hex(fields[12]),
                identity_registry_digest: digest_from_hex(fields[13]),
                assurance_basis_ref: fields[14].to_owned(),
                statement_digest: digest_from_hex(fields[15]),
                issued_at_unix_seconds: parse_u64(16),
                valid_from_unix_seconds: parse_u64(17),
                valid_until_unix_seconds: parse_u64(18),
                disclosure_status,
                assurance_basis,
            };
            let actual = canonical_lineage_attestation_bytes_v1(&attestation)
                .expect("known-answer lineage attestation is valid");
            let expected = decode_attestation_hex(fields[21]);
            assert_eq!(
                actual,
                expected,
                "lineage attestation V1 canonical bytes differ for {}",
                fields[0]
            );
            seen += 1;
        }
        assert_eq!(seen, 1, "all shared lineage attestation KATs must execute");
    }

    fn lineage_attestation_fixture() -> (LineageAttestationV1, LineageAttestationContextV1) {
        let attestation = LineageAttestationV1 {
            attestation_ref: "attestation:lineage:42".to_owned(),
            issuer_ref: "issuer:municipal-lab".to_owned(),
            issuer_key_ref: "key:municipal-lab:epoch-7".to_owned(),
            issuer_key_epoch: 7,
            evidence_ref: "evidence:pressure-channel-a".to_owned(),
            scope_digest: [1; 32],
            graph_digest: [2; 32],
            profile_ref: "profile:independence:operational-v1".to_owned(),
            profile_digest: [3; 32],
            verifier_frontier_digest: [4; 32],
            trust_policy_ref: "policy:lineage-trust:v2".to_owned(),
            trust_policy_digest: [5; 32],
            identity_registry_digest: [6; 32],
            assurance_basis_ref: "auditor:independent-accredited-lab".to_owned(),
            statement_digest: [7; 32],
            issued_at_unix_seconds: 100,
            valid_from_unix_seconds: 90,
            valid_until_unix_seconds: 200,
            disclosure_status: LineageDisclosureStatus::CompleteForDeclaredModel,
            assurance_basis: LineageAttestationBasis::IndependentlyAudited,
        };
        let context = LineageAttestationContextV1 {
            evidence_ref: attestation.evidence_ref.clone(),
            scope_digest: attestation.scope_digest,
            graph_digest: attestation.graph_digest,
            profile_ref: attestation.profile_ref.clone(),
            profile_digest: attestation.profile_digest,
            verifier_frontier_digest: attestation.verifier_frontier_digest,
            trust_policy_ref: attestation.trust_policy_ref.clone(),
            trust_policy_digest: attestation.trust_policy_digest,
            identity_registry_digest: attestation.identity_registry_digest,
            issuer_ref: attestation.issuer_ref.clone(),
            issuer_key_ref: attestation.issuer_key_ref.clone(),
            issuer_key_epoch: attestation.issuer_key_epoch,
            assurance_basis_ref: attestation.assurance_basis_ref.clone(),
            accepted_at_unix_seconds: 150,
            require_independent_audit: true,
        };
        (attestation, context)
    }

    #[test]
    fn lineage_attestation_precheck_binds_graph_profile_frontier_and_policy() {
        let (attestation, context) = lineage_attestation_fixture();
        assert_eq!(
            precheck_lineage_attestation_v1(&attestation, &context).unwrap(),
            LineageAttestationPrecheck::ReadyForExternalTrustVerification
        );
        let mut substituted = context.clone();
        substituted.verifier_frontier_digest = [9; 32];
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_issuer = context.clone();
        substituted_issuer.issuer_ref = "issuer:other".to_owned();
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_issuer)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_key_epoch = context.clone();
        substituted_key_epoch.issuer_key_epoch += 1;
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_key_epoch)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_auditor = context.clone();
        substituted_auditor.assurance_basis_ref = "auditor:other".to_owned();
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_auditor)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_policy = context.clone();
        substituted_policy.trust_policy_digest = [9; 32];
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_policy)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_graph = context.clone();
        substituted_graph.graph_digest = [9; 32];
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_graph)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_scope = context.clone();
        substituted_scope.scope_digest = [9; 32];
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_scope)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_evidence = context.clone();
        substituted_evidence.evidence_ref = "evidence:other".to_owned();
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_evidence)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_profile = context.clone();
        substituted_profile.profile_ref = "profile:other".to_owned();
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_profile)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_profile_digest = context.clone();
        substituted_profile_digest.profile_digest = [9; 32];
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_profile_digest)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_key = context.clone();
        substituted_key.issuer_key_ref = "key:other".to_owned();
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_key)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_trust_policy = context.clone();
        substituted_trust_policy.trust_policy_ref = "policy:other".to_owned();
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_trust_policy)
                .unwrap_err()
                .contains("does not bind")
        );
        let mut substituted_registry = context.clone();
        substituted_registry.identity_registry_digest = [9; 32];
        assert!(
            precheck_lineage_attestation_v1(&attestation, &substituted_registry)
                .unwrap_err()
                .contains("does not bind")
        );
    }

    #[test]
    fn withheld_or_incomplete_lineage_remains_indeterminate() {
        let (mut attestation, context) = lineage_attestation_fixture();
        attestation.disclosure_status = LineageDisclosureStatus::Withheld;
        assert_eq!(
            precheck_lineage_attestation_v1(&attestation, &context).unwrap(),
            LineageAttestationPrecheck::IndeterminateDisclosure
        );
        attestation.disclosure_status = LineageDisclosureStatus::Incomplete;
        assert_eq!(
            precheck_lineage_attestation_v1(&attestation, &context).unwrap(),
            LineageAttestationPrecheck::IndeterminateDisclosure
        );
    }

    #[test]
    fn issuer_declaration_does_not_satisfy_an_audited_lineage_profile() {
        let (mut attestation, context) = lineage_attestation_fixture();
        attestation.assurance_basis = LineageAttestationBasis::IssuerDeclared;
        assert_eq!(
            precheck_lineage_attestation_v1(&attestation, &context).unwrap(),
            LineageAttestationPrecheck::IndeterminateAssuranceBasis
        );
        let mut permissive_context = context;
        permissive_context.require_independent_audit = false;
        assert_eq!(
            precheck_lineage_attestation_v1(&attestation, &permissive_context).unwrap(),
            LineageAttestationPrecheck::ReadyForExternalTrustVerification
        );
    }

    #[test]
    fn attestation_outside_frontier_or_validity_window_is_rejected() {
        let (mut attestation, context) = lineage_attestation_fixture();
        // A self-reported issue timestamp is not a causal frontier proof;
        // its inclusion at the accepted frontier belongs to the external
        // COSE/SCITT receipt verifier.
        attestation.issued_at_unix_seconds = 151;
        assert_eq!(
            precheck_lineage_attestation_v1(&attestation, &context).unwrap(),
            LineageAttestationPrecheck::ReadyForExternalTrustVerification
        );
        attestation.valid_until_unix_seconds = 150;
        assert!(
            precheck_lineage_attestation_v1(&attestation, &context)
                .unwrap_err()
                .contains("outside its validity interval")
        );
    }

    #[test]
    fn lineage_attestation_canonical_bytes_are_deterministic_and_domain_separated() {
        let (attestation, _) = lineage_attestation_fixture();
        let first = canonical_lineage_attestation_bytes_v1(&attestation).unwrap();
        let second = canonical_lineage_attestation_bytes_v1(&attestation).unwrap();
        assert_eq!(first, second);
        assert!(
            first
                .windows(b"MYCELIX-DJCC-LINEAGE-ATTESTATION-V1".len())
                .any(|window| window == b"MYCELIX-DJCC-LINEAGE-ATTESTATION-V1")
        );
        let mut modified = attestation.clone();
        modified.statement_digest = [8; 32];
        assert_ne!(
            first,
            canonical_lineage_attestation_bytes_v1(&modified).unwrap()
        );
    }

    #[test]
    fn channel_and_node_order_are_part_of_the_canonical_input_contract() {
        let mut graph = disjoint_graph();
        graph.channels.reverse();
        assert!(assess_independence(&graph, &profile()).unwrap_err().contains("sorted"));
        let mut graph = disjoint_graph();
        graph.nodes.reverse();
        assert!(assess_independence(&graph, &profile()).unwrap_err().contains("sorted"));
    }
}
