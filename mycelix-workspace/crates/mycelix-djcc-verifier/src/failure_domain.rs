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

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct IndependenceAssessment {
    pub disposition: FailureDomainDisposition,
    /// Shared domain IDs, not merely shared labels or classes.
    pub common_domain_refs: Vec<String>,
    /// Shared IDs whose class has not been classified by this profile.
    pub unclassified_shared_domain_refs: Vec<String>,
    /// Required classes absent from each channel's reachable lineage.
    pub missing_classes_by_channel: BTreeMap<String, Vec<FailureDomainClass>>,
}

fn valid_ref(value: &str) -> bool {
    !value.trim().is_empty() && value.len() <= 4096
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
    if profile.material_shared_classes.is_empty()
        || !profile
            .material_shared_classes
            .is_subset(&profile.required_classes)
    {
        return Err(
            "material shared-domain classes must be non-empty and a subset of required classes"
                .to_owned(),
        );
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
    if graph.channels.len() < 2 {
        return Err("independence assessment requires at least two evidence channels".to_owned());
    }
    if graph.nodes.is_empty() {
        return Err("failure-domain graph is empty".to_owned());
    }

    let mut nodes = BTreeMap::new();
    let mut all_node_refs = Vec::with_capacity(graph.nodes.len());
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
        all_node_refs.push(node.domain_ref.as_str());
    }
    if !strictly_sorted_unique(&all_node_refs) {
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

    let mut channel_lineage: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut missing_classes_by_channel = BTreeMap::new();
    let mut node_channel_counts: BTreeMap<String, usize> = BTreeMap::new();

    for channel in &graph.channels {
        let mut lineage = BTreeSet::new();
        let mut active = BTreeSet::new();
        for root in &channel.root_domain_refs {
            visit(root, &nodes, &mut active, &mut lineage)?;
        }
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
        for domain_ref in &lineage {
            *node_channel_counts.entry(domain_ref.clone()).or_default() += 1;
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

    let mut common_domain_refs = Vec::new();
    let mut unclassified_shared_domain_refs = Vec::new();
    for (domain_ref, count) in &node_channel_counts {
        if *count < 2 {
            continue;
        }
        let node = nodes
            .get(domain_ref)
            .expect("reachable failure-domain node is registered");
        if profile.material_shared_classes.contains(&node.class) {
            common_domain_refs.push(domain_ref.clone());
        } else if !profile
            .explicitly_non_material_shared_classes
            .contains(&node.class)
        {
            unclassified_shared_domain_refs.push(domain_ref.clone());
        }
    }

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
    fn channel_and_node_order_are_part_of_the_canonical_input_contract() {
        let mut graph = disjoint_graph();
        graph.channels.reverse();
        assert!(assess_independence(&graph, &profile()).unwrap_err().contains("sorted"));
        let mut graph = disjoint_graph();
        graph.nodes.reverse();
        assert!(assess_independence(&graph, &profile()).unwrap_err().contains("sorted"));
    }
}
