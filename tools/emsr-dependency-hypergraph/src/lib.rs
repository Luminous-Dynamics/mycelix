use serde::de::{self, MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Number, Value};
use std::fmt;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// Parse exactly one JSON value and reject duplicate property names at every
/// nesting level. serde_json::Value's default parser is last-key-wins; that is
/// unsafe for byte-bound evidence contracts because different implementations
/// can otherwise interpret the same signed bytes differently.
#[derive(Debug)]
struct StrictValue(Value);

impl<'de> Deserialize<'de> for StrictValue {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct StrictVisitor;

        impl<'de> Visitor<'de> for StrictVisitor {
            type Value = StrictValue;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("JSON with unique object property names")
            }

            fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Bool(value)))
            }

            fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Number(Number::from(value))))
            }

            fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Number(Number::from(value))))
            }

            fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Number::from_f64(value)
                    .map(|number| StrictValue(Value::Number(number)))
                    .ok_or_else(|| E::custom("non-finite JSON number"))
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::String(value.to_owned())))
            }

            fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::String(value)))
            }

            fn visit_unit<E>(self) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Null))
            }

            fn visit_none<E>(self) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                Ok(StrictValue(Value::Null))
            }

            fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
            where
                D: serde::Deserializer<'de>,
            {
                StrictValue::deserialize(deserializer)
            }

            fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut values = Vec::new();
                while let Some(value) = sequence.next_element::<StrictValue>()? {
                    values.push(value.0);
                }
                Ok(StrictValue(Value::Array(values)))
            }

            fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
            where
                A: MapAccess<'de>,
            {
                let mut values = Map::new();
                while let Some(key) = map.next_key::<String>()? {
                    if values.contains_key(&key) {
                        return Err(de::Error::custom("duplicate JSON object property"));
                    }
                    let value = map.next_value::<StrictValue>()?;
                    values.insert(key, value.0);
                }
                Ok(StrictValue(Value::Object(values)))
            }
        }

        deserializer.deserialize_any(StrictVisitor)
    }
}

fn parse_strict_json(bytes: &[u8]) -> Result<Value, ()> {
    let mut deserializer = serde_json::Deserializer::from_slice(bytes);
    let value = StrictValue::deserialize(&mut deserializer).map_err(|_| ())?;
    deserializer.end().map_err(|_| ())?;
    Ok(value.0)
}

pub const INPUT_CONTRACT: &str = "emsr-dependency-hypergraph-input/0.1";
pub const REPORT_CONTRACT: &str = "emsr-dependency-hypergraph-report/0.1";
pub const VERIFIER_CONTRACT: &str = "emsr-dependency-hypergraph-reference-verifier/0.1";
const MAX_NODES: usize = 100_000;
const MAX_HYPEREDGES: usize = 500_000;
const CLAIM_CEILING: &str = "Declared common-dependency assessment over the exact supplied inventory snapshot and evidence frontier; not proof of physical or statistical independence, and not evidence that unknown common causes do not exist.";

#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DependencyDomain {
    Clock,
    Oscillator,
    GnssReference,
    CorrectionProvider,
    Power,
    Network,
    AntennaSite,
    FirmwareLineage,
    Collector,
    DataPipeline,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeKind {
    Source,
    Dependency,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum AssertionState {
    Qualified,
    Stale,
    Revoked,
    Conflicted,
    Unknown,
    Unavailable,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Node {
    pub id: String,
    pub kind: NodeKind,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DependencyHyperedge {
    pub id: String,
    pub subject_ids: Vec<String>,
    pub dependency_id: String,
    pub domain: DependencyDomain,
    pub valid_from: Option<i64>,
    pub valid_until: Option<i64>,
    pub recorded_at: i64,
    pub evidence_ref: String,
    pub state: AssertionState,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InventorySnapshot {
    pub snapshot_id: String,
    pub as_of: i64,
    pub evidence_frontier: i64,
    pub complete: bool,
    pub scoped_source_ids: Vec<String>,
    pub covered_node_ids: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DependencyQuery {
    pub source_ids: Vec<String>,
    pub domain: DependencyDomain,
    pub event_time: i64,
    pub expected_snapshot_id: String,
    pub expected_evidence_frontier: i64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HypergraphInput {
    pub contract: String,
    pub snapshot: InventorySnapshot,
    pub query: DependencyQuery,
    pub nodes: Vec<Node>,
    pub hyperedges: Vec<DependencyHyperedge>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Disposition {
    SharedDependency,
    UnknownDependency,
    NoDeclaredCommonDomain,
    InvalidInput,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EvaluationReport {
    pub report_contract: String,
    pub disposition: Disposition,
    pub authority: String,
    pub input_sha256: String,
    pub snapshot_id: String,
    pub evidence_frontier: i64,
    pub event_time: i64,
    pub domain: DependencyDomain,
    pub source_ids: Vec<String>,
    pub source_ancestors: BTreeMap<String, Vec<String>>,
    pub common_dependency_ids: Vec<String>,
    pub reason_codes: Vec<String>,
    pub claim_ceiling: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct VerificationReport {
    pub verifier_contract: String,
    pub status: String,
    pub authority: String,
    pub checks: Vec<VerificationCheck>,
    pub limitations: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct VerificationCheck {
    pub id: String,
    pub result: String,
}

#[derive(Clone, Debug)]
struct Validated<'a> {
    edges_by_subject: BTreeMap<&'a str, Vec<&'a DependencyHyperedge>>,
    uncertain_by_subject: BTreeMap<&'a str, Vec<&'a DependencyHyperedge>>,
}

pub fn evaluate_bytes(input_bytes: &[u8]) -> EvaluationReport {
    let input_sha256 = sha256_hex(input_bytes);
    let parsed = parse_strict_json(input_bytes)
        .and_then(|value| serde_json::from_value::<HypergraphInput>(value).map_err(|_| ()));
    match parsed {
        Ok(input) => evaluate(&input, input_sha256),
        Err(_) => invalid_report(
            input_sha256,
            "input_json_invalid_duplicate_or_unknown_contract_fields",
            DependencyDomain::Clock,
            0,
            String::new(),
            Vec::new(),
            0,
        ),
    }
}

pub fn evaluate(input: &HypergraphInput, input_sha256: String) -> EvaluationReport {
    let mut invalid = Vec::new();
    validate_contract(input, &mut invalid);
    if !invalid.is_empty() {
        invalid.sort();
        invalid.dedup();
        return invalid_report(
            input_sha256,
            &invalid.join("|"),
            input.query.domain.clone(),
            input.query.event_time,
            input.snapshot.snapshot_id.clone(),
            input.query.source_ids.clone(),
            input.snapshot.evidence_frontier,
        );
    }

    let validated = build_adjacency(input);
    let mut unknown_reasons = Vec::new();
    if !input.snapshot.complete {
        unknown_reasons.push("inventory_snapshot_incomplete".to_owned());
    }
    if input.snapshot.as_of < input.query.event_time {
        unknown_reasons.push("inventory_snapshot_older_than_event_time".to_owned());
    }
    for source_id in &input.query.source_ids {
        if input
            .snapshot
            .scoped_source_ids
            .binary_search(source_id)
            .is_err()
        {
            unknown_reasons.push("source_outside_inventory_scope".to_owned());
        }
    }

    let mut closures = BTreeMap::new();
    for source_id in &input.query.source_ids {
        if input
            .snapshot
            .covered_node_ids
            .binary_search(source_id)
            .is_err()
        {
            unknown_reasons.push("source_not_covered_by_inventory_snapshot".to_owned());
        }
        if has_cycle(source_id, &validated.edges_by_subject) {
            return invalid_report(
                input_sha256,
                "dependency_cycle_detected",
                input.query.domain.clone(),
                input.query.event_time,
                input.snapshot.snapshot_id.clone(),
                input.query.source_ids.clone(),
                input.snapshot.evidence_frontier,
            );
        }

        let (ancestors, mut reasons) = iterative_closure(
            source_id,
            &input.snapshot.covered_node_ids,
            &validated.edges_by_subject,
            &validated.uncertain_by_subject,
        );
        unknown_reasons.append(&mut reasons);
        closures.insert(source_id.clone(), ancestors);
    }

    let common = intersection(&closures, &input.query.source_ids);
    unknown_reasons.sort();
    unknown_reasons.dedup();

    // A declared, qualified shared ancestor is positive evidence of commonality.
    // Unknown lineage cannot turn that positive common-cause finding into a claim
    // of independence; this implementation conservatively reports UNKNOWN when
    // any required coverage/currentness predicate is unresolved.
    let disposition = if !unknown_reasons.is_empty() {
        Disposition::UnknownDependency
    } else if !common.is_empty() {
        Disposition::SharedDependency
    } else {
        Disposition::NoDeclaredCommonDomain
    };

    let reason_codes = if !unknown_reasons.is_empty() {
        unknown_reasons
    } else if !common.is_empty() {
        vec!["qualified_common_dependency_ancestor".to_owned()]
    } else {
        vec!["complete_inventory_has_no_declared_common_ancestor".to_owned()]
    };

    EvaluationReport {
        report_contract: REPORT_CONTRACT.to_owned(),
        disposition,
        authority: "declared_dependency_assessment_only".to_owned(),
        input_sha256,
        snapshot_id: input.snapshot.snapshot_id.clone(),
        evidence_frontier: input.snapshot.evidence_frontier,
        event_time: input.query.event_time,
        domain: input.query.domain.clone(),
        source_ids: input.query.source_ids.clone(),
        source_ancestors: closures
            .into_iter()
            .map(|(source, ancestors)| (source, ancestors.into_iter().collect()))
            .collect(),
        common_dependency_ids: common.into_iter().collect(),
        reason_codes,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    }
}

pub fn independently_verify(input_bytes: &[u8], report_bytes: &[u8]) -> VerificationReport {
    let mut checks = Vec::new();
    let input_hash = sha256_hex(input_bytes);
    let parsed_input = parse_strict_json(input_bytes)
        .and_then(|value| serde_json::from_value::<HypergraphInput>(value).map_err(|_| ()));
    let parsed_report = parse_strict_json(report_bytes)
        .and_then(|value| serde_json::from_value::<EvaluationReport>(value).map_err(|_| ()));

    match (parsed_input, parsed_report) {
        (Ok(input), Ok(report)) => {
            let reference = reference_report(&input, input_hash);
            checks.push(check(
                "input-byte-binding",
                report.input_sha256 == reference.input_sha256,
            ));
            checks.push(check(
                "query-snapshot-binding",
                report.snapshot_id == reference.snapshot_id
                    && report.evidence_frontier == reference.evidence_frontier
                    && report.event_time == reference.event_time
                    && report.domain == reference.domain
                    && report.source_ids == reference.source_ids,
            ));
            checks.push(check(
                "independent-closure",
                report.source_ancestors == reference.source_ancestors
                    && report.common_dependency_ids == reference.common_dependency_ids,
            ));
            checks.push(check(
                "derived-disposition",
                report.disposition == reference.disposition
                    && report.reason_codes == reference.reason_codes,
            ));
            checks.push(check(
                "authority-and-claim-ceiling",
                report.authority == "declared_dependency_assessment_only"
                    && report.claim_ceiling == CLAIM_CEILING
                    && report.report_contract == REPORT_CONTRACT,
            ));
        }
        _ => {
            checks.push(check("input-and-report-parse", false));
        }
    }

    let passed = checks.iter().all(|item| item.result == "pass");
    VerificationReport {
        verifier_contract: VERIFIER_CONTRACT.to_owned(),
        status: if passed { "verified" } else { "rejected" }.to_owned(),
        authority: "independent_reference_algorithm_agreement_only".to_owned(),
        checks,
        limitations: vec![
            "The verifier is a separate traversal implementation in the same Rust crate; this is code-path separation, not organizational or language independence.".to_owned(),
            "A passing report establishes agreement with the declared input contract, not physical independence or the absence of unknown common causes.".to_owned(),
            "The input snapshot and its completeness declaration are assertions supplied by the caller; this crate does not authenticate the inventory issuer.".to_owned(),
        ],
    }
}

fn validate_contract(input: &HypergraphInput, errors: &mut Vec<String>) {
    if input.contract != INPUT_CONTRACT {
        errors.push("unsupported_input_contract".to_owned());
    }
    if input.query.expected_snapshot_id != input.snapshot.snapshot_id {
        errors.push("query_snapshot_id_mismatch".to_owned());
    }
    if input.query.expected_evidence_frontier != input.snapshot.evidence_frontier {
        errors.push("query_evidence_frontier_mismatch".to_owned());
    }
    if input.snapshot.snapshot_id.trim().is_empty() {
        errors.push("snapshot_id_empty".to_owned());
    }
    if input.snapshot.as_of > input.snapshot.evidence_frontier {
        errors.push("snapshot_as_of_after_evidence_frontier".to_owned());
    }
    if input.query.source_ids.len() < 2 {
        errors.push("query_requires_at_least_two_sources".to_owned());
    }
    if !strictly_sorted_unique(&input.query.source_ids) {
        errors.push("query_sources_must_be_sorted_unique".to_owned());
    }
    if !strictly_sorted_unique(&input.snapshot.scoped_source_ids) {
        errors.push("snapshot_scope_sources_must_be_sorted_unique".to_owned());
    }
    if !strictly_sorted_unique(&input.snapshot.covered_node_ids) {
        errors.push("snapshot_covered_nodes_must_be_sorted_unique".to_owned());
    }
    if input.nodes.is_empty() || input.nodes.len() > MAX_NODES {
        errors.push("node_count_out_of_bounds".to_owned());
    }
    if input.hyperedges.len() > MAX_HYPEREDGES {
        errors.push("hyperedge_count_exceeds_limit".to_owned());
    }

    let mut node_ids = BTreeSet::new();
    let mut node_kinds = BTreeMap::new();
    for node in &input.nodes {
        if node.id.trim().is_empty() {
            errors.push("node_id_empty".to_owned());
        }
        if !node_ids.insert(node.id.as_str()) {
            errors.push("duplicate_node_id".to_owned());
        }
        node_kinds.insert(node.id.as_str(), &node.kind);
    }
    for scoped_source in &input.snapshot.scoped_source_ids {
        if !node_kinds
            .get(scoped_source.as_str())
            .is_some_and(|kind| **kind == NodeKind::Source)
        {
            errors.push("snapshot_scope_entry_is_not_a_source".to_owned());
        }
    }
    for source_id in &input.query.source_ids {
        match input.nodes.iter().find(|node| node.id == *source_id) {
            Some(node) if node.kind == NodeKind::Source => {}
            Some(_) => errors.push("queried_node_is_not_a_source".to_owned()),
            None => errors.push("queried_source_missing_from_graph".to_owned()),
        }
    }

    let mut edge_ids = BTreeSet::new();
    for edge in &input.hyperedges {
        if edge.id.trim().is_empty() || !edge_ids.insert(edge.id.as_str()) {
            errors.push("empty_or_duplicate_hyperedge_id".to_owned());
        }
        if edge.subject_ids.is_empty() || !strictly_sorted_unique(&edge.subject_ids) {
            errors.push("hyperedge_subjects_must_be_nonempty_sorted_unique".to_owned());
        }
        if edge.evidence_ref.trim().is_empty() {
            errors.push("hyperedge_evidence_reference_empty".to_owned());
        }
        if matches!((edge.valid_from, edge.valid_until), (Some(start), Some(end)) if start >= end) {
            errors.push("hyperedge_valid_time_interval_empty_or_inverted".to_owned());
        }
        if edge.subject_ids.binary_search(&edge.dependency_id).is_ok() {
            errors.push("hyperedge_dependency_cannot_be_its_own_subject".to_owned());
        }
        if !node_ids.contains(edge.dependency_id.as_str()) {
            errors.push("hyperedge_dependency_node_missing".to_owned());
        } else if !node_kinds
            .get(edge.dependency_id.as_str())
            .is_some_and(|kind| **kind == NodeKind::Dependency)
        {
            errors.push("hyperedge_dependency_must_reference_dependency_node".to_owned());
        }
        for subject in &edge.subject_ids {
            if !node_ids.contains(subject.as_str()) {
                errors.push("hyperedge_subject_node_missing".to_owned());
            }
        }
    }
    for id in &input.snapshot.covered_node_ids {
        if !node_ids.contains(id.as_str()) {
            errors.push("snapshot_covers_unknown_node".to_owned());
        }
    }
}

fn build_adjacency<'a>(input: &'a HypergraphInput) -> Validated<'a> {
    let mut edges_by_subject: BTreeMap<&str, Vec<&DependencyHyperedge>> = BTreeMap::new();
    let mut uncertain_by_subject: BTreeMap<&str, Vec<&DependencyHyperedge>> = BTreeMap::new();

    for edge in &input.hyperedges {
        if edge.domain != input.query.domain || !valid_at(edge, input.query.event_time) {
            continue;
        }
        for subject in &edge.subject_ids {
            if edge.state == AssertionState::Qualified
                && edge.recorded_at <= input.snapshot.evidence_frontier
            {
                edges_by_subject
                    .entry(subject.as_str())
                    .or_default()
                    .push(edge);
            } else {
                uncertain_by_subject
                    .entry(subject.as_str())
                    .or_default()
                    .push(edge);
            }
        }
    }
    for edges in edges_by_subject.values_mut() {
        edges.sort_by(|left, right| left.id.cmp(&right.id));
    }
    for edges in uncertain_by_subject.values_mut() {
        edges.sort_by(|left, right| left.id.cmp(&right.id));
    }

    Validated {
        edges_by_subject,
        uncertain_by_subject,
    }
}

fn valid_at(edge: &DependencyHyperedge, instant: i64) -> bool {
    edge.valid_from.is_none_or(|start| start <= instant)
        && edge.valid_until.is_none_or(|end| instant < end)
}

fn iterative_closure(
    source_id: &str,
    covered_node_ids: &[String],
    adjacency: &BTreeMap<&str, Vec<&DependencyHyperedge>>,
    uncertain: &BTreeMap<&str, Vec<&DependencyHyperedge>>,
) -> (BTreeSet<String>, Vec<String>) {
    let mut ancestors = BTreeSet::new();
    let mut reasons = Vec::new();
    let mut visited = BTreeSet::new();
    let mut queue = VecDeque::from([source_id.to_owned()]);

    while let Some(node_id) = queue.pop_front() {
        if !visited.insert(node_id.clone()) {
            continue;
        }
        if covered_node_ids.binary_search(&node_id).is_err() {
            reasons.push("reachable_node_not_covered_by_inventory_snapshot".to_owned());
        }
        if uncertain
            .get(node_id.as_str())
            .is_some_and(|items| !items.is_empty())
        {
            reasons.push("applicable_assertion_not_qualified_or_outside_frontier".to_owned());
        }
        if let Some(edges) = adjacency.get(node_id.as_str()) {
            for edge in edges {
                ancestors.insert(edge.dependency_id.clone());
                queue.push_back(edge.dependency_id.clone());
            }
        }
    }
    (ancestors, reasons)
}

fn has_cycle(start: &str, adjacency: &BTreeMap<&str, Vec<&DependencyHyperedge>>) -> bool {
    let mut color: BTreeMap<String, u8> = BTreeMap::new();
    let mut stack = vec![(start.to_owned(), false)];

    while let Some((node_id, exiting)) = stack.pop() {
        if exiting {
            color.insert(node_id, 2);
            continue;
        }
        match color.get(&node_id).copied().unwrap_or(0) {
            1 => return true,
            2 => continue,
            _ => {}
        }
        color.insert(node_id.clone(), 1);
        stack.push((node_id.clone(), true));

        if let Some(edges) = adjacency.get(node_id.as_str()) {
            for edge in edges.iter().rev() {
                let next = &edge.dependency_id;
                if color.get(next).copied() == Some(1) {
                    return true;
                }
                if color.get(next).copied().unwrap_or(0) == 0 {
                    stack.push((next.clone(), false));
                }
            }
        }
    }
    false
}

fn intersection(
    closures: &BTreeMap<String, BTreeSet<String>>,
    source_ids: &[String],
) -> BTreeSet<String> {
    let Some(first) = source_ids.first().and_then(|source| closures.get(source)) else {
        return BTreeSet::new();
    };
    let mut common = first.clone();
    for source in source_ids.iter().skip(1) {
        if let Some(ancestors) = closures.get(source) {
            common.retain(|item| ancestors.contains(item));
        } else {
            common.clear();
        }
    }
    common
}

fn reference_report(input: &HypergraphInput, input_sha256: String) -> EvaluationReport {
    // Deliberately independent of the evaluator's iterative closure.
    let mut errors = Vec::new();
    validate_contract(input, &mut errors);
    if !errors.is_empty() {
        errors.sort();
        errors.dedup();
        return invalid_report(
            input_sha256,
            &errors.join("|"),
            input.query.domain.clone(),
            input.query.event_time,
            input.snapshot.snapshot_id.clone(),
            input.query.source_ids.clone(),
            input.snapshot.evidence_frontier,
        );
    }

    let mut source_ancestors = BTreeMap::new();
    let mut unknown = Vec::new();
    if !input.snapshot.complete {
        unknown.push("inventory_snapshot_incomplete".to_owned());
    }
    if input.snapshot.as_of < input.query.event_time {
        unknown.push("inventory_snapshot_older_than_event_time".to_owned());
    }

    for source in &input.query.source_ids {
        if input
            .snapshot
            .scoped_source_ids
            .binary_search(source)
            .is_err()
        {
            unknown.push("source_outside_inventory_scope".to_owned());
        }
        if input
            .snapshot
            .covered_node_ids
            .binary_search(source)
            .is_err()
        {
            unknown.push("source_not_covered_by_inventory_snapshot".to_owned());
        }
        let mut visiting = BTreeSet::new();
        let mut done = BTreeSet::new();
        let mut ancestors = BTreeSet::new();
        if reference_visit(
            source,
            input,
            &mut visiting,
            &mut done,
            &mut ancestors,
            &mut unknown,
        ) {
            return invalid_report(
                input_sha256,
                "dependency_cycle_detected",
                input.query.domain.clone(),
                input.query.event_time,
                input.snapshot.snapshot_id.clone(),
                input.query.source_ids.clone(),
                input.snapshot.evidence_frontier,
            );
        }
        source_ancestors.insert(source.clone(), ancestors);
    }

    unknown.sort();
    unknown.dedup();
    let common = intersection(&source_ancestors, &input.query.source_ids);
    let disposition = if !unknown.is_empty() {
        Disposition::UnknownDependency
    } else if !common.is_empty() {
        Disposition::SharedDependency
    } else {
        Disposition::NoDeclaredCommonDomain
    };
    let reason_codes = if !unknown.is_empty() {
        unknown
    } else if !common.is_empty() {
        vec!["qualified_common_dependency_ancestor".to_owned()]
    } else {
        vec!["complete_inventory_has_no_declared_common_ancestor".to_owned()]
    };

    EvaluationReport {
        report_contract: REPORT_CONTRACT.to_owned(),
        disposition,
        authority: "declared_dependency_assessment_only".to_owned(),
        input_sha256,
        snapshot_id: input.snapshot.snapshot_id.clone(),
        evidence_frontier: input.snapshot.evidence_frontier,
        event_time: input.query.event_time,
        domain: input.query.domain.clone(),
        source_ids: input.query.source_ids.clone(),
        source_ancestors: source_ancestors
            .into_iter()
            .map(|(source, ancestors)| (source, ancestors.into_iter().collect()))
            .collect(),
        common_dependency_ids: common.into_iter().collect(),
        reason_codes,
        claim_ceiling: CLAIM_CEILING.to_owned(),
    }
}

fn reference_visit(
    node_id: &str,
    input: &HypergraphInput,
    visiting: &mut BTreeSet<String>,
    done: &mut BTreeSet<String>,
    ancestors: &mut BTreeSet<String>,
    unknown: &mut Vec<String>,
) -> bool {
    if done.contains(node_id) {
        return false;
    }
    if !visiting.insert(node_id.to_owned()) {
        return true;
    }
    if input
        .snapshot
        .covered_node_ids
        .binary_search(&node_id.to_owned())
        .is_err()
    {
        unknown.push("reachable_node_not_covered_by_inventory_snapshot".to_owned());
    }

    for edge in &input.hyperedges {
        if edge.domain != input.query.domain
            || !edge.subject_ids.iter().any(|subject| subject == node_id)
            || !valid_at(edge, input.query.event_time)
        {
            continue;
        }
        if edge.state != AssertionState::Qualified
            || edge.recorded_at > input.snapshot.evidence_frontier
        {
            unknown.push("applicable_assertion_not_qualified_or_outside_frontier".to_owned());
            continue;
        }
        ancestors.insert(edge.dependency_id.clone());
        if reference_visit(
            &edge.dependency_id,
            input,
            visiting,
            done,
            ancestors,
            unknown,
        ) {
            return true;
        }
    }
    visiting.remove(node_id);
    done.insert(node_id.to_owned());
    false
}

fn invalid_report(
    input_sha256: String,
    reason: &str,
    domain: DependencyDomain,
    event_time: i64,
    snapshot_id: String,
    source_ids: Vec<String>,
    evidence_frontier: i64,
) -> EvaluationReport {
    EvaluationReport {
        report_contract: REPORT_CONTRACT.to_owned(),
        disposition: Disposition::InvalidInput,
        authority: "declared_dependency_assessment_only".to_owned(),
        input_sha256,
        snapshot_id,
        evidence_frontier,
        event_time,
        domain,
        source_ids,
        source_ancestors: BTreeMap::new(),
        common_dependency_ids: Vec::new(),
        reason_codes: reason.split('|').map(str::to_owned).collect(),
        claim_ceiling: CLAIM_CEILING.to_owned(),
    }
}

fn check(id: &str, passed: bool) -> VerificationCheck {
    VerificationCheck {
        id: id.to_owned(),
        result: if passed { "pass" } else { "fail" }.to_owned(),
    }
}

fn strictly_sorted_unique(values: &[String]) -> bool {
    values.windows(2).all(|pair| pair[0] < pair[1])
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

pub fn parse_input_value(value: Value) -> Result<HypergraphInput, String> {
    serde_json::from_value(value).map_err(|error| error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> HypergraphInput {
        HypergraphInput {
            contract: INPUT_CONTRACT.to_owned(),
            snapshot: InventorySnapshot {
                snapshot_id: "snapshot-2026-11-07T10".to_owned(),
                as_of: 100,
                evidence_frontier: 120,
                complete: true,
                scoped_source_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
                covered_node_ids: vec![
                    "clock-root".to_owned(),
                    "provider-a".to_owned(),
                    "provider-b".to_owned(),
                    "rx-a".to_owned(),
                    "rx-b".to_owned(),
                ],
            },
            query: DependencyQuery {
                source_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
                domain: DependencyDomain::Clock,
                event_time: 90,
                expected_snapshot_id: "snapshot-2026-11-07T10".to_owned(),
                expected_evidence_frontier: 120,
            },
            nodes: vec![
                Node {
                    id: "clock-root".to_owned(),
                    kind: NodeKind::Dependency,
                },
                Node {
                    id: "provider-a".to_owned(),
                    kind: NodeKind::Dependency,
                },
                Node {
                    id: "provider-b".to_owned(),
                    kind: NodeKind::Dependency,
                },
                Node {
                    id: "rx-a".to_owned(),
                    kind: NodeKind::Source,
                },
                Node {
                    id: "rx-b".to_owned(),
                    kind: NodeKind::Source,
                },
            ],
            hyperedges: vec![
                DependencyHyperedge {
                    id: "edge-a".to_owned(),
                    subject_ids: vec!["rx-a".to_owned()],
                    dependency_id: "provider-a".to_owned(),
                    domain: DependencyDomain::Clock,
                    valid_from: Some(0),
                    valid_until: None,
                    recorded_at: 80,
                    evidence_ref: "evidence:a".to_owned(),
                    state: AssertionState::Qualified,
                },
                DependencyHyperedge {
                    id: "edge-b".to_owned(),
                    subject_ids: vec!["rx-b".to_owned()],
                    dependency_id: "provider-b".to_owned(),
                    domain: DependencyDomain::Clock,
                    valid_from: Some(0),
                    valid_until: None,
                    recorded_at: 80,
                    evidence_ref: "evidence:b".to_owned(),
                    state: AssertionState::Qualified,
                },
                DependencyHyperedge {
                    id: "shared-root".to_owned(),
                    subject_ids: vec!["provider-a".to_owned(), "provider-b".to_owned()],
                    dependency_id: "clock-root".to_owned(),
                    domain: DependencyDomain::Clock,
                    valid_from: Some(0),
                    valid_until: None,
                    recorded_at: 80,
                    evidence_ref: "evidence:root".to_owned(),
                    state: AssertionState::Qualified,
                },
            ],
        }
    }

    fn evaluate_fixture(input: &HypergraphInput) -> EvaluationReport {
        let bytes = serde_json::to_vec(input).unwrap();
        evaluate_bytes(&bytes)
    }


    #[test]
    fn duplicate_top_level_json_properties_are_rejected() {
        let input = br#"{"contract":"emsr-dependency-hypergraph-input/0.1","contract":"emsr-dependency-hypergraph-input/0.1"}"#;
        let report = evaluate_bytes(input);
        assert_eq!(report.disposition, Disposition::InvalidInput);
        assert_eq!(
            report.reason_codes,
            vec!["input_json_invalid_duplicate_or_unknown_contract_fields"]
        );
    }

    #[test]
    fn duplicate_nested_json_properties_are_rejected() {
        let input = br#"{"contract":"emsr-dependency-hypergraph-input/0.1","snapshot":{"snapshot_id":"one","snapshot_id":"two"}}"#;
        let report = evaluate_bytes(input);
        assert_eq!(report.disposition, Disposition::InvalidInput);
        assert_eq!(
            report.reason_codes,
            vec!["input_json_invalid_duplicate_or_unknown_contract_fields"]
        );
    }

    #[test]
    fn unknown_top_level_contract_fields_are_rejected() {
        let mut input = fixture();
        let mut value = serde_json::to_value(&input).unwrap();
        value.as_object_mut().unwrap().insert("unexpected".into(), Value::Bool(true));
        let bytes = serde_json::to_vec(&value).unwrap();
        let report = evaluate_bytes(&bytes);
        assert_eq!(report.disposition, Disposition::InvalidInput);
        assert_eq!(
            report.reason_codes,
            vec!["input_json_invalid_duplicate_or_unknown_contract_fields"]
        );
        let report_bytes = serde_json::to_vec(&report).unwrap();
        assert_eq!(independently_verify(&bytes, &report_bytes).status, "rejected");
    }

    #[test]
    fn trailing_json_values_are_rejected() {
        let mut bytes = serde_json::to_vec(&fixture()).unwrap();
        bytes.extend_from_slice(br#" {}"#);
        let report = evaluate_bytes(&bytes);
        assert_eq!(report.disposition, Disposition::InvalidInput);
        assert_eq!(
            report.reason_codes,
            vec!["input_json_invalid_duplicate_or_unknown_contract_fields"]
        );
    }

    #[test]
    fn transitive_hyperedge_detects_shared_clock_ancestor() {
        let input = fixture();
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::SharedDependency);
        assert_eq!(report.common_dependency_ids, vec!["clock-root"]);
    }

    #[test]
    fn pairwise_projection_is_derived_without_destroying_hyperedge_identity() {
        let input = fixture();
        let report = evaluate_fixture(&input);
        assert_eq!(
            report.source_ancestors["rx-a"],
            vec!["clock-root", "provider-a"]
        );
        assert_eq!(
            report.source_ancestors["rx-b"],
            vec!["clock-root", "provider-b"]
        );
    }

    #[test]
    fn separate_power_and_clock_domains_do_not_cross_infer() {
        let mut input = fixture();
        input.hyperedges.push(DependencyHyperedge {
            id: "power-bus".to_owned(),
            subject_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
            dependency_id: "clock-root".to_owned(),
            domain: DependencyDomain::Power,
            valid_from: Some(0),
            valid_until: None,
            recorded_at: 80,
            evidence_ref: "evidence:power".to_owned(),
            state: AssertionState::Qualified,
        });
        input.query.domain = DependencyDomain::Power;
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::SharedDependency);
    }

    #[test]
    fn a_shared_dependency_outside_event_time_is_not_current_for_the_query() {
        let mut input = fixture();
        input.hyperedges[2].valid_until = Some(50);
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::NoDeclaredCommonDomain);
        assert!(report.common_dependency_ids.is_empty());
    }

    #[test]
    fn stale_assertions_produce_unknown_not_independence() {
        let mut input = fixture();
        input.hyperedges[2].state = AssertionState::Stale;
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::UnknownDependency);
    }

    #[test]
    fn source_omitted_from_snapshot_scope_produces_unknown() {
        let mut input = fixture();
        input.snapshot.scoped_source_ids.pop();
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::UnknownDependency);
        assert!(
            report
                .reason_codes
                .contains(&"source_outside_inventory_scope".to_owned())
        );
    }

    #[test]
    fn direct_multi_subject_hyperedge_preserves_a_shared_dependency() {
        let mut input = fixture();
        input.hyperedges.truncate(2);
        input.hyperedges.push(DependencyHyperedge {
            id: "one-clock-for-both".to_owned(),
            subject_ids: vec!["rx-a".to_owned(), "rx-b".to_owned()],
            dependency_id: "clock-root".to_owned(),
            domain: DependencyDomain::Clock,
            valid_from: None,
            valid_until: None,
            recorded_at: 80,
            evidence_ref: "evidence:one-clock".to_owned(),
            state: AssertionState::Qualified,
        });
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::SharedDependency);
        assert_eq!(report.common_dependency_ids, vec!["clock-root"]);
    }

    #[test]
    fn incomplete_inventory_produces_unknown_not_independence() {
        let mut input = fixture();
        input.snapshot.complete = false;
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::UnknownDependency);
    }

    #[test]
    fn cycle_is_invalid_and_never_reported_as_shared_or_independent() {
        let mut input = fixture();
        input.hyperedges.push(DependencyHyperedge {
            id: "cycle".to_owned(),
            subject_ids: vec!["clock-root".to_owned()],
            dependency_id: "provider-a".to_owned(),
            domain: DependencyDomain::Clock,
            valid_from: Some(0),
            valid_until: None,
            recorded_at: 80,
            evidence_ref: "evidence:cycle".to_owned(),
            state: AssertionState::Qualified,
        });
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::InvalidInput);
        assert_eq!(report.reason_codes, vec!["dependency_cycle_detected"]);
    }

    #[test]
    fn independent_reference_verifier_rejects_tampered_closure() {
        let input = fixture();
        let input_bytes = serde_json::to_vec(&input).unwrap();
        let mut report = evaluate_bytes(&input_bytes);
        report.common_dependency_ids.clear();
        let report_bytes = serde_json::to_vec(&report).unwrap();
        let verification = independently_verify(&input_bytes, &report_bytes);
        assert_eq!(verification.status, "rejected");
        assert!(
            verification
                .checks
                .iter()
                .any(|item| { item.id == "independent-closure" && item.result == "fail" })
        );
    }

    #[test]
    fn source_order_must_be_frozen_to_make_exact_evaluation_deterministic() {
        let mut input = fixture();
        input.query.source_ids.reverse();
        let report = evaluate_fixture(&input);
        assert_eq!(report.disposition, Disposition::InvalidInput);
        assert!(
            report
                .reason_codes
                .contains(&"query_sources_must_be_sorted_unique".to_owned())
        );
    }
}
