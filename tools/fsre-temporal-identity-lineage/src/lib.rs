#![forbid(unsafe_code)]

// Event-sourced temporal identity reference implementation.
// This validates declared lineage evidence; it does not authenticate sources,
// infer physical identity, or establish population completeness.

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "event_id|event_type|predecessors|successors|effective_at|recorded_at|source_snapshot|evidence_owner|evidence_ref|adjudicator|rule_version";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineageError(pub String);

impl std::fmt::Display for LineageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for LineageError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EventType {
    Rename,
    IdentifierReassignment,
    Relocation,
    OwnershipTransfer,
    OperatorTransfer,
    Split,
    Merge,
    Retire,
    Recommission,
    ServiceBoundaryChange,
}
impl EventType {
    fn parse(value: &str) -> Result<Self, LineageError> {
        match value {
            "RENAME" => Ok(Self::Rename),
            "IDENTIFIER_REASSIGNMENT" => Ok(Self::IdentifierReassignment),
            "RELOCATION" => Ok(Self::Relocation),
            "OWNERSHIP_TRANSFER" => Ok(Self::OwnershipTransfer),
            "OPERATOR_TRANSFER" => Ok(Self::OperatorTransfer),
            "SPLIT" => Ok(Self::Split),
            "MERGE" => Ok(Self::Merge),
            "RETIRE" => Ok(Self::Retire),
            "RECOMMISSION" => Ok(Self::Recommission),
            "SERVICE_BOUNDARY_CHANGE" => Ok(Self::ServiceBoundaryChange),
            other => Err(LineageError(format!("unknown event type {other}"))),
        }
    }

    fn valid_arity(self, predecessors: usize, successors: usize) -> bool {
        match self {
            Self::Split => predecessors == 1 && successors >= 2,
            Self::Merge => predecessors >= 2 && successors == 1,
            Self::Retire => predecessors == 1 && successors == 0,
            Self::Recommission => predecessors == 0 && successors == 1,
            Self::Rename
            | Self::IdentifierReassignment
            | Self::Relocation
            | Self::OwnershipTransfer
            | Self::OperatorTransfer
            | Self::ServiceBoundaryChange => predecessors == 1 && successors == 1,
        }
    }

    fn preserves_identity(self) -> bool {
        matches!(
            self,
            Self::Rename
                | Self::Relocation
                | Self::OwnershipTransfer
                | Self::OperatorTransfer
                | Self::ServiceBoundaryChange
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineageEvent {
    pub event_id: String,
    pub event_type: EventType,
    pub predecessors: Vec<String>,
    pub successors: Vec<String>,
    pub effective_at: u64,
    pub recorded_at: u64,
    pub source_snapshot: String,
    pub evidence_owner: String,
    pub evidence_ref: String,
    pub adjudicator: String,
    pub rule_version: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineageReport {
    pub recorded_cutoff: u64,
    pub effective_cutoff: u64,
    pub input_events: usize,
    pub replayed_events: usize,
    pub excluded_not_yet_recorded: usize,
    pub excluded_not_yet_effective: usize,
    pub event_type_counts: BTreeMap<String, usize>,
    pub lineage_edges: usize,
    pub identity_status: String,
}

impl LineageReport {
    pub fn render(&self) -> String {
        let mut out = format!(
            concat!(
                "recorded_cutoff={}\n",
                "effective_cutoff={}\n",
                "input_events={}\n",
                "replayed_events={}\n",
                "excluded_not_yet_recorded={}\n",
                "excluded_not_yet_effective={}\n",
                "lineage_edges={}\n",
                "identity_status={}\n"
            ),
            self.recorded_cutoff,
            self.effective_cutoff,
            self.input_events,
            self.replayed_events,
            self.excluded_not_yet_recorded,
            self.excluded_not_yet_effective,
            self.lineage_edges,
            self.identity_status
        );
        for (kind, count) in &self.event_type_counts {
            out.push_str(&format!("event_type.{kind}={count}\n"));
        }
        out.push_str("source_authentication=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("reference_population_completeness=NOT_ESTABLISHED\n");
        out
    }
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 200
        && value.bytes().all(|b| {
            b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn parse_ids(value: &str, field: &str) -> Result<Vec<String>, LineageError> {
    if value == "-" {
        return Ok(Vec::new());
    }
    let ids: Vec<String> = value.split(',').map(str::trim).map(str::to_owned).collect();
    if ids.is_empty() || ids.iter().any(|id| !valid_token(id)) {
        return Err(LineageError(format!("invalid {field} list")));
    }
    let unique: BTreeSet<&str> = ids.iter().map(String::as_str).collect();
    if unique.len() != ids.len() {
        return Err(LineageError(format!("duplicate identifier in {field}")));
    }
    Ok(ids)
}

fn parse_u64(value: &str, field: &str) -> Result<u64, LineageError> {
    value.parse::<u64>().map_err(|e| LineageError(format!("invalid {field}: {e}")))
}

fn parse_events(source: &str) -> Result<Vec<LineageEvent>, LineageError> {
    let mut events = Vec::new();
    let mut ids = BTreeSet::new();
    let mut header_seen = false;
    for (offset, raw) in source.lines().enumerate() {
        let line_no = offset + 1;
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(LineageError(format!("line {line_no}: unexpected header")));
            }
            header_seen = true;
            continue;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 11 {
            return Err(LineageError(format!("line {line_no}: expected 11 fields")));
        }
        if !valid_token(fields[0]) || !ids.insert(fields[0].to_owned()) {
            return Err(LineageError(format!("line {line_no}: invalid or duplicate event_id")));
        }
        for (field, label) in [
            (fields[6], "source_snapshot"),
            (fields[7], "evidence_owner"),
            (fields[8], "evidence_ref"),
            (fields[9], "adjudicator"),
            (fields[10], "rule_version"),
        ] {
            if !valid_token(field) {
                return Err(LineageError(format!("line {line_no}: invalid {label}")));
            }
        }
        let event_type = EventType::parse(fields[1])?;
        let predecessors = parse_ids(fields[2], "predecessors")?;
        let successors = parse_ids(fields[3], "successors")?;
        if !event_type.valid_arity(predecessors.len(), successors.len()) {
            return Err(LineageError(format!("line {line_no}: invalid predecessor/successor cardinality")));
        }
        if event_type.preserves_identity() && predecessors.first() != successors.first() {
            return Err(LineageError(format!("line {line_no}: identity-preserving event changed entity ID")));
        }
        if event_type == EventType::IdentifierReassignment
            && predecessors.first() == successors.first()
        {
            return Err(LineageError(format!("line {line_no}: identifier reassignment requires distinct identity IDs")));
        }
        let effective_at = parse_u64(fields[4], "effective_at")?;
        let recorded_at = parse_u64(fields[5], "recorded_at")?;
        events.push(LineageEvent {
            event_id: fields[0].to_owned(),
            event_type,
            predecessors,
            successors,
            effective_at,
            recorded_at,
            source_snapshot: fields[6].to_owned(),
            evidence_owner: fields[7].to_owned(),
            evidence_ref: fields[8].to_owned(),
            adjudicator: fields[9].to_owned(),
            rule_version: fields[10].to_owned(),
        });
    }
    if !header_seen {
        return Err(LineageError("missing event table header".to_owned()));
    }
    Ok(events)
}

fn ensure_acyclic(events: &[&LineageEvent]) -> Result<(), LineageError> {
    let mut graph = BTreeMap::<String, BTreeSet<String>>::new();
    for event in events {
        for predecessor in &event.predecessors {
            for successor in &event.successors {
                if predecessor != successor {
                    graph.entry(predecessor.clone()).or_default().insert(successor.clone());
                }
            }
        }
    }
    fn visit(
        node: &str,
        graph: &BTreeMap<String, BTreeSet<String>>,
        temporary: &mut BTreeSet<String>,
        permanent: &mut BTreeSet<String>,
    ) -> bool {
        if permanent.contains(node) { return true; }
        if !temporary.insert(node.to_owned()) { return false; }
        if let Some(next) = graph.get(node) {
            for child in next {
                if !visit(child, graph, temporary, permanent) { return false; }
            }
        }
        temporary.remove(node);
        permanent.insert(node.to_owned());
        true
    }
    let mut temporary = BTreeSet::new();
    let mut permanent = BTreeSet::new();
    for node in graph.keys() {
        if !visit(node, &graph, &mut temporary, &mut permanent) {
            return Err(LineageError("temporal lineage graph contains a cycle".to_owned()));
        }
    }
    Ok(())
}

/// Replays only events knowable by recorded_cutoff and effective by effective_cutoff.
/// Both clocks are explicit to support historical decision-time reconstruction.
pub fn replay(
    source: &str,
    recorded_cutoff: u64,
    effective_cutoff: u64,
) -> Result<LineageReport, LineageError> {
    let events = parse_events(source)?;
    let mut selected = Vec::new();
    let mut excluded_not_yet_recorded = 0;
    let mut excluded_not_yet_effective = 0;
    for event in &events {
        if event.recorded_at > recorded_cutoff {
            excluded_not_yet_recorded += 1;
        } else if event.effective_at > effective_cutoff {
            excluded_not_yet_effective += 1;
        } else {
            selected.push(event);
        }
    }
    ensure_acyclic(&selected)?;
    let mut event_type_counts = BTreeMap::new();
    let mut lineage_edges = 0;
    for event in &selected {
        *event_type_counts.entry(format!("{:?}", event.event_type).to_ascii_uppercase()).or_insert(0) += 1;
        lineage_edges += event.predecessors.len() * event.successors.len();
    }
    Ok(LineageReport {
        recorded_cutoff,
        effective_cutoff,
        input_events: events.len(),
        replayed_events: selected.len(),
        excluded_not_yet_recorded,
        excluded_not_yet_effective,
        event_type_counts,
        lineage_edges,
        identity_status: if selected.is_empty() { "NO_LINEAGE_EVENTS_IN_VIEW" } else { "LINEAGE_REPLAYED_NOT_IDENTITY_AUTHENTICATED" }.to_owned(),
    })
}

#[cfg(test)]
mod unit_tests {
    use super::*;
    #[test]
    fn token_validation_rejects_empty_and_delimiter_injection() {
        assert!(!valid_token(""));
        assert!(!valid_token("x|y"));
        assert!(valid_token("plant:001"));
    }
}
