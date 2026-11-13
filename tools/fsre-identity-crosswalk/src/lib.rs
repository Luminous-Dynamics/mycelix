#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "transition_id|relation|source_system|source_record_id|target_system|target_record_id|effective_date|evidence_ref|evidence_class|review_state";

const RELATIONS: [&str; 8] = [
    "CONTINUITY",
    "RENAMED",
    "TRANSFERRED",
    "MERGED_INTO",
    "SPLIT_INTO",
    "DECOMMISSIONED",
    "CREATED",
    "UNRESOLVED",
];

const EVIDENCE_CLASSES: [&str; 5] = [
    "OFFICIAL_CROSSWALK",
    "SOURCE_NATIVE_CHANGE",
    "OPERATOR_RECORD",
    "INDEPENDENT_FIELD",
    "SYNTHETIC",
];

const REVIEW_STATES: [&str; 5] = [
    "ACCEPTED",
    "PROVISIONAL",
    "DISPUTED",
    "UNRESOLVED",
    "SYNTHETIC_ONLY",
];

#[derive(Clone, Debug, PartialEq, Eq)]
struct Metadata {
    fixture_kind: String,
    scope: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Transition {
    id: String,
    relation: String,
    source_system: Option<String>,
    source_id: Option<String>,
    target_system: Option<String>,
    target_id: Option<String>,
    effective_date: String,
    evidence_ref: String,
    evidence_class: String,
    review_state: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Group {
    relation: String,
    rows: Vec<Transition>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Report {
    fixture_kind: String,
    scope: String,
    row_count: usize,
    groups: Vec<Group>,
}

fn take_required(metadata: &mut BTreeMap<String, String>, key: &str) -> Result<String, String> {
    metadata
        .remove(key)
        .ok_or_else(|| format!("missing metadata key: {key}"))
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':'))
}

fn valid_date(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 10
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || !bytes[0..4].iter().all(u8::is_ascii_digit)
        || !bytes[5..7].iter().all(u8::is_ascii_digit)
        || !bytes[8..10].iter().all(u8::is_ascii_digit)
    {
        return false;
    }

    let year = value[0..4].parse::<u32>().unwrap_or(0);
    let month = value[5..7].parse::<u32>().unwrap_or(0);
    let day = value[8..10].parse::<u32>().unwrap_or(0);
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    year > 0 && day > 0 && day <= days
}

fn parse_metadata(metadata: &mut BTreeMap<String, String>) -> Result<Metadata, String> {
    let parsed = Metadata {
        fixture_kind: take_required(metadata, "fixture_kind")?,
        scope: take_required(metadata, "scope")?,
    };
    if let Some(key) = metadata.keys().next() {
        return Err(format!("unknown metadata key: {key}"));
    }
    if !matches!(parsed.fixture_kind.as_str(), "SYNTHETIC_ONLY" | "SOURCE_BACKED")
        || parsed.scope.is_empty()
    {
        return Err("invalid fixture kind or scope".to_owned());
    }
    Ok(parsed)
}

fn optional_pair(
    system: &str,
    record: &str,
    label: &str,
) -> Result<(Option<String>, Option<String>), String> {
    match (system.is_empty(), record.is_empty()) {
        (true, true) => Ok((None, None)),
        (false, false) if valid_identifier(system) && valid_identifier(record) => {
            Ok((Some(system.to_owned()), Some(record.to_owned())))
        }
        _ => Err(format!("{label} system and record ID must both be present or both be empty")),
    }
}

fn parse_transition(line: &str, line_number: usize) -> Result<Transition, String> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 10 {
        return Err(format!("line {line_number}: expected 10 transition fields"));
    }

    if !valid_identifier(fields[0]) || !RELATIONS.contains(&fields[1]) {
        return Err(format!("line {line_number}: invalid transition ID or relation"));
    }
    let (source_system, source_id) = optional_pair(fields[2], fields[3], "source")?;
    let (target_system, target_id) = optional_pair(fields[4], fields[5], "target")?;
    if !valid_date(fields[6]) {
        return Err(format!("line {line_number}: invalid effective_date"));
    }
    if fields[7].is_empty() || !EVIDENCE_CLASSES.contains(&fields[8]) {
        return Err(format!("line {line_number}: evidence reference/class is missing or unsupported"));
    }
    if !REVIEW_STATES.contains(&fields[9]) {
        return Err(format!("line {line_number}: unsupported review_state {}", fields[9]));
    }

    let source_required = !matches!(fields[1], "CREATED");
    let target_required = matches!(
        fields[1],
        "CONTINUITY" | "RENAMED" | "TRANSFERRED" | "MERGED_INTO" | "SPLIT_INTO"
            | "CREATED"
    );
    if source_required != source_id.is_some() || target_required != target_id.is_some() {
        return Err(format!(
            "line {line_number}: source/target presence is invalid for relation {}",
            fields[1]
        ));
    }
    if fields[1] == "UNRESOLVED"
        && !matches!(fields[9], "UNRESOLVED" | "DISPUTED" | "PROVISIONAL" | "SYNTHETIC_ONLY")
    {
        return Err(format!(
            "line {line_number}: unresolved identity cannot be marked {}",
            fields[9]
        ));
    }
    if fields[8] == "SYNTHETIC" && fields[9] != "SYNTHETIC_ONLY" {
        return Err(format!(
            "line {line_number}: synthetic evidence must remain SYNTHETIC_ONLY"
        ));
    }
    if fields[9] == "SYNTHETIC_ONLY" && fields[8] != "SYNTHETIC" {
        return Err(format!(
            "line {line_number}: SYNTHETIC_ONLY review state requires SYNTHETIC evidence"
        ));
    }
    if fields[9] == "ACCEPTED"
        && !matches!(
            fields[8],
            "OFFICIAL_CROSSWALK" | "SOURCE_NATIVE_CHANGE" | "INDEPENDENT_FIELD"
        )
    {
        return Err(format!(
            "line {line_number}: accepted transition lacks admissible source evidence"
        ));
    }

    Ok(Transition {
        id: fields[0].to_owned(),
        relation: fields[1].to_owned(),
        source_system,
        source_id,
        target_system,
        target_id,
        effective_date: fields[6].to_owned(),
        evidence_ref: fields[7].to_owned(),
        evidence_class: fields[8].to_owned(),
        review_state: fields[9].to_owned(),
    })
}

fn group_transitions(rows: Vec<Transition>) -> Result<Vec<Group>, String> {
    let mut grouped = BTreeMap::<String, Vec<Transition>>::new();
    for row in rows {
        grouped.entry(row.id.clone()).or_default().push(row);
    }

    let mut groups = Vec::new();
    for (id, rows) in grouped {
        let first = rows.first().expect("groups are created with a row");
        if rows.iter().any(|row| {
            row.relation != first.relation
                || row.effective_date != first.effective_date
                || row.evidence_ref != first.evidence_ref
                || row.evidence_class != first.evidence_class
                || row.review_state != first.review_state
                || row.source_system != first.source_system
                || row.target_system != first.target_system
        }) {
            return Err(format!("transition group {id} has inconsistent relation, scope, date, evidence or review state"));
        }

        let sources: BTreeSet<&str> = rows
            .iter()
            .filter_map(|row| row.source_id.as_deref())
            .collect();
        let targets: BTreeSet<&str> = rows
            .iter()
            .filter_map(|row| row.target_id.as_deref())
            .collect();
        let valid_shape = match first.relation.as_str() {
            "CONTINUITY" | "RENAMED" | "TRANSFERRED" => {
                rows.len() == 1 && sources.len() == 1 && targets.len() == 1
            }
            "MERGED_INTO" => rows.len() >= 2 && sources.len() >= 2 && targets.len() == 1,
            "SPLIT_INTO" => rows.len() >= 2 && sources.len() == 1 && targets.len() >= 2,
            "DECOMMISSIONED" | "UNRESOLVED" => {
                rows.len() == 1 && sources.len() == 1 && targets.is_empty()
            }
            "CREATED" => rows.len() == 1 && sources.is_empty() && targets.len() == 1,
            _ => false,
        };
        if !valid_shape {
            return Err(format!(
                "transition group {id} has invalid cardinality for relation {}",
                first.relation
            ));
        }
        groups.push(Group {
            relation: first.relation.clone(),
            rows,
        });
    }

    let mut source_assignments = BTreeMap::<(String, String, String), String>::new();
    let mut target_assignments = BTreeMap::<(String, String, String), String>::new();
    for group in &groups {
        for row in &group.rows {
            if let (Some(system), Some(record)) = (&row.source_system, &row.source_id) {
                let key = (system.clone(), record.clone(), row.effective_date.clone());
                if let Some(previous) = source_assignments.insert(key, group.id.clone()) {
                    if previous != group.id {
                        return Err(format!(
                            "source identity is assigned to competing transitions {previous} and {} on {}",
                            group.id, row.effective_date
                        ));
                    }
                }
            }
            if let (Some(system), Some(record)) = (&row.target_system, &row.target_id) {
                let key = (system.clone(), record.clone(), row.effective_date.clone());
                if let Some(previous) = target_assignments.insert(key, group.id.clone()) {
                    if previous != group.id {
                        return Err(format!(
                            "target identity is assigned from competing transitions {previous} and {} on {}",
                            group.id, row.effective_date
                        ));
                    }
                }
            }
        }
    }
    Ok(groups)
}

fn parse(source: &str) -> Result<Report, String> {
    let mut metadata = BTreeMap::new();
    let mut rows = Vec::new();
    let mut header_seen = false;

    for (index, raw) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                let key = key.trim().to_owned();
                if metadata.insert(key.clone(), value.trim().to_owned()).is_some() {
                    return Err(format!("line {line_number}: duplicate metadata key {key}"));
                }
            }
            continue;
        }
        if !header_seen {
            if line != HEADER {
                return Err(format!("line {line_number}: unexpected transition header"));
            }
            header_seen = true;
            continue;
        }
        rows.push(parse_transition(line, line_number)?);
    }

    if !header_seen || rows.is_empty() {
        return Err("missing transition header or rows".to_owned());
    }
    let metadata = parse_metadata(&mut metadata)?;
    let groups = group_transitions(rows)?;
    if metadata.fixture_kind == "SYNTHETIC_ONLY"
        && groups.iter().any(|group| {
            group
                .rows
                .iter()
                .any(|row| row.review_state != "SYNTHETIC_ONLY")
        })
    {
        return Err("synthetic fixture cannot contain non-synthetic review states".to_owned());
    }
    Ok(Report {
        fixture_kind: metadata.fixture_kind,
        scope: metadata.scope,
        row_count: groups.iter().map(|group| group.rows.len()).sum(),
        groups,
    })
}

fn render(report: &Report) -> String {
    let mut output = String::new();
    output.push_str("report_schema=fsre-identity-crosswalk-report.v0.1\n");
    output.push_str(&format!("fixture_kind={}\n", report.fixture_kind));
    output.push_str(&format!("scope={}\n", report.scope));
    output.push_str(&format!("transition_rows={}\n", report.row_count));
    output.push_str(&format!("transition_groups={}\n", report.groups.len()));

    for relation in RELATIONS {
        let count = report
            .groups
            .iter()
            .filter(|group| group.relation == relation)
            .count();
        if count > 0 {
            output.push_str(&format!("relation={relation}|{count}\n"));
        }
    }
    for state in REVIEW_STATES {
        let count = report
            .groups
            .iter()
            .filter(|group| group.rows.first().map(|row| row.review_state.as_str()) == Some(state))
            .count();
        if count > 0 {
            output.push_str(&format!("review_state={state}|{count}\n"));
        }
    }
    let accepted = report
        .groups
        .iter()
        .filter(|group| group.rows.first().map(|row| row.review_state.as_str()) == Some("ACCEPTED"))
        .count();
    let unresolved = report
        .groups
        .iter()
        .filter(|group| group.relation == "UNRESOLVED")
        .count();
    let multi_event = report
        .groups
        .iter()
        .filter(|group| matches!(group.relation.as_str(), "MERGED_INTO" | "SPLIT_INTO"))
        .count();
    output.push_str(&format!("accepted_transition_groups={accepted}\n"));
    output.push_str(&format!("unresolved_identity_groups={unresolved}\n"));
    output.push_str(&format!("merge_or_split_groups={multi_event}\n"));
    let claim_scope = if report.fixture_kind == "SYNTHETIC_ONLY" {
        "SYNTHETIC_SCHEMA_AND_CARDINALITY_CONFORMANCE_ONLY"
    } else {
        "SOURCE_BACKED_CONTRACT_CONFORMANCE_ONLY"
    };
    output.push_str(&format!("claim_scope={claim_scope}\n"));
    output.push_str("real_system_identity_linkage=NOT_ESTABLISHED_BY_THIS_TOOL\n");
    output.push_str("source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL\n");
    output.push_str("operational_authority=NONE\n");
    output
}

pub fn validate_and_report(source: &str) -> Result<String, String> {
    Ok(render(&parse(source)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/synthetic-transitions.psv");

    #[test]
    fn synthetic_transition_fixture_is_valid_and_never_promoted() {
        let report = validate_and_report(FIXTURE).expect("fixture should validate");
        assert!(report.contains("transition_rows=10"));
        assert!(report.contains("transition_groups=8"));
        assert!(report.contains("relation=MERGED_INTO|1"));
        assert!(report.contains("relation=SPLIT_INTO|1"));
        assert!(report.contains("accepted_transition_groups=0"));
        assert!(report.contains("real_system_identity_linkage=NOT_ESTABLISHED"));
    }

    #[test]
    fn single_identity_relations_require_one_source_and_one_target() {
        let invalid = FIXTURE.replace(
            "tr-continuity|CONTINUITY|dws-2022|sys-a|dws-2025|sys-a|2023-07-01|synthetic:continuity|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-continuity|CONTINUITY|dws-2022|sys-a|dws-2025|sys-a|2023-07-01|synthetic:continuity|SYNTHETIC|SYNTHETIC_ONLY\ntr-continuity|CONTINUITY|dws-2022|sys-b|dws-2025|sys-b|2023-07-01|synthetic:continuity|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("invalid cardinality"));
    }

    #[test]
    fn merge_requires_multiple_distinct_sources_and_one_target() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|dws-2025|sys-merged|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "",
        );
        assert!(parse(&invalid).unwrap_err().contains("invalid cardinality"));
    }

    #[test]
    fn split_requires_one_source_and_multiple_distinct_targets() {
        let invalid = FIXTURE.replace(
            "tr-split|SPLIT_INTO|dws-2022|sys-parent|dws-2025|sys-child-b|2023-07-01|synthetic:split|SYNTHETIC|SYNTHETIC_ONLY",
            "",
        );
        assert!(parse(&invalid).unwrap_err().contains("invalid cardinality"));
    }

    #[test]
    fn merge_group_cannot_disagree_on_effective_date() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|dws-2025|sys-merged|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|dws-2025|sys-merged|2023-08-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("inconsistent"));
    }

    #[test]
    fn source_cannot_be_assigned_to_competing_transitions_on_same_date() {
        let extra = "tr-competing|RENAMED|dws-2022|sys-a|dws-2025|sys-other|2023-07-01|synthetic:competing|SYNTHETIC|SYNTHETIC_ONLY";
        assert!(parse(&format!("{FIXTURE}{extra}\n"))
            .unwrap_err()
            .contains("competing transitions"));
    }

    #[test]
    fn synthetic_evidence_cannot_be_marked_accepted() {
        let invalid = FIXTURE.replace(
            "SYNTHETIC|SYNTHETIC_ONLY",
            "SYNTHETIC|ACCEPTED",
        );
        assert!(parse(&invalid).unwrap_err().contains("synthetic evidence"));
    }

    #[test]
    fn accepted_transition_requires_qualifying_evidence_class() {
        let invalid = FIXTURE.replace(
            "SYNTHETIC|SYNTHETIC_ONLY",
            "OPERATOR_RECORD|ACCEPTED",
        );
        assert!(parse(&invalid).unwrap_err().contains("accepted transition"));
    }

    #[test]
    fn unresolved_identity_cannot_be_accepted() {
        let invalid = FIXTURE.replace(
            "tr-unresolved|UNRESOLVED|dws-2022|sys-ambiguous|||2023-07-01|synthetic:gap|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-unresolved|UNRESOLVED|dws-2022|sys-ambiguous|||2023-07-01|synthetic:gap|OFFICIAL_CROSSWALK|ACCEPTED",
        );
        assert!(parse(&invalid).is_err());
    }

    #[test]
    fn decommissioned_relation_cannot_have_a_target() {
        let invalid = FIXTURE.replace(
            "tr-decom|DECOMMISSIONED|dws-2022|sys-closed|||2023-07-01|synthetic:decom|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-decom|DECOMMISSIONED|dws-2022|sys-closed|dws-2025|sys-new|2023-07-01|synthetic:decom|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("source/target presence"));
    }

    #[test]
    fn created_relation_cannot_have_a_source() {
        let invalid = FIXTURE.replace(
            "tr-created|CREATED|||dws-2025|sys-new|2023-07-01|synthetic:created|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-created|CREATED|dws-2022|sys-old|dws-2025|sys-new|2023-07-01|synthetic:created|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("source/target presence"));
    }

    #[test]
    fn invalid_calendar_date_is_rejected() {
        let invalid = FIXTURE.replace("2023-07-01", "2023-02-30");
        assert!(parse(&invalid).unwrap_err().contains("invalid effective_date"));
    }

    #[test]
    fn target_cannot_be_assigned_from_competing_transitions_on_same_date() {
        let extra = "tr-target-conflict|RENAMED|dws-2022|sys-other|dws-2025|sys-a|2023-07-01|synthetic:conflict|SYNTHETIC|SYNTHETIC_ONLY";
        assert!(parse(&format!("{FIXTURE}{extra}\n"))
            .unwrap_err()
            .contains("target identity is assigned from competing transitions"));
    }

    #[test]
    fn report_is_invariant_to_row_order() {
        let mut rows: Vec<&str> = FIXTURE
            .lines()
            .filter(|line| !line.starts_with('#') && !line.starts_with("transition_id|"))
            .collect();
        rows.reverse();
        let metadata: Vec<&str> = FIXTURE
            .lines()
            .filter(|line| line.starts_with('#'))
            .collect();
        let reordered = format!(
            "{}\n{}\n{}\n",
            metadata.join("\n"),
            HEADER,
            rows.join("\n")
        );
        assert_eq!(
            validate_and_report(FIXTURE).expect("original fixture"),
            validate_and_report(&reordered).expect("reordered fixture")
        );
    }
}
