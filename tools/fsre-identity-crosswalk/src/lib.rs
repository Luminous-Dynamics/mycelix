#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str = "transition_id|relation|source_system|source_record_id|source_population_scope|source_owner_class|target_system|target_record_id|target_population_scope|target_owner_class|effective_date|evidence_ref|evidence_class|review_state";

const POPULATION_SCOPES: [&str; 4] = [
    "MUNICIPAL_SYSTEMS",
    "ALL_SECTOR_WORKS",
    "WATER_SERVICE_AUTHORITIES",
    "UNKNOWN",
];

const OWNER_CLASSES: [&str; 5] = ["MUNICIPAL", "DPWI", "PRIVATE", "OTHER_PUBLIC", "UNKNOWN"];

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

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Endpoint {
    system: String,
    record_id: String,
    population_scope: String,
    owner_class: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Transition {
    id: String,
    relation: String,
    source: Option<Endpoint>,
    target: Option<Endpoint>,
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
    if !matches!(
        parsed.fixture_kind.as_str(),
        "SYNTHETIC_ONLY" | "SOURCE_BACKED"
    ) || parsed.scope.is_empty()
    {
        return Err("invalid fixture kind or scope".to_owned());
    }
    Ok(parsed)
}

fn optional_endpoint(fields: &[&str], label: &str) -> Result<Option<Endpoint>, String> {
    if fields.len() != 4 {
        return Err(format!("{label} endpoint requires four fields"));
    }
    if fields.iter().all(|value| value.is_empty()) {
        return Ok(None);
    }
    if fields.iter().any(|value| value.is_empty()) {
        return Err(format!(
            "{label} endpoint system, record, population scope and owner class must be present together"
        ));
    }
    if !valid_identifier(fields[0])
        || !valid_identifier(fields[1])
        || !POPULATION_SCOPES.contains(&fields[2])
        || !OWNER_CLASSES.contains(&fields[3])
    {
        return Err(format!(
            "{label} endpoint has an invalid namespace, ID, population scope or owner class"
        ));
    }
    Ok(Some(Endpoint {
        system: fields[0].to_owned(),
        record_id: fields[1].to_owned(),
        population_scope: fields[2].to_owned(),
        owner_class: fields[3].to_owned(),
    }))
}

fn parse_transition(line: &str, line_number: usize) -> Result<Transition, String> {
    let fields: Vec<&str> = line.split('|').map(str::trim).collect();
    if fields.len() != 14 {
        return Err(format!("line {line_number}: expected 14 transition fields"));
    }

    if !valid_identifier(fields[0]) || !RELATIONS.contains(&fields[1]) {
        return Err(format!(
            "line {line_number}: invalid transition ID or relation"
        ));
    }
    let source = optional_endpoint(&fields[2..6], "source")?;
    let target = optional_endpoint(&fields[6..10], "target")?;
    if !valid_date(fields[10]) {
        return Err(format!("line {line_number}: invalid effective_date"));
    }
    if fields[11].is_empty() || !EVIDENCE_CLASSES.contains(&fields[12]) {
        return Err(format!(
            "line {line_number}: evidence reference/class is missing or unsupported"
        ));
    }
    if !REVIEW_STATES.contains(&fields[13]) {
        return Err(format!(
            "line {line_number}: unsupported review_state {}",
            fields[13]
        ));
    }

    let source_required = !matches!(fields[1], "CREATED");
    let target_required = matches!(
        fields[1],
        "CONTINUITY" | "RENAMED" | "TRANSFERRED" | "MERGED_INTO" | "SPLIT_INTO" | "CREATED"
    );
    if source_required != source.is_some() || target_required != target.is_some() {
        return Err(format!(
            "line {line_number}: source/target presence is invalid for relation {}",
            fields[1]
        ));
    }
    if fields[1] == "UNRESOLVED"
        && !matches!(
            fields[13],
            "UNRESOLVED" | "DISPUTED" | "PROVISIONAL" | "SYNTHETIC_ONLY"
        )
    {
        return Err(format!(
            "line {line_number}: unresolved identity cannot be marked {}",
            fields[13]
        ));
    }
    if fields[12] == "SYNTHETIC" && fields[13] != "SYNTHETIC_ONLY" {
        return Err(format!(
            "line {line_number}: synthetic evidence must remain SYNTHETIC_ONLY"
        ));
    }
    if fields[13] == "SYNTHETIC_ONLY" && fields[12] != "SYNTHETIC" {
        return Err(format!(
            "line {line_number}: SYNTHETIC_ONLY review state requires SYNTHETIC evidence"
        ));
    }
    if fields[13] == "ACCEPTED" {
        if !matches!(
            fields[12],
            "OFFICIAL_CROSSWALK" | "SOURCE_NATIVE_CHANGE" | "INDEPENDENT_FIELD"
        ) {
            return Err(format!(
                "line {line_number}: accepted transition lacks admissible source evidence"
            ));
        }
        if source.iter().chain(target.iter()).any(|endpoint| {
            endpoint.population_scope == "UNKNOWN" || endpoint.owner_class == "UNKNOWN"
        }) {
            return Err(format!(
                "line {line_number}: accepted transition requires known population scope and owner class"
            ));
        }
    }

    Ok(Transition {
        id: fields[0].to_owned(),
        relation: fields[1].to_owned(),
        source,
        target,
        effective_date: fields[10].to_owned(),
        evidence_ref: fields[11].to_owned(),
        evidence_class: fields[12].to_owned(),
        review_state: fields[13].to_owned(),
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
                || row.source.as_ref().map(|endpoint| &endpoint.system)
                    != first.source.as_ref().map(|endpoint| &endpoint.system)
                || row.target.as_ref().map(|endpoint| &endpoint.system)
                    != first.target.as_ref().map(|endpoint| &endpoint.system)
        }) {
            return Err(format!("transition group {id} has inconsistent relation, namespace, date, evidence or review state"));
        }

        let sources: BTreeSet<&str> = rows
            .iter()
            .filter_map(|row| {
                row.source
                    .as_ref()
                    .map(|endpoint| endpoint.record_id.as_str())
            })
            .collect();
        let targets: BTreeSet<&str> = rows
            .iter()
            .filter_map(|row| {
                row.target
                    .as_ref()
                    .map(|endpoint| endpoint.record_id.as_str())
            })
            .collect();
        let valid_shape = match first.relation.as_str() {
            "CONTINUITY" | "RENAMED" | "TRANSFERRED" => {
                rows.len() == 1 && sources.len() == 1 && targets.len() == 1
            }
            "MERGED_INTO" => {
                rows.len() >= 2
                    && sources.len() >= 2
                    && targets.len() == 1
                    && rows.iter().all(|row| row.target == first.target)
                    && rows.iter().all(|row| {
                        row.source
                            .as_ref()
                            .map(|endpoint| &endpoint.population_scope)
                            == first
                                .source
                                .as_ref()
                                .map(|endpoint| &endpoint.population_scope)
                    })
            }
            "SPLIT_INTO" => {
                rows.len() >= 2
                    && sources.len() == 1
                    && targets.len() >= 2
                    && rows.iter().all(|row| row.source == first.source)
            }
            "DECOMMISSIONED" | "UNRESOLVED" => {
                rows.len() == 1 && sources.len() == 1 && targets.is_empty()
            }
            "CREATED" => rows.len() == 1 && sources.is_empty() && targets.len() == 1,
            _ => false,
        };
        if !valid_shape {
            return Err(format!(
                "transition group {id} has invalid cardinality or endpoint-scope consistency for relation {}",
                first.relation
            ));
        }
        groups.push(Group {
            relation: first.relation.clone(),
            rows,
        });
    }

    let mut source_assignments =
        BTreeMap::<(String, String, String), (String, String, String)>::new();
    let mut target_assignments =
        BTreeMap::<(String, String, String), (String, String, String)>::new();
    for group in &groups {
        let group_id = group.rows.first().expect("group has rows").id.clone();
        for row in &group.rows {
            if let Some(endpoint) = &row.source {
                let key = (
                    endpoint.system.clone(),
                    endpoint.record_id.clone(),
                    row.effective_date.clone(),
                );
                let claim = (
                    group_id.clone(),
                    endpoint.population_scope.clone(),
                    endpoint.owner_class.clone(),
                );
                if let Some(previous) = source_assignments.insert(key, claim.clone()) {
                    if previous != claim {
                        return Err(format!(
                            "source identity is assigned to competing or scope-conflicting transitions {} and {} on {}",
                            previous.0, group_id, row.effective_date
                        ));
                    }
                }
            }
            if let Some(endpoint) = &row.target {
                let key = (
                    endpoint.system.clone(),
                    endpoint.record_id.clone(),
                    row.effective_date.clone(),
                );
                let claim = (
                    group_id.clone(),
                    endpoint.population_scope.clone(),
                    endpoint.owner_class.clone(),
                );
                if let Some(previous) = target_assignments.insert(key, claim.clone()) {
                    if previous != claim {
                        return Err(format!(
                            "target identity is assigned from competing or scope-conflicting transitions {} and {} on {}",
                            previous.0, group_id, row.effective_date
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
                if metadata
                    .insert(key.clone(), value.trim().to_owned())
                    .is_some()
                {
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
    if metadata.fixture_kind == "SOURCE_BACKED"
        && groups.iter().any(|group| {
            group
                .rows
                .iter()
                .any(|row| row.evidence_class == "SYNTHETIC")
        })
    {
        return Err("source-backed input cannot contain synthetic evidence".to_owned());
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
    let population_scope_changes = report
        .groups
        .iter()
        .filter(|group| {
            group.rows.iter().any(|row| {
                row.source
                    .as_ref()
                    .zip(row.target.as_ref())
                    .is_some_and( |(source, target)| {
                        source.population_scope != target.population_scope
                    })
            })
        })
        .count();
    let owner_class_changes = report
        .groups
        .iter()
        .filter(|group| {
            group.rows.iter().any(|row| {
                row.source
                    .as_ref()
                    .zip(row.target.as_ref())
                    .is_some_and( |(source, target)| {
                        source.owner_class != target.owner_class
                    })
            })
        })
        .count();
    output.push_str(&format!("accepted_transition_groups={accepted}\n"));
    output.push_str(&format!("unresolved_identity_groups={unresolved}\n"));
    output.push_str(&format!("merge_or_split_groups={multi_event}\n"));
    output.push_str(&format!(
        "population_scope_change_groups={population_scope_changes}\n"
    ));
    output.push_str(&format!(
        "owner_class_change_groups={owner_class_changes}\n"
    ));
    output.push_str("scope_changes_are_cohort_changes=REQUIRE_SEPARATE_COMPARABILITY_REVIEW\n");
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
    fn source_backed_input_cannot_mix_synthetic_evidence() {
        let source_backed = FIXTURE.replace(
            "# fixture_kind=SYNTHETIC_ONLY",
            "# fixture_kind=SOURCE_BACKED",
        );
        assert!(parse(&source_backed)
            .unwrap_err()
            .contains("source-backed input cannot contain synthetic evidence"));
    }

    #[test]
    fn single_identity_relations_require_one_source_and_one_target() {
        let invalid = FIXTURE.replace(
            "tr-continuity|CONTINUITY|dws-2022|sys-a|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-a|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:continuity|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-continuity|CONTINUITY|dws-2022|sys-a|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-a|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:continuity|SYNTHETIC|SYNTHETIC_ONLY\ntr-continuity|CONTINUITY|dws-2022|sys-b|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-b|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:continuity|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("invalid cardinality"));
    }

    #[test]
    fn merge_requires_multiple_distinct_sources_and_one_target() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "",
        );
        assert!(parse(&invalid).unwrap_err().contains("invalid cardinality"));
    }

    #[test]
    fn split_requires_one_source_and_multiple_distinct_targets() {
        let invalid = FIXTURE.replace(
            "tr-split|SPLIT_INTO|dws-2022|sys-parent|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-child-b|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:split|SYNTHETIC|SYNTHETIC_ONLY",
            "",
        );
        assert!(parse(&invalid).unwrap_err().contains("invalid cardinality"));
    }

    #[test]
    fn merge_group_cannot_disagree_on_effective_date() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-08-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("inconsistent"));
    }

    #[test]
    fn merge_group_cannot_disagree_on_source_namespace() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-merge|MERGED_INTO|other-registry|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("inconsistent"));
    }

    #[test]
    fn merge_group_cannot_disagree_on_evidence_reference() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:other-source|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("inconsistent"));
    }

    #[test]
    fn transition_group_cannot_mix_relation_types() {
        let invalid = FIXTURE.replace(
            "tr-merge|MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-merge|SPLIT_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-merged|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:merge|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid).unwrap_err().contains("inconsistent"));
    }

    #[test]
    fn population_scope_and_owner_class_changes_are_reported_separately() {
        let report = validate_and_report(FIXTURE).expect("fixture should validate");
        assert!(report.contains("population_scope_change_groups=1"));
        assert!(report.contains("owner_class_change_groups=1"));
        assert!(report
            .contains("scope_changes_are_cohort_changes=REQUIRE_SEPARATE_COMPARABILITY_REVIEW"));
    }

    #[test]
    fn merge_group_cannot_mix_population_scopes() {
        let mutated = FIXTURE.replace(
            "MERGED_INTO|dws-2022|sys-b-old|MUNICIPAL_SYSTEMS|MUNICIPAL",
            "MERGED_INTO|dws-2022|sys-b-old|ALL_SECTOR_WORKS|DPWI",
        );
        assert!(mutated != FIXTURE);
        assert!(validate_and_report(&mutated)
            .unwrap_err()
            .contains("invalid cardinality or endpoint-scope consistency"));
    }

    #[test]
    fn accepted_transition_requires_known_population_scope_and_owner_class() {
        let source = format!(
            "# fixture_kind=SOURCE_BACKED\n# scope=one source-backed synthetic test record\n{HEADER}\ntest|CONTINUITY|source-registry|system-a|UNKNOWN|UNKNOWN|target-registry|system-a|MUNICIPAL_SYSTEMS|MUNICIPAL|2025-01-01|official:crosswalk|OFFICIAL_CROSSWALK|ACCEPTED\n"
        );
        assert!(validate_and_report(&source)
            .unwrap_err()
            .contains("accepted transition requires known population scope and owner class"));
    }

    #[test]
    fn source_cannot_be_assigned_to_competing_transitions_on_same_date() {
        let extra = "tr-competing|RENAMED|dws-2022|sys-a|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-other|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:competing|SYNTHETIC|SYNTHETIC_ONLY";
        assert!(parse(&format!("{FIXTURE}{extra}\n"))
            .unwrap_err()
            .contains("competing or scope-conflicting transitions"));
    }

    #[test]
    fn synthetic_evidence_cannot_be_marked_accepted() {
        let invalid = FIXTURE.replace("SYNTHETIC|SYNTHETIC_ONLY", "SYNTHETIC|ACCEPTED");
        assert!(parse(&invalid).unwrap_err().contains("synthetic evidence"));
    }

    #[test]
    fn accepted_transition_requires_qualifying_evidence_class() {
        let invalid = FIXTURE.replace("SYNTHETIC|SYNTHETIC_ONLY", "OPERATOR_RECORD|ACCEPTED");
        assert!(parse(&invalid).unwrap_err().contains("accepted transition"));
    }

    #[test]
    fn unresolved_identity_cannot_be_accepted() {
        let invalid = FIXTURE.replace(
            "tr-unresolved|UNRESOLVED|dws-2022|sys-ambiguous|MUNICIPAL_SYSTEMS|MUNICIPAL|||||2023-07-01|synthetic:gap|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-unresolved|UNRESOLVED|dws-2022|sys-ambiguous|MUNICIPAL_SYSTEMS|MUNICIPAL|||||2023-07-01|synthetic:gap|OFFICIAL_CROSSWALK|ACCEPTED",
        );
        assert!(parse(&invalid).is_err());
    }

    #[test]
    fn decommissioned_relation_cannot_have_a_target() {
        let invalid = FIXTURE.replace(
            "tr-decom|DECOMMISSIONED|dws-2022|sys-closed|MUNICIPAL_SYSTEMS|MUNICIPAL|||||2023-07-01|synthetic:decom|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-decom|DECOMMISSIONED|dws-2022|sys-closed|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-new|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:decom|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid)
            .unwrap_err()
            .contains("source/target presence"));
    }

    #[test]
    fn created_relation_cannot_have_a_source() {
        let invalid = FIXTURE.replace(
            "tr-created|CREATED|||||dws-2025|sys-new|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:created|SYNTHETIC|SYNTHETIC_ONLY",
            "tr-created|CREATED|dws-2022|sys-old|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-new|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:created|SYNTHETIC|SYNTHETIC_ONLY",
        );
        assert!(parse(&invalid)
            .unwrap_err()
            .contains("source/target presence"));
    }

    #[test]
    fn invalid_calendar_date_is_rejected() {
        let invalid = FIXTURE.replace("2023-07-01", "2023-02-30");
        assert!(parse(&invalid)
            .unwrap_err()
            .contains("invalid effective_date"));
    }

    #[test]
    fn target_cannot_be_assigned_from_competing_transitions_on_same_date() {
        let extra = "tr-target-conflict|RENAMED|dws-2022|sys-other|MUNICIPAL_SYSTEMS|MUNICIPAL|dws-2025|sys-a|MUNICIPAL_SYSTEMS|MUNICIPAL|2023-07-01|synthetic:conflict|SYNTHETIC|SYNTHETIC_ONLY";
        assert!(parse(&format!("{FIXTURE}{extra}\n")).unwrap_err().contains(
            "target identity is assigned from competing or scope-conflicting transitions"
        ));
    }

    #[test]
    fn bounded_fifty_group_sweep_covers_all_eight_transition_relations() {
        let mut source = String::from(
            "# fixture_kind=SYNTHETIC_ONLY\n# scope=generated synthetic transition cardinality sweep\n",
        );
        source.push_str(HEADER);
        source.push('\n');

        let mut append = |transition: &str, relation: &str, source_id: &str, target_id: &str| {
            let source_system = if source_id.is_empty() { "" } else { "dws-2022" };
            let source_scope = if source_id.is_empty() {
                ""
            } else {
                "MUNICIPAL_SYSTEMS"
            };
            let source_owner = if source_id.is_empty() {
                ""
            } else {
                "MUNICIPAL"
            };
            let target_system = if target_id.is_empty() { "" } else { "dws-2025" };
            let target_scope = if target_id.is_empty() {
                ""
            } else {
                "MUNICIPAL_SYSTEMS"
            };
            let target_owner = if target_id.is_empty() {
                ""
            } else {
                "MUNICIPAL"
            };
            source.push_str(&format!(
                "{transition}|{relation}|{source_system}|{source_id}|{source_scope}|{source_owner}|{target_system}|{target_id}|{target_scope}|{target_owner}|2023-07-01|synthetic:{transition}|SYNTHETIC|SYNTHETIC_ONLY\n"
            ));
        };

        for index in 0..50 {
            let transition = format!("generated-{index:02}");
            let source_a = format!("old-a-{index:02}");
            let source_b = format!("old-b-{index:02}");
            let target_a = format!("new-a-{index:02}");
            let target_b = format!("new-b-{index:02}");
            match index % 8 {
                0 => append(&transition, "CONTINUITY", &source_a, &target_a),
                1 => append(&transition, "RENAMED", &source_a, &target_a),
                2 => append(&transition, "TRANSFERRED", &source_a, &target_a),
                3 => {
                    append(&transition, "MERGED_INTO", &source_a, &target_a);
                    append(&transition, "MERGED_INTO", &source_b, &target_a);
                }
                4 => {
                    append(&transition, "SPLIT_INTO", &source_a, &target_a);
                    append(&transition, "SPLIT_INTO", &source_a, &target_b);
                }
                5 => append(&transition, "DECOMMISSIONED", &source_a, ""),
                6 => append(&transition, "CREATED", "", &target_a),
                _ => append(&transition, "UNRESOLVED", &source_a, ""),
            }
        }

        let report = parse(&source).expect("all generated transition groups should validate");
        assert_eq!(report.groups.len(), 50);
        assert_eq!(report.row_count, 62);
        for relation in RELATIONS {
            assert!(report.groups.iter().any(|group| group.relation == relation));
        }
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
        let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), HEADER, rows.join("\n"));
        assert_eq!(
            validate_and_report(FIXTURE).expect("original fixture"),
            validate_and_report(&reordered).expect("reordered fixture")
        );
    }
}
