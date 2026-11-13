#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

const HEADER: &str =
    "fact_id|metric_id|period|metric_kind|reported_value|unit|numerator|denominator|comparability";

#[derive(Clone, Debug, PartialEq, Eq)]
struct Metadata {
    source_id: String,
    source_owner: String,
    source_ref: String,
    publication_date: String,
    observation_period: String,
    geographic_scope: String,
    source_status: String,
    report_vintage: String,
    system_level_records: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MetricKind {
    PublishedCount,
    PublishedRoundedPercent,
    PublishedRoundedPercentWithDenominator,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Fact {
    fact_id: String,
    metric_id: String,
    period: u16,
    kind: MetricKind,
    reported_value: u64,
    unit: String,
    numerator: Option<u64>,
    denominator: Option<u64>,
    comparability: String,
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
            .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-')
}

fn parse_date(value: &str) -> bool {
    let bytes = value.as_bytes();
    bytes.len() == 10
        && bytes[0..4].iter().all(u8::is_ascii_digit)
        && bytes[4] == b'-'
        && bytes[5..7].iter().all(u8::is_ascii_digit)
        && bytes[7] == b'-'
        && bytes[8..10].iter().all(u8::is_ascii_digit)
}

fn parse_metadata(
    metadata: &mut BTreeMap<String, String>,
) -> Result<Metadata, String> {
    let parsed = Metadata {
        source_id: take_required(metadata, "source_id")?,
        source_owner: take_required(metadata, "source_owner")?,
        source_ref: take_required(metadata, "source_ref")?,
        publication_date: take_required(metadata, "publication_date")?,
        observation_period: take_required(metadata, "observation_period")?,
        geographic_scope: take_required(metadata, "geographic_scope")?,
        source_status: take_required(metadata, "source_status")?,
        report_vintage: take_required(metadata, "report_vintage")?,
        system_level_records: take_required(metadata, "system_level_records")?,
    };
    if let Some(key) = metadata.keys().next() {
        return Err(format!("unknown metadata key: {key}"));
    }
    if !valid_identifier(&parsed.source_id)
        || parsed.source_owner.is_empty()
        || !parsed.source_ref.starts_with("https://")
        || !parse_date(&parsed.publication_date)
        || parsed.observation_period.is_empty()
        || parsed.geographic_scope.is_empty()
        || parsed.report_vintage.is_empty()
        || parsed.source_status != "PUBLISHED_OFFICIAL_SUMMARY"
        || parsed.system_level_records != "NOT_EXTRACTED"
    {
        return Err("invalid source metadata or source-claim boundary".to_owned());
    }
    Ok(parsed)
}

fn parse_optional_integer(value: &str, label: &str, line: usize) -> Result<Option<u64>, String> {
    if value.is_empty() {
        return Ok(None);
    }
    value
        .parse::<u64>()
        .map(Some)
        .map_err(|error| format!("line {line}: invalid {label}: {error}"))
}

fn parse_fact(line: &str, line_number: usize) -> Result<Fact, String> {
    let columns: Vec<&str> = line.split('|').map(str::trim).collect();
    if columns.len() != 9 || columns.iter().any(|value| value.is_empty() && value != &columns[6] && value != &columns[7]) {
        return Err(format!(
            "line {line_number}: expected 9 fields; only numerator and denominator may be empty"
        ));
    }

    let kind = match columns[3] {
        "PUBLISHED_COUNT" => MetricKind::PublishedCount,
        "PUBLISHED_ROUNDED_PERCENT" => MetricKind::PublishedRoundedPercent,
        "PUBLISHED_ROUNDED_PERCENT_WITH_DENOMINATOR" => {
            MetricKind::PublishedRoundedPercentWithDenominator
        }
        value => return Err(format!("line {line_number}: unknown metric kind {value}")),
    };
    let period = columns[2]
        .parse::<u16>()
        .map_err(|error| format!("line {line_number}: invalid period: {error}"))?;
    let reported_value = columns[4]
        .parse::<u64>()
        .map_err(|error| format!("line {line_number}: invalid reported_value: {error}"))?;
    let numerator = parse_optional_integer(columns[6], "numerator", line_number)?;
    let denominator = parse_optional_integer(columns[7], "denominator", line_number)?;

    if !valid_identifier(columns[0])
        || !valid_identifier(columns[1])
        || period < 2000
        || columns[5].is_empty()
        || !matches!(columns[8], "SOURCE_REPORTED_COMPARISON" | "REPORT_SCOPE_TOTAL")
    {
        return Err(format!("line {line_number}: invalid fact identity, period, unit or comparison class"));
    }

    match kind {
        MetricKind::PublishedCount => {
            if columns[5] != "systems" || numerator.is_some() || denominator.is_some() {
                return Err(format!("line {line_number}: count rows require systems and no ratio fields"));
            }
        }
        MetricKind::PublishedRoundedPercent => {
            if columns[5] != "percent" || numerator.is_some() || denominator.is_some() || reported_value > 100 {
                return Err(format!("line {line_number}: denominator-free percentage must be 0..=100 with no ratio fields"));
            }
        }
        MetricKind::PublishedRoundedPercentWithDenominator => {
            let (Some(numerator), Some(denominator)) = (numerator, denominator) else {
                return Err(format!("line {line_number}: checked percentage requires numerator and denominator"));
            };
            if columns[5] != "percent"
                || denominator == 0
                || numerator > denominator
                || reported_value > 100
            {
                return Err(format!("line {line_number}: invalid percentage ratio"));
            }
            let rounded = ((u128::from(numerator) * 100) + (u128::from(denominator) / 2))
                / u128::from(denominator);
            if rounded != u128::from(reported_value) {
                return Err(format!(
                    "line {line_number}: published percentage {} does not match rounded ratio {} / {} = {}",
                    reported_value, numerator, denominator, rounded
                ));
            }
        }
    }

    Ok(Fact {
        fact_id: columns[0].to_owned(),
        metric_id: columns[1].to_owned(),
        period,
        kind,
        reported_value,
        unit: columns[5].to_owned(),
        numerator,
        denominator,
        comparability: columns[8].to_owned(),
    })
}

fn parse(source: &str) -> Result<(Metadata, Vec<Fact>), String> {
    let mut metadata = BTreeMap::new();
    let mut facts = Vec::new();
    let mut header_seen = false;
    let mut fact_ids = BTreeSet::new();
    let mut metric_periods = BTreeSet::new();

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
                return Err(format!("line {line_number}: unexpected fact header"));
            }
            header_seen = true;
            continue;
        }

        let fact = parse_fact(line, line_number)?;
        if !fact_ids.insert(fact.fact_id.clone()) {
            return Err(format!("line {line_number}: duplicate fact ID {}", fact.fact_id));
        }
        if !metric_periods.insert((fact.metric_id.clone(), fact.period)) {
            return Err(format!(
                "line {line_number}: duplicate metric-period {}-{}",
                fact.metric_id, fact.period
            ));
        }
        facts.push(fact);
    }

    if !header_seen || facts.is_empty() {
        return Err("missing fact header or fact rows".to_owned());
    }
    Ok((parse_metadata(&mut metadata)?, facts))
}

fn lookup(facts: &[Fact], metric_id: &str, period: u16) -> Option<u64> {
    facts
        .iter()
        .find(|fact| fact.metric_id == metric_id && fact.period == period)
        .map(|fact| fact.reported_value)
}

fn render(metadata: &Metadata, facts: &[Fact]) -> String {
    let mut output = String::new();
    output.push_str("report_schema=fsre-public-source-replay.v0.1\n");
    output.push_str(&format!("source_id={}\n", metadata.source_id));
    output.push_str(&format!("source_owner={}\n", metadata.source_owner));
    output.push_str(&format!("source_ref={}\n", metadata.source_ref));
    output.push_str(&format!("publication_date={}\n", metadata.publication_date));
    output.push_str(&format!("observation_period={}\n", metadata.observation_period));
    output.push_str(&format!("geographic_scope={}\n", metadata.geographic_scope));
    output.push_str(&format!("report_vintage={}\n", metadata.report_vintage));
    output.push_str(&format!("source_status={}\n", metadata.source_status));
    output.push_str(&format!("system_level_records={}\n", metadata.system_level_records));

    let mut canonical = facts.to_vec();
    canonical.sort_by(|left, right| left.fact_id.cmp(&right.fact_id));
    for fact in &canonical {
        output.push_str(&format!(
            "fact.{}.period={}\n",
            fact.fact_id, fact.period
        ));
        output.push_str(&format!(
            "fact.{}.value={}\n",
            fact.fact_id, fact.reported_value
        ));
        output.push_str(&format!("fact.{}.unit={}\n", fact.fact_id, fact.unit));
        output.push_str(&format!(
            "fact.{}.comparability={}\n",
            fact.fact_id, fact.comparability
        ));
        match fact.kind {
            MetricKind::PublishedCount => {
                output.push_str(&format!(
                    "fact.{}.derivation=SOURCE_REPORTED_COUNT\n",
                    fact.fact_id
                ));
            }
            MetricKind::PublishedRoundedPercent => {
                output.push_str(&format!(
                    "fact.{}.derivation=SOURCE_REPORTED_PERCENT;DENOMINATOR_NOT_PUBLISHED\n",
                    fact.fact_id
                ));
            }
            MetricKind::PublishedRoundedPercentWithDenominator => {
                output.push_str(&format!(
                    "fact.{}.derivation=RATIO_RECHECKED;NUMERATOR={};DENOMINATOR={};ROUNDING=HALF_UP\n",
                    fact.fact_id,
                    fact.numerator.expect("validated numerator"),
                    fact.denominator.expect("validated denominator")
                ));
            }
        }
    }

    if let (Some(critical_2022), Some(critical_2025)) = (
        lookup(facts, "critical_share", 2022),
        lookup(facts, "critical_share", 2025),
    ) {
        output.push_str(&format!(
            "critical_share_change_percentage_points={}\n",
            i64::try_from(critical_2025).unwrap_or(i64::MAX)
                - i64::try_from(critical_2022).unwrap_or(i64::MAX)
        ));
    }
    if let (Some(good_2022), Some(good_2025)) = (
        lookup(facts, "good_excellent_share", 2022),
        lookup(facts, "good_excellent_share", 2025),
    ) {
        output.push_str(&format!(
            "good_or_excellent_share_change_percentage_points={}\n",
            i64::try_from(good_2025).unwrap_or(i64::MAX)
                - i64::try_from(good_2022).unwrap_or(i64::MAX)
        ));
    }

    output.push_str("claim_class=PUBLISHED_OFFICIAL_SUMMARY_ONLY\n");
    output.push_str("comparison_basis=SOURCE_REPORTED;NOT_INDEPENDENTLY_REASSESSED\n");
    output.push_str("2022_SYSTEM_DENOMINATOR=UNKNOWN\n");
    output.push_str("COUNT_DIFFERENCE_INTERPRETATION=WITHHELD_DIFFERING_OR_UNPUBLISHED_DENOMINATORS\n");
    output.push_str("SYSTEM_LEVEL_REPLAY=NOT_PERFORMED\n");
    output.push_str("CAUSAL_EFFECT=NOT_ESTABLISHED\n");
    output.push_str("INTERVENTION_AUTHORITY=NONE\n");
    output.push_str("VALIDATION_SCOPE=SCHEMA_ARITHMETIC_AND_DECLARED_SOURCE_BOUNDARY_ONLY\n");
    output
}

pub fn validate_and_report(source: &str) -> Result<String, String> {
    let (metadata, facts) = parse(source)?;
    Ok(render(&metadata, &facts))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../fixtures/ZA_GREEN_DROP_PUBLIC_SUMMARY_2025.psv");

    #[test]
    fn official_summary_fixture_is_parseable_and_bounded() {
        let (metadata, facts) = parse(FIXTURE).expect("fixture should validate");
        assert_eq!(metadata.source_status, "PUBLISHED_OFFICIAL_SUMMARY");
        assert_eq!(metadata.system_level_records, "NOT_EXTRACTED");
        assert_eq!(facts.len(), 11);
    }

    #[test]
    fn published_critical_ratio_recomputes_to_rounded_47_percent() {
        assert_eq!((396_u128 * 100 + 848 / 2) / 848, 47);
    }

    #[test]
    fn published_good_or_excellent_ratio_recomputes_to_rounded_8_percent() {
        assert_eq!((66_u128 * 100 + 848 / 2) / 848, 8);
    }

    #[test]
    fn denominator_free_2022_rates_are_not_reconstructed() {
        let (report, _) = parse(FIXTURE).expect("fixture should validate");
        assert_eq!(report.observation_period, "2023/24 municipal financial year");
        let critical = FIXTURE
            .lines()
            .find(|line| line.starts_with("critical_share_2022|"))
            .expect("2022 critical share exists");
        let fields: Vec<&str> = critical.split('|').collect();
        assert!(fields[6].is_empty());
        assert!(fields[7].is_empty());
    }

    #[test]
    fn a_rounded_rate_mismatch_is_rejected() {
        let mutated = FIXTURE.replace(
            "critical_share_2025|critical_share|2025|PUBLISHED_ROUNDED_PERCENT_WITH_DENOMINATOR|47|percent|396|848|",
            "critical_share_2025|critical_share|2025|PUBLISHED_ROUNDED_PERCENT_WITH_DENOMINATOR|46|percent|396|848|",
        );
        assert!(parse(&mutated).is_err());
    }

    #[test]
    fn a_zero_denominator_is_rejected() {
        let mutated = FIXTURE.replace("|396|848|SOURCE_REPORTED_COMPARISON", "|396|0|SOURCE_REPORTED_COMPARISON");
        assert!(parse(&mutated).is_err());
    }

    #[test]
    fn a_missing_ratio_component_is_rejected() {
        let mutated = FIXTURE.replace("|396|848|SOURCE_REPORTED_COMPARISON", "|396||SOURCE_REPORTED_COMPARISON");
        assert!(parse(&mutated).is_err());
    }

    #[test]
    fn denominator_free_source_rates_cannot_smuggle_in_a_denominator() {
        let mutated = FIXTURE.replace(
            "critical_share_2022|critical_share|2022|PUBLISHED_ROUNDED_PERCENT|39|percent|||",
            "critical_share_2022|critical_share|2022|PUBLISHED_ROUNDED_PERCENT|39|percent|334|848|",
        );
        assert!(parse(&mutated).is_err());
    }

    #[test]
    fn duplicate_fact_ids_are_rejected() {
        let row = FIXTURE
            .lines()
            .find(|line| line.starts_with("critical_share_2022|"))
            .expect("fact exists");
        let duplicate = format!("{FIXTURE}{row}\n");
        assert!(parse(&duplicate).is_err());
    }

    #[test]
    fn unknown_source_metadata_is_rejected() {
        let mutated = FIXTURE.replace(
            "# source_status=PUBLISHED_OFFICIAL_SUMMARY",
            "# source_status=PUBLISHED_OFFICIAL_SUMMARY\n# confidence=0.99",
        );
        assert!(parse(&mutated).is_err());
    }

    #[test]
    fn summary_report_never_claims_a_field_or_causal_result() {
        let report = validate_and_report(FIXTURE).expect("fixture should validate");
        assert!(report.contains("CAUSAL_EFFECT=NOT_ESTABLISHED"));
        assert!(report.contains("INTERVENTION_AUTHORITY=NONE"));
        assert!(report.contains("SYSTEM_LEVEL_REPLAY=NOT_PERFORMED"));
        assert!(report.contains("critical_share_change_percentage_points=8"));
        assert!(report.contains("good_or_excellent_share_change_percentage_points=-6"));
    }
}
