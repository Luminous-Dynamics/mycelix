#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

const EXPECTED_HEADER: &str = "source_id|publisher|dataset_or_product|source_url|release_or_snapshot|observation_period|geographic_grain|unit_and_denominator|ingestion_state|admissibility_note";

const ALLOWED_STATES: [&str; 5] = [
    "DISCOVERED_NOT_EXTRACTED",
    "SNAPSHOT_CAPTURED",
    "EXTRACTED_UNREVIEWED",
    "QUALIFIED_FOR_CONTEXT",
    "BLOCKED_RIGHTS_OR_GAP",
];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegisterError(pub String);

impl fmt::Display for RegisterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for RegisterError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourceRecord {
    pub source_id: String,
    pub publisher: String,
    pub dataset_or_product: String,
    pub source_url: String,
    pub release_or_snapshot: String,
    pub observation_period: String,
    pub geographic_grain: String,
    pub unit_and_denominator: String,
    pub ingestion_state: String,
    pub admissibility_note: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegisterReport {
    pub record_count: usize,
    pub state_counts: Vec<(String, usize)>,
}

impl RegisterReport {
    pub fn render(&self) -> String {
        let mut output = String::new();
        output.push_str("report_schema=fsre-source-register-report.v0.1\n");
        output.push_str(&format!("source_records={}\n", self.record_count));
        output.push_str(&format!("ingestion_states={}\n", self.state_counts.len()));
        for (state, count) in &self.state_counts {
            output.push_str(&format!("state={state}|{count}\n"));
        }
        output
    }
}

fn valid_http_url(value: &str) -> bool {
    let Some((scheme, remainder)) = value.split_once("://") else {
        return false;
    };
    if scheme != "https" && scheme != "http" {
        return false;
    }
    let authority = remainder.split('/').next().unwrap_or_default();
    !authority.is_empty()
        && !authority.contains(char::is_whitespace)
        && !authority.starts_with('.')
        && !authority.ends_with('.')
}

pub fn parse_register(source: &str) -> Result<Vec<SourceRecord>, RegisterError> {
    let mut lines = source.lines().enumerate().filter_map(|(index, raw)| {
        let line = raw.trim_end_matches('\r');
        if line.trim().is_empty() || line.trim_start().starts_with('#') {
            None
        } else {
            Some((index + 1, line))
        }
    });

    let Some((header_line, header)) = lines.next() else {
        return Err(RegisterError("source register is empty".to_owned()));
    };
    if header != EXPECTED_HEADER {
        return Err(RegisterError(format!(
            "line {header_line}: unexpected source-register header"
        )));
    }

    let mut seen_ids = BTreeSet::new();
    let mut records = Vec::new();

    for (line_number, line) in lines {
        let fields: Vec<&str> = line.split('|').collect();
        if fields.len() != 10 {
            return Err(RegisterError(format!(
                "line {line_number}: expected 10 pipe-delimited fields, received {}",
                fields.len()
            )));
        }
        let fields: Vec<String> = fields.iter().map(|value| value.trim().to_owned()).collect();
        if let Some(index) = fields.iter().position(|field| field.is_empty()) {
            return Err(RegisterError(format!(
                "line {line_number}: field {} must not be empty",
                index + 1
            )));
        }

        let source_id = &fields[0];
        if !source_id
            .chars()
            .all(|ch| ch.is_ascii_lowercase() || ch.is_ascii_digit() || ch == '_' || ch == '-')
        {
            return Err(RegisterError(format!(
                "line {line_number}: source_id must use lowercase ASCII, digits, underscore or hyphen"
            )));
        }
        if !seen_ids.insert(source_id.clone()) {
            return Err(RegisterError(format!(
                "line {line_number}: duplicate source_id {source_id}"
            )));
        }
        if !valid_http_url(&fields[3]) {
            return Err(RegisterError(format!(
                "line {line_number}: source_url must be an absolute HTTP(S) URL with a non-empty authority"
            )));
        }
        if !ALLOWED_STATES.contains(&fields[8].as_str()) {
            return Err(RegisterError(format!(
                "line {line_number}: unsupported ingestion_state {}",
                fields[8]
            )));
        }

        records.push(SourceRecord {
            source_id: fields[0].clone(),
            publisher: fields[1].clone(),
            dataset_or_product: fields[2].clone(),
            source_url: fields[3].clone(),
            release_or_snapshot: fields[4].clone(),
            observation_period: fields[5].clone(),
            geographic_grain: fields[6].clone(),
            unit_and_denominator: fields[7].clone(),
            ingestion_state: fields[8].clone(),
            admissibility_note: fields[9].clone(),
        });
    }

    if records.is_empty() {
        return Err(RegisterError(
            "source register contains no records".to_owned(),
        ));
    }

    Ok(records)
}

pub fn evaluate_register(records: &[SourceRecord]) -> Result<RegisterReport, RegisterError> {
    if records.is_empty() {
        return Err(RegisterError("source register contains no records".to_owned()));
    }

    let mut seen_ids = BTreeSet::new();
    let mut state_counts = BTreeMap::<String, usize>::new();
    for record in records {
        if !seen_ids.insert(record.source_id.as_str()) {
            return Err(RegisterError(format!(
                "duplicate source_id in evaluation input: {}",
                record.source_id
            )));
        }
        if !ALLOWED_STATES.contains(&record.ingestion_state.as_str()) {
            return Err(RegisterError(format!(
                "unsupported ingestion_state for {}: {}",
                record.source_id, record.ingestion_state
            )));
        }
        *state_counts
            .entry(record.ingestion_state.clone())
            .or_default() += 1;
    }

    Ok(RegisterReport {
        record_count: records.len(),
        state_counts: state_counts.into_iter().collect(),
    })
}

pub fn validate_and_report(source: &str) -> Result<RegisterReport, RegisterError> {
    let records = parse_register(source)?;
    evaluate_register(&records)
}

#[cfg(test)]
mod tests {
    use super::{parse_register, validate_and_report, RegisterError};

    const HEADER: &str = "source_id|publisher|dataset_or_product|source_url|release_or_snapshot|observation_period|geographic_grain|unit_and_denominator|ingestion_state|admissibility_note";

    fn row(source_id: &str, state: &str) -> String {
        format!(
            "{source_id}|Publisher|Dataset|https://example.org/data|release-1|2025|national|percent of households|{state}|Context only"
        )
    }

    #[test]
    fn valid_register_is_counted_without_promoting_source_state() {
        let input = format!(
            "{HEADER}\n{}\n{}\n",
            row("statsa_ghs", "DISCOVERED_NOT_EXTRACTED"),
            row("treasury_finance", "DISCOVERED_NOT_EXTRACTED")
        );
        let report = validate_and_report(&input).unwrap();
        assert_eq!(report.record_count, 2);
        assert_eq!(
            report.state_counts,
            vec![("DISCOVERED_NOT_EXTRACTED".to_owned(), 2)]
        );
    }

    #[test]
    fn duplicate_source_ids_are_rejected() {
        let input = format!(
            "{HEADER}\n{}\n{}\n",
            row("duplicate", "DISCOVERED_NOT_EXTRACTED"),
            row("duplicate", "DISCOVERED_NOT_EXTRACTED")
        );
        assert_eq!(
            parse_register(&input).unwrap_err(),
            RegisterError("line 3: duplicate source_id duplicate".to_owned())
        );
    }

    #[test]
    fn empty_denominator_is_rejected() {
        let input = format!(
            "{HEADER}\nsource|Publisher|Dataset|https://example.org|release|2025|national||DISCOVERED_NOT_EXTRACTED|note\n"
        );
        assert!(parse_register(&input)
            .unwrap_err()
            .0
            .contains("field 8 must not be empty"));
    }

    #[test]
    fn unsupported_ingestion_state_is_rejected() {
        let input = format!("{HEADER}\n{}\n", row("source", "QUALIFIED_BY_MAGIC"));
        assert!(parse_register(&input)
            .unwrap_err()
            .0
            .contains("unsupported ingestion_state"));
    }

    #[test]
    fn malformed_or_non_http_url_is_rejected() {
        let input = format!(
            "{HEADER}\nsource|Publisher|Dataset|javascript:alert(1)|release|2025|national|units|DISCOVERED_NOT_EXTRACTED|note\n"
        );
        assert!(parse_register(&input)
            .unwrap_err()
            .0
            .contains("absolute HTTP(S) URL"));
    }

    #[test]
    fn pipes_inside_fields_are_not_silently_accepted() {
        let input = format!(
            "{HEADER}\nsource|Publisher|Dataset|https://example.org|release|2025|national|units|DISCOVERED_NOT_EXTRACTED|note with | pipe\n"
        );
        assert!(parse_register(&input)
            .unwrap_err()
            .0
            .contains("expected 10 pipe-delimited fields"));
    }

    #[test]
    fn comment_lines_are_ignored_but_not_counted_as_records() {
        let input = format!(
            "# Register snapshot: synthetic\n{HEADER}\n{}\n",
            row("source", "SNAPSHOT_CAPTURED")
        );
        let report = validate_and_report(&input).unwrap();
        assert_eq!(report.record_count, 1);
        assert_eq!(report.state_counts[0].1, 1);
    }
}
