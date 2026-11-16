//! Local-only BIDS cohort metadata intake audit.
//!
//! This tool counts directories and files and checks the first column of
//! participants.tsv. It never opens neuroimaging/electrophysiology payloads,
//! never emits participant identifiers, and is not a substitute for the
//! official BIDS Validator.

use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io::{BufRead, BufReader};
use std::path::Path;

pub const DEFAULT_MIN_REPORT_N: usize = 20;
const MODALITIES: [&str; 12] = [
    "anat", "func", "dwi", "fmap", "eeg", "meg", "ieeg", "beh", "perf", "pet", "micr", "motion",
];

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AggregateCounts {
    pub subject_directories: usize,
    pub session_directories: usize,
    pub participant_table_rows: usize,
    pub participant_table_unique_valid_ids: usize,
    pub participant_table_duplicate_rows: usize,
    pub participant_table_invalid_id_rows: usize,
    pub participant_ids_without_subject_directory: usize,
    pub subject_directories_missing_from_participant_table: usize,
    pub symlink_entries_skipped: usize,
    pub modality_file_counts: [usize; MODALITIES.len()],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IntakeAudit {
    pub dataset_description_present: bool,
    pub participants_tsv_present: bool,
    pub scan_complete: bool,
    pub finding_codes: BTreeSet<&'static str>,
    pub counts: AggregateCounts,
}

impl IntakeAudit {
    fn finding(&mut self, code: &'static str) {
        self.finding_codes.insert(code);
    }

    fn scan_incomplete(&mut self, code: &'static str) {
        self.scan_complete = false;
        self.finding(code);
    }
}

/// Audit high-level BIDS cohort metadata without reading modality payloads.
/// The root must be a real directory, not a symlink. Directory traversal never
/// follows symlinks. The returned object contains no paths or subject IDs.
pub fn audit_dataset(dataset_root: &Path) -> Result<IntakeAudit, String> {
    let root_meta = fs::symlink_metadata(dataset_root)
        .map_err(|_| "dataset root is unavailable".to_string())?;
    if root_meta.file_type().is_symlink() || !root_meta.is_dir() {
        return Err("dataset root must be a non-symlink directory".to_string());
    }
    let root = fs::canonicalize(dataset_root)
        .map_err(|_| "dataset root could not be canonicalized".to_string())?;

    let description_present = is_regular_non_symlink_file(&root.join("dataset_description.json"));
    let participants_path = root.join("participants.tsv");
    let participants_present = is_regular_non_symlink_file(&participants_path);

    let mut audit = IntakeAudit {
        dataset_description_present: description_present,
        participants_tsv_present: participants_present,
        scan_complete: true,
        finding_codes: BTreeSet::new(),
        counts: AggregateCounts::default(),
    };

    if !description_present {
        audit.finding("dataset_description_missing");
    }

    let mut subject_names = BTreeSet::new();
    scan_subjects(&root, &mut audit, &mut subject_names)?;

    if audit.counts.subject_directories == 0 {
        audit.finding("no_subject_directories");
    }

    if participants_present {
        match read_participants(&participants_path) {
            Ok(table) => {
                audit.counts.participant_table_rows = table.rows;
                audit.counts.participant_table_unique_valid_ids = table.ids.len();
                audit.counts.participant_table_duplicate_rows = table.duplicate_rows;
                audit.counts.participant_table_invalid_id_rows = table.invalid_rows;

                if table.duplicate_rows > 0 {
                    audit.finding("duplicate_participant_id");
                }
                if table.invalid_rows > 0 {
                    audit.finding("invalid_participant_id");
                }

                audit.counts.participant_ids_without_subject_directory =
                    table.ids.difference(&subject_names).count();
                audit
                    .counts
                    .subject_directories_missing_from_participant_table =
                    subject_names.difference(&table.ids).count();

                if audit.counts.participant_ids_without_subject_directory > 0 {
                    audit.finding("participant_ids_without_subject_directory");
                }
                if audit
                    .counts
                    .subject_directories_missing_from_participant_table
                    > 0
                {
                    audit.finding("subject_directories_missing_from_participant_table");
                }
            }
            Err(code) => audit.scan_incomplete(code),
        }
    } else if fs::symlink_metadata(&participants_path).is_ok() {
        audit.scan_incomplete("participants_tsv_unreadable");
    } else {
        // participants.tsv is recommended but is not required by every BIDS dataset.
        audit.finding("participants_tsv_missing_warning");
    }

    if audit.counts.symlink_entries_skipped > 0 {
        audit.finding("symlink_entries_skipped");
    }

    Ok(audit)
}

struct ParticipantsTable {
    rows: usize,
    ids: BTreeSet<String>,
    duplicate_rows: usize,
    invalid_rows: usize,
}

fn read_participants(path: &Path) -> Result<ParticipantsTable, &'static str> {
    let metadata = fs::symlink_metadata(path).map_err(|_| "participants_tsv_unreadable")?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err("participants_tsv_unreadable");
    }

    let file = File::open(path).map_err(|_| "participants_tsv_unreadable")?;
    let mut reader = BufReader::new(file);
    let mut header = read_first_column(&mut reader)
        .map_err(|_| "participants_tsv_unreadable")?
        .ok_or("participants_id_column_missing")?;
    if header.starts_with(&[0xEF, 0xBB, 0xBF]) {
        header.drain(..3);
    }
    if header.as_slice() != b"participant_id" {
        return Err("participants_id_column_missing");
    }

    let mut table = ParticipantsTable {
        rows: 0,
        ids: BTreeSet::new(),
        duplicate_rows: 0,
        invalid_rows: 0,
    };

    while let Some(value) =
        read_first_column(&mut reader).map_err(|_| "participants_tsv_unreadable")?
    {
        if value.is_empty() {
            continue;
        }
        table.rows += 1;
        let value = std::str::from_utf8(&value).unwrap_or("");
        if !valid_subject_name(value) {
            table.invalid_rows += 1;
            continue;
        }
        if !table.ids.insert(value.to_string()) {
            table.duplicate_rows += 1;
        }
    }

    Ok(table)
}

/// Read only the first TSV field. Remaining columns are discarded in-place
/// through BufRead::consume and are never materialized as strings or parsed.
fn read_first_column(reader: &mut impl BufRead) -> std::io::Result<Option<Vec<u8>>> {
    const MAX_ID_BYTES: usize = 128;
    let mut field = Vec::new();

    loop {
        let available = reader.fill_buf()?;
        if available.is_empty() {
            if field.is_empty() {
                return Ok(None);
            }
            trim_line_ending(&mut field);
            return Ok(Some(field));
        }

        let tab = available.iter().position(|byte| *byte == b'\t');
        let newline = available.iter().position(|byte| *byte == b'\n');
        let delimiter = match (tab, newline) {
            (Some(tab), Some(newline)) => Some((tab.min(newline), tab < newline)),
            (Some(tab), None) => Some((tab, true)),
            (None, Some(newline)) => Some((newline, false)),
            (None, None) => None,
        };

        if let Some((position, is_tab)) = delimiter {
            if field.len() + position > MAX_ID_BYTES {
                reader.consume(position + 1);
                if is_tab {
                    discard_to_line_end(reader)?;
                }
                return Ok(Some(Vec::new()));
            }
            field.extend_from_slice(&available[..position]);
            reader.consume(position + 1);
            if is_tab {
                discard_to_line_end(reader)?;
            }
            trim_line_ending(&mut field);
            return Ok(Some(field));
        }

        if field.len() + available.len() > MAX_ID_BYTES {
            let length = available.len();
            reader.consume(length);
            discard_to_line_end(reader)?;
            return Ok(Some(Vec::new()));
        }

        let length = available.len();
        field.extend_from_slice(available);
        reader.consume(length);
    }
}

fn discard_to_line_end(reader: &mut impl BufRead) -> std::io::Result<()> {
    loop {
        let available = reader.fill_buf()?;
        if available.is_empty() {
            return Ok(());
        }
        if let Some(newline) = available.iter().position(|byte| *byte == b'\n') {
            reader.consume(newline + 1);
            return Ok(());
        }
        let length = available.len();
        reader.consume(length);
    }
}

fn trim_line_ending(value: &mut Vec<u8>) {
    if value.last() == Some(&b'\r') {
        value.pop();
    }
}
fn is_regular_non_symlink_file(path: &Path) -> bool {
    fs::symlink_metadata(path)
        .map(|metadata| metadata.is_file() && !metadata.file_type().is_symlink())
        .unwrap_or(false)
}

fn scan_subjects(
    root: &Path,
    audit: &mut IntakeAudit,
    subject_names: &mut BTreeSet<String>,
) -> Result<(), String> {
    let entries =
        fs::read_dir(root).map_err(|_| "dataset root could not be enumerated".to_string())?;

    for entry in entries {
        let entry = entry.map_err(|_| "dataset root could not be enumerated".to_string())?;
        let file_type = entry
            .file_type()
            .map_err(|_| "dataset root entry metadata unavailable".to_string())?;
        if file_type.is_symlink() {
            audit.counts.symlink_entries_skipped += 1;
            continue;
        }
        if !file_type.is_dir() {
            continue;
        }

        let name = entry.file_name();
        let name = name.to_string_lossy();
        if !name.starts_with("sub-") {
            continue;
        }

        audit.counts.subject_directories += 1;
        if !valid_subject_name(&name) {
            audit.finding("invalid_subject_directory_name");
            continue;
        }

        subject_names.insert(name.to_string());
        scan_subject_directory(&entry.path(), audit)?;
    }

    Ok(())
}

fn scan_subject_directory(subject_path: &Path, audit: &mut IntakeAudit) -> Result<(), String> {
    let entries = fs::read_dir(subject_path)
        .map_err(|_| "subject directory could not be enumerated".to_string())?;

    for entry in entries {
        let entry = entry.map_err(|_| "subject directory could not be enumerated".to_string())?;
        let file_type = entry
            .file_type()
            .map_err(|_| "subject entry metadata unavailable".to_string())?;
        if file_type.is_symlink() {
            audit.counts.symlink_entries_skipped += 1;
            continue;
        }
        if !file_type.is_dir() {
            continue;
        }

        let name = entry.file_name();
        let name = name.to_string_lossy();
        if let Some(index) = modality_index(&name) {
            audit.counts.modality_file_counts[index] += count_files_without_following_symlinks(
                &entry.path(),
                &mut audit.counts.symlink_entries_skipped,
            )?;
        } else if let Some(label) = name.strip_prefix("ses-") {
            if !valid_label(label) {
                audit.finding("invalid_session_directory_name");
                continue;
            }
            audit.counts.session_directories += 1;
            scan_session_directory(&entry.path(), audit)?;
        }
    }

    Ok(())
}

fn scan_session_directory(session_path: &Path, audit: &mut IntakeAudit) -> Result<(), String> {
    let entries = fs::read_dir(session_path)
        .map_err(|_| "session directory could not be enumerated".to_string())?;

    for entry in entries {
        let entry = entry.map_err(|_| "session directory could not be enumerated".to_string())?;
        let file_type = entry
            .file_type()
            .map_err(|_| "session entry metadata unavailable".to_string())?;
        if file_type.is_symlink() {
            audit.counts.symlink_entries_skipped += 1;
            continue;
        }
        if !file_type.is_dir() {
            continue;
        }

        let name = entry.file_name();
        let name = name.to_string_lossy();
        if let Some(index) = modality_index(&name) {
            audit.counts.modality_file_counts[index] += count_files_without_following_symlinks(
                &entry.path(),
                &mut audit.counts.symlink_entries_skipped,
            )?;
        }
    }

    Ok(())
}

fn count_files_without_following_symlinks(
    directory: &Path,
    symlinks_skipped: &mut usize,
) -> Result<usize, String> {
    let entries = fs::read_dir(directory)
        .map_err(|_| "modality directory could not be enumerated".to_string())?;
    let mut count = 0usize;

    for entry in entries {
        let entry = entry.map_err(|_| "modality directory could not be enumerated".to_string())?;
        let file_type = entry
            .file_type()
            .map_err(|_| "modality entry metadata unavailable".to_string())?;
        if file_type.is_symlink() {
            *symlinks_skipped += 1;
        } else if file_type.is_dir() {
            count += count_files_without_following_symlinks(&entry.path(), symlinks_skipped)?;
        } else if file_type.is_file() {
            count += 1;
        }
    }

    Ok(count)
}

fn modality_index(name: &str) -> Option<usize> {
    MODALITIES.iter().position(|modality| *modality == name)
}

fn valid_subject_name(value: &str) -> bool {
    value.strip_prefix("sub-").map(valid_label).unwrap_or(false)
}

fn valid_label(value: &str) -> bool {
    !value.is_empty() && value.bytes().all(|byte| byte.is_ascii_alphanumeric())
}

/// Render aggregate-only JSON. Counts are withheld if the cohort is below the
/// minimum denominator or if any required filesystem/table scan was incomplete.
pub fn render_json(audit: &IntakeAudit, minimum_subject_count: usize) -> String {
    let minimum_subject_count = minimum_subject_count.max(DEFAULT_MIN_REPORT_N);
    let suppressed = audit.counts.subject_directories < minimum_subject_count;
    let status = if !audit.scan_complete {
        "incomplete"
    } else if suppressed {
        "suppressed"
    } else {
        "complete"
    };

    let mut output = String::new();
    output.push_str("{\n");
    output.push_str("  \"schema_version\": \"neurotwin-bids-intake-receipt-v0.1\",\n");
    output.push_str(&format!("  \"status\": \"{status}\",\n"));
    output.push_str("  \"official_bids_validator_required\": true,\n");
    output.push_str("  \"raw_modality_contents_read\": false,\n");
    output.push_str("  \"participant_identifiers_emitted\": false,\n");
    output.push_str(&format!(
        "  \"dataset_description_present\": {},\n",
        audit.dataset_description_present
    ));
    output.push_str(&format!(
        "  \"participants_tsv_present\": {},\n",
        audit.participants_tsv_present
    ));
    output.push_str(&format!(
        "  \"minimum_subject_count\": {minimum_subject_count},\n"
    ));

    output.push_str("  \"finding_codes\": [");
    for (index, code) in audit.finding_codes.iter().enumerate() {
        if index > 0 {
            output.push_str(", ");
        }
        output.push('"');
        output.push_str(code);
        output.push('"');
    }
    output.push_str("],\n");

    if status == "complete" {
        let counts = &audit.counts;
        output.push_str("  \"aggregate_counts\": {\n");
        output.push_str(&format!(
            "    \"subject_directories\": {},\n",
            counts.subject_directories
        ));
        output.push_str(&format!(
            "    \"session_directories\": {},\n",
            counts.session_directories
        ));
        output.push_str(&format!(
            "    \"participant_table_rows\": {},\n",
            counts.participant_table_rows
        ));
        output.push_str(&format!(
            "    \"participant_table_unique_valid_ids\": {},\n",
            counts.participant_table_unique_valid_ids
        ));
        output.push_str(&format!(
            "    \"participant_table_duplicate_rows\": {},\n",
            counts.participant_table_duplicate_rows
        ));
        output.push_str(&format!(
            "    \"participant_table_invalid_id_rows\": {},\n",
            counts.participant_table_invalid_id_rows
        ));
        output.push_str(&format!(
            "    \"participant_ids_without_subject_directory\": {},\n",
            counts.participant_ids_without_subject_directory
        ));
        output.push_str(&format!(
            "    \"subject_directories_missing_from_participant_table\": {},\n",
            counts.subject_directories_missing_from_participant_table
        ));
        output.push_str(&format!(
            "    \"symlink_entries_skipped\": {},\n",
            counts.symlink_entries_skipped
        ));
        output.push_str("    \"modality_file_counts\": {");
        for (index, modality) in MODALITIES.iter().enumerate() {
            if index > 0 {
                output.push_str(", ");
            }
            output.push('"');
            output.push_str(modality);
            output.push_str("\": ");
            output.push_str(&counts.modality_file_counts[index].to_string());
        }
        output.push_str("}\n");
        output.push_str("  }\n");
    } else {
        output.push_str("  \"aggregate_counts\": null\n");
    }

    output.push_str("}\n");
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicUsize, Ordering};

    static NEXT_ID: AtomicUsize = AtomicUsize::new(0);

    struct TempDataset {
        path: PathBuf,
    }

    impl TempDataset {
        fn new() -> Self {
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "neurotwin-intake-audit-{}-{id}",
                std::process::id()
            ));
            let _ = fs::remove_dir_all(&path);
            fs::create_dir_all(&path).unwrap();
            Self { path }
        }

        fn add_subject(&self, label: &str) {
            fs::create_dir_all(self.path.join(format!("sub-{label}"))).unwrap();
        }

        fn add_participants(&self, ids: &[&str]) {
            let mut text = String::from("participant_id\tage\n");
            for id in ids {
                text.push_str(id);
                text.push_str("\t30\n");
            }
            fs::write(self.path.join("participants.tsv"), text).unwrap();
        }

        fn add_subjects_and_table(&self, count: usize) {
            fs::write(self.path.join("dataset_description.json"), "{}\n").unwrap();
            let mut ids = Vec::new();
            for index in 0..count {
                let label = format!("s{:03}", index + 1);
                self.add_subject(&label);
                ids.push(format!("sub-{label}"));
            }
            let refs: Vec<&str> = ids.iter().map(String::as_str).collect();
            self.add_participants(&refs);
        }
    }

    impl Drop for TempDataset {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.path);
        }
    }

    #[test]
    fn complete_aggregate_audit_counts_subjects_sessions_and_modalities() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        fs::create_dir_all(dataset.path.join("sub-s001/anat")).unwrap();
        fs::write(
            dataset.path.join("sub-s001/anat/sub-s001_T1w.json"),
            "SYNTHETIC_ONLY_SENTINEL",
        )
        .unwrap();
        fs::create_dir_all(dataset.path.join("sub-s001/ses-01/func")).unwrap();
        fs::write(
            dataset
                .path
                .join("sub-s001/ses-01/func/sub-s001_ses-01_bold.json"),
            "{}",
        )
        .unwrap();

        let audit = audit_dataset(&dataset.path).unwrap();
        assert!(audit.scan_complete);
        assert_eq!(audit.counts.subject_directories, 21);
        assert_eq!(audit.counts.session_directories, 1);
        assert_eq!(audit.counts.modality_file_counts[0], 1);
        assert_eq!(audit.counts.modality_file_counts[1], 1);
        assert_eq!(audit.counts.participant_table_rows, 21);
        assert!(audit.finding_codes.is_empty());

        let receipt = render_json(&audit, DEFAULT_MIN_REPORT_N);
        assert!(receipt.contains("\"status\": \"complete\""));
        assert!(!receipt.contains("sub-s001"));
        assert!(!receipt.contains("SYNTHETIC_ONLY_SENTINEL"));
    }

    #[test]
    fn missing_participants_tsv_is_a_warning_not_a_failed_scan() {
        let dataset = TempDataset::new();
        fs::write(dataset.path.join("dataset_description.json"), "{}").unwrap();
        for index in 0..21 {
            dataset.add_subject(&format!("s{:03}", index + 1));
        }

        let audit = audit_dataset(&dataset.path).unwrap();
        assert!(audit.scan_complete);
        assert!(!audit.participants_tsv_present);
        assert!(
            audit
                .finding_codes
                .contains("participants_tsv_missing_warning")
        );
    }

    #[test]
    fn missing_dataset_description_is_reported() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        fs::remove_file(dataset.path.join("dataset_description.json")).unwrap();

        let audit = audit_dataset(&dataset.path).unwrap();
        assert!(!audit.dataset_description_present);
        assert!(audit.finding_codes.contains("dataset_description_missing"));
    }

    #[test]
    fn wrong_participant_header_makes_audit_incomplete() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        fs::write(
            dataset.path.join("participants.tsv"),
            "subject_id\tage\nsub-s001\t30\n",
        )
        .unwrap();

        let audit = audit_dataset(&dataset.path).unwrap();
        assert!(!audit.scan_complete);
        assert!(
            audit
                .finding_codes
                .contains("participants_id_column_missing")
        );
        assert!(render_json(&audit, DEFAULT_MIN_REPORT_N).contains("\"aggregate_counts\": null"));
    }

    #[test]
    fn duplicate_participant_ids_are_counted_without_emitting_the_ids() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        let ids = vec!["sub-s001"; 21];
        dataset.add_participants(&ids);

        let audit = audit_dataset(&dataset.path).unwrap();
        assert_eq!(audit.counts.participant_table_duplicate_rows, 20);
        assert!(audit.finding_codes.contains("duplicate_participant_id"));
        let receipt = render_json(&audit, DEFAULT_MIN_REPORT_N);
        assert!(!receipt.contains("sub-s001"));
    }

    #[test]
    fn unmatched_subjects_and_participant_ids_are_counted_aggregately() {
        let dataset = TempDataset::new();
        fs::write(dataset.path.join("dataset_description.json"), "{}").unwrap();
        for index in 0..21 {
            dataset.add_subject(&format!("s{:03}", index + 1));
        }
        let mut ids: Vec<String> = (1..=20).map(|i| format!("sub-s{i:03}")).collect();
        ids.push("sub-foreign".into());
        let refs: Vec<&str> = ids.iter().map(String::as_str).collect();
        dataset.add_participants(&refs);

        let audit = audit_dataset(&dataset.path).unwrap();
        assert_eq!(audit.counts.participant_ids_without_subject_directory, 1);
        assert_eq!(
            audit
                .counts
                .subject_directories_missing_from_participant_table,
            1
        );
        assert!(
            audit
                .finding_codes
                .contains("participant_ids_without_subject_directory")
        );
        assert!(
            audit
                .finding_codes
                .contains("subject_directories_missing_from_participant_table")
        );
    }

    #[test]
    fn invalid_participant_identifiers_are_rejected() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        let mut ids: Vec<String> = (1..=20).map(|i| format!("sub-s{i:03}")).collect();
        ids.push("../unsafe".into());
        let refs: Vec<&str> = ids.iter().map(String::as_str).collect();
        dataset.add_participants(&refs);

        let audit = audit_dataset(&dataset.path).unwrap();
        assert_eq!(audit.counts.participant_table_invalid_id_rows, 1);
        assert!(audit.finding_codes.contains("invalid_participant_id"));
    }

    #[test]
    fn small_cohort_receipt_suppresses_all_counts() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(3);
        let audit = audit_dataset(&dataset.path).unwrap();
        let receipt = render_json(&audit, DEFAULT_MIN_REPORT_N);
        assert!(receipt.contains("\"status\": \"suppressed\""));
        assert!(receipt.contains("\"aggregate_counts\": null"));
        assert!(!receipt.contains("\"subject_directories\":"));
        assert!(!receipt.contains("sub-s001"));
    }

    #[test]
    fn receipt_is_order_invariant_and_contains_no_paths_or_subject_labels() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        let receipt_one = render_json(&audit_dataset(&dataset.path).unwrap(), 20);

        let reversed: Vec<String> = (1..=21).rev().map(|i| format!("sub-s{i:03}")).collect();
        let refs: Vec<&str> = reversed.iter().map(String::as_str).collect();
        dataset.add_participants(&refs);
        let receipt_two = render_json(&audit_dataset(&dataset.path).unwrap(), 20);

        assert_eq!(receipt_one, receipt_two);
        assert!(!receipt_one.contains("participants.tsv"));
        assert!(!receipt_one.contains("dataset_description.json"));
    }

    #[cfg(unix)]
    #[test]
    fn symlinks_are_not_followed_outside_dataset_root() {
        use std::os::unix::fs::symlink;

        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(21);
        let external = TempDataset::new();
        external.add_subject("external");
        symlink(&external.path, dataset.path.join("sub-s999")).unwrap();

        let audit = audit_dataset(&dataset.path).unwrap();
        assert_eq!(audit.counts.subject_directories, 21);
        assert_eq!(audit.counts.symlink_entries_skipped, 1);
        assert!(audit.finding_codes.contains("symlink_entries_skipped"));
    }

    #[cfg(unix)]
    #[test]
    fn symlink_root_is_rejected() {
        use std::os::unix::fs::symlink;

        let dataset = TempDataset::new();
        let external = TempDataset::new();
        external.add_subject("s001");
        let link = dataset.path.join("root-link");
        symlink(&external.path, &link).unwrap();
        assert!(audit_dataset(&link).is_err());
    }

    #[test]
    fn library_renderer_cannot_lower_the_default_suppression_floor() {
        let dataset = TempDataset::new();
        dataset.add_subjects_and_table(3);
        let audit = audit_dataset(&dataset.path).unwrap();
        let receipt = render_json(&audit, 1);
        assert!(receipt.contains("\"status\": \"suppressed\""));
        assert!(receipt.contains("\"aggregate_counts\": null"));
    }

    #[test]
    fn non_directory_roots_are_rejected() {
        let dataset = TempDataset::new();
        let file = dataset.path.join("file.txt");
        fs::write(&file, "x").unwrap();
        assert!(audit_dataset(&file).is_err());
    }
}
