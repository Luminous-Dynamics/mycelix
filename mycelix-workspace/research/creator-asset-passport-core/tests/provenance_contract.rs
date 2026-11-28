use creator_asset_passport_core::provenance::{
    DiagnosticSeverity, FieldMapping, ImportDiagnostic, ImportOutcome, ImportProvenanceManifest,
    InventoryStatus, ProvenanceError, SourceDocument,
};

fn source(payload: &[u8], parser_version: &str) -> SourceDocument {
    SourceDocument::capture(
        "platform-export",
        Some("export-42"),
        "ddex-rdr-n",
        "1.5",
        "cscp-mapper",
        parser_version,
        1_797_000_000,
        Some("application/xml"),
        "vault://source/export-42",
        payload,
    )
    .unwrap()
}

fn clean_fields() -> Vec<FieldMapping> {
    vec![
        FieldMapping::mapped(
            "/Message/ResourceReference",
            "/assets/recording-a/external_ids/0",
            "resource-id",
            "1",
        )
        .unwrap(),
        FieldMapping::mapped("/Message/MessageId", "/source_record/id", "message-id", "1").unwrap(),
    ]
}

fn clean_manifest(payload: &[u8], parser_version: &str) -> ImportProvenanceManifest {
    ImportProvenanceManifest::new(
        source(payload, parser_version),
        InventoryStatus::CompleteAsReportedByParser,
        ImportOutcome::Imported,
        "00".repeat(32),
        Some("11".repeat(32).as_str()),
        clean_fields(),
        vec![],
    )
    .unwrap()
}

#[test]
fn captured_raw_payload_is_bound_to_exact_bytes() {
    let payload = b"<Message><Id>42</Id></Message>";
    let source = source(payload, "1.0");
    assert!(source.verify_payload(payload).is_ok());
    assert_eq!(
        source.verify_payload(b"<Message><Id>43</Id></Message>"),
        Err(ProvenanceError::PayloadDigestMismatch)
    );
    assert_eq!(
        source.verify_payload(b"short"),
        Err(ProvenanceError::PayloadLengthMismatch)
    );
}

#[test]
fn reordering_input_field_mappings_does_not_change_manifest_bytes() {
    let source_a = source(b"same raw source", "1.0");
    let source_b = source(b"same raw source", "1.0");
    let mut fields = clean_fields();
    let mut reversed = fields.clone();
    reversed.reverse();

    let a = ImportProvenanceManifest::new(
        source_a,
        InventoryStatus::CompleteAsReportedByParser,
        ImportOutcome::Imported,
        "00".repeat(32),
        Some("11".repeat(32).as_str()),
        fields.clone(),
        vec![],
    )
    .unwrap();
    let b = ImportProvenanceManifest::new(
        source_b,
        InventoryStatus::CompleteAsReportedByParser,
        ImportOutcome::Imported,
        "00".repeat(32),
        Some("11".repeat(32).as_str()),
        reversed,
        vec![],
    )
    .unwrap();

    assert_eq!(a.canonical_bytes().unwrap(), b.canonical_bytes().unwrap());
    assert_eq!(a.digest_hex().unwrap(), b.digest_hex().unwrap());
    fields.clear();
}

#[test]
fn parser_inventory_uncertainty_cannot_be_called_clean_import() {
    assert_eq!(
        ImportProvenanceManifest::new(
            source(b"<Message/>", "1.0"),
            InventoryStatus::Unknown,
            ImportOutcome::Imported,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            clean_fields(),
            vec![],
        ),
        Err(ProvenanceError::InvalidOutcome)
    );
}

#[test]
fn rejection_with_changed_snapshot_digest_is_invalid() {
    let error = ImportDiagnostic::new(
        "schema_validation_failed",
        DiagnosticSeverity::Error,
        None,
        "Required field absent",
    )
    .unwrap();

    assert_eq!(
        ImportProvenanceManifest::new(
            source(b"<Message/>", "1.0"),
            InventoryStatus::Unknown,
            ImportOutcome::Rejected,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![],
            vec![error],
        ),
        Err(ProvenanceError::InvalidOutcome)
    );
}

#[test]
fn unsupported_fields_require_diagnostics_bound_to_known_source_paths() {
    let unsupported = FieldMapping::unsupported_retained(
        "/Message/ExperimentalField",
        "unsupported_schema_field",
    )
    .unwrap();
    assert_eq!(
        ImportProvenanceManifest::new(
            source(b"<Message><ExperimentalField/></Message>", "1.0"),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::ImportedWithWarnings,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![unsupported.clone()],
            vec![],
        ),
        Err(ProvenanceError::MissingFieldDiagnostic)
    );

    let diagnostic = ImportDiagnostic::new(
        "unsupported_schema_field",
        DiagnosticSeverity::Warning,
        Some("/Message/NotEnumerated"),
        "The source field was not declared by the parser inventory",
    )
    .unwrap();
    assert_eq!(
        ImportProvenanceManifest::new(
            source(b"<Message><ExperimentalField/></Message>", "1.0"),
            InventoryStatus::CompleteAsReportedByParser,
            ImportOutcome::ImportedWithWarnings,
            "00".repeat(32),
            Some("11".repeat(32).as_str()),
            vec![unsupported],
            vec![diagnostic],
        ),
        Err(ProvenanceError::DiagnosticPathUnknown)
    );
}

#[test]
fn parser_version_is_part_of_the_provenance_identity() {
    let v1 = clean_manifest(b"<Message>same</Message>", "1.0");
    let v2 = clean_manifest(b"<Message>same</Message>", "1.1");
    assert_ne!(v1.source().parser_version(), v2.source().parser_version());
    assert_ne!(v1.digest_hex().unwrap(), v2.digest_hex().unwrap());
}

#[test]
fn rejected_source_retains_before_digest_and_does_not_claim_an_after_change() {
    let before = "a5".repeat(32);
    let failure = ImportDiagnostic::new(
        "schema_validation_failed",
        DiagnosticSeverity::Error,
        None,
        "The source payload failed schema validation",
    )
    .unwrap();
    let manifest = ImportProvenanceManifest::new(
        source(b"<not-valid", "1.0"),
        InventoryStatus::Unknown,
        ImportOutcome::Rejected,
        before.as_str(),
        Some(before.as_str()),
        vec![],
        vec![failure],
    )
    .unwrap();

    assert_eq!(manifest.before_snapshot_digest(), before);
    assert_eq!(manifest.after_snapshot_digest(), Some(before.as_str()));
    assert_eq!(manifest.outcome(), ImportOutcome::Rejected);
}
