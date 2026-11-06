use jsonschema::draft202012;
use serde::Serialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

type AppResult<T> = Result<T, String>;

const CORPUS_DEFAULT: &str = "../smoke_corpus_v0.1.0.json";
const SCHEMA_DEFAULT: &str = "../smoke_corpus_v0.1.0.schema.json";
const PROTOCOL_DEFAULT: &str = "../../../docs/ACCOUNTING_EVIDENCE_BENCHMARK_V1.md";

#[derive(Debug, Serialize)]
struct Check {
    invariant: String,
    passed: bool,
    detail: String,
}

#[derive(Debug, Serialize)]
struct CaseResult {
    case_id: String,
    status: String,
    checks: Vec<Check>,
    canonical_journal_sha256: Vec<String>,
}

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("fin-acc-reference-verifier: {error}");
        std::process::exit(2);
    }
}

fn run_cli() -> Result<(), Box<dyn Error>> {
    let mut corpus_path = PathBuf::from(CORPUS_DEFAULT);
    let mut schema_path = PathBuf::from(SCHEMA_DEFAULT);
    let mut protocol_path = PathBuf::from(PROTOCOL_DEFAULT);
    let mut output_path: Option<PathBuf> = None;
    let mut toolchain = env::var("RUSTC_VERSION").unwrap_or_else(|_| "not supplied".to_owned());

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--corpus" => corpus_path = PathBuf::from(next_arg(&mut args, "--corpus")?),
            "--schema" => schema_path = PathBuf::from(next_arg(&mut args, "--schema")?),
            "--protocol" => protocol_path = PathBuf::from(next_arg(&mut args, "--protocol")?),
            "--out" => output_path = Some(PathBuf::from(next_arg(&mut args, "--out")?)),
            "--toolchain" => toolchain = next_arg(&mut args, "--toolchain")?,
            "--help" | "-h" => {
                print_help();
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}; use --help").into()),
        }
    }

    let corpus_bytes = fs::read(&corpus_path)?;
    let schema_bytes = fs::read(&schema_path)?;
    let protocol_bytes = fs::read(&protocol_path)?;

    let corpus: Value = serde_json::from_slice(&corpus_bytes)?;
    let schema: Value = serde_json::from_slice(&schema_bytes)?;
    let report = verify_corpus(
        &corpus,
        &schema,
        &corpus_bytes,
        &schema_bytes,
        Some(&protocol_bytes),
        &toolchain,
    );

    let rendered = serde_json::to_string_pretty(&report)?;
    match output_path {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }

    if report["status"] != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

fn next_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, Box<dyn Error>> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a value").into())
}

fn print_help() {
    println!(
        "FIN-ACC independent reference verifier\n\n\
         --corpus PATH     Corpus JSON (default: {CORPUS_DEFAULT})\n\
         --schema PATH     Draft 2020-12 schema (default: {SCHEMA_DEFAULT})\n\
         --protocol PATH   Benchmark protocol used for provenance digest\n\
         --out PATH        Write JSON report to PATH (default: stdout)\n\
         --toolchain TEXT  Caller-supplied exact Rust toolchain identity\n\
         --help            Show this help"
    );
}

fn verify_corpus(
    corpus: &Value,
    schema: &Value,
    corpus_bytes: &[u8],
    schema_bytes: &[u8],
    protocol_bytes: Option<&[u8]>,
    toolchain: &str,
) -> Value {
    let schema_errors = match validate_schema(schema, corpus) {
        Ok(errors) => errors,
        Err(error) => vec![error],
    };

    let mut case_results = Vec::new();
    if schema_errors.is_empty() {
        if let Some(cases) = corpus.get("cases").and_then(Value::as_array) {
            for case in cases {
                case_results.push(verify_case(case, corpus));
            }
        } else {
            case_results.push(CaseResult {
                case_id: "<missing>".to_owned(),
                status: "FAIL".to_owned(),
                checks: vec![Check {
                    invariant: "cases_array".to_owned(),
                    passed: false,
                    detail: "corpus has no cases array".to_owned(),
                }],
                canonical_journal_sha256: Vec::new(),
            });
        }
    }

    let cases_pass =
        !case_results.is_empty() && case_results.iter().all(|case| case.status == "PASS");
    let status = if schema_errors.is_empty() && cases_pass {
        "PASS"
    } else {
        "FAIL"
    };

    let mut report = json!({
        "report_schema": "fin-acc-reference-verifier.report.v0.1",
        "benchmark_id": corpus.get("benchmark_id"),
        "dataset_version": corpus.get("dataset_version"),
        "status": status,
        "claim_scope": "synthetic_smoke_corpus_consistency_and_independent_reconstruction_only",
        "schema_validation": {
            "draft": "2020-12",
            "passed": schema_errors.is_empty(),
            "errors": schema_errors
        },
        "digests": {
            "corpus_sha256": sha256_hex(corpus_bytes),
            "schema_sha256": sha256_hex(schema_bytes),
            "protocol_sha256": protocol_bytes.map(sha256_hex),
            "verifier_source_sha256": verifier_source_digest(),
            "report_schema_sha256": sha256_hex(include_bytes!("../report.schema.json"))
        },
        "toolchain": toolchain,
        "network_access_during_verification": false,
        "case_count": case_results.len(),
        "cases": case_results,
        "report_schema_validation": {
            "passed": true,
            "errors": []
        }
    });

    let report_errors = validate_output_report(&report);
    report["report_schema_validation"] = json!({
        "passed": report_errors.is_empty(),
        "errors": report_errors
    });
    if report["report_schema_validation"]["passed"] != true {
        report["status"] = Value::String("FAIL".to_owned());
    }
    report
}

fn validate_output_report(report: &Value) -> Vec<String> {
    let schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(schema) => schema,
        Err(error) => return vec![format!("could not parse embedded report schema: {error}")],
    };
    if let Err(error) = jsonschema::draft202012::meta::validate(&schema) {
        return vec![format!("invalid embedded report schema: {error}")];
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
    {
        Ok(validator) => validator,
        Err(error) => return vec![format!("could not compile embedded report schema: {error}")],
    };
    validator
        .iter_errors(report)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect()
}

fn validate_schema(schema: &Value, corpus: &Value) -> AppResult<Vec<String>> {
    let declared = schema
        .get("$schema")
        .and_then(Value::as_str)
        .ok_or_else(|| "schema is missing its $schema declaration".to_owned())?;
    if !declared.contains("2020-12") {
        return Err(format!("expected Draft 2020-12 schema, got {declared}"));
    }

    jsonschema::draft202012::meta::validate(schema)
        .map_err(|error| format!("invalid Draft 2020-12 schema: {error}"))?;

    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("could not compile Draft 2020-12 schema: {error}"))?;

    let errors = validator
        .iter_errors(corpus)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect::<Vec<_>>();
    Ok(errors)
}

fn verify_case(case: &Value, corpus: &Value) -> CaseResult {
    let case_id = case
        .get("case_id")
        .and_then(Value::as_str)
        .unwrap_or("<unknown>")
        .to_owned();
    let mut checks = Vec::new();
    let mut commitments = Vec::new();

    let result = (|| -> AppResult<()> {
        let profile = field(corpus, "profile")?;
        verify_profile(profile, &mut checks)?;
        let known_entities = parse_id_set(field(corpus, "entities")?, "entities")?;
        let known_counterparties =
            parse_id_set(field(corpus, "counterparties")?, "counterparties")?;
        let account_catalog = parse_account_catalog(field(corpus, "account_catalog")?)?;
        let documents = parse_documents(
            field(case, "documents")?,
            &known_entities,
            &known_counterparties,
            string(profile, "currency")?,
            &mut checks,
        )?;
        check_dispositions(case, &documents, &mut checks)?;

        let opening = parse_balance_rows(field(case, "opening_trial_balance")?, &account_catalog)?;
        record(
            &mut checks,
            "opening_trial_balance_balanced",
            opening.1 == opening.2,
            format!("debits={} credits={}", opening.1, opening.2),
        )?;

        let mut closing = opening.0;
        let journal = field(case, "expected_journal")?
            .as_array()
            .ok_or_else(|| "expected_journal must be an array".to_owned())?;
        let mut entry_ids = BTreeSet::new();
        for entry in journal {
            let entry_id = string(entry, "entry_id")?;
            record(
                &mut checks,
                "journal_entry_identity_unique",
                entry_ids.insert(entry_id.to_owned()),
                format!("entry_id={entry_id}"),
            )?;
            verify_journal_binding(entry, &documents, profile, &mut checks)?;
            apply_journal_entry(entry, &account_catalog, &mut closing, &mut checks)?;
            verify_seed_journal_projection(case, entry, &documents, &mut checks)?;
            commitments.push(canonical_journal_commitment(entry)?);
        }

        let reconstructed = balances_to_rows(&closing, &account_catalog)?;
        let expected = parse_balance_rows(
            field(field(case, "expected_close")?, "trial_balance")?,
            &account_catalog,
        )?;
        record(
            &mut checks,
            "independent_trial_balance_reconstruction",
            reconstructed == expected.0,
            format!(
                "reconstructed={} accounts; expected={} accounts",
                reconstructed.len(),
                expected.0.len()
            ),
        )?;
        record(
            &mut checks,
            "closing_trial_balance_balanced",
            expected.1 == expected.2,
            format!(
                "expected_debits={} expected_credits={}",
                expected.1, expected.2
            ),
        )?;

        let (total_debits, total_credits) = trial_balance_totals(&reconstructed, &account_catalog)?;
        let expected_close = field(case, "expected_close")?;
        let declared_debits = amount(
            field(expected_close, "total_debits_minor")?,
            "total_debits_minor",
        )?;
        let declared_credits = amount(
            field(expected_close, "total_credits_minor")?,
            "total_credits_minor",
        )?;
        record(
            &mut checks,
            "closing_trial_balance_totals",
            total_debits == declared_debits && total_credits == declared_credits,
            format!(
                "calculated=({total_debits},{total_credits}) expected=({declared_debits},{declared_credits})"
            ),
        )?;

        let statement_totals = reconstruct_statement_totals(&closing, &account_catalog)?;
        let declared_statement = field(expected_close, "statement_totals")?;
        let declared_assets = amount(
            field(declared_statement, "total_assets_minor")?,
            "total_assets_minor",
        )?;
        let declared_liabilities = amount(
            field(declared_statement, "total_liabilities_minor")?,
            "total_liabilities_minor",
        )?;
        let declared_equity = amount(
            field(declared_statement, "total_equity_minor")?,
            "total_equity_minor",
        )?;
        let declared_profit = signed_amount(
            field(declared_statement, "net_profit_loss_minor")?,
            "net_profit_loss_minor",
        )?;
        record(
            &mut checks,
            "independent_statement_reconstruction",
            statement_totals
                == (
                    declared_assets,
                    declared_liabilities,
                    declared_equity,
                    declared_profit,
                ),
            format!(
                "calculated=({},{},{},{}) expected=({},{},{},{})",
                statement_totals.0,
                statement_totals.1,
                statement_totals.2,
                statement_totals.3,
                declared_assets,
                declared_liabilities,
                declared_equity,
                declared_profit
            ),
        )?;

        let assets =
            i128::try_from(statement_totals.0).map_err(|_| "assets exceed i128".to_owned())?;
        let liabilities =
            i128::try_from(statement_totals.1).map_err(|_| "liabilities exceed i128".to_owned())?;
        let equity =
            i128::try_from(statement_totals.2).map_err(|_| "equity exceeds i128".to_owned())?;
        let right_side = liabilities
            .checked_add(equity)
            .and_then(|value| value.checked_add(statement_totals.3))
            .ok_or_else(|| "accounting equation overflow".to_owned())?;
        record(
            &mut checks,
            "accounting_equation",
            assets == right_side,
            format!("assets={assets} liabilities+equity+profit={right_side}"),
        )?;

        Ok(())
    })();

    let status = if let Err(error) = result {
        checks.push(Check {
            invariant: "case_terminal_status".to_owned(),
            passed: false,
            detail: error,
        });
        "FAIL"
    } else {
        checks.push(Check {
            invariant: "case_terminal_status".to_owned(),
            passed: true,
            detail: "all enabled invariants passed".to_owned(),
        });
        "PASS"
    };

    CaseResult {
        case_id,
        status: status.to_owned(),
        checks,
        canonical_journal_sha256: commitments,
    }
}

#[derive(Clone, Debug)]
struct Document {
    document_id: String,
    document_kind: String,
    entity_id: String,
    counterparty_id: String,
    currency: String,
    date: String,
    ingestion_id: String,
    content: Value,
}

fn parse_documents(
    values: &Value,
    known_entities: &BTreeSet<String>,
    known_counterparties: &BTreeSet<String>,
    profile_currency: &str,
    checks: &mut Vec<Check>,
) -> AppResult<Vec<Document>> {
    let array = values
        .as_array()
        .ok_or_else(|| "documents must be an array".to_owned())?;
    let mut seen_documents: BTreeMap<String, String> = BTreeMap::new();
    let mut seen_ingestions = BTreeSet::new();
    let mut documents = Vec::with_capacity(array.len());

    for value in array {
        let document_id = string(value, "document_id")?.to_owned();
        let fingerprint = string(value, "fingerprint_sha256")?.to_owned();
        let ingestion_id = string(value, "ingestion_id")?.to_owned();
        let content = field(value, "content")?.clone();
        let calculated = sha256_hex(&canonical_json(&content)?);
        record(
            checks,
            "source_document_fingerprint",
            calculated == fingerprint,
            format!(
                "document_id={document_id} ingestion_id={ingestion_id} calculated={calculated} declared={fingerprint}"
            ),
        )?;

        let doc_kind = string(value, "document_kind")?.to_owned();
        let entity_id = string(value, "entity_id")?.to_owned();
        let counterparty_id = string(value, "counterparty_id")?.to_owned();
        let currency = string(value, "currency")?.to_owned();
        let date = string(value, "date")?.to_owned();

        record(
            checks,
            "source_document_metadata_binding",
            string(&content, "doc_id")? == document_id
                && string(&content, "doc_type")? == doc_kind
                && string(&content, "entity_id")? == entity_id
                && string(&content, "counterparty_id")? == counterparty_id
                && string(&content, "currency")? == currency
                && string(&content, "issue_date")? == date,
            format!("document_id={document_id}"),
        )?;

        record(
            checks,
            "source_entity_known",
            known_entities.contains(&entity_id),
            format!("document_id={document_id} entity_id={entity_id}"),
        )?;
        record(
            checks,
            "source_counterparty_known",
            known_counterparties.contains(&counterparty_id),
            format!("document_id={document_id} counterparty_id={counterparty_id}"),
        )?;
        record(
            checks,
            "source_currency_profile_binding",
            currency == profile_currency,
            format!(
                "document_id={document_id} source_currency={currency} profile_currency={profile_currency}"
            ),
        )?;

        record(
            checks,
            "source_ingestion_identity_unique",
            seen_ingestions.insert(ingestion_id.clone()),
            format!("ingestion_id={ingestion_id}"),
        )?;

        if let Some(previous) = seen_documents.get(&document_id) {
            record(
                checks,
                "duplicate_identity_same_fingerprint",
                previous == &fingerprint,
                format!("document_id={document_id} previous={previous} current={fingerprint}"),
            )?;
        } else {
            seen_documents.insert(document_id.clone(), fingerprint.clone());
        }

        documents.push(Document {
            document_id,
            document_kind: doc_kind,
            entity_id,
            counterparty_id,
            currency,
            date,
            ingestion_id,
            content,
        });
    }
    Ok(documents)
}

fn check_dispositions(
    case: &Value,
    documents: &[Document],
    checks: &mut Vec<Check>,
) -> AppResult<()> {
    let declared = field(case, "expected_dispositions")?
        .as_array()
        .ok_or_else(|| "expected_dispositions must be an array".to_owned())?;

    let mut disposition_by_ingestion = BTreeMap::new();
    for disposition in declared {
        let ingestion_id = string(disposition, "ingestion_id")?.to_owned();
        if disposition_by_ingestion
            .insert(
                ingestion_id.clone(),
                string(disposition, "disposition")?.to_owned(),
            )
            .is_some()
        {
            return Err(format!(
                "duplicate expected disposition for ingestion {ingestion_id}"
            ));
        }
    }
    for ingestion_id in disposition_by_ingestion.keys() {
        if !documents
            .iter()
            .any(|document| &document.ingestion_id == ingestion_id)
        {
            return Err(format!(
                "expected disposition references unknown ingestion {ingestion_id}"
            ));
        }
    }

    let mut document_occurrences: BTreeMap<&str, Vec<&Document>> = BTreeMap::new();
    for document in documents {
        document_occurrences
            .entry(&document.document_id)
            .or_default()
            .push(document);
    }

    let expected_journal = field(case, "expected_journal")?
        .as_array()
        .ok_or_else(|| "expected_journal must be an array".to_owned())?;

    for document in documents {
        let expected = disposition_by_ingestion.get(&document.ingestion_id);
        let computed =
            if let Some(occurrences) = document_occurrences.get(document.document_id.as_str()) {
                let first = occurrences.first().copied();
                let is_duplicate =
                    first.is_some_and(|first| first.ingestion_id != document.ingestion_id);
                if is_duplicate {
                    "duplicate_no_op"
                } else if document.document_kind == "supplier_invoice" {
                    if invoice_supported(document, documents)? {
                        "qualified_for_accounting_projection"
                    } else {
                        "quarantine"
                    }
                } else if document.document_kind == "bank_statement_line" {
                    if bank_settlement_supported(document, documents)? {
                        "qualified_settlement_observation"
                    } else {
                        "indeterminate"
                    }
                } else {
                    continue;
                }
            } else {
                continue;
            };

        if let Some(expected) = expected {
            record(
                checks,
                "economic_evidence_disposition",
                expected.as_str() == computed,
                format!(
                    "ingestion_id={} computed={} expected={}",
                    document.ingestion_id, computed, expected
                ),
            )?;
        } else {
            return Err(format!(
                "document {} / ingestion {} has no expected disposition",
                document.document_id, document.ingestion_id
            ));
        }

        if computed == "quarantine" || computed == "duplicate_no_op" {
            let reference_count = expected_journal
                .iter()
                .filter(|entry| {
                    field(entry, "source_document_refs")
                        .ok()
                        .and_then(Value::as_array)
                        .is_some_and(|refs| {
                            refs.iter().any(|reference| {
                                reference.as_str() == Some(document.document_id.as_str())
                            })
                        })
                })
                .count();
            if computed == "quarantine" {
                record(
                    checks,
                    "quarantined_document_not_journaled",
                    reference_count == 0,
                    format!(
                        "document_id={} referencing_entries={reference_count}",
                        document.document_id
                    ),
                )?;
            }
        }
    }

    let class = string(case, "class")?;
    match class {
        "positive_control" => record(
            checks,
            "positive_control_journal_count",
            expected_journal.len() == 2,
            format!("journal_entries={}", expected_journal.len()),
        )?,
        "idempotency_adversarial" => {
            let invoice_occurrences = documents
                .iter()
                .filter(|document| document.document_kind == "supplier_invoice")
                .count();
            record(
                checks,
                "duplicate_case_has_one_posting",
                invoice_occurrences == 2 && expected_journal.len() == 1,
                format!(
                    "invoice_occurrences={invoice_occurrences} journal_entries={}",
                    expected_journal.len()
                ),
            )?;
        }
        "entity_boundary_adversarial" => record(
            checks,
            "wrong_entity_case_has_no_journal",
            expected_journal.is_empty(),
            format!("journal_entries={}", expected_journal.len()),
        )?,
        _ => return Err(format!("unsupported smoke case class: {class}")),
    }
    Ok(())
}

fn invoice_supported(invoice: &Document, documents: &[Document]) -> AppResult<bool> {
    let po = documents.iter().find(|document| {
        document.document_kind == "purchase_order"
            && document.counterparty_id == invoice.counterparty_id
            && document.currency == invoice.currency
    });
    let grn = documents.iter().find(|document| {
        document.document_kind == "goods_received_note"
            && document.counterparty_id == invoice.counterparty_id
            && document.currency == invoice.currency
    });
    let (Some(po), Some(grn)) = (po, grn) else {
        return Ok(false);
    };
    if invoice.entity_id != po.entity_id || invoice.entity_id != grn.entity_id {
        return Ok(false);
    }
    if string(&invoice.content, "entity_id")? != string(&po.content, "entity_id")?
        || string(&invoice.content, "entity_id")? != string(&grn.content, "entity_id")?
    {
        return Ok(false);
    }

    let invoice_lines = field(&invoice.content, "lines")?
        .as_array()
        .ok_or_else(|| "invoice lines must be array".to_owned())?;
    let po_lines = field(&po.content, "lines")?
        .as_array()
        .ok_or_else(|| "PO lines must be array".to_owned())?;
    let grn_lines = field(&grn.content, "lines")?
        .as_array()
        .ok_or_else(|| "GRN lines must be array".to_owned())?;
    if invoice_lines.len() != po_lines.len() || invoice_lines.len() != grn_lines.len() {
        return Ok(false);
    }

    let mut calculated_invoice_total = 0_u128;
    for invoice_line in invoice_lines {
        let sku = string(invoice_line, "sku")?;
        let po_line = po_lines
            .iter()
            .find(|line| string(line, "sku").ok() == Some(sku));
        let grn_line = grn_lines
            .iter()
            .find(|line| string(line, "sku").ok() == Some(sku));
        let (Some(po_line), Some(grn_line)) = (po_line, grn_line) else {
            return Ok(false);
        };
        let quantity = string(invoice_line, "quantity")?;
        if string(po_line, "quantity")? != quantity
            || string(grn_line, "quantity_received")? != quantity
            || string(po_line, "unit_price_minor")? != string(invoice_line, "unit_price_minor")?
            || string(po_line, "line_total_minor")? != string(invoice_line, "line_total_minor")?
        {
            return Ok(false);
        }
        let quantity_units = amount(field(invoice_line, "quantity")?, "invoice quantity")?;
        let unit_price = amount(
            field(invoice_line, "unit_price_minor")?,
            "invoice unit price",
        )?;
        let line_total = amount(
            field(invoice_line, "line_total_minor")?,
            "invoice line total",
        )?;
        if unit_price.checked_mul(quantity_units) != Some(line_total) {
            return Ok(false);
        }
        calculated_invoice_total = calculated_invoice_total
            .checked_add(line_total)
            .ok_or_else(|| "invoice line total overflow".to_owned())?;
    }
    let invoice_total = amount(field(&invoice.content, "total_minor")?, "invoice total")?;
    let po_total = amount(field(&po.content, "total_minor")?, "purchase order total")?;
    let tax_is_zero = invoice
        .content
        .get("tax_minor")
        .map(|tax| amount(tax, "invoice tax").map(|value| value == 0))
        .transpose()?
        .unwrap_or(true);
    Ok(invoice_total == po_total && invoice_total == calculated_invoice_total && tax_is_zero)
}

fn bank_settlement_supported(bank_line: &Document, documents: &[Document]) -> AppResult<bool> {
    let reference = string(&bank_line.content, "reference")?;
    let invoice = documents.iter().find(|document| {
        document.document_id == reference && document.document_kind == "supplier_invoice"
    });
    let Some(invoice) = invoice else {
        return Ok(false);
    };
    let amount_minor = string(&bank_line.content, "amount_minor")?;
    let invoice_total = string(&invoice.content, "total_minor")?;
    Ok(bank_line.entity_id == invoice.entity_id
        && bank_line.counterparty_id == invoice.counterparty_id
        && bank_line.currency == invoice.currency
        && amount_minor == invoice_total
        && string(&bank_line.content, "direction")? == "debit")
}

fn verify_journal_binding(
    entry: &Value,
    documents: &[Document],
    profile: &Value,
    checks: &mut Vec<Check>,
) -> AppResult<()> {
    let entry_entity = string(entry, "entity_id")?;
    let entry_profile = string(entry, "currency_profile_id")?;
    let entry_policy = string(entry, "policy_id")?;
    let entry_revision = string(entry, "policy_revision")?;
    record(
        checks,
        "journal_profile_policy_binding",
        entry_profile == string(profile, "id")?
            && entry_policy == string(profile, "accounting_policy_id")?
            && entry_revision == string(profile, "accounting_policy_revision")?,
        format!(
            "entry_id={} profile={entry_profile} policy={entry_policy}@{entry_revision}",
            string(entry, "entry_id")?
        ),
    )?;

    let effective_date = string(entry, "effective_date")?;
    let profile_start = string(profile, "effective_from")?;
    record(
        checks,
        "journal_policy_effective_date",
        effective_date >= profile_start,
        format!("entry_date={effective_date} policy_effective_from={profile_start}"),
    )?;

    let refs = field(entry, "source_document_refs")?
        .as_array()
        .ok_or_else(|| "journal source_document_refs must be array".to_owned())?;
    record(
        checks,
        "journal_has_source_documents",
        !refs.is_empty(),
        format!(
            "entry_id={} source_count={}",
            string(entry, "entry_id")?,
            refs.len()
        ),
    )?;

    let mut seen_refs = BTreeSet::new();
    for source_ref in refs {
        let source_id = source_ref
            .as_str()
            .ok_or_else(|| "source ref must be string".to_owned())?;
        record(
            checks,
            "journal_source_reference_unique",
            seen_refs.insert(source_id.to_owned()),
            format!(
                "entry_id={} source_ref={source_id}",
                string(entry, "entry_id")?
            ),
        )?;
        let source = documents
            .iter()
            .find(|document| document.document_id == source_id)
            .ok_or_else(|| format!("journal references missing source document {source_id}"))?;
        record(
            checks,
            "journal_source_entity_binding",
            source.entity_id == entry_entity,
            format!(
                "entry_id={} source_ref={source_id} source_entity={} entry_entity={entry_entity}",
                string(entry, "entry_id")?,
                source.entity_id
            ),
        )?;
        record(
            checks,
            "journal_source_currency_binding",
            source.currency == string(profile, "currency")?,
            format!(
                "entry_id={} source_ref={source_id} source_currency={}",
                string(entry, "entry_id")?,
                source.currency
            ),
        )?;
        record(
            checks,
            "journal_source_cutoff",
            source.date.as_str() <= effective_date,
            format!(
                "entry_id={} source_ref={source_id} source_date={} effective_date={effective_date}",
                string(entry, "entry_id")?,
                source.date
            ),
        )?;
    }
    Ok(())
}

fn parse_id_set(values: &Value, label: &str) -> AppResult<BTreeSet<String>> {
    let rows = values
        .as_array()
        .ok_or_else(|| format!("{label} must be an array"))?;
    let mut ids = BTreeSet::new();
    for row in rows {
        let id = string(row, "id")?.to_owned();
        if !ids.insert(id.clone()) {
            return Err(format!("duplicate {label} identity {id}"));
        }
    }
    Ok(ids)
}

fn verify_profile(profile: &Value, checks: &mut Vec<Check>) -> AppResult<()> {
    let currency = string(profile, "currency")?;
    let scale = field(profile, "minor_unit_scale")?
        .as_u64()
        .ok_or_else(|| "minor_unit_scale must be a non-negative JSON integer".to_owned())?;
    record(
        checks,
        "synthetic_currency_profile",
        currency == "ZAR"
            && scale == 2
            && string(profile, "amount_encoding")?
                == "canonical_nonnegative_decimal_integer_strings"
            && string(profile, "tax_scope")?.starts_with("excluded"),
        format!(
            "profile_id={} currency={} scale={} tax_scope={}",
            string(profile, "id")?,
            currency,
            scale,
            string(profile, "tax_scope")?
        ),
    )
}

fn verify_seed_journal_projection(
    case: &Value,
    entry: &Value,
    documents: &[Document],
    checks: &mut Vec<Check>,
) -> AppResult<()> {
    let case_id = string(case, "case_id")?;
    let entry_id = string(entry, "entry_id")?;
    let source_refs = field(entry, "source_document_refs")?
        .as_array()
        .ok_or_else(|| "journal source refs must be an array".to_owned())?
        .iter()
        .map(|value| {
            value
                .as_str()
                .map(str::to_owned)
                .ok_or_else(|| "journal source ref must be a string".to_owned())
        })
        .collect::<AppResult<BTreeSet<_>>>()?;

    let mut expected_lines: BTreeMap<(String, bool), u128> = BTreeMap::new();
    let mut expected_sources = BTreeSet::new();
    let expected_date;
    let expected_entity;

    match (case_id, entry_id) {
        ("ACC-SYN-0001" | "ACC-SYN-0002", "J-INV-778") => {
            let invoice = documents
                .iter()
                .find(|document| document.document_id == "INV-778")
                .ok_or_else(|| "invoice journal is missing INV-778 source".to_owned())?;
            let po = documents
                .iter()
                .find(|document| document.document_id == "PO-001")
                .ok_or_else(|| "invoice journal is missing PO-001 source".to_owned())?;
            let grn = documents
                .iter()
                .find(|document| document.document_id == "GRN-001")
                .ok_or_else(|| "invoice journal is missing GRN-001 source".to_owned())?;
            if !invoice_supported(invoice, documents)? {
                return Err("invoice journal lacks supported PO/GRN/invoice evidence".to_owned());
            }
            let total = amount(field(&invoice.content, "total_minor")?, "invoice total")?;
            expected_lines.insert(("1200".to_owned(), true), total);
            expected_lines.insert(("2000".to_owned(), false), total);
            expected_sources.extend([
                "INV-778".to_owned(),
                "PO-001".to_owned(),
                "GRN-001".to_owned(),
            ]);
            expected_date = invoice.date.as_str();
            expected_entity = invoice.entity_id.as_str();
            if po.entity_id != invoice.entity_id || grn.entity_id != invoice.entity_id {
                return Err("invoice journal source entity conflict".to_owned());
            }
        }
        ("ACC-SYN-0001", "J-PAY-778") => {
            let invoice = documents
                .iter()
                .find(|document| document.document_id == "INV-778")
                .ok_or_else(|| "payment journal is missing INV-778 source".to_owned())?;
            let bank = documents
                .iter()
                .find(|document| document.document_id == "BANK-LINE-001")
                .ok_or_else(|| "payment journal is missing BANK-LINE-001 source".to_owned())?;
            if !bank_settlement_supported(bank, documents)? {
                return Err(
                    "payment journal lacks a matching bank settlement observation".to_owned(),
                );
            }
            let paid = amount(field(&bank.content, "amount_minor")?, "bank amount")?;
            let invoice_total = amount(field(&invoice.content, "total_minor")?, "invoice total")?;
            if paid != invoice_total {
                return Err("seed v0.1 payment projection only supports full settlement".to_owned());
            }
            expected_lines.insert(("2000".to_owned(), true), paid);
            expected_lines.insert(("1000".to_owned(), false), paid);
            expected_sources.extend(["INV-778".to_owned(), "BANK-LINE-001".to_owned()]);
            expected_date = bank.date.as_str();
            expected_entity = bank.entity_id.as_str();
        }
        _ => {
            return Err(format!(
                "unsupported seed journal mapping: {case_id} / {entry_id}"
            ));
        }
    }

    let mut actual_lines: BTreeMap<(String, bool), u128> = BTreeMap::new();
    let lines = field(entry, "lines")?
        .as_array()
        .ok_or_else(|| "journal lines must be an array".to_owned())?;
    for line in lines {
        let account = string(line, "account_id")?.to_owned();
        let debit = amount(field(line, "debit_minor")?, "debit_minor")?;
        let credit = amount(field(line, "credit_minor")?, "credit_minor")?;
        if (debit > 0 && credit > 0) || (debit == 0 && credit == 0) {
            return Err("seed journal line must contain exactly one positive side".to_owned());
        }
        let key = (account, debit > 0);
        let value = actual_lines.entry(key).or_default();
        *value = value
            .checked_add(if debit > 0 { debit } else { credit })
            .ok_or_else(|| "aggregated seed journal line overflow".to_owned())?;
    }

    record(
        checks,
        "journal_source_projection",
        actual_lines == expected_lines
            && source_refs == expected_sources
            && string(entry, "effective_date")? == expected_date
            && string(entry, "entity_id")? == expected_entity,
        format!(
            "entry_id={entry_id} lines_match={} sources_match={} date_match={} entity_match={}",
            actual_lines == expected_lines,
            source_refs == expected_sources,
            string(entry, "effective_date")? == expected_date,
            string(entry, "entity_id")? == expected_entity
        ),
    )
}

fn parse_account_catalog(value: &Value) -> AppResult<BTreeMap<String, String>> {
    let accounts = value
        .as_array()
        .ok_or_else(|| "account_catalog must be an array".to_owned())?;
    let mut result = BTreeMap::new();
    for account in accounts {
        let account_id = string(account, "id")?.to_owned();
        let class = string(account, "class")?.to_owned();
        let side = string(account, "normal_side")?;
        let expected_side = match class.as_str() {
            "asset" | "expense" => "debit",
            "liability" | "equity" | "income" => "credit",
            _ => return Err(format!("unsupported account class {class}")),
        };
        if side != expected_side {
            return Err(format!(
                "account {account_id} uses unsupported normal side {side} for class {class}"
            ));
        }
        if result.insert(account_id.clone(), class).is_some() {
            return Err(format!("duplicate account id {account_id}"));
        }
    }
    Ok(result)
}

fn parse_balance_rows(
    value: &Value,
    account_catalog: &BTreeMap<String, String>,
) -> AppResult<(BTreeMap<String, i128>, u128, u128)> {
    let rows = value
        .as_array()
        .ok_or_else(|| "trial balance must be an array".to_owned())?;
    let mut balances = BTreeMap::new();
    let mut total_debits = 0_u128;
    let mut total_credits = 0_u128;

    for row in rows {
        let account_id = string(row, "account_id")?.to_owned();
        if !account_catalog.contains_key(&account_id) {
            return Err(format!(
                "trial balance references unknown account {account_id}"
            ));
        }
        if balances.contains_key(&account_id) {
            return Err(format!(
                "duplicate trial balance row for account {account_id}"
            ));
        }
        let debit = amount(field(row, "debit_minor")?, "debit_minor")?;
        let credit = amount(field(row, "credit_minor")?, "credit_minor")?;
        if debit > 0 && credit > 0 {
            return Err(format!(
                "trial balance row {account_id} has debit and credit simultaneously"
            ));
        }
        total_debits = total_debits
            .checked_add(debit)
            .ok_or_else(|| "trial balance debit total overflow".to_owned())?;
        total_credits = total_credits
            .checked_add(credit)
            .ok_or_else(|| "trial balance credit total overflow".to_owned())?;
        balances.insert(account_id, signed_difference(debit, credit)?);
    }

    if balances.len() != account_catalog.len() {
        return Err(format!(
            "trial balance has {} accounts but catalog has {}",
            balances.len(),
            account_catalog.len()
        ));
    }
    Ok((balances, total_debits, total_credits))
}

fn apply_journal_entry(
    entry: &Value,
    account_catalog: &BTreeMap<String, String>,
    balances: &mut BTreeMap<String, i128>,
    checks: &mut Vec<Check>,
) -> AppResult<()> {
    let lines = field(entry, "lines")?
        .as_array()
        .ok_or_else(|| "journal lines must be an array".to_owned())?;
    if lines.len() < 2 {
        return Err(format!(
            "journal entry {} has fewer than two postings",
            string(entry, "entry_id")?
        ));
    }
    let mut debits = 0_u128;
    let mut credits = 0_u128;
    let mut account_deltas: BTreeMap<String, i128> = BTreeMap::new();
    for (index, line) in lines.iter().enumerate() {
        let account_id = string(line, "account_id")?;
        if !account_catalog.contains_key(account_id) {
            return Err(format!("journal references unknown account {account_id}"));
        }
        let debit = amount(field(line, "debit_minor")?, "debit_minor")?;
        let credit = amount(field(line, "credit_minor")?, "credit_minor")?;
        record(
            checks,
            "journal_posting_exactly_one_side",
            (debit > 0 && credit == 0) || (credit > 0 && debit == 0),
            format!(
                "entry_id={} line={index} debit={debit} credit={credit}",
                string(entry, "entry_id")?
            ),
        )?;
        debits = debits
            .checked_add(debit)
            .ok_or_else(|| "journal debit total overflow".to_owned())?;
        credits = credits
            .checked_add(credit)
            .ok_or_else(|| "journal credit total overflow".to_owned())?;
        let current = balances
            .get(account_id)
            .copied()
            .ok_or_else(|| format!("opening balance missing account {account_id}"))?;
        let delta = signed_difference(debit, credit)?;
        let account_delta = account_deltas.entry(account_id.to_owned()).or_default();
        *account_delta = account_delta
            .checked_add(delta)
            .ok_or_else(|| format!("journal account {account_id} delta overflow"))?;
        balances.insert(
            account_id.to_owned(),
            current
                .checked_add(delta)
                .ok_or_else(|| format!("account {account_id} balance overflow"))?,
        );
    }
    record(
        checks,
        "journal_balanced",
        debits == credits && debits > 0,
        format!(
            "entry_id={} debits={debits} credits={credits}",
            string(entry, "entry_id")?
        ),
    )?;
    let materially_changed_accounts = account_deltas.values().filter(|delta| **delta != 0).count();
    record(
        checks,
        "journal_has_nonzero_effect_on_multiple_accounts",
        materially_changed_accounts >= 2,
        format!(
            "entry_id={} net_changed_accounts={materially_changed_accounts}",
            string(entry, "entry_id")?
        ),
    )?;
    Ok(())
}

fn balances_to_rows(
    balances: &BTreeMap<String, i128>,
    account_catalog: &BTreeMap<String, String>,
) -> AppResult<BTreeMap<String, i128>> {
    if balances.keys().collect::<BTreeSet<_>>() != account_catalog.keys().collect::<BTreeSet<_>>() {
        return Err("reconstructed trial balance account set differs from catalog".to_owned());
    }
    Ok(balances.clone())
}

fn trial_balance_totals(
    balances: &BTreeMap<String, i128>,
    account_catalog: &BTreeMap<String, String>,
) -> AppResult<(u128, u128)> {
    let mut debit = 0_u128;
    let mut credit = 0_u128;
    for (account_id, balance) in balances {
        if !account_catalog.contains_key(account_id) {
            return Err(format!("unknown account {account_id}"));
        }
        if *balance >= 0 {
            debit = debit
                .checked_add(*balance as u128)
                .ok_or_else(|| "closing debit total overflow".to_owned())?;
        } else {
            credit = credit
                .checked_add(balance.unsigned_abs())
                .ok_or_else(|| "closing credit total overflow".to_owned())?;
        }
    }
    Ok((debit, credit))
}

fn reconstruct_statement_totals(
    balances: &BTreeMap<String, i128>,
    account_catalog: &BTreeMap<String, String>,
) -> AppResult<(u128, u128, u128, i128)> {
    let mut assets = 0_u128;
    let mut liabilities = 0_u128;
    let mut equity = 0_u128;
    let mut income = 0_i128;
    let mut expenses = 0_i128;

    for (account_id, balance) in balances {
        let class = account_catalog
            .get(account_id)
            .ok_or_else(|| format!("account {account_id} missing from catalog"))?;
        match class.as_str() {
            "asset" => {
                let value = u128::try_from(*balance).map_err(|_| format!("asset account {account_id} has a credit-normal balance unsupported by smoke profile"))?;
                assets = assets
                    .checked_add(value)
                    .ok_or_else(|| "assets total overflow".to_owned())?;
            }
            "liability" => {
                let value = balance
                    .checked_neg()
                    .ok_or_else(|| "liability negation overflow".to_owned())?;
                let value = u128::try_from(value).map_err(|_| format!("liability account {account_id} has a debit-normal balance unsupported by smoke profile"))?;
                liabilities = liabilities
                    .checked_add(value)
                    .ok_or_else(|| "liabilities total overflow".to_owned())?;
            }
            "equity" => {
                let value = balance
                    .checked_neg()
                    .ok_or_else(|| "equity negation overflow".to_owned())?;
                let value = u128::try_from(value).map_err(|_| format!("equity account {account_id} has a debit-normal balance unsupported by smoke profile"))?;
                equity = equity
                    .checked_add(value)
                    .ok_or_else(|| "equity total overflow".to_owned())?;
            }
            "income" => {
                income = income
                    .checked_sub(*balance)
                    .ok_or_else(|| "income total overflow".to_owned())?;
            }
            "expense" => {
                expenses = expenses
                    .checked_add(*balance)
                    .ok_or_else(|| "expenses total overflow".to_owned())?;
            }
            other => return Err(format!("unsupported account class {other}")),
        }
    }
    let profit = income
        .checked_sub(expenses)
        .ok_or_else(|| "net profit/loss overflow".to_owned())?;
    Ok((assets, liabilities, equity, profit))
}

fn canonical_journal_commitment(entry: &Value) -> AppResult<String> {
    let mut normalized = entry.clone();
    let object = normalized
        .as_object_mut()
        .ok_or_else(|| "journal entry must be an object".to_owned())?;

    if let Some(lines) = object.get_mut("lines").and_then(Value::as_array_mut) {
        lines.sort_by(|left, right| {
            (
                left.get("account_id").and_then(Value::as_str).unwrap_or(""),
                left.get("debit_minor")
                    .and_then(Value::as_str)
                    .unwrap_or(""),
                left.get("credit_minor")
                    .and_then(Value::as_str)
                    .unwrap_or(""),
            )
                .cmp(&(
                    right
                        .get("account_id")
                        .and_then(Value::as_str)
                        .unwrap_or(""),
                    right
                        .get("debit_minor")
                        .and_then(Value::as_str)
                        .unwrap_or(""),
                    right
                        .get("credit_minor")
                        .and_then(Value::as_str)
                        .unwrap_or(""),
                ))
        });
    }
    if let Some(refs) = object
        .get_mut("source_document_refs")
        .and_then(Value::as_array_mut)
    {
        refs.sort_by(|left, right| {
            left.as_str()
                .unwrap_or("")
                .cmp(right.as_str().unwrap_or(""))
        });
    }

    Ok(sha256_hex(&canonical_json(&normalized)?))
}

fn canonical_json(value: &Value) -> AppResult<Vec<u8>> {
    let mut output = Vec::new();
    write_canonical(value, &mut output)?;
    Ok(output)
}

fn write_canonical(value: &Value, output: &mut Vec<u8>) -> AppResult<()> {
    match value {
        Value::Null => output.extend_from_slice(b"null"),
        Value::Bool(value) => {
            if *value {
                output.extend_from_slice(b"true");
            } else {
                output.extend_from_slice(b"false");
            }
        }
        Value::Number(number) => output.extend_from_slice(
            serde_json::to_string(number)
                .map_err(|error| error.to_string())?
                .as_bytes(),
        ),
        Value::String(string) => output.extend_from_slice(
            serde_json::to_string(string)
                .map_err(|error| error.to_string())?
                .as_bytes(),
        ),
        Value::Array(values) => {
            output.push(b'[');
            for (index, value) in values.iter().enumerate() {
                if index > 0 {
                    output.push(b',');
                }
                write_canonical(value, output)?;
            }
            output.push(b']');
        }
        Value::Object(object) => {
            output.push(b'{');
            let mut entries = object.iter().collect::<Vec<_>>();
            entries.sort_by(|left, right| left.0.cmp(right.0));
            for (index, (key, value)) in entries.into_iter().enumerate() {
                if index > 0 {
                    output.push(b',');
                }
                output.extend_from_slice(
                    serde_json::to_string(key)
                        .map_err(|error| error.to_string())?
                        .as_bytes(),
                );
                output.push(b':');
                write_canonical(value, output)?;
            }
            output.push(b'}');
        }
    }
    Ok(())
}

fn amount(value: &Value, name: &str) -> AppResult<u128> {
    let raw = value
        .as_str()
        .ok_or_else(|| format!("{name} must be a decimal integer string"))?;
    if raw.is_empty()
        || !raw.bytes().all(|byte| byte.is_ascii_digit())
        || (raw.len() > 1 && raw.starts_with('0'))
    {
        return Err(format!(
            "{name} is not a canonical non-negative integer string: {raw:?}"
        ));
    }
    raw.parse::<u128>()
        .map_err(|_| format!("{name} exceeds u128: {raw}"))
}

fn signed_amount(value: &Value, name: &str) -> AppResult<i128> {
    let raw = value
        .as_str()
        .ok_or_else(|| format!("{name} must be a signed decimal integer string"))?;
    if raw.is_empty() || raw.trim() != raw {
        return Err(format!("{name} is not canonical: {raw:?}"));
    }
    let digits = raw.strip_prefix('-').unwrap_or(raw);
    if digits.is_empty()
        || !digits.bytes().all(|byte| byte.is_ascii_digit())
        || (digits.len() > 1 && digits.starts_with('0'))
        || raw == "-0"
    {
        return Err(format!(
            "{name} is not a canonical signed integer string: {raw:?}"
        ));
    }
    raw.parse::<i128>()
        .map_err(|_| format!("{name} exceeds i128: {raw}"))
}

fn signed_difference(debit: u128, credit: u128) -> AppResult<i128> {
    let debit = i128::try_from(debit).map_err(|_| "debit amount exceeds i128".to_owned())?;
    let credit = i128::try_from(credit).map_err(|_| "credit amount exceeds i128".to_owned())?;
    debit
        .checked_sub(credit)
        .ok_or_else(|| "signed amount difference overflow".to_owned())
}

fn record(checks: &mut Vec<Check>, invariant: &str, passed: bool, detail: String) -> AppResult<()> {
    checks.push(Check {
        invariant: invariant.to_owned(),
        passed,
        detail: detail.clone(),
    });
    if passed {
        Ok(())
    } else {
        Err(format!("{invariant}: {detail}"))
    }
}

fn field<'a>(value: &'a Value, key: &str) -> AppResult<&'a Value> {
    value.get(key).ok_or_else(|| format!("missing field {key}"))
}

fn string<'a>(value: &'a Value, key: &str) -> AppResult<&'a str> {
    field(value, key)?
        .as_str()
        .ok_or_else(|| format!("{key} must be a string"))
}

fn sha256_hex(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn verifier_source_digest() -> String {
    let mut hasher = Sha256::new();
    for (path, bytes) in [
        ("Cargo.toml", include_bytes!("../Cargo.toml").as_slice()),
        ("README.md", include_bytes!("../README.md").as_slice()),
        (
            "report.schema.json",
            include_bytes!("../report.schema.json").as_slice(),
        ),
        ("src/main.rs", include_bytes!("main.rs").as_slice()),
    ] {
        hasher.update(path.as_bytes());
        hasher.update([0]);
        hasher.update(bytes);
        hasher.update([0]);
    }
    hex::encode(hasher.finalize())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (Value, Value) {
        let corpus: Value = serde_json::from_str(include_str!("../../smoke_corpus_v0.1.0.json"))
            .expect("smoke corpus JSON must parse");
        let schema: Value =
            serde_json::from_str(include_str!("../../smoke_corpus_v0.1.0.schema.json"))
                .expect("smoke schema JSON must parse");
        (corpus, schema)
    }

    fn report_for(corpus: &Value, schema: &Value) -> Value {
        let corpus_bytes = serde_json::to_vec(corpus).expect("corpus serializes");
        let schema_bytes = serde_json::to_vec(schema).expect("schema serializes");
        verify_corpus(
            corpus,
            schema,
            &corpus_bytes,
            &schema_bytes,
            None,
            "test-toolchain",
        )
    }

    #[test]
    fn frozen_seed_cases_reconstruct() {
        let (corpus, schema) = fixture();
        let report = report_for(&corpus, &schema);
        assert_eq!(report["status"], "PASS", "{report:#}");
        assert_eq!(report["case_count"], 3);
        assert!(
            report["cases"]
                .as_array()
                .unwrap()
                .iter()
                .all(|case| case["status"] == "PASS")
        );
    }

    #[test]
    fn full_schema_validation_rejects_unknown_root_fields() {
        let (mut corpus, schema) = fixture();
        corpus["unlisted_root_field"] = Value::Bool(true);
        let report = report_for(&corpus, &schema);
        assert_eq!(report["status"], "FAIL");
        assert_eq!(report["schema_validation"]["passed"], false);
        assert_eq!(report["report_schema_validation"]["passed"], true);
    }

    #[test]
    fn source_byte_mutation_is_rejected() {
        let (mut corpus, schema) = fixture();
        let content = corpus["cases"][0]["documents"][0]["content"]["lines"][0]["quantity"]
            .as_str()
            .unwrap()
            .to_owned();
        corpus["cases"][0]["documents"][0]["content"]["lines"][0]["quantity"] =
            Value::String(format!("{content}1"));
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "source_document_fingerprint" && check["passed"] == false
                })
        );
    }

    #[test]
    fn duplicate_identity_with_changed_content_is_rejected() {
        let (mut corpus, schema) = fixture();
        let duplicate = corpus["cases"][0]["documents"][0].clone();
        let mut changed = duplicate.clone();
        changed["content"]["lines"][0]["quantity"] = Value::String("11".to_owned());
        let digest = sha256_hex(&canonical_json(&changed["content"]).unwrap());
        changed["fingerprint_sha256"] = Value::String(digest);
        changed["ingestion_id"] = Value::String("ING-PO-CONFLICT".to_owned());
        corpus["cases"][0]["documents"]
            .as_array_mut()
            .unwrap()
            .push(changed);

        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "duplicate_identity_same_fingerprint"
                        && check["passed"] == false
                })
        );
    }

    #[test]
    fn policy_revision_mutation_is_rejected_for_its_binding_reason() {
        let (mut corpus, schema) = fixture();
        corpus["cases"][0]["expected_journal"][0]["policy_revision"] =
            Value::String("2".to_owned());
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "journal_profile_policy_binding"
                        && check["passed"] == false
                })
        );
    }

    #[test]
    fn source_currency_mutation_is_rejected_after_digest_recalculation() {
        let (mut corpus, schema) = fixture();
        let bank = &mut corpus["cases"][0]["documents"][3];
        bank["currency"] = Value::String("USD".to_owned());
        bank["content"]["currency"] = Value::String("USD".to_owned());
        let digest = sha256_hex(&canonical_json(&bank["content"]).unwrap());
        bank["fingerprint_sha256"] = Value::String(digest);
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "source_currency_profile_binding"
                        && check["passed"] == false
                })
        );
    }

    #[test]
    fn tampered_statement_total_is_rejected_by_reconstruction() {
        let (mut corpus, schema) = fixture();
        corpus["cases"][0]["expected_close"]["statement_totals"]["total_assets_minor"] =
            Value::String("500001".to_owned());
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "independent_statement_reconstruction"
                        && check["passed"] == false
                })
        );
    }

    #[test]
    fn schema_date_format_is_enforced() {
        let (mut corpus, schema) = fixture();
        corpus["generated_on"] = Value::String("not-a-date".to_owned());
        let report = report_for(&corpus, &schema);
        assert_eq!(report["status"], "FAIL");
        assert_eq!(report["schema_validation"]["passed"], false);
    }

    #[test]
    fn fingerprint_only_mutation_is_rejected() {
        let (mut corpus, schema) = fixture();
        corpus["cases"][0]["documents"][0]["fingerprint_sha256"] = Value::String("00".repeat(32));
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "source_document_fingerprint" && check["passed"] == false
                })
        );
    }

    #[test]
    fn wrong_entity_mutation_is_rejected_even_with_recomputed_digest() {
        let (mut corpus, schema) = fixture();
        let invoice = &mut corpus["cases"][0]["documents"][2];
        invoice["entity_id"] = Value::String("co-002".to_owned());
        invoice["content"]["entity_id"] = Value::String("co-002".to_owned());
        let digest = sha256_hex(&canonical_json(&invoice["content"]).unwrap());
        invoice["fingerprint_sha256"] = Value::String(digest);

        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "economic_evidence_disposition"
                        && check["passed"] == false
                })
        );
    }

    #[test]
    fn one_minor_unit_journal_mutation_is_rejected() {
        let (mut corpus, schema) = fixture();
        corpus["cases"][0]["expected_journal"][0]["lines"][0]["debit_minor"] =
            Value::String("100001".to_owned());
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "journal_balanced" && check["passed"] == false
                })
        );
    }

    #[test]
    fn self_cancelling_single_account_journal_is_rejected() {
        let (mut corpus, schema) = fixture();
        corpus["cases"][0]["expected_journal"][0]["lines"][1]["account_id"] =
            Value::String("1200".to_owned());
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "journal_has_nonzero_effect_on_multiple_accounts"
                        && check["passed"] == false
                })
        );
    }

    #[test]
    fn multiple_same_side_postings_to_one_account_are_allowed() {
        let (mut corpus, schema) = fixture();
        let debit = &mut corpus["cases"][0]["expected_journal"][0]["lines"][0]["debit_minor"];
        *debit = Value::String("40000".to_owned());
        corpus["cases"][0]["expected_journal"][0]["lines"]
            .as_array_mut()
            .unwrap()
            .push(json!({
                "account_id": "1200",
                "debit_minor": "60000",
                "credit_minor": "0"
            }));
        assert_eq!(report_for(&corpus, &schema)["status"], "PASS");
    }

    #[test]
    fn balanced_but_wrong_source_amount_is_rejected() {
        let (mut corpus, schema) = fixture();
        corpus["cases"][0]["expected_journal"][0]["lines"][0]["debit_minor"] =
            Value::String("200000".to_owned());
        corpus["cases"][0]["expected_journal"][0]["lines"][1]["credit_minor"] =
            Value::String("200000".to_owned());
        let report = report_for(&corpus, &schema);
        assert_eq!(report["cases"][0]["status"], "FAIL");
        assert!(
            report["cases"][0]["checks"]
                .as_array()
                .unwrap()
                .iter()
                .any(|check| {
                    check["invariant"] == "journal_source_projection" && check["passed"] == false
                })
        );
    }

    #[test]
    fn duplicate_invoice_is_noop_in_frozen_case() {
        let (corpus, schema) = fixture();
        let report = report_for(&corpus, &schema);
        let checks = report["cases"][1]["checks"].as_array().unwrap();
        assert!(checks.iter().any(|check| {
            check["invariant"] == "economic_evidence_disposition"
                && check["passed"] == true
                && check["detail"]
                    .as_str()
                    .unwrap()
                    .contains("duplicate_no_op")
        }));
    }

    #[test]
    fn posting_permutation_does_not_change_journal_commitment_or_close() {
        let (mut corpus, schema) = fixture();
        let original =
            canonical_journal_commitment(&corpus["cases"][0]["expected_journal"][0]).unwrap();
        corpus["cases"][0]["expected_journal"][0]["lines"]
            .as_array_mut()
            .unwrap()
            .reverse();
        let permuted =
            canonical_journal_commitment(&corpus["cases"][0]["expected_journal"][0]).unwrap();
        assert_eq!(original, permuted);
        assert_eq!(report_for(&corpus, &schema)["status"], "PASS");
    }

    #[test]
    fn amount_parser_rejects_noncanonical_and_overflow_values() {
        for invalid in ["", "00", "01", "+1", "-1", " 1", "1 ", "1.0", "1e3"] {
            assert!(
                amount(&Value::String(invalid.to_owned()), "test").is_err(),
                "{invalid:?}"
            );
        }
        assert!(
            amount(
                &Value::String("340282366920938463463374607431768211456".to_owned()),
                "test"
            )
            .is_err()
        );
        assert_eq!(amount(&Value::String("0".to_owned()), "test").unwrap(), 0);
    }

    #[test]
    fn canonical_json_orders_object_keys_but_preserves_array_order() {
        let value: Value =
            serde_json::from_str(r#"{"z":1,"a":{"d":2,"b":3},"items":[2,1]}"#).unwrap();
        let bytes = canonical_json(&value).unwrap();
        assert_eq!(
            String::from_utf8(bytes).unwrap(),
            r#"{"a":{"b":3,"d":2},"items":[2,1],"z":1}"#
        );
    }

    #[test]
    fn repeated_execution_is_deterministic() {
        let (corpus, schema) = fixture();
        let one = serde_json::to_vec(&report_for(&corpus, &schema)).unwrap();
        let two = serde_json::to_vec(&report_for(&corpus, &schema)).unwrap();
        assert_eq!(one, two);
        let report: Value = serde_json::from_slice(&one).unwrap();
        assert_eq!(report["report_schema_validation"]["passed"], true);
    }
}
