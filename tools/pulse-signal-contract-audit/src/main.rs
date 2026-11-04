use regex::Regex;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;
use syn::{Fields, Item, ItemEnum, LitStr};

#[derive(Clone, Copy)]
struct ZomeSpec {
    category: &'static str,
    rust_path: &'static str,
    rust_enum: &'static str,
    ts_alias: &'static str,
}

const ZOMES: [ZomeSpec; 4] = [
    ZomeSpec { category: "message", rust_path: "holochain/zomes/messages/coordinator/src/lib.rs", rust_enum: "MailSignal", ts_alias: "MessageSignal" },
    ZomeSpec { category: "trust", rust_path: "holochain/zomes/trust/coordinator/src/lib.rs", rust_enum: "TrustSignal", ts_alias: "TrustSignal" },
    ZomeSpec { category: "sync", rust_path: "holochain/zomes/sync/coordinator/src/lib.rs", rust_enum: "SyncSignal", ts_alias: "SyncSignal" },
    ZomeSpec { category: "federation", rust_path: "holochain/zomes/federation/coordinator/src/lib.rs", rust_enum: "FederationSignal", ts_alias: "FederationSignal" },
];

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum RustRepresentation {
    InternallyTagged { tag: String },
    AdjacentlyTagged { tag: String, content: String },
    Untagged,
    ExternallyTagged,
}

#[derive(Debug, Clone, Serialize)]
struct VariantReport {
    name: String,
    rust_fields: Vec<String>,
    typescript_fields: Option<Vec<String>>,
    present_in_typescript: bool,
    field_set_matches: bool,
}

#[derive(Debug, Clone, Serialize)]
struct ZomeReport {
    category: String,
    rust_enum: String,
    typescript_alias: String,
    rust_representation: RustRepresentation,
    rust_variants: Vec<VariantReport>,
    missing_from_typescript: Vec<String>,
    extra_in_typescript: Vec<String>,
    signal_hub_filter_array_count: usize,
    signal_hub_filters_complete: bool,
    codec_compatible_with_v1_envelope_decoder: bool,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    schema_version: u32,
    status: String,
    repository_root: String,
    signal_hub_decodes_adjacent_type_data_envelope: bool,
    zomes: Vec<ZomeReport>,
    findings: Vec<String>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut root = env::current_dir()?;
    let mut output: Option<PathBuf> = None;
    let mut report_only = false;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--root" => root = PathBuf::from(args.next().ok_or("--root requires a path")?),
            "--output" => output = Some(PathBuf::from(args.next().ok_or("--output requires a path")?)),
            "--report-only" => report_only = true,
            "--help" | "-h" => {
                println!("pulse-signal-contract-audit [--root REPOSITORY_ROOT] [--output REPORT.json] [--report-only]");
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}").into()),
        }
    }

    let client_root = root.join("holochain/client/src");
    let ts_types = fs::read_to_string(client_root.join("types/index.ts"))?;
    let signal_hub = fs::read_to_string(client_root.join("signals/SignalHub.ts"))?;
    let filters = extract_filter_arrays(&signal_hub)?;
    let adjacent_envelope = signal_hub_decodes_adjacent_envelope(&signal_hub);

    let mut findings = Vec::new();
    let mut zomes = Vec::new();

    for spec in ZOMES {
        let rust_path = root.join(spec.rust_path);
        let source = fs::read_to_string(&rust_path)?;
        let file = syn::parse_file(&source)?;
        let item = file.items.iter().find_map(|item| match item {
            Item::Enum(item) if item.ident == spec.rust_enum => Some(item),
            _ => None,
        }).ok_or_else(|| format!("{} not found in {}", spec.rust_enum, rust_path.display()))?;

        let representation = rust_representation(item)?;
        let rust_variants = rust_variants(item);
        let ts_body = extract_type_alias(&ts_types, spec.ts_alias)
            .ok_or_else(|| format!("TypeScript alias {} not found", spec.ts_alias))?;
        let ts_variants = extract_typescript_variants(ts_body)?;
        let category_filters = filters.iter()
            .filter(|array| array.category == spec.category)
            .collect::<Vec<_>>();

        let rust_names = rust_variants.iter().map(|v| v.name.clone()).collect::<BTreeSet<_>>();
        let ts_names = ts_variants.keys().cloned().collect::<BTreeSet<_>>();
        let missing = rust_names.difference(&ts_names).cloned().collect::<Vec<_>>();
        let extra = ts_names.difference(&rust_names).cloned().collect::<Vec<_>>();

        for name in &missing {
            findings.push(format!("{}: Rust variant {} is missing from TypeScript {}", spec.category, name, spec.ts_alias));
        }
        for name in &extra {
            findings.push(format!("{}: TypeScript variant {} is absent from Rust {}", spec.category, name, spec.rust_enum));
        }

        let mut variant_reports = Vec::new();
        for rust_variant in &rust_variants {
            let ts_fields = ts_variants.get(&rust_variant.name).cloned();
            let field_match = ts_fields.as_ref().is_some_and(|fields| {
                fields.iter().cloned().collect::<BTreeSet<_>>()
                    == rust_variant.fields.iter().cloned().collect::<BTreeSet<_>>()
            });
            if let Some(fields) = &ts_fields {
                if !field_match {
                    findings.push(format!("{}: field drift for {} (Rust {:?}, TypeScript {:?})", spec.category, rust_variant.name, rust_variant.fields, fields));
                }
            }
            variant_reports.push(VariantReport {
                name: rust_variant.name.clone(),
                rust_fields: rust_variant.fields.clone(),
                typescript_fields: ts_fields,
                present_in_typescript: ts_names.contains(&rust_variant.name),
                field_set_matches: field_match,
            });
        }

        let filters_complete = category_filters.len() == 2
            && category_filters.iter().all(|filter| filter.variants == rust_names);
        if category_filters.len() != 2 {
            findings.push(format!("{}: expected two SignalHub category filter arrays, found {}", spec.category, category_filters.len()));
        }
        for (index, filter) in category_filters.iter().enumerate() {
            let absent = rust_names.difference(&filter.variants).cloned().collect::<Vec<_>>();
            let stale = filter.variants.difference(&rust_names).cloned().collect::<Vec<_>>();
            if !absent.is_empty() || !stale.is_empty() {
                findings.push(format!("{}: SignalHub filter {} drift; missing {:?}, stale {:?}", spec.category, index + 1, absent, stale));
            }
        }

        // Pulse v1 uses one explicit adjacent-tag wire contract: { type, data }.
        // SignalHub validates that envelope and normalizes the data object to the
        // flat discriminated union used by existing client consumers.
        let codec_compatible = matches!(
            &representation,
            RustRepresentation::AdjacentlyTagged { tag, content }
                if tag == "type" && content == "data"
        ) && adjacent_envelope;
        if !codec_compatible {
            findings.push(format!("{}: {:?} is incompatible with SignalHub's v1 type/data envelope decoder", spec.category, representation));
        }

        zomes.push(ZomeReport {
            category: spec.category.to_string(),
            rust_enum: spec.rust_enum.to_string(),
            typescript_alias: spec.ts_alias.to_string(),
            rust_representation: representation,
            rust_variants: variant_reports,
            missing_from_typescript: missing,
            extra_in_typescript: extra,
            signal_hub_filter_array_count: category_filters.len(),
            signal_hub_filters_complete: filters_complete,
            codec_compatible_with_v1_envelope_decoder: codec_compatible,
        });
    }

    let report = AuditReport {
        schema_version: 1,
        status: if findings.is_empty() { "PASS" } else { "FAIL" }.to_string(),
        repository_root: root.display().to_string(),
        signal_hub_decodes_adjacent_type_data_envelope: adjacent_envelope,
        zomes,
        findings,
    };
    let json = serde_json::to_string_pretty(&report)?;
    if let Some(path) = output {
        if let Some(parent) = path.parent() { fs::create_dir_all(parent)?; }
        fs::write(path, &json)?;
    } else {
        println!("{json}");
    }
    if report.status == "FAIL" && !report_only {
        return Err(format!("signal contract drift: {} finding(s)", report.findings.len()).into());
    }
    Ok(())
}

#[derive(Debug, Clone)]
struct RustVariant {
    name: String,
    fields: Vec<String>,
}

fn rust_variants(item: &ItemEnum) -> Vec<RustVariant> {
    item.variants.iter().map(|variant| {
        let fields = match &variant.fields {
            Fields::Named(fields) => fields.named.iter()
                .filter_map(|field| field.ident.as_ref().map(ToString::to_string))
                .collect(),
            Fields::Unnamed(fields) => (0..fields.unnamed.len()).map(|i| format!("_{i}")).collect(),
            Fields::Unit => Vec::new(),
        };
        RustVariant { name: variant.ident.to_string(), fields }
    }).collect()
}

fn rust_representation(item: &ItemEnum) -> Result<RustRepresentation, Box<dyn Error>> {
    for attr in &item.attrs {
        if !attr.path().is_ident("serde") { continue; }
        let mut tag: Option<String> = None;
        let mut content: Option<String> = None;
        let mut untagged = false;
        attr.parse_nested_meta(|meta| {
            if meta.path.is_ident("tag") {
                let value: LitStr = meta.value()?.parse()?;
                tag = Some(value.value());
            } else if meta.path.is_ident("content") {
                let value: LitStr = meta.value()?.parse()?;
                content = Some(value.value());
            } else if meta.path.is_ident("untagged") {
                untagged = true;
            }
            Ok(())
        })?;
        if untagged { return Ok(RustRepresentation::Untagged); }
        if let Some(tag) = tag {
            return Ok(if let Some(content) = content {
                RustRepresentation::AdjacentlyTagged { tag, content }
            } else {
                RustRepresentation::InternallyTagged { tag }
            });
        }
    }
    Ok(RustRepresentation::ExternallyTagged)
}

#[derive(Debug, Clone)]
struct FilterArray {
    category: String,
    variants: BTreeSet<String>,
}

fn signal_hub_decodes_adjacent_envelope(source: &str) -> bool {
    source.contains("const envelope = payload as { type?: unknown; data?: unknown }")
        && source.contains("typeof envelope.type !== 'string'")
        && source.contains("envelope.data")
        && source.contains("...data, type")
}

fn extract_filter_arrays(source: &str) -> Result<Vec<FilterArray>, Box<dyn Error>> {
    let array_re = Regex::new(r"(?s)\[([^\[\]]*)\]\.includes\(type\)")?;
    let string_re = Regex::new(r#"['"]([A-Za-z][A-Za-z0-9_]*)['"]"#)?;
    let mut arrays = Vec::new();
    let phases = [
        ("message", "// Message signals", "// Trust signals"),
        ("trust", "// Trust signals", "// Sync signals"),
        ("sync", "// Sync signals", "// Federation signals"),
        ("federation", "// Federation signals", ""),
    ];

    // Parse only the requested method body. Substring slicing between method
    // names is unsafe when one method is absent in a test fixture or renamed:
    // it can accidentally count another method's filters multiple times.
    for signature in ["private parseSignal(", "private dispatchToCategory("] {
        let Some(method) = extract_method_body(source, signature) else { continue; };
        for (category, start_marker, end_marker) in phases {
            let Some(start) = method.find(start_marker) else { continue; };
            let rest = &method[start + start_marker.len()..];
            let end = if end_marker.is_empty() {
                rest.len()
            } else {
                rest.find(end_marker).unwrap_or(rest.len())
            };
            let section = &rest[..end];
            for capture in array_re.captures_iter(section) {
                let variants = string_re.captures_iter(&capture[1])
                    .map(|c| c[1].to_string()).collect::<BTreeSet<_>>();
                arrays.push(FilterArray { category: category.to_string(), variants });
            }
        }
    }
    Ok(arrays)
}

fn extract_method_body<'a>(source: &'a str, signature: &str) -> Option<&'a str> {
    let signature_start = source.find(signature)?;
    let open = source[signature_start..].find('{')? + signature_start;
    let close = matching_brace(source, open)?;
    Some(&source[open + 1..close])
}

fn extract_type_alias<'a>(source: &'a str, alias: &str) -> Option<&'a str> {
    let marker = format!("export type {alias} =");
    let start = source.find(&marker)? + marker.len();
    let tail = &source[start..];
    let mut depth = 0i32;
    for (offset, ch) in tail.char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => depth -= 1,
            ';' if depth == 0 => return Some(&tail[..offset]),
            _ => {}
        }
    }
    None
}

fn extract_typescript_variants(body: &str) -> Result<BTreeMap<String, Vec<String>>, Box<dyn Error>> {
    let variant_re = Regex::new(r#"\{\s*type\s*:\s*['"]([A-Za-z][A-Za-z0-9_]*)['"]"#)?;
    let field_re = Regex::new(r"([A-Za-z_][A-Za-z0-9_]*)\s*:")?;
    let mut variants = BTreeMap::new();
    for capture in variant_re.captures_iter(body) {
        let name = capture[1].to_string();
        // The regex begins at the discriminated object's opening brace.
        let open = capture.get(0).unwrap().start();
        let close = matching_brace(body, open).ok_or("variant closing brace not found")?;
        let object = &body[open..=close];
        let fields = field_re.captures_iter(object)
            .map(|c| c[1].to_string())
            .filter(|field| field != "type")
            .collect::<Vec<_>>();
        if variants.insert(name.clone(), fields).is_some() {
            return Err(format!("duplicate TypeScript variant {name}").into());
        }
    }
    Ok(variants)
}

fn matching_brace(source: &str, open: usize) -> Option<usize> {
    let mut depth = 0i32;
    for (offset, ch) in source[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 { return Some(open + offset); }
            }
            _ => {}
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serde_tagged_mail_signal_is_adjacently_tagged() {
        let source = r#"
            #[derive(Serialize, Deserialize)]
            #[serde(tag = "type", content = "data")]
            pub enum MailSignal { EmailReceived { email_hash: ActionHash }, Online }
        "#;
        let file = syn::parse_file(source).unwrap();
        let item = file.items.iter().find_map(|item| match item {
            Item::Enum(item) => Some(item),
            _ => None,
        }).unwrap();
        assert!(matches!(
            rust_representation(item).unwrap(),
            RustRepresentation::AdjacentlyTagged { tag, content }
                if tag == "type" && content == "data"
        ));
    }

    #[test]
    fn external_unit_and_struct_variants_are_recorded() {
        let source = "pub enum SyncSignal { Online, SyncComplete { peer: AgentPubKey } }";
        let file = syn::parse_file(source).unwrap();
        let item = file.items.iter().find_map(|item| match item {
            Item::Enum(item) => Some(item),
            _ => None,
        }).unwrap();
        assert!(matches!(rust_representation(item).unwrap(), RustRepresentation::ExternallyTagged));
        let variants = rust_variants(item);
        assert_eq!(variants[0].name, "Online");
        assert_eq!(variants[1].fields, vec!["peer"]);
    }

    #[test]
    fn typescript_union_variant_and_fields_are_extracted() {
        let source = "export type DemoSignal = | { type: 'One'; value: string } | { type: 'Two' };";
        let body = extract_type_alias(source, "DemoSignal").unwrap();
        let variants = extract_typescript_variants(body).unwrap();
        assert_eq!(variants["One"], vec!["value"]);
        assert!(variants.contains_key("Two"));
    }

    #[test]
    fn signal_hub_adjacent_envelope_decoder_is_required() {
        let safe = r#"
            const envelope = payload as { type?: unknown; data?: unknown };
            if (typeof envelope.type !== 'string') return null;
            if (envelope.data === undefined) return null;
            const normalized = { ...data, type };
        "#;
        assert!(signal_hub_decodes_adjacent_envelope(safe));

        let flat_only = r#"
            const payload = appSignal.payload as { type?: string };
            if (typeof payload.type !== 'string') return null;
        "#;
        assert!(!signal_hub_decodes_adjacent_envelope(flat_only));
    }

    #[test]
    fn signal_hub_filter_arrays_are_extracted_per_category_twice() {
        let source = r#"
            private parseSignal() {
              // Message signals
              if (['One'].includes(type)) {}
              // Trust signals
              if (['Two'].includes(type)) {}
              // Sync signals
              if (['Three'].includes(type)) {}
              // Federation signals
              if (['Four'].includes(type)) {}
            }
            private dispatchToCategory() {
              // Message signals
              if (['One'].includes(type)) {}
              // Trust signals
              if (['Two'].includes(type)) {}
              // Sync signals
              if (['Three'].includes(type)) {}
              // Federation signals
              if (['Four'].includes(type)) {}
            }
            private matchesSubscription() {}
        "#;
        let arrays = extract_filter_arrays(source).unwrap();
        assert_eq!(arrays.iter().filter(|a| a.category == "message").count(), 2);
        assert_eq!(arrays.iter().filter(|a| a.category == "federation").count(), 2);
    }

    #[test]
    fn source_with_a_new_variant_fails_the_contract() {
        let rust_source = "pub enum MailSignal { EmailReceived, NewVariant }";
        let file = syn::parse_file(rust_source).unwrap();
        let item = file.items.iter().find_map(|item| match item {
            Item::Enum(item) => Some(item),
            _ => None,
        }).unwrap();
        let names = rust_variants(item).into_iter().map(|v| v.name).collect::<BTreeSet<_>>();
        let ts = BTreeSet::from(["EmailReceived".to_string()]);
        assert_eq!(names.difference(&ts).cloned().collect::<Vec<_>>(), vec!["NewVariant"]);
    }
}
