use mobility_configuration_qualification::{parse_corpus, qualify, QualificationResult};
use serde::Serialize;
use std::{env, fs, process};

#[derive(Serialize)]
struct Normalized<'a> {
    schema: &'static str,
    schema_version: &'a str,
    status: &'a str,
    vectors: Vec<NormalizedVector<'a>>,
}

#[derive(Serialize)]
struct NormalizedVector<'a> {
    id: &'a str,
    scenario: &'a str,
    expected_outcome: &'a str,
    forbidden_inference: &'a str,
}

fn main() {
    let path = env::args().nth(1).unwrap_or_else(|| {
        "../../../docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1.json".into()
    });
    let input = fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("cannot read {path}: {e}");
        process::exit(2);
    });
    let corpus = parse_corpus(&input).unwrap_or_else(|e| {
        eprintln!("{e}");
        process::exit(2);
    });
    if let QualificationResult::Invalid(reason) = qualify(&corpus) {
        eprintln!("qualification failed: {reason}");
        process::exit(1);
    }

    let mut vectors: Vec<_> = corpus
        .vectors
        .iter()
        .map(|v| NormalizedVector {
            id: &v.id,
            scenario: &v.scenario,
            expected_outcome: &v.expected_outcome,
            forbidden_inference: &v.forbidden_inference,
        })
        .collect();
    vectors.sort_by(|a, b| a.id.cmp(b.id));

    let normalized = Normalized {
        schema: "mobility-qualification-normalized-v1",
        schema_version: &corpus.schema_version,
        status: &corpus.status,
        vectors,
    };
    println!(
        "{}",
        serde_json::to_string(&normalized).expect("serialize normalized result")
    );
}
