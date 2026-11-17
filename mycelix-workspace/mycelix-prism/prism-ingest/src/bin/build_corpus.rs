// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Programmatic corpus builder — extracts factual claims from Wikipedia.
//!
//! Fetches Wikipedia article summaries via the REST API and extracts
//! key factual sentences with automatic epistemic classification.
//!
//! Usage: cargo run -p prism-ingest --features fetch --bin build_corpus --release
//!
//! Output: claims/wikipedia-extracts.json

use prism_egress::{EgressAuthority, EgressLimits, HeaderMap, Method, PublicEgressClient};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::time::Duration;
use url::Url;

const MAX_WIKIPEDIA_RESPONSE_BYTES: u64 = 2 * 1024 * 1024;
const SOURCE_MANIFEST_ID: &str = "luminous.prism.ingest.sources";
const SOURCE_MANIFEST_VERSION: &str = "2026-11-17.1";
const SOURCE_MANIFEST_JSON: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/sources/manifest.json"
));

#[derive(Deserialize)]
struct WikiSummary {
    title: Option<String>,
    extract: Option<String>,
    description: Option<String>,
}

#[derive(Serialize)]
struct JsonClaim {
    claim: String,
    level: String,
    sources: Vec<String>,
    tags: Vec<String>,
}

/// Topics to fetch from Wikipedia, grouped by domain.
const TOPICS: &[(&str, &[&str], &str)] = &[
    // (domain_tag, [article_titles], e_level)
    // Science
    (
        "physics",
        &[
            "Quantum_mechanics",
            "General_relativity",
            "Thermodynamics",
            "Electromagnetism",
            "Particle_physics",
            "Nuclear_physics",
            "Condensed_matter_physics",
            "Plasma_(physics)",
            "Optics",
            "Acoustics",
            "Fluid_dynamics",
            "Statistical_mechanics",
            "Quantum_field_theory",
            "String_theory",
            "Cosmology",
        ],
        "E4",
    ),
    (
        "chemistry",
        &[
            "Chemical_bond",
            "Organic_chemistry",
            "Biochemistry",
            "Electrochemistry",
            "Photochemistry",
            "Polymer_chemistry",
            "Nuclear_chemistry",
            "Analytical_chemistry",
            "Physical_chemistry",
            "Inorganic_chemistry",
            "Catalysis",
            "Chemical_reaction",
            "Stoichiometry",
            "Thermochemistry",
        ],
        "E4",
    ),
    (
        "biology",
        &[
            "Cell_(biology)",
            "DNA",
            "Evolution",
            "Ecology",
            "Genetics",
            "Microbiology",
            "Immunology",
            "Neuroscience",
            "Botany",
            "Zoology",
            "Marine_biology",
            "Molecular_biology",
            "Developmental_biology",
            "Epidemiology",
            "Virology",
        ],
        "E4",
    ),
    (
        "medicine",
        &[
            "Vaccine",
            "Antibiotic",
            "Cancer",
            "Diabetes",
            "Cardiovascular_disease",
            "Mental_health",
            "Surgery",
            "Pharmacology",
            "Public_health",
            "Nutrition",
            "Epidemiology",
            "Immunotherapy",
            "Gene_therapy",
            "Telemedicine",
        ],
        "E3",
    ),
    (
        "astronomy",
        &[
            "Black_hole",
            "Neutron_star",
            "Exoplanet",
            "Galaxy",
            "Solar_System",
            "Big_Bang",
            "Dark_matter",
            "Dark_energy",
            "Supernova",
            "Pulsar",
            "Quasar",
            "Asteroid",
            "Comet",
            "Nebula",
        ],
        "E4",
    ),
    // Technology
    (
        "computer-science",
        &[
            "Algorithm",
            "Data_structure",
            "Operating_system",
            "Computer_network",
            "Database",
            "Compiler",
            "Artificial_intelligence",
            "Machine_learning",
            "Cryptography",
            "Computer_security",
            "Software_engineering",
            "Distributed_computing",
            "Computer_graphics",
            "Natural_language_processing",
        ],
        "E4",
    ),
    (
        "technology",
        &[
            "Internet",
            "World_Wide_Web",
            "Blockchain",
            "Cloud_computing",
            "Quantum_computing",
            "Robotics",
            "Nanotechnology",
            "Biotechnology",
            "Renewable_energy",
            "Electric_vehicle",
            "3D_printing",
            "Virtual_reality",
            "Augmented_reality",
            "Internet_of_things",
        ],
        "E3",
    ),
    // Earth Sciences
    (
        "climate",
        &[
            "Climate_change",
            "Global_warming",
            "Greenhouse_gas",
            "Carbon_dioxide",
            "Sea_level_rise",
            "Ocean_acidification",
            "Deforestation",
            "Renewable_energy",
            "Paris_Agreement",
            "Carbon_capture_and_storage",
        ],
        "E4",
    ),
    (
        "geology",
        &[
            "Plate_tectonics",
            "Earthquake",
            "Volcano",
            "Mineral",
            "Rock_(geology)",
            "Fossil",
            "Geomorphology",
            "Glaciology",
            "Hydrology",
            "Oceanography",
        ],
        "E4",
    ),
    (
        "ecology",
        &[
            "Ecosystem",
            "Biodiversity",
            "Endangered_species",
            "Conservation_biology",
            "Deforestation",
            "Coral_reef",
            "Wetland",
            "Rainforest",
            "Pollination",
            "Food_web",
        ],
        "E3",
    ),
    // Social Sciences
    (
        "economics",
        &[
            "Economics",
            "Gross_domestic_product",
            "Inflation",
            "Supply_and_demand",
            "Market_economy",
            "Monetary_policy",
            "Fiscal_policy",
            "International_trade",
            "Behavioral_economics",
            "Development_economics",
        ],
        "E4",
    ),
    (
        "psychology",
        &[
            "Psychology",
            "Cognitive_psychology",
            "Developmental_psychology",
            "Social_psychology",
            "Clinical_psychology",
            "Neuroscience",
            "Memory",
            "Emotion",
            "Intelligence",
            "Motivation",
        ],
        "E3",
    ),
    (
        "sociology",
        &[
            "Sociology",
            "Social_inequality",
            "Globalization",
            "Urbanization",
            "Social_movement",
            "Culture",
            "Education",
            "Poverty",
            "Migration",
            "Demographics",
        ],
        "E3",
    ),
    (
        "politics",
        &[
            "Democracy",
            "Authoritarianism",
            "Human_rights",
            "International_law",
            "United_Nations",
            "European_Union",
            "Constitution",
            "Civil_liberties",
            "Rule_of_law",
            "Election",
        ],
        "E4",
    ),
    // Humanities
    (
        "philosophy",
        &[
            "Philosophy",
            "Ethics",
            "Epistemology",
            "Logic",
            "Metaphysics",
            "Aesthetics",
            "Political_philosophy",
            "Philosophy_of_mind",
            "Philosophy_of_science",
            "Existentialism",
        ],
        "E4",
    ),
    (
        "history",
        &[
            "Ancient_history",
            "Middle_Ages",
            "Renaissance",
            "Industrial_Revolution",
            "World_War_I",
            "World_War_II",
            "Cold_War",
            "Colonialism",
            "Decolonization",
            "Civil_rights_movement",
        ],
        "E4",
    ),
    (
        "linguistics",
        &[
            "Linguistics",
            "Phonetics",
            "Syntax",
            "Semantics",
            "Pragmatics",
            "Sociolinguistics",
            "Language_acquisition",
            "Morphology_(linguistics)",
            "Writing_system",
            "Endangered_language",
        ],
        "E3",
    ),
    // Mathematics
    (
        "mathematics",
        &[
            "Mathematics",
            "Calculus",
            "Linear_algebra",
            "Number_theory",
            "Topology",
            "Graph_theory",
            "Probability_theory",
            "Statistics",
            "Differential_equation",
            "Group_theory",
        ],
        "E4",
    ),
];

fn extract_sentences(text: &str) -> Vec<String> {
    let mut sentences = Vec::new();
    let mut current = String::new();

    for ch in text.chars() {
        current.push(ch);
        if matches!(ch, '.' | '!' | '?') && current.len() > 30 {
            let trimmed = current.trim().to_string();
            // Filter: must be substantial, factual-sounding
            if trimmed.len() >= 40
                && trimmed.len() <= 250
                && !trimmed.contains("may refer to")
                && !trimmed.contains("This article")
                && !trimmed.contains("[citation")
                && !trimmed.contains("disambiguation")
                && !trimmed.starts_with("See also")
                && !trimmed.starts_with("For other")
            {
                sentences.push(trimmed);
            }
            current = String::new();
        }
    }
    sentences
}

#[derive(Serialize)]
struct WikipediaRunProvenance {
    schema_version: &'static str,
    manifest_id: &'static str,
    manifest_version: &'static str,
    manifest_sha256: String,
    source_id: &'static str,
    authority: &'static str,
    egress_policy: &'static str,
    endpoint_template: &'static str,
    success_body_limit_bytes: u64,
    error_body_limit_bytes: u64,
    articles_fetched: usize,
    claims_generated: usize,
    output_path: &'static str,
    output_sha256: String,
}

#[cfg(test)]
mod source_manifest_tests {
    use super::*;

    #[test]
    fn wikipedia_manifest_matches_the_compiled_ingestion_policy() {
        let manifest: serde_json::Value =
            serde_json::from_str(SOURCE_MANIFEST_JSON).expect("source manifest JSON");
        assert_eq!(manifest["manifest_id"].as_str(), Some(SOURCE_MANIFEST_ID));
        assert_eq!(manifest["manifest_version"].as_str(), Some(SOURCE_MANIFEST_VERSION));
        let source = manifest["sources"]
            .as_array()
            .expect("source entries")
            .iter()
            .find(|entry| entry["source_id"] == "wikipedia-en-page-summary")
            .expect("Wikipedia source entry");
        assert_eq!(source["authority"].as_str(), Some("WikipediaSummary"));
        assert_eq!(
            source["endpoint"].as_str(),
            Some("https://en.wikipedia.org/api/rest_v1/page/summary/{title-segment}")
        );
        assert_eq!(
            source["success_body_limit_bytes"].as_u64(),
            Some(MAX_WIKIPEDIA_RESPONSE_BYTES)
        );
        assert_eq!(source["error_body_limit_bytes"].as_u64(), Some(64 * 1024));
        assert_eq!(source["redirects"].as_str(), Some("deny"));
        assert_eq!(source["method"].as_str(), Some("GET"));
    }
}

fn main() {
    let mut limits = EgressLimits::default();
    limits.max_response_body_bytes = MAX_WIKIPEDIA_RESPONSE_BYTES as usize;
    limits.request_timeout = Duration::from_secs(10);
    let client = PublicEgressClient::new(limits);
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("Tokio runtime");

    let mut all_claims: Vec<JsonClaim> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    let mut fetched = 0;
    let total_articles: usize = TOPICS.iter().map(|(_, articles, _)| articles.len()).sum();

    println!(
        "Fetching {} Wikipedia articles across {} domains...",
        total_articles,
        TOPICS.len()
    );

    for (domain, articles, level) in TOPICS {
        for title in *articles {
            let mut url =
                Url::parse("https://en.wikipedia.org/api/rest_v1/page/summary/").unwrap();
            url.path_segments_mut()
                .expect("Wikipedia summary URL is hierarchical")
                .push(title);

            match runtime.block_on(client.request_with_authority(
                EgressAuthority::WikipediaSummary,
                Method::GET,
                url.as_str(),
                HeaderMap::new(),
                Vec::new(),
            )) {
                Ok(response) => {
                    let receipt = response.receipt;
                    eprintln!(
                        "  EGRESS authority={} origin={} peer={} status={} bytes={}/{} elapsed_ms={}",
                        receipt.authority,
                        receipt.origin,
                        receipt.remote_peer,
                        receipt.status,
                        receipt.response_body_bytes,
                        receipt.response_body_limit_bytes,
                        receipt.elapsed.as_millis(),
                    );
                    if !response.status.is_success() {
                        eprintln!("  SKIP {}: HTTP {}", title, response.status);
                        continue;
                    }

                    match serde_json::from_slice::<WikiSummary>(&response.body) {
                        Ok(summary) => {
                            let extract = summary.extract.unwrap_or_default();
                            let article_title =
                                summary.title.unwrap_or_else(|| title.replace('_', " "));

                            let sentences = extract_sentences(&extract);
                            for sentence in sentences.iter().take(3) {
                                let key: String =
                                    sentence.chars().take(80).collect::<String>().to_lowercase();
                                if seen.insert(key) {
                                    all_claims.push(JsonClaim {
                                        claim: sentence.clone(),
                                        level: level.to_string(),
                                        sources: vec![format!("Wikipedia: {}", article_title)],
                                        tags: vec![domain.to_string()],
                                    });
                                }
                            }
                            fetched += 1;
                            if fetched % 20 == 0 {
                                println!(
                                    "  {} / {} articles fetched ({} claims)",
                                    fetched,
                                    total_articles,
                                    all_claims.len()
                                );
                            }
                        }
                        Err(error) => eprintln!("  SKIP {}: parse error: {}", title, error),
                    }
                }
                Err(error) => eprintln!("  SKIP {}: egress request failed: {}", title, error),
            }

            // Rate limit: 100ms between requests (Wikipedia API courtesy)
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }

    println!(
        "\nExtracted {} claims from {} articles",
        all_claims.len(),
        fetched
    );

    let output_path = "prism-ingest/claims/wikipedia-extracts.json";
    let json = serde_json::to_string_pretty(&all_claims).expect("JSON serialization");
    std::fs::write(output_path, &json).expect("Failed to write output");

    let provenance = WikipediaRunProvenance {
        schema_version: "1.0.0",
        manifest_id: SOURCE_MANIFEST_ID,
        manifest_version: SOURCE_MANIFEST_VERSION,
        manifest_sha256: hex::encode(Sha256::digest(SOURCE_MANIFEST_JSON.as_bytes())),
        source_id: "wikipedia-en-page-summary",
        authority: "WikipediaSummary",
        egress_policy: "prism-public-egress-v1",
        endpoint_template: "https://en.wikipedia.org/api/rest_v1/page/summary/{title-segment}",
        success_body_limit_bytes: MAX_WIKIPEDIA_RESPONSE_BYTES,
        error_body_limit_bytes: 64 * 1024,
        articles_fetched: fetched,
        claims_generated: all_claims.len(),
        output_path,
        output_sha256: hex::encode(Sha256::digest(json.as_bytes())),
    };
    let provenance_json =
        serde_json::to_string_pretty(&provenance).expect("provenance serialization");
    let provenance_path = "prism-ingest/claims/wikipedia-extracts.provenance.json";
    std::fs::write(provenance_path, provenance_json).expect("Failed to write provenance");
    println!("Written to {} with provenance {}", output_path, provenance_path);
}
