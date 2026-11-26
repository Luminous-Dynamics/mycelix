//! Deterministic conventional retrieval baselines for MCSF.
//!
//! This module implements NoMemory and BM25 in dependency-light Rust.
//! Dense-vector and HDC/CfC systems must implement the same adapter contract
//! around their real engines; this module does not simulate those systems.

use std::collections::{BTreeMap, BTreeSet};

use sha2::{Digest, Sha256};

use super::McsfArchitecture;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfMemoryRecord {
    pub memory_id: String,
    pub content: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct McsfRetrievalHit {
    pub memory_id: String,
    pub content_digest: String,
    /// BM25 score × 1,000,000, rounded to the nearest integer.
    pub score_micros: i64,
}

pub trait McsfRetrievalAdapter {
    fn architecture(&self) -> McsfArchitecture;

    fn search(
        &self,
        records: &[McsfMemoryRecord],
        query: &str,
        top_k: usize,
    ) -> Result<Vec<McsfRetrievalHit>, String>;
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NoMemoryAdapter;

impl McsfRetrievalAdapter for NoMemoryAdapter {
    fn architecture(&self) -> McsfArchitecture {
        McsfArchitecture::NoMemory
    }

    fn search(
        &self,
        _records: &[McsfMemoryRecord],
        query: &str,
        top_k: usize,
    ) -> Result<Vec<McsfRetrievalHit>, String> {
        if top_k == 0 {
            return Err("retrieval Top K must be greater than zero".into());
        }
        if query.trim().is_empty() {
            return Err("retrieval query must not be empty".into());
        }
        Ok(Vec::new())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bm25Config {
    pub k1: f64,
    pub b: f64,
}

impl Default for Bm25Config {
    fn default() -> Self {
        Self { k1: 1.2, b: 0.75 }
    }
}

impl Bm25Config {
    pub fn validate(self) -> Result<Self, String> {
        if !self.k1.is_finite() || self.k1 <= 0.0 || self.k1 > 5.0 {
            return Err("BM25 k1 must be finite and within (0, 5]".into());
        }
        if !self.b.is_finite() || !(0.0..=1.0).contains(&self.b) {
            return Err("BM25 b must be finite and within [0, 1]".into());
        }
        Ok(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bm25Adapter {
    config: Bm25Config,
}

impl Default for Bm25Adapter {
    fn default() -> Self {
        Self {
            config: Bm25Config::default(),
        }
    }
}

impl Bm25Adapter {
    pub fn new(config: Bm25Config) -> Result<Self, String> {
        Ok(Self {
            config: config.validate()?,
        })
    }

    pub fn config(&self) -> Bm25Config {
        self.config
    }
}

fn tokenize(value: &str) -> Vec<String> {
    value
        .split(|character: char| !character.is_alphanumeric())
        .filter(|token| !token.is_empty())
        .map(str::to_lowercase)
        .collect()
}

fn sha256_hex(value: &str) -> String {
    let digest = Sha256::digest(value.as_bytes());
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

#[derive(Clone, Debug)]
struct IndexedDocument<'a> {
    record: &'a McsfMemoryRecord,
    tokens: Vec<String>,
    term_frequency: BTreeMap<String, usize>,
}

impl<'a> IndexedDocument<'a> {
    fn new(record: &'a McsfMemoryRecord) -> Self {
        let tokens = tokenize(&record.content);
        let mut term_frequency = BTreeMap::new();
        for token in &tokens {
            *term_frequency.entry(token.clone()).or_insert(0) += 1;
        }
        Self {
            record,
            tokens,
            term_frequency,
        }
    }
}

impl McsfRetrievalAdapter for Bm25Adapter {
    fn architecture(&self) -> McsfArchitecture {
        McsfArchitecture::Bm25Sparse
    }

    fn search(
        &self,
        records: &[McsfMemoryRecord],
        query: &str,
        top_k: usize,
    ) -> Result<Vec<McsfRetrievalHit>, String> {
        if top_k == 0 {
            return Err("retrieval Top K must be greater than zero".into());
        }
        let query_terms = tokenize(query)
            .into_iter()
            .collect::<BTreeSet<_>>();
        if query_terms.is_empty() {
            return Err("retrieval query contains no searchable ASCII terms".into());
        }

        let mut ids = BTreeSet::new();
        let mut documents = Vec::with_capacity(records.len());
        let mut document_frequency = BTreeMap::<String, usize>::new();
        let mut total_document_length = 0usize;

        for record in records {
            if record.memory_id.trim().is_empty()
                || record.memory_id != record.memory_id.trim()
                || record.content.trim().is_empty()
            {
                return Err("memory records require stable non-empty IDs and content".into());
            }
            if !ids.insert(record.memory_id.as_str()) {
                return Err(format!("duplicate memory ID: {}", record.memory_id));
            }
            let document = IndexedDocument::new(record);
            total_document_length =
                total_document_length.saturating_add(document.tokens.len());
            for term in document.term_frequency.keys() {
                *document_frequency.entry(term.clone()).or_insert(0) += 1;
            }
            documents.push(document);
        }

        if documents.is_empty() {
            return Ok(Vec::new());
        }

        let average_length = total_document_length as f64 / documents.len() as f64;
        let n = documents.len() as f64;
        let mut scored = Vec::<McsfRetrievalHit>::new();

        for document in &documents {
            let document_length = document.tokens.len() as f64;
            let mut score = 0.0f64;

            for term in &query_terms {
                let Some(&frequency) = document.term_frequency.get(term) else {
                    continue;
                };
                let frequency = frequency as f64;
                let document_frequency =
                    *document_frequency.get(term).unwrap_or(&0) as f64;
                let inverse_document_frequency =
                    (1.0 + (n - document_frequency + 0.5) / (document_frequency + 0.5))
                        .ln();
                let length_normalization = self.config.k1
                    * (1.0 - self.config.b
                        + self.config.b * document_length / average_length);
                let term_score = inverse_document_frequency
                    * (frequency * (self.config.k1 + 1.0))
                    / (frequency + length_normalization);
                score += term_score;
            }

            if score > 0.0 {
                let scaled = (score * 1_000_000.0).round();
                let score_micros = scaled
                    .clamp(0.0, i64::MAX as f64)
                    .min(i64::MAX as f64) as i64;
                scored.push(McsfRetrievalHit {
                    memory_id: document.record.memory_id.clone(),
                    content_digest: sha256_hex(&document.record.content),
                    score_micros,
                });
            }
        }

        scored.sort_by(|left, right| {
            right
                .score_micros
                .cmp(&left.score_micros)
                .then_with(|| left.memory_id.cmp(&right.memory_id))
        });
        scored.truncate(top_k);
        Ok(scored)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn records() -> Vec<McsfMemoryRecord> {
        vec![
            McsfMemoryRecord {
                memory_id: "memory-003".into(),
                content: "Revocation must reconcile every transitive descendant.".into(),
            },
            McsfMemoryRecord {
                memory_id: "memory-001".into(),
                content: "A privacy boundary prevents unauthorized memory disclosure.".into(),
            },
            McsfMemoryRecord {
                memory_id: "memory-002".into(),
                content: "A memory update creates a new temporal revision.".into(),
            },
            McsfMemoryRecord {
                memory_id: "memory-004".into(),
                content: "Unrelated gardening notes describe soil and rainfall.".into(),
            },
        ]
    }

    #[test]
    fn no_memory_baseline_returns_no_candidates() {
        let adapter = NoMemoryAdapter;
        assert_eq!(adapter.architecture(), McsfArchitecture::NoMemory);
        assert!(
            adapter
                .search(&records(), "revocation descendant", 10)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn bm25_ranks_relevant_records_and_excludes_nonmatching_records() {
        let adapter = Bm25Adapter::default();
        assert_eq!(adapter.architecture(), McsfArchitecture::Bm25Sparse);
        let hits = adapter
            .search(&records(), "revocation descendant", 10)
            .unwrap();
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].memory_id, "memory-003");
        assert!(hits[0].score_micros > 0);
        assert_eq!(hits[0].content_digest, sha256_hex(&records()[0].content));
    }

    #[test]
    fn bm25_respects_top_k_and_is_deterministic() {
        let adapter = Bm25Adapter::default();
        let query = "memory privacy revocation update";
        let first = adapter.search(&records(), query, 2).unwrap();
        let second = adapter.search(&records(), query, 2).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.len(), 2);
    }

    #[test]
    fn bm25_tie_breaks_by_memory_id_independent_of_input_order() {
        let records = vec![
            McsfMemoryRecord {
                memory_id: "zeta".into(),
                content: "privacy boundary memory".into(),
            },
            McsfMemoryRecord {
                memory_id: "alpha".into(),
                content: "privacy boundary memory".into(),
            },
        ];
        let adapter = Bm25Adapter::default();
        let hits = adapter.search(&records, "privacy", 10).unwrap();
        assert_eq!(hits[0].memory_id, "alpha");
        assert_eq!(hits[1].memory_id, "zeta");
    }

    #[test]
    fn bm25_rejects_duplicate_ids_and_empty_content() {
        let adapter = Bm25Adapter::default();
        let mut duplicate = records();
        duplicate.push(duplicate[0].clone());
        assert!(adapter.search(&duplicate, "revocation", 10).is_err());

        let empty = vec![McsfMemoryRecord {
            memory_id: "empty".into(),
            content: "  ".into(),
        }];
        assert!(adapter.search(&empty, "revocation", 10).is_err());
    }

    #[test]
    fn bm25_rejects_invalid_top_k_and_empty_query() {
        let adapter = Bm25Adapter::default();
        assert!(adapter.search(&records(), "privacy", 0).is_err());
        assert!(adapter.search(&records(), "!!!", 10).is_err());
    }

    #[test]
    fn bm25_tokenizer_supports_non_ascii_words() {
        let records = vec![
            McsfMemoryRecord {
                memory_id: "zulu".into(),
                content: "ukuvikelwa kwedatha nokwabelana kolwazi".into(),
            },
            McsfMemoryRecord {
                memory_id: "sesotho".into(),
                content: "tšireletso ea tsebo le boinotši".into(),
            },
        ];
        let adapter = Bm25Adapter::default();
        let zulu = adapter.search(&records, "ukuvikelwa kwedatha", 10).unwrap();
        let sesotho = adapter.search(&records, "tšireletso boinotši", 10).unwrap();
        assert_eq!(zulu.first().map(|hit| hit.memory_id.as_str()), Some("zulu"));
        assert_eq!(sesotho.first().map(|hit| hit.memory_id.as_str()), Some("sesotho"));
    }

    #[test]
    fn bm25_configuration_is_bounded() {
        assert!(Bm25Adapter::new(Bm25Config { k1: 1.2, b: 0.75 }).is_ok());
        assert!(Bm25Adapter::new(Bm25Config { k1: f64::NAN, b: 0.75 }).is_err());
        assert!(Bm25Adapter::new(Bm25Config { k1: 1.2, b: 1.5 }).is_err());
    }

    #[test]
    fn empty_corpus_returns_empty_candidates_after_query_validation() {
        let adapter = Bm25Adapter::default();
        assert!(adapter.search(&[], "memory privacy", 5).unwrap().is_empty());
    }
}
