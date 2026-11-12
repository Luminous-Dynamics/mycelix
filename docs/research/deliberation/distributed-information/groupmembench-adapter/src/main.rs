use html_escape::decode_html_entities;
use serde_json::{Map, Value};
use sha1::{Digest, Sha1};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    fs::{self, File},
    io::{BufRead, BufReader, Read},
    path::{Component, Path, PathBuf},
    process::{Command, ExitCode},
};

const UPSTREAM_REPO: &str = "https://github.com/UCSB-NLP-Chang/GroupMemBench";
const UPSTREAM_COMMIT: &str = "461f00c3e8a1a4a3a2ed5814b729014502c4702f";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FileIdentity {
    pub path: String,
    pub git_blob_sha: String,
    pub size_bytes: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct QuestionRecord {
    pub candidate_view: Value,
    pub evaluator_answer: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IntakeReport {
    pub verified_files: usize,
    pub conversation_files: usize,
    pub conversation_bytes: u64,
    pub question_files: usize,
    pub question_bytes: u64,
}

fn strv<'a>(v: &'a Value, key: &str) -> Option<&'a str> {
    v.get(key)?.as_str().filter(|s| !s.trim().is_empty())
}

fn u64v(v: &Value, key: &str) -> Option<u64> {
    v.get(key)?.as_u64()
}

pub fn git_blob_sha1_bytes(bytes: &[u8]) -> String {
    let mut hasher = Sha1::new();
    hasher.update(format!("blob {}\0", bytes.len()).as_bytes());
    hasher.update(bytes);
    format!("{:x}", hasher.finalize())
}

pub fn git_blob_sha1_file(path: &Path) -> Result<(String, u64), String> {
    let mut file = File::open(path).map_err(|e| format!("open {}: {e}", path.display()))?;
    let size = file.metadata().map_err(|e| format!("stat {}: {e}", path.display()))?.len();
    let mut hasher = Sha1::new();
    hasher.update(format!("blob {size}\0").as_bytes());
    let mut buffer = [0u8; 1024 * 1024];
    loop {
        let n = file.read(&mut buffer).map_err(|e| format!("read {}: {e}", path.display()))?;
        if n == 0 { break; }
        hasher.update(&buffer[..n]);
    }
    Ok((format!("{:x}", hasher.finalize()), size))
}

fn safe_relative_path(raw: &str) -> Result<PathBuf, String> {
    let path = Path::new(raw);
    if raw.is_empty() || path.is_absolute() {
        return Err(format!("unsafe absolute/empty manifest path: {raw}"));
    }
    for component in path.components() {
        if !matches!(component, Component::Normal(_)) {
            return Err(format!("unsafe manifest path component: {raw}"));
        }
    }
    Ok(path.to_path_buf())
}

pub fn parse_manifest(v: &Value) -> Result<Vec<FileIdentity>, String> {
    if strv(v, "intake_id") != Some("GROUPMEMBENCH-METADATA-INTAKE-001")
        || strv(v, "version") != Some("0.1.0")
        || strv(v, "status") != Some("metadata_only_read_only_adapter_candidate") {
        return Err("GroupMemBench intake identity/status mismatch".into());
    }
    let source = &v["source"];
    if strv(source, "repository") != Some(UPSTREAM_REPO)
        || strv(source, "pinned_commit") != Some(UPSTREAM_COMMIT)
        || strv(source, "repository_code_license") != Some("MIT")
        || strv(source, "dataset_license_declared_in_readme") != Some("CC BY 4.0") {
        return Err("GroupMemBench repository/commit/license declaration mismatch".into());
    }
    if v["custody"]["payload_downloaded"].as_bool() != Some(false)
        || v["custody"]["payload_committed"].as_bool() != Some(false)
        || v["custody"]["adapter_executed"].as_bool() != Some(false) {
        return Err("metadata intake manifest claims an external payload operation".into());
    }

    let files = v["files"].as_array().ok_or("manifest files array missing")?;
    if files.len() != 30 {
        return Err(format!("expected 30 pinned file records, found {}", files.len()));
    }
    let mut identities = Vec::with_capacity(files.len());
    let mut paths = BTreeSet::new();
    let mut path_shas = BTreeMap::new();
    let mut conversations = 0usize;
    let mut conversation_bytes = 0u64;
    let mut questions = 0usize;
    let mut question_bytes = 0u64;
    let mut metadata = 0usize;

    for file in files {
        let path = strv(file, "path").ok_or("manifest file path missing")?.to_owned();
        safe_relative_path(&path)?;
        let sha = strv(file, "git_blob_sha").ok_or(format!("Git blob SHA missing for {path}"))?;
        if sha.len() != 40 || !sha.bytes().all(|b| b.is_ascii_hexdigit()) {
            return Err(format!("invalid Git blob SHA for {path}"));
        }
        let size = u64v(file, "size_bytes").ok_or(format!("file size missing for {path}"))?;
        if !paths.insert(path.clone()) {
            return Err(format!("duplicate manifest file path: {path}"));
        }
        path_shas.insert(path.clone(), sha.to_owned());
        if path.starts_with("data/final/") && path.ends_with(".json") {
            conversations += 1;
            conversation_bytes += size;
        } else if path.starts_with("questions/") && path.ends_with(".jsonl") {
            questions += 1;
            question_bytes += size;
        } else if path == "LICENSE" || path == "README.md" {
            metadata += 1;
        } else {
            return Err(format!("unexpected upstream manifest path: {path}"));
        }
        identities.push(FileIdentity { path, git_blob_sha: sha.to_owned(), size_bytes: size });
    }
    if conversations != 4 || questions != 24 || metadata != 2 {
        return Err(format!("unexpected file classes: conversations={conversations}, questions={questions}, metadata={metadata}"));
    }
    if path_shas.get("README.md").map(String::as_str) != strv(source, "readme_git_blob_sha")
        || path_shas.get("LICENSE").map(String::as_str) != strv(source, "license_git_blob_sha") {
        return Err("pinned README/LICENSE Git blob identities do not match rights metadata".into());
    }
    let benchmark = &v["benchmark"];
    let domains = ["Finance", "Technology", "Healthcare", "Manufacturing"];
    let question_types = ["multi_hop", "knowledge_update", "temporal", "user_implicit", "term_ambiguity", "abstention"];
    let declared_domains = benchmark["domains"].as_array().ok_or("benchmark domain list missing")?;
    if declared_domains.len() != domains.len()
        || !domains.iter().all(|d| declared_domains.iter().any(|x| x.as_str() == Some(d))) {
        return Err("GroupMemBench domain matrix mismatch".into());
    }
    let declared_types = benchmark["question_types"].as_array().ok_or("benchmark question type list missing")?;
    if declared_types.len() != question_types.len()
        || !question_types.iter().all(|q| declared_types.iter().any(|x| x.as_str() == Some(q))) {
        return Err("GroupMemBench question type matrix mismatch".into());
    }
    let counts = benchmark["question_counts_by_domain"].as_object().ok_or("question counts by domain missing")?;
    if counts.len() != domains.len() {
        return Err("GroupMemBench question count domain set mismatch".into());
    }
    let mut expected_question_paths = BTreeSet::new();
    let mut question_total = 0u64;
    let mut expected_conversation_paths = BTreeSet::new();
    for domain in domains {
        expected_conversation_paths.insert(format!("data/final/{domain}/synthetic_domain_channels_rolevariants_{domain}.json"));
        let row = counts.get(domain).ok_or(format!("question count row missing for {domain}"))?;
        let mut subtotal = 0u64;
        for qtype in question_types {
            let count = row[qtype].as_u64().ok_or(format!("missing question count: {domain}/{qtype}"))?;
            subtotal += count;
            expected_question_paths.insert(format!("questions/{domain}/{qtype}.jsonl"));
        }
        if row["total"].as_u64() != Some(subtotal) {
            return Err(format!("question count subtotal mismatch for {domain}"));
        }
        question_total += subtotal;
    }
    let actual_question_paths: BTreeSet<String> = paths.iter()
        .filter(|p| p.starts_with("questions/")).cloned().collect();
    let actual_conversation_paths: BTreeSet<String> = paths.iter()
        .filter(|p| p.starts_with("data/final/")).cloned().collect();
    if actual_question_paths != expected_question_paths
        || actual_conversation_paths != expected_conversation_paths
        || question_total != 715
        || benchmark["total_questions"].as_u64() != Some(715) {
        return Err("GroupMemBench domain/question inventory is not the pinned 4 x 6 / 715-question matrix".into());
    }
    let summary = &v["artifact_summary"];
    if u64v(summary, "total_selected_files") != Some(30)
        || u64v(summary, "conversation_data_files") != Some(4)
        || u64v(summary, "conversation_data_bytes") != Some(conversation_bytes)
        || u64v(summary, "question_files") != Some(24)
        || u64v(summary, "question_bytes") != Some(question_bytes)
        || u64v(summary, "metadata_files") != Some(2)
        || summary["git_tree_truncated"].as_bool() != Some(false) {
        return Err("manifest artifact summary does not match pinned file identities".into());
    }
    Ok(identities)
}

pub fn verify_local_checkout(manifest: &Value, checkout: &Path) -> Result<IntakeReport, String> {
    let identities = parse_manifest(manifest)?;
    let root = checkout.canonicalize().map_err(|e| format!("canonicalize checkout: {e}"))?;
    let output = Command::new("git")
        .arg("-C").arg(&root)
        .args(["rev-parse", "HEAD"])
        .output().map_err(|e| format!("run git rev-parse: {e}"))?;
    if !output.status.success() {
        return Err(format!("git rev-parse failed: {}", String::from_utf8_lossy(&output.stderr)));
    }
    let head = String::from_utf8_lossy(&output.stdout).trim().to_owned();
    if head != UPSTREAM_COMMIT {
        return Err(format!("upstream checkout HEAD mismatch: expected {UPSTREAM_COMMIT}, found {head}"));
    }

    let mut report = IntakeReport::default();
    for identity in &identities {
        let relative = safe_relative_path(&identity.path)?;
        let candidate = root.join(relative);
        let canonical = candidate.canonicalize().map_err(|e| format!("resolve {}: {e}", identity.path))?;
        if !canonical.starts_with(&root) {
            return Err(format!("manifest path escapes checkout root: {}", identity.path));
        }
        let (sha, size) = git_blob_sha1_file(&canonical)?;
        if sha != identity.git_blob_sha {
            return Err(format!("Git blob SHA mismatch: {}", identity.path));
        }
        if size != identity.size_bytes {
            return Err(format!("byte-size mismatch: {}", identity.path));
        }
        report.verified_files += 1;
        if identity.path.starts_with("data/final/") {
            report.conversation_files += 1;
            report.conversation_bytes += size;
        } else if identity.path.starts_with("questions/") {
            report.question_files += 1;
            report.question_bytes += size;
        }
    }
    Ok(report)
}

fn required_string<'a>(v: &'a Value, key: &str, context: &str) -> Result<&'a str, String> {
    strv(v, key).ok_or_else(|| format!("{context}: required string '{key}' missing"))
}

pub fn project_candidate_message(message: &Value, channel: &str) -> Result<Value, String> {
    required_string(message, "msg_node", "message")?;
    required_string(message, "content", "message")?;
    required_string(message, "author", "message")?;
    required_string(message, "role", "message")?;
    required_string(message, "timestamp", "message")?;

    let mut candidate = Map::new();
    for key in ["msg_node", "content", "author", "role", "timestamp", "phase_name", "topic", "reply_to"] {
        if let Some(value) = message.get(key) {
            candidate.insert(key.to_owned(), value.clone());
        }
    }
    candidate.insert("channel".into(), Value::String(channel.to_owned()));
    Ok(Value::Object(candidate))
}

pub fn project_conversation(payload: &Value) -> Result<Vec<Value>, String> {
    let channels = payload.as_object().ok_or("conversation root must be a channel object")?;
    let mut channel_names: Vec<&String> = channels.keys().collect();
    channel_names.sort();
    let mut rows: Vec<(String, String, String, Value)> = Vec::new();
    let mut message_ids = BTreeSet::new();

    for channel in channel_names {
        let messages = channels[channel].as_array().ok_or(format!("channel '{channel}' must contain a message array"))?;
        for message in messages {
            let projected = project_candidate_message(message, channel)?;
            let node = required_string(&projected, "msg_node", "projected message")?.to_owned();
            if !message_ids.insert((channel.clone(), node.clone())) {
                return Err(format!("duplicate msg_node '{node}' in channel '{channel}'"));
            }
            let timestamp = required_string(&projected, "timestamp", "projected message")?.to_owned();
            rows.push((channel.clone(), timestamp, node, projected));
        }
    }

    // Match the upstream RAG baseline's deterministic channel/timestamp/msg_node ordering.
    rows.sort_by(|a, b| (&a.0, &a.1, &a.2).cmp(&(&b.0, &b.1, &b.2)));
    Ok(rows.into_iter().map(|row| row.3).collect())
}

pub fn project_question_line(line: &str) -> Result<QuestionRecord, String> {
    let raw: Value = serde_json::from_str(line).map_err(|e| format!("question JSONL parse: {e}"))?;
    let id = required_string(&raw, "id", "question")?;
    let question = required_string(&raw, "question", id)?;
    let asking_user = required_string(&raw, "asking_user_id", id)?;
    let answer = required_string(&raw, "answer", id)?;

    let mut candidate = Map::new();
    candidate.insert("id".into(), Value::String(id.to_owned()));
    candidate.insert("question".into(), Value::String(question.to_owned()));
    candidate.insert("asking_user_id".into(), Value::String(asking_user.to_owned()));
    Ok(QuestionRecord {
        candidate_view: Value::Object(candidate),
        evaluator_answer: answer.to_owned(),
    })
}

pub fn read_question_jsonl(path: &Path) -> Result<Vec<QuestionRecord>, String> {
    let file = File::open(path).map_err(|e| format!("open questions {}: {e}", path.display()))?;
    let mut records = Vec::new();
    let mut ids = BTreeSet::new();
    for (index, line) in BufReader::new(file).lines().enumerate() {
        let line = line.map_err(|e| format!("read question line {}: {e}", index + 1))?;
        if line.trim().is_empty() { continue; }
        let record = project_question_line(&line)?;
        let id = record.candidate_view["id"].as_str().unwrap_or("");
        if !ids.insert(id.to_owned()) {
            return Err(format!("duplicate question ID: {id}"));
        }
        records.push(record);
    }
    Ok(records)
}


pub fn tokenize_baseline(text: &str) -> Vec<String> {
    let mut tokens = Vec::new();
    let mut current = Vec::new();
    for byte in text.bytes() {
        if byte.is_ascii_alphanumeric() || byte == b'_' {
            current.push(byte.to_ascii_lowercase());
        } else if !current.is_empty() {
            tokens.push(String::from_utf8(std::mem::take(&mut current)).expect("ASCII token"));
        }
    }
    if !current.is_empty() {
        tokens.push(String::from_utf8(current).expect("ASCII token"));
    }
    tokens
}

pub fn message_index_text(message: &Value) -> String {
    message.get("content")
        .and_then(Value::as_str)
        .map(|content| decode_html_entities(content).trim().to_owned())
        .unwrap_or_default()
}

pub fn format_retrieved_message(message: &Value) -> String {
    let author = message.get("author").and_then(Value::as_str).unwrap_or("?");
    let mut tags = vec![format!("user={author}")];
    for (source, label) in [
        ("role", "speaker_role"),
        ("channel", "channel"),
        ("phase_name", "phase_name"),
        ("topic", "topic"),
        ("timestamp", "timestamp"),
        ("reply_to", "reply_to"),
        ("msg_node", "msg_node"),
    ] {
        if let Some(value) = message.get(source) {
            if !value.is_null() && value != "" {
                let rendered = value.as_str().map(str::to_owned).unwrap_or_else(|| value.to_string());
                tags.push(format!("{label}={rendered}"));
            }
        }
    }
    let body = message.get("content").and_then(Value::as_str)
        .map(|content| decode_html_entities(content).trim().to_owned())
        .unwrap_or_default();
    format!("[{}]\n{}", tags.join(" / "), body).trim().to_owned()
}

#[derive(Clone, Debug)]
pub struct Bm25Index {
    lengths: Vec<usize>,
    idf: BTreeMap<String, f64>,
    term_frequencies: Vec<BTreeMap<String, usize>>,
    average_length: f64,
    k1: f64,
    b: f64,
}

impl Bm25Index {
    pub fn from_messages(messages: &[Value]) -> Self {
        let documents: Vec<Vec<String>> = messages.iter()
            .map(|message| tokenize_baseline(&message_index_text(message)))
            .collect();
        Self::from_tokenized_documents(documents)
    }

    pub fn from_tokenized_documents(documents: Vec<Vec<String>>) -> Self {
        let lengths: Vec<usize> = documents.iter().map(Vec::len).collect();
        let count = documents.len();
        let average_length = if count == 0 {
            0.0
        } else {
            lengths.iter().sum::<usize>() as f64 / count as f64
        };

        // Build per-document term frequencies and corpus document frequency once.
        // Rebuilding these maps for each query is prohibitively expensive on the full corpus.
        let mut term_frequencies = Vec::with_capacity(documents.len());
        let mut document_frequency: BTreeMap<String, usize> = BTreeMap::new();
        for document in &documents {
            let mut frequencies: BTreeMap<String, usize> = BTreeMap::new();
            for term in document {
                *frequencies.entry(term.clone()).or_default() += 1;
            }
            for term in frequencies.keys() {
                *document_frequency.entry(term.clone()).or_default() += 1;
            }
            term_frequencies.push(frequencies);
        }

        let mut raw_idf = BTreeMap::new();
        let mut idf_sum = 0.0;
        for (term, frequency) in document_frequency {
            let n = count as f64;
            let df = frequency as f64;
            let value = ((n - df + 0.5) / (df + 0.5)).ln();
            idf_sum += value;
            raw_idf.insert(term, value);
        }
        let average_idf = if raw_idf.is_empty() { 0.0 } else { idf_sum / raw_idf.len() as f64 };
        let epsilon = 0.25;
        let idf = raw_idf.into_iter().map(|(term, value)| {
            (term, if value < 0.0 { epsilon * average_idf } else { value })
        }).collect();

        Self { lengths, idf, term_frequencies, average_length, k1: 1.5, b: 0.75 }
    }

    pub fn retrieve(&self, query: &str, top_k: usize) -> Vec<usize> {
        let query_terms = tokenize_baseline(query);
        if self.lengths.is_empty() || top_k == 0 {
            return Vec::new();
        }
        // rank_bm25 returns the earliest documents for an empty query because
        // every document receives the same zero score and Python sorting is stable.
        if query_terms.is_empty() {
            return (0..self.lengths.len().min(top_k)).collect();
        }

        let mut scored = Vec::with_capacity(self.lengths.len());
        for index in 0..self.lengths.len() {
            let mut score = 0.0;
            for term in &query_terms {
                if let (Some(idf), Some(freq)) = (self.idf.get(term), self.term_frequencies[index].get(term)) {
                    let denominator = *freq as f64
                        + self.k1 * (1.0 - self.b
                            + self.b * self.lengths[index] as f64 / self.average_length.max(1.0));
                    score += *idf * (*freq as f64 * (self.k1 + 1.0)) / denominator;
                }
            }
            scored.push((index, score));
        }
        // Python's sorted(..., reverse=True) is stable; preserve input index on ties.
        scored.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        scored.into_iter().take(top_k).map(|(index, _)| index).collect()
    }

    pub fn document_count(&self) -> usize {
        self.lengths.len()
    }

    pub fn vocabulary_size(&self) -> usize {
        self.idf.len()
    }

    pub fn has_lexical_match(&self, query: &str) -> bool {
        tokenize_baseline(query).iter().any(|term| self.idf.contains_key(term))
    }
}

pub fn candidate_message_allowed_fields() -> &'static [&'static str] {
    &["msg_node", "content", "author", "role", "timestamp", "phase_name", "topic", "reply_to", "channel"]
}

fn run_bm25_probe(checkout: &Path) -> Result<(), String> {
    let domains = ["Finance", "Technology", "Healthcare", "Manufacturing"];
    let question_types = ["multi_hop", "knowledge_update", "temporal", "user_implicit", "term_ambiguity", "abstention"];
    let mut total_messages = 0usize;
    let mut total_questions = 0usize;
    let mut total_lexical_overlap = 0usize;
    for domain in domains {
        let conversation_path = checkout.join(format!("data/final/{domain}/synthetic_domain_channels_rolevariants_{domain}.json"));
        let conversation_file = File::open(&conversation_path)
            .map_err(|e| format!("open conversation {}: {e}", conversation_path.display()))?;
        let conversation: Value = serde_json::from_reader(BufReader::new(conversation_file))
            .map_err(|e| format!("parse conversation {}: {e}", conversation_path.display()))?;
        let messages = project_conversation(&conversation)?;
        let index = Bm25Index::from_messages(&messages);
        let mut question_count = 0usize;
        let mut lexical_overlap = 0usize;
        let mut top_k_returned = 0usize;
        for qtype in question_types {
            let question_path = checkout.join(format!("questions/{domain}/{qtype}.jsonl"));
            let records = read_question_jsonl(&question_path)?;
            for record in records {
                let query = record.candidate_view["question"].as_str().unwrap_or("");
                if index.has_lexical_match(query) {
                    lexical_overlap += 1;
                }
                let top = index.retrieve(query, 10);
                top_k_returned += top.len();
                question_count += 1;
                // Deliberately do not read/use evaluator_answer here.
            }
        }
        println!(
            "BM25_PROCESS_BASELINE: domain={} messages={} vocabulary={} questions={} questions_with_lexical_overlap={} top_k=10 retrieved_passages={} answer_scored=false",
            domain, messages.len(), index.vocabulary_size(), question_count, lexical_overlap, top_k_returned
        );
        total_messages += messages.len();
        total_questions += question_count;
        total_lexical_overlap += lexical_overlap;
    }
    println!(
        "BM25_PROCESS_BASELINE_TOTAL: messages={} questions={} questions_with_lexical_overlap={} lexical_overlap_rate={:.6} top_k=10 answer_scored=false model_executed=false",
        total_messages, total_questions, total_lexical_overlap,
        if total_questions == 0 { 0.0 } else { total_lexical_overlap as f64 / total_questions as f64 }
    );
    Ok(())
}

fn run(args: &[String]) -> Result<(), String> {
    if args.len() != 2 && args.len() != 3 {
        return Err("usage: groupmembench-readonly-adapter GROUPMEMBENCH_METADATA_INTAKE.json /path/to/pinned/checkout [--bm25-probe]".into());
    }
    let manifest: Value = serde_json::from_slice(
        &fs::read(&args[0]).map_err(|e| format!("read manifest: {e}"))?
    ).map_err(|e| format!("parse manifest JSON: {e}"))?;
    let report = verify_local_checkout(&manifest, Path::new(&args[1]))?;
    println!(
        "READ_ONLY_INTAKE_VERIFIED: files={} conversation_files={} conversation_bytes={} question_files={} question_bytes={} payload_copied=false model_executed=false",
        report.verified_files, report.conversation_files, report.conversation_bytes,
        report.question_files, report.question_bytes
    );
    if args.len() == 3 {
        if args[2] != "--bm25-probe" {
            return Err(format!("unsupported adapter option: {}", args[2]));
        }
        run_bm25_probe(Path::new(&args[1]))?;
    }
    Ok(())
}

fn main() -> ExitCode {
    match run(&env::args().skip(1).collect::<Vec<_>>()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("NOT_VERIFIED: {error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_git_blob_sha_matches_git_known_answer() {
        assert_eq!(git_blob_sha1_bytes(b""), "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391");
    }

    #[test]
    fn candidate_projection_drops_evaluator_flags_and_unregistered_metadata() {
        let message = serde_json::json!({
            "msg_node":"Msg_1","content":"A decision update","author":"User_2","role":"Engineer",
            "timestamp":"2026-05-01T10:00:00Z","phase_name":"approval","topic":"release",
            "reply_to":null,"is_noise":true,"is_decision_point":true,"hidden_answer":"leak",
            "expertise_score":0.98
        });
        let projected = project_candidate_message(&message, "general").unwrap();
        assert_eq!(projected["channel"], "general");
        assert_eq!(projected["content"], "A decision update");
        for hidden in ["is_noise","is_decision_point","hidden_answer","expertise_score"] {
            assert!(projected.get(hidden).is_none());
        }
    }

    #[test]
    fn question_answer_is_separate_from_candidate_view() {
        let q = project_question_line(r#"{"id":"q1","question":"What changed?","answer":"The threshold changed.","asking_user_id":"User_7","evaluator_label":"gold"}"#).unwrap();
        assert_eq!(q.candidate_view["question"], "What changed?");
        assert_eq!(q.candidate_view["asking_user_id"], "User_7");
        assert!(q.candidate_view.get("answer").is_none());
        assert!(q.candidate_view.get("evaluator_label").is_none());
        assert_eq!(q.evaluator_answer, "The threshold changed.");
    }

    #[test]
    fn message_projection_preserves_baseline_ordering_and_metadata() {
        let payload = serde_json::json!({
            "z-channel":[
                {"msg_node":"Msg_2","content":"later","author":"User_2","role":"Ops","timestamp":"2026-05-01T11:00:00Z","phase_name":"review","topic":"cutover"},
                {"msg_node":"Msg_1","content":"earlier","author":"User_1","role":"Ops","timestamp":"2026-05-01T10:00:00Z","phase_name":"review","topic":"cutover"}
            ],
            "a-channel":[
                {"msg_node":"Msg_3","content":"first channel","author":"User_3","role":"Lead","timestamp":"2026-05-01T09:00:00Z","phase_name":"planning","topic":"staffing"}
            ]
        });
        let projected = project_conversation(&payload).unwrap();
        assert_eq!(projected[0]["channel"], "a-channel");
        assert_eq!(projected[1]["content"], "earlier");
        assert_eq!(projected[2]["content"], "later");
        assert_eq!(projected[1]["phase_name"], "review");
    }

    #[test]
    fn upstream_ascii_tokenizer_is_case_insensitive_and_parameter_free() {
        assert_eq!(tokenize_baseline("Foo foo_bar 123 Caf\u{00e9}"), vec!["foo","foo_bar","123","caf"]);
    }

    #[test]
    fn bm25_indexes_content_only_not_speaker_metadata() {
        let a = serde_json::json!({"content":"payment threshold risk","role":"Compliance"});
        let b = serde_json::json!({"content":"payment threshold risk","role":"Engineer","is_noise":true});
        let index = Bm25Index::from_messages(&[a,b]);
        assert_eq!(index.document_count(), 2);
        assert_eq!(index.retrieve("payment", 2), vec![0,1]);
    }

    #[test]
    fn bm25_retrieves_matching_content_and_returns_empty_for_empty_query() {
        let messages = vec![
            serde_json::json!({"content":"payment risk threshold"}),
            serde_json::json!({"content":"office seating plan"}),
            serde_json::json!({"content":"incident response payment"}),
        ];
        let index = Bm25Index::from_messages(&messages);
        assert_eq!(index.retrieve("payment risk", 1), vec![0]);
        assert!(index.retrieve("   ", 10).is_empty());
    }

    #[test]
    fn bm25_exposes_vocabulary_and_lexical_overlap_without_using_answers() {
        let index = Bm25Index::from_tokenized_documents(vec![
            vec!["alpha".into(), "beta".into()],
            vec!["gamma".into()]
        ]);
        assert_eq!(index.vocabulary_size(), 3);
        assert!(index.has_lexical_match("ALPHA?"));
        assert!(!index.has_lexical_match("omega"));
    }

    #[test]
    fn question_answer_does_not_affect_candidate_retrieval_query() {
        let first = project_question_line(r#"{"id":"q1","question":"What changed?","answer":"secret answer","asking_user_id":"User_1"}"#).unwrap();
        let second = project_question_line(r#"{"id":"q1","question":"What changed?","answer":"different secret","asking_user_id":"User_1"}"#).unwrap();
        assert_eq!(first.candidate_view, second.candidate_view);
    }

    #[test]
    fn empty_token_query_matches_upstream_stable_zero_score_top_k() {
        let index = Bm25Index::from_tokenized_documents(vec![
            vec!["alpha".into()], vec!["beta".into()], vec!["gamma".into()]
        ]);
        assert_eq!(index.retrieve(" !!! ", 2), vec![0,1]);
    }

    #[test]
    fn bm25_ties_preserve_original_message_order() {
        let index = Bm25Index::from_tokenized_documents(vec![
            vec!["alpha".into()], vec!["alpha".into()], vec!["beta".into()]
        ]);
        assert_eq!(index.retrieve("alpha", 3), vec![0,1,2]);
    }

    #[test]
    fn html_entities_are_decoded_before_indexing_and_display() {
        let message = serde_json::json!({"content":"Risk &amp; payment"});
        assert_eq!(message_index_text(&message), "Risk & payment");
        assert!(format_retrieved_message(&serde_json::json!({"author":"User_1","role":"Ops","channel":"risk","content":"A &amp; B"})).contains("A & B"));
    }

    #[test]
    fn duplicate_message_id_in_a_channel_is_rejected() {
        let one = serde_json::json!({"msg_node":"Msg_1","content":"one","author":"User_1","role":"Ops","timestamp":"2026-05-01T10:00:00Z"});
        let two = serde_json::json!({"msg_node":"Msg_1","content":"two","author":"User_2","role":"Ops","timestamp":"2026-05-01T11:00:00Z"});
        let payload = serde_json::json!({"ops":[one,two]});
        assert!(project_conversation(&payload).is_err());
    }

    #[test]
    fn manifest_rejects_path_traversal() {
        assert!(safe_relative_path("../outside").is_err());
        assert!(safe_relative_path("/etc/passwd").is_err());
        assert!(safe_relative_path("data/final/Finance/file.json").is_ok());
    }

    #[test]
    fn pinned_metadata_manifest_is_accepted_as_read_only_intake() {
        let manifest: Value = serde_json::from_str(include_str!("../../GROUPMEMBENCH_METADATA_INTAKE_V0_1.json")).unwrap();
        let files = parse_manifest(&manifest).unwrap();
        assert_eq!(files.len(), 30);
        assert_eq!(files.iter().filter(|x| x.path.starts_with("data/final/")).count(), 4);
        assert_eq!(files.iter().filter(|x| x.path.starts_with("questions/")).count(), 24);
    }

    #[test]
    fn manifest_readme_and_license_hashes_are_bound_to_the_file_list() {
        let mut manifest: Value = serde_json::from_str(include_str!("../../GROUPMEMBENCH_METADATA_INTAKE_V0_1.json")).unwrap();
        let files = manifest["files"].as_array_mut().unwrap();
        let readme = files.iter_mut().find(|x| x["path"] == "README.md").unwrap();
        readme["git_blob_sha"] = Value::String("0000000000000000000000000000000000000000".into());
        assert!(parse_manifest(&manifest).is_err());
    }

    #[test]
    fn manifest_question_matrix_must_match_exact_domains_and_types() {
        let mut manifest: Value = serde_json::from_str(include_str!("../../GROUPMEMBENCH_METADATA_INTAKE_V0_1.json")).unwrap();
        manifest["benchmark"]["question_types"][0] = Value::String("unsupported_type".into());
        assert!(parse_manifest(&manifest).is_err());
    }
}
