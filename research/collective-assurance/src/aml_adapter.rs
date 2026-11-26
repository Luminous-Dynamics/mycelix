//! Read-only qualification of Agent Memory Leaderboard Add/Search run evidence.
//!
//! The adapter does not host an API, execute the platform evaluator, generate
//! answers, or claim participation. It validates externally captured requests,
//! responses and release-policy metadata. Official score eligibility is
//! strictly distinct from API compatibility and native, unrestricted runs.

use std::collections::{BTreeMap, BTreeSet};

use super::{QualificationReport, QualificationStatus};

const CYCLE_2_ID: &str = "agent-memory-challenge-cycle-2-2026";
const CYCLE_2_EMBEDDING_MODEL: &str = "text-embedding-v4";
const CYCLE_2_LLM_MODEL: &str = "gpt-4o-mini";
const CYCLE_2_TOP_K: u32 = 100;
const CYCLE_2_MAX_ADD_MESSAGES: u32 = 20;
const CYCLE_2_MAX_ADD_WORDS: u32 = 2_000;
const CYCLE_2_MAX_IMAGE_BYTES: u64 = 10 * 1024 * 1024;
const CYCLE_2_MAX_SEARCH_IMAGE_BYTES: u64 = 30 * 1024 * 1024;
const CYCLE_2_MAX_ADD_ATTEMPTS: usize = 32;

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty() && value == value.trim()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_git_commit(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn retryable_add_status(status: u16) -> bool {
    matches!(status, 408 | 409 | 425 | 429 | 500 | 502 | 503 | 504 | 524)
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum AmlTrack {
    Textual,
    Coding,
    Multimodal,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AmlDivision {
    OpenSourceMethods,
    CommercialProducts,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AmlRunMode {
    Smoke,
    Full,
    NativeUnrestricted,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum AmlComponentKind {
    Embedding,
    LanguageModel,
    Reranker,
    Other,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlComponentIdentity {
    pub kind: AmlComponentKind,
    pub model_id: String,
    pub model_version: String,
    /// Digest of the resolved deployment/configuration, not a secret.
    pub deployment_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlModelPolicy {
    pub required_embedding_model: Option<String>,
    pub required_language_model: Option<String>,
    /// Cycle 2 leaves reranker choice unrestricted.
    pub reranker_unrestricted: bool,
}

impl AmlModelPolicy {
    /// Exact Cycle 2 Open-source Methods model-policy profile.
    pub fn cycle_2_open_source() -> Self {
        Self {
            required_embedding_model: Some(CYCLE_2_EMBEDDING_MODEL.into()),
            required_language_model: Some(CYCLE_2_LLM_MODEL.into()),
            reranker_unrestricted: true,
        }
    }

    /// Exact Cycle 2 Commercial Products model-policy profile.
    pub fn cycle_2_commercial() -> Self {
        Self {
            required_embedding_model: None,
            required_language_model: None,
            reranker_unrestricted: true,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlReleaseContract {
    pub cycle_id: String,
    pub division: AmlDivision,
    pub release_id: String,
    pub release_manifest_digest: String,
    pub evaluation_pipeline_digest: String,
    pub dataset_bundle_digest: String,
    pub question_manifest_digest: String,
    pub answer_model_digest: String,
    pub scoring_contract_digest: String,
    pub api_contract_digest: String,
    pub top_k: u32,
    pub max_add_messages: u32,
    pub max_add_words: u32,
    pub max_add_attempts: usize,
    pub max_image_bytes: u64,
    pub max_search_image_bytes: u64,
    pub model_policy: AmlModelPolicy,
}

impl AmlReleaseContract {
    /// Cycle 2 protocol limits and one of its two division-specific policies.
    /// Official source/dataset/pipeline digests must still be supplied from
    /// the exact official release manifest; this constructor does not invent them.
    pub fn cycle_2(division: AmlDivision) -> Self {
        let model_policy = match division {
            AmlDivision::OpenSourceMethods => AmlModelPolicy::cycle_2_open_source(),
            AmlDivision::CommercialProducts => AmlModelPolicy::cycle_2_commercial(),
        };
        Self {
            cycle_id: CYCLE_2_ID.into(),
            division,
            // Populate exact release identity and artifact hashes from the
            // official bundle; the public webpage alone is not that bundle.
            release_id: String::new(),
            release_manifest_digest: String::new(),
            evaluation_pipeline_digest: String::new(),
            dataset_bundle_digest: String::new(),
            question_manifest_digest: String::new(),
            answer_model_digest: String::new(),
            scoring_contract_digest: String::new(),
            api_contract_digest: String::new(),
            top_k: CYCLE_2_TOP_K,
            max_add_messages: CYCLE_2_MAX_ADD_MESSAGES,
            max_add_words: CYCLE_2_MAX_ADD_WORDS,
            max_add_attempts: CYCLE_2_MAX_ADD_ATTEMPTS,
            max_image_bytes: CYCLE_2_MAX_IMAGE_BYTES,
            max_search_image_bytes: CYCLE_2_MAX_SEARCH_IMAGE_BYTES,
            model_policy,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlRunManifest {
    pub run_id: String,
    pub cycle_id: String,
    pub mode: AmlRunMode,
    pub track: AmlTrack,
    pub division: AmlDivision,
    pub release_id: String,
    pub release_manifest_digest: String,
    pub evaluation_pipeline_digest: String,
    pub dataset_bundle_digest: String,
    pub question_manifest_digest: String,
    pub answer_model_digest: String,
    pub scoring_contract_digest: String,
    pub api_contract_digest: String,
    pub system_id: String,
    pub system_version: String,
    pub system_commit: String,
    pub system_artifact_digest: String,
    pub container_image_digest: String,
    /// Number of unique question IDs in the fixed official task manifest.
    pub expected_question_count: u32,
    /// True only when the official Full run completed without missing tasks.
    pub full_evaluation_complete: bool,
    /// The exact model/component identities observed in the deployed system.
    pub component_uses: Vec<AmlComponentIdentity>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlAddAttempt {
    pub attempt_number: u32,
    pub http_status: u16,
    /// The response must echo the same logical request ID across retries.
    pub response_request_id: String,
    /// Independently captured persistence witness that became durable before
    /// the HTTP 200 response was emitted.
    pub persisted_before_success_response: bool,
    pub persistence_receipt_digest: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlAddOperation {
    pub request_id: String,
    pub sample_id: String,
    pub task_id: String,
    pub user_id: String,
    pub source_session_digest: String,
    pub payload_digest: String,
    pub message_count: u32,
    /// Adapter-counted words using the exact release's counting convention.
    pub word_count: u32,
    pub attempts: Vec<AmlAddAttempt>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AmlContentPart {
    Text {
        ordinal: u32,
        content_digest: String,
        nonempty: bool,
    },
    Image {
        ordinal: u32,
        media_type: String,
        data_uri_digest: String,
        decoded_bytes: u64,
    },
}

impl AmlContentPart {
    fn ordinal(&self) -> u32 {
        match self {
            Self::Text { ordinal, .. } | Self::Image { ordinal, .. } => *ordinal,
        }
    }

    fn decoded_image_bytes(&self) -> u64 {
        match self {
            Self::Image { decoded_bytes, .. } => *decoded_bytes,
            Self::Text { .. } => 0,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlSearchCandidate {
    pub memory_id: String,
    pub rank: u32,
    pub source_add_request_id: String,
    pub source_sample_id: String,
    pub source_task_id: String,
    pub source_user_id: String,
    /// Digest of exact candidate content bytes/ordered content parts.
    pub content_digest: String,
    pub content_nonempty: bool,
    /// Set by the independent response auditor; a synthesized answer is not
    /// an admissible memory record, even if it is plausible.
    pub candidate_is_generated_answer: bool,
    /// Optional relevance score; if every candidate has a score, returned
    /// order must be descending. Original order is otherwise preserved.
    pub score_micros: Option<i64>,
    pub content_parts: Vec<AmlContentPart>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlSearchOperation {
    pub request_id: String,
    pub question_id: String,
    pub question_digest: String,
    pub sample_id: String,
    pub task_id: String,
    pub user_id: String,
    pub http_status: u16,
    pub requested_top_k: u32,
    pub response_body_bytes: u64,
    pub candidates: Vec<AmlSearchCandidate>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlRunEvidence {
    pub manifest: AmlRunManifest,
    pub add_operations: Vec<AmlAddOperation>,
    pub search_operations: Vec<AmlSearchOperation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AmlQualificationReport {
    pub qualification: QualificationReport,
    /// API envelope, scope, persistence, ordering and content rules are valid.
    pub api_contract_eligible: bool,
    /// Component identities satisfy the selected release/division policy.
    pub model_policy_eligible: bool,
    /// Official release, pipeline, data, answer-model and evaluator hashes match.
    pub release_provenance_eligible: bool,
    /// Only a completed official Full run can be considered for an AML score.
    pub official_score_eligible: bool,
    pub add_operation_count: usize,
    pub add_success_count: usize,
    pub add_retry_count: usize,
    pub search_operation_count: usize,
    pub search_candidate_count: usize,
    pub unique_question_count: usize,
    pub generated_answer_substitution_count: usize,
    pub cross_scope_candidate_count: usize,
    pub multimodal_image_bytes: u64,
    pub model_policy_reasons: Vec<String>,
    pub release_provenance_reasons: Vec<String>,
    pub reasons: Vec<String>,
}

fn model_policy_matches(
    manifest: &AmlRunManifest,
    release: &AmlReleaseContract,
    report: &mut QualificationReport,
) -> bool {
    let mut observed = BTreeMap::<AmlComponentKind, BTreeSet<&str>>::new();
    for component in &manifest.component_uses {
        if !nonempty(&component.model_id)
            || !nonempty(&component.model_version)
            || !valid_digest(&component.deployment_digest)
        {
            report.indeterminate("component identity/version/deployment digest is incomplete");
            continue;
        }
        observed
            .entry(component.kind)
            .or_default()
            .insert(component.model_id.as_str());
    }

    for (kind, required) in [
        (
            AmlComponentKind::Embedding,
            release.model_policy.required_embedding_model.as_deref(),
        ),
        (
            AmlComponentKind::LanguageModel,
            release.model_policy.required_language_model.as_deref(),
        ),
    ] {
        if let Some(required_model) = required {
            match observed.get(&kind) {
                Some(models) if models.len() == 1 && models.contains(required_model) => {}
                Some(models) if !models.is_empty() => {
                    report.fail(format!("{kind:?} model identity does not satisfy the release policy"));
                }
                _ => report.indeterminate(format!("{kind:?} model identity is missing")),
            }
        }
    }

    if !release.model_policy.reranker_unrestricted {
        if let Some(models) = observed.get(&AmlComponentKind::Reranker) {
            if models.len() != 1 {
                report.indeterminate("reranker model identity is ambiguous");
            }
        }
    }
    report.status == QualificationStatus::Pass
}

fn validate_add_operation(
    operation: &AmlAddOperation,
    release: &AmlReleaseContract,
    report: &mut AmlQualificationReport,
) {
    if !nonempty(&operation.request_id)
        || !nonempty(&operation.sample_id)
        || !nonempty(&operation.task_id)
        || !nonempty(&operation.user_id)
        || !valid_digest(&operation.source_session_digest)
        || !valid_digest(&operation.payload_digest)
        || operation.message_count == 0
    {
        report
            .qualification
            .fail("Add operation identity, source, payload or message inventory is invalid");
        return;
    }
    if operation.message_count > release.max_add_messages
        || operation.word_count > release.max_add_words
        || operation.word_count == 0
    {
        report.qualification.fail(format!(
            "Add operation {} exceeds the release chunk limits or has no counted words",
            operation.request_id
        ));
    }
    if operation.attempts.is_empty() || operation.attempts.len() > release.max_add_attempts {
        report.qualification.fail(format!(
            "Add operation {} has no attempts or exceeds retry limit",
            operation.request_id
        ));
        return;
    }

    let mut success_seen = false;
    for (index, attempt) in operation.attempts.iter().enumerate() {
        let expected_attempt = (index + 1) as u32;
        if attempt.attempt_number != expected_attempt
            || attempt.response_request_id != operation.request_id
        {
            report.qualification.fail(format!(
                "Add operation {} changed request identity or attempt sequence",
                operation.request_id
            ));
        }
        if success_seen {
            report.qualification.fail(format!(
                "Add operation {} retried after a successful HTTP 200",
                operation.request_id
            ));
        }
        if attempt.http_status == 200 {
            success_seen = true;
            report.add_success_count += 1;
            if !attempt.persisted_before_success_response
                || !attempt
                    .persistence_receipt_digest
                    .as_deref()
                    .is_some_and(valid_digest)
            {
                report.qualification.fail(format!(
                    "Add operation {} returned 200 without a durable-before-ack persistence receipt",
                    operation.request_id
                ));
            }
        } else {
            if attempt.persisted_before_success_response
                || attempt.persistence_receipt_digest.is_some()
            {
                report.qualification.fail(format!(
                    "Add operation {} has a persistence-success receipt on a non-200 attempt",
                    operation.request_id
                ));
            }
            if index + 1 < operation.attempts.len()
                && !retryable_add_status(attempt.http_status)
            {
                report.qualification.fail(format!(
                    "Add operation {} retried a non-retryable HTTP status",
                    operation.request_id
                ));
            }
            if index + 1 == operation.attempts.len() {
                report.qualification.indeterminate(format!(
                    "Add operation {} did not complete with HTTP 200",
                    operation.request_id
                ));
            }
        }
    }
    if !success_seen {
        report.qualification.indeterminate(format!(
            "Add operation {} has no durable successful response",
            operation.request_id
        ));
    }
    report.add_retry_count += operation.attempts.len().saturating_sub(1);
}

fn validate_content_parts(
    track: AmlTrack,
    candidate: &AmlSearchCandidate,
    release: &AmlReleaseContract,
    report: &mut AmlQualificationReport,
) -> u64 {
    if candidate.content_parts.is_empty() {
        report
            .qualification
            .fail(format!("memory {} has no ordered content parts", candidate.memory_id));
        return 0;
    }

    let mut expected_ordinal = 0u32;
    let mut image_bytes = 0u64;
    for part in &candidate.content_parts {
        if part.ordinal() != expected_ordinal {
            report.qualification.fail(format!(
                "memory {} content part order/ordinal is not contiguous",
                candidate.memory_id
            ));
        }
        expected_ordinal = expected_ordinal.saturating_add(1);
        match part {
            AmlContentPart::Text {
                content_digest,
                nonempty,
                ..
            } => {
                if !valid_digest(content_digest) || !*nonempty {
                    report.qualification.fail(format!(
                        "memory {} contains empty or uncommitted text content",
                        candidate.memory_id
                    ));
                }
                if track == AmlTrack::Multimodal {
                    // Text is allowed in ordered multimodal messages.
                }
            }
            AmlContentPart::Image {
                media_type,
                data_uri_digest,
                decoded_bytes,
                ..
            } => {
                if track != AmlTrack::Multimodal {
                    report.qualification.fail(format!(
                        "memory {} contains image content outside the Multimodal track",
                        candidate.memory_id
                    ));
                }
                if !media_type.starts_with("image/")
                    || !valid_digest(data_uri_digest)
                    || *decoded_bytes == 0
                    || *decoded_bytes > release.max_image_bytes
                {
                    report.qualification.fail(format!(
                        "memory {} image part is malformed or exceeds per-image byte limit",
                        candidate.memory_id
                    ));
                }
                image_bytes = image_bytes.saturating_add(*decoded_bytes);
            }
        }
    }
    report.multimodal_image_bytes = report
        .multimodal_image_bytes
        .saturating_add(image_bytes);
    image_bytes
}

fn validate_search_operation(
    operation: &AmlSearchOperation,
    evidence: &AmlRunEvidence,
    release: &AmlReleaseContract,
    report: &mut AmlQualificationReport,
) {
    if !nonempty(&operation.request_id)
        || !nonempty(&operation.question_id)
        || !valid_digest(&operation.question_digest)
        || !nonempty(&operation.sample_id)
        || !nonempty(&operation.task_id)
        || !nonempty(&operation.user_id)
    {
        report
            .qualification
            .fail("Search operation identity, scope or question commitment is invalid");
        return;
    }
    if operation.http_status != 200 {
        report.qualification.indeterminate(format!(
            "Search operation {} did not return HTTP 200",
            operation.request_id
        ));
    }
    if operation.requested_top_k == 0 || operation.requested_top_k > release.top_k {
        report.qualification.fail(format!(
            "Search operation {} requested Top K outside the release limit",
            operation.request_id
        ));
    }
    if operation.candidates.len() > operation.requested_top_k as usize {
        report.qualification.fail(format!(
            "Search operation {} returned more candidates than requested Top K",
            operation.request_id
        ));
    }

    let mut seen_ids = BTreeSet::new();
    let mut scores = Vec::new();
    let mut operation_image_bytes = 0u64;
    let all_scored = operation
        .candidates
        .iter()
        .all(|candidate| candidate.score_micros.is_some());

    for (index, candidate) in operation.candidates.iter().enumerate() {
        report.search_candidate_count += 1;
        if !nonempty(&candidate.memory_id)
            || !valid_digest(&candidate.content_digest)
            || !candidate.content_nonempty
            || candidate.rank != index as u32
            || !seen_ids.insert(candidate.memory_id.as_str())
        {
            report.qualification.fail(format!(
                "Search operation {} has an invalid, duplicate or out-of-order candidate",
                operation.request_id
            ));
        }
        if candidate.candidate_is_generated_answer {
            report.generated_answer_substitution_count += 1;
            report.qualification.fail(format!(
                "Search operation {} returned a synthesized answer instead of a memory candidate",
                operation.request_id
            ));
        }
        let source_add = evidence
            .add_operations
            .iter()
            .find(|add| add.request_id == candidate.source_add_request_id);
        match source_add {
            Some(add)
                if add.sample_id == operation.sample_id
                    && add.task_id == operation.task_id
                    && add.user_id == operation.user_id => {}
            Some(_) => {
                report.cross_scope_candidate_count += 1;
                report.qualification.fail(format!(
                    "Search operation {} returned a candidate from another user/sample/task scope",
                    operation.request_id
                ));
            }
            None => {
                report.qualification.indeterminate(format!(
                    "Search operation {} candidate source Add receipt is missing",
                    operation.request_id
                ));
            }
        }
        if candidate.source_sample_id != operation.sample_id
            || candidate.source_task_id != operation.task_id
            || candidate.source_user_id != operation.user_id
        {
            report.cross_scope_candidate_count += 1;
            report.qualification.fail(format!(
                "Search operation {} candidate scope metadata differs from its query",
                operation.request_id
            ));
        }
        if all_scored {
            scores.push(candidate.score_micros.unwrap_or_default());
        }
        operation_image_bytes = operation_image_bytes.saturating_add(
            validate_content_parts(evidence.manifest.track, candidate, release, report),
        );
    }

    if evidence.manifest.track == AmlTrack::Multimodal
        && operation_image_bytes > release.max_search_image_bytes
    {
        report.qualification.fail(format!(
            "Search operation {} exceeds the combined multimodal image byte limit",
            operation.request_id
        ));
    }

    if all_scored && scores.windows(2).any(|pair| pair[0] < pair[1]) {
        report.qualification.fail(format!(
            "Search operation {} score order is not descending",
            operation.request_id
        ));
    }
    if operation.response_body_bytes == 0 {
        report.qualification.fail("Search response has zero serialized bytes");
    }
    if operation.response_body_bytes > release.max_search_image_bytes
        && evidence.manifest.track == AmlTrack::Multimodal
    {
        report.qualification.fail(format!(
            "Search operation {} exceeds the multimodal response byte limit",
            operation.request_id
        ));
    }
}

pub fn qualify_aml_run(
    evidence: &AmlRunEvidence,
    release: &AmlReleaseContract,
) -> AmlQualificationReport {
    let mut output = AmlQualificationReport {
        qualification: QualificationReport::new(),
        api_contract_eligible: false,
        model_policy_eligible: false,
        release_provenance_eligible: false,
        official_score_eligible: false,
        add_operation_count: evidence.add_operations.len(),
        add_success_count: 0,
        add_retry_count: 0,
        search_operation_count: evidence.search_operations.len(),
        search_candidate_count: 0,
        unique_question_count: 0,
        generated_answer_substitution_count: 0,
        cross_scope_candidate_count: 0,
        multimodal_image_bytes: 0,
        reasons: Vec::new(),
    };

    let manifest = &evidence.manifest;
    if !nonempty(&manifest.run_id)
        || !nonempty(&manifest.cycle_id)
        || !nonempty(&manifest.release_id)
        || !nonempty(&manifest.system_id)
        || !nonempty(&manifest.system_version)
        || !nonempty(&manifest.api_contract_digest)
        || !valid_git_commit(&manifest.system_commit)
        || !valid_digest(&manifest.system_artifact_digest)
    {
        output
            .qualification
            .fail("run or system identity/version/commit/artifact is invalid");
    }
    if manifest.cycle_id != release.cycle_id {
        output
            .qualification
            .fail("manifest cycle differs from selected release contract");
    }
    if manifest.division != release.division {
        output
            .qualification
            .fail("manifest division differs from selected release policy");
    }

    // Official release provenance is independent of API contract validity.
    let mut release_report = QualificationReport::new();
    for (label, actual, expected) in [
        ("release ID", manifest.release_id.as_str(), release.release_id.as_str()),
        ("release manifest", manifest.release_manifest_digest.as_str(), release.release_manifest_digest.as_str()),
        ("evaluation pipeline", manifest.evaluation_pipeline_digest.as_str(), release.evaluation_pipeline_digest.as_str()),
        ("dataset bundle", manifest.dataset_bundle_digest.as_str(), release.dataset_bundle_digest.as_str()),
        ("question manifest", manifest.question_manifest_digest.as_str(), release.question_manifest_digest.as_str()),
        ("answer model", manifest.answer_model_digest.as_str(), release.answer_model_digest.as_str()),
        ("scoring contract", manifest.scoring_contract_digest.as_str(), release.scoring_contract_digest.as_str()),
        ("API contract", manifest.api_contract_digest.as_str(), release.api_contract_digest.as_str()),
    ] {
        if !nonempty(expected) || !valid_digest(actual) || !valid_digest(expected) {
            release_report.indeterminate(format!(
                "{label} identity/digest is not available from a pinned official release"
            ));
        } else if actual != expected {
            release_report.fail(format!("{label} differs from pinned official release"));
        }
    }
    output.release_provenance_eligible =
        release_report.status == QualificationStatus::Pass;
    output.release_provenance_reasons = release_report.reasons.clone();

    if manifest.container_image_digest.is_empty() {
        output
            .qualification
            .indeterminate("system container/image identity is not recorded");
    } else if !valid_digest(&manifest.container_image_digest) {
        output
            .qualification
            .fail("system container/image digest is malformed");
    }
    if manifest.expected_question_count == 0 {
        output
            .qualification
            .indeterminate("official question count is missing");
    }

    // Component policy is a third independent axis; a policy mismatch does
    // not erase a correctly implemented Add/Search API contract.
    let mut policy_report = QualificationReport::new();
    output.model_policy_eligible =
        model_policy_matches(manifest, release, &mut policy_report);
    output.model_policy_reasons = policy_report.reasons.clone();

    let mut request_ids = BTreeSet::new();
    let mut add_records = BTreeMap::new();
    for operation in &evidence.add_operations {
        if !request_ids.insert(operation.request_id.as_str()) {
            output.qualification.fail("Add request IDs are duplicated");
        }
        validate_add_operation(operation, release, &mut output);
        add_records.insert(operation.request_id.as_str(), operation);
    }

    let mut question_ids = BTreeSet::new();
    let mut search_ids = BTreeSet::new();
    for operation in &evidence.search_operations {
        if !search_ids.insert(operation.request_id.as_str()) {
            output.qualification.fail("Search request IDs are duplicated");
        }
        if !question_ids.insert(operation.question_id.as_str()) {
            output.qualification.fail("question ID was evaluated more than once");
        }
        validate_search_operation(operation, evidence, release, &mut output);
    }
    output.unique_question_count = question_ids.len();

    if manifest.expected_question_count as usize != output.unique_question_count {
        output.qualification.indeterminate(format!(
            "observed {} unique questions; expected {}",
            output.unique_question_count, manifest.expected_question_count
        ));
    }
    if evidence.add_operations.is_empty() || evidence.search_operations.is_empty() {
        output
            .qualification
            .indeterminate("Add and Search operation evidence must both be present");
    }

    // Preserve native/API qualification independently from division model-policy
    // compliance. An incompatible native run is still useful research evidence,
    // but it is not eligible for the official leaderboard division.
    let api_contract_eligible = output.qualification.status == QualificationStatus::Pass;
    output.api_contract_eligible = api_contract_eligible;

    let official_reproduction = manifest.mode == AmlRunMode::Full
        && manifest.full_evaluation_complete
        && manifest.expected_question_count > 0
        && output.unique_question_count == manifest.expected_question_count
        && output.model_policy_eligible
        && output.release_provenance_eligible
        && api_contract_eligible;
    output.official_score_eligible = official_reproduction;

    // Native/unrestricted mode can never be reported as an official AML score.
    if manifest.mode == AmlRunMode::NativeUnrestricted {
        output.official_score_eligible = false;
    }
    if manifest.mode == AmlRunMode::Smoke {
        output.official_score_eligible = false;
    }

    output.reasons.push(
        "API compatibility, model-policy eligibility and official release provenance are independently qualified".into(),
    );
    output.reasons.extend(output.model_policy_reasons.clone());
    output.reasons.extend(output.release_provenance_reasons.clone());
    output.reasons.extend(output.qualification.reasons.clone());
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    const D1: &str = "1111111111111111111111111111111111111111111111111111111111111111";
    const D2: &str = "2222222222222222222222222222222222222222222222222222222222222222";
    const D3: &str = "3333333333333333333333333333333333333333333333333333333333333333";
    const D4: &str = "4444444444444444444444444444444444444444444444444444444444444444";
    const D5: &str = "5555555555555555555555555555555555555555555555555555555555555555";
    const D6: &str = "6666666666666666666666666666666666666666666666666666666666666666";

    fn release() -> AmlReleaseContract {
        AmlReleaseContract {
            cycle_id: CYCLE_2_ID.into(),
            division: AmlDivision::OpenSourceMethods,
            release_id: "cycle-2-test-fixture".into(),
            release_manifest_digest: D1.into(),
            evaluation_pipeline_digest: D2.into(),
            dataset_bundle_digest: D3.into(),
            question_manifest_digest: D4.into(),
            answer_model_digest: D5.into(),
            scoring_contract_digest: D6.into(),
            api_contract_digest: D1.into(),
            top_k: 100,
            max_add_messages: 20,
            max_add_words: 2_000,
            max_add_attempts: 32,
            max_image_bytes: 10 * 1024 * 1024,
            max_search_image_bytes: 30 * 1024 * 1024,
            model_policy: AmlModelPolicy {
                required_embedding_model: Some("text-embedding-v4".into()),
                required_language_model: Some("gpt-4o-mini".into()),
                reranker_unrestricted: true,
            },
        }
    }

    fn valid_run() -> AmlRunEvidence {
        let release = release();
        AmlRunEvidence {
            manifest: AmlRunManifest {
                run_id: "run-1".into(),
                cycle_id: CYCLE_2_ID.into(),
                mode: AmlRunMode::Full,
                track: AmlTrack::Textual,
                division: AmlDivision::OpenSourceMethods,
                release_id: release.release_id.clone(),
                release_manifest_digest: release.release_manifest_digest.clone(),
                evaluation_pipeline_digest: release.evaluation_pipeline_digest.clone(),
                dataset_bundle_digest: release.dataset_bundle_digest.clone(),
                question_manifest_digest: release.question_manifest_digest.clone(),
                answer_model_digest: release.answer_model_digest.clone(),
                scoring_contract_digest: release.scoring_contract_digest.clone(),
                api_contract_digest: release.api_contract_digest.clone(),
                system_id: "mycelix-adapter-fixture".into(),
                system_version: "0.1.0".into(),
                system_commit: "a".repeat(40),
                system_artifact_digest: D6.into(),
                container_image_digest: D5.into(),
                expected_question_count: 1,
                full_evaluation_complete: true,
                component_uses: vec![
                    AmlComponentIdentity {
                        kind: AmlComponentKind::Embedding,
                        model_id: "text-embedding-v4".into(),
                        model_version: "pinned".into(),
                        deployment_digest: D1.into(),
                    },
                    AmlComponentIdentity {
                        kind: AmlComponentKind::LanguageModel,
                        model_id: "gpt-4o-mini".into(),
                        model_version: "pinned".into(),
                        deployment_digest: D2.into(),
                    },
                    AmlComponentIdentity {
                        kind: AmlComponentKind::Reranker,
                        model_id: "mycelix-reranker".into(),
                        model_version: "pinned".into(),
                        deployment_digest: D3.into(),
                    },
                ],
            },
            add_operations: vec![AmlAddOperation {
                request_id: "add-1".into(),
                sample_id: "sample-1".into(),
                task_id: "task-1".into(),
                user_id: "user-1".into(),
                source_session_digest: D4.into(),
                payload_digest: D5.into(),
                message_count: 2,
                word_count: 100,
                attempts: vec![AmlAddAttempt {
                    attempt_number: 1,
                    http_status: 200,
                    response_request_id: "add-1".into(),
                    persisted_before_success_response: true,
                    persistence_receipt_digest: Some(D6.into()),
                }],
            }],
            search_operations: vec![AmlSearchOperation {
                request_id: "search-1".into(),
                question_id: "question-1".into(),
                question_digest: D1.into(),
                sample_id: "sample-1".into(),
                task_id: "task-1".into(),
                user_id: "user-1".into(),
                http_status: 200,
                requested_top_k: 100,
                response_body_bytes: 256,
                candidates: vec![AmlSearchCandidate {
                    memory_id: "memory-1".into(),
                    rank: 0,
                    source_add_request_id: "add-1".into(),
                    source_sample_id: "sample-1".into(),
                    source_task_id: "task-1".into(),
                    source_user_id: "user-1".into(),
                    content_digest: D2.into(),
                    content_nonempty: true,
                    candidate_is_generated_answer: false,
                    score_micros: Some(900_000),
                    content_parts: vec![AmlContentPart::Text {
                        ordinal: 0,
                        content_digest: D3.into(),
                        nonempty: true,
                    }],
                }],
            }],
        }
    }

    #[test]
    fn exact_full_run_satisfying_pinned_release_is_official_score_eligible() {
        let input = valid_run();
        let report = qualify_aml_run(&input, &release());
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert!(report.api_contract_eligible);
        assert!(report.model_policy_eligible);
        assert!(report.official_score_eligible);
        assert_eq!(report.unique_question_count, 1);
    }

    #[test]
    fn smoke_is_api_eligible_but_never_a_score() {
        let mut input = valid_run();
        input.manifest.mode = AmlRunMode::Smoke;
        let report = qualify_aml_run(&input, &release());
        assert!(report.api_contract_eligible);
        assert!(!report.official_score_eligible);
    }

    #[test]
    fn native_hdc_mode_can_pass_api_contract_without_becoming_an_aml_score() {
        let mut input = valid_run();
        input.manifest.mode = AmlRunMode::NativeUnrestricted;
        input.manifest.component_uses.retain(|component| {
            component.kind != AmlComponentKind::Embedding
                && component.kind != AmlComponentKind::LanguageModel
        });
        let report = qualify_aml_run(&input, &release());
        assert!(report.api_contract_eligible);
        assert!(!report.model_policy_eligible);
        assert!(!report.official_score_eligible);
    }

    #[test]
    fn open_source_model_policy_mismatch_does_not_erase_api_compatibility() {
        let mut input = valid_run();
        input.manifest.component_uses[0].model_id = "native-hdc-16384".into();
        let report = qualify_aml_run(&input, &release());
        assert!(report.api_contract_eligible);
        assert!(!report.model_policy_eligible);
        assert!(!report.official_score_eligible);
    }

    #[test]
    fn commercial_division_does_not_inherit_open_source_model_restrictions() {
        let mut input = valid_run();
        input.manifest.division = AmlDivision::CommercialProducts;
        input.manifest.component_uses.retain(|component| {
            component.kind != AmlComponentKind::Embedding
                && component.kind != AmlComponentKind::LanguageModel
        });
        let mut release = release();
        release.division = AmlDivision::CommercialProducts;
        release.model_policy = AmlModelPolicy::cycle_2_commercial();
        let report = qualify_aml_run(&input, &release);
        assert!(report.api_contract_eligible);
        assert!(report.model_policy_eligible);
    }

    #[test]
    fn retry_uses_same_idempotency_key_and_payload_before_success() {
        let mut input = valid_run();
        let add = &mut input.add_operations[0];
        add.attempts = vec![
            AmlAddAttempt {
                attempt_number: 1,
                http_status: 503,
                response_request_id: "add-1".into(),
                persisted_before_success_response: false,
                persistence_receipt_digest: None,
            },
            AmlAddAttempt {
                attempt_number: 2,
                http_status: 200,
                response_request_id: "add-1".into(),
                persisted_before_success_response: true,
                persistence_receipt_digest: Some(D6.into()),
            },
        ];
        let report = qualify_aml_run(&input, &release());
        assert_eq!(report.qualification.status, QualificationStatus::Pass);
        assert_eq!(report.add_retry_count, 1);
    }

    #[test]
    fn retry_with_changed_request_id_or_nonretryable_status_fails() {
        let mut input = valid_run();
        let add = &mut input.add_operations[0];
        add.attempts = vec![
            AmlAddAttempt {
                attempt_number: 1,
                http_status: 400,
                response_request_id: "add-1".into(),
                persisted_before_success_response: false,
                persistence_receipt_digest: None,
            },
            AmlAddAttempt {
                attempt_number: 2,
                http_status: 200,
                response_request_id: "different".into(),
                persisted_before_success_response: true,
                persistence_receipt_digest: Some(D6.into()),
            },
        ];
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn returning_200_before_persistence_fails() {
        let mut input = valid_run();
        input.add_operations[0].attempts[0].persisted_before_success_response = false;
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn cross_user_sample_or_task_candidate_fails() {
        let mut input = valid_run();
        input.search_operations[0].candidates[0].source_user_id = "other-user".into();
        let report = qualify_aml_run(&input, &release());
        assert!(!report.api_contract_eligible);
        assert_eq!(report.cross_scope_candidate_count, 1);
    }

    #[test]
    fn generated_answer_substitution_fails() {
        let mut input = valid_run();
        input.search_operations[0].candidates[0].candidate_is_generated_answer = true;
        let report = qualify_aml_run(&input, &release());
        assert!(!report.api_contract_eligible);
        assert_eq!(report.generated_answer_substitution_count, 1);
    }

    #[test]
    fn top_k_overrun_and_rank_reordering_fail() {
        let mut input = valid_run();
        input.search_operations[0].requested_top_k = 101;
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );

        let mut input = valid_run();
        input.search_operations[0].candidates[0].rank = 3;
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn multimodal_image_parts_are_limited_and_ordered() {
        let mut input = valid_run();
        input.manifest.track = AmlTrack::Multimodal;
        input.search_operations[0].candidates[0].content_parts = vec![
            AmlContentPart::Text {
                ordinal: 0,
                content_digest: D3.into(),
                nonempty: true,
            },
            AmlContentPart::Image {
                ordinal: 1,
                media_type: "image/png".into(),
                data_uri_digest: D4.into(),
                decoded_bytes: 1024,
            },
        ];
        let report = qualify_aml_run(&input, &release());
        assert!(report.api_contract_eligible);
        assert_eq!(report.multimodal_image_bytes, 1024);

        input.search_operations[0].candidates[0].content_parts[1] =
            AmlContentPart::Image {
                ordinal: 1,
                media_type: "image/png".into(),
                data_uri_digest: D4.into(),
                decoded_bytes: 11 * 1024 * 1024,
            };
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn nonmultimodal_track_rejects_images() {
        let mut input = valid_run();
        input.search_operations[0].candidates[0].content_parts = vec![
            AmlContentPart::Image {
                ordinal: 0,
                media_type: "image/png".into(),
                data_uri_digest: D4.into(),
                decoded_bytes: 1024,
            },
        ];
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn search_missing_candidate_source_is_indeterminate() {
        let mut input = valid_run();
        input.search_operations[0].candidates[0].source_add_request_id = "missing-add".into();
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Indeterminate
        );
    }

    #[test]
    fn missing_question_coverage_blocks_full_score() {
        let mut input = valid_run();
        input.manifest.expected_question_count = 2;
        let report = qualify_aml_run(&input, &release());
        assert!(report.api_contract_eligible);
        assert!(!report.official_score_eligible);
    }

    #[test]
    fn duplicate_question_and_add_ids_fail() {
        let mut input = valid_run();
        input.search_operations.push(input.search_operations[0].clone());
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );

        let mut input = valid_run();
        input.add_operations.push(input.add_operations[0].clone());
        assert_eq!(
            qualify_aml_run(&input, &release()).qualification.status,
            QualificationStatus::Fail
        );
    }

    #[test]
    fn missing_release_hashes_keep_api_checks_but_block_official_score() {
        let mut release = release();
        release.evaluation_pipeline_digest.clear();
        let input = valid_run();
        let report = qualify_aml_run(&input, &release);
        assert!(!report.api_contract_eligible);
        assert!(!report.official_score_eligible);
    }
}
