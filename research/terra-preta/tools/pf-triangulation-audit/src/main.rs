use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::HashSet,
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const AUDIT_VERSION: &str = "0.1.0";

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ProxyStatus {
    Positive,
    Negative,
    NotEvaluated,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum NsrSignature {
    NonsequentialResponseDetected,
    NonsequentialSignatureIncompleteProfile,
    MixedResponseOrdering,
    SequentialResponseOnly,
    SequentialAndSimultaneousResponse,
    AmbiguousSimultaneousResponse,
    NotApplicableInsufficientResponders,
    IndeterminateNoDistinctDepthPair,
    IndeterminateIncompleteProfileWithoutNsr,
}

impl NsrSignature {
    fn as_str(self) -> &'static str {
        match self {
            Self::NonsequentialResponseDetected => "nonsequential_response_detected",
            Self::NonsequentialSignatureIncompleteProfile => "nonsequential_signature_incomplete_profile",
            Self::MixedResponseOrdering => "mixed_response_ordering",
            Self::SequentialResponseOnly => "sequential_response_only",
            Self::SequentialAndSimultaneousResponse => "sequential_and_simultaneous_response",
            Self::AmbiguousSimultaneousResponse => "ambiguous_simultaneous_response",
            Self::NotApplicableInsufficientResponders => "not_applicable_insufficient_responders",
            Self::IndeterminateNoDistinctDepthPair => "indeterminate_no_distinct_depth_pair",
            Self::IndeterminateIncompleteProfileWithoutNsr => "indeterminate_incomplete_profile_without_nsr",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ReferenceMethod {
    ConservativeTracer,
    StableIsotopeTracer,
    DyeTracer,
    Ert,
    Gpr,
    WaterTable,
    LysimeterBoundaryFlux,
    OtherDeclared,
}

impl ReferenceMethod {
    fn as_str(self) -> &'static str {
        match self {
            Self::ConservativeTracer => "conservative_tracer",
            Self::StableIsotopeTracer => "stable_isotope_tracer",
            Self::DyeTracer => "dye_tracer",
            Self::Ert => "ert",
            Self::Gpr => "gpr",
            Self::WaterTable => "water_table",
            Self::LysimeterBoundaryFlux => "lysimeter_boundary_flux",
            Self::OtherDeclared => "other_declared",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ReferenceReviewClaim {
    PendingIndependentReview,
    ClaimsIndependentReviewBlinded,
    SyntheticTestOnly,
}

impl ReferenceReviewClaim {
    fn as_str(self) -> &'static str {
        match self {
            Self::PendingIndependentReview => "pending_independent_review",
            Self::ClaimsIndependentReviewBlinded => "claims_independent_review_blinded",
            Self::SyntheticTestOnly => "synthetic_test_only",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ReferenceQualityStatus {
    Qualified,
    Unqualified,
    Indeterminate,
}

impl ReferenceQualityStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::Qualified => "qualified",
            Self::Unqualified => "unqualified",
            Self::Indeterminate => "indeterminate",
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ReferenceOutcome {
    SupportsBypass,
    DoesNotSupportBypass,
    Indeterminate,
}

impl ReferenceOutcome {
    fn as_str(self) -> &'static str {
        match self {
            Self::SupportsBypass => "supports_bypass",
            Self::DoesNotSupportBypass => "does_not_support_bypass",
            Self::Indeterminate => "indeterminate",
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SensorMethodEvidence {
    sensor_id: String,
    depth_cm: f64,
    velocity_threshold_proxy: ProxyStatus,
    trajectory_threshold_proxy: ProxyStatus,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IndependentReference {
    reference_id: String,
    event_id: String,
    profile_id: String,
    method: ReferenceMethod,
    outcome: ReferenceOutcome,
    review_claim: ReferenceReviewClaim,
    quality_status: ReferenceQualityStatus,
    reviewer_evidence_ref_claimed: Option<String>,
    evidence_ref: String,
    detection_limit_or_resolution: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ArtifactReference {
    tool_name: String,
    tool_version: String,
    output_sha256: String,
    evidence_ref: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    source_sha256: String,
    source_evidence_ref: String,
    nsr_profile_signature: NsrSignature,
    sensor_evidence: Vec<SensorMethodEvidence>,
    independent_references: Vec<IndependentReference>,
    method_artifacts: Vec<ArtifactReference>,
}

#[derive(Serialize)]
struct SensorConcordance {
    sensor_id: String,
    depth_cm: f64,
    velocity_threshold_proxy: &'static str,
    trajectory_threshold_proxy: &'static str,
    concordance: &'static str,
}

#[derive(Serialize)]
struct ReferenceResult {
    reference_id: String,
    event_id: String,
    profile_id: String,
    method: &'static str,
    outcome: &'static str,
    review_claim: &'static str,
    review_verification_status: &'static str,
    quality_status_claimed: &'static str,
    reviewer_evidence_ref_claimed: Option<String>,
    evidence_ref: String,
    detection_limit_or_resolution: String,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
    study_id: String,
    site_id: String,
    plot_id: String,
    profile_id: String,
    event_id: String,
    nsr_profile_signature: &'static str,
    sensor_count: usize,
    both_proxy_methods_evaluated_count: usize,
    proxy_agree_positive_count: usize,
    proxy_agree_negative_count: usize,
    proxy_disagreement_count: usize,
    velocity_only_evaluated_count: usize,
    trajectory_only_evaluated_count: usize,
    neither_proxy_evaluable_count: usize,
    velocity_indeterminate_count: usize,
    trajectory_indeterminate_count: usize,
    independent_reference_count: usize,
    eligible_independent_reference_count: usize,
    pending_review_reference_count: usize,
    unverified_review_claim_count: usize,
    synthetic_reference_count: usize,
    unqualified_reference_count: usize,
    quality_indeterminate_reference_count: usize,
    claimed_reference_supporting_count: usize,
    claimed_reference_not_supporting_count: usize,
    claimed_reference_indeterminate_count: usize,
    independent_reference_assessment: &'static str,
    physical_claim_status: &'static str,
    sensor_concordance: Vec<SensorConcordance>,
    independent_references: Vec<ReferenceResult>,
    method_artifacts: Vec<ArtifactReference>,
    interpretation_boundary: &'static str,
    warnings: Vec<String>,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn valid_sha256(value: &str, label: &str) -> Result<(), String> {
    if value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!("{label} must contain exactly 64 hexadecimal characters"));
    }
    Ok(())
}

fn proxy_concordance(vt: ProxyStatus, trajectory: ProxyStatus) -> &'static str {
    match (vt, trajectory) {
        (ProxyStatus::Positive, ProxyStatus::Positive) => "agree_positive",
        (ProxyStatus::Negative, ProxyStatus::Negative) => "agree_negative",
        (ProxyStatus::Positive, ProxyStatus::Negative)
        | (ProxyStatus::Negative, ProxyStatus::Positive) => "disagree",
        (ProxyStatus::Positive | ProxyStatus::Negative, ProxyStatus::NotEvaluated) => {
            "velocity_threshold_only"
        }
        (ProxyStatus::NotEvaluated, ProxyStatus::Positive | ProxyStatus::Negative) => {
            "trajectory_only"
        }
        (ProxyStatus::Indeterminate, ProxyStatus::Positive | ProxyStatus::Negative)
        | (ProxyStatus::Positive | ProxyStatus::Negative, ProxyStatus::Indeterminate)
        | (ProxyStatus::Indeterminate, ProxyStatus::Indeterminate) => {
            "indeterminate_method_evidence"
        }
        _ => "neither_proxy_evaluable",
    }
}

fn validate(input: &Input) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.plot_id, "plot_id"),
        (&input.profile_id, "profile_id"),
        (&input.event_id, "event_id"),
        (&input.source_evidence_ref, "source_evidence_ref"),
    ] {
        nonblank(value, label)?;
    }
    valid_sha256(&input.source_sha256, "source_sha256")?;
    if input.sensor_evidence.is_empty() {
        return Err("sensor_evidence must contain at least one sensor".to_owned());
    }

    let mut sensor_ids = HashSet::new();
    for (index, sensor) in input.sensor_evidence.iter().enumerate() {
        nonblank(&sensor.sensor_id, &format!("sensor_evidence[{index}].sensor_id"))?;
        if !sensor_ids.insert(sensor.sensor_id.as_str()) {
            return Err(format!("duplicate sensor_id {:?}", sensor.sensor_id));
        }
        if !sensor.depth_cm.is_finite() || sensor.depth_cm <= 0.0 {
            return Err(format!("sensor_evidence[{index}].depth_cm must be finite and positive"));
        }
    }

    let mut reference_ids = HashSet::new();
    for (index, reference) in input.independent_references.iter().enumerate() {
        nonblank(&reference.reference_id, &format!("independent_references[{index}].reference_id"))?;
        nonblank(&reference.event_id, &format!("independent_references[{index}].event_id"))?;
        nonblank(&reference.profile_id, &format!("independent_references[{index}].profile_id"))?;
        if reference.event_id != input.event_id || reference.profile_id != input.profile_id {
            return Err(format!("independent reference {:?} does not match the input event/profile", reference.reference_id));
        }
        nonblank(&reference.evidence_ref, &format!("independent_references[{index}].evidence_ref"))?;
        nonblank(
            &reference.detection_limit_or_resolution,
            &format!("independent_references[{index}].detection_limit_or_resolution"),
        )?;
        match (reference.review_claim, reference.reviewer_evidence_ref_claimed.as_deref()) {
            (ReferenceReviewClaim::ClaimsIndependentReviewBlinded, Some(value))
                if !value.trim().is_empty() => {}
            (ReferenceReviewClaim::ClaimsIndependentReviewBlinded, _) => {
                return Err(format!(
                    "independent_references[{index}] review claim requires reviewer_evidence_ref_claimed"
                ));
            }
            (_, Some(value)) if value.trim().is_empty() => {
                return Err(format!(
                    "independent_references[{index}].reviewer_evidence_ref_claimed must not be blank when supplied"
                ));
            }
            _ => {}
        }
        if !reference_ids.insert(reference.reference_id.as_str()) {
            return Err(format!("duplicate reference_id {:?}", reference.reference_id));
        }
    }

    let mut artifact_keys = HashSet::new();
    for (index, artifact) in input.method_artifacts.iter().enumerate() {
        nonblank(&artifact.tool_name, &format!("method_artifacts[{index}].tool_name"))?;
        nonblank(&artifact.tool_version, &format!("method_artifacts[{index}].tool_version"))?;
        nonblank(&artifact.evidence_ref, &format!("method_artifacts[{index}].evidence_ref"))?;
        valid_sha256(&artifact.output_sha256, &format!("method_artifacts[{index}].output_sha256"))?;
        if !artifact_keys.insert((artifact.tool_name.as_str(), artifact.tool_version.as_str())) {
            return Err(format!("duplicate tool artifact {:?} {:?}", artifact.tool_name, artifact.tool_version));
        }
    }
    Ok(())
}

fn evaluate(input: &Input, input_sha256: String) -> Result<Report, String> {
    validate(input)?;
    let mut sensor_concordance = Vec::with_capacity(input.sensor_evidence.len());
    let mut both = 0usize;
    let mut agree_positive = 0usize;
    let mut agree_negative = 0usize;
    let mut disagree = 0usize;
    let mut velocity_only = 0usize;
    let mut trajectory_only = 0usize;
    let mut neither = 0usize;
    let mut velocity_indeterminate = 0usize;
    let mut trajectory_indeterminate = 0usize;

    for sensor in &input.sensor_evidence {
        let concordance = proxy_concordance(
            sensor.velocity_threshold_proxy,
            sensor.trajectory_threshold_proxy,
        );
        match concordance {
            "agree_positive" => {
                both += 1;
                agree_positive += 1;
            }
            "agree_negative" => {
                both += 1;
                agree_negative += 1;
            }
            "disagree" => {
                both += 1;
                disagree += 1;
            }
            "velocity_threshold_only" => velocity_only += 1,
            "trajectory_only" => trajectory_only += 1,
            "neither_proxy_evaluable" => neither += 1,
            "indeterminate_method_evidence" => {
                if sensor.velocity_threshold_proxy == ProxyStatus::Indeterminate {
                    velocity_indeterminate += 1;
                }
                if sensor.trajectory_threshold_proxy == ProxyStatus::Indeterminate {
                    trajectory_indeterminate += 1;
                }
            }
            _ => return Err("internal concordance state error".to_owned()),
        }
        if concordance == "neither_proxy_evaluable" {
            if sensor.velocity_threshold_proxy == ProxyStatus::Indeterminate {
                velocity_indeterminate += 1;
            }
            if sensor.trajectory_threshold_proxy == ProxyStatus::Indeterminate {
                trajectory_indeterminate += 1;
            }
        }
        sensor_concordance.push(SensorConcordance {
            sensor_id: sensor.sensor_id.clone(),
            depth_cm: sensor.depth_cm,
            velocity_threshold_proxy: proxy_status_name(sensor.velocity_threshold_proxy),
            trajectory_threshold_proxy: proxy_status_name(sensor.trajectory_threshold_proxy),
            concordance,
        });
    }

    let mut claimed_supporting = 0usize;
    let mut claimed_not_supporting = 0usize;
    let mut claimed_indeterminate = 0usize;
    let mut pending_review_count = 0usize;
    let mut unverified_review_claim_count = 0usize;
    let mut synthetic_reference_count = 0usize;
    let mut unqualified_reference_count = 0usize;
    let mut quality_indeterminate_reference_count = 0usize;

    // This v0.1 audit has no trusted issuer-key registry or signature verifier.
    // Therefore every review/quality field is a self-declared claim, never
    // sufficient to adjudicate a physical pathway claim.
    let independent_references: Vec<_> = input
        .independent_references
        .iter()
        .map(|reference| {
            match reference.review_claim {
                ReferenceReviewClaim::PendingIndependentReview => pending_review_count += 1,
                ReferenceReviewClaim::ClaimsIndependentReviewBlinded => {
                    unverified_review_claim_count += 1
                }
                ReferenceReviewClaim::SyntheticTestOnly => synthetic_reference_count += 1,
            }
            if reference.quality_status == ReferenceQualityStatus::Unqualified {
                unqualified_reference_count += 1;
            } else if reference.quality_status == ReferenceQualityStatus::Indeterminate {
                quality_indeterminate_reference_count += 1;
            }
            match reference.outcome {
                ReferenceOutcome::SupportsBypass => claimed_supporting += 1,
                ReferenceOutcome::DoesNotSupportBypass => claimed_not_supporting += 1,
                ReferenceOutcome::Indeterminate => claimed_indeterminate += 1,
            }
            ReferenceResult {
                reference_id: reference.reference_id.clone(),
                event_id: reference.event_id.clone(),
                profile_id: reference.profile_id.clone(),
                method: reference.method.as_str(),
                outcome: reference.outcome.as_str(),
                review_claim: reference.review_claim.as_str(),
                review_verification_status: "not_cryptographically_verified",
                quality_status_claimed: reference.quality_status.as_str(),
                reviewer_evidence_ref_claimed: reference.reviewer_evidence_ref_claimed.clone(),
                evidence_ref: reference.evidence_ref.clone(),
                detection_limit_or_resolution: reference.detection_limit_or_resolution.clone(),
            }
        })
        .collect();

    let eligible_reference_count = 0usize;
    let reference_state = if input.independent_references.is_empty() {
        "no_independent_reference_records"
    } else {
        "unverified_reference_claims_only"
    };
    let physical_claim_status = "not_adjudicated_external_attestation_verifier_required";

    Ok(Report {
        audit_version: AUDIT_VERSION,
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        plot_id: input.plot_id.clone(),
        profile_id: input.profile_id.clone(),
        event_id: input.event_id.clone(),
        nsr_profile_signature: input.nsr_profile_signature.as_str(),
        sensor_count: input.sensor_evidence.len(),
        both_proxy_methods_evaluated_count: both,
        proxy_agree_positive_count: agree_positive,
        proxy_agree_negative_count: agree_negative,
        proxy_disagreement_count: disagree,
        velocity_only_evaluated_count: velocity_only,
        trajectory_only_evaluated_count: trajectory_only,
        neither_proxy_evaluable_count: neither,
        velocity_indeterminate_count: velocity_indeterminate,
        trajectory_indeterminate_count: trajectory_indeterminate,
        independent_reference_count: input.independent_references.len(),
        eligible_independent_reference_count: eligible_reference_count,
        pending_review_reference_count: pending_review_count,
        unverified_review_claim_count,
        synthetic_reference_count: synthetic_reference_count,
        unqualified_reference_count: unqualified_reference_count,
        quality_indeterminate_reference_count: quality_indeterminate_reference_count,
        claimed_reference_supporting_count: claimed_supporting,
        claimed_reference_not_supporting_count: claimed_not_supporting,
        claimed_reference_indeterminate_count: claimed_indeterminate,
        independent_reference_assessment: reference_state,
        physical_claim_status,
        sensor_concordance,
        independent_references,
        method_artifacts: input.method_artifacts.iter().map(|x| ArtifactReference {
            tool_name: x.tool_name.clone(),
            tool_version: x.tool_version.clone(),
            output_sha256: x.output_sha256.clone(),
            evidence_ref: x.evidence_ref.clone(),
        }).collect(),
        interpretation_boundary: "proxy concordance and self-declared reference metadata are descriptive only; physical adjudication requires a verified external attestation",
        warnings: vec![
            "No majority vote or weighted consensus is computed across VT, NSR and trajectory signatures.".to_owned(),
            "NSR is profile-level while VT and trajectory proxies are sensor-level; this report preserves the NSR profile signature separately.".to_owned(),
            "Agreement between proxy methods is not independent validation because the methods may share sensor inputs, preprocessing and hydraulic priors.".to_owned(),
            "This v0.1 tool does not verify reviewer identity, blinding, signatures or an external Mycelix attestation. Review and quality fields are self-declared claims only; no reference can adjudicate a physical claim until a trusted attestation verifier is implemented.".to_owned(),
            "Method artifact SHA-256 values are recorded as supplied; this tool does not open or independently verify the referenced report files.".to_owned(),
            "Different tracer/geophysical methods have different detection limits and spatial supports; preserve each reference record and do not treat the summary as a universal gold standard.".to_owned(),
        ],
    })
}

fn proxy_status_name(status: ProxyStatus) -> &'static str {
    match status {
        ProxyStatus::Positive => "positive",
        ProxyStatus::Negative => "negative",
        ProxyStatus::NotEvaluated => "not_evaluated",
        ProxyStatus::Indeterminate => "indeterminate",
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: pf-triangulation-audit <method-evidence-input.json> <new-report.json>",
        )
        .into());
    }
    let bytes = fs::read(&args[1])?;
    let input_sha256 = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    let report = evaluate(&input, input_sha256)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let encoded = serde_json::to_vec_pretty(&report)?;

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| {
            io::Error::new(
                error.kind(),
                format!("refusing overwrite or cannot create report: {error}"),
            )
        })?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "PF triangulation audit complete: sensors={} proxy_disagreements={} independent_refs={} physical_claim_status={}",
        report.sensor_count,
        report.proxy_disagreement_count,
        report.independent_reference_count,
        report.physical_claim_status
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("PF triangulation audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "SYNTHETIC".to_owned(),
            site_id: "SYN1".to_owned(),
            plot_id: "P01".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            event_id: "E001".to_owned(),
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic fixture only".to_owned(),
            nsr_profile_signature: NsrSignature::NonsequentialResponseDetected,
            sensor_evidence: vec![
                SensorMethodEvidence {
                    sensor_id: "D10".to_owned(),
                    depth_cm: 10.0,
                    velocity_threshold_proxy: ProxyStatus::Positive,
                    trajectory_threshold_proxy: ProxyStatus::Positive,
                },
                SensorMethodEvidence {
                    sensor_id: "D30".to_owned(),
                    depth_cm: 30.0,
                    velocity_threshold_proxy: ProxyStatus::Positive,
                    trajectory_threshold_proxy: ProxyStatus::Negative,
                },
                SensorMethodEvidence {
                    sensor_id: "D60".to_owned(),
                    depth_cm: 60.0,
                    velocity_threshold_proxy: ProxyStatus::Negative,
                    trajectory_threshold_proxy: ProxyStatus::NotEvaluated,
                },
                SensorMethodEvidence {
                    sensor_id: "D90".to_owned(),
                    depth_cm: 90.0,
                    velocity_threshold_proxy: ProxyStatus::Indeterminate,
                    trajectory_threshold_proxy: ProxyStatus::Indeterminate,
                },
            ],
            independent_references: vec![IndependentReference {
                reference_id: "TRACER-01".to_owned(),
                event_id: "E001".to_owned(),
                profile_id: "PROFILE-01".to_owned(),
                method: ReferenceMethod::ConservativeTracer,
                outcome: ReferenceOutcome::SupportsBypass,
                review_claim: ReferenceReviewClaim::ClaimsIndependentReviewBlinded,
                quality_status: ReferenceQualityStatus::Qualified,
                reviewer_evidence_ref_claimed: Some("synthetic self-declared review reference".to_owned()),
                evidence_ref: "synthetic tracer truth".to_owned(),
                detection_limit_or_resolution: "arrival detected at lower boundary".to_owned(),
            }],
            method_artifacts: vec![
                ArtifactReference {
                    tool_name: "preferential-flow-eventizer".to_owned(),
                    tool_version: "0.1.0".to_owned(),
                    output_sha256: "b".repeat(64),
                    evidence_ref: "synthetic fixture eventizer output".to_owned(),
                },
                ArtifactReference {
                    tool_name: "trajectory-signature-audit".to_owned(),
                    tool_version: "0.1.0".to_owned(),
                    output_sha256: "c".repeat(64),
                    evidence_ref: "synthetic fixture trajectory output".to_owned(),
                },
                ArtifactReference {
                    tool_name: "profile-flow-audit".to_owned(),
                    tool_version: "0.2.0".to_owned(),
                    output_sha256: "d".repeat(64),
                    evidence_ref: "synthetic fixture NSR output".to_owned(),
                },
            ],
        }
    }

    #[test]
    fn preserves_sensor_level_proxy_agreement_and_disagreement() {
        let report = evaluate(&input(), "e".repeat(64)).unwrap();
        assert_eq!(report.proxy_agree_positive_count, 1);
        assert_eq!(report.proxy_disagreement_count, 1);
        assert_eq!(report.velocity_only_evaluated_count, 1);
        assert_eq!(report.neither_proxy_evaluable_count, 1);
        assert_eq!(report.velocity_indeterminate_count, 1);
        assert_eq!(report.trajectory_indeterminate_count, 1);
        assert_eq!(report.nsr_profile_signature, "nonsequential_response_detected");
    }

    #[test]
    fn self_declared_blinded_review_never_becomes_verified_reference() {
        let report = evaluate(&input(), "e".repeat(64)).unwrap();
        assert_eq!(report.independent_reference_count, 1);
        assert_eq!(report.unverified_review_claim_count, 1);
        assert_eq!(report.eligible_independent_reference_count, 0);
        assert_eq!(
            report.physical_claim_status,
            "not_adjudicated_external_attestation_verifier_required"
        );
        assert_eq!(
            report.independent_references[0].review_verification_status,
            "not_cryptographically_verified"
        );
    }

    #[test]
    fn proxies_alone_never_adjudicate_physical_pathway_claim() {
        let mut data = input();
        data.independent_references.clear();
        let report = evaluate(&data, "f".repeat(64)).unwrap();
        assert_eq!(report.proxy_agree_positive_count, 1);
        assert_eq!(
            report.physical_claim_status,
            "not_adjudicated_external_attestation_verifier_required"
        );
    }

    #[test]
    fn conflicting_reference_claims_are_retained_but_not_adjudicated() {
        let mut data = input();
        data.independent_references.push(IndependentReference {
            reference_id: "ERT-01".to_owned(),
            event_id: "E001".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            method: ReferenceMethod::Ert,
            outcome: ReferenceOutcome::DoesNotSupportBypass,
            review_claim: ReferenceReviewClaim::ClaimsIndependentReviewBlinded,
            quality_status: ReferenceQualityStatus::Qualified,
            reviewer_evidence_ref_claimed: Some("synthetic self-declared review reference".to_owned()),
            evidence_ref: "synthetic ERT reference".to_owned(),
            detection_limit_or_resolution: "spatial resolution 5 cm".to_owned(),
        });
        let report = evaluate(&data, "1".repeat(64)).unwrap();
        assert_eq!(
            report.independent_reference_assessment,
            "unverified_reference_claims_only"
        );
        assert_eq!(report.eligible_independent_reference_count, 0);
        assert_eq!(report.unverified_review_claim_count, 2);
        assert_eq!(
            report.physical_claim_status,
            "not_adjudicated_external_attestation_verifier_required"
        );
    }

    #[test]
    fn self_declared_non_support_claim_is_not_physical_adjudication() {
        let mut data = input();
        data.independent_references[0].outcome = ReferenceOutcome::DoesNotSupportBypass;
        let report = evaluate(&data, "2".repeat(64)).unwrap();
        assert_eq!(
            report.physical_claim_status,
            "not_adjudicated_external_attestation_verifier_required"
        );
        assert_eq!(report.proxy_agree_positive_count, 1);
    }

    #[test]
    fn multiple_self_declared_support_claims_are_not_eligible() {
        let mut data = input();
        data.independent_references.push(IndependentReference {
            reference_id: "DYE-01".to_owned(),
            event_id: "E001".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            method: ReferenceMethod::DyeTracer,
            outcome: ReferenceOutcome::SupportsBypass,
            review_claim: ReferenceReviewClaim::ClaimsIndependentReviewBlinded,
            quality_status: ReferenceQualityStatus::Qualified,
            reviewer_evidence_ref_claimed: Some("synthetic self-declared review reference".to_owned()),
            evidence_ref: "synthetic dye map".to_owned(),
            detection_limit_or_resolution: "image pixel 1 mm".to_owned(),
        });
        let report = evaluate(&data, "3".repeat(64)).unwrap();
        assert_eq!(report.claimed_reference_supporting_count, 2);
        assert_eq!(report.eligible_independent_reference_count, 0);
        assert_eq!(report.unverified_review_claim_count, 2);
        assert_eq!(report.physical_claim_status, "not_adjudicated_external_attestation_verifier_required");
    }

    #[test]
    fn self_declared_indeterminate_claim_is_not_eligible() {
        let mut data = input();
        data.independent_references[0].outcome = ReferenceOutcome::Indeterminate;
        let report = evaluate(&data, "4".repeat(64)).unwrap();
        assert_eq!(
            report.independent_reference_assessment,
            "unverified_reference_claims_only"
        );
        assert_eq!(report.eligible_independent_reference_count, 0);
        assert_eq!(
            report.physical_claim_status,
            "not_adjudicated_external_attestation_verifier_required"
        );
    }

    #[test]
    fn rejects_duplicate_sensor_and_reference_ids() {
        let mut data = input();
        data.sensor_evidence[1].sensor_id = data.sensor_evidence[0].sensor_id.clone();
        assert!(validate(&data).unwrap_err().contains("duplicate sensor_id"));

        let mut data = input();
        data.independent_references.push(IndependentReference {
            reference_id: "TRACER-01".to_owned(),
            event_id: "E001".to_owned(),
            profile_id: "PROFILE-01".to_owned(),
            method: ReferenceMethod::DyeTracer,
            outcome: ReferenceOutcome::SupportsBypass,
            review_claim: ReferenceReviewClaim::ClaimsIndependentReviewBlinded,
            quality_status: ReferenceQualityStatus::Qualified,
            reviewer_evidence_ref_claimed: Some("synthetic self-declared review reference".to_owned()),
            evidence_ref: "duplicate".to_owned(),
            detection_limit_or_resolution: "unknown".to_owned(),
        });
        assert!(validate(&data).unwrap_err().contains("duplicate reference_id"));
    }

    #[test]
    fn rejects_independent_reference_from_different_event_or_profile() {
        let mut data = input();
        data.independent_references[0].event_id = "OTHER-EVENT".to_owned();
        assert!(validate(&data)
            .unwrap_err()
            .contains("does not match the input event/profile"));
    }

    #[test]
    fn rejects_invalid_source_and_artifact_digests() {
        let mut data = input();
        data.source_sha256 = "invalid".to_owned();
        assert!(validate(&data).unwrap_err().contains("source_sha256"));

        let mut data = input();
        data.method_artifacts[0].output_sha256 = "bad".to_owned();
        assert!(validate(&data).unwrap_err().contains("output_sha256"));
    }
}
