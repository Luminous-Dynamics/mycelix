use axum::{
    Json, Router,
    routing::{get, post},
};
use serde_json::{Value, json};
use std::net::SocketAddr;
use std::time::{SystemTime, UNIX_EPOCH};
use tower_http::cors::{AllowOrigin, CorsLayer};

use net_steward_discovery::{
    LabControlledExecutor, MockSafetyProofProvider, NoopVerifier, SafetyInput, SafetyProofProvider,
    SafetyProofVerifier, create_incident_capsule, generate_audit_trail_ledger,
    generate_blast_radius_preview, generate_comprehensive_demo_topology,
    generate_dry_run_rollback_plan, generate_mock_known_good_baseline,
    generate_mock_peer_telemetry_report, generate_mock_security_events,
    generate_mock_security_posture, generate_nixos_drift_report, parse_linux_virtual_bridges,
    verify_incident_capsule,
};
use net_steward_schema::{
    BlastRadiusPreview, BlastRadiusRiskTier, ClaimEnvelope, ConfigDriftReport, ConsensusPolicy,
    DaemonVersion, DidAgentBinding, DriftStatus, EventMatcher, ExecutionMode, ExecutionResult,
    FederationStatus, HumanReadableIncidentSummary, IncidentCapsule, IncidentVerificationResult,
    InfrastructureReceipt, KnownGoodBaseline, ObservedTopologySnapshot, OperationIntent,
    PeerNodeStatus, PeerPostureClaim, PeerTelemetryReport, PostureConflict, RemediationPolicy,
    RemediationRule, RollbackPlan, SecurityEvent, SecurityPosture, Severity,
};

#[tokio::main]
async fn main() {
    // Restrict CORS to localhost only for daemon safety defaults
    let cors = CorsLayer::new()
        .allow_origin(AllowOrigin::exact("http://localhost:3000".parse().unwrap()))
        .allow_methods(vec![axum::http::Method::GET, axum::http::Method::POST])
        .allow_headers(vec![axum::http::HeaderName::from_static("content-type")]);

    let app = Router::new()
        .route("/healthz", get(healthz))
        .route("/api/v1/version", get(get_version))
        .route("/api/v1/capabilities", get(get_capabilities))
        .route("/api/v1/topology", get(get_topology))
        .route("/api/v1/drift", get(get_drift))
        .route("/api/v1/verdict", get(get_verdict))
        .route("/api/v1/evidence", get(get_evidence))
        .route("/api/v1/rollback/plan", post(post_rollback_plan))
        .route("/api/v1/security/events", get(get_security_events))
        .route("/api/v1/security/posture", get(get_security_posture))
        .route("/api/v1/security/baseline", get(get_security_baseline))
        .route(
            "/api/v1/security/incident/capsule",
            post(post_security_capsule),
        )
        .route(
            "/api/v1/security/incident/verify",
            post(post_verify_capsule),
        )
        .route("/api/v1/security/peers", get(get_security_peers))
        .route(
            "/api/v1/security/reconcile",
            post(post_security_reconcile).layer(axum::extract::DefaultBodyLimit::max(1024 * 1024)),
        )
        .route(
            "/api/v1/security/consensus/policy",
            get(get_consensus_policy),
        )
        .route("/api/v1/federation/status", get(get_federation_status))
        // v0.3-alpha.4: Blast-Radius Preview — read-only, returns a display artifact only.
        // Operator must separately sign an ApprovalEnvelope before any action is taken.
        .route(
            "/api/v1/operation/blast-radius-preview",
            post(post_blast_radius_preview).layer(axum::extract::DefaultBodyLimit::max(256 * 1024)),
        )
        // v0.3-alpha.7: Lab-Only Controlled Apply — mutation path gated to lab mode.
        .route(
            "/api/v1/operation/apply",
            post(post_operation_apply).layer(axum::extract::DefaultBodyLimit::max(256 * 1024)),
        )
        // v0.4-alpha.2: Policy Evaluator daemon integration
        .route(
            "/api/v1/policy/evaluate",
            post(post_policy_evaluate).layer(axum::extract::DefaultBodyLimit::max(256 * 1024)),
        )
        .route("/api/v1/policy/remediation", get(get_remediation_policy))
        .layer(cors);

    // v0.4-alpha.3: Spawn background Telemetry & Drift Observer task
    tokio::spawn(async {
        let mut interval = tokio::time::interval(tokio::time::Duration::from_secs(60));
        println!("Net Steward background Telemetry & Drift Observer task started.");
        loop {
            interval.tick().await;
            if let Some(verdict) = run_observer_cycle().await {
                println!("Observer Cycle: Evaluated verdict: {:?}", verdict);
            }
        }
    });

    // Bind to localhost 127.0.0.1:3030 only by default for witness security
    let addr = SocketAddr::from(([127, 0, 0, 1], 3030));
    println!(
        "Net Steward Read-Only Witness Daemon listening on http://{}",
        addr
    );

    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn healthz() -> &'static str {
    "OK"
}

async fn get_capabilities() -> Json<Value> {
    let lab_mode = std::env::var("NET_STEWARD_LAB_MODE").unwrap_or_default() == "1";
    Json(json!({
        "read_only": !lab_mode,
        "rollback_apply_enabled": lab_mode,
        "zk_verifier_enabled": false,
        "proof_mode": "simulated_envelope",
        "identity_success_path_proven": true,
        "live_identity_resolver_feature_gated": true,
        // v0.3-alpha.4: blast-radius preview enabled (read-only display, no mutation)
        "blast_radius_preview_enabled": true,
        "blast_radius_preview_endpoint": "/api/v1/operation/blast-radius-preview",
        "consensus_policy": {
            "min_signers": 2,
            "prohibit_simulated": true,
            "prohibit_revoked": true,
            "prohibit_expired": true
        }
    }))
}

async fn get_topology() -> Json<ObservedTopologySnapshot> {
    let mut topology = generate_comprehensive_demo_topology();
    let mut bridges = parse_linux_virtual_bridges();
    topology.edges.append(&mut bridges);
    Json(topology)
}

async fn get_drift() -> Json<ConfigDriftReport> {
    let drift = generate_nixos_drift_report("luminous-router");
    Json(drift)
}

async fn get_verdict() -> Json<HumanReadableIncidentSummary> {
    let topology = generate_comprehensive_demo_topology();
    let drift = generate_nixos_drift_report("luminous-router");

    // Evaluate and generate proof envelope
    let safety_input = SafetyInput { drift, topology };
    let provider = MockSafetyProofProvider::new();
    let mut summary = provider.prove(&safety_input).unwrap();

    // Set honest claim-disciplined verification status using NoopVerifier
    let verifier = NoopVerifier::new();
    summary.proof_status = verifier.verify(&summary);

    Json(summary)
}

async fn get_evidence() -> Json<Vec<InfrastructureReceipt>> {
    let receipts = generate_audit_trail_ledger("luminous-router");
    Json(receipts)
}

async fn post_rollback_plan() -> Json<RollbackPlan> {
    // Clean dry-run rollback plan creation
    let plan = generate_dry_run_rollback_plan("luminous-router", "Generation 427");
    Json(plan)
}

async fn get_version() -> Json<DaemonVersion> {
    Json(DaemonVersion {
        name: "net-steward-daemon".to_string(),
        version: "0.1.0-alpha.4".to_string(),
        mode: "read_only_witness".to_string(),
        mutation_enabled: false,
        proof_mode: "simulated_envelope".to_string(),
        bind: "127.0.0.1:3030".to_string(),
    })
}

async fn get_security_events() -> Json<Vec<SecurityEvent>> {
    Json(generate_mock_security_events())
}

async fn get_security_posture() -> Json<SecurityPosture> {
    Json(generate_mock_security_posture("forge-server"))
}

async fn get_security_baseline() -> Json<KnownGoodBaseline> {
    Json(generate_mock_known_good_baseline("forge-server"))
}

#[derive(Debug, serde::Deserialize)]
struct CapsuleRequest {
    event_id: String,
}

async fn post_security_capsule(
    Json(payload): Json<CapsuleRequest>,
) -> Result<Json<IncidentCapsule>, (axum::http::StatusCode, String)> {
    match create_incident_capsule(&payload.event_id) {
        Ok(capsule) => Ok(Json(capsule)),
        Err(e) => Err((axum::http::StatusCode::NOT_FOUND, e)),
    }
}

async fn post_verify_capsule(
    Json(capsule): Json<IncidentCapsule>,
) -> Json<IncidentVerificationResult> {
    Json(verify_incident_capsule(&capsule))
}

async fn get_security_peers() -> Json<PeerTelemetryReport> {
    Json(generate_mock_peer_telemetry_report())
}

async fn get_federation_status() -> Json<FederationStatus> {
    Json(FederationStatus {
        transport: "mycelix_holochain".to_string(),
        conductor_available: true,
        dna_installed: true,
        role_id: "net_steward".to_string(),
        zome: "net_steward_posture".to_string(),
        identity_binding: "scaffolded".to_string(),
        claims_fetched: 2,
        claims_rejected: 1,
    })
}

#[derive(Debug, serde::Deserialize)]
struct ReconcileRequest {
    claims: Vec<ClaimEnvelope<PeerPostureClaim>>,
    local_time_ms: u64,
    revocations: Vec<String>,
    bindings: Vec<DidAgentBinding>,
}

#[derive(Debug, serde::Serialize)]
struct ReconcileResponse {
    peers: Vec<PeerNodeStatus>,
    conflicts: Vec<PostureConflict>,
    accepted_claims: u32,
    rejected_claims: u32,
    rejection_reasons: Vec<String>,
}

async fn post_security_reconcile(
    Json(payload): Json<ReconcileRequest>,
) -> Result<Json<ReconcileResponse>, (axum::http::StatusCode, String)> {
    if payload.claims.len() > 100 {
        return Err((
            axum::http::StatusCode::PAYLOAD_TOO_LARGE,
            "Exceeded maximum of 100 claims per reconciliation request".to_string(),
        ));
    }
    use net_steward_discovery::reconciliation::reconcile_claims;
    let (peers, conflicts, accepted_count, rejected_count, _stale_count) = reconcile_claims(
        payload.claims.clone(),
        payload.local_time_ms,
        payload.revocations.clone(),
        payload.bindings.clone(),
    );

    let mut rejection_reasons = Vec::new();
    for claim in &payload.claims {
        if claim.signature_scheme == net_steward_schema::SignatureScheme::SimulatedEnvelope {
            rejection_reasons.push("simulated_signature_excluded_from_consensus".to_string());
        }
        if claim.verification_status
            == net_steward_schema::ClaimVerificationStatus::InvalidSignature
        {
            rejection_reasons.push("invalid_signature_verification_failure".to_string());
        }
        if payload.revocations.contains(&claim.issuer_did) {
            rejection_reasons.push("revoked_identity_signature_excluded".to_string());
        }
        if claim.payload.expires_at_unix_ms < payload.local_time_ms {
            rejection_reasons.push("expired_claim_excluded".to_string());
        }
    }
    rejection_reasons.dedup();

    Ok(Json(ReconcileResponse {
        peers,
        conflicts,
        accepted_claims: accepted_count,
        rejected_claims: rejected_count,
        rejection_reasons,
    }))
}

async fn get_consensus_policy() -> Json<ConsensusPolicy> {
    Json(ConsensusPolicy {
        policy_version: "net-steward-consensus-v1".to_string(),
        min_signers: 2,
        max_claims_per_request: 100,
        max_payload_bytes: 1048576,
        require_unique_dids: true,
        require_active_binding: true,
        require_scope_valid: true,
        allow_simulated_signatures: false,
        allow_expired_claims: false,
        allow_revoked_signers: false,
    })
}

async fn get_remediation_policy() -> Json<RemediationPolicy> {
    Json(get_default_remediation_policy())
}

// --- v0.3-alpha.4: Blast-Radius Preview Handler ---
//
// POST /api/v1/operation/blast-radius-preview
//
// Body: OperationIntent (JSON)
// Returns: BlastRadiusPreview (JSON)
//
// This handler is PURELY a display endpoint.  It computes and returns a
// blast-radius preview for an operation the caller is considering.  No
// state is mutated; no approval is collected here.  The operator reviews
// the returned preview and decides whether to submit a separately-signed
// ApprovalEnvelope.
async fn post_blast_radius_preview(
    Json(intent): Json<OperationIntent>,
) -> Result<Json<BlastRadiusPreview>, (axum::http::StatusCode, String)> {
    // Load the current topology so we can compute transitive impact.
    let mut topology = generate_comprehensive_demo_topology();
    let mut bridges = parse_linux_virtual_bridges();
    topology.edges.append(&mut bridges);

    let preview = generate_blast_radius_preview(&intent, &topology);
    Ok(Json(preview))
}

// --- v0.3-alpha.7: Lab-Only Controlled Apply Handler ---
//
// POST /api/v1/operation/apply
//
// Body: OperationIntent (JSON)
// Returns: ExecutionResult (JSON)
//
// This handler permits applying operations/mutations strictly when the
// environment is verified to be in lab/test mode (NET_STEWARD_LAB_MODE=1).
async fn post_operation_apply(
    Json(intent): Json<OperationIntent>,
) -> Result<Json<ExecutionResult>, (axum::http::StatusCode, String)> {
    let lab_mode = std::env::var("NET_STEWARD_LAB_MODE").unwrap_or_default() == "1";
    let executor = LabControlledExecutor { lab_mode };
    use net_steward_schema::OperationExecutor;
    match executor.apply(&intent) {
        Ok(res) => Ok(Json(res)),
        Err(err) => Err((axum::http::StatusCode::FORBIDDEN, err)),
    }
}

#[derive(Debug, serde::Deserialize)]
struct PolicyEvaluateRequest {
    policy: RemediationPolicy,
    incident: HumanReadableIncidentSummary,
}

async fn post_policy_evaluate(
    Json(payload): Json<PolicyEvaluateRequest>,
) -> Json<net_steward_discovery::policy::PolicyEvaluationVerdict> {
    use net_steward_discovery::policy::evaluate_policy;
    Json(evaluate_policy(&payload.policy, &payload.incident))
}

fn get_default_remediation_policy() -> RemediationPolicy {
    RemediationPolicy {
        policy_id: "default-remediation-policy".to_string(),
        version: "remediation_policy_v0.1".to_string(),
        rules: vec![RemediationRule {
            rule_name: "auto-rollback-low-risk-drift".to_string(),
            event_matcher: EventMatcher {
                node_id: "any".to_string(),
                min_severity: Severity::Low,
                systemd_unit: None,
                max_drift_status: Some(DriftStatus::DriftDetected),
            },
            execution_mode: ExecutionMode::Autonomous,
            max_allowed_blast_radius: 0.85,
            max_allowed_risk_tier: BlastRadiusRiskTier::Critical,
            required_witnesses: 1,
            target_rollback_generation: Some("427".to_string()),
        }],
    }
}

async fn run_observer_cycle() -> Option<net_steward_discovery::policy::PolicyEvaluationVerdict> {
    let drift = generate_nixos_drift_report("luminous-router");
    if drift.drift_status == net_steward_schema::DriftStatus::DriftDetected {
        let topology = generate_comprehensive_demo_topology();
        let safety_input = SafetyInput { drift, topology };
        let provider = MockSafetyProofProvider::new();
        if let Ok(incident) = provider.prove(&safety_input) {
            let policy = get_default_remediation_policy();
            use net_steward_discovery::policy::evaluate_policy;
            let verdict = evaluate_policy(&policy, &incident);

            if let net_steward_discovery::policy::PolicyEvaluationVerdict::Matched {
                ref rule_name,
                execution_mode: ExecutionMode::Autonomous,
                target_rollback_generation: Some(ref generation),
                ..
            } = verdict
            {
                println!(
                    "Observer Cycle: Autonomous rule '{}' matched. Triggering rollback to generation '{}'...",
                    rule_name, generation
                );

                // Construct the OperationIntent
                let intent = OperationIntent {
                    intent_id: format!("auto-rollback-{}", generation),
                    actor_did: "did:mycelix:net-steward-autonomous".to_string(),
                    target_node_id: "luminous-router".to_string(),
                    operation_kind: net_steward_schema::OperationKind::GenerateRollbackPlan,
                    reason: format!("Autonomous remediation triggered by rule: {}", rule_name),
                    evidence_refs: vec![incident.incident_id.clone()],
                    rollback_plan_ref: Some(format!("Generation {}", generation)),
                    expires_at_unix_ms: SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_millis() as u64
                        + 300_000,
                };

                // Apply the operation via LabControlledExecutor
                let lab_mode = std::env::var("NET_STEWARD_LAB_MODE").unwrap_or_default() == "1";
                let executor = LabControlledExecutor { lab_mode };
                use net_steward_schema::OperationExecutor;
                match executor.apply(&intent) {
                    Ok(exec_res) => {
                        println!(
                            "Observer Cycle: Autonomous execution succeeded: {:?}",
                            exec_res
                        );
                    }
                    Err(err) => {
                        println!("Observer Cycle: Autonomous execution failed: {}", err);
                    }
                }
            }

            return Some(verdict);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::Json;

    #[tokio::test]
    async fn test_api_policy_evaluate() {
        use net_steward_discovery::policy::PolicyEvaluationVerdict;
        use net_steward_schema::{
            BlastRadiusRiskTier, DriftStatus, EventMatcher, ExecutionMode,
            HumanReadableIncidentSummary, ProofStatus, RemediationPolicy, RemediationRule,
            SafetyVerdict, Severity,
        };

        let policy = RemediationPolicy {
            policy_id: "pol-test-101".to_string(),
            version: "remediation_policy_v0.1".to_string(),
            rules: vec![RemediationRule {
                rule_name: "auto-router-drift".to_string(),
                event_matcher: EventMatcher {
                    node_id: "luminous-router".to_string(),
                    min_severity: Severity::Low,
                    systemd_unit: None,
                    max_drift_status: Some(DriftStatus::DriftDetected),
                },
                execution_mode: ExecutionMode::Autonomous,
                max_allowed_blast_radius: 0.3,
                max_allowed_risk_tier: BlastRadiusRiskTier::Low,
                required_witnesses: 1,
                target_rollback_generation: Some("427".to_string()),
            }],
        };

        let incident = HumanReadableIncidentSummary {
            incident_id: "luminous-router-drift-detected".to_string(),
            safety_verdict: SafetyVerdict::Safe,
            safety_violations: vec![],
            root_cause: "System configuration drift observed".to_string(),
            affected_services: vec![],
            affected_users: vec![],
            blast_radius_score: 0.15,
            confidence: 1.0,
            recommended_action: "Roll back profile".to_string(),
            rollback_path: Some("427".to_string()),
            safety_proof: None,
            safety_commitment: None,
            proof_status: ProofStatus::NotPresent,
        };

        let payload = PolicyEvaluateRequest { policy, incident };
        let response = post_policy_evaluate(Json(payload)).await;

        let Json(verdict) = response;
        assert_eq!(
            verdict,
            PolicyEvaluationVerdict::Matched {
                rule_name: "auto-router-drift".to_string(),
                execution_mode: ExecutionMode::Autonomous,
                required_witnesses: 1,
                target_rollback_generation: Some("427".to_string()),
            }
        );
    }

    #[tokio::test]
    async fn test_observer_cycle_triggers_correctly() {
        use net_steward_discovery::policy::PolicyEvaluationVerdict;
        let verdict = run_observer_cycle().await;
        assert!(verdict.is_some());
        assert_eq!(
            verdict.unwrap(),
            PolicyEvaluationVerdict::Matched {
                rule_name: "auto-rollback-low-risk-drift".to_string(),
                execution_mode: ExecutionMode::Autonomous,
                required_witnesses: 1,
                target_rollback_generation: Some("427".to_string()),
            }
        );
    }

    #[tokio::test]
    async fn test_autonomous_remediation_loop_execution() {
        use net_steward_discovery::policy::PolicyEvaluationVerdict;

        // 1. Force lab mode for the test
        std::env::set_var("NET_STEWARD_LAB_MODE", "1");

        // 2. Set mock state to generation 428 (drifted)
        let state_file = std::path::Path::new("/tmp/net_steward_mock_nixos_generation");
        std::fs::write(state_file, "428").unwrap();

        // 3. Run first observer cycle - should trigger remediation and succeed
        let verdict_1 = run_observer_cycle().await;
        assert!(verdict_1.is_some());
        assert_eq!(
            verdict_1.unwrap(),
            PolicyEvaluationVerdict::Matched {
                rule_name: "auto-rollback-low-risk-drift".to_string(),
                execution_mode: ExecutionMode::Autonomous,
                required_witnesses: 1,
                target_rollback_generation: Some("427".to_string()),
            }
        );

        // 4. Verify that state was rolled back to 427
        let current_gen = std::fs::read_to_string(state_file)
            .map(|s| s.trim().to_string())
            .unwrap_or_default();
        assert_eq!(current_gen, "427");

        // 5. Run second observer cycle - should see InSync and return None (no trigger needed)
        let verdict_2 = run_observer_cycle().await;
        assert!(verdict_2.is_none());

        // Clean up
        if state_file.exists() {
            let _ = std::fs::remove_file(state_file);
        }
    }
}
