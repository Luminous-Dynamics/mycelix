#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct ProvenanceCounts {
    current_constitution: usize,
    bootstrap_root_adoption: usize,
    bootstrap_root: usize,
    operational_coverage_policy: usize,
    operational_context_policy: usize,
    source_head: usize,
    witness_observation: usize,
    witness_trust_binding: usize,
    authority_state_transition: usize,
    control_plane_freshness: usize,
    operational_freshness: usize,
}

impl ProvenanceCounts {
    fn observe(&mut self, role: EvidenceLeaseRole) {
        match role {
            EvidenceLeaseRole::CurrentConstitution => self.current_constitution += 1,
            EvidenceLeaseRole::BootstrapRootAdoption => self.bootstrap_root_adoption += 1,
            EvidenceLeaseRole::BootstrapRoot => self.bootstrap_root += 1,
            EvidenceLeaseRole::OperationalCoveragePolicy => self.operational_coverage_policy += 1,
            EvidenceLeaseRole::OperationalContextPolicy => self.operational_context_policy += 1,
            EvidenceLeaseRole::SourceHead => self.source_head += 1,
            EvidenceLeaseRole::WitnessObservation => self.witness_observation += 1,
            EvidenceLeaseRole::WitnessTrustBinding => self.witness_trust_binding += 1,
            EvidenceLeaseRole::AuthorityStateTransition => self.authority_state_transition += 1,
            EvidenceLeaseRole::ControlPlaneFreshness => self.control_plane_freshness += 1,
            EvidenceLeaseRole::OperationalFreshness => self.operational_freshness += 1,
        }
    }

    fn total(self) -> usize {
        self.current_constitution
            + self.bootstrap_root_adoption
            + self.bootstrap_root
            + self.operational_coverage_policy
            + self.operational_context_policy
            + self.source_head
            + self.witness_observation
            + self.witness_trust_binding
            + self.authority_state_transition
            + self.control_plane_freshness
            + self.operational_freshness
    }
}

fn qualify_closed_provenance(
    contributions: &[EvidenceLeaseContribution],
    expected: ProvenanceCounts,
    now: u64,
) -> ExternResult<QualifiedEvidenceLeaseManifest> {
    let mut observed = ProvenanceCounts::default();
    for contribution in contributions {
        observed.observe(contribution.role);
    }
    if observed != expected || observed.total() != contributions.len() {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "evidence provenance role/cardinality closure denied: observed {observed:?}, expected {expected:?}"
        ))));
    }
    qualify_evidence_lease_manifest(contributions, now).map_err(|error| {
        lease_error("canonical evidence provenance manifest qualification denied", error)
    })
}

#[hdk_extern]
pub fn resolve_current_operational_freshness(
    subject: AuthoritySubjectRef,
) -> ExternResult<CurrentOperationalFreshnessAuditReceipt> {
    subject.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "invalid operational authority subject: {error}"
        )))
    })?;

    let resolved_root = resolve_root()?;
    let root = &resolved_root.root;
    let policies = resolve_operational_policies(&subject)?;
    let control_plane = resolve_control_plane_freshness(root, &subject)?;

    let context_now = now_ms()?;
    policies
        .lease
        .validate_at(context_now)
        .map_err(|error| lease_error("operational policy lease expired before #116", error))?;
    resolved_root
        .lease
        .validate_at(context_now)
        .map_err(|error| lease_error("bootstrap-root evidence lease expired before #116", error))?;
    let root_lease = resolved_root.lease.clone();
    let policy_control_lease = intersect_leases(
        &policies.lease,
        &control_plane.lease,
        context_now,
        "policy/control-plane lease intersection denied",
    )?;
    let mut composition_lease = intersect_leases(
        &root_lease,
        &policy_control_lease,
        context_now,
        "root/policy/control-plane lease intersection denied",
    )?;
    let context = qualify_operational_policy_context(
        root,
        &subject,
        &policies.evidence.context,
        &policies.evidence.coverage,
        &control_plane.qualified,
        context_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "operational policy-context qualification denied: {error}"
        )))
    })?;

    let plan: OperationalProbePlan = call_local(
        EVIDENCE_PLAN_ZOME,
        "plan_operational_probe",
        OperationalProbePlanRequest {
            target_subject: subject.clone(),
            context_policy_digest: context.context_policy_digest(),
            coverage_policy_digest: context.coverage_policy_digest(),
        },
    )?;
    let evidence = resolve_evidence(plan.probe_action, Some(&subject))?;

    let current_now = now_ms()?;
    composition_lease = intersect_leases(
        &composition_lease,
        &evidence.lease,
        current_now,
        "control-plane/policy/operational evidence lease intersection denied",
    )?;
    let current = qualify_operational_subject_freshness(
        &context,
        &evidence.challenge,
        &evidence.source_head,
        &evidence.witnesses,
        &evidence.trust_bindings,
        &evidence.transitions,
        current_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "operational freshness qualification denied: {error}"
        )))
    })?;
    let semantic_freshness = current.to_verified_freshness();
    semantic_freshness.validate_at(current_now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "qualified operational freshness is not currently usable: {error}"
        )))
    })?;
    let semantic_lease = EvidenceLease::new(
        semantic_freshness.verified_at_ms,
        semantic_freshness.lease_until_ms,
        current_now,
    )
    .map_err(|error| lease_error("operational semantic freshness lease denied", error))?;
    composition_lease = intersect_leases(
        &composition_lease,
        &semantic_lease,
        current_now,
        "dynamic evidence/semantic authority lease intersection denied",
    )?;

    let coverage_digest = policies.evidence.coverage.policy.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute operational coverage-policy provenance identity: {error}"
        )))
    })?;
    let context_digest = policies.evidence.context.policy.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute operational context-policy provenance identity: {error}"
        )))
    })?;
    let mut contributions = Vec::with_capacity(
        6 + control_plane.contributions.len() + evidence.contributions.len(),
    );
    contributions.extend(resolved_root.contributions.iter().cloned());
    contributions.push(contribution(
        EvidenceLeaseRole::OperationalCoveragePolicy,
        coverage_digest,
        POLICY_IDENTITY_PROFILE,
        &policies.evidence.coverage.verification_ref,
        policies.lease.clone(),
    ));
    contributions.push(contribution(
        EvidenceLeaseRole::OperationalContextPolicy,
        context_digest,
        CONTEXT_POLICY_PROFILE,
        &policies.evidence.context.verification_ref,
        policies.lease.clone(),
    ));
    contributions.extend(control_plane.contributions);
    contributions.extend(evidence.contributions);
    contributions.push(contribution(
        EvidenceLeaseRole::OperationalFreshness,
        current.evidence_digest(),
        current.evidence_profile(),
        &semantic_freshness.verification_ref,
        composition_lease.clone(),
    ));

    let expected = ProvenanceCounts {
        current_constitution: 1,
        bootstrap_root_adoption: 1,
        bootstrap_root: 1,
        operational_coverage_policy: 1,
        operational_context_policy: 1,
        source_head: control_plane.probe_count + 1,
        witness_observation: control_plane.witness_count + evidence.witnesses.len(),
        witness_trust_binding: control_plane.witness_count + evidence.trust_bindings.len(),
        authority_state_transition: control_plane.transition_count + evidence.transitions.len(),
        control_plane_freshness: control_plane.probe_count,
        operational_freshness: 1,
    };

    // Close currentness provenance completely before the final external
    // constitutional authority observation. This keeps the final constitution
    // read as the last authority-plane check before deployment qualification.
    let provenance_now = now_ms()?;
    composition_lease.validate_at(provenance_now).map_err(|error| {
        lease_error(
            "global evidence lease expired before canonical provenance qualification",
            error,
        )
    })?;
    let provenance = qualify_closed_provenance(&contributions, expected, provenance_now)?;
    if provenance.aggregate_lease() != &composition_lease {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "canonical provenance manifest aggregate does not equal the computed global evidence lease"
                .into(),
        )));
    }

    // Only after #192 is closed do we re-observe the binding constitution.
    let final_constitution = resolve_binding_current_constitution()?;
    ensure_same_constitution(&resolved_root.constitution, &final_constitution)?;
    let constitution_now = now_ms()?;
    let final_constitution_receipt =
        current_constitution_receipt(&final_constitution, constitution_now)?;
    let final_constitution_context =
        qualify_binding_constitution_context(&final_constitution_receipt, root, constitution_now)
            .map_err(|error| {
                wasm_error!(WasmErrorInner::Guest(format!(
                    "final binding constitution context denied: {error}"
                )))
            })?;

    // Host DNA is a local deployment fact, not an authority-plane source. Observe
    // it after the final constitution fence, then choose a fresh qualification
    // time so neither host nor constitution evidence can appear future-dated.
    let host_dna_hash = dna_info()?.hash.to_string();
    let host_dna_observed_at = now_ms()?;
    let deployment_now = now_ms()?;

    provenance
        .aggregate_lease()
        .validate_at(deployment_now)
        .map_err(|error| {
            lease_error(
                "canonical provenance expired after final constitution recheck",
                error,
            )
        })?;

    let requested_deployment_until = deployment_now
        .checked_add(DEPLOYMENT_RETURN_LEASE_MS)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "deployment-fence lease overflow".into(),
            ))
        })?
        .min(final_constitution_context.valid_until_ms());
    let final_evidence_lease = provenance
        .aggregate_lease()
        .cap_valid_until(requested_deployment_until, deployment_now)
        .map_err(|error| lease_error("final evidence lease cap denied", error))?;
    let host_context = HostLocalDnaContext::from_host_observation(
        host_dna_hash,
        host_dna_observed_at,
        final_evidence_lease.valid_until_ms,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "host local-DNA context denied: {error}"
        )))
    })?;

    let deployment = qualify_operational_freshness_for_deployment_with_constitution_and_provenance(
        &current,
        &final_constitution_context,
        &host_context,
        &provenance,
        deployment_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constitution/provenance-bound deployment freshness denied: {error}"
        )))
    })?;
    if deployment.valid_until_ms() > final_evidence_lease.valid_until_ms
        || deployment.verified_at_ms() < final_evidence_lease.verified_at_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "constitution/provenance-bound deployment widened or predates the final evidence lease"
                .into(),
        )));
    }
    if deployment.provenance_manifest_digest() != Digest32(provenance.manifest_digest())
        || deployment.provenance_manifest_profile() != provenance.manifest_profile()
        || deployment.provenance_contributor_count() != provenance.contributor_count()
        || deployment.composition_evidence_verified_at_ms()
            != provenance.aggregate_lease().verified_at_ms
        || deployment.composition_evidence_valid_until_ms()
            != provenance.aggregate_lease().valid_until_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "deployment result does not echo the exact canonical composition manifest".into(),
        )));
    }
    if deployment.binding_constitution_context_digest()
        != final_constitution_context.context_digest()
        || deployment.binding_constitution_context_profile()
            != final_constitution_context.context_profile()
        || deployment.binding_constitution_verification_ref()
            != final_constitution_context.verification_ref()
        || deployment.binding_constitution_verified_at_ms()
            != final_constitution_context.verified_at_ms()
        || deployment.binding_constitution_valid_until_ms()
            != final_constitution_context.valid_until_ms()
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "deployment result does not echo the exact final binding constitution context".into(),
        )));
    }

    let freshness = deployment.to_verified_freshness();
    freshness.validate_at(deployment_now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "constitution/provenance-bound deployment freshness is not reusable: {error}"
        )))
    })?;
    if freshness.lease_until_ms > final_evidence_lease.valid_until_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "wire freshness lease exceeds the final deployment evidence horizon".into(),
        )));
    }

    Ok(CurrentOperationalFreshnessAuditReceipt {
        protocol: RUNTIME_PROTOCOL.into(),
        subject,
        bootstrap_root_digest: current.root_qualification_digest(),
        bootstrap_root_profile: current.root_qualification_profile().into(),
        operational_context_digest: current.operational_context_digest(),
        operational_context_profile: current.operational_context_profile().into(),
        authority_digest: current.authority_digest(),
        authority_profile: current.authority_profile().into(),
        evidence_digest: current.evidence_digest(),
        evidence_profile: current.evidence_profile().into(),
        composition_evidence_manifest_digest: deployment.provenance_manifest_digest(),
        composition_evidence_manifest_profile: deployment.provenance_manifest_profile().into(),
        composition_evidence_contributor_count: deployment.provenance_contributor_count(),
        composition_evidence_lease_protocol: mycelix_authority_evidence_lease::PROTOCOL_VERSION
            .into(),
        composition_evidence_verified_at_ms: deployment.composition_evidence_verified_at_ms(),
        composition_evidence_valid_until_ms: deployment.composition_evidence_valid_until_ms(),
        binding_constitution_context_digest: deployment.binding_constitution_context_digest(),
        binding_constitution_context_profile: deployment
            .binding_constitution_context_profile()
            .into(),
        binding_constitution_verification_ref: deployment
            .binding_constitution_verification_ref()
            .into(),
        binding_constitution_verified_at_ms: deployment.binding_constitution_verified_at_ms(),
        binding_constitution_valid_until_ms: deployment.binding_constitution_valid_until_ms(),
        local_dna_hash: deployment.dna_hash().into(),
        constitution_statement_digest: deployment.constitution_statement_digest(),
        constitution_statement_profile: deployment.constitution_statement_profile().into(),
        deployment_authority_digest: deployment.deployment_authority_digest(),
        deployment_authority_profile: deployment.deployment_authority_profile().into(),
        deployment_evidence_digest: deployment.deployment_evidence_digest(),
        deployment_evidence_profile: deployment.deployment_evidence_profile().into(),
        verification_ref: deployment.verification_ref().into(),
        verified_at_ms: deployment.verified_at_ms(),
        lease_until_ms: deployment.valid_until_ms(),
        current_freshness: freshness,
    })
}

#[hdk_extern]
pub fn current_freshness_runtime_status(_: ()) -> ExternResult<CurrentFreshnessRuntimeStatus> {
    Ok(CurrentFreshnessRuntimeStatus {
        protocol: RUNTIME_PROTOCOL.into(),
        composition_only: true,
        root_manifest_provider_grants_authority: false,
        current_constitution_source_is_binding_plane: true,
        constitution_rechecked_after_adoption: true,
        constitution_rechecked_before_return: true,
        root_adoption_proof_verifier_separate: true,
        root_adoption_constructed_locally: true,
        current_constitution_provenance_explicit: true,
        root_adoption_provenance_explicit: true,
        root_provenance_lease_constructed_at_root_boundary: true,
        final_constitution_context_qualified_locally: true,
        plain_constitution_primitives_accepted_by_active_deployment_path: false,
        final_constitution_evidence_bound_into_deployment_evidence: true,
        transition_discovery_grants_authority: false,
        transition_record_proof_verifier_separate: true,
        transition_authority_proof_verifier_separate: true,
        transition_source_bound_from_source_head: true,
        leased_source_head_consumed: true,
        leased_witness_evidence_consumed: true,
        leased_operational_policy_consumed: true,
        transition_leases_intersected: true,
        global_evidence_lease_enforced: true,
        provenance_contributor_set_constructed_locally: true,
        provenance_required_role_cardinality_closed: true,
        provenance_manifest_qualified_locally: true,
        provenance_manifest_matches_global_lease: true,
        provenance_bound_into_deployment_evidence: true,
        deployment_lease_capped_by_global_evidence: true,
        wire_receipt_transport_only: true,
        caller_supplied_positive_currentness_accepted: false,
        wire_receipt_grants_execution_authority: false,
        host_local_dna_derived: true,
        constitutional_dna_cross_checked: true,
        deployment_authority_constructed_locally: true,
        semantic_only_output: false,
        evidence_plan_grants_authority: false,
        latest_dht_record_authority: false,
        provider_positive_object_authority: false,
        probe_authority: false,
        qualification_time_after_evidence: true,
        external_effects_enabled: false,
        operational: false,
    })
}

fn digest_hex(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}
