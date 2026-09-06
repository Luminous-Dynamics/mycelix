#[hdk_extern]
pub fn resolve_current_operational_freshness(
    subject: AuthoritySubjectRef,
) -> ExternResult<VerifiedCurrentOperationalFreshnessReceipt> {
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
    let root_lease = EvidenceLease::new(root.verified_at_ms(), root.valid_until_ms(), context_now)
        .map_err(|error| lease_error("bootstrap-root evidence lease denied", error))?;
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

    let host_dna_hash = dna_info()?.hash.to_string();
    let host_dna_observed_at = now_ms()?;

    let final_constitution = resolve_binding_current_constitution()?;
    ensure_same_constitution(&resolved_root.constitution, &final_constitution)?;
    let final_now = now_ms()?;
    composition_lease.validate_at(final_now).map_err(|error| {
        lease_error(
            "global evidence lease expired before deployment qualification",
            error,
        )
    })?;

    let requested_deployment_until = final_now
        .checked_add(DEPLOYMENT_RETURN_LEASE_MS)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "deployment-fence lease overflow".into(),
            ))
        })?;
    let final_evidence_lease = composition_lease
        .cap_valid_until(requested_deployment_until, final_now)
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
    let constitution_statement_digest =
        constitution_digest_to_core(final_constitution.statement_digest);
    let deployment = qualify_operational_freshness_for_deployment(
        &current,
        &final_constitution.dna_hash,
        constitution_statement_digest,
        &host_context,
        final_now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "deployment-bound operational freshness denied: {error}"
        )))
    })?;
    if deployment.valid_until_ms() > final_evidence_lease.valid_until_ms
        || deployment.verified_at_ms() < final_evidence_lease.verified_at_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "deployment qualification widened or predates the global evidence lease".into(),
        )));
    }

    let freshness = deployment.to_verified_freshness();
    freshness.validate_at(final_now).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "deployment-bound freshness is not reusable: {error}"
        )))
    })?;
    if freshness.lease_until_ms > final_evidence_lease.valid_until_ms {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "wire freshness lease exceeds the global evidence horizon".into(),
        )));
    }

    Ok(VerifiedCurrentOperationalFreshnessReceipt {
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
        composition_evidence_lease_protocol: mycelix_authority_evidence_lease::PROTOCOL_VERSION
            .into(),
        composition_evidence_verified_at_ms: final_evidence_lease.verified_at_ms,
        composition_evidence_valid_until_ms: final_evidence_lease.valid_until_ms,
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
        transition_discovery_grants_authority: false,
        transition_record_proof_verifier_separate: true,
        transition_authority_proof_verifier_separate: true,
        transition_source_bound_from_source_head: true,
        leased_source_head_consumed: true,
        leased_witness_evidence_consumed: true,
        leased_operational_policy_consumed: true,
        transition_leases_intersected: true,
        global_evidence_lease_enforced: true,
        deployment_lease_capped_by_global_evidence: true,
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
