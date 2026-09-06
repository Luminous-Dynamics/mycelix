fn resolve_transition_lineage(
    source_head: &VerifiedAuthoritySourceHead,
) -> ExternResult<ResolvedTransitionLineage> {
    let attestation = &source_head.attestation;
    let candidates: Vec<AuthorityStateTransition> = call_local(
        TRANSITION_CANDIDATE_PROVIDER_ZOME,
        "discover_transition_candidates",
        TransitionCandidateDiscoveryRequest {
            subject: attestation.subject.clone(),
            authoritative_source_ref: attestation.authoritative_source_ref.clone(),
            head_generation: attestation.head_generation,
            head_transition_digest: attestation.head_transition_digest,
        },
    )?;
    if candidates.is_empty() || candidates.len() > MAX_TRANSITIONS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state transition candidate count must be 1-{MAX_TRANSITIONS}"
        ))));
    }

    let mut verified = Vec::with_capacity(candidates.len());
    let mut aggregate_lease: Option<EvidenceLease> = None;
    for transition in candidates {
        transition.validate().map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "invalid authority-state transition candidate: {error}"
            )))
        })?;
        if transition.subject != attestation.subject {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "transition candidate belongs to another authority subject".into(),
            )));
        }
        if transition.generation > attestation.head_generation {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "transition candidate exceeds independently authenticated source head generation"
                    .into(),
            )));
        }
        let transition_digest = transition.identity_digest().map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "cannot compute authority-state transition identity: {error}"
            )))
        })?;

        let record_proof: VerifiedTransitionRecordProof = call_local(
            TRANSITION_RECORD_PROOF_VERIFIER_ZOME,
            "verify_transition_record_proof",
            TransitionRecordProofVerificationRequest {
                transition: transition.clone(),
                transition_digest,
                transition_profile: TRANSITION_IDENTITY_PROFILE.into(),
            },
        )?;
        let authority_proof: VerifiedTransitionAuthorityProof = call_local(
            TRANSITION_AUTHORITY_PROOF_VERIFIER_ZOME,
            "verify_transition_authority_proof",
            TransitionAuthorityProofVerificationRequest {
                transition: transition.clone(),
                transition_digest,
                transition_profile: TRANSITION_IDENTITY_PROFILE.into(),
            },
        )?;

        // Qualification time follows both proof-verifier calls. The authoritative
        // source identity comes from the independently authenticated source head.
        let now = now_ms()?;
        let qualified = qualify_authority_state_transition(
            &transition,
            &record_proof,
            &authority_proof,
            &attestation.authoritative_source_ref,
            now,
        )
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "authority-state transition qualification denied: {error}"
            )))
        })?;
        let transition_lease = EvidenceLease::new(
            qualified.verified_at_ms(),
            qualified.valid_until_ms(),
            now,
        )
        .map_err(|error| lease_error("qualified transition lease denied", error))?;
        aggregate_lease = Some(match aggregate_lease {
            Some(existing) => intersect_leases(
                &existing,
                &transition_lease,
                now,
                "transition lease intersection denied",
            )?,
            None => transition_lease,
        });
        verified.push(qualified.to_verified_transition());
    }

    let final_now = now_ms()?;
    let lease = aggregate_lease.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "transition lineage produced no evidence lease".into(),
        ))
    })?;
    lease.validate_at(final_now)
        .map_err(|error| lease_error("transition lease expired during composition", error))?;
    Ok(ResolvedTransitionLineage {
        transitions: verified,
        lease,
    })
}

fn resolve_evidence(
    probe_action: ActionHash,
    expected_subject: Option<&AuthoritySubjectRef>,
) -> ExternResult<ResolvedAuthorityEvidence> {
    let challenge = verify_probe(probe_action.clone(), expected_subject)?;
    let challenge_digest = challenge.challenge.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute verified challenge identity: {error}"
        )))
    })?;

    let source: LeasedEvidence<VerifiedAuthoritySourceHead> = call_local(
        SOURCE_HEAD_VERIFIER_ZOME,
        "verify_source_head_leased",
        SourceHeadVerificationRequest { probe_action },
    )?;
    if source.evidence.attestation.challenge_digest != challenge_digest
        || source.evidence.attestation.subject != challenge.challenge.subject
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "leased source head does not bind the coordinator's exact verified challenge".into(),
        )));
    }

    let witness: LeasedEvidence<VerifiedWitnessEvidenceBundle> = call_local(
        WITNESS_VERIFIER_ZOME,
        "verify_witness_evidence_leased",
        WitnessVerificationRequest {
            challenge: challenge.clone(),
            source_head: source.evidence.clone(),
        },
    )?;
    if witness.evidence.witnesses.len() > MAX_WITNESSES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state witness count exceeds {MAX_WITNESSES}"
        ))));
    }
    if witness.evidence.trust_bindings.len() > MAX_TRUST_BINDINGS {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "authority-state trust-binding count exceeds {MAX_TRUST_BINDINGS}"
        ))));
    }

    let lineage = resolve_transition_lineage(&source.evidence)?;
    let final_now = now_ms()?;
    let source_witness_lease = intersect_leases(
        &source.lease,
        &witness.lease,
        final_now,
        "source/witness evidence lease intersection denied",
    )?;
    let lease = intersect_leases(
        &source_witness_lease,
        &lineage.lease,
        final_now,
        "source/witness/transition evidence lease intersection denied",
    )?;

    Ok(ResolvedAuthorityEvidence {
        challenge,
        source_head: source.evidence,
        witnesses: witness.evidence.witnesses,
        trust_bindings: witness.evidence.trust_bindings,
        transitions: lineage.transitions,
        lease,
    })
}

fn resolve_control_plane_freshness(
    root: &mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot,
    target_subject: &AuthoritySubjectRef,
) -> ExternResult<ResolvedControlPlaneFreshness> {
    let root_manifest_digest = root.manifest().identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute qualified bootstrap-root manifest identity: {error}"
        )))
    })?;
    let plan: ControlPlaneProbePlan = call_local(
        EVIDENCE_PLAN_ZOME,
        "plan_control_plane_probes",
        ControlPlaneProbePlanRequest {
            target_subject: target_subject.clone(),
            root_manifest_digest,
        },
    )?;
    if plan.probe_actions.is_empty() || plan.probe_actions.len() > MAX_CONTROL_PLANE_PROBES {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "control-plane probe count must be 1-{MAX_CONTROL_PLANE_PROBES}"
        ))));
    }

    let mut qualified = Vec::with_capacity(plan.probe_actions.len());
    let mut aggregate_lease: Option<EvidenceLease> = None;
    for probe_action in plan.probe_actions {
        let evidence = resolve_evidence(probe_action, None)?;
        let now = now_ms()?;
        let value = qualify_control_plane_subject_freshness(
            root,
            &evidence.challenge,
            &evidence.source_head,
            &evidence.witnesses,
            &evidence.trust_bindings,
            &evidence.transitions,
            now,
        )
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "control-plane freshness qualification denied: {error}"
            )))
        })?;
        let freshness = value.to_verified_freshness();
        let semantic_lease = EvidenceLease::new(
            freshness.verified_at_ms,
            freshness.lease_until_ms,
            now,
        )
        .map_err(|error| lease_error("control-plane semantic lease denied", error))?;
        let probe_lease = intersect_leases(
            &evidence.lease,
            &semantic_lease,
            now,
            "control-plane evidence/semantic lease intersection denied",
        )?;
        aggregate_lease = Some(match aggregate_lease {
            Some(existing) => intersect_leases(
                &existing,
                &probe_lease,
                now,
                "control-plane probe lease intersection denied",
            )?,
            None => probe_lease,
        });
        qualified.push(value);
    }

    let final_now = now_ms()?;
    let lease = aggregate_lease.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "control-plane qualification produced no evidence lease".into(),
        ))
    })?;
    lease.validate_at(final_now).map_err(|error| {
        lease_error(
            "control-plane evidence lease expired during composition",
            error,
        )
    })?;
    Ok(ResolvedControlPlaneFreshness { qualified, lease })
}
