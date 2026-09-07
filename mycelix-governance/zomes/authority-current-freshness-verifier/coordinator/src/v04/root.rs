fn call_local<I, O>(zome: &str, function: &str, input: I) -> ExternResult<O>
where
    I: Serialize,
    O: DeserializeOwned,
{
    let response = call(
        CallTargetCell::Local,
        ZomeName::from(zome),
        FunctionName::from(function),
        None,
        ExternIO::encode(input)
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?,
    )?;
    let io = match response {
        ZomeCallResponse::Ok(io) => io,
        other => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "{zome}::{function} unavailable; current freshness fails closed: {other:?}"
            ))));
        }
    };
    io.decode::<O>().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot decode {zome}::{function} response: {error}"
        )))
    })
}

fn now_ms() -> ExternResult<u64> {
    let micros = sys_time()?.as_micros();
    if micros <= 0 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "current time must be positive".into(),
        )));
    }
    Ok(micros as u64 / 1_000)
}

fn lease_error(context: &str, error: impl std::fmt::Display) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{context}: {error}")))
}

fn intersect_leases(
    left: &EvidenceLease,
    right: &EvidenceLease,
    now: u64,
    context: &str,
) -> ExternResult<EvidenceLease> {
    left.intersect(right, now)
        .map_err(|error| lease_error(context, error))
}

fn contribution(
    role: EvidenceLeaseRole,
    evidence_digest: Digest32,
    evidence_profile: &str,
    evidence_ref: &str,
    lease: EvidenceLease,
) -> EvidenceLeaseContribution {
    EvidenceLeaseContribution {
        role,
        evidence_digest: evidence_digest.0,
        evidence_profile: evidence_profile.into(),
        evidence_ref: evidence_ref.into(),
        lease,
    }
}

fn constitution_digest_to_core(digest: ConstitutionDigest32) -> Digest32 {
    Digest32(digest.0)
}

fn verify_constitution_projection(
    current: &VerifiedCurrentConstitutionMirror,
    now: u64,
) -> ExternResult<()> {
    if current.protocol != LEASED_CURRENT_CONSTITUTION_PROTOCOL {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution verifier returned wrong leased-currentness protocol".into(),
        )));
    }
    if current.legacy_constitution_authoritative {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution verifier reported legacy constitution authority".into(),
        )));
    }
    if current.dna_hash.trim().is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution verifier returned empty DNA identity".into(),
        )));
    }
    current.statement.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "binding constitution verifier returned invalid statement: {error}"
        )))
    })?;
    let recomputed = current.statement.digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot digest leased binding constitution: {error}"
        )))
    })?;
    if recomputed != current.statement_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution verifier returned mismatched statement digest".into(),
        )));
    }
    if current.verified_transition_count != 0
        || !current.genesis_currentness_by_amendments_disabled
        || current.transition_currentness_supported
        || current.candidate_discovery_used_for_positive_currentness
        || current.lease_basis != GENESIS_CURRENTNESS_LEASE_BASIS
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution verifier returned unsupported amendment/candidate currentness mode"
                .into(),
        )));
    }
    if current.verification_ref.trim().is_empty()
        || now == 0
        || current.verified_at_ms == 0
        || current.verified_at_ms > now
        || current.valid_until_ms <= now
        || current.valid_until_ms <= current.verified_at_ms
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitution verifier returned invalid or stale lease".into(),
        )));
    }
    Ok(())
}

fn resolve_binding_current_constitution() -> ExternResult<VerifiedCurrentConstitutionMirror> {
    let current: VerifiedCurrentConstitutionMirror = call_local(
        CONSTITUTION_CURRENTNESS_VERIFIER_ZOME,
        CURRENT_CONSTITUTION_FUNCTION,
        (),
    )?;
    let now = now_ms()?;
    verify_constitution_projection(&current, now)?;
    Ok(current)
}

fn ensure_same_constitution(
    expected: &VerifiedCurrentConstitutionMirror,
    observed: &VerifiedCurrentConstitutionMirror,
) -> ExternResult<()> {
    if expected.dna_hash != observed.dna_hash
        || expected.statement_digest != observed.statement_digest
        || expected.statement != observed.statement
        || expected.verified_transition_count != observed.verified_transition_count
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "binding constitutional head changed during current-authority composition".into(),
        )));
    }
    Ok(())
}

fn current_constitution_receipt(
    current: &VerifiedCurrentConstitutionMirror,
    now: u64,
) -> ExternResult<VerifiedCurrentConstitutionReceipt> {
    verify_constitution_projection(current, now)?;
    Ok(VerifiedCurrentConstitutionReceipt {
        protocol_version: CURRENT_CONSTITUTION_RECEIPT_PROTOCOL.into(),
        statement: current.statement.clone(),
        statement_digest: constitution_digest_to_core(current.statement_digest),
        statement_profile: STATEMENT_PROFILE.into(),
        dna_hash: current.dna_hash.clone(),
        verification_ref: current.verification_ref.clone(),
        verified_at_ms: current.verified_at_ms,
        valid_until_ms: current.valid_until_ms,
    })
}

fn resolve_root() -> ExternResult<ResolvedBootstrapRoot> {
    let constitution_before = resolve_binding_current_constitution()?;

    let manifest: AuthorityStateBootstrapRootManifest = call_local(
        ROOT_MANIFEST_PROVIDER_ZOME,
        "resolve_bootstrap_root_manifest",
        (),
    )?;
    manifest.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "bootstrap-root manifest candidate is invalid: {error}"
        )))
    })?;
    let root_manifest_digest = manifest.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute bootstrap-root manifest identity: {error}"
        )))
    })?;

    let adoption_claim = build_adoption_claim(&manifest, &constitution_before.statement)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "bootstrap-root adoption claim denied: {error}"
            )))
        })?;
    let adoption_claim_digest = adoption_claim.identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot compute bootstrap-root adoption claim identity: {error}"
        )))
    })?;
    let adoption_proof: VerifiedBootstrapRootAdoptionProof = call_local(
        ROOT_ADOPTION_PROOF_VERIFIER_ZOME,
        ROOT_ADOPTION_PROOF_VERIFIER_FUNCTION,
        RootAdoptionProofVerificationRequest {
            claim: adoption_claim,
            claim_digest: adoption_claim_digest,
            claim_profile: ADOPTION_CLAIM_PROFILE.into(),
        },
    )?;

    let constitution_after = resolve_binding_current_constitution()?;
    ensure_same_constitution(&constitution_before, &constitution_after)?;

    let now = now_ms()?;
    let qualified_adoption = qualify_bootstrap_root_adoption(
        &manifest,
        &constitution_after.statement,
        &adoption_proof,
        now,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "bootstrap-root adoption qualification denied: {error}"
        )))
    })?;
    if qualified_adoption.claim_digest() != adoption_claim_digest {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "qualified bootstrap-root adoption claim changed during composition".into(),
        )));
    }

    let adoption = qualified_adoption.to_verified_adoption();
    let constitution_receipt = current_constitution_receipt(&constitution_after, now)?;
    let root = qualify_bootstrap_root(&manifest, &constitution_receipt, &adoption, now)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "bootstrap-root qualification denied: {error}"
            )))
        })?;

    if root.manifest().identity_digest().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "cannot recompute qualified bootstrap-root manifest identity: {error}"
        )))
    })? != root_manifest_digest
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "qualified bootstrap-root manifest identity changed during composition".into(),
        )));
    }

    let constitution_lease = EvidenceLease::new(
        constitution_receipt.verified_at_ms,
        constitution_receipt.valid_until_ms,
        now,
    )
    .map_err(|error| lease_error("current-constitution root provenance lease denied", error))?;
    let adoption_lease = EvidenceLease::new(adoption.verified_at_ms, adoption.valid_until_ms, now)
        .map_err(|error| lease_error("bootstrap-root adoption provenance lease denied", error))?;
    let root_lease = EvidenceLease::new(root.verified_at_ms(), root.valid_until_ms(), now)
        .map_err(|error| lease_error("qualified bootstrap-root provenance lease denied", error))?;

    let root_inputs_lease = intersect_leases(
        &constitution_lease,
        &adoption_lease,
        now,
        "constitution/adoption root provenance lease intersection denied",
    )?;
    let root_containment = intersect_leases(
        &root_inputs_lease,
        &root_lease,
        now,
        "root provenance containment intersection denied",
    )?;
    if root_containment != root_lease {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "qualified bootstrap-root lease does not conservatively contain constitution/adoption evidence"
                .into(),
        )));
    }

    let contributions = vec![
        contribution(
            EvidenceLeaseRole::CurrentConstitution,
            constitution_receipt.statement_digest,
            &constitution_receipt.statement_profile,
            &constitution_receipt.verification_ref,
            constitution_lease,
        ),
        contribution(
            EvidenceLeaseRole::BootstrapRootAdoption,
            qualified_adoption.evidence_digest(),
            qualified_adoption.evidence_profile(),
            &adoption.verification_ref,
            adoption_lease,
        ),
        contribution(
            EvidenceLeaseRole::BootstrapRoot,
            root.qualification_digest(),
            root.qualification_profile(),
            root.verification_ref(),
            root_lease.clone(),
        ),
    ];

    Ok(ResolvedBootstrapRoot {
        root,
        constitution: constitution_after,
        lease: root_lease,
        contributions,
    })
}

fn resolve_operational_policies(
    subject: &AuthoritySubjectRef,
) -> ExternResult<LeasedEvidence<OperationalPolicyCandidateBundle>> {
    call_local(
        POLICY_PROVIDER_ZOME,
        "resolve_operational_policy_candidates_leased",
        subject.clone(),
    )
}

fn verify_probe(
    probe_action: ActionHash,
    expected_subject: Option<&AuthoritySubjectRef>,
) -> ExternResult<VerifiedCoverageChallenge> {
    let challenge: VerifiedCoverageChallenge = call_local(
        PROBE_VERIFIER_ZOME,
        "verify_issued_authority_state_probe",
        probe_action,
    )?;
    if let Some(subject) = expected_subject {
        if &challenge.challenge.subject != subject {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "verified authority-state probe belongs to another subject".into(),
            )));
        }
    }
    Ok(challenge)
}
