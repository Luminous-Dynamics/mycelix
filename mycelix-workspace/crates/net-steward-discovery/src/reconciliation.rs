use net_steward_schema::{
    CapabilityScope, ClaimEnvelope, ClaimVerificationStatus, ConflictKind, ConflictingClaimRef,
    DidAgentBinding, DidBindingResolver, PeerKind, PeerNodeStatus, PeerPostureClaim,
    PeerTrustStatus, PostureConflict, PostureSummary, SignatureScheme,
};

pub fn reconcile_claims(
    claims: Vec<ClaimEnvelope<PeerPostureClaim>>,
    local_time_ms: u64,
    revocations: Vec<String>,
    bindings: Vec<DidAgentBinding>,
) -> (Vec<PeerNodeStatus>, Vec<PostureConflict>, u32, u32, u32) {
    let mut accepted_count = 0;
    let mut rejected_count = 0;
    let mut stale_count = 0;

    let mut filtered_claims = Vec::new();

    for envelope in claims {
        // 1. Structural checks & revocation filters
        if revocations.contains(&envelope.issuer_did) || revocations.contains(&envelope.envelope_id)
        {
            rejected_count += 1;
            continue;
        }

        if envelope.verification_status == ClaimVerificationStatus::InvalidSignature {
            rejected_count += 1;
            continue;
        }

        if envelope.payload.expires_at_unix_ms <= envelope.payload.issued_at_unix_ms {
            rejected_count += 1;
            continue;
        }

        // Verify DID scope if binding exists
        if let Some(binding) = bindings
            .iter()
            .find(|b| b.issuer_did == envelope.issuer_did)
        {
            if binding.revoked {
                rejected_count += 1;
                continue;
            }
            // Reject if claiming NixOS metadata but scope is missing
            let has_unauthorized_scope = !envelope.payload.evidence_refs.is_empty()
                && envelope.payload.evidence_refs.iter().any(|ref_str| {
                    ref_str == "nixos_profile_collector"
                        && !binding
                            .claim_scopes
                            .contains(&CapabilityScope::NixosGenerationObserve)
                });

            if has_unauthorized_scope {
                rejected_count += 1;
                continue;
            }
        }

        // Check if expired dynamically
        if envelope.payload.expires_at_unix_ms < local_time_ms {
            stale_count += 1;
            // Kept as historical but flagged
        }

        accepted_count += 1;
        filtered_claims.push(envelope);
    }

    // Group filtered claims by subject node ID
    use std::collections::HashMap;
    let mut claims_by_node: HashMap<String, Vec<ClaimEnvelope<PeerPostureClaim>>> = HashMap::new();
    for claim in filtered_claims {
        claims_by_node
            .entry(claim.payload.subject_node_id.clone())
            .or_default()
            .push(claim);
    }

    let mut reconciled_peers = Vec::new();
    let mut conflicts = Vec::new();

    for (node_id, node_claims) in claims_by_node {
        if node_claims.is_empty() {
            continue;
        }

        // Check if there are conflicting posture summaries or signatures
        let first_posture = node_claims[0].payload.posture_summary;
        let has_posture_conflict = node_claims
            .iter()
            .any(|c| c.payload.posture_summary != first_posture);

        // Calculate trust status and fresh status for node
        let mut best_trust = PeerTrustStatus::UnsignedPeer;
        let mut total_staleness_ms = 0;
        let mut display_name = format!("Peer Node {}", node_id);
        let mut peer_kind = PeerKind::Server;
        let mut evidence_refs = Vec::new();
        let mut capsule_refs = Vec::new();
        let mut issuer_did = "unknown".to_string();

        for envelope in &node_claims {
            let is_stale = envelope.payload.expires_at_unix_ms < local_time_ms;
            let current_trust = if is_stale {
                PeerTrustStatus::StalePeer
            } else if envelope.signature_scheme == SignatureScheme::SimulatedEnvelope
                || envelope.verification_status == ClaimVerificationStatus::VerifiedSignature
            {
                PeerTrustStatus::SignedPeer
            } else {
                PeerTrustStatus::UnsignedPeer
            };

            if current_trust == PeerTrustStatus::SignedPeer
                && best_trust != PeerTrustStatus::LocalSelf
            {
                best_trust = PeerTrustStatus::SignedPeer;
            }

            if envelope.payload.issued_at_unix_ms < local_time_ms {
                total_staleness_ms =
                    total_staleness_ms.max(local_time_ms - envelope.payload.issued_at_unix_ms);
            }

            evidence_refs.extend(envelope.payload.evidence_refs.clone());
            capsule_refs.extend(envelope.payload.capsule_refs.clone());
            issuer_did = envelope.issuer_did.clone();

            if envelope.payload.subject_node_id == "forge-server" {
                display_name = "Forge Build Server".to_string();
                peer_kind = PeerKind::Server;
            } else if envelope.payload.subject_node_id == "luminous-router" {
                display_name = "Primary Border Router".to_string();
                peer_kind = PeerKind::Gateway;
                best_trust = PeerTrustStatus::LocalSelf;
            }
        }

        evidence_refs.sort();
        evidence_refs.dedup();
        capsule_refs.sort();
        capsule_refs.dedup();

        let mut unique_signers = std::collections::HashSet::new();
        let mut unique_devices = std::collections::HashSet::new();
        let mut unique_bound_fresh_signers = std::collections::HashSet::new();
        for envelope in &node_claims {
            let is_expired = envelope.payload.expires_at_unix_ms < local_time_ms;
            if is_expired {
                continue;
            }

            let mut is_did_revoked = revocations.contains(&envelope.issuer_did);
            let mut envelope_device_id = None;
            let mut envelope_has_valid_binding = false;
            if let Some(b) = bindings
                .iter()
                .find(|b| b.issuer_did == envelope.issuer_did)
            {
                if b.revoked {
                    is_did_revoked = true;
                }
                let has_required_scope = b
                    .claim_scopes
                    .contains(&net_steward_schema::CapabilityScope::SecurityPostureObserve);
                if !b.revoked && has_required_scope {
                    envelope_has_valid_binding = true;
                }
                envelope_device_id = b.device_id.clone();
            }

            if envelope.verification_status == ClaimVerificationStatus::VerifiedSignature
                && envelope.signature_scheme != SignatureScheme::SimulatedEnvelope
                && !is_did_revoked
            {
                let device_allowed = match &envelope_device_id {
                    Some(dev) => unique_devices.insert(dev.clone()),
                    None => true,
                };
                if device_allowed {
                    unique_signers.insert(envelope.issuer_did.clone());
                    if envelope_has_valid_binding {
                        unique_bound_fresh_signers.insert(envelope.issuer_did.clone());
                    }
                }
            }

            if let Some(ref sigs) = envelope.signatures {
                for sig in sigs {
                    let mut is_sig_did_revoked = revocations.contains(&sig.issuer_did);
                    let mut sig_device_id = None;
                    let mut sig_has_valid_binding = false;
                    if let Some(b) = bindings.iter().find(|b| b.issuer_did == sig.issuer_did) {
                        if b.revoked {
                            is_sig_did_revoked = true;
                        }
                        let has_required_scope = b
                            .claim_scopes
                            .contains(&net_steward_schema::CapabilityScope::SecurityPostureObserve);
                        if !b.revoked && has_required_scope {
                            sig_has_valid_binding = true;
                        }
                        sig_device_id = b.device_id.clone();
                    }
                    if sig.signature_scheme != SignatureScheme::SimulatedEnvelope
                        && !is_sig_did_revoked
                    {
                        let device_allowed = match &sig_device_id {
                            Some(dev) => unique_devices.insert(dev.clone()),
                            None => true,
                        };
                        if device_allowed {
                            unique_signers.insert(sig.issuer_did.clone());
                            if sig_has_valid_binding {
                                unique_bound_fresh_signers.insert(sig.issuer_did.clone());
                            }
                        }
                    }
                }
            }
        }

        let consensus_reached = unique_bound_fresh_signers.len() >= 2 && !has_posture_conflict;
        let single_bound_fresh = unique_bound_fresh_signers.len() == 1 && !has_posture_conflict;

        let final_trust = if has_posture_conflict {
            PeerTrustStatus::ConflictingClaims
        } else if best_trust == PeerTrustStatus::LocalSelf {
            PeerTrustStatus::LocalSelf
        } else if consensus_reached {
            PeerTrustStatus::FederatedConsensus
        } else if single_bound_fresh {
            PeerTrustStatus::VerifiedBoundFresh
        } else {
            best_trust
        };

        let reconciled_status = PeerNodeStatus {
            peer_id: format!("peer-{}", node_id),
            node_id: node_id.clone(),
            display_name,
            peer_kind,
            posture_summary: if has_posture_conflict {
                PostureSummary::Unknown
            } else {
                first_posture
            },
            trust_status: final_trust,
            last_seen_unix_ms: local_time_ms - total_staleness_ms.min(local_time_ms),
            staleness_ms: total_staleness_ms,
            evidence_refs,
            capsule_refs,
            claimed_by: issuer_did,
        };

        reconciled_peers.push(reconciled_status);

        if has_posture_conflict {
            let conflict_id = format!("conflict-posture-{}", node_id);
            let claim_refs = node_claims
                .iter()
                .map(|c| {
                    let is_stale = c.payload.expires_at_unix_ms < local_time_ms;
                    let trust = if is_stale {
                        PeerTrustStatus::StalePeer
                    } else if c.signature_scheme == SignatureScheme::SimulatedEnvelope {
                        PeerTrustStatus::SignedPeer
                    } else {
                        PeerTrustStatus::UnsignedPeer
                    };

                    ConflictingClaimRef {
                        claim_id: c.payload.claim_id.clone(),
                        issuer_did: c.issuer_did.clone(),
                        value_summary: format!("{:?}", c.payload.posture_summary),
                        trust_status: trust,
                    }
                })
                .collect();

            conflicts.push(PostureConflict {
                conflict_id,
                subject_node_id: node_id,
                conflict_kind: ConflictKind::PostureSeverityConflict,
                claims: claim_refs,
                recommended_display_status: PeerTrustStatus::ConflictingClaims,
                operator_summary:
                    "Conflicting claims discovered regarding host posture health rating."
                        .to_string(),
            });
        }
    }

    (
        reconciled_peers,
        conflicts,
        accepted_count,
        rejected_count,
        stale_count,
    )
}

pub fn reconcile_claims_with_resolver<R: DidBindingResolver>(
    claims: Vec<ClaimEnvelope<PeerPostureClaim>>,
    local_time_ms: u64,
    revocations: Vec<String>,
    resolver: &R,
) -> (Vec<PeerNodeStatus>, Vec<PostureConflict>, u32, u32, u32) {
    let mut resolved_bindings = Vec::new();
    for claim in &claims {
        if let Ok(Some(binding)) = resolver.resolve_binding(&claim.issuer_did) {
            resolved_bindings.push(binding);
        }
    }
    reconcile_claims(claims, local_time_ms, revocations, resolved_bindings)
}
