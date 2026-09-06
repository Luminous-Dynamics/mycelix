#!/usr/bin/env python3
"""Materialize producer-side quarantine for legacy `get_agent_trust_level`.

Qualification-only transformation: the committed branch keeps product source
unchanged. The exact legacy endpoint before-image must match once or the script
fails closed.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COORDINATOR = (
    ROOT
    / "mycelix-workspace/mycelix-identity/zomes/trust_credential/coordinator/src/lib.rs"
)


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous producer quarantine"
        )
    return text.replace(before, after, 1)


text = COORDINATOR.read_text()

legacy_endpoint = '''/// Get the trust level for an agent, identified by their AgentPubKey.
///
/// Derives the agent's DID from their public key using the standard
/// `did:mycelix:{agent_pubkey}` convention, then queries all non-revoked
/// trust credentials for that DID. Returns the highest trust tier found.
///
/// This is the endpoint called by mycelix-space's observations coordinator
/// for trust-weighted sensor fusion via `CallTargetCell::OtherRole("identity")`.
///
/// Returns the highest `TrustTier` across all active credentials.
/// If no credentials exist, returns `TrustTier::Observer` (lowest tier).
#[hdk_extern]
pub fn get_agent_trust_level(agent: AgentPubKey) -> ExternResult<TrustTier> {
    // Derive DID from AgentPubKey using the same convention as issue_trust_credential
    let did = format!("did:mycelix:{}", agent);

    // Query all non-revoked credentials for this DID
    let anchor = anchor_hash(&format!("subject:{}", did))?;
    let links = get_links(
        LinkQuery::try_new(anchor, LinkTypes::SubjectToCredential)?,
        GetStrategy::default(),
    )?;

    let mut highest_tier = TrustTier::Observer;

    for link in links {
        let ah = ActionHash::try_from(link.target)
            .map_err(|_| wasm_error!(WasmErrorInner::Guest("Invalid link target".into())))?;
        if let Some(record) = get(ah, GetOptions::default())? {
            if let Some(cred) = record
                .entry()
                .to_app_option::<TrustCredential>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                if !cred.revoked {
                    // Take the highest tier across all credentials (compare via min_score)
                    if cred.trust_tier.min_score() > highest_tier.min_score() {
                        highest_tier = cred.trust_tier.clone();
                    }
                }
            }
        }
    }

    Ok(highest_tier)
}
'''

quarantined_endpoint = '''/// Legacy compatibility endpoint for agent trust level.
///
/// This endpoint is deliberately producer-quarantined. Historical linked
/// credential roots do not establish current revocation/expiration state, and
/// current credential proofs do not yet establish cryptographic verification.
/// Until the network-aware `TrustResolutionV1` producer replaces this API, no
/// caller may obtain differentiated trust from this legacy endpoint.
fn quarantined_legacy_agent_trust_level() -> TrustTier {
    TrustTier::Observer
}

#[hdk_extern]
pub fn get_agent_trust_level(_agent: AgentPubKey) -> ExternResult<TrustTier> {
    Ok(quarantined_legacy_agent_trust_level())
}
'''

text = replace_once(text, legacy_endpoint, quarantined_endpoint, "legacy trust-level endpoint")

text = replace_once(
    text,
    '''    fn valid_commitment() -> Vec<u8> {
        let mut c = vec![0u8; 32];
        c[0] = 1;
        c
    }
''',
    '''    fn valid_commitment() -> Vec<u8> {
        let mut c = vec![0u8; 32];
        c[0] = 1;
        c
    }

    #[test]
    fn legacy_agent_trust_level_is_producer_quarantined() {
        assert_eq!(quarantined_legacy_agent_trust_level(), TrustTier::Observer);
    }
''',
    "producer quarantine regression test",
)

start = text.index("pub fn get_agent_trust_level(")
end = text.index("/// Pure verification logic", start)
endpoint = text[start:end]
for forbidden in [
    "get_links(",
    "highest_tier",
    "min_score()",
    "SubjectToCredential",
]:
    if forbidden in endpoint:
        raise SystemExit(f"ERROR: legacy trust endpoint still contains authority lookup: {forbidden}")

for required in [
    "quarantined_legacy_agent_trust_level()",
    "TrustTier::Observer",
    "legacy_agent_trust_level_is_producer_quarantined",
]:
    if required not in text:
        raise SystemExit(f"ERROR: producer quarantine after-image missing: {required}")

COORDINATOR.write_text(text)
print("Materialized legacy Identity agent-trust producer quarantine V1.")
