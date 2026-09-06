#!/usr/bin/env python3
"""Materialize coordinator-wide adoption of the canonical trust-score theorem.

Qualification-only transformation. It edits the CI checkout, not committed
product source, and requires every security-sensitive before-image exactly once
inside the specific extern/helper that owns it.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COORD = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/coordinator"
MANIFEST = COORD / "Cargo.toml"
LIB = COORD / "src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous coordinator transformation"
        )
    return text.replace(before, after, 1)


def replace_in_region(
    text: str,
    start_marker: str,
    end_marker: str,
    before: str,
    after: str,
    label: str,
) -> str:
    start = text.index(start_marker)
    end = text.index(end_marker, start)
    region = text[start:end]
    count = region.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count} in scoped region, "
            "expected exactly 1"
        )
    region = region.replace(before, after, 1)
    return text[:start] + region + text[end:]


manifest = MANIFEST.read_text()
manifest = replace_once(
    manifest,
    'hdk = { workspace = true }\ntrust_credential_integrity = { path = "../integrity" }\n',
    'hdk = { workspace = true }\n'
    'mycelix-trust-score-assertion-policy = { path = "../../../crates/trust-score-assertion-policy" }\n'
    'trust_credential_integrity = { path = "../integrity" }\n',
    "coordinator policy dependency",
)
MANIFEST.write_text(manifest)

text = LIB.read_text()
text = replace_once(
    text,
    "use hdk::prelude::*;\nuse mycelix_zome_helpers as _;\n",
    "use hdk::prelude::*;\n"
    "use mycelix_trust_score_assertion_policy::{\n"
    "    TrustTierBandV1, validate_score_assertion_v1, validate_score_range_v1,\n"
    "};\n"
    "use mycelix_zome_helpers as _;\n",
    "coordinator policy imports",
)

anchor = '''fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    let hash = holo_hash::blake2b_256(anchor_str.as_bytes());
    Ok(EntryHash::from_raw_32(hash.to_vec()))
}
'''
helpers = anchor + '''
/// Explicit local-tier mapping into the canonical structural score-policy band.
fn trust_tier_band_v1(tier: &TrustTier) -> TrustTierBandV1 {
    match tier {
        TrustTier::Observer => TrustTierBandV1::Observer,
        TrustTier::Basic => TrustTierBandV1::Basic,
        TrustTier::Standard => TrustTierBandV1::Standard,
        TrustTier::Elevated => TrustTierBandV1::Elevated,
        TrustTier::Guardian => TrustTierBandV1::Guardian,
    }
}

/// Explicit canonical score-policy band mapping back into the Identity tier.
fn trust_tier_from_band_v1(tier: TrustTierBandV1) -> TrustTier {
    match tier {
        TrustTierBandV1::Observer => TrustTier::Observer,
        TrustTierBandV1::Basic => TrustTier::Basic,
        TrustTierBandV1::Standard => TrustTier::Standard,
        TrustTierBandV1::Elevated => TrustTier::Elevated,
        TrustTierBandV1::Guardian => TrustTier::Guardian,
    }
}

/// Coordinator admission helper. DHT integrity remains authoritative, but every
/// coordinator path rejects malformed score ranges before consuming them.
fn validate_coordinator_score_range(lower: f32, upper: f32) -> ExternResult<TrustTier> {
    let band = validate_score_range_v1(lower, upper).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid trust score range: {error:?}"
        )))
    })?;
    Ok(trust_tier_from_band_v1(band))
}

/// Require a claimed Identity tier to match the canonical midpoint-derived tier.
/// This is structural self-consistency only; it does not establish proof validity
/// or bind a presentation to a source credential.
fn validate_coordinator_score_assertion(
    lower: f32,
    upper: f32,
    claimed_tier: &TrustTier,
) -> ExternResult<()> {
    validate_score_assertion_v1(lower, upper, trust_tier_band_v1(claimed_tier)).map_err(
        |error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid trust score assertion: {error:?}"
            )))
        },
    )
}
'''
text = replace_once(text, anchor, helpers, "coordinator score-policy helpers")

old_range_guard = '''    if input.trust_score_lower < 0.0
        || input.trust_score_upper > 1.0
        || input.trust_score_lower > input.trust_score_upper
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Trust scores must be in [0.0, 1.0] with lower <= upper".into()
        )));
    }
'''

text = replace_in_region(
    text,
    "pub fn issue_trust_credential(",
    "/// Input for issuing a trust credential",
    old_range_guard,
    '''    let trust_tier =
        validate_coordinator_score_range(input.trust_score_lower, input.trust_score_upper)?;
''',
    "issue trust score admission",
)
text = replace_in_region(
    text,
    "pub fn issue_trust_credential(",
    "/// Input for issuing a trust credential",
    '''    // Determine trust tier from the proven range
    let mid_score = (input.trust_score_lower as f64 + input.trust_score_upper as f64) / 2.0;
    let trust_tier = TrustTier::from_score(mid_score);

''',
    "",
    "duplicate issue midpoint derivation",
)

text = replace_in_region(
    text,
    "pub fn self_attest_trust(",
    "/// Input for self-attesting trust",
    old_range_guard,
    '''    let _ = validate_coordinator_score_range(
        input.trust_score_lower,
        input.trust_score_upper,
    )?;
''',
    "self-attestation trust score admission",
)

# A disclosed presentation must at least be numerically self-consistent. This does
# not claim the presentation is bound to the named source credential; that is a
# separate schema/authority invariant.
text = replace_in_region(
    text,
    "pub fn create_presentation(",
    "/// Input for creating a presentation",
    '''    if input.purpose.is_empty() || input.purpose.len() > 2048 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Purpose must be 1-2048 characters".into()
        )));
    }
''',
    '''    if input.purpose.is_empty() || input.purpose.len() > 2048 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Purpose must be 1-2048 characters".into()
        )));
    }
    if input.disclose_range {
        validate_coordinator_score_assertion(
            input.trust_range.lower,
            input.trust_range.upper,
            &input.disclosed_tier,
        )?;
    }
''',
    "presentation disclosed score assertion",
)

text = replace_in_region(
    text,
    "pub fn request_attestation(",
    "/// Input for requesting attestation",
    '''    if let Some(score) = input.min_trust_score {
        if !(0.0..=1.0).contains(&score) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Min trust score must be between 0.0 and 1.0".into()
            )));
        }
    }
''',
    '''    if let Some(score) = input.min_trust_score {
        let _ = validate_coordinator_score_range(score, score)?;
    }
''',
    "attestation request minimum score admission",
)

# Fulfillment must admit the caller-supplied score before any request lookup or
# request-state mutation can consume that evidence.
text = replace_in_region(
    text,
    "pub fn fulfill_attestation(",
    "/// Input for fulfilling an attestation",
    old_range_guard,
    '''    let trust_tier =
        validate_coordinator_score_range(input.trust_score_lower, input.trust_score_upper)?;
''',
    "fulfillment score admission",
)
text = replace_in_region(
    text,
    "pub fn fulfill_attestation(",
    "/// Input for fulfilling an attestation",
    '''    // Verify the provided proof meets request requirements
    let mid_score = (input.trust_score_lower as f64 + input.trust_score_upper as f64) / 2.0;
    let trust_tier = TrustTier::from_score(mid_score);

''',
    '''    // Structural score admission occurred before request lookup/state changes.
    // The checks below only compare the already-admitted structural evidence
    // against this request's requirements; they do not verify STARK semantics.
''',
    "fulfillment duplicate midpoint derivation",
)
text = replace_in_region(
    text,
    "pub fn fulfill_attestation(",
    "/// Input for fulfilling an attestation",
    '''    if let Some(min_score) = req.min_trust_score {
        if input.trust_score_lower < min_score {
''',
    '''    if let Some(min_score) = req.min_trust_score {
        // Historical requests can predate strict finite-score admission.
        let _ = validate_coordinator_score_range(min_score, min_score)?;
        if input.trust_score_lower < min_score {
''',
    "historical request minimum score audit",
)
text = replace_in_region(
    text,
    "pub fn fulfill_attestation(",
    "/// Input for fulfilling an attestation",
    '''    if let Some(ref min_tier) = req.min_tier {
        if (mid_score) < min_tier.min_score() {
''',
    '''    if let Some(ref min_tier) = req.min_tier {
        if trust_tier_band_v1(&trust_tier) < trust_tier_band_v1(min_tier) {
''',
    "fulfillment minimum tier comparison",
)

old_verify = '''    // Validate score range is well-formed
    let range_valid = cred.trust_score_range.lower >= 0.0
        && cred.trust_score_range.upper <= 1.0
        && cred.trust_score_range.lower <= cred.trust_score_range.upper
        && !cred.trust_score_range.lower.is_nan()
        && !cred.trust_score_range.upper.is_nan();

    // Check tier consistency: verify tier matches the score range
    let mid_score =
        (cred.trust_score_range.lower as f64 + cred.trust_score_range.upper as f64) / 2.0;
    let expected_tier = TrustTier::from_score(mid_score);
    let tier_consistent = range_valid && cred.trust_tier == expected_tier;
'''
new_verify = '''    // Structural range/tier diagnostics use the same canonical theorem as
    // coordinator admission and DHT validation. This still does not verify STARK
    // proof semantics; `proof_format_valid` below remains format-only.
    let range_valid = validate_score_range_v1(
        cred.trust_score_range.lower,
        cred.trust_score_range.upper,
    )
    .is_ok();
    let tier_consistent = validate_score_assertion_v1(
        cred.trust_score_range.lower,
        cred.trust_score_range.upper,
        trust_tier_band_v1(&cred.trust_tier),
    )
    .is_ok();
'''
text = replace_in_region(
    text,
    "pub fn verify_credential(",
    "/// Result of credential verification",
    old_verify,
    new_verify,
    "runtime structural verification policy",
)

old_pure = '''    let range_valid = trust_score_range.lower >= 0.0
        && trust_score_range.upper <= 1.0
        && trust_score_range.lower <= trust_score_range.upper
        && !trust_score_range.lower.is_nan()
        && !trust_score_range.upper.is_nan();

    let mid_score = (trust_score_range.lower as f64 + trust_score_range.upper as f64) / 2.0;
    let expected_tier = TrustTier::from_score(mid_score);
    let tier_consistent = range_valid && *trust_tier == expected_tier;
'''
new_pure = '''    let range_valid =
        validate_score_range_v1(trust_score_range.lower, trust_score_range.upper).is_ok();
    let tier_consistent = validate_score_assertion_v1(
        trust_score_range.lower,
        trust_score_range.upper,
        trust_tier_band_v1(trust_tier),
    )
    .is_ok();
'''
text = replace_in_region(
    text,
    "pub fn verify_credential_pure(",
    "#[cfg(test)]\nmod tests {",
    old_pure,
    new_pure,
    "pure structural verification policy",
)

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
    fn local_and_policy_tier_mappings_round_trip_all_variants() {
        let tiers = [
            TrustTier::Observer,
            TrustTier::Basic,
            TrustTier::Standard,
            TrustTier::Elevated,
            TrustTier::Guardian,
        ];
        for tier in tiers {
            assert_eq!(trust_tier_from_band_v1(trust_tier_band_v1(&tier)), tier);
        }
    }

    #[test]
    fn structural_verifier_rejects_infinite_range_via_shared_policy() {
        let (_, range_valid, tier_consistent, _) = verify_credential_pure(
            &valid_commitment(),
            &TrustScoreRange {
                lower: 0.5,
                upper: f32::INFINITY,
            },
            &TrustTier::Standard,
            &[1],
        );
        assert!(!range_valid);
        assert!(!tier_consistent);
    }
''',
    "coordinator score policy regression tests",
)

for required in [
    "validate_coordinator_score_range(input.trust_score_lower, input.trust_score_upper)?",
    "validate_coordinator_score_range(score, score)?",
    "validate_coordinator_score_range(min_score, min_score)?",
    "validate_coordinator_score_assertion(",
    "trust_tier_band_v1(&trust_tier) < trust_tier_band_v1(min_tier)",
    "validate_score_assertion_v1(",
    "local_and_policy_tier_mappings_round_trip_all_variants",
    "structural_verifier_rejects_infinite_range_via_shared_policy",
]:
    if required not in text:
        raise SystemExit(f"ERROR: coordinator score-policy after-image missing: {required}")

# High-risk coordinator paths may no longer carry private numeric semantics.
for start_marker, end_marker, forbidden in [
    (
        "pub fn issue_trust_credential(",
        "/// Input for issuing a trust credential",
        ["input.trust_score_lower < 0.0", "TrustTier::from_score(mid_score)"],
    ),
    (
        "pub fn self_attest_trust(",
        "/// Input for self-attesting trust",
        ["input.trust_score_lower < 0.0"],
    ),
    (
        "pub fn request_attestation(",
        "/// Input for requesting attestation",
        ["(0.0..=1.0).contains(&score)"],
    ),
    (
        "pub fn fulfill_attestation(",
        "/// Input for fulfilling an attestation",
        ["let mid_score =", "min_tier.min_score()", "input.trust_score_lower < 0.0"],
    ),
]:
    start = text.index(start_marker)
    end = text.index(end_marker, start)
    body = text[start:end]
    for needle in forbidden:
        if needle in body:
            raise SystemExit(
                f"ERROR: legacy private score rule remains in {start_marker}: {needle}"
            )

LIB.write_text(text)
print("Materialized coordinator-wide canonical trust-score policy adoption V1.")
