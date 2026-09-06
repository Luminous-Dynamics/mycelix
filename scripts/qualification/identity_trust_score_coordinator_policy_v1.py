#!/usr/bin/env python3
"""Materialize coordinator-wide adoption of the canonical trust-score theorem.

Qualification-only transformation. It edits the CI checkout, not committed
product source, and requires every security-sensitive before-image exactly once.
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


manifest = MANIFEST.read_text()
manifest = replace_once(
    manifest,
    "hdk = { workspace = true }\ntrust_credential_integrity = { path = \"../integrity\" }\n",
    "hdk = { workspace = true }\n"
    "mycelix-trust-score-assertion-policy = { path = \"../../../crates/trust-score-assertion-policy\" }\n"
    "trust_credential_integrity = { path = \"../integrity\" }\n",
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
/// coordinator path should reject malformed scores before performing business or
/// request-state logic.
fn validate_coordinator_score_range(lower: f32, upper: f32) -> ExternResult<TrustTier> {
    let band = validate_score_range_v1(lower, upper).map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid trust score range: {error:?}"
        )))
    })?;
    Ok(trust_tier_from_band_v1(band))
}
'''
text = replace_once(text, anchor, helpers, "coordinator score-policy helpers")

old_issue_guard = '''    if input.trust_score_lower < 0.0
        || input.trust_score_upper > 1.0
        || input.trust_score_lower > input.trust_score_upper
    {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Trust scores must be in [0.0, 1.0] with lower <= upper".into()
        )));
    }
'''
text = replace_once(
    text,
    old_issue_guard,
    '''    let trust_tier =
        validate_coordinator_score_range(input.trust_score_lower, input.trust_score_upper)?;
''',
    "issue trust score admission",
)

text = replace_once(
    text,
    '''    // Determine trust tier from the proven range
    let mid_score = (input.trust_score_lower as f64 + input.trust_score_upper as f64) / 2.0;
    let trust_tier = TrustTier::from_score(mid_score);

''',
    "",
    "duplicate issue midpoint derivation",
)

# The same weak guard appears once more in self-attestation. After the first
# issue-path replacement above, it must still occur exactly once.
text = replace_once(
    text,
    old_issue_guard,
    '''    let _ = validate_coordinator_score_range(
        input.trust_score_lower,
        input.trust_score_upper,
    )?;
''',
    "self-attestation trust score admission",
)

text = replace_once(
    text,
    '''    // Verify the provided proof meets request requirements
    let mid_score = (input.trust_score_lower as f64 + input.trust_score_upper as f64) / 2.0;
    let trust_tier = TrustTier::from_score(mid_score);

''',
    '''    // Reject malformed/non-finite score evidence before request requirement
    // comparisons or request-state fulfillment logic can consume it.
    let trust_tier =
        validate_coordinator_score_range(input.trust_score_lower, input.trust_score_upper)?;

''',
    "fulfillment score admission",
)

text = replace_once(
    text,
    '''    if let Some(min_score) = req.min_trust_score {
        if input.trust_score_lower < min_score {
''',
    '''    if let Some(min_score) = req.min_trust_score {
        // Historical requests may predate strict finite-score admission. Re-audit
        // the requested scalar before using it as a comparison threshold.
        let _ = validate_coordinator_score_range(min_score, min_score)?;
        if input.trust_score_lower < min_score {
''',
    "historical request minimum score audit",
)

text = replace_once(
    text,
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
text = replace_once(text, old_verify, new_verify, "runtime structural verification policy")

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
text = replace_once(text, old_pure, new_pure, "pure structural verification policy")

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

# Static after-image checks.
for required in [
    "validate_coordinator_score_range(input.trust_score_lower, input.trust_score_upper)?",
    "validate_coordinator_score_range(min_score, min_score)?",
    "trust_tier_band_v1(&trust_tier) < trust_tier_band_v1(min_tier)",
    "validate_score_assertion_v1(",
    "local_and_policy_tier_mappings_round_trip_all_variants",
    "structural_verifier_rejects_infinite_range_via_shared_policy",
]:
    if required not in text:
        raise SystemExit(f"ERROR: coordinator score-policy after-image missing: {required}")

# No admission path may retain the legacy ordinary comparison gate or local
# midpoint derivation after this transformation.
issue_start = text.index("pub fn issue_trust_credential(")
issue_end = text.index("/// Input for issuing a trust credential", issue_start)
issue_body = text[issue_start:issue_end]
for forbidden in [
    "input.trust_score_lower < 0.0",
    "TrustTier::from_score(mid_score)",
]:
    if forbidden in issue_body:
        raise SystemExit(f"ERROR: legacy issue score rule remains: {forbidden}")

fulfill_start = text.index("pub fn fulfill_attestation(")
fulfill_end = text.index("/// Input for fulfilling an attestation", fulfill_start)
fulfill_body = text[fulfill_start:fulfill_end]
for forbidden in [
    "let mid_score =",
    "min_tier.min_score()",
]:
    if forbidden in fulfill_body:
        raise SystemExit(f"ERROR: legacy fulfillment score rule remains: {forbidden}")

LIB.write_text(text)
print("Materialized coordinator-wide canonical trust-score policy adoption V1.")
