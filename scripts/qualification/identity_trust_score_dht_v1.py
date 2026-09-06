#!/usr/bin/env python3
"""Materialize the proposed #136 trust-score DHT hardening against exact before-images.

This script is qualification-only: it edits the CI checkout, not the committed
product tree. Every replacement requires exactly one matching before-image so
source drift fails closed instead of fuzzily applying a stale security patch.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INTEGRITY = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/integrity"
MANIFEST = INTEGRITY / "Cargo.toml"
LIB = INTEGRITY / "src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse to apply stale/ambiguous security transformation"
        )
    return text.replace(before, after, 1)


manifest = MANIFEST.read_text()
manifest = replace_once(
    manifest,
    "holochain_serialized_bytes = { workspace = true }\nserde = { workspace = true }\n",
    "holochain_serialized_bytes = { workspace = true }\n"
    "mycelix-trust-score-assertion-policy = { path = \"../../../crates/trust-score-assertion-policy\" }\n"
    "serde = { workspace = true }\n",
    "integrity policy dependency",
)
MANIFEST.write_text(manifest)

text = LIB.read_text()

text = replace_once(
    text,
    "use hdi::prelude::*;\n",
    "use hdi::prelude::*;\n"
    "use mycelix_trust_score_assertion_policy::TrustTierBandV1;\n"
    "use mycelix_trust_score_assertion_policy::validate_score_assertion_v1;\n",
    "policy import",
)

text = replace_once(
    text,
    "    }\n}\n\n/// K-Vector Attestation Request\n",
    "    }\n}\n\n"
    "/// Map the Identity-local credential tier into the canonical structural\n"
    "/// score-policy vocabulary. The mapping is exhaustive so enum ordinal or\n"
    "/// serialization representation can never substitute for semantics.\n"
    "fn trust_tier_band_v1(tier: &TrustTier) -> TrustTierBandV1 {\n"
    "    match tier {\n"
    "        TrustTier::Observer => TrustTierBandV1::Observer,\n"
    "        TrustTier::Basic => TrustTierBandV1::Basic,\n"
    "        TrustTier::Standard => TrustTierBandV1::Standard,\n"
    "        TrustTier::Elevated => TrustTierBandV1::Elevated,\n"
    "        TrustTier::Guardian => TrustTierBandV1::Guardian,\n"
    "    }\n"
    "}\n\n"
    "/// K-Vector Attestation Request\n",
    "tier-band mapping",
)

old_validation = '''    // Trust score range must be valid
    if cred.trust_score_range.lower < 0.0 || cred.trust_score_range.upper > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score range must be within [0, 1]".into(),
        ));
    }

    if cred.trust_score_range.lower > cred.trust_score_range.upper {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score range lower bound cannot exceed upper bound".into(),
        ));
    }

    // Trust tier must be consistent with range
    let tier_min = cred.trust_tier.min_score();
    if (cred.trust_score_range.upper as f64) < tier_min {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score range is inconsistent with claimed tier".into(),
        ));
    }
'''

new_validation = '''    // Canonical structural trust-score theorem. This is DHT-authoritative:
    // modified coordinators cannot bypass finite/bounded/ordered range admission
    // or exact midpoint-derived tier consistency.
    if let Err(error) = validate_score_assertion_v1(
        cred.trust_score_range.lower,
        cred.trust_score_range.upper,
        trust_tier_band_v1(&cred.trust_tier),
    ) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid trust score assertion: {error:?}"
        )));
    }
'''

text = replace_once(text, old_validation, new_validation, "credential score validation")

text = replace_once(
    text,
    '''    #[test]
    fn create_credential_valid_when_issuer_matches_committer() {
        let cred = valid_credential(format!("did:mycelix:{}", me()));
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }
''',
    '''    #[test]
    fn create_credential_valid_when_issuer_matches_committer() {
        let cred = valid_credential(format!("did:mycelix:{}", me()));
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_credential_nan_range_is_rejected_by_dht_validation() {
        let mut cred = valid_credential(format!("did:mycelix:{}", me()));
        cred.trust_score_range.lower = f32::NAN;
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_credential_infinite_range_is_rejected_by_dht_validation() {
        let mut cred = valid_credential(format!("did:mycelix:{}", me()));
        cred.trust_score_range.upper = f32::INFINITY;
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_credential_guardian_claim_with_elevated_midpoint_is_rejected() {
        let mut cred = valid_credential(format!("did:mycelix:{}", me()));
        cred.trust_score_range = TrustScoreRange {
            lower: 0.4,
            upper: 0.8,
        };
        cred.trust_tier = TrustTier::Guardian;
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_credential_underclaimed_tier_is_rejected() {
        let mut cred = valid_credential(format!("did:mycelix:{}", me()));
        cred.trust_score_range = TrustScoreRange {
            lower: 0.8,
            upper: 0.9,
        };
        cred.trust_tier = TrustTier::Basic;
        let result = validate_create_credential(test_action(me()), cred).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
''',
    "DHT score regression tests",
)

# Fail closed if the weak upper-bound/min-score rule somehow remains in the
# credential-create validator after transformation.
create_start = text.index("fn validate_create_credential(")
create_end = text.index("/// Pure credential-update policy.", create_start)
create_body = text[create_start:create_end]
for forbidden in [
    "let tier_min = cred.trust_tier.min_score();",
    "if (cred.trust_score_range.upper as f64) < tier_min",
]:
    if forbidden in create_body:
        raise SystemExit(f"ERROR: weak credential score rule remains after patch: {forbidden}")

for required in [
    "validate_score_assertion_v1(",
    "trust_tier_band_v1(&cred.trust_tier)",
    "create_credential_nan_range_is_rejected_by_dht_validation",
    "create_credential_guardian_claim_with_elevated_midpoint_is_rejected",
]:
    if required not in text:
        raise SystemExit(f"ERROR: required DHT hardening after-image missing: {required}")

LIB.write_text(text)
print("Materialized identity trust-score DHT hardening V1 against exact before-images.")
