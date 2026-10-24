use std::collections::HashSet;
use std::sync::{Arc, Barrier, Mutex};
use std::thread;

use base64::{Engine as _, engine::general_purpose::URL_SAFE_NO_PAD};

use affinidi_sd_jwt::{
    Disclosure, SdJwt, Sha256Hasher, holder, issuer, verifier,
    holder::KbJwtInput,
    signer::{JwtSigner, JwtVerifier, test_utils::{HmacSha256Signer, HmacSha256Verifier}},
    verifier::VerificationOptions,
};
use serde_json::{Value, json};

const ISSUER_KEY: &[u8] = b"test-key-for-hmac-256-signing!!";
const WRONG_ISSUER_KEY: &[u8] = b"wrong-key-for-hmac-256-signing!";
const HOLDER_KEY: &[u8] = b"holder-key-for-hmac-signing!!!";
const WRONG_HOLDER_KEY: &[u8] = b"wrong-holder-key-for-hmac-signing";
const HOLDER_JWK_K: &str = "aG9sZGVyLWtleS1mb3ItaG1hYy1zaWduaW5nISEh";
const AUDIENCE: &str = "https://verifier.example.test";
const NONCE: &str = "test-nonce-0001";
const ISSUED_AT: u64 = 1_800_000_000;

#[derive(Debug)]
struct CaseResult {
    id: &'static str,
    expected: &'static str,
    actual: &'static str,
}

impl CaseResult {
    fn new(id: &'static str, expected_accept: bool, actual_accept: bool) -> Self {
        Self {
            id,
            expected: outcome(expected_accept),
            actual: outcome(actual_accept),
        }
    }

    fn passed(&self) -> bool {
        self.expected == self.actual
    }
}

fn outcome(accepted: bool) -> &'static str {
    if accepted { "ACCEPT" } else { "REJECT" }
}

fn issue(claims: &Value, frame: &Value, holder_jwk: Option<&Value>) -> SdJwt {
    let signer = HmacSha256Signer::new(ISSUER_KEY);
    issuer::issue(claims, frame, &signer, &Sha256Hasher, holder_jwk)
        .expect("fixed synthetic fixture must issue")
}

fn base_claims() -> Value {
    json!({
        "sub": "synthetic-subject-001",
        "given_name": "Asha",
        "family_name": "Ndlovu",
        "credential_type": "SyntheticEmploymentCredential"
    })
}

fn base_frame() -> Value {
    json!({ "_sd": ["given_name", "family_name"] })
}

fn base_credential(holder_jwk: Option<&Value>) -> SdJwt {
    issue(&base_claims(), &base_frame(), holder_jwk)
}

fn select_given_name(credential: &SdJwt) -> Vec<&Disclosure> {
    holder::select_disclosures(credential, &["given_name"])
}

fn partial_presentation(credential: &SdJwt) -> SdJwt {
    holder::present(
        credential,
        &select_given_name(credential),
        None,
        &Sha256Hasher,
    )
    .expect("synthetic partial presentation must be created")
}

fn parse_wire(presentation: &SdJwt) -> Option<SdJwt> {
    SdJwt::parse(&presentation.serialize(), &Sha256Hasher).ok()
}

fn basic_accepts(presentation: &SdJwt, issuer_key: &[u8]) -> bool {
    let Some(canonical) = parse_wire(presentation) else {
        return false;
    };
    let issuer_verifier = HmacSha256Verifier::new(issuer_key);
    verifier::verify(
        &canonical,
        &issuer_verifier,
        &Sha256Hasher,
        &VerificationOptions::default(),
        None,
    )
    .is_ok_and(|result| result.is_verified())
}

fn issue_holder_bound() -> SdJwt {
    let jwk = json!({
        "kty": "oct",
        "k": HOLDER_JWK_K
    });
    base_credential(Some(&jwk))
}

fn key_bound_presentation(nonce: &str, audience: &str) -> SdJwt {
    key_bound_presentation_at_iat(nonce, audience, ISSUED_AT)
}

fn key_bound_presentation_at_iat(nonce: &str, audience: &str, iat: u64) -> SdJwt {
    key_bound_presentation_at_iat_with_key(nonce, audience, iat, HOLDER_KEY)
}

fn key_bound_presentation_at_iat_with_key(
    nonce: &str,
    audience: &str,
    iat: u64,
    holder_signing_key: &[u8],
) -> SdJwt {
    // The issuer always binds cnf.jwk to HOLDER_KEY. This helper can deliberately
    // sign with another key to test that callers enforce the cnf/verifier link.
    let credential = issue_holder_bound();
    let selected = select_given_name(&credential);
    let holder_signer = HmacSha256Signer::new(holder_signing_key);
    let kb_input = KbJwtInput {
        audience,
        nonce,
        signer: &holder_signer,
        iat,
    };
    holder::present(&credential, &selected, Some(&kb_input), &Sha256Hasher)
        .expect("synthetic key-bound presentation must be created")
}

fn key_bound_presentation_with_typ(typ: &str) -> SdJwt {
    let original = key_bound_presentation(NONCE, AUDIENCE);
    let kb_jwt = original.kb_jwt.as_deref().expect("fixture has KB-JWT");
    let mut parts = kb_jwt.split('.');
    let (_header, payload_b64, _signature) = (
        parts.next().expect("header"),
        parts.next().expect("payload"),
        parts.next().expect("signature"),
    );
    assert!(parts.next().is_none(), "KB-JWT must be compact");
    let payload_bytes = URL_SAFE_NO_PAD.decode(payload_b64).expect("payload base64url");
    let payload: Value = serde_json::from_slice(&payload_bytes).expect("KB-JWT payload JSON");
    let signer = HmacSha256Signer::new(HOLDER_KEY);
    let header = json!({ "alg": "HS256", "typ": typ });
    let signed_kb_jwt = signer.sign_jwt(&header, &payload).expect("signed synthetic KB-JWT");
    SdJwt::new(
        original.jws.clone(),
        original.disclosures.clone(),
        Some(signed_kb_jwt),
    )
}

fn library_key_binding_accepts(
    presentation: &SdJwt,
    expected_nonce: &str,
    expected_audience: &str,
    holder_key: Option<&[u8]>,
) -> bool {
    let Some(canonical) = parse_wire(presentation) else {
        return false;
    };
    let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
    let options = VerificationOptions {
        verify_kb: true,
        expected_audience: Some(expected_audience),
        expected_nonce: Some(expected_nonce),
    };
    match holder_key {
        Some(key) => {
            let holder_verifier = HmacSha256Verifier::new(key);
            verifier::verify(
                &canonical,
                &issuer_verifier,
                &Sha256Hasher,
                &options,
                Some(&holder_verifier),
            )
            .is_ok_and(|result| result.kb_verified == Some(true))
        }
        None => verifier::verify(
            &canonical,
            &issuer_verifier,
            &Sha256Hasher,
            &options,
            None,
        )
        .is_ok_and(|result| result.kb_verified == Some(true)),
    }
}

fn synthetic_cnf_matches_holder_key(presentation: &SdJwt, holder_key: &[u8]) -> bool {
    let Some(canonical) = parse_wire(presentation) else {
        return false;
    };
    let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
    // cnf.jwk is trusted only after the issuer's signed JWS is verified.
    let Ok(payload) = issuer_verifier.verify_jwt(&canonical.jws) else {
        return false;
    };
    let Some(encoded_key) = payload
        .get("cnf")
        .and_then(|cnf| cnf.get("jwk"))
        .and_then(|jwk| jwk.get("k"))
        .and_then(Value::as_str)
    else {
        return false;
    };
    URL_SAFE_NO_PAD
        .decode(encoded_key)
        .is_ok_and(|encoded| encoded.as_slice() == holder_key)
}

fn signed_kb_jwt_has_typ(presentation: &SdJwt, expected_typ: &str) -> bool {
    let Some(kb_jwt) = presentation.kb_jwt.as_deref() else {
        return false;
    };
    let mut parts = kb_jwt.split('.');
    let Some(header_b64) = parts.next() else {
        return false;
    };
    if parts.next().is_none() || parts.next().is_none() || parts.next().is_some() {
        return false;
    }
    let Ok(header_bytes) = URL_SAFE_NO_PAD.decode(header_b64) else {
        return false;
    };
    let Ok(header) = serde_json::from_slice::<Value>(&header_bytes) else {
        return false;
    };
    header.get("typ").and_then(Value::as_str) == Some(expected_typ)
}

fn key_binding_accepts(
    presentation: &SdJwt,
    expected_nonce: &str,
    expected_audience: &str,
    holder_key: Option<&[u8]>,
) -> bool {
    let Some(holder_key) = holder_key else {
        return false;
    };
    // The pinned 0.1.5 verifier checks signature, audience, nonce, and sd_hash,
    // but the caller must enforce both KB-JWT typ and the issuer-authenticated
    // cnf.jwk -> holder-verifier binding.
    library_key_binding_accepts(
        presentation,
        expected_nonce,
        expected_audience,
        Some(holder_key),
    ) && signed_kb_jwt_has_typ(presentation, "kb+jwt")
        && synthetic_cnf_matches_holder_key(presentation, holder_key)
}

fn key_binding_freshness_accepts(
    presentation: &SdJwt,
    expected_nonce: &str,
    expected_audience: &str,
    now: i64,
    max_age_secs: i64,
) -> bool {
    // Run published verifier checks first; never apply local freshness policy
    // to an unverified timestamp.
    if !library_key_binding_accepts(
        presentation,
        expected_nonce,
        expected_audience,
        Some(HOLDER_KEY),
    ) || !signed_kb_jwt_has_typ(presentation, "kb+jwt")
    {
        return false;
    }

    let Some(canonical) = parse_wire(presentation) else {
        return false;
    };
    let Some(kb_jwt) = canonical.kb_jwt.as_deref() else {
        return false;
    };
    let mut parts = kb_jwt.split('.');
    let (Some(_header), Some(payload), Some(_signature), None) =
        (parts.next(), parts.next(), parts.next(), parts.next())
    else {
        return false;
    };
    let Ok(payload_bytes) = URL_SAFE_NO_PAD.decode(payload) else {
        return false;
    };
    let Ok(payload) = serde_json::from_slice::<Value>(&payload_bytes) else {
        return false;
    };
    let Some(iat) = payload.get("iat").and_then(Value::as_i64) else {
        return false;
    };

    if iat > now.saturating_add(60) {
        return false;
    }
    now.saturating_sub(iat) <= max_age_secs
}

fn mutate_signature(jws: &str) -> String {
    let mut parts: Vec<String> = jws.split('.').map(str::to_owned).collect();
    assert_eq!(parts.len(), 3, "fixture JWS must be compact");
    let replacement = if parts[2].starts_with('A') { "B" } else { "A" };
    parts[2].replace_range(0..1, replacement);
    parts.join(".")
}

#[derive(Default)]
struct ConcurrentSingleUseNonceLedger {
    consumed: Mutex<HashSet<(String, String)>>,
}

impl ConcurrentSingleUseNonceLedger {
    // Namespace nonce uniqueness by verifier identity. A poisoned ledger fails
    // closed rather than accepting a request whose replay state is uncertain.
    fn consume(&self, verifier_id: &str, nonce: &str) -> bool {
        self.consumed
            .lock()
            .is_ok_and(|mut consumed| consumed.insert((verifier_id.to_owned(), nonce.to_owned())))
    }
}

fn run_matrix() -> Vec<CaseResult> {
    let mut cases = Vec::new();

    // Positive utility control: only the selected claim is returned.
    let credential = base_credential(None);
    let partial = partial_presentation(&credential);
    let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
    let result = verifier::verify(
        &partial,
        &issuer_verifier,
        &Sha256Hasher,
        &VerificationOptions::default(),
        None,
    );
    let utility_ok = result.is_ok_and(|verified| {
        verified.claims["given_name"] == "Asha"
            && verified.claims.get("family_name").is_none()
            && verified.claims["sub"] == "synthetic-subject-001"
    });
    cases.push(CaseResult::new("valid_partial_disclosure", true, utility_ok));

    // The library exposes mutable parsed claim fields. Re-parse the exact wire
    // representation at the verifier boundary so those cached fields cannot
    // disagree with the issuer-bound encoded disclosure.
    let mut mutated_cache = partial.clone();
    mutated_cache.disclosures[0].claim_value = json!("Mallory");
    let canonical = parse_wire(&mutated_cache).expect("serialized presentation must parse");
    let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
    let cache_integrity_ok = verifier::verify(
        &canonical,
        &issuer_verifier,
        &Sha256Hasher,
        &VerificationOptions::default(),
        None,
    )
    .is_ok_and(|verified| verified.claims["given_name"] == "Asha");
    cases.push(CaseResult::new(
        "wire_reparse_restores_issuer_bound_disclosure_value",
        true,
        cache_integrity_ok,
    ));

    // The exact same signed material must not verify under a wrong issuer key.
    cases.push(CaseResult::new(
        "wrong_issuer_key",
        false,
        basic_accepts(&partial, WRONG_ISSUER_KEY),
    ));

    let tampered_jws = mutate_signature(&partial.jws);
    let tampered_signature = SdJwt::new(
        tampered_jws,
        partial.disclosures.clone(),
        partial.kb_jwt.clone(),
    );
    cases.push(CaseResult::new(
        "tampered_issuer_signature",
        false,
        basic_accepts(&tampered_signature, ISSUER_KEY),
    ));

    let duplicated = SdJwt::new(
        partial.jws.clone(),
        vec![
            partial.disclosures[0].clone(),
            partial.disclosures[0].clone(),
        ],
        None,
    );
    cases.push(CaseResult::new(
        "duplicate_disclosure",
        false,
        basic_accepts(&duplicated, ISSUER_KEY),
    ));

    let other_claims = json!({
        "sub": "synthetic-subject-002",
        "department": "sensitive-department"
    });
    let other_credential = issue(&other_claims, &json!({ "_sd": ["department"] }), None);
    let mut disclosures = partial.disclosures.clone();
    disclosures.push(other_credential.disclosures[0].clone());
    let unissued = SdJwt::new(partial.jws.clone(), disclosures, None);
    cases.push(CaseResult::new(
        "unissued_disclosure",
        false,
        basic_accepts(&unissued, ISSUER_KEY),
    ));

    // When the RP requires key binding, a holder proof must be present.
    let holder_bound = issue_holder_bound();
    let no_kb = partial_presentation(&holder_bound);
    cases.push(CaseResult::new(
        "key_binding_required_but_missing",
        false,
        key_binding_accepts(&no_kb, NONCE, AUDIENCE, Some(HOLDER_KEY)),
    ));

    let good_kb = key_bound_presentation(NONCE, AUDIENCE);
    cases.push(CaseResult::new(
        "valid_key_binding",
        true,
        key_binding_accepts(&good_kb, NONCE, AUDIENCE, Some(HOLDER_KEY)),
    ));
    let wrong_typ_kb = key_bound_presentation_with_typ("JWT");
    cases.push(CaseResult::new(
        "published_verifier_accepts_signed_wrong_kb_typ",
        true,
        library_key_binding_accepts(&wrong_typ_kb, NONCE, AUDIENCE, Some(HOLDER_KEY)),
    ));
    cases.push(CaseResult::new(
        "application_policy_rejects_signed_wrong_kb_typ",
        false,
        key_binding_accepts(&wrong_typ_kb, NONCE, AUDIENCE, Some(HOLDER_KEY)),
    ));
    let wrong_key_kb =
        key_bound_presentation_at_iat_with_key(NONCE, AUDIENCE, ISSUED_AT, WRONG_HOLDER_KEY);
    cases.push(CaseResult::new(
        "published_verifier_accepts_non_cnf_holder_key",
        true,
        library_key_binding_accepts(&wrong_key_kb, NONCE, AUDIENCE, Some(WRONG_HOLDER_KEY)),
    ));
    cases.push(CaseResult::new(
        "application_policy_rejects_non_cnf_holder_key",
        false,
        key_binding_accepts(&wrong_key_kb, NONCE, AUDIENCE, Some(WRONG_HOLDER_KEY)),
    ));
    let fresh_now = ISSUED_AT as i64 + 30;
    let fresh_kb = key_bound_presentation_at_iat(NONCE, AUDIENCE, ISSUED_AT);
    cases.push(CaseResult::new(
        "key_binding_iat_within_freshness_window",
        true,
        key_binding_freshness_accepts(&fresh_kb, NONCE, AUDIENCE, fresh_now, 300),
    ));
    cases.push(CaseResult::new(
        "stale_key_binding_iat_rejected",
        false,
        key_binding_freshness_accepts(
            &fresh_kb,
            NONCE,
            AUDIENCE,
            ISSUED_AT as i64 + 500,
            300,
        ),
    ));
    let future_kb = key_bound_presentation_at_iat(NONCE, AUDIENCE, ISSUED_AT + 1000);
    cases.push(CaseResult::new(
        "future_key_binding_iat_rejected",
        false,
        key_binding_freshness_accepts(
            &future_kb,
            NONCE,
            AUDIENCE,
            ISSUED_AT as i64,
            300,
        ),
    ));
    cases.push(CaseResult::new(
        "wrong_nonce",
        false,
        key_binding_accepts(&good_kb, "different-nonce", AUDIENCE, Some(HOLDER_KEY)),
    ));
    cases.push(CaseResult::new(
        "wrong_audience",
        false,
        key_binding_accepts(&good_kb, NONCE, "https://other.example.test", Some(HOLDER_KEY)),
    ));
    cases.push(CaseResult::new(
        "missing_holder_verifier",
        false,
        key_binding_accepts(&good_kb, NONCE, AUDIENCE, None),
    ));
    cases.push(CaseResult::new(
        "wrong_holder_key",
        false,
        key_binding_accepts(&good_kb, NONCE, AUDIENCE, Some(WRONG_HOLDER_KEY)),
    ));

    // The KB-JWT's sd_hash binds the holder proof to this exact disclosure set.
    let altered_disclosure_set = SdJwt::new(
        good_kb.jws.clone(),
        Vec::new(),
        good_kb.kb_jwt.clone(),
    );
    cases.push(CaseResult::new(
        "key_binding_disclosure_set_mutated",
        false,
        key_binding_accepts(
            &altered_disclosure_set,
            NONCE,
            AUDIENCE,
            Some(HOLDER_KEY),
        ),
    ));

    // A valid signed proof can be cryptographically replayed if the exact same
    // nonce is accepted more than once. The app-level single-use ledger closes
    // that gap; it is intentionally reported as a separate policy layer.
    let stateless_accepts_replay =
        key_binding_accepts(&good_kb, NONCE, AUDIENCE, Some(HOLDER_KEY));
    cases.push(CaseResult::new(
        "stateless_same_nonce_replay_is_cryptographically_valid",
        true,
        stateless_accepts_replay,
    ));

    let ledger = ConcurrentSingleUseNonceLedger::default();
    let first_accepted = stateless_accepts_replay && ledger.consume(AUDIENCE, NONCE);
    cases.push(CaseResult::new(
        "single_use_nonce_first_consumption",
        true,
        first_accepted,
    ));
    let second_accepted = stateless_accepts_replay && ledger.consume(AUDIENCE, NONCE);
    cases.push(CaseResult::new(
        "single_use_nonce_replay_rejected_by_policy",
        false,
        second_accepted,
    ));

    cases
}

fn main() {
    let cases = run_matrix();
    println!("case_id,expected,actual,pass");
    for case in &cases {
        println!(
            "{},{},{},{}",
            case.id,
            case.expected,
            case.actual,
            if case.passed() { "true" } else { "false" }
        );
    }
    if cases.iter().any(|case| !case.passed()) {
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verifier_decision_matrix_matches_frozen_expected_outcomes() {
        let cases = run_matrix();
        assert_eq!(cases.len(), 23);
        for case in &cases {
            assert!(
                case.passed(),
                "case {} expected {}, got {}",
                case.id,
                case.expected,
                case.actual
            );
        }
    }

    #[test]
    fn selected_claim_is_returned_without_unselected_claims() {
        let credential = base_credential(None);
        let presentation = partial_presentation(&credential);
        let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
        let result = verifier::verify(
            &presentation,
            &issuer_verifier,
            &Sha256Hasher,
            &VerificationOptions::default(),
            None,
        )
        .expect("valid partial presentation must verify");

        assert_eq!(result.claims["given_name"], "Asha");
        assert!(result.claims.get("family_name").is_none());
    }

    #[test]
    fn typed_disclosure_cache_is_not_authoritative_for_verification() {
        let credential = base_credential(None);
        let mut presentation = partial_presentation(&credential);
        presentation.disclosures[0].claim_value = json!("Mallory");

        // Direct typed-object verification can observe caller-mutated cached
        // fields even though serialized disclosure bytes still encode "Asha".
        let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
        let direct = verifier::verify(
            &presentation,
            &issuer_verifier,
            &Sha256Hasher,
            &VerificationOptions::default(),
            None,
        )
        .expect("underlying signed wire bytes remain valid");
        assert_eq!(direct.claims["given_name"], "Mallory");

        // Re-parsing the exact serialized presentation makes the signed wire
        // representation authoritative and reconstructs the issuer-bound value.
        let canonical = parse_wire(&presentation).expect("presentation wire must parse");
        let issuer_verifier = HmacSha256Verifier::new(ISSUER_KEY);
        let reparsed = verifier::verify(
            &canonical,
            &issuer_verifier,
            &Sha256Hasher,
            &VerificationOptions::default(),
            None,
        )
        .expect("canonical wire representation must verify");
        assert_eq!(reparsed.claims["given_name"], "Asha");
    }

    #[test]
    fn nonce_ledger_rejects_replay_but_namespaces_verifiers() {
        let ledger = ConcurrentSingleUseNonceLedger::default();
        assert!(ledger.consume("verifier-a", "one-time-test-nonce"));
        assert!(!ledger.consume("verifier-a", "one-time-test-nonce"));
        assert!(ledger.consume("verifier-a", "different-test-nonce"));
        assert!(ledger.consume("verifier-b", "one-time-test-nonce"));
    }

    #[test]
    fn concurrent_nonce_race_has_exactly_one_winner() {
        const CONTENDERS: usize = 64;
        let ledger = Arc::new(ConcurrentSingleUseNonceLedger::default());
        let barrier = Arc::new(Barrier::new(CONTENDERS));
        let mut workers = Vec::with_capacity(CONTENDERS);

        for _ in 0..CONTENDERS {
            let ledger = Arc::clone(&ledger);
            let barrier = Arc::clone(&barrier);
            workers.push(thread::spawn(move || {
                barrier.wait();
                ledger.consume("verifier-a", "racing-one-time-nonce")
            }));
        }

        let accepted = workers
            .into_iter()
            .map(|worker| worker.join().expect("nonce contender thread must finish"))
            .filter(|won| *won)
            .count();
        assert_eq!(accepted, 1, "one-time nonce must have exactly one winner");
    }
}
