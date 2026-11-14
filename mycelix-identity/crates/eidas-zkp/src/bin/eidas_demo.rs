// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Experimental credential-shaped serialization walkthrough.
//!
//! Usage:
//!   cargo run -p mycelix-eidas-zkp --bin eidas_demo
//!
//! This program demonstrates only:
//! 1. Construction of VC-shaped JSON data.
//! 2. Filtering a set of claims for a presentation-shaped object.
//! 3. Serialization of the resulting data structure.
//!
//! It does NOT generate or verify a signature, generate or verify a ZK proof,
//! verify a credential, establish unlinkability, or demonstrate eIDAS/EUDI
//! Wallet, W3C Data Integrity, or OpenID conformance. All proof strings below
//! are explicit placeholders.

use mycelix_eidas_zkp::*;

fn main() {
    println!("Experimental VC-shaped data / selective-claim walkthrough");
    println!("NOT a cryptographic verification or standards-conformance test");
    println!();

    println!("Step 1: construct credential-shaped data");
    let credential = create_eidas_credential(
        "did:mycelix:government-nl",
        "did:mycelix:citizen-alice",
        serde_json::json!({
            "givenName": "Alice",
            "familyName": "van den Berg",
            "dateOfBirth": "1990-03-15",
            "nationality": "NL",
            "age": 36
        }),
        "EXAMPLE-ONLY-NOT-A-REAL-SIGNATURE",
    );
    println!("  Credential ID: {}", credential.id);
    println!("  Context URI present: {}", credential.context.iter().any(|x| x == "https://www.w3.org/ns/credentials/v2"));
    println!("  Proof metadata label: {}", credential.proof.cryptosuite);
    println!("  Proof value: caller-supplied example placeholder (not checked)");
    println!();

    println!("Step 2: construct presentation-shaped data by filtering visible claims");
    let presentation = create_selective_presentation(
        &credential,
        &["nationality"],
        vec![ProvenClaim {
            claim_key: "age".to_string(),
            proof_type: "range".to_string(),
            description: "age >= 18".to_string(),
            proof_value: "EXAMPLE-ONLY-NOT-A-ZK-PROOF".to_string(),
        }],
        "EXAMPLE-ONLY-NOT-A-HOLDER-SIGNATURE",
    );

    let disclosed_credential = &presentation.verifiable_credential[0];
    let claims = disclosed_credential.credential_subject.claims.as_object().unwrap();
    let metadata = disclosed_credential.proof.selective_disclosure.as_ref().unwrap();

    println!("  Visible claims in this serialized object:");
    for (key, value) in claims {
        println!("    {}: {}", key, value);
    }
    println!("  Hidden from this object by ordinary JSON filtering:");
    for key in ["givenName", "familyName", "dateOfBirth", "age"] {
        if !claims.contains_key(key) {
            println!("    {}", key);
        }
    }
    println!("  Descriptive predicate metadata:");
    for claim in &metadata.proven_claims {
        println!("    {}: {}", claim.claim_key, claim.description);
    }
    println!("  Merkle proof entries present: {}", metadata.merkle_proofs.len());
    println!();

    println!("Step 3: checks not performed by this crate");
    println!("  Issuer signature verification: NOT RUN");
    println!("  Holder signature verification: NOT RUN");
    println!("  ZK predicate verification: NOT RUN");
    println!("  Credential expiry / status verification: NOT RUN");
    println!("  W3C Data Integrity cryptosuite conformance: NOT TESTED");
    println!("  OpenID4VCI / OpenID4VP conformance: NOT TESTED");
    println!("  eIDAS / EUDI Wallet conformance: NOT TESTED");
    println!();

    println!("Serialization walkthrough complete. No credential or predicate was verified.");
    println!("See the Mycelix Identity interoperability and assurance roadmap before making");
    println!("cryptographic, W3C, eIDAS, or EUDI Wallet claims.");

    println!();
    println!("--- Presentation-shaped JSON ---");
    println!("{}", serde_json::to_string_pretty(&presentation).unwrap());
}
