//! Standalone real-ML-DSA-65 signer for the Pulse Sweettest suite.
//!
//! Two subcommands, both hex in/out over stdout (one value per line):
//!   keygen                          -> signing_key_hex\nverifying_key_hex
//!   sign <signing_key_hex> <msg_hex> -> signature_hex
//!
//! Exists only because `ml-dsa`'s full signing feature set can't link
//! alongside `holochain`'s dependency tree in one Cargo process — see this
//! crate's Cargo.toml doc comment. Never used outside test fixtures.

use ml_dsa::{
    Generate, Keypair, MlDsa65, Signature, SigningKey, common::KeyExport, common::KeyInit,
};
use signature::{SignatureEncoding, Signer};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("keygen") => keygen(),
        Some("sign") => {
            let signing_key_hex = args
                .get(2)
                .expect("usage: sign <signing_key_hex> <msg_hex>");
            let msg_hex = args
                .get(3)
                .expect("usage: sign <signing_key_hex> <msg_hex>");
            sign(signing_key_hex, msg_hex);
        }
        _ => {
            eprintln!("usage: ml-dsa-test-signer keygen | sign <signing_key_hex> <msg_hex>");
            std::process::exit(1);
        }
    }
}

fn keygen() {
    let signing_key = SigningKey::<MlDsa65>::generate();
    let verifying_key = signing_key.verifying_key();
    println!("{}", hex::encode(signing_key.to_bytes()));
    println!("{}", hex::encode(verifying_key.to_bytes()));
}

fn sign(signing_key_hex: &str, msg_hex: &str) {
    let signing_key_bytes = hex::decode(signing_key_hex).expect("invalid signing key hex");
    let msg = hex::decode(msg_hex).expect("invalid message hex");
    let signing_key = SigningKey::<MlDsa65>::new_from_slice(&signing_key_bytes)
        .expect("invalid signing key bytes");
    let signature: Signature<MlDsa65> = signing_key.sign(&msg);
    println!("{}", hex::encode(signature.to_bytes()));
}
