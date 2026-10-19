// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_crypto::AlgorithmId;

#[test]
fn fips_mldsa65_metadata_is_frozen() {
    assert_eq!(AlgorithmId::MlDsa65.public_key_size(), 1952);
    assert_eq!(AlgorithmId::MlDsa65.signature_size(), 3309);
    assert_eq!(
        AlgorithmId::HybridEd25519MlDsa65.public_key_size(),
        32 + 1952
    );
    assert_eq!(
        AlgorithmId::HybridEd25519MlDsa65.signature_size(),
        64 + 3309
    );
}

#[cfg(feature = "native")]
mod native_contract {
    use super::*;
    use mycelix_crypto::pqc::dilithium::MlDsa65Signer;
    use mycelix_crypto::pqc::hybrid::HybridSigner;
    use mycelix_crypto::traits::Signer;

    #[test]
    fn native_mldsa65_signer_must_match_algorithm_id_wire_size() {
        let signer = MlDsa65Signer::generate();
        let public_key = signer.public_key();
        assert_eq!(public_key.algorithm, AlgorithmId::MlDsa65);
        assert_eq!(
            public_key.key_bytes.len(),
            AlgorithmId::MlDsa65.public_key_size()
        );

        let signature = signer
            .sign(b"mycelix-crypto-wire-contract-v1")
            .expect("native ML-DSA-65 signer must emit the AlgorithmId wire format");
        assert_eq!(signature.algorithm, AlgorithmId::MlDsa65);
        assert_eq!(
            signature.signature_bytes.len(),
            AlgorithmId::MlDsa65.signature_size()
        );
    }

    #[test]
    fn native_hybrid_signer_must_match_algorithm_id_wire_size() {
        let signer = HybridSigner::generate();
        let public_key = signer.public_key();
        assert_eq!(
            public_key.algorithm,
            AlgorithmId::HybridEd25519MlDsa65
        );
        assert_eq!(
            public_key.key_bytes.len(),
            AlgorithmId::HybridEd25519MlDsa65.public_key_size()
        );

        let signature = signer
            .sign(b"mycelix-hybrid-wire-contract-v1")
            .expect("native hybrid signer must emit the AlgorithmId wire format");
        assert_eq!(
            signature.algorithm,
            AlgorithmId::HybridEd25519MlDsa65
        );
        assert_eq!(
            signature.signature_bytes.len(),
            AlgorithmId::HybridEd25519MlDsa65.signature_size()
        );
    }
}

#[cfg(feature = "hybrid-rc")]
mod rustcrypto_contract {
    use super::*;
    use mycelix_crypto::hybrid_sig::{verify, HybridSigner};

    #[test]
    fn rustcrypto_mldsa65_and_hybrid_sizes_match_algorithm_id() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let message = b"mycelix-rustcrypto-hybrid-wire-contract-v1";
        let signature = signer.sign(message);

        assert_eq!(
            keys.ed25519.len(),
            AlgorithmId::Ed25519.public_key_size()
        );
        assert_eq!(
            keys.ml_dsa.len(),
            AlgorithmId::MlDsa65.public_key_size()
        );
        assert_eq!(
            signature.ed25519.len(),
            AlgorithmId::Ed25519.signature_size()
        );
        assert_eq!(
            signature.ml_dsa.len(),
            AlgorithmId::MlDsa65.signature_size()
        );
        assert_eq!(
            keys.ed25519.len() + keys.ml_dsa.len(),
            AlgorithmId::HybridEd25519MlDsa65.public_key_size()
        );
        assert_eq!(
            signature.ed25519.len() + signature.ml_dsa.len(),
            AlgorithmId::HybridEd25519MlDsa65.signature_size()
        );
        assert!(verify(&keys, message, &signature).is_ok());
    }
}
