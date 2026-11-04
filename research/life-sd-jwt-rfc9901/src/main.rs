use affinidi_sd_jwt::{Disclosure, Sha256Hasher};
use reallyme_sd_jwt::{decode_disclosure, digest_disclosure, DisclosureKind, SdJwtHashAlgorithm};

#[derive(Clone, Copy, Debug)]
struct Vector {
    id: &'static str,
    section: &'static str,
    disclosure: &'static str,
    expected_sha256_b64url: &'static str,
    expected_salt: &'static str,
    expected_claim_name: Option<&'static str>,
    expected_scalar_value: Option<&'static str>,
    expected_address_value: bool,
    object_property: bool,
}

// Encoded disclosure strings and SHA-256 digests are the published examples in
// RFC 9901 §5.1. These are specification examples, not an exhaustive test suite.
const RFC_VECTORS: [Vector; 5] = [
    Vector {
        id: "given-name",
        section: "RFC9901-5.1",
        disclosure: "WyIyR0xDNDJzS1F2ZUNmR2ZyeU5STjl3IiwgImdpdmVuX25hbWUiLCAiSm9obiJd",
        expected_sha256_b64url: "jsu9yVulwQQlhFlM_3JlzMaSFzglhQG0DpfayQwLUK4",
        expected_salt: "2GLC42sKQveCfGfryNRN9w",
        expected_claim_name: Some("given_name"),
        expected_scalar_value: Some("John"),
        expected_address_value: false,
        object_property: true,
    },
    Vector {
        id: "family-name",
        section: "RFC9901-5.1",
        disclosure: "WyJlbHVWNU9nM2dTTklJOEVZbnN4QV9BIiwgImZhbWlseV9uYW1lIiwgIkRvZSJd",
        expected_sha256_b64url: "TGf4oLbgwd5JQaHyKVQZU9UdGE0w5rtDsrZzfUaomLo",
        expected_salt: "eluV5Og3gSNII8EYnsxA_A",
        expected_claim_name: Some("family_name"),
        expected_scalar_value: Some("Doe"),
        expected_address_value: false,
        object_property: true,
    },
    Vector {
        id: "email",
        section: "RFC9901-5.1",
        disclosure: "WyI2SWo3dE0tYTVpVlBHYm9TNXRtdlZBIiwgImVtYWlsIiwgImpvaG5kb2VAZXhhbXBsZS5jb20iXQ",
        expected_sha256_b64url: "JzYjH4svliH0R3PyEMfeZu6Jt69u5qehZo7F7EPYlSE",
        expected_salt: "6Ij7tM-a5iVPGboS5tmvVA",
        expected_claim_name: Some("email"),
        expected_scalar_value: Some("johndoe@example.com"),
        expected_address_value: false,
        object_property: true,
    },
    Vector {
        id: "flat-address",
        section: "RFC9901-5.1",
        disclosure: "WyJBSngtMDk1VlBycFR0TjRRTU9xUk9BIiwgImFkZHJlc3MiLCB7InN0cmVldF9hZGRyZXNzIjogIjEyMyBNYWluIFN0IiwgImxvY2FsaXR5IjogIkFueXRvd24iLCAicmVnaW9uIjogIkFueXN0YXRlIiwgImNvdW50cnkiOiAiVVMifV0",
        expected_sha256_b64url: "XzFrzwscM6Gn6CJDc6vVK8BkMnfG8vOSKfpPIZdAfdE",
        expected_salt: "AJx-095VPrpTtN4QMOqROA",
        expected_claim_name: Some("address"),
        expected_scalar_value: None,
        expected_address_value: true,
        object_property: true,
    },
    Vector {
        id: "nationality-array-element-us",
        section: "RFC9901-5.1",
        disclosure: "WyJsa2x4RjVqTVlsR1RQVW92TU5JdkNBIiwgIlVTIl0",
        expected_sha256_b64url: "pFndjkZ_VCzmyTa6UjlZo3dh-ko8aIKQc9DlGzhaVYo",
        expected_salt: "lklxF5jMYlGTPUovMNIvCA",
        expected_claim_name: None,
        expected_scalar_value: Some("US"),
        expected_address_value: false,
        object_property: false,
    },
];

fn digest_pair(encoded: &str) -> Result<(String, String), String> {
    let hasher = Sha256Hasher;
    let affinidi = Disclosure::parse(encoded, &hasher)
        .map_err(|error| format!("affinidi-sd-jwt parse failed: {error}"))?;

    decode_disclosure(encoded)
        .map_err(|error| format!("reallyme-sd-jwt parse failed: {error}"))?;

    let reallyme = digest_disclosure(encoded, SdJwtHashAlgorithm::Sha256)
        .map_err(|error| format!("reallyme-sd-jwt digest failed: {error}"))?;

    Ok((affinidi.digest, reallyme))
}

fn run() -> Result<(), String> {
    println!(
        "vector_id,source_section,expected_sha256_b64url,affinidi_sd_jwt_0_1_5,reallyme_sd_jwt_0_4_1,match"
    );

    for vector in RFC_VECTORS {
        let (affinidi_digest, reallyme_digest) = digest_pair(vector.disclosure)?;
        let matches = affinidi_digest == vector.expected_sha256_b64url
            && reallyme_digest == vector.expected_sha256_b64url
            && affinidi_digest == reallyme_digest;

        println!(
            "{},{},{},{},{},{}",
            vector.id,
            vector.section,
            vector.expected_sha256_b64url,
            affinidi_digest,
            reallyme_digest,
            if matches { "PASS" } else { "FAIL" }
        );

        if !matches {
            return Err(format!("digest mismatch for vector {}", vector.id));
        }

        let hasher = Sha256Hasher;
        let parsed = Disclosure::parse(vector.disclosure, &hasher)
            .map_err(|error| format!("affinidi-sd-jwt parse failed: {error}"))?;
        if parsed.claim_name.is_some() != vector.object_property {
            return Err(format!("disclosure shape mismatch for vector {}", vector.id));
        }
    }

    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn published_rfc_9901_digests_match_both_implementations() {
        for vector in RFC_VECTORS {
            let (affinidi, reallyme) =
                digest_pair(vector.disclosure).expect("RFC disclosure should parse");
            assert_eq!(affinidi, vector.expected_sha256_b64url, "{}", vector.id);
            assert_eq!(reallyme, vector.expected_sha256_b64url, "{}", vector.id);
            assert_eq!(affinidi, reallyme, "implementations disagree: {}", vector.id);
        }
    }

    #[test]
    fn published_disclosure_salts_shapes_and_claim_contents_match_rfc() {
        let hasher = Sha256Hasher;
        for vector in RFC_VECTORS {
            let parsed =
                Disclosure::parse(vector.disclosure, &hasher).expect("RFC disclosure should parse");
            assert_eq!(parsed.salt, vector.expected_salt, "salt: {}", vector.id);
            assert_eq!(
                parsed.claim_name.as_deref(),
                vector.expected_claim_name,
                "claim name: {}",
                vector.id
            );
            assert_eq!(
                parsed.claim_name.is_some(),
                vector.object_property,
                "shape: {}",
                vector.id
            );

            if let Some(expected) = vector.expected_scalar_value {
                assert_eq!(
                    parsed.claim_value.as_str(),
                    Some(expected),
                    "scalar value: {}",
                    vector.id
                );
            }

            if vector.expected_address_value {
                assert_eq!(
                    parsed.claim_value["street_address"].as_str(),
                    Some("123 Main St"),
                    "street address: {}",
                    vector.id
                );
                assert_eq!(
                    parsed.claim_value["locality"].as_str(),
                    Some("Anytown"),
                    "locality: {}",
                    vector.id
                );
                assert_eq!(
                    parsed.claim_value["region"].as_str(),
                    Some("Anystate"),
                    "region: {}",
                    vector.id
                );
                assert_eq!(
                    parsed.claim_value["country"].as_str(),
                    Some("US"),
                    "country: {}",
                    vector.id
                );
            }

            let independently_parsed = decode_disclosure(vector.disclosure)
                .expect("published disclosure should parse in reallyme");
            assert_eq!(
                independently_parsed.salt(),
                vector.expected_salt,
                "reallyme salt: {}",
                vector.id
            );
            match independently_parsed.kind() {
                DisclosureKind::ObjectProperty { claim_name, claim_value } => {
                    assert_eq!(
                        Some(claim_name.as_str()),
                        vector.expected_claim_name,
                        "reallyme claim name: {}",
                        vector.id
                    );
                    if let Some(expected) = vector.expected_scalar_value {
                        assert_eq!(
                            claim_value.as_str(),
                            Some(expected),
                            "reallyme scalar value: {}",
                            vector.id
                        );
                    }
                    if vector.expected_address_value {
                        assert_eq!(claim_value["street_address"].as_str(), Some("123 Main St"));
                        assert_eq!(claim_value["locality"].as_str(), Some("Anytown"));
                        assert_eq!(claim_value["region"].as_str(), Some("Anystate"));
                        assert_eq!(claim_value["country"].as_str(), Some("US"));
                    }
                }
                DisclosureKind::ArrayElement { claim_value } => {
                    assert_eq!(vector.expected_claim_name, None, "reallyme array shape: {}", vector.id);
                    assert_eq!(
                        claim_value.as_str(),
                        vector.expected_scalar_value,
                        "reallyme array value: {}",
                        vector.id
                    );
                }
                _ => panic!("unexpected disclosure kind for {}", vector.id),
            }
        }
    }

    #[test]
    fn both_parsers_reject_invalid_base64url_disclosure() {
        let hasher = Sha256Hasher;
        assert!(Disclosure::parse("%%%invalid%%%", &hasher).is_err());
        assert!(decode_disclosure("%%%invalid%%%").is_err());
    }

    #[test]
    fn both_parsers_reject_padded_base64url_disclosure() {
        let hasher = Sha256Hasher;
        let padded = format!("{}=", RFC_VECTORS[0].disclosure);
        assert!(Disclosure::parse(&padded, &hasher).is_err());
        assert!(decode_disclosure(&padded).is_err());
    }

    #[test]
    fn both_parsers_reject_malformed_json_and_invalid_disclosure_shapes() {
        use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine as _};

        let hasher = Sha256Hasher;
        let invalid_json = URL_SAFE_NO_PAD.encode(b"{not json");
        let invalid_shapes = [
            URL_SAFE_NO_PAD.encode(br#"["salt"]"#),
            URL_SAFE_NO_PAD.encode(br#"["salt","name","value","extra"]"#),
            URL_SAFE_NO_PAD.encode(br#"[7,"name","value"]"#),
            URL_SAFE_NO_PAD.encode(br#"["salt",7,"value"]"#),
        ];

        assert!(Disclosure::parse(&invalid_json, &hasher).is_err());
        assert!(decode_disclosure(&invalid_json).is_err());

        for encoded in invalid_shapes {
            assert!(Disclosure::parse(&encoded, &hasher).is_err(), "{encoded}");
            assert!(decode_disclosure(&encoded).is_err(), "{encoded}");
        }
    }

    #[test]
    fn distinct_published_disclosures_have_distinct_digests() {
        let digests: Vec<String> = RFC_VECTORS
            .iter()
            .map(|vector| {
                digest_pair(vector.disclosure)
                    .expect("RFC disclosure should parse")
                    .0
            })
            .collect();
        let mut unique = digests.clone();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), digests.len());
    }
}
