use affinidi_sd_jwt::{Disclosure, Sha256Hasher};
use reallyme_sd_jwt::{decode_disclosure, digest_disclosure, SdJwtHashAlgorithm};

#[derive(Clone, Copy, Debug)]
struct Vector {
    id: &'static str,
    section: &'static str,
    disclosure: &'static str,
    expected_sha256_b64url: &'static str,
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
        object_property: true,
    },
    Vector {
        id: "family-name",
        section: "RFC9901-5.1",
        disclosure: "WyJlbHVWNU9nM2dTTklJOEVZbnN4QV9BIiwgImZhbWlseV9uYW1lIiwgIkRvZSJd",
        expected_sha256_b64url: "TGf4oLbgwd5JQaHyKVQZU9UdGE0w5rtDsrZzfUaomLo",
        object_property: true,
    },
    Vector {
        id: "email",
        section: "RFC9901-5.1",
        disclosure: "WyI2SWo3dE0tYTVpVlBHYm9TNXRtdlZBIiwgImVtYWlsIiwgImpvaG5kb2VAZXhhbXBsZS5jb20iXQ",
        expected_sha256_b64url: "JzYjH4svliH0R3PyEMfeZu6Jt69u5qehZo7F7EPYlSE",
        object_property: true,
    },
    Vector {
        id: "flat-address",
        section: "RFC9901-5.1",
        disclosure: "WyJBSngtMDk1VlBycFR0TjRRTU9xUk9BIiwgImFkZHJlc3MiLCB7InN0cmVldF9hZGRyZXNzIjogIjEyMyBNYWluIFN0IiwgImxvY2FsaXR5IjogIkFueXRvd24iLCAicmVnaW9uIjogIkFueXN0YXRlIiwgImNvdW50cnkiOiAiVVMifV0",
        expected_sha256_b64url: "XzFrzwscM6Gn6CJDc6vVK8BkMnfG8vOSKfpPIZdAfdE",
        object_property: true,
    },
    Vector {
        id: "nationality-array-element-us",
        section: "RFC9901-5.1",
        disclosure: "WyJsa2x4RjVqTVlsR1RQVW92TU5JdkNBIiwgIlVTIl0",
        expected_sha256_b64url: "pFndjkZ_VCzmyTa6UjlZo3dh-ko8aIKQc9DlGzhaVYo",
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
    fn object_property_and_array_element_shapes_are_distinguished() {
        let hasher = Sha256Hasher;
        for vector in RFC_VECTORS {
            let parsed =
                Disclosure::parse(vector.disclosure, &hasher).expect("RFC disclosure should parse");
            assert_eq!(
                parsed.claim_name.is_some(),
                vector.object_property,
                "shape mismatch: {}",
                vector.id
            );
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
