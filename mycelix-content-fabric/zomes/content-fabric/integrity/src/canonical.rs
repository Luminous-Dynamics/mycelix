use crate::types::{ContentDigestRefV1, DigestAlgorithmV1, ProviderAdvertisementV1};

const ENDPOINT_BINDING_DOMAIN_V1: &[u8] = b"mycelix-content-fabric/provider-endpoint-binding/v1";

fn append_field(out: &mut Vec<u8>, bytes: &[u8]) {
    let len = u32::try_from(bytes.len()).expect("v1 canonical field lengths fit u32");
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
}

/// Serializer-independent bytes signed by the advertised Iroh endpoint key.
/// The signature field itself is intentionally excluded.
pub fn provider_binding_bytes_v1(ad: &ProviderAdvertisementV1) -> Vec<u8> {
    let mut out = Vec::new();
    append_field(&mut out, ENDPOINT_BINDING_DOMAIN_V1);
    out.extend_from_slice(&ad.schema_version.to_be_bytes());
    append_field(&mut out, ad.provider.get_raw_39());
    append_field(&mut out, ad.binding_prev_action.get_raw_39());
    append_field(&mut out, &ad.iroh_endpoint_id);
    append_field(&mut out, ad.protocol.as_bytes());
    out.extend_from_slice(&ad.max_blob_size_bytes.to_be_bytes());
    out.extend_from_slice(&ad.ttl_seconds.to_be_bytes());

    out.push(ad.supported_algorithms.len() as u8);
    for algorithm in &ad.supported_algorithms {
        out.push(algorithm.wire_tag());
    }

    out.push(ad.failure_domains.len() as u8);
    for claim in &ad.failure_domains {
        out.push(claim.kind.wire_tag());
        append_field(&mut out, claim.value.as_bytes());
    }
    out
}

pub fn all_providers_anchor_v1() -> &'static str {
    "providers/all"
}

pub fn algorithm_anchor_v1(algorithm: DigestAlgorithmV1) -> String {
    format!("algorithm/{}", algorithm.token())
}

pub fn digest_anchor_v1(digest: &ContentDigestRefV1) -> String {
    format!(
        "digest/{}/{}",
        digest.algorithm.token(),
        hex::encode(digest.bytes)
    )
}

pub fn is_valid_anchor_v1(value: &str) -> bool {
    if value == all_providers_anchor_v1() {
        return true;
    }
    if value == algorithm_anchor_v1(DigestAlgorithmV1::Blake3_256)
        || value == algorithm_anchor_v1(DigestAlgorithmV1::Sha256)
    {
        return true;
    }
    for algorithm in [DigestAlgorithmV1::Blake3_256, DigestAlgorithmV1::Sha256] {
        let prefix = format!("digest/{}/", algorithm.token());
        if let Some(hex_digest) = value.strip_prefix(&prefix) {
            return hex_digest.len() == 64
                && hex_digest
                    .as_bytes()
                    .iter()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(byte));
        }
    }
    false
}
