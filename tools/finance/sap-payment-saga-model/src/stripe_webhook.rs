//! Stripe-style HMAC-SHA256 webhook signature verification over exact raw bytes.
//!
//! This authenticates possession of a configured signing secret and integrity
//! of the received bytes. It does NOT authenticate payment settlement, refund
//! finality, or the truth of arbitrary event fields.

use hmac::{Hmac, Mac};
use sha2::{Digest, Sha256};

type HmacSha256 = Hmac<Sha256>;
pub const STRIPE_DEFAULT_TOLERANCE_SECONDS: u64 = 300;
const MAX_SIGNATURE_HEADER_BYTES: usize = 16 * 1024;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StripeWebhookVerifyError {
    EmptySecret,
    EmptyHeader,
    HeaderTooLarge,
    InvalidHeaderEncoding,
    InvalidHeaderField,
    DuplicateTimestamp,
    MissingTimestamp,
    MissingV1Signature,
    InvalidTimestamp,
    InvalidSignatureEncoding,
    InvalidTolerance,
    TimestampOutsideTolerance,
    SignatureMismatch,
}

impl std::fmt::Display for StripeWebhookVerifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for StripeWebhookVerifyError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StripeWebhookVerification {
    pub timestamp_unix_seconds: u64,
    pub raw_body_sha256: String,
    pub matching_v1_signature_count: u16,
}

pub fn verify_stripe_webhook(
    raw_body: &[u8],
    signature_header: &str,
    signing_secret: &[u8],
    now_unix_seconds: u64,
    tolerance_seconds: u64,
) -> Result<StripeWebhookVerification, StripeWebhookVerifyError> {
    if signing_secret.is_empty() {
        return Err(StripeWebhookVerifyError::EmptySecret);
    }
    if tolerance_seconds == 0 {
        return Err(StripeWebhookVerifyError::InvalidTolerance);
    }
    if signature_header.is_empty() {
        return Err(StripeWebhookVerifyError::EmptyHeader);
    }
    if signature_header.len() > MAX_SIGNATURE_HEADER_BYTES {
        return Err(StripeWebhookVerifyError::HeaderTooLarge);
    }
    if !signature_header.is_ascii() {
        return Err(StripeWebhookVerifyError::InvalidHeaderEncoding);
    }

    let mut timestamp_raw: Option<&str> = None;
    let mut timestamp_value: Option<u64> = None;
    let mut signatures: Vec<&str> = Vec::new();

    for field in signature_header.split(',') {
        let field = field.trim();
        if field.is_empty() {
            return Err(StripeWebhookVerifyError::InvalidHeaderField);
        }
        let (name, value) = field
            .split_once('=')
            .ok_or(StripeWebhookVerifyError::InvalidHeaderField)?;
        let name = name.trim();
        let value = value.trim();
        if name.is_empty() || value.is_empty() {
            return Err(StripeWebhookVerifyError::InvalidHeaderField);
        }
        match name {
            "t" => {
                if timestamp_raw.is_some() {
                    return Err(StripeWebhookVerifyError::DuplicateTimestamp);
                }
                let parsed = value
                    .parse::<u64>()
                    .map_err(|_| StripeWebhookVerifyError::InvalidTimestamp)?;
                timestamp_raw = Some(value);
                timestamp_value = Some(parsed);
            }
            "v1" => signatures.push(value),
            _ => {}
        }
    }

    let timestamp_raw = timestamp_raw.ok_or(StripeWebhookVerifyError::MissingTimestamp)?;
    let timestamp = timestamp_value.ok_or(StripeWebhookVerifyError::MissingTimestamp)?;
    if signatures.is_empty() {
        return Err(StripeWebhookVerifyError::MissingV1Signature);
    }
    let mut preimage = Vec::with_capacity(timestamp_raw.len() + 1 + raw_body.len());
    preimage.extend_from_slice(timestamp_raw.as_bytes());
    preimage.push(b'.');
    preimage.extend_from_slice(raw_body);

    let mut matching = 0_u16;
    let mut any_well_formed = false;
    for signature in signatures {
        let decoded = match decode_sha256_hex(signature) {
            Some(value) => value,
            None => continue,
        };
        any_well_formed = true;
        let mut mac =
            HmacSha256::new_from_slice(signing_secret).expect("HMAC accepts keys of every length");
        mac.update(&preimage);
        if mac.verify_slice(&decoded).is_ok() {
            matching = matching.saturating_add(1);
        }
    }

    if !any_well_formed {
        return Err(StripeWebhookVerifyError::InvalidSignatureEncoding);
    }
    if matching == 0 {
        return Err(StripeWebhookVerifyError::SignatureMismatch);
    }

    // Authenticate first, then apply the replay window to the signed timestamp.
    if now_unix_seconds.abs_diff(timestamp) > tolerance_seconds {
        return Err(StripeWebhookVerifyError::TimestampOutsideTolerance);
    }

    let body_digest = Sha256::digest(raw_body);
    let mut raw_body_sha256 = String::with_capacity(71);
    raw_body_sha256.push_str("sha256:");
    for byte in body_digest {
        use std::fmt::Write as _;
        write!(&mut raw_body_sha256, "{byte:02x}").expect("writing to String cannot fail");
    }

    Ok(StripeWebhookVerification {
        timestamp_unix_seconds: timestamp,
        raw_body_sha256,
        matching_v1_signature_count: matching,
    })
}

/// Verify against every currently active endpoint secret during secret rotation.
/// Returns on the first matching secret; no candidate secret is exposed.
pub fn verify_stripe_webhook_with_secrets(
    raw_body: &[u8],
    signature_header: &str,
    active_signing_secrets: &[&[u8]],
    now_unix_seconds: u64,
    tolerance_seconds: u64,
) -> Result<StripeWebhookVerification, StripeWebhookVerifyError> {
    if active_signing_secrets.is_empty() || active_signing_secrets.iter().all(|s| s.is_empty()) {
        return Err(StripeWebhookVerifyError::EmptySecret);
    }
    for secret in active_signing_secrets
        .iter()
        .filter(|secret| !secret.is_empty())
    {
        match verify_stripe_webhook(
            raw_body,
            signature_header,
            secret,
            now_unix_seconds,
            tolerance_seconds,
        ) {
            Ok(verified) => return Ok(verified),
            Err(StripeWebhookVerifyError::SignatureMismatch) => {}
            Err(error) => return Err(error),
        }
    }
    Err(StripeWebhookVerifyError::SignatureMismatch)
}

fn decode_sha256_hex(value: &str) -> Option<[u8; 32]> {
    if value.len() != 64 || !value.is_ascii() {
        return None;
    }
    let bytes = value.as_bytes();
    let mut output = [0_u8; 32];
    for index in 0..32 {
        let high = decode_hex_nibble(bytes[index * 2])?;
        let low = decode_hex_nibble(bytes[index * 2 + 1])?;
        output[index] = (high << 4) | low;
    }
    Some(output)
}

fn decode_hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        b'A'..=b'F' => Some(value - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SECRET: &[u8] = b"whsec_test_secret_2026";
    const TIMESTAMP: u64 = 1_794_000_000;
    const BODY: &[u8] = br#"{"id":"evt_test_123","object":"event","type":"refund.updated"}"#;
    const SIGNATURE: &str = "7379bbb016143ced7ebf6f8083e935c452f08f6c7d3716de1715a9716ea21912";

    #[test]
    fn shared_known_answer_verifies_exact_raw_body() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        let verified = verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP + 10, 300).unwrap();
        assert_eq!(verified.timestamp_unix_seconds, TIMESTAMP);
        assert_eq!(
            verified.raw_body_sha256,
            "sha256:c975a1417a20a38f83fab9b3877f56da003b9da45d3b34e1f3f83b3aa89fbc6f"
        );
        assert_eq!(verified.matching_v1_signature_count, 1);
    }

    #[test]
    fn rotated_v1_signature_accepts_any_matching_candidate_and_ignores_unknown_versions() {
        let header = format!(
            "v0=legacy,t={TIMESTAMP},v1={},v1={SIGNATURE},future=metadata",
            "00".repeat(32)
        );
        let verified = verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP, 300).unwrap();
        assert_eq!(verified.matching_v1_signature_count, 1);
    }

    #[test]
    fn whitespace_and_header_order_do_not_change_the_signed_preimage() {
        let header = format!(" v1={SIGNATURE} , t={TIMESTAMP} ");
        assert!(verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP, 300).is_ok());
    }

    #[test]
    fn modified_raw_bytes_fail_signature_verification() {
        let changed = br#"{"id":"evt_test_123","object":"event","type":"refund.failed"}"#;
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert_eq!(
            verify_stripe_webhook(changed, &header, SECRET, TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::SignatureMismatch)
        );
    }

    #[test]
    fn wrong_secret_fails_signature_verification() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert_eq!(
            verify_stripe_webhook(BODY, &header, b"wrong-secret", TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::SignatureMismatch)
        );
    }

    #[test]
    fn stale_and_future_timestamps_are_both_rejected() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert_eq!(
            verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP + 301, 300),
            Err(StripeWebhookVerifyError::TimestampOutsideTolerance)
        );
        assert_eq!(
            verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP - 301, 300),
            Err(StripeWebhookVerifyError::TimestampOutsideTolerance)
        );
    }

    #[test]
    fn timestamp_tolerance_boundary_is_inclusive() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert!(verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP + 300, 300).is_ok());
        assert_eq!(
            verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP + 301, 300),
            Err(StripeWebhookVerifyError::TimestampOutsideTolerance)
        );
    }

    #[test]
    fn duplicate_or_missing_timestamp_is_rejected() {
        let duplicate = format!("t={TIMESTAMP},t={TIMESTAMP},v1={SIGNATURE}");
        assert_eq!(
            verify_stripe_webhook(BODY, &duplicate, SECRET, TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::DuplicateTimestamp)
        );
        assert_eq!(
            verify_stripe_webhook(BODY, &format!("v1={SIGNATURE}"), SECRET, TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::MissingTimestamp)
        );
    }

    #[test]
    fn missing_malformed_or_empty_signature_is_rejected() {
        assert_eq!(
            verify_stripe_webhook(
                BODY,
                &format!("t={TIMESTAMP},v0={SIGNATURE}"),
                SECRET,
                TIMESTAMP,
                300
            ),
            Err(StripeWebhookVerifyError::MissingV1Signature)
        );
        assert_eq!(
            verify_stripe_webhook(
                BODY,
                &format!("t={TIMESTAMP},v1=not-hex"),
                SECRET,
                TIMESTAMP,
                300
            ),
            Err(StripeWebhookVerifyError::InvalidSignatureEncoding)
        );
        assert_eq!(
            verify_stripe_webhook(BODY, &format!("t={TIMESTAMP},v1="), SECRET, TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::InvalidHeaderField)
        );
    }

    #[test]
    fn missing_secret_header_and_oversized_header_fail_closed() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert_eq!(
            verify_stripe_webhook(BODY, &header, b"", TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::EmptySecret)
        );
        assert_eq!(
            verify_stripe_webhook(BODY, "", SECRET, TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::EmptyHeader)
        );
        let oversized = "x".repeat(16 * 1024 + 1);
        assert_eq!(
            verify_stripe_webhook(BODY, &oversized, SECRET, TIMESTAMP, 300),
            Err(StripeWebhookVerifyError::HeaderTooLarge)
        );
    }

    #[test]
    fn unknown_or_duplicate_v1_candidates_do_not_bypass_hmac_verification() {
        let header = format!("t={TIMESTAMP},v1={},v1={SIGNATURE}", "11".repeat(32));
        let verified = verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP, 300).unwrap();
        assert_eq!(verified.matching_v1_signature_count, 1);
    }

    #[test]
    fn zero_tolerance_is_rejected_instead_of_disabling_replay_protection() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert_eq!(
            verify_stripe_webhook(BODY, &header, SECRET, TIMESTAMP, 0),
            Err(StripeWebhookVerifyError::InvalidTolerance)
        );
    }

    #[test]
    fn secret_rotation_accepts_any_active_endpoint_secret() {
        let header = format!("t={TIMESTAMP},v1={SIGNATURE}");
        assert!(
            verify_stripe_webhook_with_secrets(
                BODY,
                &header,
                &[b"old-secret", SECRET],
                TIMESTAMP + 1,
                STRIPE_DEFAULT_TOLERANCE_SECONDS,
            )
            .is_ok()
        );
        assert_eq!(
            verify_stripe_webhook_with_secrets(
                BODY,
                &header,
                &[b"old-secret", b"previous-secret"],
                TIMESTAMP + 1,
                STRIPE_DEFAULT_TOLERANCE_SECONDS,
            ),
            Err(StripeWebhookVerifyError::SignatureMismatch)
        );
    }

    #[test]
    fn shared_fixture_pins_timestamp_signature_and_raw_body_digest() {
        let fixture = include_str!("../fixtures/sap-provider-contract-v1.kat");
        let value = |name: &str| {
            fixture
                .lines()
                .find_map(|line| line.strip_prefix(&format!("{name}=")))
                .unwrap_or_else(|| panic!("missing shared KAT value {name}"))
        };
        assert_eq!(value("stripe_webhook_timestamp"), TIMESTAMP.to_string());
        assert_eq!(value("stripe_webhook_signature_v1"), SIGNATURE);
        let header = format!(
            "t={},v1={}",
            value("stripe_webhook_timestamp"),
            value("stripe_webhook_signature_v1")
        );
        let verified = verify_stripe_webhook(
            BODY,
            &header,
            SECRET,
            TIMESTAMP + 10,
            STRIPE_DEFAULT_TOLERANCE_SECONDS,
        )
        .unwrap();
        assert_eq!(
            verified.raw_body_sha256,
            value("stripe_webhook_raw_body_sha256")
        );
    }
}
