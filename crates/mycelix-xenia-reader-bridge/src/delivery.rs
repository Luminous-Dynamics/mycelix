use std::sync::Arc;

use mycelix_content_node::LocalCasV1;
use mycelix_infrastructure_types::StableIdV1;
use mycelix_reader_enrollment::{ReaderEnrollmentErrorV1, XeniaHybridReaderCredentialV1};
use thiserror::Error;
use xenia_peer_core::{
    AuthenticatedPeerApplicationChannelV1,
    transport::Transport,
};

use crate::{AuthorizedNixReadV1, response::XeniaNixResponseStreamErrorV1};

/// Fail-closed failures while binding one CF-07C3 request authorization to the
/// exact authenticated Xenia channel used for its response.
#[derive(Debug, Error)]
pub enum XeniaNixResponseDeliveryErrorV1 {
    /// The outbound channel belongs to a different authenticated handshake
    /// transcript generation than the request that produced the authorization.
    #[error("authorized request transcript does not match outbound Xenia channel")]
    HandshakeTranscriptMismatch,
    /// The outbound channel's negotiated authenticated context differs from the
    /// request generation that produced the authorization.
    #[error("authorized request negotiated context does not match outbound Xenia channel")]
    NegotiatedContextMismatch,
    /// The outbound channel authenticated a different exact Ed25519 + ML-DSA-65
    /// credential than the one enrolled for the authorized request.
    #[error("authorized request hybrid credential does not match outbound Xenia channel")]
    AuthenticatedCredentialMismatch,
    /// The sealed Xenia peer keys could not be represented as the exact
    /// CF-07C1 hybrid credential profile.
    #[error(transparent)]
    ReaderEnrollment(#[from] ReaderEnrollmentErrorV1),
    /// Exact response preparation/framing/deadline/Xenia-send failure after the
    /// channel-generation binding was established.
    #[error(transparent)]
    Response(#[from] XeniaNixResponseStreamErrorV1),
}

/// Send one authorized Nix representation only through the exact authenticated
/// Xenia channel generation and hybrid peer credential that produced the
/// request authority.
///
/// The channel and authorization are both consumed by value. Before any CAS
/// verification or response send, this function compares the authorization's
/// transcript/context lineage against the sealed outbound channel and
/// recomputes the CF-07C1 credential commitment from that channel's exact
/// authenticated Ed25519 + ML-DSA-65 public keys.
///
/// A valid authorization therefore cannot be redirected onto another peer's
/// otherwise-valid authenticated Xenia channel. The channel is returned only
/// after the lower CF-07C5 response stream sends a complete `End` frame.
pub async fn send_authorized_nix_read_over_xenia_v1<T: Transport>(
    channel: AuthenticatedPeerApplicationChannelV1<T>,
    authorized: AuthorizedNixReadV1,
    cas: Arc<LocalCasV1>,
) -> Result<AuthenticatedPeerApplicationChannelV1<T>, XeniaNixResponseDeliveryErrorV1> {
    let handshake = channel.handshake();
    validate_delivery_lineage_v1(
        authorized.transcript_hash(),
        authorized.negotiated_context_hash(),
        authorized.credential_id(),
        handshake.transcript_hash(),
        handshake.negotiated_context_hash(),
        handshake.peer_ed25519_public_key(),
        handshake.peer_ml_dsa_public_key(),
    )?;

    crate::response::send_authorized_nix_read_over_xenia_v1(channel, authorized, cas)
        .await
        .map_err(Into::into)
}

#[allow(clippy::too_many_arguments)]
fn validate_delivery_lineage_v1(
    expected_transcript_hash: [u8; 32],
    expected_context_hash: Option<[u8; 32]>,
    expected_credential_id: StableIdV1,
    actual_transcript_hash: [u8; 32],
    actual_context_hash: Option<[u8; 32]>,
    actual_ed25519_public_key: [u8; 32],
    actual_ml_dsa_65_public_key: &[u8],
) -> Result<(), XeniaNixResponseDeliveryErrorV1> {
    if actual_transcript_hash != expected_transcript_hash {
        return Err(XeniaNixResponseDeliveryErrorV1::HandshakeTranscriptMismatch);
    }
    if actual_context_hash != expected_context_hash {
        return Err(XeniaNixResponseDeliveryErrorV1::NegotiatedContextMismatch);
    }
    let actual_credential = XeniaHybridReaderCredentialV1::new(
        actual_ed25519_public_key,
        actual_ml_dsa_65_public_key.to_vec(),
    )?;
    if actual_credential.id() != expected_credential_id {
        return Err(XeniaNixResponseDeliveryErrorV1::AuthenticatedCredentialMismatch);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use mycelix_reader_enrollment::XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1;

    use super::*;

    fn credential_id(ed: u8, ml: u8) -> StableIdV1 {
        XeniaHybridReaderCredentialV1::new(
            [ed; 32],
            vec![ml; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1],
        )
        .unwrap()
        .id()
    }

    #[test]
    fn exact_generation_and_hybrid_credential_match() {
        let transcript = [0x11; 32];
        let context = Some([0x22; 32]);
        let expected = credential_id(0x33, 0x44);
        validate_delivery_lineage_v1(
            transcript,
            context,
            expected,
            transcript,
            context,
            [0x33; 32],
            &[0x44; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1],
        )
        .unwrap();
    }

    #[test]
    fn different_transcript_cannot_redirect_authority() {
        let error = validate_delivery_lineage_v1(
            [0x11; 32],
            Some([0x22; 32]),
            credential_id(0x33, 0x44),
            [0x12; 32],
            Some([0x22; 32]),
            [0x33; 32],
            &[0x44; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1],
        )
        .unwrap_err();
        assert!(matches!(
            error,
            XeniaNixResponseDeliveryErrorV1::HandshakeTranscriptMismatch
        ));
    }

    #[test]
    fn same_transcript_with_different_context_is_rejected() {
        let error = validate_delivery_lineage_v1(
            [0x11; 32],
            Some([0x22; 32]),
            credential_id(0x33, 0x44),
            [0x11; 32],
            Some([0x23; 32]),
            [0x33; 32],
            &[0x44; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1],
        )
        .unwrap_err();
        assert!(matches!(
            error,
            XeniaNixResponseDeliveryErrorV1::NegotiatedContextMismatch
        ));
    }

    #[test]
    fn ed25519_only_match_cannot_redirect_authority() {
        let error = validate_delivery_lineage_v1(
            [0x11; 32],
            None,
            credential_id(0x33, 0x44),
            [0x11; 32],
            None,
            [0x33; 32],
            &[0x45; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1],
        )
        .unwrap_err();
        assert!(matches!(
            error,
            XeniaNixResponseDeliveryErrorV1::AuthenticatedCredentialMismatch
        ));
    }

    #[test]
    fn ml_dsa_only_match_cannot_redirect_authority() {
        let error = validate_delivery_lineage_v1(
            [0x11; 32],
            None,
            credential_id(0x33, 0x44),
            [0x11; 32],
            None,
            [0x34; 32],
            &[0x44; XENIA_ML_DSA_65_PUBLIC_KEY_LEN_V1],
        )
        .unwrap_err();
        assert!(matches!(
            error,
            XeniaNixResponseDeliveryErrorV1::AuthenticatedCredentialMismatch
        ));
    }
}
