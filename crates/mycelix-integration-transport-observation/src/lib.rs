//! Provider-neutral transport observations for exact Mycelix integration attempts.
//!
//! This crate distinguishes evidence that no application request byte was issued
//! from evidence that an external effect may have become possible. It does not
//! interpret provider business semantics and does not mint `Confirmed` or
//! `RejectedBeforeCommit` outcomes from HTTP status codes.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{
    ConnectorInstanceId, ContentCommitment, ExecutionAttemptId, ExternalOpaqueId,
    IntegrationCommandId,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const OBSERVATION_PROFILE: &str =
    "mycelix-integration-transport-observation-v1-blake3-framed";
const DOMAIN_OBSERVATION: &[u8] = b"mycelix/integration/transport-observation/v1";
const MAX_TRANSPORT_BYTES: u64 = 64 * 1024 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportFailureClass {
    NameResolution,
    Connect,
    TlsHandshake,
    RequestWrite,
    ResponseRead,
    Timeout,
    Cancelled,
    Protocol,
    LocalResource,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "termination", content = "data", rename_all = "snake_case")]
pub enum TransportTermination {
    CompleteResponse {
        status_code: u16,
        response_bytes: u64,
        headers_commitment: ContentCommitment,
        body_commitment: ContentCommitment,
        provider_request_id: Option<ExternalOpaqueId>,
    },
    Failure {
        class: TransportFailureClass,
        detail_commitment: Option<ContentCommitment>,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransportAttemptEvidence {
    pub attempt_id: ExecutionAttemptId,
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub request_commitment: ContentCommitment,
    pub planned_request_bytes: u64,
    /// Count of application/request bytes accepted by the instrumented transport
    /// write path. TLS handshake bytes are deliberately excluded.
    pub application_bytes_written: u64,
    pub write_started_at_ms: Option<i64>,
    pub write_completed_at_ms: Option<i64>,
    pub response_started_at_ms: Option<i64>,
    pub observed_at_ms: i64,
    pub termination: TransportTermination,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportObservationDisposition {
    /// The evidence says no application request byte entered the transport write
    /// path before termination. This is not itself permission to retry.
    DefinitelyNotIssued,
    /// A complete provider response was observed. Business success/rejection
    /// remains provider-semantic work for a later verifier.
    CompleteProviderResponse,
    /// At least one request byte may have left the process, or an incomplete
    /// response leaves provider-side commit state unknown.
    AmbiguousPossibleIssue,
}

/// Serializable observation record suitable for durable runtime evidence.
///
/// This object classifies transport facts only. It never grants authority,
/// retry permission, or provider business outcome semantics.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedTransportObservation {
    pub evidence: TransportAttemptEvidence,
    pub disposition: TransportObservationDisposition,
    pub observation_digest: Digest32,
    pub observation_profile: String,
}

impl QualifiedTransportObservation {
    pub const fn transport_evidence_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn provider_business_semantics_verified_here(&self) -> bool {
        false
    }

    pub const fn safe_reexecution_granted_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_transport_observation(
    evidence: TransportAttemptEvidence,
) -> Result<QualifiedTransportObservation, TransportObservationError> {
    validate_evidence(&evidence)?;
    let disposition = classify(&evidence);
    let observation_digest = observation_digest(&evidence, disposition);
    Ok(QualifiedTransportObservation {
        evidence,
        disposition,
        observation_digest,
        observation_profile: OBSERVATION_PROFILE.to_owned(),
    })
}

fn classify(evidence: &TransportAttemptEvidence) -> TransportObservationDisposition {
    match &evidence.termination {
        TransportTermination::CompleteResponse { .. } => {
            TransportObservationDisposition::CompleteProviderResponse
        }
        TransportTermination::Failure { .. } if evidence.application_bytes_written == 0 => {
            TransportObservationDisposition::DefinitelyNotIssued
        }
        TransportTermination::Failure { .. } => {
            TransportObservationDisposition::AmbiguousPossibleIssue
        }
    }
}

fn validate_evidence(evidence: &TransportAttemptEvidence) -> Result<(), TransportObservationError> {
    if evidence.planned_request_bytes == 0
        || evidence.planned_request_bytes > MAX_TRANSPORT_BYTES
        || evidence.application_bytes_written > evidence.planned_request_bytes
    {
        return Err(TransportObservationError::InvalidByteCounts);
    }
    if evidence.observed_at_ms < 0 {
        return Err(TransportObservationError::InvalidTime);
    }

    validate_timeline(
        evidence.write_started_at_ms,
        evidence.write_completed_at_ms,
        evidence.response_started_at_ms,
        evidence.observed_at_ms,
    )?;

    if evidence.application_bytes_written == 0 {
        if evidence.write_completed_at_ms.is_some() || evidence.response_started_at_ms.is_some() {
            return Err(TransportObservationError::ImpossibleZeroWriteEvidence);
        }
    } else if evidence.write_started_at_ms.is_none() {
        return Err(TransportObservationError::MissingWriteStart);
    }

    if evidence.write_completed_at_ms.is_some()
        && evidence.application_bytes_written != evidence.planned_request_bytes
    {
        return Err(TransportObservationError::CompletedWriteByteCountMismatch);
    }

    match &evidence.termination {
        TransportTermination::CompleteResponse {
            status_code,
            response_bytes,
            ..
        } => {
            if !(100..=599).contains(status_code)
                || *response_bytes == 0
                || *response_bytes > MAX_TRANSPORT_BYTES
                || evidence.response_started_at_ms.is_none()
            {
                return Err(TransportObservationError::InvalidCompleteResponse);
            }
        }
        TransportTermination::Failure { .. } => {}
    }
    Ok(())
}

fn validate_timeline(
    write_started: Option<i64>,
    write_completed: Option<i64>,
    response_started: Option<i64>,
    observed: i64,
) -> Result<(), TransportObservationError> {
    for value in [write_started, write_completed, response_started]
        .into_iter()
        .flatten()
    {
        if value < 0 || value > observed {
            return Err(TransportObservationError::InvalidTime);
        }
    }
    if let (Some(start), Some(done)) = (write_started, write_completed) {
        if done < start {
            return Err(TransportObservationError::InvalidTimeline);
        }
    }
    if let (Some(done), Some(response)) = (write_completed, response_started) {
        if response < done {
            // v0.1 intentionally requires a fully completed request write before
            // response observation so early-response/full-duplex behavior cannot
            // create ambiguous byte accounting in the canonical evidence model.
            return Err(TransportObservationError::InvalidTimeline);
        }
    }
    Ok(())
}

fn observation_digest(
    evidence: &TransportAttemptEvidence,
    disposition: TransportObservationDisposition,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_OBSERVATION);
    frame(&mut h, OBSERVATION_PROFILE.as_bytes());
    frame(&mut h, evidence.attempt_id.as_str().as_bytes());
    frame(&mut h, evidence.command_id.as_str().as_bytes());
    frame(&mut h, evidence.connector_instance.as_str().as_bytes());
    frame(&mut h, &evidence.request_commitment.digest);
    frame(&mut h, &evidence.planned_request_bytes.to_le_bytes());
    frame(
        &mut h,
        &evidence.application_bytes_written.to_le_bytes(),
    );
    frame_optional_i64(&mut h, evidence.write_started_at_ms);
    frame_optional_i64(&mut h, evidence.write_completed_at_ms);
    frame_optional_i64(&mut h, evidence.response_started_at_ms);
    frame(&mut h, &evidence.observed_at_ms.to_le_bytes());
    frame(&mut h, &[disposition_code(disposition)]);
    match &evidence.termination {
        TransportTermination::CompleteResponse {
            status_code,
            response_bytes,
            headers_commitment,
            body_commitment,
            provider_request_id,
        } => {
            frame(&mut h, &[1]);
            frame(&mut h, &status_code.to_le_bytes());
            frame(&mut h, &response_bytes.to_le_bytes());
            frame(&mut h, &headers_commitment.digest);
            frame(&mut h, &body_commitment.digest);
            frame_optional_text(
                &mut h,
                provider_request_id.as_ref().map(ExternalOpaqueId::as_str),
            );
        }
        TransportTermination::Failure {
            class,
            detail_commitment,
        } => {
            frame(&mut h, &[2]);
            frame(&mut h, &[failure_code(*class)]);
            match detail_commitment {
                Some(commitment) => {
                    frame(&mut h, &[1]);
                    frame(&mut h, &commitment.digest);
                }
                None => frame(&mut h, &[0]),
            }
        }
    }
    Digest32(*h.finalize().as_bytes())
}

fn disposition_code(value: TransportObservationDisposition) -> u8 {
    match value {
        TransportObservationDisposition::DefinitelyNotIssued => 1,
        TransportObservationDisposition::CompleteProviderResponse => 2,
        TransportObservationDisposition::AmbiguousPossibleIssue => 3,
    }
}

fn failure_code(value: TransportFailureClass) -> u8 {
    match value {
        TransportFailureClass::NameResolution => 1,
        TransportFailureClass::Connect => 2,
        TransportFailureClass::TlsHandshake => 3,
        TransportFailureClass::RequestWrite => 4,
        TransportFailureClass::ResponseRead => 5,
        TransportFailureClass::Timeout => 6,
        TransportFailureClass::Cancelled => 7,
        TransportFailureClass::Protocol => 8,
        TransportFailureClass::LocalResource => 9,
    }
}

fn frame_optional_i64(h: &mut blake3::Hasher, value: Option<i64>) {
    match value {
        Some(value) => {
            frame(h, &[1]);
            frame(h, &value.to_le_bytes());
        }
        None => frame(h, &[0]),
    }
}

fn frame_optional_text(h: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            frame(h, &[1]);
            frame(h, value.as_bytes());
        }
        None => frame(h, &[0]),
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum TransportObservationError {
    #[error("transport byte counts are invalid")]
    InvalidByteCounts,
    #[error("transport observation time is invalid")]
    InvalidTime,
    #[error("transport observation timeline is invalid")]
    InvalidTimeline,
    #[error("zero-byte issuance evidence contradicts write/response facts")]
    ImpossibleZeroWriteEvidence,
    #[error("non-zero application bytes require a write-start timestamp")]
    MissingWriteStart,
    #[error("completed request write must account for all planned bytes")]
    CompletedWriteByteCountMismatch,
    #[error("complete provider response evidence is malformed")]
    InvalidCompleteResponse,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ids() -> (ExecutionAttemptId, IntegrationCommandId, ConnectorInstanceId) {
        (
            ExecutionAttemptId::new("attempt-1").unwrap(),
            IntegrationCommandId::new("command-1").unwrap(),
            ConnectorInstanceId::new("connector-1").unwrap(),
        )
    }

    fn failure(bytes_written: u64, class: TransportFailureClass) -> TransportAttemptEvidence {
        let (attempt_id, command_id, connector_instance) = ids();
        TransportAttemptEvidence {
            attempt_id,
            command_id,
            connector_instance,
            request_commitment: ContentCommitment::sha256(b"request"),
            planned_request_bytes: 7,
            application_bytes_written: bytes_written,
            write_started_at_ms: (bytes_written > 0).then_some(10),
            write_completed_at_ms: None,
            response_started_at_ms: None,
            observed_at_ms: 20,
            termination: TransportTermination::Failure {
                class,
                detail_commitment: None,
            },
        }
    }

    #[test]
    fn zero_application_bytes_classify_as_definitely_not_issued() {
        let qualified = qualify_transport_observation(failure(
            0,
            TransportFailureClass::Connect,
        ))
        .unwrap();
        assert_eq!(
            qualified.disposition,
            TransportObservationDisposition::DefinitelyNotIssued
        );
        assert!(!qualified.safe_reexecution_granted_here());
    }

    #[test]
    fn one_application_byte_makes_timeout_ambiguous() {
        let qualified = qualify_transport_observation(failure(
            1,
            TransportFailureClass::Timeout,
        ))
        .unwrap();
        assert_eq!(
            qualified.disposition,
            TransportObservationDisposition::AmbiguousPossibleIssue
        );
    }

    #[test]
    fn complete_http_error_status_is_observation_not_business_rejection() {
        let (attempt_id, command_id, connector_instance) = ids();
        let evidence = TransportAttemptEvidence {
            attempt_id,
            command_id,
            connector_instance,
            request_commitment: ContentCommitment::sha256(b"request"),
            planned_request_bytes: 7,
            application_bytes_written: 7,
            write_started_at_ms: Some(10),
            write_completed_at_ms: Some(11),
            response_started_at_ms: Some(12),
            observed_at_ms: 13,
            termination: TransportTermination::CompleteResponse {
                status_code: 500,
                response_bytes: 4,
                headers_commitment: ContentCommitment::sha256(b"headers"),
                body_commitment: ContentCommitment::sha256(b"body"),
                provider_request_id: None,
            },
        };
        let qualified = qualify_transport_observation(evidence).unwrap();
        assert_eq!(
            qualified.disposition,
            TransportObservationDisposition::CompleteProviderResponse
        );
        assert!(!qualified.provider_business_semantics_verified_here());
    }

    #[test]
    fn byte_count_overflow_fails_closed() {
        let mut evidence = failure(0, TransportFailureClass::Connect);
        evidence.application_bytes_written = 8;
        evidence.write_started_at_ms = Some(10);
        assert_eq!(
            qualify_transport_observation(evidence),
            Err(TransportObservationError::InvalidByteCounts)
        );
    }
}
