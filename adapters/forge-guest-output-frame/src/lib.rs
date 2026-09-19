// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! FORGE-004D3B2D2B: bounded framed stdout transport for raw guest evidence.
//!
//! The frame is transport, not authority. The parent must still re-qualify the
//! decoded [`GuestEvidenceEnvelopeV1`] against the exact plan/tool-map subjects.

use mycelix_forge_guest_envelope::GuestEvidenceEnvelopeV1;
use serde::de::DeserializeOwned;
use sha2::{Digest as _, Sha256};
use std::io::{Read, Write};
use thiserror::Error;

const MAGIC: &[u8; 8] = b"MXFGOUT1";
const DIGEST_LEN: usize = 32;
pub const MAX_GUEST_ENVELOPE_BYTES: u64 = 16 * 1024 * 1024;

pub fn write_guest_envelope_frame<W: Write>(
    mut writer: W,
    envelope: &GuestEvidenceEnvelopeV1,
) -> Result<(), GuestOutputFrameError> {
    let payload = serde_json::to_vec(envelope)?;
    write_payload_frame(&mut writer, &payload)?;
    writer.flush()?;
    Ok(())
}

pub fn read_guest_envelope_frame<R: Read>(
    mut reader: R,
) -> Result<GuestEvidenceEnvelopeV1, GuestOutputFrameError> {
    read_json_frame(&mut reader)
}

pub fn decode_guest_envelope_frame(
    bytes: &[u8],
) -> Result<GuestEvidenceEnvelopeV1, GuestOutputFrameError> {
    read_json_frame(&mut std::io::Cursor::new(bytes))
}

fn read_json_frame<R: Read, T: DeserializeOwned>(
    reader: &mut R,
) -> Result<T, GuestOutputFrameError> {
    let payload = read_payload_frame(reader)?;
    Ok(serde_json::from_slice(&payload)?)
}

fn write_payload_frame<W: Write>(
    writer: &mut W,
    payload: &[u8],
) -> Result<(), GuestOutputFrameError> {
    let length = u64::try_from(payload.len()).map_err(|_| GuestOutputFrameError::FrameTooLarge)?;
    validate_length(length)?;
    let digest = Sha256::digest(payload);
    writer.write_all(MAGIC)?;
    writer.write_all(&length.to_be_bytes())?;
    writer.write_all(payload)?;
    writer.write_all(&digest)?;
    Ok(())
}

fn read_payload_frame<R: Read>(reader: &mut R) -> Result<Vec<u8>, GuestOutputFrameError> {
    let mut magic = [0_u8; MAGIC.len()];
    reader.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err(GuestOutputFrameError::InvalidMagic);
    }

    let mut length_bytes = [0_u8; 8];
    reader.read_exact(&mut length_bytes)?;
    let length = u64::from_be_bytes(length_bytes);
    validate_length(length)?;
    let length = usize::try_from(length).map_err(|_| GuestOutputFrameError::FrameTooLarge)?;

    let mut payload = vec![0_u8; length];
    reader.read_exact(&mut payload)?;
    let mut expected_digest = [0_u8; DIGEST_LEN];
    reader.read_exact(&mut expected_digest)?;
    let observed = Sha256::digest(&payload);
    if observed[..] != expected_digest {
        return Err(GuestOutputFrameError::ChecksumMismatch);
    }

    let mut trailing = [0_u8; 1];
    if reader.read(&mut trailing)? != 0 {
        return Err(GuestOutputFrameError::TrailingBytes);
    }
    Ok(payload)
}

fn validate_length(length: u64) -> Result<(), GuestOutputFrameError> {
    if length == 0 {
        return Err(GuestOutputFrameError::EmptyFrame);
    }
    if length > MAX_GUEST_ENVELOPE_BYTES {
        return Err(GuestOutputFrameError::FrameTooLarge);
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum GuestOutputFrameError {
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("guest output frame has invalid magic/version")]
    InvalidMagic,
    #[error("guest output frame may not be empty")]
    EmptyFrame,
    #[error("guest output frame exceeds the v1 maximum")]
    FrameTooLarge,
    #[error("guest output frame checksum mismatch")]
    ChecksumMismatch,
    #[error("guest output contains trailing bytes after the single frame")]
    TrailingBytes,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_value(value: &serde_json::Value) -> Vec<u8> {
        let payload = serde_json::to_vec(value).unwrap();
        let mut bytes = Vec::new();
        write_payload_frame(&mut bytes, &payload).unwrap();
        bytes
    }

    #[test]
    fn framed_json_round_trips() {
        let expected = serde_json::json!({"phase":"verified","count":8});
        let bytes = frame_value(&expected);
        let observed: serde_json::Value = read_json_frame(&mut std::io::Cursor::new(bytes)).unwrap();
        assert_eq!(observed, expected);
    }

    #[test]
    fn checksum_corruption_fails_closed() {
        let mut bytes = frame_value(&serde_json::json!({"ok":true}));
        let payload_start = MAGIC.len() + 8;
        bytes[payload_start] ^= 1;
        assert!(matches!(
            read_payload_frame(&mut std::io::Cursor::new(bytes)),
            Err(GuestOutputFrameError::ChecksumMismatch)
        ));
    }

    #[test]
    fn trailing_bytes_fail_closed() {
        let mut bytes = frame_value(&serde_json::json!({"ok":true}));
        bytes.push(0);
        assert!(matches!(
            read_payload_frame(&mut std::io::Cursor::new(bytes)),
            Err(GuestOutputFrameError::TrailingBytes)
        ));
    }

    #[test]
    fn oversized_length_fails_before_payload_allocation() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&(MAX_GUEST_ENVELOPE_BYTES + 1).to_be_bytes());
        assert!(matches!(
            read_payload_frame(&mut std::io::Cursor::new(bytes)),
            Err(GuestOutputFrameError::FrameTooLarge)
        ));
    }
}
