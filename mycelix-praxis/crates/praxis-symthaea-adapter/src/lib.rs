// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Host-only, size-bounded decoding primitives for the optional Symthaea advisor.
//!
//! This crate intentionally has no Symthaea runtime or network-client dependency.
//! A host transport supplies a reader with its own authentication and timeout
//! policy. This module caps bytes read before parsing and delegates strict JSON
//! decoding to `praxis-core`; callers must still run semantic validation with
//! trusted request-local context.

#![forbid(unsafe_code)]

#[cfg(target_family = "wasm")]
compile_error!("praxis-symthaea-adapter is host-only and must not be compiled into WASM guest zomes");

use std::io::{self, Read};

use praxis_core::{
    AdviceWireRejection, MAX_ADVISORY_RESPONSE_BYTES, SymthaeaAdvice,
    parse_symthaea_advice_response,
};

/// Failure while reading and decoding a host-provided advisor response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BoundedAdviceReadError {
    /// The underlying reader failed. The original payload is never accepted.
    ReaderFailed(io::ErrorKind),
    /// More than the configured maximum arrived; only max + 1 bytes were read.
    ResponseTooLarge,
    /// The bounded bytes were not a valid strict `SymthaeaAdvice` JSON object.
    MalformedJson,
}

/// Read and parse one advisory response with a hard pre-buffering byte ceiling.
///
/// The reader is wrapped in `Read::take(MAX_ADVISORY_RESPONSE_BYTES + 1)`, so
/// the function buffers at most one byte beyond the permitted body size. That
/// extra byte is enough to distinguish an exactly-at-limit body from an
/// oversized one. It never reads the remainder of an oversized stream.
///
/// The caller is responsible for configuring transport/process authentication,
/// read timeouts, learner consent, and request freshness. The returned object
/// remains untrusted: call `praxis_core::validate_symthaea_advice` with trusted
/// per-turn context before displaying any suggestion.
pub fn read_bounded_symthaea_advice<R: Read>(
    reader: R,
) -> Result<SymthaeaAdvice, BoundedAdviceReadError> {
    let mut bounded = reader.take((MAX_ADVISORY_RESPONSE_BYTES + 1) as u64);
    let mut bytes = Vec::with_capacity(MAX_ADVISORY_RESPONSE_BYTES + 1);
    bounded
        .read_to_end(&mut bytes)
        .map_err(|error| BoundedAdviceReadError::ReaderFailed(error.kind()))?;

    if bytes.len() > MAX_ADVISORY_RESPONSE_BYTES {
        return Err(BoundedAdviceReadError::ResponseTooLarge);
    }

    parse_symthaea_advice_response(&bytes).map_err(|error| match error {
        AdviceWireRejection::ResponseTooLarge => BoundedAdviceReadError::ResponseTooLarge,
        AdviceWireRejection::MalformedJson => BoundedAdviceReadError::MalformedJson,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };

    fn valid_advice_json() -> String {
        format!(
            r#"{{"schema_version":1,"model_id":"symthaea","model_version":"0.5.0","skill_id":"math.fractions","curriculum_revision":"2026.1","input_digest":"blake3:{}","action":"offer_hint","reported_confidence_permille":500,"reason":"repeated_incorrect_attempts","supporting_content_ids":["lesson.fractions.01"]}}"#,
            "0".repeat(64)
        )
    }

    struct CountingReader {
        remaining: usize,
        observed_bytes: Arc<AtomicUsize>,
    }

    impl Read for CountingReader {
        fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
            let count = buffer.len().min(self.remaining);
            buffer[..count].fill(b' ');
            self.remaining -= count;
            self.observed_bytes.fetch_add(count, Ordering::SeqCst);
            Ok(count)
        }
    }

    struct FailingReader;

    impl Read for FailingReader {
        fn read(&mut self, _buffer: &mut [u8]) -> io::Result<usize> {
            Err(io::Error::new(io::ErrorKind::BrokenPipe, "test reader failure"))
        }
    }

    #[test]
    fn reads_a_valid_response_from_a_host_reader() {
        let encoded = valid_advice_json();
        let advice = read_bounded_symthaea_advice(Cursor::new(encoded.as_bytes()))
            .expect("valid bounded response parses");
        assert_eq!(advice.model_id, "symthaea");
        assert_eq!(advice.action, praxis_core::LearningSuggestion::OfferHint);
    }

    #[test]
    fn rejects_oversized_stream_after_reading_only_limit_plus_one() {
        let observed_bytes = Arc::new(AtomicUsize::new(0));
        let reader = CountingReader {
            remaining: MAX_ADVISORY_RESPONSE_BYTES * 4,
            observed_bytes: Arc::clone(&observed_bytes),
        };

        assert_eq!(
            read_bounded_symthaea_advice(reader),
            Err(BoundedAdviceReadError::ResponseTooLarge)
        );
        assert_eq!(
            observed_bytes.load(Ordering::SeqCst),
            MAX_ADVISORY_RESPONSE_BYTES + 1,
            "oversized streams must not be fully buffered"
        );
    }

    #[test]
    fn exact_limit_body_reaches_parser_without_crossing_the_size_gate() {
        let reader = Cursor::new(vec![b' '; MAX_ADVISORY_RESPONSE_BYTES]);
        assert_eq!(
            read_bounded_symthaea_advice(reader),
            Err(BoundedAdviceReadError::MalformedJson)
        );
    }

    #[test]
    fn malformed_json_is_rejected() {
        assert_eq!(
            read_bounded_symthaea_advice(Cursor::new(&b"{"[..])),
            Err(BoundedAdviceReadError::MalformedJson)
        );
    }

    #[test]
    fn reader_errors_fail_closed() {
        assert_eq!(
            read_bounded_symthaea_advice(FailingReader),
            Err(BoundedAdviceReadError::ReaderFailed(
                io::ErrorKind::BrokenPipe
            ))
        );
    }
}
