use std::{fmt, time::SystemTime};

use thiserror::Error;

#[derive(Debug, Error, Clone, Copy, PartialEq, Eq)]
pub enum RemoteClockErrorV1 {
    #[error("system clock is before the Unix epoch")]
    BeforeUnixEpoch,
    #[error("system clock milliseconds exceed the v1 timestamp range")]
    TimestampOverflow,
}

/// Trusted server-side request clock.
///
/// Implementations MUST derive time from server-controlled state, never from an
/// untrusted request header or query parameter. CF-07B asks this clock on every
/// request so a stale `RemoteServingSnapshotV1` cannot keep serving indefinitely.
pub trait RemoteClockV1: fmt::Debug + Send + Sync + 'static {
    fn now_unix_ms(&self) -> Result<u64, RemoteClockErrorV1>;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SystemUnixClockV1;

impl RemoteClockV1 for SystemUnixClockV1 {
    fn now_unix_ms(&self) -> Result<u64, RemoteClockErrorV1> {
        let duration = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .map_err(|_| RemoteClockErrorV1::BeforeUnixEpoch)?;
        u64::try_from(duration.as_millis()).map_err(|_| RemoteClockErrorV1::TimestampOverflow)
    }
}
