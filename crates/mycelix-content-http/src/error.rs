use std::{io, net::SocketAddr};

use thiserror::Error;

#[derive(Debug, Error)]
pub enum HttpFacadeErrorV1 {
    #[error("content HTTP v0.1 refuses non-loopback bind address: {0}")]
    NonLoopbackBind(SocketAddr),
    #[error("verification concurrency must be at least one")]
    ZeroVerificationConcurrency,
    #[error("failed to bind content HTTP listener at {addr}: {source}")]
    Bind {
        addr: SocketAddr,
        #[source]
        source: io::Error,
    },
    #[error("content HTTP server failed: {0}")]
    Serve(#[source] io::Error),
}
