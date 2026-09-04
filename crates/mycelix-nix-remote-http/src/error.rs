use thiserror::Error;

const MAX_VERIFICATION_CONCURRENCY_V1: usize = 64;

#[derive(Debug, Error)]
pub enum RemoteNixRouterErrorV1 {
    #[error("remote Nix verification concurrency must be non-zero")]
    ZeroVerificationConcurrency,
    #[error("remote Nix verification concurrency {requested} exceeds maximum {maximum}")]
    VerificationConcurrencyTooHigh { requested: usize, maximum: usize },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RemoteNixRouterConfigV1 {
    pub priority: u32,
    pub max_concurrent_verifications: usize,
}

impl Default for RemoteNixRouterConfigV1 {
    fn default() -> Self {
        Self {
            priority: 40,
            max_concurrent_verifications: 4,
        }
    }
}

impl RemoteNixRouterConfigV1 {
    pub fn validate(self) -> Result<Self, RemoteNixRouterErrorV1> {
        if self.max_concurrent_verifications == 0 {
            return Err(RemoteNixRouterErrorV1::ZeroVerificationConcurrency);
        }
        if self.max_concurrent_verifications > MAX_VERIFICATION_CONCURRENCY_V1 {
            return Err(RemoteNixRouterErrorV1::VerificationConcurrencyTooHigh {
                requested: self.max_concurrent_verifications,
                maximum: MAX_VERIFICATION_CONCURRENCY_V1,
            });
        }
        Ok(self)
    }
}
