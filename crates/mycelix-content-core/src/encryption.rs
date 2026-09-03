use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EncryptionRequirementV1 {
    NotRequired,
    ProviderAtRest,
    ClientSide,
    ClientSideAndProviderAtRest,
}

impl EncryptionRequirementV1 {
    pub fn requires_client_side(self) -> bool {
        matches!(self, Self::ClientSide | Self::ClientSideAndProviderAtRest)
    }

    pub(crate) fn tag(self) -> u8 {
        match self {
            Self::NotRequired => 0,
            Self::ProviderAtRest => 1,
            Self::ClientSide => 2,
            Self::ClientSideAndProviderAtRest => 3,
        }
    }
}
