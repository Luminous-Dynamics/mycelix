#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MaturityState {
    Designed,
    SourceImplemented,
    LocallyTested,
    RepositoryQualified,
    PilotObserved,
}

impl MaturityState {
    /// Whether the state is strong enough to be shown as operational in the
    /// reference application. Designed/source-implemented/local-test states
    /// remain visibly pre-operational.
    pub const fn is_operational(self) -> bool {
        matches!(self, Self::RepositoryQualified | Self::PilotObserved)
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Designed => "Designed",
            Self::SourceImplemented => "SourceImplemented",
            Self::LocallyTested => "LocallyTested",
            Self::RepositoryQualified => "RepositoryQualified",
            Self::PilotObserved => "PilotObserved",
        }
    }
}
