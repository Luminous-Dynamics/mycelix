use crate::{IntegralSystem, MaturityState};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavigationSurface {
    Overview,
    System(IntegralSystem),
    Sources,
    NodeStatus,
    Conformance,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavigationEntry {
    pub label: &'static str,
    pub surface: NavigationSurface,
    pub maturity: MaturityState,
}

pub const NAVIGATION_SURFACES: [NavigationEntry; 9] = [
    NavigationEntry {
        label: "Overview",
        surface: NavigationSurface::Overview,
        maturity: MaturityState::SourceImplemented,
    },
    NavigationEntry {
        label: "CDS",
        surface: NavigationSurface::System(IntegralSystem::Cds),
        maturity: MaturityState::Designed,
    },
    NavigationEntry {
        label: "OAD",
        surface: NavigationSurface::System(IntegralSystem::Oad),
        maturity: MaturityState::Designed,
    },
    NavigationEntry {
        label: "ITC",
        surface: NavigationSurface::System(IntegralSystem::Itc),
        maturity: MaturityState::Designed,
    },
    NavigationEntry {
        label: "COS",
        surface: NavigationSurface::System(IntegralSystem::Cos),
        maturity: MaturityState::Designed,
    },
    NavigationEntry {
        label: "FRS",
        surface: NavigationSurface::System(IntegralSystem::Frs),
        maturity: MaturityState::Designed,
    },
    NavigationEntry {
        label: "Sources",
        surface: NavigationSurface::Sources,
        maturity: MaturityState::SourceImplemented,
    },
    NavigationEntry {
        label: "Node status",
        surface: NavigationSurface::NodeStatus,
        maturity: MaturityState::Designed,
    },
    NavigationEntry {
        label: "Conformance",
        surface: NavigationSurface::Conformance,
        maturity: MaturityState::Designed,
    },
];
