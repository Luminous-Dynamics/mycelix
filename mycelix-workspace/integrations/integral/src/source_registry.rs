#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SourceFamily {
    TechnicalSpecifications,
    DevelopmentGuide,
    WhitePaper,
    DecisionRecord,
    ExplanatoryPage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExternalStatus {
    Draft,
    Pending,
    Ratified,
    Explanatory,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ContractKind {
    DataStructure,
    Interface,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ExternalContractRef {
    pub id: &'static str,
    pub title: &'static str,
    pub family: SourceFamily,
    pub status: ExternalStatus,
    pub kind: ContractKind,
}

/// Current public Integral Phase-0 contract identities observed 2026-09-27.
/// These are adapter-owned external identities, not canonical Mycelix types.
pub const CURRENT_EXTERNAL_CONTRACTS: [ExternalContractRef; 9] = [
    ExternalContractRef {
        id: "SPEC-DS-01",
        title: "Certified Design",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Draft,
        kind: ContractKind::DataStructure,
    },
    ExternalContractRef {
        id: "SPEC-DS-02",
        title: "Labor Event",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Draft,
        kind: ContractKind::DataStructure,
    },
    ExternalContractRef {
        id: "SPEC-DS-03",
        title: "Material Consumption Event",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Draft,
        kind: ContractKind::DataStructure,
    },
    ExternalContractRef {
        id: "SPEC-DS-04",
        title: "ITC Ledger Entry",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Draft,
        kind: ContractKind::DataStructure,
    },
    ExternalContractRef {
        id: "SPEC-DS-05",
        title: "FRS Signal Packet",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Draft,
        kind: ContractKind::DataStructure,
    },
    ExternalContractRef {
        id: "SPEC-DS-06",
        title: "Decision Packet",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Draft,
        kind: ContractKind::DataStructure,
    },
    ExternalContractRef {
        id: "SPEC-IF-01",
        title: "OAD -> COS",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Pending,
        kind: ContractKind::Interface,
    },
    ExternalContractRef {
        id: "SPEC-IF-02",
        title: "COS -> ITC",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Pending,
        kind: ContractKind::Interface,
    },
    ExternalContractRef {
        id: "SPEC-IF-03",
        title: "FRS -> CDS",
        family: SourceFamily::TechnicalSpecifications,
        status: ExternalStatus::Pending,
        kind: ContractKind::Interface,
    },
];
