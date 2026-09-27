#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum IntegralSystem {
    Cds,
    Oad,
    Itc,
    Cos,
    Frs,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdapterBoundary {
    pub system: IntegralSystem,
    pub namespace: &'static str,
    pub composes_existing_mycelix_owners: bool,
    pub owns_integral_normative_semantics: bool,
}

pub const ADAPTER_BOUNDARIES: [AdapterBoundary; 5] = [
    AdapterBoundary {
        system: IntegralSystem::Cds,
        namespace: "integral.cds",
        composes_existing_mycelix_owners: true,
        owns_integral_normative_semantics: true,
    },
    AdapterBoundary {
        system: IntegralSystem::Oad,
        namespace: "integral.oad",
        composes_existing_mycelix_owners: true,
        owns_integral_normative_semantics: true,
    },
    AdapterBoundary {
        system: IntegralSystem::Itc,
        namespace: "integral.itc",
        composes_existing_mycelix_owners: true,
        owns_integral_normative_semantics: true,
    },
    AdapterBoundary {
        system: IntegralSystem::Cos,
        namespace: "integral.cos",
        composes_existing_mycelix_owners: true,
        owns_integral_normative_semantics: true,
    },
    AdapterBoundary {
        system: IntegralSystem::Frs,
        namespace: "integral.frs",
        composes_existing_mycelix_owners: true,
        owns_integral_normative_semantics: true,
    },
];
