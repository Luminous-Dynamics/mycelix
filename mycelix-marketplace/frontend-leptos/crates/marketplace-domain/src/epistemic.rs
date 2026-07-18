use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EpistemicClassification {
    pub empirical: EmpiricalLevel,
    pub normative: NormativeLevel,
    pub materiality: MaterialityLevel,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EmpiricalLevel {
    E0Null,
    E1Testimonial,
    E2PrivateVerify,
    E3Cryptographic,
    E4PublicRepro,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormativeLevel {
    N0Personal,
    N1Communal,
    N2Network,
    N3Axiomatic,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum MaterialityLevel {
    M0Ephemeral,
    M1Temporal,
    M2Persistent,
    M3Foundational,
}
