//! Conservative protocol-kernel resource bounds.
//!
//! These values constrain verifier resource use. They are not constitutional
//! semantics and MUST NOT be interpreted as authority or policy.

/// Static ASSURE-002A resource-bound profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct KernelBounds {
    /// Maximum encoded capsule size accepted by the initial profile.
    pub max_input_bytes: u32,
    /// Maximum number of normative nodes.
    pub max_nodes: u32,
    /// Maximum number of normative edges.
    pub max_edges: u32,
    /// Maximum text field size.
    pub max_text_bytes: u32,
    /// Maximum provenance-chain depth.
    pub max_provenance_depth: u16,
    /// Maximum authority-delegation depth.
    pub max_delegation_depth: u16,
}

impl KernelBounds {
    /// Initial conservative profile. Later changes are resource-policy changes,
    /// not changes to constitutional meaning.
    pub const V1_INITIAL: Self = Self {
        max_input_bytes: 16 * 1024 * 1024,
        max_nodes: 100_000,
        max_edges: 500_000,
        max_text_bytes: 64 * 1024,
        max_provenance_depth: 256,
        max_delegation_depth: 64,
    };
}
