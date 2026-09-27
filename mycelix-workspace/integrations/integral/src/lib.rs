//! Integral Reference Solution shell.
//!
//! This crate intentionally owns only adapter/source/status boundaries.
//! It does not implement Integral governance, accounting, certification,
//! authority, execution, or runtime persistence semantics.

pub mod adapters;
pub mod maturity;
pub mod navigation;
pub mod source_registry;

pub use adapters::{AdapterBoundary, IntegralSystem, ADAPTER_BOUNDARIES};
pub use maturity::MaturityState;
pub use navigation::{NavigationSurface, NAVIGATION_SURFACES};
pub use source_registry::{
    ContractKind, ExternalContractRef, ExternalStatus, SourceFamily, CURRENT_EXTERNAL_CONTRACTS,
};
