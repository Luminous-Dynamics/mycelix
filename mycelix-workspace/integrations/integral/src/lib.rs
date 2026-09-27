//! Integral Reference Solution shell.
//!
//! This crate intentionally owns only adapter/source/status and provisional
//! external compatibility boundaries. It does not implement Integral governance,
//! accounting, certification, authority, execution, or runtime persistence semantics.

pub mod adapters;
pub mod external_profiles;
pub mod maturity;
pub mod navigation;
pub mod source_registry;

pub use adapters::{AdapterBoundary, IntegralSystem, ADAPTER_BOUNDARIES};
pub use external_profiles::{
    AuthoritySemantics, CertifiedDesignDraft20260927, DecisionPacketDraft20260927,
    DiagnosticFindingDevGuideV01, ExternalFieldOrigin, ExternalSchemaBinding,
    FieldOriginKind, FrsSignalPacketDraft20260927, ItcAccountDevGuideV01,
    ItcLedgerEntryDraft20260927, LaborEventDraft20260927,
    MaterialConsumptionEventDraft20260927, ProfiledExternalObject,
    RecommendationDevGuideV01, CERTIFIED_DESIGN_FIELD_ORIGINS,
    DEVELOPMENT_GUIDE_BLOB, LABOR_EVENT_FIELD_ORIGINS,
    TECHNICAL_SPECIFICATIONS_OBSERVATION,
};
pub use maturity::MaturityState;
pub use navigation::{NavigationSurface, NAVIGATION_SURFACES};
pub use source_registry::{
    ContractKind, ExternalContractRef, ExternalStatus, SourceFamily, CURRENT_EXTERNAL_CONTRACTS,
};
