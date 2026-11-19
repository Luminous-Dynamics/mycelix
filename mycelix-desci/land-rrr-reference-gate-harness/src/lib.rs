//! Isolated test harness for the actual Mycelix-DeSci land RRR source module.
//! The path module compiles the production source file directly without
//! compiling unrelated DeSci core modules with pre-existing integration debt.

#[path = "../../src/core/src/land_rrr_reference.rs"]
pub mod land_rrr_reference;

#[path = "../../src/core/src/land_operation_lifecycle.rs"]
pub mod land_operation_lifecycle;
