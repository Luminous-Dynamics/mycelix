//! Reusable inference components for the Terra Preta research programme.
//!
//! This library currently provides a tested mathematical primitive for a
//! finite model posterior over a jointly observed correlated Gaussian pair.
//! It is not yet wired into the categorical acquisition executable.

pub mod correlated_likelihood;
pub mod multivariate_gaussian;
pub mod observation_history;
