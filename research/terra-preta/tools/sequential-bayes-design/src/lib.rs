//! Reusable inference components for the Terra Preta research programme.
//!
//! This library currently provides a tested mathematical primitive for a
//! finite model posterior over a jointly observed correlated Gaussian pair.
//! It is not yet wired into the categorical acquisition executable.

pub mod correlated_likelihood;
pub mod multivariate_gaussian;
pub mod observation_history;
pub mod information_gain;

#[cfg(test)]
mod posterior_sbc;
pub mod robust_information;
pub mod covariance_identifiability;
pub mod stochastic_policy;
pub mod transient_exchange;
