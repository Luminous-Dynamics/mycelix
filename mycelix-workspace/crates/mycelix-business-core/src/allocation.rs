// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural conservation contracts for domain-owned allocations.

use core::fmt;

use crate::ids::{DomainAllocationRef, SemanticProfileId, SourceEventRef, SubjectRef, UnitId};

/// Explicit quantity in one semantic unit/currency.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Quantity {
    pub unit: UnitId,
    pub amount: u128,
}

impl Quantity {
    #[must_use]
    pub fn new(unit: UnitId, amount: u128) -> Self {
        Self { unit, amount }
    }
}

/// One accepted allocation line from a conserved source effect.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AllocationLine {
    pub target: SubjectRef,
    pub quantity: Quantity,
}

/// Structural allocation errors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AllocationError {
    UnitMismatch {
        expected: UnitId,
        actual: UnitId,
    },
    ZeroAllocation {
        target: SubjectRef,
    },
    ArithmeticOverflow,
    OverAllocation {
        available: u128,
        allocated: u128,
    },
}

impl fmt::Display for AllocationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnitMismatch { expected, actual } => {
                write!(f, "allocation unit mismatch: expected {expected}, got {actual}")
            }
            Self::ZeroAllocation { target } => {
                write!(f, "zero allocation is not a consequential allocation for {target}")
            }
            Self::ArithmeticOverflow => f.write_str("allocation arithmetic overflow"),
            Self::OverAllocation {
                available,
                allocated,
            } => write!(
                f,
                "allocation exceeds declared source capacity: available {available}, allocated {allocated}"
            ),
        }
    }
}

impl std::error::Error for AllocationError {}

/// Result of structural conservation validation.
///
/// The positive summary is sealed and retains the exact statement it validated,
/// so a result cannot float to a different or subsequently modified statement.
///
/// ```compile_fail
/// use mycelix_business_core::AllocationSummary;
///
/// fn cannot_forge(summary: &AllocationSummary) {
///     let _ = &summary.allocated;
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AllocationSummary {
    pub(crate) statement: AllocationStatement,
    pub(crate) allocated: Quantity,
    pub(crate) remaining: Quantity,
}

impl AllocationSummary {
    /// Exact candidate statement whose arithmetic conservation was validated.
    #[must_use]
    pub fn statement(&self) -> &AllocationStatement {
        &self.statement
    }

    /// Total quantity structurally allocated by the validated statement.
    #[must_use]
    pub fn allocated(&self) -> &Quantity {
        &self.allocated
    }

    /// Unallocated remainder under the statement's declared source capacity.
    #[must_use]
    pub fn remaining(&self) -> &Quantity {
        &self.remaining
    }
}

/// Domain-owned allocation statement for one conserved source effect.
///
/// The domain namespace participates in `allocation_ref` identity so equal
/// local allocation IDs from different domains cannot collapse. The crate
/// verifies only structural conservation against the declared source capacity.
/// It does not decide whether the named domain is authoritative, whether the
/// source event is authentic, whether the declared capacity is true, or whether
/// a target is legitimately entitled to the allocation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AllocationStatement {
    pub allocation_ref: DomainAllocationRef,
    pub source_event: SourceEventRef,
    pub semantic_profile: SemanticProfileId,
    pub total_available: Quantity,
    pub lines: Vec<AllocationLine>,
}

impl AllocationStatement {
    /// Validate unit consistency and arithmetic conservation.
    pub fn validate_conservation(&self) -> Result<AllocationSummary, AllocationError> {
        let mut allocated = 0_u128;

        for line in &self.lines {
            if line.quantity.unit != self.total_available.unit {
                return Err(AllocationError::UnitMismatch {
                    expected: self.total_available.unit.clone(),
                    actual: line.quantity.unit.clone(),
                });
            }

            if line.quantity.amount == 0 {
                return Err(AllocationError::ZeroAllocation {
                    target: line.target.clone(),
                });
            }

            allocated = allocated
                .checked_add(line.quantity.amount)
                .ok_or(AllocationError::ArithmeticOverflow)?;
        }

        if allocated > self.total_available.amount {
            return Err(AllocationError::OverAllocation {
                available: self.total_available.amount,
                allocated,
            });
        }

        let remaining = self.total_available.amount - allocated;

        Ok(AllocationSummary {
            statement: self.clone(),
            allocated: Quantity::new(self.total_available.unit.clone(), allocated),
            remaining: Quantity::new(self.total_available.unit.clone(), remaining),
        })
    }
}
