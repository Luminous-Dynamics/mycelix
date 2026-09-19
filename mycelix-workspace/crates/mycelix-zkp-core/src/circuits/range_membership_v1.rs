// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//!
//! Candidate MYC-ZKP-RANGE-001A range-membership AIR.
//!
//! This module is deliberately **not** part of ordinary `backend-winterfell` or
//! the `full` convenience feature. It is available only through
//! `candidate-range-membership-v1` until exact-head adversarial qualification
//! executes and passes.
//!
//! ## Statement
//!
//! The candidate proves only the computational-integrity statement:
//!
//! ```text
//! there exists one signed-i64 fixed-point trace witness x
//! such that
//! min_raw <= x <= max_raw
//! ```
//!
//! It does **not** prove commitment opening, application/domain binding, FL
//! contribution validity, or application authority.
//!
//! ## Privacy boundary
//!
//! The raw witness is not a public input, but Winterfell 0.13.1 is not treated
//! by Mycelix as a witness-privacy / zero-knowledge profile. See issue #1899 and
//! `ZKP_RANGE_WITNESS_PRIVACY_BOUNDARY.md`.
//!
//! ## Numeric / field-safety profile
//!
//! Storage is the full signed-i64 raw fixed-point domain with 16 fractional bits.
//! A raw value is order-preservingly bias-encoded by `+2^63`, mapping:
//!
//! ```text
//! i64::MIN -> 0
//! i64::MAX -> 2^64 - 1
//! ```
//!
//! Both non-negative differences therefore fit in exactly 64 bits. The fixed
//! 128-row trace uses row 0 as an accumulator seed, rows 1..=63 as forced-zero
//! prefix padding, and rows 64..=127 for the 64 MSB-first difference bits.
//!
//! Winterfell's selected `f128::BaseElement` modulus is roughly 2^128. Every
//! theorem-bearing integer relation here stays below 2^66: encoded values and
//! reconstructed differences are <2^64, `min + low < 2^65`, and the largest
//! intermediate `x + high < 3*2^64 < 2^66`. Thus a field wrap cannot masquerade
//! as a valid signed range relation for this profile.

use thiserror::Error;
use winterfell::{
    AcceptableOptions, Air, AirContext, Assertion, AuxRandElements, BatchingMethod,
    CompositionPoly, CompositionPolyTrace, DefaultConstraintCommitment,
    DefaultConstraintEvaluator, DefaultTraceLde, EvaluationFrame, PartitionOptions, Proof,
    ProofOptions, Prover as WinterfellProver, StarkDomain, Trace, TraceInfo, TracePolyTable,
    TraceTable, TransitionConstraintDegree,
    crypto::{DefaultRandomCoin, MerkleTree, hashers::Blake3_256},
    math::{FieldElement, ToElements, fields::f128::BaseElement},
    matrix::ColMatrix,
};

/// Stable semantic tag for the exact candidate statement/profile.
///
/// Little-endian ASCII `MXRNGV1\0`. Changing theorem-bearing semantics requires
/// a new tag/profile rather than reinterpreting old proofs.
pub const RANGE_MEMBERSHIP_V1_STATEMENT_TAG: u64 = 0x0031_5647_4e52_584d;

/// Bias mapping the full signed-i64 raw domain to `[0, u64::MAX]`.
pub const RANGE_MEMBERSHIP_V1_BIAS: u64 = 1_u64 << 63;
/// Number of binary digits used for each non-negative difference.
pub const RANGE_MEMBERSHIP_V1_DIFFERENCE_BITS: usize = 64;
/// Exact trace length for this profile.
pub const RANGE_MEMBERSHIP_V1_TRACE_LENGTH: usize = 128;
/// First row that carries a real difference bit (bit 63).
const FIRST_DATA_ROW: usize = 64;
const TRACE_WIDTH: usize = 8;
const LAST_STEP: usize = RANGE_MEMBERSHIP_V1_TRACE_LENGTH - 1;

mod col {
    pub const LOW_BIT: usize = 0;
    pub const LOW_ACC: usize = 1;
    pub const HIGH_BIT: usize = 2;
    pub const HIGH_ACC: usize = 3;
    pub const X_ENCODED: usize = 4;
    pub const MIN_CHECK: usize = 5;
    pub const MAX_CHECK: usize = 6;
    pub const STATEMENT_TAG: usize = 7;
}

type Hasher = Blake3_256<BaseElement>;
type VC = MerkleTree<Hasher>;
type RandCoin = DefaultRandomCoin<Hasher>;

/// Typed failures for the candidate range theorem.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RangeMembershipV1Error {
    #[error("invalid range bounds: min_raw {min_raw} exceeds max_raw {max_raw}")]
    InvalidBounds { min_raw: i64, max_raw: i64 },

    #[error("witness raw value {value_raw} is outside [{min_raw}, {max_raw}]")]
    WitnessOutsideBounds {
        value_raw: i64,
        min_raw: i64,
        max_raw: i64,
    },

    #[error(
        "invalid proof trace profile: expected width {expected_width} length {expected_length}, got width {actual_width} length {actual_length}"
    )]
    InvalidProofTraceProfile {
        expected_width: usize,
        expected_length: usize,
        actual_width: usize,
        actual_length: usize,
    },

    #[error("Winterfell proving failed: {0}")]
    ProofGenerationFailed(String),

    #[error("Winterfell verification failed: {0}")]
    ProofVerificationFailed(String),
}

/// Exact public statement consumed by the candidate AIR.
///
/// The witness is intentionally absent from these elements. That is a statement
/// about the public-input surface only and is **not** a zero-knowledge claim.
#[derive(Clone, Debug)]
pub struct RangeMembershipPublicInputsV1 {
    statement_tag: u64,
    min_encoded: u64,
    max_encoded: u64,
}

impl RangeMembershipPublicInputsV1 {
    pub fn new(min_raw: i64, max_raw: i64) -> Result<Self, RangeMembershipV1Error> {
        if min_raw > max_raw {
            return Err(RangeMembershipV1Error::InvalidBounds { min_raw, max_raw });
        }

        Ok(Self {
            statement_tag: RANGE_MEMBERSHIP_V1_STATEMENT_TAG,
            min_encoded: encode_raw(min_raw),
            max_encoded: encode_raw(max_raw),
        })
    }

    pub const fn statement_tag(&self) -> u64 {
        self.statement_tag
    }

    pub const fn min_encoded(&self) -> u64 {
        self.min_encoded
    }

    pub const fn max_encoded(&self) -> u64 {
        self.max_encoded
    }
}

impl ToElements<BaseElement> for RangeMembershipPublicInputsV1 {
    fn to_elements(&self) -> Vec<BaseElement> {
        vec![
            BaseElement::from(self.statement_tag),
            BaseElement::from(self.min_encoded),
            BaseElement::from(self.max_encoded),
        ]
    }
}

/// Order-preserving full-domain signed-i64 encoding.
fn encode_raw(raw: i64) -> u64 {
    let encoded = i128::from(raw) + (1_i128 << 63);
    // Every i64 maps exactly into [0, 2^64 - 1].
    u64::try_from(encoded).expect("full i64 bias encoding is always representable")
}

/// Candidate AIR for signed range membership.
pub struct RangeMembershipAirV1 {
    context: AirContext<BaseElement>,
    statement_tag: BaseElement,
    min_encoded: BaseElement,
    max_encoded: BaseElement,
}

impl Air for RangeMembershipAirV1 {
    type BaseField = BaseElement;
    type PublicInputs = RangeMembershipPublicInputsV1;

    fn new(
        trace_info: TraceInfo,
        pub_inputs: Self::PublicInputs,
        options: ProofOptions,
    ) -> Self {
        assert_eq!(TRACE_WIDTH, trace_info.width(), "unexpected range-v1 trace width");
        assert_eq!(
            RANGE_MEMBERSHIP_V1_TRACE_LENGTH,
            trace_info.length(),
            "unexpected range-v1 trace length"
        );

        let degrees = vec![
            TransitionConstraintDegree::new(2), // next low bit is binary
            TransitionConstraintDegree::new(2), // next high bit is binary
            TransitionConstraintDegree::new(1), // low accumulator recurrence
            TransitionConstraintDegree::new(1), // high accumulator recurrence
            TransitionConstraintDegree::new(1), // one shared x across trace
            TransitionConstraintDegree::new(1), // min = x - low
            TransitionConstraintDegree::new(1), // max = x + high
            TransitionConstraintDegree::new(1), // statement/profile identity constant
        ];

        // 4 seed assertions + 2*63 prefix-padding assertions + profile identity
        // + exact final public min/max = 133 assertions.
        let num_assertions = 133;
        let context = AirContext::new(trace_info, degrees, num_assertions, options);

        Self {
            context,
            statement_tag: BaseElement::from(pub_inputs.statement_tag),
            min_encoded: BaseElement::from(pub_inputs.min_encoded),
            max_encoded: BaseElement::from(pub_inputs.max_encoded),
        }
    }

    fn context(&self) -> &AirContext<Self::BaseField> {
        &self.context
    }

    fn evaluate_transition<E: FieldElement + From<Self::BaseField>>(
        &self,
        frame: &EvaluationFrame<E>,
        _periodic_values: &[E],
        result: &mut [E],
    ) {
        let current = frame.current();
        let next = frame.next();
        let one = E::ONE;
        let two = E::from(2u32);

        // Every processed bit, including the final row, is verifier-constrained.
        result[0] = next[col::LOW_BIT] * (next[col::LOW_BIT] - one);
        result[1] = next[col::HIGH_BIT] * (next[col::HIGH_BIT] - one);

        // MSB-first reconstruction over a fixed-width zero-prefixed trace.
        result[2] = next[col::LOW_ACC] - current[col::LOW_ACC] * two - next[col::LOW_BIT];
        result[3] = next[col::HIGH_ACC] - current[col::HIGH_ACC] * two - next[col::HIGH_BIT];

        // Both decompositions must refer to one and the same trace witness.
        result[4] = next[col::X_ENCODED] - current[col::X_ENCODED];

        // Derived check columns tie the final reconstructed differences to the
        // exact absolute public bounds. These are not merely width constraints.
        result[5] = next[col::MIN_CHECK] - (next[col::X_ENCODED] - next[col::LOW_ACC]);
        result[6] = next[col::MAX_CHECK] - (next[col::X_ENCODED] + next[col::HIGH_ACC]);

        // Statement/profile identity is part of the committed execution trace.
        result[7] = next[col::STATEMENT_TAG] - current[col::STATEMENT_TAG];
    }

    fn get_assertions(&self) -> Vec<Assertion<Self::BaseField>> {
        let mut assertions = Vec::with_capacity(133);

        // Exact zero seeds.
        assertions.push(Assertion::single(col::LOW_ACC, 0, BaseElement::ZERO));
        assertions.push(Assertion::single(col::HIGH_ACC, 0, BaseElement::ZERO));
        assertions.push(Assertion::single(col::LOW_BIT, 0, BaseElement::ZERO));
        assertions.push(Assertion::single(col::HIGH_BIT, 0, BaseElement::ZERO));

        // A 128-row trace has 127 transition-consumed next rows. Force the first
        // 63 of those rows to zero, leaving exactly rows 64..=127 for 64 data bits.
        for row in 1..FIRST_DATA_ROW {
            assertions.push(Assertion::single(col::LOW_BIT, row, BaseElement::ZERO));
            assertions.push(Assertion::single(col::HIGH_BIT, row, BaseElement::ZERO));
        }

        // Bind exact theorem/profile identity and the absolute public bounds.
        assertions.push(Assertion::single(col::STATEMENT_TAG, 0, self.statement_tag));
        assertions.push(Assertion::single(col::MIN_CHECK, LAST_STEP, self.min_encoded));
        assertions.push(Assertion::single(col::MAX_CHECK, LAST_STEP, self.max_encoded));

        debug_assert_eq!(assertions.len(), 133);
        assertions
    }
}

fn build_trace(
    value_raw: i64,
    min_raw: i64,
    max_raw: i64,
) -> Result<TraceTable<BaseElement>, RangeMembershipV1Error> {
    let pub_inputs = RangeMembershipPublicInputsV1::new(min_raw, max_raw)?;

    if value_raw < min_raw || value_raw > max_raw {
        return Err(RangeMembershipV1Error::WitnessOutsideBounds {
            value_raw,
            min_raw,
            max_raw,
        });
    }

    let x_encoded = encode_raw(value_raw);
    let low_diff = x_encoded - pub_inputs.min_encoded;
    let high_diff = pub_inputs.max_encoded - x_encoded;

    let mut columns = vec![
        vec![BaseElement::ZERO; RANGE_MEMBERSHIP_V1_TRACE_LENGTH];
        TRACE_WIDTH
    ];

    let mut low_acc = 0u64;
    let mut high_acc = 0u64;

    for row in 0..RANGE_MEMBERSHIP_V1_TRACE_LENGTH {
        let (low_bit, high_bit) = if row < FIRST_DATA_ROW {
            (0u64, 0u64)
        } else {
            // row 64 consumes bit 63; row 127 consumes bit 0.
            let bit_index = LAST_STEP - row;
            (
                (low_diff >> bit_index) & 1,
                (high_diff >> bit_index) & 1,
            )
        };

        if row > 0 {
            low_acc = (low_acc << 1) | low_bit;
            high_acc = (high_acc << 1) | high_bit;
        }

        // For every honest prefix, low_acc <= low_diff <= x_encoded and
        // x_encoded + high_acc <= max_encoded, so host arithmetic remains inside
        // u64. The AIR independently enforces the corresponding field relations.
        let min_check = x_encoded - low_acc;
        let max_check = x_encoded + high_acc;

        columns[col::LOW_BIT][row] = BaseElement::from(low_bit);
        columns[col::LOW_ACC][row] = BaseElement::from(low_acc);
        columns[col::HIGH_BIT][row] = BaseElement::from(high_bit);
        columns[col::HIGH_ACC][row] = BaseElement::from(high_acc);
        columns[col::X_ENCODED][row] = BaseElement::from(x_encoded);
        columns[col::MIN_CHECK][row] = BaseElement::from(min_check);
        columns[col::MAX_CHECK][row] = BaseElement::from(max_check);
        columns[col::STATEMENT_TAG][row] = BaseElement::from(RANGE_MEMBERSHIP_V1_STATEMENT_TAG);
    }

    debug_assert_eq!(low_acc, low_diff);
    debug_assert_eq!(high_acc, high_diff);
    debug_assert_eq!(x_encoded - low_acc, pub_inputs.min_encoded);
    debug_assert_eq!(x_encoded + high_acc, pub_inputs.max_encoded);

    Ok(TraceTable::init(columns))
}

struct RangeMembershipProverV1 {
    options: ProofOptions,
    pub_inputs: RangeMembershipPublicInputsV1,
}

impl WinterfellProver for RangeMembershipProverV1 {
    type BaseField = BaseElement;
    type Air = RangeMembershipAirV1;
    type Trace = TraceTable<BaseElement>;
    type HashFn = Hasher;
    type VC = VC;
    type RandomCoin = RandCoin;
    type TraceLde<E: FieldElement<BaseField = BaseElement>> = DefaultTraceLde<E, Hasher, VC>;
    type ConstraintCommitment<E: FieldElement<BaseField = BaseElement>> =
        DefaultConstraintCommitment<E, Hasher, VC>;
    type ConstraintEvaluator<'a, E: FieldElement<BaseField = BaseElement>> =
        DefaultConstraintEvaluator<'a, RangeMembershipAirV1, E>;

    fn options(&self) -> &ProofOptions {
        &self.options
    }

    fn get_pub_inputs(&self, _trace: &Self::Trace) -> RangeMembershipPublicInputsV1 {
        self.pub_inputs.clone()
    }

    fn new_trace_lde<E: FieldElement<BaseField = BaseElement>>(
        &self,
        trace_info: &TraceInfo,
        main_trace: &ColMatrix<BaseElement>,
        domain: &StarkDomain<BaseElement>,
        partition_options: PartitionOptions,
    ) -> (Self::TraceLde<E>, TracePolyTable<E>) {
        DefaultTraceLde::new(trace_info, main_trace, domain, partition_options)
    }

    fn build_constraint_commitment<E: FieldElement<BaseField = BaseElement>>(
        &self,
        composition_poly_trace: CompositionPolyTrace<E>,
        num_constraint_composition_columns: usize,
        domain: &StarkDomain<BaseElement>,
        partition_options: PartitionOptions,
    ) -> (Self::ConstraintCommitment<E>, CompositionPoly<E>) {
        DefaultConstraintCommitment::new(
            composition_poly_trace,
            num_constraint_composition_columns,
            domain,
            partition_options,
        )
    }

    fn new_evaluator<'a, E: FieldElement<BaseField = BaseElement>>(
        &self,
        air: &'a Self::Air,
        aux_rand_elements: Option<AuxRandElements<E>>,
        composition_coefficients: winterfell::ConstraintCompositionCoefficients<E>,
    ) -> Self::ConstraintEvaluator<'a, E> {
        DefaultConstraintEvaluator::new(air, aux_rand_elements, composition_coefficients)
    }
}

/// Exact proof options for this unqualified candidate profile.
///
/// Keeping this as a dedicated function lets the verifier admit only this exact
/// profile. These parameters are **not** yet claimed as a production security
/// profile until MYC-ZKP-RANGE-001AQ evaluates them explicitly.
pub fn candidate_range_membership_v1_options() -> ProofOptions {
    ProofOptions::new(
        32,
        8,
        0,
        winterfell::FieldExtension::None,
        8,
        31,
        BatchingMethod::Linear,
        BatchingMethod::Linear,
    )
}

fn validate_candidate_trace_info(trace_info: &TraceInfo) -> Result<(), RangeMembershipV1Error> {
    let actual_width = trace_info.width();
    let actual_length = trace_info.length();
    if actual_width != TRACE_WIDTH || actual_length != RANGE_MEMBERSHIP_V1_TRACE_LENGTH {
        return Err(RangeMembershipV1Error::InvalidProofTraceProfile {
            expected_width: TRACE_WIDTH,
            expected_length: RANGE_MEMBERSHIP_V1_TRACE_LENGTH,
            actual_width,
            actual_length,
        });
    }
    Ok(())
}

/// Generate a candidate STARK proving signed range membership.
///
/// This performs honest-prover admission checks for diagnostics, but authority
/// comes only from the AIR constraints and later qualification evidence.
pub fn prove_range_membership_v1(
    value_raw: i64,
    min_raw: i64,
    max_raw: i64,
) -> Result<Proof, RangeMembershipV1Error> {
    let pub_inputs = RangeMembershipPublicInputsV1::new(min_raw, max_raw)?;
    let trace = build_trace(value_raw, min_raw, max_raw)?;
    let prover = RangeMembershipProverV1 {
        options: candidate_range_membership_v1_options(),
        pub_inputs,
    };

    prover
        .prove(trace)
        .map_err(|e| RangeMembershipV1Error::ProofGenerationFailed(format!("{e:?}")))
}

/// Verify a candidate signed range-membership proof against exact public bounds.
///
/// The verifier accepts only the exact candidate proof-options profile and
/// rejects incompatible trace dimensions before Winterfell constructs the AIR.
pub fn verify_range_membership_v1(
    proof: Proof,
    min_raw: i64,
    max_raw: i64,
) -> Result<(), RangeMembershipV1Error> {
    validate_candidate_trace_info(proof.trace_info())?;
    let pub_inputs = RangeMembershipPublicInputsV1::new(min_raw, max_raw)?;
    let acceptable = AcceptableOptions::OptionSet(vec![candidate_range_membership_v1_options()]);

    winterfell::verify::<RangeMembershipAirV1, Hasher, RandCoin, VC>(
        proof,
        pub_inputs,
        &acceptable,
    )
    .map_err(|e| RangeMembershipV1Error::ProofVerificationFailed(format!("{e:?}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::panic::{AssertUnwindSafe, catch_unwind};

    fn assert_malformed_trace_rejected(
        trace: TraceTable<BaseElement>,
        pub_inputs: RangeMembershipPublicInputsV1,
    ) {
        let prover = RangeMembershipProverV1 {
            options: candidate_range_membership_v1_options(),
            pub_inputs: pub_inputs.clone(),
        };

        // Winterfell debug builds may reject an invalid trace during proving;
        // release-like paths may produce a proof that the verifier rejects.
        // Either outcome is acceptable; successful verification is not.
        let proving = catch_unwind(AssertUnwindSafe(|| prover.prove(trace)));
        match proving {
            Err(_) => {}
            Ok(Err(_)) => {}
            Ok(Ok(proof)) => {
                let acceptable =
                    AcceptableOptions::OptionSet(vec![candidate_range_membership_v1_options()]);
                assert!(
                    winterfell::verify::<RangeMembershipAirV1, Hasher, RandCoin, VC>(
                        proof,
                        pub_inputs,
                        &acceptable,
                    )
                    .is_err(),
                    "malformed trace must never verify"
                );
            }
        }
    }

    #[test]
    fn valid_interior_value_verifies() {
        let proof = prove_range_membership_v1(35 << 16, 18 << 16, 65 << 16).expect("prove");
        verify_range_membership_v1(proof, 18 << 16, 65 << 16).expect("verify");
    }

    #[test]
    fn exact_boundaries_verify() {
        let min = -5_i64 << 16;
        let max = 9_i64 << 16;

        let proof = prove_range_membership_v1(min, min, max).expect("prove min");
        verify_range_membership_v1(proof, min, max).expect("verify min");

        let proof = prove_range_membership_v1(max, min, max).expect("prove max");
        verify_range_membership_v1(proof, min, max).expect("verify max");
    }

    #[test]
    fn negative_and_cross_zero_intervals_verify() {
        let proof = prove_range_membership_v1(-7 << 16, -10 << 16, -2 << 16).expect("prove");
        verify_range_membership_v1(proof, -10 << 16, -2 << 16).expect("verify");

        let proof = prove_range_membership_v1(-1, -10, 10).expect("prove cross zero");
        verify_range_membership_v1(proof, -10, 10).expect("verify cross zero");
    }

    #[test]
    fn exact_value_interval_verifies() {
        let raw = 123_456_i64;
        let proof = prove_range_membership_v1(raw, raw, raw).expect("prove");
        verify_range_membership_v1(proof, raw, raw).expect("verify");
    }

    #[test]
    fn honest_prover_rejects_out_of_range_witness() {
        assert!(matches!(
            prove_range_membership_v1(17, 18, 65),
            Err(RangeMembershipV1Error::WitnessOutsideBounds { .. })
        ));
        assert!(matches!(
            prove_range_membership_v1(66, 18, 65),
            Err(RangeMembershipV1Error::WitnessOutsideBounds { .. })
        ));
    }

    #[test]
    fn invalid_bounds_fail_closed() {
        assert!(matches!(
            prove_range_membership_v1(5, 10, 0),
            Err(RangeMembershipV1Error::InvalidBounds { .. })
        ));
        assert!(matches!(
            RangeMembershipPublicInputsV1::new(10, 0),
            Err(RangeMembershipV1Error::InvalidBounds { .. })
        ));
    }

    #[test]
    fn trace_profile_preflight_rejects_wrong_dimensions() {
        let wrong_width = TraceInfo::new(TRACE_WIDTH - 1, RANGE_MEMBERSHIP_V1_TRACE_LENGTH);
        assert!(matches!(
            validate_candidate_trace_info(&wrong_width),
            Err(RangeMembershipV1Error::InvalidProofTraceProfile {
                actual_width,
                actual_length,
                ..
            }) if actual_width == TRACE_WIDTH - 1 && actual_length == RANGE_MEMBERSHIP_V1_TRACE_LENGTH
        ));

        let wrong_length = TraceInfo::new(TRACE_WIDTH, RANGE_MEMBERSHIP_V1_TRACE_LENGTH / 2);
        assert!(matches!(
            validate_candidate_trace_info(&wrong_length),
            Err(RangeMembershipV1Error::InvalidProofTraceProfile {
                actual_width,
                actual_length,
                ..
            }) if actual_width == TRACE_WIDTH && actual_length == RANGE_MEMBERSHIP_V1_TRACE_LENGTH / 2
        ));
    }

    #[test]
    fn full_signed_i64_domain_is_supported() {
        assert_eq!(encode_raw(i64::MIN), 0);
        assert_eq!(encode_raw(i64::MAX), u64::MAX);

        let proof = prove_range_membership_v1(i64::MIN, i64::MIN, i64::MAX)
            .expect("prove full-domain lower edge");
        verify_range_membership_v1(proof, i64::MIN, i64::MAX)
            .expect("verify full-domain lower edge");

        let proof = prove_range_membership_v1(i64::MAX, i64::MIN, i64::MAX)
            .expect("prove full-domain upper edge");
        verify_range_membership_v1(proof, i64::MIN, i64::MAX)
            .expect("verify full-domain upper edge");
    }

    #[test]
    fn translated_same_width_interval_does_not_verify() {
        let proof = prove_range_membership_v1(35, 18, 65).expect("prove");
        assert!(verify_range_membership_v1(proof, 19, 66).is_err());
    }

    #[test]
    fn mutated_public_min_does_not_verify() {
        let proof = prove_range_membership_v1(35, 18, 65).expect("prove");
        assert!(verify_range_membership_v1(proof, 17, 65).is_err());
    }

    #[test]
    fn mutated_public_max_does_not_verify() {
        let proof = prove_range_membership_v1(35, 18, 65).expect("prove");
        assert!(verify_range_membership_v1(proof, 18, 66).is_err());
    }

    #[test]
    fn statement_profile_tag_is_bound() {
        let proof = prove_range_membership_v1(35, 18, 65).expect("prove");
        let mut pub_inputs = RangeMembershipPublicInputsV1::new(18, 65).unwrap();
        pub_inputs.statement_tag ^= 1;
        let acceptable = AcceptableOptions::OptionSet(vec![candidate_range_membership_v1_options()]);
        assert!(
            winterfell::verify::<RangeMembershipAirV1, Hasher, RandCoin, VC>(
                proof,
                pub_inputs,
                &acceptable,
            )
            .is_err()
        );
    }

    #[test]
    fn witness_is_not_serialized_as_a_public_input() {
        let inputs = RangeMembershipPublicInputsV1::new(i64::MIN, i64::MAX).unwrap();
        let elements = inputs.to_elements();
        assert_eq!(elements.len(), 3);
        assert_eq!(elements[0], BaseElement::from(RANGE_MEMBERSHIP_V1_STATEMENT_TAG));
        assert_eq!(elements[1], BaseElement::from(0u64));
        assert_eq!(elements[2], BaseElement::from(u64::MAX));
        // This is only a public-input census. It is not a zero-knowledge claim.
    }

    #[test]
    fn fabricated_accumulator_is_rejected_by_air() {
        let mut trace = build_trace(35, 18, 65).unwrap();
        trace.set(col::LOW_ACC, LAST_STEP, BaseElement::ZERO);
        let pub_inputs = RangeMembershipPublicInputsV1::new(18, 65).unwrap();
        assert_malformed_trace_rejected(trace, pub_inputs);
    }

    #[test]
    fn non_binary_bit_is_rejected_by_air() {
        let mut trace = build_trace(35, 18, 65).unwrap();
        trace.set(col::HIGH_BIT, 90, BaseElement::from(2u64));
        let pub_inputs = RangeMembershipPublicInputsV1::new(18, 65).unwrap();
        assert_malformed_trace_rejected(trace, pub_inputs);
    }

    #[test]
    fn split_witness_trace_is_rejected_by_air() {
        let mut trace = build_trace(35, 18, 65).unwrap();
        let original = trace.get(col::X_ENCODED, 90);
        trace.set(col::X_ENCODED, 90, original + BaseElement::ONE);
        let pub_inputs = RangeMembershipPublicInputsV1::new(18, 65).unwrap();
        assert_malformed_trace_rejected(trace, pub_inputs);
    }
}
