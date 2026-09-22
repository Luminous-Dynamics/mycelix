use mycelix_assurance_core::{
    DecodeFailureCode, DecodeFailureWitness, DecodeLimits, ForbiddenValueKind, ResourceLimit,
    decode_canonical,
};

const V001_CANONICAL_ARRAY: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v001-canonical-array.cbor");
const V002_CANONICAL_MAP: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v002-canonical-map.cbor");
const V003_CANONICAL_BYTES: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v003-canonical-bytes.cbor");
const V004_CANONICAL_TEXT: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v004-canonical-text.cbor");
const V005_CANONICAL_TRUE: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v005-canonical-true.cbor");
const V006_TEXT_NFC: &[u8] = include_bytes!("../../../assurance/vectors/v1/l0/v006-text-nfc.cbor");
const V007_TEXT_NFD: &[u8] = include_bytes!("../../../assurance/vectors/v1/l0/v007-text-nfd.cbor");
const V101_NON_SHORTEST_INTEGER: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v101-non-shortest-integer.cbor");
const V102_INDEFINITE_ARRAY: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v102-indefinite-array.cbor");
const V103_UNSORTED_MAP: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v103-unsorted-map.cbor");
const V104_DUPLICATE_MAP_KEY: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v104-duplicate-map-key.cbor");
const V105_TEXT_MAP_KEY: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v105-text-map-key.cbor");
const V106_FLOAT: &[u8] = include_bytes!("../../../assurance/vectors/v1/l0/v106-float.cbor");
const V107_TAG: &[u8] = include_bytes!("../../../assurance/vectors/v1/l0/v107-tag.cbor");
const V108_NULL: &[u8] = include_bytes!("../../../assurance/vectors/v1/l0/v108-null.cbor");
const V109_TRAILING_DATA: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v109-trailing-data.cbor");
const V110_INVALID_UTF8: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v110-invalid-utf8.cbor");
const V111_NEGATIVE_MAP_KEY: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v111-negative-map-key.cbor");
const V112_SIMPLE_VALUE: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v112-simple-value.cbor");
const V113_EMPTY_INPUT: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v113-empty-input.cbor");
const V114_TRUNCATED_ARRAY: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v114-truncated-array.cbor");
const V115_UNDEFINED: &[u8] =
    include_bytes!("../../../assurance/vectors/v1/l0/v115-undefined.cbor");

fn code(input: &[u8]) -> DecodeFailureCode {
    decode_canonical(input, &DecodeLimits::V1_INITIAL)
        .expect_err("fixture must be rejected")
        .code()
}

fn assert_resource_limit(input: &[u8], limits: DecodeLimits, expected: ResourceLimit) {
    let failure = decode_canonical(input, &limits).unwrap_err();
    assert_eq!(failure.code(), DecodeFailureCode::ResourceLimitExceeded);
    assert_eq!(
        failure.witness(),
        DecodeFailureWitness::ResourceLimit(expected)
    );
}

#[test]
fn frozen_positive_vectors_are_accepted_verbatim() {
    for input in [
        V001_CANONICAL_ARRAY,
        V002_CANONICAL_MAP,
        V003_CANONICAL_BYTES,
        V004_CANONICAL_TEXT,
        V005_CANONICAL_TRUE,
        V006_TEXT_NFC,
        V007_TEXT_NFD,
    ] {
        let canonical = decode_canonical(input, &DecodeLimits::V1_INITIAL).unwrap();
        assert_eq!(canonical.as_bytes(), input);
    }
}

#[test]
fn distinct_unicode_wire_forms_are_preserved() {
    let nfc = decode_canonical(V006_TEXT_NFC, &DecodeLimits::V1_INITIAL).unwrap();
    let nfd = decode_canonical(V007_TEXT_NFD, &DecodeLimits::V1_INITIAL).unwrap();
    assert_ne!(nfc.as_bytes(), nfd.as_bytes());
    assert_eq!(nfc.as_bytes(), V006_TEXT_NFC);
    assert_eq!(nfd.as_bytes(), V007_TEXT_NFD);
}

#[test]
fn frozen_noncanonical_vectors_are_rejected() {
    for input in [
        V101_NON_SHORTEST_INTEGER,
        V102_INDEFINITE_ARRAY,
        V103_UNSORTED_MAP,
    ] {
        assert_eq!(code(input), DecodeFailureCode::NonCanonicalEncoding);
    }
}

#[test]
fn frozen_duplicate_key_vector_has_stable_witness() {
    let failure = decode_canonical(V104_DUPLICATE_MAP_KEY, &DecodeLimits::V1_INITIAL).unwrap_err();
    assert_eq!(failure.code(), DecodeFailureCode::DuplicateMapKey);
    assert_eq!(failure.witness(), DecodeFailureWitness::DuplicateMapKey(1));
}

#[test]
fn frozen_invalid_key_vectors_are_rejected() {
    for input in [V105_TEXT_MAP_KEY, V111_NEGATIVE_MAP_KEY] {
        assert_eq!(code(input), DecodeFailureCode::InvalidMapKey);
    }
}

#[test]
fn frozen_forbidden_kind_vectors_are_rejected() {
    let fixtures = [
        (V106_FLOAT, ForbiddenValueKind::Float),
        (V107_TAG, ForbiddenValueKind::Tag),
        (V108_NULL, ForbiddenValueKind::Null),
        (V112_SIMPLE_VALUE, ForbiddenValueKind::Simple),
    ];

    for (bytes, expected_kind) in fixtures {
        let failure = decode_canonical(bytes, &DecodeLimits::V1_INITIAL).unwrap_err();
        assert_eq!(failure.code(), DecodeFailureCode::ForbiddenValueKind);
        assert_eq!(
            failure.witness(),
            DecodeFailureWitness::ForbiddenValueKind(expected_kind)
        );
    }
}

#[test]
fn frozen_undefined_wire_value_is_rejected() {
    // cbor2 1.1.5 deliberately maps CBOR undefined (0xf7) to Value::Null.
    // The profile requirement is therefore frozen at the decode failure-code
    // boundary rather than inventing a distinct semantic witness the parser
    // cannot preserve.
    assert_eq!(code(V115_UNDEFINED), DecodeFailureCode::ForbiddenValueKind);
}

#[test]
fn frozen_malformed_vectors_are_rejected() {
    for input in [
        V109_TRAILING_DATA,
        V110_INVALID_UTF8,
        V113_EMPTY_INPUT,
        V114_TRUNCATED_ARRAY,
    ] {
        assert_eq!(code(input), DecodeFailureCode::MalformedEncoding);
    }
}

#[test]
fn every_declared_resource_limit_is_enforced() {
    assert_resource_limit(
        V001_CANONICAL_ARRAY,
        DecodeLimits {
            max_input_bytes: 1,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::InputBytes,
    );

    assert_resource_limit(
        V001_CANONICAL_ARRAY,
        DecodeLimits {
            max_nesting_depth: 0,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::NestingDepth,
    );

    assert_resource_limit(
        V001_CANONICAL_ARRAY,
        DecodeLimits {
            max_total_items: 3,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::TotalItems,
    );

    assert_resource_limit(
        V001_CANONICAL_ARRAY,
        DecodeLimits {
            max_array_len: 2,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::ArrayLength,
    );

    assert_resource_limit(
        V002_CANONICAL_MAP,
        DecodeLimits {
            max_map_len: 1,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::MapLength,
    );

    assert_resource_limit(
        V003_CANONICAL_BYTES,
        DecodeLimits {
            max_bytes_len: 1,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::ByteStringLength,
    );

    assert_resource_limit(
        V004_CANONICAL_TEXT,
        DecodeLimits {
            max_text_bytes: 1,
            ..DecodeLimits::V1_INITIAL
        },
        ResourceLimit::TextLength,
    );
}

#[test]
fn parser_safety_ceiling_has_nesting_resource_precedence() {
    let limits = DecodeLimits {
        max_nesting_depth: u8::MAX,
        ..DecodeLimits::V1_INITIAL
    };

    let mut canonical = vec![0x81; 300];
    canonical.push(0x00);
    assert_resource_limit(&canonical, limits, ResourceLimit::NestingDepth);

    // Even if the item is also truncated below the parser's safety horizon,
    // the active ASSURE nesting policy was necessarily exceeded first.
    let truncated = vec![0x81; 300];
    assert_resource_limit(&truncated, limits, ResourceLimit::NestingDepth);
}

#[test]
fn trailing_payload_is_not_parsed_for_exact_one_check() {
    let limits = DecodeLimits {
        max_nesting_depth: 1,
        ..DecodeLimits::V1_INITIAL
    };

    let mut input = vec![0x00];
    input.extend(vec![0x81; 300]);
    input.push(0x00);

    let failure = decode_canonical(&input, &limits).unwrap_err();
    assert_eq!(failure.code(), DecodeFailureCode::MalformedEncoding);
    assert_eq!(failure.witness(), DecodeFailureWitness::None);
}

#[test]
fn stable_failure_codes_match_registry() {
    let cases = [
        (
            DecodeFailureCode::MalformedEncoding,
            "D001_MALFORMED_ENCODING",
        ),
        (
            DecodeFailureCode::NonCanonicalEncoding,
            "D002_NON_CANONICAL_ENCODING",
        ),
        (
            DecodeFailureCode::UnsupportedSchema,
            "D003_UNSUPPORTED_SCHEMA",
        ),
        (
            DecodeFailureCode::ResourceLimitExceeded,
            "D004_RESOURCE_LIMIT_EXCEEDED",
        ),
        (
            DecodeFailureCode::ForbiddenValueKind,
            "D005_FORBIDDEN_VALUE_KIND",
        ),
        (DecodeFailureCode::InvalidMapKey, "D006_INVALID_MAP_KEY"),
        (DecodeFailureCode::DuplicateMapKey, "D007_DUPLICATE_MAP_KEY"),
    ];

    for (code, expected) in cases {
        assert_eq!(code.stable_code(), expected);
    }
}
