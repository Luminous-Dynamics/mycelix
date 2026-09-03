use mycelix_content_planner::{PlannerNormalizationProfileV1, PlannerProfileIdV1};
use mycelix_content_planner_protocol::{decode_request_json_v1, ExternalPlannerRequestIdV1};

#[test]
fn external_planner_request_v1_matches_cross_repo_golden_vector() {
    let bytes = include_bytes!("fixtures/external_planner_request_v1.json");
    let request = decode_request_json_v1(bytes).expect("decode golden request");
    let expected_profile = PlannerProfileIdV1([
        0xcb, 0x4f, 0x6c, 0x8e, 0xba, 0x88, 0xb5, 0x50, 0xd9, 0x5f, 0x6f, 0x7f, 0xdd,
        0x78, 0x66, 0x2a, 0x0c, 0xa1, 0xf1, 0x76, 0xea, 0x17, 0xfa, 0xd3, 0x62, 0x72,
        0xe0, 0x32, 0x0b, 0x7c, 0x7a, 0x86,
    ]);
    let expected_request = ExternalPlannerRequestIdV1([
        0xd3, 0xf8, 0xe7, 0xf6, 0x4b, 0x1e, 0xae, 0xe2, 0x63, 0xbd, 0x7c, 0x1f, 0x05,
        0xd9, 0x90, 0xb2, 0xeb, 0x43, 0xe8, 0x84, 0x73, 0x73, 0x31, 0x39, 0xac, 0x6a,
        0x2a, 0xc9, 0x72, 0xba, 0xf4, 0x54,
    ]);
    let profile = PlannerNormalizationProfileV1::new(1_000, 500, 10_000, 20_000)
        .expect("golden normalization profile is valid");

    assert_eq!(profile.id, expected_profile);
    assert_eq!(request.profile.id, expected_profile);
    assert_eq!(request.id, expected_request);
    assert_eq!(request.recompute_id(), expected_request);
    assert!(request.validate_id());
}
