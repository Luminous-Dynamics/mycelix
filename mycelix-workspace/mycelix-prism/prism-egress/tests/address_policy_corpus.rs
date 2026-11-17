use std::net::IpAddr;

#[test]
fn versioned_address_policy_corpus_matches_prism_classifier() {
    let corpus: serde_json::Value =
        serde_json::from_str(include_str!("data/address_policy_cases.json"))
            .expect("benchmark corpus must be valid JSON");

    assert_eq!(
        corpus["schema_version"].as_str(),
        Some("1.0.0"),
        "unexpected corpus schema version"
    );
    assert_eq!(
        corpus["policy_snapshot"].as_str(),
        Some("prism-public-egress-v1"),
        "corpus policy version must match the tested policy"
    );

    let cases = corpus["cases"]
        .as_array()
        .expect("corpus must contain a cases array");
    assert!(cases.len() >= 30, "corpus unexpectedly small");

    for case in cases {
        let id = case["id"].as_str().expect("case ID");
        let raw = case["address"].as_str().expect("IP address");
        let expected = case["expected_public"]
            .as_bool()
            .expect("expected_public boolean");
        let address: IpAddr = raw.parse().expect("valid IP address in corpus");
        let actual = prism_egress::is_public_unicast_ip(&address);
        assert_eq!(actual, expected, "case {id} ({raw})");
    }
}
