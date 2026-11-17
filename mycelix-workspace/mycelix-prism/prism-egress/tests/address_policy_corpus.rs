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
    assert_eq!(
        corpus["provenance"]["registry_last_updated"].as_str(),
        Some("2025-10-09"),
        "address corpus must identify the checked-in IANA registry revision"
    );
    assert_eq!(
        corpus["provenance"]["verified_against_registry_on"].as_str(),
        Some("2026-11-17"),
        "address corpus must record the date its IANA provenance was reviewed"
    );
    assert!(
        corpus["provenance"]["registry_urls"]["ipv4"].as_str().is_some(),
        "IPv4 registry source URL must be retained"
    );
    assert!(
        corpus["provenance"]["registry_urls"]["ipv6"].as_str().is_some(),
        "IPv6 registry source URL must be retained"
    );

    let cases = corpus["cases"]
        .as_array()
        .expect("corpus must contain a cases array");
    assert!(cases.len() >= 47, "corpus unexpectedly small or missing reviewed IANA vectors");

    for id in ["v4-deny-017", "v6-deny-019", "v6-deny-020"] {
        let case = cases
            .iter()
            .find(|case| case["id"].as_str() == Some(id))
            .unwrap_or_else(|| panic!("missing conservative-aggregate case {id}"));
        assert_eq!(case["expected_public"].as_bool(), Some(false));
        assert!(
            case["reason"].as_str().is_some_and(|reason| !reason.is_empty()),
            "conservative false-positive case {id} must explain its rationale"
        );
    }

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

#[test]
fn versioned_dns_answer_set_corpus_enforces_all_or_nothing_policy() {
    let corpus: serde_json::Value =
        serde_json::from_str(include_str!("data/dns_answer_sets.json"))
            .expect("DNS answer-set corpus must be valid JSON");

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
    assert!(cases.len() >= 8, "DNS answer-set corpus unexpectedly small");

    for case in cases {
        let id = case["id"].as_str().expect("case ID");
        let expected = case["expected_allowed"]
            .as_bool()
            .expect("expected_allowed boolean");
        let values = case["addresses"]
            .as_array()
            .expect("addresses array");
        let addresses = values
            .iter()
            .map(|value| {
                value
                    .as_str()
                    .expect("socket address")
                    .parse()
                    .expect("valid socket address")
            })
            .collect::<Vec<std::net::SocketAddr>>();
        let actual = prism_egress::validate_address_set(&addresses).is_ok();
        assert_eq!(actual, expected, "DNS answer-set case {id}");
    }
}
