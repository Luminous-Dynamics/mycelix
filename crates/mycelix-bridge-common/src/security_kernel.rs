        let malformed: Capability = serde_json::from_str(
            r#"{
                "subject":"alice",
                "issuer":"issuer",
                "resource":"ledger",
                "actions":["Read","Read"],
                "not_before_us":1,
                "expires_at_us":2,
                "policy_version":1
            }"#,
        )
        .unwrap();
        let evidence =
            VerificationEvidence::new_for_capability(&malformed, true, true, true);
        assert_eq!(verify_capability(malformed, evidence, 1),
            Err(AuthorizationDecision::Deny(
                AuthorizationDenial::InvalidCapability,
            ))
        );
    }
