// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Regression contract for the current Personal route surface.
//!
//! This module is test-only. It does not become a second router or introduce a
//! local task-shell abstraction; it freezes the existing declarations so page
//! extraction can be proven semantics-preserving before the shared task shell
//! lineage is adopted.

#[cfg(test)]
mod tests {
    const APP_SOURCE: &str = include_str!("app.rs");

    const ROUTES: [(&str, &str); 11] = [
        ("path!(\"/\")", "view=VaultPage"),
        ("path!(\"/identity\")", "view=IdentityPage"),
        ("path!(\"/wallet\")", "view=WalletPage"),
        ("path!(\"/health\")", "view=HealthPage"),
        ("path!(\"/preferences\")", "view=PreferencesPage"),
        ("path!(\"/activity\")", "view=ActivityPage"),
        ("path!(\"/profile\")", "view=IdentityPage"),
        ("path!(\"/unlock\")", "view=UnlockPage"),
        (
            "path!(\"/civic\")",
            "EmbeddedSatellite name=\"Civic\" port=5174",
        ),
        (
            "path!(\"/knowledge\")",
            "EmbeddedSatellite name=\"Knowledge\" port=5175",
        ),
        (
            "path!(\"/finance\")",
            "EmbeddedSatellite name=\"Finance\" port=5176",
        ),
    ];

    fn declared_route_lines() -> Vec<&'static str> {
        APP_SOURCE
            .lines()
            .filter(|line| line.contains("<Route path=path!("))
            .collect()
    }

    #[test]
    fn personal_route_inventory_is_exact() {
        let lines = declared_route_lines();
        assert_eq!(
            lines.len(),
            ROUTES.len(),
            "Personal route count changed; update the route contract only with an explicit semantic tranche"
        );

        for (path, binding) in ROUTES {
            let line = lines
                .iter()
                .find(|line| line.contains(path))
                .unwrap_or_else(|| panic!("missing Personal route declaration for {path}"));
            assert!(
                line.contains(binding),
                "Personal route {path} changed binding: {line}"
            );
        }
    }

    #[test]
    fn profile_route_remains_identity_alias() {
        let line = declared_route_lines()
            .into_iter()
            .find(|line| line.contains("path!(\"/profile\")"))
            .expect("/profile route must remain present during structural extraction");
        assert!(line.contains("view=IdentityPage"));
    }

    #[test]
    fn satellite_ports_remain_explicit_local_scaffold_contract() {
        for (path, name, port) in [
            ("/civic", "Civic", "5174"),
            ("/knowledge", "Knowledge", "5175"),
            ("/finance", "Finance", "5176"),
        ] {
            let line = declared_route_lines()
                .into_iter()
                .find(|line| line.contains(&format!("path!(\"{path}\")")))
                .unwrap_or_else(|| panic!("missing satellite route {path}"));
            assert!(line.contains(&format!("name=\"{name}\"")));
            assert!(line.contains(&format!("port={port}")));
        }
    }
}
