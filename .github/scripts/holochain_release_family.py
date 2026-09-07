#!/usr/bin/env python3
"""Shared fail-closed Holochain release-family and source classification.

Holochain's package ecosystem does not use one universal version number. Most core
crates follow the conductor release line, while HDI, the client, CHC, serialization,
Wasmer, Kitsune2, and Lair each have their own version relationships. Qualification
code must classify those relationships explicitly instead of inferring them from a
package-name prefix.

For this migration lineage there are no intentional Cargo source replacements or
crates.io patches. A tracked family package therefore must also resolve from the
official crates.io registry. Same-version git/path substitutions fail qualification.

Unknown ``holochain_*`` packages deliberately raise until their upstream release
relationship is reviewed and added here.
"""

from __future__ import annotations

from collections.abc import Mapping

CRATES_IO_SOURCE = "registry+https://github.com/rust-lang/crates.io-index"

# Crates verified to follow the main Holochain 0.6.x release line. This includes
# the SweetConductor test-WASM crates pulled by holochain/sweettest -> test_utils.
HOLOCHAIN_RELEASE_COUPLED = frozenset(
    {
        "holochain",
        "holochain_cascade",
        "holochain_conductor_api",
        "holochain_conductor_config",
        "holochain_integrity_types",
        "holochain_keystore",
        "holochain_metrics",
        "holochain_nonce",
        "holochain_p2p",
        "holochain_secure_primitive",
        "holochain_sqlite",
        "holochain_state",
        "holochain_state_types",
        "holochain_test_wasm_common",
        "holochain_timestamp",
        "holochain_trace",
        "holochain_types",
        "holochain_util",
        "holochain_wasm_test_utils",
        "holochain_websocket",
        "holochain_zome_types",
    }
)


class UnclassifiedFamilyPackage(ValueError):
    """Raised when a Holochain-family name lacks an explicit release rule."""


class FamilySourceMismatch(ValueError):
    """Raised when a tracked family package resolves from an unauthorized source."""


def expected_family_version(name: str, contract: Mapping) -> str | None:
    """Return the exact qualified version for a tracked family package.

    ``None`` means the package is outside the compatibility family this migration
    qualifies. A Holochain-looking package without a known rule fails closed.
    """

    rust = contract["rust"]
    target = contract["next_0_6"]

    if name == "hdi":
        return rust["hdi"]
    if name in {"hdk", "hdk_derive"}:
        return rust["hdk"]
    if name == "holochain_client":
        return rust["holochain_client"]
    if name == "holo_hash":
        return rust["holo_hash"]
    if name in {"holochain_serialized_bytes", "holochain_serialized_bytes_derive"}:
        return rust["holochain_serialized_bytes"]

    # Upstream holochain-wasmer v0.0.102 uses one workspace version for common,
    # guest, and host. Keep the explicit family prefix because those package names
    # are owned by that separately versioned repository, not the main conductor.
    if name.startswith("holochain_wasmer_"):
        return rust["holochain_wasmer_host"]

    # CHC intentionally versions independently; Holochain 0.6.3 uses CHC 0.3.3.
    if name == "holochain_chc":
        return target["holochain_chc"]

    if name in HOLOCHAIN_RELEASE_COUPLED:
        return rust["holochain"]
    if name.startswith("holochain_"):
        raise UnclassifiedFamilyPackage(
            f"unclassified Holochain-family package {name!r}; "
            "classify its upstream version line explicitly"
        )

    # Kitsune2 v0.4.1 and Lair v0.6.3 each use a workspace-wide package version.
    if name == "kitsune2" or name.startswith("kitsune2_"):
        return rust["kitsune2"]
    if name == "lair_keystore" or name.startswith("lair_keystore_"):
        return rust["lair_keystore"]

    return None


def qualified_family_package(package: Mapping, contract: Mapping) -> tuple[str, str] | None:
    """Return ``(expected_version, expected_source)`` for a tracked package.

    The package name selects the release-family rule. Tracked packages must resolve
    from crates.io in this lineage; source overrides are qualification failures even
    when they preserve the expected semantic version string.
    """

    name = package["name"]
    expected = expected_family_version(name, contract)
    if expected is None:
        return None

    source = package.get("source")
    if source != CRATES_IO_SOURCE:
        raise FamilySourceMismatch(
            f"{name!r} resolved from {source!r}, expected {CRATES_IO_SOURCE!r}"
        )
    return expected, CRATES_IO_SOURCE
