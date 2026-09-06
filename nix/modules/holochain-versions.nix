# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# Single review surface for Mycelix's Holochain compatibility boundary.
#
# `active` describes the version family currently qualified on main.
# `candidate` describes the next family under migration.  Do not promote the
# candidate by changing this file alone: the qualification workflow must first
# prove zome semantics, client/signal compatibility, fresh-conductor operation,
# and the explicit no-database-migration boundary.
{
  active = {
    series = "0.6";
    holochain = "0.6.1";
    hdk = "0.6.1";
    hdi = "0.7.1";
    holochainClient = "0.8.1";
    serializedBytes = "0.0.57";
    wasmerHost = "0.0.102";
    kitsune2 = "0.4.1";
    lairKeystore = "0.6.3";
    holonixCommit = "d21b35431e425e615bc05da790987380a84b8280";
  };

  candidate = {
    series = "0.7";
    holochain = "0.7.0";
    hdk = "0.7.0";
    hdi = "0.8.0";
    holochainClient = "0.9.0";
    serializedBytes = "0.0.57";
    wasmerHost = "0.0.103";
    kitsune2 = "0.5.0";
    lairKeystore = "0.7.1";
    holonixCommit = "ffcc7c63b4b87dde16a69247775639b49c5778b1";
  };

  # 0.7 deliberately does not carry holochain_sqlite as a normal compatibility
  # pin.  The crate is published as 0.7.0+deprecated and Holochain 0.7 moved the
  # conductor persistence boundary to holochain_data.  Any Mycelix direct use of
  # holochain_sqlite/holochain_state must therefore be audited and removed or
  # explicitly justified during migration rather than silently version-bumped.
  candidateDeprecatedDirectDependencies = [
    "holochain_sqlite"
    "holochain_state"
  ];
}
