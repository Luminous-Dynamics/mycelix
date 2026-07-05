# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
# Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
# Prism — NixOS production services
#
# Deployment:
#   1. Build: cd /srv/luminous-dynamics/mycelix-workspace && cargo build -p prism-serve -p prism-proxy --release
#   2. Build UI: cd mycelix-prism/prism-ui && trunk build --release
#   3. Mirror this file to /etc/nixos/modules/system/prism-service.nix and
#      import it there (flakes evaluate in pure mode — an absolute-path
#      import of a file outside /etc/nixos fails with "access to absolute
#      path ... is forbidden in pure evaluation mode", found 2026-07-04).
#   4. sudo nixos-rebuild switch
#
# Services:
#   - prism: static file server for the WASM app (port 8130)
#   - prism-proxy: CORS proxy for URL fetching (port 8131)

{ config, pkgs, ... }:

let
  prismDir = "/srv/luminous-dynamics/mycelix-workspace/mycelix-prism";
  distDir = "${prismDir}/prism-ui/dist";
  # mycelix-prism's crates are members of the mycelix-workspace Cargo
  # workspace, so `cargo build --release` places binaries in the
  # *workspace root's* target/, not mycelix-prism/target/ (which doesn't
  # exist at all) -- found 2026-07-04 while deploying this module for the
  # first time; this path was wrong from the start.
  workspaceRoot = "/srv/luminous-dynamics/mycelix-workspace";
  serveBin = "${workspaceRoot}/target/release/prism-serve";
  proxyBin = "${workspaceRoot}/target/release/prism-proxy";

  # Bridges to the Symthaea Service Daemon's Unix socket (see
  # /srv/luminous-dynamics/symthaea/nix/modules/symthaea-service.nix and
  # Part 2 of /home/tstoltz/.claude/plans/fuzzy-beaming-brook.md) via the
  # opt-in "Ask Symthaea" route. Reuses that module's bearer-token secret
  # rather than minting a second one, to avoid the two ever drifting out
  # of sync. sops-nix decrypts secrets to their raw value with no
  # `KEY=value` framing, so this needs the same launcher-wrapper pattern
  # symthaea-service.nix uses, not a plain EnvironmentFile=.
  prismLauncher = pkgs.writeShellScript "prism-serve-launcher" ''
    set -euo pipefail
    export SYMTHAEA_SERVICE_BEARER_TOKEN="$(cat "$CREDENTIALS_DIRECTORY/bearer_token")"
    exec "${serveBin}"
  '';
in
{
  # Redeclared here (matching symthaea-service.nix's declaration, same
  # value) so this module evaluates standalone even if symthaea-service.nix
  # isn't imported — the secret's backing entry in secrets.yaml still needs
  # to exist regardless, but the option itself won't error on a missing
  # attribute path.
  sops.secrets.symthaea_service_bearer_token.owner = "tstoltz";

  # ── Static file server (serves the WASM app) ──
  systemd.services.prism = {
    description = "Prism — Epistemic Search Browser";
    wantedBy = [ "multi-user.target" ];
    after = [ "network.target" "symthaea-service.service" ];

    environment = {
      PRISM_DIST = distDir;
      PRISM_ADDR = "0.0.0.0:8130";
      RUST_LOG = "info";
      SYMTHAEA_SOCKET_PATH = "/run/symthaea-service/symthaea.sock";
    };

    # StartLimitBurst/StartLimitIntervalSec are [Unit]-section directives,
    # not [Service] — under serviceConfig they produce a silent "Unknown
    # key ... ignoring" warning and the Restart-limit protection never
    # actually applies (found 2026-07-04 while deploying the analogous
    # symthaea-service module, which had copied this same pattern).
    unitConfig = {
      StartLimitBurst = 5;
      StartLimitIntervalSec = 60;
    };

    serviceConfig = {
      Type = "simple";
      User = "tstoltz";
      LoadCredential = [ "bearer_token:${config.sops.secrets.symthaea_service_bearer_token.path}" ];
      ExecStart = "${prismLauncher}";
      Restart = "always";
      RestartSec = "3s";

      # Security hardening
      NoNewPrivileges = true;
      ProtectSystem = "strict";
      ProtectHome = "read-only";
      ReadOnlyPaths = [ distDir ];
      PrivateTmp = true;
    };
  };

  # ── CORS proxy (enables URL fetching from WASM) ──
  systemd.services.prism-proxy = {
    description = "Prism CORS Proxy";
    wantedBy = [ "multi-user.target" ];
    after = [ "network.target" ];

    environment = {
      RUST_LOG = "info";
    };

    unitConfig = {
      StartLimitBurst = 5;
      StartLimitIntervalSec = 60;
    };

    serviceConfig = {
      Type = "simple";
      User = "tstoltz";
      ExecStart = proxyBin;
      Restart = "always";
      RestartSec = "3s";

      NoNewPrivileges = true;
      ProtectSystem = "strict";
      ProtectHome = "read-only";
      PrivateTmp = true;
    };
  };
}
