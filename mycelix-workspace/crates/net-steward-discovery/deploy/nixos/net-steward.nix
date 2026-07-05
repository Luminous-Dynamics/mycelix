{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.net-steward;
in {
  options.services.net-steward = {
    enable = mkEnableOption "Net Steward Witness Daemon";

    bindAddress = mkOption {
      type = types.str;
      default = "127.0.0.1";
      description = "IP address to bind the Net Steward daemon REST API to.";
    };

    port = mkOption {
      type = types.port;
      default = 3030;
      description = "Port to bind the Net Steward daemon REST API to.";
    };

    readOnly = mkOption {
      type = types.bool;
      default = true;
      description = "Hardcodes Net Steward daemon in read-only witness mode.";
    };

    rollbackApplyEnabled = mkOption {
      type = types.bool;
      default = false;
      description = "Enables active generation rollback modifications (Unimplemented/Disabled in beta).";
    };

    zkVerifierEnabled = mkOption {
      type = types.bool;
      default = false;
      description = "Enables ZK-STARK proof verification verification loops.";
    };

    scenarioModeEnabled = mkOption {
      type = types.bool;
      default = false;
      description = "Enables mock scenario configurations for operator testing.";
    };
  };

  config = mkIf cfg.enable {
    systemd.services.net-steward = {
      description = "Net Steward Read-Only Witness Daemon";
      after = [ "network.target" ];
      wantedBy = [ "multi-user.target" ];

      serviceConfig = {
        ExecStart = "${pkgs.net-steward}/bin/net-steward-daemon --bind ${cfg.bindAddress} --port ${toString cfg.port}";
        Restart = "on-failure";
        RestartSec = "5s";

        # Secure isolation systemd defaults
        NoNewPrivileges = true;
        PrivateTmp = true;
        ProtectSystem = "strict";
        ProtectHome = "read-only";
        CapabilityBoundingSet = "";
        RestrictAddressFamilies = [ "AF_INET" "AF_INET6" "AF_UNIX" "AF_NETLINK" ];
      };
    };
  };
}
