{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.net-steward-discovery;
in {
  options.services.net-steward-discovery = {
    enable = mkEnableOption "Luminous Net Steward Witness Daemon";

    bindAddress = mkOption {
      type = types.str;
      default = "127.0.0.1";
      description = "IP address to bind the witness daemon HTTP service to.";
    };

    port = mkOption {
      type = types.port;
      default = 3030;
      description = "Port to bind the witness daemon HTTP service to.";
    };

    readOnly = mkOption {
      type = types.bool;
      default = true;
      description = "Force the daemon to run in read-only witness mode.";
    };

    package = mkOption {
      type = types.package;
      default = pkgs.net-steward-discovery; # Set default package placeholder
      description = "The net-steward-discovery package to use.";
    };
  };

  config = mkIf cfg.enable {
    systemd.services.net-steward-discovery = {
      description = "Luminous Net Steward Witness Daemon";
      after = [ "network.target" ];
      wantedBy = [ "multi-user.target" ];

      environment = {
        PORT = toString cfg.port;
        BIND_ADDRESS = cfg.bindAddress;
        READ_ONLY = if cfg.readOnly then "true" else "false";
      };

      serviceConfig = {
        ExecStart = "${cfg.package}/bin/net-steward-daemon";
        Restart = "on-failure";

        # systemd hardening rules
        NoNewPrivileges = true;
        PrivateTmp = true;
        ProtectSystem = "strict";
        ProtectHome = "read-only";
        RestrictAddressFamilies = [ "AF_INET" "AF_INET6" "AF_UNIX" "AF_NETLINK" ];
        CapabilityBoundingSet = "";
        MemoryDenyWriteExecute = true;
        RestrictRealtime = true;
        DevicePolicy = "strict";
      };
    };
  };
}
