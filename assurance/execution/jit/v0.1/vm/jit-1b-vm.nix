{ pkgs, jitRoot }:

let
  nixosTest =
    if pkgs ? testers && pkgs.testers ? nixosTest
    then pkgs.testers.nixosTest
    else throw "pkgs.testers.nixosTest is missing (nixpkgs too old?)";

  jitFixture = pkgs.writeShellApplication {
    name = "jit-1b-inert";
    runtimeInputs = [ pkgs.coreutils pkgs.python3 ];
    text = ''
      set -euo pipefail

      # Prove the transient unit actually applied the intended privilege ceiling.
      test "$(id -u)" -ne 0
      for field in CapEff CapBnd CapAmb; do
        value="$(awk -v key="$field:" '$1 == key { print $2 }' /proc/self/status)"
        test "$value" = "0000000000000000"
      done

      for var in \
        GITHUB_TOKEN GH_TOKEN \
        ACTIONS_ID_TOKEN_REQUEST_TOKEN ACTIONS_ID_TOKEN_REQUEST_URL \
        AWS_ACCESS_KEY_ID AWS_SECRET_ACCESS_KEY \
        AZURE_CLIENT_SECRET GOOGLE_APPLICATION_CREDENTIALS
      do
        test -z "''${!var-}"
      done

      ${pkgs.python3}/bin/python3 - <<'PY'
      import errno
      import socket

      for family in (socket.AF_INET, socket.AF_INET6):
          try:
              sock = socket.socket(family, socket.SOCK_STREAM)
          except OSError as exc:
              if exc.errno not in (errno.EAFNOSUPPORT, errno.EPERM):
                  raise
          else:
              sock.close()
              raise SystemExit(f"network address family unexpectedly available: {family}")
      print("JIT-1B_SANDBOX_SELFTEST_PASS")
      PY

      state=/run/jit-1b
      mkdir -p "$state"

      if ! ( set -o noclobber; : > "$state/consumed" ) 2>/dev/null; then
        echo "JIT-1B_REUSE_BLOCKED" >&2
        exit 23
      fi

      cd /etc/mycelix-jit
      ${pkgs.python3}/bin/python3 harness/check_lock.py
      ${pkgs.python3}/bin/python3 check_spec.py
      ${pkgs.python3}/bin/python3 harness/check_harness.py
      ${pkgs.python3}/bin/python3 harness/check_policy_guard.py
      ${pkgs.python3}/bin/python3 harness/check_lock.py

      printf '%s\n' "JIT-1B_INERT_FIXTURE_PASS" > "$state/result"
      cat "$state/result"
    '';
  };
in
nixosTest {
  name = "mycelix-jit-1b-isolation";

  nodes.machine = { pkgs, lib, ... }: {
    networking.hostName = "jit-1b";
    networking.useDHCP = false;
    services.openssh.enable = false;

    users.groups.jitqual = { };
    users.users.jitqual = {
      isSystemUser = true;
      group = "jitqual";
      home = "/var/empty";
    };

    environment.systemPackages = [
      jitFixture
      pkgs.coreutils
      pkgs.curl
      pkgs.gnugrep
      pkgs.iproute2
      pkgs.python3
      pkgs.systemd
      pkgs.util-linux
    ];

    # Entire frozen JIT subtree is available read-only from the Nix store.
    environment.etc."mycelix-jit".source = jitRoot;

    virtualisation = {
      memorySize = 1024;
      cores = 2;
      graphics = false;

      # No persistent root disk between VM instances.
      diskImage = null;

      # Do not share the host Nix store. Build a private, read-only store image.
      mountHostNixStore = false;
      useNixStoreImage = true;
      writableStore = false;

      # qemu-vm.nix normally injects writable xchg/shared 9p mounts for the
      # test driver. Force the merged option empty so those devices are never
      # part of this qualification VM. Shell control remains virtio-console.
      sharedDirectories = lib.mkForce { };

      # No test VLAN and no guest routing through the host/outside world.
      vlans = [ ];
      restrictNetwork = true;
    };

    system.stateVersion = "26.05";
  };

  testScript = ''
    def assert_no_host_exchange():
        machine.succeed("! mountpoint -q /tmp/shared")
        machine.succeed("! mountpoint -q /tmp/xchg")
        machine.succeed("! findmnt -rn -t 9p,virtiofs")

    def prepare_subject_state():
        machine.succeed("install -d -o jitqual -g jitqual -m 0700 /run/jit-1b")

    def subject_command(unit):
        return (
            "systemd-run --quiet --wait --pipe "
            f"--unit={unit} "
            "--property=User=jitqual "
            "--property=Group=jitqual "
            "--property=NoNewPrivileges=yes "
            "--property=CapabilityBoundingSet= "
            "--property=AmbientCapabilities= "
            "--property=PrivateDevices=yes "
            "--property=PrivateTmp=yes "
            "--property=PrivateMounts=yes "
            "--property=ProtectSystem=strict "
            "--property=ProtectHome=yes "
            "--property=ProtectKernelTunables=yes "
            "--property=ProtectKernelModules=yes "
            "--property=ProtectKernelLogs=yes "
            "--property=ProtectControlGroups=yes "
            "--property=ProtectHostname=yes "
            "--property=ProtectClock=yes "
            "--property=RestrictNamespaces=yes "
            "--property=RestrictSUIDSGID=yes "
            "--property=LockPersonality=yes "
            "--property=RemoveIPC=yes "
            "--property=SystemCallArchitectures=native "
            "--property=RestrictAddressFamilies=AF_UNIX "
            "--property=ReadWritePaths=/run/jit-1b "
            "--property=UMask=0077 "
            "--setenv=HOME=/var/empty "
            "--setenv=PATH=/run/current-system/sw/bin "
            "jit-1b-inert"
        )

    def run_subject(unit):
        return machine.succeed(subject_command(unit))

    start_all()
    machine.wait_for_unit("multi-user.target")

    first_boot = machine.succeed("cat /proc/sys/kernel/random/boot_id").strip()
    machine.succeed("test ! -e /run/jit-1b")
    assert_no_host_exchange()
    prepare_subject_state()

    # Clean-instance / no-host-capability checks.
    machine.succeed("test ! -e /run/jit-1b/consumed")
    machine.succeed("test ! -S /var/run/docker.sock")
    machine.succeed("test ! -S /run/podman/podman.sock")
    machine.succeed("! systemctl show-environment | grep -E '^(GITHUB_TOKEN|GH_TOKEN|ACTIONS_ID_TOKEN_REQUEST_TOKEN|ACTIONS_ID_TOKEN_REQUEST_URL|AWS_ACCESS_KEY_ID|AWS_SECRET_ACCESS_KEY|AZURE_CLIENT_SECRET|GOOGLE_APPLICATION_CREDENTIALS)='")

    # The VM itself must not be able to reach the public network.
    machine.fail("curl -fsS --connect-timeout 2 http://1.1.1.1/")

    first_result = run_subject("jit-fixture-first")
    print("JIT1B_EXTERNAL_LOG_FIRST_BEGIN")
    print(first_result)
    print("JIT1B_EXTERNAL_LOG_FIRST_END")
    machine.succeed("grep -Fx JIT-1B_INERT_FIXTURE_PASS /run/jit-1b/result")
    assert_no_host_exchange()

    # A second job attempt in the exact same sandbox must fail closed.
    machine.fail(subject_command("jit-fixture-reuse"))
    machine.succeed("test \"$(systemctl show -p ExecMainStatus --value jit-fixture-reuse.service)\" -eq 23")
    machine.succeed("grep -Fx JIT-1B_INERT_FIXTURE_PASS /run/jit-1b/result")
    assert_no_host_exchange()

    # Prove ordinary root state, not merely /run, disappears across VM restart.
    machine.succeed("printf '%s\n' JIT-1B-EPHEMERAL-SENTINEL > /var/lib/jit-1b-persistence-sentinel")
    machine.succeed("grep -Fx JIT-1B-EPHEMERAL-SENTINEL /var/lib/jit-1b-persistence-sentinel")

    # Explicitly terminate the first disposable environment.
    machine.shutdown()
    machine.wait_for_shutdown()

    # Starting the test machine again with diskImage=null creates a fresh root.
    machine.start()
    machine.wait_for_unit("multi-user.target")
    second_boot = machine.succeed("cat /proc/sys/kernel/random/boot_id").strip()
    assert second_boot != first_boot, "VM boot identity was unexpectedly reused"
    machine.succeed("test ! -e /var/lib/jit-1b-persistence-sentinel")
    machine.succeed("test ! -e /run/jit-1b")
    assert_no_host_exchange()
    prepare_subject_state()
    machine.succeed("test ! -e /run/jit-1b/consumed")

    second_result = run_subject("jit-fixture-second")
    print("JIT1B_EXTERNAL_LOG_SECOND_BEGIN")
    print(second_result)
    print("JIT1B_EXTERNAL_LOG_SECOND_END")
    machine.succeed("grep -Fx JIT-1B_INERT_FIXTURE_PASS /run/jit-1b/result")
    assert_no_host_exchange()

    machine.shutdown()
    machine.wait_for_shutdown()
  '';
}
