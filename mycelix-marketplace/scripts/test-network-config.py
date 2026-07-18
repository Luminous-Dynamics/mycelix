#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "scripts/generate-networked-conductor-config.py"
VALIDATOR = ROOT / "scripts/validate-network-services-receipt.py"

with tempfile.TemporaryDirectory() as raw:
    temp = Path(raw)
    output = temp / "conductor.yaml"
    subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--data-root",
            str(temp / "data"),
            "--admin-port",
            "12345",
            "--bootstrap-url",
            "http://127.0.0.1:7000",
            "--signal-url",
            "ws://127.0.0.1:7001",
            "--output",
            str(output),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    rendered = output.read_text()
    assert "bootstrap_url: \"http://127.0.0.1:7000\"" in rendered
    assert "signal_url: \"ws://127.0.0.1:7001\"" in rendered
    rejected = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--data-root",
            str(temp / "public"),
            "--admin-port",
            "12346",
            "--bootstrap-url",
            "https://example.com",
            "--signal-url",
            "wss://example.com",
            "--output",
            str(temp / "public.yaml"),
        ],
        text=True,
        capture_output=True,
    )
    assert rejected.returncode != 0
    hook = temp / "services-hook"
    hook.write_text("#!/bin/sh\n")
    receipt = temp / "services.json"
    receipt.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "bootstrap_url": "http://127.0.0.1:7000",
                "signal_url": "ws://127.0.0.1:7001",
                "isolation": "local_controlled",
                "implementation_sha256": hashlib.sha256(hook.read_bytes()).hexdigest(),
            }
        )
    )
    subprocess.run(
        [sys.executable, str(VALIDATOR), str(receipt), "--implementation", str(hook)],
        check=True,
        stdout=subprocess.DEVNULL,
    )
print("network conductor configuration tests passed")
