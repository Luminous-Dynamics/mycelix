# Retired Security Kernel source carrier

This file records the former candidate-controlled source-carrier workflow that was used
while the independent qualification root was being assembled.

It has been removed from `.github/workflows/` because the trusted dispatcher is now
implemented as a default-branch `pull_request_target` metadata-only workflow. Leaving a
second candidate-controlled trigger would add an unnecessary suppression/trigger surface.

The retired carrier had:

- `pull_request` as its only trigger;
- `contents: read`;
- `cache-mode: none`;
- one immutable `actions/checkout`;
- no `run:` command.

It was never a qualification authority and its removal does not change the Security Kernel
candidate qualification gates.

The active trust path is:

`pull_request_target S0 -> same-commit reusable S1 -> trusted S2`.
