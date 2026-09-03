# mycelix-accountability-credential-v2

Additive high-assurance release credential for the Mycelix -> Xenia SIF boundary.

It wraps the complete canonical `sif-release-credential-v1` statement and commits one exact non-zero `required_sif_profile_digest`. The v2 credential ID and every authority signature change when that profile changes.

This crate does not replace v1 historical evidence. It exists so high-assurance releases can fail closed when the authorizing policy did not explicitly name the SIF transfer profile Xenia must negotiate.
