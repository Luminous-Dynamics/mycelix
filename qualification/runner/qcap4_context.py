from __future__ import annotations

from qcap_canon import CapsuleError, H64, canonical_json, exact_keys, hex_value, validate_profile
from qcap3_limits import limits_ref
from qcap4_containment import containment_ref, validate_containment_profile

CTX4_KEYS = {
    "execution_context_format_revision",
    "runner_profile_ref",
    "toolchain_profile_ref",
    "environment_profile_ref",
    "execution_limits_profile_ref",
    "containment_profile_ref",
    "resolved_runner_commitment",
    "resolved_toolchain_commitment",
    "resolved_environment_commitment",
    "resolved_containment_commitment",
}


def validate_execution_context_v4(
    context,
    manifest,
    limits,
    containment_profile,
    expected_runner_commitment=None,
    expected_resolved_containment_commitment=None,
):
    exact_keys(context, CTX4_KEYS, "execution context v4")
    if context["execution_context_format_revision"] != 4:
        raise CapsuleError("unsupported execution context")

    validate_profile(context["runner_profile_ref"], "runner profile")
    validate_profile(context["toolchain_profile_ref"], "execution toolchain")
    validate_profile(context["environment_profile_ref"], "execution environment")
    validate_profile(context["execution_limits_profile_ref"], "execution limits profile")
    validate_profile(context["containment_profile_ref"], "containment profile")
    validate_containment_profile(containment_profile)

    if context["toolchain_profile_ref"] != manifest["toolchain_profile_ref"]:
        raise CapsuleError("toolchain profile mismatch")
    if context["environment_profile_ref"] != manifest["environment_profile_ref"]:
        raise CapsuleError("environment profile mismatch")
    if context["execution_limits_profile_ref"] != limits_ref(limits):
        raise CapsuleError("execution limits profile mismatch")
    if context["containment_profile_ref"] != containment_ref(containment_profile):
        raise CapsuleError("containment profile mismatch")

    for key in (
        "resolved_runner_commitment",
        "resolved_toolchain_commitment",
        "resolved_environment_commitment",
        "resolved_containment_commitment",
    ):
        hex_value(context[key], H64, key)

    if (
        expected_runner_commitment is not None
        and context["resolved_runner_commitment"] != expected_runner_commitment
    ):
        raise CapsuleError("resolved runner commitment mismatch")
    if (
        expected_resolved_containment_commitment is not None
        and context["resolved_containment_commitment"]
        != expected_resolved_containment_commitment
    ):
        raise CapsuleError("resolved containment commitment mismatch")

    canonical_json(context)
    return context
