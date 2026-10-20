# Independent lifecycle-state-machine verifier.
# Input fields are fixture expectations + release/context only; this does not load or call
# the Rust evaluator. SHA-256 for the exact UTF-8 profile_bytes string is supplied by
# the workflow after writing that string without transformation to a temporary file.

def digest_valid:
  type == "string" and test("^[a-f0-9]{64}$");

def status_code($status):
  ({
    invalid_signature: "metadata_signature_invalid",
    unknown_publisher_key: "publisher_key_unknown",
    revoked_publisher_key: "publisher_key_revoked",
    metadata_expired: "metadata_expired",
    metadata_stale: "metadata_stale",
    rollback_detected: "distribution_rollback_detected",
    mix_and_match_detected: "distribution_mix_and_match_detected",
    signature_threshold_not_met: "signature_threshold_not_met"
  }[$status] // empty);

def valid_head_scope($head; $context):
  $head != null
  and $head.policy_id == $context.expected_policy_id
  and $head.audience == $context.expected_audience
  and $head.version > 0
  and ($head.target_profile_sha256 | digest_valid);

def expected_codes($vector; $actual_profile_sha256):
  $vector.release as $release
  | $vector.trusted_context as $context
  | ($context.installed_head // null) as $head
  | [
      status_code($context.distribution_status),
      (if $release.policy_id != $context.expected_policy_id
       then "policy_id_mismatch" else empty end),
      (if $release.audience != $context.expected_audience
       then "policy_audience_mismatch" else empty end),
      (if $release.policy_profile_schema_sha256
            != $context.expected_policy_profile_schema_sha256
       then "policy_schema_digest_mismatch" else empty end),
      (if $release.target_profile_sha256 != $actual_profile_sha256
       then "profile_target_digest_mismatch" else empty end),
      (if $release.issued_at_unix_seconds >= $release.expires_at_unix_seconds
       then "invalid_release_interval" else empty end),
      (if $release.revocation_epoch < $context.minimum_revocation_epoch
       then "revocation_epoch_stale" else empty end),
      (if $context.trusted_time_unix_seconds == null
       then "trusted_time_unavailable" else empty end),
      (if $context.trusted_time_unix_seconds != null
           and $context.trusted_time_unix_seconds < $release.issued_at_unix_seconds
       then "release_not_yet_valid" else empty end),
      (if $context.trusted_time_unix_seconds != null
           and $context.trusted_time_unix_seconds >= $release.expires_at_unix_seconds
       then "release_expired" else empty end),
      (if $head != null and $head.policy_id != $context.expected_policy_id
       then "installed_state_policy_mismatch" else empty end),
      (if $head != null and $head.audience != $context.expected_audience
       then "installed_state_audience_mismatch" else empty end),
      (if $head != null
           and ($head.version == 0 or (($head.target_profile_sha256 | digest_valid) | not))
       then "installed_state_invalid" else empty end),
      (if valid_head_scope($head; $context)
           and $release.revocation_epoch < $head.revocation_epoch
       then "policy_revocation_epoch_rollback" else empty end),
      (if valid_head_scope($head; $context)
           and $release.version < $head.version
       then "policy_version_rollback" else empty end),
      (if valid_head_scope($head; $context)
           and $release.version == $head.version
           and ($release.target_profile_sha256 != $head.target_profile_sha256
                or $release.revocation_epoch != $head.revocation_epoch)
       then "policy_version_equivocation" else empty end)
    ]
  | unique
  | sort;

def expected_disposition($vector; $codes):
  if ($codes | length) > 0 then "rejected"
  elif ($vector.trusted_context.installed_head != null)
       and ($vector.release.version == $vector.trusted_context.installed_head.version)
       and ($vector.release.target_profile_sha256
            == $vector.trusted_context.installed_head.target_profile_sha256)
       and ($vector.release.revocation_epoch
            == $vector.trusted_context.installed_head.revocation_epoch)
       and ($vector.release.policy_id
            == $vector.trusted_context.expected_policy_id)
       and ($vector.release.audience
            == $vector.trusted_context.expected_audience)
    then "already_current"
  else "accepted_new_release"
  end;

$corpus[0] as $fixture
| ($fixture.schema_version == "mycelix.claim-policy-lifecycle.golden.v1"
   and ($fixture.cases | length) == 20) as $corpus_is_frozen
| $fixture.cases[$index] as $vector
| expected_codes($vector; $actual_profile_sha256) as $codes
| expected_disposition($vector; $codes) as $disposition
| (
    $corpus_is_frozen
    and $vector.id != null
    and $vector.expected_disposition == $disposition
    and (($vector.expected_violation_codes | sort) == $codes)
  )
