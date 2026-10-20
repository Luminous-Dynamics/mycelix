# Independent JSON-level policy-profile verifier.
# Does not import, invoke, or parse the Rust policy implementation.
def difference($left; $right):
  [$left[] as $item | select(($right | index($item)) == null) | $item];

def expected_violations($request; $profile; $pinned):
  if ($pinned | not) then
    ["policy_profile_digest_mismatch"]
  else
    [
      if (difference($request.candidate_ceiling.claim_atoms; $profile.maximum_ceiling.claim_atoms) | length) > 0
        then "claim_atom_not_allowed" else empty end,
      if (difference($request.candidate_ceiling.subject_ids; $profile.maximum_ceiling.subject_ids) | length) > 0
        then "subject_outside_policy" else empty end,
      if (difference($request.candidate_ceiling.context_ids; $profile.maximum_ceiling.context_ids) | length) > 0
        then "context_outside_policy" else empty end,
      if (
        $request.candidate_ceiling.valid_time.start_unix_seconds < $profile.maximum_ceiling.valid_time.start_unix_seconds
        or $request.candidate_ceiling.valid_time.end_unix_seconds_exclusive > $profile.maximum_ceiling.valid_time.end_unix_seconds_exclusive
      ) then "time_window_outside_policy" else empty end,
      if (difference($request.candidate_ceiling.action_scopes; $profile.maximum_ceiling.action_scopes) | length) > 0
        then "action_scope_not_allowed" else empty end,
      if (difference($profile.maximum_ceiling.required_qualifications; $request.candidate_ceiling.required_qualifications) | length) > 0
        then "required_qualification_missing" else empty end,
      if (difference($profile.maximum_ceiling.required_independence_controls; $request.candidate_ceiling.required_independence_controls) | length) > 0
        then "required_independence_control_missing" else empty end,
      if (difference($profile.maximum_ceiling.required_provenance; $request.candidate_ceiling.required_provenance) | length) > 0
        then "required_provenance_missing" else empty end
    ] | unique
  end;

. as $receipt
| $request[0] as $req
| $profile[0] as $approved_profile
| ($actual_policy_sha256 == $expected_policy_sha256) as $pinned
| expected_violations($req; $approved_profile; $pinned) as $codes
| (
    $receipt.schema_version == "mycelix.claim-ceiling-policy.receipt.v1"
    and $receipt.request_id == $req.request_id
    and $receipt.profile_id == $approved_profile.profile_id
    and $receipt.source_commit == $source
    and $receipt.source_commit_pinned == ($source != "UNPINNED_LOCAL_RUN")
    and $receipt.input_sha256 == $input_sha256
    and $receipt.policy_profile_sha256 == $actual_policy_sha256
    and $receipt.expected_policy_profile_sha256 == $expected_policy_sha256
    and $receipt.policy_profile_pinned == $pinned
    and $receipt.candidate_ceiling_sha256 == $candidate_sha256
    and $receipt.request_schema_sha256 == $request_schema_sha256
    and $receipt.policy_profile_schema_sha256 == $policy_profile_schema_sha256
    and $receipt.receipt_schema_sha256 == $receipt_schema_sha256
    and $receipt.disposition == (if ($codes | length) == 0 then "admissible_within_profile" else "rejected_by_profile" end)
    and (($receipt.violations | map(.code) | unique | sort) == ($codes | sort))
    and $receipt.policy_admissibility_only == true
    and $receipt.action_authority_granted == false
    and ($receipt.nonclaims | length) > 0
  )
