# Independent JSON-level implementation of the finite claim-ceiling meet.
# This deliberately does not call or load the Rust implementation.
def intersection($a; $b):
  [$a[] as $x | select(any($b[]; . == $x)) | $x] | unique;
def union($a; $b): ($a + $b) | unique;
def canonical_atoms($xs):
  $xs | sort_by(.predicate, .evidence_class, .derivation, .intended_use);
def canonical_ceiling($c):
  {
    claim_atoms: canonical_atoms($c.claim_atoms),
    subject_ids: ($c.subject_ids | sort),
    context_ids: ($c.context_ids | sort),
    valid_time: {
      start_unix_seconds: $c.valid_time.start_unix_seconds,
      end_unix_seconds_exclusive: $c.valid_time.end_unix_seconds_exclusive
    },
    action_scopes: ($c.action_scopes | sort),
    required_qualifications: ($c.required_qualifications | sort),
    required_independence_controls: ($c.required_independence_controls | sort),
    required_provenance: ($c.required_provenance | sort)
  };
def expected_meet($l; $r):
  {
    claim_atoms: intersection($l.claim_atoms; $r.claim_atoms),
    subject_ids: intersection($l.subject_ids; $r.subject_ids),
    context_ids: intersection($l.context_ids; $r.context_ids),
    valid_time: {
      start_unix_seconds: ([$l.valid_time.start_unix_seconds, $r.valid_time.start_unix_seconds] | max),
      end_unix_seconds_exclusive: ([$l.valid_time.end_unix_seconds_exclusive, $r.valid_time.end_unix_seconds_exclusive] | min)
    },
    action_scopes: intersection($l.action_scopes; $r.action_scopes),
    required_qualifications: union($l.required_qualifications; $r.required_qualifications),
    required_independence_controls: union($l.required_independence_controls; $r.required_independence_controls),
    required_provenance: union($l.required_provenance; $r.required_provenance)
  };
def empty_dimensions($c):
  [
    if ($c.claim_atoms | length) == 0 then "claim_atoms" else empty end,
    if ($c.subject_ids | length) == 0 then "subject_ids" else empty end,
    if ($c.context_ids | length) == 0 then "context_ids" else empty end,
    if $c.valid_time.start_unix_seconds >= $c.valid_time.end_unix_seconds_exclusive then "time_window" else empty end
  ] | sort;

. as $receipt
| $request[0] as $req
| expected_meet($req.left; $req.right) as $expected
| empty_dimensions($expected) as $empty
| (
    $receipt.request_id == $req.request_id
    and $receipt.source_commit == $source
    and $receipt.source_commit_pinned == ($source != "UNPINNED_LOCAL_RUN")
    and $receipt.input_sha256 == $input_sha256
    and $receipt.request_schema_sha256 == $request_schema_sha256
    and $receipt.receipt_schema_sha256 == $receipt_schema_sha256
    and $receipt.claim_ceiling_only == true
    and $receipt.non_amplification.no_broader_than_left == true
    and $receipt.non_amplification.no_broader_than_right == true
    and (
      if ($empty | length) > 0 then
        $receipt.disposition == "bottom_no_common_permitted_claim"
        and $receipt.ceiling == null
        and (($receipt.empty_dimensions | sort) == $empty)
      else
        $receipt.disposition == "usable_intersection"
        and (($receipt.empty_dimensions | sort) == [])
        and (canonical_ceiling($receipt.ceiling) == canonical_ceiling($expected))
      end
    )
  )
