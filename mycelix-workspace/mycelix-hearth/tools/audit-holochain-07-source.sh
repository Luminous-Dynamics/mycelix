#!/usr/bin/env bash
# HEARTH-0.7 source migration audit.
# Deterministic, read-only: exits non-zero on any known 0.6 API/config residue.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

fail=0
check_absent() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" --       'mycelix-workspace/mycelix-hearth/**/*.rs'       'mycelix-workspace/mycelix-hearth/**/*.ts'       'mycelix-workspace/mycelix-hearth/**/*.tsx'       'mycelix-workspace/mycelix-hearth/**/*.js'       'mycelix-workspace/mycelix-hearth/**/*.json'       'mycelix-workspace/mycelix-hearth/**/*.toml'       'mycelix-workspace/mycelix-hearth/**/*.yaml'       'mycelix-workspace/mycelix-hearth/**/*.yml'       'mycelix-workspace/mycelix-hearth/**/*.nix'       2>/dev/null; then
    echo "FAIL: $label"
    fail=1
  else
    echo "OK:   $label"
  fi
}

check_present() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" --       'mycelix-workspace/mycelix-hearth/**/*.toml'       'mycelix-workspace/mycelix-hearth/**/*.nix'       'mycelix-workspace/mycelix-hearth/**/*.json'       'mycelix-workspace/mycelix-hearth/**/*.yaml'       'mycelix-workspace/mycelix-hearth/**/*.yml'; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

check_present_any() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" --       'mycelix-workspace/mycelix-hearth/**/*.rs'       'mycelix-workspace/mycelix-hearth/**/*.ts'       'mycelix-workspace/mycelix-hearth/**/*.tsx'       'mycelix-workspace/mycelix-hearth/**/*.js'       'mycelix-workspace/mycelix-hearth/**/*.json'       'mycelix-workspace/mycelix-hearth/**/*.toml'       'mycelix-workspace/mycelix-hearth/**/*.yaml'       'mycelix-workspace/mycelix-hearth/**/*.yml'       'mycelix-workspace/mycelix-hearth/**/*.nix' >/dev/null 2>&1; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

check_lock_present() {
  local label="$1"
  local pattern="$2"
  if rg -n --pcre2 "$pattern" mycelix-workspace/mycelix-hearth/flake.lock >/dev/null; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

check_absent_multiline() {
  local label="$1"
  local pattern="$2"
  if rg -nU --pcre2 "$pattern" mycelix-workspace/mycelix-hearth; then
    echo "FAIL: $label"
    fail=1
  else
    echo "OK:   $label"
  fi
}

echo "HEARTH-0.7 deterministic source audit"
echo "HEAD: $(git rev-parse HEAD)"
echo

# Legacy Rust action model / removed APIs.
check_absent "legacy FlatOp variants" 'FlatOp::(StoreEntry|StoreRecord|RegisterUpdate|RegisterDelete|RegisterCreateLink|RegisterDeleteLink|RegisterAgentActivity)'
check_absent "legacy Action enum variants" 'Action::(Create|Update|Delete|CreateLink|DeleteLink|Dna|AgentValidationPkg|InitZomesComplete|OpenChain|CloseChain)([({])'
check_absent "removed action builders" '\b(ActionBuilder|ActionBuilderCommon|NewEntryAction|NewEntryActionRef)\b'
check_absent "removed EntryCreationAction" '\bEntryCreationAction\b'
check_absent "removed agent blocking APIs" '\b(block_agent|unblock_agent)\s*\('
check_absent "old generic SignedActionHashed" 'SignedActionHashed\s*<'
check_absent "old transport/config symbols" '\b(signal_url|webrtc_config|transport-iroh|wasmer_sys|sqlite-encrypted)\b'
check_absent "legacy serialized-bytes pin" 'holochain_serialized_bytes[^0-9]*0\.0\.56'
check_absent "legacy SweetConductor constructor" '\bSweetConductor::from_standard_config\s*\('

# Old client/test package names and 0.6 version declarations.
check_absent "legacy Tryorama package" '@holochain/tryorama'
check_absent "legacy Holochain 0.6 package versions" '(@holochain/client[^0-9]*0\.20\.|hdk[^0-9]*0\.6\.|hdi[^0-9]*0\.7\.|holochain[^0-9]*0\.6\.)'

# Subtle 0.7 source/API changes that can compile incorrectly or regress at runtime.
check_absent "legacy DnaStorageInfo size fields" '\b(authored_data_size|cache_data_size)\b'
check_absent "legacy client WebRTC predicate" '\bis_webrtc\b'
check_absent "legacy client signaling-server field" '\bsignalingServerUrl\b'
check_absent "legacy config sync strategy field" '\bdb_sync_strategy\b'
check_absent "legacy 0.6 sandbox transport spelling" '\bwebrtc\b'
check_absent_multiline "removed ChainFilter builder methods" 'ChainFilter::new\([^)]*\)[[:space:]]*\.[[:space:]]*(until_hash|take|until_timestamp)[[:space:]]*\('

# 0.7 dependency floor must be visible in Hearth manifests.
check_present "Hearth HDK 0.7.0 dependency floor" 'hdk\s*=\s*"=0\.7\.0"'
check_present "Hearth HDI 0.8.0 dependency floor" 'hdi\s*=\s*"=0\.8\.0"'
check_present "Hearth serialized-bytes 0.0.57 floor" 'holochain_serialized_bytes\s*=\s*"0\.0\.57"'
check_present "Hearth Holochain 0.7.0 test dependency" 'holochain\s*=.*version\s*=\s*"0\.7\.0"'
check_present_any "Hearth JS client 0.21 floor" '@holochain/client.*\^0\.21\.0'
check_present "Hearth Sweettest uses encryption feature" 'holochain.*features.*encryption'
check_present "Hearth Sweettest uses wasmer-sys-cranelift" 'holochain.*features.*wasmer-sys-cranelift'
check_present_any "Hearth uses SweetConductor::standard" 'SweetConductor::standard\s*\('
check_present "Hearth dev shell provides Node.js 24" 'nodejs_24'
check_present "Hearth dev shell provides Perl" '\bperl\b'
check_present "Hearth flake uses Holonix main-0.7" 'holonix.*ref=main-0\.7'
check_lock_present "flake.lock pins Holochain 0.7.0" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"holochain-0\.7\.0"'
check_lock_present "flake.lock pins Kitsune2 0.5.0" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"v0\.5\.0"'
check_lock_present "flake.lock pins Lair 0.7.1" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"v0\.7\.1"'
check_lock_present "flake.lock pins Holonix main-0.7" '"original"[[:space:]]*:[[:space:]]*\{[[:space:]]*"owner"[[:space:]]*:[[:space:]]*"holochain"[[:space:]]*,[[:space:]]*"ref"[[:space:]]*:[[:space:]]*"main-0\.7"'

# Coordinator/client action access must use the 0.7 header/data split where action
# content is inspected. This is intentionally a presence audit, not a style gate.
if git grep -nE -- '\bActionData::|\.hashed\.content\.header\.(author|timestamp)|\.hashed\.content\.data' --     'mycelix-workspace/mycelix-hearth/**/*.rs'     'mycelix-workspace/mycelix-hearth/**/*.ts'     'mycelix-workspace/mycelix-hearth/**/*.tsx' >/dev/null 2>&1; then
  echo "OK:   0.7 action header/data access present"
else
  echo "WARN: no explicit ActionData/header access found in Hearth source"
fi


# Semantic 0.7 source-chain validation coverage.
if git grep -nE -- '\bmust_get_agent_activity\s*\(' -- 'mycelix-workspace/mycelix-hearth/**/*.rs' >/dev/null 2>&1; then
  check_present_any "0.7 activity response: UntilHashMissing" 'MustGetAgentActivityResponse::UntilHashMissing'
  check_present_any "0.7 activity response: UntilHashAfterChainHead" 'MustGetAgentActivityResponse::UntilHashAfterChainHead'
  check_present_any "0.7 activity response: UntilTimestampIndeterminate" 'MustGetAgentActivityResponse::UntilTimestampIndeterminate'
  check_present_any "0.7 activity response: UntilTimestampGreaterThanChainHead" 'MustGetAgentActivityResponse::UntilTimestampGreaterThanChainHead'
  check_present_any "0.7 activity response: IncompleteChain" 'MustGetAgentActivityResponse::IncompleteChain'
else
  echo "OK:   no must_get_agent_activity call sites require response coverage"
fi

# Every tracked integrity zome must expose the 0.7 validation seam, flatten
# operations through FlatOp, and inspect action semantics explicitly.
integrity_files=()
while IFS= read -r -d '' file; do integrity_files+=("$file"); done < <(git ls-files -z 'mycelix-workspace/mycelix-hearth/zomes/*/integrity/src/lib.rs')
if ((${#integrity_files[@]} == 0)); then
  echo "FAIL: no tracked Hearth integrity zomes discovered for semantic coverage"
  fail=1
else
  for file in "${integrity_files[@]}"; do
    if rg -n --pcre2 '\bfn\s+validate\s*\(\s*(?:op\s*:\s*)?Op\b|\bvalidate\s*\(\s*op\s*:\s*Op\b' "$file" >/dev/null; then
      echo "OK:   $file exposes validate(Op)"
    else
      echo "FAIL: $file missing validate(Op) semantic seam"; fail=1
    fi
    if rg -n --pcre2 '\bFlatOp::|flattened\s*<[^>]*>\s*\(' "$file" >/dev/null; then
      echo "OK:   $file handles flattened 0.7 operations"
    else
      echo "FAIL: $file missing FlatOp/flattened 0.7 operation handling"; fail=1
    fi
    if rg -n --pcre2 'ActionData::|action\.(?:author|timestamp)\s*\(|\.header\.(?:author|timestamp)\b' "$file" >/dev/null; then
      echo "OK:   $file inspects 0.7 action semantics"
    else
      echo "FAIL: $file missing explicit 0.7 action semantic access"; fail=1
    fi
# 0.7 validation must account for every FlatOp family. A zome may
    # explicitly handle a family or intentionally cover it with a terminal
    # catch-all, but silently dropping a family is a migration defect.
    for family in CreateEntry CreateRecord Update Delete Link AgentActivity; do
      if rg -n --pcre2 "\bFlatOp::${family}\b|\b_\s*=>\s*Ok\s*\(" "$file" >/dev/null; then
        echo "OK:   $file covers FlatOp::$family (explicit or catch-all)"
      else
        echo "FAIL: $file has no FlatOp::$family or terminal catch-all coverage"; fail=1
      fi
    done
  done
fi

    # Validation must remain deterministic. Holochain explicitly disallows
# state-changing / time-varying retrievals and other non-deterministic inputs
# from validation callbacks. Keep this gate scoped to production source before
# the test module so test-only helpers do not create false positives.
check_validation_determinism() {
  local file="$1"
  local source
  source="$(sed '/^\#\[cfg(test)\]/,$d' "$file")"
  if printf '%s\n' "$source" | rg -n --pcre2 '\b(get_links|get_details|get_agent_activity|sys_time|random_bytes|call)\s*\(' >/tmp/hearth07_validation_forbidden.$$ 2>/dev/null; then
    echo "FAIL: $file contains non-deterministic validation host API usage"
    cat /tmp/hearth07_validation_forbidden.$$
    fail=1
  fi
  if printf '%s\n' "$source" | rg -n --pcre2 '\b(?:SystemTime|Instant|thread_rng|random::<|rand::|getrandom::)\b' >/tmp/hearth07_validation_random.$$ 2>/dev/null; then
    echo "FAIL: $file contains non-deterministic time/random source in validation production code"
    cat /tmp/hearth07_validation_random.$$
    fail=1
  fi
  rm -f /tmp/hearth07_validation_forbidden.$$ /tmp/hearth07_validation_random.$$
  echo "OK:   $file validation determinism surface"
}

# Link deletion is its own 0.7 operation family. Require an explicit
# DeleteLink arm and an author comparison between the deleting action and the
# original CreateLink action. A terminal catch-all must never silently accept
# link deletion without this authorization invariant.
check_delete_link_authorization() {
  local file="$1"
  if ! rg -n --pcre2 'FlatOp::Link\s*\([^)]*OpLink::DeleteLink' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file has no explicit FlatOp::Link(OpLink::DeleteLink) coverage"
    fail=1
    return
  fi
  if rg -n --pcre2 '(check_link_author_match|original_record\.action\(\)\.author\(\)|original_action\.author\(\)|original_action\\(\\).*author)' "$file" >/dev/null 2>&1; then
    echo "OK:   $file DeleteLink authorization compares original and deleting authors"
  else
    echo "FAIL: $file DeleteLink path lacks an explicit original/deleting author comparison"
    fail=1
  fi
  if rg -n --pcre2 'FlatOp::Link\s*\(\s*link\s*@\s*OpLink::DeleteLink[\s\S]{0,1400}must_get_valid_record\s*\(\s*action\.link_add_address' "$file" >/dev/null 2>&1; then
    echo "OK:   $file DeleteLink retrieves the original CreateLink through must_get_valid_record"
  else
    echo "FAIL: $file DeleteLink path does not establish original CreateLink validity with must_get_valid_record"
    fail=1
  fi
}

# Action-family authorization must not be accidentally absorbed by a terminal
# catch-all. If a zome validates entry updates at CreateEntry(UpdateEntry), it
# must also expose an explicit Update(OpUpdate::Entry) path for action-level
# authorization. Holochain validation can receive these operations separately.
check_update_action_coverage() {
  local file="$1"
  if rg -n --pcre2 'FlatOp::CreateEntry\s*\(\s*OpEntry::UpdateEntry' "$file" >/dev/null 2>&1; then
    if rg -n --pcre2 'FlatOp::Update\s*\(\s*OpUpdate::Entry' "$file" >/dev/null 2>&1; then
      echo "OK:   $file has explicit FlatOp::Update(OpUpdate::Entry) coverage"
    else
      echo "FAIL: $file validates UpdateEntry data but has no explicit Update action coverage"
      fail=1
    fi
  else
    echo "OK:   $file has no CreateEntry(UpdateEntry) path requiring paired Update coverage"
  fi
}

# Every declared EntryTypes variant must have at least one explicit validation dispatch reference.
check_entry_type_dispatch() {
  local file="$1"
  local source enum_block variant
  source="$(sed '/^\#\[cfg(test)\]/,$d' "$file")"
  enum_block="$(printf '%s\n' "$source" | sed -n '/^pub enum EntryTypes[[:space:]]*{/,/^}/p')"
  if [[ -z "$enum_block" ]]; then
    echo "FAIL: $file has no parseable EntryTypes enum"
    fail=1
    return
  fi
  while IFS= read -r variant; do
    [[ -z "$variant" ]] && continue
    if printf '%s\n' "$source" | rg -n --pcre2 "\\bEntryTypes::${variant}\\b" >/dev/null 2>&1; then
      echo "OK:   $file EntryTypes::$variant has validation dispatch coverage"
    else
      echo "FAIL: $file EntryTypes::$variant has no validation dispatch reference"
      fail=1
    fi
  done < <(printf '%s\n' "$enum_block" | rg --pcre2 -o '^\\s*[A-Za-z_][A-Za-z0-9_]*\\s*\\(' | sed -E 's/^\\s*([A-Za-z_][A-Za-z0-9_]*).*$/\\1/')
}

# Link validation must receive the typed base/target addresses and must dispatch
# on every declared LinkTypes variant inside the CreateLink policy itself. A
# reference elsewhere in tests or coordinators does not establish validation coverage.
check_link_type_policy() {
  local file="$1"
  local source enum_block policy_block variant
  source="$(sed '/^#\[cfg(test)\]/,$d' "$file")"
  enum_block="$(printf '%s\n' "$source" | sed -n '/^pub enum LinkTypes[[:space:]]*{/,/^}/p')"
  if [[ -z "$enum_block" ]]; then
    echo "FAIL: $file has no parseable LinkTypes enum"
    fail=1
    return
  fi
  if ! printf '%s\n' "$source" | grep -Fq 'FlatOp::Link(OpLink::CreateLink { link_type, action })'; then
    echo "FAIL: $file CreateLink validation does not bind link_type and action"
    fail=1
    return
  fi
  if ! printf '%s\n' "$source" | grep -Fq 'action.data.base_address' || ! printf '%s\n' "$source" | grep -Fq 'action.data.target_address'; then
    echo "FAIL: $file CreateLink validation does not inspect both base_address and target_address"
    fail=1
    return
  fi
  policy_block="$(printf '%s\n' "$source" | awk '
    /^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+validate_create_link[[:space:]]*\(/ {
      in_block=1
      print
      next
    }
    in_block && /^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+validate_[A-Za-z0-9_]+[[:space:]]*\(/ {
      exit
    }
    in_block { print }
  ')"
  if [[ -z "$policy_block" ]] || ! printf '%s\n' "$policy_block" | grep -Fq 'match link_type {'; then
    echo "FAIL: $file lacks an explicit LinkTypes match in its CreateLink validation policy"
    fail=1
    return
  fi
  if printf '%s\n' "$policy_block" | grep -Eq '^[[:space:]]*_[[:space:]]*=>'; then
    echo "FAIL: $file has a wildcard arm in its LinkTypes CreateLink policy"
    fail=1
  fi
  while IFS= read -r variant; do
    [[ -z "$variant" ]] && continue
    if printf '%s\n' "$policy_block" | grep -Fq "LinkTypes::$variant"; then
      echo "OK:   $file LinkTypes::$variant has CreateLink policy coverage"
    else
      echo "FAIL: $file LinkTypes::$variant is not handled in CreateLink validation policy"
      fail=1
    fi
  done < <(printf '%s\n' "$enum_block" | grep -E '^    [A-Za-z_][A-Za-z0-9_]*,$' | sed -E 's/^    ([A-Za-z_][A-Za-z0-9_]*),$/\1/')
}

# Link tags are application data. Hearth coordinators use empty tags for ordinary
# relationship/index links; only explicitly payload-bearing link types are allowed
# to opt out. This keeps arbitrary tag data from becoming an unvalidated shadow
# channel while preserving the two intentional tagged-link designs.
check_link_tag_contract() {
  local file="$1"
  if [[ "$file" == *"/hearth-bridge/"* ]]; then
    if rg -n --pcre2 '!matches!\(link_type,\s*LinkTypes::DispatchRateLimit\s*\|\s*LinkTypes::NotificationSubscription\)' "$file" >/dev/null 2>&1; then
      echo "OK:   $file constrains Bridge link tags except intentional dispatch-rate-limit/subscription cases"
    else
      echo "FAIL: $file missing Bridge empty-tag contract"
      fail=1
    fi
  elif [[ "$file" == *"/hearth-stories/"* ]]; then
    if rg -n --pcre2 '!matches!\(link_type,\s*LinkTypes::TagToStories\)' "$file" >/dev/null 2>&1; then
      echo "OK:   $file constrains Stories link tags except TagToStories"
    else
      echo "FAIL: $file missing Stories empty-tag contract"
      fail=1
    fi
  else
    if rg -n --pcre2 'if !tag\.0\.is_empty\(\)' "$file" >/dev/null 2>&1; then
      echo "OK:   $file constrains ordinary LinkTypes to empty tags"
    else
      echo "FAIL: $file missing ordinary empty-tag contract"
      fail=1
    fi
  fi
}

# Holochain emits CreateEntry and CreateRecord operations for entry writes; a

# permissive CreateRecord catch-all would leave a second validation surface
# without the application-level entry policy. Updates are checked the same way.
check_create_record_coverage() {
  local file="$1"
  if ! rg -n --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::CreateEntry' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file has no explicit FlatOp::CreateRecord(OpRecord::CreateEntry) validation"
    fail=1
  else
    echo "OK:   $file explicitly validates CreateRecord entry creation"
  fi
  if rg -n --pcre2 'FlatOp::CreateEntry\s*\(\s*OpEntry::UpdateEntry' "$file" >/dev/null 2>&1; then
    if rg -n --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::UpdateEntry' "$file" >/dev/null 2>&1; then
      echo "OK:   $file explicitly validates CreateRecord update data"
    else
      echo "FAIL: $file has UpdateEntry validation but no CreateRecord update validation"
      fail=1
    fi
  fi
  if rg -n --pcre2 'FlatOp::CreateRecord\s*\(\)\s*=>\s*Ok\s*\(\s*ValidateCallbackResult::Valid' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file has permissive CreateRecord(_) => Valid catch-all"
    fail=1
  fi
}

# CreateRecord dispatch must preserve the same EntryTypes policy as CreateEntry.
# This catches a subtler regression than merely requiring the operation arm:
# a new variant could be added to CreateEntry while silently falling through
# the corresponding CreateRecord branch.
check_create_record_entry_dispatch() {
  local file="$1"
  local source enum_block variant create_block update_block
  source="$(sed '/^\#\[cfg(test)\]/,$d' "$file")"
  enum_block="$(printf '%s\n' "$source" | sed -n '/^pub enum EntryTypes[[:space:]]*{/,/^}/p')"
  create_block="$(printf '%s\n' "$source" | awk '/FlatOp::CreateRecord\\(OpRecord::CreateEntry/{in_block=1} /FlatOp::CreateRecord\\(OpRecord::UpdateEntry/{in_block=0} in_block')"
  update_block="$(printf '%s\n' "$source" | awk '/FlatOp::CreateRecord\\(OpRecord::UpdateEntry/{in_block=1} /FlatOp::Link/{if(in_block){in_block=0}} in_block')"
  while IFS= read -r variant; do
    [[ -z "$variant" ]] && continue
    if printf '%s\n' "$create_block" | rg -n --pcre2 "\\bEntryTypes::${variant}\\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateRecord create dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateRecord create dispatch misses EntryTypes::$variant"
      fail=1
    fi
    if printf '%s\n' "$update_block" | rg -n --pcre2 "\\bEntryTypes::${variant}\\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateRecord update dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateRecord update dispatch misses EntryTypes::$variant"
      fail=1
    fi
  done < <(printf '%s\n' "$enum_block" | rg --pcre2 -o '^\\s*[A-Za-z_][A-Za-z0-9_]*\\s*\\(' | sed -E 's/^\\s*([A-Za-z_][A-Za-z0-9_]*).*$/\\1/')
}
# Dangerous operation families must never be accepted solely by a terminal
# wildcard. Delete and Link carry authorization/state semantics of their own;
# CreateRecord is guarded above, and Update is paired with explicit action-level
# authorization by check_update_action_coverage().
check_dangerous_operation_catchalls() {
  local file="$1"
  for family in Delete Link; do
    if rg -n --pcre2 "FlatOp::${family}\s*\(\s*_\s*\)\s*=>\s*Ok\s*\(\s*ValidateCallbackResult::Valid" "$file" >/dev/null 2>&1; then
      echo "FAIL: $file has permissive FlatOp::$family(_) => Valid catch-all"
      fail=1
    else
      echo "OK:   $file has no permissive FlatOp::$family(_) => Valid catch-all"
    fi
  done
}
# Runtime semantic-validation qualification must remain wired into the test crate.
# This is a structural gate only: the ignored Sweettests still provide the actual
# runtime evidence once executed in the pinned conductor environment.
check_semantic_validation_suite_wiring() {
  local tests_root="mycelix-workspace/mycelix-hearth/tests"
  local manifest="$tests_root/hearth-07-semantic-validation-cases.json"
  local rust_test="$tests_root/sweettest_semantic_validation.rs"
  local cargo_manifest="$tests_root/Cargo.toml"

  for required in "$manifest" "$rust_test" "$cargo_manifest"; do
    if [[ ! -f "$required" ]]; then
      echo "FAIL: missing Hearth semantic-validation qualification file: $required"
      fail=1
    fi
  done

  if [[ -f "$manifest" ]]; then
    check_present_any "Hearth semantic-validation case schema" '"schema_version"[[:space:]]*:[[:space:]]*"HEARTH-SEMANTIC-0.7-CASESET-1"'
    check_present_any "Hearth semantic-validation runtime claim ceiling" 'RuntimeQualificationPending'
    for case_id in SEM-01 SEM-02 SEM-03; do
      if rg -n --fixed-strings "\"case_id\": \"$case_id\"" "$manifest" >/dev/null 2>&1; then
        echo "OK:   semantic-validation manifest contains $case_id"
      else
        echo "FAIL: semantic-validation manifest missing $case_id"
        fail=1
      fi
    done
  fi

  if [[ -f "$cargo_manifest" ]]; then
    if rg -n --pcre2 'name[[:space:]]*=[[:space:]]*"sweettest_semantic_validation"' "$cargo_manifest" >/dev/null 2>&1; then
      echo "OK:   semantic-validation Sweettest is registered in tests/Cargo.toml"
    else
      echo "FAIL: semantic-validation Sweettest is not registered in tests/Cargo.toml"
      fail=1
    fi
  fi

  if [[ -f "$rust_test" ]]; then
    for test_name in \
      test_invalid_decision_entry_reaches_integrity_validation \
      test_invalid_resource_entry_reaches_integrity_validation \
      test_invalid_story_entry_reaches_integrity_validation; do
      if rg -n --pcre2 "\basync[[:space:]]+fn[[:space:]]+$test_name\b" "$rust_test" >/dev/null 2>&1; then
        echo "OK:   semantic-validation test is present: $test_name"
      else
        echo "FAIL: semantic-validation test is missing: $test_name"
        fail=1
      fi
    done
  fi
}

# Dependency retrieval semantics: must_get_action only proves retrieval; it does not prove
# that the referenced record passed application validation. Update/delete authorization
# therefore uses must_get_valid_record before trusting the referenced author. Valid-record
# consumers must also inspect the referenced entry/action rather than treating retrieval
# itself as the invariant.
check_dependency_semantics() {
  local file="$1"
  if rg -n --pcre2 'must_get_action\(action\.(?:original_action_address|deletes_address)' "$file" >/tmp/hearth07_weak_dependency.$ 2>/dev/null; then
    echo "FAIL: $file uses must_get_action for update/delete authorization"
    cat /tmp/hearth07_weak_dependency.$
    fail=1
  fi
  if rg -n --pcre2 'must_get_valid_record\(' "$file" >/dev/null 2>&1; then
    if rg -n --pcre2 '\.(?:entry\(\)\.to_app_option|action\(\))|try_from_action' "$file" >/dev/null 2>&1; then
      echo "OK:   $file valid-record dependencies are semantically inspected"
    else
      echo "FAIL: $file retrieves a valid record without inspecting its entry/action"
      fail=1
    fi
  else
    echo "OK:   $file has no must_get_valid_record dependency sites"
  fi
}

# Immutable-field helpers must prove the referenced CreateRecord is valid and
# deserialize the original entry before comparing fields. This guards against a
# future helper that retrieves a record but accidentally treats retrieval as proof.
check_immutable_dependency_semantics() {
  local file="$1"
  local helper_count
  helper_count="$(rg -n --pcre2 '^\s*(?:pub\s+)?fn\s+validate_[A-Za-z0-9_]*immutable_fields\s*\(' "$file" | wc -l)"
  if [[ "$helper_count" -eq 0 ]]; then
    echo "OK:   $file has no immutable-field helper sites"
    return
  fi
  if ! rg -n --pcre2 'validate_[A-Za-z0-9_]*immutable_fields\s*\(' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file declares immutable-field helpers but no call site was found"
    fail=1
  fi
  if ! rg -n --pcre2 'must_get_valid_record\(' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file immutable-field helpers do not use must_get_valid_record"
    fail=1
  fi
  if ! rg -n --pcre2 '\.entry\(\)\s*\.to_app_option\(\)' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file immutable-field helpers do not deserialize the original entry"
    fail=1
  fi
  echo "OK:   $file immutable-field dependency semantics"
}


check_standalone_tests_workspace_boundary() {
  local manifest="mycelix-workspace/mycelix-hearth/tests/Cargo.toml"
  if [[ -f "$manifest" ]] && rg -n --fixed-strings "[workspace]" "$manifest" >/dev/null 2>&1; then
    echo "OK:   Hearth integration tests declare their standalone Cargo workspace boundary"
  else
    echo "FAIL: Hearth integration tests must declare an explicit standalone Cargo workspace boundary"
    fail=1
  fi
}

check_qualification_workflow_provenance() {
  local workflow=".github/workflows/hearth-07-qualification.yml"
  if [[ ! -f "$workflow" ]]; then
    echo "FAIL: missing Hearth 0.7 qualification workflow"
    fail=1
    return
  fi
  if rg -n --fixed-strings 'target_sha:' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'ref: ${{ env.QUALIFY_SHA }}' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'git rev-parse HEAD' "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow binds execution to an exact candidate SHA"
  else
    echo "FAIL: qualification workflow does not enforce exact candidate-SHA checkout provenance"
    fail=1
  fi
  if rg -n --fixed-strings 'cargo build --locked' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'cargo test --locked' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'cargo generate-lockfile' "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow generates and consumes locked Rust closures"
  else
    echo "FAIL: qualification workflow is missing locked Rust dependency closure enforcement"
    fail=1
  fi
}

check_dna_source_completeness() {
  local dna="mycelix-workspace/mycelix-hearth/dna/dna.yaml"
  if [[ ! -f "$dna" ]]; then
    echo "FAIL: missing Hearth DNA manifest"
    fail=1
    return
  fi
  local expected_names=()
  while IFS= read -r -d '' file; do
    local dir name
    dir="$(basename "$(dirname "$(dirname "$(dirname "$file")")")")"
    name="${dir//-/_}_integrity"
    expected_names+=("$name")
  done < <(git ls-files -z 'mycelix-workspace/mycelix-hearth/zomes/*/integrity/src/lib.rs')
  for name in "${expected_names[@]}"; do
    if rg -n --fixed-strings -- "- name: $name" "$dna" >/dev/null 2>&1; then
      echo "OK:   DNA packages discovered integrity zome $name"
    else
      echo "FAIL: DNA is missing discovered integrity zome $name"
      fail=1
    fi
  done
  local integrity_count
  integrity_count="$(rg -n --fixed-strings -- '- name:' "$dna" | rg 'hearth_.*_integrity
for file in "${integrity_files[@]}"; do
  check_create_record_coverage "$file"
done

for file in "${integrity_files[@]}"; do
  check_dangerous_operation_catchalls "$file"
done

for file in "${integrity_files[@]}"; do
  check_create_record_entry_dispatch "$file"
done
for file in "${integrity_files[@]}"; do
  check_dependency_semantics "$file"
done

for file in "${integrity_files[@]}"; do
  check_update_action_coverage "$file"
done

for file in "${integrity_files[@]}"; do
  check_entry_type_dispatch "$file"
done

for file in "${integrity_files[@]}"; do
  check_delete_link_authorization "$file"
done

for file in "${integrity_files[@]}"; do
  check_link_type_policy "$file"
done

for file in "${integrity_files[@]}"; do
  check_link_tag_contract "$file"
done

for file in "${integrity_files[@]}"; do
  check_immutable_dependency_semantics "$file"
done

for file in "${integrity_files[@]}"; do
  check_validation_determinism "$file"
done

echo
if [[ "$fail" -ne 0 ]]; then
  echo "HEARTH-0.7 source audit: FAIL"
  exit "$fail"
fi
echo "HEARTH-0.7 source audit: PASS"
