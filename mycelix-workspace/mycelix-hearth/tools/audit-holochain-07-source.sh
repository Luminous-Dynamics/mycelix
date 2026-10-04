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
  if git grep -nE -- "$pattern" -- mycelix-workspace/mycelix-hearth >/dev/null 2>&1; then
    echo "FAIL: $label"
    fail=1
  else
    echo "OK:   $label"
  fi
}
check_present() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" -- mycelix-workspace/mycelix-hearth >/dev/null 2>&1; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}
check_present_any() {
  local label="$1"
  local pattern="$2"
  if git grep -nE -- "$pattern" -- mycelix-workspace/mycelix-hearth >/dev/null 2>&1; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}
check_present_file() {
  local label="$1"
  local file="$2"
  local pattern="$3"
  if rg -n --pcre2 "$pattern" "$file" >/dev/null 2>&1; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

check_lock_ref() {
  local label="$1" node="$2" owner="$3" repo="$4" ref="$5"
  if python3 - "$node" "$owner" "$repo" "$ref" <<'PY'
import json, sys
from pathlib import Path
node_name, owner, repo, ref = sys.argv[1:]
lock=json.loads(Path("mycelix-workspace/mycelix-hearth/flake.lock").read_text())
node=lock["nodes"].get(node_name,{})
original=node.get("original",{}); locked=node.get("locked",{})
ok=(original.get("owner")==owner and original.get("repo")==repo and original.get("ref")==ref and locked.get("owner")==owner and locked.get("repo")==repo and bool(locked.get("rev")) and bool(locked.get("narHash")))
raise SystemExit(0 if ok else 1)
PY
  then echo "OK:   $label"; else echo "FAIL: $label"; fail=1; fi
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
check_absent_file() {
  local label="$1"
  local file="$2"
  local pattern="$3"
  if rg -n --pcre2 "$pattern" "$file" >/dev/null 2>&1; then
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
check_present_file "Hearth HDK 0.7.0 dependency floor" "mycelix-workspace/mycelix-hearth/Cargo.toml" '^[[:space:]]*hdk[[:space:]]*=[[:space:]]*"=0\.7\.0"'
check_present_file "Hearth HDI 0.8.0 dependency floor" "mycelix-workspace/mycelix-hearth/Cargo.toml" '^[[:space:]]*hdi[[:space:]]*=[[:space:]]*"=0\.8\.0"'
check_present_file "Hearth serialized-bytes 0.0.57 floor" "mycelix-workspace/mycelix-hearth/Cargo.toml" '^[[:space:]]*holochain_serialized_bytes[[:space:]]*=[[:space:]]*"0\.0\.57"'
check_present_file "Hearth Holochain 0.7.0 test dependency" "mycelix-workspace/mycelix-hearth/tests/Cargo.toml" '^[[:space:]]*holochain[[:space:]]*=[[:space:]]*\{[^}]*version[[:space:]]*=[[:space:]]*"0\.7\.0"'
check_present_file "Hearth Sweettest uses encryption feature" "mycelix-workspace/mycelix-hearth/tests/Cargo.toml" '^holochain[[:space:]]*=[[:space:]]*\{[^}]*features[[:space:]]*=.*"encryption"'
check_present_file "Hearth Sweettest uses wasmer-sys-cranelift" "mycelix-workspace/mycelix-hearth/tests/Cargo.toml" '^holochain[[:space:]]*=[[:space:]]*\{[^}]*features[[:space:]]*=.*"wasmer-sys-cranelift"'
check_present_file "Hearth uses SweetConductor::standard" "mycelix-workspace/mycelix-hearth/tests/sweettest_semantic_validation.rs" 'SweetConductor::standard[[:space:]]*\('
check_present_file "Hearth dev shell provides Node.js 24" "mycelix-workspace/mycelix-hearth/flake.nix" 'nodejs_24'
check_present_file "Hearth dev shell provides Perl" "mycelix-workspace/mycelix-hearth/flake.nix" '\bperl\b'
check_present_file "Hearth flake uses Holonix main-0.7" "mycelix-workspace/mycelix-hearth/flake.nix" 'github:holochain/holonix\?ref=main-0\.7'
check_present_file "Hearth package builds use Holonix Rust" "mycelix-workspace/mycelix-hearth/flake.nix" 'nativeBuildInputs[[:space:]]*=[[:space:]]*\[[[:space:]]*holochainPackages\.rust[[:space:]]'
check_absent_file "Hearth package builds do not use shared Rust toolchain" "mycelix-workspace/mycelix-hearth/flake.nix" 'holochainBase\.rustToolchain'
check_present_file "Hearth default shell prepends Holonix Rust" "mycelix-workspace/mycelix-hearth/flake.nix" 'export PATH="\$\{holochainPackages\.rust\}/bin:\$PATH'

check_lock_ref "flake.lock pins Holochain 0.7.0" "holochain" "holochain" "holochain" "holochain-0.7.0"
check_lock_ref "flake.lock pins Kitsune2 0.5.0" "kitsune2" "holochain" "kitsune2" "v0.5.0"
check_lock_ref "flake.lock pins Lair 0.7.1" "lair-keystore" "holochain" "lair" "v0.7.1"
check_lock_ref "flake.lock pins Holonix main-0.7" "holonix" "holochain" "holonix" "main-0.7"

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
  local delete_link_block
  delete_link_block="$(awk '
    /^[[:space:]]*FlatOp::Link[[:space:]]*\([[:space:]]*link[[:space:]]*@[[:space:]]*OpLink::DeleteLink/ {
      in_block=1
      print
      next
    }
    in_block && /^[[:space:]]*FlatOp::/ {
      exit
    }
    in_block { print }
  ' "$file")"

  if [[ -z "$delete_link_block" ]]; then
    echo "FAIL: $file has no explicit FlatOp::Link(OpLink::DeleteLink) coverage"
    fail=1
    return
  fi

  if printf '%s\n' "$delete_link_block" | rg -n --pcre2 'check_link_author_match|original_record\.action\(\)\.author\(\)|original_action\.author\(\)|original_action\\(\\).*author' >/dev/null 2>&1; then
    echo "OK:   $file DeleteLink authorization compares original and deleting authors"
  else
    echo "FAIL: $file DeleteLink path lacks an explicit original/deleting author comparison"
    fail=1
  fi

  if printf '%s\n' "$delete_link_block" | rg -n --pcre2 'must_get_valid_record\s*\(\s*action\.link_add_address' >/dev/null 2>&1; then
    echo "OK:   $file DeleteLink retrieves the original CreateLink through must_get_valid_record"
  else
    echo "FAIL: $file DeleteLink path does not establish original CreateLink validity with must_get_valid_record"
    fail=1
  fi

  if printf '%s\n' "$delete_link_block" | rg -n --fixed-strings 'TypedAction::<CreateLinkData>::try_from_action(original_record.action().clone())?' >/dev/null 2>&1; then
    echo "OK:   $file DeleteLink narrows the referenced action to CreateLinkData"
  else
    echo "FAIL: $file DeleteLink path does not explicitly narrow the referenced action to CreateLinkData"
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

# Update/Delete are distinct DHT operations from CreateEntry/CreateRecord.
# When an integrity zome exposes these paths, require the original record to be
# fetched through must_get_valid_record before applying author authorization.
check_update_delete_authorization() {
  local file="$1"
  if grep -Fq "FlatOp::Update(OpUpdate::Entry { action" "$file"; then
    local update_block
    update_block="$(awk 'index($0, "FlatOp::Update(OpUpdate::Entry") { in_block=1 } index($0, "FlatOp::Update(_)") && in_block { in_block=0 } in_block' "$file")"
    if printf '%s\n' "$update_block" | grep -Fq "must_get_valid_record(action.original_action_address"; then
      echo "OK:   $file Update path validates the original record"
    else
      echo "FAIL: $file Update path lacks must_get_valid_record(action.original_action_address)"
      fail=1
    fi
    if printf '%s\n' "$update_block" | grep -Fq "check_author_match("; then
      echo "OK:   $file Update path checks original-author authorization"
    else
      echo "FAIL: $file Update path lacks explicit original-author authorization"
      fail=1
    fi
  else
    echo "OK:   $file has no explicit entry Update authorization path"
  fi

  if grep -Fq "FlatOp::Delete(OpDelete { action" "$file"; then
    local delete_block
    delete_block="$(awk 'index($0, "FlatOp::Delete(OpDelete") { in_block=1 } index($0, "FlatOp::Update") && in_block { in_block=0 } in_block' "$file")"
    if printf '%s\n' "$delete_block" | grep -Fq "must_get_valid_record(action.deletes_address"; then
      echo "OK:   $file Delete path validates the original record"
    else
      echo "FAIL: $file Delete path lacks must_get_valid_record(action.deletes_address)"
      fail=1
    fi
    if printf '%s\n' "$delete_block" | grep -Fq "check_author_match("; then
      echo "OK:   $file Delete path checks original-author authorization"
    else
      echo "FAIL: $file Delete path lacks explicit original-author authorization"
      fail=1
    fi
  else
    echo "OK:   $file has no explicit entry Delete authorization path"
  fi
}

# Every declared EntryTypes variant must have at least one explicit validation dispatch reference.
# Every declared EntryTypes variant must have an explicit reference in the
# 0.7 validate dispatcher itself. References in helper functions are not enough:
# the dispatcher is the authoritative operation-to-entry policy boundary.
# Every declared EntryTypes variant must have an explicit reference in the
# 0.7 validate dispatcher itself. References in helper functions are not enough:
# the dispatcher is the authoritative operation-to-entry policy boundary.
check_entry_type_dispatch() {
  local file="$1"
  local source enum_block dispatch_block variant
  source="$(sed '/^\#\[cfg(test)\]/,$d' "$file")"
  enum_block="$(printf '%s\n' "$source" | sed -n '/^pub enum EntryTypes[[:space:]]*{/,/^}/p')"
  if [[ -z "$enum_block" ]]; then
    echo "FAIL: $file has no parseable EntryTypes enum"
    fail=1
    return
  fi

  dispatch_block="$(printf '%s\n' "$source" | awk '
    /^[[:space:]]*pub[[:space:]]+fn[[:space:]]+validate[[:space:]]*\(/ {
      in_block=1
      print
      next
    }
    in_block && /^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+[A-Za-z0-9_]+[[:space:]]*\(/ {
      exit
    }
    in_block { print }
  ')"

  if [[ -z "$dispatch_block" ]]; then
    echo "FAIL: $file has no parseable validate(Op) dispatcher block"
    fail=1
    return
  fi

  while IFS= read -r variant; do
    [[ -z "$variant" ]] && continue
    if printf '%s\n' "$dispatch_block" | rg -n --pcre2 "\\bEntryTypes::${variant}\\b" >/dev/null 2>&1; then
      echo "OK:   $file EntryTypes::$variant has validate dispatcher coverage"
    else
      echo "FAIL: $file EntryTypes::$variant has no validate dispatcher reference"
      fail=1
    fi
  done < <(printf '%s\n' "$enum_block" | rg --pcre2 -o '^\s*[A-Za-z_][A-Za-z0-9_]*\s*\(' | sed -E 's/^\s*([A-Za-z_][A-Za-z0-9_]*).*$/\1/')
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
  create_block="$(printf '%s\n' "$source" | awk '
    index($0, "FlatOp::CreateRecord(OpRecord::CreateEntry") { in_block=1 }
    index($0, "FlatOp::CreateRecord(OpRecord::UpdateEntry") { in_block=0 }
    in_block
  ')"
  update_block="$(printf '%s\n' "$source" | awk '
    index($0, "FlatOp::CreateRecord(OpRecord::UpdateEntry") { in_block=1 }
    index($0, "FlatOp::Link") && in_block { in_block=0 }
    in_block
  ')"
  while IFS= read -r variant; do
    [[ -z "$variant" ]] && continue
    if printf '%s\n' "$create_block" | rg -n --pcre2 "\bEntryTypes::${variant}\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateRecord create dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateRecord create dispatch misses EntryTypes::$variant"
      fail=1
    fi
    if [[ "$variant" != "Anchor" ]] && printf '%s\n' "$create_block" | rg -nU --pcre2 "EntryTypes::${variant}\([^)]*\)[[:space:]]*=>[[:space:]]*Ok[[:space:]]*\([[:space:]]*ValidateCallbackResult::Valid" >/dev/null 2>&1; then
      echo "FAIL: $file CreateRecord accepts non-anchor EntryTypes::$variant without application validation"
      fail=1
    fi
    if printf '%s\n' "$update_block" | rg -n --pcre2 "\bEntryTypes::${variant}\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateRecord update dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateRecord update dispatch misses EntryTypes::$variant"
      fail=1
    fi
  done < <(printf '%s\n' "$enum_block" | rg --pcre2 -o '^\s*[A-Za-z_][A-Za-z0-9_]*\s*\(' | sed -E 's/^\s*([A-Za-z_][A-Za-z0-9_]*).*$/\1/')
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
    if python3 - "$manifest" <<'PY'
import json, sys
from pathlib import Path
def reject_duplicates(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f"duplicate JSON object key: {key}")
        obj[key] = value
    return obj
try:
    json.loads(Path(sys.argv[1]).read_text(), object_pairs_hook=reject_duplicates)
except Exception as exc:
    print(f"duplicate/invalid semantic manifest JSON: {exc}", file=sys.stderr)
    raise SystemExit(1)
PY
    then
      echo "OK:   Hearth semantic-validation manifest has unique JSON object keys"
    else
      echo "FAIL: Hearth semantic-validation manifest contains duplicate or invalid JSON object keys"
      fail=1
    fi
    if rg -n --pcre2 '"schema_version"[[:space:]]*:[[:space:]]*"HEARTH-SEMANTIC-0.7-CASESET-1"' "$manifest" >/dev/null 2>&1; then
      echo "OK:   Hearth semantic-validation case schema is pinned"
    else
      echo "FAIL: Hearth semantic-validation case schema is missing or changed"
      fail=1
    fi
    if rg -n --pcre2 'RuntimeQualificationPending' "$manifest" >/dev/null 2>&1; then
      echo "OK:   Hearth semantic-validation runtime claim ceiling remains pending"
    else
      echo "FAIL: Hearth semantic-validation manifest must retain RuntimeQualificationPending ceiling"
      fail=1
    fi

    manifest_tests="$(sed -n 's/^[[:space:]]*"test"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$manifest" | sort -u)"
    # Discover only functions that are actually test entrypoints. Helper async
    # functions must not inflate the executable qualification-test count.
    executable_tests="$(awk '
      /^[[:space:]]*#\[(tokio::test|test)([^]]*)\][[:space:]]*$/ { pending_test=1; next }
      /^[[:space:]]*#\[ignore([^]]*)\][[:space:]]*$/ { next }
      {
        if (pending_test && $0 ~ /^[[:space:]]*(pub[[:space:]]+)?async[[:space:]]+fn[[:space:]]+test_[A-Za-z0-9_]+[[:space:]]*\(/) {
          sub(/^[[:space:]]*(pub[[:space:]]+)?async[[:space:]]+fn[[:space:]]+/, "", $0)
          sub(/[[:space:]]*\(.*/, "", $0)
          print $0
          pending_test=0
          next
        }
        if ($0 !~ /^[[:space:]]*$/ && $0 !~ /^[[:space:]]*\/\// && $0 !~ /^[[:space:]]*#\[/) {
          pending_test=0
        }
      }
    ' "$rust_test" | sort -u)"

    manifest_case_count="$(printf '%s\n' "$manifest_tests" | sed '/^$/d' | wc -l)"
    executable_test_count="$(printf '%s\n' "$executable_tests" | sed '/^$/d' | wc -l)"
    ignored_test_count="$(rg -n '^#\[ignore' "$rust_test" | wc -l)"

    if [[ "$manifest_case_count" -eq 0 || "$manifest_case_count" -ne "$executable_test_count" ]]; then
      echo "FAIL: semantic manifest/test count mismatch: cases=$manifest_case_count tests=$executable_test_count"
      echo "--- manifest tests"
      printf '%s\n' "$manifest_tests"
      echo "--- executable tests"
      printf '%s\n' "$executable_tests"
      fail=1
    elif [[ "$manifest_tests" != "$executable_tests" ]]; then
      echo "FAIL: semantic manifest/test names do not match exactly"
      echo "--- manifest-only / executable-only diff"
      diff -u <(printf '%s\n' "$manifest_tests") <(printf '%s\n' "$executable_tests") || true
      fail=1
    else
      echo "OK:   semantic-validation manifest exactly maps to $manifest_case_count executable tests"
    fi

    if [[ "$ignored_test_count" -ne "$executable_test_count" ]]; then
      echo "FAIL: semantic-validation executable/ignored test mismatch: executable=$executable_test_count ignored=$ignored_test_count"
      fail=1
    else
      echo "OK:   all semantic-validation executable tests remain ignored until pinned runtime qualification"
    fi
  fi

  if [[ -f "$cargo_manifest" ]]; then
    if rg -n --pcre2 'name[[:space:]]*=[[:space:]]*"sweettest_semantic_validation"' "$cargo_manifest" >/dev/null 2>&1; then
      echo "OK:   semantic-validation Sweettest is registered in tests/Cargo.toml"
    else
      echo "FAIL: semantic-validation Sweettest is not registered in tests/Cargo.toml"
      fail=1
    fi
  fi

  # Exact manifest/test-name equality above is the authoritative structural
  # mapping; do not maintain a second hard-coded list that can drift.
}

# Semantic cases must resolve to real coordinator entrypoints and the runtime
# witness must actually name the same zome/function. This closes the gap between
# a declarative case manifest and executable source.
check_semantic_case_entrypoints() {
  local manifest="mycelix-workspace/mycelix-hearth/tests/hearth-07-semantic-validation-cases.json"
  local rust_test="mycelix-workspace/mycelix-hearth/tests/sweettest_semantic_validation.rs"
  local tests zomes operations
  mapfile -t tests < <(sed -n 's/^[[:space:]]*"test"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$manifest")
  mapfile -t zomes < <(sed -n 's/^[[:space:]]*"zome"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$manifest")
  mapfile -t operations < <(sed -n 's/^[[:space:]]*"operation"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' "$manifest")

  if [[ "${#tests[@]}" -eq 0 || "${#tests[@]}" -ne "${#zomes[@]}" || "${#tests[@]}" -ne "${#operations[@]}" ]]; then
    echo "FAIL: semantic manifest test/zome/operation declaration counts differ"
    fail=1
    return
  fi

  local i test_name zome operation coord_file
  for i in "${!tests[@]}"; do
    test_name="${tests[$i]}"
    zome="${zomes[$i]}"
    operation="${operations[$i]}"
    coord_file="mycelix-workspace/mycelix-hearth/zomes/${zome//_/-}/coordinator/src/lib.rs"

    if [[ -f "$coord_file" ]]; then
      echo "OK:   semantic case ${zome}/${operation} resolves to coordinator source"
    else
      echo "FAIL: semantic case ${zome}/${operation} has no coordinator source: $coord_file"
      fail=1
      continue
    fi

    if awk -v op="$operation" '
      /^[[:space:]]*#\[hdk_extern\][[:space:]]*$/ { saw_extern=1; next }
      {
        if (saw_extern && ($0 ~ /^[[:space:]]*$/ || $0 ~ /^[[:space:]]*\/\//)) {
          next
        }
        if ($0 ~ "^[[:space:]]*(pub[[:space:]]+)?(async[[:space:]]+)?fn[[:space:]]+" op "[[:space:]]*\\(") {
          found=saw_extern
          exit
        }
        saw_extern=0
      }
      END { exit(found ? 0 : 1) }
    ' "$coord_file"; then
      echo "OK:   semantic case ${operation} resolves to the #[hdk_extern]-annotated function"
    else
      echo "FAIL: semantic case ${operation} is not the function immediately annotated by #[hdk_extern]"
      fail=1
    fi

    mapfile -t test_fn_lines < <(
      rg -n --fixed-strings "async fn ${test_name}" "$rust_test" |
        cut -d: -f1
    )
    local test_start test_next test_end test_block
    if [[ "${#test_fn_lines[@]}" -ne 1 ]]; then
      echo "FAIL: semantic case ${zome}/${operation} must map to exactly one test function: ${test_name}"
      fail=1
      continue
    fi
    test_start="${test_fn_lines[0]}"
    test_next=""
    while IFS= read -r test_line; do
      if [[ "$test_line" -gt "$test_start" ]]; then
        test_next="$test_line"
        break
      fi
    done < <(rg -n "^async fn [A-Za-z_][A-Za-z0-9_]*" "$rust_test" | cut -d: -f1 | sort -n)
    if [[ -n "$test_next" ]]; then
      test_end=$((test_next - 1))
    else
      test_end="$(wc -l < "$rust_test")"
    fi
    test_block="$(sed -n "${test_start},${test_end}p" "$rust_test")"

    # Bind the operation to the same call expression, rather than merely
    # requiring the zome name and operation string to coexist somewhere in the
    # test body. This prevents unrelated calls from satisfying the witness.
    if printf "%s\\n" "$test_block" | rg -nU --pcre2 "&alice\\.zome\\(\\\"\\${zome}\\\"\\)[[:space:]]*,[[:space:]]*\\\"\\${operation}\\\"[[:space:]]*," >/dev/null 2>&1; then
      echo "OK:   semantic runtime witness ${test_name} is bound to ${zome}/${operation} call expression"
    else
      echo "FAIL: semantic runtime witness ${zome}/${operation} is not invoked by ${test_name} matching call expression"
      fail=1
    fi
  done
}

# Each semantic case must point at an invariant and operation surface that
# actually exist in the integrity implementation. This prevents a green manifest
# from drifting away from the validator it claims to witness.
check_semantic_case_integrity_bindings() {
  local manifest="mycelix-workspace/mycelix-hearth/tests/hearth-07-semantic-validation-cases.json"
  local ids tests zomes operations invariants surfaces results validator_sources validator_symbols
  mapfile -t ids < <(sed -n 's/^[[:space:]]*"case_id"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t tests < <(sed -n 's/^[[:space:]]*"test"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t zomes < <(sed -n 's/^[[:space:]]*"zome"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t operations < <(sed -n 's/^[[:space:]]*"operation"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t invariants < <(sed -n 's/^[[:space:]]*"invariant"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t surfaces < <(sed -n 's/^[[:space:]]*"operation_surface"[[:space:]]*:[[:space:]]*\[\([^]]*\)\].*/\1/p' "$manifest")
  mapfile -t results < <(sed -n 's/^[[:space:]]*"expected_result"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t validator_sources < <(sed -n 's/^[[:space:]]*"validator_source"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")
  mapfile -t validator_symbols < <(sed -n 's/^[[:space:]]*"validator_symbol"[[:space:]]*:[[:space:]]*"\([^\"]*\)".*/\1/p' "$manifest")

  local count="${#ids[@]}"
  if [[ "$count" -eq 0 || "$count" -ne "${#tests[@]}" || "$count" -ne "${#zomes[@]}" || "$count" -ne "${#operations[@]}" || "$count" -ne "${#invariants[@]}" || "$count" -ne "${#surfaces[@]}" || "$count" -ne "${#results[@]}" || "$count" -ne "${#validator_sources[@]}" || "$count" -ne "${#validator_symbols[@]}" ]]; then
    echo "FAIL: semantic manifest fields are not structurally aligned"
    fail=1
    return
  fi

  local i id zome operation invariant expected_result surface integrity_file validator_source validator_symbol
  for i in "${!ids[@]}"; do
    id="${ids[$i]}"
    zome="${zomes[$i]}"
    operation="${operations[$i]}"
    invariant="${invariants[$i]}"
    expected_result="${results[$i]}"
    surface="${surfaces[$i]}"
    validator_source="${validator_sources[$i]}"
    validator_symbol="${validator_symbols[$i]}"
    integrity_file="mycelix-workspace/mycelix-hearth/zomes/${zome//_/-}/integrity/src/lib.rs"

    if [[ ! -f "$integrity_file" ]]; then
      echo "FAIL: $id references missing integrity source: $integrity_file"
      fail=1
      continue
    fi
    if [[ ! -f "$validator_source" ]]; then
      echo "FAIL: $id references missing validator source: $validator_source"
      fail=1
    else
      echo "OK:   $id declares validator source $validator_source"
      if rg -n --pcre2 "^[[:space:]]*(?:pub[[:space:]]+)?fn[[:space:]]+${validator_symbol}[[:space:]]*\\(" "$validator_source" >/dev/null 2>&1; then
        echo "OK:   $id validator symbol $validator_symbol is defined in declared source"
      else
        echo "FAIL: $id validator symbol $validator_symbol is not defined in declared source"
        fail=1
      fi
      if rg -n --fixed-strings "$invariant" "$validator_source" >/dev/null 2>&1; then
        echo "OK:   $id invariant is present in declared validator source"
      else
        echo "FAIL: $id invariant is absent from declared validator source: $invariant"
        fail=1
      fi
    fi

    if [[ "$validator_source" != "$integrity_file" ]]; then
      if rg -n --pcre2 "\\b${validator_symbol}[[:space:]]*\\(" "$integrity_file" >/dev/null 2>&1; then
        echo "OK:   $id integrity dispatcher invokes declared shared validator $validator_symbol"
      else
        echo "FAIL: $id integrity dispatcher does not invoke declared shared validator $validator_symbol"
        fail=1
      fi
    fi

    if [[ "$expected_result" != "Invalid" ]]; then
      echo "FAIL: $id expected_result must be Invalid, got $expected_result"
      fail=1
    else
      echo "OK:   $id declares expected validation result Invalid"
    fi

    for declared_surface in ${surface//,/ }; do
      declared_surface="${declared_surface//\"/}"
      declared_surface="${declared_surface//[[:space:]]/}"
      case "$declared_surface" in
        CreateEntry) pattern='FlatOp::CreateEntry' ;;
        CreateRecord) pattern='FlatOp::CreateRecord' ;;
        Update) pattern='FlatOp::Update' ;;
        Delete) pattern='FlatOp::Delete' ;;
        Link.CreateLink) pattern='FlatOp::Link(OpLink::CreateLink' ;;
        Link.DeleteLink) pattern='FlatOp::Link(link @ OpLink::DeleteLink' ;;
        *) echo "FAIL: $id contains unknown operation surface: $declared_surface"; fail=1; continue ;;
      esac
      if rg -n --fixed-strings "$pattern" "$integrity_file" >/dev/null 2>&1; then
        echo "OK:   $id declares operation surface $declared_surface present in integrity source"
      else
        echo "FAIL: $id declares operation surface $declared_surface absent from integrity source"
        fail=1
      fi
    done
  done
}
# Dependency retrieval semantics: must_get_action only proves retrieval; it does not prove
# that the referenced record passed application validation. Update/delete authorization
# therefore uses must_get_valid_record before trusting the referenced author. Valid-record
# consumers must also inspect the referenced entry/action rather than treating retrieval
# itself as the invariant.
check_dependency_semantics() {
  local file="$1"
  if python3 - "$file" <<'PY'
import re, sys
from pathlib import Path

path = Path(sys.argv[1])
source = path.read_text()
prod = source.split("#[cfg(test)]", 1)[0]
if re.search(r'must_get_action\(action\.(?:original_action_address|deletes_address)', prod):
    print(f"FAIL: {path} uses must_get_action for update/delete authorization")
    raise SystemExit(2)

lines = prod.splitlines()
starts = [i for i, line in enumerate(lines) if re.match(r'^\s*(?:pub\s+)?fn\s+[A-Za-z0-9_]+\s*\(', line)]
call_count = prod.count("must_get_valid_record(")
if call_count == 0:
    print(f"OK:   {path} has no must_get_valid_record dependency sites")
    raise SystemExit(0)

consumers = 0
for idx, start in enumerate(starts):
    end = starts[idx + 1] if idx + 1 < len(starts) else len(lines)
    block = "\n".join(lines[start:end])
    if "must_get_valid_record(" not in block:
        continue
    consumers += 1
    name_match = re.search(r'fn\s+([A-Za-z0-9_]+)', lines[start])
    fn_name = name_match.group(1) if name_match else f"<line {start + 1}>"
    if re.search(r'\.entry\(\)|\.action\(\)|try_from_action', block):
        print(f"OK:   {path} {fn_name} inspects each valid-record dependency within its function scope")
    else:
        print(f"FAIL: {path} {fn_name} retrieves a valid record without inspecting its entry/action")
        raise SystemExit(2)

if consumers == 0:
    print(f"FAIL: {path} has must_get_valid_record calls outside recognized function scope")
    raise SystemExit(2)
PY
  then
    return
  else
    fail=1
  fi
}

# Immutable-field helpers must prove the referenced CreateRecord is valid and
# deserialize the original entry before comparing fields. This guards against a
# future helper that retrieves a record but accidentally treats retrieval as proof.
# Immutable-field helpers must each prove their own referenced CreateRecord is
# valid and deserialize the original entry. File-wide evidence is insufficient:
# one well-formed helper must not mask another helper's missing dependency proof.
# Immutable-field helpers must each prove their own referenced CreateRecord is
# valid and deserialize the original entry. File-wide evidence is insufficient:
# one well-formed helper must not mask another helper's missing dependency proof.
check_immutable_dependency_semantics() {
  local file="$1"
  if python3 - "$file" <<'PY'
import re, sys
from pathlib import Path

path = Path(sys.argv[1])
source = path.read_text()
prod = source.split("#[cfg(test)]", 1)[0]
lines = prod.splitlines()
helper_starts = [
    i for i, line in enumerate(lines)
    if re.match(r'^\s*(?:pub\s+)?fn\s+validate_[A-Za-z0-9_]*immutable_fields\s*\(', line)
]
if not helper_starts:
    print(f"OK:   {path} has no immutable-field helper sites")
    raise SystemExit(0)

fn_starts = [
    i for i, line in enumerate(lines)
    if re.match(r'^\s*(?:pub\s+)?fn\s+[A-Za-z0-9_]+\s*\(', line)
]

for start in helper_starts:
    next_fn = next((line for line in fn_starts if line > start), len(lines))
    block = "\n".join(lines[start:next_fn])
    match = re.search(r'fn\s+(validate_[A-Za-z0-9_]*immutable_fields)', lines[start])
    if not match:
        print(f"FAIL: {path} could not resolve immutable-field helper name at line {start + 1}")
        raise SystemExit(2)
    name = match.group(1)

    if "must_get_valid_record(" not in block:
        print(f"FAIL: {path} {name} lacks must_get_valid_record")
        raise SystemExit(2)

    normalized = " ".join(block.splitlines())
    if not re.search(r'\.entry\(\)\s*\.to_app_option\(\)', normalized):
        print(f"FAIL: {path} {name} retrieves a valid record without deserializing its original entry")
        raise SystemExit(2)

    uses = len(re.findall(re.escape(name) + r'\(', prod))
    if uses < 2:
        print(f"FAIL: {path} {name} has no production call site outside its declaration")
        raise SystemExit(2)

    print(f"OK:   {path} {name} validates and deserializes its own immutable dependency")
    print(f"OK:   {path} {name} has an explicit production call site")
PY
  then
    return
  else
    fail=1
  fi
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
  if rg -n --fixed-strings "runs-on: ubuntu-24.04" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow pins the GitHub-hosted runner image to Ubuntu 24.04"
  else
    echo "FAIL: qualification workflow must pin runs-on to ubuntu-24.04"
    fail=1
  fi
  if rg -n --fixed-strings 'echo "runner_image_os=\${ImageOS:-unknown}"' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'echo "runner_image_version=\${ImageVersion:-unknown}"' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'echo "runner_arch=\${RUNNER_ARCH:-unknown}"' "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow captures resolved hosted-runner provenance"
  else
    echo "FAIL: qualification workflow must capture resolved hosted-runner provenance"
    fail=1
  fi
  if rg -n --fixed-strings 'mapfile -t semantic_validator_sources' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'tests/hearth-07-semantic-validation-cases.json' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings '"\${semantic_validator_sources[@]}"' "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow hashes every manifest-declared semantic validator source"
  else
    echo "FAIL: qualification workflow must hash every manifest-declared semantic validator source"
    fail=1
  fi
  if rg -n --fixed-strings "target_sha:" "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'ref: ${{ env.QUALIFY_SHA }}' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings "git rev-parse HEAD" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow binds execution to an exact candidate SHA"
  else
    echo "FAIL: qualification workflow does not enforce exact candidate-SHA checkout provenance"
    fail=1
  fi
  local expected_action_ref action_use_count pinned_action_count
  local expected_action_refs=(
    "actions/checkout@d23441a48e516b6c34aea4fa41551a30e30af803"
    "NixOS/nix-installer-action@62c1943b776c509394b550f3f983adc14e9212d6"
    "cachix/cachix-action@38b082610b782e7e93e209c35fd730d399dee866"
    "actions/upload-artifact@b7c566a772e6b6bfb58ed0dc250532a479d7789f"
  )
  if rg -n --fixed-strings 'NixOS/nix-installer-action@62c1943b776c509394b550f3f983adc14e9212d6' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'dogfood: "true"' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'dogfood-path: "/tmp/nix-installer"' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'releases/download/2.35.2/nix-installer-x86_64-linux' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings '5448a1cd70ad945cb4d36365defbaf3731eba38e23859f3dc8bd7418e1946acc' "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings 'sha256sum --check --status -' "$workflow" >/dev/null 2>&1 \
    && ! rg -n --fixed-strings 'cachix/install-nix-action@' "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow pins and hash-locks the Nix installer binary"
  else
    echo "FAIL: qualification workflow must pin and hash-lock the Nix installer binary"
    fail=1
  fi

  local submodule_url
  submodule_url="$(git config -f .gitmodules --get submodule.mycelix-health.url 2>/dev/null || true)"
  if [[ "$submodule_url" == "https://github.com/Luminous-Dynamics/mycelix-health.git" ]]; then
    echo "OK:   qualification checkout declares the expected Hearth submodule source"
  else
    echo "FAIL: qualification checkout has an unexpected or missing Hearth submodule source"
    fail=1
  fi

  if rg -n --fixed-strings 'group: hearth-07-qualification-${{ github.head_ref || github.ref_name }}' "$workflow" >/dev/null 2>&1 \
    && ! rg -n --fixed-strings 'github.event.pull_request.head.ref || github.ref' "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow uses one branch-name cancellation domain across push/PR events"
  else
    echo "FAIL: qualification workflow must normalize push/PR refs to the same branch-name concurrency key"
    fail=1
  fi

  if ! rg -n --fixed-strings "nix develop .#ci --impure" "$workflow" >/dev/null 2>&1 \
    && ! rg -n --fixed-strings "nix_path:" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow uses pure flake evaluation without mutable NIX_PATH channels"
  else
    echo "FAIL: qualification workflow must not use --impure evaluation or mutable NIX_PATH channels"
    fail=1
  fi

  if rg -n --fixed-strings "persist-credentials: false" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow does not persist the GitHub token after checkout"
  else
    echo "FAIL: qualification workflow must set persist-credentials: false"
    fail=1
  fi

  if rg -n --fixed-strings "Verify Nix credential isolation" "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings "sudo grep -Eq 'access-tokens[[:space:]]*=.*github\\.com' /etc/nix/nix.conf" "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings "Nix configuration unexpectedly contains a GitHub access token" "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings "exit 1" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow enforces Nix credential isolation with an executable guard"
  else
    echo "FAIL: qualification workflow must contain an executable Nix credential-isolation guard"
    fail=1
  fi

  action_use_count="$(rg -n --pcre2 '^[[:space:]]*(?:-[[:space:]]+)?uses:' "$workflow" | wc -l)"
  pinned_action_count="$(rg -n --pcre2 '^[[:space:]]*(?:-[[:space:]]+)?uses:[[:space:]]+[^[:space:]@]+@[0-9a-f]{40}[[:space:]]*(#.*)?$' "$workflow" | wc -l)"
  if [[ "$action_use_count" -ne "${#expected_action_refs[@]}" ]]; then
    echo "FAIL: qualification workflow action count changed: expected ${#expected_action_refs[@]}, got $action_use_count"
    fail=1
  elif [[ "$pinned_action_count" -ne "$action_use_count" ]]; then
    echo "FAIL: qualification workflow contains unpinned action refs"
    fail=1
  else
    echo "OK:   qualification workflow pins every action to a full commit SHA"
  fi
  for expected_action_ref in "${expected_action_refs[@]}"; do
    if rg -n --fixed-strings "$expected_action_ref" "$workflow" >/dev/null 2>&1; then
      echo "OK:   qualification workflow uses reviewed action ref $expected_action_ref"
    else
      echo "FAIL: qualification workflow is missing reviewed action ref $expected_action_ref"
      fail=1
    fi
  done

  if rg -n --fixed-strings "cargo test --locked --release --test sweettest_semantic_validation -- --include-ignored --test-threads=1" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow runs only the authoritative semantic Sweettest target"
  else
    echo "FAIL: qualification workflow must target sweettest_semantic_validation explicitly"
    fail=1
  fi
  if rg -n --fixed-strings "cargo build --locked" "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings "cargo test --locked" "$workflow" >/dev/null 2>&1 \
    && rg -n --fixed-strings "cargo generate-lockfile" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow generates and consumes locked Rust closures"
  else
    echo "FAIL: qualification workflow is missing locked Rust dependency closure enforcement"
    fail=1
  fi
}

# Coordinator-to-integrity operation binding. A static validator can be internally
# complete while the coordinator silently uses an operation family that the integrity
# zome does not model explicitly. Tie the application write surface to its validator
# counterpart without assuming every zome must expose every operation family.
# Coordinator symbol parity: coordinator code may only construct entry/link types
# that the paired integrity zome actually declares. This catches stale coordinator
# references after entry/link migrations or renames.
check_coordinator_symbol_parity() {
  local coordinator file zome enum_block variant source
  while IFS= read -r -d "" coordinator; do
    zome="$(basename "$(dirname "$(dirname "$(dirname "$coordinator")")")")"
    file="mycelix-workspace/mycelix-hearth/zomes/$zome/integrity/src/lib.rs"
    [[ -f "$file" ]] || continue
    source="$(sed '/^\#\[cfg(test)\]/,$d' "$coordinator")"

    enum_block="$(sed '/^\#\[cfg(test)\]/,$d' "$file" | sed -n '/^pub enum EntryTypes[[:space:]]*{/,/^}/p')"
    while IFS= read -r variant; do
      [[ -z "$variant" ]] && continue
      if printf '%s\n' "$enum_block" | grep -Eq "^[[:space:]]*$variant\("; then
        echo "OK:   $zome coordinator EntryTypes::$variant matches integrity declaration"
      else
        echo "FAIL: $zome coordinator references undeclared EntryTypes::$variant"
        fail=1
      fi
    done < <(printf '%s\n' "$source" | rg -o --pcre2 'EntryTypes::[A-Za-z_][A-Za-z0-9_]*' | sed 's/.*EntryTypes:://' | sort -u)

    enum_block="$(sed '/^\#\[cfg(test)\]/,$d' "$file" | sed -n '/^pub enum LinkTypes[[:space:]]*{/,/^}/p')"
    while IFS= read -r variant; do
      [[ -z "$variant" ]] && continue
      if printf '%s\n' "$enum_block" | grep -Eq "^[[:space:]]*$variant,$"; then
        echo "OK:   $zome coordinator LinkTypes::$variant matches integrity declaration"
      else
        echo "FAIL: $zome coordinator references undeclared LinkTypes::$variant"
        fail=1
      fi
    done < <(printf '%s\n' "$source" | rg -o --pcre2 'LinkTypes::[A-Za-z_][A-Za-z0-9_]*' | sed 's/.*LinkTypes:://' | sort -u)
  done < <(git ls-files -z -- "mycelix-workspace/mycelix-hearth/zomes/*/coordinator/src/**/*.rs")
}

check_coordinator_operation_bindings() {
  local coordinator file zome source
  while IFS= read -r -d "" coordinator; do
    zome="$(basename "$(dirname "$(dirname "$(dirname "$coordinator")")")")"
    file="mycelix-workspace/mycelix-hearth/zomes/$zome/integrity/src/lib.rs"
    if [[ ! -f "$file" ]]; then
      echo "FAIL: coordinator $coordinator has no paired integrity source $file"
      fail=1
      continue
    fi
    source="$(sed '/^\#\[cfg(test)\]/,$d' "$coordinator")"

    # Entry creation is paired with both 0.7 validation surfaces.
    if printf '%s\n' "$source" | rg -n --pcre2 '\bcreate_entry\s*\(' >/dev/null 2>&1; then
      if rg -n --pcre2 'FlatOp::CreateEntry\s*\(' "$file" >/dev/null 2>&1 \
        && rg -n --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::CreateEntry' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator entry writes map to CreateEntry + CreateRecord validation"
      else
        echo "FAIL: $zome coordinator calls create_entry but integrity lacks paired 0.7 create validation"
        fail=1
      fi
    fi

    # Mutable entry writes need content validation, CreateRecord parity, and
    # action-level author authorization.
    if printf '%s\n' "$source" | rg -n --pcre2 '\bupdate_entry\s*\(' >/dev/null 2>&1; then
      if rg -n --pcre2 'FlatOp::CreateEntry\s*\(\s*OpEntry::UpdateEntry' "$file" >/dev/null 2>&1 \
        && rg -n --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::UpdateEntry' "$file" >/dev/null 2>&1 \
        && rg -n --pcre2 'FlatOp::Update\s*\(\s*OpUpdate::Entry' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator update_entry maps to content + CreateRecord + Update authorization"
      else
        echo "FAIL: $zome coordinator calls update_entry without complete 0.7 update validation coverage"
        fail=1
      fi
    fi

    if printf '%s\n' "$source" | rg -n --pcre2 '\bdelete_entry\s*\(' >/dev/null 2>&1; then
      if rg -n --pcre2 'FlatOp::Delete\s*\(\s*OpDelete\s*\{\s*action' "$file" >/dev/null 2>&1 \
        && rg -n --pcre2 'must_get_valid_record\s*\(\s*action\.deletes_address' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator delete_entry maps to validated Delete authorization"
      else
        echo "FAIL: $zome coordinator calls delete_entry without validated Delete authorization coverage"
        fail=1
      fi
    fi

    if printf '%s\n' "$source" | rg -n --pcre2 '\bcreate_link\s*\(' >/dev/null 2>&1; then
      if rg -n --pcre2 'FlatOp::Link\s*\(\s*OpLink::CreateLink' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator create_link maps to explicit CreateLink validation"
      else
        echo "FAIL: $zome coordinator calls create_link but integrity lacks explicit CreateLink validation"
        fail=1
      fi
    fi

    if printf '%s\n' "$source" | rg -n --pcre2 '\bdelete_link\s*\(' >/dev/null 2>&1; then
      if rg -n --pcre2 'FlatOp::Link\s*\([^)]*OpLink::DeleteLink' "$file" >/dev/null 2>&1 \
        && rg -n --pcre2 'must_get_valid_record\s*\(\s*action\.link_add_address' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator delete_link maps to validated DeleteLink authorization"
      else
        echo "FAIL: $zome coordinator calls delete_link without validated DeleteLink authorization coverage"
        fail=1
      fi
    fi
  done < <(git ls-files -z -- "mycelix-workspace/mycelix-hearth/zomes/*/coordinator/src/**/*.rs")
}

check_dna_source_completeness() {
  local dna="mycelix-workspace/mycelix-hearth/dna/dna.yaml"
  local count=0
  while IFS= read -r -d "" file; do
    local dir name
    dir="$(basename "$(dirname "$(dirname "$(dirname "$file")")")")"
    name="${dir//-/_}_integrity"
    if rg -n --fixed-strings "- name: $name" "$dna" >/dev/null 2>&1; then
      echo "OK:   DNA packages discovered integrity zome $name"
    else
      echo "FAIL: DNA is missing discovered integrity zome $name"
      fail=1
    fi
    count=$((count + 1))
  done < <(git ls-files -z -- "mycelix-workspace/mycelix-hearth/zomes/*/integrity/src/lib.rs")
  local dna_count
  dna_count="$(awk '/^integrity:/,/^coordinator:/ { if ($0 ~ /^[[:space:]]*- name: hearth_[A-Za-z0-9_]+_integrity$/) print $3 }' "$dna" | sort -u | wc -l)"
  if [[ "$dna_count" -eq "$count" ]]; then
    echo "OK:   DNA integrity-zome count matches tracked Hearth integrity zomes ($count)"
  else
    echo "FAIL: DNA integrity-zome count ($dna_count) differs from tracked Hearth integrity zomes ($count)"
    fail=1
  fi
}
run_audit_check() {
  local name="$1"
  shift
  local started finished status=0 tee_status=0 trace_dir trace_file fifo_file tee_pid
  started="$(date +%s)"
  echo "AUDIT_START ${name} epoch=${started}"

  # Run each predicate in the caller's shell so its global 'fail' updates are
  # preserved, while a FIFO-backed tee keeps diagnostics streaming live and
  # stores the exact predicate output for a second, independent failure check.
  trace_dir="$(mktemp -d)"
  trace_file="${trace_dir}/predicate.log"
  fifo_file="${trace_dir}/predicate.fifo"
  mkfifo "${fifo_file}"
  tee "${trace_file}" < "${fifo_file}" &
  tee_pid=$!

  if "$@" >"${fifo_file}" 2>&1; then
    status=0
  else
    status=$?
    fail=1
  fi

  wait "${tee_pid}" || tee_status=$?
  if [[ "${tee_status}" -ne 0 ]]; then
    echo "FAIL: predicate diagnostic tee failed with status ${tee_status}"
    status=1
    fail=1
  fi

  # A predicate that prints FAIL but accidentally returns success must still
  # fail the aggregate audit. This protects the failure-propagation contract
  # independently of each predicate's local control flow.
  if grep -qE '^FAIL:' "${trace_file}"; then
    if [[ "${status}" -eq 0 ]]; then
      echo "AUDIT_FAIL_OUTPUT ${name}: predicate emitted FAIL output despite success status"
      status=1
    else
      echo "AUDIT_FAIL_OUTPUT ${name}: predicate emitted FAIL output"
    fi
    fail=1
  fi

  rm -rf "${trace_dir}"
  finished="$(date +%s)"
  echo "AUDIT_END ${name} status=${status} duration=$((finished - started))s"
  # Individual predicate failures are aggregated through the global 'fail'
  # accumulator; they must not trigger errexit before later diagnostics run.
  return 0
}
run_audit_check check_standalone_tests_workspace_boundary check_standalone_tests_workspace_boundary
run_audit_check check_qualification_workflow_provenance check_qualification_workflow_provenance
run_audit_check check_coordinator_operation_bindings check_coordinator_operation_bindings
run_audit_check check_coordinator_symbol_parity check_coordinator_symbol_parity
run_audit_check check_dna_source_completeness check_dna_source_completeness
run_audit_check check_semantic_validation_suite_wiring check_semantic_validation_suite_wiring
run_audit_check check_semantic_case_entrypoints check_semantic_case_entrypoints
run_audit_check check_semantic_case_integrity_bindings check_semantic_case_integrity_bindings

for file in "${integrity_files[@]}"; do
  run_audit_check "check_create_record_coverage:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_create_record_coverage "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_dangerous_operation_catchalls:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_dangerous_operation_catchalls "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_create_record_entry_dispatch:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_create_record_entry_dispatch "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_dependency_semantics:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_dependency_semantics "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_update_action_coverage:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_update_action_coverage "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_update_delete_authorization:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_update_delete_authorization "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_entry_type_dispatch:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_entry_type_dispatch "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_delete_link_authorization:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_delete_link_authorization "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_link_type_policy:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_link_type_policy "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_link_tag_contract:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_link_tag_contract "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_immutable_dependency_semantics:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_immutable_dependency_semantics "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_validation_determinism:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_validation_determinism "$file"
done

echo
if [[ "$fail" -ne 0 ]]; then
  echo "HEARTH-0.7 source audit: FAIL"
  exit "$fail"
fi
echo "HEARTH-0.7 source audit: PASS"
