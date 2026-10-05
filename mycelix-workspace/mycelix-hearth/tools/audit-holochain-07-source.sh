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
  if rg -nU --pcre2 "$pattern" "$file" >/dev/null 2>&1; then
    echo "OK:   $label"
  else
    echo "FAIL: $label"
    fail=1
  fi
}

# Read semantic manifest fields from canonical JSON in case order. This avoids
# relying on JSON object key order, which is not semantically significant.
semantic_case_field() {
  local manifest="$1"
  local field="$2"
  python3 - "$manifest" "$field" <<'PY'
import json
import sys
from pathlib import Path

manifest, field = sys.argv[1:]
data = json.loads(Path(manifest).read_text())
cases = data.get("cases")
if not isinstance(cases, list):
    raise SystemExit("semantic manifest cases must be an array")
for index, case in enumerate(cases, start=1):
    if not isinstance(case, dict) or field not in case:
        raise SystemExit(f"semantic manifest case #{index} missing field {field!r}")
    value = case[field]
    if isinstance(value, list):
        if any(not isinstance(item, str) for item in value):
            raise SystemExit(f"semantic manifest case #{index} field {field!r} contains non-string values")
        value = ",".join(value)
    if not isinstance(value, str) or "\n" in value or "\r" in value:
        raise SystemExit(f"semantic manifest case #{index} field {field!r} is not a single-line string")
    print(value)
PY
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
  if rg -nU --pcre2 "$pattern" "$file" >/dev/null 2>&1; then
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
    if rg -nU --pcre2 '\bfn\s+validate\s*\(\s*(?:op\s*:\s*)?Op\b|\bvalidate\s*\(\s*op\s*:\s*Op\b' "$file" >/dev/null; then
      echo "OK:   $file exposes validate(Op)"
    else
      echo "FAIL: $file missing validate(Op) semantic seam"; fail=1
    fi
    if rg -nU --pcre2 '\bFlatOp::|flattened\s*<[^>]*>\s*\(' "$file" >/dev/null; then
      echo "OK:   $file handles flattened 0.7 operations"
    else
      echo "FAIL: $file missing FlatOp/flattened 0.7 operation handling"; fail=1
    fi
    if rg -nU --pcre2 'ActionData::|action\.(?:author|timestamp)\s*\(|\.header\.(?:author|timestamp)\b' "$file" >/dev/null; then
      echo "OK:   $file inspects 0.7 action semantics"
    else
      echo "FAIL: $file missing explicit 0.7 action semantic access"; fail=1
    fi
# 0.7 validation must account for every FlatOp family. A zome may
    # explicitly handle a family or intentionally cover it with a terminal
    # catch-all, but silently dropping a family is a migration defect.
    for family in CreateEntry CreateRecord Update Delete Link AgentActivity; do
      if rg -nU --pcre2 "\bFlatOp::${family}\b|\b_\s*=>\s*Ok\s*\(" "$file" >/dev/null; then
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
  local source diagnostic_dir forbidden_log random_log
  source="$(sed '/^\#\[cfg(test)\]/,$d' "$file")"
  diagnostic_dir="$(mktemp -d)"
  forbidden_log="${diagnostic_dir}/forbidden.log"
  random_log="${diagnostic_dir}/random.log"

  if printf '%s\n' "$source" | rg -nU --pcre2 '(?<![A-Za-z0-9_.])(?:get|get_links|get_links_details|get_details|count_links|get_agent_activity|get_validation_receipts|agent_info|call_info|create_entry|update_entry|delete_entry|create_link|delete_link|call|call_remote|send_remote_signal|emit_signal|sys_time|random_bytes)\s*\(' >"$forbidden_log" 2>&1; then
    echo "FAIL: $file contains non-deterministic validation host API usage"
    cat "$forbidden_log"
    fail=1
  else
    echo "OK:   $file validation host API surface is deterministic"
  fi

  if printf '%s\n' "$source" | rg -nU --pcre2 '\b(?:SystemTime|Instant|thread_rng|random::<|rand::|getrandom::)\b' >"$random_log" 2>&1; then
    echo "FAIL: $file contains non-deterministic time/random source in validation production code"
    cat "$random_log"
    fail=1
  else
    echo "OK:   $file validation time/random surface is deterministic"
  fi

  rm -rf "$diagnostic_dir"
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

  if printf '%s\n' "$delete_link_block" | rg -nU --pcre2 'check_link_author_match|original_record\.action\(\)\.author\(\)|original_action\.author\(\)|original_action\\(\\).*author' >/dev/null 2>&1; then
    echo "OK:   $file DeleteLink authorization compares original and deleting authors"
  else
    echo "FAIL: $file DeleteLink path lacks an explicit original/deleting author comparison"
    fail=1
  fi

  if printf '%s\n' "$delete_link_block" | rg -nU --pcre2 'must_get_valid_record\s*\(\s*action\.link_add_address' >/dev/null 2>&1; then
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
  if rg -nU --pcre2 'FlatOp::CreateEntry\s*\(\s*OpEntry::UpdateEntry' "$file" >/dev/null 2>&1; then
    if rg -nU --pcre2 'FlatOp::Update\s*\(\s*OpUpdate::Entry' "$file" >/dev/null 2>&1; then
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
  if rg -nU --pcre2 'FlatOp::Update\(\s*OpUpdate::Entry\s*\{\s*action' "$file" >/dev/null 2>&1; then
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

  if rg -nU --pcre2 'FlatOp::Delete\(\s*OpDelete\s*\{\s*action' "$file" >/dev/null 2>&1; then
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
    if printf '%s\n' "$dispatch_block" | rg -nU --pcre2 "\\bEntryTypes::${variant}\\b" >/dev/null 2>&1; then
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
  if ! printf '%s\n' "$source" | rg -nU --pcre2 'FlatOp::Link\(\s*OpLink::CreateLink\s*\{\s*link_type\s*,\s*action\s*\}' >/dev/null 2>&1; then
    echo "FAIL: $file CreateLink validation does not bind link_type and action"
    fail=1
    return
  fi
  if ! printf '%s\n' "$source" | rg -nU --fixed-strings 'action.data.base_address' >/dev/null 2>&1 || ! printf '%s\n' "$source" | rg -nU --fixed-strings 'action.data.target_address' >/dev/null 2>&1; then
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
    if rg -nU --pcre2 '!matches!\(link_type,\s*LinkTypes::DispatchRateLimit\s*\|\s*LinkTypes::NotificationSubscription\)' "$file" >/dev/null 2>&1; then
      echo "OK:   $file constrains Bridge link tags except intentional dispatch-rate-limit/subscription cases"
    else
      echo "FAIL: $file missing Bridge empty-tag contract"
      fail=1
    fi
  elif [[ "$file" == *"/hearth-stories/"* ]]; then
    if rg -nU --pcre2 '!matches!\(link_type,\s*LinkTypes::TagToStories\)' "$file" >/dev/null 2>&1; then
      echo "OK:   $file constrains Stories link tags except TagToStories"
    else
      echo "FAIL: $file missing Stories empty-tag contract"
      fail=1
    fi
  else
    if rg -nU --pcre2 'if !tag\.0\.is_empty\(\)' "$file" >/dev/null 2>&1; then
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
  if ! rg -nU --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::CreateEntry' "$file" >/dev/null 2>&1; then
    echo "FAIL: $file has no explicit FlatOp::CreateRecord(OpRecord::CreateEntry) validation"
    fail=1
  else
    echo "OK:   $file explicitly validates CreateRecord entry creation"
  fi
  if rg -nU --pcre2 'FlatOp::CreateEntry\s*\(\s*OpEntry::UpdateEntry' "$file" >/dev/null 2>&1; then
    if rg -nU --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::UpdateEntry' "$file" >/dev/null 2>&1; then
      echo "OK:   $file explicitly validates CreateRecord update data"
    else
      echo "FAIL: $file has UpdateEntry validation but no CreateRecord update validation"
      fail=1
    fi
  fi
  if rg -nU --pcre2 'FlatOp::CreateRecord\(\s*_\s*\)\s*=>\s*Ok\s*\(\s*ValidateCallbackResult::Valid' "$file" >/dev/null 2>&1; then
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
    if printf '%s\n' "$create_block" | rg -nU --pcre2 "\bEntryTypes::${variant}\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateRecord create dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateRecord create dispatch misses EntryTypes::$variant"
      fail=1
    fi
    if [[ "$variant" != "Anchor" ]] && printf '%s\n' "$create_block" | rg -nU --pcre2 "EntryTypes::${variant}\([^)]*\)[[:space:]]*=>[[:space:]]*Ok[[:space:]]*\([[:space:]]*ValidateCallbackResult::Valid" >/dev/null 2>&1; then
      echo "FAIL: $file CreateRecord accepts non-anchor EntryTypes::$variant without application validation"
      fail=1
    fi
    if printf '%s\n' "$update_block" | rg -nU --pcre2 "\bEntryTypes::${variant}\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateRecord update dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateRecord update dispatch misses EntryTypes::$variant"
      fail=1
    fi
  done < <(printf '%s\n' "$enum_block" | rg --pcre2 -o '^\s*[A-Za-z_][A-Za-z0-9_]*\s*\(' | sed -E 's/^\s*([A-Za-z_][A-Za-z0-9_]*).*$/\1/')
}
# CreateEntry dispatch must preserve the EntryTypes policy at the first validation surface.
# A permissive FlatOp::CreateEntry(_) => Valid arm can otherwise swallow a newly
# introduced entry variant even when CreateRecord validation remains exhaustive.
check_create_entry_entry_dispatch() {
  local file="$1"
  local source enum_block variant create_block
  source="$(sed '/^\#\[cfg(test)\]/,$d' "$file")"
  enum_block="$(printf '%s\n' "$source" | sed -n '/^pub enum EntryTypes[[:space:]]*{/,/^}/p')"
  create_block="$(printf '%s\n' "$source" | awk '
    index($0, "FlatOp::CreateEntry") { in_block=1 }
    in_block && /^        FlatOp::CreateRecord/ { in_block=0 }
    in_block
  ')"
  if printf '%s\n' "$create_block" | rg -nU --pcre2 'FlatOp::CreateEntry\(\s*_\s*\)\s*=>\s*Ok\s*\(\s*ValidateCallbackResult::Valid' >/dev/null 2>&1; then
    echo "FAIL: $file has permissive FlatOp::CreateEntry(_) => Valid catch-all"
    fail=1
  else
    echo "OK:   $file has no permissive CreateEntry catch-all"
  fi
  while IFS= read -r variant; do
    [[ -z "$variant" ]] && continue
    if printf '%s\n' "$create_block" | rg -nU --pcre2 "\\bEntryTypes::${variant}\\b" >/dev/null 2>&1; then
      echo "OK:   $file CreateEntry dispatch covers EntryTypes::$variant"
    else
      echo "FAIL: $file CreateEntry dispatch misses EntryTypes::$variant"
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
    if rg -nU --pcre2 "FlatOp::${family}\s*\(\s*_\s*\)\s*=>\s*Ok\s*\(\s*ValidateCallbackResult::Valid" "$file" >/dev/null 2>&1; then
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
    if rg -nU --pcre2 '"schema_version"[[:space:]]*:[[:space:]]*"HEARTH-SEMANTIC-0.7-CASESET-1"' "$manifest" >/dev/null 2>&1; then
      echo "OK:   Hearth semantic-validation case schema is pinned"
    else
      echo "FAIL: Hearth semantic-validation case schema is missing or changed"
      fail=1
    fi
    if rg -nU --pcre2 'RuntimeQualificationPending' "$manifest" >/dev/null 2>&1; then
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
    if rg -nU --pcre2 'name[[:space:]]*=[[:space:]]*"sweettest_semantic_validation"' "$cargo_manifest" >/dev/null 2>&1; then
      echo "OK:   semantic-validation Sweettest is registered in tests/Cargo.toml"
    else
      echo "FAIL: semantic-validation Sweettest is not registered in tests/Cargo.toml"
      fail=1
    fi
  fi

  # Exact manifest/test-name equality above is the authoritative structural
  # mapping; do not maintain a second hard-coded list that can drift.
}

# The semantic manifest is itself a qualification input. Parse it as JSON before
# using line-oriented extraction so malformed structure, duplicate keys, or repeated
# case identities cannot silently produce a different audit meaning.
check_semantic_manifest_schema() {
  local manifest="mycelix-workspace/mycelix-hearth/tests/hearth-07-semantic-validation-cases.json"
  if python3 - "$manifest" <<'PY'
import json
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
expected_case_keys = {
    "case_id",
    "test",
    "zome",
    "operation",
    "operation_surface",
    "invariant",
    "rejection_reason",
    "expected_result",
    "boundary",
    "validator_source",
    "validator_symbol",
    "dispatch_symbol",
    "target_variant",
    "coordinator_primitive",
    "invariant_code",
}
expected_top_keys = {"schema_version", "claim_ceiling", "cases"}
allowed_surfaces = {
    "CreateEntry",
    "CreateRecord",
    "Update",
    "Delete",
    "Link.CreateLink",
    "Link.DeleteLink",
}

def reject(message):
    print(f"FAIL: semantic manifest {message}")
    raise SystemExit(2)

def no_duplicate_pairs(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            reject(f"contains duplicate JSON key {key!r}")
        obj[key] = value
    return obj

try:
    data = json.loads(path.read_text(), object_pairs_hook=no_duplicate_pairs)
except (OSError, UnicodeError, json.JSONDecodeError) as exc:
    reject(f"cannot be parsed as valid UTF-8 JSON: {exc}")

if not isinstance(data, dict):
    reject("top level must be an object")
if set(data) != expected_top_keys:
    reject(
        f"top-level keys mismatch: expected {sorted(expected_top_keys)}, "
        f"got {sorted(data)}"
    )
if data.get("schema_version") != "HEARTH-SEMANTIC-0.7-CASESET-1":
    reject(
        "schema_version is not HEARTH-SEMANTIC-0.7-CASESET-1: "
        f"{data.get('schema_version')!r}"
    )
claim = data.get("claim_ceiling")
if not isinstance(claim, str) or "RuntimeQualificationPending" not in claim:
    reject("claim_ceiling must retain the RuntimeQualificationPending evidence ceiling")
if not isinstance(claim, str) or "do not constitute observed runtime results" not in claim:
    reject("claim_ceiling must explicitly deny observed runtime qualification")

cases = data.get("cases")
if not isinstance(cases, list) or not cases:
    reject("cases must be a non-empty array")

seen_ids = set()
seen_tests = set()
seen_entrypoints = set()
for index, case in enumerate(cases, start=1):
    if not isinstance(case, dict):
        reject(f"case #{index} must be an object")
    if set(case) != expected_case_keys:
        reject(
            f"case #{index} keys mismatch: expected {sorted(expected_case_keys)}, "
            f"got {sorted(case)}"
        )
    for key in expected_case_keys:
        value = case[key]
        expected_type = list if key == "operation_surface" else str
        if not isinstance(value, expected_type):
            reject(f"case #{index} field {key!r} has the wrong JSON type")
        if isinstance(value, str):
            if not value.strip():
                reject(f"case #{index} field {key!r} is empty")
            if "\n" in value or "\r" in value:
                reject(f"case #{index} field {key!r} contains a newline")

    case_id = case["case_id"]
    test = case["test"]
    zome = case["zome"]
    operation = case["operation"]
    validator_symbol = case["validator_symbol"]
    validator_source = case["validator_source"]
    dispatch_symbol = case["dispatch_symbol"]
    target_variant = case["target_variant"]
    coordinator_primitive = case["coordinator_primitive"]
    if not re.fullmatch(r"SEM-[0-9]+", case_id):
        reject(f"{case_id!r} is not a stable SEM-N numeric case identifier")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", test):
        reject(f"{case_id} test name is not a safe Rust identifier")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", zome):
        reject(f"{case_id} zome name is not a safe underscore identifier")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", operation):
        reject(f"{case_id} operation name is not a safe Rust identifier")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", validator_symbol):
        reject(f"{case_id} validator_symbol is not a safe Rust identifier")
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", dispatch_symbol):
        reject(f"{case_id} dispatch_symbol is not a safe Rust identifier")
    if not re.fullmatch(r"(?:EntryTypes|LinkTypes)::[A-Za-z_][A-Za-z0-9_]*", target_variant):
        reject(f"{case_id} target_variant must be EntryTypes::<Name> or LinkTypes::<Name>")
    if coordinator_primitive not in {"create_entry", "update_entry", "create_link"}:
        reject(f"{case_id} coordinator_primitive must be create_entry, update_entry, or create_link")
    if coordinator_primitive in {"create_entry", "update_entry"} and not target_variant.startswith("EntryTypes::"):
        reject(f"{case_id} {coordinator_primitive} requires an EntryTypes::<Name> target_variant")
    if coordinator_primitive == "create_link" and not target_variant.startswith("LinkTypes::"):
        reject(f"{case_id} create_link requires a LinkTypes::<Name> target_variant")
    if (
        not (
            validator_source.startswith("mycelix-workspace/mycelix-hearth/")
            or validator_source.startswith("crates/")
        )
        or ".." in Path(validator_source).parts
        or not validator_source.endswith(".rs")
    ):
        reject(f"{case_id} validator_source is outside the approved repository Rust source boundary")
    entrypoint = (zome, operation)
    if case_id in seen_ids:
        reject(f"case_id {case_id!r} is duplicated")
    if test in seen_tests:
        reject(f"test {test!r} is duplicated")
    if entrypoint in seen_entrypoints:
        reject(f"zome/operation entrypoint {entrypoint!r} is duplicated")
    seen_ids.add(case_id)
    seen_tests.add(test)
    seen_entrypoints.add(entrypoint)

    if case["boundary"] != "integrity_validation":
        reject(f"{case_id} boundary must be integrity_validation")
    if case["expected_result"] != "Invalid":
        reject(f"{case_id} expected_result must be Invalid")
    surfaces = case["operation_surface"]
    if not surfaces or any(not isinstance(surface, str) for surface in surfaces):
        reject(f"{case_id} operation_surface must be a non-empty string array")
    if len(surfaces) != len(set(surfaces)):
        reject(f"{case_id} contains duplicate operation surfaces")
    unknown = [surface for surface in surfaces if surface not in allowed_surfaces]
    if unknown:
        reject(f"{case_id} contains unknown operation surfaces: {unknown}")
    expected_surfaces = {
        "create_entry": {"CreateEntry", "CreateRecord"},
        "update_entry": {"CreateEntry", "CreateRecord", "Update"},
        "create_link": {"Link.CreateLink"},
    }[coordinator_primitive]
    if set(surfaces) != expected_surfaces:
        reject(
            f"{case_id} operation_surface is inconsistent with coordinator_primitive "
            f"{coordinator_primitive!r}: expected {sorted(expected_surfaces)}, got {sorted(surfaces)}"
        )
    if case["rejection_reason"].strip() != case["rejection_reason"]:
        reject(f"{case_id} rejection_reason has leading/trailing whitespace")

print(
    f"OK:   semantic manifest schema is valid, duplicate-free, and fail-closed "
    f"({len(cases)} cases)"
)
PY
  then
    return
  else
    fail=1
  fi
}

# Semantic cases must resolve to real coordinator entrypoints and the runtime
# witness must actually name the same zome/function. This closes the gap between
# a declarative case manifest and executable source.
check_semantic_case_entrypoints() {
  local manifest="mycelix-workspace/mycelix-hearth/tests/hearth-07-semantic-validation-cases.json"
  local rust_test="mycelix-workspace/mycelix-hearth/tests/sweettest_semantic_validation.rs"
  local tests zomes operations
  mapfile -t tests < <(semantic_case_field "$manifest" test)
  mapfile -t zomes < <(semantic_case_field "$manifest" zome)
  mapfile -t operations < <(semantic_case_field "$manifest" operation)

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

    # Bind the manifest case to the result-producing call itself. Requiring the
    # exact zome/operation inside call_fallible prevents unrelated calls from
    # satisfying the witness.
    if printf "%s\\n" "$test_block" | rg -nU --pcre2 "let[[:space:]]+result(?:[[:space:]]*:[^=;]+)?[[:space:]]*=[[:space:]]*conductor[[:space:]]*\\.[[:space:]]*call_fallible\\([[:space:]]*&alice\\.zome\\(\\\"\\${zome}\\\"\\)[[:space:]]*,[[:space:]]*\\\"\\${operation}\\\"[[:space:]]*," >/dev/null 2>&1; then
      echo "OK:   semantic runtime witness ${test_name} binds ${zome}/${operation} to the asserted result call"
    else
      echo "FAIL: semantic runtime witness ${zome}/${operation} is not bound to a result-producing call_fallible expression in ${test_name}"
      fail=1
    fi
    if printf "%s\\n" "$test_block" | rg -nU --pcre2 "assert_integrity_rejection\\([[:space:]]*result[[:space:]]*," >/dev/null 2>&1; then
      echo "OK:   semantic runtime witness ${test_name} asserts rejection of that result value"
    else
      echo "FAIL: semantic runtime witness ${test_name} does not assert rejection of the result value"
      fail=1
    fi
    # A negative-only witness can pass even if the validator rejects everything.
    # Require a repaired-input control that invokes the same coordinator operation
    # through call_fallible and explicitly observes success.
    if printf "%s\\n" "$test_block" | rg -nU --pcre2 "let[[:space:]]+valid_result(?:[[:space:]]*:[^=;]+)?[[:space:]]*=[[:space:]]*conductor[[:space:]]*\\.[[:space:]]*call_fallible\\([[:space:]]*&alice\\.zome\\(\\\"\\${zome}\\\"\\\)[[:space:]]*,[[:space:]]*\\\"\\${operation}\\\"[[:space:]]*," >/dev/null 2>&1; then
      echo "OK:   semantic runtime witness ${test_name} contains a repaired-input success control for ${zome}/${operation}"
    else
      echo "FAIL: semantic runtime witness ${test_name} lacks a repaired-input call_fallible success control for ${zome}/${operation}"
      fail=1
    fi
    if printf "%s\\n" "$test_block" | rg -nU --pcre2 "assert\\!\\([[:space:]]*valid_result\\.is_ok\\(\\)" >/dev/null 2>&1; then
      echo "OK:   semantic runtime witness ${test_name} asserts repaired-input acceptance"
    else
      echo "FAIL: semantic runtime witness ${test_name} does not assert repaired-input acceptance"
      fail=1
    fi
    # Bind the rejection reason to the same manifest case, including multiline formatting.

    if printf "%s\\n" "$test_block" | rg -nU --pcre2 "expected_reason\\([[:space:]]*\"${test_name}\"[[:space:]]*\\)" >/dev/null 2>&1; then
      echo "OK:   semantic runtime witness ${test_name} binds its rejection reason to the manifest case"
    else
      echo "FAIL: semantic runtime witness ${test_name} does not bind expected_reason to the manifest case"
      fail=1
    fi
  done
}

# Each semantic case must point at an invariant and operation surface that
# actually exist in the integrity implementation. This prevents a green manifest
# from drifting away from the validator it claims to witness.

check_semantic_case_integrity_bindings() {
  local manifest="mycelix-workspace/mycelix-hearth/tests/hearth-07-semantic-validation-cases.json"
  local ids tests zomes operations invariants rejection_reasons invariant_codes surfaces results validator_sources validator_symbols dispatch_symbols target_variants coordinator_primitives
  mapfile -t ids < <(semantic_case_field "$manifest" case_id)
  mapfile -t tests < <(semantic_case_field "$manifest" test)
  mapfile -t zomes < <(semantic_case_field "$manifest" zome)
  mapfile -t operations < <(semantic_case_field "$manifest" operation)
  mapfile -t invariants < <(semantic_case_field "$manifest" invariant)
  mapfile -t rejection_reasons < <(semantic_case_field "$manifest" rejection_reason)
  mapfile -t invariant_codes < <(semantic_case_field "$manifest" invariant_code)
  mapfile -t surfaces < <(semantic_case_field "$manifest" operation_surface)
  mapfile -t results < <(semantic_case_field "$manifest" expected_result)
  mapfile -t validator_sources < <(semantic_case_field "$manifest" validator_source)
  mapfile -t validator_symbols < <(semantic_case_field "$manifest" validator_symbol)
  mapfile -t dispatch_symbols < <(semantic_case_field "$manifest" dispatch_symbol)
  mapfile -t target_variants < <(semantic_case_field "$manifest" target_variant)
  mapfile -t coordinator_primitives < <(semantic_case_field "$manifest" coordinator_primitive)
  local count="${#ids[@]}"
  if [[ "$count" -eq 0 || "$count" -ne "${#tests[@]}" || "$count" -ne "${#zomes[@]}" || "$count" -ne "${#operations[@]}" || "$count" -ne "${#invariants[@]}" || "$count" -ne "${#rejection_reasons[@]}" || "$count" -ne "${#invariant_codes[@]}" || "$count" -ne "${#surfaces[@]}" || "$count" -ne "${#results[@]}" || "$count" -ne "${#validator_sources[@]}" || "$count" -ne "${#validator_symbols[@]}" || "$count" -ne "${#dispatch_symbols[@]}" || "$count" -ne "${#target_variants[@]}" || "$count" -ne "${#coordinator_primitives[@]}" ]]; then
    echo "FAIL: semantic manifest fields are not structurally aligned"
    fail=1
    return
  fi

  local i id zome operation invariant rejection_reason invariant_code expected_result surface integrity_file validator_source validator_symbol dispatch_symbol target_variant coordinator_primitive
  for i in "${!ids[@]}"; do
    id="${ids[$i]}"
    zome="${zomes[$i]}"
    operation="${operations[$i]}"
    invariant="${invariants[$i]}"
    rejection_reason="${rejection_reasons[$i]}"
    invariant_code="${invariant_codes[$i]}"
    if [[ -z "$rejection_reason" ]]; then
      echo "FAIL: $id has no rejection_reason"
      fail=1
      continue
    fi
    if [[ -z "$invariant_code" ]]; then
      echo "FAIL: $id has no executable invariant predicate"
      fail=1
      continue
    fi
    expected_result="${results[$i]}"
    surface="${surfaces[$i]}"
    validator_source="${validator_sources[$i]}"
    validator_symbol="${validator_symbols[$i]}"
    dispatch_symbol="${dispatch_symbols[$i]}"
    target_variant="${target_variants[$i]}"
    coordinator_primitive="${coordinator_primitives[$i]}"
    integrity_file="mycelix-workspace/mycelix-hearth/zomes/${zome//_/-}/integrity/src/lib.rs"

    coord_file="mycelix-workspace/mycelix-hearth/zomes/${zome//_/-}/coordinator/src/lib.rs"
    if [[ ! -f "$coord_file" ]]; then
      echo "FAIL: $id has no coordinator source: $coord_file"
      fail=1
      continue
    fi
    coord_block="$(awk -v operation="$operation" '
      /^[[:space:]]*#\[hdk_extern\][[:space:]]*$/ { pending_extern=1; next }
      pending_extern && $0 ~ "^[[:space:]]*(pub[[:space:]]+)?(async[[:space:]]+)?fn[[:space:]]+" operation "[[:space:]]*\\(" {
        in_block=1
        print
        pending_extern=0
        next
      }
      pending_extern=0
      in_block && /^[[:space:]]*(pub[[:space:]]+)?(async[[:space:]]+)?fn[[:space:]]+[A-Za-z0-9_]+[[:space:]]*\(/ { exit }
      in_block { print }
    ' "$coord_file")"
    if [[ -z "$coord_block" ]]; then
      echo "FAIL: $id could not isolate coordinator operation $operation for typed target binding"
      fail=1
    elif COORD_WITNESS="$coord_block" python3 - "$coordinator_primitive" "$target_variant" <<'PY'
import os
import re
import sys

primitive, target = sys.argv[1:]
source = os.environ["COORD_WITNESS"]

token_re = re.compile(
    r'//[^\n]*'
    r'|/\*.*?\*/'
    r'|(?:br|rb|r)(#{0,255})"(?:.|\n)*?"\1'
    r'|"(?:\\.|[^"\\])*"'
    r"|b?'(?:\\\\.|[^'\\\\\n])'(?![A-Za-z0-9_])",
    re.S,
)
masked = token_re.sub(
    lambda m: "".join("\n" if c == "\n" else " " for c in m.group(0)),
    source,
)

escaped_target = re.escape(target)
if primitive == "create_entry":
    pattern = rf'\bcreate_entry\s*\(\s*&\s*EntryTypes::{escaped_target}\s*\('
elif primitive == "update_entry":
    pattern = rf'\bupdate_entry\s*\([^;]*?&\s*EntryTypes::{escaped_target}\s*\('
elif primitive == "create_link":
    pattern = rf'\bcreate_link\s*\([^;]*?LinkTypes::{escaped_target}\b'
else:
    raise SystemExit(f"unsupported coordinator primitive {primitive!r}")

if not re.search(pattern, masked, re.S):
    print(f"missing concrete {primitive} -> {target} binding", file=sys.stderr)
    raise SystemExit(2)

oracle_false = (
    f'fn example() {{\n'
    f'    // {primitive}({target});\n'
    f'    let text = "{primitive} EntryTypes::{target}";\n'
    f'}}'
)
oracle_masked = token_re.sub(lambda m: " " * len(m.group(0)), oracle_false)
assert not re.search(pattern, oracle_masked, re.S), (
    "coordinator target matcher accepted a comment/string false positive"
)

wrong_target = "__HEARTH_WRONG_TARGET__"
if wrong_target == target:
    wrong_target = "__HEARTH_WRONG_TARGET_2__"
if primitive in {"create_entry", "update_entry"}:
    oracle_wrong = (
        f'fn example() {{\n'
        f'    create_entry(&EntryTypes::{wrong_target}(Default::default()));\n',
        f'    update_entry(&EntryTypes::{wrong_target}(Default::default()));\n',
        f'}}'
    )
else:
    oracle_wrong = (
        f'fn example() {{\n'
        f'    create_link(&base, LinkTypes::{wrong_target}, tag);\n',
        f'}}'
    )
assert not re.search(pattern, oracle_wrong, re.S), (
    "coordinator target matcher accepted a concrete but wrong enum variant"
)

print(f"OK:   coordinator {primitive} is token-aware and binds concrete target {target}")
PY
    then
      echo "OK:   $id coordinator $operation binds concrete target $target_variant with token-aware matching"
    else
      echo "FAIL: $id coordinator $operation does not bind declared target $target_variant with token-aware matching"
      fail=1
    fi
    target_name="${target_variant#*::}"
    if [[ "$target_variant" == EntryTypes::* ]]; then
      if rg -nU --pcre2 "\bEntryTypes::${target_name}[[:space:]]*\(" "$integrity_file" >/dev/null 2>&1; then
        echo "OK:   $id integrity source contains concrete entry target $target_variant"
      else
        echo "FAIL: $id integrity source does not contain concrete entry target $target_variant"
        fail=1
      fi
      entry_enum="$(sed -n '/^[[:space:]]*pub[[:space:]]\+enum EntryTypes[[:space:]]*{/,/^}/p' "$integrity_file")"
      if printf '%s\n' "$entry_enum" | grep -Eq "^[[:space:]]*${target_name}\("; then
        echo "OK:   $id target $target_variant is declared by integrity EntryTypes"
      else
        echo "FAIL: $id target $target_variant is not declared by integrity EntryTypes"
        fail=1
      fi
      unset entry_enum
    else
      if rg -nU --pcre2 "\bLinkTypes::${target_name}\b" "$integrity_file" >/dev/null 2>&1; then
        echo "OK:   $id integrity source contains concrete link target $target_variant"
      else
        echo "FAIL: $id integrity source does not contain concrete link target $target_variant"
        fail=1
      fi
      link_enum="$(sed -n '/^[[:space:]]*pub[[:space:]]*enum LinkTypes[[:space:]]*{/,/^}/p' "$integrity_file")"
      if printf '%s\n' "$link_enum" | grep -Eq "^[[:space:]]*${target_name},$"; then
        echo "OK:   $id target $target_variant is declared by integrity LinkTypes"
      else
        echo "FAIL: $id target $target_variant is not declared by integrity LinkTypes"
        fail=1
      fi
      unset link_enum
    fi
    unset target_name

    # Bind the manifest target to the declared dispatch symbol in the same
    # authoritative validation arm. A file-wide dispatcher hit is too weak:
    # a sibling target could invoke the declared helper while this target routes
    # elsewhere.
    if python3 - "$integrity_file" "$coordinator_primitive" "$target_variant" "$dispatch_symbol" <<'PY'
import re
import sys
from pathlib import Path

source = Path(sys.argv[1]).read_text()
primitive, target, dispatch_symbol = sys.argv[2:]

token_re = re.compile(
    r'//[^\n]*'
    r'|/\*.*?\*/'
    r'|(?:br|rb|r)(#{0,255})"(?:.|\n)*?"\1'
    r'|"(?:\\.|[^"\\])*"'
    r"|b?'(?:\\.|[^'\\\n])'(?![A-Za-z0-9_])",
    re.S,
)

def mask(text):
    return token_re.sub(
        lambda m: "".join("\n" if c == "\n" else " " for c in m.group(0)),
        text,
    )

masked = mask(source)

def balanced_end(text, opening):
    depth = 0
    for i in range(opening, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError("unbalanced braces")

def function_block(symbol):
    match = re.search(
        rf"^\s*(?:pub\s+)?fn\s+{re.escape(symbol)}\s*\(",
        masked,
        re.M,
    )
    if not match:
        raise ValueError("missing function " + symbol)
    opening = masked.find("{", match.end())
    if opening < 0:
        raise ValueError("function has no body: " + symbol)
    return masked[match.start():balanced_end(masked, opening)]

def validation_arm(dispatcher, pattern, label):
    match = re.search(pattern, dispatcher, re.S)
    if not match:
        raise ValueError("missing validation arm: " + label)
    opening = match.end() - 1
    if dispatcher[opening] != "{":
        raise ValueError("validation arm match-body opening not found: " + label)
    return dispatcher[match.start():balanced_end(dispatcher, opening)]

def entry_arm(block, variant):
    match = re.search(
        rf"\bEntryTypes::{re.escape(variant)}\s*\([^)]*\)\s*=>",
        block,
    )
    if not match:
        return None
    start = match.end()
    while start < len(block) and block[start].isspace():
        start += 1
    if start < len(block) and block[start] == "{":
        return block[start:balanced_end(block, start)]
    comma = block.find(",", start)
    return block[start:] if comma < 0 else block[start:comma]

def link_arm(block, variant):
    match = re.search(
        rf"\bLinkTypes::{re.escape(variant)}\s*=>",
        block,
    )
    if not match:
        return None
    start = match.end()
    while start < len(block) and block[start].isspace():
        start += 1
    if start < len(block) and block[start] == "{":
        return block[start:balanced_end(block, start)]
    comma = block.find(",", start)
    return block[start:] if comma < 0 else block[start:comma]

def require_dispatch(arm, label):
    if not re.search(rf"\b{re.escape(dispatch_symbol)}\s*\(", arm):
        raise ValueError(label + " does not invoke " + dispatch_symbol)

try:
    target_name = target.split("::", 1)[1]

    if primitive == "create_link":
        policy = function_block(dispatch_symbol)
        target_arm = link_arm(policy, target_name)
        if target_arm is None:
            raise ValueError(
                "LinkTypes::" + target_name +
                " is not an explicit arm of " + dispatch_symbol
            )

    elif primitive in {"create_entry", "update_entry"}:
        if not target.startswith("EntryTypes::"):
            raise ValueError("entry primitive requires EntryTypes target")
        dispatcher = function_block("validate")
        if primitive == "create_entry":
            patterns = [
                (
                    r"FlatOp::CreateEntry\s*\(\s*"
                    r"OpEntry::CreateEntry\s*\{.*?\}\s*\)"
                    r"\s*=>\s*match\s+app_entry\s*\{",
                    "CreateEntry/CreateEntry",
                ),
                (
                    r"FlatOp::CreateRecord\s*\(\s*"
                    r"OpRecord::CreateEntry\s*\{.*?\}\s*\)"
                    r"\s*=>\s*match\s+app_entry\s*\{",
                    "CreateRecord/CreateEntry",
                ),
            ]
        else:
            patterns = [
                (
                    r"FlatOp::CreateEntry\s*\(\s*"
                    r"OpEntry::UpdateEntry\s*\{.*?\}\s*\)"
                    r"\s*=>\s*match\s+app_entry\s*\{",
                    "CreateEntry/UpdateEntry",
                ),
                (
                    r"FlatOp::CreateRecord\s*\(\s*"
                    r"OpRecord::UpdateEntry\s*\{.*?\}\s*\)"
                    r"\s*=>\s*match\s+app_entry\s*\{",
                    "CreateRecord/UpdateEntry",
                ),
            ]

        for pattern, label in patterns:
            block = validation_arm(dispatcher, pattern, label)
            arm = entry_arm(block, target_name)
            if arm is None:
                raise ValueError(label + " has no EntryTypes::" + target_name + " arm")
            require_dispatch(arm, label + " EntryTypes::" + target_name)

    else:
        raise ValueError("unsupported coordinator primitive " + primitive)

    # Adversarial oracle: the same dispatch symbol under a sibling target must
    # not satisfy the declared target witness.
    oracle = mask(
        "match app_entry {\\n"
        "  EntryTypes::__HEARTH_TARGET(_) => wrong_dispatch(),\\n"
        "  EntryTypes::Sibling(_) => " + dispatch_symbol + "(),\\n"
        "}"
    )
    oracle_arm = entry_arm(oracle, "__HEARTH_TARGET")
    if primitive != "create_link":
        assert oracle_arm is not None
    if primitive != "create_link":
        assert not re.search(
            rf"\b{re.escape(dispatch_symbol)}\s*\(",
            oracle_arm,
        ), "entry target witness accepted sibling-arm false positive"

    if primitive == "create_link":
        oracle_link = mask(
            "match link_type {\\n"
            "  LinkTypes::__HEARTH_TARGET => wrong_dispatch(),\\n"
            "  LinkTypes::Sibling => " + dispatch_symbol + "(),\\n"
            "}"
        )
        oracle_link_arm = link_arm(oracle_link, "__HEARTH_TARGET")
        assert oracle_link_arm is not None
        assert not re.search(
            rf"\b{re.escape(dispatch_symbol)}\s*\(",
            oracle_link_arm,
        ), "link target witness accepted sibling-arm false positive"

except (ValueError, AssertionError) as exc:
    print("FAIL: semantic target/dispatch binding: " + str(exc), file=sys.stderr)
    raise SystemExit(2)

print(
    "OK: semantic target "
    + target
    + " binds to dispatch "
    + dispatch_symbol
    + " within the authoritative validation arm(s)"
)
PY

    case "$coordinator_primitive" in
      create_entry)
        primitive_pattern="create_entry[[:space:]]*\\([^;]{0,160}EntryTypes::${target_variant#*::}"
        ;;
      update_entry)
        primitive_pattern="update_entry[[:space:]]*\\([^;]{0,280}EntryTypes::${target_variant#*::}"
        ;;
      create_link)
        primitive_pattern="create_link[[:space:]]*\\([^;]{0,280}LinkTypes::${target_variant#*::}"
        ;;
      *)
        echo "FAIL: $id has unsupported coordinator primitive $coordinator_primitive"
        fail=1
        primitive_pattern="a^"
        ;;
    esac
    if [[ -n "$coord_block" ]] && printf '%s\n' "$coord_block" | rg -nU --pcre2 "$primitive_pattern" >/dev/null 2>&1; then
      echo "OK:   $id coordinator $operation binds $coordinator_primitive to $target_variant"
    else
      echo "FAIL: $id coordinator $operation does not bind $coordinator_primitive to $target_variant"
      fail=1
    fi

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
      local validator_block dispatch_block
      validator_block="$(awk -v symbol="$validator_symbol" '
        {
          pattern = "^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+" symbol "[[:space:]]*\\("
          if (!in_block && $0 ~ pattern) {
            in_block = 1
            print
            next
          }
          if (in_block && /^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+[A-Za-z0-9_]+[[:space:]]*\\(/) {
            exit
          }
          if (in_block) print
        }
      ' "$validator_source")"
      if [[ -n "$validator_block" ]]; then
        echo "OK:   $id validator symbol $validator_symbol is defined in declared source"
      else
        echo "FAIL: $id validator symbol $validator_symbol is not defined in declared source"
        fail=1
      fi
      if [[ -n "$validator_block" ]] && python3 - "$invariant_code" "$rejection_reason" "$validator_block" <<'PY'
import re
import sys

code_pattern, expected_reason, source = sys.argv[1:]

# Remove comments and string literals before matching the executable predicate.
# This prevents a copied predicate in documentation, diagnostics, or examples
# from satisfying the semantic case. Preserve newlines so branch boundaries remain
# stable while braces inside masked tokens cannot forge a block boundary.
token_re = re.compile(
    r'//[^\n]*'
    r'|/\*.*?\*/'
    r'|(?:br|rb|r)(#{0,255})"(?:.|\n)*?"\1'
    r'|"(?:\\.|[^"\\])*"'
    # Mask character literals too: branch brace accounting must not treat
    # Rust char literals such as '{' or '}' as syntax delimiters. The negative
    # lookarounds keep Rust lifetimes like 'a from being classified as chars.
    r"|b?'(?:\\\\.|[^'\\\\\n])'(?![A-Za-z0-9_])",
    re.S,
)
# Preserve source length while masking comments, strings, and chars so indices
# in the masked source can safely slice the corresponding raw source branch.
masked = token_re.sub(
    lambda m: "".join("\n" if c == "\n" else " " for c in m.group(0)),
    source,
)

predicate = re.escape(code_pattern)
predicate_match = re.search(rf'\bif\s+{predicate}\s*\{{', masked)
if not predicate_match:
    print(
        f"FAIL: {code_pattern!r} is not an executable invariant predicate "
        f"inside the declared validator; manifest rejection_reason={expected_reason!r}"
    )
    raise SystemExit(2)

# The rejection proof must be branch-local. A file-wide Invalid/Err match is too
# weak: an unrelated invariant elsewhere in the validator could otherwise make
# this semantic case look executable while the declared predicate merely logs,
# computes, or returns Valid. Scan the masked branch with balanced-brace depth so
# nested blocks remain inside the same predicate branch.
open_brace = predicate_match.end() - 1
depth = 0
close_brace = None
for index in range(open_brace, len(masked)):
    char = masked[index]
    if char == "{":
        depth += 1
    elif char == "}":
        depth -= 1
        if depth == 0:
            close_brace = index
            break

if close_brace is None:
    print(
        f"FAIL: executable invariant predicate {code_pattern!r} has no "
        "balanced closing brace in the declared validator"
    )
    raise SystemExit(2)

predicate_branch = masked[open_brace + 1 : close_brace]
raw_predicate_branch = source[open_brace + 1 : close_brace]

# For these qualification cases, the manifest claims a definitive validation
# rejection, not a host failure. Require the predicate branch itself to return
# the Holochain Invalid result, and bind the exact rejection text to the manifest.
if not re.search(
    r'\breturn\s+Ok\s*\(\s*ValidateCallbackResult::Invalid\s*\(',
    predicate_branch,
):
    print(
        f"FAIL: executable invariant predicate {code_pattern!r} does not "
        "return ValidateCallbackResult::Invalid from its own branch"
    )
    raise SystemExit(2)

string_literals = [
    match.group(0)
    for match in token_re.finditer(raw_predicate_branch)
    if match.group(0).startswith('"')
]
if f'"{expected_reason}"' not in string_literals:
    print(
        f"FAIL: executable invariant predicate {code_pattern!r} does not "
        f"emit the exact manifest rejection reason {expected_reason!r} "
        "as a Rust string literal"
    )
    raise SystemExit(2)

# Adversarial oracle: reject the known false-positive shape where the predicate
# branch is non-rejecting but a later, unrelated branch returns Invalid.
oracle_valid = '''
fn validate_example(x: &Example) -> ExternResult<ValidateCallbackResult> {
    if x.field.is_empty() {
        println!("diagnostic only");
    }
    if x.other_bad {
        return Ok(ValidateCallbackResult::Invalid("unrelated".into()));
    }
    Ok(ValidateCallbackResult::Valid)
}
'''
oracle_invalid = '''
fn validate_example(x: &Example) -> ExternResult<ValidateCallbackResult> {
    if x.field.is_empty() {
        let brace = '{';
        let close = '}';
        let _lifetime_marker = PhantomData::<&'a ()>;
        return Ok(ValidateCallbackResult::Invalid("field cannot be empty".into()));
    }
    if x.other_bad {
        return Ok(ValidateCallbackResult::Invalid("unrelated".into()));
    }
}
'''

def branch_has_rejection(test_source, code, expected_reason=None):
    test_masked = token_re.sub(
        lambda m: "".join("\n" if c == "\n" else " " for c in m.group(0)),
        test_source,
    )
    match = re.search(rf'\bif\s+{re.escape(code)}\s*\{{', test_masked)
    if not match:
        return False
    depth = 0
    opening = match.end() - 1
    for index in range(opening, len(test_masked)):
        if test_masked[index] == "{":
            depth += 1
        elif test_masked[index] == "}":
            depth -= 1
            if depth == 0:
                branch = test_masked[opening + 1 : index]
                strict_invalid = re.search(
                    r'\breturn\s+Ok\s*\(\s*ValidateCallbackResult::Invalid\s*\(',
                    branch,
                )
                if not strict_invalid:
                    return False
                if expected_reason is None:
                    return True
                raw_branch = test_source[opening + 1 : index]
                literals = [
                    match.group(0)
                    for match in token_re.finditer(raw_branch)
                    if match.group(0).startswith('"')
                ]
                return f'"{expected_reason}"' in literals
    return False

assert not branch_has_rejection(oracle_valid, "x.field.is_empty()", "field cannot be empty"), (
    "branch-local oracle accepted unrelated Invalid rejection"
)
assert branch_has_rejection(
    oracle_invalid, "x.field.is_empty()", "field cannot be empty"
), "branch-local oracle rejected the genuine exact Invalid branch"
oracle_err = oracle_invalid.replace(
    'return Ok(ValidateCallbackResult::Invalid("field cannot be empty".into()));',
    'return Err("field cannot be empty".into());',
)
assert not branch_has_rejection(
    oracle_err, "x.field.is_empty()", "field cannot be empty"
), "branch-local oracle accepted Err as an Invalid qualification result"
oracle_wrong_reason = oracle_invalid.replace(
    '"field cannot be empty"',
    '"different reason"',
    1,
)
assert not branch_has_rejection(
    oracle_wrong_reason, "x.field.is_empty()", "field cannot be empty"
), "branch-local oracle accepted a mismatched rejection reason"

print(f"OK:   {code_pattern} is bound to a branch-local executable validator rejection path")
PY
      then
        echo "OK:   $id invariant predicate is executable inside declared validator $validator_symbol"
      else
        fail=1
      fi
      dispatch_block="$(awk '
        /^[[:space:]]*pub[[:space:]]+fn[[:space:]]+validate[[:space:]]*\\(/ {
          in_block=1
          print
          next
        }
        in_block && /^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+[A-Za-z0-9_]+[[:space:]]*\\(/ {
          exit
        }
        in_block { print }
      ' "$integrity_file")"
      if [[ -n "$dispatch_block" ]] && printf '%s\n' "$dispatch_block" | rg -nU --pcre2 "\\b${dispatch_symbol}[[:space:]]*\\(" >/dev/null 2>&1; then
        echo "OK:   $id integrity validate dispatcher invokes declared dispatch symbol $dispatch_symbol"
      else
        echo "FAIL: $id integrity validate dispatcher does not invoke declared dispatch symbol $dispatch_symbol"
        fail=1
      fi

      if [[ "$validator_source" != "$integrity_file" ]]; then
        wrapper_block="$(awk -v symbol="$dispatch_symbol" '
          {
            pattern = "^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+" symbol "[[:space:]]*\\("
            if (!in_block && $0 ~ pattern) {
              in_block = 1
              print
              next
            }
            if (in_block && /^[[:space:]]*(pub[[:space:]]+)?fn[[:space:]]+[A-Za-z0-9_]+[[:space:]]*\\(/) {
              exit
            }
            if (in_block) print
          }
        ' "$integrity_file")"
        if [[ -n "$wrapper_block" ]] && printf '%s\n' "$wrapper_block" | rg -nU --pcre2 "\\b${validator_symbol}[[:space:]]*\\(" >/dev/null 2>&1; then
          echo "OK:   $id dispatch wrapper $dispatch_symbol delegates to external validator $validator_symbol"
        else
          echo "FAIL: $id dispatch wrapper $dispatch_symbol does not directly delegate to external validator $validator_symbol"
          fail=1
        fi
        if python3 - "$validator_source" "$integrity_file" "$validator_symbol" <<'PY'
import json
import re
import subprocess
import sys
from pathlib import Path

validator_source = Path(sys.argv[1]).resolve()
integrity_source = Path(sys.argv[2]).resolve()
validator_symbol = sys.argv[3]
integrity_manifest = integrity_source.parent.parent / "Cargo.toml"

def fail(message):
    print(f"FAIL: {message}")
    raise SystemExit(2)

if not integrity_manifest.is_file():
    fail(f"integrity source has no owning Cargo.toml: {integrity_manifest}")

def nearest_manifest(path):
    for parent in [path.parent, *path.parents]:
        candidate = parent / "Cargo.toml"
        if candidate.is_file():
            return candidate.resolve()
    return None

validator_manifest = nearest_manifest(validator_source)
if validator_manifest is None:
    fail(f"external validator source has no owning Cargo.toml: {validator_source}")

metadata_cmd = [
    "cargo",
    "metadata",
    "--format-version",
    "1",
    "--locked",
    "--manifest-path",
    str(integrity_manifest),
]
try:
    metadata_run = subprocess.run(
        metadata_cmd,
        cwd=Path.cwd(),
        text=True,
        capture_output=True,
        check=False,
    )
except OSError as exc:
    fail(f"could not execute cargo metadata for validator provenance: {exc}")

if metadata_run.returncode != 0:
    diagnostic = (metadata_run.stderr or metadata_run.stdout or "").strip()
    fail(
        f"cargo metadata could not resolve {integrity_manifest} "
        f"(exit={metadata_run.returncode}): {diagnostic[-1200:]}"
    )

try:
    metadata = json.loads(metadata_run.stdout)
except json.JSONDecodeError as exc:
    fail(f"cargo metadata emitted invalid JSON: {exc}")

packages = metadata.get("packages")
resolve = metadata.get("resolve") or {}
nodes = resolve.get("nodes")
if not isinstance(packages, list) or not isinstance(nodes, list):
    fail("cargo metadata output is missing packages/resolve.nodes")

def canonical_manifest(pkg):
    raw = pkg.get("manifest_path")
    if not isinstance(raw, str):
        return None
    return Path(raw).resolve()

integrity_packages = [
    pkg for pkg in packages
    if canonical_manifest(pkg) == integrity_manifest.resolve()
]
validator_packages = [
    pkg for pkg in packages
    if canonical_manifest(pkg) == validator_manifest.resolve()
]

if len(integrity_packages) != 1:
    fail(
        f"cargo metadata resolved {len(integrity_packages)} packages for "
        f"{integrity_manifest}; expected exactly one"
    )
if len(validator_packages) != 1:
    fail(
        f"cargo metadata resolved {len(validator_packages)} packages for "
        f"{validator_manifest}; expected exactly one"
    )

integrity_package = integrity_packages[0]
validator_package = validator_packages[0]
validator_package_id = validator_package.get("id")
integrity_package_id = integrity_package.get("id")
if not isinstance(validator_package_id, str) or not isinstance(integrity_package_id, str):
    fail("cargo metadata package IDs are missing")

lib_targets = [
    target for target in validator_package.get("targets", [])
    if isinstance(target, dict) and "lib" in (target.get("kind") or [])
]
if len(lib_targets) != 1:
    fail(
        f"validator package {validator_package.get('name')} has "
        f"{len(lib_targets)} library targets; expected exactly one"
    )

lib_target = lib_targets[0]
lib_source = Path(lib_target.get("src_path", "")).resolve()
if lib_source != validator_source:
    fail(
        f"declared validator source {validator_source} is not Cargo's library "
        f"target source {lib_source}"
    )

root_node = next(
    (node for node in nodes if node.get("id") == integrity_package_id),
    None,
)
if root_node is None:
    fail(
        f"cargo metadata resolve graph has no node for integrity package "
        f"{integrity_package.get('name')}"
    )

normal_deps = []
for dep in root_node.get("deps", []):
    if not isinstance(dep, dict):
        continue
    dep_kinds = dep.get("dep_kinds") or []
    if any(
        isinstance(kind, dict) and kind.get("kind") in (None, "normal")
        for kind in dep_kinds
    ):
        normal_deps.append(dep)

matching = [
    dep for dep in normal_deps
    if dep.get("pkg") == validator_package_id
]

if len(matching) != 1:
    fail(
        f"validator package {validator_package.get('name')} is not a unique "
        f"normal direct dependency of {integrity_package.get('name')}; "
        f"matching dependency edges={len(matching)}"
    )

dependency = matching[0]
imported_crate = dependency.get("name")
cargo_crate_name = lib_target.get("name")
if not isinstance(imported_crate, str) or not imported_crate:
    fail("cargo metadata did not report the dependency library target name")
if imported_crate != cargo_crate_name:
    fail(
        f"Cargo resolved dependency crate {imported_crate!r}, but validator "
        f"library target is {cargo_crate_name!r}"
    )

integrity_text = integrity_source.read_text()
integrity_prod = integrity_text.split("#[cfg(test)]", 1)[0]

use_statements = re.findall(
    r"(?ms)^[[:space:]]*(?:pub[[:space:]]+)?use[[:space:]]+[^;]+;",
    integrity_prod,
)
crate_pattern = re.escape(imported_crate)
symbol_pattern = re.escape(validator_symbol)

direct_use = re.compile(
    rf"(?ms)^[[:space:]]*(?:pub[[:space:]]+)?use[[:space:]]+"
    rf"{crate_pattern}::{re.escape(validator_symbol)}[[:space:]]*;"
)
group_use = re.compile(
    rf"(?ms)^[[:space:]]*(?:pub[[:space:]]+)?use[[:space:]]+"
    rf"{crate_pattern}::\{{[^;]*\}}[[:space:]]*;"
)

matching_uses = []
for statement in use_statements:
    normalized = " ".join(statement.split())
    if not re.search(rf"\b{crate_pattern}::", normalized):
        continue

    if direct_use.fullmatch(normalized):
        matching_uses.append(normalized)
        continue

    group_match = group_use.fullmatch(normalized)
    if not group_match:
        continue

    body = group_match.group(0)
    brace_start = body.find("{")
    brace_end = body.rfind("}")
    items = [item.strip() for item in body[brace_start + 1:brace_end].split(",")]
    aliases = [
        item for item in items
        if re.fullmatch(
            rf"{re.escape(validator_symbol)}[[:space:]]+as[[:space:]]+[A-Za-z_][A-Za-z0-9_]*",
            item,
        )
    ]
    if aliases:
        fail(
            f"{integrity_source} aliases {validator_symbol} instead of directly "
            "binding the validator symbol"
        )
    if validator_symbol in items:
        matching_uses.append(normalized)

if not matching_uses:
    fail(
        f"{integrity_source} does not directly import "
        f"{imported_crate}::{validator_symbol} from Cargo's resolved dependency"
    )

conflicting_uses = []
for statement in use_statements:
    normalized = " ".join(statement.split())
    if not re.search(rf"\b{re.escape(validator_symbol)}\b", normalized):
        continue
    if not re.search(rf"\b{crate_pattern}::", normalized):
        conflicting_uses.append(normalized)

if conflicting_uses:
    fail(
        f"{integrity_source} contains additional imports of validator symbol "
        f"{validator_symbol} outside resolved provenance: {' | '.join(conflicting_uses)}"
    )

local_def_re = re.compile(
    rf"(?m)^[[:space:]]*(?:pub[[:space:]]+)?"
    rf"(?:async[[:space:]]+)?(?:fn|const|static|struct|enum|type|mod)[[:space:]]+"
    rf"{re.escape(validator_symbol)}\b"
)
if local_def_re.search(integrity_prod):
    fail(
        f"{integrity_source} locally defines {validator_symbol}; "
        "external validator provenance would be ambiguous"
    )

print(
    f"OK:   external validator {validator_package.get('name')}::{validator_symbol} "
    f"is bound by Cargo to {validator_source} via crate {imported_crate}"
)
PY
        then
          echo "OK:   $id external validator ownership/import provenance"
        else
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
  if [[ ! -f ".github/workflows/hearth-07-workflow-lint.yml" ]]; then
    echo "FAIL: missing independent Hearth 0.7 workflow-lint workflow"
    fail=1
  elif rg -n --fixed-strings 'version="1.7.12"' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings '8aca8db96f1b94770f1b0d72b6dddcb1ebb8123cb3712530b08cc387b349a3d8' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings 'https://github.com/rhysd/actionlint/releases/download/v${version}/actionlint_${version}_linux_amd64.tar.gz' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings '.github/workflows/hearth-07-qualification.yml' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings 'actions/checkout@d23441a48e516b6c34aea4fa41551a30e30af803' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1; then
    echo "OK:   independent qualification workflow lint verifies the official actionlint release by SHA-256"
  else
    echo "FAIL: independent qualification workflow lint must pin and hash-lock official actionlint"
    fail=1
  fi
  if [[ ! -f "$workflow" ]]; then
    echo "FAIL: missing Hearth 0.7 qualification workflow"
    fail=1
    return
  fi
  if [[ ! -f ".github/workflows/hearth-07-workflow-lint.yml" ]]; then
    echo "FAIL: missing independent Hearth 0.7 workflow-lint workflow"
    fail=1
  elif rg -n --fixed-strings "raven-actions/actionlint@3d39aea434753780c3b3d4a1a31c854b4dbf49d7" ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings 'version: "1.7.12"' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings 'files: ".github/workflows/hearth-07-qualification.yml"' ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1     && rg -n --fixed-strings "actions/checkout@d23441a48e516b6c34aea4fa41551a30e30af803" ".github/workflows/hearth-07-workflow-lint.yml" >/dev/null 2>&1; then
    echo "OK:   independent qualification workflow lint is present, pinned, and targets the qualification workflow"
  else
    echo "FAIL: independent qualification workflow lint must be pinned and target hearth-07-qualification.yml"
    fail=1
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

  action_use_count="$(rg -nU --pcre2 '^[[:space:]]*(?:-[[:space:]]+)?uses:' "$workflow" | wc -l)"
  pinned_action_count="$(rg -nU --pcre2 '^[[:space:]]*(?:-[[:space:]]+)?uses:[[:space:]]+[^[:space:]@]+@[0-9a-f]{40}[[:space:]]*(#.*)?$' "$workflow" | wc -l)"
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

  if rg -n --fixed-strings "Verify every semantic qualification case executed" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "semantic qualification case was still ignored" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "semantic qualification case was not observed in test output" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "cargo test summary reports ignored tests despite --include-ignored" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification workflow proves every manifest case actually executed and was not merely collected/skipped"
  else
    echo "FAIL: qualification workflow must prove every manifest semantic case executed and passed"
    fail=1
  fi


  if python3 - "$workflow" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_text()
verify = source.find("      - name: Verify every semantic qualification case executed")
capture = source.find("      - name: Capture immutable qualification evidence")
sweettest = source.find("      - name: Run Hearth 0.7 SweetConductor qualification")
metadata = source.find("      - name: Capture qualification metadata")

if not (0 <= sweettest < verify < capture < metadata):
    print("FAIL: semantic execution verification must follow SweetConductor and precede immutable evidence/metadata capture")
    raise SystemExit(2)

capture_end = source.find("\n      - name:", capture + 1)
if capture_end < 0:
    capture_end = len(source)
capture_block = source[capture:capture_end]

if 'qualification-semantic-execution.txt" "unavailable_case_execution_not_reached"' not in capture_block:
    print("FAIL: immutable evidence capture does not preserve semantic execution receipt")
    raise SystemExit(2)

if 'grep -qx "status=passed" "${hearth}/qualification-semantic-execution.txt"' not in source:
    print("FAIL: qualification completeness is not gated on a passed semantic execution receipt")
    raise SystemExit(2)

print("OK: semantic execution verification precedes evidence capture and gates completeness")
PY
  then
    true
  else
    fail=1
  fi

  if rg -n --fixed-strings "Capture immutable qualification evidence" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "if: ${{ !cancelled() }}" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "unavailable_source_audit_not_reached" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "qualification-evidence-status.txt" "$workflow" >/dev/null 2>&1     && rg -n --fixed-strings "source_contract_digest=unavailable" "$workflow" >/dev/null 2>&1; then
    echo "OK:   qualification evidence capture is failure-monotonic with explicit unavailable markers"
  else
    echo "FAIL: qualification evidence capture must preserve artifacts across early failures"
    fail=1
  fi

  if python3 - "$workflow" <<'PY'
import re, sys
from pathlib import Path

path = Path(sys.argv[1])
source = path.read_text()
push_start = source.find("  push:")
pr_start = source.find("  pull_request:")
wd_start = source.find("  workflow_dispatch:")
if min(push_start, pr_start, wd_start) < 0:
    print("FAIL: qualification workflow trigger sections are incomplete")
    raise SystemExit(2)
push_block = source[push_start:pr_start]
pr_block = source[pr_start:wd_start]
for label, block in [("push", push_block), ("pull_request", pr_block)]:
    for required in ['      - ".gitmodules"', '      - "mycelix-health/**"']:
        if required not in block:
            print(f"FAIL: qualification {label} trigger omits provenance-sensitive path {required}")
            raise SystemExit(2)
print("OK: qualification workflow triggers on .gitmodules and mycelix-health gitlink changes")
PY
  then
    true
  else
    fail=1
  fi

  if python3 - "$workflow" <<'PY'
import re, sys
from pathlib import Path

path = Path(sys.argv[1])
source = path.read_text()
start = source.find("      - name: Capture immutable qualification evidence")
end = source.find("      - name: Capture qualification metadata", start)
if start < 0 or end < 0:
    raise SystemExit("capture step boundaries not found")
block = source[start:end]

checks = [
    ('root workspace capture', r'root="\$\{GITHUB_WORKSPACE\}"'),
    ('non-cancelled evidence condition', r'if: \$\{\{ !cancelled\(\) \}\}'),
    ('marker helper', r'ensure_marker\(\)'),
    ('non-destructive marker creation', r'if \[\[ ! -s "\$path" \]\]'),
    ('root-level source-contract verification', r'\(cd "\$\{root\}" && sha256sum -c mycelix-workspace/mycelix-hearth/qualification-source-contract-sha256\.txt\)'),
    ('evidence status artifact', r'qualification-evidence-status\.txt'),
]
for label, pattern in checks:
    if not re.search(pattern, block):
        print(f"FAIL: qualification evidence capture missing {label}")
        raise SystemExit(2)

if re.search(r'working-directory:\s*mycelix-workspace/mycelix-hearth', block):
    print("FAIL: evidence capture must not depend on Hearth working-directory existing")
    raise SystemExit(2)

print("OK: qualification evidence capture is root-anchored and non-destructive")
PY
  then
    true
  else
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
    if printf '%s\n' "$source" | rg -nU --pcre2 '\bcreate_entry\s*\(' >/dev/null 2>&1; then
      if rg -nU --pcre2 'FlatOp::CreateEntry\s*\(' "$file" >/dev/null 2>&1 \
        && rg -nU --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::CreateEntry' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator entry writes map to CreateEntry + CreateRecord validation"
      else
        echo "FAIL: $zome coordinator calls create_entry but integrity lacks paired 0.7 create validation"
        fail=1
      fi
    fi

    # Mutable entry writes need content validation, CreateRecord parity, and
    # action-level author authorization.
    if printf '%s\n' "$source" | rg -nU --pcre2 '\bupdate_entry\s*\(' >/dev/null 2>&1; then
      if rg -nU --pcre2 'FlatOp::CreateEntry\s*\(\s*OpEntry::UpdateEntry' "$file" >/dev/null 2>&1 \
        && rg -nU --pcre2 'FlatOp::CreateRecord\s*\(\s*OpRecord::UpdateEntry' "$file" >/dev/null 2>&1 \
        && rg -nU --pcre2 'FlatOp::Update\s*\(\s*OpUpdate::Entry' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator update_entry maps to content + CreateRecord + Update authorization"
      else
        echo "FAIL: $zome coordinator calls update_entry without complete 0.7 update validation coverage"
        fail=1
      fi
    fi

    if printf '%s\n' "$source" | rg -nU --pcre2 '\bdelete_entry\s*\(' >/dev/null 2>&1; then
      if rg -nU --pcre2 'FlatOp::Delete\s*\(\s*OpDelete\s*\{\s*action' "$file" >/dev/null 2>&1 \
        && rg -nU --pcre2 'must_get_valid_record\s*\(\s*action\.deletes_address' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator delete_entry maps to validated Delete authorization"
      else
        echo "FAIL: $zome coordinator calls delete_entry without validated Delete authorization coverage"
        fail=1
      fi
    fi

    if printf '%s\n' "$source" | rg -nU --pcre2 '\bcreate_link\s*\(' >/dev/null 2>&1; then
      if rg -nU --pcre2 'FlatOp::Link\s*\(\s*OpLink::CreateLink' "$file" >/dev/null 2>&1; then
        echo "OK:   $zome coordinator create_link maps to explicit CreateLink validation"
      else
        echo "FAIL: $zome coordinator calls create_link but integrity lacks explicit CreateLink validation"
        fail=1
      fi
    fi

    if printf '%s\n' "$source" | rg -nU --pcre2 '\bdelete_link\s*\(' >/dev/null 2>&1; then
      if rg -nU --pcre2 'FlatOp::Link\s*\([^)]*OpLink::DeleteLink' "$file" >/dev/null 2>&1 \
        && rg -nU --pcre2 'must_get_valid_record\s*\(\s*action\.link_add_address' "$file" >/dev/null 2>&1; then
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
  local started finished status=0 tee_status=0 trace_dir="" trace_file="" fifo_file="" tee_pid=""
  started="$(date +%s)"
  echo "AUDIT_START ${name} epoch=${started}"

  # Harness setup failures are audit failures too. Keep them inside the
  # aggregate failure contract instead of allowing set -e to abort the whole
  # script before the failure summary and later predicates are reached.
  if ! trace_dir="$(mktemp -d)"; then
    echo "FAIL: ${name} audit harness could not create a temporary directory"
    fail=1
    finished="$(date +%s)"
    echo "AUDIT_END ${name} status=1 duration=$((finished - started))s"
    return 0
  fi
  trace_file="${trace_dir}/predicate.log"
  fifo_file="${trace_dir}/predicate.fifo"
  if ! mkfifo "${fifo_file}"; then
    echo "FAIL: ${name} audit harness could not create its diagnostic FIFO"
    rm -rf "${trace_dir}"
    fail=1
    finished="$(date +%s)"
    echo "AUDIT_END ${name} status=1 duration=$((finished - started))s"
    return 0
  fi

  # Launch the tee directly; its exit status is made authoritative by wait.
  # This avoids racing the FIFO with a background if/then compound.
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
  # independently of each predicate’s local control flow.
  if [[ -f "${trace_file}" ]] && grep -qE "^FAIL:" "${trace_file}"; then
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
  # Individual predicate failures are aggregated through the global fail
  # accumulator; they must not trigger errexit before later diagnostics run.
  return 0
}
run_audit_check check_standalone_tests_workspace_boundary check_standalone_tests_workspace_boundary
run_audit_check check_qualification_workflow_provenance check_qualification_workflow_provenance
run_audit_check check_coordinator_operation_bindings check_coordinator_operation_bindings
run_audit_check check_coordinator_symbol_parity check_coordinator_symbol_parity
run_audit_check check_dna_source_completeness check_dna_source_completeness
run_audit_check check_semantic_validation_suite_wiring check_semantic_validation_suite_wiring
run_audit_check check_semantic_manifest_schema check_semantic_manifest_schema
run_audit_check check_semantic_case_entrypoints check_semantic_case_entrypoints
run_audit_check check_semantic_case_integrity_bindings check_semantic_case_integrity_bindings

for file in "${integrity_files[@]}"; do
  run_audit_check "check_create_record_coverage:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_create_record_coverage "$file"
done

for file in "${integrity_files[@]}"; do
  run_audit_check "check_dangerous_operation_catchalls:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_dangerous_operation_catchalls "$file"
done


for file in "${integrity_files[@]}"; do
  run_audit_check "check_create_entry_entry_dispatch:$(basename "$(dirname "$(dirname "$(dirname "$file")")")")" check_create_entry_entry_dispatch "$file"
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
