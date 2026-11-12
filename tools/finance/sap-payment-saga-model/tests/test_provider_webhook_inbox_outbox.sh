#!/usr/bin/env bash
set -euo pipefail

: "$PGHOST"
: "$PGPORT"
: "$PGUSER"
: "$PGDATABASE"
: "$PGPASSWORD"

accept_event() {
    local event_id="$1"
    local body_digest="$2"
    local work_id="$3"
    local ciphertext_hex="$4"
    psql -X -q -v ON_ERROR_STOP=1 -At <<SQL
SELECT accept_provider_webhook_v1(
    'integration-provider',
    'integration-endpoint',
    'integration-company-account',
    'us',
    'integration-resource-account',
    'test',
    '$event_id',
    'refund.updated',
    're_test_provider_object',
    '2025-03-31',
    'snapshot',
    NULL,
    1794000000,
    'stripe-v1',
    'integration-webhook-secret-v3',
    'stripe-verifier-v1',
    'sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa',
    '$body_digest',
    decode('$ciphertext_hex', 'hex'),
    'kms:integration-test-key-v1',
    clock_timestamp(),
    '$work_id'
);
SQL
}

expect_status() {
    local expected="$1"
    local actual="$2"
    local label="$3"
    if [[ "$actual" != "$expected" ]]; then
        printf 'FAIL %s: expected=%s actual=%s\n' "$label" "$expected" "$actual" >&2
        exit 1
    fi
    printf 'PASS %s: %s\n' "$label" "$actual"
}

assert_sql() {
    local query="$1"
    local expected="$2"
    local label="$3"
    local actual
    actual="$(psql -X -q -v ON_ERROR_STOP=1 -At -c "$query")"
    expect_status "$expected" "$actual" "$label"
}

psql -X -q -v ON_ERROR_STOP=1 -f \
    tools/finance/sap-payment-saga-model/migrations/0001_provider_webhook_inbox.sql

EVENT_ID="evt_integration_exact_duplicate"
WORK_ID="refresh-work:integration-exact-duplicate"
BODY_A="sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
BODY_B="sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"

expect_status accepted "$(accept_event "$EVENT_ID" "$BODY_A" "$WORK_ID" "01020304")" "first delivery accepted"
expect_status duplicate "$(accept_event "$EVENT_ID" "$BODY_A" "$WORK_ID" "01020304")" "exact retry is duplicate"
expect_status conflict "$(accept_event "$EVENT_ID" "$BODY_B" "$WORK_ID" "05060708")" "same ID changed body is quarantined"

assert_sql "
SELECT count(*) FROM provider_webhook_inbox
 WHERE provider_id='integration-provider' AND event_id='$EVENT_ID';
" "1" "one canonical inbox event"

assert_sql "
SELECT count(*) FROM provider_webhook_conflict
 WHERE provider_id='integration-provider' AND event_id='$EVENT_ID';
" "1" "one changed-body conflict"

assert_sql "
SELECT count(*) FROM provider_webhook_outbox work
 JOIN provider_webhook_inbox event USING (inbox_id)
 WHERE event.provider_id='integration-provider' AND event.event_id='$EVENT_ID';
" "1" "one outbox work row"

assert_sql "
SELECT string_agg(disposition, ',' ORDER BY delivery_id)
 FROM provider_webhook_delivery
 WHERE provider_id='integration-provider' AND event_id='$EVENT_ID';
" "accepted,duplicate,conflict" "every authenticated delivery is preserved"

assert_sql "
SELECT requested_generation FROM provider_webhook_object_reconciliation
 WHERE provider_id='integration-provider'
   AND event_object_id='re_test_provider_object';
" "1" "duplicate and conflict do not advance object generation"

CONCURRENT_EVENT="evt_integration_concurrent_duplicate"
CONCURRENT_WORK="refresh-work:integration-concurrent"
CONCURRENT_N=8
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

for index in $(seq 1 "$CONCURRENT_N"); do
    (
        accept_event "$CONCURRENT_EVENT" "$BODY_A" "$CONCURRENT_WORK" "01020304" \
            > "$tmpdir/result-$index.txt"
    ) &
done
wait

accepted=0
duplicates=0
for result in "$tmpdir"/result-*.txt; do
    value="$(tr -d '\r\n' < "$result")"
    case "$value" in
        accepted) accepted=$((accepted + 1)) ;;
        duplicate) duplicates=$((duplicates + 1)) ;;
        *) printf 'FAIL unexpected concurrent disposition: %s\n' "$value" >&2; exit 1 ;;
    esac
done

[[ "$accepted" -eq 1 ]] || { printf 'FAIL concurrent accepted count=%s\n' "$accepted" >&2; exit 1; }
[[ "$duplicates" -eq $((CONCURRENT_N - 1)) ]] || {
    printf 'FAIL concurrent duplicate count=%s\n' "$duplicates" >&2
    exit 1
}
printf 'PASS concurrent deliveries: accepted=%s duplicates=%s\n' "$accepted" "$duplicates"

assert_sql "
SELECT count(*) FROM provider_webhook_inbox
 WHERE provider_id='integration-provider' AND event_id='$CONCURRENT_EVENT';
" "1" "concurrent race creates one inbox event"

assert_sql "
SELECT count(*) FROM provider_webhook_outbox work
 JOIN provider_webhook_inbox event USING (inbox_id)
 WHERE event.provider_id='integration-provider' AND event.event_id='$CONCURRENT_EVENT';
" "1" "concurrent race creates one outbox row"

assert_sql "
SELECT count(*) FROM provider_webhook_delivery
 WHERE provider_id='integration-provider' AND event_id='$CONCURRENT_EVENT';
" "$CONCURRENT_N" "all concurrent deliveries have audit receipts"

assert_sql "
SELECT requested_generation FROM provider_webhook_object_reconciliation
 WHERE provider_id='integration-provider'
   AND event_object_id='re_test_provider_object';
" "2" "one concurrent event creates one additional refresh generation"

printf 'PASS provider webhook inbox/outbox PostgreSQL integration suite\n'
