/^BenchmarkOutbox(ClaimDue|Metrics)\/items_[0-9]+-/ {
    key = $1
    ns = ""
    bytes = ""
    allocs = ""
    for (i = 1; i <= NF; i++) {
        if ($i == "ns/op") ns = $(i - 1)
        if ($i == "B/op") bytes = $(i - 1)
        if ($i == "allocs/op") allocs = $(i - 1)
    }
    if (ns != "" && bytes != "" && allocs != "") {
        count[key]++
        latency[key, count[key]] = ns + 0
        bytes_sum[key] += bytes
        allocs_sum[key] += allocs
    }
}

function sort_samples(values, n,    i, j, value) {
    for (i = 2; i <= n; i++) {
        value = values[i]
        j = i - 1
        while (j >= 1 && values[j] > value) {
            values[j + 1] = values[j]
            j--
        }
        values[j + 1] = value
    }
}

function percentile_rank(p, n) {
    return int((p * n + 99) / 100)
}

END {
    split("BenchmarkOutboxClaimDue BenchmarkOutboxMetrics", operations, " ")
    split("1000 10000 100000", sizes, " ")
    failed = 0
    for (o = 1; o <= 2; o++) {
        for (s = 1; s <= 3; s++) {
            key = operations[o] "/items_" sizes[s] "-8"
            actual_key = ""
            for (candidate in count) {
                prefix = operations[o] "/items_" sizes[s] "-"
                if (index(candidate, prefix) == 1) actual_key = candidate
            }
            n = count[actual_key] + 0
            if (n != 30) {
                printf "expected 30 samples for %s; got %d\n", prefix, n > "/dev/stderr"
                failed = 1
                continue
            }
            for (i in values) delete values[i]
            for (i = 1; i <= n; i++) values[i] = latency[actual_key, i]
            sort_samples(values, n)
            printf "Go outbox %s samples=%d p50_ns/op=%.3f p95_ns/op=%.3f p99_ns/op=%.3f mean_B/op=%.1f mean_allocs/op=%.1f\n", actual_key, n, values[percentile_rank(50,n)], values[percentile_rank(95,n)], values[percentile_rank(99,n)], bytes_sum[actual_key] / n, allocs_sum[actual_key] / n
        }
    }
    if (failed) exit 1
}
