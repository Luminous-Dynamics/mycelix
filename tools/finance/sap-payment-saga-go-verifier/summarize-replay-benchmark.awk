/^RUST_REPLAY_SAMPLE / {
    events = ""
    ns = ""
    for (i = 1; i <= NF; i++) {
        if ($i ~ /^events=/) {
            events = substr($i, 8) + 0
        }
        if ($i ~ /^ns_per_op=/) {
            ns = substr($i, 11) + 0
        }
    }
    if (events != "" && ns != "") {
        key = "rust_events_" events
        rust_count[key]++
        rust_latency[key, rust_count[key]] = ns
    }
}

/^BenchmarkReplayJournal\/events_[0-9]+-/ {
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
        go_count[key]++
        go_latency[key, go_count[key]] = ns + 0
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
    split("100 1000 10000", sizes, " ")
    for (s = 1; s <= 3; s++) {
        size = sizes[s]
        key = "rust_events_" size
        n = rust_count[key] + 0
        if (n != 30) {
            printf "Rust expected 30 samples for %s events; got %d\n", size, n > "/dev/stderr"
            failed = 1
        } else {
            for (i in values) delete values[i]
            for (i = 1; i <= n; i++) values[i] = rust_latency[key, i]
            sort_samples(values, n)
            printf "Rust replay events=%s samples=%d p50_ns/op=%.3f p95_ns/op=%.3f p99_ns/op=%.3f\n", size, n, values[percentile_rank(50,n)], values[percentile_rank(95,n)], values[percentile_rank(99,n)]
        }
    }

    for (key in go_count) {
        n = go_count[key]
        if (n != 30) {
            printf "Go expected 30 samples for %s; got %d\n", key, n > "/dev/stderr"
            failed = 1
            continue
        }
        for (i in values) delete values[i]
        for (i = 1; i <= n; i++) values[i] = go_latency[key, i]
        sort_samples(values, n)
        printf "Go replay %s samples=%d p50_ns/op=%.3f p95_ns/op=%.3f p99_ns/op=%.3f mean_B/op=%.1f mean_allocs/op=%.1f\n", key, n, values[percentile_rank(50,n)], values[percentile_rank(95,n)], values[percentile_rank(99,n)], bytes_sum[key] / n, allocs_sum[key] / n
    }

    for (s = 1; s <= 3; s++) {
        size = sizes[s]
        prefix = "BenchmarkReplayJournal/events_" size "-"
        n = 0
        for (key in go_count) {
            if (index(key, prefix) == 1) n += go_count[key]
        }
        if (n != 30) {
            printf "Go expected 30 total samples for %s events; got %d\n", size, n > "/dev/stderr"
            failed = 1
        }
    }
    if (failed) exit 1
}
