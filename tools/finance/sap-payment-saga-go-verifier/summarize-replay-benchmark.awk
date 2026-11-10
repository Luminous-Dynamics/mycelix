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
        count[key]++
        latency[key, count[key]] = ns + 0
        bytes_sum[key] += bytes
        allocs_sum[key] += allocs
        if (key ~ /events_100-/) samples_100++
        if (key ~ /events_1000-/) samples_1000++
        if (key ~ /events_10000-/) samples_10000++
    }
}
END {
    if (samples_100 != 30 || samples_1000 != 30 || samples_10000 != 30) {
        printf "expected 30 samples per history size; got 100=%d 1000=%d 10000=%d\n", samples_100, samples_1000, samples_10000 > "/dev/stderr"
        exit 1
    }
    for (key in count) {
        n = count[key]
        if (n != 30) {
            printf "%s expected 30 samples, got %d\n", key, n > "/dev/stderr"
            exit 1
        }
        for (i = 2; i <= n; i++) {
            value = latency[key, i]
            j = i - 1
            while (j >= 1 && latency[key, j] > value) {
                latency[key, j + 1] = latency[key, j]
                j--
            }
            latency[key, j + 1] = value
        }
        p50 = int((50 * n + 99) / 100)
        p95 = int((95 * n + 99) / 100)
        p99 = int((99 * n + 99) / 100)
        printf "%s samples=%d p50_ns=%g p95_ns=%g p99_ns=%g mean_B_per_op=%.1f mean_allocs_per_op=%.1f\n", key, n, latency[key, p50], latency[key, p95], latency[key, p99], bytes_sum[key] / n, allocs_sum[key] / n
    }
}
