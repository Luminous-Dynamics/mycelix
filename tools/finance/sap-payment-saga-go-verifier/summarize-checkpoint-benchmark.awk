/^RUST_CHECKPOINT_SAMPLE / {
    owner = ""
    checkpoints = ""
    ns = ""
    for (i = 1; i <= NF; i++) {
        if ($i ~ /^owner=/) owner = substr($i, 7)
        if ($i ~ /^checkpoints=/) checkpoints = substr($i, 12) + 0
        if ($i ~ /^ns_per_checkpoint=/) ns = substr($i, 19) + 0
    }
    if (owner != "" && checkpoints != "" && ns != "") {
        key = owner "_" checkpoints
        rust_count[key]++
        rust_latency[key, rust_count[key]] = ns
    }
}

/^BenchmarkCheckpointDigest\/(payments|treasury)_[0-9]+-/ {
    name = $1
    sub(/^BenchmarkCheckpointDigest\//, "", name)
    sub(/-[0-9]+$/, "", name)
    split(name, parts, "_")
    owner = parts[1]
    checkpoints = parts[2] + 0
    ns = ""
    bytes = ""
    allocs = ""
    for (i = 1; i <= NF; i++) {
        if ($i == "ns/op") ns = $(i - 1)
        if ($i == "B/op") bytes = $(i - 1)
        if ($i == "allocs/op") allocs = $(i - 1)
    }
    if (ns != "" && bytes != "" && allocs != "" && checkpoints > 0) {
        key = owner "_" checkpoints
        go_count[key]++
        go_latency[key, go_count[key]] = (ns + 0) / checkpoints
        go_bytes[key] += (bytes + 0) / checkpoints
        go_allocs[key] += (allocs + 0) / checkpoints
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
    split("payments treasury", owners, " ")
    split("100 1000 10000", sizes, " ")
    for (o = 1; o <= 2; o++) {
        for (s = 1; s <= 3; s++) {
            key = owners[o] "_" sizes[s]
            n = rust_count[key] + 0
            if (n != 30) {
                printf "Rust expected 30 samples for %s; got %d\n", key, n > "/dev/stderr"
                failed = 1
            } else {
                for (i in values) delete values[i]
                for (i = 1; i <= n; i++) values[i] = rust_latency[key, i]
                sort_samples(values, n)
                printf "Rust checkpoint %s samples=%d p50_ns/checkpoint=%.3f p95_ns/checkpoint=%.3f p99_ns/checkpoint=%.3f\n", key, n, values[percentile_rank(50,n)], values[percentile_rank(95,n)], values[percentile_rank(99,n)]
            }

            n = go_count[key] + 0
            if (n != 30) {
                printf "Go expected 30 samples for %s; got %d\n", key, n > "/dev/stderr"
                failed = 1
            } else {
                for (i in values) delete values[i]
                for (i = 1; i <= n; i++) values[i] = go_latency[key, i]
                sort_samples(values, n)
                printf "Go checkpoint %s samples=%d p50_ns/checkpoint=%.3f p95_ns/checkpoint=%.3f p99_ns/checkpoint=%.3f mean_B/checkpoint=%.2f mean_allocs/checkpoint=%.4f\n", key, n, values[percentile_rank(50,n)], values[percentile_rank(95,n)], values[percentile_rank(99,n)], go_bytes[key] / n, go_allocs[key] / n
            }
        }
    }
    if (failed) exit 1
}
