# Preferential-flow eventizer v0.1

This standalone Rust CLI converts already QA-filtered and 10-minute-normalized rainfall and soil-moisture series into event/sensor response records. It is an executable preprocessing contract, not a preferential-flow ground-truth detector or hydraulic simulator.

## Run

    cargo test --manifest-path research/terra-preta/tools/preferential-flow-eventizer/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/preferential-flow-eventizer/Cargo.toml -- \
      research/terra-preta/tools/preferential-flow-eventizer/fixtures/synthetic-one-event.json \
      /tmp/terra-preta-pf-events.json

## Encoded event semantics

- Input series are UTC-aligned, gap-free 10-minute bins. Missing time bins are rejected rather than silently interpreted as dry periods.
- Soil moisture must already have passed product-specific QA filtering and 10-minute median aggregation. Interpolated gaps shorter than six hours must be marked upstream; missing bins before response yield an indeterminate result.
- Nonzero 10-minute precipitation bins start events. Pulses separated by up to six hours are grouped; events split after a five-day maximum duration.
- Peak 10-minute rainfall is converted to mm/hour by multiplying by six. Event rainfall total is the sum of 10-minute depths.
- Response is the first valid post-onset soil-moisture value at least 0.01 m3/m3 above the immediately preceding 10-minute sample. This baseline is an operationalization and must be checked against author code/supplement before claiming exact reproduction.
- Apparent response velocity is sensor depth divided by response lag. It is not direct pore-water velocity.
- A binary preferential-flow proxy is emitted only for 10–100 cm sensors and only when a maximum matrix-flow Ksat reference is supplied. Ksat is not calculated here; the proxy is not tracer ground truth.
- Historical DP1.00006.001 and current DP1.00045.001 can be declared as rainfall inputs. Hourly/daily DP1.00044.001 and throughfall DP1.00046.001 are rejected as interchangeable high-frequency open rainfall inputs.

## Data preparation required

This tool does not parse raw NEON packages, apply quality flags, calculate 10-minute medians, or resolve the historical precipitation release against the current tipping-bucket product. Preserve raw files and query responses separately. Throughfall remains a separate input pathway.

## Limitations

The tests cover six-hour grouping, five-day splitting, threshold detection, unit conversion, missing-data abstention, sensor depth range and product semantics. They are software tests only, not field validation, tracer confirmation, treatment efficacy or South African transfer evidence.