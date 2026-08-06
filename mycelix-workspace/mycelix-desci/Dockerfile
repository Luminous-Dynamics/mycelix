# syntax=docker/dockerfile:1.7
# Mycelix-DeSci canonical API image.
#
# The repository currently depends on the sibling `mycelix-zkp-core` crate.
# Supply it as a BuildKit named context:
#   docker buildx build \
#     --build-context zkp-core=../crates/mycelix-zkp-core \
#     -t mycelix-desci-api .

FROM rust:1.96-bookworm AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace/mycelix-desci
COPY --from=zkp-core . /workspace/crates/mycelix-zkp-core
WORKDIR /workspace/mycelix-desci

RUN cargo build --locked --release --package mycelix-desci-api --bin mycelix-api

FROM debian:bookworm-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    libssl3 \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 --shell /usr/sbin/nologin mycelix \
    && mkdir -p /app/data/scientific-events /app/config \
    && chown -R mycelix:mycelix /app

COPY --from=builder /workspace/mycelix-desci/target/release/mycelix-api /usr/local/bin/mycelix-api

WORKDIR /app
USER mycelix
EXPOSE 8080

HEALTHCHECK --interval=30s --timeout=3s --start-period=10s --retries=3 \
    CMD curl --fail --silent http://127.0.0.1:8080/health >/dev/null || exit 1

CMD ["mycelix-api"]
