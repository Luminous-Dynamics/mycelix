SHELL := /bin/bash

.PHONY: help dev test build clean web rust zomes fmt lint bench check integration-test conductor-up conductor-down dna happ

help: ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-15s %s\n", $$1, $$2}'

dev: build start ## Build and start development environment

build: rust web ## Build all components

rust: ## Build Rust workspace
	@echo "▶ Building Rust workspace..."
	cargo fmt --all
	cargo clippy --all -- -D warnings
	cargo build --workspace

web: ## Build web application
	@echo "▶ Building web application..."
	cd apps/leptos && trunk build --release

zomes: ## Build Holochain zomes to WASM
	@echo "▶ Building Holochain zomes..."
	./scripts/build-wasm.sh

fmt: ## Format all code
	cargo fmt --all
	cargo fmt --manifest-path apps/leptos/Cargo.toml

lint: ## Lint all code
	cargo clippy --all -- -D warnings
	cargo clippy --manifest-path apps/leptos/Cargo.toml --target wasm32-unknown-unknown -- -D warnings

test: ## Run all tests
	@echo "▶ Running Rust tests..."
	cargo test --all
	@echo "▶ Validating Leptos application..."
	cargo check --manifest-path apps/leptos/Cargo.toml --target wasm32-unknown-unknown

bench: ## Run benchmarks
	@echo "▶ Running benchmarks..."
	cargo bench --workspace

check: ## Quick sanity check (fmt + clippy + test)
	@echo "▶ Running quick sanity check..."
	python3 scripts/validate_curriculum.py
	python3 scripts/validate_learning_contract.py
	cargo fmt --all -- --check
	cargo clippy --all -- -D warnings
	cargo test --all --quiet
	cargo check --manifest-path apps/leptos/Cargo.toml --target wasm32-unknown-unknown

start: ## Start development servers
	./scripts/dev.sh

clean: ## Clean all build artifacts
	@echo "⚠ Cleaning all build artifacts..."
	cargo clean
	cargo clean --manifest-path apps/leptos/Cargo.toml
	rm -rf apps/leptos/dist
	find . -name "*.dna" -o -name "*.happ" -o -name "*.webhapp" | xargs rm -f

reset: ## Reset Holochain dev environment
	./scripts/hc-reset.sh

dna: zomes ## Package DNA bundle
	@echo "▶ Packaging DNA bundle..."
	cd dna && hc dna pack . -o praxis.dna
	@echo "  DNA: dna/praxis.dna"

happ: dna ## Package hApp bundle
	@echo "▶ Packaging hApp bundle..."
	hc app pack happ/ -o happ/mycelix-praxis.happ
	@echo "  hApp: happ/mycelix-praxis.happ"

integration-test: ## Run ignored integration tests (builds WASM + DNA + hApp first)
	./scripts/run-integration-tests.sh

conductor-up: happ ## Start conductor via docker-compose
	@echo "▶ Starting Holochain conductor..."
	docker compose up -d conductor
	@echo "  Admin: ws://localhost:8888"
	@echo "  App:   ws://localhost:8889"

conductor-down: ## Stop conductor
	docker compose down
