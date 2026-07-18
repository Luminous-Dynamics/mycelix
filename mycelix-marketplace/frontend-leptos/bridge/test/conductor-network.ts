import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { execFile } from "node:child_process";
import { readFile } from "node:fs/promises";
import { promisify } from "node:util";
import {
  AdminWebsocket,
  AppWebsocket,
  type ActionHash,
  type AgentPubKey,
  type AppWebsocketConnectionOptions,
} from "@holochain/client";
import { emitLiveEvidence, requireActiveRoles } from "./evidence";

const execFileAsync = promisify(execFile);
const ROLE = "marketplace";
const LISTINGS = "listings";
const TRANSACTIONS = "transactions";

type ListingOutput = {
  listing_hash: ActionHash;
  seller_agent_id: AgentPubKey;
  listing: { price_cents: number; status: string };
};

type TransactionOutput = {
  transaction_hash: ActionHash;
  transaction: {
    buyer: AgentPubKey;
    seller: AgentPubKey;
    status: string;
  };
};

type TransactionResolution = {
  root_transaction_hash: ActionHash;
  policy_version: number;
  state: "resolved" | "auto_resolved" | "authorized_resolved" | "conflicted";
  reason:
    | "single_head"
    | "cancellation_dominates_pre_shipment"
    | "dispute_dominates_lifecycle"
    | "bilateral_agreement"
    | "arbitration_award"
    | "convergent_explicit_authorities"
    | "conflicting_explicit_authorities"
    | "unsafe_concurrent_lifecycle";
  canonical: TransactionOutput | null;
  heads: TransactionOutput[];
  superseded_heads: TransactionOutput[];
  applied_conflict_resolutions: Array<{
    resolution_hash: ActionHash;
    protocol_version: number;
    selected_head_hash: ActionHash;
    bound_head_hashes: ActionHash[];
    authority: unknown;
  }>;
  revision_count: number;
};


type TransactionConflictApprovalOutput = {
  approval_hash: ActionHash;
  approval: {
    protocol_version: number;
    root_transaction_hash: ActionHash;
    head_hashes: ActionHash[];
    selected_head_hash: ActionHash;
    approver: AgentPubKey;
    rationale: string;
  };
};

type IdentityConfig = {
  label: string;
  appUrl: URL;
  adminUrl: URL;
  token: number[];
};

type ConnectedIdentity = {
  app: AppWebsocket;
  admin: AdminWebsocket;
};

type ControlReceipt = {
  schema_version: 1;
  command: "partition" | "heal" | "status";
  result: "pass";
  state: "healthy" | "partitioned" | "healed";
  method: string;
  topology_sha256: string;
};

type NetworkTopology = {
  schema_version: 1;
  topology: "two_conductor_isolated_network";
  conductors: Record<string, { admin_url: string; app_port: number; pid: number }>;
  service_receipt_sha256: string;
  control_hook_sha256: string;
};

function required(name: string): string {
  const value = process.env[name];
  if (!value) throw new Error(`missing required environment variable ${name}`);
  return value;
}

function decodeToken(value: string): number[] {
  const normalized = value.replace(/-/g, "+").replace(/_/g, "/");
  return Array.from(Buffer.from(normalized, "base64"));
}

function identity(prefix: "SELLER" | "BUYER"): IdentityConfig {
  return {
    label: prefix.toLowerCase(),
    appUrl: new URL(required(`MARKETPLACE_${prefix}_APP_URL`)),
    adminUrl: new URL(required(`MARKETPLACE_${prefix}_ADMIN_URL`)),
    token: decodeToken(required(`MARKETPLACE_${prefix}_TOKEN_BASE64`)),
  };
}

async function connectIdentity(config: IdentityConfig): Promise<ConnectedIdentity> {
  const options: AppWebsocketConnectionOptions = {
    url: config.appUrl,
    token: config.token,
    defaultTimeout: 30_000,
  };
  const app = await AppWebsocket.connect(options);
  const admin = await AdminWebsocket.connect({ url: config.adminUrl });
  const info = await app.appInfo();
  const cellId = app.getCellIdFromRoleName(ROLE, info);
  await admin.authorizeSigningCredentials(cellId);
  return { app, admin };
}

async function call<T>(
  app: AppWebsocket,
  zome: string,
  fnName: string,
  payload: unknown,
): Promise<T> {
  return app.callZome<T>({
    role_name: ROLE,
    zome_name: zome,
    fn_name: fnName,
    payload,
  });
}

async function eventually<T>(
  label: string,
  operation: () => Promise<T>,
  accept: (value: T) => boolean,
  timeoutMs = 120_000,
): Promise<{ value: T; elapsedMs: number }> {
  const started = Date.now();
  const deadline = started + timeoutMs;
  let lastError: unknown;
  while (Date.now() < deadline) {
    try {
      const value = await operation();
      if (accept(value)) return { value, elapsedMs: Date.now() - started };
      lastError = new Error(`${label}: predicate was not satisfied`);
    } catch (error) {
      lastError = error;
    }
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
  throw new Error(`${label}: timed out after ${timeoutMs}ms`, { cause: lastError });
}

function hashKey(value: Uint8Array): string {
  return Buffer.from(value).toString("base64url");
}

function headSet(resolution: TransactionResolution): string[] {
  return resolution.heads.map((head) => hashKey(head.transaction_hash)).sort();
}

function headStatuses(resolution: TransactionResolution): string[] {
  return resolution.heads.map((head) => head.transaction.status).sort();
}

function supersededStatuses(resolution: TransactionResolution): string[] {
  return resolution.superseded_heads
    .map((head) => head.transaction.status)
    .sort();
}

async function sha256File(path: string): Promise<string> {
  return createHash("sha256").update(await readFile(path)).digest("hex");
}

async function control(command: "partition" | "heal" | "status"): Promise<ControlReceipt> {
  const executable = required("MARKETPLACE_NETWORK_CONTROL");
  const topology = required("MARKETPLACE_NETWORK_TOPOLOGY");
  const { stdout } = await execFileAsync(executable, [command, topology], {
    timeout: 60_000,
    maxBuffer: 1024 * 1024,
  });
  const receipt = JSON.parse(stdout) as ControlReceipt;
  assert.equal(receipt.schema_version, 1);
  assert.equal(receipt.command, command);
  assert.equal(receipt.result, "pass");
  assert.ok(receipt.method.trim());
  assert.match(receipt.topology_sha256, /^[0-9a-f]{64}$/);
  return receipt;
}

async function resolution(
  app: AppWebsocket,
  root: ActionHash,
): Promise<TransactionResolution | null> {
  return call<TransactionResolution | null>(
    app,
    TRANSACTIONS,
    "get_transaction_resolution",
    root,
  );
}

async function createPendingTransaction(
  seller: ConnectedIdentity,
  buyer: ConnectedIdentity,
  listing: ListingOutput,
): Promise<{ root: ActionHash; propagationMs: number }> {
  const pending = await call<TransactionOutput>(
    buyer.app,
    TRANSACTIONS,
    "create_transaction",
    {
      seller: listing.seller_agent_id,
      listing_hash: listing.listing_hash,
      quantity: 1,
      total_price_cents: listing.listing.price_cents,
    },
  );
  const propagated = await eventually(
    "pending transaction propagation to seller conductor",
    () => resolution(seller.app, pending.transaction_hash),
    (value) =>
      value?.state === "resolved" &&
      value.canonical?.transaction.status === "pending",
  );
  return { root: pending.transaction_hash, propagationMs: propagated.elapsedMs };
}

async function main(): Promise<void> {
  const topologyPath = required("MARKETPLACE_NETWORK_TOPOLOGY");
  const servicesReceiptPath = required("MARKETPLACE_NETWORK_SERVICES_RECEIPT");
  const controlPath = required("MARKETPLACE_NETWORK_CONTROL");
  const topology = JSON.parse(await readFile(topologyPath, "utf8")) as NetworkTopology;
  assert.equal(topology.schema_version, 1);
  assert.equal(topology.topology, "two_conductor_isolated_network");
  assert.equal(Object.keys(topology.conductors).length, 2);
  assert.notEqual(topology.conductors.seller.pid, topology.conductors.buyer.pid);
  assert.notEqual(
    topology.conductors.seller.admin_url,
    topology.conductors.buyer.admin_url,
  );
  assert.equal(topology.control_hook_sha256, await sha256File(controlPath));
  assert.equal(
    topology.service_receipt_sha256,
    await sha256File(servicesReceiptPath),
  );

  const seller = await connectIdentity(identity("SELLER"));
  const buyer = await connectIdentity(identity("BUYER"));
  const activeRoleSets = {
    seller: await requireActiveRoles(seller.app, [ROLE]),
    buyer: await requireActiveRoles(buyer.app, [ROLE]),
  };
  let partitioned = false;

  try {
    assert.notDeepEqual(Array.from(seller.app.myPubKey), Array.from(buyer.app.myPubKey));
    const initialStatus = await control("status");
    assert.ok(["healthy", "healed"].includes(initialStatus.state));

    const listing = await call<ListingOutput>(seller.app, LISTINGS, "create_listing", {
      title: `Network conflict-policy evidence ${Date.now()}`,
      description:
        "Two-conductor evidence for narrow safety projection and explicit unsafe conflict.",
      price_cents: 14_500,
      category: "Electronics",
      photos_ipfs_cids: [],
      quantity_available: 4,
    });
    const listingPropagation = await eventually(
      "listing propagation to buyer conductor",
      () => call<ListingOutput | null>(buyer.app, LISTINGS, "get_listing", listing.listing_hash),
      (value) => value !== null,
    );

    // Case A: cancellation before shipment is a narrow safety-dominant projection.
    const safeCase = await createPendingTransaction(seller, buyer, listing);
    const safePartition = await control("partition");
    assert.equal(safePartition.state, "partitioned");
    partitioned = true;
    assert.equal((await control("status")).state, "partitioned");

    await Promise.all([
      call<TransactionOutput>(seller.app, TRANSACTIONS, "confirm_transaction", safeCase.root),
      call<TransactionOutput>(buyer.app, TRANSACTIONS, "cancel_transaction", safeCase.root),
    ]);
    const safeSellerLocal = await eventually(
      "seller local confirmed branch",
      () => resolution(seller.app, safeCase.root),
      (value) =>
        value?.state === "resolved" &&
        value.canonical?.transaction.status === "confirmed",
      45_000,
    );
    const safeBuyerLocal = await eventually(
      "buyer local cancelled branch",
      () => resolution(buyer.app, safeCase.root),
      (value) =>
        value?.state === "resolved" &&
        value.canonical?.transaction.status === "cancelled",
      45_000,
    );

    const safeHeal = await control("heal");
    assert.equal(safeHeal.state, "healed");
    partitioned = false;
    const safeSellerProjection = await eventually(
      "seller safety projection visibility",
      () => resolution(seller.app, safeCase.root),
      (value) =>
        value?.state === "auto_resolved" &&
        value.reason === "cancellation_dominates_pre_shipment" &&
        value.canonical?.transaction.status === "cancelled" &&
        JSON.stringify(headStatuses(value)) === JSON.stringify(["cancelled", "confirmed"]) &&
        JSON.stringify(supersededStatuses(value)) === JSON.stringify(["confirmed"]),
    );
    const safeBuyerProjection = await eventually(
      "buyer safety projection visibility",
      () => resolution(buyer.app, safeCase.root),
      (value) =>
        value?.state === "auto_resolved" &&
        value.reason === "cancellation_dominates_pre_shipment" &&
        value.canonical?.transaction.status === "cancelled" &&
        JSON.stringify(headStatuses(value)) === JSON.stringify(["cancelled", "confirmed"]) &&
        JSON.stringify(supersededStatuses(value)) === JSON.stringify(["confirmed"]),
    );
    assert.deepEqual(headSet(safeSellerProjection.value!), headSet(safeBuyerProjection.value!));
    assert.equal(safeSellerProjection.value?.policy_version, 2);
    assert.equal(safeBuyerProjection.value?.policy_version, 2);

    // Case B: once shipping evidence exists, cancellation may not dominate.
    const unsafeCase = await createPendingTransaction(seller, buyer, listing);
    await call<TransactionOutput>(
      seller.app,
      TRANSACTIONS,
      "confirm_transaction",
      unsafeCase.root,
    );
    await eventually(
      "confirmed transaction propagation to buyer conductor",
      () => resolution(buyer.app, unsafeCase.root),
      (value) =>
        value?.state === "resolved" &&
        value.canonical?.transaction.status === "confirmed",
    );

    const unsafePartition = await control("partition");
    assert.equal(unsafePartition.state, "partitioned");
    partitioned = true;
    assert.equal((await control("status")).state, "partitioned");

    await Promise.all([
      call<TransactionOutput>(seller.app, TRANSACTIONS, "mark_shipped", {
        transaction_hash: unsafeCase.root,
        tracking_info: "NETWORK-PARTITION-TRACKING",
      }),
      call<TransactionOutput>(buyer.app, TRANSACTIONS, "cancel_transaction", unsafeCase.root),
    ]);
    const unsafeSellerLocal = await eventually(
      "seller local shipped branch",
      () => resolution(seller.app, unsafeCase.root),
      (value) =>
        value?.state === "resolved" &&
        value.canonical?.transaction.status === "shipped",
      45_000,
    );
    const unsafeBuyerLocal = await eventually(
      "buyer local cancelled branch after confirmation",
      () => resolution(buyer.app, unsafeCase.root),
      (value) =>
        value?.state === "resolved" &&
        value.canonical?.transaction.status === "cancelled",
      45_000,
    );

    const unsafeHeal = await control("heal");
    assert.equal(unsafeHeal.state, "healed");
    partitioned = false;
    const sellerConflict = await eventually(
      "seller unsafe conflict visibility",
      () => resolution(seller.app, unsafeCase.root),
      (value) =>
        value?.state === "conflicted" &&
        value.reason === "unsafe_concurrent_lifecycle" &&
        value.canonical === null &&
        value.superseded_heads.length === 0 &&
        JSON.stringify(headStatuses(value)) === JSON.stringify(["cancelled", "shipped"]),
    );
    const buyerConflict = await eventually(
      "buyer unsafe conflict visibility",
      () => resolution(buyer.app, unsafeCase.root),
      (value) =>
        value?.state === "conflicted" &&
        value.reason === "unsafe_concurrent_lifecycle" &&
        value.canonical === null &&
        value.superseded_heads.length === 0 &&
        JSON.stringify(headStatuses(value)) === JSON.stringify(["cancelled", "shipped"]),
    );
    assert.deepEqual(headSet(sellerConflict.value!), headSet(buyerConflict.value!));
    assert.equal(sellerConflict.value?.policy_version, 2);
    assert.equal(buyerConflict.value?.policy_version, 2);

    const shippedHead = sellerConflict.value!.heads.find(
      (head) => head.transaction.status === "shipped",
    );
    assert.ok(shippedHead, "unsafe conflict must expose one shipped head");
    const sellerApproval = await call<TransactionConflictApprovalOutput>(
      seller.app,
      TRANSACTIONS,
      "approve_transaction_conflict",
      {
        transaction_hash: unsafeCase.root,
        selected_head_hash: shippedHead.transaction_hash,
        rationale: "Shipment evidence should remain authoritative after bilateral review",
      },
    );
    const buyerApproval = await call<TransactionConflictApprovalOutput>(
      buyer.app,
      TRANSACTIONS,
      "approve_transaction_conflict",
      {
        transaction_hash: unsafeCase.root,
        selected_head_hash: shippedHead.transaction_hash,
        rationale: "Buyer accepts the already-authored shipment branch",
      },
    );
    const approvalVisibility = await eventually(
      "bilateral approval propagation",
      () =>
        call<TransactionConflictApprovalOutput[]>(
          seller.app,
          TRANSACTIONS,
          "get_transaction_conflict_approvals",
          unsafeCase.root,
        ),
      (items) =>
        items.some((item) => hashKey(item.approval_hash) === hashKey(sellerApproval.approval_hash)) &&
        items.some((item) => hashKey(item.approval_hash) === hashKey(buyerApproval.approval_hash)),
    );
    assert.equal(approvalVisibility.value.length >= 2, true);

    await call<TransactionResolution>(
      seller.app,
      TRANSACTIONS,
      "finalize_bilateral_transaction_conflict",
      {
        transaction_hash: unsafeCase.root,
        buyer_approval_hash: buyerApproval.approval_hash,
        seller_approval_hash: sellerApproval.approval_hash,
        summary: "Both parties selected the shipped branch after reviewing the preserved conflict",
      },
    );
    const sellerAuthorized = await eventually(
      "seller bilateral projection visibility",
      () => resolution(seller.app, unsafeCase.root),
      (value) =>
        value?.state === "authorized_resolved" &&
        value.reason === "bilateral_agreement" &&
        value.canonical?.transaction.status === "shipped" &&
        JSON.stringify(headStatuses(value)) === JSON.stringify(["cancelled", "shipped"]) &&
        JSON.stringify(supersededStatuses(value)) === JSON.stringify(["cancelled"]) &&
        value.applied_conflict_resolutions.length >= 1,
    );
    const buyerAuthorized = await eventually(
      "buyer bilateral projection visibility",
      () => resolution(buyer.app, unsafeCase.root),
      (value) =>
        value?.state === "authorized_resolved" &&
        value.reason === "bilateral_agreement" &&
        value.canonical?.transaction.status === "shipped" &&
        JSON.stringify(headStatuses(value)) === JSON.stringify(["cancelled", "shipped"]) &&
        JSON.stringify(supersededStatuses(value)) === JSON.stringify(["cancelled"]) &&
        value.applied_conflict_resolutions.length >= 1,
    );
    assert.deepEqual(headSet(sellerAuthorized.value!), headSet(buyerAuthorized.value!));
    assert.equal(
      hashKey(sellerAuthorized.value!.canonical!.transaction_hash),
      hashKey(buyerAuthorized.value!.canonical!.transaction_hash),
    );

    const serviceReceipt = JSON.parse(await readFile(servicesReceiptPath, "utf8")) as {
      isolation: string;
      implementation_sha256: string;
    };
    assert.equal(serviceReceipt.isolation, "local_controlled");
    assert.match(serviceReceipt.implementation_sha256, /^[0-9a-f]{64}$/);

    await emitLiveEvidence("network", activeRoleSets, {
      topology: topology.topology,
      conductor_count: 2,
      distinct_conductor_processes: true,
      distinct_admin_endpoints: true,
      service_isolation: serviceReceipt.isolation,
      service_implementation_sha256: serviceReceipt.implementation_sha256,
      control_hook_sha256: topology.control_hook_sha256,
      topology_sha256: await sha256File(topologyPath),
      conflict_policy_version: 2,
      listing_propagation_ms: listingPropagation.elapsedMs,
      transaction_propagation_ms: Math.max(
        safeCase.propagationMs,
        unsafeCase.propagationMs,
      ),
      safe_seller_local_divergence_ms: safeSellerLocal.elapsedMs,
      safe_buyer_local_divergence_ms: safeBuyerLocal.elapsedMs,
      safe_projection_visibility_ms: Math.max(
        safeSellerProjection.elapsedMs,
        safeBuyerProjection.elapsedMs,
      ),
      unsafe_seller_local_divergence_ms: unsafeSellerLocal.elapsedMs,
      unsafe_buyer_local_divergence_ms: unsafeBuyerLocal.elapsedMs,
      unsafe_conflict_visibility_ms: Math.max(
        sellerConflict.elapsedMs,
        buyerConflict.elapsedMs,
      ),
      partition_divergence_observed: true,
      safe_terminal_dominance_observed: true,
      safe_projection_seen_by_all: true,
      safe_projection_state: "auto_resolved",
      safe_projection_reason: "cancellation_dominates_pre_shipment",
      safe_canonical_status: "cancelled",
      safe_head_statuses: headStatuses(safeSellerProjection.value!),
      safe_superseded_statuses: supersededStatuses(safeSellerProjection.value!),
      safe_head_hashes: headSet(safeSellerProjection.value!),
      unsafe_conflict_observed: true,
      unsafe_conflict_seen_by_all: true,
      unsafe_arbitrary_winner_selected: false,
      unsafe_head_statuses: headStatuses(sellerConflict.value!),
      unsafe_conflict_head_hashes: headSet(sellerConflict.value!),
      bilateral_resolution_observed: true,
      bilateral_resolution_seen_by_all: true,
      bilateral_resolution_state: "authorized_resolved",
      bilateral_resolution_reason: "bilateral_agreement",
      bilateral_canonical_status: "shipped",
      bilateral_head_hashes: headSet(sellerAuthorized.value!),
      bilateral_superseded_statuses: supersededStatuses(sellerAuthorized.value!),
      bilateral_authority_count: sellerAuthorized.value!.applied_conflict_resolutions.length,
      bilateral_approval_visibility_ms: approvalVisibility.elapsedMs,
      bilateral_resolution_visibility_ms: Math.max(
        sellerAuthorized.elapsedMs,
        buyerAuthorized.elapsedMs,
      ),
      safe_partition_method: safePartition.method,
      safe_heal_method: safeHeal.method,
      unsafe_partition_method: unsafePartition.method,
      unsafe_heal_method: unsafeHeal.method,
    });
  } finally {
    if (partitioned) {
      await control("heal").catch((error) => console.error("emergency heal failed", error));
    }
    await Promise.allSettled([
      seller.app.client.close(),
      seller.admin.client.close(),
      buyer.app.client.close(),
      buyer.admin.client.close(),
    ]);
  }
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
