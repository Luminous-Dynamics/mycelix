import assert from "node:assert/strict";
import {
  AdminWebsocket,
  AppWebsocket,
  type ActionHash,
  type AgentPubKey,
  type AppWebsocketConnectionOptions,
} from "@holochain/client";
import { emitLiveEvidence, requireActiveRoles } from "./evidence";

const ROLE = "marketplace";
const LISTINGS = "listings";
const TRANSACTIONS = "transactions";

type ListingOutput = {
  listing_hash: ActionHash;
  seller_agent_id: AgentPubKey;
  listing: {
    price_cents: number;
    quantity_available: number;
    status: string;
  };
};

type TransactionOutput = {
  transaction_hash: ActionHash;
  transaction: {
    buyer: AgentPubKey;
    seller: AgentPubKey;
    listing_hash: ActionHash;
    quantity: number;
    total_price_cents: number;
    status: string;
    tracking_info: string | null;
  };
};

type TransactionResolution = {
  root_transaction_hash: ActionHash;
  state: "resolved" | "auto_resolved" | "authorized_resolved" | "conflicted";
  policy_version: number;
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

function required(name: string): string {
  const value = process.env[name];
  if (!value) {
    throw new Error(`missing required environment variable ${name}`);
  }
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

async function connectIdentity(
  config: IdentityConfig,
): Promise<ConnectedIdentity> {
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
  console.log(`${config.label}: connected and authorized signing credentials`);
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
): Promise<T> {
  const deadline = Date.now() + 45_000;
  let lastError: unknown;
  while (Date.now() < deadline) {
    try {
      const value = await operation();
      if (accept(value)) {
        return value;
      }
      lastError = new Error(`${label}: returned a value that did not satisfy the predicate`);
    } catch (error) {
      lastError = error;
    }
    await new Promise((resolve) => setTimeout(resolve, 750));
  }
  throw new Error(`${label}: timed out`, { cause: lastError });
}

async function waitForStatus(
  app: AppWebsocket,
  root: ActionHash,
  status: string,
): Promise<TransactionResolution> {
  return eventually(
    `wait for ${status}`,
    () =>
      call<TransactionResolution | null>(
        app,
        TRANSACTIONS,
        "get_transaction_resolution",
        root,
      ),
    (resolution): resolution is TransactionResolution =>
      resolution !== null &&
      resolution.state === "resolved" &&
      resolution.canonical?.transaction.status === status,
  );
}

async function main(): Promise<void> {
  const sellerConfig = identity("SELLER");
  const buyerConfig = identity("BUYER");
  const seller = await connectIdentity(sellerConfig);
  const buyer = await connectIdentity(buyerConfig);
  const activeRoleSets = {
    seller: await requireActiveRoles(seller.app, [ROLE]),
    buyer: await requireActiveRoles(buyer.app, [ROLE]),
  };

  try {
    assert.notDeepEqual(Array.from(seller.app.myPubKey), Array.from(buyer.app.myPubKey));

    const listing = await call<ListingOutput>(seller.app, LISTINGS, "create_listing", {
      title: `Lifecycle evidence fixture ${Date.now()}`,
      description:
        "Conductor-backed evidence listing for the conflict-aware Leptos transaction lifecycle.",
      price_cents: 12_500,
      category: "Electronics",
      photos_ipfs_cids: [],
      quantity_available: 3,
    });
    assert.equal(listing.listing.status, "active");
    assert.deepEqual(
      Array.from(listing.seller_agent_id),
      Array.from(seller.app.myPubKey),
    );

    await eventually(
      "listing propagation to buyer",
      () => call<ListingOutput | null>(buyer.app, LISTINGS, "get_listing", listing.listing_hash),
      (value) => value !== null,
    );

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
    const root = pending.transaction_hash;
    assert.equal(pending.transaction.status, "pending");

    const sellerPending = await waitForStatus(seller.app, root, "pending");
    assert.equal(sellerPending.revision_count, 1);

    await call<TransactionOutput>(seller.app, TRANSACTIONS, "confirm_transaction", root);
    const buyerConfirmed = await waitForStatus(buyer.app, root, "confirmed");
    assert.equal(buyerConfirmed.revision_count, 2);

    await call<TransactionOutput>(seller.app, TRANSACTIONS, "mark_shipped", {
      transaction_hash: root,
      tracking_info: "MYCELIX-EVIDENCE-001",
    });
    const buyerShipped = await waitForStatus(buyer.app, root, "shipped");
    assert.equal(buyerShipped.revision_count, 3);
    assert.equal(
      buyerShipped.canonical?.transaction.tracking_info,
      "MYCELIX-EVIDENCE-001",
    );

    await call<TransactionOutput>(buyer.app, TRANSACTIONS, "confirm_delivery", root);
    const delivered = await waitForStatus(seller.app, root, "delivered");
    assert.equal(delivered.revision_count, 4);
    assert.equal(delivered.heads.length, 1);

    const cancellationPending = await call<TransactionOutput>(
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
    await waitForStatus(seller.app, cancellationPending.transaction_hash, "pending");
    await call<TransactionOutput>(
      buyer.app,
      TRANSACTIONS,
      "cancel_transaction",
      cancellationPending.transaction_hash,
    );
    const cancelled = await waitForStatus(
      seller.app,
      cancellationPending.transaction_hash,
      "cancelled",
    );
    assert.equal(cancelled.revision_count, 2);

    await emitLiveEvidence(
      "lifecycle",
      activeRoleSets,
      {
          delivered_root: Array.from(root),
          delivered_revisions: delivered.revision_count,
          cancellation_revisions: cancelled.revision_count,
          completion_tested: false,
          dispute_tested: false,
        },
    );
  } finally {
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
