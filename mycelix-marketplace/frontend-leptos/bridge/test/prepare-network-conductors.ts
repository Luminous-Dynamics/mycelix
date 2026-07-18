import { readFile, writeFile } from "node:fs/promises";
import { AdminWebsocket, CellType, type AppInfo } from "@holochain/client";

type Actor = {
  prefix: "SELLER" | "BUYER";
  admin_url: string;
  installed_app_id: string;
  app_port: number;
};

type Config = {
  happ_path: string;
  allowed_origin: string;
  output_env: string;
  actors: Actor[];
};

function base64Url(bytes: number[]): string {
  return Buffer.from(bytes)
    .toString("base64")
    .replace(/=/g, "")
    .replace(/\+/g, "-")
    .replace(/\//g, "_");
}

function activeCellIds(info: AppInfo) {
  return Object.values(info.cell_info)
    .flat()
    .filter(
      (cell) =>
        cell.type === CellType.Provisioned || cell.type === CellType.Cloned,
    )
    .map((cell) => cell.value.cell_id);
}

async function main(): Promise<void> {
  const configPath = process.env.MARKETPLACE_NETWORK_SETUP_CONFIG;
  if (!configPath) {
    throw new Error("MARKETPLACE_NETWORK_SETUP_CONFIG is required");
  }
  const config = JSON.parse(await readFile(configPath, "utf8")) as Config;
  if (config.actors.length !== 2) {
    throw new Error("network promotion requires exactly seller and buyer conductors");
  }
  if (new Set(config.actors.map((actor) => actor.admin_url)).size !== 2) {
    throw new Error("network promotion actors must use distinct admin interfaces");
  }

  const lines: string[] = [];
  for (const actor of config.actors) {
    const admin = await AdminWebsocket.connect({ url: new URL(actor.admin_url) });
    try {
      const agentKey = await admin.generateAgentPubKey();
      await admin.installApp({
        source: { type: "path", value: config.happ_path },
        agent_key: agentKey,
        installed_app_id: actor.installed_app_id,
      });
      const appInfo = await admin.enableApp({
        installed_app_id: actor.installed_app_id,
      });
      const cells = activeCellIds(appInfo);
      if (!cells.length) {
        throw new Error(`${actor.prefix}: installed app has no active cells`);
      }
      for (const cellId of cells) {
        await admin.authorizeSigningCredentials(cellId);
      }
      const attached = await admin.attachAppInterface({
        port: actor.app_port,
        allowed_origins: config.allowed_origin,
        installed_app_id: actor.installed_app_id,
      });
      const issued = await admin.issueAppAuthenticationToken({
        installed_app_id: actor.installed_app_id,
        single_use: false,
      });
      lines.push(`MARKETPLACE_${actor.prefix}_ADMIN_URL=${actor.admin_url}`);
      lines.push(`MARKETPLACE_${actor.prefix}_APP_URL=ws://127.0.0.1:${attached.port}`);
      lines.push(
        `MARKETPLACE_${actor.prefix}_TOKEN_BASE64=${base64Url(issued.token)}`,
      );
    } finally {
      await admin.client.close();
    }
  }

  await writeFile(config.output_env, `${lines.join("\n")}\n`, {
    mode: 0o600,
    flag: "wx",
  });
}

await main();
