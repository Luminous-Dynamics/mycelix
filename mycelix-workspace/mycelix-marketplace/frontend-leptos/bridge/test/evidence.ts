import { mkdir, writeFile } from "node:fs/promises";
import { join } from "node:path";
import {
  CellType,
  type AppInfo,
  type AppWebsocket,
} from "@holochain/client";

export type LiveEvidenceDetails = Record<string, unknown>;

export function activeRoles(info: AppInfo): string[] {
  return Object.keys(info.cell_info)
    .filter((roleName) =>
      info.cell_info[roleName]?.some(
        (cell) =>
          cell.type === CellType.Provisioned || cell.type === CellType.Cloned,
      ),
    )
    .sort();
}

export async function requireActiveRoles(
  app: AppWebsocket,
  requiredRoles: string[],
): Promise<string[]> {
  const roles = activeRoles(await app.appInfo());
  for (const role of requiredRoles) {
    if (!roles.includes(role)) {
      throw new Error(`connected hApp has no active ${role} role`);
    }
  }
  return roles;
}

function evidenceEnv(name: string): string {
  const value = process.env[name];
  if (!value) {
    throw new Error(
      `${name} is required when MARKETPLACE_EVIDENCE_DIR is configured`,
    );
  }
  return value;
}

export async function emitLiveEvidence(
  scenario: "lifecycle" | "arbitration" | "settlement" | "network",
  activeRoleSets: Record<string, string[]>,
  details: LiveEvidenceDetails,
): Promise<void> {
  const outputDirectory = process.env.MARKETPLACE_EVIDENCE_DIR;
  const receipt = {
    schema_version: 1,
    scenario,
    result: "pass",
    fixture: false,
    generated_at: new Date().toISOString(),
    source_revision: outputDirectory
      ? evidenceEnv("MARKETPLACE_SOURCE_REVISION")
      : process.env.MARKETPLACE_SOURCE_REVISION ?? "unrecorded",
    happ_sha256: outputDirectory
      ? evidenceEnv("MARKETPLACE_HAPP_SHA256")
      : process.env.MARKETPLACE_HAPP_SHA256 ?? "unrecorded",
    marketplace_dna_sha256: outputDirectory
      ? evidenceEnv("MARKETPLACE_DNA_SHA256")
      : process.env.MARKETPLACE_DNA_SHA256 ?? "unrecorded",
    finance_dna_sha256:
      scenario === "settlement"
        ? outputDirectory
          ? evidenceEnv("FINANCE_DNA_SHA256")
          : process.env.FINANCE_DNA_SHA256 ?? "unrecorded"
        : null,
    holochain_client_version: outputDirectory
      ? evidenceEnv("MARKETPLACE_CLIENT_VERSION")
      : process.env.MARKETPLACE_CLIENT_VERSION ?? "0.20.5",
    active_roles: activeRoleSets,
    details,
  };

  console.log(JSON.stringify(receipt, null, 2));
  if (!outputDirectory) return;

  await mkdir(outputDirectory, { recursive: true });
  await writeFile(
    join(outputDirectory, `${scenario}.json`),
    `${JSON.stringify(receipt, null, 2)}\n`,
    { flag: "wx" },
  );
}
