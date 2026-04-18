// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

export const index = 0;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/_layout.svelte.js')).default;
export const universal = {
  "prerender": false,
  "ssr": false
};
export const universal_id = "src/routes/+layout.ts";
export const imports = ["_app/immutable/nodes/0.cKxibHHF.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/B4sKVuNd.js","_app/immutable/chunks/BefYpqPH.js","_app/immutable/chunks/D2_CMSXj.js","_app/immutable/chunks/CatO2vgv.js","_app/immutable/chunks/aRspOZbs.js","_app/immutable/chunks/CBcrUV2C.js","_app/immutable/chunks/DEQuCKXn.js","_app/immutable/chunks/aGrU48bv.js","_app/immutable/chunks/DLw1pTtx.js"];
export const stylesheets = ["_app/immutable/assets/0.LxVoTd4q.css"];
export const fonts = [];
