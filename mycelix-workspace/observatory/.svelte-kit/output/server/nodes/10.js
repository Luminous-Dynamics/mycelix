

export const index = 10;
let component_cache;
export const component = async () => component_cache ??= (await import('../entries/pages/governance/_page.svelte.js')).default;
export const imports = ["_app/immutable/nodes/10.B6VG9ISB.js","_app/immutable/chunks/CujjRMGB.js","_app/immutable/chunks/DIAPMP-w.js","_app/immutable/chunks/B4sKVuNd.js","_app/immutable/chunks/CatO2vgv.js"];
export const stylesheets = [];
export const fonts = [];
