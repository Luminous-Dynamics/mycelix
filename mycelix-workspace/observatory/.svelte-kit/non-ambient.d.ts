
// this file is generated — do not edit it


declare module "svelte/elements" {
	export interface HTMLAttributes<T> {
		'data-sveltekit-keepfocus'?: true | '' | 'off' | undefined | null;
		'data-sveltekit-noscroll'?: true | '' | 'off' | undefined | null;
		'data-sveltekit-preload-code'?:
			| true
			| ''
			| 'eager'
			| 'viewport'
			| 'hover'
			| 'tap'
			| 'off'
			| undefined
			| null;
		'data-sveltekit-preload-data'?: true | '' | 'hover' | 'tap' | 'off' | undefined | null;
		'data-sveltekit-reload'?: true | '' | 'off' | undefined | null;
		'data-sveltekit-replacestate'?: true | '' | 'off' | undefined | null;
	}
}

export {};


declare module "$app/types" {
	type MatcherParam<M> = M extends (param : string) => param is (infer U extends string) ? U : string;

	export interface AppTypes {
		RouteId(): "/" | "/admin" | "/analytics" | "/attribution" | "/care-circles" | "/emergency" | "/epistemic-markets" | "/food" | "/governance" | "/household" | "/knowledge" | "/mutual-aid" | "/mygov" | "/network" | "/print" | "/resilience" | "/shelter" | "/supplies" | "/tend" | "/value-anchor" | "/water" | "/welcome";
		RouteParams(): {
			
		};
		LayoutParams(): {
			"/": Record<string, never>;
			"/admin": Record<string, never>;
			"/analytics": Record<string, never>;
			"/attribution": Record<string, never>;
			"/care-circles": Record<string, never>;
			"/emergency": Record<string, never>;
			"/epistemic-markets": Record<string, never>;
			"/food": Record<string, never>;
			"/governance": Record<string, never>;
			"/household": Record<string, never>;
			"/knowledge": Record<string, never>;
			"/mutual-aid": Record<string, never>;
			"/mygov": Record<string, never>;
			"/network": Record<string, never>;
			"/print": Record<string, never>;
			"/resilience": Record<string, never>;
			"/shelter": Record<string, never>;
			"/supplies": Record<string, never>;
			"/tend": Record<string, never>;
			"/value-anchor": Record<string, never>;
			"/water": Record<string, never>;
			"/welcome": Record<string, never>
		};
		Pathname(): "/" | "/admin" | "/analytics" | "/attribution" | "/care-circles" | "/emergency" | "/epistemic-markets" | "/food" | "/governance" | "/household" | "/knowledge" | "/mutual-aid" | "/network" | "/print" | "/resilience" | "/shelter" | "/supplies" | "/tend" | "/value-anchor" | "/water" | "/welcome";
		ResolvedPathname(): `${"" | `/${string}`}${ReturnType<AppTypes['Pathname']>}`;
		Asset(): "/favicon.png" | "/icon-192.png" | "/icon-512.png" | "/manifest.webmanifest" | string & {};
	}
}