// Keep the already-reviewed A2 proposal coordinator intact and layer A3
// assignment-transition APIs beside it.
#[path = "lib.rs"]
mod proposal;
pub use proposal::*;

mod assignment;
pub use assignment::*;
