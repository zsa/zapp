//! Updating a Voyager 2, as a library.
//!
//! The order an update is done in, what is checked first, and how a pair of
//! halves is kept or put back, for any front end: the `zapp` CLI draws it on a
//! terminal, and a desktop app can draw it however it likes. Nothing in here
//! prints, prompts or exits.

pub mod confirm;
pub mod text;

pub use zapp_smp::{Bundle, Half};
