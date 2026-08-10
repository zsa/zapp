//! Choosing one board out of everything the platform can see.
//!
//! Pure, so the awkward cases — nothing, several, one that was never bonded —
//! are settled without a radio. Shared, because every service this crate talks
//! to has to answer the same four questions first, and only differs in which
//! sightings could be carrying it.

use crate::{identity, BleSighting};

/// What a hint picked out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Picked<'a> {
    /// Exactly one board, and it is bonded.
    One(&'a BleSighting),
    /// Nothing matched.
    None,
    /// More than one did, described for the message that asks which.
    Several(Vec<String>),
    /// One did, but it was never bonded — and everything worth writing to is
    /// behind an encrypted link.
    Unbonded(String),
}

/// The one board a hint names, out of those `speaks` says could carry the
/// service.
pub fn pick<'a>(
    sightings: &'a [BleSighting],
    hint: Option<&str>,
    speaks: &dyn Fn(&BleSighting) -> bool,
) -> Picked<'a> {
    let named = |sighting: &BleSighting| match hint {
        None => true,
        Some(hint) => {
            sighting.key.eq_ignore_ascii_case(hint)
                || sighting
                    .name
                    .as_deref()
                    .is_some_and(|name| name.to_lowercase().contains(&hint.to_lowercase()))
        }
    };

    let matches: Vec<&BleSighting> = sightings
        .iter()
        .filter(|sighting| speaks(sighting) && named(sighting))
        .collect();

    // A board already connected is the one a caller means, and is also the only
    // one that will answer promptly.
    let connected: Vec<&BleSighting> = matches
        .iter()
        .copied()
        .filter(|sighting| sighting.connected)
        .collect();
    let shortlist = if connected.is_empty() {
        matches
    } else {
        connected
    };

    match shortlist.as_slice() {
        [] => Picked::None,
        [only] if only.paired => Picked::One(only),
        [only] => Picked::Unbonded(describe(only)),
        several => Picked::Several(several.iter().map(|sighting| describe(sighting)).collect()),
    }
}

/// A board in the words a person would recognise it by.
pub fn describe(sighting: &BleSighting) -> String {
    match (&sighting.name, identity::keyboard_for(sighting)) {
        (Some(name), Some(keyboard)) => format!("{keyboard} {name:?}"),
        (Some(name), None) => format!("{name:?}"),
        (None, Some(keyboard)) => format!("{keyboard} ({})", sighting.key),
        (None, None) => sighting.key.clone(),
    }
}
