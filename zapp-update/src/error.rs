//! How an update fails.
//!
//! Inside, a failure is built the way the CLI has always built it: an
//! `anyhow` chain of contexts, whose `{:#}` is the one line a person reads.
//! What leaves the crate is an [`UpdateError`] carrying that line unchanged,
//! with the kind of failure and the half it concerns read off markers the
//! orchestration leaves in the chain.

use std::fmt;

use zapp_smp::Half;

/// A failed update: what kind of failure, which half when one is named, and
/// the sentence to show.
#[derive(Debug, Clone)]
pub struct UpdateError {
    kind: ErrorKind,
    half: Option<Half>,
    message: String,
}

impl UpdateError {
    /// What kind of failure this is.
    pub fn kind(&self) -> &ErrorKind {
        &self.kind
    }

    /// The half the failure concerns, when it concerns one.
    pub fn half(&self) -> Option<Half> {
        self.half
    }

    /// The sentence to show: exactly what the CLI prints after `Error: `.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for UpdateError {
    fn fmt(&self, out: &mut fmt::Formatter<'_>) -> fmt::Result {
        out.write_str(&self.message)
    }
}

impl std::error::Error for UpdateError {}

/// The failures a front end may want to tell apart.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ErrorKind {
    /// The bundle file could not be read, or is not a bundle.
    Bundle,
    /// No bootloader showed up on USB in time.
    NoBootloader,
    /// A bootloader is plugged in and this user may not open it.
    AccessDenied,
    /// The batteries are too low to update over Bluetooth.
    LowBattery,
    /// The right half could not be reached through the left over Bluetooth,
    /// before anything was written.
    RadioLinkDown,
    /// The bundle's image for [`UpdateError::half`] is older than the one the
    /// half carries. Only checked over the cable; `force` writes it anyway.
    Downgrade,
    /// The half's bootloader refused the written image: a bad signature or a
    /// corrupt file.
    Rejected,
    /// The half stopped responding while it was being written.
    HalfGone,
    /// The pair did not come back on the bundle, and both halves were put back
    /// on the firmware they ran before.
    Reverted,
    /// The pair did not come back on the bundle, and these halves did not come
    /// back on their previous firmware either.
    NotReverted { halves: Vec<Half> },
    /// The pair did not come back on the bundle and could not be restarted to
    /// put it back: these halves never answered, or none when both answered on
    /// different builds.
    NotConfirmed { halves: Vec<Half> },
    /// Both halves kept the update, but the keyboard did not come back on USB
    /// with both halves reporting.
    NotBackOnUsb,
    /// Anything else.
    Other,
}

/// A marker left in a chain: the kind and half a failure is reported with.
///
/// Displays as the chain it wraps, and has no source of its own, so wrapping
/// changes nothing about the line a person reads.
#[derive(Debug)]
pub(crate) struct Tagged {
    kind: Option<ErrorKind>,
    half: Option<Half>,
    inner: anyhow::Error,
}

impl fmt::Display for Tagged {
    fn fmt(&self, out: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(out, "{:#}", self.inner)
    }
}

impl std::error::Error for Tagged {}

/// Marks `err` as concerning `half`.
pub(crate) fn in_half(half: Half, err: anyhow::Error) -> anyhow::Error {
    anyhow::Error::new(Tagged {
        kind: None,
        half: Some(half),
        inner: err,
    })
}

/// Marks `err` as a failure of `kind`, concerning `half` if it names one.
pub(crate) fn kind_of(kind: ErrorKind, half: Option<Half>, err: anyhow::Error) -> anyhow::Error {
    anyhow::Error::new(Tagged {
        kind: Some(kind),
        half,
        inner: err,
    })
}

/// The public error out of an orchestration failure, the outermost marker's
/// kind and half winning over any inside it.
pub(crate) fn classify(err: anyhow::Error) -> UpdateError {
    let message = format!("{err:#}");
    let mut kind = None;
    let mut half = None;
    visit(&err, &mut kind, &mut half);
    UpdateError {
        kind: kind.unwrap_or(ErrorKind::Other),
        half,
        message,
    }
}

fn visit(err: &anyhow::Error, kind: &mut Option<ErrorKind>, half: &mut Option<Half>) {
    for link in err.chain() {
        if let Some(tagged) = link.downcast_ref::<Tagged>() {
            if kind.is_none() {
                kind.clone_from(&tagged.kind);
            }
            if half.is_none() {
                *half = tagged.half;
            }
            visit(&tagged.inner, kind, half);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_marker_changes_nothing_about_the_line() {
        let err = anyhow::anyhow!("the board stopped taking the image at offset 5")
            .context("The right half's upload failed");
        let plain = format!("{err:#}");
        let marked = in_half(Half::Right, err).context("outside");
        assert_eq!(format!("{marked:#}"), format!("outside: {plain}"));

        let classified = classify(marked);
        assert_eq!(classified.message(), format!("outside: {plain}"));
        assert_eq!(classified.half(), Some(Half::Right));
        assert_eq!(classified.kind(), &ErrorKind::Other);
    }

    #[test]
    fn the_outermost_marker_wins_and_an_inner_one_fills_what_it_leaves() {
        let inner = kind_of(ErrorKind::Downgrade, None, anyhow::anyhow!("older"));
        let outer = in_half(Half::Left, inner);
        let classified = classify(outer);
        assert_eq!(classified.kind(), &ErrorKind::Downgrade);
        assert_eq!(classified.half(), Some(Half::Left));
        assert_eq!(classified.to_string(), "older");
    }
}
