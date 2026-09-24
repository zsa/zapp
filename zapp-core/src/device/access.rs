//! Opening a device that has just appeared.
//!
//! On Linux the kernel creates a new USB node as root's, and the udev rule
//! that opens it up to everyone runs a moment later. An open that lands in
//! between is refused, so a refusal right after a reset is usually that moment
//! rather than a missing rule.

use std::time::{Duration, Instant};

/// How long a device that refuses to open is given for the udev rule to reach
/// it, counted from the first refusal.
pub const ACCESS_GRACE: Duration = Duration::from_secs(3);

/// How long between tries at a device that refused to open.
pub const ACCESS_RETRY: Duration = Duration::from_millis(250);

/// How long a device has been refusing to open.
#[derive(Debug, Default)]
pub struct AccessGrace {
    since: Option<Instant>,
}

impl AccessGrace {
    /// Records a refusal at `now`; true while it is still worth trying again.
    pub fn retry(&mut self, now: Instant) -> bool {
        let since = *self.since.get_or_insert(now);
        now.saturating_duration_since(since) < ACCESS_GRACE
    }

    /// Forgets earlier refusals, for a device that went away in between.
    pub fn reset(&mut self) {
        self.since = None;
    }
}

/// Whether `nusb` refused an open for lack of permission.
///
/// The errno is checked as well as the kind: before 0.2.7, `nusb` files
/// EACCES, the errno a node without the udev rule gives, under
/// [`nusb::ErrorKind::Other`].
pub fn permission_denied(err: &nusb::Error) -> bool {
    if err.kind() == nusb::ErrorKind::PermissionDenied {
        return true;
    }
    // EPERM and EACCES.
    cfg!(target_os = "linux") && matches!(err.os_error(), Some(1 | 13))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_refusal_is_retried_inside_the_grace_window() {
        let start = Instant::now();
        let mut grace = AccessGrace::default();
        assert!(grace.retry(start));
        assert!(grace.retry(start + Duration::from_millis(2900)));
    }

    #[test]
    fn a_refusal_past_the_grace_window_is_the_answer() {
        let start = Instant::now();
        let mut grace = AccessGrace::default();
        assert!(grace.retry(start));
        assert!(!grace.retry(start + ACCESS_GRACE));
    }

    #[test]
    fn the_window_counts_from_the_first_refusal() {
        // A later refusal does not move the start along.
        let start = Instant::now();
        let mut grace = AccessGrace::default();
        assert!(grace.retry(start));
        assert!(grace.retry(start + Duration::from_secs(2)));
        assert!(!grace.retry(start + Duration::from_secs(3)));
    }

    #[test]
    fn a_device_that_went_away_starts_a_fresh_window() {
        let start = Instant::now();
        let mut grace = AccessGrace::default();
        assert!(grace.retry(start));
        grace.reset();
        assert!(grace.retry(start + Duration::from_secs(10)));
    }
}
