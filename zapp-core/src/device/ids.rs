/// ZSA vendor ID.
pub const ZSA_VID: u16 = 0x3297;

/// STMicroelectronics vendor ID (generic STM32 DFU bootloader).
pub const STM32_VID: u16 = 0x0483;
pub const STM32_DFU_PID: u16 = 0xDF11;

/// PJRC / Teensyduino vendor ID (HALFKAY bootloader).
pub const HALFKAY_VID: u16 = 0x16C0;
pub const HALFKAY_PID: u16 = 0x0478;

/// ZSA's BLE UUID space: `3297xxxx-e5aa-4f42-a596-1ea5a0200001`, keyed on the
/// vendor id. `32970001`-`32970004` is the firmware-update service and its
/// characteristics; per-product identity UUIDs start at `32970101`.
pub const ZSA_BLE_DFU_SERVICE: u128 = 0x32970001_e5aa_4f42_a596_1ea5a0200001;

/// Known ZSA keyboard models.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Keyboard {
    ErgodoxEz,
    PlanckEz,
    Moonlander,
    Voyager,
    Voyager2,
}

impl std::fmt::Display for Keyboard {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ErgodoxEz => write!(f, "Ergodox EZ"),
            Self::PlanckEz => write!(f, "Planck EZ"),
            Self::Moonlander => write!(f, "Moonlander"),
            Self::Voyager => write!(f, "Voyager"),
            Self::Voyager2 => write!(f, "Voyager 2"),
        }
    }
}

/// Bootloader protocol variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootloaderKind {
    /// Generic STM32 DFU bootloader (flash at 0x0800_0000).
    Stm32Dfu,
    /// ZSA Ignition bootloader on an STM32 chip (flash at 0x0800_2000).
    IgnitionStm32,
    /// ZSA Ignition bootloader on a GD32 chip (flash at 0x0800_2000).
    IgnitionGd32,
    /// ZSA Ignition v2 bootloader, 16 KiB reserved (flash at 0x0800_4000).
    /// Used by the Voyager 2.
    IgnitionV2,
    /// Legacy HALFKAY (Teensy) bootloader for original Ergodox EZ.
    Halfkay,
}

/// A bootloader device ready for flashing.
pub struct BootloaderDevice {
    pub device: nusb::Device,
    pub vid: u16,
    pub pid: u16,
    pub kind: BootloaderKind,
    pub keyboard: Option<Keyboard>,
}

/// Identify a bootloader from its VID/PID. Returns `None` if not a known bootloader.
pub fn identify_bootloader(vid: u16, pid: u16) -> Option<BootloaderKind> {
    match (vid, pid) {
        (HALFKAY_VID, HALFKAY_PID) => Some(BootloaderKind::Halfkay),
        (STM32_VID, STM32_DFU_PID) => Some(BootloaderKind::Stm32Dfu),
        (ZSA_VID, 0x0791) => Some(BootloaderKind::IgnitionStm32),
        (ZSA_VID, 0x1791) => Some(BootloaderKind::IgnitionGd32),
        (ZSA_VID, 0x0792) => Some(BootloaderKind::IgnitionV2),
        // Ignition Ergodox variants (all STM32)
        (ZSA_VID, 0x2000..=0x2002) => Some(BootloaderKind::IgnitionStm32),
        // Ignition Moonlander
        (ZSA_VID, 0x2003) => Some(BootloaderKind::IgnitionStm32),
        _ => None,
    }
}

/// Get the keyboard model for a bootloader PID.
pub fn keyboard_for_bootloader(vid: u16, pid: u16) -> Option<Keyboard> {
    match (vid, pid) {
        (HALFKAY_VID, HALFKAY_PID) => Some(Keyboard::ErgodoxEz),
        (STM32_VID, STM32_DFU_PID) => None, // generic, could be any STM32 keyboard
        (ZSA_VID, 0x0791 | 0x1791) => Some(Keyboard::Voyager),
        (ZSA_VID, 0x0792) => Some(Keyboard::Voyager2),
        (ZSA_VID, 0x2000..=0x2002) => Some(Keyboard::ErgodoxEz),
        (ZSA_VID, 0x2003) => Some(Keyboard::Moonlander),
        _ => None,
    }
}

/// Human-readable name for a bootloader device.
pub fn friendly_name(vid: u16, pid: u16) -> &'static str {
    match (vid, pid) {
        (HALFKAY_VID, HALFKAY_PID) => "Ergodox EZ (HALFKAY)",
        (STM32_VID, STM32_DFU_PID) => "Keyboard in Reset Mode (STM32 DFU)",
        (ZSA_VID, 0x0791) => "Voyager (Ignition STM32)",
        (ZSA_VID, 0x1791) => "Voyager (Ignition GD32)",
        (ZSA_VID, 0x0792) => "Voyager 2 (Ignition)",
        (ZSA_VID, 0x2000) => "Ergodox EZ Glow (Ignition)",
        (ZSA_VID, 0x2001) => "Ergodox EZ Shine (Ignition)",
        (ZSA_VID, 0x2002) => "Ergodox EZ Original (Ignition)",
        (ZSA_VID, 0x2003) => "Moonlander (Ignition)",
        _ => "Unknown",
    }
}

/// Identify a keyboard from its normal-mode VID/PID.
/// Returns `None` if not a known ZSA keyboard.
pub fn identify_keyboard(vid: u16, pid: u16) -> Option<Keyboard> {
    if vid != ZSA_VID {
        return None;
    }
    match pid {
        0x1307 | 0x4974 | 0x4975 | 0x4976 | 0x2010 | 0x2020 | 0x2030 => Some(Keyboard::ErgodoxEz),
        0x6060 | 0xC6CE | 0xC6CF => Some(Keyboard::PlanckEz),
        0x1969 | 0x1972 => Some(Keyboard::Moonlander),
        0x1977 => Some(Keyboard::Voyager),
        0x1978 => Some(Keyboard::Voyager2),
        _ => None,
    }
}

/// Identify a keyboard from the product identity UUID it advertises over BLE.
///
/// Advertised rather than a real GATT service, so it names the product before
/// anything connects. A bonded board stops advertising and is named by its DIS
/// PnP id instead, which carries the same PID `identify_keyboard` takes.
pub fn keyboard_for_ble_identity(uuid: u128) -> Option<Keyboard> {
    match uuid {
        0x32970101_e5aa_4f42_a596_1ea5a0200001 => Some(Keyboard::Voyager2),
        _ => None,
    }
}

/// Returns true if the PID corresponds to a Moonlander rev B.
pub fn is_moonlander_revb(pid: u16) -> bool {
    pid == 0x1972
}

/// Human-readable target name from a firmware PID (the PID embedded in the DFU suffix).
/// These are the normal-mode PIDs that identify which keyboard the firmware is built for.
pub fn target_name_for_pid(pid: u16) -> &'static str {
    match pid {
        0x1307 => "Ergodox EZ",
        0x4974 | 0x2030 => "Ergodox EZ Original",
        0x4975 | 0x2020 => "Ergodox EZ Shine",
        0x4976 | 0x2010 => "Ergodox EZ Glow",
        0x6060 => "Planck EZ",
        0xC6CE => "Planck EZ Standard",
        0xC6CF => "Planck EZ Glow",
        0x1969 | 0x1972 => "Moonlander MK1",
        0x1977 => "Voyager",
        0x1978 => "Voyager 2",
        // Bootloader PIDs that may appear in firmware suffixes
        0x0791 => "Voyager (STM32)",
        0x1791 => "Voyager (GD32)",
        0x0792 => "Voyager 2",
        0x2000 => "Ergodox EZ Glow",
        0x2001 => "Ergodox EZ Shine",
        0x2002 => "Ergodox EZ Original",
        0x2003 => "Moonlander MK1",
        0xDF11 => "STM32 DFU",
        _ => "Unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identify_bootloader() {
        assert_eq!(
            identify_bootloader(HALFKAY_VID, HALFKAY_PID),
            Some(BootloaderKind::Halfkay)
        );
        assert_eq!(
            identify_bootloader(STM32_VID, STM32_DFU_PID),
            Some(BootloaderKind::Stm32Dfu)
        );
        assert_eq!(
            identify_bootloader(ZSA_VID, 0x0791),
            Some(BootloaderKind::IgnitionStm32)
        );
        assert_eq!(
            identify_bootloader(ZSA_VID, 0x1791),
            Some(BootloaderKind::IgnitionGd32)
        );
        assert_eq!(
            identify_bootloader(ZSA_VID, 0x2003),
            Some(BootloaderKind::IgnitionStm32)
        );
        // Voyager 2 has its own kind — reusing IgnitionStm32 would flash at the wrong base
        assert_eq!(
            identify_bootloader(ZSA_VID, 0x0792),
            Some(BootloaderKind::IgnitionV2)
        );
        // Unknown device
        assert_eq!(identify_bootloader(0x1234, 0x5678), None);
    }

    #[test]
    fn test_keyboard_for_bootloader() {
        assert_eq!(
            keyboard_for_bootloader(HALFKAY_VID, HALFKAY_PID),
            Some(Keyboard::ErgodoxEz)
        );
        assert_eq!(
            keyboard_for_bootloader(ZSA_VID, 0x0791),
            Some(Keyboard::Voyager)
        );
        assert_eq!(
            keyboard_for_bootloader(ZSA_VID, 0x2003),
            Some(Keyboard::Moonlander)
        );
        // Generic STM32 DFU — no specific keyboard
        assert_eq!(keyboard_for_bootloader(STM32_VID, STM32_DFU_PID), None);
        assert_eq!(
            keyboard_for_bootloader(ZSA_VID, 0x0792),
            Some(Keyboard::Voyager2)
        );
    }

    #[test]
    fn the_voyager_2_has_its_own_product_id() {
        // qmk voyager/keyboard.json is 0x1977, voyager_tng/keyboard.json 0x1978.
        assert_eq!(identify_keyboard(ZSA_VID, 0x1977), Some(Keyboard::Voyager));
        assert_eq!(identify_keyboard(ZSA_VID, 0x1978), Some(Keyboard::Voyager2));
        assert_eq!(target_name_for_pid(0x1977), "Voyager");
        assert_eq!(target_name_for_pid(0x1978), "Voyager 2");
    }

    #[test]
    fn the_advertised_identity_uuid_names_the_product() {
        assert_eq!(
            keyboard_for_ble_identity(0x32970101_e5aa_4f42_a596_1ea5a0200001),
            Some(Keyboard::Voyager2)
        );
        // The DFU service says "a ZSA board", never which one.
        assert_eq!(keyboard_for_ble_identity(ZSA_BLE_DFU_SERVICE), None);
        assert_eq!(keyboard_for_ble_identity(0), None);
    }
}
