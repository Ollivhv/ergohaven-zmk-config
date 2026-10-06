//! Shared persistent device settings for Ergohaven firmware.
//!
//! Two groups of settings share one Vial `DeviceSettings` record:
//!
//! * **layer names** — Entropy discovers QSID 200..=215 through Vial's
//!   supported-settings query and reads/writes one UTF-8 name per layer;
//! * **Qube screen settings** — everything the dongle screen can draw
//!   (WPM panel, header content, modifier chips, battery cards, the
//!   connection indicator, software brightness, idle blanking, palette).
//!
//! The file keeps its historical name (`layer_names.rs`) because
//! `keyboards/classic_qube/build.rs` selects
//! `crate::layer_names::vial_device_settings` for every product except the
//! Velvet one. Renaming the module would mean touching the build script and
//! the `#[path]` include in `qube.rs` for no functional gain.
//!
//! # Storage layout
//!
//! One blob, at most `VIAL_DEVICE_SETTINGS_MAX_LEN` (224) bytes:
//!
//! ```text
//! off  size  field
//!   0     1  marker 0xE5
//!   1     1  version (6)
//!   2   176  layer names: 16 x [len: u8, 10 bytes]
//! 178     1  screen flags (bit0 wpm, bit1 modifiers, bit2 batteries, bit3 output)
//! 179     1  header mode (0 media, 1 clock, 2 media + clock)
//! 180     1  idle timeout, seconds (0 = never blank)
//! 181     1  brightness (10..=100)
//! 182     2  accent colour RGB565
//! 184     2  accent shadow colour RGB565
//! 186     2  background colour RGB565
//! 188     1  left battery label length
//! 189     6  left battery label bytes
//! 195     1  right battery label length
//! 196     6  right battery label bytes
//! 202     1  connection badge placement (0 header, 1 chip row)
//! 203     1  screen concept (0 dashboard v2, 1..=10 alternative layouts)
//! 204     2  panel colour RGB565
//! 206     2  border colour RGB565
//! 208     2  main text colour RGB565
//! 210     2  caption / muted text colour RGB565
//! 212     2  bar / scale fill colour RGB565
//! 214     2  low-battery (yellow) colour RGB565
//! 216     2  critical-battery (red) colour RGB565
//! 218        (end of record, 6 bytes of the 224-byte budget stay unused)
//! ```
//!
//! Colours are RGB565 words: every one is written by the client as three `u8`
//! components (R, G, B) through three consecutive QSIDs and stored as one word,
//! which saves a byte per colour against the version-5 layout. Version 5 (and
//! 3/4) stored three components per colour; such records are read with the same
//! conversion the panel uses for 8-bit colours, so a stored palette keeps its
//! look and no screen setting is lost.
//! Version 4 lacked the screen-concept byte, version 3 additionally lacked the
//! badge placement; all older records are still read and upgraded in place.
//! Version 2 used `LAYER_NAME_MAX = 12` (210 bytes of names, no screen
//! settings), version 1 additionally stored placeholder names that have to be
//! migrated to the factory profile. Both are still readable.

use core::str;
use core::sync::atomic::{AtomicU16, AtomicU8, Ordering};

use rmk::config::{VialDeviceSettings, VialDeviceSettingsData};

// --- Layer names -----------------------------------------------------------

pub const LAYER_NAME_COUNT: usize = 16;
/// Hard cap on one layer name. 10 bytes keep every factory name intact
/// ("Navigation" is exactly 10) while freeing 32 bytes of the settings blob
/// for the screen settings. Entropy truncates on the client side as well.
pub const LAYER_NAME_MAX: usize = 10;

const LAYER_NAME_QSID_BASE: u16 = 200;
const LEGACY_LAYER_NAME_MAX: usize = 12;

// --- Screen settings QSID map ---------------------------------------------

/// Show the WPM panel (`0`/`1`).
pub const QSID_SCREEN_WPM: u16 = 216;
/// Header content: `0` media, `1` clock, `2` media + clock.
pub const QSID_SCREEN_HEADER: u16 = 217;
/// Idle blanking timeout in seconds, `0` = never.
pub const QSID_SCREEN_TIMEOUT: u16 = 218;
/// Show the modifier chips (`0`/`1`).
pub const QSID_SCREEN_MODIFIERS: u16 = 219;
/// Show the battery cards (`0`/`1`).
pub const QSID_SCREEN_BATTERIES: u16 = 220;
/// Show the connection indicator (`0`/`1`).
pub const QSID_SCREEN_OUTPUT: u16 = 221;
/// Left / right battery card label (UTF-8, up to [`BATTERY_LABEL_MAX`] bytes).
pub const QSID_SCREEN_LEFT_LABEL: u16 = 222;
pub const QSID_SCREEN_RIGHT_LABEL: u16 = 223;
/// Dim variant of the accent colour, R/G/B (advanced, no Entropy UI yet).
pub const QSID_SCREEN_ACCENT_DIM: u16 = 224;
pub const QSID_SCREEN_ACCENT_DIM_G: u16 = QSID_SCREEN_ACCENT_DIM + 1;
pub const QSID_SCREEN_ACCENT_DIM_B: u16 = QSID_SCREEN_ACCENT_DIM + 2;
/// Where the connection badge lives: `0` right end of the header (default),
/// `1` sixth slot of the modifier row.
pub const QSID_SCREEN_OUTPUT_PLACE: u16 = 227;
/// Screen concept: `0` = dashboard v2 (the shipped layout), `1..=10` = one of
/// the alternative whole-screen layouts. **The numbers are a contract with
/// Entropy — never renumber them.**
pub const QSID_SCREEN_CONCEPT: u16 = 228;
/// Colour block: every colour is written as three `u8` components (R, G, B,
/// 0..=255) and stored as one RGB565 word. The numbers are a contract with the
/// Entropy page — never renumber them.
///
/// Panel: fill of every card / plate (`COL_PANEL`).
pub const QSID_SCREEN_PANEL: u16 = 230;
pub const QSID_SCREEN_PANEL_G: u16 = QSID_SCREEN_PANEL + 1;
pub const QSID_SCREEN_PANEL_B: u16 = QSID_SCREEN_PANEL + 2;
/// Borders and hairlines (`COL_BORDER`).
pub const QSID_SCREEN_BORDER: u16 = 233;
pub const QSID_SCREEN_BORDER_G: u16 = QSID_SCREEN_BORDER + 1;
pub const QSID_SCREEN_BORDER_B: u16 = QSID_SCREEN_BORDER + 2;
/// Main text (`COL_FG`).
pub const QSID_SCREEN_TEXT: u16 = 236;
pub const QSID_SCREEN_TEXT_G: u16 = QSID_SCREEN_TEXT + 1;
pub const QSID_SCREEN_TEXT_B: u16 = QSID_SCREEN_TEXT + 2;
/// Captions and muted text (`COL_MUTED`, `COL_LABEL`, `COL_DIM`).
pub const QSID_SCREEN_LABEL: u16 = 239;
pub const QSID_SCREEN_LABEL_G: u16 = QSID_SCREEN_LABEL + 1;
pub const QSID_SCREEN_LABEL_B: u16 = QSID_SCREEN_LABEL + 2;
/// Bar / scale fill (`COL_BAR_FG`).
pub const QSID_SCREEN_BAR: u16 = 242;
pub const QSID_SCREEN_BAR_G: u16 = QSID_SCREEN_BAR + 1;
pub const QSID_SCREEN_BAR_B: u16 = QSID_SCREEN_BAR + 2;
/// Low-battery threshold colour (`COL_YELLOW`).
pub const QSID_SCREEN_WARN: u16 = 245;
pub const QSID_SCREEN_WARN_G: u16 = QSID_SCREEN_WARN + 1;
pub const QSID_SCREEN_WARN_B: u16 = QSID_SCREEN_WARN + 2;
/// Critical-battery threshold colour (`COL_RED`).
pub const QSID_SCREEN_BAD: u16 = 248;
pub const QSID_SCREEN_BAD_G: u16 = QSID_SCREEN_BAD + 1;
pub const QSID_SCREEN_BAD_B: u16 = QSID_SCREEN_BAD + 2;

/// Screen colours. Each colour occupies three consecutive QSIDs (R, G, B — one
/// `u8` each, as the Ergohaven pages already do for the accent and background)
/// and is stored as one RGB565 word.
pub const COLOR_COUNT: usize = 10;
/// First (R) QSID of every colour, in the order of [`ScreenSettings::colors`].
pub const COLOR_FIRST_QSID: [u16; COLOR_COUNT] = [
    QSID_SCREEN_ACCENT,
    QSID_SCREEN_ACCENT_DIM,
    QSID_SCREEN_BACKGROUND,
    QSID_SCREEN_PANEL,
    QSID_SCREEN_BORDER,
    QSID_SCREEN_TEXT,
    QSID_SCREEN_LABEL,
    QSID_SCREEN_BAR,
    QSID_SCREEN_WARN,
    QSID_SCREEN_BAD,
];
/// Names of the same ten roles, for logs and the host harness.
pub const COLOR_ROLES: [&str; COLOR_COUNT] = [
    "accent",
    "accent-shadow",
    "background",
    "panel",
    "border",
    "text",
    "label",
    "bar",
    "warning",
    "critical",
];
/// Software brightness, 10..=100. Same QSID the Ergohaven LCD screens use.
pub const QSID_SCREEN_BRIGHTNESS: u16 = 318;
/// Accent colour R/G/B. Same QSID pair group as the Ergohaven LCD text colour.
pub const QSID_SCREEN_ACCENT: u16 = 320;
pub const QSID_SCREEN_ACCENT_G: u16 = QSID_SCREEN_ACCENT + 1;
pub const QSID_SCREEN_ACCENT_B: u16 = QSID_SCREEN_ACCENT + 2;
/// Background colour R/G/B. Same QSID triple as the Ergohaven LCD background.
pub const QSID_SCREEN_BACKGROUND: u16 = 330;
pub const QSID_SCREEN_BACKGROUND_G: u16 = QSID_SCREEN_BACKGROUND + 1;
pub const QSID_SCREEN_BACKGROUND_B: u16 = QSID_SCREEN_BACKGROUND + 2;

pub const SCREEN_HEADER_MEDIA: u8 = 0;
pub const SCREEN_HEADER_CLOCK: u8 = 1;
pub const SCREEN_HEADER_BOTH: u8 = 2;

pub const SCREEN_OUTPUT_HEADER: u8 = 0;
pub const SCREEN_OUTPUT_CHIP: u8 = 1;

// Screen concepts, in the order Entropy lists them (QSID 228).
pub const SCREEN_CONCEPT_DASHBOARD: u8 = 0;
pub const SCREEN_CONCEPT_HUD: u8 = 1;
pub const SCREEN_CONCEPT_TERMINAL: u8 = 2;
pub const SCREEN_CONCEPT_MINIMAL: u8 = 3;
pub const SCREEN_CONCEPT_TILES: u8 = 4;
pub const SCREEN_CONCEPT_SPEEDO: u8 = 5;
pub const SCREEN_CONCEPT_INFOCENTER: u8 = 6;
pub const SCREEN_CONCEPT_TWOCOL: u8 = 7;
pub const SCREEN_CONCEPT_SPARKLINE: u8 = 8;
pub const SCREEN_CONCEPT_SIGNAL: u8 = 9;
pub const SCREEN_CONCEPT_MOOD: u8 = 10;
/// Highest valid concept id; anything above falls back to the dashboard.
pub const SCREEN_CONCEPT_MAX: u8 = SCREEN_CONCEPT_MOOD;

pub const SCREEN_BRIGHTNESS_MIN: u8 = 10;
pub const SCREEN_BRIGHTNESS_MAX: u8 = 100;

/// Longest battery card label; "RIGHT" (5 bytes) already needs more than the
/// four bytes a shorter cap would allow.
pub const BATTERY_LABEL_MAX: usize = 6;

/// Factory palette, as 8-bit components (R, G, B). Each value is the 8-bit
/// expansion of the constant the screen used before colours became settings, so
/// the RGB565 word stored from it reproduces that constant exactly and a device
/// with factory settings renders the previous frame (asserted in
/// `qube_display.rs`).
pub const DEFAULT_ACCENT: [u8; 3] = [24, 154, 255];
pub const DEFAULT_ACCENT_DIM: [u8; 3] = [8, 65, 148];
pub const DEFAULT_BACKGROUND: [u8; 3] = [0, 8, 33];
pub const DEFAULT_PANEL: [u8; 3] = [16, 24, 74];
pub const DEFAULT_BORDER: [u8; 3] = [41, 52, 132];
pub const DEFAULT_TEXT: [u8; 3] = [239, 247, 247];
pub const DEFAULT_LABEL: [u8; 3] = [90, 97, 165];
pub const DEFAULT_BAR: [u8; 3] = [24, 170, 247];
pub const DEFAULT_WARN: [u8; 3] = [255, 203, 0];
pub const DEFAULT_BAD: [u8; 3] = [255, 21, 42];

/// The factory palette in the stored format, in [`COLOR_ROLES`] order.
pub const DEFAULT_COLORS: [u16; COLOR_COUNT] = [
    rgb8_to_565(DEFAULT_ACCENT),
    rgb8_to_565(DEFAULT_ACCENT_DIM),
    rgb8_to_565(DEFAULT_BACKGROUND),
    rgb8_to_565(DEFAULT_PANEL),
    rgb8_to_565(DEFAULT_BORDER),
    rgb8_to_565(DEFAULT_TEXT),
    rgb8_to_565(DEFAULT_LABEL),
    rgb8_to_565(DEFAULT_BAR),
    rgb8_to_565(DEFAULT_WARN),
    rgb8_to_565(DEFAULT_BAD),
];

/// 8-bit components → RGB565 word (the same rounding the panel uses).
pub const fn rgb8_to_565(color: [u8; 3]) -> u16 {
    (((color[0] >> 3) as u16) << 11) | (((color[1] >> 2) as u16) << 5) | ((color[2] >> 3) as u16)
}

/// RGB565 word → 8-bit components, so a client that writes R/G/B reads back a
/// value within one 5/6-bit step of what it wrote.
pub const fn rgb565_components(word: u16) -> [u8; 3] {
    let r = ((word >> 11) & 0x1F) as u8;
    let g = ((word >> 5) & 0x3F) as u8;
    let b = (word & 0x1F) as u8;
    [
        (r << 3) | (r >> 2),
        (g << 2) | (g >> 4),
        (b << 3) | (b >> 2),
    ]
}

/// Replaces one 8-bit component of an RGB565 word.
fn rgb565_with_component(word: u16, component: usize, value: u8) -> u16 {
    match component {
        0 => (word & 0x07FF) | (((value >> 3) as u16) << 11),
        1 => (word & 0xF81F) | (((value >> 2) as u16) << 5),
        _ => (word & 0xFFE0) | ((value >> 3) as u16),
    }
}

const SCREEN_FLAG_WPM: u8 = 1 << 0;
const SCREEN_FLAG_MODIFIERS: u8 = 1 << 1;
const SCREEN_FLAG_BATTERIES: u8 = 1 << 2;
const SCREEN_FLAG_OUTPUT: u8 = 1 << 3;

const STORAGE_MARKER: u8 = 0xE5;
/// Current record version. Bump together with the layout below.
const STORAGE_VERSION: u8 = 6;
/// Version 5 stored every colour as three `u8` components and had no colour
/// block; its record is 11 bytes shorter than the current one.
const STORAGE_VERSION_V5: u8 = 5;
/// Version 4 shipped every field except the screen concept (one byte shorter).
const STORAGE_VERSION_V4: u8 = 4;
/// Version 3 shared every field except the badge placement (its record is one
/// byte shorter and is still accepted).
const STORAGE_VERSION_V3: u8 = 3;
const LEGACY_STORAGE_VERSION_V2: u8 = 2;
const LEGACY_STORAGE_VERSION_V1: u8 = 1;
const STORAGE_HEADER_LEN: usize = 2;

const LAYER_NAMES_OFFSET: usize = STORAGE_HEADER_LEN;
const LAYER_NAMES_LEN: usize = LAYER_NAME_COUNT * (1 + LAYER_NAME_MAX);
const FLAGS_OFFSET: usize = LAYER_NAMES_OFFSET + LAYER_NAMES_LEN;
const HEADER_OFFSET: usize = FLAGS_OFFSET + 1;
const TIMEOUT_OFFSET: usize = HEADER_OFFSET + 1;
const BRIGHTNESS_OFFSET: usize = TIMEOUT_OFFSET + 1;
const ACCENT_OFFSET: usize = BRIGHTNESS_OFFSET + 1;
const ACCENT_DIM_OFFSET: usize = ACCENT_OFFSET + 2;
const BACKGROUND_OFFSET: usize = ACCENT_DIM_OFFSET + 2;
const LEFT_LABEL_LEN_OFFSET: usize = BACKGROUND_OFFSET + 2;
const LEFT_LABEL_OFFSET: usize = LEFT_LABEL_LEN_OFFSET + 1;
const RIGHT_LABEL_LEN_OFFSET: usize = LEFT_LABEL_OFFSET + BATTERY_LABEL_MAX;
const RIGHT_LABEL_OFFSET: usize = RIGHT_LABEL_LEN_OFFSET + 1;
const PLACE_OFFSET: usize = RIGHT_LABEL_OFFSET + BATTERY_LABEL_MAX;
const CONCEPT_OFFSET: usize = PLACE_OFFSET + 1;
const PANEL_OFFSET: usize = CONCEPT_OFFSET + 1;
const BORDER_OFFSET: usize = PANEL_OFFSET + 2;
const TEXT_OFFSET: usize = BORDER_OFFSET + 2;
const LABEL_OFFSET: usize = TEXT_OFFSET + 2;
const BAR_OFFSET: usize = LABEL_OFFSET + 2;
const WARN_OFFSET: usize = BAR_OFFSET + 2;
const BAD_OFFSET: usize = WARN_OFFSET + 2;
pub(crate) const SERIALIZED_LEN: usize = BAD_OFFSET + 2;

// Offsets of the version 3..5 record: colours as three components and no
// colour block behind the concept byte.
const LEGACY_ACCENT_OFFSET: usize = BRIGHTNESS_OFFSET + 1;
const LEGACY_ACCENT_DIM_OFFSET: usize = LEGACY_ACCENT_OFFSET + 3;
const LEGACY_BACKGROUND_OFFSET: usize = LEGACY_ACCENT_DIM_OFFSET + 3;
/// Length of a version-5 record (3-byte colours, placement + concept bytes).
const SERIALIZED_LEN_V5: usize = 207;
/// Placement and concept bytes as they sat in the version 3..5 layout.
const LEGACY_PLACE_OFFSET: usize = 205;
const LEGACY_CONCEPT_OFFSET: usize = 206;
/// Length of a version-4 record (same layout without the concept byte).
const SERIALIZED_LEN_V4: usize = 206;
/// Length of a version-3 record (same layout without the placement byte).
const SERIALIZED_LEN_V3: usize = 205;

/// Length of a version-1/2 record (12-byte layer names, no screen settings).
const LEGACY_SERIALIZED_LEN: usize =
    STORAGE_HEADER_LEN + LAYER_NAME_COUNT * (1 + LEGACY_LAYER_NAME_MAX);

const _: () = assert!(SERIALIZED_LEN <= 224);
const _: () = assert!(SERIALIZED_LEN <= u8::MAX as usize);
// Version detection matches on the version byte, so a longer record than the
// legacy one is fine: those records carry version 1/2.
const _: () = assert!(SERIALIZED_LEN_V5 < SERIALIZED_LEN);

/// Every key the firmware answers. Entropy walks this list with a
/// "first key greater than the last one" query, so it must stay sorted
/// ascending.
const SETTING_KEYS: [u16; 57] = [
    // Layer names.
    200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,
    // Qube screen block.
    QSID_SCREEN_WPM,
    QSID_SCREEN_HEADER,
    QSID_SCREEN_TIMEOUT,
    QSID_SCREEN_MODIFIERS,
    QSID_SCREEN_BATTERIES,
    QSID_SCREEN_OUTPUT,
    QSID_SCREEN_LEFT_LABEL,
    QSID_SCREEN_RIGHT_LABEL,
    QSID_SCREEN_ACCENT_DIM,
    QSID_SCREEN_ACCENT_DIM_G,
    QSID_SCREEN_ACCENT_DIM_B,
    QSID_SCREEN_OUTPUT_PLACE,
    QSID_SCREEN_CONCEPT,
    // Colour block (three QSIDs per colour: R, G, B).
    QSID_SCREEN_PANEL,
    QSID_SCREEN_PANEL_G,
    QSID_SCREEN_PANEL_B,
    QSID_SCREEN_BORDER,
    QSID_SCREEN_BORDER_G,
    QSID_SCREEN_BORDER_B,
    QSID_SCREEN_TEXT,
    QSID_SCREEN_TEXT_G,
    QSID_SCREEN_TEXT_B,
    QSID_SCREEN_LABEL,
    QSID_SCREEN_LABEL_G,
    QSID_SCREEN_LABEL_B,
    QSID_SCREEN_BAR,
    QSID_SCREEN_BAR_G,
    QSID_SCREEN_BAR_B,
    QSID_SCREEN_WARN,
    QSID_SCREEN_WARN_G,
    QSID_SCREEN_WARN_B,
    QSID_SCREEN_BAD,
    QSID_SCREEN_BAD_G,
    QSID_SCREEN_BAD_B,
    // Palette settings shared with the Ergohaven LCD numbering.
    QSID_SCREEN_BRIGHTNESS,
    QSID_SCREEN_ACCENT,
    QSID_SCREEN_ACCENT_G,
    QSID_SCREEN_ACCENT_B,
    QSID_SCREEN_BACKGROUND,
    QSID_SCREEN_BACKGROUND_G,
    QSID_SCREEN_BACKGROUND_B,
];

// --- State -----------------------------------------------------------------

static LAYER_NAME_LEN: [AtomicU8; LAYER_NAME_COUNT] = [const { AtomicU8::new(0) }; LAYER_NAME_COUNT];
static LAYER_NAME_BYTES: [AtomicU8; LAYER_NAME_COUNT * LAYER_NAME_MAX] =
    [const { AtomicU8::new(0) }; LAYER_NAME_COUNT * LAYER_NAME_MAX];
static LAYER_NAMES_VERSION: AtomicU8 = AtomicU8::new(0);

static SCREEN_FLAGS: AtomicU8 = AtomicU8::new(0);
static SCREEN_HEADER: AtomicU8 = AtomicU8::new(SCREEN_HEADER_BOTH);
static SCREEN_TIMEOUT: AtomicU8 = AtomicU8::new(0);
static SCREEN_PLACE: AtomicU8 = AtomicU8::new(SCREEN_OUTPUT_HEADER);
static SCREEN_CONCEPT: AtomicU8 = AtomicU8::new(SCREEN_CONCEPT_DASHBOARD);
static SCREEN_BRIGHTNESS: AtomicU8 = AtomicU8::new(SCREEN_BRIGHTNESS_MAX);
/// Screen colours as RGB565 words (the panel's native format).
static SCREEN_COLORS: [AtomicU16; COLOR_COUNT] = [
    AtomicU16::new(DEFAULT_COLORS[0]),
    AtomicU16::new(DEFAULT_COLORS[1]),
    AtomicU16::new(DEFAULT_COLORS[2]),
    AtomicU16::new(DEFAULT_COLORS[3]),
    AtomicU16::new(DEFAULT_COLORS[4]),
    AtomicU16::new(DEFAULT_COLORS[5]),
    AtomicU16::new(DEFAULT_COLORS[6]),
    AtomicU16::new(DEFAULT_COLORS[7]),
    AtomicU16::new(DEFAULT_COLORS[8]),
    AtomicU16::new(DEFAULT_COLORS[9]),
];
static BATTERY_LABEL_LEN: [AtomicU8; 2] = [AtomicU8::new(0), AtomicU8::new(0)];
static BATTERY_LABEL_BYTES: [AtomicU8; 2 * BATTERY_LABEL_MAX] =
    [const { AtomicU8::new(0) }; 2 * BATTERY_LABEL_MAX];
/// `0` means "nothing stored yet": readers then see [`ScreenSettings::default`].
static SCREEN_READY: AtomicU8 = AtomicU8::new(0);
static SCREEN_SETTINGS_VERSION: AtomicU8 = AtomicU8::new(0);

// --- Public data types -----------------------------------------------------

/// One battery card label, stored as bytes plus length.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BatteryLabel {
    bytes: [u8; BATTERY_LABEL_MAX],
    len: u8,
}

impl BatteryLabel {
    pub const fn empty() -> Self {
        Self {
            bytes: [0; BATTERY_LABEL_MAX],
            len: 0,
        }
    }

    pub fn as_str(&self) -> &str {
        let len = (self.len as usize).min(BATTERY_LABEL_MAX);
        str::from_utf8(&self.bytes[..len]).unwrap_or("")
    }
}

impl Default for BatteryLabel {
    fn default() -> Self {
        Self::empty()
    }
}

impl core::fmt::Debug for BatteryLabel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Snapshot of every screen setting, cheap to copy into the renderer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ScreenSettings {
    pub wpm_visible: bool,
    /// `0` media, `1` clock, `2` media + clock.
    pub header_mode: u8,
    /// Idle blanking timeout in seconds; `0` never blanks.
    pub timeout_s: u8,
    pub show_modifiers: bool,
    pub show_batteries: bool,
    /// Connection indicator (`USB` / `BT n`) in the header.
    pub output_visible: bool,
    /// Badge placement: [`SCREEN_OUTPUT_HEADER`] or [`SCREEN_OUTPUT_CHIP`].
    pub output_place: u8,
    /// Screen concept: [`SCREEN_CONCEPT_DASHBOARD`] or `1..=SCREEN_CONCEPT_MAX`.
    pub concept: u8,
    /// Software dimming of the finished frame, `10..=100`.
    pub brightness: u8,
    /// Colours as RGB565 words, in [`COLOR_ROLES`] order: accent, accent shadow,
    /// background, panel, border, text, label, bar, warning, critical.
    pub colors: [u16; COLOR_COUNT],
    pub left_label: BatteryLabel,
    pub right_label: BatteryLabel,
}

impl ScreenSettings {
    /// One colour as 8-bit components, for clients that write R/G/B.
    pub const fn color_components(&self, index: usize) -> [u8; 3] {
        rgb565_components(self.colors[index])
    }

    /// Whether the header decodes a media ticker at all.
    pub const fn shows_media(&self) -> bool {
        matches!(self.header_mode, SCREEN_HEADER_MEDIA | SCREEN_HEADER_BOTH)
    }

    /// Whether the header draws the clock when the host provides one.
    pub const fn shows_clock(&self) -> bool {
        matches!(self.header_mode, SCREEN_HEADER_CLOCK | SCREEN_HEADER_BOTH)
    }

}

/// Upper bound of the media ticker window. Layout v2 derives the real window
/// from the header geometry (badge slot, clock width) and never exceeds this.
pub const MEDIA_VISIBLE_CHARS: usize = 26;

impl Default for ScreenSettings {
    fn default() -> Self {
        Self {
            wpm_visible: true,
            header_mode: SCREEN_HEADER_BOTH,
            timeout_s: 0,
            show_modifiers: true,
            show_batteries: true,
            output_visible: true,
            output_place: SCREEN_OUTPUT_HEADER,
            concept: SCREEN_CONCEPT_DASHBOARD,
            brightness: SCREEN_BRIGHTNESS_MAX,
            colors: DEFAULT_COLORS,
            left_label: default_label(0),
            right_label: default_label(1),
        }
    }
}

const DEFAULT_LABELS: [[u8; BATTERY_LABEL_MAX]; 2] = [*b"LEFT\0\0", *b"RIGHT\0"];
const DEFAULT_LABEL_LENS: [u8; 2] = [4, 5];

const fn default_label(index: usize) -> BatteryLabel {
    BatteryLabel {
        bytes: DEFAULT_LABELS[index],
        len: DEFAULT_LABEL_LENS[index],
    }
}

// --- Vial device settings --------------------------------------------------

pub const fn vial_device_settings() -> VialDeviceSettings<'static> {
    VialDeviceSettings {
        setting_keys: &SETTING_KEYS,
        get_setting,
        set_setting,
        serialize,
        deserialize,
    }
}

#[allow(dead_code)]
pub fn version() -> u8 {
    LAYER_NAMES_VERSION.load(Ordering::Relaxed)
}

/// Bumped whenever any screen setting changes; the screen polls this to know
/// when to re-read the snapshot and repaint.
#[allow(dead_code)]
pub fn screen_settings_version() -> u8 {
    SCREEN_SETTINGS_VERSION.load(Ordering::Relaxed)
}

#[allow(dead_code)]
pub fn screen_settings() -> ScreenSettings {
    if SCREEN_READY.load(Ordering::Acquire) == 0 {
        return ScreenSettings::default();
    }
    let flags = SCREEN_FLAGS.load(Ordering::Relaxed);
    let mut colors = DEFAULT_COLORS;
    for (index, slot) in colors.iter_mut().enumerate() {
        *slot = SCREEN_COLORS[index].load(Ordering::Relaxed);
    }
    ScreenSettings {
        wpm_visible: flags & SCREEN_FLAG_WPM != 0,
        header_mode: SCREEN_HEADER.load(Ordering::Relaxed).min(SCREEN_HEADER_BOTH),
        timeout_s: SCREEN_TIMEOUT.load(Ordering::Relaxed),
        show_modifiers: flags & SCREEN_FLAG_MODIFIERS != 0,
        show_batteries: flags & SCREEN_FLAG_BATTERIES != 0,
        output_visible: flags & SCREEN_FLAG_OUTPUT != 0,
        output_place: SCREEN_PLACE.load(Ordering::Relaxed).min(SCREEN_OUTPUT_CHIP),
        concept: SCREEN_CONCEPT.load(Ordering::Relaxed).min(SCREEN_CONCEPT_MAX),
        brightness: SCREEN_BRIGHTNESS
            .load(Ordering::Relaxed)
            .clamp(SCREEN_BRIGHTNESS_MIN, SCREEN_BRIGHTNESS_MAX),
        colors,
        left_label: read_battery_label(0),
        right_label: read_battery_label(1),
    }
}

#[allow(dead_code)]
pub fn copy_layer_name(layer: u8, out: &mut [u8; LAYER_NAME_MAX]) -> Option<usize> {
    let index = usize::from(layer);
    if index >= LAYER_NAME_COUNT {
        return None;
    }

    let len = usize::from(LAYER_NAME_LEN[index].load(Ordering::Acquire));
    if len == 0 || len > LAYER_NAME_MAX {
        return None;
    }

    out.fill(0);
    let base = index * LAYER_NAME_MAX;
    for (offset, byte) in out.iter_mut().take(len).enumerate() {
        *byte = LAYER_NAME_BYTES[base + offset].load(Ordering::Relaxed);
    }
    str::from_utf8(&out[..len]).ok().map(|_| len)
}

// --- get/set ---------------------------------------------------------------

pub(crate) fn get_setting(qsid: u16, out: &mut [u8]) -> Option<usize> {
    if let Some(index) = layer_index(qsid) {
        return get_layer_name(index, out);
    }

    match qsid {
        QSID_SCREEN_LEFT_LABEL => return get_battery_label(0, out),
        QSID_SCREEN_RIGHT_LABEL => return get_battery_label(1, out),
        _ => {}
    }

    let settings = screen_settings();
    if let Some((index, component)) = color_lookup(qsid) {
        let value = settings.color_components(index)[component];
        if out.is_empty() {
            return Some(0);
        }
        out[0] = value;
        return Some(1);
    }
    let value = match qsid {
        QSID_SCREEN_WPM => settings.wpm_visible as u8,
        QSID_SCREEN_HEADER => settings.header_mode,
        QSID_SCREEN_TIMEOUT => settings.timeout_s,
        QSID_SCREEN_MODIFIERS => settings.show_modifiers as u8,
        QSID_SCREEN_BATTERIES => settings.show_batteries as u8,
        QSID_SCREEN_OUTPUT => settings.output_visible as u8,
        QSID_SCREEN_OUTPUT_PLACE => settings.output_place.min(SCREEN_OUTPUT_CHIP),
        QSID_SCREEN_CONCEPT => settings.concept.min(SCREEN_CONCEPT_MAX),
        QSID_SCREEN_BRIGHTNESS => settings.brightness,
        _ => return None,
    };
    if out.is_empty() {
        return Some(0);
    }
    out[0] = value;
    Some(1)
}

pub(crate) fn set_setting(qsid: u16, value: &[u8]) -> bool {
    if let Some(index) = layer_index(qsid) {
        if value.is_empty() {
            return false;
        }
        let end = value
            .iter()
            .position(|&byte| byte == 0 || byte == 0xFF)
            .unwrap_or(value.len());
        let Ok(text) = str::from_utf8(&value[..end]) else {
            return false;
        };
        store_layer_name(index, text);
        return true;
    }

    match qsid {
        QSID_SCREEN_LEFT_LABEL => return set_battery_label(0, value),
        QSID_SCREEN_RIGHT_LABEL => return set_battery_label(1, value),
        _ => {}
    }

    let Some(&first) = value.first() else {
        return false;
    };
    // A first write must start from the factory values, otherwise the fields
    // the client has not touched yet would stay at their zero initialisers.
    ensure_screen_loaded();
    if let Some((index, component)) = color_lookup(qsid) {
        let word = SCREEN_COLORS[index].load(Ordering::Relaxed);
        SCREEN_COLORS[index].store(rgb565_with_component(word, component, first), Ordering::Relaxed);
        SCREEN_SETTINGS_VERSION.fetch_add(1, Ordering::Relaxed);
        return true;
    }
    let stored = match qsid {
        QSID_SCREEN_WPM => update_flags(SCREEN_FLAG_WPM, first != 0),
        QSID_SCREEN_MODIFIERS => update_flags(SCREEN_FLAG_MODIFIERS, first != 0),
        QSID_SCREEN_BATTERIES => update_flags(SCREEN_FLAG_BATTERIES, first != 0),
        QSID_SCREEN_OUTPUT => update_flags(SCREEN_FLAG_OUTPUT, first != 0),
        QSID_SCREEN_OUTPUT_PLACE => store_screen(PLACE_OFFSET, first.min(SCREEN_OUTPUT_CHIP)),
        QSID_SCREEN_CONCEPT => store_screen(CONCEPT_OFFSET, first.min(SCREEN_CONCEPT_MAX)),
        QSID_SCREEN_HEADER => store_screen(HEADER_OFFSET, first.min(SCREEN_HEADER_BOTH)),
        QSID_SCREEN_TIMEOUT => store_screen(TIMEOUT_OFFSET, first),
        QSID_SCREEN_BRIGHTNESS => {
            SCREEN_BRIGHTNESS.store(
                first.clamp(SCREEN_BRIGHTNESS_MIN, SCREEN_BRIGHTNESS_MAX),
                Ordering::Relaxed,
            );
            true
        }
        _ => false,
    };
    if stored {
        SCREEN_SETTINGS_VERSION.fetch_add(1, Ordering::Relaxed);
    }
    stored
}

// --- Serialization ---------------------------------------------------------

pub(crate) fn serialize() -> VialDeviceSettingsData {
    let mut data = VialDeviceSettingsData::empty();
    data.data[0] = STORAGE_MARKER;
    data.data[1] = STORAGE_VERSION;

    let mut pos = LAYER_NAMES_OFFSET;
    for index in 0..LAYER_NAME_COUNT {
        let len = usize::from(LAYER_NAME_LEN[index].load(Ordering::Acquire)).min(LAYER_NAME_MAX);
        data.data[pos] = len as u8;
        pos += 1;
        let base = index * LAYER_NAME_MAX;
        for offset in 0..LAYER_NAME_MAX {
            data.data[pos + offset] = LAYER_NAME_BYTES[base + offset].load(Ordering::Relaxed);
        }
        pos += LAYER_NAME_MAX;
    }

    let settings = screen_settings();
    let mut flags = 0u8;
    flags |= if settings.wpm_visible { SCREEN_FLAG_WPM } else { 0 };
    flags |= if settings.show_modifiers {
        SCREEN_FLAG_MODIFIERS
    } else {
        0
    };
    flags |= if settings.show_batteries {
        SCREEN_FLAG_BATTERIES
    } else {
        0
    };
    flags |= if settings.output_visible {
        SCREEN_FLAG_OUTPUT
    } else {
        0
    };
    data.len = SERIALIZED_LEN as u8;
    data.data[FLAGS_OFFSET] = flags;
    data.data[HEADER_OFFSET] = settings.header_mode.min(SCREEN_HEADER_BOTH);
    data.data[TIMEOUT_OFFSET] = settings.timeout_s;
    data.data[BRIGHTNESS_OFFSET] = settings.brightness;
    for (index, offset) in COLOR_OFFSETS.iter().enumerate() {
        data.data[*offset..*offset + 2].copy_from_slice(&settings.colors[index].to_le_bytes());
    }
    data.data[LEFT_LABEL_LEN_OFFSET] = settings.left_label.len;
    data.data[LEFT_LABEL_OFFSET..LEFT_LABEL_OFFSET + BATTERY_LABEL_MAX]
        .copy_from_slice(&settings.left_label.bytes);
    data.data[RIGHT_LABEL_LEN_OFFSET] = settings.right_label.len;
    data.data[RIGHT_LABEL_OFFSET..RIGHT_LABEL_OFFSET + BATTERY_LABEL_MAX]
        .copy_from_slice(&settings.right_label.bytes);
    data.data[PLACE_OFFSET] = settings.output_place.min(SCREEN_OUTPUT_CHIP);
    data.data[CONCEPT_OFFSET] = settings.concept.min(SCREEN_CONCEPT_MAX);
    data
}

pub(crate) fn deserialize(bytes: &[u8]) {
    clear_layer_names();
    if bytes.len() < STORAGE_HEADER_LEN || bytes[0] != STORAGE_MARKER {
        load_defaults();
        bump_versions();
        return;
    }

    match (bytes[1], bytes.len()) {
        (STORAGE_VERSION, len) if len >= SERIALIZED_LEN => {
            store_screen_settings(&stored_screen_settings(
                bytes,
                bytes[PLACE_OFFSET],
                bytes[CONCEPT_OFFSET],
            ));
        }
        // Version 5 stored every colour as three components and had no colour
        // block: read the old fields and convert them to RGB565.
        (STORAGE_VERSION_V5, len) if len >= SERIALIZED_LEN_V5 => {
            store_screen_settings(&stored_screen_settings_legacy(
                bytes,
                bytes[LEGACY_PLACE_OFFSET],
                bytes[LEGACY_CONCEPT_OFFSET],
            ));
        }
        // Version 4 shipped every screen field except the concept.
        (STORAGE_VERSION_V4, len) if len >= SERIALIZED_LEN_V4 => {
            store_screen_settings(&stored_screen_settings_legacy(
                bytes,
                bytes[LEGACY_PLACE_OFFSET],
                SCREEN_CONCEPT_DASHBOARD,
            ));
        }
        // Version 3 shipped every screen field except the badge placement.
        (STORAGE_VERSION_V3, len) if len >= SERIALIZED_LEN_V3 => {
            store_screen_settings(&stored_screen_settings_legacy(
                bytes,
                SCREEN_OUTPUT_HEADER,
                SCREEN_CONCEPT_DASHBOARD,
            ));
        }
        (LEGACY_STORAGE_VERSION_V2, len) if len >= LEGACY_SERIALIZED_LEN => {
            read_layer_names(
                bytes,
                STORAGE_HEADER_LEN,
                LEGACY_LAYER_NAME_MAX,
                LAYER_NAME_COUNT,
            );
            store_screen_settings(&ScreenSettings::default());
        }
        (LEGACY_STORAGE_VERSION_V1, len) if len >= LEGACY_SERIALIZED_LEN => {
            read_layer_names(
                bytes,
                STORAGE_HEADER_LEN,
                LEGACY_LAYER_NAME_MAX,
                LAYER_NAME_COUNT,
            );
            migrate_legacy_placeholders();
            store_screen_settings(&ScreenSettings::default());
        }
        _ => load_defaults(),
    }
    bump_versions();
}

/// Offsets of the ten colours in the current record, in [`COLOR_ROLES`] order.
const COLOR_OFFSETS: [usize; COLOR_COUNT] = [
    ACCENT_OFFSET,
    ACCENT_DIM_OFFSET,
    BACKGROUND_OFFSET,
    PANEL_OFFSET,
    BORDER_OFFSET,
    TEXT_OFFSET,
    LABEL_OFFSET,
    BAR_OFFSET,
    WARN_OFFSET,
    BAD_OFFSET,
];

/// Decodes the screen block of the current record: colours as RGB565 words.
fn stored_screen_settings(bytes: &[u8], output_place: u8, concept: u8) -> ScreenSettings {
    read_layer_names(bytes, LAYER_NAMES_OFFSET, LAYER_NAME_MAX, LAYER_NAME_COUNT);
    let mut colors = DEFAULT_COLORS;
    for (index, offset) in COLOR_OFFSETS.iter().enumerate() {
        colors[index] = u16::from_le_bytes([bytes[*offset], bytes[*offset + 1]]);
    }
    // A truncated (or empty) screen block must not leak zeros: prefer the
    // factory defaults whenever the stored bytes are missing.
    ScreenSettings {
        wpm_visible: bytes[FLAGS_OFFSET] & SCREEN_FLAG_WPM != 0,
        header_mode: bytes[HEADER_OFFSET].min(SCREEN_HEADER_BOTH),
        timeout_s: bytes[TIMEOUT_OFFSET],
        show_modifiers: bytes[FLAGS_OFFSET] & SCREEN_FLAG_MODIFIERS != 0,
        show_batteries: bytes[FLAGS_OFFSET] & SCREEN_FLAG_BATTERIES != 0,
        output_visible: bytes[FLAGS_OFFSET] & SCREEN_FLAG_OUTPUT != 0,
        output_place: output_place.min(SCREEN_OUTPUT_CHIP),
        concept: concept.min(SCREEN_CONCEPT_MAX),
        brightness: bytes[BRIGHTNESS_OFFSET].clamp(SCREEN_BRIGHTNESS_MIN, SCREEN_BRIGHTNESS_MAX),
        colors,
        left_label: stored_battery_label(
            bytes[LEFT_LABEL_LEN_OFFSET],
            &bytes[LEFT_LABEL_OFFSET..LEFT_LABEL_OFFSET + BATTERY_LABEL_MAX],
            0,
        ),
        right_label: stored_battery_label(
            bytes[RIGHT_LABEL_LEN_OFFSET],
            &bytes[RIGHT_LABEL_OFFSET..RIGHT_LABEL_OFFSET + BATTERY_LABEL_MAX],
            1,
        ),
    }
}

/// Decodes a version 3/4/5 record: the same screen block, but every colour is
/// still three 8-bit components. They are converted to RGB565 with the same
/// rounding the panel uses, so a stored factory palette keeps looking identical.
fn stored_screen_settings_legacy(bytes: &[u8], output_place: u8, concept: u8) -> ScreenSettings {
    read_layer_names(bytes, LAYER_NAMES_OFFSET, LAYER_NAME_MAX, LAYER_NAME_COUNT);
    let mut colors = DEFAULT_COLORS;
    let legacy_colors: [[u8; 3]; 3] = [
        legacy_color(bytes, LEGACY_ACCENT_OFFSET),
        legacy_color(bytes, LEGACY_ACCENT_DIM_OFFSET),
        legacy_color(bytes, LEGACY_BACKGROUND_OFFSET),
    ];
    colors[0] = rgb8_to_565(legacy_colors[0]);
    colors[1] = rgb8_to_565(legacy_colors[1]);
    colors[2] = rgb8_to_565(legacy_colors[2]);
    ScreenSettings {
        wpm_visible: bytes[FLAGS_OFFSET] & SCREEN_FLAG_WPM != 0,
        header_mode: bytes[HEADER_OFFSET].min(SCREEN_HEADER_BOTH),
        timeout_s: bytes[TIMEOUT_OFFSET],
        show_modifiers: bytes[FLAGS_OFFSET] & SCREEN_FLAG_MODIFIERS != 0,
        show_batteries: bytes[FLAGS_OFFSET] & SCREEN_FLAG_BATTERIES != 0,
        output_visible: bytes[FLAGS_OFFSET] & SCREEN_FLAG_OUTPUT != 0,
        output_place: output_place.min(SCREEN_OUTPUT_CHIP),
        concept: concept.min(SCREEN_CONCEPT_MAX),
        brightness: bytes[BRIGHTNESS_OFFSET].clamp(SCREEN_BRIGHTNESS_MIN, SCREEN_BRIGHTNESS_MAX),
        colors,
        left_label: stored_battery_label(
            bytes[LEFT_LABEL_LEN_OFFSET],
            &bytes[LEFT_LABEL_OFFSET..LEFT_LABEL_OFFSET + BATTERY_LABEL_MAX],
            0,
        ),
        right_label: stored_battery_label(
            bytes[RIGHT_LABEL_LEN_OFFSET],
            &bytes[RIGHT_LABEL_OFFSET..RIGHT_LABEL_OFFSET + BATTERY_LABEL_MAX],
            1,
        ),
    }
}

/// Three 8-bit components of a legacy record.
fn legacy_color(bytes: &[u8], offset: usize) -> [u8; 3] {
    [bytes[offset], bytes[offset + 1], bytes[offset + 2]]
}

fn bump_versions() {
    LAYER_NAMES_VERSION.fetch_add(1, Ordering::Relaxed);
    SCREEN_SETTINGS_VERSION.fetch_add(1, Ordering::Relaxed);
}

/// Reads a name table written with `stride` bytes per entry (10 for version 3,
/// 12 for versions 1 and 2) and re-stores it in the current layout.
fn read_layer_names(bytes: &[u8], offset: usize, stride: usize, count: usize) {
    let mut entry = offset;
    for index in 0..count.min(LAYER_NAME_COUNT) {
        let len = usize::from(bytes[entry]).min(stride);
        let start = entry + 1;
        store_raw_layer_name(index, &bytes[start..start + len]);
        entry += 1 + stride;
    }
}

fn get_layer_name(index: usize, out: &mut [u8]) -> Option<usize> {
    let len = usize::from(LAYER_NAME_LEN[index].load(Ordering::Acquire)).min(LAYER_NAME_MAX);
    let copy_len = len.min(out.len().saturating_sub(1));
    let base = index * LAYER_NAME_MAX;
    for (offset, byte) in out.iter_mut().take(copy_len).enumerate() {
        *byte = LAYER_NAME_BYTES[base + offset].load(Ordering::Relaxed);
    }
    if out.len() > copy_len {
        out[copy_len] = 0;
        Some(copy_len + 1)
    } else {
        Some(copy_len)
    }
}

fn get_battery_label(slot: usize, out: &mut [u8]) -> Option<usize> {
    let label = read_battery_label(slot);
    let len = label.len as usize;
    let copy_len = len.min(out.len().saturating_sub(1));
    out[..copy_len].copy_from_slice(&label.bytes[..copy_len]);
    if out.len() > copy_len {
        out[copy_len] = 0;
        Some(copy_len + 1)
    } else {
        Some(copy_len)
    }
}

fn set_battery_label(slot: usize, value: &[u8]) -> bool {
    let end = value
        .iter()
        .position(|&byte| byte == 0 || byte == 0xFF)
        .unwrap_or(value.len());
    let Ok(text) = str::from_utf8(&value[..end]) else {
        return false;
    };
    ensure_screen_loaded();
    store_battery_label(slot, text.trim());
    SCREEN_SETTINGS_VERSION.fetch_add(1, Ordering::Relaxed);
    true
}

fn stored_battery_label(raw_len: u8, bytes: &[u8], slot: usize) -> BatteryLabel {
    let len = usize::from(raw_len).min(BATTERY_LABEL_MAX);
    let mut out = BatteryLabel::empty();
    let mut kept = 0usize;
    for byte in bytes.iter().take(len) {
        if kept >= BATTERY_LABEL_MAX {
            break;
        }
        out.bytes[kept] = *byte;
        kept += 1;
    }
    // Drop a broken tail so readers never see invalid UTF-8.
    while kept > 0 && str::from_utf8(&out.bytes[..kept]).is_err() {
        kept -= 1;
    }
    out.len = kept as u8;
    if kept == 0 {
        return default_label(slot);
    }
    out
}

// --- Layer name storage ----------------------------------------------------

fn store_layer_name(index: usize, text: &str) {
    let mut bytes = [0u8; LAYER_NAME_MAX];
    let mut len = 0usize;
    let mut chars = text.trim().chars().peekable();
    while let Some(ch) = chars.next() {
        if ch == '%' && chars.peek() == Some(&'%') {
            let _ = chars.next();
        }
        let mut encoded = [0u8; 4];
        let encoded = ch.encode_utf8(&mut encoded).as_bytes();
        if len + encoded.len() > LAYER_NAME_MAX {
            break;
        }
        bytes[len..len + encoded.len()].copy_from_slice(encoded);
        len += encoded.len();
    }
    store_raw_layer_name(index, &bytes[..len]);
    LAYER_NAMES_VERSION.fetch_add(1, Ordering::Relaxed);
}

fn store_raw_layer_name(index: usize, bytes: &[u8]) {
    if index >= LAYER_NAME_COUNT {
        return;
    }
    let mut len = bytes.len().min(LAYER_NAME_MAX);
    // Never leave a half-encoded character in the slot.
    while len > 0 && str::from_utf8(&bytes[..len]).is_err() {
        len -= 1;
    }
    LAYER_NAME_LEN[index].store(0, Ordering::Release);
    let base = index * LAYER_NAME_MAX;
    for offset in 0..LAYER_NAME_MAX {
        LAYER_NAME_BYTES[base + offset].store(bytes.get(offset).copied().unwrap_or(0), Ordering::Relaxed);
    }
    LAYER_NAME_LEN[index].store(len as u8, Ordering::Release);
}

fn clear_layer_names() {
    for index in 0..LAYER_NAME_COUNT {
        store_raw_layer_name(index, &[]);
    }
}

// --- Screen settings storage ----------------------------------------------

fn ensure_screen_loaded() {
    if SCREEN_READY.load(Ordering::Acquire) != 0 {
        return;
    }
    store_screen_settings(&ScreenSettings::default());
}

fn store_screen(offset: usize, value: u8) -> bool {
    ensure_screen_loaded();
    match offset {
        HEADER_OFFSET => SCREEN_HEADER.store(value, Ordering::Relaxed),
        TIMEOUT_OFFSET => SCREEN_TIMEOUT.store(value, Ordering::Relaxed),
        PLACE_OFFSET => SCREEN_PLACE.store(value.min(SCREEN_OUTPUT_CHIP), Ordering::Relaxed),
        CONCEPT_OFFSET => SCREEN_CONCEPT.store(value.min(SCREEN_CONCEPT_MAX), Ordering::Relaxed),
        _ => return false,
    }
    true
}

fn update_flags(mask: u8, enabled: bool) -> bool {
    ensure_screen_loaded();
    let mut flags = SCREEN_FLAGS.load(Ordering::Relaxed);
    if enabled {
        flags |= mask;
    } else {
        flags &= !mask;
    }
    SCREEN_FLAGS.store(flags, Ordering::Relaxed);
    SCREEN_SETTINGS_VERSION.fetch_add(1, Ordering::Relaxed);
    true
}

fn read_battery_label(slot: usize) -> BatteryLabel {
    if slot >= 2 {
        return BatteryLabel::empty();
    }
    let len = usize::from(BATTERY_LABEL_LEN[slot].load(Ordering::Acquire)).min(BATTERY_LABEL_MAX);
    let mut out = BatteryLabel::empty();
    let base = slot * BATTERY_LABEL_MAX;
    for offset in 0..len {
        out.bytes[offset] = BATTERY_LABEL_BYTES[base + offset].load(Ordering::Relaxed);
    }
    let mut kept = len;
    while kept > 0 && str::from_utf8(&out.bytes[..kept]).is_err() {
        kept -= 1;
    }
    out.len = kept as u8;
    if kept == 0 {
        // Nothing stored yet: show the factory label instead of an empty card.
        return default_label(slot);
    }
    out
}

fn store_battery_label(slot: usize, text: &str) {
    let mut bytes = [0u8; BATTERY_LABEL_MAX];
    let mut len = 0usize;
    for ch in text.chars() {
        let mut encoded = [0u8; 4];
        let encoded = ch.encode_utf8(&mut encoded).as_bytes();
        if len + encoded.len() > BATTERY_LABEL_MAX {
            break;
        }
        bytes[len..len + encoded.len()].copy_from_slice(encoded);
        len += encoded.len();
    }
    BATTERY_LABEL_LEN[slot].store(0, Ordering::Release);
    let base = slot * BATTERY_LABEL_MAX;
    for offset in 0..BATTERY_LABEL_MAX {
        BATTERY_LABEL_BYTES[base + offset].store(bytes[offset], Ordering::Relaxed);
    }
    BATTERY_LABEL_LEN[slot].store(len as u8, Ordering::Release);
}

fn store_screen_settings(settings: &ScreenSettings) {
    let mut flags = 0u8;
    flags |= if settings.wpm_visible { SCREEN_FLAG_WPM } else { 0 };
    flags |= if settings.show_modifiers {
        SCREEN_FLAG_MODIFIERS
    } else {
        0
    };
    flags |= if settings.show_batteries {
        SCREEN_FLAG_BATTERIES
    } else {
        0
    };
    flags |= if settings.output_visible {
        SCREEN_FLAG_OUTPUT
    } else {
        0
    };
    SCREEN_FLAGS.store(flags, Ordering::Relaxed);
    SCREEN_HEADER.store(settings.header_mode.min(SCREEN_HEADER_BOTH), Ordering::Relaxed);
    SCREEN_TIMEOUT.store(settings.timeout_s, Ordering::Relaxed);
    SCREEN_PLACE.store(settings.output_place.min(SCREEN_OUTPUT_CHIP), Ordering::Relaxed);
    SCREEN_CONCEPT.store(settings.concept.min(SCREEN_CONCEPT_MAX), Ordering::Relaxed);
    SCREEN_BRIGHTNESS.store(
        settings
            .brightness
            .clamp(SCREEN_BRIGHTNESS_MIN, SCREEN_BRIGHTNESS_MAX),
        Ordering::Relaxed,
    );
    for (index, value) in settings.colors.iter().enumerate() {
        SCREEN_COLORS[index].store(*value, Ordering::Relaxed);
    }
    store_battery_label(0, settings.left_label.as_str());
    store_battery_label(1, settings.right_label.as_str());
    SCREEN_READY.store(1, Ordering::Release);
}

fn load_default_layer_names() {
    for (index, name) in crate::DEFAULT_LAYER_NAMES.iter().enumerate() {
        store_raw_layer_name(index, name.as_bytes());
    }
}

fn load_defaults() {
    load_default_layer_names();
    store_screen_settings(&ScreenSettings::default());
}

fn migrate_legacy_placeholders() {
    let mut bytes = [0u8; LAYER_NAME_MAX];
    for (index, default_name) in crate::DEFAULT_LAYER_NAMES.iter().enumerate() {
        let len = usize::from(LAYER_NAME_LEN[index].load(Ordering::Acquire)).min(LAYER_NAME_MAX);
        let base = index * LAYER_NAME_MAX;
        for (offset, byte) in bytes.iter_mut().enumerate() {
            *byte = LAYER_NAME_BYTES[base + offset].load(Ordering::Relaxed);
        }
        if crate::default_layer_names::is_legacy_placeholder(index, &bytes[..len]) {
            store_raw_layer_name(index, default_name.as_bytes());
        }
    }
}

fn layer_index(qsid: u16) -> Option<usize> {
    let offset = qsid.checked_sub(LAYER_NAME_QSID_BASE)?;
    (offset < LAYER_NAME_COUNT as u16).then_some(usize::from(offset))
}

/// `QSID → (colour index, component)`, component `0 = R, 1 = G, 2 = B`.
fn color_lookup(qsid: u16) -> Option<(usize, usize)> {
    for (index, first) in COLOR_FIRST_QSID.iter().enumerate() {
        if qsid >= *first && qsid < *first + 3 {
            return Some((index, usize::from(qsid - *first)));
        }
    }
    None
}
