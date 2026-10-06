//! Ergohaven Qube dongle display (ST7789V over SPI).
//!
//! Full landscape **280×240** UI without a full-frame RGB565 buffer
//! (~134 KiB would OOM / kill HID on nRF52840).
//!
//! Strategy: **stripe multipass**
//! - Logical size: 280×240 (full panel after Deg90)
//! - Physical FB: 280×48×2 ≈ 27 KiB (< EasyDMA MAXCNT 65535, RAM-safe)
//! - Each redraw: for each stripe → clip-draw full UI → SPI that stripe
//!
//! Every zone, colour and the connection indicator are driven by the runtime
//! settings stored in `crate::layer_names` (Vial device settings, QSID
//! 216..=226 / 318 / 320..=322 / 330..=332). The palette, the software
//! brightness and the idle blanking are applied to the finished stripe, so the
//! UI code always draws with the original constants and a device that never
//! stored settings renders exactly the pre-settings frame.
//!
//! Pinout (`qube.overlay`):
//! SPI3 SCK=P1.11 MOSI=P1.10 · CS=P1.13 · DC=P0.28 · RST=P0.03 · BL=P0.02

use core::fmt::Write as _;

use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::{P0_02, P0_03, P0_28, P1_10, P1_11, P1_13, SPI3};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::{interrupt, Peri};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10, FONT_8X13};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle,
};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use lcd_async::interface::SpiInterface;
use lcd_async::models::ST7789;
use lcd_async::options::{ColorInversion, Orientation, Rotation};
use lcd_async::{Builder, Display as LcdDisplay};
use rmk::core_traits::Runnable;
use rmk::display::{DisplayRenderer, RenderContext};
use rmk::event::{
    BatteryStatusEvent, BleAdvertisingMode, BleAdvertisingModeEvent, CentralConnectedEvent,
    ConnectionStatusChangeEvent, EventSubscriber, KeyboardEvent, LayerChangeEvent,
    LedIndicatorEvent, ModifierEvent, PeripheralBatteryEvent, PeripheralConnectedEvent,
    SleepStateEvent, SubscribableEvent, WpmUpdateEvent,
};
use rmk::processor::Processor;
use rmk_types::battery::BatteryStatus;
use rmk_types::ble::BleState;
use rmk_types::connection::{ConnectionStatus, ConnectionType};
use static_cell::StaticCell;

use crate::layer_names::{
    self, ScreenSettings, BATTERY_LABEL_MAX, DEFAULT_ACCENT, DEFAULT_ACCENT_DIM,
    DEFAULT_BACKGROUND, SCREEN_BRIGHTNESS_MAX, SCREEN_HEADER_CLOCK, SCREEN_HEADER_MEDIA,
    SCREEN_OUTPUT_CHIP, SCREEN_OUTPUT_HEADER,
};

// --- Panel geometry ---------------------------------------------------------

pub const PANEL_NATIVE_W: usize = 240;
pub const PANEL_NATIVE_H: usize = 280;
const PANEL_ROTATION: Rotation = Rotation::Deg90;

/// Full landscape frame (after Deg90).
pub const SCREEN_W: usize = 280;
pub const SCREEN_H: usize = 240;

/// Stripe height: 280×48×2 = 26_880 B < 64 KiB EasyDMA, comfortable RAM.
const STRIPE_H: usize = 48;
const STRIPE_BYTES: usize = SCREEN_W * STRIPE_H * 2;

const BACKLIGHT_ACTIVE_HIGH: bool = true;
const SAFE_X: i32 = 18;
const SAFE_W: u32 = SCREEN_W as u32 - (SAFE_X as u32 * 2);
const MEDIA_GAP_CHARS: usize = 3;
/// Badge text buffer: `USB` / `BTn` / `BTn*` / `BTn~` / `---`.
const OUTPUT_BADGE_CHARS: usize = 4;
/// Sixth chip slot of the modifier row (connection badge when placed here).
const BADGE_CHIP_X: i32 = 216;
const BADGE_CHIP_W: u32 = 38;
const PANEL_RADIUS: u32 = 14;
const CHIP_RADIUS: u32 = 7;
const BAR_RADIUS: u32 = 5;

// Layout v2: the layer name and the WPM block share one row, the connection
// badge sits either in the header or in a sixth chip slot, and every band —
// including the dirty regions — comes from `compute_layout`/`geometry`, so a
// hidden zone hands its height to the row.

/// Geometry of one vertical zone: top edge plus height.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Band {
    y: i32,
    h: u32,
}

impl Band {
    const fn bottom(&self) -> i32 {
        self.y + self.h as i32
    }
}

/// Zone stack of the dashboard. Identical to the host stand's
/// `compute_layout_v2`, which is what makes the PNG previews comparable.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Layout {
    header: Band,
    /// One row: layer name on the left, compact WPM block on the right.
    row: Band,
    modifiers: Option<Band>,
    batteries: Option<Band>,
}

const LAYOUT_TOP: i32 = 14;
const LAYOUT_BOTTOM_MARGIN: i32 = 12;
const LAYOUT_HEADER_H: u32 = 28;
const LAYOUT_ROW_MIN_H: u32 = 54;
const LAYOUT_MODS_H: u32 = 16;
const LAYOUT_BAT_H: u32 = 40;
const LAYOUT_GAP_HEADER: i32 = 4;
const LAYOUT_GAP: i32 = 6;

fn compute_layout(settings: &ScreenSettings) -> Layout {
    let header = Band {
        y: LAYOUT_TOP,
        h: LAYOUT_HEADER_H,
    };
    let mut reserve = 0i32;
    if settings.show_modifiers {
        reserve += LAYOUT_GAP + LAYOUT_MODS_H as i32;
    }
    if settings.show_batteries {
        reserve += LAYOUT_GAP + LAYOUT_BAT_H as i32;
    }

    let row_top = header.bottom() + LAYOUT_GAP_HEADER;
    let row_h = ((SCREEN_H as i32 - LAYOUT_BOTTOM_MARGIN) - reserve - row_top)
        .max(LAYOUT_ROW_MIN_H as i32);
    let row = Band {
        y: row_top,
        h: row_h as u32,
    };

    let mut y = row.bottom();
    let modifiers = if settings.show_modifiers {
        y += LAYOUT_GAP;
        let band = Band { y, h: LAYOUT_MODS_H };
        y = band.bottom();
        Some(band)
    } else {
        None
    };
    let batteries = if settings.show_batteries {
        y += LAYOUT_GAP;
        Some(Band {
            y,
            h: LAYOUT_BAT_H,
        })
    } else {
        None
    };

    Layout {
        header,
        row,
        modifiers,
        batteries,
    }
}

// Dirty regions, all cut from the same layout: fixed header band, the row, and
// the two optional bands padded like the stand's report (4 px above, 2 below).
const HEADER_DIRTY: DirtyRegion = DirtyRegion::range(12, 44);

fn row_dirty(row: Band) -> DirtyRegion {
    DirtyRegion::range(44, clamp_u16(row.bottom() + 2))
}

fn band_dirty(band: Band) -> DirtyRegion {
    DirtyRegion::range(clamp_u16(band.y - 4), clamp_u16(band.bottom() + 2))
}

fn clamp_u16(value: i32) -> u16 {
    if value <= 0 {
        0
    } else {
        value.min(SCREEN_H as i32) as u16
    }
}

/// Horizontal geometry of layout v2, derived from the font metrics exactly
/// like the stand's `v2_geometry` (name starts at `SAFE_X+6`, 6 px padding on
/// the right, badge slot `3 + 4 + 4` glyphs, clock 5 glyphs of `FONT_8X13`).
#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Geometry {
    f6: i32,
    f10: i32,
    name_x: i32,
    row_right: i32,
    header_right: i32,
    badge_w: i32,
    badge_in_header: bool,
    clock_right: i32,
    media_x: i32,
    media_limit: usize,
    wpm_right: i32,
    name_limit: i32,
}

fn font_advance(font: &embedded_graphics::mono_font::MonoFont<'_>) -> i32 {
    (font.character_size.width + font.character_spacing) as i32
}

fn geometry(settings: &ScreenSettings) -> Geometry {
    let f6 = font_advance(&FONT_6X10);
    let f8 = font_advance(&FONT_8X13);
    let f10 = font_advance(&FONT_10X20);

    let name_x = SAFE_X + 6;
    let row_right = SAFE_X + SAFE_W as i32 - 6;
    let header_right = SAFE_X + SAFE_W as i32 - 12;
    let badge_w = 3 + 4 + 4 * f6;
    let badge_in_header =
        settings.output_visible && settings.output_place == SCREEN_OUTPUT_HEADER;
    let clock_right = if badge_in_header {
        header_right - badge_w - 8
    } else {
        SAFE_X + SAFE_W as i32 - 14
    };
    let clock_w = 5 * f8;

    let media_x = SAFE_X + 22;
    let media_limit = {
        let right = match settings.header_mode {
            SCREEN_HEADER_CLOCK => media_x,
            SCREEN_HEADER_MEDIA => {
                if badge_in_header {
                    header_right - badge_w - 6
                } else {
                    header_right
                }
            }
            _ => clock_right - clock_w - 6,
        };
        (((right - media_x) / f6).max(0) as usize).min(layer_names::MEDIA_VISIBLE_CHARS)
    };

    let wpm_right = if settings.wpm_visible { row_right } else { 0 };
    let name_limit = if settings.wpm_visible {
        wpm_right - 3 * f6 - 8
    } else {
        row_right
    };

    Geometry {
        f6,
        f10,
        name_x,
        row_right,
        header_right,
        badge_w,
        badge_in_header,
        clock_right,
        media_x,
        media_limit,
        wpm_right,
        name_limit,
    }
}
// Display state may be a few frames late, but cursor motion must never wait
// behind framebuffer rendering or SPI. Apply pending UI changes once the
// pointing stream has been quiet for this window.
const POINTING_REDRAW_QUIET_PERIOD: Duration = Duration::from_millis(100);
// Modifier chips are interactive feedback. Render the complete band once,
// then commit it in one asynchronous EasyDMA transfer so the screen never
// exposes intermediate stripe renders and the executor remains available.
const MODIFIER_REDRAW_MIN_INTERVAL: Duration = Duration::from_millis(16);

const COL_BG: Rgb565 = Rgb565::new(0, 2, 4);
const COL_FG: Rgb565 = Rgb565::new(29, 61, 30);
const COL_MUTED: Rgb565 = Rgb565::new(11, 24, 20);
const COL_LABEL: Rgb565 = Rgb565::new(16, 36, 28);
const COL_DIM: Rgb565 = Rgb565::new(5, 12, 14);
const COL_ACCENT: Rgb565 = Rgb565::new(3, 38, 31);
const COL_ACCENT_DIM: Rgb565 = Rgb565::new(1, 16, 18);
const COL_YELLOW: Rgb565 = Rgb565::new(31, 50, 0);
const COL_RED: Rgb565 = Rgb565::new(31, 5, 5);
const COL_BAR_BG: Rgb565 = Rgb565::new(2, 7, 9);
const COL_BAR_FG: Rgb565 = Rgb565::new(3, 42, 30);
const COL_PANEL: Rgb565 = Rgb565::new(2, 6, 9);
const COL_PANEL_HI: Rgb565 = Rgb565::new(3, 9, 13);
const COL_BORDER: Rgb565 = Rgb565::new(5, 13, 16);
const COL_BORDER_DIM: Rgb565 = Rgb565::new(3, 8, 11);

// --- Settings-driven look ---------------------------------------------------

/// Raw RGB565 of the pre-settings palette entries. `DEFAULT_*` from
/// `layer_names` are the 8-bit expansions of exactly these values, so the
/// factory settings rebuild the original colours (checked at compile time).
const COL_ACCENT_RAW: u16 = (3 << 11) | (38 << 5) | 31;
const COL_ACCENT_DIM_RAW: u16 = (1 << 11) | (16 << 5) | 18;
const COL_BG_RAW: u16 = (2 << 5) | 4;

const fn rgb8_to_raw565(color: [u8; 3]) -> u16 {
    (((color[0] >> 3) as u16) << 11)
        | (((color[1] >> 2) as u16) << 5)
        | ((color[2] >> 3) as u16)
}

const _: () = assert!(rgb8_to_raw565(DEFAULT_ACCENT) == COL_ACCENT_RAW);
const _: () = assert!(rgb8_to_raw565(DEFAULT_ACCENT_DIM) == COL_ACCENT_DIM_RAW);
const _: () = assert!(rgb8_to_raw565(DEFAULT_BACKGROUND) == COL_BG_RAW);

fn rgb565_from_rgb8(color: [u8; 3]) -> Rgb565 {
    Rgb565::new(color[0] >> 3, color[1] >> 2, color[2] >> 3)
}

/// Connection indicator colours: green = active transport, blue = searching or
/// reconnecting, grey = the dongle is not talking to anything.
const COL_OUTPUT_OK: Rgb565 = Rgb565::new(4, 46, 12);
const COL_OUTPUT_SEARCH: Rgb565 = Rgb565::new(4, 24, 31);
const COL_OUTPUT_IDLE: Rgb565 = COL_DIM;

/// What the header indicator shows right now.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum OutputState {
    /// USB carries the active link (configured or suspended for remote wakeup).
    Usb,
    /// Bluetooth profile `n` is connected.
    Ble { profile: u8 },
    /// Advertising for a new host (no bond for the active profile).
    Pairing { profile: u8 },
    /// Advertising to reconnect a bonded host (also used before the first
    /// [`BleAdvertisingModeEvent`] arrives).
    Reconnecting { profile: u8 },
    /// Nothing is connected and nothing is being searched for.
    Idle,
}

impl OutputState {
    fn from_connection(
        connection: &ConnectionStatus,
        advertising: Option<BleAdvertisingMode>,
    ) -> Self {
        match connection.decide_active() {
            Some(ConnectionType::Usb) => Self::Usb,
            Some(ConnectionType::Ble) => Self::Ble {
                profile: connection.ble.profile,
            },
            None => {
                if matches!(connection.ble.state, BleState::Advertising) {
                    let profile = connection.ble.profile;
                    match advertising {
                        Some(BleAdvertisingMode::Pairing) => Self::Pairing { profile },
                        Some(BleAdvertisingMode::Reconnecting) => Self::Reconnecting { profile },
                        None => Self::Reconnecting { profile },
                    }
                } else {
                    Self::Idle
                }
            }
        }
    }

    /// Badge text and colour: `USB` / `BT{n}` / `BT{n}*` (pairing) /
    /// `BT{n}~` (reconnecting) / `---`, green for an active transport, blue
    /// while searching, grey when nothing is in use.
    fn badge(self) -> (heapless::String<OUTPUT_BADGE_CHARS>, Rgb565) {
        let mut text: heapless::String<OUTPUT_BADGE_CHARS> = heapless::String::new();
        let digit = |profile: u8| (b'0' + profile.min(9)) as char;
        let color = match self {
            Self::Usb => {
                let _ = text.push_str("USB");
                COL_OUTPUT_OK
            }
            Self::Ble { profile } => {
                let _ = write!(&mut text, "BT{}", digit(profile));
                COL_OUTPUT_OK
            }
            Self::Pairing { profile } => {
                let _ = write!(&mut text, "BT{}*", digit(profile));
                COL_OUTPUT_SEARCH
            }
            Self::Reconnecting { profile } => {
                let _ = write!(&mut text, "BT{}~", digit(profile));
                COL_OUTPUT_SEARCH
            }
            Self::Idle => {
                let _ = text.push_str("---");
                COL_OUTPUT_IDLE
            }
        };
        (text, color)
    }
}

/// Post-pass over a finished stripe: palette substitution, software brightness
/// and the idle blank. Same maths the host stand applies to its PNGs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Look {
    accent: Rgb565,
    accent_dim: Rgb565,
    background: Rgb565,
    brightness: u8,
    blank: bool,
}

impl Look {
    fn from_settings(settings: &ScreenSettings, blank: bool) -> Self {
        Self {
            accent: rgb565_from_rgb8(settings.accent),
            accent_dim: rgb565_from_rgb8(settings.accent_dim),
            background: rgb565_from_rgb8(settings.background),
            brightness: settings.brightness.min(SCREEN_BRIGHTNESS_MAX),
            blank,
        }
    }
}

/// Rewrites one RGB565 pixel: the three palette entries the UI draws with are
/// replaced by the configured colours, every component is scaled by the
/// brightness setting, and a blanked screen becomes black. The panel driver
/// applies this to each finished stripe.
fn look_pixel(raw: u16, look: &Look) -> u16 {
    if look.blank {
        return 0;
    }
    let mut raw = if raw == COL_ACCENT_RAW {
        look.accent.into_storage()
    } else if raw == COL_ACCENT_DIM_RAW {
        look.accent_dim.into_storage()
    } else if raw == COL_BG_RAW {
        look.background.into_storage()
    } else {
        raw
    };
    let scale = look.brightness.min(SCREEN_BRIGHTNESS_MAX) as u32;
    if scale < SCREEN_BRIGHTNESS_MAX as u32 {
        let r5 = ((raw >> 11) & 0x1F) as u32 * scale / 100;
        let g6 = ((raw >> 5) & 0x3F) as u32 * scale / 100;
        let b5 = (raw & 0x1F) as u32 * scale / 100;
        raw = ((r5 as u16) << 11) | ((g6 as u16) << 5) | b5 as u16;
    }
    raw
}

// --- Boot splash ------------------------------------------------------------


/// How long the wordmark stays on screen after power-up.
const SPLASH_DURATION: Duration = Duration::from_millis(2000);


/// Firmware version injected by `build.rs` (`RMK_FIRMWARE_VERSION`).
const FIRMWARE_VERSION: &str = match option_env!("RMK_FIRMWARE_VERSION") {
    Some(version) => version,
    None => "dev",
};


/// Wordmark anchor in *screen* coordinates: top edge, horizontally centred.
const SPLASH_WORD_TOP: Point = Point::new(SCREEN_W as i32 / 2, 84);
/// WPM value anchor: right edge (the band height comes from the layout).
/// Battery card geometry (two cards plus the gap fill `SAFE_W` exactly).
const BATTERY_CARD_W: i32 = 116;
const BATTERY_CARD_GAP: i32 = 12;


// --- x2 text scaling --------------------------------------------------------


/// Scratch target that rasterises text at 1x into a tiny fixed buffer.
///
/// The buffer is a plain `DrawTarget` of its own: text rendering therefore
/// never re-enters the panel target with synthesised shapes, and the later
/// blit writes enlarged pixels through `draw_iter` on the panel target, in
/// *screen* coordinates — exactly the path the stock renderer already uses.
const GLYPH_BUF_W: u32 = 12;
const GLYPH_BUF_H: u32 = 22;
const GLYPH_BUF_LEN: usize = (GLYPH_BUF_W * GLYPH_BUF_H) as usize;

struct GlyphBuf {
    pixels: [bool; GLYPH_BUF_LEN],
}

impl GlyphBuf {
    fn new() -> Self {
        Self {
            pixels: [false; GLYPH_BUF_LEN],
        }
    }

    fn clear(&mut self) {
        self.pixels = [false; GLYPH_BUF_LEN];
    }

    fn is_set(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 {
            return false;
        }
        let (x, y) = (x as u32, y as u32);
        if x >= GLYPH_BUF_W || y >= GLYPH_BUF_H {
            return false;
        }
        let index = (y * GLYPH_BUF_W + x) as usize;
        index < GLYPH_BUF_LEN && self.pixels[index]
    }
}

impl OriginDimensions for GlyphBuf {
    fn size(&self) -> Size {
        Size::new(GLYPH_BUF_W, GLYPH_BUF_H)
    }
}

impl DrawTarget for GlyphBuf {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(point, _) in pixels {
            if point.x < 0 || point.y < 0 {
                continue;
            }
            let (x, y) = (point.x as u32, point.y as u32);
            if x >= GLYPH_BUF_W || y >= GLYPH_BUF_H {
                continue;
            }
            let index = (y * GLYPH_BUF_W + x) as usize;
            if index < GLYPH_BUF_LEN {
                self.pixels[index] = true;
            }
        }
        Ok(())
    }
}


/// Draws `text` at double size: each glyph is rasterised at 1x into `GlyphBuf`
/// and then blitted as 2x2 blocks of `color`.
///
/// `anchor` is in screen coordinates: `y` is the top edge of the text box and
/// `x` is the left edge, centre or right edge depending on `align`.
fn draw_text_x2<D>(
    display: &mut D,
    text: &str,
    anchor: Point,
    style: MonoTextStyle<'_, Rgb565>,
    align: Alignment,
) where
    D: DrawTarget<Color = Rgb565>,
{
    let top_left = TextStyleBuilder::new().baseline(Baseline::Top).build();
    let advance = (style.font.character_size.width + style.font.character_spacing) as i32;
    let spacing = style.font.character_spacing as i32;
    let glyphs = text.chars().count() as i32;
    let width = (glyphs * advance - spacing).max(0) * 2;

    let mut pen_x = match align {
        Alignment::Left => anchor.x,
        Alignment::Center => anchor.x - width / 2,
        Alignment::Right => anchor.x - width,
    };

    let mut buffer = GlyphBuf::new();
    let mut single: heapless::String<8> = heapless::String::new();
    for ch in text.chars() {
        buffer.clear();
        single.clear();
        let _ = single.push(ch);
        let _ = Text::with_text_style(&single, Point::zero(), style, top_left).draw(&mut buffer);

        for y in 0..GLYPH_BUF_H as i32 {
            for x in 0..GLYPH_BUF_W as i32 {
                if !buffer.is_set(x, y) {
                    continue;
                }
                let sx = pen_x + x * 2;
                let sy = anchor.y + y * 2;
                if let Some(color) = style.text_color {
                    let _ = display.draw_iter([
                        Pixel(Point::new(sx, sy), color),
                        Pixel(Point::new(sx + 1, sy), color),
                        Pixel(Point::new(sx, sy + 1), color),
                        Pixel(Point::new(sx + 1, sy + 1), color),
                    ]);
                }
            }
        }
        pen_x += advance * 2;
    }
}

type SpiDev = ExclusiveDevice<Spim<'static>, Output<'static>, NoDelay>;
type Di = SpiInterface<SpiDev, Output<'static>>;
type Panel = LcdDisplay<Di, ST7789, Output<'static>>;

#[derive(Clone, Copy)]
enum DirtyRegion {
    Full,
    Range { y0: u16, y1: u16 },
}

impl DirtyRegion {
    const fn range(y0: u16, y1: u16) -> Self {
        Self::Range { y0, y1 }
    }

    fn union(self, other: Self) -> Self {
        match (self, other) {
            (Self::Full, _) | (_, Self::Full) => Self::Full,
            (Self::Range { y0: a0, y1: a1 }, Self::Range { y0: b0, y1: b1 }) => Self::Range {
                y0: a0.min(b0),
                y1: a1.max(b1),
            },
        }
    }
}

// --- Stripe framebuffer (clip window into full screen) ----------------------

struct StripeLcd {
    display: Panel,
    buffer: &'static mut [u8; STRIPE_BYTES],
    /// Top of the active stripe in full-screen coordinates.
    band_y: u16,
    /// Height of the active stripe (≤ STRIPE_H), last stripe may be shorter.
    band_h: u16,
}

impl StripeLcd {
    fn set_band(&mut self, y: u16, h: u16) {
        self.band_y = y;
        self.band_h = h.min(STRIPE_H as u16);
    }

    fn clear_stripe(&mut self, color: Rgb565) {
        let c = color.into_storage().to_be_bytes();
        let bytes = SCREEN_W * self.band_h as usize * 2;
        for pix in self.buffer[..bytes].chunks_exact_mut(2) {
            pix[0] = c[0];
            pix[1] = c[1];
        }
    }

    /// Rewrites the finished stripe in place: the three palette entries the UI
    /// draws with are replaced by the configured colours, every component is
    /// scaled by the brightness setting, and a blanked screen becomes black.
    ///
    /// Doing it here keeps every drawing helper on the original constants and
    /// costs one pass over ≤ 26 880 bytes per stripe.
    fn apply_look(&mut self, look: &Look) {
        let bytes = SCREEN_W * self.band_h as usize * 2;
        for pix in self.buffer[..bytes].chunks_exact_mut(2) {
            let raw = ((pix[0] as u16) << 8) | pix[1] as u16;
            let out = look_pixel(raw, look).to_be_bytes();
            pix[0] = out[0];
            pix[1] = out[1];
        }
    }

    fn put_pixel(&mut self, x: i32, y: i32, color: Rgb565) {
        if x < 0 || y < 0 {
            return;
        }
        let x = x as u32;
        let y = y as u32;
        if x >= SCREEN_W as u32 {
            return;
        }
        let by = self.band_y as u32;
        let bh = self.band_h as u32;
        if y < by || y >= by + bh {
            return;
        }
        let ly = (y - by) as usize;
        let lx = x as usize;
        let off = (ly * SCREEN_W + lx) * 2;
        if off + 1 >= self.buffer.len() {
            return;
        }
        let c = color.into_storage().to_be_bytes();
        self.buffer[off] = c[0];
        self.buffer[off + 1] = c[1];
    }

    async fn flush_band(&mut self, look: &Look) {
        self.apply_look(look);
        let w = SCREEN_W as u16;
        let h = self.band_h;
        let y = self.band_y;
        // Only send used rows (last stripe may be shorter).
        let bytes = (SCREEN_W * h as usize) * 2;
        let slice = &self.buffer[..bytes];
        let _ = self.display.show_raw_data(0, y, w, h, slice).await;
    }
}

impl OriginDimensions for StripeLcd {
    fn size(&self) -> Size {
        Size::new(SCREEN_W as u32, SCREEN_H as u32)
    }
}

impl DrawTarget for StripeLcd {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Rgb565>>,
    {
        for Pixel(p, col) in pixels {
            self.put_pixel(p.x, p.y, col);
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Rgb565) -> Result<(), Self::Error> {
        let by = self.band_y as i32;
        let bh = self.band_h as i32;
        let band = Rectangle::new(Point::new(0, by), Size::new(SCREEN_W as u32, bh as u32));
        let isect = area.intersection(&band);
        if isect.is_zero_sized() {
            return Ok(());
        }
        // Fast path: full-width clear of the stripe
        if isect.top_left.x == 0
            && isect.size.width == SCREEN_W as u32
            && isect.top_left.y == by
            && isect.size.height == bh as u32
        {
            self.clear_stripe(color);
            return Ok(());
        }
        let x0 = isect.top_left.x;
        let y0 = isect.top_left.y;
        let x1 = x0 + isect.size.width as i32;
        let y1 = y0 + isect.size.height as i32;
        for y in y0..y1 {
            for x in x0..x1 {
                self.put_pixel(x, y, color);
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Rgb565) -> Result<(), Self::Error> {
        // Only clear the active stripe (multipass re-renders full UI per band).
        self.clear_stripe(color);
        Ok(())
    }
}

// --- Lazy init --------------------------------------------------------------

struct PendingPins {
    spi: Peri<'static, SPI3>,
    sck: Peri<'static, P1_11>,
    mosi: Peri<'static, P1_10>,
    cs: Peri<'static, P1_13>,
    dc: Peri<'static, P0_28>,
    rst: Peri<'static, P0_03>,
}

enum LcdState {
    Pending(PendingPins),
    Active(StripeLcd),
    Failed,
}

pub struct LazyQubeLcd<I> {
    state: LcdState,
    irq: I,
}

impl<I> LazyQubeLcd<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    async fn ensure_init(&mut self) {
        let pins = match core::mem::replace(&mut self.state, LcdState::Failed) {
            LcdState::Pending(p) => p,
            LcdState::Active(d) => {
                self.state = LcdState::Active(d);
                return;
            }
            LcdState::Failed => return,
        };
        match try_init_lcd(pins, self.irq).await {
            Some(lcd) => {
                info!("ST7789 ready (full-screen stripe mode)");
                self.state = LcdState::Active(lcd);
            }
            None => {
                warn!("ST7789 init failed — HID keeps running");
                self.state = LcdState::Failed;
            }
        }
    }

    /// Redraw the requested vertical region via stripe multipass.
    async fn present_dirty(
        &mut self,
        renderer: &mut QubeStatusRenderer,
        ctx: &RenderContext,
        dirty: DirtyRegion,
        look: &Look,
    ) {
        self.ensure_init().await;
        let LcdState::Active(lcd) = &mut self.state else {
            return;
        };

        let (y0, y1) = match dirty {
            DirtyRegion::Full => (0, SCREEN_H as u16),
            DirtyRegion::Range { y0, y1 } => (y0.min(SCREEN_H as u16), y1.min(SCREEN_H as u16)),
        };

        let mut y = y0;
        while y < y1 {
            let remaining = y1.saturating_sub(y);
            let h = remaining.min(STRIPE_H as u16);
            lcd.set_band(y, h);
            lcd.clear_stripe(COL_BG);
            // Re-run full UI; DrawTarget keeps only this stripe's pixels.
            renderer.render(ctx, lcd);
            lcd.flush_band(look).await;
            y = y.saturating_add(h);
        }
    }

    /// Render interactive modifier feedback once before touching the panel.
    async fn present_modifiers(
        &mut self,
        renderer: &QubeStatusRenderer,
        ctx: &RenderContext,
        band: Band,
        look: &Look,
    ) {
        self.ensure_init().await;
        let LcdState::Active(lcd) = &mut self.state else {
            return;
        };

        lcd.set_band(if band.y < 0 { 0 } else { band.y as u16 }, band.h as u16);
        lcd.clear_stripe(COL_BG);
        renderer.render_modifiers(ctx, lcd, band);
        lcd.flush_band(look).await;
    }
}

impl<I> OriginDimensions for LazyQubeLcd<I> {
    fn size(&self) -> Size {
        Size::new(SCREEN_W as u32, SCREEN_H as u32)
    }
}

// DrawTarget on LazyQubeLcd only needed if something draws before present;
// multipass uses StripeLcd directly via present().

async fn try_init_lcd<I>(pins: PendingPins, irq: I) -> Option<StripeLcd>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    let mut spi_cfg = spim::Config::default();
    spi_cfg.frequency = spim::Frequency::M8;
    let spim = Spim::new_txonly(pins.spi, irq, pins.sck, pins.mosi, spi_cfg);

    let cs = Output::new(pins.cs, Level::High, OutputDrive::Standard);
    let dc = Output::new(pins.dc, Level::Low, OutputDrive::Standard);
    let rst = Output::new(pins.rst, Level::High, OutputDrive::Standard);

    let spi_dev = ExclusiveDevice::new_no_delay(spim, cs).ok()?;
    let di = SpiInterface::new(spi_dev, dc);

    let mut delay = Delay;
    let display = Builder::new(ST7789, di)
        .reset_pin(rst)
        .display_size(PANEL_NATIVE_W as u16, PANEL_NATIVE_H as u16)
        .display_offset(0, 20)
        .invert_colors(ColorInversion::Inverted)
        .orientation(Orientation::new().rotate(PANEL_ROTATION))
        .init(&mut delay)
        .await
        .ok()?;

    static FB: StaticCell<[u8; STRIPE_BYTES]> = StaticCell::new();
    let buffer = FB.init([0; STRIPE_BYTES]);
    Some(StripeLcd {
        display,
        buffer,
        band_y: 0,
        band_h: STRIPE_H as u16,
    })
}

// --- Dongle screen processor (own event loop + multipass present) -----------

pub struct DongleScreen<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    lcd: LazyQubeLcd<I>,
    renderer: QubeStatusRenderer,
    ctx: RenderContext,
    /// Panel backlight (P0_02, plain GPIO): switched off while the screen is
    /// blanked by the idle timeout.
    backlight: &'static mut Output<'static>,
    /// Uptime when this processor was created — drives the boot splash.
    boot_at: Instant,
    last_host_data: rmk::host_data::HostData,
    last_layer_names_version: u8,
    /// Settings snapshot used by the layout and by the look post-pass.
    settings: ScreenSettings,
    last_screen_settings_version: u8,
    /// Idle blanking state and the last user activity that resets it.
    blanked: bool,
    blank_frame_due: bool,
    last_activity: Instant,
    last_render: Instant,
    last_modifier_render: Instant,
    pending: bool,
    modifier_pending: bool,
    dirty: DirtyRegion,
    min_interval: Duration,
}

pub fn create_processor<I>(
    spi: Peri<'static, SPI3>,
    sck: Peri<'static, P1_11>,
    mosi: Peri<'static, P1_10>,
    cs: Peri<'static, P1_13>,
    dc: Peri<'static, P0_28>,
    rst: Peri<'static, P0_03>,
    bl: Peri<'static, P0_02>,
    irq: I,
) -> DongleScreen<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    let level = backlight_level(true);
    static BL: StaticCell<Output<'static>> = StaticCell::new();
    let backlight = BL.init(Output::new(bl, level, OutputDrive::Standard));
    let host_data = rmk::host_data::snapshot();
    let settings = layer_names::screen_settings();

    DongleScreen {
        lcd: LazyQubeLcd {
            state: LcdState::Pending(PendingPins {
                spi,
                sck,
                mosi,
                cs,
                dc,
                rst,
            }),
            irq,
        },
        renderer: QubeStatusRenderer::new(host_data.clone(), true),
        ctx: RenderContext::default(),
        backlight,
        boot_at: Instant::now(),
        last_host_data: host_data,
        last_layer_names_version: layer_names::version(),
        settings,
        last_screen_settings_version: layer_names::screen_settings_version(),
        blanked: false,
        blank_frame_due: false,
        last_activity: Instant::now(),
        last_render: Instant::from_ticks(0),
        last_modifier_render: Instant::from_ticks(0),
        pending: true,
        modifier_pending: false,
        dirty: DirtyRegion::Full,
        min_interval: Duration::from_millis(80),
    }
}

fn backlight_level(on: bool) -> Level {
    if on == BACKLIGHT_ACTIVE_HIGH {
        Level::High
    } else {
        Level::Low
    }
}

impl<I> DongleScreen<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    fn redraw_wait(&self) -> Duration {
        let rate_limit_wait = self
            .min_interval
            .checked_sub(self.last_render.elapsed())
            .unwrap_or(Duration::MIN);
        let pointing_wait = rmk::split::ble::central::pointing_quiet_period_remaining(
            POINTING_REDRAW_QUIET_PERIOD,
        );
        if pointing_wait > rate_limit_wait {
            pointing_wait
        } else {
            rate_limit_wait
        }
    }

    fn modifier_redraw_wait(&self) -> Duration {
        MODIFIER_REDRAW_MIN_INTERVAL
            .checked_sub(self.last_modifier_render.elapsed())
            .unwrap_or(Duration::MIN)
    }

    fn next_redraw_wait(&self) -> Option<Duration> {
        match (self.pending, self.modifier_pending) {
            (true, true) => Some(self.redraw_wait().min(self.modifier_redraw_wait())),
            (true, false) => Some(self.redraw_wait()),
            (false, true) => Some(self.modifier_redraw_wait()),
            (false, false) => None,
        }
    }

    async fn redraw(&mut self) {
        self.sync_host_data();
        self.sync_layer_names();
        self.sync_settings();
        if self.redraw_wait() != Duration::MIN {
            self.pending = true;
            return;
        }
        if self.blanked {
            // The backlight is off; paint one black frame so the panel buffer
            // matches the dark screen, then stay quiet until an event wakes us.
            if self.blank_frame_due {
                self.blank_frame_due = false;
                let look = self.look();
                self.lcd
                    .present_dirty(&mut self.renderer, &self.ctx, DirtyRegion::Full, &look)
                    .await;
            }
            self.pending = false;
            self.dirty = DirtyRegion::Full;
            return;
        }
        let look = self.look();
        self.lcd
            .present_dirty(&mut self.renderer, &self.ctx, self.dirty, &look)
            .await;
        self.ctx.key_press_latch = false;
        self.pending = false;
        self.dirty = DirtyRegion::Full;
        self.last_render = Instant::now();
    }

    async fn redraw_modifiers(&mut self) {
        if self.blanked || self.modifier_redraw_wait() != Duration::MIN {
            return;
        }
        let Some(band) = self.renderer.layout().modifiers else {
            self.modifier_pending = false;
            return;
        };
        let look = self.look();
        self.lcd
            .present_modifiers(&self.renderer, &self.ctx, band, &look)
            .await;
        self.modifier_pending = false;
        self.last_modifier_render = Instant::now();
    }

    fn request_redraw(&mut self) {
        self.pending = true;
        self.dirty = DirtyRegion::Full;
    }

    fn request_redraw_region(&mut self, dirty: DirtyRegion) {
        self.dirty = if self.pending { self.dirty.union(dirty) } else { dirty };
        self.pending = true;
    }

    fn request_modifier_redraw(&mut self) {
        self.modifier_pending = true;
    }

    /// The badge lives either in the header or in the chip row, so a transport
    /// change repaints whichever band currently carries it.
    fn request_badge_redraw(&mut self) {
        if !self.renderer.settings.output_visible {
            return;
        }
        if self.renderer.settings.output_place == SCREEN_OUTPUT_CHIP {
            self.request_modifier_redraw();
        } else {
            self.request_redraw_region(HEADER_DIRTY);
        }
    }

    fn sync_host_data(&mut self) {
        let host_data = rmk::host_data::snapshot();
        if host_data != self.last_host_data {
            self.last_host_data = host_data.clone();
            self.renderer.host_data = host_data;
            self.request_redraw_region(HEADER_DIRTY);
        }
    }

    fn sync_layer_names(&mut self) {
        let version = layer_names::version();
        if version != self.last_layer_names_version {
            self.last_layer_names_version = version;
            self.request_redraw_region(row_dirty(self.renderer.layout().row));
        }
    }

    /// Re-reads the settings whenever a client changed one and repaints the
    /// whole frame (zones may have moved, colours changed).
    fn sync_settings(&mut self) {
        let version = layer_names::screen_settings_version();
        if version == self.last_screen_settings_version {
            return;
        }
        self.last_screen_settings_version = version;
        self.settings = layer_names::screen_settings();
        self.renderer.settings = self.settings;
        self.request_redraw();
    }

    fn look(&self) -> Look {
        Look::from_settings(&self.renderer.settings, self.blanked)
    }

    /// Any keyboard/connection event counts as activity: it clears the blank
    /// and restarts the idle timer.
    fn note_activity(&mut self) {
        self.last_activity = Instant::now();
        if self.blanked {
            self.blanked = false;
            self.blank_frame_due = false;
            self.backlight.set_level(backlight_level(true));
            self.request_redraw();
        }
    }

    /// Turns the panel dark once the configured idle timeout elapsed.
    fn check_idle_timeout(&mut self) {
        if self.blanked {
            return;
        }
        let timeout = self.renderer.settings.timeout_s;
        if timeout == 0 {
            return;
        }
        if self.last_activity.elapsed() < Duration::from_secs(timeout as u64) {
            return;
        }
        self.blanked = true;
        self.blank_frame_due = true;
        self.backlight.set_level(backlight_level(false));
        self.request_redraw();
    }
}

pub struct NeverEvent;
struct NeverSub;

impl EventSubscriber for NeverSub {
    type Event = NeverEvent;
    async fn next_event(&mut self) -> NeverEvent {
        core::future::pending().await
    }
}

impl<I> Runnable for DongleScreen<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    async fn run(&mut self) -> ! {
        self.pending = true;
        self.sync_host_data();
        self.redraw().await;

        let mut layer_sub = LayerChangeEvent::subscriber();
        let mut wpm_sub = WpmUpdateEvent::subscriber();
        let mut led_sub = LedIndicatorEvent::subscriber();
        let mut mod_sub = ModifierEvent::subscriber();
        let mut key_sub = KeyboardEvent::subscriber();
        let mut sleep_sub = SleepStateEvent::subscriber();
        let mut bat_sub = BatteryStatusEvent::subscriber();
        let mut conn_sub = ConnectionStatusChangeEvent::subscriber();
        let mut adv_sub = BleAdvertisingModeEvent::subscriber();
        let mut peri_conn_sub = PeripheralConnectedEvent::subscriber();
        let mut peri_bat_sub = PeripheralBatteryEvent::subscriber();
        let mut central_sub = CentralConnectedEvent::subscriber();

        loop {
            // Wait for at least one event (or deferred redraw timer).
            if let Some(wait) = self.next_redraw_wait() {
                match select(
                    Timer::after(wait),
                    Self::next_any_or_host_tick(
                        &mut layer_sub,
                        &mut wpm_sub,
                        &mut led_sub,
                        &mut mod_sub,
                        &mut key_sub,
                        &mut sleep_sub,
                        &mut bat_sub,
                        &mut conn_sub,
                        &mut adv_sub,
                        &mut peri_conn_sub,
                        &mut peri_bat_sub,
                        &mut central_sub,
                    ),
                )
                .await
                {
                    Either::First(_) => {}
                    Either::Second(ev) => {
                        self.apply(ev);
                    }
                }
            } else {
                let ev = Self::next_any_or_host_tick(
                    &mut layer_sub,
                    &mut wpm_sub,
                    &mut led_sub,
                    &mut mod_sub,
                    &mut key_sub,
                    &mut sleep_sub,
                    &mut bat_sub,
                    &mut conn_sub,
                    &mut adv_sub,
                    &mut peri_conn_sub,
                    &mut peri_bat_sub,
                    &mut central_sub,
                )
                .await;
                self.apply(ev);
            }

            // Coalesce a burst of events that arrived during the previous
            // multipass present (layer MO + OSM mods, etc.) before redrawing.
            for _ in 0..16 {
                match select(
                    Timer::after(Duration::from_millis(0)),
                    Self::next_any(
                        &mut layer_sub,
                        &mut wpm_sub,
                        &mut led_sub,
                        &mut mod_sub,
                        &mut key_sub,
                        &mut sleep_sub,
                        &mut bat_sub,
                        &mut conn_sub,
                        &mut adv_sub,
                        &mut peri_conn_sub,
                        &mut peri_bat_sub,
                        &mut central_sub,
                    ),
                )
                .await
                {
                    Either::First(_) => break,
                    Either::Second(ev) => self.apply(ev),
                }
            }

            if self.modifier_pending {
                self.redraw_modifiers().await;
            }
            self.check_idle_timeout();
            if self.pending {
                self.redraw().await;
            }
        }
    }
}

/// Unified UI event for the dongle screen loop.
enum UiEv {
    Layer(LayerChangeEvent),
    Wpm(WpmUpdateEvent),
    Led(LedIndicatorEvent),
    Mod(ModifierEvent),
    Key(KeyboardEvent),
    Sleep(SleepStateEvent),
    Bat(BatteryStatusEvent),
    Conn(ConnectionStatusChangeEvent),
    Adv(BleAdvertisingModeEvent),
    PeriConn(PeripheralConnectedEvent),
    PeriBat(PeripheralBatteryEvent),
    Central(CentralConnectedEvent),
    HostDataTick,
}

impl<I> DongleScreen<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    async fn next_any_or_host_tick(
        layer: &mut impl EventSubscriber<Event = LayerChangeEvent>,
        wpm: &mut impl EventSubscriber<Event = WpmUpdateEvent>,
        led: &mut impl EventSubscriber<Event = LedIndicatorEvent>,
        mods: &mut impl EventSubscriber<Event = ModifierEvent>,
        key: &mut impl EventSubscriber<Event = KeyboardEvent>,
        sleep: &mut impl EventSubscriber<Event = SleepStateEvent>,
        bat: &mut impl EventSubscriber<Event = BatteryStatusEvent>,
        conn: &mut impl EventSubscriber<Event = ConnectionStatusChangeEvent>,
        adv: &mut impl EventSubscriber<Event = BleAdvertisingModeEvent>,
        peri_conn: &mut impl EventSubscriber<Event = PeripheralConnectedEvent>,
        peri_bat: &mut impl EventSubscriber<Event = PeripheralBatteryEvent>,
        central: &mut impl EventSubscriber<Event = CentralConnectedEvent>,
    ) -> UiEv {
        match select(
            Timer::after(Duration::from_millis(250)),
            Self::next_any(
                layer, wpm, led, mods, key, sleep, bat, conn, adv, peri_conn, peri_bat, central,
            ),
        )
        .await
        {
            Either::First(_) => UiEv::HostDataTick,
            Either::Second(ev) => ev,
        }
    }

    async fn next_any(
        layer: &mut impl EventSubscriber<Event = LayerChangeEvent>,
        wpm: &mut impl EventSubscriber<Event = WpmUpdateEvent>,
        led: &mut impl EventSubscriber<Event = LedIndicatorEvent>,
        mods: &mut impl EventSubscriber<Event = ModifierEvent>,
        key: &mut impl EventSubscriber<Event = KeyboardEvent>,
        sleep: &mut impl EventSubscriber<Event = SleepStateEvent>,
        bat: &mut impl EventSubscriber<Event = BatteryStatusEvent>,
        conn: &mut impl EventSubscriber<Event = ConnectionStatusChangeEvent>,
        adv: &mut impl EventSubscriber<Event = BleAdvertisingModeEvent>,
        peri_conn: &mut impl EventSubscriber<Event = PeripheralConnectedEvent>,
        peri_bat: &mut impl EventSubscriber<Event = PeripheralBatteryEvent>,
        central: &mut impl EventSubscriber<Event = CentralConnectedEvent>,
    ) -> UiEv {
        // Nested select — a bit verbose but no heap / macro dependency.
        // Prefer input events; depth is fine for status UI.
        use embassy_futures::select::{select, select3, Either, Either3};

        match select3(
            select3(layer.next_event(), wpm.next_event(), led.next_event()),
            select3(mods.next_event(), key.next_event(), sleep.next_event()),
            select3(
                select3(bat.next_event(), conn.next_event(), adv.next_event()),
                select(peri_conn.next_event(), peri_bat.next_event()),
                central.next_event(),
            ),
        )
        .await
        {
            Either3::First(Either3::First(e)) => UiEv::Layer(e),
            Either3::First(Either3::Second(e)) => UiEv::Wpm(e),
            Either3::First(Either3::Third(e)) => UiEv::Led(e),
            Either3::Second(Either3::First(e)) => UiEv::Mod(e),
            Either3::Second(Either3::Second(e)) => UiEv::Key(e),
            Either3::Second(Either3::Third(e)) => UiEv::Sleep(e),
            Either3::Third(Either3::First(Either3::First(e))) => UiEv::Bat(e),
            Either3::Third(Either3::First(Either3::Second(e))) => UiEv::Conn(e),
            Either3::Third(Either3::First(Either3::Third(e))) => UiEv::Adv(e),
            Either3::Third(Either3::Second(Either::First(e))) => UiEv::PeriConn(e),
            Either3::Third(Either3::Second(Either::Second(e))) => UiEv::PeriBat(e),
            Either3::Third(Either3::Third(e)) => UiEv::Central(e),
        }
    }

    fn apply(&mut self, ev: UiEv) {
        // Keyboard matrix floods KeyboardEvent; UI doesn't show individual
        // keys — skip redraw for those so multipass can keep up with layer/mod.
        // Every real event (but not the bare host-data tick) counts as user
        // activity and wakes a blanked screen.
        if !matches!(&ev, UiEv::HostDataTick) {
            self.note_activity();
        }
        let mut need_redraw = true;
        match ev {
            UiEv::Layer(e) => {
                self.ctx.layer = e.0;
                self.request_redraw_region(row_dirty(self.renderer.layout().row));
                need_redraw = false;
            }
            UiEv::Wpm(e) => {
                // Only repaint when the number really changed: RMK may publish
                // repeated WPM updates and each band repaint costs a stripe.
                if self.ctx.wpm != e.0 {
                    self.ctx.wpm = e.0;
                    // The WPM block shares the row with the layer name: there is
                    // no separate band to repaint.
                    self.request_redraw_region(row_dirty(self.renderer.layout().row));
                }
                need_redraw = false;
            }
            UiEv::Led(e) => {
                self.ctx.caps_lock = e.0.caps_lock();
                self.ctx.num_lock = e.0.num_lock();
                self.request_modifier_redraw();
                need_redraw = false;
            }
            UiEv::Mod(e) => {
                self.ctx.modifiers = e.modifier;
                self.request_modifier_redraw();
                need_redraw = false;
            }
            UiEv::Key(e) => {
                self.ctx.key_pressed = e.pressed;
                if e.pressed {
                    self.ctx.key_press_latch = true;
                }
                need_redraw = false;
            }
            UiEv::Sleep(e) => self.ctx.sleeping = e.0,
            UiEv::Bat(e) => {
                self.ctx.battery = e;
                if let Some(bat) = self.renderer.layout().batteries {
                    self.request_redraw_region(band_dirty(bat));
                }
                need_redraw = false;
            }
            UiEv::Conn(e) => {
                self.ctx.ble_status = e.0.ble;
                self.renderer.connection = e.0;
                self.request_badge_redraw();
                need_redraw = false;
            }
            UiEv::Adv(e) => {
                self.renderer.advertising = Some(e.0);
                self.request_badge_redraw();
                need_redraw = false;
            }
            UiEv::PeriConn(e) => {
                if let Some(slot) = self.ctx.peripherals_connected.get_mut(e.id) {
                    *slot = e.connected;
                }
                if let Some(bat) = self.renderer.layout().batteries {
                    self.request_redraw_region(band_dirty(bat));
                }
                need_redraw = false;
            }
            UiEv::PeriBat(e) => {
                if let Some(slot) = self.ctx.peripheral_batteries.get_mut(e.id) {
                    *slot = e.state;
                }
                if let Some(bat) = self.renderer.layout().batteries {
                    self.request_redraw_region(band_dirty(bat));
                }
                need_redraw = false;
            }
            UiEv::Central(e) => self.ctx.central_connected = e.connected,
            UiEv::HostDataTick => {
                if self.renderer.splash && self.boot_at.elapsed() >= SPLASH_DURATION {
                    self.renderer.splash = false;
                    self.request_redraw();
                }
                self.sync_host_data();
                self.sync_layer_names();
                self.sync_settings();
                if self.renderer.media_needs_marquee() {
                    self.request_redraw_region(HEADER_DIRTY);
                }
                need_redraw = false;
            }
        }
        if need_redraw {
            self.request_redraw();
        }
    }
}

impl<I> Processor for DongleScreen<I>
where
    I: interrupt::typelevel::Binding<
            <SPI3 as spim::Instance>::Interrupt,
            spim::InterruptHandler<SPI3>,
        > + Copy
        + 'static,
{
    type Event = NeverEvent;
    fn subscriber() -> impl EventSubscriber<Event = NeverEvent> {
        NeverSub
    }
    async fn process(&mut self, _: NeverEvent) {}
    async fn process_loop(&mut self) -> ! {
        self.run().await
    }
}

// Silence unused DisplayDriver import path if needed — keep for future.

// --- Full-screen UI ---------------------------------------------------------
//
// Vertical zones (280x240) so nothing overlaps:
//   14..42   compact header (indicator + media ticker + clock)
//   46..100  layer panel (compact, x2 name)
//   106..160 WPM panel (x2 value)
//   166..182 modifier state
//   188..228 battery cards
//
// Hiding a zone moves the ones below it up and hands the freed height to the
// layer panel; see `compute_layout`.

pub struct QubeStatusRenderer {
    host_data: rmk::host_data::HostData,
    /// Boot splash is on screen, the dashboard is not drawn yet.
    splash: bool,
    /// Screen settings snapshot (kept in sync by [`DongleScreen::sync_settings`]).
    settings: ScreenSettings,
    /// Last `ConnectionStatusChangeEvent`.
    connection: ConnectionStatus,
    /// Last `BleAdvertisingModeEvent`, used to tell pairing from reconnecting.
    advertising: Option<BleAdvertisingMode>,
}

impl QubeStatusRenderer {
    pub fn new(host_data: rmk::host_data::HostData, splash: bool) -> Self {
        Self {
            host_data,
            splash,
            settings: layer_names::screen_settings(),
            connection: ConnectionStatus::default(),
            advertising: None,
        }
    }

    fn layout(&self) -> Layout {
        compute_layout(&self.settings)
    }

    fn geometry(&self) -> Geometry {
        geometry(&self.settings)
    }

    /// Badge text + colour for the current connection state.
    fn output_badge(&self) -> (heapless::String<OUTPUT_BADGE_CHARS>, Rgb565) {
        OutputState::from_connection(&self.connection, self.advertising).badge()
    }

    fn media_needs_marquee(&self) -> bool {
        if !self.settings.shows_media() {
            return false;
        }
        let mut media: heapless::String<72> = heapless::String::new();
        push_media_label(&mut media, &self.host_data);
        media.len() > self.geometry().media_limit
    }

    /// Modifier chips (six fixed slots in v2; the last one is the badge when the
    /// placement setting says so).
    fn render_modifiers<D: DrawTarget<Color = Rgb565>>(
        &self,
        ctx: &RenderContext,
        display: &mut D,
        band: Band,
    ) {
        if self.splash {
            return;
        }
        let y = band.y;
        draw_chip(display, 26, y, 34, "CAPS", ctx.caps_lock);
        draw_chip(
            display,
            64,
            y,
            34,
            "CTRL",
            ctx.modifiers.left_ctrl() || ctx.modifiers.right_ctrl(),
        );
        draw_chip(
            display,
            102,
            y,
            42,
            "SHIFT",
            ctx.modifiers.left_shift() || ctx.modifiers.right_shift(),
        );
        draw_chip(
            display,
            148,
            y,
            30,
            "ALT",
            ctx.modifiers.left_alt() || ctx.modifiers.right_alt(),
        );
        draw_chip(
            display,
            182,
            y,
            30,
            "GUI",
            ctx.modifiers.left_gui() || ctx.modifiers.right_gui(),
        );
        if self.settings.output_visible && self.settings.output_place == SCREEN_OUTPUT_CHIP {
            let (text, color) = self.output_badge();
            let rect = Rectangle::new(
                Point::new(BADGE_CHIP_X, y),
                Size::new(BADGE_CHIP_W, band.h),
            );
            let style = PrimitiveStyleBuilder::new()
                .fill_color(COL_ACCENT_DIM)
                .stroke_color(color)
                .stroke_width(1)
                .build();
            let _ = RoundedRectangle::with_equal_corners(rect, Size::new(CHIP_RADIUS, CHIP_RADIUS))
                .into_styled(style)
                .draw(display);
            draw_round_fill(display, BADGE_CHIP_X + 6, y + 3, 3, 10, 2, color);
            let _ = Text::with_text_style(
                &text,
                Point::new(BADGE_CHIP_X + 12, y + 3),
                MonoTextStyle::new(&FONT_6X10, COL_FG),
                TextStyleBuilder::new().baseline(Baseline::Top).build(),
            )
            .draw(display);
        }
    }
}

impl DisplayRenderer<Rgb565> for QubeStatusRenderer {
    fn render<D: DrawTarget<Color = Rgb565>>(&mut self, ctx: &RenderContext, display: &mut D) {
        let _ = display.clear(COL_BG);

        if self.splash {
            render_splash(display);
            return;
        }

        let layout = self.layout();
        let geo = self.geometry();
        let settings = self.settings;
        let layer_meta = MonoTextStyle::new(&FONT_6X10, COL_LABEL);
        let header_media = MonoTextStyle::new(&FONT_6X10, COL_FG);
        let header_fallback = MonoTextStyle::new(&FONT_8X13, COL_ACCENT);
        let body = MonoTextStyle::new(&FONT_8X13, COL_FG);
        let badge_text_style = MonoTextStyle::new(&FONT_6X10, COL_FG);
        let top_left = TextStyleBuilder::new().baseline(Baseline::Top).build();
        let tr = TextStyleBuilder::new()
            .alignment(Alignment::Right)
            .baseline(Baseline::Top)
            .build();
        let left = ctx.peripherals_connected.first().copied().unwrap_or(false);
        let right = ctx.peripherals_connected.get(1).copied().unwrap_or(false);
        let lp = battery_reading(ctx.peripheral_batteries.first().map(|b| b.0));
        let rp = battery_reading(ctx.peripheral_batteries.get(1).map(|b| b.0));
        let mut custom_name = [0u8; layer_names::LAYER_NAME_MAX];
        let name = layer_names::copy_layer_name(ctx.layer, &mut custom_name)
            .and_then(|len| core::str::from_utf8(&custom_name[..len]).ok())
            .unwrap_or_else(|| layer_name(ctx.layer));

        // Header.
        draw_panel(
            display,
            SAFE_X,
            layout.header.y,
            SAFE_W,
            layout.header.h,
            COL_PANEL,
            COL_BORDER_DIM,
        );
        let (badge_text, badge_color) = self.output_badge();
        // Decorative accent dot, unchanged from the pre-settings frame.
        draw_round_fill(
            display,
            SAFE_X + 11,
            layout.header.y + 9,
            3,
            10,
            2,
            COL_ACCENT,
        );
        let clock_drawn = match settings.header_mode {
            SCREEN_HEADER_CLOCK => true,
            SCREEN_HEADER_MEDIA => false,
            _ => host_time_available(&self.host_data),
        };
        let mut s: heapless::String<16> = heapless::String::new();
        draw_media_or_fallback(
            display,
            &self.host_data,
            &settings,
            geo.media_x,
            geo.media_limit,
            layout.header.y,
            clock_drawn,
            header_media,
            header_fallback,
        );
        if clock_drawn {
            push_host_time(&mut s, self.host_data.hour, self.host_data.minute);
            let _ = Text::with_text_style(
                &s,
                Point::new(geo.clock_right, layout.header.y + 7),
                body,
                tr,
            )
            .draw(display);
        }
        // Connection badge: right end of the header, after the clock.
        if geo.badge_in_header {
            let dot_x = geo.header_right - geo.badge_w;
            draw_round_fill(display, dot_x, layout.header.y + 9, 3, 10, 2, badge_color);
            let _ = Text::with_text_style(
                &badge_text,
                Point::new(dot_x + 7, layout.header.y + 8),
                badge_text_style,
                top_left,
            )
            .draw(display);
        }

        // One row: layer name on the left, compact WPM block on the right.
        // The row absorbs the height of every hidden zone.
        draw_panel(
            display,
            SAFE_X,
            layout.row.y,
            SAFE_W,
            layout.row.h,
            COL_PANEL_HI,
            COL_BORDER_DIM,
        );
        s.clear();
        let _ = write!(&mut s, "L{}", ctx.layer);
        let _ = Text::with_text_style(
            &s,
            Point::new(geo.name_x, layout.row.y + 6),
            layer_meta,
            top_left,
        )
        .draw(display);

        // Compact WPM: caption above the value, both FONT_6X10, right aligned.
        if settings.wpm_visible {
            let stack_h = 10 + 4 + 10;
            let top = layout.row.y + (layout.row.h as i32 - stack_h) / 2;
            let _ = Text::with_text_style(
                "WPM",
                Point::new(geo.wpm_right, top),
                MonoTextStyle::new(&FONT_6X10, COL_MUTED),
                tr,
            )
            .draw(display);
            s.clear();
            let _ = write!(&mut s, "{}", ctx.wpm);
            let _ = Text::with_text_style(
                &s,
                Point::new(geo.wpm_right, top + 14),
                MonoTextStyle::new(&FONT_6X10, COL_ACCENT),
                tr,
            )
            .draw(display);
        }

        draw_layer_name_fitted(
            display,
            name,
            geo.name_x,
            layout.row,
            geo.name_limit - geo.name_x,
            geo.f10,
        );

        // Modifier chips.
        if let Some(mods) = layout.modifiers {
            self.render_modifiers(ctx, display, mods);
        }

        // Battery cards.
        if let Some(bat) = layout.batteries {
            let left_label = fit_battery_label(
                settings.left_label.as_str(),
                BATTERY_CARD_W,
                battery_value_len(lp, left),
            );
            let right_label = fit_battery_label(
                settings.right_label.as_str(),
                BATTERY_CARD_W,
                battery_value_len(rp, right),
            );
            draw_bat(
                display,
                SAFE_X,
                bat.y,
                BATTERY_CARD_W,
                lp,
                left,
                left_label.as_str(),
            );
            draw_bat(
                display,
                SAFE_X + BATTERY_CARD_W + BATTERY_CARD_GAP,
                bat.y,
                BATTERY_CARD_W,
                rp,
                right,
                right_label.as_str(),
            );
        }
    }
}

/// Boot splash: accent bars, x2 wordmark and the firmware version.
fn render_splash<D: DrawTarget<Color = Rgb565>>(display: &mut D) {
    let top = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Top)
        .build();
    let version_style = MonoTextStyle::new(&FONT_8X13, COL_MUTED);


    draw_round_fill(display, 96, 72, 88, 3, 1, COL_ACCENT);
    draw_text_x2(
        display,
        "QUBE",
        Point::new(SPLASH_WORD_TOP.x + 1, SPLASH_WORD_TOP.y + 1),
        MonoTextStyle::new(&FONT_10X20, COL_ACCENT_DIM),
        Alignment::Center,
    );
    draw_text_x2(
        display,
        "QUBE",
        SPLASH_WORD_TOP,
        MonoTextStyle::new(&FONT_10X20, COL_FG),
        Alignment::Center,
    );
    draw_round_fill(display, 60, 140, 160, 2, 1, COL_ACCENT_DIM);


    let mut version: heapless::String<24> = heapless::String::new();
    let _ = write!(&mut version, "RMK {}", FIRMWARE_VERSION);
    let _ = Text::with_text_style(
        &version,
        Point::new(SCREEN_W as i32 / 2, 152),
        version_style,
        top,
    )
    .draw(display);
}


fn draw_panel<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    x: i32,
    y: i32,
    w: u32,
    h: u32,
    fill: Rgb565,
    stroke: Rgb565,
) {
    let rect = Rectangle::new(Point::new(x, y), Size::new(w, h));
    let style = PrimitiveStyleBuilder::new()
        .fill_color(fill)
        .stroke_color(stroke)
        .stroke_width(1)
        .build();
    let _ = RoundedRectangle::with_equal_corners(rect, Size::new(PANEL_RADIUS, PANEL_RADIUS))
        .into_styled(style)
        .draw(display);
}

fn draw_round_fill<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    x: i32,
    y: i32,
    w: u32,
    h: u32,
    radius: u32,
    fill: Rgb565,
) {
    let rect = Rectangle::new(Point::new(x, y), Size::new(w, h));
    let _ = RoundedRectangle::with_equal_corners(rect, Size::new(radius, radius))
        .into_styled(PrimitiveStyle::with_fill(fill))
        .draw(display);
}

fn draw_chip<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    x: i32,
    y: i32,
    w: u32,
    label: &str,
    active: bool,
) {
    let text = if active {
        MonoTextStyle::new(&FONT_6X10, COL_FG)
    } else {
        MonoTextStyle::new(&FONT_6X10, COL_DIM)
    };
    if active {
        let rect = Rectangle::new(Point::new(x, y), Size::new(w, 16));
        let style = PrimitiveStyleBuilder::new()
            .fill_color(COL_ACCENT_DIM)
            .stroke_color(COL_ACCENT)
            .stroke_width(1)
            .build();
        let _ = RoundedRectangle::with_equal_corners(rect, Size::new(CHIP_RADIUS, CHIP_RADIUS))
            .into_styled(style)
            .draw(display);
    }
    let tc = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Top)
        .build();
    let _ =
        Text::with_text_style(label, Point::new(x + w as i32 / 2, y + 3), text, tc).draw(display);
}
#[derive(Clone, Copy)]
enum BatReading {
    Unknown,
    Pending,
    Pct(u8),
}

fn battery_reading(status: Option<BatteryStatus>) -> BatReading {
    match status {
        Some(BatteryStatus::Available {
            level: Some(level), ..
        }) => BatReading::Pct(level),
        Some(BatteryStatus::Available { level: None, .. }) => BatReading::Pending,
        Some(BatteryStatus::Unavailable) | None => BatReading::Unknown,
    }
}

/// Width of the value text `draw_bat` right-aligns: `--`, `??` or `NN%`.
fn battery_value_len(reading: BatReading, connected: bool) -> usize {
    match (connected, reading) {
        (false, _) | (true, BatReading::Unknown) | (true, BatReading::Pending) => 2,
        (true, BatReading::Pct(pct)) => {
            let digits = if pct >= 100 {
                3
            } else if pct >= 10 {
                2
            } else {
                1
            };
            digits + 1
        }
    }
}

/// Draws the layer name at the largest size that fits the row:
/// 2× if it fits, else 1×, else 1× truncated with a trailing `..`.
/// Same stepped rule as the stand's `draw_layer_name_fitted`.
fn draw_layer_name_fitted<D>(
    display: &mut D,
    name: &str,
    x: i32,
    row: Band,
    avail: i32,
    f10: i32,
) where
    D: DrawTarget<Color = Rgb565>,
{
    if name.is_empty() {
        return;
    }
    let len = name.chars().count() as i32;
    let two_x_px = 2 * f10 * len;
    let one_x_px = f10 * len;

    if two_x_px <= avail {
        let y = row.y + (row.h as i32 - 22 * 2) / 2;
        draw_text_x2(
            display,
            name,
            Point::new(x + 1, y + 1),
            MonoTextStyle::new(&FONT_10X20, COL_ACCENT_DIM),
            Alignment::Left,
        );
        draw_text_x2(
            display,
            name,
            Point::new(x, y),
            MonoTextStyle::new(&FONT_10X20, COL_FG),
            Alignment::Left,
        );
    } else if one_x_px <= avail {
        let y = row.y + (row.h as i32 - 20) / 2;
        let top_left = TextStyleBuilder::new().baseline(Baseline::Top).build();
        let _ = Text::with_text_style(
            name,
            Point::new(x + 1, y + 1),
            MonoTextStyle::new(&FONT_10X20, COL_ACCENT_DIM),
            top_left,
        )
        .draw(display);
        let _ = Text::with_text_style(
            name,
            Point::new(x, y),
            MonoTextStyle::new(&FONT_10X20, COL_FG),
            top_left,
        )
        .draw(display);
    } else {
        let keep = ((avail - 2 * f10) / f10).max(1) as usize;
        let mut cut: heapless::String<16> = heapless::String::new();
        for ch in name.chars().take(keep) {
            let _ = cut.push(ch);
        }
        let _ = cut.push_str("..");
        let y = row.y + (row.h as i32 - 20) / 2;
        let _ = Text::with_text_style(
            cut.as_str(),
            Point::new(x, y),
            MonoTextStyle::new(&FONT_10X20, COL_FG),
            TextStyleBuilder::new().baseline(Baseline::Top).build(),
        )
        .draw(display);
    }
}

/// Clips a card label to the space left of the value. `draw_bat` puts the
/// label at `x + 10` (FONT_6X10) and the value right-aligned at `x + w - 12`
/// (FONT_10X20), leaving a 6 px gap — the same budget the host stand uses, so
/// the PNG previews and the panel truncate identically.
fn fit_battery_label(label: &str, card_w: i32, value_len: usize) -> heapless::String<BATTERY_LABEL_MAX> {
    let value_px = value_len as i32 * 10;
    let max_px = ((card_w - 12) - value_px - 6) - 10;
    let max_chars = (max_px.max(0) / 6) as usize;
    let mut text: heapless::String<BATTERY_LABEL_MAX> = heapless::String::new();
    for (index, ch) in label.chars().enumerate() {
        if index >= max_chars {
            break;
        }
        let _ = text.push(ch);
    }
    text
}

fn draw_bat<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    x: i32,
    y: i32,
    w: i32,
    reading: BatReading,
    connected: bool,
    side: &str,
) {
    let (label, col, fill_pct): (heapless::String<8>, Rgb565, Option<u8>) =
        match (connected, reading) {
            (false, _) => {
                let mut s = heapless::String::new();
                let _ = s.push_str("--");
                (s, COL_DIM, None)
            }
            (true, BatReading::Unknown) | (true, BatReading::Pending) => {
                let mut s = heapless::String::new();
                let _ = s.push_str("??");
                (s, COL_DIM, None)
            }
            (true, BatReading::Pct(p)) => {
                let mut s = heapless::String::new();
                let _ = write!(&mut s, "{}%", p);
                let c = if p < 10 {
                    COL_RED
                } else if p < 25 {
                    COL_YELLOW
                } else {
                    COL_FG
                };
                (s, c, Some(p))
            }
        };

    draw_panel(display, x, y, w as u32, 40, COL_PANEL, COL_BORDER_DIM);

    let title = MonoTextStyle::new(&FONT_6X10, COL_MUTED);
    let percent = MonoTextStyle::new(&FONT_10X20, col);
    let top = TextStyleBuilder::new().baseline(Baseline::Top).build();
    let tr = TextStyleBuilder::new()
        .alignment(Alignment::Right)
        .baseline(Baseline::Top)
        .build();
    let _ = Text::with_text_style(side, Point::new(x + 10, y + 8), title, top).draw(display);
    let _ = Text::with_text_style(&label, Point::new(x + w - 12, y + 6), percent, tr).draw(display);

    let bx = x + 10;
    let by = y + 30;
    let bw = w - 28;
    let bh = 8u32;
    let bar = RoundedRectangle::with_equal_corners(
        Rectangle::new(Point::new(bx, by), Size::new(bw as u32, bh)),
        Size::new(BAR_RADIUS, BAR_RADIUS),
    );
    let bar_style = PrimitiveStyleBuilder::new()
        .fill_color(COL_BAR_BG)
        .stroke_color(COL_BORDER)
        .stroke_width(1)
        .build();
    let _ = bar.into_styled(bar_style).draw(display);
    draw_round_fill(display, bx + bw + 2, by + 2, 4, 3, 2, COL_BORDER_DIM);
    if let Some(pct) = fill_pct {
        if pct > 0 {
            let inner = (bw - 4).max(1) as u32;
            let fw = (inner * pct as u32 / 100).max(2);
            let fc = if pct < 10 {
                COL_RED
            } else if pct < 25 {
                COL_YELLOW
            } else {
                COL_BAR_FG
            };
            draw_round_fill(display, bx + 2, by + 2, fw, bh - 4, 3, fc);
        }
    }
}

fn layer_name(layer: u8) -> &'static str {
    crate::DEFAULT_LAYER_NAMES
        .get(layer as usize)
        .copied()
        .unwrap_or("?")
}

fn push_host_time(buffer: &mut heapless::String<16>, hour: Option<u8>, minute: Option<u8>) {
    match (hour, minute) {
        (Some(hour), Some(minute)) => {
            let _ = write!(buffer, "{:02}:{:02}", hour, minute);
        }
        _ => {
            let _ = buffer.push_str("--:--");
        }
    }
}

fn draw_media_or_fallback<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    host_data: &rmk::host_data::HostData,
    settings: &ScreenSettings,
    media_x: i32,
    media_limit: usize,
    header_y: i32,
    clock_drawn: bool,
    media_style: MonoTextStyle<'_, Rgb565>,
    fallback_style: MonoTextStyle<'_, Rgb565>,
) {
    if !settings.shows_media() {
        return;
    }
    let top = TextStyleBuilder::new().baseline(Baseline::Top).build();
    let mut media: heapless::String<72> = heapless::String::new();
    push_media_label(&mut media, host_data);

    if media.is_empty() {
        // With a clock on screen the wordmark stays away; in media-only mode it
        // is the only thing that can fill the header.
        if !clock_drawn {
            let _ = Text::with_text_style("QUBE", Point::new(media_x, header_y + 7), fallback_style, top)
                .draw(display);
        }
        return;
    }

    let visible_chars = media_limit;
    let mut visible: heapless::String<32> = heapless::String::new();
    if media.chars().count() <= visible_chars {
        let _ = visible.push_str(&media);
    } else {
        let elapsed = Instant::now()
            .duration_since(Instant::from_ticks(0))
            .as_millis() as usize;
        let offset = (elapsed / 300) % (media.len() + MEDIA_GAP_CHARS);
        push_marquee_slice(&mut visible, &media, offset, visible_chars);
    }

    let _ = Text::with_text_style(&visible, Point::new(media_x, header_y + 8), media_style, top)
        .draw(display);
}

fn push_media_label(buffer: &mut heapless::String<72>, host_data: &rmk::host_data::HostData) {
    if !host_data.media_artist.is_empty() {
        push_ascii_text(buffer, &host_data.media_artist);
    }
    if !host_data.media_title.is_empty() {
        if !buffer.is_empty() {
            let _ = buffer.push_str(" - ");
        }
        push_ascii_text(buffer, &host_data.media_title);
    }
}

fn push_ascii_text<const N: usize>(buffer: &mut heapless::String<N>, value: &str) {
    for ch in value.chars() {
        let ch = if ch.is_ascii_graphic() || ch == ' ' {
            ch
        } else {
            '?'
        };
        if buffer.push(ch).is_err() {
            break;
        }
    }
}

fn push_marquee_slice(
    buffer: &mut heapless::String<32>,
    text: &str,
    offset: usize,
    visible_chars: usize,
) {
    let bytes = text.as_bytes();
    let cycle_len = bytes.len() + MEDIA_GAP_CHARS;
    for i in 0..visible_chars {
        let idx = (offset + i) % cycle_len;
        let ch = if idx < bytes.len() {
            bytes[idx] as char
        } else {
            ' '
        };
        let _ = buffer.push(ch);
    }
}

fn host_time_available(host_data: &rmk::host_data::HostData) -> bool {
    host_data.hour.is_some() && host_data.minute.is_some()
}
