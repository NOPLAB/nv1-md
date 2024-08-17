use embassy_stm32::time::{hz, khz, Hertz};

pub const C3: Hertz = hz(130);
pub const D3: Hertz = hz(146);
pub const E3: Hertz = hz(164);
pub const F3: Hertz = hz(174);
pub const G3: Hertz = hz(196);
pub const A3: Hertz = hz(220);
pub const B3: Hertz = hz(246);
pub const C4: Hertz = hz(261);
pub const D4: Hertz = hz(293);
pub const E4: Hertz = hz(329);
pub const F4: Hertz = hz(349);
pub const G4: Hertz = hz(392);
pub const A4: Hertz = hz(440);
pub const B4: Hertz = hz(493);
pub const C5: Hertz = hz(523);
pub const D5: Hertz = hz(587);
pub const E5: Hertz = hz(659);
pub const F5: Hertz = hz(698);
pub const G5: Hertz = hz(783);
pub const A5: Hertz = hz(880);
pub const B5: Hertz = hz(987);
pub const C6: Hertz = hz(1046);

pub const MUSIC_DOREMI: [(Hertz, u32); 13] = [
    (E5, 500),
    (D5, 500),
    (C5, 500),
    (D5, 500),
    (E5, 500),
    (E5, 500),
    (E5, 500),
    (D5, 500),
    (D5, 500),
    (E5, 500),
    (D5, 500),
    (C5, 500),
    (khz(1), 500),
];
