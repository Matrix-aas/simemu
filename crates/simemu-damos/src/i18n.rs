//! Global CLI / session language selector (`en` by default, `ru` when requested via `--lang ru` / `--ru` / `SIMEMU_LANG=ru`).

use std::sync::atomic::{AtomicU8, Ordering};

/// Console and backend log language (`En` is the default).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Lang {
    En = 0,
    Ru = 1,
}

static CURRENT_LANG: AtomicU8 = AtomicU8::new(Lang::En as u8);

impl Lang {
    pub fn parse(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "en" | "eng" | "english" => Ok(Self::En),
            "ru" | "rus" | "russian" | "ру" | "рус" => Ok(Self::Ru),
            other => Err(format!(
                "unsupported language {other:?} (expected 'en' or 'ru')"
            )),
        }
    }

    pub fn code(self) -> &'static str {
        match self {
            Self::En => "en",
            Self::Ru => "ru",
        }
    }
}

pub fn lang() -> Lang {
    match CURRENT_LANG.load(Ordering::Relaxed) {
        1 => Lang::Ru,
        _ => Lang::En,
    }
}

pub fn set_lang(l: Lang) {
    CURRENT_LANG.store(l as u8, Ordering::Relaxed);
}

#[inline]
pub fn is_ru() -> bool {
    lang() == Lang::Ru
}

#[inline]
pub fn tr<'a>(en: &'a str, ru: &'a str) -> &'a str {
    if is_ru() {
        ru
    } else {
        en
    }
}
