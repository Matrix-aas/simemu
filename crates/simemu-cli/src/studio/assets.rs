use std::sync::OnceLock;

const STYLES_CSS: &str = include_str!("assets/styles.css");
const BODY_HTML: &str = include_str!("assets/body.html");
/// Plain-JS modules sharing one script scope, in load order; `diag.js` ends with `boot()`.
const SCRIPTS: [&str; 9] = [
    include_str!("assets/core.js"),
    include_str!("assets/channels.js"),
    include_str!("assets/live.js"),
    include_str!("assets/dyno.js"),
    include_str!("assets/calib.js"),
    include_str!("assets/sound.js"),
    include_str!("assets/build.js"),
    include_str!("assets/check.js"),
    include_str!("assets/diag.js"),
];

static COMPOSED_STUDIO_HTML_EN: OnceLock<String> = OnceLock::new();
static COMPOSED_STUDIO_HTML_RU: OnceLock<String> = OnceLock::new();

fn compose_html(lang: &str) -> String {
    let js = SCRIPTS.join("\n");
    format!(
        "<!DOCTYPE html>\n\
         <html lang=\"{lang}\">\n\
         <head>\n\
         <meta charset=\"UTF-8\">\n\
         <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n\
         <title>SIMEMU Studio</title>\n\
         <link rel=\"icon\" href=\"data:,\">\n\
         <style>\n{STYLES_CSS}\n</style>\n\
         </head>\n\
         <body>\n{BODY_HTML}\n\
         <script>\nwindow.__SIMEMU_DEFAULT_LANG = \"{lang}\";\n{js}\n</script>\n\
         </body>\n\
         </html>\n"
    )
}

/// Composes the CSS, HTML markup and JS into the single-page Studio HTML (no external resources).
pub fn studio_index_html() -> &'static str {
    if simemu_damos::is_ru() {
        COMPOSED_STUDIO_HTML_RU.get_or_init(|| compose_html("ru"))
    } else {
        COMPOSED_STUDIO_HTML_EN.get_or_init(|| compose_html("en"))
    }
}
