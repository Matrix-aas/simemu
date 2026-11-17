fn main() {
    if let Err(e) = simemu_cli::run_cli() {
        let prefix = simemu_damos::tr("Error", "Ошибка");
        eprintln!("{prefix}: {e}");
        std::process::exit(1);
    }
}
