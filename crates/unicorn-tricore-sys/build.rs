use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Find `<venv>/lib/python*/site-packages/unicorn/lib` (Unix virtualenv layout).
fn unix_venv_lib(venv: &Path) -> Option<PathBuf> {
    let lib = venv.join("lib");
    let mut entries: Vec<PathBuf> = fs::read_dir(&lib)
        .ok()?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("python"))
        })
        .collect();
    entries.sort();
    entries
        .into_iter()
        .map(|p| p.join("site-packages/unicorn/lib"))
        .find(|p| p.exists())
}

/// If `workspace_root` is a linked git worktree (`.git` file pointing at
/// `<main>/.git/worktrees/<name>`), return `<main>` so shared `.venv` and
/// `target/unicorn-simemu` builds in the parent checkout are also discovered.
fn git_worktree_parent_repo(workspace_root: &Path) -> Option<PathBuf> {
    let dot_git = workspace_root.join(".git");
    if !dot_git.is_file() {
        return None;
    }
    let text = fs::read_to_string(&dot_git).ok()?;
    let gitdir_raw = text.strip_prefix("gitdir:")?.trim();
    let gitdir = {
        let p = PathBuf::from(gitdir_raw);
        if p.is_absolute() {
            p
        } else {
            workspace_root.join(p)
        }
    };
    // `<main>/.git/worktrees/<name>` -> 3 parents up is `<main>`.
    let worktrees_dir = gitdir.parent()?;
    if worktrees_dir.file_name().and_then(|s| s.to_str()) != Some("worktrees") {
        return None;
    }
    let main_dot_git = worktrees_dir.parent()?;
    main_dot_git.parent().map(Path::to_path_buf)
}

/// Return the Cargo link library name (`"unicorn"` or `"unicorn-static"`) if `dir`
/// contains a linkable Unicorn library for the target OS/toolchain.
fn find_unicorn_in_dir(
    dir: &Path,
    target_os: &str,
    target_env: &str,
    want_static: bool,
) -> Option<&'static str> {
    if !dir.is_dir() {
        return None;
    }
    if target_os == "windows" {
        if want_static {
            if dir.join("unicorn-static.lib").is_file() {
                return Some("unicorn-static");
            }
            if dir.join("libunicorn.a").is_file() || dir.join("unicorn.lib").is_file() {
                return Some("unicorn");
            }
            None
        } else if target_env == "gnu" {
            let names = [
                "libunicorn.dll.a",
                "unicorn.dll.lib",
                "unicorn.lib",
                "unicorn.dll",
                "libunicorn.dll",
            ];
            names
                .iter()
                .any(|n| dir.join(n).is_file())
                .then_some("unicorn")
        } else {
            dir.join("unicorn.lib").is_file().then_some("unicorn")
        }
    } else if target_os == "macos" {
        if want_static {
            dir.join("libunicorn.a").is_file().then_some("unicorn")
        } else {
            let names = ["libunicorn.dylib", "libunicorn.2.dylib", "libunicorn.a"];
            names
                .iter()
                .any(|n| dir.join(n).is_file())
                .then_some("unicorn")
        }
    } else if want_static {
        dir.join("libunicorn.a").is_file().then_some("unicorn")
    } else {
        let names = ["libunicorn.so", "libunicorn.so.2", "libunicorn.a"];
        names
            .iter()
            .any(|n| dir.join(n).is_file())
            .then_some("unicorn")
    }
}

/// Query `pkg-config --libs-only-L unicorn` on non-Windows targets.
fn pkg_config_lib_dirs() -> Vec<PathBuf> {
    let Ok(out) = Command::new("pkg-config")
        .args(["--libs-only-L", "unicorn"])
        .output()
    else {
        return Vec::new();
    };
    if !out.status.success() {
        return Vec::new();
    }
    let stdout = String::from_utf8_lossy(&out.stdout);
    stdout
        .split_whitespace()
        .filter_map(|tok| tok.strip_prefix("-L"))
        .map(PathBuf::from)
        .filter(|p| p.is_dir())
        .collect()
}

/// Standard system library directories for the target platform.
fn system_lib_dirs(target_os: &str, target_arch: &str) -> Vec<PathBuf> {
    let mut raw: Vec<&str> = Vec::new();
    match target_os {
        "macos" => {
            raw.extend(["/opt/homebrew/lib", "/usr/local/lib"]);
        }
        "linux" => {
            raw.extend(["/usr/local/lib", "/usr/local/lib64"]);
            match target_arch {
                "x86_64" => raw.push("/usr/lib/x86_64-linux-gnu"),
                "aarch64" => raw.push("/usr/lib/aarch64-linux-gnu"),
                "arm" => raw.push("/usr/lib/arm-linux-gnueabihf"),
                "riscv64" => raw.push("/usr/lib/riscv64-linux-gnu"),
                _ => {}
            }
            raw.extend([
                "/usr/lib/x86_64-linux-gnu",
                "/usr/lib/aarch64-linux-gnu",
                "/usr/lib64",
                "/usr/lib",
            ]);
        }
        "freebsd" | "openbsd" | "netbsd" => {
            raw.extend(["/usr/local/lib", "/usr/lib"]);
        }
        _ => {}
    }
    let mut out = Vec::new();
    for s in raw {
        let p = PathBuf::from(s);
        if !out.contains(&p) {
            out.push(p);
        }
    }
    out
}

/// Candidate directories where `scripts/build_unicorn.py` places its output.
fn patched_build_candidates(workspace_root: &Path, target_os: &str) -> Vec<PathBuf> {
    let mut bases = Vec::new();
    if let Ok(ctd) = env::var("CARGO_TARGET_DIR") {
        let ctd_path = PathBuf::from(ctd);
        bases.push(ctd_path.join("unicorn-simemu/build"));
        if let Some(parent) = ctd_path.parent() {
            bases.push(parent.join("unicorn-simemu/build"));
        }
    }
    for anc in workspace_root.ancestors() {
        bases.push(anc.join("target/unicorn-simemu/build"));
    }
    if let Some(main_repo) = git_worktree_parent_repo(workspace_root) {
        bases.push(main_repo.join("target/unicorn-simemu/build"));
    }

    let mut out = Vec::new();
    for b in bases {
        // Multi-config CMake generators on Windows (Visual Studio) place artifacts in `build/Release`.
        if target_os == "windows" {
            let rel = b.join("Release");
            if !out.contains(&rel) {
                out.push(rel);
            }
        }
        if !out.contains(&b) {
            out.push(b);
        }
    }
    out
}

/// Extra ambient linker search paths supplied via toolchain environment variables.
fn ambient_linker_dirs(target_os: &str) -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    let path_var = if target_os == "windows" {
        "LIB"
    } else {
        "LIBRARY_PATH"
    };
    if let Some(val) = env::var_os(path_var) {
        dirs.extend(env::split_paths(&val));
    }
    for flag_var in ["RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "NIX_LDFLAGS"] {
        if let Ok(val) = env::var(flag_var) {
            let sep_tokens: Vec<&str> = if flag_var == "CARGO_ENCODED_RUSTFLAGS" {
                val.split('\x1f').collect()
            } else {
                val.split_whitespace().collect()
            };
            let mut it = sep_tokens.into_iter();
            while let Some(tok) = it.next() {
                if let Some(rest) = tok.strip_prefix("-Lnative=") {
                    dirs.push(PathBuf::from(rest));
                } else if let Some(rest) = tok.strip_prefix("-L") {
                    if rest.is_empty() {
                        if let Some(next) = it.next() {
                            dirs.push(PathBuf::from(next));
                        }
                    } else {
                        dirs.push(PathBuf::from(rest));
                    }
                }
            }
        }
    }
    dirs
}

fn copy_windows_dll(dir: &Path) {
    for dll_name in ["unicorn.dll", "libunicorn.dll"] {
        let dll_src = dir.join(dll_name);
        if dll_src.is_file() {
            println!("cargo:rerun-if-changed={}", dll_src.display());
            if let Ok(out_dir) = env::var("OUT_DIR") {
                let out_path = PathBuf::from(out_dir);
                // OUT_DIR is target/<profile>/build/<crate>/out -> 3 levels up is target/<profile>.
                if let Some(profile_dir) = out_path.ancestors().nth(3) {
                    let _ = fs::copy(&dll_src, profile_dir.join(dll_name));
                    let deps_dir = profile_dir.join("deps");
                    let _ = fs::create_dir_all(&deps_dir);
                    let _ = fs::copy(&dll_src, deps_dir.join(dll_name));
                }
            }
            break;
        }
    }
}

fn emit_rerun_for_lib_dir(dir: &Path, target_os: &str) {
    let candidates: &[&str] = if target_os == "windows" {
        &["unicorn.lib", "unicorn-static.lib", "unicorn.dll", "libunicorn.a"]
    } else if target_os == "macos" {
        &["libunicorn.a", "libunicorn.dylib", "libunicorn.2.dylib"]
    } else {
        &["libunicorn.a", "libunicorn.so", "libunicorn.so.2"]
    };
    for name in candidates {
        let p = dir.join(name);
        if p.exists() {
            println!("cargo:rerun-if-changed={}", p.display());
        }
    }
}

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root");

    // `cfg!(target_os)` in a build script describes the HOST; the target comes from Cargo.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let target_env = env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    let target_arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let want_static_env = env::var_os("UNICORN_STATIC").is_some();

    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_LINK");
    println!("cargo:rerun-if-env-changed=UNICORN_STATIC");
    println!("cargo:rerun-if-env-changed=UNICORN_LIB_DIR");
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");
    println!("cargo:rerun-if-changed=build.rs");

    if env::var_os("CARGO_FEATURE_LINK").is_none() {
        return;
    }

    // 1. Explicit UNICORN_LIB_DIR takes highest precedence and must be valid if set.
    if let Some(explicit_os) = env::var_os("UNICORN_LIB_DIR") {
        let dir = PathBuf::from(&explicit_os);
        let Some(link_name) =
            find_unicorn_in_dir(&dir, &target_os, &target_env, want_static_env)
        else {
            panic!(
                "UNICORN_LIB_DIR={} does not contain a linkable Unicorn 2.1.x library for target `{}-{}` (static={}).\n\
                 Expected e.g. `unicorn.lib` (+ `unicorn.dll`) on Windows MSVC, `libunicorn.dylib`/`libunicorn.a` on macOS, \
                 or `libunicorn.so`/`libunicorn.a` on Linux.\n\
                 Install the Python wheel (`pip install unicorn==2.1.4` in `.venv`), install a system package, \
                 or run `python scripts/build_unicorn.py`.",
                dir.display(),
                target_os,
                target_env,
                want_static_env,
            );
        };
        emit_rerun_for_lib_dir(&dir, &target_os);
        println!("cargo:rustc-link-search=native={}", dir.display());
        if target_os == "windows" && !want_static_env {
            copy_windows_dll(&dir);
        }
        if want_static_env {
            println!("cargo:rustc-link-lib=static={link_name}");
            if target_os != "windows" {
                println!("cargo:rustc-link-lib=dylib=m");
                println!("cargo:rustc-link-lib=dylib=pthread");
            }
        } else {
            println!("cargo:rustc-link-lib=dylib={link_name}");
        }
        return;
    }

    // 2. Patched Unicorn build from `scripts/build_unicorn.py` (`target/unicorn-simemu/build`).
    //    On Unix we link `libunicorn.a` statically (copied into an isolated directory so `ld64`/`ld`
    //    will not prefer a sibling `.dylib`/`.so`). On Windows, `scripts/build_unicorn.py` produces
    //    `unicorn.lib` + `unicorn.dll` (linked dynamically unless `UNICORN_STATIC=1` is requested).
    let patched_candidates = patched_build_candidates(workspace_root, &target_os);
    let patched_static = want_static_env || target_os != "windows";
    if let Some((patched_dir, link_name)) = patched_candidates.iter().find_map(|d| {
        find_unicorn_in_dir(d, &target_os, &target_env, patched_static).map(|n| (d.clone(), n))
    }) {
        emit_rerun_for_lib_dir(&patched_dir, &target_os);
        if patched_static && target_os != "windows" {
            let isolated = PathBuf::from(env::var("OUT_DIR").unwrap()).join("unicorn-static");
            let _ = fs::create_dir_all(&isolated);
            let _ = fs::copy(
                patched_dir.join("libunicorn.a"),
                isolated.join("libunicorn.a"),
            );
            println!("cargo:rustc-link-search=native={}", isolated.display());
            println!("cargo:rustc-link-lib=static={link_name}");
            println!("cargo:rustc-link-lib=dylib=m");
            println!("cargo:rustc-link-lib=dylib=pthread");
        } else {
            println!("cargo:rustc-link-search=native={}", patched_dir.display());
            if target_os == "windows" && !patched_static {
                copy_windows_dll(&patched_dir);
            }
            let kind = if patched_static { "static" } else { "dylib" };
            println!("cargo:rustc-link-lib={kind}={link_name}");
        }
        return;
    }
    // Watch default patched output path so building it triggers a rebuild automatically.
    let default_patched = workspace_root.join("target/unicorn-simemu/build");
    println!(
        "cargo:rerun-if-changed={}",
        default_patched.join("libunicorn.a").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        default_patched.join("unicorn.lib").display()
    );

    // 3. Project `.venv` (in this workspace or a parent checkout of a git worktree),
    //    `pkg-config`, and platform system library paths.
    let mut search_dirs: Vec<PathBuf> = Vec::new();
    let mut venv_roots = vec![workspace_root.join(".venv")];
    if let Some(main_repo) = git_worktree_parent_repo(workspace_root) {
        venv_roots.push(main_repo.join(".venv"));
    }
    for venv in venv_roots {
        if target_os == "windows" {
            let win = venv.join("Lib/site-packages/unicorn/lib");
            if win.is_dir() {
                search_dirs.push(win);
            }
        } else if let Some(u) = unix_venv_lib(&venv) {
            search_dirs.push(u);
        }
    }
    if target_os != "windows" {
        for d in pkg_config_lib_dirs() {
            if !search_dirs.contains(&d) {
                search_dirs.push(d);
            }
        }
    }
    for d in system_lib_dirs(&target_os, &target_arch) {
        if d.is_dir() && !search_dirs.contains(&d) {
            search_dirs.push(d);
        }
    }

    let matched = search_dirs.iter().find_map(|d| {
        find_unicorn_in_dir(d, &target_os, &target_env, want_static_env).map(|n| (d.clone(), n))
    });
    let ambient_matched = ambient_linker_dirs(&target_os).into_iter().find_map(|d| {
        find_unicorn_in_dir(&d, &target_os, &target_env, want_static_env).map(|n| (d, n))
    });

    let Some((chosen_dir, link_name)) = matched.or(ambient_matched) else {
        let checked: Vec<String> = search_dirs.iter().map(|p| p.display().to_string()).collect();
        panic!(
            "Could not find a linkable Unicorn 2.1.x library (`libunicorn` / `unicorn.lib`) for target `{}-{}`.\n\
             Searched directories: {:?}\n\
             Provide Unicorn via one of:\n\
               1. Python virtualenv in repo root: `python -m venv .venv && .venv/bin/pip install unicorn==2.1.4` \
                  (Windows: `.venv\\Scripts\\pip install unicorn==2.1.4`)\n\
               2. System package: `brew install unicorn` (macOS) or `libunicorn-dev` + `pkg-config` (Linux)\n\
               3. Patched source build: `python scripts/build_unicorn.py`\n\
               4. Custom location: set `UNICORN_LIB_DIR=/path/to/unicorn/lib`\n\
             Note: pure-Rust crates (`tricore-core` lib, `simemu-physics`, `simemu-damos`, `simemu-acoustics`, \
             `simemu-bridge`) do not require Unicorn unless building tests/crates that link `unicorn-tricore-sys`.",
            target_os, target_env, checked,
        );
    };

    emit_rerun_for_lib_dir(&chosen_dir, &target_os);
    for d in &search_dirs {
        println!("cargo:rustc-link-search=native={}", d.display());
    }
    if !search_dirs.contains(&chosen_dir) {
        println!("cargo:rustc-link-search=native={}", chosen_dir.display());
    }
    if target_os == "windows" && !want_static_env {
        copy_windows_dll(&chosen_dir);
    }

    if want_static_env {
        println!("cargo:rustc-link-lib=static={link_name}");
        if target_os != "windows" {
            println!("cargo:rustc-link-lib=dylib=m");
            println!("cargo:rustc-link-lib=dylib=pthread");
        }
    } else {
        println!("cargo:rustc-link-lib=dylib={link_name}");
    }
}
