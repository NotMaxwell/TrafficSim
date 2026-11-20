//! Smoke tests of the headless CLI.

use std::process::Command;

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_traffic-batch"))
}

fn scenario(name: &str) -> String {
    format!("{}/../../scenarios/{name}.toml", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn sweep_writes_one_row_per_share() {
    let out = std::env::temp_dir().join(format!("traffic-batch-test-{}.csv", std::process::id()));
    let st = bin()
        .args(["sweep", &scenario("city_a_to_b"), "--minutes", "6", "--step", "25", "--warmup", "2", "--out"])
        .arg(&out)
        .status()
        .unwrap();
    assert!(st.success());
    let csv = std::fs::read_to_string(&out).unwrap();
    std::fs::remove_file(&out).ok();
    assert_eq!(csv.lines().count(), 6, "{csv}");
    assert!(csv.starts_with("auto_share,throughput_vph"));
}

#[test]
fn bad_arguments_fail_with_usage() {
    let o = bin().arg("bogus").output().unwrap();
    assert!(!o.status.success());
    assert!(String::from_utf8_lossy(&o.stderr).contains("usage"));
}
