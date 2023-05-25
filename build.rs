use chrono::{Timelike, Datelike, FixedOffset, NaiveDateTime};

fn main() {

	let repo = git2::Repository::open(env!("CARGO_MANIFEST_DIR")).unwrap();

	let head = repo.head().unwrap();
	let branch = head.shorthand().unwrap();
	let commit = head.peel_to_commit().unwrap();
	let time = commit.time().seconds();	
    
	let tz_offset = FixedOffset::west_opt(1 * 3600).unwrap();
	let datetime = NaiveDateTime::from_timestamp_opt(time, 0).unwrap()
		.and_local_timezone(tz_offset).unwrap();

	let local = datetime.naive_utc();

	let mut version_info = String::new();
	version_info += format!("pub const VERSION_BRANCH: &str = {:?};\r\n", branch).as_ref();
	version_info += format!("pub const VERSION_YEAR: i32 = {:?};\r\n", local.year()).as_ref();
	version_info += format!("pub const VERSION_MONTH: i32 = {:?};\r\n", local.month()).as_ref();
	version_info += format!("pub const VERSION_DAY: i32 = {:?};\r\n", local.day()).as_ref();
	version_info += format!("pub const VERSION_HOUR: i32 = {:?};\r\n", local.hour()).as_ref();
	version_info += format!("pub const VERSION_MINUTE: i32 = {:?};\r\n", local.minute()).as_ref();
	version_info += format!("pub const VERSION_SECOND: i32 = {:?};\r\n", local.second()).as_ref();

	let out_dir = std::env::var("OUT_DIR").unwrap();
	let dest_path = std::path::Path::new(&out_dir).join("version_info.rs");
	std::fs::write(&dest_path, version_info).unwrap();
	
}