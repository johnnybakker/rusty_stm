use core::panic;



fn main() {

	let repo = git2::Repository::open(env!("CARGO_MANIFEST_DIR")).unwrap();

	let head = repo.head().unwrap();
	
	if head.is_tag() {
		let tag = head.peel_to_tag().unwrap();
		panic!("Tag: {}", tag.name().unwrap());
	}

	let commit = head.peel_to_commit().unwrap();
	panic!("Commit: {}", commit.id());
}