use std::fs::File;
use std::io::BufReader;
use std::io::Read;


use std::fs;
use std::io;

fn get_games() -> io::Result<()> {
    let folder_path = "./../roms";

    // Read the directory
    for entry in fs::read_dir(folder_path)? {
        let entry = entry?;
        let path = entry.path();

        if path.is_file() {
            println!("File: {}", path.display());
        } 
        // else if path.is_dir() {
        //     println!("Directory: {}", path.display());
        // }
    }

    Ok(())
}
fn main() {
    let my_buf = BufReader::new(File::open("./../roms/superMario.nes").unwrap());
    let _ = get_games();
    // for byte_or_error in my_buf.bytes() {
    //     let byte = byte_or_error.unwrap();
    //     println!("{:?}", byte);
    // }
}