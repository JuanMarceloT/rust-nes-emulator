use std::fs::File;
use std::io::Read;
use std::path::Path;


use std::fs;
use std::io;



#[derive(Debug)]
#[allow(dead_code)]
struct INesHeader {
    magic: [u8; 4],      // "NES\x1A"
    prg_rom_size: u8,    // Tamanho da PRG-ROM (em blocos de 16 KB)
    chr_rom_size: u8,    // Tamanho da CHR-ROM (em blocos de 8 KB)
    flags_6: u8,         // Flags de controle
    flags_7: u8,         // Flags adicionais
    reserved: [u8; 8],   
}

impl INesHeader {
    fn is_valid(&self) -> bool {
        self.magic == [0x4E, 0x45, 0x53, 0x1A] // "NES\x1A"
    }
}


#[derive(Debug)]
struct NesRom {
    header: INesHeader,  // Cabeçalho iNES
    prg_rom: Vec<u8>,    // Código - PRG-ROM
    chr_rom: Vec<u8>,    // Gráficos - CHR-ROM
}

impl NesRom {
    fn load<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let mut file = File::open(path)?;

        let mut header_bytes = [0u8; 16];
        file.read_exact(&mut header_bytes)?;

        let header = INesHeader {
            magic: [header_bytes[0], header_bytes[1], header_bytes[2], header_bytes[3]],
            prg_rom_size: header_bytes[4],
            chr_rom_size: header_bytes[5],
            flags_6: header_bytes[6],
            flags_7: header_bytes[7],
            reserved: header_bytes[8..16].try_into().unwrap(),
        };

        if !header.is_valid() {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "ROM inválida"));
        }

        let prg_size = (header.prg_rom_size as usize) * 16 * 1024; // Blocos de 16 KB
        let chr_size = (header.chr_rom_size as usize) * 8 * 1024;  // Blocos de 8 KB

        let mut prg_rom = vec![0u8; prg_size];
        file.read_exact(&mut prg_rom)?;

        let mut chr_rom = vec![0u8; chr_size];
        if chr_size > 0 {
            file.read_exact(&mut chr_rom)?;
        }

        Ok(Self {
            header,
            prg_rom,
            chr_rom,
        })
    }

    fn print_info(&self) {
        println!("ROM iNES:");
        println!("  Tamanho da PRG-ROM: {} KB", self.prg_rom.len() / 1024);
        println!(
            "  Tamanho da CHR-ROM: {} KB",
            self.chr_rom.len() / 1024
        );
        println!("  Flags 6: {:08b}", self.header.flags_6);
        println!("  Flags 7: {:08b}", self.header.flags_7);
    }
}


fn get_games() -> io::Result<()> {
    let folder_path = "./../roms";

    // Read the directory
    for entry in fs::read_dir(folder_path)? {
        let entry = entry?;
        let path = entry.path();

        if path.is_file() {
            println!("File: {}", path.display());
            match NesRom::load(path) {
                Ok(rom) => {
                    rom.print_info();
                }
                Err(e) => eprintln!("Erro ao carregar ROM: {}", e),
            }
        } 
    }

    Ok(())
}
fn main() {
    let _ = get_games();
}