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
                    let mut i = 0;
                    while i < rom.prg_rom.len() {
                        let byte = rom.prg_rom[i];
                        
                        if byte == 0b10101001 {
                            match Opcode::from_binary(byte >> 4) {
                                Some(opcode) => println!("Binary '{:08b}' corresponds to opcode {:?}", byte, opcode),
                                None => println!("No corresponding opcode for binary '{:08b}'", byte),
                            }

                            match AddressingMode::from_binary(byte & 0b00001111, i) {
                                Some(opcode) => println!("Binary '{:08b}' corresponds to Addressing Mode {:?}", byte, opcode),
                                None => println!("No corresponding opcode for binary '{:08b}'", byte),
                            }
                        }

                        i += 1; // Move to the next byte
                    }
                }
                Err(e) => eprintln!("Erro ao carregar ROM: {}", e),
            }
        } 
    }

    Ok(())
}


#[derive(Debug)]
enum AddressingMode {
    Immediate(usize),       
    ZeroPage(usize),         
    Absolute(usize),        
    Indirect(usize),       
}

impl AddressingMode {
    fn to_binary(&self) -> (u8, usize) {
        match self {
            AddressingMode::Immediate(addr) => (0b00001001, *addr),
            AddressingMode::ZeroPage(addr) => (0b00000101, *addr),
            AddressingMode::Absolute(addr) => (0b00001101, *addr),
            AddressingMode::Indirect(addr) => (0b00000001, *addr),
        }
    }

    fn from_binary(binary: u8, address: usize) -> Option<AddressingMode> {
        match binary {
            0b00001001 => Some(AddressingMode::Immediate(address)),
            0b00000101 => Some(AddressingMode::ZeroPage(address)),
            0b00001101 => Some(AddressingMode::Absolute(address)),
            0b00000001 => Some(AddressingMode::Indirect(address)),
            _ => None,  
        }
    }
}

#[derive(Debug)]

enum Opcode {
    LDA,
    STA,
    ADD,
    SUB,
}

impl Opcode {
    fn to_binary(&self) -> u8 {
        match self {
            Opcode::LDA => 0b1010,
            Opcode::STA => 0b1011,
            Opcode::ADD => 0b1100,
            Opcode::SUB => 0b1101,
        }
    }

    fn from_binary(binary: u8) -> Option<Opcode> {
        match binary {
            0b1010 => Some(Opcode::LDA),
            0b1011 => Some(Opcode::STA),
            0b1100 => Some(Opcode::ADD),
            0b1101 => Some(Opcode::SUB),
            _ => None, 
        }
    }
}
fn main() {
    let _ = get_games();
}