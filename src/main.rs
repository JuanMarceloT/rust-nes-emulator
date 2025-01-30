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
                        
                        
                            match Opcode::from_binary(byte) {
                                Some(opcode) => println!("Binary '{:08b}' corresponds to opcode {:?}", byte, opcode),
                                None => println!("No corresponding opcode for binary '{:08b}'", byte),
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
    Immediate,       // Example: #00 - The value is provided directly in the instruction itself.
    ZeroPage,        // Example: $00 - The address is within the first 256 bytes of memory.
    ZeroPageX,       // Example: $00,X - The address is within the first 256 bytes, plus an X register offset.
    ZeroPageY,       // Example: $00,Y - The address is within the first 256 bytes, plus a Y register offset.
    Absolute,        // Example: $1234 - The full 16-bit memory address is given in the instruction.
    AbsoluteX,       // Example: $1234,X - The instruction gives a full 16-bit address, plus an X register offset.
    AbsoluteY,       // Example: $1234,Y - The instruction gives a full 16-bit address, plus a Y register offset.
    Indirect,        // Example: ($1234) - The instruction points to a memory address that contains the actual address to use.
    IndirectX,       // Example: ($1234,X) - The instruction points to a memory address with an X register offset.
    IndirectY,       // Example: ($1234),Y - The instruction points to a memory address with a Y register offset.
    Relative,        // Example: $12 - A signed offset that is added to the program counter for branching instructions.
    Implied,         // Example: NOP, TAX - No operand needed. The operation affects a specific register, like the accumulator or X/Y registers.
}


// impl AddressingMode {
// }

#[derive(Debug)]
enum Opcode {
    ADC,  // Add with carry
    AND,  // AND with accumulator
    ASL,  // Arithmetic shift left
    BCC,  // Branch on carry clear
    BCS,  // Branch on carry set
    BEQ,  // Branch on equal
    BIT,  // Bit test
    BMI,  // Branch on minus
    BNE,  // Branch on not equal
    BPL,  // Branch on plus
    BRK,  // Break
    BVC,  // Branch on overflow clear
    BVS,  // Branch on overflow set
    CLC,  // Clear carry flag
    CLD,  // Clear decimal mode
    CLI,  // Clear interrupt disable
    CLV,  // Clear overflow flag
    CMP,  // Compare accumulator
    CPX,  // Compare X register
    CPY,  // Compare Y register
    DEC,  // Decrement memory
    DEX,  // Decrement X register
    DEY,  // Decrement Y register
    EOR,  // Exclusive OR with accumulator
    INC,  // Increment memory
    INX,  // Increment X register
    INY,  // Increment Y register
    JMP,  // Jump to address
    JSR,  // Jump to subroutine
    LDA,  // Load accumulator
    LDX,  // Load X register
    LDY,  // Load Y register
    LSR,  // Logical shift right
    NOP,  // No operation
    ORA,  // OR with accumulator
    PHA,  // Push accumulator
    PHP,  // Push processor status
    PLA,  // Pull accumulator
    PLP,  // Pull processor status
    ROL,  // Rotate left
    ROR,  // Rotate right
    RTI,  // Return from interrupt
    RTS,  // Return from subroutine
    SBC,  // Subtract with carry
    SEC,  // Set carry flag
    SED,  // Set decimal mode
    SEI,  // Set interrupt disable
    STA,  // Store accumulator
    STX,  // Store X register
    STY,  // Store Y register
    TAX,  // Transfer accumulator to X
    TAY,  // Transfer accumulator to Y
    TSX,  // Transfer stack pointer to X
    TXA,  // Transfer X to accumulator
    TXS,  // Transfer X to stack pointer
    TYA,  // Transfer Y to accumulator
}

impl Opcode {
    fn from_binary(binary: u8) -> Option<(Opcode, AddressingMode, u8, u8)> {
        match binary {
            // ADC (Add with carry)
            0x69 => Some((Opcode::ADC, AddressingMode::Immediate, 2, 2)),
            0x65 => Some((Opcode::ADC, AddressingMode::ZeroPage, 2, 3)),
            0x75 => Some((Opcode::ADC, AddressingMode::ZeroPageX, 2, 4)),
            0x6D => Some((Opcode::ADC, AddressingMode::Absolute, 3, 4)),
            0x7D => Some((Opcode::ADC, AddressingMode::AbsoluteX, 3, 4)),
            0x79 => Some((Opcode::ADC, AddressingMode::AbsoluteY, 3, 4)),
            0x61 => Some((Opcode::ADC, AddressingMode::IndirectX, 2, 6)),
            0x71 => Some((Opcode::ADC, AddressingMode::IndirectY, 2, 5)),

            // AND (AND with accumulator)
            0x29 => Some((Opcode::AND, AddressingMode::Immediate, 2, 2)),
            0x25 => Some((Opcode::AND, AddressingMode::ZeroPage, 2, 3)),
            0x35 => Some((Opcode::AND, AddressingMode::ZeroPageX, 2, 4)),
            0x2D => Some((Opcode::AND, AddressingMode::Absolute, 3, 4)),
            0x3D => Some((Opcode::AND, AddressingMode::AbsoluteX, 3, 4)),
            0x39 => Some((Opcode::AND, AddressingMode::AbsoluteY, 3, 4)),
            0x21 => Some((Opcode::AND, AddressingMode::IndirectX, 2, 6)),
            0x31 => Some((Opcode::AND, AddressingMode::IndirectY, 2, 5)),

            // ASL (Arithmetic shift left)
            0x0A => Some((Opcode::ASL, AddressingMode::Implied, 1, 2)),
            0x06 => Some((Opcode::ASL, AddressingMode::ZeroPage, 2, 5)),
            0x16 => Some((Opcode::ASL, AddressingMode::ZeroPageX, 2, 6)),
            0x0E => Some((Opcode::ASL, AddressingMode::Absolute, 3, 6)),
            0x1E => Some((Opcode::ASL, AddressingMode::AbsoluteX, 3, 7)),

            // BCC (Branch on carry clear)
            0x90 => Some((Opcode::BCC, AddressingMode::Relative, 2, 2)),

            // BCS (Branch on carry set)
            0xB0 => Some((Opcode::BCS, AddressingMode::Relative, 2, 2)),

            // BEQ (Branch on equal)
            0xF0 => Some((Opcode::BEQ, AddressingMode::Relative, 2, 2)),

            // BIT (Bit test)
            0x24 => Some((Opcode::BIT, AddressingMode::ZeroPage, 2, 3)),
            0x2C => Some((Opcode::BIT, AddressingMode::Absolute, 3, 4)),

            // BMI (Branch on minus)
            0x30 => Some((Opcode::BMI, AddressingMode::Relative, 2, 2)),

            // BNE (Branch on not equal)
            0xD0 => Some((Opcode::BNE, AddressingMode::Relative, 2, 2)),

            // BPL (Branch on plus)
            0x10 => Some((Opcode::BPL, AddressingMode::Relative, 2, 2)),

            // BRK (Break)
            0x00 => Some((Opcode::BRK, AddressingMode::Implied, 1, 7)),

            // BVC (Branch on overflow clear)
            0x50 => Some((Opcode::BVC, AddressingMode::Relative, 2, 2)),

            // BVS (Branch on overflow set)
            0x70 => Some((Opcode::BVS, AddressingMode::Relative, 2, 2)),

            // CLC (Clear carry flag)
            0x18 => Some((Opcode::CLC, AddressingMode::Implied, 1, 2)),

            // CLD (Clear decimal mode)
            0xD8 => Some((Opcode::CLD, AddressingMode::Implied, 1, 2)),

            // CLI (Clear interrupt disable)
            0x58 => Some((Opcode::CLI, AddressingMode::Implied, 1, 2)),

            // CLV (Clear overflow flag)
            0xB8 => Some((Opcode::CLV, AddressingMode::Implied, 1, 2)),

            // CMP (Compare accumulator)
            0xC9 => Some((Opcode::CMP, AddressingMode::Immediate, 2, 2)),
            0xC5 => Some((Opcode::CMP, AddressingMode::ZeroPage, 2, 3)),
            0xD5 => Some((Opcode::CMP, AddressingMode::ZeroPageX, 2, 4)),
            0xCD => Some((Opcode::CMP, AddressingMode::Absolute, 3, 4)),
            0xDD => Some((Opcode::CMP, AddressingMode::AbsoluteX, 3, 4)),
            0xD9 => Some((Opcode::CMP, AddressingMode::AbsoluteY, 3, 4)),
            0xC1 => Some((Opcode::CMP, AddressingMode::IndirectX, 2, 6)),
            0xD1 => Some((Opcode::CMP, AddressingMode::IndirectY, 2, 5)),

            // CPX (Compare X register)
            0xE0 => Some((Opcode::CPX, AddressingMode::Immediate, 2, 2)),
            0xE4 => Some((Opcode::CPX, AddressingMode::ZeroPage, 2, 3)),
            0xEC => Some((Opcode::CPX, AddressingMode::Absolute, 3, 4)),

            // CPY (Compare Y register)
            0xC0 => Some((Opcode::CPY, AddressingMode::Immediate, 2, 2)),
            0xC4 => Some((Opcode::CPY, AddressingMode::ZeroPage, 2, 3)),
            0xCC => Some((Opcode::CPY, AddressingMode::Absolute, 3, 4)),

            // DEC (Decrement memory)
            0xC6 => Some((Opcode::DEC, AddressingMode::ZeroPage, 2, 5)),
            0xD6 => Some((Opcode::DEC, AddressingMode::ZeroPageX, 2, 6)),
            0xCE => Some((Opcode::DEC, AddressingMode::Absolute, 3, 6)),
            0xDE => Some((Opcode::DEC, AddressingMode::AbsoluteX, 3, 7)),

            // DEX (Decrement X register)
            0xCA => Some((Opcode::DEX, AddressingMode::Implied, 1, 2)),

            // DEY (Decrement Y register)
            0x88 => Some((Opcode::DEY, AddressingMode::Implied, 1, 2)),

            // EOR (Exclusive OR with accumulator)
            0x49 => Some((Opcode::EOR, AddressingMode::Immediate, 2, 2)),
            0x45 => Some((Opcode::EOR, AddressingMode::ZeroPage, 2, 3)),
            0x55 => Some((Opcode::EOR, AddressingMode::ZeroPageX, 2, 4)),
            0x4D => Some((Opcode::EOR, AddressingMode::Absolute, 3, 4)),
            0x5D => Some((Opcode::EOR, AddressingMode::AbsoluteX, 3, 4)),
            0x59 => Some((Opcode::EOR, AddressingMode::AbsoluteY, 3, 4)),
            0x41 => Some((Opcode::EOR, AddressingMode::IndirectX, 2, 6)),
            0x51 => Some((Opcode::EOR, AddressingMode::IndirectY, 2, 5)),

            // INC (Increment memory)
            0xE6 => Some((Opcode::INC, AddressingMode::ZeroPage, 2, 5)),
            0xF6 => Some((Opcode::INC, AddressingMode::ZeroPageX, 2, 6)),
            0xEE => Some((Opcode::INC, AddressingMode::Absolute, 3, 6)),
            0xFE => Some((Opcode::INC, AddressingMode::AbsoluteX, 3, 7)),

            // INX (Increment X register)
            0xE8 => Some((Opcode::INX, AddressingMode::Implied, 1, 2)),

            // INY (Increment Y register)
            0xC8 => Some((Opcode::INY, AddressingMode::Implied, 1, 2)),

            // JMP (Jump to address)
            0x4C => Some((Opcode::JMP, AddressingMode::Absolute, 3, 3)),
            0x6C => Some((Opcode::JMP, AddressingMode::Indirect, 3, 5)),

            // JSR (Jump to subroutine)
            0x20 => Some((Opcode::JSR, AddressingMode::Absolute, 3, 6)),

            // LDA (Load accumulator)
            0xA9 => Some((Opcode::LDA, AddressingMode::Immediate, 2, 2)),
            0xA5 => Some((Opcode::LDA, AddressingMode::ZeroPage, 2, 3)),
            0xB5 => Some((Opcode::LDA, AddressingMode::ZeroPageX, 2, 4)),
            0xAD => Some((Opcode::LDA, AddressingMode::Absolute, 3, 4)),
            0xBD => Some((Opcode::LDA, AddressingMode::AbsoluteX, 3, 4)),
            0xB9 => Some((Opcode::LDA, AddressingMode::AbsoluteY, 3, 4)),
            0xA1 => Some((Opcode::LDA, AddressingMode::IndirectX, 2, 6)),
            0xB1 => Some((Opcode::LDA, AddressingMode::IndirectY, 2, 5)),

            // LDX (Load X register)
            0xA2 => Some((Opcode::LDX, AddressingMode::Immediate, 2, 2)),
            0xA6 => Some((Opcode::LDX, AddressingMode::ZeroPage, 2, 3)),
            0xB6 => Some((Opcode::LDX, AddressingMode::ZeroPageY, 2, 4)),
            0xAE => Some((Opcode::LDX, AddressingMode::Absolute, 3, 4)),
            0xBE => Some((Opcode::LDX, AddressingMode::AbsoluteY, 3, 4)),

            // LDY (Load Y register)
            0xA0 => Some((Opcode::LDY, AddressingMode::Immediate, 2, 2)),
            0xA4 => Some((Opcode::LDY, AddressingMode::ZeroPage, 2, 3)),
            0xB4 => Some((Opcode::LDY, AddressingMode::ZeroPageX, 2, 4)),
            0xAC => Some((Opcode::LDY, AddressingMode::Absolute, 3, 4)),
            0xBC => Some((Opcode::LDY, AddressingMode::AbsoluteX, 3, 4)),

            // LSR (Logical shift right)
            0x4A => Some((Opcode::LSR, AddressingMode::Implied, 1, 2)),
            0x46 => Some((Opcode::LSR, AddressingMode::ZeroPage, 2, 5)),
            0x56 => Some((Opcode::LSR, AddressingMode::ZeroPageX, 2, 6)),
            0x4E => Some((Opcode::LSR, AddressingMode::Absolute, 3, 6)),
            0x5E => Some((Opcode::LSR, AddressingMode::AbsoluteX, 3, 7)),

            // NOP (No operation)
            0xEA => Some((Opcode::NOP, AddressingMode::Implied, 1, 2)),

            // ORA (OR with accumulator)
            0x09 => Some((Opcode::ORA, AddressingMode::Immediate, 2, 2)),
            0x05 => Some((Opcode::ORA, AddressingMode::ZeroPage, 2, 3)),
            0x15 => Some((Opcode::ORA, AddressingMode::ZeroPageX, 2, 4)),
            0x0D => Some((Opcode::ORA, AddressingMode::Absolute, 3, 4)),
            0x1D => Some((Opcode::ORA, AddressingMode::AbsoluteX, 3, 4)),
            0x19 => Some((Opcode::ORA, AddressingMode::AbsoluteY, 3, 4)),
            0x01 => Some((Opcode::ORA, AddressingMode::IndirectX, 2, 6)),
            0x11 => Some((Opcode::ORA, AddressingMode::IndirectY, 2, 5)),

            // PHA (Push accumulator)
            0x48 => Some((Opcode::PHA, AddressingMode::Implied, 1, 3)),

            // PHP (Push processor status)
            0x08 => Some((Opcode::PHP, AddressingMode::Implied, 1, 3)),

            // PLA (Pull accumulator)
            0x68 => Some((Opcode::PLA, AddressingMode::Implied, 1, 4)),

            // PLP (Pull processor status)
            0x28 => Some((Opcode::PLP, AddressingMode::Implied, 1, 4)),

            // ROL (Rotate left)
            0x2A => Some((Opcode::ROL, AddressingMode::Implied, 1, 2)),
            0x26 => Some((Opcode::ROL, AddressingMode::ZeroPage, 2, 5)),
            0x36 => Some((Opcode::ROL, AddressingMode::ZeroPageX, 2, 6)),
            0x2E => Some((Opcode::ROL, AddressingMode::Absolute, 3, 6)),
            0x3E => Some((Opcode::ROL, AddressingMode::AbsoluteX, 3, 7)),

            // ROR (Rotate right)
            0x6A => Some((Opcode::ROR, AddressingMode::Implied, 1, 2)),
            0x66 => Some((Opcode::ROR, AddressingMode::ZeroPage, 2, 5)),
            0x76 => Some((Opcode::ROR, AddressingMode::ZeroPageX, 2, 6)),
             0x6E => Some((Opcode::ROR, AddressingMode::Absolute, 3, 6)),
             0x7E => Some((Opcode::ROR, AddressingMode::AbsoluteX, 3, 7)),

            // RTI (Return from interrupt)
            0x40 => Some((Opcode::RTI, AddressingMode::Implied, 1, 6)),

            // RTS (Return from subroutine)
            0x60 => Some((Opcode::RTS, AddressingMode::Implied, 1, 6)),

            // SBC (Subtract with carry)
            0xE9 => Some((Opcode::SBC, AddressingMode::Immediate, 2, 2)),
            0xE5 => Some((Opcode::SBC, AddressingMode::ZeroPage, 2, 3)),
            0xF5 => Some((Opcode::SBC, AddressingMode::ZeroPageX, 2, 4)),
            0xED => Some((Opcode::SBC, AddressingMode::Absolute, 3, 4)),
            0xFD => Some((Opcode::SBC, AddressingMode::AbsoluteX, 3, 4)),
            0xF9 => Some((Opcode::SBC, AddressingMode::AbsoluteY, 3, 4)),
            0xE1 => Some((Opcode::SBC, AddressingMode::IndirectX, 2, 6)),
            0xF1 => Some((Opcode::SBC, AddressingMode::IndirectY, 2, 5)),

            // SEC (Set carry flag)
            0x38 => Some((Opcode::SEC, AddressingMode::Implied, 1, 2)),

            // SED (Set decimal mode)
            0xF8 => Some((Opcode::SED, AddressingMode::Implied, 1, 2)),

            // SEI (Set interrupt disable)
            0x78 => Some((Opcode::SEI, AddressingMode::Implied, 1, 2)),

            // STA (Store accumulator)
            0x85 => Some((Opcode::STA, AddressingMode::ZeroPage, 2, 3)),
            0x95 => Some((Opcode::STA, AddressingMode::ZeroPageX, 2, 4)),
            0x8D => Some((Opcode::STA, AddressingMode::Absolute, 3, 4)),
            0x9D => Some((Opcode::STA, AddressingMode::AbsoluteX, 3, 5)),
            0x99 => Some((Opcode::STA, AddressingMode::AbsoluteY, 3, 5)),
            0x81 => Some((Opcode::STA, AddressingMode::IndirectX, 2, 6)),
            0x91 => Some((Opcode::STA, AddressingMode::IndirectY, 2, 6)),

            // STX (Store X register)
            0x86 => Some((Opcode::STX, AddressingMode::ZeroPage, 2, 3)),
            0x96 => Some((Opcode::STX, AddressingMode::ZeroPageY, 2, 4)),
            0x8E => Some((Opcode::STX, AddressingMode::Absolute, 3, 4)),

            // STY (Store Y register)
            0x84 => Some((Opcode::STY, AddressingMode::ZeroPage, 2, 3)),
            0x94 => Some((Opcode::STY, AddressingMode::ZeroPageX, 2, 4)),
            0x8C => Some((Opcode::STY, AddressingMode::Absolute, 3, 4)),

            // TAX (Transfer accumulator to X)
            0xAA => Some((Opcode::TAX, AddressingMode::Implied, 1, 2)),

            // TAY (Transfer accumulator to Y)
            0xA8 => Some((Opcode::TAY, AddressingMode::Implied, 1, 2)),

            // TSX (Transfer stack pointer to X)
            0xBA => Some((Opcode::TSX, AddressingMode::Implied, 1, 2)),

            // TXA (Transfer X to accumulator)
            0x8A => Some((Opcode::TXA, AddressingMode::Implied, 1, 2)),

            // TXS (Transfer X to stack pointer)
            0x9A => Some((Opcode::TXS, AddressingMode::Implied, 1, 2)),

            // TYA (Transfer Y to accumulator)
            0x98 => Some((Opcode::TYA, AddressingMode::Implied, 1, 2)),
            _ => None,
        }
    }
}


fn main() {
    let _ = get_games();
}