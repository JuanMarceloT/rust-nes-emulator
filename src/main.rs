
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

    for entry in fs::read_dir(folder_path)? {
        let entry = entry?;
        let path = entry.path();

        if path.is_file() {
            println!("File: {}", path.display());
            match NesRom::load(path) {
                Ok(rom) => {
                    rom.print_info();
        
                    process_rom_with_bus(rom);
                }
                Err(e) => eprintln!("Erro ao carregar ROM: {}", e),
            }
        }
        
    }

    Ok(())
}


pub struct Bus {
    memory: [u8; 64 * 1024], 
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            memory: [0; 64 * 1024],
        }
    }

    pub fn read(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    pub fn write(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }

    pub fn load_prg_rom(&mut self, prg_rom: &[u8]) -> Result<(), &'static str> {
        if prg_rom.len() > 0x8000 {
            return Err("PRG-ROM size exceeds 32 KB limit!");
        }
        self.memory[0x8000..0x8000 + prg_rom.len()].copy_from_slice(prg_rom);
        Ok(())
    }
}

fn process_rom_with_bus(rom: NesRom) {
    let mut bus = Bus::new();

    if let Err(e) = bus.load_prg_rom(&rom.prg_rom) {
        eprintln!("Erro ao carregar PRG-ROM: {}", e);
        return;
    }

    let mut pc: u16 = 0x8000; // PC start in PRG-ROM
    let rom_end = 0x8000_usize + rom.prg_rom.len(); 
    
    while (pc as usize) < rom_end {
        let byte = bus.read(pc);
        match Opcode::from_binary(byte) {
            Some(opcode) => {
                println!("Binary  addressing_mode:'{:?}' corresponds o opcode {:?}", opcode.addressing_mode, opcode.opcode);
                for i in 1..opcode.bytes {
                    println!("{:?}", bus.read(pc + (i as u16)));
                }
                println!("{:?} in the addres, and the value is {:?}", opcode.effective_address(pc, &bus, 0 ,1), bus.read(opcode.effective_address(pc, &bus,0 ,1)));
                pc += opcode.bytes as u16;
            }
            None => {
                println!("No corresponding opcode for binary '{:08b}'", byte);
                pc += 1;
            }
        }
    }
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

struct Instruction {
    opcode: Opcode,
    addressing_mode: AddressingMode,
    bytes: u8,
    cycles: u8,
}


impl Instruction {
    fn effective_address(&self, pc: u16, bus : &Bus, x: u8, y: u8) -> u16 {
        match self.addressing_mode {
            AddressingMode::Immediate => {
                // For Immediate, the operand is the value itself
                pc + 1 as u16
            }
            AddressingMode::ZeroPage => {
                // ZeroPage is just the operand as an address (only low byte)
                let low_byte =  bus.read(pc + 1) as u16; 
                
                low_byte
            }
            AddressingMode::ZeroPageX => {
                // ZeroPageX: Add the X register value
                let low_byte =  bus.read(pc + 1) as u16; 
                low_byte + x as u16
            }
            AddressingMode::ZeroPageY => {
                // ZeroPageY: Add the Y register value
                let low_byte =  bus.read(pc + 1) as u16; 
                low_byte + y as u16
            }
            AddressingMode::Absolute => {
                // Absolute: Combine the two operand bytes (16-bit address)
                // NES use little-endian 
                let low_byte =  bus.read(pc + 1) as u16;  
                let high_byte = bus.read(pc + 2) as u16; 
                (high_byte << 8) | low_byte 
            }
            AddressingMode::AbsoluteX => {
               // AbsoluteX: Add the X register value to the address
                let low_byte =  bus.read(pc + 1) as u16;  
                let high_byte = bus.read(pc + 2) as u16; 
                (high_byte << 8) | low_byte + x as u16
            }
            AddressingMode::AbsoluteY => {
                // AbsoluteY: Add the Y register value to the address
                 let low_byte =  bus.read(pc + 1) as u16;  
                 let high_byte = bus.read(pc + 2) as u16; 
                 (high_byte << 8) | low_byte + y as u16
             }
            AddressingMode::Indirect => {
                // Indirect: Read the address from the operand (little-endian)
                let low_byte = bus.read(pc + 1) as u16;  
                let high_byte = bus.read(pc + 2) as u16;
                let pointer_address = (high_byte << 8) | low_byte; 

                let low_byte_target = bus.read(pointer_address) as u16;  
                let high_byte_target = bus.read(pointer_address + 1) as u16; 
                let final_address = (high_byte_target << 8) | low_byte_target; 

                final_address
            }
            AddressingMode::IndirectX => {
                // Indirect: Read the address from the operand (little-endian)
                let low_byte = bus.read(pc + 1) as u16;  
                let high_byte = bus.read(pc + 2) as u16;
                let pointer_address = (high_byte << 8) | low_byte; 

                let low_byte_target = bus.read(pointer_address) as u16;  
                let high_byte_target = bus.read(pointer_address + 1) as u16; 
                let final_address = (high_byte_target << 8) | low_byte_target; 

                final_address + x as u16
            }
            AddressingMode::IndirectY => {
                // IndirectY: Use Y register to index into memory
                let low_byte = bus.read(pc + 1) as u16;  
                let high_byte = bus.read(pc + 2) as u16;
                let pointer_address = (high_byte << 8) | low_byte; 

                let low_byte_target = bus.read(pointer_address) as u16;  
                let high_byte_target = bus.read(pointer_address + 1) as u16; 
                let final_address = (high_byte_target << 8) | low_byte_target; 

                final_address + y as u16
            }
            AddressingMode::Relative => {
                // Relative: Program counter is updated with a signed offset
                let offset = bus.read(pc + 1) as u16;
                pc + offset
            }
            AddressingMode::Implied => {
                // Implied: No address calculation needed
                0
            }
        }
    }
}

impl Opcode {
    fn from_binary(binary: u8) -> Option<Instruction> { // opcode, bytes and cycles
        match binary {
            // ADC (Add with carry)
            0x69 => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0x65 => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x75 => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0x6D => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0x7D => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0x79 => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0x61 => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0x71 => Some(Instruction { opcode: Opcode::ADC, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // AND (AND with accumulator)
            0x29 => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0x25 => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x35 => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0x2D => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0x3D => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0x39 => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0x21 => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0x31 => Some(Instruction { opcode: Opcode::AND, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // ASL (Arithmetic shift left)
            0x0A => Some(Instruction { opcode: Opcode::ASL, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),
            0x06 => Some(Instruction { opcode: Opcode::ASL, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 5}),
            0x16 => Some(Instruction { opcode: Opcode::ASL, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 6}),
            0x0E => Some(Instruction { opcode: Opcode::ASL, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),
            0x1E => Some(Instruction { opcode: Opcode::ASL, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 7}),

            // BCC (Branch on carry clear)
            0x90 => Some(Instruction { opcode: Opcode::BCC, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BCS (Branch on carry set)
            0xB0 => Some(Instruction { opcode: Opcode::BCS, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BEQ (Branch on equal)
            0xF0 => Some(Instruction { opcode: Opcode::BEQ, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BIT (Bit test)
            0x24 => Some(Instruction { opcode: Opcode::BIT, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x2C => Some(Instruction { opcode: Opcode::BIT, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),

            // BMI (Branch on minus)
            0x30 => Some(Instruction { opcode: Opcode::BMI, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BNE (Branch on not equal)
            0xD0 => Some(Instruction { opcode: Opcode::BNE, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BPL (Branch on plus)
            0x10 => Some(Instruction { opcode: Opcode::BPL, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BRK (Break)
            0x00 => Some(Instruction { opcode: Opcode::BRK, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 7}),

            // BVC (Branch on overflow clear)
            0x50 => Some(Instruction { opcode: Opcode::BVC, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // BVS (Branch on overflow set)
            0x70 => Some(Instruction { opcode: Opcode::BVS, addressing_mode: AddressingMode::Relative, bytes: 2, cycles: 2}),

            // CLC (Clear carry flag)
            0x18 => Some(Instruction { opcode: Opcode::CLC, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // CLD (Clear decimal mode)
            0xD8 => Some(Instruction { opcode: Opcode::CLD, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // CLI (Clear interrupt disable)
            0x58 => Some(Instruction { opcode: Opcode::CLI, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // CLV (Clear overflow flag)
            0xB8 => Some(Instruction { opcode: Opcode::CLV, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // CMP (Compare accumulator)
            0xC9 => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xC5 => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xD5 => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0xCD => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0xDD => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0xD9 => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0xC1 => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0xD1 => Some(Instruction { opcode: Opcode::CMP, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // CPX (Compare X register)
            0xE0 => Some(Instruction { opcode: Opcode::CPX, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xE4 => Some(Instruction { opcode: Opcode::CPX, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xEC => Some(Instruction { opcode: Opcode::CPX, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),

            // CPY (Compare Y register)
            0xC0 => Some(Instruction { opcode: Opcode::CPY, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xC4 => Some(Instruction { opcode: Opcode::CPY, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xCC => Some(Instruction { opcode: Opcode::CPY, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),

            // DEC (Decrement memory)
            0xC6 => Some(Instruction { opcode: Opcode::DEC, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 5}),
            0xD6 => Some(Instruction { opcode: Opcode::DEC, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 6}),
            0xCE => Some(Instruction { opcode: Opcode::DEC, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),
            0xDE => Some(Instruction { opcode: Opcode::DEC, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 7}),

            // DEX (Decrement X register)
            0xCA => Some(Instruction { opcode: Opcode::DEX, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // DEY (Decrement Y register)
            0x88 => Some(Instruction { opcode: Opcode::DEY, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // EOR (Exclusive OR with accumulator)
            0x49 => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0x45 => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x55 => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0x4D => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0x5D => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0x59 => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0x41 => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0x51 => Some(Instruction { opcode: Opcode::EOR, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // INC (Increment memory)
            0xE6 => Some(Instruction { opcode: Opcode::INC, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 5}),
            0xF6 => Some(Instruction { opcode: Opcode::INC, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 6}),
            0xEE => Some(Instruction { opcode: Opcode::INC, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),
            0xFE => Some(Instruction { opcode: Opcode::INC, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 7}),

            // INX (Increment X register)
            0xE8 => Some(Instruction { opcode: Opcode::INX, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // INY (Increment Y register)
            0xC8 => Some(Instruction { opcode: Opcode::INY, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // JMP (Jump to address)
            0x4C => Some(Instruction { opcode: Opcode::JMP, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 3}),
            0x6C => Some(Instruction { opcode: Opcode::JMP, addressing_mode: AddressingMode::Indirect, bytes: 3, cycles: 5}),

            // JSR (Jump to subroutine)
            0x20 => Some(Instruction { opcode: Opcode::JSR, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),

            // LDA (Load accumulator)
            0xA9 => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xA5 => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xB5 => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0xAD => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0xBD => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0xB9 => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0xA1 => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0xB1 => Some(Instruction { opcode: Opcode::LDA, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // LDX (Load X register)
            0xA2 => Some(Instruction { opcode: Opcode::LDX, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xA6 => Some(Instruction { opcode: Opcode::LDX, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xB6 => Some(Instruction { opcode: Opcode::LDX, addressing_mode: AddressingMode::ZeroPageY, bytes: 2, cycles: 4}),
            0xAE => Some(Instruction { opcode: Opcode::LDX, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0xBE => Some(Instruction { opcode: Opcode::LDX, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),

            // LDY (Load Y register)
            0xA0 => Some(Instruction { opcode: Opcode::LDY, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xA4 => Some(Instruction { opcode: Opcode::LDY, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xB4 => Some(Instruction { opcode: Opcode::LDY, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0xAC => Some(Instruction { opcode: Opcode::LDY, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0xBC => Some(Instruction { opcode: Opcode::LDY, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),

            // LSR (Logical shift right)
            0x4A => Some(Instruction { opcode: Opcode::LSR, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),
            0x46 => Some(Instruction { opcode: Opcode::LSR, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 5}),
            0x56 => Some(Instruction { opcode: Opcode::LSR, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 6}),
            0x4E => Some(Instruction { opcode: Opcode::LSR, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),
            0x5E => Some(Instruction { opcode: Opcode::LSR, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 7}),

            // NOP (No operation)
            0xEA => Some(Instruction { opcode: Opcode::NOP, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // ORA (OR with accumulator)
            0x09 => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0x05 => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x15 => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0x0D => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0x1D => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0x19 => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0x01 => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0x11 => Some(Instruction { opcode: Opcode::ORA, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // PHA (Push accumulator)
            0x48 => Some(Instruction { opcode: Opcode::PHA, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 3}),

            // PHP (Push processor status)
            0x08 => Some(Instruction { opcode: Opcode::PHP, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 3}),

            // PLA (Pull accumulator)
            0x68 => Some(Instruction { opcode: Opcode::PLA, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 4}),

            // PLP (Pull processor status)
            0x28 => Some(Instruction { opcode: Opcode::PLP, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 4}),

            // ROL (Rotate left)
            0x2A => Some(Instruction { opcode: Opcode::ROL, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),
            0x26 => Some(Instruction { opcode: Opcode::ROL, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 5}),
            0x36 => Some(Instruction { opcode: Opcode::ROL, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 6}),
            0x2E => Some(Instruction { opcode: Opcode::ROL, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),
            0x3E => Some(Instruction { opcode: Opcode::ROL, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 7}),

            // ROR (Rotate right)
            0x6A => Some(Instruction { opcode: Opcode::ROR, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),
            0x66 => Some(Instruction { opcode: Opcode::ROR, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 5}),
            0x76 => Some(Instruction { opcode: Opcode::ROR, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 6}),
             0x6E => Some(Instruction { opcode: Opcode::ROR, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 6}),
             0x7E => Some(Instruction { opcode: Opcode::ROR, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 7}),

            // RTI (Return from interrupt)
            0x40 => Some(Instruction { opcode: Opcode::RTI, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 6}),

            // RTS (Return from subroutine)
            0x60 => Some(Instruction { opcode: Opcode::RTS, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 6}),

            // SBC (Subtract with carry)
            0xE9 => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::Immediate, bytes: 2, cycles: 2}),
            0xE5 => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0xF5 => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0xED => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0xFD => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 4}),
            0xF9 => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 4}),
            0xE1 => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0xF1 => Some(Instruction { opcode: Opcode::SBC, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 5}),

            // SEC (Set carry flag)
            0x38 => Some(Instruction { opcode: Opcode::SEC, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // SED (Set decimal mode)
            0xF8 => Some(Instruction { opcode: Opcode::SED, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // SEI (Set interrupt disable)
            0x78 => Some(Instruction { opcode: Opcode::SEI, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // STA (Store accumulator)
            0x85 => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x95 => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0x8D => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),
            0x9D => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::AbsoluteX, bytes: 3, cycles: 5}),
            0x99 => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::AbsoluteY, bytes: 3, cycles: 5}),
            0x81 => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::IndirectX, bytes: 2, cycles: 6}),
            0x91 => Some(Instruction { opcode: Opcode::STA, addressing_mode: AddressingMode::IndirectY, bytes: 2, cycles: 6}),

            // STX (Store X register)
            0x86 => Some(Instruction { opcode: Opcode::STX, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x96 => Some(Instruction { opcode: Opcode::STX, addressing_mode: AddressingMode::ZeroPageY, bytes: 2, cycles: 4}),
            0x8E => Some(Instruction { opcode: Opcode::STX, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),

            // STY (Store Y register)
            0x84 => Some(Instruction { opcode: Opcode::STY, addressing_mode: AddressingMode::ZeroPage, bytes: 2, cycles: 3}),
            0x94 => Some(Instruction { opcode: Opcode::STY, addressing_mode: AddressingMode::ZeroPageX, bytes: 2, cycles: 4}),
            0x8C => Some(Instruction { opcode: Opcode::STY, addressing_mode: AddressingMode::Absolute, bytes: 3, cycles: 4}),

            // TAX (Transfer accumulator to X)
            0xAA => Some(Instruction { opcode: Opcode::TAX, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // TAY (Transfer accumulator to Y)
            0xA8 => Some(Instruction { opcode: Opcode::TAY, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // TSX (Transfer stack pointer to X)
            0xBA => Some(Instruction { opcode: Opcode::TSX, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // TXA (Transfer X to accumulator)
            0x8A => Some(Instruction { opcode: Opcode::TXA, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // TXS (Transfer X to stack pointer)
            0x9A => Some(Instruction { opcode: Opcode::TXS, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),

            // TYA (Transfer Y to accumulator)
            0x98 => Some(Instruction { opcode: Opcode::TYA, addressing_mode: AddressingMode::Implied, bytes: 1, cycles: 2}),
            _ => None,
        }
    }
}


fn main() {
    let _ = get_games();
}