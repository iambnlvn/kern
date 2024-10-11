const std = @import("std");

const ELF = struct {
    const Header = extern struct {
        magic: u32,
        bits: Bits,
        endianness: Endianness,
        version: u8,
        abi: ABI,
        _unused: [8]u8,
        objectType: ObjectType,
        instructionSet: InstructionSet,
        objectFileFmtVersion: u32,
        entryPointVA: u64,
        programHeaderTableOffset: u64,
        sectionHeaderTableOffset: u64,
        cpuSpecificFlags: u32,
        fileHeaderSize: u16,
        programHeaderSize: u16,
        programHeaderCount: u16,
        sectionHeaderSize: u16,
        sectionHeaderCount: u16,
        sectionHeaderStringTableIndex: u16,

        const Bits = enum(u8) {
            @"32",
            @"64",
        };

        const Endianness = enum(u8) {
            little = 1,
            big = 2,
        };

        const ABI = enum(u8) { SystemV };

        const ObjectType = enum(u16) {
            none = 0,
            relocatable,
            executable,
            shared,
            core,
        };

        const InstructionSet = enum(u16) {
            ARM = 0x28,
            X86_64 = 0x3e,
            X86 = 0x03,
            AARCH64 = 0xb7,
        };
    };
};
