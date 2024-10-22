const std = @import("std");
const uefi = std.os.uefi;
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
    const ProgramHeader = extern struct {
        type: PHType,
        flags: u32,
        offset: u64,
        vaddr: u64,
        reserved: u64,
        filesz: u64,
        memsz: u64,
        alignment: u64,

        const PHType = enum(u32) {
            unused = 0,
            load, // loadable segment
            dynamic, // dynamic linking information
            interp,
            note, // auxiliary information
            shlib,
            phdr,
            tls, // thread local storage
            gnu_eh_frame, // exception handling frame
            gnu_stack, // stack segment
            nu_relro, //read-only after relocation
        };

        const Flags = struct {
            const executable: u32 = 1 << 0;
            const writable: u32 = 1 << 1;
            const readable: u32 = 1 << 2;
        };
    };

    const SectionHeader = extern struct {
        type: Type,
        sectionNameOffset: u32,
        flags: u64,
        va: u64,
        offset: u64,
        size: u64,
        link: u32,
        info: u32,
        alignment: u64,
        entrySize: u64,

        const Type = enum(u32) {
            null = 0, // Marks an unused section header
            progbits, // Contains information defined by the program
            symtab, // Contains a linker symbol table
            strtab, // Contains a string table
            rela, // Contains "Rela" type relocation entries
            hash, // Contains a symbol hash table
            dynamic, // Contains dynamic linking tables
            note, // Contains note information
            nobits, // Contains uninitialized space; does not occupy any space in the file
            rel, // Contains "Rel" type relocation entries
            reserved, // Reserved
            dynsym, // Contains a dynamic loader symbol table
            init_array, // Contains an array of constructors
            fini_array, // Contains an array of destructors
            preinit_array, // Contains an array of pre-constructors
            group, // Contains a section group
            symtab_shndx, // Contains extended section indices
            num, // Number of defined types
        };

        const Flags = struct {
            const writable: u64 = 1 << 0;
            const alloc: u64 = 1 << 1;
            const execinstr: u64 = 1 << 2;
        };
    };
};

const StdOut = struct {
    const Self = @This();
    protocol: *uefi.protocol.SimpleTextOutput,

    fn write(self: *Self, msg: []const u8) void {
        for (msg) |c| {
            const u16Char = [2]u16{ c, 0 };
            _ = self.protocol.outputString(&u16Char);
        }
    }
};
var stdOutFmtBuffer: [0x1000]u8 = undefined;
var stdOut = StdOut{ .protocol = undefined };

fn print(comptime fmt: []const u8, args: anytype) void {
    const formattedStr = std.fmt.bufPrint(stdOutFmtBuffer[0..], fmt, args) catch unreachable;
    StdOut.write(formattedStr);
}

fn panic(comptime fmt: []const u8, args: anytype) void {
    print("\r\nPANIC:\r\n\r\n" ++ fmt, args);
    haltCpu();
}

inline fn haltCpu() noreturn {
    @setCold(true);
    @setRuntimeSafety(false);
    while (true) {
        asm volatile (
            \\.intel_syntax noprefix
            \\cli
            \\hlt
        );
    }
    unreachable;
}

fn checkEfiStatus(status: uefi.Status, comptime errMessage: []const u8) void {
    if (status != .Success) {
        panic("EFI ERROR: {}. :" ++ errMessage, .{status});
    }
}

// Global Descriptor Table (GDT) structure
const GDT = extern struct {
    nullEntry: u64, // Null descriptor, not used. Required as the first entry.
    codeEntry: u64, // Code segment descriptor for 32-bit code.
    dataEntry: u64, // Data segment descriptor for 32-bit data.
    codeEntry16: u64, // Code segment descriptor for 16-bit code.
    dataEntry16: u64, // Data segment descriptor for 16-bit data.
    userCodeEntry: u64, // User-mode code segment descriptor for 32-bit code.
    userDataEntry: u64, // User-mode data segment descriptor for 32-bit data.
    taskStateSegment1: u64, // First part of the Task State Segment (TSS) descriptor.
    taskStateSegment2: u64, // Second part of the Task State Segment (TSS) descriptor.
    codeEntry64: u64, // Code segment descriptor for 64-bit code.
    dataEntry64: u64, // Data segment descriptor for 64-bit data.
    userCodeEntry64: u64, // User-mode code segment descriptor for 64-bit code.
    userCodeEntry64c: u64, // Another user-mode code segment descriptor for 64-bit cod
    userDataEntry64: u64, // User-mode data segment descriptor for 64-bit data.

    const Entry = packed struct {
        // limitLow: u16,
        // baseLow: u16,
        // baseMid: u8,
        // access: u8,
        // granularity: u8,
        // baseHigh: u8,
        const Descriptor = packed struct {
            limit: u16,
            base: u64,
        };
    };
};
var bootServices: *uefi.tables.BootServices = undefined;
const pageSize = 0x1000;
const uefiAsmAddress = 0x180000;
pub fn main() noreturn {
    stdOut = .{ .protocol = uefi.system_table.con_out.? };
    bootServices = uefi.system_table.boot_services;
    _ = stdOut.protocol.clearScreen();

    stdOut.write("Hello, UEFI!\r\n");

    const baseAddress = 0x100000;
    const kernelAddress = 0x200000;
    const pageCount = 0x200;
    var address = @as([*]align(pageSize) u8, @ptrFromInt(baseAddress));
    checkEfiStatus(bootServices.allocatePages(.AllocateAddress, .LoaderData, pageCount, &address), "Unable to allocate pages");

    const RSDPEntryVendorTable: *anyopaque = blk: {
        const confEntries = uefi.system_table.configuration_table[0..uefi.system_table.number_of_table_entries];
        for (confEntries) |entry| {
            if (entry.vendor_guid == uefi.tables.ConfigurationTable.acpi_20_table_guid) {
                break :blk entry.vendor_table;
            }
        }
        panic("ACPI 2.0 table not found", .{});
    };
    _ = RSDPEntryVendorTable;

    const loadedImgProto = blk: {
        const loadedImgProtoGuid = uefi.protocol.LoadedImage.guid;
        var ptr: ?*anyopaque = undefined;
        checkEfiStatus(bootServices.openProtocol(uefi.handle, @as(*align(8) const uefi.Guid, @ptrCast(&loadedImgProtoGuid)), &ptr, uefi.handle, null, .{ .get_protocol = true }), "Failed to open protocol");
        break :blk @as(*align(1) uefi.protocols.LoadedImageProtocol, @ptrCast(ptr.?));
    };

    const simpleFileSysProto = blk: {
        const simpleFsProtoGuid = uefi.protocol.SimpleFileSystem.guid;
        var ptr: ?*uefi.protocols.SimpleFileSystemProtocol = undefined;
        checkEfiStatus(bootServices.openProtocol(loadedImgProto.device_handle.?, @as(*align(8) const uefi.Guid, @ptrCast(&simpleFsProtoGuid)), @as(*?*anyopaque, @ptrCast(&ptr)), uefi.handle, null, .{ .get_protocol = true }), "Failed to open protocol");
        break :blk ptr.?;
    };

    const fsRoot = blk: {
        var fileProto: *const uefi.protocol.File = undefined;
        checkEfiStatus(simpleFileSysProto.openVolume(&fileProto), "Failed to open ESP volume");
        break :blk fileProto;
    };

    const kernelFile = blk: {
        var file: *uefi.protocol.File = undefined;
        checkEfiStatus(fsRoot.open(&file, std.unicode.utf8ToUtf16LeStringLiteral("Kernel.elf"), uefi.protocol.File.efi_file_mode_read, 0), "Failed to open kernel file");
        break :blk file;
    };
    const maxKernelSize = pageCount * pageSize + (kernelAddress - baseAddress);
    var kernelSize: usize = maxKernelSize;
    checkEfiStatus(kernelFile.read(&kernelSize, @as([*]u8, kernelAddress)), "Failed to read kernel file");

    if (kernelSize >= maxKernelSize) {
        panic("Kernel file is too large\n Max size:{any} Kernel Size:{any}", .{ maxKernelSize, kernelSize });
    }

    const uefiAsmFile = blk: {
        var file: *uefi.protocol.File = undefined;
        checkEfiStatus(fsRoot.open(&file, std.unicode.utf8ToUtf16LeStringLiteral("uefi_asm.bin"), uefi.protocol.File.efi_file_mode_read, 0), "Failed to open UEFI ASM file");
        break :blk file;
    };
    var size: usize = 0x80000;
    checkEfiStatus(uefiAsmFile.read(&size, @as([*]u8, uefiAsmAddress)), "Failed to read UEFI ASM file");
}
