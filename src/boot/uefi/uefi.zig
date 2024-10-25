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

fn isInRange(ph: *ELF.ProgramHeader) bool {
    const va = ph.virtual_address;
    const size = ph.size_in_memory;
    return va <= faultAddr and va + size >= faultAddr;
}

fn calculateIndex(vA: u64, level: u64, _pageBitCount: u64, _entryPerPageTableBitCount: u64) u64 {
    return (vA >> (_pageBitCount + entryPerPageTableBitCount * level)) & (_entryPerPageTableBitCount - 1);
}

fn initializeTable(table: [*]u64, idx: usize, nextPageTable: *u64, _pageSize: usize) *u64 {
    if (table[idx] & 1 == 0) {
        table[idx] = nextPageTable | 0xb111;
        @memset(@as([*]u8, nextPageTable[0.._pageSize]), 0);
        nextPageTable += pageSize;
    }
    return @as([*]u64, table[idx] & ~@as(u64, _pageSize - 1));
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
const VideoInfo = extern struct {
    frameBufferBase: *u8,
    horizontalResolution: u32,
    verticalResolution: u32,
    pixelsPerScanLine: u32,
    pixelFormat: u32,
};
const VideoModeInfo = extern struct {
    width: u16,
    height: u16,
    bytesPerScanLine: u16,
    bitesPerPixel: u8,
    pyhsicalBufferAddress: u64,
    validEdid: u8,
    ediData: [128]u8,
};

const MemoryRegion = extern struct {
    baseAddress: u64,
    regionSize: u64,
};
var memRegions: [1024]MemoryRegion = undefined;

var bootServices: *uefi.tables.BootServices = undefined;
const pageSize = 0x1000;
const uefiAsmAddress = 0x180000;
const memMapBuffer: [0x4000]u8 = undefined;
const pageBitCount = 12;
const faultAddr = 0xffffffff800d9000;
const entryPerPageTableBitCount = 9;
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

    const videoInfo = blk: {
        var gop: *uefi.protocol.GraphicsOutput = undefined;
        const gopGuid = uefi.protocol.GraphicsOutput.guid;
        checkEfiStatus(bootServices.locateProtocol(@as(*align(8) const uefi.Guid, @ptrCast(&gopGuid)), null, @as(*?*anyopaque, &gop)), "Failed to locate GOP protocol");
        break :blk VideoInfo{
            .frameBufferBase = gop.mode.frame_buffer_base,
            .horizontalResolution = gop.mode.info.horizontal_resolution,
            .verticalResolution = gop.mode.info.vertical_resolution,
            .pixelsPerScanLine = gop.mode.info.pixels_per_scan_line,
            .pixelFormat = gop.mode.info.pixel_format,
        };
    };
    var mapKey: usize = 0;
    var memoryMapSize: usize = memMapBuffer.len;
    var descriptorSize: usize = 0;
    var descriptorVersion: u32 = 0;

    checkEfiStatus(bootServices.getMemoryMap(&memoryMapSize, @as(
        [*]uefi.tables.MemoryDescriptor,
        @intFromPtr(&memMapBuffer),
    ), &mapKey, &descriptorSize, &descriptorVersion), "Failed to get memory map");

    if (memoryMapSize == 0 or descriptorSize == 0) {
        panic("Invalid memory map size or descriptor size", .{});
    }
    var memRegionCount: u64 = 0;
    const maxCount = memoryMapSize / descriptorSize;

    var regionIter: u64 = 0;

    while (regionIter < maxCount and memRegionCount != memRegions.len - 1) {
        const descriptor = @as(*uefi.tables.MemoryDescriptor, @ptrFromInt(@intFromPtr(&memMapBuffer) + regionIter * descriptorSize));

        if (descriptor.type == .ConventionalMemory and descriptor.physical_start >= 0x300000) {
            memRegions[memRegionCount] = MemoryRegion{
                .baseAddress = descriptor.physical_start,
                .regionSize = descriptor.number_of_pages * pageSize,
            };
            memRegionCount += 1;
        }
        memRegions[memRegionCount].baseAddress = 0;
        regionIter += 1;
    }
    checkEfiStatus(bootServices.exitBootServices(uefi.handle, mapKey), "Failed to exit boot services\n");
    var paging = @as([*]u8, @ptrFromInt(0x140000));
    @memset(@as([*]u8, @ptrCast(paging)), 0x5000);
    paging[0x1FE] = 0x140003;
    paging[0x000] = 0x141003;
    paging[0x200] = 0x142003;
    paging[0x400] = 0x143003;
    paging[0x401] = 0x144003;
    for (paging[0x600 .. 0x600 + 0x400], 0..) |*ptr, idx| {
        ptr.* = (idx * pageSize) | 0x3;
    }

    var dest = @as([*]u8, @ptrFromInt(0x107ff0))[0..16];
    @memset(dest, 0);

    var video = @as(*VideoModeInfo, @ptrFromInt(0x107000));
    video.width = @as(u16, videoInfo.horizontalResolution);
    video.height = @as(u16, videoInfo.verticalResolution);
    video.bytesPerScanLine = @as(u16, videoInfo.pixelsPerScanLine * @sizeOf(u32));
    video.bitesPerPixel = 32;
    video.pyhsicalBufferAddress = @as(u64, videoInfo.frameBufferBase);
    video.validEdid = (0 << 1) | (1 << 0);

    var nextPageTable: u64 = 0x1c0000;
    const elfHeader = @as(*ELF.Header, @ptrFromInt(kernelAddress));

    const programHeaders = @as([*]ELF.ProgramHeader, @ptrFromInt(kernelAddress + elfHeader.programHeaderTableOffset))[0..elfHeader.programHeaderCount];

    var bool = false;

    for (programHeaders) |*progHeader| {
        if (progHeader.type == .load) continue;
        if (progHeader.vaddr & 0xfff != 0) panic("Invalid vaddr alignment", .{});

        const page2AllocCount = (progHeader.memsz >> pageBitCount) + @intFromBool(progHeader.memsz & 0xfff != 0) + @intFromBool(progHeader.vaddr & 0xfff != 0);

        var physicalAddr = blk: {
            for (memRegions) |*region| {
                if (region.baseAddress == 0) break;
                if (region.regionSize >= page2AllocCount) {
                    const res = region.baseAddress;
                    region.regionSize -= page2AllocCount;
                    region.baseAddress += page2AllocCount << 12;
                    break :blk res & 0xFFFFFFFFFFFFF000;
                }
            }
            @ptrFromInt(videoInfo.frameBufferBase + @sizeOf(u32)).* = 0x00000000;
            panic("Failed to allocate memory", .{});
        };

        var pageIdx: u64 = 0;
        while (pageIdx < page2AllocCount) : ({
            pageIdx += 1;
            physicalAddr += pageSize;
        }) {
            const base = (progHeader.vaddr + (pageIdx * pageSize));
            if (isInRange(progHeader)) {
                if (base >= faultAddr and base - faultAddr < 0x1000) {
                    bool = true;
                }
            }

            const vA = base & 0x0000FFFFFFFFF000;
            const l4Idx: u64 = calculateIndex(vA, 3, pageBitCount, entryPerPageTableBitCount);
            const l3Idx: u64 = calculateIndex(vA, 2, pageBitCount, entryPerPageTableBitCount);
            const l2Idx: u64 = calculateIndex(vA, 1, pageBitCount, entryPerPageTableBitCount);
            const l1Idx: u64 = calculateIndex(vA, 0, pageBitCount, entryPerPageTableBitCount);

            //TOdo?: which one should I keep
            // var l4Table = @as([*]u64, @ptrFromInt(0x140000));

            // if (l4Table[l4Idx] & 1 == 0) {
            //     l4Table[l4Idx] = nextPageTable | 0xb111;
            //     @memset(@as([*]u8, nextPageTable[0..pageSize]), 0);
            //     nextPageTable += pageSize;
            // }

            // var l3Table = @as([*]u64, l4Table[l4Idx] & ~@as(u64, pageSize - 1));
            // if (l3Table[l3Idx] & 1 == 0) {
            //     l4Table[l3Idx] = nextPageTable | 0xb111;
            //     @memset(@as([*]u8, nextPageTable[0..pageSize]), 0);
            //     nextPageTable += pageSize;
            // }

            // var l2Table = @as([*]u64, l3Table[l3Idx] & ~@as(u64, pageSize - 1));
            // if (l2Table[l2Idx] & 1 == 0) {
            //     l2Table[l2Idx] = nextPageTable | 0xb111;
            //     @memset(@as([*]u8, nextPageTable[0..pageSize]), 0);
            //     nextPageTable += pageSize;
            // }
            // var l1Table = @as([*]u64, l2Table[l2Idx] & ~@as(u64, pageSize - 1));
            // l1Table[l1Idx] = physicalAddr | 0xb111;
            var l4Table = @as([*]u64, @ptrFromInt(0x140000));
            var l3Table = initializeTable(l4Table, l4Idx, nextPageTable, pageSize);
            var l2Table = initializeTable(l3Table, l3Idx, nextPageTable, pageSize);
            var l1Table = initializeTable(l2Table, l2Idx, nextPageTable, pageSize);
            l1Table[l1Idx] = physicalAddr | 0xb111;
        }
    }
}
