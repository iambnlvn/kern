const kernel = @import("kernel.zig");
const kUtils = @import("kernelUtils.zig");
const Error = kernel.Error;
const Errors = kernel.Errors;
const AHCI = kernel.drivers.AHCI;

const Filesystem = kernel.FileSytem;

const AddressSpace = kernel.memory.AddressSpace;
const Region = kernel.memory.Region;

pub const ReadError = error{
    failed,
};

pub const LoadedExecutable = extern struct {
    startAddress: u64,
    tlsImageStart: u64,
    tlsImageByteCount: u64,
    tlsByteCount: u64,

    isDesktop: bool,
    isBundle: bool,
};

pub fn readFileInBuffer(fileBuffer: []u8, fileDescriptor: *const Filesystem.File.Descriptor) ReadError!void {
    try AHCI.driver.getDrive().readFile(fileBuffer, fileDescriptor);
}

pub fn readFileAlloc(space: *AddressSpace, fileDescriptor: *const Filesystem.File.Descriptor) ![]u8 {
    const drive = AHCI.Driver.getDrive();
    const sectorAlignedSize = kUtils.alignu64(fileDescriptor.size, drive.blockDevice.sectorSize);
    const fileBuffer = @as(?[*]u8, @ptrFromInt(space.alloc(sectorAlignedSize, Region.Flags.empty(), 0, true))) orelse return ReadError.failed;
    return try drive.readFile(fileBuffer, fileDescriptor);
}

var superblock: Filesystem.Superblock = undefined;

pub fn parseSuperblock() void {
    var superblockBuff: [0x200]u8 align(0x200) = undefined;
    readFileInBuffer(&superblockBuff, &Filesystem.Superblock.fileDescriptor) catch kernel.panic("Unable to read superblock\n");
    const superblockBufPtr = @as(*Filesystem.Superblock, @ptrFromInt(@intFromPtr(&superblockBuff)));
    superblock = superblockBufPtr.*;
}

const ELF = extern struct {
    const Header = extern struct {
        magic: u32,
        bits: u8,
        endianness: u8,
        version1: u8,
        abi: u8,
        unused: [8]u8,
        type_: u16,
        instructionSet: u16,
        version2: u32,
        entry: u64,
        programHeaderTable: u64,
        sectionHeaderTable: u64,
        flags: u32,
        headerSize: u16,
        programHeaderEntrySize: u16,
        programHeaderEntryCount: u16,
        sectionHeaderEntrySize: u16,
        sectionHeaderEntryCount: u16,
        sectionNameIndex: u16,
    };

    const ProgramHeader = extern struct {
        type: u32,
        flags: u32,
        fileOffset: u64,
        virtualAddr: u64,
        unused0: u64,
        dataInFile: u64,
        segmentSize: u64,
        alignment: u64,

        fn isBad(self: *@This()) bool {
            return self.virtualAddr >= 0xC0000000 or self.virtualAddr < 0x1000 or self.segmentSize > 0x10000000;
        }
    };
};
