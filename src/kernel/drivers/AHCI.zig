const std = @import("std");
const kernel = @import("../kernel.zig");
const PCI = kernel.drivers.PCI;
const Mutex = kernel.sync.Mutex;
const arch = @import("../arch/x86_64.zig");

const DMASegment = extern struct {
    physicalAddr: u64,
    byteCount: u64,
    isLast: bool,
};

const DMABuffer = extern struct {
    virtualAddr: u64,
    totalByteCount: u64,
    offset: u64,

    fn isComplete(self: *@This()) bool {
        return self.offset == self.totalByteCount;
    }

    fn nextSegment(self: *@This(), peek: bool) DMASegment {
        if (self.offset >= self.totalByteCount or self.virtualAddr == 0) kernel.panic("Invalid state of DMA buffer");

        var transferByteCount: u64 = arch.pageSize;
        const va = self.virtualAddr + self.offset;
        var physicalAddr = arch.translateAddr(va, false);
        const offsetIntoPage = va & (arch.pageSize - 1);

        if (offsetIntoPage > 0) {
            transferByteCount = arch.pageSize - offsetIntoPage;
            physicalAddr += offsetIntoPage;
        }

        const remainingBytes = self.totalByteCount - self.offset;
        transferByteCount = @min(transferByteCount, remainingBytes);

        const isLast = self.offset + transferByteCount == self.totalByteCount;
        if (!peek) self.offset += transferByteCount;

        return DMASegment{
            .physicalAddr = physicalAddr,
            .byteCount = transferByteCount,
            .isLast = isLast,
        };
    }
};

const MBR = extern struct {
    const Partition = extern struct {
        offset: u32,
        count: u32,
        present: bool,
    };
};

pub const Driver = struct {
    drives: []Drive,
    mbrPartitions: [4]MBR.Partition,
    mbrPartitionsCount: u64,
    partitionDevices: []PartitionDevice,
    capabilities: u32,
    cap2: u32,
    commandSlotCount: u64,
    timeoutTimer: kernel.scheduling.Timer,
    isDm64Supported: bool,
    ports: [MAX_PORT_COUNT]Port,

    const Port = extern struct {
        connected: bool,
        atapi: bool,
        ssd: bool,

        cmdList: [*]u32,
        cmdTables: [*]u8,
        sectorByteCount: u64,
        sectorCount: u64,

        cmdStartTimestamps: [32]u64,
        runningCmds: u32,

        cmdSpinlock: kernel.sync.SpinLock,
        cmdSlotsAvailableEvent: kernel.sync.Event,

        model: [41]u8,
    };

    const MAX_PORT_COUNT = 32;
};

// TODO!: Implement this
const PartitionDevice = extern struct {
    sectorOffset: u64,
};

pub const Drive = extern struct {
    blockDevice: BlockDevice,
    port: u64,

    const maxCount = 64;
};

const DriveType = enum(u8) {
    other = 0,
    hdd = 1,
    ssd = 2,
    cdrom = 3,
    usbStorage = 4,
};

const BlockDevice = extern struct {
    access: u64,
    sectorSize: u64,
    sectorCount: u64,
    readOnly: bool,
    nestLevel: u8,
    driveType: DriveType,
    modelBytes: u8,
    model: [64]u8,
    maxAccessSectorCount: u64,
    signatureBlock: [*]u8,
    detectFilesystemMutex: Mutex,

    const read = 0;
    const write = 1;
};
