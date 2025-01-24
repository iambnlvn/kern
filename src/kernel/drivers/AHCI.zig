const std = @import("std");
const kernel = @import("../kernel.zig");
const PCI = kernel.drivers.PCI;
const Mutex = kernel.sync.Mutex;
const arch = @import("../arch/x86_64.zig");
const kUtils = @import("../kernelUtils.zig");

const PRDT_ENTRY_COUNT = 0x48;
const CMD_TABLE_SIZE = 0x80 + PRDT_ENTRY_COUNT * 0x10;

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
fn GlobalRegister(comptime offset: u32) type {
    return struct {
        fn write(d: *Driver, value: u32) void {
            d.pci.writeBaseAddrReg32(5, offset, value);
        }

        fn read(d: *Driver) u32 {
            return d.pci.readBaseAddrReg(5, offset);
        }
    };
}

fn PortRegister(comptime offset: u32) type {
    return struct {
        fn write(d: *Driver, port: u32, value: u32) void {
            d.pci.writeBaseAddrReg32(5, offset + port * 0x80, value);
        }

        fn read(d: *Driver, port: u32) u32 {
            return d.pci.readBaseAddrReg(5, offset + port * 0x80);
        }
    };
}
const PTFD = PortRegister(0x120);
const PCIRegister = PortRegister(0x138);
const IS = GlobalRegister(8);
const PIS = PortRegister(0x110);
const MBR = extern struct {
    const Partition = extern struct {
        offset: u32,
        count: u32,
        present: bool,
    };
    fn getPartitions(firstBlk: [*]u8, sectorCount: u64) bool {
        const isMagicBootOk = firstBlk[510] == 0x55 and firstBlk[511] == 0xaa;
        if (!isMagicBootOk) return false;

        const partitionEntrySize = 16;
        const partitionTableOffset = 0x1be;

        for (driver.mbrPartitions, 0..) |*mbrPartition, i| {
            const entryOffset = partitionTableOffset + i * partitionEntrySize;

            if (firstBlk[entryOffset + 4] == 0) {
                mbrPartition.present = false;
                continue;
            }

            mbrPartition.offset = (@as(u32, @intCast(firstBlk[entryOffset + 8])) << 0) +
                (@as(u32, @intCast(firstBlk[entryOffset + 9])) << 8) +
                (@as(u32, @intCast(firstBlk[entryOffset + 10])) << 16) +
                (@as(u32, @intCast(firstBlk[entryOffset + 11])) << 24);
            mbrPartition.count = (@as(u32, @intCast(firstBlk[entryOffset + 12])) << 0) +
                (@as(u32, @intCast(firstBlk[entryOffset + 13])) << 8) +
                (@as(u32, @intCast(firstBlk[entryOffset + 14])) << 16) +
                (@as(u32, @intCast(firstBlk[entryOffset + 15])) << 24);
            mbrPartition.present = true;

            if (mbrPartition.offset > sectorCount or mbrPartition.count > sectorCount - mbrPartition.offset or mbrPartition.count < 32) {
                return false;
            }
            driver.mbrPartitionsCount += 1;
        }

        return true;
    }
};
pub var driver: *Driver = undefined;

pub const Driver = struct {
    drives: []Drive,
    mbrPartitions: [4]MBR.Partition,
    mbrPartitionsCount: u64,
    partitionDevices: []PartitionDevice,
    capabilities: u32,
    cap2: u32,
    pci: PCI.Device,
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
        cmdCTXs: [32]?*kernel.Workgroup,
        sectorByteCount: u64,
        sectorCount: u64,

        cmdStartTimestamps: [32]u64,
        runningCmds: u32,

        cmdSpinlock: kernel.sync.SpinLock,
        cmdSlotsAvailableEvent: kernel.sync.Event,

        model: [41]u8,
    };

    const MAX_PORT_COUNT = 32;

    pub fn init() void {
        const driveOffset = kUtils.roundUp(u64, @sizeOf(Driver), @alignOf(Drive));
        const partitionDevicesOffset = kUtils.roundUp(u64, driveOffset + (@sizeOf(Drive) * Drive.maxCount), @alignOf(PartitionDevice));
        const allocSize = partitionDevicesOffset + (@sizeOf(PartitionDevice) * PartitionDevice.maxCount);
        const address = kernel.addrSpace.alloc(allocSize, kernel.memory.Region.Flags.fromFlag(.fixed), arch.modulePtr, true);
        if (address == 0) kernel.panic("Could not allocate memory for PCI driver");
        arch.modulePtr += kUtils.roundUp(u64, allocSize, arch.pageSize);
        driver = @as(*Driver, @ptrFromInt(address));
        driver.drives.ptr = @as([*]Drive, @ptrFromInt(address + driveOffset));
        driver.drives.len = 0;
        driver.partitionDevices.ptr = @as([*]PartitionDevice, @ptrFromInt(address + partitionDevicesOffset));
        driver.partitionDevices.len = 0;
        //TODO!: implement a setup
    }

    pub fn sendSingleCmd(self: *@This(), port: u32) bool {
        const timeout = kernel.Timeout.new(GENERAL_TIMEOUT);

        while (PTFD.read(self, port) & ((1 << 7) | (1 << 3)) != 0 and !timeout.hit()) {}
        if (timeout.hit()) return false;
        @fence(.SeqCst);
        PCIRegister.write(self, port, 1 << 0);
        var isComplete = false;

        while (!timeout.hit()) {
            isComplete = ~PCIRegister.read(self, port) & (1 << 0) != 0;
            if (isComplete) {
                break;
            }
        }

        return isComplete;
    }
    pub fn handleIRQ(self: *@This()) bool {
        const globalInterruptStatus = IS.read(self);
        if (globalInterruptStatus == 0) return false;
        IS.write(self, globalInterruptStatus);

        const event = &recentInterruptEvents[recentInterruptEventsPtr.readVolatile()];
        event.accessVolatile().timestamp = kernel.scheduler.timeMs;
        event.accessVolatile().globalInterruptStatus = globalInterruptStatus;
        event.accessVolatile().complete = false;
        recentInterruptEventsPtr.writeVolatile((recentInterruptEventsPtr.readVolatile() + 1) % recentInterruptEvents.len);

        var isCmdCompleted = false;

        for (self.ports, 0..) |*port, portIdx| {
            const portIndex = @as(u32, @intCast(portIdx));
            if (~globalInterruptStatus & (@as(u32, 1) << @as(u5, @intCast(portIndex))) != 0) continue;

            const interruptStatus = PIS.read(self, portIndex);
            if (interruptStatus == 0) continue;

            PIS.write(self, portIndex, interruptStatus);

            if (interruptStatus & ((1 << 30 | (1 << 29) | (1 << 28) | (1 << 27) | (1 << 26) | (1 << 24) | (1 << 23))) != 0) {}
            port.cmdSpinlock.acquire();
            const issuedCmd = PCIRegister.read(self, portIndex);

            if (portIndex == 0) {
                event.accessVolatile().port0IssuedCmds = issuedCmd;
                event.accessVolatile().port0RunningCmds = port.runningCmds;
            }

            var i: u32 = 0;
            while (i < self.ports.len) : (i += 1) {
                const shifter = (@as(u32, 1) << @as(u5, @intCast(i)));
                if (~port.runningCmds & shifter != 0) continue;
                if (issuedCmd & shifter != 0) continue;

                port.cmdCTXs[i].?.end(true);
                port.cmdCTXs[i] = null;
                _ = port.cmdSlotsAvailableEvent.set(true);
                port.runningCmds &= ~shifter;

                isCmdCompleted = true;
            }

            port.cmdSpinlock.release();
        }

        if (isCmdCompleted) {
            arch.getLocalStorage().?.IRQSwitchCtx = true;
        }

        event.accessVolatile().complete = true;
        return true;
    }

    pub fn getDrive(self: *@This()) *Drive {
        return &self.drives[0];
    }
    pub fn accessCallback(request: BlockDevice.AccessRequest) kernel.Error {
        const drive = @as(*Drive, @ptrCast(request.device));
        request.dispatchGroup.?.start();

        if (!driver.access(drive.port, request.offset, request.count, request.op, request.buffer, request.flags, request.dispatchGroup)) {
            request.dispatchGroup.?.end(false);
        }

        return kernel.ES_SUCCESS;
    }
    pub fn access(self: *@This(), _portIdx: u64, offset: u64, byteCount: u64, op: i32, buffer: *DMABuffer, flags: BlockDevice.AccessRequest.Flags, dispatchGroup: ?*kernel.Workgroup) bool {
        _ = flags;
        const portIndex = @as(u32, @intCast(_portIdx));
        const port = &self.ports[portIndex];

        var cmdIdx: u64 = 0;

        while (true) {
            port.cmdSpinlock.acquire();

            const availableCmd = ~PCIRegister.read(self, portIndex);

            var isFound = false;
            var slot: u64 = 0;
            while (slot < self.commandSlotCount) : (slot += 1) {
                if (availableCmd & (@as(u32, 1) << @as(u5, @intCast(slot))) != 0 and port.cmdCTXs[slot] == null) {
                    cmdIdx = slot;
                    isFound = true;
                    break;
                }
            }

            if (!isFound) {
                port.cmdSlotsAvailableEvent.reset();
            } else {
                port.cmdCTXs[cmdIdx] = dispatchGroup;
            }

            port.cmdSpinlock.release();

            if (!isFound) {
                _ = port.cmdSlotsAvailableEvent.wait();
            } else {
                break;
            }
        }

        const sectorCount = byteCount / port.sector_byte_count;
        const offsetSectors = offset / port.sector_byte_count;

        const command_FIS = @as([*]u32, @ptrFromInt(@intFromPtr(port.cmdTables) + CMD_TABLE_SIZE * cmdIdx));
        command_FIS[0] = 0x27 | (1 << 15) | (@as(u32, if (op == BlockDevice.write) 0x35 else 0x25) << 16);
        command_FIS[1] = (@as(u32, @intCast(offsetSectors)) & 0xffffff) | (1 << 30);
        command_FIS[2] = @as(u32, @intCast(offsetSectors >> 24)) & 0xffffff;
        command_FIS[3] = @as(u16, @truncate(sectorCount));
        command_FIS[4] = 0;

        var m_PRDT_entry_count: u64 = 0;
        const prdt = @as([*]u32, @ptrFromInt(@intFromPtr(port.cmdTables) + CMD_TABLE_SIZE * cmdIdx + 0x80));

        while (!buffer.isComplete()) {
            if (m_PRDT_entry_count == PRDT_ENTRY_COUNT) kernel.panic("Too many PRDT entries");

            const segment = buffer.nextSegment(false);

            prdt[0 + 4 * m_PRDT_entry_count] = @as(u32, @truncate(segment.physicalAddr));
            prdt[1 + 4 * m_PRDT_entry_count] = @as(u32, @truncate(segment.physicalAddr >> 32));
            prdt[2 + 4 * m_PRDT_entry_count] = 0;
            prdt[3 + 4 * m_PRDT_entry_count] = (@as(u32, @intCast(segment.byteCount)) - 1) | @as(u32, if (segment.isLast) (1 << 31) else 0);
            m_PRDT_entry_count += 1;
        }

        port.cmdList[cmdIdx * 8 + 0] = 5 | (@as(u32, @intCast(m_PRDT_entry_count)) << 16) | @as(u32, if (op == BlockDevice.write) (1 << 6) else 0);
        port.cmdList[cmdIdx * 8 + 1] = 0;

        if (port.atapi) {
            port.cmdList[cmdIdx * 8 + 0] |= (1 << 5);
            command_FIS[0] = 0x27 | (1 << 15) | (0xa0 << 16);
            command_FIS[1] = @as(u32, @intCast(byteCount)) << 8;

            const scsi_command = @as([*]u8, @ptrFromInt(@intFromPtr(command_FIS) + 0x40));
            std.mem.set(u8, scsi_command[0..10], 0);
            scsi_command[0] = 0xa8;
            scsi_command[2] = @as(u8, @truncate(offsetSectors >> 0x18));
            scsi_command[3] = @as(u8, @truncate(offsetSectors >> 0x10));
            scsi_command[4] = @as(u8, @truncate(offsetSectors >> 0x08));
            scsi_command[5] = @as(u8, @truncate(offsetSectors >> 0x00));
            scsi_command[9] = @intCast(sectorCount);
        }

        port.cmdSpinlock.acquire();
        port.runningCmds |= @as(u32, @intCast(1)) << @as(u5, @intCast(cmdIdx));
        @fence(.SeqCst);
        PCIRegister.write(self, portIndex, @as(u32, @intCast(1)) << @as(u5, @intCast(cmdIdx)));
        port.cmdStartTimestamps[cmdIdx] = kernel.scheduler.timeMs;
        port.cmdSpinlock.release();

        return true;
    }
};
const GENERAL_TIMEOUT = 5000;

// TODO!: Implement this
const PartitionDevice = extern struct {
    sectorOffset: u64,
    block: BlockDevice,
    parent: *BlockDevice,
    const maxCount = 64;

    fn register(self: *@This(), parent: *BlockDevice, offset: u64, sectorCount: u64, model: []const u8) void {
        @memcpy(self.block.model[0..model.len], model);

        self.parent = parent;
        self.block.sectorSize = parent.sectorSize;
        self.block.maxAccessSectorCount = parent.maxAccessSectorCount;
        self.sectorOffset = offset;
        self.block.sectorCount = sectorCount;
        self.block.readOnly = parent.readOnly;
        self.block.modelBytes = @as(u8, @intCast(model.len));
        self.block.nestLevel = parent.nestLevel + 1;
        self.block.driveType = parent.driveType;
    }

    fn access(req: BlockDevice.AccessRequest) kernel.Error {
        var request = req;
        const device = @as(*PartitionDevice, @ptrCast(request.device));
        request.device = @as(*BlockDevice, @ptrCast(device.parent));
        request.offset += device.sectorOffset * device.block.sectorSize;
        return FSBlockDeviceAccess(request);
    }
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

const SIGNATURE_BLOCK_SIZE = 65536;
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

    const AccessRequest = extern struct {
        device: *BlockDevice,
        offset: u64,
        count: u64,
        op: i32,
        buffer: *DMABuffer,
        flags: Flags,
        dispatchGroup: ?*kernel.Workgroup,

        const Flags = kernel.ds.Bitflag(enum(u64) {
            cache = 0,
            softErrors = 1,
        });
        const Callback = fn (self: @This()) i64;
    };
    fn detectFilesystem(self: *@This()) void {
        _ = self.detectFilesystemMutex.acquire();
        defer self.detectFilesystemMutex.release();

        if (self.nestLevel > 4) kernel.panic("Filesystem nest limit");

        const sectorsToRead = (SIGNATURE_BLOCK_SIZE + self.sectorSize - 1) / self.sectorSize;
        if (sectorsToRead > self.sectorCount) kernel.panic("Drive too small");

        const bytesToRead = sectorsToRead * self.sectorSize;
        self.signatureBlock = @as(?[*]u8, @ptrFromInt(kernel.heapFixed.alloc(bytesToRead, false))) orelse kernel.panic("unable to allocate memory for fs detection");
        var dmaBuffer = kernel.zeroes(DMABuffer);
        dmaBuffer.virtualAddr = @intFromPtr(self.signatureBlock);
        var request = kernel.zeroes(BlockDevice.AccessRequest);
        request.device = self;
        request.count = bytesToRead;
        request.op = read;
        request.buffer = &dmaBuffer;
        //TODO?: should have access

        if (!self.checkMbr()) kernel.panic("Only MBR is supported\n");

        kernel.heapFixed.free(@intFromPtr(self.signatureBlock), bytesToRead);
    }
    fn checkMbr(self: *@This()) bool {
        if (MBR.getPartitions(self.signatureBlock, self.sectorCount)) {
            for (driver.mbrPartitions) |partition| {
                if (partition.present) {
                    const idx = driver.partitionDevices.len;
                    driver.partitionDevices.len += 1;
                    const partitionDev = &driver.partitionDevices[idx];
                    partitionDev.register(self, partition.offset, partition.count, "MBR partition");
                    return true;
                }
            }
        }

        return false;
    }
};

fn FSBlockDeviceAccess(_request: BlockDevice.AccessRequest) kernel.Error {
    var request = _request;
    const device = request.device;

    if (request.count == 0) return kernel.ES_SUCCESS;

    if (device.readOnly and request.op == BlockDevice.write) {
        if (request.flags.contains(.softErrors)) return kernel.Errors.ES_ERROR_BLOCK_ACCESS_INVALID;
        kernel.panic("The device is read-only and the access requests for write permission");
    }

    if (request.offset / device.sectorSize > device.sectorCount or (request.offset + request.count) / device.sectorSize > device.sectorCount) {
        if (request.flags.contains(.softErrors)) return kernel.Errors.ES_ERROR_BLOCK_ACCESS_INVALID;
        kernel.panic("Disk access out of bounds");
    }

    if (request.offset % device.sectorSize != 0 or request.count % device.sectorSize != 0) {
        if (request.flags.contains(.softErrors)) return kernel.Errors.ES_ERROR_BLOCK_ACCESS_INVALID;
        kernel.panic("Unaligned access\n");
    }

    var buffer = request.buffer.*;

    if (buffer.virtualAddr & 3 != 0) {
        if (request.flags.contains(.softErrors)) return kernel.Errors.ES_ERROR_BLOCK_ACCESS_INVALID;
        kernel.panic("Buffer must be 4-byte aligned");
    }

    var fakeDispatchGroup = kernel.zeroes(kernel.Workgroup);
    if (request.dispatchGroup == null) {
        fakeDispatchGroup.init();
        request.dispatchGroup = &fakeDispatchGroup;
    }

    var r = kernel.zeroes(BlockDevice.AccessRequest);
    r.device = request.device;
    r.buffer = &buffer;
    r.flags = request.flags;
    r.dispatchGroup = request.dispatchGroup;
    r.op = request.op;
    r.offset = request.offset;

    while (request.count != 0) {
        r.count = device.maxAccessSectorCount * device.sectorSize;
        if (r.count > request.count) r.count = request.count;

        buffer.offset = 0;
        buffer.totalByteCount = r.count;
        const callback = @as(BlockDevice.AccessRequest.Callback, @ptrFromInt(device.access));
        _ = callback(r);
        r.offset += r.count;
        buffer.virtualAddr += r.count;
        request.count -= r.count;
    }

    if (request.dispatchGroup == &fakeDispatchGroup) {
        return if (fakeDispatchGroup.wait()) kernel.ES_SUCCESS else kernel.Errors.ES_ERROR_DRIVE_CONTROLLER_REPORTED;
    } else {
        return kernel.ES_SUCCESS;
    }
}

export var recentInterruptEvents: [64]kernel.Volatile(InterruptEvent) = undefined;
export var recentInterruptEventsPtr: kernel.Volatile(u64) = undefined;
const InterruptEvent = extern struct {
    timestamp: u64,
    globalInterruptStatus: u32,
    port0RunningCmds: u32,
    port0IssuedCmds: u32,
    isComplete: bool,
};
