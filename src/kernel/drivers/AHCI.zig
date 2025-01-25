const std = @import("std");
const kernel = @import("../kernel.zig");
const PCI = kernel.drivers.PCI;
const Mutex = kernel.sync.Mutex;
const arch = @import("../arch/x86_64.zig");
const kUtils = @import("../kernelUtils.zig");
const COMMAND_LIST_SIZE = 0x400;
const RECEIVED_FIS_SIZE = 0x100;
const PRDT_ENTRY_COUNT = 0x48;
const CMD_TABLE_SIZE = 0x80 + PRDT_ENTRY_COUNT * 0x10;
const classCode = 1;
const subClassCode = 6;
const progIF = 1;
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
const CAP = GlobalRegister(0);
const CAP2 = GlobalRegister(0x24);
const BOHC = GlobalRegister(0x28);
const GHC = GlobalRegister(4);
const PI = GlobalRegister(0xc);

const PIE = PortRegister(0x114);
const PCMD = PortRegister(0x118);
const PSIG = PortRegister(0x124);
const PSSTS = PortRegister(0x128);
const PSCTL = PortRegister(0x12c);
const PSERR = PortRegister(0x130);
const PCLB = PortRegister(0x100);
const PCLBU = PortRegister(0x104);
const PFB = PortRegister(0x108);
const PFBU = PortRegister(0x10c);

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
        driver = @as(*Driver, @intFromPtr(address));
        driver.drives.ptr = @as([*]Drive, @intFromPtr(address + driveOffset));
        driver.drives.len = 0;
        driver.partitionDevices.ptr = @as([*]PartitionDevice, @intFromPtr(address + partitionDevicesOffset));
        driver.partitionDevices.len = 0;
        driver.setup();
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
    //TODO: improve this
    pub fn setup(self: *@This()) void {
        self.pci = &PCI.driver.devices[3];

        const is_AHCPI_Device = self.pci.classCode == classCode and self.pci.subClassCode == subClassCode and self.pci.progIF == progIF;

        if (!is_AHCPI_Device) kernel.panic("AHCI PCI device not found");

        _ = self.pci.enableFeatures(PCI.Features.fromFlags(.{ .interrupts, .busMasterDMA, .memorySpaceAccess, .bar5 }));

        if (CAP2.read(self) & (1 << 0) != 0) {
            BOHC.write(self, BOHC.read(self) | (1 << 1));
            const timeout = kernel.Timeout.new(25);
            var status: u32 = undefined;

            while (true) {
                status = BOHC.read(self);
                if (status & (1 << 0) != 0) break;
                if (timeout.hit()) break;
            }

            if (status & (1 << 0) != 0) {
                var event = kernel.zeroes(kernel.sync.Event);
                _ = event.waitTimeout(2000);
            }
        }

        {
            const timeout = kernel.Timeout.new(GENERAL_TIMEOUT);
            GHC.write(self, GHC.read(self) | (1 << 0));
            while (GHC.read(self) & (1 << 0) != 0 and !timeout.hit()) {}

            if (timeout.hit()) kernel.panic("AHCI timeout hit");
        }
        if (!self.pci.enableSingleInterrupt(handler, @intFromPtr(self), "AHCI")) kernel.panic("Unable to initialize AHCI");

        GHC.write(self, GHC.read(self) | (1 << 31) | (1 << 1));
        self.capabilities = CAP.read(self);
        self.cap2 = CAP2.read(self);
        self.commandSlotCount = ((self.capabilities >> 8) & 31) + 1;
        self.isDm64Supported = self.capabilities & (1 << 31) != 0;
        if (!self.isDm64Supported) kernel.panic("DMA is not supported");

        const maxPortNumber = (self.capabilities & 31) + 1;
        var isPortCountFound: u64 = 0;
        const implementedPorts = PI.read(self);

        for (self.ports, 0..) |*port, i| {
            if (implementedPorts & (@as(u32, 1) << @as(u5, @intCast(i))) != 0) {
                isPortCountFound += 1;
                if (isPortCountFound <= maxPortNumber) port.connected = true;
            }
        }

        for (self.ports, 0..) |*port, _port_i| {
            if (port.connected) {
                const port_i = @as(u32, @intCast(_port_i));
                const neededByteCount = COMMAND_LIST_SIZE + RECEIVED_FIS_SIZE + CMD_TABLE_SIZE * self.commandSlotCount;

                var virtualAddr: u64 = 0;
                var physicalAddr: u64 = 0;

                if (!kernel.memory.physicalAllocFlagged(neededByteCount, arch.pageSize, if (self.isDm64Supported) 64 else 32, true, kernel.memory.Region.Flags.fromFlag(.notCacheable), &virtualAddr, &physicalAddr)) {
                    kernel.panic("AHCI allocation failure");
                }

                port.cmdList = @as([*]u32, @intFromPtr(virtualAddr));
                port.cmdTables = @as([*]u8, @intFromPtr(virtualAddr + COMMAND_LIST_SIZE + RECEIVED_FIS_SIZE));

                PCLB.write(self, port_i, @as(u32, @truncate(physicalAddr)));
                PFB.write(self, port_i, @as(u32, @truncate(physicalAddr + 0x400)));
                if (self.isDm64Supported) {
                    PCLBU.write(self, port_i, @as(u32, @truncate(physicalAddr >> 32)));
                    PFBU.write(self, port_i, @as(u32, @truncate((physicalAddr + 0x400) >> 32)));
                }

                var cmdSlot: u64 = 0;
                while (cmdSlot < self.commandSlotCount) : (cmdSlot += 1) {
                    const address = physicalAddr + COMMAND_LIST_SIZE + RECEIVED_FIS_SIZE + COMMAND_LIST_SIZE * cmdSlot;
                    port.cmdList[cmdSlot * 8 + 2] = @as(u32, @truncate(address));
                    port.cmdList[cmdSlot * 8 + 3] = @as(u32, @truncate(address >> 32));
                }

                const timeout = kernel.Timeout.new(GENERAL_TIMEOUT);
                const runningBits = (1 << 0) | (1 << 4) | (1 << 15) | (1 << 14);

                while (true) {
                    const status = PCMD.read(self, port_i);
                    if (status & runningBits == 0 or timeout.hit()) break;
                    PCMD.write(self, port_i, status & ~@as(u32, (1 << 0) | (1 << 4)));
                }

                const resetPortTimeout = PCMD.read(self, port_i) & runningBits != 0;
                if (resetPortTimeout) {
                    port.connected = false;
                    continue;
                }

                PIE.write(self, port_i, PIE.read(self, port_i) & 0x0e3fff0);
                PIS.write(self, port_i, PIS.read(self, port_i));

                PSCTL.write(self, port_i, PSCTL.read(self, port_i) | (3 << 8));
                PCMD.write(self, port_i, (PCMD.read(self, port_i) & 0x0FFFFFFF) |
                    (1 << 1) |
                    (1 << 2) |
                    (1 << 4) |
                    (1 << 28));

                var linkTimeout = kernel.Timeout.new(10);

                while (PSSTS.read(self, port_i) & 0xf != 3 and !linkTimeout.hit()) {}
                const activatePortTimeout = PSSTS.read(self, port_i) & 0xf != 3;
                if (activatePortTimeout) {
                    port.connected = false;
                    continue;
                }

                PSERR.write(self, port_i, PSERR.read(self, port_i));

                while (PTFD.read(self, port_i) & 0x88 != 0 and !timeout.hit()) {}
                const portReadyTimeout = PTFD.read(self, port_i) & 0x88 != 0;
                if (portReadyTimeout) {
                    port.connected = false;
                    continue;
                }

                PCMD.write(self, port_i, PCMD.read(self, port_i) | (1 << 0));
                PIE.write(self, port_i, PIE.read(self, port_i) |
                    (1 << 5) |
                    (1 << 0) |
                    (1 << 30) |
                    (1 << 29) |
                    (1 << 28) |
                    (1 << 27) |
                    (1 << 26) |
                    (1 << 24) |
                    (1 << 23));
            }
        }

        for (self.ports, 0..) |*port, _port_i| {
            if (port.connected) {
                const port_i = @as(u32, @intCast(_port_i));

                const status = PSSTS.read(self, port_i);

                if (status & 0xf != 0x3 or status & 0xf0 == 0 or status & 0xf00 != 0x100) {
                    port.connected = false;
                    continue;
                }

                const signature = PSIG.read(self, port_i);

                if (signature == 0x00000101) {
                    // handle SATA drive
                } else if (signature == 0xEB140101) {
                    // SATAPI drive
                    port.atapi = true;
                } else if (signature == 0) {
                    // no drive connected
                    port.connected = false;
                } else {
                    // unrecognized drive signature
                    port.connected = false;
                }
            }
        }

        var identifyData: u64 = 0;
        var identifyDataPhysical: u64 = 0;

        if (!kernel.memory.physicalAllocFlagged(0x200, arch.pageSize, if (self.isDm64Supported) 64 else 32, true, kernel.memory.Region.Flags.fromFlag(.notCacheable), &identifyData, &identifyDataPhysical)) {
            kernel.panic("Allocation failure");
        }

        for (self.ports, 0..) |*port, _port_i| {
            if (port.connected) {
                const port_i = @as(u32, @intCast(_port_i));
                kernel.EsMemoryZero(identifyData, 0x200);

                port.cmdList[0] = 5 | (1 << 16);
                port.cmdList[1] = 0;

                const opcode: u32 = if (port.atapi) 0xa1 else 0xec;
                const cmdFIS = @as([*]u32, @ptrCast(@alignCast(port.cmdTables)));
                cmdFIS[0] = 0x27 | (1 << 15) | (opcode << 16);
                cmdFIS[1] = 0;
                cmdFIS[2] = 0;
                cmdFIS[3] = 0;
                cmdFIS[4] = 0;

                const prdt = @as([*]u32, @intFromPtr(@intFromPtr(port.cmdTables) + 0x80));
                prdt[0] = @as(u32, @truncate(identifyDataPhysical));
                prdt[1] = @as(u32, @truncate(identifyDataPhysical >> 32));
                prdt[2] = 0;
                prdt[3] = 0x200 - 1;

                if (!self.sendSingleCmd(port_i)) {
                    PCMD.write(self, port_i, PCMD.read(self, port_i) & ~@as(u32, 1 << 0));
                    port.connected = false;
                    continue;
                }

                port.sectorByteCount = 0x200;

                const identifyPtr = @as([*]u16, @intFromPtr(identifyData));
                if (identifyPtr[106] & (1 << 14) != 0 and ~identifyPtr[106] & (1 << 15) != 0 and identifyPtr[106] & (1 << 12) != 0) {
                    port.sectorByteCount = identifyPtr[117] | (@as(u32, @intCast(identifyPtr[118])) << 16);
                }

                port.sectorCount = identifyPtr[100] + (@as(u64, @intCast(identifyPtr[101])) << 16) + (@as(u64, @intCast(identifyPtr[102])) << 32) + (@as(u64, @intCast(identifyPtr[103])) << 48);

                if (!(identifyPtr[49] & (1 << 9) != 0 and identifyPtr[49] & (1 << 8) != 0)) {
                    port.connected = false;
                    continue;
                }

                if (port.atapi) {
                    port.cmdList[0] = 5 | (1 << 16) | (1 << 5);
                    cmdFIS[0] = 0x27 | (1 << 15) | (0xa0 << 16);
                    cmdFIS[1] = 8 << 8;
                    prdt[3] = 8 - 1;

                    const scsiCMD = @as([*]u8, @ptrCast(cmdFIS));
                    kernel.EsMemoryZero(@intFromPtr(scsiCMD), 10);
                    scsiCMD[0] = 0x25;

                    if (!self.sendSingleCmd(port_i)) {
                        PCMD.write(self, port_i, PCMD.read(self, port_i) & ~@as(u32, 1 << 0));
                        port.connected = false;
                        continue;
                    }

                    const capacity = @as([*]u8, @intFromPtr(identifyData));
                    port.sectorCount = capacity[3] + (@as(u64, @intCast(capacity[2])) << 8) + (@as(u64, @intCast(capacity[1])) << 16) + (@as(u64, @intCast(capacity[0])) << 24) + 1;
                    port.sectorByteCount = capacity[7] + (@as(u64, @intCast(capacity[6])) << 8) + (@as(u64, @intCast(capacity[5])) << 16) + (@as(u64, @intCast(capacity[4])) << 24);
                }

                if (port.sectorCount <= 128 or port.sectorByteCount & 0x1ff != 0 or port.sectorByteCount == 0 or port.sectorByteCount > 0x1000) {
                    port.connected = false;
                    continue;
                }

                var model: u64 = 0;
                while (model < 20) : (model += 1) {
                    port.model[model * 2 + 0] = @as(u8, @truncate(identifyPtr[27 + model] >> 8));
                    port.model[model * 2 + 1] = @as(u8, @truncate(identifyPtr[27 + model]));
                }

                port.model[40] = 0;

                model = 39;

                while (model > 0) : (model -= 1) {
                    if (port.model[model] == ' ') port.model[model] = 0 else break;
                }

                port.ssd = identifyPtr[217] == 1;

                var i: u64 = 10;
                while (i < 20) : (i += 1) {
                    identifyPtr[i] = (identifyPtr[i] >> 8) | (identifyPtr[i] << 8);
                }

                i = 23;
                while (i < 27) : (i += 1) {
                    identifyPtr[i] = (identifyPtr[i] >> 8) | (identifyPtr[i] << 8);
                }

                i = 27;
                while (i < 47) : (i += 1) {
                    identifyPtr[i] = (identifyPtr[i] >> 8) | (identifyPtr[i] << 8);
                }
            }
        }

        _ = kernel.addrSpace.free(identifyData, 0, false);
        kernel.memory.physicalFree(identifyDataPhysical, false, 1);

        self.timeoutTimer.setEx(GENERAL_TIMEOUT, TimeoutTimerHit, @intFromPtr(self));

        for (self.ports, 0..) |*port, _port_i| {
            if (port.connected) {
                const port_i = @as(u32, @intCast(_port_i));
                const driveIndex = self.drives.len;
                self.drives.len += 1;
                const drive = &self.drives[driveIndex];
                drive.port = port_i;
                drive.blockDevice.sectorSize = port.sectorByteCount;
                drive.blockDevice.sectorCount = port.sectorCount;
                drive.blockDevice.maxAccessSectorCount = if (port.atapi) (65535 / drive.blockDevice.sectorSize) else (PRDT_ENTRY_COUNT - 1) * arch.pageSize / drive.blockDevice.sectorSize;
                drive.blockDevice.readOnly = port.atapi;
                @memcpy(drive.blockDevice.model[0..port.model.len], port.model[0..]);
                drive.blockDevice.modelBytes = port.model.len;
                drive.blockDevice.driveType = if (port.atapi) DriveType.cdrom else if (port.ssd) DriveType.ssd else DriveType.hdd;

                drive.blockDevice.access = @intFromPtr(accessCallback);
                //TODO: do some fs registering
            }
        }
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

        const sectorCount = byteCount / port.sectorByteCount;
        const offsetSectors = offset / port.sectorByteCount;

        const cmdFIS = @as([*]u32, @intFromPtr(@intFromPtr(port.cmdTables) + CMD_TABLE_SIZE * cmdIdx));
        cmdFIS[0] = 0x27 | (1 << 15) | (@as(u32, if (op == BlockDevice.write) 0x35 else 0x25) << 16);
        cmdFIS[1] = (@as(u32, @intCast(offsetSectors)) & 0xffffff) | (1 << 30);
        cmdFIS[2] = @as(u32, @intCast(offsetSectors >> 24)) & 0xffffff;
        cmdFIS[3] = @as(u16, @truncate(sectorCount));
        cmdFIS[4] = 0;

        var m_PRDT_entry_count: u64 = 0;
        const prdt = @as([*]u32, @intFromPtr(@intFromPtr(port.cmdTables) + CMD_TABLE_SIZE * cmdIdx + 0x80));

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
            cmdFIS[0] = 0x27 | (1 << 15) | (0xa0 << 16);
            cmdFIS[1] = @as(u32, @intCast(byteCount)) << 8;

            const scsiCMD = @as([*]u8, @intFromPtr(@intFromPtr(cmdFIS) + 0x40));
            std.mem.set(u8, scsiCMD[0..10], 0);
            scsiCMD[0] = 0xa8;
            scsiCMD[2] = @as(u8, @truncate(offsetSectors >> 0x18));
            scsiCMD[3] = @as(u8, @truncate(offsetSectors >> 0x10));
            scsiCMD[4] = @as(u8, @truncate(offsetSectors >> 0x08));
            scsiCMD[5] = @as(u8, @truncate(offsetSectors >> 0x00));
            scsiCMD[9] = @intCast(sectorCount);
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

    pub fn readFile(self: *@This(), fileBuffer: []u8, fileDescriptor: *const kernel.FileSytem.File.Descriptor) kernel.files.ReadError!void {
        if (fileDescriptor.offset & (self.blockDevice.sectorSize - 1) != 0) kernel.panic("Disk offset should be sector-aligned");
        const sectorAlignedSize = kUtils.alignu64(fileDescriptor.size, self.blockDevice.sectorSize);
        if (fileBuffer.len < sectorAlignedSize) kernel.panic("Buffer too small\n");

        var buffer = kernel.zeroes(DMABuffer);
        buffer.virtualAddr = @intFromPtr(fileBuffer.ptr);

        var request = kernel.zeroes(BlockDevice.AccessRequest);
        request.offset = fileDescriptor.offset;
        request.count = sectorAlignedSize;
        request.operation = BlockDevice.read;
        request.device = &self.blockDevice;
        request.buffer = &buffer;

        const result = FSBlockDeviceAccess(request);
        if (result != kernel.ES_SUCCESS) return kernel.files.ReadError.failed;
    }
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
        self.signatureBlock = @as(?[*]u8, @intFromPtr(kernel.heapFixed.alloc(bytesToRead, false))) orelse kernel.panic("unable to allocate memory for fs detection");
        var dmaBuffer = kernel.zeroes(DMABuffer);
        dmaBuffer.virtualAddr = @intFromPtr(self.signatureBlock);
        var request = kernel.zeroes(BlockDevice.AccessRequest);
        request.device = self;
        request.count = bytesToRead;
        request.op = read;
        request.buffer = &dmaBuffer;
        if (FSBlockDeviceAccess(request) != kernel.ES_SUCCESS) kernel.panic("Could not read disk");

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
        const callback = @as(BlockDevice.AccessRequest.Callback, @intFromPtr(device.access));
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

fn TimeoutTimerHit(task: *kernel.scheduling.AsyncTask) void {
    const _driver = @as(Driver, @fieldParentPtr("timeoutTimer", @as(kernel.scheduling.Timer, @fieldParentPtr("asyncTask", task))));
    const currentTimestamp = kernel.scheduler.timeMs;

    for (_driver.ports) |*port| {
        port.cmdSpinlock.acquire();

        var slot: u64 = 0;
        while (slot < _driver.commandSlotCount) : (slot += 1) {
            const slotMask = @as(u32, @intCast(1)) << @as(u5, @intCast(slot));
            if (port.runningCmds & slotMask != 0 and port.cmdStartTimestamps[slot] + GENERAL_TIMEOUT < currentTimestamp) {
                port.cmdCTXs[slot].?.end(false);
                port.cmdCTXs[slot] = null;
                port.runningCmds &= ~slotMask;
            }
        }

        port.cmdSpinlock.release();
    }

    _driver.timeoutTimer.setEx(GENERAL_TIMEOUT, TimeoutTimerHit, @intFromPtr(_driver));
}
fn handler(_: u64, context: u64) callconv(.C) bool {
    const tmp = @as(*Driver, @intFromPtr(context));
    return tmp.handleIRQ();
}
