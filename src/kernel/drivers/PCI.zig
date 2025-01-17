const std = @import("std");
const kernel = @import("../kernel.zig");
const utils = @import("../kernelUtils.zig");
const roundUp = utils.roundUp;
const arch = kernel.arch;
const pageSize = arch.pageSize;

pub var driver: *Driver = undefined;
pub const Device = extern struct {
    deviceID: u32,
    sybsystemID: u32,
    domain: u32,
    classCode: u8,
    subClassCode: u8,
    progIF: u8,
    bus: u8,
    slot: u8,
    func: u8,
    interruptPin: u8,
    interruptLine: u8,
    baseVA: [6]u64,
    basePA: [6]u64,
    baseSize: [6]u64,
    baseAddrSpace: [6]u32,

    const maxCount = 32;
    pub fn readConfig(comptime T: type, self: *@This(), offset: u8) T {
        const bitWidth = @sizeOf(T) * 8;
        return switch (bitWidth) {
            8 => @as(T, @truncate(arch.readPciConfig(self.bus, self.slot, self.func, offset, 8))),
            16 => @as(T, @truncate(arch.readPciConfig(self.bus, self.slot, self.func, offset, 16))),
            32 => @as(T, @truncate(arch.readPciConfig(self.bus, self.slot, self.func, offset, 32))),
            else => @panic("Unsupported bit width"),
        };
    }
    pub fn writeConfig(comptime T: type, self: *@This(), offset: u8, value: T) void {
        const bitWidth = @sizeOf(T) * 8;
        switch (bitWidth) {
            8 => arch.writePciConfig(self.bus, self.slot, self.func, offset, value, 8),
            16 => arch.writePciConfig(self.bus, self.slot, self.func, offset, value, 16),
            32 => arch.writePciConfig(self.bus, self.slot, self.func, offset, value, 32),
            else => @panic("Unsupported bit width"),
        }
    }

    pub fn readBaseAddrReg(comptime T: type, self: *@This(), index: u32, offset: u32) T {
        const baseAddr = self.baseAddrSpace[index];
        if (baseAddr & 1 != 0) {
            return switch (@sizeOf(T)) {
                1 => @as(T, arch.in8(@as(u16, @intCast((baseAddr & ~@as(u32, 3)) + offset)))),
                4 => @as(T, arch.in32(@as(u32, @intCast((baseAddr & ~@as(u32, 3)) + offset)))),
                else => @panic("Unsupported type size for I/O access"),
            };
        } else {
            return @as(*volatile T, @ptrFromInt(self.baseVA[index] + offset)).*;
        }
    }

    pub fn writeBaseAddrReg32(self: @This(), index: u32, offset: u32, value: u32) void {
        const baseAddr = self.baseAddrSpace[index];
        if (baseAddr & 1 != 0) {
            arch.out32(@as(u16, @intCast((baseAddr & ~@as(u32, 3)) + offset)), value);
        } else {
            @as(*volatile u32, @ptrFromInt(self.baseVA[index] + offset)).* = value;
        }
    }
    pub fn enableFeatures(self: *@This(), features: Features) bool {
        var config = self.readConfig(u32, 0x4);
        if (features.contains(.interrups)) config &= ~(@as(u32, 1) << 10);
        if (features.contains(.busMasterDMA)) config |= 1 << 2;
        if (features.contains(.memorySpaceAccess)) config |= 1 << 1;
        if (features.contains(.ioPortAccess)) config |= 1 << 0;
        self.writeConfig(u32, 0x4, config);

        if (self.readConfig(u32, 4) != config) return false;

        var i: u8 = 0;
        while (i < 6) : (i += 1) {
            if (~features.bits & (@as(u32, 1) << @as(u5, @intCast(i))) != 0) continue;
            const isIOPort = self.baseAddrSpace[i] & 1 != 0;
            if (isIOPort) continue;
            const is64 = self.baseAddrSpace[i] & 4 != 0;
            if (self.baseAddrSpace[i] & 8 == 0) {
                self.writeBaseAddrReg32(i, 0, @as(u32, @truncate(self.baseVA[i])));
                continue;
            }

            var address: u64 = undefined;
            var size: u64 = undefined;

            if (is64) {
                self.writeConfig(u32, 0x10 + 4 * i, 0xffffffff);
                self.writeConfig(u32, 0x10 + 4 * (i + 1), 0xffffffff);
                size = self.readConfig(u32, 0x10 + 4 * i);
                size |= @as(u64, @intCast(self.readConfig(u32, 0x10 + 4 * (i + 1)))) << 32;
                self.writeConfig(u32, 0x10 + 4 * i, self.baseAddrSpace[i]);
                self.writeConfig(u32, 0x10 + 4 * (i + 1), self.baseAddrSpace[i + 1]);
                address = self.baseAddrSpace[i];
                address |= @as(u64, @intCast(self.baseAddrSpace[i + 1])) << 32;
            } else {
                self.writeConfig(u32, 0x10 + 4 * i, 0xffffffff);
                size = self.readConfig(u32, 0x10 + 4 * i);
                size |= @as(u64, @intCast(0xffffffff)) << 32;
                self.writeConfig(u32, 0x10 + 4 * i, self.baseAddrSpace[i]);
                address = self.baseAddrSpace[i];
            }

            if (size == 0) return false;
            if (address == 0) return false;

            size &= ~@as(@TypeOf(size), 0xf);
            size = ~size + 1;
            address &= ~@as(@TypeOf(address), 0xf);

            self.baseVA[i] = kernel.addrSpace.mapPhysical(address, size, kernel.memory.Region.Flags.fromFlag(.notCachable));

            self.basePA[i] = address;
            self.baseSize[i] = size;
            kernel.memory.checkUnusable(address, size);
        }
        return true;
    }

    pub fn enableMSI(self: *@This(), handler: arch.KIRQHandler, ctx: u64, ownerName: []const u8) bool {
        const status = @as(u16, @truncate(self.readConfig(u32, 0x4 >> 16)));
        if (~status & (1 << 4) != 0) return false;

        var ptr = self.readConfig(u8, 0x34);
        var idx: u64 = 0;

        while (true) {
            if (ptr == 0) break;
            const _idx = idx;
            idx += 1;
            if (_idx >= 0xff) break;

            var dw = self.readConfig(u32, ptr);
            const nextPtr = @as(u8, @truncate(dw >> 8));
            const id = @as(u8, @truncate(dw));

            if (id != 5) {
                ptr = nextPtr;
                continue;
            }

            const msi = arch.MSI.register(handler, ctx, ownerName);

            if (msi.address == 0) return false;
            var control = @as(u16, @truncate(dw >> 16));

            if (msi.data & ~@as(u64, 0xffff) != 0) {
                arch.MSI.unregister(msi.tag);
                return false;
            }

            if (msi.address & 0b11 != 0) {
                arch.MSI.unregister(msi.tag);
                return false;
            }
            if (msi.address & 0xFFFFFFFF00000000 != 0 and ~control & (1 << 7) != 0) {
                arch.MSI.unregister(msi.tag);
                return false;
            }

            control = (control & ~@as(u16, 7 << 4)) | (1 << 0);
            dw = @as(u16, @truncate(dw)) | (@as(u32, control) << 16);

            self.writeConfig(u32, ptr + 0, dw);
            self.writeConfig(u32, ptr + 4, @as(u32, @truncate(msi.address)));

            if (control & (1 << 7) != 0) {
                self.writeConfig(ptr + 8, @as(u32, @truncate(msi.address >> 32)));
                self.writeConfig(u16, ptr + 12, @as(u16, @intCast((self.readConfig(u16, ptr + 12) & 0x3800) | msi.data)));
                if (control & (1 << 8) != 0) self.writeConfig(u32, ptr + 16, 0);
            } else {
                self.writeConfig(u16, ptr + 8, @as(u16, @intCast(msi.data)));
                if (control & (1 << 8) != 0) self.writeConfig(u32, ptr + 12, 0);
            }

            return true;
        }
        return false;
    }

    pub fn enableSingleInterrupt(self: *@This(), handler: arch.KIRQHandler, ctx: u64, ownerName: []const u8) bool {
        if (self.enableMSI(handler, ctx, ownerName)) return true;
        if (self.interruptPin == 0 or self.interruptPin > 4) return false;

        // const res = self.enableFeatures(Features.fromFlag(.interrups));
        // var line = @as(i64, @intCast(self.interruptLine));
        //TODO!: figure it out (depends on the bl)
        // if (res) {
        //     arch.registerIRQ(line, handler, ctx, ownerName);
        //     return true;
        // }
    }
};

pub const Features = kernel.ds.Bitflag(enum(u64) {
    bar0 = 0,
    bar1 = 1,
    bar2 = 2,
    bar3 = 3,
    bar4 = 4,
    bar5 = 5,
    interrups = 8,
    busMasterDMA = 9,
    memorySpaceAccess = 10,
    ioPortAccess = 11,
});

pub const Driver = struct {
    devices: []Device,
    busScanStates: [256]u8,

    pub fn init() void {
        const devicesOffset = roundUp(u64, @sizeOf(Driver), @alignOf(Device));

        const allocSize = devicesOffset + (@sizeOf(Device) * Device.maxCount);

        const address = kernel.addrSpace.alloc(allocSize, kernel.memory.Region.Flags.fromFlag(.fixed), arch.modulePtr, true);
        if (address == 0) kernel.panic("Could not allocate memory for PCI driver");
        arch.modulePtr += roundUp(u64, allocSize, pageSize);

        driver = @as(*Driver, @ptrFromInt(address));
        driver.devices.ptr = @as([*]Device, @ptrFromInt(address + devicesOffset));
        driver.devices.len = 0;
        driver.setup();
    }

    fn setup(self: *@This()) void {
        const baseHeaderType = arch.readPciConfig(u32, 0, 0, 0, 0x0c);
        const baseBusCount: u8 = if (baseHeaderType & 0x80 != 0) 8 else 1;
        var busToScanCount: u8 = 0;

        var baseBus: u8 = 0;
        while (baseBus < baseBusCount) : (baseBus += 1) {
            const deviceID = arch.readPciConfig(u32, 0, 0, baseBus, 0);
            if (deviceID & 0xffff != 0xffff) {
                self.busScanStates[baseBus] = @intFromEnum(Bus.scanNext);
                busToScanCount += 1;
            }
        }

        if (busToScanCount == 0) kernel.panic("No bus found");

        var foundUSB = false;

        while (busToScanCount > 0) {
            for (self.busScanStates, 0..) |*bss, _bus| {
                const bus = @as(u8, @intCast(_bus));
                if (bss.* == @intFromEnum(Bus.scanNext)) {
                    bss.* = @intFromEnum(Bus.scanned);
                    busToScanCount -= 1;

                    var device: u8 = 0;
                    while (device < 32) : (device += 1) {
                        const _deviceID = arch.readPciConfig(u32, bus, device, 0, 0);
                        if (_deviceID & 0xffff != 0xffff) {
                            const headerType = @as(u8, @truncate(arch.readPciConfig(u32, bus, device, 0, 0x0c) >> 16));
                            const funcCount: u8 = if (headerType & 0x80 != 0) 8 else 1;

                            var function: u8 = 0;
                            while (function < funcCount) : (function += 1) {
                                const deviceID = arch.readPciConfig(u32, bus, device, function, 0);
                                if (deviceID & 0xffff != 0xffff) {
                                    const deviceClass = arch.readPciConfig(u32, bus, device, function, 0x08);
                                    const interruptInfo = arch.readPciConfig(u32, bus, device, function, 0x3c);
                                    const index = self.devices.len;
                                    self.devices.len += 1;

                                    const pciDevice = &self.devices[index];
                                    pciDevice.classCode = @as(u8, @truncate(deviceClass >> 24));
                                    pciDevice.subClassCode = @as(u8, @truncate(deviceClass >> 16));
                                    pciDevice.progIF = @as(u8, @truncate(deviceClass >> 8));
                                    pciDevice.bus = bus;
                                    pciDevice.slot = device;
                                    pciDevice.func = function;
                                    pciDevice.interruptPin = @as(u8, @truncate(interruptInfo >> 8));
                                    pciDevice.interruptLine = @as(u8, @truncate(interruptInfo >> 0));
                                    pciDevice.deviceID = arch.readPciConfig(u32, bus, device, function, 0);
                                    pciDevice.sybsystemID = arch.readPciConfig(u32, bus, device, function, 0x2c);

                                    for (pciDevice.baseAddrSpace, 0..) |*baseAddr, i| {
                                        baseAddr.* = pciDevice.readConfig(u32, @as(u8, @intCast(0x10 + 4 * i)));
                                    }

                                    const isPCIBridge = pciDevice.classCode == 0x06 and pciDevice.subClassCode == 0x04;
                                    if (isPCIBridge) {
                                        const secondaryBus = @as(u8, @truncate(arch.readPciConfig(u32, bus, device, function, 0x18) >> 8));
                                        if (self.busScanStates[secondaryBus] == @intFromEnum(Bus.doNotScan)) {
                                            busToScanCount += 1;
                                            self.busScanStates[secondaryBus] = @intFromEnum(Bus.scanNext);
                                        }
                                    }

                                    const isUSB = pciDevice.classCode == 12 and pciDevice.subClassCode == 3;
                                    if (isUSB) foundUSB = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};

pub const Bus = enum(u8) {
    doNotScan = 0,
    scanNext = 1,
    scanned = 2,
};
