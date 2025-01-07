const std = @import("std");
const kernel = @import("../kernel.zig");
const Bitflag = kernel.ds.Bitflag;

const serial = kernel.drivers.serial;

pub var driver: Driver = undefined;
var info: *VideoModeInformation = undefined;
pub const Driver = struct {
    linearFramebuffer: [*]volatile u8,
    width: u32,
    height: u32,
    pixelCountX: u32,
    pixelCountY: u32,

    fn setup(self: *@This()) void {
        info = @as(?*VideoModeInformation, @ptrFromInt(kernel.addrSpace.mapPhysical(0x7000 + kernel.arch.bootloaderInfoOffset, @sizeOf(VideoModeInformation), kernel.memory.Region.Flags.empty())));

        if (!info.validation.contains(.valid)) std.debug.panic("SVGA: No valid video mode information found");

        if (info.validation.contains(.edidValid)) {
            serial.write("SVGA: EDID information found\n");
            for (info.edid, 0..) |byte, i| {
                if (i % 10 == 0) serial.write("\n");
                kernel.log(" 0x{x:0>2} |", .{byte});
            }
            serial.write("\n");
        }

        self.linearFramebuffer = @as(?[*]volatile u8, @ptrFromInt(kernel.addrSpace.mapPhysical(info.physicalAddrBuffer, @as(u32, @intCast(info.bytesPerScanline)) * @as(u32, @intCast(info.height)), kernel.memory.Region.Flags.fromFlag(.writeCombining)))) orelse kernel.panic("Unable to map VBE");

        self.width = @as(u32, @intCast(info.width));
        self.height = @as(u32, @intCast(info.height));
        self.pixelCountX = info.bitsPerPixel >> 3;
        self.pixelCountY = info.bytesPerScanline;

        if (info.bitsPerPixel != 32) {
            kernel.panicf("Only 32 bit pixels are supported. VBE pixel count: {}", .{info.bitsPerPixel});
        }
    }
};

const VideoModeInformation = extern struct {
    validation: Validation,
    bitsPerPixel: u8,
    width: u16,
    height: u16,
    bytesPerScanline: u16,
    physicalAddrBuffer: u64,
    edid: [128]u8,

    const Validation = Bitflag(enum(u8) {
        valid = 0,
        edidValid = 1,
    });
};
