const std = @import("std");

pub const File = extern struct {
    pub const Descriptor = extern struct {
        offset: u64,
        size: u64,
    };
};

pub const Superblock = extern struct {
    kernelExe: File.Descriptor,
    desktopExe: File.Descriptor,
    diskStart: u64,

    pub const offset = 0x2000;
    pub const fileDescriptor = File.Descriptor{ .offset = offset, .size = @sizeOf(Superblock) };

    pub fn format(self: *@This(), kernelRawOffset: u64, kernelSize: u64, diskStart: u64, desktopRawOffset: u64, desktopSize: u64) void {
        self.kernelExe = .{ .offset = kernelRawOffset, .size = kernelSize };
        self.desktopExe = .{ .offset = desktopRawOffset, .size = desktopSize };
        self.diskStart = diskStart;
    }
};
