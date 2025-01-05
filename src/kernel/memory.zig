const std = @import("std");
const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const sync = kernel.sync;
const Mutex = sync.Mutex;
const SpinLock = sync.SpinLock;
const Event = sync.Event;
const WriterLock = kernel.sync.WriterLock;
const AVLTree = kernel.ds.AVLTree;
const LinkedList = kernel.ds.LinkedList;
const Bitflag = kernel.ds.Bitflag;
const Range = kernel.ds.Range;
const AsyncTask = scheduling.AsyncTask;
const List = kernel.ds.List;
const arch = kernel.arch;
const scheduling = kernel.scheduling;
const Process = scheduling.Process;
const Thread = scheduling.Thread;
const pageSize = arch.pageSize;
const pageBitCount = arch.pageBitCount;
const EsHeapFree = kernel.EsHeapFree;

pub const SharedRegion = extern struct {
    size: u64,
    handleCount: Volatile(u64),
    mutex: Mutex,
    addr: u64,
};
pub const Region = extern struct {
    descriptor: Region.Descriptor,
    flags: Region.Flags,
    data: extern struct {
        u: extern union {
            physical: extern struct {
                offset: u64,
            },
            shared: extern struct {
                region: ?*SharedRegion,
                offset: u64,
            },
            file: extern struct {
                node: ?*anyopaque,
                offset: u64,
                zeroedBytes: u64,
                fileHandleFlags: u64,
            },
            normal: extern struct {
                commit: Range.Set,
                commitPageCount: u64,
                guardBefore: ?*Region,
                guardAfter: ?*Region,
            },
        },
        pin: WriterLock,
        mapMutex: Mutex,
    },
    u: extern union {
        item: extern struct {
            base: AVLTree(Region).Node,
            u: extern union {
                size: AVLTree(Region).Node,
                nonGuard: LinkedList(Region).Node,
            },
        },
        core: extern struct {
            used: bool,
        },
    },

    pub const Descriptor = extern struct {
        baseAddr: u64,
        pageCount: u64,
    };

    pub const Flags = Bitflag(enum(u32) {
        fixed = 0,
        notCachable = 1,
        notCOmmitTracking = 2,
        readOnly = 3,
        copyOnWrite = 4,
        writeCombining = 5,
        executable = 6,
        user = 7,
        physical = 8,
        normal = 9,
        shared = 10,
        guard = 11,
        cache = 12,
        file = 13,
    });

    pub fn zeroDataField(self: *@This()) void {
        @memset(@as([*]u8, @ptrCast(&self.data))[0..@sizeOf(@TypeOf(self.data))], 0);
    }
};

pub const HandlePageFaultFlags = Bitflag(enum(u32) {
    write = 0,
    lockAcquired = 1,
    forSupervisor = 2,
});

pub const AddressSpace = extern struct {
    arch: arch.AddressSpace,
    freeRegionBase: AVLTree(Region),
    freeRegionSize: AVLTree(Region),
    usedRegionsNonGuard: LinkedList(Region),
    usedRegions: AVLTree(Region),
    reserveMutex: Mutex,
    refCount: Volatile(i32),
    isUser: bool,
    commitCount: u64,
    reserveCount: u64,
    removeAsyncTask: AsyncTask,

    const Self = @This();

    pub fn findRegion(self: *Self, addr: u64) ?*Region {
        self.reserveMutex.assertLocked();

        if (self == &kernel.coreAddressSpace) {
            for (kernel.mmCoreRegions[0..kernel.mmCoreRegionCount]) |*region| {
                if (region.u.core.used and region.descriptor.baseAddr <= addr and region.descriptor.baseAddr + region.descriptor.pageCount * pageSize > addr) {
                    return region;
                } else {
                    return null;
                }
            }
        } else {
            const node = self.usedRegions.find(addr, .LargestBelowOrEqual) orelse return null;

            const region = node.value.?;
            if (region.descriptor.baseAddr > addr) std.debug.panic("Unvalid regions tree", .{});
            if (region.descriptor.baseAddr + region.descriptor.pageCount * pageSize <= addr) return null;
            return region;
        }
    }
    //TODO: implement the rest of the functions
    pub fn init(space: *Self) callconv(.C) bool {
        space.isUser = true;
    }
    pub fn alloc(space: *Self, byteCount: u64, flags: Region.Flags, baseAddr: u64, commitAll: bool) callconv(.C) u64 {
        space.reserveMutex.lock();
        defer space.reserveMutex.unlock();
        const region = space.reserve(byteCount, flags.orFlag(.normal), baseAddr) orelse return 0;
        if (commitAll) {
            if (!space.commitRange(region, 0, region.descriptor.pageCount)) {
                space.unreserve(region, false, false);
                return 0;
            }
        }
        return region.descriptor.baseAddr;
    }

    pub fn openRef(space: *AddressSpace) callconv(.C) void {
        if (space != &kernel.addrSpace) {
            if (space.refCount.readVolatile() < 1) std.debug.panic("space has invalid reference count", .{});
            _ = space.refCount.atomicFetchAdd(1);
        }
    }

    pub fn closeRef(space: *AddressSpace) callconv(.C) void {
        if (space == &kernel.addrSpace) return;
        if (space.refCount.readVolatile() < 1) std.debug.panic("space has invalid reference count", .{});
        if (space.refCount.atomicFetchSub(1) > 1) return;

        space.removeAsyncTask.register(CloseReferenceTask);
    }

    pub fn free(space: *AddressSpace, addr: u64, expectedSize: u64, userOnly: bool) callconv(.C) bool {
        _ = space.reserveMutex.acquire();
        defer space.reserveMutex.release();

        const region = space.findRegion(addr) orelse return false;

        if (userOnly and !region.flags.contains(.user)) return false;
        if (!region.data.pin.takeEx(WriterLock.exclusive, true)) return false;
        if (region.descriptor.baseAddr != addr and !region.flags.contains(.physical)) return false;
        if (expectedSize != 0 and (expectedSize + pageSize - 1) / pageSize != region.descriptor.pageCount) return false;

        var unmapPages = true;

        if (region.flags.contains(.normal)) {
            if (!space.decommitRange(region, 0, region.descriptor.pageCount)) {
                std.debug.panic("Failed to decommit range", .{});
            }
            if (region.data.u.normal.commitPageCount != 0) {
                std.debug.panic("Failed to decommit range", .{});
            }
            region.data.u.normal.commit.ranges.free();
            unmapPages = false;
        } else if (region.flags.contains(.shared)) {
            //Todo: implement handle closing for objects

        } else if (region.flags.contains(.file) or region.flags.contains(.physical)) {
            std.debug.panic("Not implemented", .{});
        } else if (region.flags.contains(.guard)) return false else std.debug.panic("unsupported region type", .{});
        return true;
    }

    pub fn handlePageFault(space: *AddressSpace, askedAddr: u64, flags: HandlePageFaultFlags) callconv(.C) bool {
        const address = askedAddr & ~@as(u64, pageSize - 1);
        const isLockAcquired = flags.contains(.lockAcquired);

        var region = blk: {
            if (isLockAcquired) {
                space.reserveMutex.assertLocked();
                const result = space.findRegion(askedAddr) orelse return false;
                if (!result.data.pin.takeEx(WriterLock.shared, true)) return false;
                break :blk result;
            } else {
                if (kernel.physicalMemoryManager.getAvailablePageCount() < Physical.Allocator.criticalAvailablePageThreshold and arch.getCurrentThread() != null and !arch.getCurrentThread().?.isPageGenThread) {
                    _ = kernel.physicalMemoryManager.availableCriticalEvent.wait();
                }
                _ = space.reserveMutex.acquire();
                defer space.reserveMutex.release();

                const result = space.findRegion(askedAddr) orelse return false;
                if (!result.data.pin.takeEx(WriterLock.shared, true)) return false;
                break :blk result;
            }
        };

        defer region.data.pin.returnLock(WriterLock.shared);
        _ = region.data.mapMutex.acquire();
        defer region.data.mapMutex.release();

        if (arch.translateAddr(address, flags.contains(.write)) != 0) return true;

        var copyOnWrite = false;
        var markModified = false;

        if (flags.contains(.write)) {
            if (region.flags.contains(.copyOnWrite)) copyOnWrite = true else if (region.flags.contains(.readOnly)) return false else markModified = true;
        }

        const offsetIntoRegion = address - region.descriptor.baseAddr;
        var physicalAllocFlags = Physical.Flags.empty();
        var isZeroPage = true;

        if (space.user) {
            physicalAllocFlags = physicalAllocFlags.orFlag(.zeroed);
            isZeroPage = false;
        }

        var mapPageFlags = MapPageFlags.empty();
        if (space.user) mapPageFlags = mapPageFlags.orFlag(.user);
        if (region.flags.contains(.notCachable)) mapPageFlags = mapPageFlags.orFlag(.notCacheable);
        if (region.flags.contains(.writeCombining)) mapPageFlags = mapPageFlags.orFlag(.writeCombining);
        if (!markModified and !region.flags.contains(.fixed) and region.flags.contains(.file)) mapPageFlags = mapPageFlags.orFlag(.readOnly);

        if (region.flags.contains(.physical)) {
            _ = arch.mapPage(space, region.data.u.physical.offset + address - region.descriptor.baseAddr, address, mapPageFlags);
            return true;
        } else if (region.flags.contains(.shared)) {
            const sharedRegion = region.data.u.shared.region.?;
            if (sharedRegion.handleCount.readVolatile() == 0) std.debug.panic("Shared region has no handles", .{});
            _ = sharedRegion.mutex.acquire();

            const offset = address - region.descriptor.baseAddr + region.data.u.shared.offset;

            if (offset >= sharedRegion.size) {
                sharedRegion.mutex.release();
                return false;
            }

            const entry = @as(*u64, @ptrFromInt(sharedRegion.addr + (offset / pageSize * @sizeOf(u64))));

            if (entry.* & 1 != 0) isZeroPage = false else entry.* = physicalAllocFlagged(physicalAllocFlags) | 1;

            _ = arch.mapPage(space, entry.* & ~@as(u64, pageSize - 1), address, mapPageFlags);
            if (isZeroPage) kernel.EsMemoryZero(address, pageSize);
            sharedRegion.mutex.release();
            return true;
        } else if (region.flags.contains(.file)) {} else if (region.flags.contains(.normal)) {
            if (!region.flags.contains(.notCOmmitTracking)) {
                if (!region.data.u.normal.commit.contains(offsetIntoRegion >> pageBitCount)) {
                    return false;
                }
            }

            _ = arch.mapPage(space, physicalAllocFlagged(physicalAllocFlags), address, mapPageFlags);
            if (isZeroPage) kernel.EsMemoryZero(address, pageSize);
            return true;
        } else if (region.flags.contains(.guard)) {} else {
            return false;
        }
    }
    pub fn decommitRange(space: *AddressSpace, region: *Region, pageOffset: u64, pageCount: u64) callconv(.C) bool {
        space.reserveMutex.assertLocked();

        if (region.flags.contains(.notCOmmitTracking)) std.debug.panic("Region does not support commit tracking", .{});
        if (pageOffset >= region.descriptor.pageCount or pageCount > region.descriptor.pageCount - pageOffset) std.debug.panic("invalid region offset and page count", .{});
        if (!region.flags.contains(.normal)) std.debug.panic("Cannot decommit from non-normal region", .{});

        var delta: i64 = 0;

        if (!region.data.u.normal.commit.clear(pageOffset, pageOffset + pageCount, &delta, true)) return false;

        if (delta > 0) std.debug.panic("invalid delta calculation", .{});
        const deltaUnwrapped = @as(u64, @intCast(-delta));

        if (region.data.u.normal.commitPageCount < deltaUnwrapped) std.debug.panic("invalid delta calculation", .{});

        decommit(deltaUnwrapped * pageSize, region.flags.contains(.fixed));
        space.commitCount -= deltaUnwrapped;
        region.data.u.normal.commitPageCount -= deltaUnwrapped;
        arch.unMapPages(space, region.descriptor.baseAddr + pageOffset * pageSize, pageCount, UnmapPagesFlags.fromFlag(.free), 0, null);

        return true;
    }
    pub fn findAndPin(space: *AddressSpace, addr: u64, size: u64) callconv(.C) ?*Region {
        if (@as(u64, @addWithOverflow(addr, size))) {
            return null;
        }

        _ = space.reserveMutex.acquire();
        defer space.reserveMutex.release();

        const region = space.findRegion(addr) orelse return null;

        if (region.descriptor.baseAddr > addr) return null;
        if (region.descriptor.baseAddr + region.descriptor.pageCount * pageSize < addr + size) return null;
        if (!region.data.pin.takeEx(WriterLock.shared, true)) return null;

        return region;
    }

    pub fn unpinRegion(space: *AddressSpace, region: *Region) callconv(.C) void {
        _ = space.reserveMutex.acquire();
        defer space.reserveMutex.release();
        region.data.pin.returnLock(WriterLock.shared);
    }

    pub fn destroy(space: *Self) callconv(.C) void {
        var maybeNode = space.usedRegionsNonGuard.first;
        while (maybeNode) |node| {
            const region = node.value.?;
            maybeNode = node.next;
            _ = space.free(region.descriptor.baseAddr, 0, false);
        }

        while (space.freeRegionBase.find(0, .SmallestAboveOrEqual)) |node| {
            space.freeRegionBase.remove(&node.value.?.u.item.base);
            space.freeRegionSize.remove(&node.value.?.u.item.u.size);
            EsHeapFree(@intFromPtr(node.value), @sizeOf(Region), &kernel.heapCore);
        }

        arch.freeAddressSpace(space);
    }
    pub fn unreserve(space: *AddressSpace, region: *Region, unmapPages: bool, isGuardRegion: bool) callconv(.C) void {
        space.reserveMutex.assertLocked();

        if (kernel.physicalMemoryManager.nextRegionToBalance == region) {
            kernel.physicalMemoryManager.nextRegionToBalance = region.u.item.u.nonGuard.next orelse null;
            kernel.physicalMemoryManager.balanceResumePosition = 0;
        }

        if (region.flags.contains(.normal)) {
            handleNormalRegion(space, region);
        } else if (region.flags.contains(.guard) and !isGuardRegion) {
            std.debug.panic("Cannot unreserve guard region", .{});
            return;
        }

        if (region.u.item.u.nonGuard.list != null and !isGuardRegion) {
            region.u.item.u.nonGuard.removeFromList();
        }

        if (unmapPages) {
            unmapRegionPages(space, region);
        }

        space.reserveCount += region.descriptor.pageCount;

        if (space == &kernel.coreAddressSpace) {
            handleCoreRegion(space, region);
        } else {
            handleNonCoreRegion(space, region);
        }
    }

    fn handleNormalRegion(space: *AddressSpace, region: *Region) void {
        if (region.data.u.normal.guardBefore) |before| space.unreserve(before, false, true);
        if (region.data.u.normal.guardAfter) |after| space.unreserve(after, false, true);
    }

    fn unmapRegionPages(space: *AddressSpace, region: *Region) void {
        _ = arch.unMapPages(space, region.descriptor.baseAddr, region.descriptor.pageCount, UnmapPagesFlags.empty(), 0, null);
    }

    fn handleCoreRegion(region: *Region) void {
        region.u.core.used = false;

        var remove1: i64 = -1;
        var remove2: i64 = -1;

        for (kernel.mmCoreRegions[0..kernel.mmCoreRegionCount], 0..) |*r, i| {
            if (!(remove1 != -1 or remove2 != 1)) break;
            if (r.u.core.used) continue;
            if (r == region) continue;

            if (r.descriptor.baseAddr == region.descriptor.baseAddr + (region.descriptor.pageCount << pageBitCount)) {
                region.descriptor.pageCount += r.descriptor.pageCount;
                remove1 = @as(i64, @intCast(i));
            } else if (region.descriptor.baseAddr == r.descriptor.baseAddr + (r.descriptor.pageCount << pageBitCount)) {
                region.descriptor.pageCount += r.descriptor.pageCount;
                region.descriptor.baseAddr = r.descriptor.baseAddr;
                remove2 = @as(i64, @intCast(i));
            }
        }

        removeCoreRegion(remove1, remove2);
    }

    fn removeCoreRegion(remove1: i64, remove2: i64) void {
        if (remove1 != -1) {
            kernel.mmCoreRegionCount -= 1;
            kernel.mmCoreRegions[@as(u64, @intCast(remove1))] = kernel.mmCoreRegions[kernel.mmCoreRegionCount];
            if (remove2 == @as(i64, @intCast(kernel.mmCoreRegionCount))) remove2 = remove1;
        }

        if (remove2 != -1) {
            kernel.mmCoreRegionCount -= 1;
            kernel.mmCoreRegions[@as(u64, @intCast(remove2))] = kernel.mmCoreRegions[kernel.mmCoreRegionCount];
        }
    }

    fn handleNonCoreRegion(space: *AddressSpace, region: *Region) void {
        space.usedRegions.remove(&region.u.item.base);
        const address = region.descriptor.baseAddr;

        mergeWithAdjacentFreeRegion(space, region, address);
        _ = space.freeRegionBase.insert(&region.u.item.base, region, region.descriptor.baseAddr, .panic);
        _ = space.freeRegionSize.insert(&region.u.item.u.size, region, region.descriptor.pageCount, .allow);
    }

    fn mergeWithAdjacentFreeRegion(space: *AddressSpace, region: *Region, address: u64) void {
        if (space.freeRegionBase.find(address, .LargestBelowOrEqual)) |before| {
            if (before.value.?.descriptor.baseAddr + before.value.?.descriptor.pageCount * pageSize == region.descriptor.baseAddr) {
                region.descriptor.baseAddr = before.value.?.descriptor.baseAddr;
                region.descriptor.pageCount += before.value.?.descriptor.pageCount;
                space.freeRegionBase.remove(before);
                space.freeRegionSize.remove(&before.value.?.u.item.u.size);
                EsHeapFree(@intFromPtr(before.value), @sizeOf(Region), &kernel.heapCore);
            }
        }

        if (space.freeRegionBase.find(address, .SmallestAboveOrEqual)) |after| {
            if (region.descriptor.baseAddr + region.descriptor.pageCount * pageSize == after.value.?.descriptor.baseAddr) {
                region.descriptor.pageCount += after.value.?.descriptor.pageCount;
                space.freeRegionBase.remove(after);
                space.freeRegionSize.remove(&after.value.?.u.item.u.size);
                EsHeapFree(@intFromPtr(after.value), @sizeOf(Region), &kernel.heapCore);
            }
        }
    }

    pub fn reserve(space: *AddressSpace, byteCount: u64, flags: Region.Flags, forcedAddr: u64) callconv(.C) ?*Region {
        const requiredPageCount = ((byteCount + pageSize - 1) & ~@as(u64, pageSize - 1)) / pageSize;
        if (requiredPageCount == 0) return null;

        space.reserveMutex.assertLocked();
        const guardPageCount = 1; // Number of guard pages to allocate on each side
        const region = blk: {
            if (space == &kernel.coreAddressSpace) {
                if (kernel.mmCoreRegionCount == arch.coreMemRegionCount) return null;

                if (forcedAddr != 0) std.debug.panic("Using a forced address in core address space\n", .{});

                {
                    const newRegionCount = kernel.mmCoreRegionCount + 1;
                    const requiredCommitPgaeCount = newRegionCount * @sizeOf(Region) / pageSize;

                    while (kernel.mmCoreRegionArrayCommit < requiredCommitPgaeCount) : (kernel.mmCoreRegionArrayCommit += 1) {
                        if (!commit(pageSize, true)) return null;
                    }
                }

                for (kernel.mmCoreRegions[0..kernel.mmCoreRegionCount]) |*region| {
                    if (!region.u.core.used and region.descriptor.pageCount >= requiredPageCount) {
                        if (region.descriptor.pageCount > requiredPageCount) {
                            const last = kernel.mmCoreRegionCount;
                            kernel.mmCoreRegionCount += 1;
                            var split = &kernel.mmCoreRegions[last];
                            split.* = region.*;
                            split.descriptor.baseAddr += requiredPageCount * pageSize;
                            split.descriptor.pageCount -= requiredPageCount;
                        }

                        region.u.core.used = true;
                        region.descriptor.pageCount = requiredPageCount;
                        region.flags = flags;
                        region.data = kernel.zeroes(@TypeOf(region.data));

                        break :blk region;
                    }
                }

                return null;
            } else if (forcedAddr != 0) {
                if (space.usedRegions.find(forcedAddr, .Exact)) |_| return null;

                if (space.usedRegions.find(forcedAddr, .SmallestAboveOrEqual)) |item| {
                    if (item.value.?.descriptor.baseAddr < forcedAddr + (requiredPageCount + 2 * guardPageCount) * pageSize) return null;
                }

                if (space.usedRegions.find(forcedAddr + (requiredPageCount + 2 * guardPageCount) * pageSize - 1, .LargestBelowOrEqual)) |item| {
                    if (item.value.?.descriptor.baseAddr + item.value.?.descriptor.pageCount * pageSize > forcedAddr) return null;
                }

                if (space.freeRegionBase.find(forcedAddr, .Exact)) |_| return null;

                if (space.freeRegionBase.find(forcedAddr, .SmallestAboveOrEqual)) |item| {
                    if (item.value.?.descriptor.baseAddr < forcedAddr + (requiredPageCount + 2 * guardPageCount) * pageSize) return null;
                }

                if (space.freeRegionBase.find(forcedAddr + (requiredPageCount + 2 * guardPageCount) * pageSize - 1, .LargestBelowOrEqual)) |item| {
                    if (item.value.?.descriptor.baseAddr + item.value.?.descriptor.pageCount * pageSize > forcedAddr) return null;
                }

                const region = @as(?*Region, @ptrFromInt(EsHeapAllocate(@sizeOf(Region), true, &kernel.heapCore))) orelse unreachable;
                region.descriptor.baseAddr = forcedAddr + guardPageCount * pageSize;
                region.descriptor.pageCount = requiredPageCount;
                region.flags = flags;

                _ = space.usedRegions.insert(&region.u.item.base, region, region.descriptor.baseAddr, .panic);

                region.data = kernel.zeroes(@TypeOf(region.data));
                break :blk region;
            } else {
                if (space.freeRegionSize.find(requiredPageCount + 2 * guardPageCount, .SmallestAboveOrEqual)) |item| {
                    const region = item.value.?;
                    space.freeRegionBase.remove(&region.u.item.base);
                    space.freeRegionSize.remove(&region.u.item.u.size);

                    if (region.descriptor.pageCount > requiredPageCount + 2 * guardPageCount) {
                        const split = @as(?*Region, @ptrFromInt(EsHeapAllocate(@sizeOf(Region), true, &kernel.heapCore))) orelse unreachable;
                        split.* = region.*;

                        split.descriptor.baseAddr += (requiredPageCount + 2 * guardPageCount) * pageSize;
                        split.descriptor.pageCount -= requiredPageCount + 2 * guardPageCount;

                        space.freeRegionBase.insert(&split.u.item.base, split, split.descriptor.baseAddr, .panic);
                        space.freeRegionSize.insert(&split.u.item.u.size, split, split.descriptor.pageCount, .panic);
                    }

                    region.descriptor.pageCount = requiredPageCount;
                    region.descriptor.baseAddr += guardPageCount * pageSize;

                    _ = space.usedRegions.insert(&region.u.item.base, region, region.descriptor.baseAddr, .panic);

                    region.data = kernel.zeroes(@TypeOf(region.data));
                    break :blk region;
                } else {
                    return null;
                }
            }
        };

        if (!arch.commitPageTables(space, region)) {
            space.unreserve(region, false, false);
            return null;
        }

        if (space != &kernel.coreAddressSpace) {
            region.u.item.u.nonGuard = kernel.zeroes(@TypeOf(region.u.item.u.nonGuard));
            region.u.item.u.nonGuard.value = region;
            space.usedRegionsNonGuard.append(&region.u.item.u.nonGuard);
        }

        space.reserveCount += requiredPageCount;

        return region;
    }

    pub fn mapPhysical(space: *AddressSpace, askedOffset: u64, askedByteCount: u64, caching: Region.Flags) callconv(.C) u64 {
        const offset2 = askedOffset & (pageSize - 1);
        const offset = askedOffset - offset2;
        const byteCount = if (offset2 != 0) askedByteCount + pageSize else askedByteCount;

        const region = blk: {
            _ = space.reserveMutex.acquire();
            defer space.reserveMutex.release();

            const result = space.reserve(byteCount, caching.orFlag(.fixed).orFlag(.physical), 0) orelse return 0;
            result.data.u.physical.offset = offset;
            break :blk result;
        };

        var i: u64 = 0;
        while (i < region.descriptor.pageCount) : (i += 1) {
            _ = space.handlePageFault(region.descriptor.baseAddr + i * pageSize, HandlePageFaultFlags.empty());
        }

        return region.descriptor.baseAddr + offset2;
    }
};

fn CloseReferenceTask(task: *AsyncTask) void {
    const space = @as(AddressSpace, @fieldParentPtr("removeAsyncTask", task));
    kernel.scheduler.addrSpacePool.remove(@intFromPtr(space));
}
pub fn decommit(byteCount: u64, fixed: bool) callconv(.C) void {
    if (byteCount & (pageSize - 1) != 0) {
        std.debug.panic("byte count not page-aligned", .{});
    }

    const requiredPageCount = @as(i64, @intCast(byteCount / pageSize));
    _ = kernel.physicalMemoryManager.commitMutex.acquire();
    defer kernel.physicalMemoryManager.commitMutex.release();

    decommitPages(fixed, requiredPageCount);
}

fn decommitPages(fixed: bool, requiredPageCount: i64) void {
    if (fixed) {
        ensureSufficientPages(kernel.physicalMemoryManager.commitFixed, requiredPageCount, "decommitted too many fixed pages");
        kernel.physicalMemoryManager.commitFixed -= requiredPageCount;
    } else {
        ensureSufficientPages(kernel.physicalMemoryManager.commitPageable, requiredPageCount, "decommitted too many pageable pages");
        kernel.physicalMemoryManager.commitPageable -= requiredPageCount;
    }
}

fn ensureSufficientPages(current: i64, required: i64, errorMessage: []const u8) void {
    if (current < required) {
        std.debug.panic(errorMessage, .{});
    }
}

pub const MapPageFlags = Bitflag(enum(u32) {
    notCacheable = 0,
    user = 1,
    overwrite = 2,
    commitTablesNow = 3,
    readOnly = 4,
    copied = 5,
    noNewTables = 6,
    frameLockAquired = 7,
    writeCombining = 8,
    ignoreIfMapped = 9,
});

pub const UnmapPagesFlags = Bitflag(enum(u32) {
    free = 0,
    freeCopied = 1,
    balanceFile = 2,
});

pub const PageFrame = extern struct {
    state: Volatile(PageFrame.State),
    flags: Volatile(u8),
    cacheRef: ?*volatile u8,
    u: extern union {
        list: extern union {
            next: Volatile(u64),
            prev: ?*volatile u64,
        },
        active: extern struct {
            refs: Volatile(u64),
        },
    },
    pub const State = enum(i8) {
        unusable,
        bad,
        zeroed,
        free,
        standby,
        active,
        modified,
        modifiedNoWrite,
        modifiedNoWriteNoRead,
        modifiedNoRead,
        modifiedNoReadNoWrite,
        modifiedNoReadNoWriteNoExecute,
        modifiedNoReadNoExecute,
        modifiedNoWriteNoExecute,
    };
};

const ObjectCache = extern struct {
    lock: SpinLock,
    items: List,
    count: u64,
    trim: fn (cache: *ObjectCache) callconv(.C) bool,
    trimLock: WriterLock,
    node: LinkedList(ObjectCache).Node,
    averageObjBytes: u64,

    const trimGroupeCount = 1024;
};
pub const Physical = extern struct {
    pub const Allocator = extern struct {
        pageFrames: [*]PageFrame,
        pageFrameDBInitialized: bool,
        pageFrameDBCount: u64,

        firstFreePage: u64,
        firstZeroedPage: u64,
        firstStandbyPage: u64,
        firstModifiedPage: u64,
        firstModifiedNoWritePage: u64,
        firstModifiedNoWriteNoReadPage: u64,
        firstModifiedNoReadPage: u64,
        firstModifiedNoReadNoWritePage: u64,
        firstModifiedNoReadNoWriteNoExecutePage: u64,
        firstModifiedNoReadNoExecutePage: u64,
        firstModifiedNoWriteNoExecutePage: u64,
        freeOrZeroedPageBitset: kernel.ds.BitSet,
        freePageCount: u64,
        activePageCount: u64,
        zeroedPageCount: u64,
        standbyPageCount: u64,
        modifiedPageCount: u64,
        modifiedNoWritePageCount: u64,
        modifiedNoWriteNoReadPageCount: u64,
        modifiedNoReadPageCount: u64,
        modifiedNoReadNoWritePageCount: u64,
        modifiedNoReadNoWriteNoExecutePageCount: u64,
        modifiedNoReadNoExecutePageCount: u64,
        modifiedNoWriteNoExecutePageCount: u64,
        commitFixed: i64,
        commitPageable: i64,
        commitFixedLimit: i64,
        commitLimit: i64,
        commitMutex: Mutex,
        pageFrameMutex: Mutex,
        lastStandbyPage: u64,
        manipulationLock: Mutex,
        manipulationProcLock: SpinLock,
        manipulationRegion: ?*Region,
        zeroPageThread: *Thread,
        zeroPageEvent: Event,

        objCacheList: LinkedList(ObjectCache),
        objCacheListMutex: Mutex,

        availableCriticalEvent: Event,
        availableLowEvent: Event,
        availableNormalEvent: Event,
        availableHighEvent: Event,

        approxTotalObjCacheByteCount: u64,
        trimObjCacheEvent: Event,
        nextProcToBalance: ?*Process,
        nextRegionToBalance: ?*Region,
        balanceResumePosition: u64,
        const Self = @This();
        const zeroPageThreshold = 16;

        inline fn getAvailablePageCount(self: Self) u64 {
            return self.zeroedPageCount + self.freePageCount + self.standbyPageCount;
        }
        inline fn getRemainingCommit(self: Self) i64 {
            return self.commitLimit - self.commitPageable - self.commitedFixed;
        }

        inline fn shouldTrimObjCache(self: Self) bool {
            return self.approxTotalObjCacheByteCount / pageSize > self.getObjCacheMaxCachePageCount();
        }

        inline fn getObjCacheMaxCachePageCount(self: Self) i64 {
            return @divTrunc(self.commitedFixed - self.commitPageable - @as(i64, @intCast(self.approxTotalObjCacheByteCount)), pageSize);
        }

        inline fn getNonCacheMemPageCount(self: Self) i64 {
            return @divTrunc(self.commitedFixed - self.commitPageable - @as(i64, @intCast(self.approxTotalObjCacheByteCount)), pageSize);
        }

        fn updateAvailablePageCount(self: *Self, increase: bool) void {
            if (self.getAvailablePageCount() >= criticalAvailablePageThreshold) {
                _ = self.availableCriticalEvent.set(true);
                self.availableNormalEvent.reset();
            } else {
                self.availableNormalEvent.reset();
                _ = self.availableCriticalEvent.set(true);
                if (!increase) {
                    _ = self.availableLowEvent.set(true);
                }
            }
            if (self.getAvailablePageCount() >= lowAvailablePageThreshold) _ = self.availableLowEvent.reset() else _ = self.availableLowEvent.reset();
        }

        const criticalAvailablePageThreshold = 1048576 / pageSize;
        const lowAvailablePageThreshold = 16777216 / pageSize;
    };
    pub const Flags = Bitflag(enum(u32) {
        canFail = 0,
        commitNow = 1,
        zeroed = 2,
        lockAquired = 3,
    });
    pub const memoryManipulationRegionPageCount = 0x10;
};

pub const HeapRegion = extern struct {
    u1: extern union {
        next: u16,
        size: u16,
    },

    previous: u16,
    offset: u16,
    used: u16,

    u2: extern union {
        allocationSize: u64,
        regionListNext: ?*HeapRegion,
    },

    regionListRef: ?*?*@This(),

    const usedHeaderSize = @sizeOf(HeapRegion) - @sizeOf(?*?*HeapRegion);
    const freeHeaderSize = @sizeOf(HeapRegion);
    const usedMagic = 0xabcd;

    fn removeFree(self: *@This()) void {
        if (self.regionListRef == null or self.used != 0) std.debug.panic("Invalid free region", .{});

        self.regionListRef.?.* = self.u2.regionListNext;

        if (self.u2.regionListNext) |reg| {
            reg.regionListRef = self.regionListRef;
        }
        self.regionListRef = null;
    }

    fn getHeader(self: *@This()) ?*HeapRegion {
        return @as(?*HeapRegion, @ptrFromInt(@intFromPtr(self) - usedHeaderSize));
    }

    fn getData(self: *@This()) u64 {
        return @intFromPtr(self) + usedHeaderSize;
    }

    fn getNext(self: *@This()) ?*HeapRegion {
        return @as(?*HeapRegion, @ptrFromInt(@intFromPtr(self) - self.u1.next));
    }

    fn getPrev(self: *@This()) ?*HeapRegion {
        if (self.previous != 0) {
            return @as(?*HeapRegion, @ptrFromInt(@intFromPtr(self) - self.previous));
        } else {
            return null;
        }
    }
};

fn calcHeapIdx(size: u64) u64 {
    return @bitSizeOf(u32) - @clz(@as(u32, @truncate(size))) - 5;
}

pub const Heap = extern struct {
    mutex: Mutex,
    regions: [12]?*HeapRegion,
    allocationCount: Volatile(u64),
    size: Volatile(u64),
    blockCount: Volatile(u64),
    blocks: [16]?*HeapRegion,
    canValidate: bool,
    const largeAllocationThreshold = 32768;

    const Self = @This();

    pub fn alloc(self: *Self, askedSize: u64, isZero: bool) u64 {
        if (askedSize == 0) return 0;
        if (@as(i64, @bitCast(askedSize)) < 0) std.debug.panic("Invalid size", .{});

        const size = (askedSize + HeapRegion.usedHeaderSize + 0x1F) & ~@as(u64, 0x1F);

        if (size >= largeAllocationThreshold) {
            if (@as(?*HeapRegion, @ptrFromInt(self.allocCall(size)))) |region| {
                region.used = HeapRegion.usedMagic;
                region.u1.size = 0;
                region.u2.allocationSize = askedSize;
                _ = self.size.atomicFetchAdd(askedSize);
                return region.getData();
            } else {
                return 0;
            }
        }
        _ = self.mutex.acquire();
        self.validate();

        const region = regionBlk: {
            const heapIdx = calcHeapIdx(size);
            if (heapIdx < self.regions.len) {
                for (self.regions[heapIdx..]) |maybeRegion| {
                    if (maybeRegion) |heapRegion| {
                        if (heapRegion.u1.size >= size) {
                            const result = heapRegion;
                            result.removeFree();
                            break :regionBlk result;
                        }
                    }
                }
            }

            const allocation = @as(?*HeapRegion, @intFromPtr(self.allocCall(size)));
            if (self.blockCount.readVolatile() < 16) {
                self.blocks[self.blockCount.readVolatile()] = allocation;
            } else {
                self.canValidate = false;
            }

            self.blockCount.increment();
            if (allocation) |result| {
                result.u1.size = 65536 - 32;
                const endRegion = result.getNext().?;
                endRegion.used = HeapRegion.usedMagic;
                endRegion.offset = 65536 - 32;
                endRegion.u1.next = 32;
                @as(?*?*Heap, @ptrFromInt(endRegion.getData())).?.* = self;
                break :regionBlk result;
            } else {
                self.mutex.release();
                return 0;
            }
        };

        if (region.used != 0 or region.u1.size < size) std.debug.panic("Invalid region", .{});
        self.allocationCount.increment();
        _ = self.size.atomicFetchAdd(size);

        if (region.u1.size != size) {
            const oldSize = region.u1.size;
            const truncatedSize = @as(u16, @intCast(size));
            region.u1.size = truncatedSize;
            region.used = HeapRegion.usedMagic;

            const freeRegion = region.getNext().?;
            freeRegion.u1.size = oldSize - truncatedSize;
            freeRegion.previous = truncatedSize;
            freeRegion.offset = region.offset + truncatedSize;
            freeRegion.used = 0;
            self.addFreeRegion(freeRegion);
            const nextRegion = freeRegion.getNext().?;
            nextRegion.previous = freeRegion.u1.size;

            self.validate();
        }

        region.used = HeapRegion.usedMagic;
        region.u2.allocationSize = askedSize;
        self.mutex.release();

        const addr = region.getData();
        const mem = @as([*]u8, @ptrFromInt(addr[0..askedSize]));

        if (isZero) {
            @memset(mem, 0);
        } else {
            @memset(mem, 0xa1);
        }
        return addr;
    }

    fn callAddressSpace(
        self: *Self,
        size: u64,
        isAlloc: bool,
        region: ?*HeapRegion, // `null` for allocation, non-null for freeing
    ) u64 {
        const addressSpace = if (self == &kernel.heapCore)
            &kernel.coreAddressSpace
        else
            &kernel.addrSpace;

        if (isAlloc) {
            return addressSpace.alloc(size, Region.Flags.fromFlag(.fixed), 0, true);
        } else {
            if (region == null) std.debug.panic("Invalid region", .{});
            _ = addressSpace.free(@intFromPtr(region.?), 0, false);
            return 0;
        }
    }

    fn allocCall(self: *Self, size: u64) u64 {
        return self.callAddressSpace(size, true, null);
    }

    fn freeCall(self: *Self, region: *HeapRegion) void {
        _ = self.callAddressSpace(0, false, region);
    }

    fn addFreeRegion(self: *@This(), region: *HeapRegion) void {
        if (region.used != 0 or region.u1.size < 32) {
            std.debug.panic("heap panic", .{});
        }

        const idx = calcHeapIdx(region.u1.size);
        region.u2.regionListNext = self.regions[idx];
        if (region.u2.regionListNext) |regionLN| {
            regionLN.regionListRef = &region.u2.regionListNext;
        }
        self.regions[idx] = region;
        region.regionListRef = &self.regions[idx];
    }

    fn free(self: *Self, addr: u64, expectedSize: u64) void {
        if (addr == 0 and expectedSize != 0) std.debug.panic("Invalid free", .{});
        if (addr == 0) return;

        var region = @as(*HeapRegion, @ptrFromInt(addr)).getHeader().?;

        if (region.used != HeapRegion.usedMagic) std.debug.panic("Invalid free", .{});
        if (expectedSize != 0 and region.u2.allocationSize != expectedSize) std.debug.panic("Invalid free", .{});

        if (region.u1.size == 0) {
            _ = self.size.atomicFetchSub(region.u2.allocationSize);
            self.freeCall(region);
            return;
        }

        const firstRegion = @as(*HeapRegion, @ptrFromInt(@intFromPtr(region) - region.offset + 65536 - 32));
        if (@as(**Heap, @ptrFromInt(firstRegion.getData())).* != self) {
            std.debug.panic("Heap mismatch: the first region's data does not point to the expected heap", .{});
        }

        _ = self.mutex.acquire();
        self.validate();

        region.used = 0;
        if (region.offset < region.previous) std.debug.panic("Region offset is less than the previous region offset", .{});

        self.allocationCount.decrement();
        _ = self.size.atomicFetchSub(region.u1.size);

        if (region.getNext()) |nextRegion| {
            if (nextRegion.used == 0) {
                region.removeFree();
                region.u1.size += nextRegion.u1.size;
                nextRegion.getNext().?.previous = region.u1.size;
            }
        }

        if (region.getPrev()) |prevRegion| {
            if (prevRegion.used == 0) {
                prevRegion.removeFree();
                prevRegion.u1.size += region.u1.size;
                region.getNext().?.previous = prevRegion.u1.size;
                region = prevRegion;
            }
        }

        if (region.u1.size == 65536 - 32) {
            if (region.offset != 0) std.debug.panic("Invalid region offset", .{});
            self.blockCount.decrement();

            if (self.canValidate) {
                var found = false;

                for (self.blocks[0 .. self.blockCount.readVolatile() + 1]) |*heapRegion| {
                    if (heapRegion.* == region) {
                        heapRegion.* = self.blocks[self.blockCount.readVolatile()];
                        found = true;
                        break;
                    } else {
                        std.debug.panic("Invalid block", .{});
                    }
                }
            }
            self.freeCall(region);
            self.mutex.release();
            return;
        }

        self.addFreeRegion(region);
        self.validate();
        self.mutex.release();
    }

    //Todo?: should I keep this version
    // fn free(self: *Self, addr: u64, expectedSize: u64) void {
    //     if (addr == 0 and expectedSize != 0) std.debug.panic("Invalid free", .{});
    //     if (addr == 0) return;

    //     var region = @as(*HeapRegion, @ptrFromInt(addr)).getHeader().?;
    //     validateRegionHeader(region, expectedSize);

    //     if (region.u1.size == 0) {
    //         self.handleFreeZeroSizeRegion(region);
    //         return;
    //     }

    //     const firstRegion = getFirstRegion(region);
    //     self.validateHeap(firstRegion);

    //     _ = self.mutex.acquire();
    //     self.validate();

    //     region.used = 0;

    //     handleRegionOffsets(region);

    //     self.allocationCount.decrement();
    //     _ = self.size.atomicFetchSub(region.u1.size);

    //     mergeAdjacentFreeRegions(region);
    //     self.handleFullRegion(region);
    //     self.mutex.release();
    // }

    // fn validateRegionHeader(region: *HeapRegion, expectedSize: u64) void {
    //     if (region.used != HeapRegion.usedMagic) std.debug.panic("Invalid free", .{});
    //     if (expectedSize != 0 and region.u2.allocationSize != expectedSize) std.debug.panic("Invalid free", .{});
    // }

    // fn handleFreeZeroSizeRegion(self: *Self, region: *HeapRegion) void {
    //     _ = self.size.atomicFetchSub(region.u2.allocationSize);
    //     self.freeCall(region);
    // }

    // fn getFirstRegion(region: *HeapRegion) *HeapRegion {
    //     return @as(*HeapRegion, @ptrFromInt(@intFromPtr(region) - region.offset + 65536 - 32));
    // }

    // fn validateHeap(self: Self, firstRegion: *HeapRegion) void {
    //     if (@as(**Heap, @ptrFromInt(firstRegion.getData())).* != self) {
    //         std.debug.panic("Heap mismatch: the first region's data does not point to the expected heap", .{});
    //     }
    // }

    // fn handleRegionOffsets(region: *HeapRegion) void {
    //     if (region.offset < region.previous) std.debug.panic("Region offset is less than the previous region offset", .{});
    // }

    // fn mergeAdjacentFreeRegions(region: *HeapRegion) void {
    //     if (region.getNext()) |nextRegion| {
    //         if (nextRegion.used == 0) {
    //             region.removeFree();
    //             region.u1.size += nextRegion.u1.size;
    //             nextRegion.getNext().?.previous = region.u1.size;
    //         }
    //     }

    //     if (region.getPrev()) |prevRegion| {
    //         if (prevRegion.used == 0) {
    //             prevRegion.removeFree();
    //             prevRegion.u1.size += region.u1.size;
    //             region.getNext().?.previous = prevRegion.u1.size;
    //             region = prevRegion;
    //         }
    //     }
    // }

    // fn handleFullRegion(self: *Self, region: *HeapRegion) void {
    //     if (region.u1.size == 65536 - 32) {
    //         if (region.offset != 0) std.debug.panic("Invalid region offset", .{});
    //         self.blockCount.decrement();

    //         if (self.canValidate) {
    //             var found = false;

    //             for (self.blocks[0 .. self.blockCount.readVolatile() + 1]) |*heapRegion| {
    //                 if (heapRegion.* == region) {
    //                     heapRegion.* = self.blocks[self.blockCount.readVolatile()];
    //                     found = true;
    //                     break;
    //                 } else {
    //                     std.debug.panic("Invalid block", .{});
    //                 }
    //             }
    //         }
    //         self.freeCall(region);
    //     } else {
    //         self.addFreeRegion(region);
    //         self.validate();
    //     }
    // }

    fn validate(self: *@This()) void {
        if (!self.canValidate) return;

        for (self.blocks[0..self.blockCount.readVolatile()], 0..) |maybeStart, i| {
            if (!maybeStart) continue;

            const start = maybeStart;
            const end = @intFromPtr(@as(*HeapRegion, @ptrFromInt(self.blocks[i])) + 65536);
            var maybePrev: ?*HeapRegion = null;
            var region = start;

            while (@intFromPtr(region) < @intFromPtr(end)) {
                validateRegion(start, region, maybePrev);

                maybePrev = region;
                region = region.getNext().?;
            }

            if (region != end) {
                std.debug.panic("heap panic: region does not match expected end", .{});
            }
        }
    }

    fn validateRegion(start: *HeapRegion, region: *HeapRegion, maybePrev: ?*HeapRegion) void {
        if (maybePrev) |previous| {
            if (@intFromPtr(previous) != @intFromPtr(region.getPrev())) {
                std.debug.panic("heap panic: previous region mismatch", .{});
            }
        } else {
            if (region.previous != 0) {
                std.debug.panic("heap panic: invalid previous pointer for first region", .{});
            }
        }

        if (region.u1.size & 31 != 0) {
            std.debug.panic("heap panic: size misalignment", .{});
        }

        if (@intFromPtr(region) - @intFromPtr(start) != region.offset) {
            std.debug.panic("heap panic: offset mismatch", .{});
        }

        if (region.used != HeapRegion.usedMagic and region.used != 0) {
            std.debug.panic("heap panic: invalid usage flag", .{});
        }

        if (region.used == 0 and region.regionListRef == null) {
            std.debug.panic("heap panic: invalid region list reference for free region", .{});
        }

        if (region.used == 0 and region.u2.regionListNext != null and
            region.u2.regionListNext.?.regionListRef != &region.u2.regionListNext)
        {
            std.debug.panic("heap panic: invalid region list linkage", .{});
        }
    }
};

export fn PMZero(askedPagePtr: [*]u64, askedPageCount: u64, isContiguous: bool) callconv(.C) void {
    _ = kernel.physicalMemoryManager.manipulationLock.acquire();

    var pageCount = askedPageCount;
    var pages = askedPagePtr;

    while (true) {
        const pagesToProcess = if (pageCount > Physical.memoryManipulationRegionPageCount) Physical.memoryManipulationRegionPageCount else pageCount;
        pageCount -= pagesToProcess;

        const region = @intFromPtr(kernel.physicalMemoryManager.manipulationRegion);
        var i: u64 = 0;
        while (i < pagesToProcess) : (i += 1) {
            _ = arch.mapPage(&kernel.coreAddressSpace, if (isContiguous) pages[0] + (i << pageBitCount) else pages[i], region + pageSize * i, MapPageFlags.fromFlags(.{ .overwrite, .noNewTables }));
        }

        kernel.physicalMemoryManager.manipulationProcLock.acquire();

        i = 0;
        while (i < pagesToProcess) : (i += 1) {
            arch.invalidatePage(region + i * pageSize);
        }
        kernel.EsMemoryZero(region, pagesToProcess * pageSize);
        kernel.physicalMemoryManager.manipulationProcLock.release();

        if (pageCount != 0) {
            if (!isContiguous) pages = @as([*]u64, @ptrFromInt(@intFromPtr(pages) + Physical.memoryManipulationRegionPageCount));
        } else break;
    }

    kernel.physicalMemoryManager.manipulationLock.release();
}

pub fn physicalAllocFlagged(flags: Physical.Flags) u64 {
    return physicalAlloc(flags, 1, 1, 0);
}

export var earlyZeroBuffer: [pageSize]u8 align(pageSize) = undefined;

export fn physicalAlloc(flags: Physical.Flags, count: u64, alignment: u64, below: u64) callconv(.C) u64 {
    const mutexAlreadyAquired = flags.contains(.lockAquired);
    if (!mutexAlreadyAquired) _ = kernel.physicalMemoryManager.pageFrameMutex.acquire() else kernel.physicalMemoryManager.pageFrameMutex.assertLocked();
    defer if (!mutexAlreadyAquired) kernel.physicalMemoryManager.pageFrameMutex.release();

    var commitNow = @as(i64, @intCast(count * pageSize));

    if (flags.contains(.commitNow)) {
        if (!commit(@as(u64, @intCast(commitNow)), true)) return 0;
    } else commitNow = 0;

    const simple = count == 1 and alignment == 1 and below == 0;

    if (!kernel.physicalMemoryManager.pageFrameDBInitialized) {
        if (!simple) std.debug.panic("non-simple allocation before initialization of the pageframe database", .{});
        const page = arch.EarlyAllocPage();
        if (flags.contains(.zeroed)) {
            _ = arch.mapPage(&kernel.coreAddressSpace, page, @intFromPtr(&earlyZeroBuffer), MapPageFlags.fromFlags(.{ .overwrite, .noNewTables, .frameLockAquired }));
            earlyZeroBuffer = kernel.zeroes(@TypeOf(earlyZeroBuffer));
        }

        return page;
    } else if (!simple) {
        const pages = kernel.physicalMemoryManager.freeOrZeroedPageBitset.get(count, alignment, below);
        if (pages != std.math.maxInt(u64)) {
            MMPhysicalActivatePages(pages, count);
            var address = pages << pageBitCount;
            if (flags.contains(.zeroed)) PMZero(@as([*]u64, @ptrCast(&address)), count, true); //todo!: implement PMZero
            return address;
        }
    } else {
        var notZeroed = false;
        var page = kernel.physicalMemoryManager.firstZeroedPage;

        if (page == 0) {
            page = kernel.physicalMemoryManager.firstFreePage;
            notZeroed = true;
        }

        if (page == 0) {
            page = kernel.physicalMemoryManager.lastStandbyPage;
            notZeroed = true;
        }

        if (page != 0) {
            const frame = &kernel.physicalMemoryManager.pageFrames[page];

            switch (frame.state.readVolatile()) {
                .active => std.debug.panic("corrupt page frame database", .{}),
                .standby => {
                    if (frame.cacheRef.?.* != ((page << pageBitCount) | 1)) {
                        std.debug.panic("corrupt shared reference back pointer in frame", .{});
                    }

                    frame.cacheRef.?.* = 0;
                },
                .modified => {
                    if (frame.cacheRef.?.* != ((page << pageBitCount) | 1)) {
                        std.debug.panic("corrupt shared reference back pointer in frame", .{});
                    }

                    frame.cacheRef.?.* = 0;
                },
                .modifiedNoRead => {
                    if (frame.cacheRef.?.* != ((page << pageBitCount) | 1)) {
                        std.debug.panic("corrupt shared reference back pointer in frame", .{});
                    }

                    frame.cacheRef.?.* = 0;
                },
                .modifiedNoReadNoExecute => {
                    if (frame.cacheRef.?.* != ((page << pageBitCount) | 1)) {
                        std.debug.panic("corrupt shared reference back pointer in frame", .{});
                    }

                    frame.cacheRef.?.* = 0;
                },
                .bad => {
                    std.debug.panic("bad page frame", .{});
                },
                .unusable => {
                    std.debug.panic("unusable page frame", .{});
                },
                .free => {
                    if (kernel.physicalMemoryManager.freePageCount == 0) {
                        std.debug.panic("freePageCount underflow", .{});
                    }
                    _ = kernel.physicalMemoryManager.freePageCount.atomicSub(1, .SeqCst);
                },
                else => {
                    kernel.physicalMemoryManager.freeOrZeroedPageBitset.take(page);
                },
            }

            MMPhysicalActivatePages(page, 1);

            var address = page << pageBitCount;
            if (notZeroed and flags.contains(.zeroed)) PMZero(@as([*]u64, @ptrCast(&address)), 1, false);

            return address;
        }
    }

    if (!flags.contains(.canFail)) {
        std.debug.panic("out of memory", .{});
    }

    decommit(@as(u64, @intCast(commitNow)), true);
    return 0;
}

pub export fn commit(byteCount: u64, fixed: bool) callconv(.C) bool {
    if (byteCount & (pageSize - 1) != 0) std.debug.panic("Bytes should be page-aligned", .{});

    const requiredPageCount = @as(i64, @intCast(byteCount / pageSize));

    _ = kernel.physicalMemoryManager.commitMutex.acquire();
    defer kernel.physicalMemoryManager.commitMutex.release();

    if (kernel.physicalMemoryManager.commitLimit != 0) {
        if (fixed) {
            if (requiredPageCount > kernel.physicalMemoryManager.commitFixedLimit - kernel.physicalMemoryManager.commitFixed) return false;
            if (@as(i64, @intCast(kernel.physicalMemoryManager.getAvailablePageCount())) - requiredPageCount < Physical.Allocator.criticalAvailablePageThreshold and !arch.getCurrentThread().?.isPageGenThread) return false;
            kernel.physicalMemoryManager.commitFixed += requiredPageCount;
        } else {
            if (requiredPageCount > kernel.physicalMemoryManager.getRemainingCommit() - if (arch.getCurrentThread().?.isPageGenThread) @as(i64, 0) else @as(i64, Physical.Allocator.criticalAvailablePageThreshold)) return false;
            kernel.physicalMemoryManager.commitPageable += requiredPageCount;
        }

        if (kernel.physicalMemoryManager.shouldTrimObjCache()) _ = kernel.physicalMemoryManager.trimObjCacheEvent.set(true);
    }

    return true;
}

export fn MMPhysicalActivatePages(pages: u64, count: u64) callconv(.C) void {
    kernel.physicalMemoryManager.pageFrameMutex.assertLocked();

    for (kernel.physicalMemoryManager.pageFrames[pages .. pages + count], 0..) |*frame, i| {
        switch (frame.state.readVolatile()) {
            .free => kernel.physicalMemoryManager.freePageCount -= 1,
            .zeroed => kernel.physicalMemoryManager.zeroedPageCount -= 1,
            .standby => {
                kernel.physicalMemoryManager.standbyPageCount -= 1;

                if (kernel.physicalMemoryManager.lastStandbyPage == pages + i) {
                    if (frame.u.list.prev == &kernel.physicalMemoryManager.firstStandbyPage) {
                        kernel.physicalMemoryManager.lastStandbyPage = 0;
                    } else {
                        kernel.physicalMemoryManager.lastStandbyPage = (@intFromPtr(frame.u.list.prev) - @intFromPtr(kernel.physicalMemoryManager.pageFrames)) / @sizeOf(PageFrame);
                    }
                }
            },
            .modified => kernel.physicalMemoryManager.modifiedPageCount -= 1,
            .modifiedNoWrite => kernel.physicalMemoryManager.modifiedNoWritePageCount -= 1,
            .modifiedNoWriteNoRead => kernel.physicalMemoryManager.modifiedNoWriteNoReadPageCount -= 1,
            .modifiedNoRead => kernel.physicalMemoryManager.modifiedNoReadPageCount -= 1,
            .modifiedNoReadNoWrite => kernel.physicalMemoryManager.modifiedNoReadNoWritePageCount -= 1,
            .modifiedNoReadNoWriteNoExecute => kernel.physicalMemoryManager.modifiedNoReadNoWriteNoExecutePageCount -= 1,
            .modifiedNoReadNoExecute => kernel.physicalMemoryManager.modifiedNoReadNoExecutePageCount -= 1,
            .modifiedNoWriteNoExecute => kernel.physicalMemoryManager.modifiedNoWriteNoExecutePageCount -= 1,
            else => std.debug.panic("Corrupt page frame database", .{}),
        }

        frame.u.list.prev.?.* = frame.u.list.next.readVolatile();

        if (frame.u.list.next.readVolatile() != 0) {
            kernel.physicalMemoryManager.pageFrames[frame.u.list.next.readVolatile()].u.list.prev = frame.u.list.prev;
        }
        kernel.EsMemoryZero(@intFromPtr(frame), @sizeOf(PageFrame));
        frame.state.writeVolatile(.active);
    }

    kernel.physicalMemoryManager.activePageCount += count;
    MMUpdateAvailablePageCount(false);
}

export fn MMUpdateAvailablePageCount(increase: bool) callconv(.C) void {
    if (kernel.physicalMemoryManager.getAvailablePageCount() >= Physical.Allocator.criticalAvailablePageThreshold) {
        _ = kernel.physicalMemoryManager.availableNormalEvent.set(true);
        kernel.physicalMemoryManager.availableCriticalEvent.reset();
    } else {
        kernel.physicalMemoryManager.availableNormalEvent.reset();
        _ = kernel.physicalMemoryManager.availableCriticalEvent.set(true);

        if (!increase) {
            std.debug.panic("decreased available page count", .{});
        }
    }

    if (kernel.physicalMemoryManager.getAvailablePageCount() >= Physical.Allocator.lowAvailablePageThreshold) {
        kernel.physicalMemoryManager.availableLowEvent.reset();
    } else {
        _ = kernel.physicalMemoryManager.availableLowEvent.set(true);
    }
}

pub fn physicalFree(askedPage: u64, mtxAlreadyAquired: bool, count: u64) callconv(.C) void {
    if (askedPage == 0) std.debug.panic("invalid page", .{});

    if (mtxAlreadyAquired) kernel.physicalMemoryManager.pageFrameMutex.assertLocked() else _ = kernel.physicalMemoryManager.pageFrameMutex.acquire();
    if (!kernel.physicalMemoryManager.pageFrameDBInitialized) std.debug.panic("PMM not yet initialized", .{});

    const page = askedPage >> pageBitCount;

    MMPhysicalInsertFreePagesStart();

    for (kernel.physicalMemoryManager.pageFrames[0..count]) |*frame| {
        if (frame.state.readVolatile() == .free) std.debug.panic("attempting to free a free page", .{});
        if (kernel.physicalMemoryManager.commitFixedLimit != 0) kernel.physicalMemoryManager.activePageCount -= 1;
        physicalInsertFreePagesNext(page);
    }

    MMPhysicalInsertFreePagesEnd();

    if (!mtxAlreadyAquired) kernel.physicalMemoryManager.pageFrameMutex.release();
}

export fn MMPhysicalInsertFreePagesStart() callconv(.C) void {}

pub fn physicalInsertFreePagesNext(page: u64) callconv(.C) void {
    const frame = &kernel.physicalMemoryManager.pageFrames[page];
    frame.state.writeVolatile(.free);

    frame.u.list.next.writeVolatile(kernel.physicalMemoryManager.firstFreePage);
    frame.u.list.prev = &kernel.physicalMemoryManager.firstFreePage;
    if (kernel.physicalMemoryManager.firstFreePage != 0) kernel.physicalMemoryManager.pageFrames[kernel.physicalMemoryManager.firstFreePage].u.list.prev = &frame.u.list.next.value;
    kernel.physicalMemoryManager.firstFreePage = page;

    kernel.physicalMemoryManager.freeOrZeroedPageBitset.put(page);
    kernel.physicalMemoryManager.freePageCount += 1;
}

export fn MMPhysicalInsertFreePagesEnd() callconv(.C) void {
    if (kernel.physicalMemoryManager.freePageCount > Physical.Allocator.zeroPageThreshold) _ = kernel.physicalMemoryManager.zeroPageEvent.set(true);
    MMUpdateAvailablePageCount(true);
}

export fn EsHeapAllocate(size: u64, isZero: bool, heap: *Heap) callconv(.C) u64 {
    return heap.alloc(size, isZero);
}
