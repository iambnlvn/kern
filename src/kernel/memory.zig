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
const AsyncTask = kernel.scheduling.AsyncTask;
const List = kernel.ds.List;
const arch = @import("./arch/x86_64.zig");
const scheduling = @import("scheduling.zig");
const Process = scheduling.Process;
const Thread = scheduling.Thread;
const pageSize = arch.pageSize;

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
                // commit: Range.Set, TODO: implement Range.Set
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
            base: AVLTree(Region).Item,
            u: extern union {
                size: AVLTree(Region).Item,
                nonGuard: LinkedList(Region).Item,
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
        wopyOnWrite = 4,
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
    arch: AddressSpace.Arch,
    freeRegionBase: AVLTree(Region),
    freeRegionSize: AVLTree(Region),
    freeRegionsNonGuard: LinkedList(Region),
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
};

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
        commitedFixed: i64,
        commitPageable: i64,
        commitFixedLimit: i64,
        commitLimit: i64,
        commitMutex: Mutex,
        pageFrameMutex: Mutex,

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
        _ = self.mutex.aquire();
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

    fn allocCall(self: *Self, size: u64) u64 {
        if (self == &kernel.heapCore) {
            return kernel.coreAddressSpace.alloc(size, Region.Flags.fromFlag(.fixed), 0, true);
        } else {
            return kernel.addrSpace.alloc(size, Region.Flags.fromFlag(.fixed), 0, true);
        }
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
