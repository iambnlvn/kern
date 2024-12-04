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
    // removeAsyncTask: AsyncTask, //TODO!: implement AsyncTask

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
        // freeOrZeroedPageBitset: BitSet, //TODO!: implement BitSet
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
        //TODO!: implement allocator methods
    };
};
