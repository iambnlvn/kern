const kernel = @import("kernel.zig");
const Volatile = kernel.Volatile;
const Mutex = kernel.sync.Mutex;
const WriterLock = kernel.sync.WriterLock;
const AVLTree = kernel.ds.AVLTree;
const LinkedList = kernel.ds.LinkedList;
const Bitflag = kernel.ds.Bitflag;

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
