pub const LEN = 0x200;
pub const DiskIdentifierLen = 10;

pub const Offset = struct {
    pub const DiskIdentifier = 0x1B4;
    pub const kernelSize = DiskIdentifier - 4;
    pub const partitionTable = Offset.DiskIdentifier + DiskIdentifierLen;
};
