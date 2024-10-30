[bits 16]
[org 0x600]

start :
        cli
        xor ax,ax
        mov ds,ax
        mov es,ax
        mov fs,ax
        mov gs,ax
        mov ss,ax
        mov sp,0x7c00

        sti
        xor ax, ax
        int 0x10
        mov ax, 3
        int 0x10

        cld
        mov si, 0x7c00
        mov di, 0x600
        mov cx, 0x200
        rep movsb

jmp 0x0:findPartition


findPartition:
        mov byte [driveNumber], dl
        mov ah, 0x8
        xor di,di
        int 0x13

        mov si , diskReadFailure
        jc error
        and cx, 0x3f
        mov word [sectorCount], cx
        inc dh
        shr dx, 8
        mov word [headCount], dx
        mov si, invalidGeometryError
        or cx, cx
        jz error
        or dx, dx
        jz error

        mov bx, part1
        cmp byte [bx], 0x80
        je foundPartition
        or cx, cx

        mov bx, part2
        cmp byte [bx], 0x80
        je foundPartition
        or cx, cx

        mov bx, part3
        cmp byte [bx], 0x80
        je foundPartition
        or cx, cx

        mov bx, part4
        cmp byte [bx], 0x80
        je foundPartition
        or cx, cx

        mov si, noBootablePartitionError

foundPartition:

        push bx
        mov di,[bx + 8]
        mov bx, 0x7c00
        call loadSector
        
        mov dl, [driveNumber]
        pop si
        mov dh, 0x01
        mov eax, dword [kernelSize
        jmp 0x0:0x7c00]

loadSector:
        mov	ax,di
        xor	dx,dx
        div	word [sectorCount]
        xor	dx,dx
        div	word [headCount]
        push	dx
        mov	ch,al 
        shl	ah,6
        mov	cl,ah

        mov ax,di
        xor dx,dx
        div word [sectorCount]

        inc dx
        or cl, dl

        pop	dx
        mov	dh,dl
        mov	dl,[drive_number]
        mov	ax,0x0201
        int	0x13
        mov	si,diskReadFailure
        jc	error

        ret

error:
    .loop:
        lodsb
        or al, al
        jz .break
        mov ah, 0x0e
        int 0x10
        int 0x10
        jmp .loop
    .break:
cli,
hlt
cannotReadDiskError db "Cannot read disk", 0
invalidGeometryError db "Invalid geometry", 0
noBootablePartitionError db "No bootable partition found", 0
diskReadFailure db "Disk read failure", 0
driveNumber db 0
sectorCount dw 0
headCount dw 0
    
times 0x1B0-($-$$) db 0
diskIdentifier: times 10 db 0
part1: times 16 db 0
part2: times 16 db 0
part3: times 16 db 0
part4: times 16 db 0

dw 0xaa55