[bits 16]
[org 0x7c00]

%define stage2Addr 0x1000
%define tmpLoadBuffer 0x9000

start:
    cli
    mov dword [kernelSize], eax
    xor eax,eax
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax
    mov sp, 0x7c00
    jmp 0x0:.continue

    .continue
    sti
    mov [driveNumber], dl
    mov [partitionEntry],si

    mov ah, 0x08
    xor di, di
    int 0x13
    mov si, diskReadError
    jc error
    and cx, 0x3f
    mov [sectorCount], cx
    inc dh
    shr dx, 8
    mov [headCount], dx

    mov si, startupMessage
    .loop:
    lodsb
    or al, al
    jz .loadStage2
    mov ah, 0x0e
    int 0x10
    jmp .loop

    .loadStage2:
    mov cx, 15
    mov eax, 1
    mov edi, stage2Addr
    call loadSectors

    mov dl, [driveNumber]
    mov si, [partitionEntry]
    mov bx, [sectorCount]
    mov cx, [headCount]
    mov eax, dword [kernelSize]
    jmp 0x0:stage2Addr

loadSectors:
    pusha
    push edi

    mov bx, [partitionEntry]
    mov ebx, [bx + 8]
    add eax, ebx

    mov [readStructure.lba], eax
    mov ah, 0x42
    mov dl, [driveNumber]
    mov si, readStructure
    call loadEmuSector

    mov si, diskReadError
    jc error

    pop edi
    mov cx, 0x200
    mov eax, edi
    shr eax, 4
    and eax, 0xf000
    mov es, ax
    mov si, tmpLoadBuffer
    rep movsb
    
    popa
    add edi, 0x200
    inc eax
    loop loadSectors
    ret

    loadEmuSector:
    mov di, [readStructure.lba]
    xor ax, ax
    mov es, ax
    mov bx, 0x9000

    mov ax, di
    xor dx, dx
    div word [sectorCount]
    xor dx, dx
    div word [headCount]
    push dx
    mov ch, al
    shl ah, 6
    mov cl, ah

    mov ax, di
    xor dx, dx
    div word [sectorCount]
    inc dx
    or cl, dl

    pop dx
    mov dh, dl
    mov dl, [driveNumber]
    mov ax, 0x0201
    int 0x13

    ret
error:
    .loop:
    lodsb
    or al, al
    jz .break
    mov ah, 0x0e
    int 0x10
    jmp .loop

    .break:
    cli
    hlt


diskReadError db "ERROR:Disk read error at stage 1", 0
startupMessage db "Loading stage 2...", 10, 13, 0
kernelSize dd 0
driveNumber db 0
partitionEntry dd 0
sectorCount dw 0
headCount dw 0
readStructure:
    dw 0x10
    dw 1
    dd tmpLoadBuffer
    .lba: dq 0
    
times (0x200 - ($-$$)) db 0x0