[bits 16]
[org 0x1000]


%define vesaInfo 0x7000               
%define superBlock 0x8000             
%define kernelFileEntry 0x8800      
%define pageDirectory 0x40000        
%define pageDirectoryLength 0x20000   
%define mMap 0x60000            
%define temporaryLoadBuffer 0x9000    
%define partitionOffset (0x800 * 0x200)
%define kernelSector 0x10

start:
    mov dword [kernelSize], eax
    mov esp, 0x7C00    
    mov [driveNumber], dl
    mov [partitionEntry], si
    mov [sectorCount], bx
    mov [headCount], cx

checkPci:
    mov ax, 0xb101
    xor edi, edi
    int 0x1a
    mov si, pciError
    jc error
    or ah, ah
    jnz error

checkCpuId:
    mov dword [24], .noCpuId
    mov eax, 0
    cpuid
    jmp .hasCpuId
    .no_cpuid:
    mov si, noCpuIdError
    jmp error
    .hasCpuId:

checkMsr:
    mov dword [24], .noMsr
    mov ecx, 0xC0000080
    rdmsr
    jmp .hasMsr
    .no_msr:
    mov si, noMsrError
    jmp error
    .hasMsr:

checkA20:
    mov	ax,0
	mov	es,ax
	mov	ax,0xFFFF
	mov	fs,ax
	mov	byte [es:0x600],0
	mov	byte [fs:0x610],0xFF
	cmp	byte [es:0x600],0xFF
	je	.enabled
	stc
	ret
	.enabled: 
	clc
	ret

enableA20:
    cli
    call check_a20
    jc .a20Enabled
    mov ax, 0x2401
    int 0x15
    call checkA20
    jc .a20Enabled
    mov si, errorEnableA20
    jmp error
    .a20Enabled:
    sti
    jmp identityPaging

identityPaging:
    mov eax, pageDirectory / 16
    mov es, ax

    xor eax, eax
    mov ecx, 0x400
    xor di, di
    rep stosd

    mov dword [es: 0x3ff * 4], pageDirectory | 3
    mov dword [es: 0], (pageDirectory + 0x1000) | 3

    mov edi, 0x1000
    mov cx, 0x400
    mov eax, 3
    .loop:
    mov [es:edi], eax
    add edi, 4
    add eax, 0x1000
    loop .loop

    mov eax, pageDirectory
    mov cr3, eax

loadGDT:
    lgdt [lgdtData.gdt]

informBiosMixMode:
    mov eax, 0xec00
    mov ebx, 3
    int 0x15

loadMmap:
    xor ebx, ebx
    xor ax, ax
    mov es, ax
    mov ax, mmap / 16
    mov fs, ax

    .loop:
    mov di, .entry
    mov edx, 0x534D4150
    mov ecx, 24
    mov eax, 0xe820
    mov byte [.acpi], 1
    int 0x15
    jc .finished

    cmp eax, 0x534D4150
    jne .fail

    cmp dword [.type], 1
    jne .tryNext
    cmp dword [.size], 0
    je .tryNext
    cmp dword [.acpi], 0
    je .tryNext

    mov eax, [.size]
    and eax, ~0x3ff
    or eax, eax
    jz .tryNext

    .baseGood:
    mov eax, [.base]
    and eax, 0xfff
    or eax, eax
    jz .base_aligned
    mov eax, [.base]
    and eax, ~0xfff
    add eax, 0x1000
    mov [.base], eax
    sub dword [.size], 0x1000
    sbb dword [.size + 4], 0

    .baseAligned:
    mov eax, [.size]
    and eax, ~0xfff
    mov [.size], eax

    mov eax, [.size]
    shr eax, 12
    push ebx
    mov ebx, [.size + 4]
    shl ebx, 20
    add eax, ebx
    pop ebx
    mov [.size], eax
    mov dword [.size + 4], 0

    push ebx
    mov ebx, [.pointer]
    mov eax, [.base]
    mov [fs:bx], eax
    mov eax, [.base + 4]
    mov [fs:bx + 4], eax
    mov eax, [.size]
    mov [fs:bx + 8], eax
    add [.totalMem], eax
    mov eax, [.size + 4]
    adc [.totalMem + 4], eax
    mov [fs:bx + 12], eax
    add dword [.pointer], 16
    pop ebx

    .tryNext:
        or ebx, ebx
        jnz .loop
    
    .finished:
      mov eax, [.pointer]
    shr eax, 4
    or eax, eax
    jz .fail

    mov ebx, [.pointer]
    mov dword [fs:bx], 0
    mov dword [fs:bx + 4], 0
    mov eax, [.totalMem]
    mov dword [fs:bx + 8], eax
    mov eax, [.totalMem + 4]
    mov dword [fs:bx + 12], eax
    jmp allocKernBuff

    .fail:
    mov si, memoryMapGettingError
    jmp error

    .pointer:
        dd 0
    .entry:
        .base: dq 0
        .size: dq 0
        .type: dd 0
        .acpi: dd 0
    .totalMem: dq 0

allocKernBuff:
    push ds
    push es
    push ss
    cli
    mov eax, cr0
    or eax, 0x80000001
    mov cr0, eax
    jmp 0x8:.protectedMode

    [bits 32]
    .protectedMode:
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov ss, ax

    mov ecx, [kernelSize]
    shr ecx, 12
    inc ecx
    mov edx, ecx
    shl edx, 12

    xor ebx, ebx

    .memRegionLoop:
    mov eax, [ebx + mMap + 4]
    or eax, eax
    jnz .tryNextMemRegion
    mov eax, [ebx + mMap]
    cmp eax, 0x100000
    jne .tryNextMemRegion

    mov eax, [ebx + mMap + 8]
    cmp eax, ecx
    jl .tryNextMemRegion

    mov eax, [ebx + mMap + 0]
    mov [kernBuffer], eax
    add eax, edx
    mov [ebx + mMap + 0], eax
    sub dword [ebx + mMap + 8], ecx
    sbb dword [ebx + mMap + 12], 0

    jmp .foundBuffer

    .tryNextMemRegion:
    add ebx, 16
    mov eax, [loadMmap.pointer]
    cmp ebx, eax
    jne .memRegionLoop
    mov si, noMemory
    jmp error32

    .foundBuffer:
    mov eax, cr0
    and eax, 0x7FFFFFFF
    mov cr0, eax
    jmp 0x18:.realMode

    [bits 16]
    .realMode:
    mov eax, cr0
    and eax, 0x7FFFFFFE
    mov cr0, eax
    jmp 0x0:.finish

    .finish:
    pop ss
    pop es
    pop ds


error32:
    mov eax, cr0
    and eax, 0x7FFFFFFF
    mov cr0, eax
    jmp 0x18:.realMode

    [bits 16]
    .real_mode:
    mov realMode, cr0
    and eax, 0x7FFFFFFE
    mov cr0, eax
    jmp 0x0:.finish

    .finish:
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax
    jmp error

loadKernIntoMem:
    mov eax, dword [kernelSize]
    xor edx, edx
    mov ecx, 0x200
    div ecx
    cmp eax, 0xffff
    mov si, kernelSectorCountExceeded
    jg error
    mov cx, ax
    mov edi, [kernBuffer]

    mov eax, kernelSector
    call loadSectors
    jmp enableVideoMode

loadSectors:
    .loop:
    pushad
    push edi
    mov bx, [partitionEntry]
    mov ebx, [bx + 8]
    add eax, ebx
    mov [readStructure.lba], eax
    mov ah, 0x42
    mov dl, [driveNumber]
    mov si, readStructure
    call loadSector

    mov si, errorReadingDisk
    jc error
    pop edi
    call moveSectorToTarget
    popad
    add edi, 0x200
    inc eax
    loop .loop
    ret

loadSector:
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



readStructure:
    dw 0x10
    dw 1
    dd temporaryLoadBuffer
    .lba: dq 0


moveSectorToTarget:
    push ss
    push ds
    push es

    cli
    mov eax, cr0
    or eax, 0x80000001
    mov cr0, eax
    jmp 0x8:.protectedMode

    [bits 32]
    .protectedMode:
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov ss, ax

    mov ecx, 0x200
    mov esi, temporaryLoadBuffer
    rep movsb

    mov eax, cr0
    and eax, 0x7FFFFFFF
    mov cr0, eax
    jmp 0x18:.realMode

    [bits 16]
    .real_mode:
    mov eax, cr0
    and eax, 0x7FFFFFFE
    mov cr0, eax
    jmp 0x0:.finish

    .finish:
    pop es
    pop ds
    pop ss
    sti
    ret


enableVideoMode:
    call vbe_init
    jmp enableVideoModeDone ; TODO!add this marker later below



vbeInit:
	mov ax, vesaInfo
    shr ax, 4
	mov	es, ax
    xor	di,di
    mov	ax,0x4F15
	mov	bl,1
	xor	cx,cx
	xor	dx,dx
	xor	di,di
	int	0x10
	cmp	ax,0x4F
	jne	.noEdid
	cmp	byte [es:1],0xFF
	jne	.noEdid
	mov	al,[es:0x38]
	mov	ah,[es:0x3A]
	shr	ah,4
	mov	bl,[es:0x3B]
	mov	bh,[es:0x3D]
	shr	bh,4
	or	ax,ax
	jz	.noEdid
	or	bx,bx
	jz	.noEdid
	mov	[wbeSuitableWidth],ax
	mov	[vbeSuitableHeight],bx
	mov	byte [vbeHasEdid],1
	jmp	.noFlatPanel

    .noEdid:
    mov	ax,0x4F11
	mov	bx,1
	xor	di,di
	int	0x10
	cmp	ax,0x4F
	jne	.noFlatPanel
	mov	ax,[es:0x00]
	mov	bx,[es:0x02]
	or	ax,ax
	jz	.noFlatPanel
	or	bx,bx
	jz	.noFlatPanel
	cmp	ax,4096
	ja	.noFlatPanel
	cmp	bx,4096
	ja	.noFlatPanel
	mov	[vbeSuitableWidth],ax
	mov	[vbeSuitableHeight],bx

    .noFlatPanel:
    xor	di,di
	mov	ax,0x4F00
	int	0x10
	cmp	ax,0x4F
	jne badVbe

	add	di,0x200
	mov	eax,[es:14]
	cmp	eax,0
	je	.findDone
	mov	ax,[es:16]
	mov	fs,ax
	mov	si,[es:14]
	xor	cx,cx
    .findLoop:
    mov	ax,[fs:si]
	cmp	ax,0xFFFF
	je	.findDone
	mov	[es:di],ax
	add	di,2
	add	si,2
	jmp	.findLoop

    .findDone:
    mov	word [es:di],0xFFFF
	cmp	di,0x200
	jne	.addedModes
	mov	word [es:di + 0],257
	mov	word [es:di + 2],259
	mov	word [es:di + 4],261
	mov	word [es:di + 6],263
	mov	word [es:di + 8],273
	mov	word [es:di + 10],276
	mov	word [es:di + 12],279
	mov	word [es:di + 14],282
	mov	word [es:di + 16],274
	mov	word [es:di + 18],277
	mov	word [es:di + 20],280
	mov	word [es:di + 22],283
	mov	word [es:di + 24],0xFFFF
    .addedModes:
    mov	si,0x200
	mov	di,0x200


    .checkLoop:
	mov	cx,[es:si]
	mov	[es:di],cx
	cmp	cx,0xFFFF
	je	.checkDone
	push	di
	push	si
	mov	ax,0x4F01
	xor	di,di
	or	cx, 0x4000
	int	0x10
	pop	si
	pop	di
	add	si,2
	cmp	ax,0x4F			; Interrupt failure
	jne	.checkLoop
	cmp	byte [es:0x19],24	; 24 and 32 bit support only
	je	.validBpp
	cmp	byte [es:0x19],32
	je	.validBpp
	jne	.checkLoop

    .validBpp:
    cmp	word [es:0x14],480    ; 640x480 minimum resolution
	jl	.checkLoop
	mov	ax,[vbeSuitableWidth]
	cmp	[es:0x12],ax
	jne	.modeNotSuitable
	mov	ax,[vbeSuitableHeight]
	cmp	[es:0x14],ax
	jne	.modeNotSuitable
	mov	ax,[es:di]
	mov	[vbeSuitabeMode],ax

    .modeNotSuitable:
    add	di,2
	jmp	.checkLoop

    .checkDone:
    mov	bx,[vbeSuitabeMode]
	or	bx,bx
	jnz	.setGraphicsMode

    .setGraphicsMode:
    or	bx, 0x4000
	mov	cx,bx
	mov	ax,0x4F02
	int	0x10
	cmp	ax,0x4F
	jne	vbeFailed

	; save mode info for the kernel
	mov	ax,0x4F01
	xor	di,di
	int	0x10
	mov	byte [es:0],1 
	mov	al,[es:0x19]
	mov	[es:1],al     
	mov	ax,[es:0x12]
	mov	[es:2],ax     
	mov	ax,[es:0x14]
	mov	[es:4],ax     
	mov	ax,[es:0x10]
	mov	[es:6],ax    
	mov	eax,[es:40]
	mov	[es:8],eax    
	xor	eax,eax
	mov	[es:12],eax
	mov	ax,0x4F15
	mov	bl,1
	xor	cx,cx
	xor	dx,dx
	mov	di,0x10
	int	0x10
	mov	al,[vbeHasEdid]
	shl	al,1
	or	[es:0],al
	ret

;Todo: handle the case of no suitable vbe


badVbe:
	mov	byte [es:di],0
	ret

;TODO! implement this later
;vbeFailed:

driveNumber db 0
partitionEntry dw 0
sectorCount dw 0
headCount dw 0

kernBuffer dq 0
kernelSize dd 0

pciError: db "Error: could not find PCI bus", 0
noCpuIdError: db "Error: could not find CPUID", 0
noMsrError: db "Error: could not find MSR", 0
errorEnableA20: db "Error: could not enable A20 line", 0
memoryMapGettingError: db "Error: could not get memory map", 0
noMemory: db "Error: could not find memory for kernel", 0
kernelSectorCountExceeded: db "Error: kernel sector count exceeded", 0
errorReadingDisk: db "Error: could not read disk", 0


wbeSuitableWidth: dw 0
vbeSuitableHeight: dw 0
vbeSuitabeMode: dw 0
vbeHasEdid: db 0