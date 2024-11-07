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