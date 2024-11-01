[bits 16]
[org 0x1000]


%define vesaInfo 0x7000               
%define superBlock 0x8000             
%define kernelFileEntry 0x8800      
%define pageDirectory 0x40000        
%define pageDirectoryLength 0x20000   
%define memoryMap 0x60000            
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
    jc .a20_enabled
    mov ax, 0x2401
    int 0x15
    call check_a20
    jc .a20_enabled
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

driveNumber db 0
partitionEntry dw 0
sectorCount dw 0
headCount dw 0
pciError: db "Error: could not find PCI bus", 0
noCpuIdError: db "Error: could not find CPUID", 0
noMsrError: db "Error: could not find MSR", 0
errorEnableA20: db "Error: could not enable A20 line", 0