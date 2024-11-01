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


driveNumber db 0
partition_entry dw 0
sectorCount dw 0
headCount dw 0
pciError: db "Error: could not find PCI bus", 0
noCpuIdError: db "Error: could not find CPUID", 0
