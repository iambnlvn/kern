[bits 64]
[org 0x180000]
[section .text]
; stole some stuff from other oses, plus wherever chatgpt got his data from!!
; 0x107000 Graphic info
; 0x107FE8 RSDP
; 0x108000 ACPI
; 0x107FF0 Installation ID
; 0x140000-0x150000 Identity paging table
; 0x180000-0x1C0000 Loader
; 0x1F0000-0x200000 Stack
; 0x200000-0x300000 Kernel

%define memmap 0x160000
%define kbuffer 0x200000

start:
; set up the env
    lgdt    [gdtData.gdt]
    mov	    rax, 0x140000
    mov     cr3, rax
    mov     rax, 0x200000
    mov     rsp, rax
    mov     rax, 0x50
    mov     ds, rax
    mov     es, rax
    mov     ss, rax
    
    ; Retrieve prog headers
    ; RAX = ELF header, RBX = program headers
    mov rax, kbuffer
    mov rbx, [rax + 32]
    add rbx, rax

    ; ECX = entries, EDX = size of each entry
    movzx rcx, word [rax + 56]
    movzx rdx, word [rax + 54]

    ; loop through the program headers
    .loopProgHeaders:
    push rax
    push rcx

    ; work with load segments only

    mov eax, [rbx]
    cmp eax, 1
    jne .nextEntry

    ; clear mem
    mov rsi, [rbx + 8]
    xor  rax, rax
    mov rdi, [rbx + 16]
    rep stosb

    ; copy mem
    mov rcx, [rbx + 32]
    mov rsi, [rbx + 8]
    add rsi, kbuffer
    mov rdi, [rbx + 16]
    rep movsb

    ; next entry
    .nextEntry:
    pop rcx
    pop rax

    add rbx,rdx
    dec rcx
    or rcx, rcx
    jnz .loopProgHeaders
    jmp runKernel64

runKernel64:
    ; get kernel entry point
    mov rbx, kbuffer
    mov rcx, [rbx + 24]
    
    ; map the kernel

    mov rdi, 0xFFFFFF7FBFDFE000
    mov rax, [rdi]
    mov rdi, 0xFFFFFF7FBFDFEFE0
    mov [rdi], rax
    mov rax, cr3
    mov cr3, rax

    ; use the kernel entry point
    mov rax, 0xFFFFFE0000000000
    add qword [gdtData.gdt2], rax
    lgdt [gdtData.gdt]
    call setCodeSegment
    
    ; execute the kernel _start fn
    mov rdi, 0x100000
    mov rsi, 2
    jmp rcx

setCodeSegment:
    pop rax
    push 0x48
    push rax
    db 0x48, 0xCB

gdtData:
    .nullEntry  dq 0
    .codeEntry  dq 0xFFFF 
    		    db 0
			    dw 0xCF9A
			    db 0
	.dataEntry:	dd 0xFFFF	
			    db 0
			    dw 0xCF92
			    db 0
	.codeEntry16:	dd 0xFFFF	
			    db 0
			    dw 0x0F9A
			    db 0
	.dataEntry16:	dd 0xFFFF	
                db 0
                dw 0x0F92
                db 0
	.userCode:	dd 0xFFFF	
                db 0
                dw 0xCFFA
                db 0
	.userData:	dd 0xFFFF	
                db 0
                dw 0xCFF2
                db 0
	.tss:		dd 0x68		
                db 0
                dw 0xE9
                db 0
                dq 0
	.codeEntry64:	dd 0xFFFF	
                db 0
                dw 0xAF9A
                db 0
	.dataEntry64:	dd 0xFFFF	
                db 0
                dw 0xAF92
                db 0
	.userCode64:	dd 0xFFFF	
                db 0
                dw 0xAFFA
                db 0
	.userData64:	dd 0xFFFF	
                db 0
                dw 0xAFF2
                db 0
	.userCode64c:	dd 0xFFFF	
                db 0
                dw 0xAFFA
                db 0
	.gdt:		dw (gdtData.gdt - gdtData - 1)
	.gdt2:		dq gdtData

