ENTRY(_start)

virtualBase = 0xFFFFFFFF80000000;
SECTIONS
{
    . = virtualBase;
	.text ALIGN(4K) : AT(ADDR(.text) - virtualBase)
	{
		*(.text)
        *(.text.memset)
        *(.text.memcpy)
        *(.text.compiler_rt.udivmod.udivmod)
        *(.text.__udivti3)
	}
	.rodata ALIGN(4K) : AT(ADDR(.rodata) - virtualBase)
	{
		*(.rodata)
        *(.rodata.str1.1)
        *(.rodata.cst4)
        *(.rodata.cst8)
        *(.rodata.cst16)
        *(.rodata.cst32)
	}
	.data ALIGN(4K) : AT(ADDR(.data) - virtualBase)
	{
		*(.data)
        *(.data.rel.ro)
	}
	.bss ALIGN(4K) : AT(ADDR(.bss) - virtualBase)
	{
		*(COMMON)
		*(.bss)
	}
}