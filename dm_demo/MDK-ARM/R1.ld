/* STM32H723VGTx 链接脚本（GCC 版）
 *
 * 内存布局按 Keil 工程 R1.sct 抄过来，保持一致：
 *   FLASH 0x08000000 1024K   —— 从 0x08000000 起 1MB，和 STM32H723VG 的 Flash 一致
 *   RAM   0x20000000  128K   —— DTCM，.data/.bss/栈放这里（Keil 的 RW_IRAM1）
 *   RAM2  0x24000000  320K   —— AXI SRAM，Keil 的 RW_IRAM2，这里留给以后放 DMA 缓冲
 *
 * 注意：DMA 不能访问 DTCM，以后要用串口/ADC 的 DMA，缓冲区得显式放到 .RAM2 段
 *       （__attribute__((section(".RAM2")))）。
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    RAM2  (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);   /* 栈顶 = DTCM 末尾 0x20020000 */

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    . = ALIGN(4);
    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* DMA 专用区：以后需要放 DMA 缓冲时用
     *   __attribute__((section(".RAM2"))) static uint8_t buf[256]; */
    .RAM2 (NOLOAD) :
    {
        . = ALIGN(4);
        *(.RAM2)
        *(.RAM2*)
        . = ALIGN(4);
    } >RAM2

    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.gnu.lto_*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
