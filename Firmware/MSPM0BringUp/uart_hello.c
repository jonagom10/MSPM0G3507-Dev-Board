#include "ti_msp_dl_config.h"

static void uart_puts(const char *s)
{
    while (*s) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *s++);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        uart_puts("Hello from the board! Im a board and Im talking! \r\n");
        delay_cycles(CPUCLK_FREQ);   // ~1 s, whatever your clock is
    }
} 