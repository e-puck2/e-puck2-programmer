
#include <ch.h>
#include <hal.h>

/**
 * @brief Panic handler called when a kernel panic is invoked
 * @param reason Information given by the OS
 */
void panic_handler(const char *reason)
{
    (void)reason;

    palClearLine(LINE_LED_GREEN);
    palClearLine(LINE_LED_RED);
    palClearLine(LINE_LED_BLUE);
  
    while (1) {

    }
}