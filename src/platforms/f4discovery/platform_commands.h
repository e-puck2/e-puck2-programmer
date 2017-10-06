#if defined(PLATFORM_COMMANDS_DEFINE)
/********************************************/
/* Begining of platform dedicated commands. */
/********************************************/
static void cmd_bulk(target *t, int argc, const char **argv);
/***************************************/
/* End of platform dedicated commands. */
/***************************************/
#undef PLATFORM_COMMANDS_DEFINE
#endif

#if defined(PLATFORM_COMMANDS_LIST)
/****************************************************/
/* Begining of List of platform dedicated commands. */
/* IMPORTANT : Each line MUST finish with "\"       */
/****************************************************/
	{"bulk", (cmd_handler)cmd_bulk, "send datas over UART"}, \
/***********************************************/
/* End of List of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_LIST
#endif

#if defined(PLATFORM_COMMANDS_CODE)
#include "cdcacm.h"
#include <libopencm3/cm3/nvic.h>
#include "gdb_packet.h" 

static void cmd_bulk(target *t, int argc, const char **argv)
{
    (void) argc;
    (void) argv;
    (void) t;

    uint8_t tableau[1000];
    uint32_t i = 0;

    for(i=0; i<1000; i++){
        tableau[i] = i;
    }
    uint16_t nb = 0;
    uint16_t nb2 = 0;
    uint8_t* pointeur = (uint8_t*) tableau;
    while(nb2<100){
        while(nb<15){
            while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, &pointeur[nb * 63], 63) <= 0);
            nb++;
        }
        nb2++;
        nb = 0;
    }
    nb2=0;
    
    char out[30] = {0};

    sprintf(out,"nb = %d \n",nb);

    gdb_out(out);
        //while(extended_stm32fx07_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, pointeur, 200) <= 0);
    return;
}

/***********************************************/
/* End of Code of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
