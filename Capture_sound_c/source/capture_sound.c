/*
 * This is a simple example to communicate with a CDC-ACM USB device
 * using libusb.
 *
 * Thanks to the original Author: Christophe Augier <christophe.augier@gmail.com>
 * Modification : Eliot Ferragni
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include <libusb-1.0/libusb.h>


#define VENDOR_ID      0x1D50   // 
#define PRODUCT_ID     0x6018   // 

#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02

//device handle
static struct libusb_device_handle *devh = NULL;

/* The Endpoint address are hard coded. You should use lsusb -v to find
 * the values corresponding to your device.
 */
static int ep_in_addr  = 0x83;
static int ep_out_addr = 0x03;

void write_char(unsigned char c)
{
    /* To send a char to the device simply initiate a bulk_transfer to the
     * Endpoint with address ep_out_addr.
     */
    int actual_length;
    if (libusb_bulk_transfer(devh, ep_out_addr, &c, 1,
                             &actual_length, 0) < 0) {
        fprintf(stderr, "Error while sending char\n");
    }
}

int read_chars(unsigned char * data, int size)
{
    /* To receive characters from the device initiate a bulk_transfer to the
     * Endpoint with address ep_in_addr.
     */
    int actual_length;
    int rc = libusb_bulk_transfer(devh, ep_in_addr, data, size, &actual_length,
                                  1000);
    if (rc == LIBUSB_ERROR_TIMEOUT) {
        printf("timeout (%d)\n", actual_length);
        return -1;
    } else if (rc < 0) {
        fprintf(stderr, "Error while waiting for char\n");
        return -1;
    }

    return actual_length;
}

int main(int argc, char **argv)
{
    int rc;

    /* Initialize libusb
     */
    rc = libusb_init(NULL);
    if (rc < 0) {
        fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(rc));
        exit(1);
    }

    /* Set debugging output to max level.
     */
    libusb_set_debug(NULL, 3);

    /* Look for a specific device and open it.
     */
    devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!devh) {
        fprintf(stderr, "Error finding USB device\n");
        goto out;
    }

    /* As we are dealing with a CDC-ACM device, it's highly probable that
     * Linux already attached the cdc-acm driver to this device.
     * We need to detach the drivers from all the USB interfaces. The CDC-ACM
     * Class defines two interfaces: the Control interface and the
     * Data interface.
     * interfaces 2 and 3 are for the UART com of the blackmagic
     */
    int if_num = 0;
    for (if_num = 2; if_num < 4; if_num++) {
        if (libusb_kernel_driver_active(devh, if_num)) {
            libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) {
            fprintf(stderr, "Error claiming interface: %s\n",
                    libusb_error_name(rc));
            goto out;
        }
    }

    /* Start configuring the device:
     * - set line state
     */
    rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS,
                                2, NULL, 0, 0);
    if (rc < 0) {
        fprintf(stderr, "Error during control transfer: %s\n",
                libusb_error_name(rc));
    }

    /* - set line encoding: here 115200 8N1
     * 115200 = 0x0001c200 ~> 0x00,0xc2,0x01,0x00 in little endian
     */
    unsigned char encoding[] = { 0x00,0xc2,0x01,0x00, 0x00, 0x00, 0x00, 0x00, 0x08 };
    rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 2, encoding,
                                sizeof(encoding), 0);
    if (rc < 0) {
        fprintf(stderr, "Error during control transfer: %s\n",
                libusb_error_name(rc));
    }

    /* We can now start sending or receiving data to the device
     */
    uint8_t buf[64];
    uint32_t* pt_buf = (uint32_t*) buf;
    int len;
    uint32_t total = 0;
    
    while(1) {
        //write_char('t');
        len = read_chars(buf, 64);
        if(len>0){
          total += len;
          fprintf(stdout, "Received: %d \"%8X\"\n", total, *pt_buf);
          usleep(100);
          if(total >=(44100*4)){
            fprintf(stdout, "total recu = %d\n", total);
            break;
          }
        }
    }

    libusb_release_interface(devh, 2);

out:
    if (devh)
            libusb_close(devh);
    libusb_exit(NULL);
    return rc;
}