/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/flash.h>

// USB Code

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1337,
	.idProduct = 0x1337,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};


static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};
/*
static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 3,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 6,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};
*/
static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
	.iInterface = 2,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char serial_str[] __attribute__ ((section (".flash"))) = "MSEM0000";

static const char *usb_strings[] = {
	"James Pallister",
	"Medium speed energy monitor",
    serial_str,
};

int running = 0;
int head_ptr = 0, tail_ptr = 0;
// char globbuf[2] = {0xAB, 0xCD};

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, u8 **buf,
		u16 *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    int i;

	(void)complete;
	(void)buf;
	(void)usbd_dev;

    gpio_toggle(GPIOD, GPIO14);


	switch (req->bRequest) {
	case 0:    // toggle LEDS
        gpio_toggle(GPIOD, GPIO13);
        *len = 0;
        break;
    case 1:     // Start
    {
        running = 1;
        head_ptr = 0;
        tail_ptr = 0;
        timer_enable_counter(TIM2);
        adc_power_on(ADC1);
        *len = 0;
        break;
    }
    case 2:     // Stop
    {
        running = 0;
        timer_disable_counter(TIM2);
        adc_off(ADC1);
        *len = 0;
        break;
    }
    case 3:     // Set serial
    {
        u32 base_addr = (u32) serial_str;

        if(*len != 8)
            return 0;

        flash_unlock();
        flash_erase_sector(FLASH_CR_SECTOR_1, FLASH_CR_PROGRAM_X32);
        for(i = 0; i < 8 ;++i)
        {
            flash_program_byte(base_addr+i, (*buf)[i], FLASH_CR_PROGRAM_X8);
        }
        flash_program_byte(base_addr+8, 0x0, FLASH_CR_PROGRAM_X8);
        flash_lock();

        break;
    }
    default:
        return 0;
	}
	return 1;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, u8 ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
	int i;

    for(i = 0; i < len; ++i)
    {
        if(buf[i] == 'S')
        {
            running = 1;
            head_ptr = 0;
            tail_ptr = 0;
			timer_enable_counter(TIM2);
        	adc_power_on(ADC1);
        }
        if(buf[i] == 'F')
        {
            running = 0;
			timer_disable_counter(TIM2);
        	adc_off(ADC1);
        }
    }

	gpio_toggle(GPIOD, GPIO15);
}


static void usb_reset_cb()
{
    running = 0;
}

static void cdcacm_set_config(usbd_device *usbd_dev, u16 wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	// usbd_register_control_callback(
	// 			usbd_dev,
	// 			USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
	// 			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
	// 			cdcacm_control_request);
	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);

    usbd_register_reset_callback(usbd_dev, usb_reset_cb);
}

// Data structs

#define DATA_BUF_BYTES      64
#define DATA_BUF_SHORTS     42
#define NUM_BUFFERS         2000
#define NUM_BUFFERS_MASK    (NUM_BUFFERS-1)

#define HIGH_THRESH         64
#define LOW_THRESH          960

typedef struct {
    unsigned char data[DATA_BUF_BYTES];
} power_data;

power_data data_bufs[NUM_BUFFERS] = {0};

short dbuf0[DATA_BUF_SHORTS];
int sent_counter=0;

//#define TPERIOD_INIT    800
#define TPERIOD_INIT    1600

int tperiod=TPERIOD_INIT;

void dma_setup()
{
    dma_stream_reset(DMA2, DMA_STREAM0);

    dma_set_peripheral_address(DMA2, DMA_STREAM0, (u32)&ADC1_DR);
    dma_set_memory_address(DMA2, DMA_STREAM0, (u32)&dbuf0);
    dma_set_number_of_data(DMA2, DMA_STREAM0, DATA_BUF_SHORTS);
    dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_16BIT);
    dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_HIGH);
    dma_enable_circular_mode(DMA2, DMA_STREAM0);
    dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
    dma_set_peripheral_burst(DMA2, DMA_STREAM0, DMA_SxCR_PBURST_SINGLE);
    dma_set_memory_burst(DMA2, DMA_STREAM0, DMA_SxCR_MBURST_SINGLE);

    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);

    nvic_set_priority(NVIC_DMA2_STREAM0_IRQ, 3);
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    dma_enable_stream(DMA2, DMA_STREAM0);

    // Memory to memory dma
    // dma_stream_reset(DMA2, DMA_STREAM1);
    // dma_set_transfer_mode(DMA2, DMA_STREAM1, DMA_SxCR_DIR_MEM_TO_MEM);
    // dma_set_priority(DMA2, DMA_STREAM1, DMA_SxCR_PL_VERY_HIGH);
    // dma_set_peripheral_size(DMA2, DMA_STREAM1, DMA_SxCR_PSIZE_32BIT);
    // dma_set_memory_size(DMA2, DMA_STREAM1, DMA_SxCR_MSIZE_32BIT);
    // dma_enable_memory_increment_mode(DMA2, DMA_STREAM1);
    // dma_enable_peripheral_increment_mode(DMA2, DMA_STREAM1);
    // dma_set_peripheral_address(DMA2, DMA_STREAM1, dbuf0);
    // // dma_set_number_of_data(DMA2, DMA_STREAM1, DATA_BUF_LONGLEN);

    // dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM1);
    // nvic_set_priority(NVIC_DMA2_STREAM1_IRQ, 4);
    // nvic_enable_irq(NVIC_DMA2_STREAM1_IRQ);
}

void timer_setup()
{
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);

	timer_reset(TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM2, tperiod>>2);
	timer_set_prescaler(TIM2, 0);
	timer_set_clock_division(TIM2, TIM_CR1_CKD_CK_INT);
	timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    timer_enable_preload(TIM2);

    timer_reset(TIM3);
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM3, 100);
	timer_set_prescaler(TIM3, 167);
	timer_set_clock_division(TIM3, TIM_CR1_CKD_CK_INT);
	timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
    timer_enable_irq(TIM3, TIM_DIER_UIE);

    // timer_enable_counter(TIM3);

	// nvic_set_priority(NVIC_TIM3_IRQ, 0);
 //    nvic_enable_irq(NVIC_TIM3_IRQ);

	// nvic_set_priority(NVIC_ADC_IRQ, 0);
 //    nvic_enable_irq(NVIC_ADC_IRQ);
}

void adc_setup()
{
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1 | GPIO2);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
	adc_set_clk_prescale(0);
	adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR1_SMP_1DOT5CYC);
	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR1_SMP_1DOT5CYC);

	u8 channels[] = {ADC_CHANNEL1, ADC_CHANNEL1, ADC_CHANNEL2};
	adc_set_regular_sequence(ADC1, 3, channels);
    adc_enable_scan_mode(ADC1);
    adc_enable_discontinuous_mode_regular(ADC1, ADC_CR1_DISCNUM_1CHANNELS);

	adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);

	adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

    adc_enable_dma(ADC1);
    adc_set_dma_continue(ADC1);
    ADC_CCR |= ADC_CCR_DMA_MODE_1;

	adc_set_right_aligned(ADC1);
	adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);
	adc_power_on(ADC1);
}

usbd_device *usbd_dev;


int main(void)
{
	int c_started=0, n, cpy;
	short s;

	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);

	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

    dma_setup();
	adc_setup();
	timer_setup();

	gpio_toggle(GPIOA, GPIO12);


	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3);
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	while (1)
	{
        usbd_poll(usbd_dev);

        if(head_ptr == tail_ptr)
            continue;

		if(usbd_ep_write_packet(usbd_dev, 0x81, data_bufs[tail_ptr].data, DATA_BUF_BYTES) != 0)
        {
            gpio_toggle(GPIOD, GPIO15);

            tail_ptr = (tail_ptr+1);
            if(tail_ptr >= NUM_BUFFERS)
                tail_ptr = 0;
            if(sent_counter < NUM_BUFFERS*2)
                sent_counter++;
        }
	}
}



// void adc_isr()
// {
// 	short s;
//     char buf[64];

//     if(!running)
//         return;
// 	ADC_SR(ADC1) &= ~ADC_SR_EOC;
// 	s = adc_read_regular(ADC1);
// 	while(usbd_ep_write_packet(usbd_dev, 0x81, &s, 2)==0 && running == 1);// usbd_poll(usbd_dev);
//	while(usbd_ep_write_packet(usbd_dev, 0x81, buf, 64)==0 && running == 1);// usbd_poll(usbd_dev);
//	s = timer_get_counter(TIM2);
//	while(usbd_ep_write_packet(usbd_dev, 0x81, &s, 2)==0 && running == 1) usbd_poll(usbd_dev);
// }

int lastErr = 0;

void dma2_stream0_isr()
{
    int nhead,i;

    if((DMA2_LISR & DMA_LISR_TCIF0) != 0)
    {
        dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_LISR_TCIF0);

        nhead = (head_ptr+1);
        if(nhead >= NUM_BUFFERS)
            nhead -= NUM_BUFFERS;
        if(nhead == tail_ptr)
            while(1);
             //return;

        head_ptr = nhead;

        data_bufs[head_ptr].data[0] = tperiod>>3;
        for(i = 0; i < DATA_BUF_SHORTS/2; ++i)
        {
            data_bufs[head_ptr].data[i*3+1] = dbuf0[i*2]&0xFF;
            data_bufs[head_ptr].data[i*3+2] = dbuf0[i*2+1]&0xFF;
            data_bufs[head_ptr].data[i*3+3] = (dbuf0[i*2]>>8) | ((dbuf0[i*2+1]>>4)&0xF0);
        }

        // for(i = 0; i < DATA_BUF_SHORTLEN; ++i)
        // {
        //     data_bufs[head_ptr].data.asShorts[i] = tperiod;
        // }

        if(running)
        {
            // dif is number of full buffers
            int dif = (head_ptr+NUM_BUFFERS-tail_ptr);
            int err;

            if(dif >= NUM_BUFFERS)
                dif -= NUM_BUFFERS;

            err = 32 - dif;

            if(sent_counter < NUM_BUFFERS)
                err = (int)((float)err * ((float)sent_counter/(float)(NUM_BUFFERS)));

            tperiod -= ((err>>4) + ((err - lastErr)<<3));
            lastErr = err;

            timer_set_period(TIM2, tperiod >> 3);
        }
    }
}

//void dma2_stream1_isr()
//{
//    if((DMA2_LISR & DMA_LISR_TCIF1) != 0)
//    {
//        dma_clear_interrupt_flags(DMA2, DMA_STREAM1, DMA_LISR_TCIF1);
//        dma_disable_stream(DMA2, DMA_STREAM1);
//        // while(usbd_ep_write_packet(usbd_dev, 0x81, data_bufs[cur_buf].data.asBytes, DATA_BUF_BYTELEN)==0 && running == 1);
//    }
//}


// void tim3_isr()
// {
//     TIM_SR(TIM3) &= ~TIM_SR_UIF;
//     if(running)
//     {
//         // dif is number of full buffers
//         int dif = (head_ptr+NUM_BUFFERS-tail_ptr) & NUM_BUFFERS_MASK;
//         int err;

//         err = NUM_BUFFERS/2 - dif;

//         if(sent_counter < NUM_BUFFERS)
//             err = (int)((float)err * ((float)sent_counter/(float)(NUM_BUFFERS)));

//         tperiod -= (err>>4) + ((err - lastErr)<<2);
//         lastErr = err;
//         // timer_set_period(TIM2, tperiod >> 2);
//     }
// }

void exit(int a)
{
	while(1);
}
