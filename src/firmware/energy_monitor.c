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
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/pwr.h>

// USB Code

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0xF539,
    .idProduct = 0xF539,
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
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 3,
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

static const char serial_str[] __attribute__ ((section (".flash"))) = "EE00";

static const char *usb_strings[] = {
    "James Pallister",
    "Medium speed energy monitor",
    serial_str,
};

void dma_setup();
void adc_setup();

int running = 0;
int head_ptr = 0, tail_ptr = 0;
int trigger_port = -1, trigger_pin = -1;

typedef struct {
    uint64_t energy_accum;
    uint64_t elapsed_time;
    unsigned peak_power;
    unsigned peak_voltage;
    unsigned peak_current;
    unsigned n_samples;
} accumulated_data;
//uint64_t energy_accum=123;

accumulated_data a_data;

unsigned n_samples=0;

#define TPERIOD_INIT    1600

int tperiod=TPERIOD_INIT;


void exti_setup()
{
    nvic_disable_irq(NVIC_EXTI0_IRQ);
    nvic_disable_irq(NVIC_EXTI1_IRQ);
    nvic_disable_irq(NVIC_EXTI2_IRQ);
    nvic_disable_irq(NVIC_EXTI3_IRQ);
    nvic_disable_irq(NVIC_EXTI4_IRQ);
    nvic_disable_irq(NVIC_EXTI9_5_IRQ);
    nvic_disable_irq(NVIC_EXTI15_10_IRQ);
    // timer_disable_counter(TIM3);
    exti_reset_request(EXTI0 | EXTI1 | EXTI2 | EXTI3 | EXTI4 | EXTI5 | EXTI6  | EXTI7
            | EXTI8 | EXTI9 | EXTI10 | EXTI11 | EXTI12 | EXTI13 | EXTI14  | EXTI15);

    if(trigger_port == -1)
        return;

    exti_select_source(trigger_pin, trigger_port);
    exti_set_trigger(trigger_pin, EXTI_TRIGGER_BOTH);
    exti_enable_request(trigger_pin);
    gpio_mode_setup(trigger_port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, trigger_pin);

    switch(trigger_pin)
    {
        case 1<<0: nvic_enable_irq(NVIC_EXTI0_IRQ); break;
        case 1<<1: nvic_enable_irq(NVIC_EXTI1_IRQ); break;
        case 1<<2: nvic_enable_irq(NVIC_EXTI2_IRQ); break;
        case 1<<3: nvic_enable_irq(NVIC_EXTI3_IRQ); break;
        case 1<<4: nvic_enable_irq(NVIC_EXTI4_IRQ); break;
        case 1<<5:
        case 1<<6:
        case 1<<7:
        case 1<<8:
        case 1<<9: nvic_enable_irq(NVIC_EXTI9_5_IRQ); break;
        case 1<<10:
        case 1<<11:
        case 1<<12:
        case 1<<13:
        case 1<<14:
        case 1<<15: nvic_enable_irq(NVIC_EXTI15_10_IRQ); break;
    }
}

void start_measurement()
{
    running = 1;
    head_ptr = 0;
    tail_ptr = 0;
    n_samples = 0;

    a_data.energy_accum = 0;
    a_data.elapsed_time = 0;
    a_data.peak_power = 0;
    a_data.peak_voltage = 0;
    a_data.peak_current = 0;
    a_data.n_samples = 0;

    tperiod = 500;

    timer_set_period(TIM2, tperiod);
    timer_enable_counter(TIM2);

    adc_power_on(ADC1);
    // adc_power_on(ADC2);
    // adc_power_on(ADC3);
    gpio_set(GPIOD, GPIO12);
}

void stop_measurement()
{
    running = 0;
    head_ptr = 0;
    tail_ptr = 0;
    timer_disable_counter(TIM2);
    adc_off(ADC1);
    // adc_off(ADC2);
    // adc_off(ADC3);
        gpio_clear(GPIOD, GPIO12);
}

static int usbdev_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    int i;

    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest) {
    case 0:    // toggle LEDS
        gpio_toggle(GPIOD, GPIO13);
        *len = 0;
        break;
    case 1:     // Start
    {
        gpio_set(GPIOD, GPIO12);

        start_measurement();
        *len = 0;
        break;
    }
    case 2:     // Stop
    {
        gpio_clear(GPIOD, GPIO12);
        stop_measurement();
        *len = 0;
        break;
    }
    case 3:     // Set serial
    {
        uint32_t base_addr = (uint32_t) serial_str;

        if(*len != 0)
            return 0;

        flash_unlock();
        flash_erase_sector(1, FLASH_CR_PROGRAM_X32);

        flash_program_byte(base_addr+0, req->wValue & 0xFF);
        flash_program_byte(base_addr+1, req->wValue >> 8);
        flash_program_byte(base_addr+2, req->wIndex & 0xFF);
        flash_program_byte(base_addr+3, req->wIndex >> 8);

        flash_program_byte(base_addr+4, 0x0);
        flash_lock();

        break;
    }
    case 4:     // Set Trigger
    {
        if(*len != 0)
            return 0;

        gpio_toggle(GPIOD, GPIO14);

        switch(req->wValue)
        {
            case 'A': trigger_port = GPIOA; break;
            case 'B': trigger_port = GPIOB; break;
            case 'C': trigger_port = GPIOC; break;
            case 'D': trigger_port = GPIOD; break;
            case 'E': trigger_port = GPIOE; break;
            case 'F': trigger_port = GPIOF; break;
            case 'G': trigger_port = GPIOG; break;
            case 'H': trigger_port = GPIOH; break;
            default:
                trigger_port = -1; break;
        }

        trigger_pin = 1 << req->wIndex;

        if(trigger_port != GPIOA)
            gpio_toggle(GPIOD, GPIO12);

        exti_setup();
        break;
    }
    case 6:     // Get energy
    {
        *len = sizeof(accumulated_data);
        *buf = &a_data;
        break;
    }
    default:
        return 0;
    }
    return 1;
}

static void usbdev_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
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

static void usbdev_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbdev_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 64, NULL);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                usbdev_control_request);

    usbd_register_reset_callback(usbd_dev, usb_reset_cb);
}

// Data structs

#define DATA_BUF_BYTES      64
#define DATA_BUF_SHORTS     42
#define NUM_BUFFERS         1000
#define NUM_BUFFERS_MASK    (NUM_BUFFERS-1)

#define HIGH_THRESH         64
#define LOW_THRESH          960

#define REGULAR_ADC_SHORTS  45
#define DUAL_ADC_SHORTS     (42*2)

#define DMA_SHORTS          128

typedef struct {
    unsigned char data[DATA_BUF_BYTES];
} power_data;

power_data data_bufs[NUM_BUFFERS] = {0};

unsigned short dbuf0[DMA_SHORTS] __attribute__ ((aligned (8)));
unsigned short dbuf1[DMA_SHORTS] __attribute__ ((aligned (8)));
int sent_counter=0;



void dma_setup()
{
    dma_stream_reset(DMA2, DMA_STREAM0);

    dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t)&ADC1_DR);
    dma_set_number_of_data(DMA2, DMA_STREAM0, DMA_SHORTS);

    dma_set_memory_address(DMA2, DMA_STREAM0, (uint32_t)&dbuf0);
    dma_set_memory_address_1(DMA2, DMA_STREAM0, (uint32_t)&dbuf1);
    dma_set_initial_target(DMA2, DMA_STREAM0, 0);
    dma_enable_double_buffer_mode(DMA2, DMA_STREAM0);

    dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
    dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_16BIT);
    dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_16BIT);
    dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_HIGH);
    dma_enable_circular_mode(DMA2, DMA_STREAM0);
    dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_0);
    dma_set_peripheral_burst(DMA2, DMA_STREAM0, DMA_SxCR_PBURST_SINGLE);
    dma_set_memory_burst(DMA2, DMA_STREAM0, DMA_SxCR_MBURST_INCR4);
    dma_enable_fifo_mode(DMA2, DMA_STREAM0);
    dma_set_fifo_threshold(DMA2, DMA_STREAM0, DMA_SxFCR_FTH_4_4_FULL);

    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM0);
    dma_enable_fifo_error_interrupt(DMA2, DMA_STREAM0);
    dma_enable_transfer_error_interrupt(DMA2, DMA_STREAM0);

    nvic_set_priority(NVIC_DMA2_STREAM0_IRQ, 1);
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    // dma_enable_stream(DMA2, DMA_STREAM0);
}

void timer_setup()
{
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

    timer_disable_counter(TIM2);
    timer_reset(TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM2, tperiod);
    timer_set_prescaler(TIM2, 0);
    timer_set_clock_division(TIM2, TIM_CR1_CKD_CK_INT);
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
    timer_enable_preload(TIM2);
}

void adc_setup()
{
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1 | GPIO2 | GPIO3);
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2 | GPIO4 | GPIO5);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC3EN);
    rcc_peripheral_reset(&RCC_APB2ENR, RCC_APB2RSTR_ADCRST);
    adc_off(ADC1);
    adc_off(ADC2);
    adc_off(ADC3);

    ADC_CCR = 0;
    adc_set_clk_prescale(0);
    adc_set_single_conversion_mode(ADC1);
    adc_set_single_conversion_mode(ADC2);
	adc_set_single_conversion_mode(ADC3);

    // Input 1
    uint8_t channels1[] = {2, 12};   // Voltage, PA2, ADC123
    // uint8_t channels1[] = {ADC_CHANNEL2, ADC_CHANNEL12};   // Voltage, PA2, ADC123
    // uint8_t channels2[] = {ADC_CHANNEL12};  // Current, PC2, ADC123
    // Input 2
    // uint8_t channels1[] = {ADC_CHANNEL3};   // Voltage, PA3, ADC123
    // uint8_t channels2[] = {ADC_CHANNEL1};   // Current, PA1, ADC123
    // Input 3
    // uint8_t channels1[] = {ADC_CHANNEL9};   // Voltage, PB1, ADC12
    // uint8_t channels2[] = {ADC_CHANNEL15};  // Current, PC5, ADC12
    // Input self
    // uint8_t channels1[] = {ADC_CHANNEL8};   // Voltage, PB0, ADC12
    // uint8_t channels2[] = {ADC_CHANNEL14};  // Current, PC4, ADC12
    adc_set_regular_sequence(ADC1, 2, channels1);
    // adc_enable_discontinuous_mode_regular(ADC1, 1);
    adc_enable_scan_mode(ADC1);
    // adc_set_regular_sequence(ADC2, 1, channels2);

    adc_disable_external_trigger_regular(ADC2);

    adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_15CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL12, ADC_SMPR_SMP_15CYC);
    adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC3, ADC_CR1_RES_12BIT);

    // adc_enable_dma(ADC1);
    // adc_enable_dma(ADC2);
    // adc_enable_dma(ADC3);
    adc_set_dma_continue(ADC1);
    // adc_set_dma_continue(ADC2);
    // adc_set_dma_continue(ADC3);

    adc_set_right_aligned(ADC1);
    // adc_set_right_aligned(ADC2);
	// adc_set_right_aligned(ADC3);

    adc_enable_overrun_interrupt(ADC1);

    adc_enable_eoc_interrupt(ADC1);
    adc_eoc_after_each(ADC1);

    adc_power_on(ADC1);
    // adc_power_on(ADC2);
    // adc_power_on(ADC3);

    nvic_enable_irq(NVIC_ADC_IRQ);
}

void exti_timer_setup()
{
    // Timer is used for deboucing
    // If output on trigger is the same 3ms later, accept as input
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
    timer_reset(TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM3, 300);
    timer_set_prescaler(TIM3, 1679);
    timer_set_clock_division(TIM3, TIM_CR1_CKD_CK_INT);
    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
    timer_enable_irq(TIM3, TIM_DIER_UIE);

    nvic_set_priority(NVIC_TIM3_IRQ, 10);
    nvic_enable_irq(NVIC_TIM3_IRQ);

    nvic_set_priority(NVIC_EXTI0_IRQ, 9);
    nvic_set_priority(NVIC_EXTI1_IRQ, 9);
    nvic_set_priority(NVIC_EXTI2_IRQ, 9);
    nvic_set_priority(NVIC_EXTI3_IRQ, 9);
    nvic_set_priority(NVIC_EXTI4_IRQ, 9);
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 9);
    nvic_set_priority(NVIC_EXTI15_10_IRQ, 9);
}

usbd_device *usbd_dev;

int send_int = 0;
char interrupt_buf[4]= {0};

uint8_t control_buffer[128] __attribute__((aligned (16)));

short *buffer_to_process=NULL;

void process_buffer(unsigned short *cur_buf)
{
    int i;
    unsigned pp_tot = 0, pv_tot = 0, pi_tot=0;

    for(i = 0; i < DMA_SHORTS; i+=2)
    {
        unsigned short c = cur_buf[i+1];
        unsigned short v = cur_buf[i];
        unsigned short p = c*v;
        a_data.energy_accum += p;
        pp_tot += p;
        pi_tot += c;
        pv_tot += v;

        a_data.n_samples += 1;
        a_data.elapsed_time += tperiod;

        // if(cur_buf[i] < 2000 || cur_buf[i] > 2100)
        // {
        //     // dma_disable_stream(DMA2, DMA_STREAM0);
        //     // while(1);
        // gpio_toggle(GPIOD, GPIO15);
        // }
        // gpio_toggle(GPIOD, GPIO14);
    }

    pp_tot /= DMA_SHORTS/2;
    pv_tot /= DMA_SHORTS/2;
    pi_tot /= DMA_SHORTS/2;

    if(pp_tot > a_data.peak_power)
        a_data.peak_power = pp_tot;
    if(pv_tot > a_data.peak_voltage)
        a_data.peak_voltage = pv_tot;
    if(pi_tot > a_data.peak_current)
        a_data.peak_current = pi_tot;

}

int main(void)
{
    int c_started=0, n, cpy;
    short s;

    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);

    // gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

    dma_setup();
    adc_setup();
    timer_setup();
    exti_timer_setup();

    gpio_toggle(GPIOA, GPIO12);


    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3, control_buffer, 128);
    usbd_register_set_config_callback(usbd_dev, usbdev_set_config);

    while (1)
    {

        usbd_poll(usbd_dev);

        if(send_int)
        {
            usbd_ep_write_packet(usbd_dev, 0x82, interrupt_buf, 4);
            send_int = 0;
        }

        if(buffer_to_process != NULL)
        {
            process_buffer(buffer_to_process);
            buffer_to_process = NULL;
        }
    }
}


int lastErr = 0;

void dma2_stream0_isr()
{
    int nhead,i;

    // if((DMA2_LISR & DMA_LISR_TCIF0) != 0)
    {
        unsigned voltage, b0, b1;
        short c, dptr;

        unsigned short * cur_buf;

        // dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_LISR_TCIF0);

        // if(adc_get_overrun_flag(ADC1))
        //     while(1);

        // if(dma_get_target(DMA2, DMA_STREAM0) == 0)
        //     cur_buf = dbuf1;
        // else
            cur_buf = dbuf0;

        if(buffer_to_process == NULL)
        {
            buffer_to_process = cur_buf;
        }
    }
    // else
    // {/
    //     while(1);
    // }
}

int status = -1;

void exti0_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti1_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti2_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti3_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti4_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti9_5_isr () __attribute__ ((weak, alias ("exti_isr")));
void exti15_10_isr () __attribute__ ((weak, alias ("exti_isr")));

void exti_isr()
{
    exti_reset_request(trigger_pin);

//    gpio_toggle(GPIOD, GPIO13);

    if(status == -1 || 1 )
    {
  //      gpio_toggle(GPIOD, GPIO12);
        send_int = 1;

        if (gpio_get(trigger_port, trigger_pin))
        {
            interrupt_buf[0] = 1;
            start_measurement();
        }
        else
        {
            interrupt_buf[0] = 2;
            stop_measurement();
        }

        // Timeout to ignore other spurious edges
        status = gpio_get(trigger_port, trigger_pin);
        timer_enable_counter(TIM3);
        timer_set_counter(TIM3, 0);
    }
}

void tim3_isr()
{
    TIM_SR(TIM3) &= ~TIM_SR_UIF;
    timer_disable_counter(TIM3);
    status = -1;
}

void exit(int a)
{
    while(1);
}

// TODO, calculations here instead of dma
int nbuff = 0;
unsigned last = 0;
void adc_isr()
{
    if(adc_eoc(ADC1))
    {
        dbuf0[nbuff++] = ADC1_DR;

        if(abs(dbuf0[nbuff-1] - last) < 20 && last != 0)
        {
            while(1);
        }
        last = dbuf0[nbuff-1];

        // if(nbuff > DMA_SHORTS)while(1);

        if(nbuff >= DMA_SHORTS)
        {
            nbuff = 0;

            if(buffer_to_process == NULL)
            {
                buffer_to_process = dbuf0;
            }
            else
                while(1);
        }
        adc_enable_eoc_interrupt(ADC1);
    }
    if(adc_get_overrun_flag(ADC1))
    {
    gpio_toggle(GPIOD, GPIO15);
        while(1);
    }
}
