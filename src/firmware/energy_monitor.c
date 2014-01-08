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

// Accumulated data, to be sent back ////////////////////////////////

typedef struct {
    uint64_t energy_accum;
    uint64_t elapsed_time;
    unsigned peak_power;
    unsigned peak_voltage;
    unsigned peak_current;
    unsigned n_samples;
    uint64_t avg_current;
    uint64_t avg_voltage;
} accumulated_data;

accumulated_data a_data;

int tperiod=500;


// Power data temporary storage /////////////////////////////////////

#define NUM_BUFFERS         64
#define DMA_SHORTS          128

typedef struct {
    unsigned short data[DMA_SHORTS];
    unsigned short idx;
} power_data;

power_data data_bufs[NUM_BUFFERS] = {0};

int running = 0; // Are we collecting measurements

// Implement a circular buffer in data_bufs
int head_ptr = 0, tail_ptr = 0;
int trigger_port = -1, trigger_pin = -1;

// USB communication globals ////////////////////////////////////////
usbd_device *usbd_dev;

int send_int = 0;
char interrupt_buf[4]= {0};

uint8_t control_buffer[128] __attribute__((aligned (16)));

/////////////////////////////////////////////////////////////////////

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

    a_data.energy_accum = 0;
    a_data.elapsed_time = 0;
    a_data.peak_power = 0;
    a_data.peak_voltage = 0;
    a_data.peak_current = 0;
    a_data.n_samples = 0;
    a_data.avg_voltage = 0;
    a_data.avg_current = 0;

    tperiod = 500;

    adc_power_on(ADC1);

    timer_set_period(TIM2, tperiod);
    timer_enable_counter(TIM2);

    // adc_power_on(ADC2);
    // adc_power_on(ADC3);
    gpio_set(GPIOD, GPIO12);
}

void stop_measurement()
{
    running = 0;
    timer_disable_counter(TIM2);
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
    case 7:     // Map ADC to measurement point
    {
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
    ADC1_CR1 = 0;
    ADC1_CR2 = 0;

    adc_set_single_conversion_mode(ADC1);

    // Input 1
    uint8_t channels1[] = {2, 12};   // CH2 Voltage, PA2, ADC123
    // uint8_t channels1[] = {ADC_CHANNEL2, ADC_CHANNEL12};   // Voltage, PA2, ADC123
    // uint8_t channels2[] = {ADC_CHANNEL12};  // Ch12 Current, PC2, ADC123
    // Input 2
    // uint8_t channels1[] = {ADC_CHANNEL3};   // Voltage, PA3, ADC123
    // uint8_t channels2[] = {ADC_CHANNEL1};   // Current, PA1, ADC123
    // Input 3
    // uint8_t channels1[] = {ADC_CHANNEL9};   // Voltage, PB1, ADC12
    // uint8_t channels2[] = {ADC_CHANNEL15};  // Current, PC5, ADC12
    // Input self
    // uint8_t channels1[] = {ADC_CHANNEL8};   // Voltage, PB0, ADC12
    // uint8_t channels2[] = {ADC_CHANNEL14};  // Current, PC4, ADC12
    adc_set_regular_sequence(ADC1, 1, channels1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_15CYC);
    adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC3, ADC_CR1_RES_12BIT);

    adc_set_right_aligned(ADC1);

    adc_enable_overrun_interrupt(ADC1);

    adc_enable_eoc_interrupt(ADC1);
    adc_eoc_after_each(ADC1);

    adc_power_on(ADC1);

    nvic_set_priority(NVIC_ADC_IRQ, 0xF);
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

void error_condition()
{
    gpio_set(GPIOD, GPIO15);
    while(1);
}

void process_buffer(power_data *pd)
{
    int i;
    unsigned pp_tot = 0, pv_tot = 0, pi_tot=0;

    // Subtract 1 incase we have an unpaired sample at the end
    unsigned short n_samples = pd->idx - 1;

    for(i = 0; i < n_samples; i+=2)
    {
        unsigned short c = pd->data[i+1];
        unsigned short v = pd->data[i];
        unsigned p = c*v;

        a_data.energy_accum += p;
        pp_tot += p;
        pi_tot += c;
        pv_tot += v;

        a_data.n_samples += 1;
        a_data.elapsed_time += tperiod;
        a_data.avg_voltage += v;
        a_data.avg_current += c;
    }

    pp_tot /= n_samples/2;
    pv_tot /= n_samples/2;
    pi_tot /= n_samples/2;

    if(pp_tot > a_data.peak_power)
        a_data.peak_power = pp_tot;
    if(pv_tot > a_data.peak_voltage)
        a_data.peak_voltage = pv_tot;
    if(pi_tot > a_data.peak_current)
        a_data.peak_current = pi_tot;

}

int cnt=0;

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

    // dma_setup();
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

        if(head_ptr != tail_ptr)
        {
            process_buffer(&data_bufs[head_ptr]);

            head_ptr++;
            if(head_ptr >= NUM_BUFFERS)
                head_ptr = 0;
        }
    }
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

void adc_isr()
{
    if(adc_get_overrun_flag(ADC1))
    {
        error_condition();
    }
    if(adc_eoc(ADC1))
    {
        power_data *pd = &data_bufs[tail_ptr];
        pd->data[pd->idx++] = ADC1_DR;

        if(pd->idx >= DMA_SHORTS)
        {
            tail_ptr = (tail_ptr + 1);
            if(tail_ptr >= NUM_BUFFERS)
                tail_ptr = 0;

            data_bufs[tail_ptr].idx = 0;

            if(tail_ptr == head_ptr)
                error_condition();
        }

        // HACK. Here we initialise the next channel to read from
        // because very occasionally the ADC seems to skip the next channel
        // suspect an odd timing bug, but only happens 1/10000000 times.
        unsigned char chan[1];

        if((pd->idx&1) == 0)
            chan[0] = 2;
        else
            chan[0] = 12;
        adc_set_regular_sequence(ADC1, 1, chan);
        adc_enable_eoc_interrupt(ADC1);
    }

}
