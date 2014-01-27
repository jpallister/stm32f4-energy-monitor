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

// Power data ///////////////////////////////////////////////////////

#define NUM_BUFFERS         128
#define PWR_SAMPLES         32

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

typedef struct {
    unsigned voltage;
    unsigned current;
    unsigned current_time;
} instant_data;

typedef struct {
    unsigned short data[PWR_SAMPLES];
    unsigned short idx;
} power_data;

int tperiod=500;

typedef struct {
    accumulated_data accum_data;
    instant_data id;
    power_data data_bufs[NUM_BUFFERS];

    int running; // Are we collecting measurements
    int number_of_runs;

    // Implement a circular buffer in data_bufs
    int head_ptr, tail_ptr;
    int trigger_port, trigger_pin;

    int assigned_adc;

    unsigned short lastI, lastV;

    unsigned char chans[2];
} measurement_point;

measurement_point m_points[4] = {0};

int adc_to_mpoint[3] = {-1, -1, -1};

// USB communication globals ////////////////////////////////////////
usbd_device *usbd_dev;

int send_int = 0;
char interrupt_buf[4]= {0};

uint8_t control_buffer[128] __attribute__((aligned (16)));

/////////////////////////////////////////////////////////////////////

void exti_setup(int m_point)
{
    int i;

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

    // if(m_points[m_point].trigger_port == -1)
        // return;

    for(i = 0; i < 4; ++i)
    {
        exti_select_source(m_points[i].trigger_pin, m_points[i].trigger_port);
        exti_set_trigger(m_points[i].trigger_pin, EXTI_TRIGGER_BOTH);
        exti_enable_request(m_points[i].trigger_pin);
        gpio_mode_setup(m_points[i].trigger_port, GPIO_MODE_INPUT, GPIO_PUPD_NONE, m_points[i].trigger_pin);

        switch(m_points[i].trigger_pin)
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
}

void start_measurement(int m_point)
{
    m_points[m_point].running = 1;
    m_points[m_point].head_ptr = 0;
    m_points[m_point].tail_ptr = 0;

    m_points[m_point].accum_data.energy_accum = 0;
    m_points[m_point].accum_data.elapsed_time = 0;
    m_points[m_point].accum_data.peak_power = 0;
    m_points[m_point].accum_data.peak_voltage = 0;
    m_points[m_point].accum_data.peak_current = 0;
    m_points[m_point].accum_data.n_samples = 0;
    m_points[m_point].accum_data.avg_voltage = 0;
    m_points[m_point].accum_data.avg_current = 0;

    switch(m_points[m_point].assigned_adc)
    {
        case 0:
            adc_set_regular_sequence(ADC1, 1, m_points[m_point].chans);
            adc_power_on(ADC1);
            break;
        case 1:
            adc_set_regular_sequence(ADC2, 1, m_points[m_point].chans);
            adc_power_on(ADC2);
            break;
        case 2:
            adc_set_regular_sequence(ADC3, 1, m_points[m_point].chans);
            adc_power_on(ADC3);
            break;
        case -1:
            error_condition(); return;
        default:
            error_condition(); return;
    }
}

void stop_measurement(int m_point)
{
    m_points[m_point].running = 0;
    m_points[m_point].number_of_runs++;

    switch(m_points[m_point].assigned_adc)
    {
        case 0: adc_off(ADC1); break;
        case 1: adc_off(ADC2); break;
        case 2: adc_off(ADC3); break;
        case -1:
            error_condition(); return;
        default:
            error_condition(); return;
    }
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

        start_measurement(req->wValue-1);
        *len = 0;
        break;
    }
    case 2:     // Stop
    {
        gpio_clear(GPIOD, GPIO12);
        stop_measurement(req->wValue-1);
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

        int m_point = (req->wValue >> 8) - 1;

        gpio_toggle(GPIOD, GPIO14);

        switch(req->wValue & 0xFF)
        {
            case 'A': m_points[m_point].trigger_port = GPIOA; break;
            case 'B': m_points[m_point].trigger_port = GPIOB; break;
            case 'C': m_points[m_point].trigger_port = GPIOC; break;
            case 'D': m_points[m_point].trigger_port = GPIOD; break;
            case 'E': m_points[m_point].trigger_port = GPIOE; break;
            case 'F': m_points[m_point].trigger_port = GPIOF; break;
            case 'G': m_points[m_point].trigger_port = GPIOG; break;
            case 'H': m_points[m_point].trigger_port = GPIOH; break;
            default:
                m_points[m_point].trigger_port = -1; break;
        }

        m_points[m_point].trigger_pin = 1 << (req->wIndex & 0xFF);

        if(m_points[m_point].trigger_port != GPIOA)
            gpio_toggle(GPIOD, GPIO12);

        exti_setup(m_point);
        break;
    }
    case 6:     // Get energy
    {
        *len = sizeof(accumulated_data);
        *buf = &m_points[req->wValue-1].accum_data;
        break;
    }
    case 7:     // Map ADC to measurement point
    {
        int adc = req->wIndex;
        int m_point = req->wValue - 1;

        m_points[m_point].assigned_adc = adc;
        adc_to_mpoint[adc] = m_point;
        break;
    }
    case 8:     // Is running
    {
        *len = sizeof(m_points[req->wValue-1].running);
        *buf = &m_points[req->wValue-1].running;
        break;
    }
    case 9:     // Get number of runs
    {
        *len = sizeof(m_points[req->wValue-1].number_of_runs);
        *buf = &m_points[req->wValue-1].number_of_runs;
        break;
    }
    case 10:    // Clear number of runs
    {
        m_points[req->wValue-1].number_of_runs = 0;
        break;
    }
    case 11:    // Get instantaneous
    {
        int m_point = req->wValue - 1;
        measurement_point *mp = &m_points[m_point];

        mp->id.current = mp->lastI;
        mp->id.voltage = mp->lastV;
        mp->id.current_time = mp->accum_data.elapsed_time;

        *len = sizeof(instant_data);
        *buf = &m_points[req->wValue-1].id;
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
        // if(buf[i] == 'S')
        // {
        //     running = 1;
        //     head_ptr = 0;
        //     tail_ptr = 0;
        //     timer_enable_counter(TIM2);
        //     adc_power_on(ADC1);
        // }
        // if(buf[i] == 'F')
        // {
        //     running = 0;
        //     timer_disable_counter(TIM2);
        //     adc_off(ADC1);
        // }
    }

    gpio_toggle(GPIOD, GPIO15);
}


static void usb_reset_cb()
{
    int i;

    for(i = 0; i < 4; ++i)
    {
        m_points[i].running = 0;
        m_points[i].trigger_port = -1;
        m_points[i].trigger_pin = -1;
        m_points[i].assigned_adc = -1;
        m_points[i].head_ptr = 0;
        m_points[i].tail_ptr = 0;
    }
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
    timer_enable_counter(TIM2);
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
    ADC2_CR1 = 0;
    ADC3_CR1 = 0;
    ADC1_CR2 = 0;
    ADC2_CR2 = 0;
    ADC3_CR2 = 0;

    adc_set_single_conversion_mode(ADC1);
    adc_set_single_conversion_mode(ADC2);
    adc_set_single_conversion_mode(ADC3);

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
    adc_set_regular_sequence(ADC2, 1, channels1);
    adc_set_regular_sequence(ADC3, 1, channels1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_15CYC);
    adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_15CYC);
    adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR_SMP_15CYC);
    adc_enable_external_trigger_regular(ADC1,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);
    adc_enable_external_trigger_regular(ADC2,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);
    adc_enable_external_trigger_regular(ADC3,ADC_CR2_EXTSEL_TIM2_TRGO, ADC_CR2_EXTEN_RISING_EDGE);

    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC2, ADC_CR1_RES_12BIT);
    adc_set_resolution(ADC3, ADC_CR1_RES_12BIT);

    adc_set_right_aligned(ADC1);
    adc_set_right_aligned(ADC2);
    adc_set_right_aligned(ADC3);

    adc_enable_overrun_interrupt(ADC1);
    adc_enable_overrun_interrupt(ADC2);
    adc_enable_overrun_interrupt(ADC3);

    adc_enable_eoc_interrupt(ADC1);
    adc_enable_eoc_interrupt(ADC2);
    adc_enable_eoc_interrupt(ADC3);
    adc_eoc_after_each(ADC1);
    adc_eoc_after_each(ADC2);
    adc_eoc_after_each(ADC3);

    nvic_set_priority(NVIC_ADC_IRQ, 0x10);
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

    nvic_set_priority(NVIC_TIM3_IRQ, 0x30);
    nvic_enable_irq(NVIC_TIM3_IRQ);

    nvic_set_priority(NVIC_EXTI0_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI1_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI2_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI3_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI4_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x40);
    nvic_set_priority(NVIC_EXTI15_10_IRQ, 0x40);
}

void error_condition()
{
    gpio_set(GPIOD, GPIO15);
    while(1);
}

void process_buffer(power_data *pd, accumulated_data *a_data)
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

        a_data->energy_accum += p;
        pp_tot += p;
        pi_tot += c;
        pv_tot += v;

        a_data->n_samples += 1;
        a_data->elapsed_time += tperiod;
        a_data->avg_voltage += v;
        a_data->avg_current += c;
    }

    pp_tot /= n_samples/2;
    pv_tot /= n_samples/2;
    pi_tot /= n_samples/2;

    if(pp_tot > a_data->peak_power)
        a_data->peak_power = pp_tot;
    if(pv_tot > a_data->peak_voltage)
        a_data->peak_voltage = pv_tot;
    if(pi_tot > a_data->peak_current)
        a_data->peak_current = pi_tot;

}

int cnt=0;

int main(void)
{
    int c_started=0, n, cpy, i, offset=0;
    short s;

    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
    rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

    for(i = 0;i < 4; ++i)
    {
        m_points[i].assigned_adc = -1;
        m_points[i].trigger_port = -1;
        m_points[i].trigger_pin = -1;
    }

    m_points[0].chans[0] = 2;
    m_points[0].chans[1] = 12;

    m_points[1].chans[0] = 3;
    m_points[1].chans[1] = 1;

    m_points[2].chans[0] = 9;
    m_points[2].chans[1] = 15;

    m_points[3].chans[0] = 8;
    m_points[3].chans[1] = 14;


    // dma_setup();
    adc_setup();
    timer_setup();
    exti_timer_setup();

    gpio_toggle(GPIOA, GPIO12);


    usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3, control_buffer, 128);
    usbd_register_set_config_callback(usbd_dev, usbdev_set_config);

    offset = 0;
    while (1)
    {
        usbd_poll(usbd_dev);
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
    int i;

    for(i = 0; i < 4; ++i)
        exti_reset_request(m_points[i].trigger_pin);

    if(status == -1 || 1 )
    {
        for(i = 0; i < 4; ++i)
        {
            if(m_points[i].trigger_port == -1 || m_points[i].trigger_pin == -1)
                continue;

            if(gpio_get(m_points[i].trigger_port, m_points[i].trigger_pin))
            {
                if(!m_points[i].running)
                start_measurement(i);
            }
            else
            {
                if(m_points[i].running)
                    stop_measurement(i);
            }
        }

        // Timeout to ignore other spurious edges
        // status = gpio_get(trigger_port, trigger_pin);
        // timer_enable_counter(TIM3);
        // timer_set_counter(TIM3, 0);
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
    int m_point;
    measurement_point *mp;
    int adcs[3] = {ADC1, ADC2, ADC3};
    int i;

    for(i = 0; i < 3; ++i)
    {
        if(adc_get_overrun_flag(adcs[i]))
            error_condition();

        if(adc_eoc(adcs[i]))
        {
            unsigned short val;

            m_point = adc_to_mpoint[i];
            if(m_point == -1)
                error_condition();

            // Get measurement point & buffer
            mp = &m_points[m_point];
            power_data *pd = &mp->data_bufs[mp->tail_ptr];

            // Read ADC
            val = ADC_DR(adcs[i]);
            pd->data[pd->idx] = val;

            // Save last value
            if(pd->idx&1)
                mp->lastI = val;
            else
                mp->lastV = val;

            if((pd->idx & 1) == 1)
            {
                accumulated_data *a_data = &mp->accum_data;
                unsigned short c = pd->data[1];
                unsigned short v = pd->data[0];
                unsigned p = c*v;

                a_data->energy_accum += p;

                a_data->n_samples += 1;
                a_data->elapsed_time += tperiod;
                a_data->avg_voltage += v;
                a_data->avg_current += c;

                if(p > a_data->peak_power)
                    a_data->peak_power = p;
                if(v > a_data->peak_voltage)
                    a_data->peak_voltage = v;
                if(c > a_data->peak_current)
                    a_data->peak_current = c;
            }

            pd->idx = 1-pd->idx;

            // HACK. Here we initialise the next channel to read from
            // because very occasionally the ADC seems to skip the next channel
            // suspect an odd timing bug, but only happens 1/10000000 times.
            unsigned char chan[1];

            // Select odd or even channel
            chan[0] = mp->chans[(pd->idx&1)];
            adc_set_regular_sequence(adcs[i], 1, chan);

            adc_enable_eoc_interrupt(adcs[i]);
        }
    }
}
