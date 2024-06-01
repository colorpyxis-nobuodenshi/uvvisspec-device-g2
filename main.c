#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "opt.h"
#include "spi.h"
#include "i2c.h"
#include "VirtualSerial.h"

#define C12880MA_CLK PORTF1
#define C12880MA_TRG PORTF4
#define C12880MA_EOS PORTF0
#define C12880MA_ST PORTF5
#define C12880MA_PORT PORTF

#define digital_out_high(port, pin) \
    {                               \
        port |= (1 << pin);         \
    }
#define digital_out_low(port, pin) \
    {                              \
        port &= ~(1 << pin);       \
    }

static bool cdc_recived = 0;
static char cdc_recive_buffer[32];
static int cdc_recive_index = 0;

static float opticalPower[C12880MA_CHANELS] = {0};
static int darkopticalPower[C12880MA_CHANELS] = {0};
static float darkopticalPowerGain[EXPOSURE_TIME_SEL_N] = {1.0f};

#define STATUS_RUN 0
#define STATUS_WARN 1
#define STATUS_ERROR 2
#define AUTO_EXPOSURE_AUTO 1
#define AUTO_EXPOSURE_MANUAL 0

static int status = STATUS_RUN;
static int auto_exposure = AUTO_EXPOSURE_MANUAL;
static int exposure_time_sel = EXPOSURE_TIME_SEL_100us;
static long exposure_time = EXPOSURE_TIME_100us;
static float temperature = 25.0;
static float temperature_dark = 0.0;

static FILE USBSerialStream;

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
    {
        .Config =
            {
                .ControlInterfaceNumber = INTERFACE_ID_CDC_CCI,
                .DataINEndpoint =
                    {
                        .Address = CDC_TX_EPADDR,
                        .Size = CDC_TXRX_EPSIZE,
                        .Banks = 1,
                    },
                .DataOUTEndpoint =
                    {
                        .Address = CDC_RX_EPADDR,
                        .Size = CDC_TXRX_EPSIZE,
                        .Banks = 1,
                    },
                .NotificationEndpoint =
                    {
                        .Address = CDC_NOTIFICATION_EPADDR,
                        .Size = CDC_NOTIFICATION_EPSIZE,
                        .Banks = 1,
                    },
            },
        .State =
            {
                .LineEncoding =
                    {
                        .BaudRateBPS = 9600,
                        .CharFormat = CDC_LINEENCODING_OneStopBit,
                        .DataBits = 8,
                        .ParityType = CDC_PARITY_None,
                    }}};

void EVENT_USB_Device_Connect(void)
{
}

void EVENT_USB_Device_Disconnect(void)
{
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
}
void delay()
{
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
}
void adc_temp_init()
{
}

float adc_temp_read()
{
    int channel = 7;
    ADMUX = 0b01000000;
    ADMUX |= (channel & 0x1F);
    ADCSRB = 0b00000000;
    ADCSRB |= (((channel >> 5) & 1) << MUX5);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | 7;

    while ((ADCSRA & (1 << ADSC)))
        ;

    float v = (ADCL | (ADCH << 8)) & 0x3FF;

    const float resis = 10.0e3;
    const float resi0 = 10.0e3;
    const float tr25 = 25;
    const float K = 273.15;
    const float B = 3435;
    float vcc = 5.0;
    float vcoff = 5.0 / 1.023;
    v = v / 1023 * 5.0;

    float resi1 = (vcc - v) == 0 ? 0 : (v * resis) / (vcc - v);
    float t = 1 / (logf(resi1 / resi0) / B + (1 / (tr25 + K))) - K;

    return t;
}

void mcp3201_init()
{
    spi_init();
    DDRB |= (1 << PORTB0);
    PORTB |= (1 << PORTB0);
}
unsigned int mcp3201_read()
{
    PORTB &= ~(1 << PORTB0);
    asm volatile("nop");
    char high_byte = spi_transfer(0) & 0x1F;
    char low_byte = spi_transfer(0);
    PORTB |= (1 << PORTB0);

    return (((high_byte << 7) & 0xF80) | ((low_byte >> 1) & 0x7F)) & 0xFFF;
}

static float conv_wl(int index)
{
    float a1 = pgm_read_float(&wccoeff[0]);
    float a2 = pgm_read_float(&wccoeff[1]);
    float a3 = pgm_read_float(&wccoeff[2]);
    float a4 = pgm_read_float(&wccoeff[3]);
    float a5 = pgm_read_float(&wccoeff[4]);
    float a6 = pgm_read_float(&wccoeff[5]);
    int i = index + 1;

    return a1 + a2 * i + a3 * i * i + a4 * i * i * i + a5 * i * i * i * i + a6 * i * i * i * i * i;
}

static float linear(int index)
{
    int i = index;
    int x = pgm_read_word(&wl_1nm[i]);
    int j = pgm_read_word(&wl_lut_1nm[i]);
    float x1 = conv_wl(j - 1);
    float x2 = conv_wl(j);
    float y1 = opticalPower[j - 1];
    float y2 = opticalPower[j];

    float y = lerp(x1, y1, x2, y2, x);
    if (y < 0.0f)
    {
        y = 0.0f;
    }
    return y;
}

static float lagrange(int index)
{
    float x = pgm_read_word(&wl_1nm[index]);
    int j = pgm_read_word(&wl_lut_1nm[index]);

    if(index < 2)
    {
        return opticalPower[0];
    }

    float x0 = conv_wl(j - 2);
    float x1 = conv_wl(j - 1);
    float x2 = conv_wl(j);
    float x3 = conv_wl(j + 1);
    float y0 = opticalPower[j - 2];
    float y1 = opticalPower[j - 1];
    float y2 = opticalPower[j];
    float y3 = opticalPower[j + 1];

    float y = ((x-x1)*(x-x2)*(x-x3))/((x0-x1)*(x0-x2)*(x0-x3))*y0
    +((x-x0)*(x-x2)*(x-x3))/((x1-x0)*(x1-x2)*(x1-x3))*y1
    +((x-x0)*(x-x1)*(x-x3))/((x2-x0)*(x2-x1)*(x2-x3))*y2
    +((x-x0)*(x-x1)*(x-x2))/((x3-x0)*(x3-x1)*(x3-x2))*y3;

    if(y < 0)
    {
        y = 0;
    }

    return y;
}

void c12880ma_init()
{
    DDRF |= (1 << C12880MA_CLK);
    PORTF &= ~(1 << C12880MA_CLK);
    DDRF |= (1 << C12880MA_ST);
    PORTF &= ~(1 << C12880MA_ST);

    for (int i = 0; i < C12880MA_CHANELS; i++)
    {
        opticalPower[i] = 0;
    }
}

void c12880ma_read()
{
    for (int i = 0; i < C12880MA_CHANELS; i++)
    {
        opticalPower[i] = 0;
    }

    digital_out_low(C12880MA_PORT, C12880MA_CLK);
    delay();
    digital_out_high(C12880MA_PORT, C12880MA_CLK);
    delay();
    digital_out_low(C12880MA_PORT, C12880MA_CLK);
    digital_out_high(C12880MA_PORT, C12880MA_ST);
    delay();

    for (long i = 0; i < exposure_time; i++)
    {
        digital_out_high(C12880MA_PORT, C12880MA_CLK);
        delay();
        digital_out_low(C12880MA_PORT, C12880MA_CLK);
        delay();
    }

    digital_out_low(C12880MA_PORT, C12880MA_ST);

    for (int i = 0; i < 88; i++)
    {
        digital_out_high(C12880MA_PORT, C12880MA_CLK);
        delay();
        digital_out_low(C12880MA_PORT, C12880MA_CLK);
        delay();
    }

    for (int i = 0; i < C12880MA_CHANELS; i++)
    {
        opticalPower[i] = (float)mcp3201_read();
        digital_out_high(C12880MA_PORT, C12880MA_CLK);
        delay();
        digital_out_low(C12880MA_PORT, C12880MA_CLK);
        delay();
    }

    for (int i = 0; i < 32; i++)
    {
        //mcp3201_read();
        digital_out_high(C12880MA_PORT, C12880MA_CLK);
        delay();
        digital_out_low(C12880MA_PORT, C12880MA_CLK);
        delay();
    }

    float sat = 3890;//4095 * 95 / 100;
    for (int i = 0; i < C12880MA_CHANELS; i++)
    {
        if(opticalPower[i] > sat)
        {
            opticalPower[i] = sat;
        }
    }
}

// void c12880ma_read_integ()
// {
//     static int opticalPowerInteg[C12880MA_CHANELS];
//     int integ = 5;
    
//     for(int i=0;i<C12880MA_CHANELS;i++)
//     {
//         opticalPowerInteg[i] = 0;
//     }

//     for(int j = 0; j < integ; j++)
//     {
//         c12880ma_read();
//         for(int i = 0; i < C12880MA_CHANELS; i++)
//         {
//             opticalPowerInteg[i] += opticalPower[i]; 
//         }    
//     }
//     for(int i = 0; i < C12880MA_CHANELS; i++)
//     {
//         opticalPower[i] = opticalPowerInteg[i] / integ; 
//     }
// }

int check_status()
{
    if (temperature > 40.0f)
    {
        return STATUS_ERROR;
    }
    if (fabs(temperature_dark - temperature) >= 2.0f)
    {
        return STATUS_WARN;
    }

    return STATUS_RUN;
}

void SetupHardware(void)
{
    // MCUCR = 0x80;
    // MCUCR = 0x80;

    c12880ma_init();
    mcp3201_init();
    adc_temp_init();

    MCUSR &= ~(1 << WDRF);
    wdt_disable();
    clock_prescale_set(clock_div_1);

    USB_Init();
}

void correct()
{
    float sat = 1.0f;
    for (int i = 0; i < C12880MA_CHANELS; i++)
    {
        sat = MIN(sat, 3890.0f / pgm_read_float(&unitcoeff[exposure_time_sel]) / pgm_read_float(&spectralsensitivitycoeff[i]));
    }

    for (int i = 0; i < C12880MA_CHANELS; i++)
    {
        float value = opticalPower[i] - darkopticalPower[i] * darkopticalPowerGain[exposure_time_sel];
        value = value / pgm_read_float(&unitcoeff[exposure_time_sel]) / pgm_read_float(&spectralsensitivitycoeff[i]);
        if (value < 1e-9f)
        {
            value = 0.0f;
        }
        if(value > sat)
        {
            value = sat;
        }
        opticalPower[i] = value;
    }
}

long select_exposure_time(int sel)
{
    switch (sel)
    {
    case EXPOSURE_TIME_SEL_100us:
        return EXPOSURE_TIME_100us;
    case EXPOSURE_TIME_SEL_200us:
        return EXPOSURE_TIME_200us;
    case EXPOSURE_TIME_SEL_500us:
        return EXPOSURE_TIME_500us;
    case EXPOSURE_TIME_SEL_1ms:
        return EXPOSURE_TIME_1ms;
    case EXPOSURE_TIME_SEL_2ms:
        return EXPOSURE_TIME_2ms;
    case EXPOSURE_TIME_SEL_5ms:
        return EXPOSURE_TIME_5ms;
    case EXPOSURE_TIME_SEL_10ms:
        return EXPOSURE_TIME_10ms;
    case EXPOSURE_TIME_SEL_20ms:
        return EXPOSURE_TIME_20ms;
    case EXPOSURE_TIME_SEL_50ms:
        return EXPOSURE_TIME_50ms;
    case EXPOSURE_TIME_SEL_100ms:
        return EXPOSURE_TIME_100ms;
    default:
        return EXPOSURE_TIME_100us;
    }
}
void measure()
{
    if (auto_exposure)
    {
        for (int i = 0; i < EXPOSURE_TIME_SEL_N; i++)
        {
            exposure_time = select_exposure_time(i);
            exposure_time_sel = i;
            c12880ma_read();
            //c12880ma_read_integ();
            float max = 0;
            float th = 1400.0f;
            for (int i = 0; i < C12880MA_CHANELS; i++)
            {
                max = MAX(max, opticalPower[i]);
            }
            if (max >= th)
                break;
        }
    }
    else
    {
        exposure_time = select_exposure_time(exposure_time_sel);
        c12880ma_read();
        //c12880ma_read_integ();
    }
}

void dark()
{
    for (int i = 0; i < EXPOSURE_TIME_SEL_N; i++)
    {

        exposure_time = select_exposure_time(i);
        c12880ma_read();
        //c12880ma_read_integ();
        
        for(int j=0;j<C12880MA_CHANELS;j++)
        {
            if(j == 0)
            {
                opticalPower[j] = (-3.0f * opticalPower[j] + 12.0f * opticalPower[j] + 17.0f * opticalPower[j] + 12.0f * opticalPower[j+1] - 3.0f * opticalPower[j+2]) / 35.0f;
            }
            else if(j == 1)
            {
                opticalPower[j] = (-3.0f * opticalPower[j-1] + 12.0f * opticalPower[j-1] + 17.0f * opticalPower[j] + 12.0f * opticalPower[j+1] - 3.0f * opticalPower[j+2]) / 35.0f;
            }
            else if(j == C12880MA_CHANELS - 2)
            {
                opticalPower[j] = (-3.0f * opticalPower[j-2] + 12.0f * opticalPower[j-1] + 17.0f * opticalPower[j] + 12.0f * opticalPower[j+1] - 3.0f * opticalPower[j+1]) / 35.0f;
            }
            else if(j == C12880MA_CHANELS - 1)
            {
                opticalPower[j] = (-3.0f * opticalPower[j-2] + 12.0f * opticalPower[j-1] + 17.0f * opticalPower[j] + 12.0f * opticalPower[j] - 3.0f * opticalPower[j]) / 35.0f;
            }
            else
            {
                opticalPower[j] = (-3.0f * opticalPower[j-2] + 12.0f * opticalPower[j-1] + 17.0f * opticalPower[j] + 12.0f * opticalPower[j+1] - 3.0f * opticalPower[j+2]) / 35.0f;
            }
        }

        darkopticalPowerGain[i] = 0.0f;
        for (int j = 0; j < C12880MA_CHANELS; j++)
        {
            if (i == EXPOSURE_TIME_SEL_1ms)
            {
                darkopticalPower[j] = opticalPower[j];
            }
            darkopticalPowerGain[i] += opticalPower[j];
        }
        //darkopticalPowerGain[i] /= (float)C12880MA_CHANELS;
    }
    float x = darkopticalPowerGain[EXPOSURE_TIME_SEL_1ms];
    for (int i = 0; i < EXPOSURE_TIME_SEL_N; i++)
    {
        darkopticalPowerGain[i] /= x;
    }
    temperature_dark = adc_temp_read();
}

void CDC_Recive_Event()
{
    int d = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
    if (d > 0)
    {
        cdc_recive_buffer[cdc_recive_index] = d;
        cdc_recive_index++;
        if (d == '\n')
        {
            cdc_recived = true;
        }
    }
}

void CDC_Recive_Event_Process()
{
    if (cdc_recived)
    {
        cdc_recived = false;
        char *message = cdc_recive_buffer;

        if (strncmp(message, "MEAS\n", 5) == 0)
        {
            measure();
            correct();

            for (int i = 0; i < C12880MA_CHANELS; i++)
            {
#ifdef CP150
                if (i >= pgm_read_word(&wl_lut_1nm[0]) - 1 && i <= pgm_read_word(&wl_lut_1nm[470]))
#endif
#ifdef CP160
                    if (i >= pgm_read_word(&wl_lut_1nm[0]) - 1 && i <= pgm_read_word(&wl_lut_1nm[490]))
#endif
#ifdef CP180
                    if (i >= pgm_read_word(&wl_lut_1nm[0]) - 1 && i <= pgm_read_word(&wl_lut_1nm[490]))
#endif
                    {
                        float wl = conv_wl(i);
                        char msg[32] = {0};
                        sprintf(msg, "%f:%g\r", wl, opticalPower[i]);
                        CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
                    }
            }
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else if (strncmp(message, "MEAS1nm\n", 9) == 0)
        {
            measure();
            correct();
#ifdef CP150
            for (int i = 0; i < 471; i++)
#endif
#ifdef CP160
                for (int i = 0; i < 491; i++)
#endif
#ifdef CP180
                for (int i = 0; i < 491; i++)
#endif
                {
                    
                    float op = lagrange(i);
                    int wl = pgm_read_word(&wl_1nm[i]);
                    char msg[32] = {0};
                    sprintf(msg, "%d:%g\r", wl, op);
                    CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
                    wl += 1;
                }
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else if (strncmp(message, "RAW", 3) == 0)
        {
            measure();

            for (int i = 0; i < C12880MA_CHANELS; i++)
            {
                char msg[16] = {0};
                sprintf(msg, "%d\r", (int)opticalPower[i]);
                CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
            }
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        }
        else if (strncmp(message, "DARK", 4) == 0)
        {
            dark();
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, "ACK\n");
            // char msg[100] = {0};
            // sprintf(msg, "100us %g\n200us %g\n500us %g\n1ms %g\n2ms %g\n5ms %g\n10ms %g\n20ms %g\n50ms %g\n100ms %g\n", darkopticalPowerGain[0],
            //     darkopticalPowerGain[1],darkopticalPowerGain[2],darkopticalPowerGain[3],
            //     darkopticalPowerGain[4],darkopticalPowerGain[5],darkopticalPowerGain[6],
            //     darkopticalPowerGain[7],darkopticalPowerGain[8],darkopticalPowerGain[9]);
            // CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
    
        }
        // else if(strncmp(message,"SICF", 4) == 0)
        // {
        //     for(int i=0;i<C12880MA_CHANELS;i++)
        //     {
        //         char msg[16] = {0};
        //         sprintf(msg, "%g\r", pgm_read_float(&spectralsensitivitycoeff[i]));
        //         CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        //     }
        //     CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n');
        // }
        else if (strncmp(message, "MODEL?", 6) == 0)
        {

            char msg[32] = {0};
            sprintf(msg, "MODEL/%s\n", (char*)pgm_read_word(&model[0]));
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        }
        else if (strncmp(message, "NAME?", 5) == 0)
        {

            char msg[32] = {0};
            sprintf(msg, "NAME/%s\n", (char*)pgm_read_word(&name[0]));
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        }
        else if (strncmp(message, "SN?", 3) == 0)
        {

            char msg[32] = {0};
            sprintf(msg, "SN/%s\n", (char*)pgm_read_word(&serialnumber[0]));
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        }
        else if (strncmp(message, "WCCF", 3) == 0)
        {

            char msg[100] = {0};
            sprintf(msg, "WCCF/%g:%g:%g:%g:%g:%g\n", pgm_read_float(&wccoeff[0]), pgm_read_float(&wccoeff[1]), pgm_read_float(&wccoeff[2]), pgm_read_float(&wccoeff[3]), pgm_read_float(&wccoeff[4]), pgm_read_float(&wccoeff[5]));
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        }
        else if (strncmp(message, "ST?", 3) == 0)
        {

            int msg[32] = {0};
            char st = status == STATUS_RUN ? 'R' : status == STATUS_WARN ? 'W'
                                                                         : 'E';
            sprintf(msg, "ST/%c:%.3f\n", st, temperature);
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, msg);
        }
        else if (strncmp(message, "EXP/", 4) == 0)
        {
            if (strncmp(message, "EXP/AUTO", 8) == 0)
            {
                auto_exposure = 1;
            }
            else if (strncmp(message, "EXP/100us", 9) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_100us;
            }
            else if (strncmp(message, "EXP/200us", 9) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_200us;
            }
            else if (strncmp(message, "EXP/500us", 9) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_500us;
            }
            else if (strncmp(message, "EXP/1ms", 7) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_1ms;
            }
            else if (strncmp(message, "EXP/2ms", 7) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_2ms;
            }
            else if (strncmp(message, "EXP/5ms", 7) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_5ms;
            }
            else if (strncmp(message, "EXP/10ms", 8) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_10ms;
            }
            else if (strncmp(message, "EXP/20ms", 8) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_20ms;
            }
            else if (strncmp(message, "EXP/50ms", 8) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_50ms;
            }
            else if (strncmp(message, "EXP/100ms", 9) == 0)
            {
                auto_exposure = 0;
                exposure_time_sel = EXPOSURE_TIME_SEL_100ms;
            }
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, "ACK\n");
        }
        else
        {
            CDC_Device_SendString(&VirtualSerial_CDC_Interface, "NAK\n");
        }
        memset(cdc_recive_buffer, 0, 32);
        cdc_recive_index = 0;
    }
}

int main(void)
{
    SetupHardware();

    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    GlobalInterruptEnable();

    c12880ma_read();

    temperature = adc_temp_read();

    for (;;)
    {
        temperature = adc_temp_read();
        status = check_status();

        CDC_Recive_Event_Process();
        CDC_Recive_Event();

        CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
        USB_USBTask();
    }
}
