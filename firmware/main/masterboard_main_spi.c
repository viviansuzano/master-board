#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "driver/gpio.h"

#include "esp_timer.h"

#include "spi_manager.h"
#include "spi_quad_packet.h"
#include "quad_crc.h"
#include "uart_imu.h"
#include "ws2812_led_control.h"

#include "nvs_flash.h"
#include "esp_event.h"

#include "defines.h"

#define ENABLE_DEBUG_PRINTF false

#define SPI_AUTODETECT_MAX_COUNT 50 // number of spi transaction for which the master board will try to detect spi slaves

#define TEST_BIT(field, bit) (((field) & (1 << (bit))) >> (bit))

long int spi_count = 0;

bool spi_autodetect = true;
int spi_n_attempt = CONFIG_SPI_N_ATTEMPT;

uint8_t spi_connected = 0xff; // least significant bit: SPI0
                              // most significant bit: SPI7
                              // initialized at "every slave connected" so that sensor data can be gathered just after boot

long int spi_ok[CONFIG_N_SLAVES] = {0};

unsigned int ms_cpt = 0;

struct led_state ws_led;

static uint16_t spi_index_trans = 0;

static uint16_t spi_rx_packet[CONFIG_N_SLAVES][SPI_TOTAL_LEN + 1]; // +1 prevents any overflow //TODO understand why we need this?
static uint16_t spi_tx_packet[CONFIG_N_SLAVES][SPI_TOTAL_LEN];

bool send_zero_cmd = true; //If true, all cmd packet sent to udrivers ar set to 0.

void set_all_leds(uint32_t rgb)
{
    for (int led = 0; led < NUM_LEDS; led++)
    {
        ws_led.leds[led] = rgb;
    }
}

void print_spi_connected()
{
    printf("spi connected: [ ");
    for (int i = 0; i < CONFIG_N_SLAVES; i++)
    {
        printf("%d ", TEST_BIT(spi_connected, i));
    }
    printf("]\n\n");
}

void print_packet(uint8_t *data, int len)
{
    for (int i = 0; i < len; i++)
    {
/*        if (i % 4 == 0)*/
/*            printf(" ");*/
/*        if (i % 8 == 0)*/
/*            printf("\n\t\t");*/
        printf("%02X ", data[i]);
    }
    printf("\n");
}

static void periodic_timer_callback(void *arg)
{
    // Prepare spi transactions
    static bool spi_done[CONFIG_N_SLAVES] = {0}; // used to keep track of which spi transactions are done
    spi_index_trans++;
    send_zero_cmd = true;
    spi_count++;

    /* Prepare the outgoing spi TX packet */
    for (int i = 0; i < CONFIG_N_SLAVES; i++)
    {

        if (!TEST_BIT(spi_connected, i) && !spi_autodetect)
            continue; // ignoring this slave if it is not connected

        SPI_REG_u16(spi_tx_packet[i], SPI_COMMAND_MODE) = SPI_SWAP_DATA_TX(1<<15, 16); //In case of zero commands, keep the system enabled
        SPI_REG_32(spi_tx_packet[i], SPI_COMMAND_POS_1) = 0;
        SPI_REG_32(spi_tx_packet[i], SPI_COMMAND_POS_2) = 0;
        SPI_REG_16(spi_tx_packet[i], SPI_COMMAND_VEL_1) = 0;
        SPI_REG_16(spi_tx_packet[i], SPI_COMMAND_VEL_2) = 0;
        SPI_REG_16(spi_tx_packet[i], SPI_COMMAND_IQ_1) = 0;
        SPI_REG_16(spi_tx_packet[i], SPI_COMMAND_IQ_2) = 0;
        SPI_REG_u16(spi_tx_packet[i], SPI_COMMAND_KP_1) = 0;
        SPI_REG_u16(spi_tx_packet[i], SPI_COMMAND_KP_2) = 0;
        SPI_REG_u16(spi_tx_packet[i], SPI_COMMAND_KD_1) = 0;
        SPI_REG_u16(spi_tx_packet[i], SPI_COMMAND_KD_2) = 0;
        SPI_REG_u16(spi_tx_packet[i], SPI_COMMAND_ISAT_12) = 0;

        SPI_REG_u16(spi_tx_packet[i], SPI_TOTAL_INDEX) = SPI_SWAP_DATA_TX(spi_index_trans, 16);
        SPI_REG_u32(spi_tx_packet[i], SPI_TOTAL_CRC) = SPI_SWAP_DATA_TX(packet_compute_CRC(spi_tx_packet[i]), 32);

    }

    /* Perform every transaction the needed number of times */
    for (int spi_try = 0; spi_try < spi_n_attempt; spi_try++)
    {
		printf("\n>>> Attempt %ld:\n", spi_count);

        //for (int i = 0; i < CONFIG_N_SLAVES; i++)
		for (int i = 0; i < 4; i++)
        {
			printf("\n> Testing slave %d\n", i);

            if (!TEST_BIT(spi_connected, i) && !spi_autodetect) {
                continue; // ignoring this slave if it is not connected
			}

		    if (spi_done[i]) {
		        continue; // ignoring this slave if the transaction has already been done
			}
		    
			printf("Sending spi packet.\n");
			printf("TX packet:\n");
			print_packet(spi_tx_packet[i], SPI_TOTAL_LEN * 2);

		    spi_send(i, (uint8_t *)spi_tx_packet[i], (uint8_t *)spi_rx_packet[i], SPI_TOTAL_LEN * 2);
			printf("RX packet:\n");
			print_packet(spi_rx_packet[i], SPI_TOTAL_LEN * 2);

		    // checking if data is correct
		    if (packet_check_CRC(spi_rx_packet[i]))
		    {
				printf("Received correct packet from slave. SPI %d connected.\n", i);
		        spi_connected |= (1 << i); // noting that this slave is connected and working properly
		        spi_done[i] = true;
		        spi_ok[i]++;

		        //for debug:
		        if (ENABLE_DEBUG_PRINTF && spi_count % 1000 == 0 && i == 0)
		        {
		            printf("\nlast SENSOR packet:\n");
		            print_packet(spi_rx_packet[i], SPI_TOTAL_LEN * 2);
		        }
		    }
		    else
		    {
		        printf("Wrong CRC check.\n");
		    }
		}
    }

}

void setup_spi()
{
    spi_init();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "spi_send"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10000));

    gpio_set_direction(CONFIG_BUTTON_GPIO, GPIO_MODE_INPUT);

	// SPI auto-detection
	spi_autodetect = true;
    spi_connected = 0;
    memset(spi_ok, 0, CONFIG_N_SLAVES * sizeof(long int));
    spi_count = 0;
	spi_n_attempt = 1;
}

void app_main()
{
	printf("Starting...\n");
	uart_set_baudrate(UART_NUM_0, 2000000);
    nvs_flash_init();

	printf("Setting up leds.\n");
    ws2812_control_init(); //init the LEDs
    set_all_leds(0x0f0f0f);
    ws2812_write_leds(ws_led);

	printf("Setting up spi.\n");
    setup_spi();

	printf("Creating esp event loop.\n");
    ESP_ERROR_CHECK(esp_event_loop_create_default());

	printf("Setup done.\n");

    while (1)
    {
        vTaskDelay(10);
        ws2812_write_leds(ws_led);
    }
}
