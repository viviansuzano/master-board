
#define N_SLAVES 6
#define N_SLAVES_CONTROLED 3

struct raw_sensor_data {
uint16_t status;
uint16_t timestamp;
int32_t position[2];
int16_t velocity[2];
int16_t current[2];
uint16_t coil_resistance[2];
uint16_t adc[2];
}__attribute__ ((packed));

struct si_sensor_data {
	uint16_t status;
	uint16_t timestamp;// to be removed
	bool is_system_enabled; 
	bool is_motor_enabled[2];
	bool is_motor_ready[2];
	uint8_t error_code;  
	float position[2];
	float velocity[2];
	float current[2];
	float coil_resistance[2];
	float adc[2];

}__attribute__ ((packed));


struct wifi_eth_packet_sensor {
    struct raw_sensor_data raw_uDriver_sensor_data[N_SLAVES];
    uint8_t IMU[18]; //TODO create the appropriate struct
    uint16_t sensor_index;
    uint16_t last_index;
} __attribute__ ((packed));

struct wifi_eth_packet_command {
    uint16_t command[N_SLAVES][SPI_TOTAL_INDEX];
    uint16_t sensor_index;
}__attribute__ ((packed));