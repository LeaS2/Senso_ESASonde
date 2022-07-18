#include 	"mbed.h"
#include 	"MS5611_01BA03.h"
#include 	"USBSerial.h"
#include 	"EthernetInterface.h"
#include    "INA219.h"
#include	"net_com.h"
#include 	"HTU21D.h"
#include    "Honeywell_RSC.h"
#include    "icm20948_dmp.h"
#include    <cstring>
#include    "sensor_config.h"

#define IP_ADDRESS         "192.168.000.003"
#define NETMASK_ADDRESS    "192.168.000.001"
#define GATEWAY_ADDRESS    "255.255.255.000"
#define IP_ADDRESS_SOCKET  "192.168.000.005"
#define ECHO_SERVER_PORT   	7
#define DIAG_PORT   		8
#define FLAG_CONVERSATION_SENSORS                 1
#define FLAG_CONVERSATION_ETHERNET                2
#define FLAG_DIAG           2
#define BOARD_ID			0x03
#define USB_SERIAL			0
EthernetInterface eth;
#if (USB_SERIAL == 1)
USBSerial usb_serial;
#endif
rtos::EventFlags event_flags;

uint32_t JumpAddress;
SPI spi1(PB_5,PA_6,PA_5);
SPI spi4(PE_6,PE_5,PE_2);
SPI spi2(PB_15,PB_14,PD_3);
SPI spi3(PG_14,PG_12,PG_13);
SPI spi5(PF_9,PF_8,PF_7);
SPI spi6(PG_14,PG_12,PG_13);
I2C i2c2(PF_0,PF_1);
I2C i2c4(PF_15,PF_14);


Honeywell_RSC pressure_sensor1(&spi4, PF_2, PG_2);
Honeywell_RSC pressure_sensor2(&spi2, PF_3, PG_3);
Honeywell_RSC pressure_sensor3(&spi2, PF_4, PG_4);
Honeywell_RSC pressure_sensor4(&spi2, PC_0, PG_5);
Honeywell_RSC pressure_sensor5(&spi2, PA_3, PG_6);
MS5611_01BA03 pressure_sensor6(&spi6, PD_4);
MS5611_01BA03 pressure_sensor7(&spi5, PF_10);



struct data_frame		// Struct dynamisch anpassen je nach Platinenversion
{
	uint32_t counter;
	uint8_t id;
	uint32_t timestamp;
	float sensor1;
	float sensor2;
	float sensor3;
	float sensor4;
	float sensor5;
	int32_t sensor6;
	int32_t sensor7;
	float temp1;
	float temp2;
	float temp3;
	float temp4;
	float temp5;
	int32_t temp6;
	int32_t temp7;
};

struct data_frame sensor_data;
struct data_frame ethernet_data;
struct data_frame exchange_data;

Thread  	thread1;
Thread 		thread2;
Thread 		thread3;

uint8_t sensor_thread_delay1;
uint8_t sensor_thread_delay2;

struct diag_data
{
	uint8_t id;
	uint8_t data[8];
};



/*
 *Thread zum Auslesen der Sensordaten
 */
void sensor_thread() {
	int32_t temp = 0;
	sensor_data.id = 0x01;
	while (true)
    {

		pressure_sensor1.write_command_adc_read(TEMPERATURE, &temp);
		sensor_data.sensor1 = pressure_sensor1.calculate_pressure(temp);

		pressure_sensor2.write_command_adc_read(TEMPERATURE, &temp);
		sensor_data.sensor2 = pressure_sensor2.calculate_pressure(temp);

		pressure_sensor3.write_command_adc_read(TEMPERATURE, &temp);
		sensor_data.sensor3 = pressure_sensor3.calculate_pressure(temp);

		pressure_sensor4.write_command_adc_read(TEMPERATURE, &temp);
		sensor_data.sensor4 = pressure_sensor4.calculate_pressure(temp);

		pressure_sensor5.write_command_adc_read(TEMPERATURE, &temp);
		sensor_data.sensor5 = pressure_sensor5.calculate_pressure(temp);

		pressure_sensor6.read_conv_press();
		pressure_sensor7.read_conv_press();
    	pressure_sensor6.start_conv_temp();
    	pressure_sensor7.start_conv_temp();

    	rtos::ThisThread::sleep_for(sensor_thread_delay1);			// !!! auf 50 für 20 SPS

		pressure_sensor1.write_command_adc_read(PRESSURE, &temp);
		sensor_data.temp1 = pressure_sensor1.calculate_temperature(temp);

		pressure_sensor2.write_command_adc_read(PRESSURE, &temp);
		sensor_data.temp2 = pressure_sensor2.calculate_temperature(temp);

		pressure_sensor3.write_command_adc_read(PRESSURE, &temp);
		sensor_data.temp3 = pressure_sensor3.calculate_temperature(temp);

		pressure_sensor4.write_command_adc_read(PRESSURE, &temp);
		sensor_data.temp4 = pressure_sensor4.calculate_temperature(temp);

		pressure_sensor5.write_command_adc_read(PRESSURE, &temp);
		sensor_data.temp5 = pressure_sensor5.calculate_temperature(temp);

		pressure_sensor6.read_conv_temp();
		pressure_sensor7.read_conv_temp();
		pressure_sensor6.start_conv_press();
		pressure_sensor7.start_conv_press();
		pressure_sensor6.calculate();
		pressure_sensor7.calculate();

		sensor_data.sensor6 = pressure_sensor6.getPressure();
		sensor_data.temp6 = pressure_sensor6.getTemperature();

		sensor_data.sensor7 = pressure_sensor7.getPressure();
		sensor_data.temp7 = pressure_sensor7.getTemperature();

#if (USB_SERIAL == 1)
	    usb_serial.printf("p1: %.2f\t t1: %.2f\tp2: %.2f\tt2: %.2f\tp3: %.2f\tt3: %.2f\tp4: %.2f\tt4: %.2f\tp5: %.2f\tt5: %.2f\tp6: %.2f\tt6: %.2f\tp7: %.2f\tt7: %.2f\n\r", tx_data.sensor1, tx_data.temp1, tx_data.sensor2, tx_data.temp2, tx_data.sensor3, tx_data.temp3, tx_data.sensor4, tx_data.temp4, tx_data.sensor5, tx_data.temp5, tx_data.sensor6, tx_data.temp6, tx_data.sensor7, tx_data.temp7);
#endif

	    event_flags.wait_any(FLAG_CONVERSATION_ETHERNET);

        //copy sensor values in exchange buffer
	    std::memcpy(&exchange_data, &sensor_data, sizeof(struct data_frame));

	    event_flags.set(FLAG_CONVERSATION_SENSORS);

	    rtos::ThisThread::sleep_for(sensor_thread_delay2);						// !!! auf 50 für 20 SPS
    }
}
/*
* Thread zum Senden der Daten via Ethernet
*/

void ethernet_thread(Net_com* net_com) {

	static uint32_t packages = 0;

	while (true)
    {
		event_flags.wait_any(FLAG_CONVERSATION_SENSORS);		// Funktion wartet bis alle Sensordaten ausgelesen sind

		exchange_data.timestamp = us_ticker_read() / 1000u;
		exchange_data.counter = packages++;
		std::memcpy(&ethernet_data, &exchange_data,  sizeof(struct data_frame));
		event_flags.set(FLAG_CONVERSATION_ETHERNET);

		net_com->net_com_sendto(&ethernet_data,  sizeof(struct data_frame));	// erst dann werden Daten verschickt

    }
}

/*
 *  Callback for diagnostic via Ethernet
 */
void net_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	struct diag_data* recv_diag = (struct diag_data*)(p->payload);
	struct sensor_config sensor_thread_config;

	switch(recv_diag->id)
	{
		//get Session
		case 0x30:
			//usb_serial.printf("get session\n\r");
			recv_diag->id = recv_diag->id + 0x20;
			recv_diag->data[0] = 0x02;

		break;

		//start Bootloader
		case 0x40:
			//usb_serial.printf("starting Bootloader\n\r");
			SCB->VTOR = (FLASH_BASE);
			JumpAddress = *(__IO uint32_t*) (FLASH_BASE);
			mbed_start_application(FLASH_BASE);

		break;

		//get board ID
		case 0x50:
			recv_diag->id = recv_diag->id + 0x20;
			recv_diag->data[0] = BOARD_ID;
		break;

		//start sending sensor data
		case 0x60:
			recv_diag->id = recv_diag->id + 0x20;
			thread1.start(sensor_thread);
			thread2.start(sensor_thread);
		break;

		//stop sending sensor data
		case 0x70:
			recv_diag->id = recv_diag->id + 0x20;
			thread1.terminate();
			thread2.terminate();
		break;
		//get error informations
		case 0x80:
			recv_diag->id = recv_diag->id + 0x20;
		break;

		//set the config parameter for sensor thread
		case 0x90:
			recv_diag->id = recv_diag->id + 0x20;
			std::memcpy(&sensor_thread_config, recv_diag->data, sizeof(struct sensor_config));

			thread1.terminate();
			thread2.terminate();

			pressure_sensor1.set_data_rate((RSC_DATA_RATE)sensor_thread_config.datarate);
			pressure_sensor1.set_mode((RSC_MODE)sensor_thread_config.mode);
			pressure_sensor2.set_data_rate((RSC_DATA_RATE)sensor_thread_config.datarate);
			pressure_sensor2.set_mode((RSC_MODE)sensor_thread_config.mode);
			pressure_sensor3.set_data_rate((RSC_DATA_RATE)sensor_thread_config.datarate);
			pressure_sensor3.set_mode((RSC_MODE)sensor_thread_config.mode);
			pressure_sensor4.set_data_rate((RSC_DATA_RATE)sensor_thread_config.datarate);
			pressure_sensor4.set_mode((RSC_MODE)sensor_thread_config.mode);
			pressure_sensor5.set_data_rate((RSC_DATA_RATE)sensor_thread_config.datarate);
			pressure_sensor5.set_mode((RSC_MODE)sensor_thread_config.mode);
			sensor_thread_delay1 = sensor_thread_config.delay;
			sensor_thread_delay2 = sensor_thread_config.delay;

			thread1.start(sensor_thread);
			thread2.start(sensor_thread);

		break;


	}

	udp_connect(pcb, addr, 8);

	udp_send(pcb, p);

	//udp_send(upcb, p);
	/* free the UDP connection, so we can accept new clients */
	udp_disconnect(pcb);

	/* Free the p buffer */
	pbuf_free(p);
}

/*
* main function, Initialisierung der Sensoren, Threads und Ethernet interface
*/

int main()
{

	eth.set_network(IP_ADDRESS,NETMASK_ADDRESS,GATEWAY_ADDRESS);
	eth.connect();


	struct udp_pcb * upcb = udp_new();
	Net_com 	net_com(IP_ADDRESS_SOCKET, ECHO_SERVER_PORT, upcb, NULL);
	net_com.net_com_bind();

	// upcb = udp_new();
	// Net_com 	diag_com(IP_ADDRESS_SOCKET, DIAG_PORT, upcb, net_receive_callback);
	// diag_com.net_com_bind();


	// Einstellung unterschiedlicher Modi entsprechend Datenblatt
	pressure_sensor1.init(N_DR_90_SPS,NORMAL_MODE);
	pressure_sensor2.init(N_DR_90_SPS,NORMAL_MODE);
	pressure_sensor3.init(N_DR_90_SPS,NORMAL_MODE);
	pressure_sensor4.init(N_DR_90_SPS,NORMAL_MODE);
	pressure_sensor5.init(N_DR_90_SPS,NORMAL_MODE);
	pressure_sensor6.init();
	pressure_sensor7.init();

	//Set default values
	sensor_thread_delay1 = 20;
	sensor_thread_delay2 = 20;

	//set start event
	event_flags.set(FLAG_CONVERSATION_ETHERNET);

    thread1.start(sensor_thread);				// Sensor thread aktivieren -> liest Daten aus Sensoren aus
    thread2.start(callback(ethernet_thread, &net_com));	// Ethernet thread aktivieren -> Aktiviert Datenbübertragung via Ethernet
//    thread3.start(imu_task);
//    thread3.start(diag_thread);

    while(1);

}
