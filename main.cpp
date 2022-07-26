#include    "mbed.h"
#include    "MS5611_01BA03.h"
#include    "USBSerial.h"
#include    "EthernetInterface.h"
#include    "INA219.h"
#include    "net_com.h"
#include    "HTU21D.h"
#include    "Honeywell_RSC.h"
#include    "icm20948_dmp.h"
#include    <cstring>
#include    "sensor_config.h"
#include    "diag_sensorboard.h"
#include    <string>
#include    <sstream>

#define IP_ADDRESS         "192.168.000.003"
#define NETMASK_ADDRESS    "192.168.000.001"
#define GATEWAY_ADDRESS    "255.255.255.000"
#define IP_ADDRESS_SOCKET  "192.168.000.005"
#define ECHO_SERVER_PORT       7
#define DIAG_PORT           8

//event flags
#define FLAG_CONVERSATION_SENSORS_ETHERNET        1
#define FLAG_CONVERSATION_SENSORS_SERIAL          2
#define FLAG_CONVERSATION_ETHERNET                4
#define FLAG_CONVERSATION_SERIAL                  8
#define FLAG_CONVERSATION_DIAG                    16
#define STOP_FLAG 1
#define FLAG_DIAG           2

//board config
#define BOARD_ID            0x03
#define USB_SERIAL            0
#define SENSOR_THREAD_DELAY 12

//serial port
#define SERIAL_PORT_BAUDRATE 115200
#define SERIAL_PORT_BITS     8
#define SERIAL_PORT_STOPBIT  1
#define SERIAL_PORT_PARITY   BufferedSerial::None


EthernetInterface eth;
#if (USB_SERIAL == 1)
USBSerial usb_serial;
#endif
rtos::EventFlags event_flags;
rtos::EventFlags diag_event_flags;


uint32_t JumpAddress;
SPI spi1(PB_5,PA_6,PA_5);
SPI spi4(PE_6,PE_5,PE_2);
SPI spi2(PB_15,PB_14,PD_3);
SPI spi3(PG_14,PG_12,PG_13);
SPI spi5(PF_9,PF_8,PF_7);
SPI spi6(PG_14,PG_12,PG_13);
I2C i2c2(PF_0,PF_1);
I2C i2c4(PF_15,PF_14);

static BufferedSerial serial_port(PB_6, PB_7);  // tx, rx


Honeywell_RSC pressure_sensor1(&spi4, PF_2, PG_2);
Honeywell_RSC pressure_sensor2(&spi2, PF_3, PG_3);
Honeywell_RSC pressure_sensor3(&spi2, PF_4, PG_4);
Honeywell_RSC pressure_sensor4(&spi2, PC_0, PG_5);
Honeywell_RSC pressure_sensor5(&spi2, PA_3, PG_6);
MS5611_01BA03 pressure_sensor6(&spi6, PD_4);
MS5611_01BA03 pressure_sensor7(&spi5, PF_10);



struct data_frame        // Struct dynamisch anpassen je nach Platinenversion
{
    uint32_t counter;
    uint8_t id;
    uint32_t timestamp;
    int32_t sensor1;
    int32_t sensor2;
    int32_t sensor3;
    int32_t sensor4;
    int32_t sensor5;
    int32_t sensor6;
    int32_t sensor7;
    int32_t temp1;
    int32_t temp2;
    int32_t temp3;
    int32_t temp4;
    int32_t temp5;
    int32_t temp6;
    int32_t temp7;
};

struct data_frame sensor_data;
struct data_frame ethernet_data;
struct data_frame exchange_data;

Thread      thread1;
Thread      thread2;
Thread      thread3;
Thread      thread4;

uint8_t sensor_thread_delay1;
uint8_t sensor_thread_delay2;

Net_com*     diag_com;
Net_com*     net_com;

struct sensor_config sensor_thread_config;
string  sensor_buffer1;
string  sensor_buffer2;
/*
 *Thread zum Auslesen der Sensordaten
 */
void sensor_thread() {
    int32_t temp = 0;
    sensor_data.id = 0x01;
    string* buffer;
    buffer = &sensor_buffer1;
    static uint32_t packages = 0;

    while (true)
    {
        if(ThisThread::flags_get() == STOP_FLAG)
        {
            ThisThread::flags_clear(STOP_FLAG);
            break;
        }

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

        rtos::ThisThread::sleep_for(sensor_thread_delay1);            // !!! auf 50 für 20 SPS

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

        event_flags.wait_all(FLAG_CONVERSATION_ETHERNET | FLAG_CONVERSATION_SERIAL);
        //increase message counter
        sensor_data.counter = packages++;

        *buffer = "," + to_string(Kernel::get_ms_count()) + "," + \
                  to_string(sensor_data.counter) + "," + \

                  to_string(sensor_data.sensor1) + "," + \
                  to_string(sensor_data.sensor2) + "," + \
                  to_string(sensor_data.sensor3) + "," + \
                  to_string(sensor_data.sensor4) + "," + \
                  to_string(sensor_data.sensor5) + "," + \
                  to_string(sensor_data.sensor6) + "," + \
                  to_string(sensor_data.sensor7) + "," + \

                  to_string(sensor_data.temp1) + "," + \
                  to_string(sensor_data.temp2) + "," + \
                  to_string(sensor_data.temp3) + "," + \
                  to_string(sensor_data.temp4) + "," + \
                  to_string(sensor_data.temp5) + "," + \
                  to_string(sensor_data.temp6) + "," + \
                  to_string(sensor_data.temp7) + "\n\r";
        //set complete message
        *buffer = "$" + to_string(buffer->size()) + *buffer;
        //set events
        event_flags.set(FLAG_CONVERSATION_SENSORS_ETHERNET | FLAG_CONVERSATION_SENSORS_SERIAL);

        rtos::ThisThread::sleep_for(sensor_thread_delay2);
    }
}
/*
* Thread zum Senden der Daten via Ethernet
*/

void ethernet_thread(Net_com* net_com)
{

   // static uint32_t packages = 0;

    while (true)
    {
        if(ThisThread::flags_get() == STOP_FLAG)
        {
            ThisThread::flags_clear(STOP_FLAG);
            break;
        }
        event_flags.wait_any(FLAG_CONVERSATION_SENSORS_ETHERNET);        // Funktion wartet bis alle Sensordaten ausgelesen sind
        net_com->net_com_sendto((void*)sensor_buffer1.c_str(),  sensor_buffer1.size());    // erst dann werden Daten verschickt
        event_flags.set(FLAG_CONVERSATION_ETHERNET);

    }
}

/*
* Thread zum Senden der Daten via serial port
*/
void serial_thread()
{
    //init serial interface
    serial_port.set_baud(SERIAL_PORT_BAUDRATE);
    serial_port.set_format(
            /* bits */ SERIAL_PORT_BITS,
            /* parity */ SERIAL_PORT_PARITY,
            /* stop bit */ SERIAL_PORT_STOPBIT);

    while(true)
    {
        if(ThisThread::flags_get() == STOP_FLAG)
        {
            ThisThread::flags_clear(STOP_FLAG);
            break;
        }
        //wait for the sensor values
        event_flags.wait_any(FLAG_CONVERSATION_SENSORS_SERIAL);        // Funktion wartet bis alle Sensordaten ausgelesen sind
        //send data
        serial_port.write((void*)sensor_buffer1.c_str(), sensor_buffer1.size());
        //sending data finished
        event_flags.set(FLAG_CONVERSATION_SERIAL);
    }
}
/*
 *
 */
void diagnostic_thread()
{
    while(true)
    {
        diag_event_flags.wait_any(FLAG_CONVERSATION_DIAG);

        thread1.flags_set(STOP_FLAG);
        thread2.flags_set(STOP_FLAG);

//        for(uint8_t i = 0; i < sizeof(struct sensor_config); i++)
//            usb_serial.printf("test: %i\n\r",  ((uint8_t *)(&sensor_thread_config))[i]);

        pressure_sensor1.set_data_rate(sensor_thread_config.datarate);
        pressure_sensor1.set_mode(sensor_thread_config.mode);
        pressure_sensor2.set_data_rate(sensor_thread_config.datarate);
        pressure_sensor2.set_mode(sensor_thread_config.mode);
        pressure_sensor3.set_data_rate(sensor_thread_config.datarate);
        pressure_sensor3.set_mode(sensor_thread_config.mode);
        pressure_sensor4.set_data_rate(sensor_thread_config.datarate);
        pressure_sensor4.set_mode(sensor_thread_config.mode);
        pressure_sensor5.set_data_rate(sensor_thread_config.datarate);
        pressure_sensor5.set_mode(sensor_thread_config.mode);
        sensor_thread_delay1 = sensor_thread_config.delay;
        sensor_thread_delay2 = sensor_thread_config.delay;

        //set start event
        event_flags.set(FLAG_CONVERSATION_ETHERNET);

        //threads starten
        thread1.start(sensor_thread);
        thread2.start(callback(ethernet_thread, net_com));

        rtos::ThisThread::sleep_for(10);
    }
}

/*
 *  Callback for diagnostic via Ethernet
 */
void net_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    struct diag_data* recv_diag = (struct diag_data*)(p->payload);


    switch(recv_diag->id)
    {
        //get Session
        case GET_SESSION:
            //usb_serial.printf("get session\n\r");
            recv_diag->id = recv_diag->id + CONTROL_WORD;
            recv_diag->data[0] = 0x02;

        break;

        //start Bootloader
        case START_BOOTLOADER:
            //usb_serial.printf("starting Bootloader\n\r");
            SCB->VTOR = (FLASH_BASE);
            JumpAddress = *(__IO uint32_t*) (FLASH_BASE);
            mbed_start_application(FLASH_BASE);
        break;

        //get board ID
        case GET_BOARD_ID:
            recv_diag->id = recv_diag->id + CONTROL_WORD;
            recv_diag->data[0] = BOARD_ID;
        break;

        //start sending sensor data
        case START_SENDING_SENSOR_DATA:
            recv_diag->id = recv_diag->id + CONTROL_WORD;

            //set start event
              event_flags.set(FLAG_CONVERSATION_ETHERNET);

              //threads starten
            thread1.start(sensor_thread);
            thread2.start(callback(ethernet_thread, net_com));
        break;

        //stop sending sensor data
        case STOP_SENDING_SENSOR_DATA:

            recv_diag->id = recv_diag->id + CONTROL_WORD;
            //threads stoppen
            thread1.terminate();
            thread2.terminate();
        break;

        //get error informations
        case GET_ERROR_INFORMATION:
            recv_diag->id = recv_diag->id + CONTROL_WORD;
        break;

        //set the config parameter for sensor thread
        case SET_SENSOR_CONFIG:
            recv_diag->id = recv_diag->id + CONTROL_WORD;
            std::memcpy(&sensor_thread_config, recv_diag->data, sizeof(struct sensor_config));

            diag_event_flags.set(FLAG_CONVERSATION_DIAG);
        break;
    }
    diag_com->net_com_sendto(recv_diag, sizeof(struct diag_data));
}

/*
* main function, Initialisierung der Sensoren, Threads und Ethernet interface
*/

int main()
{
    eth.set_network(IP_ADDRESS,NETMASK_ADDRESS,GATEWAY_ADDRESS);
    eth.connect();

    struct udp_pcb * upcb = udp_new();
    net_com = new Net_com(IP_ADDRESS_SOCKET, ECHO_SERVER_PORT, upcb, NULL);
    net_com->net_com_bind();

    struct udp_pcb * upcb_diag = udp_new();
    diag_com = new Net_com(IP_ADDRESS_SOCKET, DIAG_PORT, upcb_diag, net_receive_callback);
    diag_com->net_com_bind();


    // Einstellung unterschiedlicher Modi entsprechend Datenblatt
    pressure_sensor1.init(N_DR_90_SPS,NORMAL_MODE);
    pressure_sensor2.init(N_DR_90_SPS,NORMAL_MODE);
    pressure_sensor3.init(N_DR_90_SPS,NORMAL_MODE);
    pressure_sensor4.init(N_DR_90_SPS,NORMAL_MODE);
    pressure_sensor5.init(N_DR_90_SPS,NORMAL_MODE);
    pressure_sensor6.init();
    pressure_sensor7.init();

    //Set default values
    sensor_thread_delay1 = SENSOR_THREAD_DELAY;
    sensor_thread_delay2 = SENSOR_THREAD_DELAY;

    //set start events
    event_flags.set(FLAG_CONVERSATION_ETHERNET | FLAG_CONVERSATION_SERIAL);

    thread1.start(sensor_thread);                // Sensor thread aktivieren -> liest Daten aus Sensoren aus
    thread2.start(callback(ethernet_thread, net_com));    // Ethernet thread aktivieren -> Aktiviert Datenbübertragung via Ethernet
//    thread3.start(diag_thread);
    thread4.start(serial_thread);

    while(1);

}
