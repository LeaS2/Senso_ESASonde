/*
 * net_com.h
 *
 *  Created on: 21.04.2020
 *      Author: markus
 */

#ifndef NET_COM_H_
#define NET_COM_H_

#include 	<stdio.h>
#include 	<stdlib.h>
#include 	<iostream>
#include 	<cstring>
#include 	"mbed.h"
#include 	"EthernetInterface.h"
#include 	"lwip/pbuf.h"
#include 	"lwip/udp.h"
#include 	"lwip/tcp.h"
#include 	"lwip/ip_addr.h"
#include 	"lwip/inet.h"

#define IP_ADDRESS         "192.168.000.003"
#define NETMASK_ADDRESS    "192.168.000.001"
#define GATEWAY_ADDRESS    "255.255.255.000"
#define IP_ADDRESS_SOCKET  "192.168.000.005"
#define ECHO_SERVER_PORT   7

class Net_com
{
	public:

		uint32_t m_port;

		struct udp_pcb *m_upcb;
		struct udp_pcb *m_upcb_diag;
		struct pbuf *m_pbuffer;
		ip4_addr_t m_host_ip_address;
		udp_recv_fn m_callback;

		Net_com(char* host_address, uint32_t port, struct udp_pcb* upcb, udp_recv_fn net_receive_callback);
		uint8_t net_com_bind(void);
		void net_com_close(void);
		ssize_t net_com_sendto(void* data, size_t length);
		ssize_t net_com_receive(void* data, size_t length);
		void net_com_set_Callback(mbed::Callback<void()> func);
};


#endif /* NET_COM_H_ */
