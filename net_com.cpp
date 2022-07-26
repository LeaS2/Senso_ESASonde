/*
 * net_com.cpp
 *
 *  Created on: 21.04.2020
 *      Author: markus
 */

#include "net_com.h"

Net_com::Net_com(char* host_address, uint32_t port, struct udp_pcb* upcb, udp_recv_fn net_receive_callback)
{
    m_upcb = upcb;
    inet_aton(host_address, &m_host_ip_address);
    m_port = port;
    m_callback = net_receive_callback;
}

/*
 *
 */
uint8_t Net_com::net_com_bind(void)
{
    uint8_t ret = 0;
    err_t err;
    if (m_upcb)
    {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
        err = udp_bind(m_upcb, IP_ADDR_ANY, m_port);
        //err = udp_bind(m_upcb_diag, IP_ADDR_ANY, 8);
        if(err == ERR_OK)
        {
            /* Set a receive callback for the upcb */
            udp_recv(m_upcb, m_callback, NULL);
        }
        else
        {
            udp_remove(m_upcb);
        }
    }
    return ret;
}

void Net_com::net_com_set_Callback(mbed::Callback<void()> func)
{

}
/*
 *
 */
void Net_com::net_com_close(void)
{
    udp_disconnect(m_upcb);
}

ssize_t Net_com::net_com_sendto(void* data, size_t length)
{
    static err_t ret;

    // Connect to the remote client
    if(udp_connect(m_upcb, &m_host_ip_address, m_port) == ERR_OK)
    {
        m_pbuffer = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_RAM);
        pbuf_take(m_pbuffer, data, length);

        // Tell the client that we have accepted it */
        ret = udp_send(m_upcb, m_pbuffer);

        /* Free the p buffer */
        pbuf_free(m_pbuffer);
    }
    return ret;
}

ssize_t Net_com::net_com_receive(void* data, size_t length)
{
    ssize_t ret = 0;

    return ret;
}
