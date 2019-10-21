/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"

#if LWIP_NETCONN

#include "tcpecho.h"
#include "lwip/netifapi.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"

#include "board.h"

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USE_DHCP
#ifndef USE_DHCP
/* IP address configuration. */
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 0
#define configIP_ADDR3 102

/* Netmask configuration. */
#define configNET_MASK0 255
#define configNET_MASK1 255
#define configNET_MASK2 255
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 192
#define configGW_ADDR1 168
#define configGW_ADDR2 0
#define configGW_ADDR3 100
#else
/* IP address configuration. */
#define configIP_ADDR0 0
#define configIP_ADDR1 0
#define configIP_ADDR2 0
#define configIP_ADDR3 0

/* Netmask configuration. */
#define configNET_MASK0 0
#define configNET_MASK1 0
#define configNET_MASK2 0
#define configNET_MASK3 0

/* Gateway address configuration. */
#define configGW_ADDR0 0
#define configGW_ADDR1 0
#define configGW_ADDR2 0
#define configGW_ADDR3 0
#endif

/* MAC address configuration. */
#define configMAC_ADDR                     \
		{                                      \
	0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
		}

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* System clock name. */
#define EXAMPLE_CLOCK_NAME kCLOCK_CoreSysClk

#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void tcpserver_init(struct netif* eth_netif);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
	static struct netif fsl_netif0;
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
	static mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
	ip4_addr_t fsl_netif0_ipaddr, fsl_netif0_netmask, fsl_netif0_gw;
	ethernetif_config_t fsl_enet_config0 = {
			.phyAddress = EXAMPLE_PHY_ADDRESS,
			.clockName  = EXAMPLE_CLOCK_NAME,
			.macAddress = configMAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
			.non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
	};

	SYSMPU_Type *base = SYSMPU;
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	/* Disable SYSMPU. */
	base->CESR &= ~SYSMPU_CESR_VLD_MASK;

	IP4_ADDR(&fsl_netif0_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
	IP4_ADDR(&fsl_netif0_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
	IP4_ADDR(&fsl_netif0_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

	tcpip_init(NULL, NULL);

	netifapi_netif_add(&fsl_netif0, &fsl_netif0_ipaddr, &fsl_netif0_netmask, &fsl_netif0_gw, &fsl_enet_config0,
			ethernetif0_init, tcpip_input);
	netifapi_netif_set_default(&fsl_netif0);
	netifapi_netif_set_up(&fsl_netif0);

#ifdef USE_DHCP
	netifapi_dhcp_start(&fsl_netif0);
#endif

	PRINTF("\r\n************************************************\r\n");
	PRINTF(" TCP Server example\r\n");
	PRINTF("************************************************\r\n");

	tcpserver_init(&fsl_netif0);

	vTaskStartScheduler();

	/* Will not get here unless a task calls vTaskEndScheduler ()*/
	return 0;
}

/*-----------------------------------------------------------------------------------*/
static void tcpserver_thread(void *arg)
{
	struct netif *netif = (struct netif *)arg;
	struct netconn *conn, *newconn;
	err_t err;
	struct dhcp *dhcp;
	char response[] = "GotIt!";
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
	/* Wait for the board to have a valid IP address */
	do {
		dhcp = netif_dhcp_data(netif);
		if (dhcp != NULL) {
			if(dhcp->state == DHCP_STATE_BOUND) {
				PRINTF("\r\n IPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
				PRINTF(" IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
				PRINTF(" IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));
				break;
			}
		}
		sys_msleep(20U);
	}while(1);

	/* Create a new connection identifier. */
	/* Bind connection to well known port number 7. */
	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, IP_ADDR_ANY, 7);
	LWIP_ERROR("tcpecho: invalid conn", (conn != NULL), return;);

	/* Tell connection to go into listening mode. */
	netconn_listen(conn);

	while (1) {

		/* Grab new connection. */
		err = netconn_accept(conn, &newconn);
		PRINTF("accepted new connection %p\n\r", newconn);
		/* Process the new connection. */
		if (err == ERR_OK) {
			struct netbuf *buf;
			void *data;
			u16_t len;

			while ((err = netconn_recv(newconn, &buf)) == ERR_OK) {
				PRINTF("Recved\n\r");
				do {
					netbuf_data(buf, &data, &len);
					if(len != 0){
						char received[len+1];
						memset(received, 0, len+1);
						memcpy(received, data, len);
						PRINTF("%s\n\r", received);
						if(!strcmp(received, "ToggleLED")) {
							GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
						}
					}

					err = netconn_write(newconn, (void*)response, strlen(response), NETCONN_COPY);
#if 0
					if (err != ERR_OK) {
						printf("tcpecho: netconn_write: error \"%s\"\n", lwip_strerr(err));
					}
#endif
				} while (netbuf_next(buf) >= 0);
				netbuf_delete(buf);
			}
			PRINTF("Got EOF, looping\n\r");
			/* Close connection and discard connection identifier. */
			netconn_close(newconn);
			netconn_delete(newconn);
		}
	}
}
/*-----------------------------------------------------------------------------------*/
void tcpserver_init(struct netif* eth_netif)
{
	sys_thread_new("tcpserver_thread", tcpserver_thread, eth_netif, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/






#endif
