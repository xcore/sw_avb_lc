#ifndef __app_config_h__
#define __app_config_h__

/***** APPLICATION CONFIGURATION ******/

/** Global switch to enable or disable AVB Talker functionality in the demo */
#define AVB_DEMO_ENABLE_TALKER 1
/** Global switch to enable or disable AVB Listener functionality in the demo */
#define AVB_DEMO_ENABLE_LISTENER 0

/***** PORTS *********/

// This include fill will get included if you use a XMOS development target
// e.g. the XR-AVB-LC-BRD avb kit or the XMOS sliceKIT and set
// all the port defines accordingly

#ifdef __avb_app_board_config_h_exists__
#include "avb_app_board_config.h"
#endif

// If you are not using a standard dev board (e.g. creating an application for
// your own board), you can update the port mappings here

#if !USING_XMOS_DEV_BOARD

#define ETHERNET_PHY_ADDRESS 0

#define PORT_ETH_RXCLK     on tile[1]: XS1_PORT_1A
#define PORT_ETH_ERR       on tile[1]: XS1_PORT_1B
#define PORT_ETH_RXD       on tile[1]  XS1_PORT_4C
#define PORT_ETH_RXDV      on tile[1]: XS1_PORT_1D
#define PORT_ETH_TXCLK     on tile[1]: XS1_PORT_1C
#define PORT_ETH_TXEN      on tile[1]: XS1_PORT_1E
#define PORT_ETH_TXD       on tile[1]: XS1_PORT_4D

// ETHERNET_PORTS
#define ETHERNET_MII_INIT_full \
   { ETHERNET_CLKBLK_0, ETHERNET_CLKBLK_1, \
     PORT_ETH_RXCLK, PORT_ETH_ERR, PORT_ETH_RXD, PORT_ETH_RXDV, \
     PORT_ETH_TXCLK, PORT_ETH_TXEN, PORT_ETH_TXD }

#if SMI_COMBINE_MDC_MDIO
#define PORT_ETH_MDIOC
#define ETHERNET_SMI_INIT {ETHERNET_PHY_ADDRESS, \
                           PORT_ETH_MDIOC}
#else
#define PORT_ETH_MDIO     on tile[1]:XS1_PORT_1G
#define PORT_ETH_MDC      on tile[1]:XS1_PORT_1G
#define ETHERNET_SMI_INIT {ETHERNET_PHY_ADDRESS, \
                           PORT_ETH_MDIO,                \
                           PORT_ETH_MDC}
#endif

#define USER_PORT_BUTTONS     on tile[1]:XS1_PORT_4A
#define PORT_BUTTONS USER_PORT_BUTTONS

#define USER_PORT_MUTE_LED_REMOTE  on tile[1]:XS1_PORT_4E
#define PORT_MUTE_LED_REMOTE USER_PORT_MUTE_LED_REMOTE

#define USER_PORT_LEDS  on tile[1]:XS1_PORT_4F
#define PORT_LEDS USER_PORT_LEDS

#endif // !USING_XMOS_DEV_BOARD




#endif // __app_config_h__
