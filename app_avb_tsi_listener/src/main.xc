#include <platform.h>
#include <print.h>
#include <xccompat.h>
#include <string.h>
#include <xscope.h>
#include "spi.h"
#include "i2c.h"
#include "avb.h"
#include "debug_print.h"
#include "media_fifo.h"
#include "ethernet_board_support.h"
#include "simple_demo_controller.h"
#include "avb_1722_1_adp.h"
#include "app_config.h"
#include "avb_ethernet.h"
#include "avb_1722.h"
#include "gptp.h"
#include "media_clock_server.h"
#include "avb_1722_1.h"
#include "avb_srp.h"
#include "aem_descriptor_types.h"
#include "tsi_output.h"

on tile[0]: otp_ports_t otp_ports0 = OTP_PORTS_INITIALIZER;
avb_ethernet_ports_t avb_ethernet_ports =
{
  on tile[1]: OTP_PORTS_INITIALIZER,
    ETHERNET_SMI_INIT,
    ETHERNET_MII_INIT_full,
    ETHERNET_DEFAULT_RESET_INTERFACE_INIT
};

on tile[1]: out port p_mute_led_remote = PORT_MUTE_LED_REMOTE;
on tile[1]: out port p_chan_leds = PORT_LEDS;
on tile[1]: in port p_buttons = PORT_BUTTONS;

on tile[AVB_I2C_TILE]: struct r_i2c r_i2c = { PORT_I2C_SCL, PORT_I2C_SDA };

//***** Transport Stream SPI output resources ****
on tile[0]: in port p_ts_clk = XS1_PORT_1I;
on tile[0]: out port p_ts_valid = XS1_PORT_1H;
on tile[0]: out buffered port:4 p_ts_sync = XS1_PORT_1J;
on tile[0]: out buffered port:32 p_ts_data = XS1_PORT_8B;
on tile[0]: clock clk_ts = XS1_CLKBLK_1;

media_output_fifo_data_t ofifo_data[AVB_NUM_MEDIA_OUTPUTS];
media_output_fifo_t ofifos[AVB_NUM_MEDIA_OUTPUTS];

[[combinable]] void application_task(client interface avb_interface avb, server interface avb_1722_1_control_callbacks i_1722_1_callbacks);

enum mac_rx_chans {
  MAC_RX_TO_MEDIA_CLOCK = 0,
  MAC_RX_TO_LISTENER,
  MAC_RX_TO_SRP,
  MAC_RX_TO_1722_1,
  NUM_MAC_RX_CHANS
};

enum mac_tx_chans {
  MAC_TX_TO_MEDIA_CLOCK = 0,
  MAC_TX_TO_SRP,
  MAC_TX_TO_1722_1,
  MAC_TX_TO_AVB_MANAGER,
  NUM_MAC_TX_CHANS
};

enum avb_manager_chans {
  AVB_MANAGER_TO_SRP = 0,
  AVB_MANAGER_TO_1722_1,
  AVB_MANAGER_TO_DEMO,
  NUM_AVB_MANAGER_CHANS
};

enum ptp_chans {
  PTP_TO_AVB_MANAGER = 0,
  PTP_TO_LISTENER,
  PTP_TO_1722_1,
  NUM_PTP_CHANS
};

int main(void)
{
  // Ethernet channels
  chan c_mac_tx[NUM_MAC_TX_CHANS];
  chan c_mac_rx[NUM_MAC_RX_CHANS];

  // PTP channels
  chan c_ptp[NUM_PTP_CHANS];

  // AVB unit control
  #define c_talker_ctl null
  chan c_listener_ctl[AVB_NUM_LISTENER_UNITS];

  // Media control
  chan c_media_ctl[AVB_NUM_MEDIA_UNITS];
  interface avb_interface i_avb[NUM_AVB_MANAGER_CHANS];
  interface srp_interface i_srp;
  interface avb_1722_1_control_callbacks i_1722_1_callbacks;

  par
  {
    on tile[1]: avb_ethernet_server(avb_ethernet_ports,
                                        c_mac_rx, NUM_MAC_RX_CHANS,
                                        c_mac_tx, NUM_MAC_TX_CHANS);

    on tile[0]: ptp_server(c_mac_rx[MAC_RX_TO_MEDIA_CLOCK],
                           c_mac_tx[MAC_TX_TO_MEDIA_CLOCK],
                           c_ptp, NUM_PTP_CHANS,
                           PTP_GRANDMASTER_CAPABLE);

    on tile[0]:
    {
      init_media_output_fifos(ofifos, ofifo_data, AVB_NUM_MEDIA_OUTPUTS);
      media_ctl_register(c_media_ctl[0], 0, null, 1, ofifos, 0);
      tsi_output(clk_ts, p_ts_data, p_ts_clk, p_ts_sync, p_ts_valid, ofifo_data[0]);
    }

    on tile[0]: avb_1722_listener(c_mac_rx[MAC_RX_TO_LISTENER],
                                  null,
                                  c_ptp[PTP_TO_LISTENER],
                                  c_listener_ctl[0],
                                  AVB_NUM_SINKS);

    on tile[1]: [[combine]] par {
      avb_manager(i_avb, NUM_AVB_MANAGER_CHANS,
                  i_srp,
                  c_media_ctl,
                  c_listener_ctl,
                  c_talker_ctl,
                  c_mac_tx[MAC_TX_TO_AVB_MANAGER],
                  null,
                  c_ptp[PTP_TO_AVB_MANAGER]);
      avb_srp_task(i_avb[AVB_MANAGER_TO_SRP],
                   i_srp,
                   c_mac_rx[MAC_RX_TO_SRP],
                   c_mac_tx[MAC_TX_TO_SRP]);
    }

    on tile[1]: application_task(i_avb[AVB_MANAGER_TO_DEMO], i_1722_1_callbacks);

    on tile[0]: avb_1722_1_maap_task(otp_ports0,
                                    i_avb[AVB_MANAGER_TO_1722_1],
                                    i_1722_1_callbacks,
                                    null,
                                    c_mac_rx[MAC_RX_TO_1722_1],
                                    c_mac_tx[MAC_TX_TO_1722_1],
                                    c_ptp[PTP_TO_1722_1]);
  }

  return 0;
}

/** The main application control task **/
[[combinable]]
void application_task(client interface avb_interface avb, server interface avb_1722_1_control_callbacks i_1722_1_callbacks)
{
  unsigned char aem_identify_control_value = 0;
  int map[1];
  p_mute_led_remote <: ~0;

  map[0] = 0;
  avb.set_source_map(0, map, 1);
  avb.set_source_channels(0, 1);

  while (1)
  {
    select
    {
      case i_1722_1_callbacks.get_control_value(unsigned short control_index,
                                            unsigned short &values_length,
                                            unsigned char values[AEM_MAX_CONTROL_VALUES_LENGTH_BYTES]) -> unsigned char return_status:
      {
        return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;

        switch (control_index)
        {
          case DESCRIPTOR_INDEX_CONTROL_IDENTIFY:
              values[0] = aem_identify_control_value;
              values_length = 1;
              return_status = AECP_AEM_STATUS_SUCCESS;
            break;
        }

        break;
      }

      case i_1722_1_callbacks.set_control_value(unsigned short control_index,
                                            unsigned short values_length,
                                            unsigned char values[AEM_MAX_CONTROL_VALUES_LENGTH_BYTES]) -> unsigned char return_status:
      {
        return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;

        switch (control_index) {
          case DESCRIPTOR_INDEX_CONTROL_IDENTIFY: {
            if (values_length == 1) {
              aem_identify_control_value = values[0];
              p_mute_led_remote <: (~0) & ~((int)aem_identify_control_value<<1);
              if (aem_identify_control_value) {
                debug_printf("IDENTIFY Ping\n");
              }
              return_status = AECP_AEM_STATUS_SUCCESS;
            }
            else
            {
              return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
            }
            break;
          }
        }


        break;
      }
    }
  }
}
