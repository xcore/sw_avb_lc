#include <platform.h>
#include <print.h>
#include <xccompat.h>
#include <string.h>
#include <xscope.h>
#include <inttypes.h>
#include "audio_i2s.h"
#include "spi.h"
#include "i2c.h"
#include "avb.h"
#include "audio_clock_CS2100CP.h"
#include "audio_clock_CS2300CP.h"
#include "audio_codec_CS4270.h"
#include "debug_print.h"
#include "media_fifo.h"
#include "ethernet_board_support.h"
#include "avb_1722_1_adp.h"
#include "app_config.h"
#include "avb_ethernet.h"
#include "avb_1722.h"
#include "gptp.h"
#include "media_clock_server.h"
#include "avb_1722_1.h"
#include "avb_srp.h"
#include "aem_descriptor_types.h"

on tile[0]: otp_ports_t otp_ports0 = OTP_PORTS_INITIALIZER;
avb_ethernet_ports_t avb_ethernet_ports =
{
  on ETHERNET_DEFAULT_TILE: OTP_PORTS_INITIALIZER,
    ETHERNET_SMI_INIT,
    ETHERNET_MII_INIT_full,
    ETHERNET_DEFAULT_RESET_INTERFACE_INIT
};


on tile[0]: fl_spi_ports spi_ports = {
  PORT_SPI_MISO,
  PORT_SPI_SS,
  PORT_SPI_CLK,
  PORT_SPI_MOSI,
  XS1_CLKBLK_1
};

on tile[0]: out port p_leds = XS1_PORT_4F;
on tile[0]: out port p_audio_shared = PORT_AUDIO_SHARED;

//***** AVB audio ports ****
on tile[AVB_I2C_TILE]: struct r_i2c r_i2c = { PORT_I2C_SCL, PORT_I2C_SDA };

on tile[0]: out buffered port:32 p_fs[1] = { PORT_SYNC_OUT };
on tile[0]: i2s_ports_t i2s_ports =
{
  XS1_CLKBLK_3,
  XS1_CLKBLK_4,
  PORT_MCLK,
  PORT_SCLK,
  PORT_LRCLK
};

on tile[0]: out buffered port:32 p_aud_dout[AVB_DEMO_NUM_CHANNELS/2] = PORT_SDATA_OUT;

#define p_aud_din null

media_output_fifo_data_t ofifo_data[AVB_NUM_MEDIA_OUTPUTS];
media_output_fifo_t ofifos[AVB_NUM_MEDIA_OUTPUTS];

#define ififos null

[[combinable]] void application_task(client interface avb_interface avb, server interface avb_1722_1_control_callbacks i_1722_1_entity);

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
  chan c_buf_ctl[AVB_NUM_LISTENER_UNITS];

  // Media control
  chan c_media_ctl[AVB_NUM_MEDIA_UNITS];
  interface media_clock_if i_media_clock_ctl;

  interface avb_interface i_avb[NUM_AVB_MANAGER_CHANS];
  interface srp_interface i_srp;
  interface avb_1722_1_control_callbacks i_1722_1_entity;

  par
  {
    on ETHERNET_DEFAULT_TILE: avb_ethernet_server(avb_ethernet_ports,
                                        c_mac_rx, NUM_MAC_RX_CHANS,
                                        c_mac_tx, NUM_MAC_TX_CHANS);

    on tile[0]: media_clock_server(i_media_clock_ctl,
                                   null,
                                   c_buf_ctl,
                                   AVB_NUM_LISTENER_UNITS,
                                   p_fs,
                                   c_mac_rx[MAC_RX_TO_MEDIA_CLOCK],
                                   c_mac_tx[MAC_TX_TO_MEDIA_CLOCK],
                                   c_ptp, NUM_PTP_CHANS,
                                   PTP_GRANDMASTER_CAPABLE);

    // AVB - Audio
    on tile[0]:
    {
      init_media_output_fifos(ofifos, ofifo_data, AVB_NUM_MEDIA_OUTPUTS);

      i2s_master(i2s_ports,
                 p_aud_din, AVB_NUM_MEDIA_INPUTS,
                 p_aud_dout, AVB_NUM_MEDIA_OUTPUTS,
                 MASTER_TO_WORDCLOCK_RATIO,
                 ififos,
                 ofifos,
                 c_media_ctl[0],
                 0);
    }

    // AVB Listener
    on tile[0]: avb_1722_listener(c_mac_rx[MAC_RX_TO_LISTENER],
                                  c_buf_ctl[0],
                                  null,
                                  c_listener_ctl[0],
                                  AVB_NUM_SINKS);

    on tile[1]: [[combine]] par {
      avb_manager(i_avb, NUM_AVB_MANAGER_CHANS,
                  i_srp,
                  c_media_ctl,
                  c_listener_ctl,
                  c_talker_ctl,
                  c_mac_tx[MAC_TX_TO_AVB_MANAGER],
                  i_media_clock_ctl,
                  c_ptp[PTP_TO_AVB_MANAGER]);
      avb_srp_task(i_avb[AVB_MANAGER_TO_SRP],
                   i_srp,
                   c_mac_rx[MAC_RX_TO_SRP],
                   c_mac_tx[MAC_TX_TO_SRP]);
    }

    on tile[0]: application_task(i_avb[AVB_MANAGER_TO_DEMO], i_1722_1_entity);

    on tile[0]: avb_1722_1_maap_task(otp_ports0,
                                    i_avb[AVB_MANAGER_TO_1722_1],
                                    i_1722_1_entity,
                                    null,
                                    c_mac_rx[MAC_RX_TO_1722_1],
                                    c_mac_tx[MAC_TX_TO_1722_1],
                                    c_ptp[PTP_TO_1722_1]);
  }

    return 0;
}


static void audio_hardware_setup(void)
{
#if PLL_TYPE_CS2100
  audio_clock_CS2100CP_init(r_i2c, MASTER_TO_WORDCLOCK_RATIO);
#elif PLL_TYPE_CS2300
  audio_clock_CS2300CP_init(r_i2c, MASTER_TO_WORDCLOCK_RATIO);
#endif
  const int codec1_addr = 0x48;
  const int codec2_addr = 0x49;
  audio_codec_CS4270_init(p_audio_shared, 0xff, codec1_addr, r_i2c);
  audio_codec_CS4270_init(p_audio_shared, 0xff, codec2_addr, r_i2c);
}

static unsigned abs(int x)
{
  int const mask = x >> sizeof(int) * 8 - 1;
  return (x + mask) ^ mask;
}

/** The main application control task **/
[[combinable]]
void application_task(client interface avb_interface avb, server interface avb_1722_1_control_callbacks i_1722_1_entity)
{
  const unsigned default_sample_rate = 48000;
  unsigned lr_channel_assignment[2] = {0, 1};
  int lr_volume[2] = {0, 0};
  unsigned char lr_mute[2] = {0, 0};
  unsigned char mute_reg_val[1] = {0};
  int sink_map[AVB_MAX_CHANNELS_PER_LISTENER_STREAM] = {0, 1, -1, -1, -1, -1, -1, -1};

  audio_hardware_setup();

  // Initialize the media clock
  avb.set_device_media_clock_type(0, DEVICE_MEDIA_CLOCK_INPUT_STREAM_DERIVED);
  avb.set_device_media_clock_rate(0, default_sample_rate);
  avb.set_device_media_clock_state(0, DEVICE_MEDIA_CLOCK_STATE_ENABLED);

  avb.set_sink_format(0, AVB_SOURCE_FORMAT_MBLA_24BIT, default_sample_rate);
  avb.set_sink_sync(0, 0);
  avb.set_sink_channels(0, AVB_MAX_CHANNELS_PER_LISTENER_STREAM);
  avb.set_sink_map(0, sink_map, AVB_MAX_CHANNELS_PER_LISTENER_STREAM);

  while (1)
  {
    select
    {
      case i_1722_1_entity.get_control_value(unsigned short control_index,
                                             unsigned int &value_size,
                                             unsigned short &values_length,
                                             unsigned char values[]) -> unsigned char return_status:
      {
        return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;
        switch (control_index) {
          case DESCRIPTOR_INDEX_CONTROL_GAIN_LEFT:  // 0
          case DESCRIPTOR_INDEX_CONTROL_GAIN_RIGHT: // 1
          {
            value_size = values_length = AEM_CONTROL_SIZE_LINEAR_INT16;
            HTON_U16(values, lr_volume[control_index]);
            return_status = AECP_AEM_STATUS_SUCCESS;
            break;
          }
          case DESCRIPTOR_INDEX_CONTROL_MUTE_LEFT:  // 2
          case DESCRIPTOR_INDEX_CONTROL_MUTE_RIGHT: // 3
          {
            const unsigned channel = control_index-2;
            value_size = values_length = AEM_CONTROL_SIZE_LINEAR_UINT8;
            values[0] = lr_volume[channel];
            return_status = AECP_AEM_STATUS_SUCCESS;
            break;
          }
        }
        break;
      }

      case i_1722_1_entity.set_control_value(unsigned short control_index,
                                            unsigned short values_length,
                                            unsigned char values[]) -> unsigned char return_status:
      {
        return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;

        switch (control_index) {
          case DESCRIPTOR_INDEX_CONTROL_GAIN_LEFT:  // 0
          case DESCRIPTOR_INDEX_CONTROL_GAIN_RIGHT: // 1
          {
            if (values_length == AEM_CONTROL_SIZE_LINEAR_INT16) {
              short volume = NTOH_U16(values);
              if (volume > 0 || volume < -64) {
                return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
                break;
              }
              lr_volume[control_index] = volume;
              unsigned char volume_reg_val[1];
              volume_reg_val[0] = abs(lr_volume[control_index]) << 1;
              debug_printf("Setting chan %d volume to %d db (register val 0x%x)\n", control_index, volume, volume_reg_val[0]);
              i2c_master_write_reg(0x48, CODEC_DACA_VOL_ADDR+control_index, volume_reg_val, 1, r_i2c);
              return_status = AECP_AEM_STATUS_SUCCESS;
            }
            else {
              return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
            }
            break;
          }
          case DESCRIPTOR_INDEX_CONTROL_MUTE_LEFT:  // 2
          case DESCRIPTOR_INDEX_CONTROL_MUTE_RIGHT: // 3
          {
            if (values_length == AEM_CONTROL_SIZE_LINEAR_UINT8) {
              const unsigned channel = control_index-2;
              if (values[0] == 0) {
                mute_reg_val[0] &= ~(1 << channel); // Unmute
                debug_printf("Unmuting channel %d\n", channel);
              } else if (values[0] == 255) {
                debug_printf("Muting channel %d\n", channel);
                mute_reg_val[0] |= (1 << channel); // Mute
              } else {
                return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
                break;
              }
              lr_mute[channel] = values[0];
              i2c_master_write_reg(0x48, CODEC_MUTE_CTRL_ADDR, mute_reg_val, 1, r_i2c);
              return_status = AECP_AEM_STATUS_SUCCESS;
            }
            else {
              return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
            }
            break;
          }
        }
        break;
      }
      case i_1722_1_entity.get_signal_selector(unsigned short selector_index, 
                                               unsigned short &signal_type,
                                               unsigned short &signal_index,
                                               unsigned short &signal_output) -> unsigned char return_status:
      {
        return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;
        switch (selector_index) {
          case DESCRIPTOR_INDEX_SELECTOR_LEFT:
          case DESCRIPTOR_INDEX_SELECTOR_RIGHT:
          {
            signal_type = AEM_AUDIO_CLUSTER_TYPE;
            signal_index = lr_channel_assignment[selector_index];
            signal_output = 0;
            return_status = AECP_AEM_STATUS_SUCCESS;
            break;
          }
        }
        break;
      }
      case i_1722_1_entity.set_signal_selector(unsigned short selector_index, 
                                               unsigned short signal_type,
                                               unsigned short signal_index,
                                               unsigned short signal_output) -> unsigned char return_status:
      {
        return_status = AECP_AEM_STATUS_NO_SUCH_DESCRIPTOR;

        switch (selector_index) {
          case DESCRIPTOR_INDEX_SELECTOR_LEFT:
          case DESCRIPTOR_INDEX_SELECTOR_RIGHT:
          {
            if (signal_type != AEM_AUDIO_CLUSTER_TYPE ||
                signal_index >= AVB_MAX_CHANNELS_PER_LISTENER_STREAM) {
              return_status = AECP_AEM_STATUS_BAD_ARGUMENTS;
              break;
            }
            sink_map[lr_channel_assignment[selector_index]] = -1; // Clear current map value
            sink_map[signal_index] = selector_index; // Set new map value
            lr_channel_assignment[selector_index] = signal_index; // Store value
            avb.set_sink_map(0, sink_map, AVB_MAX_CHANNELS_PER_LISTENER_STREAM);

            return_status = AECP_AEM_STATUS_SUCCESS;
            break;
          }
        }
        break;
      }
    }
  }
}
