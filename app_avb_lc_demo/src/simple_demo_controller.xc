#include <xccompat.h>
#include <print.h>
#include "debug_print.h"
#include "avb.h"
#include "avb_conf.h"
#include "avb_1722_common.h"
#include "avb_1722_maap.h"
#include "avb_1722_maap_protocol.h"
#include "avb_control_types.h"
#if AVB_ENABLE_1722_1
#include "avb_1722_1_common.h"
#include "avb_1722_1_acmp.h"
#include "avb_1722_1_adp.h"
#include "avb_1722_1_app_hooks.h"
#endif

#if AVB_ENABLE_1722_1
extern avb_1722_1_entity_record entities[AVB_1722_1_MAX_ENTITIES];
static int controller_state = 0;

#define XMOS_VENDOR_ID 0x00229700

void simple_demo_controller(int *change_stream, int *toggle_remote, chanend c_tx)
{
  if (*toggle_remote != controller_state)
  {
    avb_1722_1_controller_disconnect_all_listeners(0, c_tx);

    if (*toggle_remote)
    {
      avb_1722_1_acmp_controller_deinit();
    }
    else
    {
      avb_1722_1_acmp_controller_init();
      avb_1722_1_entity_database_flush();
      avb_1722_1_adp_discover_all();
    }
  }
  controller_state = *toggle_remote;
}


void avb_entity_on_new_entity_available(client interface avb_interface avb, const_guid_ref_t my_guid, avb_1722_1_entity_record *entity, chanend c_tx)
{
  // If Talker is enabled, connect to the first XMOS listener we see
  if (AVB_DEMO_ENABLE_TALKER && AVB_1722_1_CONTROLLER_ENABLED)
  {
    if ((entity->vendor_id == XMOS_VENDOR_ID) &&
       ((entity->listener_capabilities & AVB_1722_1_ADP_LISTENER_CAPABILITIES_AUDIO_SINK) == AVB_1722_1_ADP_LISTENER_CAPABILITIES_AUDIO_SINK) &&
       (entity->listener_stream_sinks >= 1))
    {
      // Ensure that the listener knows our GUID
      avb_1722_1_adp_announce();

      avb_1722_1_controller_connect(my_guid, entity->guid, 0, 0, c_tx);
    }
  }
}

/* The controller has indicated that a listener is connecting to this talker stream */
void avb_talker_on_listener_connect(client interface avb_interface avb, int source_num, const_guid_ref_t listener_guid)
{
  avb_talker_on_listener_connect_default(avb, source_num, listener_guid);
}

void avb_talker_on_listener_connect_failed(client interface avb_interface avb, const_guid_ref_t my_guid, int source_num,
        const_guid_ref_t listener_guid, avb_1722_1_acmp_status_t status, chanend c_tx)
{
  avb_talker_on_listener_connect_failed_default(avb, my_guid, source_num, listener_guid, status, c_tx);
}

/* The controller has indicated to connect this listener sink to a talker stream */
avb_1722_1_acmp_status_t avb_listener_on_talker_connect(client interface avb_interface avb,
                                                        int sink_num,
                                                        const_guid_ref_t talker_guid,
                                                        unsigned char dest_addr[6],
                                                        unsigned int stream_id[2],
                                                        unsigned short vlan_id,
                                                        const_guid_ref_t my_guid)
{
#if AVB_NUM_SINKS > 0
  // Ensure XMOS devices only connect when they are known entities to ensure correct synchronisation
  int do_connect = 0;

  if (AVB_1722_1_CONTROLLER_ENABLED == 0) {
    do_connect = 1;
  }

  const int channels_per_stream = AVB_NUM_MEDIA_OUTPUTS/AVB_NUM_SINKS;
  int map[AVB_NUM_MEDIA_OUTPUTS/AVB_NUM_SINKS];
  for (int i = 0; i < channels_per_stream; i++) map[i] = sink_num ? sink_num*channels_per_stream+i  : sink_num+i;


  avb.set_device_media_clock_type(0, DEVICE_MEDIA_CLOCK_INPUT_STREAM_DERIVED);

  if (AVB_1722_1_CONTROLLER_ENABLED) {
    if ((talker_guid.l >> 40) != (XMOS_VENDOR_ID>>8))
    {
      // Non XMOS talker
      do_connect = 1;
    }
    else
    {
      int i = avb_1722_1_entity_database_find(talker_guid);

      if (i != AVB_1722_1_MAX_ENTITIES)
      {
        do_connect = 1;
        if (AVB_DEMO_ENABLE_TALKER && AVB_DEMO_ENABLE_LISTENER && (talker_guid.l < my_guid.l))
        {
          // Check if the remote Talker is also a Listener
          if (entities[i].listener_stream_sinks >= 1)
          {
            // We can be master clock
            avb.set_device_media_clock_type(0, DEVICE_MEDIA_CLOCK_LOCAL_CLOCK);
            printstrln("Entity elected Master audio clock");
          }
          // else we remain input stream derived
        }
      }
    }
  }

  if (do_connect)
  {
    debug_printf("CONNECTING Listener sink #%d -> Talker stream %x%x, DA: ", sink_num, stream_id[0], stream_id[1]); print_mac_ln(dest_addr);

    unsigned current_stream_id[2];
    avb.get_sink_id(sink_num, current_stream_id);

    if ((current_stream_id[0] != stream_id[0]) || (current_stream_id[1] != stream_id[1])) {
      avb.set_sink_state(sink_num, AVB_SINK_STATE_DISABLED);
    }

    avb.set_sink_sync(sink_num, 0);
    avb.set_sink_channels(sink_num, channels_per_stream);
    avb.set_sink_map(sink_num, map, channels_per_stream);
    avb.set_sink_id(sink_num, stream_id);
    avb.set_sink_addr(sink_num, dest_addr, 6);
    avb.set_sink_vlan(sink_num, vlan_id);

    avb.set_sink_state(sink_num, AVB_SINK_STATE_POTENTIAL);
    return ACMP_STATUS_SUCCESS;
  }

  debug_printf("CONNECTING Listener : entity not found : "); print_guid_ln(talker_guid);
  return ACMP_STATUS_NOT_SUPPORTED;
#else
  __builtin_unreachable();
  return ACMP_STATUS_LISTENER_UNKNOWN_ID;
#endif
}

/* The controller has indicated to disconnect this listener sink from a talker stream */
void avb_listener_on_talker_disconnect(client interface avb_interface avb,
                                       int sink_num,
                                       const_guid_ref_t talker_guid,
                                       unsigned char dest_addr[6],
                                       unsigned int stream_id[2],
                                       const_guid_ref_t my_guid)
{
  avb_listener_on_talker_disconnect_default(avb, sink_num, talker_guid, dest_addr, stream_id, my_guid);
}

/* The controller has indicated that a listener is disconnecting from this talker stream */
void avb_talker_on_listener_disconnect(client interface avb_interface avb,
                                       int source_num,
                                       const_guid_ref_t listener_guid,
                                       int connection_count)
{
  avb_talker_on_listener_disconnect_default(avb, source_num, listener_guid, connection_count);
}

void avb_talker_on_source_address_reserved(client interface avb_interface avb, int source_num, unsigned char mac_addr[6])
{
  avb_talker_on_source_address_reserved_default(avb, source_num, mac_addr);
}
#endif
