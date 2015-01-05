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

void avb_entity_on_new_entity_available(client interface avb_interface avb, const_guid_ref_t my_guid, avb_1722_1_entity_record *entity, chanend c_tx)
{
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
  debug_printf("CONNECTING Listener sink #%d -> Talker stream %x%x, DA: ", sink_num, stream_id[0], stream_id[1]); print_mac_ln(dest_addr);

  unsigned current_stream_id[2];
  avb.get_sink_id(sink_num, current_stream_id);

  if ((current_stream_id[0] != stream_id[0]) || (current_stream_id[1] != stream_id[1])) {
    avb.set_sink_state(sink_num, AVB_SINK_STATE_DISABLED);
  }

  avb.set_sink_id(sink_num, stream_id);
  avb.set_sink_addr(sink_num, dest_addr, 6);
  avb.set_sink_vlan(sink_num, vlan_id);

  avb.set_sink_state(sink_num, AVB_SINK_STATE_POTENTIAL);
  return ACMP_STATUS_SUCCESS;
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
