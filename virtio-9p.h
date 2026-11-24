#ifndef VIRTIO_9P_H
#define VIRTIO_9P_H

/* virtio-9p device translator for GNU/Hurd.

   Boundary conventions: the legacy PCI interface (BAR0 in I/O space), one
   split virtqueue, one outstanding PDU, and a byte-stream front end for the
   unmodified 9pfs translator.  The transport is the generic legacy virtio
   PCI core (virtio-pci.c, virtio-queue.c); this header keeps the 9p-specific
   policy (device ID, mount tag, msize caps) and the adapter entry points.  */

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>
#include <pciaccess.h>
#include <hurd.h>

#include "virtio-pci.h"
#include "virtio-queue.h"

/* VIRTIO_9P_MOUNT_TAG is feature bit 0 and is required: without it the
   mount tag is not delivered.  */
#define V9P_F_MOUNT_TAG 0

#define V9P_DEVICE_VIRTIO_9P 0x1009

/* QEMU's virtio-9p transport caps the negotiated msize at
   (VIRTQUEUE_MAX_SIZE - 2) * 4096, and 9pfs issues PDUs close to that, so
   both DMA directions need full-size buffers.  vm_allocate_contiguous rounds
   to power-of-two blocks, so these must not be silently undersized.  */
#define V9P_TX_BUF_SIZE (4u << 20)
#define V9P_RX_BUF_SIZE (4u << 20)

/* QEMU's virtio-9p transport msize cap, asserted by the selftest.  */
#define V9P_MSIZE_CAP ((1024u - 2) * 4096u)

#define V9P_MAX_TAG_LEN 32
#define V9P_DEFAULT_MOUNT_TAG "hostshare"

/* Per-exchange bound on the interrupt wait; the fallback for a lost
   interrupt, not the normal completion path.  */
#define V9P_EXCHANGE_TIMEOUT 30

/* Passes spent looking for the reply before arming the interrupt, with
   the used notification still suppressed.  One pass is one queue lock
   and one used index read, about 1.5 us in the guest.  Measured on the
   bench guest against QEMU 11.1 under KVM: 1000 passes recovers none of
   the gap, 5000 recovers most, 50000 recovers all of it, so the device
   answers a small 9P request in 10 to 75 us there.  A slower device or a
   loaded host falls through to the armed wait as before.  */
#ifndef V9P_EXCHANGE_SPINS
#define V9P_EXCHANGE_SPINS 50000
#endif

/* Upper bound on the pause passes spent looking for a completion before
   the armed interrupt wait.  Sizing: the device answers a 9P request in
   a few microseconds, a pause pass is some tens of nanoseconds, so this
   covers the completion with a wide margin and costs under ten
   microseconds when the device is slow.  */
#ifndef V9P_EXCHANGE_SPINS
#define V9P_EXCHANGE_SPINS 200
#endif

/* The TX/RX DMA buffers of the single 9P exchange, allocated by
   v9p_queue_setup and used by v9p_exchange.  */
struct v9p_dma {
  uint8_t *tx_buf;
  uint8_t *rx_buf;
  uint64_t tx_phys;
  uint64_t rx_phys;
};

extern struct virtio_pci_dev v9p_pci;
extern struct virtio_queue v9p_q;
/* True when the device and driver negotiated VIRTIO_RING_F_EVENT_IDX.  */
extern int v9p_event_idx;
extern struct virtio_irq v9p_irq;
extern struct v9p_dma v9p_dma;
extern char v9p_tag[V9P_MAX_TAG_LEN + 1];
extern uint16_t v9p_tag_len;
extern uint64_t v9p_vring_phys;

/* virtio-9p.c */
error_t v9p_pci_probe (const char *override);
error_t v9p_pci_init (void);
error_t v9p_queue_setup (void);
error_t v9p_irq_install (void);
void v9p_irq_uninstall (void);

/* virtio-9p-req.c */
error_t v9p_exchange (const void *pdu, uint32_t pdu_len,
                      void *reply, uint32_t reply_cap, uint32_t *reply_len);
error_t v9p_selftest (const char *required_tag);

/* virtio-9p-ios.c */
error_t v9p_ios_init (void);
ssize_t v9p_ios_write (const void *buf, size_t len);
ssize_t v9p_ios_read (void *buf, size_t len);
int v9p_ios_can_read (void);
int v9p_ios_can_write (void);

#endif /* VIRTIO_9P_H */
