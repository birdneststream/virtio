#ifndef VIRTIO_BLK_H
#define VIRTIO_BLK_H

/* virtio-blk driver for GNU/Hurd.

   The protocol is the legacy request queue: one virtqueue (requestq), each
   request a descriptor chain of [outhdr (device-readable)] + [data
   (direction per request type)] + [status (device-writable, 1 byte)], with
   the header's sector field in 512-byte units and the final status byte
   written by the device.  The transport is the generic legacy virtio PCI
   core (virtio-pci.c, virtio-queue.c) shared with virtio-9p and
   virtio-balloon; this header keeps the block-specific policy: device ID,
   feature set, request types, config offsets, and the request pool.

   The Hurd-facing layer (virtio-blk-hurd.c) maps Mach device semantics
   (recnum in records of record_size, DEV_GET_SIZE/DEV_GET_RECORDS, the D_*
   errors) onto those 512-byte-sector virtio operations; this module
   exposes the RPC subset's semantics as functions the selftest drives
   directly.

   Authoritative references (full citations with anchors):
   - QEMU include/standard-headers/linux/virtio_blk.h: feature bits,
     struct virtio_blk_config field order, virtio_blk_outhdr, the T_*
     request types and S_* status values.
   - QEMU hw/block/virtio-blk.c: request dispatch and the used-ring
     completion behavior the validation here must tolerate.
   - gnumach include/device/device.defs and device/device_types.h: RPC
     shapes, dev_mode_t, recnum_t, DEV_GET_SIZE/DEV_GET_RECORDS/
     DEV_FLUSH_CACHE, the D_* error codes.
   - hurd libstore/device.c: the consumer baseline (recnum carries the
     store address in records, mode 0 for I/O, DEV_GET_RECORDS preferred,
     DEV_FLUSH_CACHE through device_set_status).
   - The virtio specification, legacy PCI transport and Block Device
     sections, for anything the above disagree on.  */

#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <mach.h>
#include <device/device_types.h>

#include "virtio-pci.h"
#include "virtio-queue.h"
#include "virtio-pool.h"

/* PCI 0x1af4:0x1001 (PCI_DEVICE_ID_VIRTIO_BLOCK), virtio device ID 2.
   The probe matches on vendor/device ID, not PCI class, which QEMU sets
   to PCI_CLASS_STORAGE_SCSI for this device.  */
#define VBLK_DEVICE_ID 0x1001

/* Feature bits this driver implements and therefore accepts.  Everything
   else offered by the device is rejected: in particular MQ, SCSI,
   CONFIG_WCE, discard/write-zeroes/secure-erase/zoned/topology, and every
   request type beyond IN/OUT/FLUSH.  Each accepted bit has matching
   behavior: RO rejects writes, BLK_SIZE aligns transfers and sets the
   record size, SIZE_MAX/SEG_MAX bound the request, FLUSH is served.  */
#define VBLK_F_SIZE_MAX  1
#define VBLK_F_SEG_MAX   2
#define VBLK_F_RO        5
#define VBLK_F_BLK_SIZE  6
#define VBLK_F_FLUSH     9 /* legacy name; modern is VIRTIO_BLK_F_WCE */

#define VBLK_FEATURES \
  ((1u << VBLK_F_SIZE_MAX) | (1u << VBLK_F_SEG_MAX) | \
   (1u << VBLK_F_RO) | (1u << VBLK_F_BLK_SIZE) | (1u << VBLK_F_FLUSH))

/* struct virtio_blk_config field offsets, little-endian on the wire.
   Only capacity is unconditional; the rest are read only when the
   matching feature bit was accepted.  */
#define VBLK_CFG_CAPACITY 0
#define VBLK_CFG_SIZE_MAX 8
#define VBLK_CFG_SEG_MAX  12
#define VBLK_CFG_BLK_SIZE 20

/* struct virtio_blk_outhdr: type, ioprio, sector (512-byte units).  */
#define VBLK_HDR_SIZE 16

/* Request types the MVP issues; anything else must not be constructed.  */
#define VBLK_T_IN    0
#define VBLK_T_OUT   1
#define VBLK_T_FLUSH 4

/* Device-written final byte of every request chain.  */
#define VBLK_S_OK    0
#define VBLK_S_IOERR 1
#define VBLK_S_UNSUPP 2

/* Virtio block geometry is always 512-byte sectors, independent of any
   negotiated blk_size (which only sets the consumer's record size).  */
#define VBLK_SECTOR_SIZE 512

/* Per-request DMA slot: [outhdr][data][status], so the header and status
   never need to be re-derived from the used ring's length.  One slot is
   one descriptor chain; VBLK_DATA_MAX bounds a single chain's data.  256
   KiB keeps the round trips per byte low while the engine is serialized
   (one chain at a time, so every chunk costs an IRQ round trip), and the
   8-slot region stays ~2 MiB, well inside what vm_allocate_contiguous
   provably hands out (the 9p driver uses two 4 MiB regions).  */
#define VBLK_DATA_MAX (256u << 10)
#define VBLK_SLOT_SIZE (VBLK_HDR_SIZE + VBLK_DATA_MAX + 1)
#define VBLK_MAX_REQ 8

/* Per-request interrupt wait; the fallback for a lost interrupt, not the
   normal completion path.  */
#define VBLK_TIMEOUT 30

/* I/O counters, printed at teardown and by --verbose completions.  */
struct vblk_stats {
  uint64_t reads, writes, flushes;
  uint64_t read_bytes, write_bytes;
  uint64_t errors;
  uint64_t bad_completions;
  uint64_t config_changes;
};

struct vblk_dev {
  struct virtio_pci_dev pci;
  struct virtio_queue reqq;
  struct virtio_irq irq;

  uint32_t offered;
  uint32_t accepted;

  int ro;          /* VIRTIO_BLK_F_RO accepted */
  int flush;       /* VIRTIO_BLK_F_FLUSH accepted */
  uint32_t blk_size;
  uint32_t size_max;
  uint32_t seg_max;

  uint64_t capacity;   /* 512-byte sectors */
  uint64_t size_bytes;
  uint32_t max_transfer; /* effective per-request data cap, bytes */

  /* Request pool: the shared slot-pool engine (one contiguous DMA
     region of VBLK_MAX_REQ slots plus the head-descriptor-to-slot map
     completions are validated against, with the stale-completion
     discard discipline).  The pool has no internal locking, and
     serve-mode RPCs run on a single thread, so req_lock enforces
     the one-chain-in-flight policy across alloc, post and completion
     wait (the net driver's tx_lock discipline).  */
  struct virtio_slot_pool reqs;
  pthread_mutex_t req_lock;

  struct vblk_stats stats;
};

/* The Mach device semantics layer: the MVP subset
   (open/close, read/write with recnum/record_size units, get_status for
   DEV_GET_SIZE/DEV_GET_RECORDS, set_status for DEV_FLUSH_CACHE, and the
   D_* error mapping), without the port publication/MIG demux that the
   serve layer adds.  */
struct vblk_hurd {
  struct vblk_dev *dev;
  int opened;
  int mode;            /* D_READ or D_READ|D_WRITE as opened */
  uint32_t record_size;
  uint64_t records;    /* extents in records */
  uint64_t size_bytes; /* records * record_size */
};

extern int vblk_verbose;

/* virtio-blk.c */
error_t vblk_start_device (const char *override);
void vblk_stop_device (void);
error_t vblk_selftest (int write_test);
error_t vblk_status (const char *override);

/* virtio-blk-serve.c: MIG device server / trivfs translator.
   Brings up no device itself: DEV must be initialized already.  Never
   returns; exits when trivfs_goaway tears the translator down.

   Weak so the static build (linked with libmachdev instead of this
   translator) resolves the symbol to absent; virtio-blk.c checks it
   before falling back from the machdev path.  */
void vblk_serve (struct vblk_dev *dev, const char *name)
  __attribute__ ((weak));

/* virtio-blk-machdev.c: libmachdev bootstrap/translator entry, linked
   only into the static build (see virtio-blk-machdev.c).  Weak in
   every build: virtio-blk.c tests it at run time to pick the machdev
   path over vblk_serve.  Brings DEV up itself (DEVICE is the --device
   override, or NULL) after machdev_trivfs_init has resolved the
   privileged ports: PCI access needs a device master that serves the
   "pci" device, which the grub-supplied kernel port does not.
   NEXT_TASK != MACH_PORT_NULL bootstraps the next task in the chain,
   MACH_PORT_NULL is plain translator mode.  NAME restricts the
   accepted device_open names (NULL: the built-in set).  Never
   returns.  */
void vblk_machdev_run (struct vblk_dev *dev, mach_port_t next_task,
                       const char *name, const char *device,
                       int argc, char **argv)
  __attribute__ ((weak));

/* virtio-blk-dev.c */
error_t vblk_dev_init (struct vblk_dev *, const char *override);
void vblk_dev_quiesce (struct vblk_dev *);
void vblk_dev_reread_config (struct vblk_dev *);
error_t vblk_dev_read (struct vblk_dev *, uint64_t sector, void *data,
                       size_t len, size_t *nread);
error_t vblk_dev_write (struct vblk_dev *, uint64_t sector, const void *data,
                        size_t len, size_t *nwritten);
error_t vblk_dev_flush (struct vblk_dev *);
const struct vblk_stats *vblk_dev_stats (struct vblk_dev *);

/* virtio-blk-req.c */
error_t vblk_req_init (struct vblk_dev *);
void vblk_req_release (struct vblk_dev *);
error_t vblk_req_rw (struct vblk_dev *, int is_write, uint64_t sector,
                     void *data, size_t len, size_t *xferred);
error_t vblk_req_flush (struct vblk_dev *);

/* virtio-blk-hurd.c */
error_t vblk_hurd_init (struct vblk_hurd *, struct vblk_dev *);
error_t vblk_hurd_open (struct vblk_hurd *, int mode);
error_t vblk_hurd_close (struct vblk_hurd *);
error_t vblk_hurd_read (struct vblk_hurd *, int mode, recnum_t recnum,
                        void *data, int bytes_wanted, int *bytes_read);
error_t vblk_hurd_write (struct vblk_hurd *, int mode, recnum_t recnum,
                         const void *data, int bytes, int *bytes_written);
error_t vblk_hurd_get_status (struct vblk_hurd *, dev_flavor_t flavor,
                              int *status, mach_msg_type_number_t *count);
error_t vblk_hurd_set_status (struct vblk_hurd *, dev_flavor_t flavor,
                              const int *status, mach_msg_type_number_t count);
/* Inband and map are not implemented (IO_INBAND_MAX is far too small for
   block data, and no consumer in the MVP path maps the device); they
   return the compatible error rather than crashing.  */
error_t vblk_hurd_read_inband (struct vblk_hurd *, int mode, recnum_t recnum,
                               int bytes_wanted, io_buf_ptr_inband_t data);
error_t vblk_hurd_write_inband (struct vblk_hurd *, int mode,
                                recnum_t recnum, const char *data,
                                int bytes, int *bytes_written);
error_t vblk_hurd_map (struct vblk_hurd *, vm_prot_t prot, vm_offset_t offset,
                       vm_size_t size, memory_object_t *pager, int unmap);

#endif /* VIRTIO_BLK_H */
