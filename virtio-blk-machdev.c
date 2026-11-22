/* virtio-blk-machdev.c - libmachdev adapter for the virtio-blk driver.

   The static build serves block I/O through libmachdev's Mach device
   emulation (the rumpdisk/devnode bootstrap pattern) instead of the
   dynamic build's virtio-serve/libports translator scaffold: the two
   define the same trivfs hook variables, trivfs_goaway and ds_device_*
   handlers, so they cannot be linked into one binary.  This file is
   therefore only linked into virtio-blk.static.

   All block semantics stay in virtio-blk-hurd.c (record size/extent
   math, DEV_GET_SIZE/DEV_GET_RECORDS, open mode checks, bounds and
   unit errors, RO/flush policy); this file only adapts call shapes.

   Bootstrap contract (machdev_trivfs_init resumes the next task before
   it returns): the emulation ops are registered before
   machdev_trivfs_init is called, and the device is brought up after
   it returns.  The order matters for PCI: machdev_trivfs_init resolves
   the privileged ports from the bootstrap chain (fsys_getpriv), and
   libpciaccess needs a device master whose open of "pci" reaches the
   pci-arbiter; the kernel device master grub gives the first servers
   cannot serve "pci" at all.  This mirrors rumpdisk, whose hardware
   init also runs after machdev_trivfs_init.  ext2fs's first
   device_open lands on the trivfs control port and queues there until
   machdev_trivfs_server_loop starts receiving, and the device is up
   and machdev_device_init has created the device bucket by then, so
   no message is lost in the window.
   */

#include <errno.h>
#include <error.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <mach.h>
#include <mach/mach_host.h>   /* thread_wire */
#include <device/device.h>
#include <hurd.h>
#include <hurd/ports.h>

#include <hurd/machdev.h>

#include "virtio-blk.h"

/* libshouldbeinlibc's wire.h is not installed in the cross prefix;
   this matches its `error_t wire_task_self (void)'.  */
extern error_t wire_task_self (void);

/* Kernel-console logging.  A bootstrap task has no stdio until
   resume_bootstrap_server reopens it inside machdev_trivfs_init, so
   earlier milestones would be lost with printf; mach_print reaches
   the kernel console regardless of stdio.  */
static void
vblk_log (const char *fmt, ...)
{
  char buf[256];
  va_list ap;

  va_start (ap, fmt);
  vsnprintf (buf, sizeof buf, fmt, ap);
  va_end (ap);
  mach_print (buf);
}

/* CRC32 (IEEE 802.3: reflected, polynomial 0xedb88320, the same variant
   zlib and Python's zlib.crc32 compute) over the bytes of one transfer.
   The log line carries it so the data the guest received can be compared
   against the same region of the disk image on the host, byte for byte,
   without dumping megabytes of console output.  */
static uint32_t
vblk_crc32 (const void *data, size_t len)
{
  const unsigned char *bytes = data;
  uint32_t crc = 0xffffffffu;

  for (size_t i = 0; i < len; i++)
    {
      crc ^= bytes[i];
      for (int bit = 0; bit < 8; bit++)
        crc = (crc >> 1) ^ (0xedb88320u & (uint32_t) - (int32_t) (crc & 1));
    }
  return ~crc;
}

/* Name and devnode passed to machdev_trivfs_init: the translator
   identity (what resume_bootstrap_server prints and where the
   bootstrapping translator installs itself on /dev), distinct from the
   device_open names the emulation layer answers to.  */
#define VBLK_MACHDEV_NAME "virtio-blk"
#define VBLK_MACHDEV_PATH "/dev/" VBLK_MACHDEV_NAME

/* Bound for the device_open names we accept ("wd0", "vd0", or the
   --name restriction); longer names are refused outright.  */
#define VBLK_PORT_NAME_MAX 16

/* One of these is created per device_open.  The port_info must stay
   first: libmachdev's MIG mutations resolve device_t arguments with
   ports_lookup_port over machdev_device_bucket/machdev_device_class,
   which index the struct by its leading port_info.  */
struct vblk_machdev_port
{
  struct port_info pi;
  struct machdev_emul_device dev;  /* emul_ops/emul_data for libmachdev */
  struct vblk_hurd hurd;           /* shared semantics layer */
  dev_mode_t mode;
  char name[VBLK_PORT_NAME_MAX];
};

/* The one device, brought up by virtio-blk.c's main before this file
   runs anything.  */
static struct vblk_dev *machdev_dev;

/* --name restriction on device_open names; NULL accepts the built-in
   name set.  */
static const char *serving_name;

static struct machdev_device_emulation_ops vblk_emul_ops;

/* The device_open names this server accepts: the grub root device
   name ("wd0"), the virtio convention ("vd0"), and the empty name
   storeio passes for an unnamed device (the translator this replaces
   accepted any name, and libstore's `@node:' form ends up empty).
   Anything else answers D_NO_SUCH_DEVICE, after which libmachdev's
   ds_device_open falls back to the kernel device master (devnode
   behavior).  */
static boolean_t
vblk_emul_name_ok (const char *name)
{
  if (serving_name)
    return strcmp (name, serving_name) == 0;
  return name[0] == '\0' || strcmp (name, "wd0") == 0
         || strcmp (name, "vd0") == 0;
}

/* Failure here leaves the port half-built; tear it down the way
   virtio-serve.c does (destroy the receive right, then drop the
   reference ports_create_port returned).  */
static void
vblk_emul_open_fail (struct vblk_machdev_port *p)
{
  ports_destroy_right (p);
  ports_port_deref (p);
}

static io_return_t
vblk_emul_open (mach_port_t reply_port, mach_msg_type_name_t reply_type,
                dev_mode_t mode, const char *name, device_t *devp,
                mach_msg_type_name_t *devicePoly)
{
  struct vblk_machdev_port *p;
  error_t err;

  (void) reply_port;
  (void) reply_type;

  if (!vblk_emul_name_ok (name))
    {
      if (vblk_verbose)
        vblk_log ("virtio-blk: open \"%s\": unknown device\n", name);
      return D_NO_SUCH_DEVICE;
    }
  if (strlen (name) >= VBLK_PORT_NAME_MAX)
    return D_NO_SUCH_DEVICE;

  err = machdev_create_device_port (sizeof *p, &p);
  if (err)
    {
      vblk_log ("virtio-blk: open \"%s\": no device port: %d\n",
                name, err);
      return D_NO_MEMORY;
    }

  err = vblk_hurd_init (&p->hurd, machdev_dev);
  if (err)
    {
      vblk_log ("virtio-blk: open \"%s\": init failed: %d\n", name, err);
      vblk_emul_open_fail (p);
      return D_NO_MEMORY;
    }
  err = vblk_hurd_open (&p->hurd, mode);
  if (err)
    {
      vblk_log ("virtio-blk: open \"%s\" failed: %d\n", name, err);
      vblk_emul_open_fail (p);
      return err;
    }

  p->mode = mode;
  strcpy (p->name, name);
  p->dev.emul_data = p;
  p->dev.emul_ops = &vblk_emul_ops;

  /* MAKE_SEND (rumpdisk's model): the client makes its own send right
     to the port; the reference ports_create_port returned stays ours
     until device_close tears the port down.  */
  *devp = ports_get_right (p);
  *devicePoly = MACH_MSG_TYPE_MAKE_SEND;
  vblk_log ("virtio-blk: opened \"%s\" (mode 0x%x)\n", name,
            (unsigned) mode);
  return D_SUCCESS;
}

static io_return_t
vblk_emul_close (void *d)
{
  struct vblk_machdev_port *p = d;

  vblk_log ("virtio-blk: close \"%s\"\n", p->name);
  vblk_hurd_close (&p->hurd);
  ports_port_deref (p);      /* the creator reference from open */
  ports_destroy_right (p);
  return D_SUCCESS;
}

/* Return a send right associated with the device port.  */
static mach_port_t
vblk_emul_dev_to_port (void *d)
{
  return (d ? ports_get_send_right (d) : MACH_PORT_NULL);
}

static io_return_t
vblk_emul_read (void *d, mach_port_t reply_port,
                mach_msg_type_name_t reply_type, dev_mode_t mode,
                recnum_t recnum, int count, io_buf_ptr_t *data,
                unsigned *bytes_read)
{
  struct vblk_machdev_port *p = d;
  vm_address_t buf = 0;
  error_t err;
  int nread = 0;

  (void) reply_port;
  (void) reply_type;

  if (count <= 0)
    return D_INVALID_SIZE;

  /* The reply carries the data out of line and deallocates it after
     the send (device.defs: `out data : io_buf_ptr_t, dealloc'), so it
     must live in its own vm region; read straight into it to keep a
     single copy on the path.  */
  err = vm_allocate (mach_task_self (), &buf, count, TRUE);
  if (err)
    return D_NO_MEMORY;

  err = vblk_hurd_read (&p->hurd, mode, recnum, (void *) buf, count,
                        &nread);
  if (err)
    {
      vblk_log ("virtio-blk: read at %llu (%d bytes): error %d\n",
                (unsigned long long) recnum, count, err);
      vm_deallocate (mach_task_self (), buf, count);
      return err;
    }

  if (vblk_verbose)
    vblk_log ("virtio-blk: read at %llu (%d bytes) -> %d crc32=%08x\n",
              (unsigned long long) recnum, count, nread,
              vblk_crc32 ((void *) buf, (size_t) (nread > 0 ? nread : 0)));

  *data = (io_buf_ptr_t) buf;
  *bytes_read = nread;
  return D_SUCCESS;
}

static io_return_t
vblk_emul_write (void *d, mach_port_t reply_port,
                 mach_msg_type_name_t reply_type, dev_mode_t mode,
                 recnum_t recnum, io_buf_ptr_t data, unsigned count,
                 int *bytes_written)
{
  struct vblk_machdev_port *p = d;
  error_t err;

  (void) reply_port;
  (void) reply_type;

  if (data == 0)
    return D_INVALID_SIZE;

  err = vblk_hurd_write (&p->hurd, mode, recnum, data, count,
                         bytes_written);
  /* The write data arrived out of line (the MIG stub moved the pages
     into this task).  After a successful return it is ours to free.
     After an error the server loop destroys the request and the data
     with it (glibc's mach_msg_server), so freeing it here as well
     would unmap whatever another thread mapped at that address in
     between (virtio-blk-serve.c documents the same discipline).  */
  if (err)
    vblk_log ("virtio-blk: write at %llu (%u bytes): error %d\n",
              (unsigned long long) recnum, count, err);
  else if (vblk_verbose)
    vblk_log ("virtio-blk: write at %llu (%u bytes) -> %d crc32=%08x\n",
              (unsigned long long) recnum, count, *bytes_written,
              vblk_crc32 (data, (size_t) (*bytes_written > 0
                                           ? *bytes_written : 0)));
  if (!err)
    vm_deallocate (mach_task_self (), (vm_address_t) data, count);
  return err;
}

static io_return_t
vblk_emul_get_status (void *d, dev_flavor_t flavor, dev_status_t status,
                      mach_msg_type_number_t *count)
{
  struct vblk_machdev_port *p = d;
  io_return_t err;

  err = vblk_hurd_get_status (&p->hurd, flavor, status, count);
  if (vblk_verbose)
    vblk_log ("virtio-blk: get_status flavor %d: %d\n",
              (int) flavor, err);
  return err;
}

static io_return_t
vblk_emul_set_status (void *d, dev_flavor_t flavor, dev_status_t status,
                      mach_msg_type_number_t count)
{
  struct vblk_machdev_port *p = d;
  io_return_t err;

  err = vblk_hurd_set_status (&p->hurd, flavor, status, count);
  if (vblk_verbose)
    vblk_log ("virtio-blk: set_status flavor %d: %d\n",
              (int) flavor, err);
  return err;
}

static io_return_t
vblk_emul_write_inband (void *d, mach_port_t reply_port,
                        mach_msg_type_name_t reply_type, dev_mode_t mode,
                        recnum_t recnum, const char *data, unsigned count,
                        int *bytes_written)
{
  struct vblk_machdev_port *p = d;

  (void) reply_port;
  (void) reply_type;
  *bytes_written = 0;
  return vblk_hurd_write_inband (&p->hurd, mode, recnum, data, count,
                                 bytes_written);
}

static io_return_t
vblk_emul_read_inband (void *d, mach_port_t reply_port,
                       mach_msg_type_name_t reply_type, dev_mode_t mode,
                       recnum_t recnum, int bytes_wanted, char *data,
                       unsigned *bytes_read)
{
  struct vblk_machdev_port *p = d;

  (void) reply_port;
  (void) reply_type;
  *bytes_read = 0;
  return vblk_hurd_read_inband (&p->hurd, mode, recnum, bytes_wanted,
                                data);
}

static io_return_t
vblk_emul_map (void *d, vm_prot_t prot, vm_offset_t offset,
               vm_size_t size, mach_port_t *pager, boolean_t unmap)
{
  struct vblk_machdev_port *p = d;

  return vblk_hurd_map (&p->hurd, prot, offset, size, pager, unmap);
}

/* S_startup_dosync (libmachdev) calls this at system shutdown: flush
   the device's volatile cache when the flush feature was negotiated;
   otherwise the engine holds nothing back and this is a no-op.  */
static void
vblk_emul_sync (void)
{
  if (machdev_dev)
    vblk_dev_flush (machdev_dev);
}

/* Field order follows struct machdev_device_emulation_ops.  */
static struct machdev_device_emulation_ops vblk_emul_ops =
{
  NULL,                    /* init: the device is up before registration */
  NULL,                    /* reference: not called by libmachdev */
  NULL,                    /* dealloc: not called by libmachdev */
  vblk_emul_dev_to_port,
  vblk_emul_open,
  vblk_emul_close,
  vblk_emul_write,
  vblk_emul_write_inband,
  vblk_emul_read,
  vblk_emul_read_inband,
  vblk_emul_set_status,
  vblk_emul_get_status,
  NULL,                    /* set_filter: block devices take no filters */
  vblk_emul_map,
  NULL,                    /* no_senders: libports' default teardown */
  NULL,                    /* write_trap */
  NULL,                    /* writev_trap */
  vblk_emul_sync
};

/* Like rumpdisk: every device-server thread needs vm privilege for the
   OOL RPC buffers, and a thread cannot wire itself before its first
   message arrives, so wire it in the demuxer.  In translator mode
   (_hurd_host_priv unset) there is nothing to wire against; the task's
   memory is swappable like any translator's.  */
static int __thread vblk_wired;

static int
vblk_demuxer (mach_msg_header_t *inp, mach_msg_header_t *outp)
{
  if (!vblk_wired && _hurd_host_priv != MACH_PORT_NULL)
    {
      mach_port_t self = mach_thread_self ();

      thread_wire (_hurd_host_priv, self, TRUE);
      mach_port_deallocate (mach_task_self (), self);
      vblk_wired = 1;
    }
  return machdev_demuxer (inp, outp);
}

/* Device ports (machdev_device_bucket) are served by this thread pool;
   the trivfs control/protid ports (port_bucket) by
   machdev_trivfs_server_loop in the main thread.  The multithread
   server keeps a blocked read from stalling device_open for other
   clients; request concurrency is bounded by the engine's serialized
   one-chain-at-a-time discipline under req_lock.  */
static void *
vblk_device_server (void *arg)
{
  (void) arg;
  pthread_setname_np (pthread_self (), "vblk-device");

  do
    {
      ports_manage_port_operations_multithread (machdev_device_bucket,
                                                vblk_demuxer,
                                                1000 * 60 * 2,
                                                1000 * 60 * 10,
                                                0);
    } while (1);

  return NULL;
}

/* Serve DEV through libmachdev; never returns.  NEXT_TASK non-NULL is
   bootstrap mode (the task is resumed inside machdev_trivfs_init);
   MACH_PORT_NULL is plain translator mode (--serve).  NAME restricts
   the accepted device_open names (NULL: built-in set); DEVICE is the
   --device override for the bring-up this function performs.  */
void
vblk_machdev_run (struct vblk_dev *dev, mach_port_t next_task,
                  const char *name, const char *device,
                  int argc, char **argv)
{
  mach_port_t bootstrap = MACH_PORT_NULL;
  error_t err;
  pthread_t t;

  machdev_dev = dev;
  serving_name = name;

  /* Register before machdev_trivfs_init: it resumes the next task,
     whose first device_open must already find our emulation ops.  */
  machdev_register (&vblk_emul_ops);

  if (next_task != MACH_PORT_NULL)
    vblk_log ("virtio-blk: ops registered, calling machdev_trivfs_init\n");

  machdev_trivfs_init (argc, argv, next_task, VBLK_MACHDEV_NAME,
                       VBLK_MACHDEV_PATH, &bootstrap);

  if (next_task != MACH_PORT_NULL)
    {
      /* Lock this task in memory: it drives the disk the system (and
         the pager) boots from, so it must never be swapped out.  */
      err = wire_task_self ();
      if (err)
        error (1, err, "cannot lock all memory");
      err = thread_wire (_hurd_host_priv, mach_thread_self (), TRUE);
      if (err != KERN_SUCCESS)
        error (1, err, "cannot get vm_privilege");
      vblk_log ("virtio-blk: trivfs_init done, task wired\n");
    }

  /* Bring the device up here, not in virtio-blk.c's main: only now
     are the privileged ports the driver needs resolved (see the
     bootstrap contract at the top of this file).  In bootstrap mode
     stdio is on the console from here on, so error() output for a
     failed bring-up is visible.  */
  err = vblk_start_device (device);
  if (err)
    error (4, err, "virtio-blk device not usable");
  if (next_task != MACH_PORT_NULL)
    vblk_log ("virtio-blk: device up, %llu bytes\n",
              (unsigned long long) dev->size_bytes);

  machdev_device_init ();
  err = pthread_create (&t, NULL, vblk_device_server, NULL);
  if (err)
    error (1, err, "cannot start the device server thread");
  pthread_detach (t);

  if (next_task != MACH_PORT_NULL)
    vblk_log ("virtio-blk: device server running, entering trivfs loop\n");

  machdev_trivfs_server_startup (bootstrap);
  machdev_trivfs_server_loop (NULL);
  /* Never reached.  */
}
