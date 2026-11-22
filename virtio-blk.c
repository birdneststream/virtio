/* virtio-blk driver: main, argp, lifecycle, selftest.

   A standalone long-lived daemon like virtio-balloon: started with
   --serve it becomes the Mach device translator for the block device;
   without --serve it owns the device, logs config changes and counters,
   and proves the datapath.  --selftest drives raw read/write/flush plus
   the Mach device semantics layer, so the whole pipeline is exercised.
   Every exit path is clean: a guest booted without a virtio-blk device
   must produce a diagnostic, not a hang.  */

#include <argp.h>
#include <error.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <hurd.h>          /* _hurd_host_priv, _hurd_device_master */
#include <mach.h>          /* mach_print: kernel console before stdio */

#include "virtio-blk.h"

const char *argp_program_version = "virtio-blk 0.1";

int vblk_verbose;

/* Keys for the long-only options; 0 would not distinguish them.  */
enum {
  OPT_STATUS = 1000,
  OPT_WRITE_TEST,
  OPT_READ_RANGE,
  OPT_SERVE,
  OPT_NAME,
  OPT_HOST_PRIV,
  OPT_DEV_MASTER,
  OPT_NEXT_TASK
};

static const char *opt_device;
static int opt_selftest;
static int opt_status;
static int opt_write_test;
static const char *opt_read_range;
static int opt_serve;
static const char *opt_name;

/* Bootstrap-chain arguments (rumpdisk's -h/-d/-N shape).  The kernel
   bootstrap writes the port/task names as decimal task-local port
   numbers on the module command line.  */
static int opt_host_priv;
static int opt_dev_master;
static int opt_next_task;

static const struct argp_option options[] =
{
  {"device", 'd', "DOM:BUS:DEV.FN", 0,
   "Use the virtio-blk device at this PCI slot instead of the first match"},
  {"selftest", 's', 0, 0,
   "Bring the device up, run the Phase 1 datapath selftest, and exit"},
  {"write-test", OPT_WRITE_TEST, 0, 0,
   "With --selftest, also write a pattern to sector 0, flush, and read it "
   "back; DESTROYS the first sector of the backing image (disposable "
   "images only)"},
  {"status", OPT_STATUS, 0, 0,
   "Print the device's capacity, features, and limits and exit, without "
   "disturbing a running driver"},
  {"read-range", OPT_READ_RANGE, "SECTOR[:COUNT]", 0,
   "Read COUNT sectors (default 1) at SECTOR and write the raw bytes to "
   "standard output, then exit; a debugging aid for harness data checks"},
  {"serve", OPT_SERVE, 0, 0,
   "Run as a Mach device translator (settrans): answer device_open/read/"
   "write/status on the node; requires the bootstrap port of a "
   "translator startup"},
  {"name", OPT_NAME, "NAME", 0,
   "With --serve: only accept device_open for this name (default: accept "
   "any, including the empty rest of libstore's `@node:' form)"},
  {"host-priv-port", OPT_HOST_PRIV, "PORT", 0,
   "Host privileged port PORT (bootstrap; stored for compatibility, the "
   "libmachdev bootstrap fetches fresh ones via fsys_getpriv)"},
  {"device-master-port", OPT_DEV_MASTER, "PORT", 0,
   "Device master port PORT (bootstrap; see --host-priv-port)"},
  {"next-task", OPT_NEXT_TASK, "TASK", 0,
   "Resume TASK as the next bootstrap task after bringing the disk up "
   "(bootstrap mode: serve as the root disk server)"},
  {"verbose", 'v', 0, 0,
   "Log every request posting and completion"},
  {0}
};

static error_t
parse_opt (int key, char *arg, struct argp_state *state)
{
  switch (key)
    {
    case 'd':
      opt_device = arg;
      break;
    case 's':
      opt_selftest = 1;
      break;
    case 'v':
      vblk_verbose = 1;
      break;
    case OPT_STATUS:
      opt_status = 1;
      break;
    case OPT_WRITE_TEST:
      opt_write_test = 1;
      break;
    case OPT_READ_RANGE:
      opt_read_range = arg;
      break;
    case OPT_SERVE:
      opt_serve = 1;
      break;
    case OPT_NAME:
      opt_name = arg;
      break;
    case OPT_HOST_PRIV:
      opt_host_priv = atoi (arg);
      break;
    case OPT_DEV_MASTER:
      opt_dev_master = atoi (arg);
      break;
    case OPT_NEXT_TASK:
      opt_next_task = atoi (arg);
      break;
    case ARGP_KEY_ARG:
      argp_error (state, "unexpected argument '%s' (translator mode is "
                  "--serve)", arg);
      return EINVAL;
    default:
      return ARGP_ERR_UNKNOWN;
    }
  (void) state;
  return 0;
}

static const struct argp argp =
{
  options, parse_opt,
  "[--device=SLOT] [--selftest [--write-test]] [--status] "
  "[--read-range=SECTOR[:COUNT]] [--verbose] | bootstrap: "
  "--next-task=TASK [--host-priv-port=PORT --device-master-port=PORT]",
  "Virtio block driver: raw block I/O on QEMU's virtio-blk PCI device "
  "(0x1af4:0x1001), with the Mach device semantics layer and a Mach "
  "device translator mode for Hurd storage consumers"
};

static struct vblk_dev vblk;

/* --read-range: bring the device up, read SECTOR[:COUNT] through the
   normal request path, and dump the raw bytes to standard output.  The
   test harness checksums this output against the same range of the
   backing image on the host, so stdout carries data only; every
   diagnostic goes to stderr.  */
static void
read_range (const char *spec)
{
  char *end;
  uint64_t sector, count = 1, total;
  uint8_t *buf;
  size_t n = 0;
  error_t err;

  errno = 0;
  sector = strtoull (spec, &end, 0);
  if (end == spec || errno != 0)
    error (1, errno ?: EINVAL, "--read-range: bad sector in %s", spec);
  if (*end == ':')
    {
      count = strtoull (end + 1, &end, 0);
      if (*end != '\0' || errno != 0 || count == 0)
        error (1, errno ?: EINVAL, "--read-range: bad count in %s", spec);
    }
  if (*end != '\0')
    error (1, EINVAL, "--read-range: trailing junk in %s", spec);
  /* 64 MiB per invocation is far past what any harness asks for and
     bounds the allocation a typo could request.  */
  if (count > (64ULL << 20) / VBLK_SECTOR_SIZE)
    error (1, EINVAL, "--read-range: count %llu too large",
           (unsigned long long) count);

  err = vblk_start_device (opt_device);
  if (err)
    error (1, err, "device bring-up failed");
  if (sector > vblk.capacity || count > vblk.capacity - sector)
    error (1, EIO, "--read-range: sectors %llu..%llu past capacity %llu",
           (unsigned long long) sector,
           (unsigned long long) (sector + count - 1),
           (unsigned long long) vblk.capacity);

  total = count * VBLK_SECTOR_SIZE;
  buf = malloc (total);
  if (!buf)
    error (1, ENOMEM, "--read-range: %llu bytes", (unsigned long long) total);
  err = vblk_dev_read (&vblk, sector, buf, total, &n);
  if (err || n != total)
    error (1, err ?: EIO, "--read-range: read at %llu: %s (n %zu)",
           (unsigned long long) sector, strerror (err ?: EIO), n);
  if (fwrite (buf, 1, n, stdout) != n || fflush (stdout) != 0)
    error (1, errno, "--read-range: writing %zu bytes to stdout", n);

  free (buf);
  vblk_stop_device ();
}

static volatile sig_atomic_t quit_flag;

static void
quit_handler (int sig)
{
  quit_flag = 1;
}

error_t
vblk_start_device (const char *override)
{
  return vblk_dev_init (&vblk, override);
}

void
vblk_stop_device (void)
{
  vblk_dev_quiesce (&vblk);
}

/* --status: probe and read the config region only.  The probe enables
   the PCI device but never touches the virtio status register, so no
   reset, no handshake, no disturbance of a running driver.  */
error_t
vblk_status (const char *override)
{
  struct virtio_pci_dev v;
  uint32_t offered, lo, hi;
  uint64_t capacity;
  error_t err;

  v.name = "virtio-blk";
  v.device_id = VBLK_DEVICE_ID;
  err = virtio_pci_probe (&v, override);
  if (err)
    return err;

  offered = virtio_pci_read_host_features (&v);
  lo = virtio_pci_read_config (&v, VBLK_CFG_CAPACITY);
  hi = virtio_pci_read_config (&v, VBLK_CFG_CAPACITY + 4);
  capacity = (uint64_t) lo | ((uint64_t) hi << 32);
  printf ("capacity=%llu sectors", (unsigned long long) capacity);
  if (capacity <= UINT64_MAX / VBLK_SECTOR_SIZE)
    printf (" (%llu bytes)", (unsigned long long) (capacity * VBLK_SECTOR_SIZE));
  printf (", offered features 0x%x%s%s%s%s\n",
          offered,
          offered & (1u << VBLK_F_RO) ? ", read-only" : "",
          offered & (1u << VBLK_F_FLUSH) ? ", flush" : "",
          offered & (1u << VBLK_F_BLK_SIZE) ? ", blk_size" : "",
          offered & (1u << VBLK_F_SIZE_MAX) ? ", size_max" : "");
  /* The config region is static, so the optional fields are readable
     before the handshake; interpret them only under their offered bits,
     the same gate init applies after negotiation.  */
  {
    uint32_t blk_size = 0, size_max = 0, seg_max = 0;

    if (offered & (1u << VBLK_F_BLK_SIZE))
      blk_size = virtio_pci_read_config (&v, VBLK_CFG_BLK_SIZE);
    if (offered & (1u << VBLK_F_SIZE_MAX))
      size_max = virtio_pci_read_config (&v, VBLK_CFG_SIZE_MAX);
    if (offered & (1u << VBLK_F_SEG_MAX))
      seg_max = virtio_pci_read_config (&v, VBLK_CFG_SEG_MAX);
    printf ("limits: blk_size %u, size_max %u, seg_max %u\n",
            blk_size, size_max, seg_max);
  }
  return 0;
}

/* Phase 1 gate (plan T2-T4): device found, features negotiated, queue
   programmed, config readable; raw read of sector 0 and a bounds check;
   the Mach semantics layer consistent (size/records, open modes, unit
   and range errors, flush flavor); and, with --write-test, a write/
   flush/read-back round trip on the disposable-image policy of plan D5.
   */
error_t
vblk_selftest (int write_test)
{
  struct vblk_hurd h, h_ro;
  dev_status_data_t sizes;
  mach_msg_type_number_t count;
  uint8_t a[VBLK_SECTOR_SIZE], b[VBLK_SECTOR_SIZE];
  uint8_t pattern[VBLK_SECTOR_SIZE];
  size_t n = 0;
  int i;
  error_t err;

  /* Handshake and feature policy: the readback must equal exactly what
     was accepted, the same check the balloon selftest makes.  */
  if (virtio_pci_read_guest_features (&vblk.pci) != vblk.accepted)
    {
      fprintf (stderr, "virtio-blk selftest: guest_features 0x%x does not "
               "match accepted 0x%x\n",
               virtio_pci_read_guest_features (&vblk.pci), vblk.accepted);
      return EIO;
    }
  if (vblk.capacity == 0 || vblk.size_bytes != vblk.capacity * VBLK_SECTOR_SIZE)
    {
      fprintf (stderr, "virtio-blk selftest: implausible capacity %llu\n",
               (unsigned long long) vblk.capacity);
      return EIO;
    }

  /* Raw read of sector 0, and one past the end, which must fail.  */
  err = vblk_dev_read (&vblk, 0, a, sizeof a, &n);
  if (err || n != sizeof a)
    {
      fprintf (stderr, "virtio-blk selftest: read sector 0: %s (n %zu)\n",
               strerror (err), n);
      return err ? err : EIO;
    }
  err = vblk_dev_read (&vblk, vblk.capacity, b, sizeof b, &n);
  if (err == 0)
    {
      fprintf (stderr, "virtio-blk selftest: read past capacity succeeded\n");
      return EIO;
    }

  /* The chunk loop: a transfer longer than one request exercises the
     per-chunk posting and the sector advance between chunks.  Compare
     the halves of one long read against single-chunk reads of the same
     ranges, so an off-by-one in the advance or a misassembled data
     offset fails loudly.  */
  {
    size_t chunk = vblk.max_transfer;
    size_t unit = vblk.blk_size ? vblk.blk_size : VBLK_SECTOR_SIZE;
    uint8_t *whole, *part;
    size_t got;

    chunk -= chunk % unit;
    if (chunk >= unit
        && vblk.capacity >= 2 * chunk / VBLK_SECTOR_SIZE)
      {
        whole = malloc (2 * chunk);
        part = malloc (chunk);
        if (whole && part)
          {
            err = vblk_dev_read (&vblk, 0, whole, 2 * chunk, &got);
            if (err || got != 2 * chunk)
              {
                fprintf (stderr, "virtio-blk selftest: multi-chunk read: "
                         "err %d (n %zu)\n", err, got);
                free (whole);
                free (part);
                return err ? err : EIO;
              }
            err = vblk_dev_read (&vblk, 0, part, chunk, &got);
            if (err || got != chunk || memcmp (whole, part, chunk) != 0)
              {
                fprintf (stderr, "virtio-blk selftest: multi-chunk first "
                         "half mismatch (err %d, n %zu)\n", err, got);
                free (whole);
                free (part);
                return err ? err : EIO;
              }
            err = vblk_dev_read (&vblk, chunk / VBLK_SECTOR_SIZE, part,
                                 chunk, &got);
            if (err || got != chunk
                || memcmp (whole + chunk, part, chunk) != 0)
              {
                fprintf (stderr, "virtio-blk selftest: multi-chunk second "
                         "half mismatch (err %d, n %zu)\n", err, got);
                free (whole);
                free (part);
                return err ? err : EIO;
              }
            free (whole);
            free (part);
            printf ("multi-chunk read of %zu bytes ok\n", 2 * chunk);
          }
        else
          {
            free (whole);
            free (part);
            printf ("multi-chunk read skipped: cannot allocate buffers\n");
          }
      }
    else
      printf ("multi-chunk read skipped: capacity %llu sectors too small "
              "for two %zu-byte chunks\n",
              (unsigned long long) vblk.capacity, chunk);
  }

  /* Mach semantics layer: size flavors agree with each other and with
     the raw config.  */
  err = vblk_hurd_init (&h, &vblk);
  if (err)
    return err;
  memset (sizes, 0, sizeof sizes);
  count = DEV_STATUS_MAX;
  err = vblk_hurd_get_status (&h, DEV_GET_RECORDS, sizes, &count);
  if (err || count != DEV_GET_RECORDS_COUNT
      || (uint64_t) sizes[DEV_GET_RECORDS_DEVICE_RECORDS] != h.records
      || sizes[DEV_GET_RECORDS_RECORD_SIZE] != (int) h.record_size)
    {
      fprintf (stderr, "virtio-blk selftest: DEV_GET_RECORDS: err %s, count "
               "%u, [%d %d]\n", strerror (err), count,
               sizes[DEV_GET_RECORDS_DEVICE_RECORDS],
               sizes[DEV_GET_RECORDS_RECORD_SIZE]);
      return EIO;
    }
  memset (sizes, 0, sizeof sizes);
  count = DEV_STATUS_MAX;
  err = vblk_hurd_get_status (&h, DEV_GET_SIZE, sizes, &count);
  if (err || count != DEV_GET_SIZE_COUNT
      || (uint64_t) sizes[DEV_GET_SIZE_DEVICE_SIZE] != h.size_bytes
      || sizes[DEV_GET_SIZE_RECORD_SIZE] != (int) h.record_size)
    {
      fprintf (stderr, "virtio-blk selftest: DEV_GET_SIZE: err %s, count %u, "
               "[%d %d]\n", strerror (err), count,
               sizes[DEV_GET_SIZE_DEVICE_SIZE],
               sizes[DEV_GET_SIZE_RECORD_SIZE]);
      return EIO;
    }
  if (h.size_bytes != h.records * h.record_size
      || h.size_bytes != vblk.size_bytes)
    {
      fprintf (stderr, "virtio-blk selftest: size/records disagree with "
               "capacity\n");
      return EIO;
    }

  /* Open semantics: RW by default (RO on a read-only device), RO
     handles reject writes, double open is refused, close is idempotent,
     and I/O needs an open.  */
  err = vblk_hurd_open (&h, vblk.ro ? D_READ : (D_READ | D_WRITE));
  if (err)
    {
      /* D_* codes are not errno values, so they print numerically.  */
      fprintf (stderr, "virtio-blk selftest: open: err %d\n", err);
      return err;
    }
  if (vblk_hurd_open (&h, D_READ) != D_ALREADY_OPEN)
    {
      fprintf (stderr, "virtio-blk selftest: double open not refused\n");
      return EIO;
    }
  err = vblk_hurd_open (&h_ro, D_READ);
  if (err)
    {
      fprintf (stderr, "virtio-blk selftest: open RO: err %d\n", err);
      return err;
    }

  /* recnum 0 through the semantics layer must return the same bytes the
     raw path read.  */
  memset (b, 0, sizeof b);
  err = vblk_hurd_read (&h, 0, 0, b, sizeof b, &i);
  if (err || i != (int) sizeof b || memcmp (a, b, sizeof b) != 0)
    {
      fprintf (stderr, "virtio-blk selftest: semantics read: err %d (n %d)\n",
               err, i);
      return err ? err : EIO;
    }

  /* Unit and range policy.  */
  if (vblk_hurd_read (&h, 0, 0, b, 100, &i) != D_INVALID_SIZE)
    {
      fprintf (stderr, "virtio-blk selftest: misaligned read not refused\n");
      return EIO;
    }
  if (vblk_hurd_read (&h, 0, h.records, b, sizeof b, &i) != D_INVALID_RECNUM)
    {
      fprintf (stderr, "virtio-blk selftest: out-of-range read not refused\n");
      return EIO;
    }
  if (vblk_hurd_write (&h_ro, 0, 0, a, sizeof a, &i) != D_READ_ONLY)
    {
      fprintf (stderr, "virtio-blk selftest: RO handle accepted a write\n");
      return EIO;
    }

  /* Status flavors: unknown flavor refused; flush served only when the
     feature was negotiated.  */
  count = DEV_STATUS_MAX;
  if (vblk_hurd_get_status (&h, 0x1234, sizes, &count) != D_INVALID_OPERATION)
    {
      fprintf (stderr, "virtio-blk selftest: unknown flavor accepted\n");
      return EIO;
    }
  if (vblk_hurd_set_status (&h, 0x1234, NULL, 0) != D_INVALID_OPERATION)
    {
      fprintf (stderr, "virtio-blk selftest: unknown set_status accepted\n");
      return EIO;
    }
  /* The inband/map surface is refused rather than crash-prone;
     IO_INBAND_MAX (128 bytes) could never carry block data anyway.  */
  if (vblk_hurd_read_inband (&h, 0, 0, 0, NULL) != D_INVALID_OPERATION
      || vblk_hurd_write_inband (&h, 0, 0, NULL, 0, &i) != D_INVALID_OPERATION)
    {
      fprintf (stderr, "virtio-blk selftest: inband I/O not refused\n");
      return EIO;
    }
  {
    memory_object_t pager;

    if (vblk_hurd_map (&h, 0, 0, 0, &pager, 0) != D_INVALID_OPERATION)
      {
        fprintf (stderr, "virtio-blk selftest: device_map not refused\n");
        return EIO;
      }
  }
  err = vblk_hurd_set_status (&h, DEV_FLUSH_CACHE, NULL, 0);
  if (vblk.flush ? err != 0 : err != D_INVALID_OPERATION)
    {
      fprintf (stderr, "virtio-blk selftest: DEV_FLUSH_CACHE: %s\n",
               strerror (err));
      return EIO;
    }

  err = vblk_hurd_close (&h);
  if (err || vblk_hurd_close (&h) != 0)
    {
      fprintf (stderr, "virtio-blk selftest: close: %s\n", strerror (err));
      return err ? err : EIO;
    }
  if (vblk_hurd_read (&h, 0, 0, b, sizeof b, &i) != D_INVALID_OPERATION)
    {
      fprintf (stderr, "virtio-blk selftest: I/O after close not refused\n");
      return EIO;
    }

  /* Write path, only when explicitly requested on a writable image.  */
  if (write_test)
    {
      if (vblk.ro)
        printf ("write test skipped: device is read-only\n");
      else
        {
          err = vblk_hurd_open (&h, D_READ | D_WRITE);
          if (err)
            {
              fprintf (stderr, "virtio-blk selftest: reopen RW: err %d\n",
                       err);
              return err;
            }
          for (i = 0; i < (int) sizeof pattern; i++)
            pattern[i] = (uint8_t) (i * 7 + 1);
          err = vblk_hurd_write (&h, 0, 0, pattern, sizeof pattern, &i);
          if (err || i != (int) sizeof pattern)
            {
              fprintf (stderr, "virtio-blk selftest: write: err %d (n %d)\n",
                       err, i);
              return err ? err : EIO;
            }
          err = vblk_hurd_set_status (&h, DEV_FLUSH_CACHE, NULL, 0);
          if (err)
            {
              fprintf (stderr, "virtio-blk selftest: flush: err %d\n", err);
              return err;
            }
          memset (b, 0, sizeof b);
          err = vblk_hurd_read (&h, 0, 0, b, sizeof b, &i);
          if (err || i != (int) sizeof b || memcmp (b, pattern, sizeof b) != 0)
            {
              fprintf (stderr, "virtio-blk selftest: write/read-back "
                       "mismatch: %s\n", strerror (err));
              return err ? err : EIO;
            }
          vblk_hurd_close (&h);
          printf ("write/flush/read-back of sector 0 ok\n");
        }
    }

  printf ("offered features 0x%x, accepted 0x%x\n", vblk.offered,
          vblk.accepted);
  printf ("capacity=%llu sectors (%llu bytes), record_size %u, records %llu\n",
          (unsigned long long) vblk.capacity,
          (unsigned long long) vblk.size_bytes, h.record_size,
          (unsigned long long) h.records);
  printf ("raw read of sector 0 ok, read past capacity refused\n");
  printf ("Mach semantics: size/records consistent, open/mode/range/flush "
          "policy enforced\n");
  return 0;
}

/* Daemon mode: own the device, log config changes and counters, and
   leave cleanly on SIGINT/SIGTERM.  */
int
main (int argc, char **argv)
{
  error_t err;
  struct sigaction sa;
  uint32_t seq = 0;
  mach_port_t next_task;

  argp_parse (&argp, argc, argv, 0, 0, 0);

  if (opt_write_test && !opt_selftest)
    error (1, 0, "--write-test only applies to --selftest");

  if (opt_name && !opt_serve && !opt_next_task)
    error (1, 0, "--name only applies to --serve");

  if (opt_next_task && (opt_selftest || opt_status || opt_read_range
                        || opt_serve))
    error (1, 0, "--next-task (bootstrap) is exclusive of --selftest, "
                 "--status, --read-range and --serve");

  /* Bootstrap mode: serve the disk through libmachdev and resume the
     next task in the chain.  The device is deliberately NOT brought up
     here: libpciaccess needs a device master whose open of "pci"
     reaches the pci-arbiter, and the --device-master-port grub hands
     us is the kernel's, which serves no "pci" device.  The bring-up
     happens inside vblk_machdev_run, after machdev_trivfs_init has
     replaced these ports from fsys_getpriv (the rumpdisk order).
     Stdio is not on the console until machdev_trivfs_init reopens it,
     so milestones before that point go through mach_print.  */
  if (opt_next_task)
    {
      if (!vblk_machdev_run)
        {
          mach_print ("virtio-blk: bootstrap requires the static build "
                      "(virtio-blk.static, linked with libmachdev)\n");
          error (1, 0, "bootstrap requires the static build "
                       "(virtio-blk.static, linked with libmachdev)");
        }

      mach_print ("virtio-blk: bootstrap mode\n");
      _hurd_host_priv = opt_host_priv;
      _hurd_device_master = opt_dev_master;
      next_task = opt_next_task;

      vblk_machdev_run (&vblk, next_task, opt_name, opt_device,
                        argc, argv);
      /* Never returns.  */
    }

  if (opt_read_range)
    {
      read_range (opt_read_range);
      /* One-shot mode: returning is mandatory, otherwise main falls
         through into daemon mode and the process never exits (the
         harness's `--read-range | sha256sum` pipeline then blocks on
         the pipe forever).  */
      return 0;
    }

  if (opt_status)
    {
      err = vblk_status (opt_device);
      if (err)
        error (1, err, "virtio-blk device not usable");
      return 0;
    }

  if (opt_selftest)
    {
      err = vblk_start_device (opt_device);
      if (err)
        error (1, err, "device bring-up failed");
      err = vblk_selftest (opt_write_test);
      if (err)
        error (1, err, "selftest failed");
      printf ("selftest ok\n");
      vblk_stop_device ();
      return 0;
    }

  if (opt_serve)
    {
      if (vblk_machdev_run)
        /* Static build: the libmachdev translator (devnode pattern)
           serves device_open on the node; vblk_serve's libports
           scaffold is not linked here.  Bring-up happens inside,
           after machdev_trivfs_init (bootstrap mode needs the
           fsys_getpriv ports first; --serve just follows the same
           order).  */
        vblk_machdev_run (&vblk, MACH_PORT_NULL, opt_name, opt_device,
                          argc, argv);
      else
        {
          /* Become the Mach device translator.  Bring-up failure is
             fatal before trivfs_startup replies to the parent, so
             settrans reports it instead of hanging on a dead node.  */
          err = vblk_start_device (opt_device);
          if (err)
            error (4, err, "virtio-blk device not usable");

          if (vblk_serve)
            vblk_serve (&vblk, opt_name);
          else
            error (1, 0, "--serve unavailable: no translator backend "
                         "linked");
        }
    }

  sa.sa_handler = quit_handler;
  sa.sa_flags = 0;
  sigemptyset (&sa.sa_mask);
  sigaction (SIGINT, &sa, NULL);
  sigaction (SIGTERM, &sa, NULL);

  err = vblk_start_device (opt_device);
  if (err)
    /* No device, or one that will not talk: die visibly instead of
       leaving a daemon behind that looks alive.  */
    error (4, err, "virtio-blk device not usable");

  for (; !quit_flag; )
    {
      err = virtio_irq_wait_config (&vblk.irq, 1, &seq);
      if (err && err != ETIMEDOUT)
        error (1, err, "waiting for config change");
      if (!err)
        vblk_dev_reread_config (&vblk);
    }

  {
    const struct vblk_stats *s = vblk_dev_stats (&vblk);

    fprintf (stderr,
             "virtio-blk: reads %llu (%llu bytes), writes %llu (%llu bytes), "
             "flushes %llu, errors %llu, bad completions %llu, config "
             "changes %llu\n",
             (unsigned long long) s->reads, (unsigned long long) s->read_bytes,
             (unsigned long long) s->writes, (unsigned long long) s->write_bytes,
             (unsigned long long) s->flushes,
             (unsigned long long) s->errors,
             (unsigned long long) s->bad_completions,
             (unsigned long long) s->config_changes);
  }
  vblk_stop_device ();
  return 0;
}
