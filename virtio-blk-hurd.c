/* virtio-blk Hurd integration: the Mach device semantics layer.

   This module implements the MVP subset of the
   Mach block-device contract on top of the raw request
   codes libstore/storeio expect are fixed and testable before any port
   publication exists:

   - record_size is 512, or the negotiated blk_size when it is a whole
     number of virtio sectors; virtio sector = recnum * (record_size/512),
     with byte amounts carried as counts, the way libstore's device
     backend forwards store addresses (hurd/libstore/device.c).
   - DEV_GET_SIZE reports [size_bytes, record_size] and DEV_GET_RECORDS
     reports [records, record_size], and the two are consistent by
     construction (size == records * record_size), which is what
     store_device_create's run math asserts.
   - virtio status maps to D_IO_ERROR/D_INVALID_OPERATION, writes on a
     read-only device or handle to D_READ_ONLY, out-of-range recnums to
     D_INVALID_RECNUM, misaligned sizes to D_INVALID_SIZE, and unknown
     flavors/modes to D_INVALID_OPERATION instead of crashing.

   What is deliberately absent: the MIG demuxer, port set, and name
   publication (virtio-blk-serve.c).  */

#include <errno.h>
#include <limits.h>
#include <stdio.h>

#include "virtio-blk.h"

/* libstore's device backend passes mode 0 for I/O and D_READ or
   D_READ|D_WRITE for open, with D_NOWAIT the only other bit it sets.
   Anything else is a caller bug; reject it rather than guessing.  D_WRITE
   alone is refused the way an open-only-for-write handle is: reads are
   what the Mach block contract is built around.  */
static int
vblk_hurd_mode_ok (int mode)
{
  int rw = mode & (D_READ | D_WRITE);

  if (mode & ~(D_READ | D_WRITE | D_NOWAIT))
    return 0;
  return rw == D_READ || rw == (D_READ | D_WRITE);
}

error_t
vblk_hurd_init (struct vblk_hurd *h, struct vblk_dev *d)
{
  h->dev = d;
  h->opened = 0;
  h->mode = 0;

  /* BLK_SIZE only sets the record size when it is a whole number of
     virtio sectors; the engine already normalized it otherwise.  */
  h->record_size = d->blk_size ? d->blk_size : VBLK_SECTOR_SIZE;

  /* Round the extent down to whole records so DEV_GET_SIZE and
     DEV_GET_RECORDS agree exactly (size == records * record_size); a
     capacity that is not a whole number of records is truncated, the
     same MVP baseline policy.  */
  h->records = d->size_bytes / h->record_size;
  h->size_bytes = h->records * h->record_size;

  /* The Mach status array is int[], so extents wider than INT_MAX are
     truncated for the consumer; libstore's run math would reject the
     result anyway, so say so instead of reporting silently wrong size.
     */
  if (h->records > INT_MAX || h->size_bytes > INT_MAX)
    fprintf (stderr,
             "virtio-blk: extents exceed the Mach int status width; "
             "consumers see truncated values\n");
  return 0;
}

error_t
vblk_hurd_open (struct vblk_hurd *h, int mode)
{
  int rw = mode & (D_READ | D_WRITE);

  if (h->opened)
    return D_ALREADY_OPEN;
  if (!vblk_hurd_mode_ok (mode))
    {
      fprintf (stderr, "virtio-blk: open: unsupported mode 0x%x\n", mode);
      return D_INVALID_OPERATION;
    }
  if ((rw & D_WRITE) && h->dev->ro)
    /* 2509 both here and as Hurd's ED_READ_ONLY, which is the value
       libstore's RW->RO open fallback checks (hurd/libstore/device.c).
       */
    return D_READ_ONLY;
  h->opened = 1;
  h->mode = rw;
  return 0;
}

error_t
vblk_hurd_close (struct vblk_hurd *h)
{
  /* Idempotent, so a close after a failed open is not an error.  */
  h->opened = 0;
  h->mode = 0;
  return 0;
}

error_t
vblk_hurd_get_status (struct vblk_hurd *h, dev_flavor_t flavor, int *status,
                      mach_msg_type_number_t *count)
{
  /* The status array is an int array by Mach convention, so values are
     reported through it even though the extents are 64-bit on this
     host; consumers assert the returned count first.  */
  if (*count < 2)
    {
      fprintf (stderr, "virtio-blk: get_status: status array too small (%u)\n",
               (unsigned) *count);
      *count = 0;
      return D_INVALID_OPERATION;
    }

  switch (flavor)
    {
    case DEV_GET_RECORDS:
      status[DEV_GET_RECORDS_DEVICE_RECORDS] = (int) h->records;
      status[DEV_GET_RECORDS_RECORD_SIZE] = (int) h->record_size;
      *count = DEV_GET_RECORDS_COUNT;
      return 0;
    case DEV_GET_SIZE:
      status[DEV_GET_SIZE_DEVICE_SIZE] = (int) h->size_bytes;
      status[DEV_GET_SIZE_RECORD_SIZE] = (int) h->record_size;
      *count = DEV_GET_SIZE_COUNT;
      return 0;
    default:
      fprintf (stderr, "virtio-blk: get_status: unknown flavor %u\n", flavor);
      *count = 0;
      return D_INVALID_OPERATION;
    }
}

error_t
vblk_hurd_set_status (struct vblk_hurd *h, dev_flavor_t flavor,
                      const int *status, mach_msg_type_number_t count)
{
  error_t err;

  (void) status;
  (void) count;
  switch (flavor)
    {
    case DEV_FLUSH_CACHE:
      err = vblk_dev_flush (h->dev);
      if (err == EOPNOTSUPP)
        return D_INVALID_OPERATION;
      return err ? D_IO_ERROR : 0;
    default:
      fprintf (stderr, "virtio-blk: set_status: unknown flavor %u\n", flavor);
      return D_INVALID_OPERATION;
    }
}

/* Read/write through the recnum/record_size contract.  Mode is 0 (or
   D_NOWAIT, which a synchronous disk never distinguishes).  Returns the
   transferred count for partial-transfer reporting, as the Mach device
   RPCs do.  */
static error_t
vblk_hurd_rw (struct vblk_hurd *h, int mode, recnum_t recnum, void *data,
              int bytes, int *xferred, int is_write)
{
  struct vblk_dev *d = h->dev;
  uint64_t off, nrecords;
  size_t done = 0;
  error_t err;

  *xferred = 0;
  if (!h->opened)
    return D_INVALID_OPERATION;
  if (mode & ~D_NOWAIT)
    {
      fprintf (stderr, "virtio-blk: %s: unsupported mode 0x%x\n",
               is_write ? "write" : "read", mode);
      return D_INVALID_OPERATION;
    }
  if (is_write)
    {
      if (!(h->mode & D_WRITE))
        return D_READ_ONLY;
      if (d->ro)
        return D_READ_ONLY;
    }
  if (bytes < 0)
    return D_INVALID_SIZE;
  if (bytes == 0)
    return 0;
  if ((uint64_t) bytes % h->record_size)
    return D_INVALID_SIZE;
  if (recnum >= h->records)
    return D_INVALID_RECNUM;

  nrecords = (uint64_t) bytes / h->record_size;
  if (nrecords > h->records - recnum)
    return D_INVALID_RECNUM;
  off = (uint64_t) recnum * h->record_size;

  err = is_write
    ? vblk_dev_write (d, off / VBLK_SECTOR_SIZE, data, bytes, &done)
    : vblk_dev_read (d, off / VBLK_SECTOR_SIZE, data, bytes, &done);
  *xferred = (int) done;
  switch (err)
    {
    case 0:
      return 0;
    case EIO:
      return D_IO_ERROR;
    case EROFS:
      return D_READ_ONLY;
    case EINVAL:
      return D_INVALID_SIZE;
    case EOPNOTSUPP:
      return D_INVALID_OPERATION;
    default:
      return err;
    }
}

error_t
vblk_hurd_read (struct vblk_hurd *h, int mode, recnum_t recnum, void *data,
                int bytes_wanted, int *bytes_read)
{
  return vblk_hurd_rw (h, mode, recnum, data, bytes_wanted, bytes_read, 0);
}

error_t
vblk_hurd_write (struct vblk_hurd *h, int mode, recnum_t recnum,
                 const void *data, int bytes, int *bytes_written)
{
  return vblk_hurd_rw (h, mode, recnum, (void *) data, bytes, bytes_written,
                       1);
}

/* Inband carries at most IO_INBAND_MAX (128) bytes, far below any block
   transfer; map is unused by the MVP consumers.  Answer with the
   compatible error rather than crashing.  */
error_t
vblk_hurd_read_inband (struct vblk_hurd *h, int mode, recnum_t recnum,
                       int bytes_wanted, io_buf_ptr_inband_t data)
{
  (void) h;
  (void) mode;
  (void) recnum;
  (void) bytes_wanted;
  (void) data;
  fprintf (stderr, "virtio-blk: read_inband is not implemented\n");
  return D_INVALID_OPERATION;
}

error_t
vblk_hurd_write_inband (struct vblk_hurd *h, int mode, recnum_t recnum,
                        const char *data, int bytes,
                        int *bytes_written)
{
  (void) h;
  (void) mode;
  (void) recnum;
  (void) data;
  (void) bytes;
  *bytes_written = 0;
  fprintf (stderr, "virtio-blk: write_inband is not implemented\n");
  return D_INVALID_OPERATION;
}

error_t
vblk_hurd_map (struct vblk_hurd *h, vm_prot_t prot, vm_offset_t offset,
               vm_size_t size, memory_object_t *pager, int unmap)
{
  (void) h;
  (void) prot;
  (void) offset;
  (void) size;
  (void) pager;
  (void) unmap;
  fprintf (stderr, "virtio-blk: device_map is not implemented\n");
  return D_INVALID_OPERATION;
}
