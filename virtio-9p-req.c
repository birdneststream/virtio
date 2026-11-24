/* 9P PDU framing, the request/response exchange, and the Phase 1 selftest.

   This file stays a framing layer, not a protocol interpreter: it assembles
   the 4-byte little-endian size prefix, owns the TX/RX DMA buffers through
   the queue module, and runs the one hand-built exchange (Tversion) that
   proves the device, the feature negotiation, the queue and the DMA
   addresses all work before the stream adapter exists.  */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "virtio-9p.h"

static void
put_le16 (void *p, uint16_t v)
{
  uint8_t *b = p;
  b[0] = v;
  b[1] = v >> 8;
}

static void
put_le32 (void *p, uint32_t v)
{
  uint8_t *b = p;
  b[0] = v;
  b[1] = v >> 8;
  b[2] = v >> 16;
  b[3] = v >> 24;
}

static uint16_t
get_le16 (const void *p)
{
  const uint8_t *b = p;
  return b[0] | (b[1] << 8);
}

static uint32_t
get_le32 (const void *p)
{
  const uint8_t *b = p;
  return b[0] | (b[1] << 8) | (b[2] << 16) | ((uint32_t) b[3] << 24);
}

/* One request: the complete outbound PDU in TX, reply capacity in RX.  The
   reply lands in REPLY (up to REPLY_CAP bytes) and its length in REPLY_LEN.
   Submits the request, then tries one inline pop of the reply before
   falling back to the armed interrupt wait; the timeout is the fallback
   for a lost interrupt, not the normal path.  */
error_t
v9p_exchange (const void *pdu, uint32_t pdu_len,
              void *reply, uint32_t reply_cap, uint32_t *reply_len)
{
  struct virtio_buf bufs[2];
  struct virtio_used_elem used;
  uint32_t used_len;
  error_t err;

  if (pdu_len < 7 || pdu_len > V9P_TX_BUF_SIZE)
    return EMSGSIZE;
  if (reply_cap > V9P_RX_BUF_SIZE)
    return EMSGSIZE;

  memcpy (v9p_dma.tx_buf, pdu, pdu_len);

  bufs[0].addr = v9p_dma.tx_phys;
  bufs[0].len = pdu_len;
  bufs[0].flags = 0;
  bufs[1].addr = v9p_dma.rx_phys;
  bufs[1].len = reply_cap;
  bufs[1].flags = VIRTQ_DESC_F_WRITE;

  /* The emulated device completes in microseconds, while the interrupt
   * path costs a VM exit, an irqhelp thread wake and a condvar handoff
   * before the waiter sees the entry.  With EVENT_IDX negotiated the
   * completion interrupt is suppressed, one inline pop usually collects
   * the reply, and the blocking fallback arms the used event first so
   * the wake is exactly the next completion.  */
  virtio_queue_disable_used_cb (&v9p_q);

  err = virtio_queue_post (&v9p_q, bufs, 2);
  if (err)
    return err;

  /* Look for the reply with the used notification still suppressed, so
   * the device cannot deliver an interrupt that races this loop.  The
   * first pass is the opportunistic pop.  On the emulated device under
   * KVM the completion does not land before the kick's exit returns, it
   * comes some 75 to 130 us later, and the interrupt path to deliver it
   * costs a VM exit, an irqhelp thread wake and a condvar handoff on
   * top.  So keep looking, with the cheapest primitive that also takes
   * the entry, until the bound runs out.  Past the bound the armed wait
   * below is exactly as before.  */
  {
    unsigned spins;

    for (spins = 0; spins < V9P_EXCHANGE_SPINS; spins++)
      {
        err = virtio_queue_try_pop_used (&v9p_q, &used);
        if (err != EAGAIN)
          goto got_reply;
      }
  }

  /* Not collected inline: arm the used event at the consumed position,
   * close the arm race, then sleep until the used index moves.  */
  {
    uint16_t token = virtio_queue_enable_used_cb_prepare (&v9p_q);

    if (!virtio_queue_poll_used (&v9p_q, token))
      {
        err = virtio_queue_wait_progress (&v9p_q, token,
                                          V9P_EXCHANGE_TIMEOUT);
        if (err == ETIMEDOUT)
          {
            fprintf (stderr, "virtio-9p: reply wait timed out\n");
            return err;
          }
        if (err)
          {
            fprintf (stderr, "virtio-9p: reply wait failed: %s\n",
                     strerror (err));
            return err;
          }
      }
    /* Progress was observed, so with this driver as the queue's only
     * consumer the entry must be collectable now.  A miss is an
     * invariant violation, not something to retry through.  */
    err = virtio_queue_try_pop_used (&v9p_q, &used);
    if (err == EAGAIN)
      {
        fprintf (stderr, "virtio-9p: completion vanished after progress\n");
        return EIO;
      }
  }

got_reply:
  if (err)
    {
      fprintf (stderr, "virtio-9p: reply completion failed: %s\n",
               strerror (err));
      return err;
    }
  if (err)
    {
      fprintf (stderr, "virtio-9p: reply wait failed: %s\n", strerror (err));
      return err;
    }
  used_len = used.len;
  if (used_len > reply_cap)
    {
      fprintf (stderr, "virtio-9p: reply %u > capacity %u\n",
               used_len, reply_cap);
      return EMSGSIZE;
    }
  memcpy (reply, v9p_dma.rx_buf, used_len);
  *reply_len = used_len;
  return 0;
}

/* Hand-built Tversion: size[4] type=100[1] tag[2] msize[4] version[2+8].
   9pfs asks for 16 MiB; QEMU answers with its transport cap.  */
error_t
v9p_selftest (const char *required_tag)
{
  uint8_t pdu[32], reply[128];
  uint32_t pdu_len, rlen, rmsize, vlen;
  error_t err;

  pdu_len = 4 + 1 + 2 + 4 + 2 + 8;
  put_le32 (pdu + 0, pdu_len);
  pdu[4] = 100; /* Tversion */
  put_le16 (pdu + 5, 0);
  put_le32 (pdu + 7, 16u << 20);
  put_le16 (pdu + 11, 8);
  memcpy (pdu + 13, "9P2000.L", 8);

  err = v9p_exchange (pdu, pdu_len, reply, sizeof reply, &rlen);
  if (err)
    {
      fprintf (stderr, "virtio-9p selftest: Tversion exchange: %s\n",
               strerror (err));
      return err;
    }

  if (rlen < 7 + 4 + 2 || get_le32 (reply + 0) != rlen
      || reply[4] != 101 || get_le16 (reply + 5) != 0)
    {
      fprintf (stderr, "virtio-9p selftest: malformed Rversion "
               "(len %u type %u tag %u)\n",
               rlen, rlen > 4 ? reply[4] : 0, rlen > 5 ? get_le16 (reply + 5) : 0);
      return EIO;
    }
  rmsize = get_le32 (reply + 7);
  vlen = get_le16 (reply + 11);

  if (strcmp (required_tag, v9p_tag) != 0)
    {
      fprintf (stderr, "virtio-9p selftest: mount tag mismatch "
               "(wanted \"%s\", device has \"%s\")\n",
               required_tag, v9p_tag);
      return ENODEV;
  }

  {
    uint32_t guest_features = virtio_pci_read_guest_features (&v9p_pci);
    if (!(guest_features & (1u << V9P_F_MOUNT_TAG)))
      {
        fprintf (stderr, "virtio-9p selftest: mount tag feature not "
                 "acknowledged in guest_features (0x%x)\n", guest_features);
        return EIO;
      }
  }

  {
    uint32_t pfn = virtio_pci_read_queue_pfn (&v9p_pci);
    if (pfn != v9p_vring_phys >> 12)
      {
        fprintf (stderr, "virtio-9p selftest: QUEUE_PFN readback 0x%x != "
                 "phys >> 12 (0x%llx)\n", pfn,
                 (unsigned long long) (v9p_vring_phys >> 12));
        return EIO;
      }
    printf ("pfn=%u == phys 0x%llx >> 12\n", pfn,
            (unsigned long long) v9p_vring_phys);
  }

  if (rmsize != V9P_MSIZE_CAP)
    {
      fprintf (stderr, "virtio-9p selftest: negotiated msize %u, expected "
               "%u (transport cap)\n", rmsize, V9P_MSIZE_CAP);
      return EIO;
  }
  if (vlen != 8 || memcmp (reply + 13, "9P2000.L", 8) != 0)
    {
      fprintf (stderr, "virtio-9p selftest: bad Rversion version string\n");
      return EIO;
    }

  printf ("mount tag feature negotiated\n");
  printf ("Rversion ok\n");
  printf ("msize=%u\n", rmsize);
  printf ("version=%.*s\n", (int) vlen, reply + 13);
  return 0;
}
