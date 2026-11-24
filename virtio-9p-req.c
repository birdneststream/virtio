/* 9P PDU framing, the request pool, and the Phase 1 selftest.

   This file stays a framing layer, not a protocol interpreter: it checks
   the 4-byte little-endian size prefix, owns the TX/RX DMA buffers
   through the queue module, posts requests and collects their
   completions into the reply queue the stream adapter reads.  It also
   runs the one hand-built exchange, a Tversion, that proves the device,
   the feature negotiation, the queue and the DMA addresses all work
   before the stream adapter exists.  */

#include <errno.h>
#include <pthread.h>
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

/* Request slots.  The device has one queue, but one queue carries many
   descriptor chains at once: every request owns a slot with its own TX
   and RX DMA buffers for as long as it is in flight, so several 9pfs
   threads can have a request on the device at the same time.

   A slot is FREE, INFLIGHT from the post until its completion is
   collected, and DONE while its reply waits in the done ring for the
   reader.  The done ring orders replies by completion, which need not
   match the order of the requests; the stream adapter hands them to
   9pfs in that order and 9pfs matches each one by tag.  A slot becomes
   FREE only once its reply has been read out, so a reader that falls
   behind slows the writers instead of letting replies pile up
   somewhere else.

   Slots are allocated on first use.  Each costs V9P_TX_BUF_SIZE +
   V9P_RX_BUF_SIZE of contiguous DMA memory, which only concurrency
   needs, so a translator that never sees two requests at once never
   allocates a second slot.  */

struct v9p_slot
{
  uint8_t *tx_buf;
  uint8_t *rx_buf;
  uint64_t tx_phys;
  uint64_t rx_phys;
  /* Head descriptor of the posted chain, the handle a completion
     carries.  NO_DESC while the slot holds no posted chain.  */
  uint16_t head;
  uint32_t reply_len;   /* bytes of reply in rx_buf */
  uint32_t reply_off;   /* bytes of the reply already read out */
  int state;
};

enum
{
  SLOT_FREE,
  SLOT_INFLIGHT,
  SLOT_DONE
};

static struct v9p_slot slots[V9P_MAX_INFLIGHT];
static unsigned int nslots;     /* slots allocated so far */
static unsigned int max_slots;  /* min (V9P_MAX_INFLIGHT, queue size / 2) */
static error_t pool_error;      /* sticky, fatal */

/* Completed requests in completion order, by slot index.  */
static unsigned int done_ring[V9P_MAX_INFLIGHT];
static unsigned int done_head, done_len;
/* Set while a reader copies out of the head slot's rx buffer, outside
   the lock.  */
static int reply_reading;

static pthread_mutex_t pool_lock = PTHREAD_MUTEX_INITIALIZER;
/* Signalled when a slot becomes FREE.  */
static pthread_cond_t free_cv = PTHREAD_COND_INITIALIZER;
/* Signalled when a reply is queued, when the head reply has been read
   out, and when the pool fails.  */
static pthread_cond_t done_cv = PTHREAD_COND_INITIALIZER;

/* Under pool_lock.  Wakes every waiter; each one rechecks its own
   condition and sees the error where it applies.  */
static void
pool_fail_locked (error_t err)
{
  if (pool_error)
    return;
  pool_error = err;
  pthread_cond_broadcast (&free_cv);
  pthread_cond_broadcast (&done_cv);
}

/* Mark the whole transport failed: the device lost a reply or the
   completion path is out of sync.  Replies already queued are still
   read; everything after them fails with ERR.  */
void
v9p_fail (error_t err)
{
  pthread_mutex_lock (&pool_lock);
  pool_fail_locked (err);
  pthread_mutex_unlock (&pool_lock);
}

static error_t
slot_alloc (struct v9p_slot *s)
{
  error_t err;

  err = virtio_dma_alloc (&v9p_pci, V9P_TX_BUF_SIZE,
                          (void **) &s->tx_buf, &s->tx_phys);
  if (err)
    return err;
  err = virtio_dma_alloc (&v9p_pci, V9P_RX_BUF_SIZE,
                          (void **) &s->rx_buf, &s->rx_phys);
  if (err)
    /* The TX buffer stays allocated.  Nothing in this driver frees
       contiguous memory: the vring is never released either.  */
    return err;
  s->state = SLOT_FREE;
  s->head = VIRTIO_QUEUE_NO_DESC;
  fprintf (stderr, "virtio-9p: request slot %u TX/RX 0x%llx/0x%llx\n",
           (unsigned int) (s - slots),
           (unsigned long long) s->tx_phys,
           (unsigned long long) s->rx_phys);
  return 0;
}

/* Under pool_lock: a FREE slot, allocated now if the pool still has
   room, waiting while every slot is in flight.  */
static error_t
slot_acquire (struct v9p_slot **out)
{
  for (;;)
    {
      unsigned int i;
      error_t err;

      if (pool_error)
        return pool_error;
      for (i = 0; i < nslots; i++)
        if (slots[i].state == SLOT_FREE)
          {
            *out = &slots[i];
            return 0;
          }
      if (nslots < max_slots)
        {
          err = slot_alloc (&slots[nslots]);
          if (!err)
            {
              *out = &slots[nslots];
              nslots++;
              return 0;
            }
          /* Contiguous memory ran out.  Serve with the slots that
             exist rather than retry a failing allocation per request.  */
          fprintf (stderr, "virtio-9p: request slot %u: %s, keeping %u "
                   "slot(s)\n", nslots, strerror (err), nslots);
          max_slots = nslots;
          if (nslots == 0)
            return err;
        }
      pthread_cond_wait (&free_cv, &pool_lock);
    }
}

/* Under pool_lock: the INFLIGHT slot whose posted chain starts at
   descriptor HEAD.  */
static struct v9p_slot *
slot_for_head (uint16_t head)
{
  unsigned int i;

  for (i = 0; i < nslots; i++)
    if (slots[i].state == SLOT_INFLIGHT && slots[i].head == head)
      return &slots[i];
  return NULL;
}

/* Under pool_lock: move every completion in the used ring to the done
   ring.  */
static void
collect_used (void)
{
  for (;;)
    {
      struct virtio_used_elem used;
      struct v9p_slot *s;
      error_t err;

      err = virtio_queue_try_pop_used (&v9p_q, &used);
      if (err == EAGAIN)
        return;
      if (err)
        {
          pool_fail_locked (err);
          return;
        }

      /* The chain's head descriptor identifies the slot.  The pool
         lock serialises this lookup against posts: the pop has already
         returned the chain's descriptors to the free list, so a post
         racing it could hand the same head to a new chain before the
         lookup ran.  */
      s = slot_for_head (used.id);
      if (!s)
        {
          fprintf (stderr, "virtio-9p: completion for unknown chain %u\n",
                   used.id);
          pool_fail_locked (EIO);
          return;
        }
      /* Every reply carries at least the 7-byte header.  A shorter one
         is a device fault, and it must not reach the reader: zero bytes
         read is how the client sees end of stream.  */
      if (used.len < 7 || used.len > V9P_RX_BUF_SIZE)
        {
          fprintf (stderr, "virtio-9p: reply length %u outside 7..%u\n",
                   used.len, V9P_RX_BUF_SIZE);
          pool_fail_locked (EIO);
          return;
        }
      s->state = SLOT_DONE;
      s->reply_len = used.len;
      s->reply_off = 0;
      done_ring[(done_head + done_len) % V9P_MAX_INFLIGHT] = s - slots;
      done_len++;
      pthread_cond_broadcast (&done_cv);
    }
}

/* Under pool_lock, the second half of a completion window: drain the
   used ring, publish the consumed position as the interrupt threshold,
   and recheck the used index, repeating while completions keep landing.
   Returns with the ring drained and the device armed for the next
   completion, and returns that position: the token a sleeper passes to
   virtio_queue_wait_progress.  */
static uint16_t
drain_and_arm (void)
{
  uint16_t token;

  do
    {
      collect_used ();
      token = virtio_queue_enable_used_cb_prepare (&v9p_q);
    }
  while (!pool_error && virtio_queue_poll_used (&v9p_q, token));
  return token;
}

/* Post one request PDU and return without waiting for its reply.  The
   reply is queued for v9p_reply_next when the completion is collected,
   by this call, by another submitter, or by the collector thread.
   Blocks only while every slot is in flight.  */
error_t
v9p_submit (const void *pdu, uint32_t pdu_len)
{
  struct virtio_buf bufs[2];
  struct v9p_slot *s;
  uint16_t head;
  error_t err;

  if (pdu_len < 7 || pdu_len > V9P_TX_BUF_SIZE)
    return EMSGSIZE;

  pthread_mutex_lock (&pool_lock);
  err = slot_acquire (&s);
  if (err)
    {
      pthread_mutex_unlock (&pool_lock);
      return err;
    }
  s->state = SLOT_INFLIGHT;
  s->head = VIRTIO_QUEUE_NO_DESC;
  s->reply_len = s->reply_off = 0;
  pthread_mutex_unlock (&pool_lock);

  memcpy (s->tx_buf, pdu, pdu_len);

  bufs[0].addr = s->tx_phys;
  bufs[0].len = pdu_len;
  bufs[0].flags = 0;
  bufs[1].addr = s->rx_phys;
  bufs[1].len = V9P_RX_BUF_SIZE;
  bufs[1].flags = VIRTQ_DESC_F_WRITE;

  pthread_mutex_lock (&pool_lock);
  /* Post without the kick and record the head before the device can
     complete the chain, so that a drain on any thread finds the slot.
     The pool lock held across the post and the recording is what keeps
     that ahead of the free list handing the head to the next post.  */
  err = virtio_queue_post_head_deferred (&v9p_q, bufs, 2, &head);
  if (err)
    {
      s->state = SLOT_FREE;
      pthread_cond_broadcast (&free_cv);
      pthread_mutex_unlock (&pool_lock);
      return err;
    }
  s->head = head;

  /* A completion window, the same one the collector thread runs minus
     the sleep.  The interrupt is quieted over the kick and the inline
     drain: the emulated device often completes before the kick's exit
     returns, so the drain below usually collects the reply without the
     interrupt, the irqhelp wake and the collector hop.  The window ends
     armed at the consumed position like every window does, which is
     what keeps a sleeping collector wakeable after this thread has
     consumed a completion it never saw.  */
  virtio_queue_disable_used_cb (&v9p_q);
  virtio_queue_notify (&v9p_q);
  drain_and_arm ();
  pthread_mutex_unlock (&pool_lock);
  return 0;
}

int
v9p_reply_ready (void)
{
  int r;

  pthread_mutex_lock (&pool_lock);
  r = done_len > 0;
  pthread_mutex_unlock (&pool_lock);
  return r;
}

/* Copy up to LEN bytes of the oldest queued reply into BUF, waiting for
   a reply when none is queued.  Returns as soon as any reply byte is
   available, so *NREAD may be shorter than LEN; the slot goes back to
   the pool once the last of its reply has been copied out.  Replies
   already queued are returned even after the transport failed; then the
   error is returned.  */
error_t
v9p_reply_next (void *buf, size_t len, size_t *nread)
{
  struct v9p_slot *s;
  size_t n;
  error_t err;

  pthread_mutex_lock (&pool_lock);
  while (done_len == 0 || reply_reading)
    {
      if (done_len == 0 && pool_error)
        {
          err = pool_error;
          pthread_mutex_unlock (&pool_lock);
          return err;
        }
      pthread_cond_wait (&done_cv, &pool_lock);
    }
  s = &slots[done_ring[done_head]];
  reply_reading = 1;
  pthread_mutex_unlock (&pool_lock);

  /* Nothing else touches a DONE slot's buffers, so the copy runs
     outside the lock.  */
  n = s->reply_len - s->reply_off;
  if (n > len)
    n = len;
  memcpy (buf, s->rx_buf + s->reply_off, n);

  pthread_mutex_lock (&pool_lock);
  s->reply_off += n;
  if (s->reply_off == s->reply_len)
    {
      done_head = (done_head + 1) % V9P_MAX_INFLIGHT;
      done_len--;
      s->state = SLOT_FREE;
      pthread_cond_broadcast (&free_cv);
    }
  reply_reading = 0;
  pthread_cond_broadcast (&done_cv);
  pthread_mutex_unlock (&pool_lock);
  *nread = n;
  return 0;
}

/* Collect completions that arrive when no submitter is draining.  It
   owns the queue's completion-moderation policy: one window per wakeup,
   then a sleep armed at the consumed position.  Submitters run the same
   window without the sleep, and every window ends armed, so a sleeper
   is woken by the next completion whichever thread consumed the last
   one.  */
static void *
collector_thread (void *arg)
{
  (void) arg;

  for (;;)
    {
      unsigned int inflight, i;
      uint16_t token;
      error_t err;

      pthread_mutex_lock (&pool_lock);
      if (pool_error)
        {
          pthread_mutex_unlock (&pool_lock);
          break;
        }
      virtio_queue_disable_used_cb (&v9p_q);
      token = drain_and_arm ();
      inflight = 0;
      for (i = 0; i < nslots; i++)
        inflight += slots[i].state == SLOT_INFLIGHT;
      pthread_mutex_unlock (&pool_lock);

      err = virtio_queue_wait_progress (&v9p_q, token,
                                        V9P_EXCHANGE_TIMEOUT);
      if (err == ETIMEDOUT && inflight)
        {
          fprintf (stderr, "virtio-9p: no completion for %u request(s) in "
                   "%d s\n", inflight, V9P_EXCHANGE_TIMEOUT);
          v9p_fail (ETIMEDOUT);
          break;
        }
    }
  return NULL;
}

error_t
v9p_collect_start (void)
{
  pthread_t thread;
  error_t err;

  /* Two descriptors per request, and a post must never find the ring
     short of descriptors.  */
  max_slots = v9p_q.num / 2;
  if (max_slots > V9P_MAX_INFLIGHT)
    max_slots = V9P_MAX_INFLIGHT;

  err = pthread_create (&thread, NULL, collector_thread, NULL);
  if (err)
    return err;
  pthread_detach (thread);
  return 0;
}

/* One synchronous request: the complete outbound PDU in TX, reply
   capacity in RX.  The reply lands in REPLY, at most REPLY_CAP bytes,
   and its length in REPLY_LEN.  Only valid while no other request is in
   flight, since it takes the oldest queued reply: that holds for the
   selftest, which runs before the stream adapter accepts a client.  */
error_t
v9p_exchange (const void *pdu, uint32_t pdu_len,
              void *reply, uint32_t reply_cap, uint32_t *reply_len)
{
  size_t got = 0, n;
  uint32_t size;
  error_t err;

  if (reply_cap > V9P_RX_BUF_SIZE)
    return EMSGSIZE;

  err = v9p_submit (pdu, pdu_len);
  if (err)
    return err;

  for (got = 0; got < 4 || get_le32 (reply) > got; got += n)
    {
      if (got >= 4)
        {
          size = get_le32 (reply);
          if (size > reply_cap)
            {
              fprintf (stderr, "virtio-9p: reply %u > capacity %u\n",
                       size, reply_cap);
              return EMSGSIZE;
            }
        }
      err = v9p_reply_next ((uint8_t *) reply + got, reply_cap - got, &n);
      if (err)
        {
          fprintf (stderr, "virtio-9p: reply wait failed: %s\n",
                   strerror (err));
          return err;
        }
    }
  *reply_len = got;
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
