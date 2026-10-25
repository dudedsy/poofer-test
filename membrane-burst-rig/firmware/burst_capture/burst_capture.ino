// burst_capture.ino
// Membrane burst rig: fast pressure-transient capture on an RP2040-Zero.
//
// Board: "Waveshare RP2040 Zero" in the arduino-pico core (Earle Philhower).
//
// What it does
//   - Runs the ADC continuously in round-robin over N_CH inputs (GP26..GP29)
//     at SPS_TOTAL samples/s, streamed by DMA into a 192 KB ring buffer.
//   - Between captures, prints 1 ms means of every channel (~1 kHz "slow stream").
//   - Watches for a pressure drop: any sensor channel's 1 ms mean falling more than
//     trig_drop counts below its maximum over the previous 20 ms.
//   - On a trigger (or the 't' command) it waits until POST_MS of data exist after
//     the trigger, stops the ADC, dumps PRE_MS + POST_MS of raw samples as binary,
//     then restarts.
//
// Serial protocol (USB CDC, text lines unless noted)
//   host -> device:  t          force a capture now
//                    d<N>       set trigger threshold to N counts (e.g. d20)
//                    ?          print the JSON header line
//   device -> host:  {...}      JSON header (one line, starts with '{')
//                    S,ms,m0,m1,...        slow-stream 1 ms means (counts)
//                    #text      info messages
//                    C,num,frames,n_ch,trig_frame,fs_ch,forced\n
//                        followed by frames*n_ch*2 bytes of little-endian uint16,
//                        channel-interleaved (frame = one sample of every channel),
//                        followed by "\nE\n".
//                        Each sample: bits 0..11 = ADC counts, bit 15 = ADC error flag.
//
// Wiring and supply options: see README / summary doc.

#include <Arduino.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ---------------- configuration ----------------
// 3 sensors on GP26/27/28 (3.3 V supply mode). For 5 V mode with a rail-monitor
// divider on GP29, set N_CH 4 and RAIL_CH 3.
#define N_CH        3
#define RAIL_CH     (-1)        // channel index carrying the 5 V rail divider, or -1

#define SPS_TOTAL   300000UL    // total ADC rate across all channels (max ~500k)
#define PRE_MS      50          // capture length before the trigger point
#define POST_MS     200         // capture length after the trigger point
#define HIST_MS     20          // look-back window for the drop trigger
#define HOLDOFF_MS  300         // re-arm delay after a capture / restart
#define TRIG_DROP_DEFAULT 30    // counts; ~1 psi for a 0-100 psi 0.5-4.5 V sensor at 3.3 V

// ---------------- derived sizes ----------------
#define SPMS        (SPS_TOTAL / 1000UL)       // samples per ms, all channels (300)
#define FPMS        (SPMS / N_CH)              // frames per ms (100 at 3 ch)
#define BLOCK_MS    10
#define BLOCK_SAMP  (SPMS * BLOCK_MS)          // 3000 samples = 6000 bytes
#define NBLK        32
#define RING_SAMP   (BLOCK_SAMP * NBLK)        // 96000 samples = 192000 bytes
#define RING_MS     (BLOCK_MS * NBLK)          // 320 ms

#if (SPS_TOTAL % 1000UL) != 0 || (SPMS % N_CH) != 0
#error "SPS_TOTAL/1000 must be a multiple of N_CH so frames stay aligned"
#endif
#if (PRE_MS + POST_MS + 2 * BLOCK_MS + HIST_MS) >= RING_MS
#error "capture window does not fit in the ring buffer"
#endif

static uint16_t ring[RING_SAMP] __attribute__((aligned(4)));

static int dma_a, dma_b;
static volatile uint32_t blocks_done = 0;   // blocks completed since (re)start

static uint32_t blocks_processed = 0;
static uint32_t epoch_ms = 0;               // ms offset carried across restarts (slow stream only)

// trigger state
static uint16_t trig_drop = TRIG_DROP_DEFAULT;
static float    hist[N_CH][HIST_MS];
static uint32_t hist_n = 0;                 // ms of history collected since (re)start
static uint32_t arm_at_ms = HOLDOFF_MS;     // ms index when the trigger arms
static bool     pending = false;            // a capture is waiting for its POST_MS
static bool     pending_forced = false;
static uint32_t trig_ms = 0;                // ms index of the trigger (relative to restart)
static uint32_t capture_no = 0;

// ---------------- DMA ----------------
static inline uint16_t *block_ptr(uint32_t b) { return &ring[(b % NBLK) * BLOCK_SAMP]; }

static void dma_irq_handler() {
  uint32_t ints = dma_hw->ints1;
  uint32_t mask = (1u << dma_a) | (1u << dma_b);
  if (!(ints & mask)) return;                // shared IRQ, not ours
  // Exactly one of our channels finishes per block. The next block is already being
  // written by the other channel (chained), so point the finished channel at the
  // block after that. TRANS_COUNT reloads automatically on retrigger.
  for (int k = 0; k < 2; k++) {
    int ch = k ? dma_b : dma_a;
    if (ints & (1u << ch)) {
      dma_hw->ints1 = 1u << ch;              // acknowledge
      uint32_t done = blocks_done + 1;
      blocks_done = done;
      dma_channel_set_write_addr(ch, block_ptr(done + 1), false);
    }
  }
}

static void config_channel(int ch, int chain_to, uint16_t *dst) {
  dma_channel_config c = dma_channel_get_default_config(ch);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_ADC);
  channel_config_set_chain_to(&c, chain_to);
  dma_channel_configure(ch, &c, dst, &adc_hw->fifo, BLOCK_SAMP, false);
}

static void start_acq() {
  adc_run(false);
  adc_fifo_drain();
  adc_select_input(0);                       // round robin starts at channel 0
  adc_set_round_robin((1u << N_CH) - 1);
  adc_fifo_setup(true, true, 1, true, false);  // FIFO on, DREQ on, thresh 1, err bit, no shift
  adc_set_clkdiv(48000000.0f / SPS_TOTAL - 1.0f);

  blocks_done = 0;
  blocks_processed = 0;
  hist_n = 0;
  config_channel(dma_a, dma_b, block_ptr(0));
  config_channel(dma_b, dma_a, block_ptr(1));
  dma_hw->ints1 = (1u << dma_a) | (1u << dma_b);
  dma_channel_start(dma_a);
  adc_run(true);
}

static void stop_acq() {
  adc_run(false);
  // Disable both channels before aborting so an abort cannot fire the chain (RP2040-E13).
  hw_clear_bits(&dma_hw->ch[dma_a].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  hw_clear_bits(&dma_hw->ch[dma_b].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  dma_channel_abort(dma_a);
  dma_channel_abort(dma_b);
  dma_hw->ints1 = (1u << dma_a) | (1u << dma_b);
  adc_fifo_drain();
}

// ---------------- output ----------------
static void print_header() {
  Serial.print("{\"fw\":\"burst_capture 2.0\",\"sps_total\":");
  Serial.print(SPS_TOTAL);
  Serial.print(",\"n_ch\":");      Serial.print(N_CH);
  Serial.print(",\"fs_ch\":");     Serial.print(SPS_TOTAL / N_CH);
  Serial.print(",\"rail_ch\":");   Serial.print(RAIL_CH);
  Serial.print(",\"pre_ms\":");    Serial.print(PRE_MS);
  Serial.print(",\"post_ms\":");   Serial.print(POST_MS);
  Serial.print(",\"hist_ms\":");   Serial.print(HIST_MS);
  Serial.print(",\"ring_ms\":");   Serial.print(RING_MS);
  Serial.print(",\"trig_drop\":"); Serial.print(trig_drop);
  Serial.print(",\"adc_bits\":12,\"gpio\":[");
  for (int c = 0; c < N_CH; c++) { if (c) Serial.print(','); Serial.print(26 + c); }
  Serial.println("]}");
}

static void dump_capture(uint32_t t_ms, bool forced) {
  uint32_t start_samp = (t_ms - PRE_MS) * SPMS;          // absolute sample index since restart
  uint32_t n_samp = (uint32_t)(PRE_MS + POST_MS) * SPMS;
  uint32_t frames = n_samp / N_CH;

  capture_no++;
  Serial.print("C,"); Serial.print(capture_no);
  Serial.print(','); Serial.print(frames);
  Serial.print(','); Serial.print(N_CH);
  Serial.print(','); Serial.print((uint32_t)PRE_MS * FPMS);
  Serial.print(','); Serial.print(SPS_TOTAL / N_CH);
  Serial.print(','); Serial.println(forced ? 1 : 0);

  uint32_t idx = start_samp % RING_SAMP;
  uint32_t left = n_samp;
  while (left) {
    uint32_t run = RING_SAMP - idx;
    if (run > left) run = left;
    const uint8_t *p = (const uint8_t *)&ring[idx];
    uint32_t bytes = run * 2;
    while (bytes) {
      uint32_t chunk = bytes > 4096 ? 4096 : bytes;
      Serial.write(p, chunk);
      p += chunk; bytes -= chunk;
    }
    left -= run;
    idx = 0;
  }
  Serial.print("\nE\n");
  Serial.flush();
}

// ---------------- per-ms processing ----------------
static void process_ms(const uint16_t *s, uint32_t ms_index) {
  float mean[N_CH];
  uint32_t sum[N_CH] = {0};
  for (uint32_t i = 0; i < SPMS; i += N_CH)
    for (int c = 0; c < N_CH; c++) sum[c] += s[i + c] & 0x0FFF;
  for (int c = 0; c < N_CH; c++) mean[c] = (float)sum[c] / FPMS;

  // slow stream (skipped if the host is not keeping up)
  if (!pending && Serial && Serial.availableForWrite() > 64) {
    Serial.print("S,"); Serial.print(epoch_ms + ms_index);
    for (int c = 0; c < N_CH; c++) { Serial.print(','); Serial.print(mean[c], 1); }
    Serial.println();
  }

  // drop trigger
  if (!pending && hist_n >= HIST_MS && ms_index >= arm_at_ms) {
    for (int c = 0; c < N_CH; c++) {
      if (c == RAIL_CH) continue;
      float mx = hist[c][0];
      for (int k = 1; k < HIST_MS; k++) if (hist[c][k] > mx) mx = hist[c][k];
      if (mx - mean[c] > trig_drop) {
        pending = true; pending_forced = false; trig_ms = ms_index;
        break;
      }
    }
  }
  for (int c = 0; c < N_CH; c++) hist[c][ms_index % HIST_MS] = mean[c];
  hist_n++;
}

static void handle_serial() {
  static char buf[16];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n' && n < sizeof(buf) - 1) buf[n++] = ch;
    // single-character commands act immediately
    if (n == 1 && (buf[0] == 't' || buf[0] == '?')) {
      if (buf[0] == '?') print_header();
      else if (!pending) {
        uint32_t now_ms = blocks_processed * BLOCK_MS;
        if (now_ms >= PRE_MS) { pending = true; pending_forced = true; trig_ms = now_ms; }
        else Serial.println("#too soon after restart, try again");
      }
      n = 0;
      continue;
    }
    if (ch == '\n') {
      buf[n] = 0;
      if (n >= 2 && buf[0] == 'd') {
        int v = atoi(buf + 1);
        if (v > 0 && v < 4096) { trig_drop = v; Serial.print("#trig_drop="); Serial.println(v); }
      }
      n = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int g = 26; g < 26 + N_CH; g++) adc_gpio_init(g);
  adc_init();

  dma_a = dma_claim_unused_channel(true);
  dma_b = dma_claim_unused_channel(true);
  irq_add_shared_handler(DMA_IRQ_1, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  dma_channel_set_irq1_enabled(dma_a, true);
  dma_channel_set_irq1_enabled(dma_b, true);
  irq_set_enabled(DMA_IRQ_1, true);

  start_acq();
}

void loop() {
  handle_serial();

  uint32_t done = blocks_done;
  if (done - blocks_processed >= NBLK - 2) {
    // Fell too far behind (host blocked us?). Skip ahead; history is now stale.
    blocks_processed = done - 1;
    hist_n = 0;
    Serial.println("#overrun, skipped blocks");
  }
  while (blocks_processed < done) {
    const uint16_t *b = block_ptr(blocks_processed);
    for (uint32_t m = 0; m < BLOCK_MS; m++)
      process_ms(b + m * SPMS, blocks_processed * BLOCK_MS + m);
    blocks_processed++;
  }

  if (pending && blocks_processed * BLOCK_MS >= trig_ms + POST_MS) {
    stop_acq();
    uint32_t ran_ms = blocks_processed * BLOCK_MS;
    dump_capture(trig_ms, pending_forced);
    pending = false;
    epoch_ms += ran_ms;
    arm_at_ms = HOLDOFF_MS;
    start_acq();
  }
}
