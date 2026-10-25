// render_screens.cpp - draws every firmware screen on a PC and writes PBM images,
// so layout can be checked without hardware.
//   g++ -O1 -I ../firmware/burst_capture render_screens.cpp \
//       ../firmware/burst_capture/gfx.cpp ../firmware/burst_capture/screens.cpp -o render
//   ./render outdir
#include "gfx.h"
#include "screens.h"
#include <stdio.h>
#include <string>

static void save(const Gfx &g, const std::string &path) {
  FILE *f = fopen(path.c_str(), "wb");
  fprintf(f, "P1\n%d %d\n", GFX_W, GFX_H);
  for (int y = 0; y < GFX_H; y++) {
    for (int x = 0; x < GFX_W; x++) fputs((g.fb[(y >> 3) * GFX_W + x] >> (y & 7)) & 1 ? "1 " : "0 ", f);
    fputc('\n', f);
  }
  fclose(f);
}

int main(int argc, char **argv) {
  std::string out = argc > 1 ? argv[1] : ".";
  Gfx g;

  MenuView m{};
  m.title = "MENU   A 0.1 B -0.2";
  const char *items[] = {"Collect", "Live (USB)", "Dump (USB)", "Channels        2", "Tag             1",
                         "Auto arm     5 psi", "Drop trig  1.0 psi", "Zero sensors", "Captures   12/34",
                         "Delete all"};
  m.n_items = 10;
  for (int i = 0; i < m.n_items; i++) m.items[i] = items[i];
  m.sel = 0; draw_menu(g, m); save(g, out + "/menu_top.pbm");
  m.sel = 5; m.editing = true; draw_menu(g, m); save(g, out + "/menu_edit.pbm");
  m.sel = 9; m.editing = false; draw_menu(g, m); save(g, out + "/menu_bottom.pbm");

  CollectView c{};
  c.p.n_ch = 1; c.p.psi[0] = 0.2f; c.p.valid = true;
  c.state = CS_WAIT; c.arm_psi = 5; c.free_slots = 27; c.tag = 1; c.storage_ok = true;
  draw_collect(g, c); save(g, out + "/collect_wait_1ch.pbm");
  c.p.n_ch = 2; c.p.psi[0] = 42.1f; c.p.psi[1] = 41.8f;
  c.state = CS_ARMED; c.peak_psi = 42.3f;
  draw_collect(g, c); save(g, out + "/collect_armed_2ch.pbm");
  c.p.n_ch = 1; c.p.psi[0] = 100.4f; c.manual_armed = true;
  draw_collect(g, c); save(g, out + "/collect_armed_1ch_manual.pbm");
  c.p.n_ch = 3; c.p.psi[2] = -0.3f; c.state = CS_PENDING;
  draw_collect(g, c); save(g, out + "/collect_pending_3ch.pbm");

  PromptView p{};
  p.id = 13; p.tag = 1; p.n_ch = 2; p.p0_psi[0] = 42.1f; p.p0_psi[1] = 41.9f;
  p.end_psi[0] = 0.2f; p.end_psi[1] = -0.1f; p.free_slots = 27; p.storage_ok = true;
  draw_prompt(g, p); save(g, out + "/prompt.pbm");
  p.free_slots = 0; p.err_samples = 3; p.forced = true;
  draw_prompt(g, p); save(g, out + "/prompt_full.pbm");

  LiveView l{};
  l.p.n_ch = 2; l.p.psi[0] = 12.3f; l.p.psi[1] = 12.1f; l.p.valid = true; l.sent = 4; l.trig_psi = 1.0f; l.host = true;
  draw_live(g, l); save(g, out + "/live.pbm");

  const char *msg[] = {"DUMP (USB)", "stored 12  free 22", "", "last: sent #0012", "", "", "", "hold SET: menu"};
  draw_message(g, msg, 8); save(g, out + "/dump.pbm");
  return 0;
}
