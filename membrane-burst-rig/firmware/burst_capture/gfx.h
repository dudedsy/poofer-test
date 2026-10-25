// gfx.h - 128x64 1-bit framebuffer with a 5x7 font. No hardware dependencies,
// so it also builds on a PC for rendering test screens (see tools/render_screens).
#pragma once
#include <stdint.h>

#define GFX_W 128
#define GFX_H 64

struct Gfx {
  uint8_t fb[GFX_W * GFX_H / 8];   // SSD13xx page layout: byte = 8 vertical pixels

  void clear();
  void pixel(int x, int y, bool on = true);
  void hline(int x0, int x1, int y);
  void fill(int x, int y, int w, int h, bool on = true);
  void invert(int x, int y, int w, int h);
  // Draws text with its top-left at (x, y). scale 1 = 6x8 cell, 2 = 12x16, 3 = 18x24.
  // Returns the x just past the last character.
  int text(int x, int y, const char *s, int scale = 1);
  // Text width in pixels at the given scale (last column gap included).
  static int width(const char *s, int scale = 1);
  // Text line n (0..7) at scale 1.
  void line(int n, const char *s) { text(0, n * 8, s, 1); }
  // Right-aligned text ending at x_right.
  void text_right(int x_right, int y, const char *s, int scale = 1);
};
