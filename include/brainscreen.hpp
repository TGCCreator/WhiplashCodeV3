#pragma once

// 42747W WHIPLASH - V5 Brain UI (PROS 4 / LVGL 8 / EZ-Template 3.x)
namespace brain_ui {

// Call once in initialize(), in place of ez::as::initialize().
void init();

// Optional: switch pages from code (e.g. a controller button).
void next_page();
void prev_page();
void set_page(int index);

}  // namespace brain_ui
