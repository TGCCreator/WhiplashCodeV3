// =============================================================================
//  42747W WHIPLASH - V5 Brain UI
//  PROS 4 (LVGL 8) + EZ-Template 3.x
//
//  Pages: AUTON | DIAGNOSTICS | SENSORS | PID / ODOM | POSITION
//  Everything you need to edit is in the two marked sections below:
//    1) USER CONFIG   - ports, names, thresholds
//    2) DATA SOURCES  - the only place that touches EZ-Template
// =============================================================================
#include "brainscreen.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "main.h"  // pulls in PROS, LVGL, EZ-Template and your `chassis`

namespace brain_ui {
namespace {

// =============================================================================
//  1) USER CONFIG
// =============================================================================
constexpr const char* TEAM_NUMBER = "42747W";
constexpr const char* TEAM_NAME   = "WHIPLASH";

// Motor temperature thresholds (Celsius). V5 motors start cutting power ~55C.
constexpr float TEMP_WARN_C = 45.0f;
constexpr float TEMP_HOT_C  = 55.0f;

// Battery thresholds (percent)
constexpr float BAT_WARN_PCT = 40.0f;
constexpr float BAT_CRIT_PCT = 20.0f;

// Motors from your main.cpp / subsystems.hpp. Port sign is ignored.
struct MotorCfg { const char* name; int port; };
const MotorCfg MOTORS[] = {
    {"LEFT 20", 20}, {"LEFT 15", 15}, {"LEFT 16", 16},
    {"RIGHT 14", 14}, {"RIGHT 12", 12}, {"RIGHT 18", 18},
    {"4BAR 1", 1}, {"4BAR 10", 10},
    {"CLAW ROT 9", 9},
};

// Rotation sensors (your tracking wheels). Max 4.
struct RotCfg { const char* name; int port; };
const RotCfg ROTATIONS[] = {
    {"VERTICAL", 17},
    {"HORIZONTAL", 9},
};

// Static odometry constants (mirror default_constants() in autons.cpp). Max 4 rows.
// The tracker offsets are read live from the chassis, so they are not listed here.
struct InfoRow { const char* label; const char* value; };
const InfoRow ODOM_INFO[] = {
    {"LOOKAHEAD", "7 in"},
    {"BOOM DIST", "16 in"},
    {"DLEAD",     "0.625"},
    {"TURN BIAS", "0.9"},
};

// =============================================================================
//  2) DATA SOURCES  (EZ-Template calls live ONLY here)
// =============================================================================
int auton_count() { return static_cast<int>(ez::as::auton_selector.Autons.size()); }

std::string first_line(const std::string& s) {
  size_t n = s.find('\n');
  return n == std::string::npos ? s : s.substr(0, n);
}
std::string auton_name(int i) { return first_line(ez::as::auton_selector.Autons[i].Name); }
std::string auton_desc(int i) { return first_line(ez::as::auton_selector.Autons[i].Description); }
int  auton_selected() { return ez::as::auton_selector.current_auton_page; }
// Sets EZ's own selection index, so ez::as::auton_selector.call_selected_auton()
// in autonomous() runs the routine shown on screen.
void auton_select(int i) { ez::as::auton_selector.current_auton_page = i; }

struct Pose { float x, y, theta; };
Pose  get_pose() { return {(float)chassis.odom_x_get(), (float)chassis.odom_y_get(), (float)chassis.odom_theta_get()}; }
float get_imu()  { return (float)chassis.drive_imu_get(); }

constexpr int PID_ROWS = 6;
struct PidRow { const char* name; float kp, ki, kd, si; };
void get_pid_rows(PidRow out[PID_ROWS]) {
  auto d  = chassis.pid_drive_constants_get();
  auto h  = chassis.pid_heading_constants_get();
  auto t  = chassis.pid_turn_constants_get();
  auto s  = chassis.pid_swing_constants_get();
  auto oa = chassis.pid_odom_angular_constants_get();
  auto ob = chassis.pid_odom_boomerang_constants_get();
  out[0] = {"DRIVE",     (float)d.kp,  (float)d.ki,  (float)d.kd,  (float)d.start_i};
  out[1] = {"HEADING",   (float)h.kp,  (float)h.ki,  (float)h.kd,  (float)h.start_i};
  out[2] = {"TURN",      (float)t.kp,  (float)t.ki,  (float)t.kd,  (float)t.start_i};
  out[3] = {"SWING",     (float)s.kp,  (float)s.ki,  (float)s.kd,  (float)s.start_i};
  out[4] = {"ODOM ANG",  (float)oa.kp, (float)oa.ki, (float)oa.kd, (float)oa.start_i};
  out[5] = {"BOOMERANG", (float)ob.kp, (float)ob.ki, (float)ob.kd, (float)ob.start_i};
}

// Live tracking-wheel offsets (distance to center), as set in main.cpp.
std::string tracker_offset(ez::tracking_wheel* t) {
  if (t == nullptr) return "N/A";
  char b[24];
  snprintf(b, sizeof(b), "%.2f in", t->distance_to_center_get());
  return b;
}
std::string odom_left_offset() { return tracker_offset(chassis.odom_tracker_left); }
std::string odom_back_offset() { return tracker_offset(chassis.odom_tracker_back); }

// =============================================================================
//  Theme
// =============================================================================
constexpr uint32_t C_BG     = 0x0B0B0D;
constexpr uint32_t C_HEADER = 0x131316;
constexpr uint32_t C_PANEL  = 0x1B1B20;
constexpr uint32_t C_PANEL2 = 0x26262C;
constexpr uint32_t C_RED    = 0xE10600;
constexpr uint32_t C_RED_DK = 0x6E0A06;
constexpr uint32_t C_WHITE  = 0xF2F2F5;
constexpr uint32_t C_GRAY   = 0x8A8A93;
constexpr uint32_t C_GREEN  = 0x2ECC71;
constexpr uint32_t C_AMBER  = 0xFFB020;

// Fonts, with fallbacks if a size isn't enabled in your PROS lv_conf.
#if LV_FONT_MONTSERRAT_12
#define F_SM (&lv_font_montserrat_12)
#else
#define F_SM LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_16
#define F_MD (&lv_font_montserrat_16)
#else
#define F_MD LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_20
#define F_LG (&lv_font_montserrat_20)
#else
#define F_LG LV_FONT_DEFAULT
#endif
#if LV_FONT_MONTSERRAT_28
#define F_XL (&lv_font_montserrat_28)
#elif LV_FONT_MONTSERRAT_24
#define F_XL (&lv_font_montserrat_24)
#elif LV_FONT_MONTSERRAT_20
#define F_XL (&lv_font_montserrat_20)
#else
#define F_XL LV_FONT_DEFAULT
#endif

// =============================================================================
//  Layout
// =============================================================================
constexpr int HEADER_H  = 30;
constexpr int CONTENT_Y = 36;
constexpr int CONTENT_H = 158;
constexpr int NAV_Y     = 198;
constexpr int PAGE_COUNT = 5;
constexpr float PI_F = 3.14159265f;
constexpr int MAX_MOTORS = 12;
constexpr int MAX_ROT    = 4;

enum Page { PAGE_AUTON, PAGE_DIAG, PAGE_SENSORS, PAGE_PID, PAGE_POS };
const char* PAGE_TITLES[PAGE_COUNT] = {"AUTON", "DIAGNOSTICS", "SENSORS", "PID / ODOM", "POSITION"};

constexpr int N_MOTORS = (int)(sizeof(MOTORS) / sizeof(MOTORS[0])) > MAX_MOTORS ? MAX_MOTORS : (int)(sizeof(MOTORS) / sizeof(MOTORS[0]));
constexpr int N_ROT    = (int)(sizeof(ROTATIONS) / sizeof(ROTATIONS[0])) > MAX_ROT ? MAX_ROT : (int)(sizeof(ROTATIONS) / sizeof(ROTATIONS[0]));
constexpr int N_INFO   = (int)(sizeof(ODOM_INFO) / sizeof(ODOM_INFO[0])) > 4 ? 4 : (int)(sizeof(ODOM_INFO) / sizeof(ODOM_INFO[0]));

lv_obj_t* pages[PAGE_COUNT];
lv_obj_t* dots[PAGE_COUNT];
lv_obj_t* nav_title;
lv_obj_t* hdr_bat_lbl;
lv_obj_t* hdr_bat_bar;
int current_page = 0;

// =============================================================================
//  Widget helpers
// =============================================================================
lv_color_t col(uint32_t hex) { return lv_color_hex(hex); }

lv_obj_t* box(lv_obj_t* parent, int x, int y, int w, int h, uint32_t color, int radius = 6) {
  lv_obj_t* o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, col(color), 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(o, radius, 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
  return o;
}

lv_obj_t* label(lv_obj_t* parent, const char* txt, int x, int y, int w, const lv_font_t* font,
                uint32_t color, lv_text_align_t align = LV_TEXT_ALIGN_LEFT) {
  lv_obj_t* l = lv_label_create(parent);
  lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
  lv_label_set_text(l, txt);
  lv_obj_set_width(l, w);
  lv_obj_set_pos(l, x, y);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, col(color), 0);
  lv_obj_set_style_text_align(l, align, 0);
  return l;
}

void set_color(lv_obj_t* l, uint32_t color) { lv_obj_set_style_text_color(l, col(color), 0); }

lv_obj_t* button(lv_obj_t* parent, const char* txt, int x, int y, int w, int h, lv_event_cb_t cb,
                 intptr_t user) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_remove_style_all(b);
  lv_obj_set_pos(b, x, y);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_bg_color(b, col(C_PANEL2), 0);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(b, 6, 0);
  lv_obj_set_style_border_width(b, 2, 0);
  lv_obj_set_style_border_color(b, col(C_RED), 0);
  lv_obj_set_style_bg_color(b, col(C_RED), LV_STATE_PRESSED);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, (void*)user);
  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, F_MD, 0);
  lv_obj_set_style_text_color(l, col(C_WHITE), 0);
  lv_obj_center(l);
  return b;
}

lv_obj_t* bar(lv_obj_t* parent, int x, int y, int w, int h, uint32_t fill) {
  lv_obj_t* b = lv_bar_create(parent);
  lv_obj_remove_style_all(b);
  lv_obj_set_pos(b, x, y);
  lv_obj_set_size(b, w, h);
  lv_bar_set_range(b, 0, 100);
  lv_obj_set_style_bg_color(b, col(C_PANEL2), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(b, h / 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(b, col(fill), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_radius(b, h / 2, LV_PART_INDICATOR);
  return b;
}

void bar_set(lv_obj_t* b, int v, uint32_t color) {
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  lv_bar_set_value(b, v, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(b, col(color), LV_PART_INDICATOR);
}

lv_obj_t* make_page(lv_obj_t* parent) {
  lv_obj_t* p = lv_obj_create(parent);
  lv_obj_remove_style_all(p);
  lv_obj_set_pos(p, 0, CONTENT_Y);
  lv_obj_set_size(p, 480, CONTENT_H);
  lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
  return p;
}

// =============================================================================
//  Callbacks
// =============================================================================
void on_page_btn(lv_event_t* e) {
  int dir = (int)(intptr_t)lv_event_get_user_data(e);
  set_page((current_page + dir + PAGE_COUNT) % PAGE_COUNT);
}

void on_auton_btn(lv_event_t* e) {
  int dir = (int)(intptr_t)lv_event_get_user_data(e);
  int n = auton_count();
  if (n <= 0) return;
  auton_select((auton_selected() + dir + n) % n);
}

// =============================================================================
//  PAGE 0 : AUTON
// =============================================================================
struct {
  lv_obj_t *prev, *tag, *name, *desc, *next;
  int last_idx = -2, last_n = -2;
} aui;

void build_auton(lv_obj_t* p) {
  button(p, LV_SYMBOL_LEFT, 6, 14, 60, 130, on_auton_btn, -1);
  button(p, LV_SYMBOL_RIGHT, 414, 14, 60, 130, on_auton_btn, +1);
  aui.prev = label(p, "", 74, 8, 332, F_MD, C_GRAY, LV_TEXT_ALIGN_CENTER);
  box(p, 74, 34, 332, 76, C_PANEL);
  box(p, 74, 34, 6, 76, C_RED, 2);
  aui.tag  = label(p, "SELECTED", 92, 40, 300, F_SM, C_RED);
  aui.name = label(p, "", 92, 56, 300, F_XL, C_WHITE);
  aui.desc = label(p, "", 92, 90, 300, F_SM, C_GRAY);
  aui.next = label(p, "", 74, 120, 332, F_MD, C_GRAY, LV_TEXT_ALIGN_CENTER);
}

void update_auton() {
  int n = auton_count(), idx = auton_selected();
  if (n == aui.last_n && idx == aui.last_idx) return;
  aui.last_n = n;
  aui.last_idx = idx;
  if (n <= 0 || idx < 0 || idx >= n) {
    lv_label_set_text(aui.tag, "SELECTED");
    lv_label_set_text(aui.name, "NO AUTONS");
    lv_label_set_text(aui.desc, "");
    lv_label_set_text(aui.prev, "");
    lv_label_set_text(aui.next, "");
    return;
  }
  lv_label_set_text_fmt(aui.tag, "SELECTED   %d / %d", idx + 1, n);
  lv_label_set_text(aui.name, auton_name(idx).c_str());
  lv_label_set_text(aui.desc, auton_desc(idx).c_str());
  if (n > 1) {
    lv_label_set_text(aui.prev, auton_name((idx - 1 + n) % n).c_str());
    lv_label_set_text(aui.next, auton_name((idx + 1) % n).c_str());
  } else {
    lv_label_set_text(aui.prev, "");
    lv_label_set_text(aui.next, "");
  }
}

// =============================================================================
//  PAGE 1 : DIAGNOSTICS
// =============================================================================
struct { lv_obj_t *bar, *pct, *volt; } bui;
struct MotorUI { lv_obj_t *temp, *bar, *stat; } mui[MAX_MOTORS];

void build_diag(lv_obj_t* p) {
  label(p, "BATTERY", 8, 6, 70, F_SM, C_GRAY);
  bui.bar  = bar(p, 72, 8, 170, 12, C_GREEN);
  bui.pct  = label(p, "--%", 252, 3, 60, F_MD, C_WHITE);
  bui.volt = label(p, "--.-V", 312, 6, 60, F_SM, C_GRAY);
  char buf[24];
  snprintf(buf, sizeof(buf), "LIMIT %dC", (int)TEMP_HOT_C);
  label(p, buf, 392, 6, 80, F_SM, C_RED, LV_TEXT_ALIGN_RIGHT);

  int rows = (N_MOTORS + 1) / 2;
  if (rows < 1) rows = 1;
  int rowH = 124 / rows;
  if (rowH > 28) rowH = 28;
  int off = (rowH - 14) / 2;

  for (int i = 0; i < N_MOTORS; i++) {
    int colIdx = i / rows, r = i % rows;
    int x = 8 + colIdx * 236, y = 30 + r * rowH;
    box(p, x, y, 228, rowH - 3, C_PANEL, 4);
    label(p, MOTORS[i].name, x + 6, y + off, 78, F_SM, C_WHITE);
    mui[i].temp = label(p, "--", x + 84, y + off, 40, F_SM, C_WHITE, LV_TEXT_ALIGN_RIGHT);
    mui[i].bar  = bar(p, x + 130, y + (rowH - 3 - 8) / 2, 56, 8, C_GREEN);
    mui[i].stat = label(p, "--", x + 190, y + off, 36, F_SM, C_GRAY);
  }
}

uint32_t temp_color(float t) { return t >= TEMP_HOT_C ? C_RED : (t >= TEMP_WARN_C ? C_AMBER : C_GREEN); }

void update_diag() {
  double cap = pros::battery::get_capacity();
  double mv = pros::battery::get_voltage();
  uint32_t bc = cap < BAT_CRIT_PCT ? C_RED : (cap < BAT_WARN_PCT ? C_AMBER : C_GREEN);
  bar_set(bui.bar, (int)cap, bc);
  lv_label_set_text_fmt(bui.pct, "%d%%", (int)cap);
  set_color(bui.pct, bc);
  char v[16];
  snprintf(v, sizeof(v), "%.1fV", mv / 1000.0);
  lv_label_set_text(bui.volt, v);

  for (int i = 0; i < N_MOTORS; i++) {
    double t = pros::c::motor_get_temperature(std::abs(MOTORS[i].port));
    if (std::isinf(t) || std::isnan(t) || t > 200) {
      lv_label_set_text(mui[i].temp, "---");
      lv_label_set_text(mui[i].stat, "N/C");
      set_color(mui[i].stat, C_RED);
      bar_set(mui[i].bar, 0, C_RED);
      continue;
    }
    uint32_t c = temp_color((float)t);
    lv_label_set_text_fmt(mui[i].temp, "%dC", (int)t);
    lv_label_set_text(mui[i].stat, t >= TEMP_HOT_C ? "HOT" : (t >= TEMP_WARN_C ? "WARM" : "OK"));
    set_color(mui[i].stat, c);
    bar_set(mui[i].bar, (int)(t / TEMP_HOT_C * 100.0f), c);
  }
}

// =============================================================================
//  PAGE 2 : SENSORS (rotation)
// =============================================================================
struct RotUI { lv_obj_t *arc, *ang, *pos, *vel; } rui[MAX_ROT];

void build_sensors(lv_obj_t* p) {
  if (N_ROT == 0) {
    label(p, "NO ROTATION SENSORS CONFIGURED", 0, 70, 480, F_MD, C_GRAY, LV_TEXT_ALIGN_CENTER);
    return;
  }
  int cw = (464 - 8 * (N_ROT - 1)) / N_ROT;
  if (cw > 150) cw = 150;
  int total = N_ROT * cw + 8 * (N_ROT - 1);
  int x0 = (480 - total) / 2;
  for (int i = 0; i < N_ROT; i++) {
    int x = x0 + i * (cw + 8), cx = x + cw / 2;
    box(p, x, 4, cw, 150, C_PANEL);
    label(p, ROTATIONS[i].name, x, 10, cw, F_SM, C_GRAY, LV_TEXT_ALIGN_CENTER);
    lv_obj_t* a = lv_arc_create(p);
    lv_obj_set_size(a, 80, 80);
    lv_obj_set_pos(a, cx - 40, 28);
    lv_arc_set_rotation(a, 270);
    lv_arc_set_bg_angles(a, 0, 360);
    lv_arc_set_range(a, 0, 360);
    lv_obj_remove_style(a, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(a, 7, LV_PART_MAIN);
    lv_obj_set_style_arc_width(a, 7, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(a, col(C_PANEL2), LV_PART_MAIN);
    lv_obj_set_style_arc_color(a, col(C_RED), LV_PART_INDICATOR);
    rui[i].arc = a;
    rui[i].ang = label(p, "--", cx - 35, 58, 70, F_MD, C_WHITE, LV_TEXT_ALIGN_CENTER);
    rui[i].pos = label(p, "--", x, 116, cw, F_SM, C_WHITE, LV_TEXT_ALIGN_CENTER);
    rui[i].vel = label(p, "--", x, 133, cw, F_SM, C_GRAY, LV_TEXT_ALIGN_CENTER);
  }
}

void update_sensors() {
  for (int i = 0; i < N_ROT; i++) {
    int port = std::abs(ROTATIONS[i].port);
    int32_t a = pros::c::rotation_get_angle(port);
    if (a == PROS_ERR || a == INT32_MAX) {
      lv_label_set_text(rui[i].ang, "N/C");
      set_color(rui[i].ang, C_RED);
      lv_label_set_text(rui[i].pos, "NO SIGNAL");
      lv_label_set_text(rui[i].vel, "");
      lv_arc_set_value(rui[i].arc, 0);
      continue;
    }
    float deg = a / 100.0f;
    float revs = pros::c::rotation_get_position(port) / 36000.0f;
    float dps = pros::c::rotation_get_velocity(port) / 100.0f;
    set_color(rui[i].ang, C_WHITE);
    lv_arc_set_value(rui[i].arc, (int)deg);
    char b[24];
    snprintf(b, sizeof(b), "%.1f", deg);
    lv_label_set_text(rui[i].ang, b);
    snprintf(b, sizeof(b), "POS %.2f rev", revs);
    lv_label_set_text(rui[i].pos, b);
    snprintf(b, sizeof(b), "%d dps", (int)dps);
    lv_label_set_text(rui[i].vel, b);
  }
}

// =============================================================================
//  PAGE 3 : PID / ODOM
// =============================================================================
struct PidUI { lv_obj_t *kp, *ki, *kd, *si; } pui[PID_ROWS];
lv_obj_t* ov[6];  // odom value labels: [0]=L tracker offset, [1]=B tracker offset, [2..]=static

void build_pid(lv_obj_t* p) {
  box(p, 8, 4, 304, 152, C_PANEL);
  label(p, "PID CONSTANTS", 14, 8, 120, F_SM, C_RED);
  const int cx[4] = {90, 142, 194, 246};
  const char* hdr[4] = {"kP", "kI", "kD", "I-START"};
  for (int c = 0; c < 4; c++) label(p, hdr[c], cx[c], 8, 56, F_SM, C_GRAY);
  const char* names[PID_ROWS] = {"DRIVE", "HEADING", "TURN", "SWING", "ODOM ANG", "BOOMERANG"};
  for (int r = 0; r < PID_ROWS; r++) {
    int y = 26 + r * 21;
    box(p, 12, y + 20, 296, 1, C_PANEL2, 0);
    label(p, names[r], 14, y + 3, 74, F_SM, C_WHITE);
    pui[r].kp = label(p, "-", cx[0], y, 52, F_MD, C_WHITE);
    pui[r].ki = label(p, "-", cx[1], y, 52, F_MD, C_WHITE);
    pui[r].kd = label(p, "-", cx[2], y, 52, F_MD, C_WHITE);
    pui[r].si = label(p, "-", cx[3], y, 56, F_MD, C_WHITE);
  }

  box(p, 318, 4, 154, 152, C_PANEL);
  label(p, "ODOMETRY", 326, 8, 120, F_SM, C_RED);
  const char* odom_labels[2] = {"L TRK OFF", "B TRK OFF"};
  for (int i = 0; i < 2 + N_INFO; i++) {
    int y = 26 + i * 21;
    const char* lb = i < 2 ? odom_labels[i] : ODOM_INFO[i - 2].label;
    const char* vl = i < 2 ? "--" : ODOM_INFO[i - 2].value;
    label(p, lb, 326, y + 3, 70, F_SM, C_GRAY);
    ov[i] = label(p, vl, 392, y, 74, F_MD, C_WHITE, LV_TEXT_ALIGN_RIGHT);
  }
}

void update_pid() {
  PidRow rows[PID_ROWS];
  get_pid_rows(rows);
  for (int r = 0; r < PID_ROWS; r++) {
    lv_label_set_text_fmt(pui[r].kp, "%.4g", rows[r].kp);
    lv_label_set_text_fmt(pui[r].ki, "%.4g", rows[r].ki);
    lv_label_set_text_fmt(pui[r].kd, "%.4g", rows[r].kd);
    lv_label_set_text_fmt(pui[r].si, "%.4g", rows[r].si);
  }
  lv_label_set_text(ov[0], odom_left_offset().c_str());
  lv_label_set_text(ov[1], odom_back_offset().c_str());
}

// =============================================================================
//  PAGE 4 : POSITION
// =============================================================================
struct { lv_obj_t *x, *y, *th, *imu, *line; } posui;
lv_point_t needle_pts[2];
constexpr int DIAL_CX = 350, DIAL_CY = 79, DIAL_R = 68;

void build_pos(lv_obj_t* p) {
  const char* tags[3] = {"X  (IN)", "Y  (IN)", "HEADING  (DEG)"};
  lv_obj_t** vals[3] = {&posui.x, &posui.y, &posui.th};
  for (int i = 0; i < 3; i++) {
    int y = 4 + i * 52;
    box(p, 8, y, 214, 48, C_PANEL);
    box(p, 8, y, 5, 48, C_RED, 2);
    label(p, tags[i], 20, y + 3, 190, F_SM, C_RED);
    *vals[i] = label(p, "0.00", 20, y + 15, 196, F_XL, C_WHITE);
  }

  lv_obj_t* ring = box(p, DIAL_CX - DIAL_R, DIAL_CY - DIAL_R, DIAL_R * 2, DIAL_R * 2, C_PANEL, LV_RADIUS_CIRCLE);
  lv_obj_set_style_border_width(ring, 3, 0);
  lv_obj_set_style_border_color(ring, col(C_RED_DK), 0);
  label(p, "FWD", DIAL_CX - 20, DIAL_CY - DIAL_R + 6, 40, F_SM, C_GRAY, LV_TEXT_ALIGN_CENTER);

  posui.line = lv_line_create(p);
  lv_obj_set_pos(posui.line, 0, 0);
  lv_obj_set_style_line_width(posui.line, 5, 0);
  lv_obj_set_style_line_color(posui.line, col(C_RED), 0);
  lv_obj_set_style_line_rounded(posui.line, true, 0);
  needle_pts[0] = {(lv_coord_t)DIAL_CX, (lv_coord_t)(DIAL_CY + 12)};
  needle_pts[1] = {(lv_coord_t)DIAL_CX, (lv_coord_t)(DIAL_CY - 50)};
  lv_line_set_points(posui.line, needle_pts, 2);

  box(p, DIAL_CX - 6, DIAL_CY - 6, 12, 12, C_WHITE, LV_RADIUS_CIRCLE);
  posui.imu = label(p, "IMU --", DIAL_CX - 40, DIAL_CY + 30, 80, F_SM, C_GRAY, LV_TEXT_ALIGN_CENTER);
}

float wrap360(float d) {
  d = fmodf(d, 360.0f);
  return d < 0 ? d + 360.0f : d;
}

void update_pos() {
  Pose po = get_pose();
  float th = wrap360(po.theta);
  char b[24];
  snprintf(b, sizeof(b), "%.2f", po.x);
  lv_label_set_text(posui.x, b);
  snprintf(b, sizeof(b), "%.2f", po.y);
  lv_label_set_text(posui.y, b);
  snprintf(b, sizeof(b), "%.1f", th);
  lv_label_set_text(posui.th, b);
  snprintf(b, sizeof(b), "IMU %.1f", wrap360(get_imu()));
  lv_label_set_text(posui.imu, b);

  float r = th * PI_F / 180.0f;
  float s = sinf(r), c = cosf(r);
  needle_pts[0] = {(lv_coord_t)(DIAL_CX - s * 12), (lv_coord_t)(DIAL_CY + c * 12)};
  needle_pts[1] = {(lv_coord_t)(DIAL_CX + s * 50), (lv_coord_t)(DIAL_CY - c * 50)};
  lv_line_set_points(posui.line, needle_pts, 2);
}

// =============================================================================
//  Header + nav + refresh timer
// =============================================================================
void build_header(lv_obj_t* scr) {
  box(scr, 0, 0, 480, HEADER_H, C_HEADER, 0);
  box(scr, 0, HEADER_H, 480, 3, C_RED_DK, 0);
  box(scr, 0, 0, 104, HEADER_H, C_RED, 0);
  label(scr, TEAM_NUMBER, 0, 5, 104, F_MD, C_WHITE, LV_TEXT_ALIGN_CENTER);
  label(scr, TEAM_NAME, 116, 4, 200, F_LG, C_WHITE);
  hdr_bat_lbl = label(scr, "--%", 340, 5, 72, F_MD, C_WHITE, LV_TEXT_ALIGN_RIGHT);
  hdr_bat_bar = bar(scr, 420, 10, 52, 10, C_GREEN);
}

void build_nav(lv_obj_t* scr) {
  button(scr, LV_SYMBOL_LEFT " PREV", 8, NAV_Y + 2, 120, 38, on_page_btn, -1);
  button(scr, "NEXT " LV_SYMBOL_RIGHT, 352, NAV_Y + 2, 120, 38, on_page_btn, +1);
  nav_title = label(scr, PAGE_TITLES[0], 140, NAV_Y + 3, 200, F_MD, C_WHITE, LV_TEXT_ALIGN_CENTER);
  int startX = 240 - (PAGE_COUNT * 8 + (PAGE_COUNT - 1) * 8) / 2;
  for (int i = 0; i < PAGE_COUNT; i++)
    dots[i] = box(scr, startX + i * 16, NAV_Y + 28, 8, 8, C_PANEL2, LV_RADIUS_CIRCLE);
}

void update_header() {
  double cap = pros::battery::get_capacity();
  uint32_t bc = cap < BAT_CRIT_PCT ? C_RED : (cap < BAT_WARN_PCT ? C_AMBER : C_GREEN);
  lv_label_set_text_fmt(hdr_bat_lbl, "%d%%", (int)cap);
  bar_set(hdr_bat_bar, (int)cap, bc);
}

void tick(lv_timer_t*) {
  update_header();
  switch (current_page) {
    case PAGE_AUTON:   update_auton();   break;
    case PAGE_DIAG:    update_diag();    break;
    case PAGE_SENSORS: update_sensors(); break;
    case PAGE_PID:     update_pid();     break;
    case PAGE_POS:     update_pos();     break;
  }
}

}  // namespace

// =============================================================================
//  Public API
// =============================================================================
void set_page(int index) {
  if (index < 0 || index >= PAGE_COUNT) return;
  current_page = index;
  for (int i = 0; i < PAGE_COUNT; i++) {
    if (i == index) lv_obj_clear_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(dots[i], col(i == index ? C_RED : C_PANEL2), 0);
  }
  lv_label_set_text(nav_title, PAGE_TITLES[index]);
  aui.last_idx = -2;  // force auton page redraw when returning to it
  tick(nullptr);
}

void next_page() { set_page((current_page + 1) % PAGE_COUNT); }
void prev_page() { set_page((current_page + PAGE_COUNT - 1) % PAGE_COUNT); }

void init() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_clean(scr);  // removes anything LLEMU / the EZ selector drew
  lv_obj_set_style_bg_color(scr, col(C_BG), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  build_header(scr);
  for (int i = 0; i < PAGE_COUNT; i++) pages[i] = make_page(scr);
  build_auton(pages[PAGE_AUTON]);
  build_diag(pages[PAGE_DIAG]);
  build_sensors(pages[PAGE_SENSORS]);
  build_pid(pages[PAGE_PID]);
  build_pos(pages[PAGE_POS]);
  build_nav(scr);

  lv_timer_create(tick, 100, nullptr);  // runs inside LVGL's own task: thread-safe
  set_page(0);
}

}  // namespace brain_ui
