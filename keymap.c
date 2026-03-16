/* Copyright 2019 Thomas Baart <thomas@splitkb.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H
#include "raw_hid.h"
#include "rossmeyerza.h"

enum layers {
    _QWERTY = 0,
    _DVORAK,
    _NAV,
    _SYM,
    _FUNCTION,
    _ADJUST
};

enum custom_keycodes { QMKBEST = SAFE_RANGE, QMKPPM, QMKALT };

// Aliases for readability
#define QWERTY DF(_QWERTY)
#define DVORAK DF(_DVORAK)

#define SYM MO(_SYM)
#define NAV MO(_NAV)
#define FKEYS MO(_FUNCTION)
#define ADJUST MO(_ADJUST)

#define CTL_ESC MT(MOD_LCTL, KC_ESC)
#define CTL_QUOT MT(MOD_RCTL, KC_QUOTE)
#define CTL_MINS MT(MOD_RCTL, KC_MINUS)
#define ALT_ENT MT(MOD_LALT, KC_ENT)

// Note: LAlt/Enter (ALT_ENT) is not the same thing as the keyboard shortcut Alt+Enter.
// The notation `mod/tap` denotes a key that activates the modifier `mod` when held down, and
// produces the key `tap` when tapped (i.e. pressed and released).

// Raw HID protocol definitions
#define MSG_TYPE_STATS     0x01
#define MSG_TYPE_HEARTBEAT 0x02
#define HID_TIMEOUT        5000  // ms before considering HID data stale

// Raw HID system stats storage
static uint8_t  sys_cpu_percent = 0;
static uint8_t  sys_ram_percent = 0;
static uint8_t  sys_cpu_temp    = 0;
static uint8_t  sys_gpu_percent = 0xFF;  // 0xFF = unavailable
static uint8_t  sys_gpu_temp    = 0;
static uint32_t last_hid_update = 0;

// Sparkline history buffers (88 samples each, ~3 min at 2s intervals)
#define GRAPH_WIDTH   88
#define GRAPH_X_START 40  // pixels reserved for text labels on left
#define GRAPH_HEIGHT  14  // pixels tall per sparkline row
#define NUM_ROWS      4   // CPU, RAM, GPU, WPM

static uint8_t hist_cpu[GRAPH_WIDTH] = {0};
static uint8_t hist_ram[GRAPH_WIDTH] = {0};
static uint8_t hist_gpu[GRAPH_WIDTH] = {0};
static uint8_t hist_wpm[GRAPH_WIDTH] = {0};
static uint8_t hist_idx = 0;
static bool    hist_full = false;

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/*
 * Base Layer: QWERTY
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |  Tab   |   Q  |   W  |   E  |   R  |   T  |                              |   Y  |   U  |   I  |   O  |   P  |  Bksp  |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |Ctrl/Esc|   A  |   S  |   D  |   F  |   G  |                              |   H  |   J  |   K  |   L  | ;  : |Ctrl/' "|
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * | LShift |   Z  |   X  |   C  |   V  |   B  | [ {  |CapsLk|  |F-keys|  ] } |   N  |   M  | ,  < | . >  | /  ? | RShift |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |Adjust| LGUI | LAlt/| Space| Nav  |  | Sym  | Space| AltGr| RGUI | Menu |
 *                        |      |      | Enter|      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
    [_QWERTY] = LAYOUT(
     KC_TAB  , KC_Q ,  KC_W   ,  KC_E  ,   KC_R ,   KC_T ,                                        KC_Y,   KC_U ,  KC_I ,   KC_O ,  KC_P , KC_BSPC,
     CTL_ESC , KC_A ,  KC_S   ,  KC_D  ,   KC_F ,   KC_G ,                                        KC_H,   KC_J ,  KC_K ,   KC_L ,KC_SCLN,CTL_QUOT,
     KC_LSFT , KC_Z ,  KC_X   ,  KC_C  ,   KC_V ,   KC_B , KC_LBRC,KC_CAPS,     FKEYS  , KC_RBRC, KC_N,   KC_M ,KC_COMM, KC_DOT ,KC_SLSH, KC_RSFT,
                                ADJUST , KC_LGUI, ALT_ENT, FKEYS , NAV   ,      SYM    , KC_SPC , ADJUST, KC_LALT, KC_APP
    ),

/*
 * Base Layer: Dvorak
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |  Tab   | ' "  | , <  | . >  |   P  |   Y  |                              |   F  |   G  |   C  |   R  |   L  |  Bksp  |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |Ctrl/Esc|   A  |   O  |   E  |   U  |   I  |                              |   D  |   H  |   T  |   N  |   S  |Ctrl/- _|
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * | LShift | ; :  |   Q  |   J  |   K  |   X  | [ {  |CapsLk|  |F-keys|  ] } |   B  |   M  |   W  |   V  |   Z  | RShift |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |Adjust| LGUI | LAlt/| Space| Nav  |  | Sym  | Space| AltGr| RGUI | Menu |
 *                        |      |      | Enter|      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
    [_DVORAK] = LAYOUT(
     KC_TAB  ,KC_QUOTE,KC_COMM,  KC_DOT,   KC_P ,   KC_Y ,                                        KC_F,   KC_G ,  KC_C ,   KC_R ,  KC_L , KC_BSPC,
     CTL_ESC , KC_A ,  KC_O   ,  KC_E  ,   KC_U ,   KC_I ,                                        KC_D,   KC_H ,  KC_T ,   KC_N ,  KC_S , CTL_MINS,
    KC_LSFT ,KC_SCLN, KC_Q   ,  KC_J  ,   KC_K ,   KC_X , KC_LBRC,KC_CAPS,     FKEYS  , KC_RBRC, KC_B,   KC_M ,  KC_W ,   KC_V ,  KC_Z , KC_RSFT,
                                 ADJUST, KC_LGUI, ALT_ENT, FKEYS , NAV   ,     SYM    , KC_SPC ,KC_RALT, KC_RGUI, KC_APP
    ),

/*
 * Nav Layer: Media, navigation
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |        |      |      |      |      |      |                              | PgUp | Home |   ↑  | End  | VolUp| Delete |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |        |  GUI |  Alt | Ctrl | Shift|      |                              | PgDn |  ←   |   ↓  |   →  | VolDn| Insert |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |        |      |      |      |      |      |      |ScLck |  |      |      | Pause|M Prev|M Play|M Next|VolMut| PrtSc  |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
    [_NAV] = LAYOUT(
      _______, _______, _______, _______, _______, _______,                                     KC_PGUP, KC_HOME, KC_UP,   KC_END,  KC_VOLU, KC_DEL,
      _______, KC_LGUI, KC_LALT, KC_LCTL, KC_LSFT, _______,                                     KC_PGDN, KC_LEFT, KC_DOWN, KC_RGHT, KC_VOLD, KC_INS,
      _______, _______, _______, _______, _______, _______, _______, KC_SCRL, _______, _______,KC_PAUSE, KC_MPRV, KC_MPLY, KC_MNXT, KC_MUTE, KC_PSCR,
                                 _______, _______, _______, _______, _______, _______, _______, LCA(KC_LEFT), LCA(KC_RIGHT), _______
    ),

/*
 * Sym Layer: Numbers and symbols
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |    `   |  1   |  2   |  3   |  4   |  5   |                              |   6  |  7   |  8   |  9   |  0   |   =    |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |    ~   |  !   |  @   |  #   |  $   |  %   |                              |   ^  |  &   |  *   |  (   |  )   |   +    |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |    |   |   \  |  :   |  ;   |  -   |  [   |  {   |      |  |      |   }  |   ]  |  _   |  ,   |  .   |  /   |   ?    |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
    [_SYM] = LAYOUT(
      KC_GRV ,   KC_1 ,   KC_2 ,   KC_3 ,   KC_4 ,   KC_5 ,                                       KC_6 ,   KC_7 ,   KC_8 ,   KC_9 ,   KC_0 , KC_EQL ,
     KC_TILD , KC_EXLM,  KC_AT , KC_HASH,  KC_DLR, KC_PERC,                                     KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_PLUS,
     KC_PIPE , KC_BSLS, KC_COLN, KC_SCLN, KC_MINS, KC_LBRC, KC_LCBR, _______, _______, _______, KC_RBRC, KC_UNDS, KC_COMM,  KC_DOT, KC_SLSH, KC_QUES,
                                 _______, _______, _______, _______, _______, _______, KC_RCBR, _______, _______, _______
    ),

/*
 * Function Layer: Function keys
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |        |  F9  | F10  | F11  | F12  |      |                              |      |      |      |      |      |        |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |        |  F5  |  F6  |  F7  |  F8  |      |                              |      | Shift| Ctrl |  Alt |  GUI |        |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |        |  F1  |  F2  |  F3  |  F4  |      |      |      |  |      |      |      |      |      |      |      |        |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
    [_FUNCTION] = LAYOUT(
      _______,  KC_F9 ,  KC_F10,  KC_F11,  KC_F12, _______,                                     _______, KC_KP_7, KC_KP_8, KC_KP_9, _______, _______,
      _______,  KC_F5 ,  KC_F6 ,  KC_F7 ,  KC_F8 , _______,                                     _______, KC_KP_4, KC_KP_5, KC_KP_6, _______, _______,
      _______,  KC_F1 ,  KC_F2 ,  KC_F3 ,  KC_F4 , _______, _______, _______, _______, _______, _______, KC_KP_1, KC_KP_2, KC_KP_3, _______, _______,
                                 _______, _______, _______, _______, _______, _______, _______, KC_NUM , KC_KP_0, _______
    ),

/*
 * Adjust Layer: Default layer settings, RGB, mouse, password macros
 *
 * ,-------------------------------------------.                              ,-------------------------------------------.
 * |        | WhUp |      | MsUp |      | WhLft|                              |      |      |      |AltPwd| Pwd  | LPwd   |
 * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
 * |        | WhDn | MsLft| MsDn |MsRght| WhRgt|                              | TOG  | SAI  | HUI  | VAI  | MOD  |        |
 * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
 * |        |      |      |      |      |      |QWERTY|DVORAK|  |      |      |      | SAD  | HUD  | VAD  | RMOD |        |
 * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
 *                        |      |      | Btn1 | Btn2 |      |  |      |      |      |      |      |
 *                        |      |      |      |      |      |  |      |      |      |      |      |
 *                        `----------------------------------'  `----------------------------------'
 */
    [_ADJUST] = LAYOUT(
      _______, MS_WHLU, _______, MS_UP  , _______, MS_WHLL,                                    _______, _______, _______, QMKALT,  QMKBEST, QMKPPM,
      _______, MS_WHLD, MS_LEFT, MS_DOWN, MS_RGHT, MS_WHLR,                                    UG_TOGG, UG_SATU, UG_HUEU, UG_VALU,  UG_NEXT, _______,
      _______, _______, _______, _______, _______, _______, QWERTY, DVORAK, _______, _______, _______, UG_SATD, UG_HUED, UG_VALD, UG_PREV, _______,
                                 _______, _______, MS_BTN1,MS_BTN2, _______, _______, _______, _______, _______, _______
    ),

// /*
//  * Layer template
//  *
//  * ,-------------------------------------------.                              ,-------------------------------------------.
//  * |        |      |      |      |      |      |                              |      |      |      |      |      |        |
//  * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
//  * |        |      |      |      |      |      |                              |      |      |      |      |      |        |
//  * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
//  * |        |      |      |      |      |      |      |      |  |      |      |      |      |      |      |      |        |
//  * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
//  *                        |      |      |      |      |      |  |      |      |      |      |      |
//  *                        |      |      |      |      |      |  |      |      |      |      |      |
//  *                        `----------------------------------'  `----------------------------------'
//  */
//     [_LAYERINDEX] = LAYOUT(
//       _______, _______, _______, _______, _______, _______,                                     _______, _______, _______, _______, _______, _______,
//       _______, _______, _______, _______, _______, _______,                                     _______, _______, _______, _______, _______, _______,
//       _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
//                                  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
//     ),
};

/* The default OLED and rotary encoder code can be found at the bottom of qmk_firmware/keyboards/splitkb/kyria/rev1/rev1.c
 * These default settings can be overriden by your own settings in your keymap.c
 * For your convenience, here's a copy of those settings so that you can uncomment them if you wish to apply your own modifications.
 * DO NOT edit the rev1.c file; instead override the weakly defined default functions by your own.
 */

#ifdef OLED_ENABLE
oled_rotation_t oled_init_user(oled_rotation_t rotation) {
    return OLED_ROTATION_180;
}

char wpm_str[10];

static void render_my_logo(void) {
  static const char PROGMEM my_logo[] = {
// 'WT_Logo_Black_Positive_RGB', 128x58px
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xc0, 0xfc, 0xfc,
0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x0f,
0x00, 0x00, 0x00, 0x00, 0x70, 0x80, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00,
0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x70, 0xc0, 0x00,
0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x30, 0x60, 0x80, 0x00,
0x00, 0x00, 0x00, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x10, 0x10,
0x10, 0x10, 0x20, 0xc0, 0x00, 0x00, 0x00, 0xf0, 0xf0, 0xc0, 0x00, 0x00, 0x80, 0xc0, 0xf0, 0xf0,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x70, 0xf0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
0x60, 0x80, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x78, 0x1f, 0x01, 0x00, 0x3f, 0x70, 0x1f, 0x00, 0x00, 0x00,
0x00, 0x1f, 0x60, 0x40, 0x40, 0x40, 0x40, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x03,
0x0e, 0x18, 0x78, 0x7f, 0x00, 0x00, 0x00, 0x7f, 0x40, 0x40, 0x40, 0x40, 0x60, 0x30, 0x0f, 0x00,
0x00, 0x00, 0x00, 0x7f, 0x42, 0x42, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x04, 0x04,
0x04, 0x0e, 0x3b, 0x41, 0x00, 0x00, 0x00, 0x7f, 0x7f, 0x00, 0x03, 0x06, 0x03, 0x00, 0x7f, 0x7f,
0x00, 0x00, 0x00, 0x60, 0x1e, 0x0b, 0x08, 0x08, 0x0b, 0x3c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x7f,
0x00, 0x01, 0x06, 0x0c, 0x30, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0xf0, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00,
0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x30, 0x10,
0x10, 0x30, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xe0, 0x80, 0x00, 0x00, 0x80, 0xe0, 0xf0, 0x00,
0x00, 0x00, 0x00, 0xf0, 0x10, 0x10, 0x10, 0x30, 0x60, 0xc0, 0x00, 0x00, 0x00, 0xe0, 0x30, 0x10,
0x10, 0x10, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xc0, 0x00,
0x00, 0x00, 0x00, 0xe0, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x7f, 0x02, 0x02, 0x02, 0x02, 0x02, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x60, 0x40, 0x40,
0x40, 0x40, 0x30, 0x0f, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x01, 0x06, 0x06, 0x01, 0x00, 0x7f, 0x00,
0x00, 0x00, 0x00, 0x7f, 0x04, 0x04, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x60, 0x41, 0x42,
0x42, 0x42, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x60, 0x40, 0x40, 0x40, 0x60, 0x3f, 0x00,
0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x03, 0x0c, 0x18, 0x78, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  oled_write_raw_P(my_logo, sizeof(my_logo));
}

// Push a new sample into a history buffer (circular)
static void hist_push(uint8_t *buf, uint8_t val) {
    buf[hist_idx] = val;
}

// Draw a sparkline for one row
// y_start: top pixel of this row's graph area
static void draw_sparkline(uint8_t *hist, uint8_t y_start) {
    uint8_t count = hist_full ? GRAPH_WIDTH : hist_idx;
    for (uint8_t i = 0; i < GRAPH_WIDTH; i++) {
        uint8_t x = GRAPH_X_START + i;
        // Read from oldest to newest in the circular buffer
        uint8_t idx;
        if (hist_full) {
            idx = (hist_idx + i) % GRAPH_WIDTH;
        } else {
            if (i >= count) break;
            idx = i;
        }
        // Scale value (0-100) to graph height (0 to GRAPH_HEIGHT-1)
        uint8_t bar_h = (uint16_t)hist[idx] * (GRAPH_HEIGHT - 1) / 100;
        // Draw filled column from bottom up
        for (uint8_t py = 0; py < GRAPH_HEIGHT; py++) {
            uint8_t y = y_start + (GRAPH_HEIGHT - 1) - py;
            oled_write_pixel(x, y, py <= bar_h);
        }
    }
}

// Write a label like "CPU:012%" at a text cursor position
static void write_label(uint8_t col, uint8_t row, const char *name, uint8_t val) {
    char buf[9];
    buf[0] = name[0]; buf[1] = name[1]; buf[2] = name[2];
    buf[3] = '0' + val / 100;
    buf[4] = '0' + (val / 10) % 10;
    buf[5] = '0' + val % 10;
    buf[6] = '%';
    buf[7] = '\0';
    oled_set_cursor(col, row);
    oled_write(buf, false);
}

static void render_sys_stats(void) {
    // Record new samples
    hist_push(hist_cpu, sys_cpu_percent);
    hist_push(hist_ram, sys_ram_percent);
    hist_push(hist_gpu, sys_gpu_percent == 0xFF ? 0 : sys_gpu_percent);
    hist_push(hist_wpm, get_current_wpm() > 100 ? 100 : get_current_wpm());

    // Advance circular buffer index
    hist_idx++;
    if (hist_idx >= GRAPH_WIDTH) {
        hist_idx = 0;
        hist_full = true;
    }

    oled_clear();

    // Row 0: CPU  (y: 0-13,  text row 0)
    write_label(0, 0, "CPU", sys_cpu_percent);
    draw_sparkline(hist_cpu, 2);  // offset 2px to not overlap text top

    // Row 1: RAM  (y: 16-29, text row 2)
    write_label(0, 2, "RAM", sys_ram_percent);
    draw_sparkline(hist_ram, 18);

    // Row 2: GPU  (y: 32-45, text row 4)
    if (sys_gpu_percent != 0xFF) {
        write_label(0, 4, "GPU", sys_gpu_percent);
    } else {
        oled_set_cursor(0, 4);
        oled_write("GPU N/A", false);
    }
    draw_sparkline(hist_gpu, 34);

    // Row 3: WPM  (y: 48-61, text row 6)
    uint8_t wpm = get_current_wpm();
    write_label(0, 6, "WPM", wpm > 100 ? 100 : wpm);
    draw_sparkline(hist_wpm, 50);
}

bool wpm_keycode_user(uint16_t keycode) {
    return true;
}

bool oled_task_user(void) {
    if (is_keyboard_master()) {
        // Show system stats when HID data is fresh, otherwise show logo + WPM
        if (last_hid_update > 0 && timer_elapsed32(last_hid_update) < HID_TIMEOUT) {
            render_sys_stats();
        } else {
            render_my_logo();
            oled_set_cursor(0, 7);
            uint8_t n = get_current_wpm();
            wpm_str[3] = '\0';
            wpm_str[2] = '0' + n % 10;
            wpm_str[1] = '0' + (n /= 10) % 10;
            wpm_str[0] = '0' + n / 10;
            oled_write_P(PSTR("WPM: "), false);
            oled_write(wpm_str, false);
        }
    } else {
        // QMK Logo and version information
        // clang-format off
        static const char PROGMEM qmk_logo[] = {
            0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,
            0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,
            0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,0xd2,0xd3,0xd4,0};
        // clang-format on

        oled_write_P(qmk_logo, false);
        oled_write_P(PSTR("Kyria rev2.0\n\n"), false);

        // Host Keyboard Layer Status
        oled_write_P(PSTR("Layer: "), false);
        switch (get_highest_layer(layer_state | default_layer_state)) {
            case _QWERTY:
                oled_write_P(PSTR("QWERTY\n"), false);
                break;
            case _DVORAK:
                oled_write_P(PSTR("Dvorak\n"), false);
                break;
            case _NAV:
                oled_write_P(PSTR("Nav\n"), false);
                break;
            case _SYM:
                oled_write_P(PSTR("Sym\n"), false);
                break;
            case _FUNCTION:
                oled_write_P(PSTR("Fun/num\n"), false);
                break;
            case _ADJUST:
                oled_write_P(PSTR("Adjust\n"), false);
                break;
            default:
                oled_write_P(PSTR("Undefined\n"), false);
        }

        // Write host Keyboard LED Status to OLEDs
        led_t led_usb_state = host_keyboard_led_state();
        oled_write_P(led_usb_state.num_lock ? PSTR("NUMLCK ") : PSTR("       "), false);
        oled_write_P(led_usb_state.caps_lock ? PSTR("CAPLCK ") : PSTR("       "), false);
        oled_write_P(led_usb_state.scroll_lock ? PSTR("SCRLCK ") : PSTR("       "), false);
    }
    return false;
}
#endif

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case QMKBEST:
            if (record->event.pressed) {
                SEND_STRING(PASSWORD);
            }
            break;
        case QMKPPM:
            if (record->event.pressed) {
                SEND_STRING(L_PASSWORD);
            }
            break;
        case QMKALT:
            if (record->event.pressed) {
                SEND_STRING(ALT_PASSWORD);
            }
            break;
    }
    return true;
}

void keyboard_post_init_user(void) {
}

void raw_hid_receive(uint8_t *data, uint8_t length) {
    if (length < 1) return;

    uint8_t msg_type = data[0];

    switch (msg_type) {
        case MSG_TYPE_STATS:
            if (length >= 7) {
                sys_cpu_percent = data[1];
                sys_ram_percent = data[2];
                sys_cpu_temp    = data[3];
                // data[4] reserved
                sys_gpu_percent = data[5];
                sys_gpu_temp    = data[6];
#ifdef OLED_ENABLE
                oled_on();  // wake OLED when receiving HID stats
#endif
                last_hid_update = timer_read32();
            }
            break;
        case MSG_TYPE_HEARTBEAT:
            raw_hid_send(data, length);
            break;
    }
}
