/* Copyright 2015-2021 Jack Humbert
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
#include "muse.h"

enum planck_layers {
  _COLEMAK,
  _COLEMAK_NO,
  _COLEMAK_LSFT,
  _COLEMAK_RSFT,
  _BRK,
  _SYM,
  _NAV,
  _NUM,
  _FUNC,
  _ADJUST,
};

enum planck_keycodes {
  COLEMAK = SAFE_RANGE,
  BACKLIT,
};

enum tap_dances {
  _TD_LPRN,
};

enum unicode_names {
  U_SMILE,
};

const uint32_t PROGMEM unicode_map[] = {
  [U_SMILE]   = 0x1f642,  // :-)
};

/* On the mac, this requires the input mode to be switched to Unicode Hex Input,
   which breaks the RALT keycodes for special characters below, as well as not
   working in Emacs. So for now this is just for reference.
*/
#define K_SMILE X(U_SMILE)

/* Maps special symbols to corresponding mac RALT keycodes. */
#define K_AE    RALT(KC_QUOT) // æ
#define K_OSTR  RALT(KC_O)    // ø
#define K_ARNG  RALT(KC_A)    // å
#define K_EMDSH RALT(KC_MINS) // –

#define TGL_NO TG(_COLEMAK_NO)
#define TGL_ADJ TG(_ADJUST)

// Left-hand home row mods
#define HM_A LALT_T(KC_A)
#define HM_R LGUI_T(KC_R)
#define HM_S LT(_COLEMAK_RSFT, KC_S)
#define HM_T LCTL_T(KC_T)

// Right-hand home row mods
#define HM_N RCTL_T(KC_N)
#define HM_E LT(_COLEMAK_LSFT, KC_E)
#define HM_I LGUI_T(KC_I)
#define HM_O RALT_T(KC_O)

// Thumb keys with layer switches
#define HM_BRK LT(_BRK, KC_SPC)
#define HM_NAV LT(_NAV, KC_TAB)
#define HM_NUM LT(_NUM, KC_ESC)
#define HM_SYM LT(_SYM, KC_ENT)

// Shifted keys
#define SK_TAB S(KC_TAB)

// Tap dances
#define TD_LPRN TD(_TD_LPRN)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_COLEMAK] = LAYOUT_planck_grid(
    _______, KC_W,    KC_F,    KC_P,    KC_B,    KC_BRID, KC_BRIU, KC_J,    KC_L,    KC_U,    KC_Y,    _______,
    KC_Q,    HM_R,    HM_S,    HM_T,    KC_G,    KC_VOLD, KC_VOLU, KC_M,    HM_N,    HM_E,    HM_I,    KC_SCLN,
    HM_A,    KC_X,    KC_C,    KC_D,    KC_V,    _______, _______, KC_K,    KC_H,    KC_COMM, KC_DOT,  HM_O,
    KC_Z,    KC_LEFT, KC_RGHT, TGL_NO,  HM_BRK,  HM_NAV,  HM_NUM,  HM_SYM,  KC_CAPS, KC_DOWN, KC_UP,   KC_SLSH
),

[_COLEMAK_RSFT] = LAYOUT_planck_grid(
    _______, _______, _______, _______, _______, _______, _______, S(KC_J), S(KC_L), S(KC_U), S(KC_Y), _______,
    _______, KC_LGUI, _______, KC_LCTL, _______, _______, _______, S(KC_M), S(KC_N), S(KC_E), S(KC_I), KC_COLN,
    KC_LALT, _______, _______, _______, _______, _______, _______, S(KC_K), S(KC_H), KC_LT,   KC_GT,   S(KC_O),
    _______, _______, _______, _______, KC_DEL,  _______, _______, KC_UNDS, _______, _______, _______, KC_QUES
),

[_COLEMAK_LSFT] = LAYOUT_planck_grid(
    _______, S(KC_W), S(KC_F), S(KC_P), S(KC_B), _______, _______, _______, _______, _______, _______, _______,
    S(KC_Q), S(KC_R), S(KC_S), S(KC_T), S(KC_G), _______, _______, _______, KC_RCTL, _______, KC_LGUI, _______,
    S(KC_A), S(KC_X), S(KC_C), S(KC_D), S(KC_V), _______, _______, _______, _______, _______, _______, KC_RALT,
    S(KC_Z), KC_LEFT, KC_RGHT, KC_UNDS, KC_BSPC, SK_TAB,  _______, _______, _______, _______, _______, _______
),

[_COLEMAK_NO] = LAYOUT_planck_grid(
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
    _______, K_OSTR,  K_ARNG,  _______, _______, _______, _______, _______, _______, _______, _______, _______,
    K_AE,    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
),

[_BRK] = LAYOUT_planck_grid(
    _______, _______, _______, _______, _______, _______, _______, _______, KC_QUOT, KC_DQUO, KC_GRV,  _______,
    _______, KC_LGUI, KC_LSFT, KC_LCTL, _______, _______, _______, _______, TD_LPRN, KC_RPRN, KC_LCBR, _______,
    KC_LALT, _______, _______, _______, _______, _______, _______, _______, KC_LBRC, KC_RBRC, _______, KC_RCBR,
    _______, _______, _______, _______, _______, _______, _______, KC_UNDS, _______, _______, _______, _______
),

[_NAV] = LAYOUT_planck_grid(
    _______, _______, _______, _______, _______, _______, _______, KC_FIND, KC_COPY, _______, _______, _______,
    _______, KC_LGUI, KC_LSFT, KC_LCTL, _______, _______, _______, KC_CAPS, KC_LEFT, KC_DOWN, KC_UP,   _______,
    KC_LALT, _______, _______, _______, _______, _______, _______, _______, KC_HOME, KC_PGDN, KC_PGUP, KC_RGHT,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_END
),

[_NUM] = LAYOUT_planck_grid(
    _______, KC_6,    KC_5,    KC_4,    KC_A,    _______, _______, KC_D,    _______, _______, _______, _______,
    KC_X,    KC_2,    KC_1,    KC_0,    KC_B,    _______, _______, KC_E,    KC_RCTL, KC_RSFT, KC_LGUI, _______,
    KC_3,    KC_9,    KC_8,    KC_7,    KC_C,    _______, _______, KC_F,    _______, _______, _______, KC_RALT,
    _______, _______, _______, _______, KC_DOT,  _______, _______, _______, _______, _______, _______, _______
),

[_SYM] = LAYOUT_planck_grid(
    TGL_ADJ, KC_PERC, KC_DLR,  KC_TILD,  _______, _______, _______, _______, _______, _______, _______, _______,
    KC_PIPE, KC_MINS, KC_PLUS, KC_ASTR, KC_CIRC, _______, _______, _______, KC_RCTL, KC_RSFT, KC_LGUI, _______,
    KC_EXLM, KC_HASH, KC_AT,   KC_AMPR, _______, _______, _______, _______, _______, _______, _______, KC_RALT,
    KC_BSLS, _______, _______, _______, KC_EQL,  _______, _______, _______, _______, _______, _______, _______
),

[_FUNC] = LAYOUT_planck_grid(
    TGL_ADJ, KC_F6,   KC_F5,   KC_F4,   _______, _______, _______, _______, _______, _______, _______, _______,
    KC_F11,  KC_F2,   KC_F1,   KC_F10,  _______, _______, _______, _______, KC_RCTL, KC_RSFT, KC_LGUI, _______,
    KC_F3,   KC_F9,   KC_F8,   KC_F7,   _______, _______, _______, _______, _______, _______, _______, KC_RALT,
    KC_F12,  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
),

/* Adjust (Sym + Num)
 *                      v------------------------RGB CONTROL--------------------v
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|Debug | RGB  |RGBMOD| HUE+ | HUE- | SAT+ | SAT- |BRGTH+|BRGTH-|  Del |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |MUSmod|Aud on|Audoff|AGnorm|AGswap|      |Colemk|      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|TermOn|TermOf|      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_grid(
    _______, RESET,   DEBUG,   RGB_TOG, RGB_MOD, RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD,  RGB_VAI, RGB_VAD, KC_DEL ,
    _______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, _______,  COLEMAK, _______, _______, _______,
    _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
),
};

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _NAV, _NUM, _FUNC);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case COLEMAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_COLEMAK);
      }
      return false;
      break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
      } else {
        unregister_code(KC_RSFT);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void dance_lprn_finished(qk_tap_dance_state_t *state, void *user_data) {
    register_code16(KC_LPRN);
    if (state->count > 1) {
        register_code16(KC_RPRN);
        register_code16(KC_LEFT);
    }
}

void dance_lprn_reset(qk_tap_dance_state_t *state, void *user_data) {
    unregister_code16(KC_LPRN);
    if (state->count > 1) {
        unregister_code16(KC_RPRN);
        unregister_code16(KC_LEFT);
    }
}

qk_tap_dance_action_t tap_dance_actions[] = {
    [_TD_LPRN] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_lprn_finished, dance_lprn_reset),
};

/*
bool encoder_update_user(uint8_t index, bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
    return true;
}
*/
bool dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0: {
            if (active) {
                layer_on(_ADJUST);
            } else {
                layer_off(_ADJUST);
            }
            break;
        }
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
    return true;
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}
