// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "uni_property.h"

// Globals
// Keep them sorted
// Keys name should not be longer than NVS_KEY_NAME_MAX_SIZE (15 chars).
const char* UNI_PROPERTY_KEY_ALLOWLIST_ENABLED = "bp.bt.allow_en";
const char* UNI_PROPERTY_KEY_ALLOWLIST_LIST = "bp.bt.allowlist";
const char* UNI_PROPERTY_KEY_BLE_ENABLED = "bp.ble.enabled";
const char* UNI_PROPERTY_KEY_GAP_INQ_LEN = "bp.gap.inq_len";
const char* UNI_PROPERTY_KEY_GAP_LEVEL = "bp.gap.level";
const char* UNI_PROPERTY_KEY_GAP_MAX_PERIODIC_LEN = "bp.gap.max_len";
const char* UNI_PROPERTY_KEY_GAP_MIN_PERIODIC_LEN = "bp.gap.min_len";
const char* UNI_PROPERTY_KEY_MOUSE_SCALE = "bp.mouse.scale";
const char* UNI_PROPERTY_KEY_VIRTUAL_DEVICE_ENABLED = "bp.virt_dev_en";

// Unijoysticle only
// TODO: Move them to the Unijoysticle file.
// Keep them sorted
const char* UNI_PROPERTY_KEY_UNI_AUTOFIRE_CPS = "bp.uni.autofire";
const char* UNI_PROPERTY_KEY_UNI_BB_FIRE_THRESHOLD = "bp.uni.bb_fire";
const char* UNI_PROPERTY_KEY_UNI_BB_MOVE_THRESHOLD = "bp.uni.bb_move";
const char* UNI_PROPERTY_KEY_UNI_C64_POT_MODE = "bp.uni.c64pot";
const char* UNI_PROPERTY_KEY_UNI_MODEL = "bp.uni.model";
const char* UNI_PROPERTY_KEY_UNI_MOUSE_EMULATION = "bp.uni.mouseemu";
const char* UNI_PROPERTY_KEY_UNI_SERIAL_NUMBER = "bp.uni.serial";
const char* UNI_PROPERTY_KEY_UNI_VENDOR = "bp.uni.vendor";
