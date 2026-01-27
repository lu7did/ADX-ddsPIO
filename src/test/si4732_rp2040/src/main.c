/*
 * =======================================================================================
 * si4723_rp2040
 * (c) Dr. Pedro E. Colla (LU7DZ) <pedro.colla@gmail.com>
 * 
 * new generation rp2040 ADX based digital transceiver 
 * 
 * This is mainly an integration effort with some new code developed for this project,
 * some unique features has been developed for this firmware as well such as the
 * quadrature digital frequency synth.
 *  
 * The integration effort is being built on top of previous work from many parties,
 * including myself as follows:
 *----------------------------------------------------------------------------
 *                       This program uses a RDX_rp2040 board
 *  modifications will be made later to adapt the hardware configuration to suit
 *  the ADX-ddsPIO project which is the one that will receive the integration 
 *----------------------------------------------------------------------------
 *
 * Implementation of a SI4732 based demo with TinyUSB CDC (stand-alone, integration-friendly)
 * - CDC command interface for debug/control
 * - I2C on GPIO16 (SDA) and GPIO17 (SCL)
 *
 * Commands (send via USB serial):
 *   help
 *   region us|eu|jp|ar
 *   mode 
 *   fm|am
 *   band fm|mw|49m|40m|31m
 *   tune <freq>
 *     - FM: <freq> in 10kHz units (e.g. 10030 -> 100.30 MHz)
 *     - AM: <freq> in kHz (e.g. 7100)
 *   seek up|down
 *   vol <0..63>
 *   mute <0|1>
 *----------------------------------------------------------------------------
  */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include "si4732.h"


#define I2C_PORT i2c0
#define SDA_PIN  16
#define SCL_PIN  17

static void cdc_write_str(const char *s) {
  if (!tud_cdc_connected()) return;
  tud_cdc_write_str(s);
  tud_cdc_write_flush();
}

static void cdc_write_ln(const char *s) {
  cdc_write_str(s);
  cdc_write_str("\r\n");
}

static si4732_region_t parse_region(const char *s) {
  if (!s) return SI4732_REGION_AR;
  if (!strcmp(s, "us")) return SI4732_REGION_US;
  if (!strcmp(s, "eu")) return SI4732_REGION_EU;
  if (!strcmp(s, "jp")) return SI4732_REGION_JP;
  return SI4732_REGION_AR;
}

static si4732_band_preset_t parse_band(const char *s) {
  if (!s) return SI4732_BAND_FM_BROADCAST;
  if (!strcmp(s, "fm"))  return SI4732_BAND_FM_BROADCAST;
  if (!strcmp(s, "mw"))  return SI4732_BAND_AM_MW;
  if (!strcmp(s, "49m")) return SI4732_BAND_SW_49M;
  if (!strcmp(s, "40m")) return SI4732_BAND_SW_40M;
  return SI4732_BAND_SW_31M;
}

static void print_help(void) {
  cdc_write_ln("SI4732 CDC Demo commands:");
  cdc_write_ln("  help");
  cdc_write_ln("  region us|eu|jp|ar");
  cdc_write_ln("  mode fm|am");
  cdc_write_ln("  band fm|mw|49m|40m|31m");
  cdc_write_ln("  tune <freq>   (FM: 10kHz units, AM: kHz)");
  cdc_write_ln("  seek up|down");
  cdc_write_ln("  vol <0..63>");
  cdc_write_ln("  mute <0|1>");
}

static void process_line(si4732_t *radio, char *line) {
  // tokenize
  char *cmd = strtok(line, " \t\r\n");
  if (!cmd) return;

  if (!strcmp(cmd, "help")) {
    print_help();
    return;
  }

  if (!strcmp(cmd, "region")) {
    char *arg = strtok(NULL, " \t\r\n");
    si4732_region_t r = parse_region(arg);
    si4732_status_t rc = si4732_apply_region(radio, r);
    cdc_write_str("region set rc="); char buf[32]; snprintf(buf, sizeof(buf), "%d", (int)rc); cdc_write_ln(buf);
    return;
  }

  if (!strcmp(cmd, "mode")) {
    char *arg = strtok(NULL, " \t\r\n");
    if (arg && !strcmp(arg, "fm")) {
      si4732_status_t rc = si4732_power_up_fm(radio);
      cdc_write_str("mode fm rc="); char b[32]; snprintf(b, sizeof(b), "%d", (int)rc); cdc_write_ln(b);
      return;
    }
    if (arg && !strcmp(arg, "am")) {
      si4732_status_t rc = si4732_power_up_am(radio, false);
      cdc_write_str("mode am rc="); char b[32]; snprintf(b, sizeof(b), "%d", (int)rc); cdc_write_ln(b);
      return;
    }
    cdc_write_ln("ERR: mode fm|am");
    return;
  }

  if (!strcmp(cmd, "band")) {
    char *arg = strtok(NULL, " \t\r\n");
    si4732_band_preset_t bp = parse_band(arg);
    si4732_band_t b = si4732_band_preset(bp, radio->region_profile);

    //*--- Ensure correct power up for FM/AM

    si4732_status_t rc = (b.mode == SI4732_MODE_FM) ? si4732_power_up_fm(radio) : si4732_power_up_am(radio, false);
    if (rc != SI4732_OK) { cdc_write_ln("ERR: power up failed"); return; }

    rc = si4732_set_band(radio, &b);
    cdc_write_str("band set rc="); char out[32]; snprintf(out, sizeof(out), "%d", (int)rc); cdc_write_ln(out);
    return;
  }

  if (!strcmp(cmd, "tune")) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) { cdc_write_ln("ERR: tune <freq>"); return; }
    uint32_t f = (uint32_t)strtoul(arg, NULL, 10);
    si4732_status_t rc = si4732_tune(radio, f);
    cdc_write_str("tune rc="); char out[32]; snprintf(out, sizeof(out), "%d", (int)rc); cdc_write_ln(out);
    return;
  }

  if (!strcmp(cmd, "seek")) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) { cdc_write_ln("ERR: seek up|down"); return; }
    bool up = !strcmp(arg, "up");
    si4732_status_t rc = si4732_seek(radio, up, true);
    cdc_write_str("seek rc="); char out[32]; snprintf(out, sizeof(out), "%d", (int)rc); cdc_write_ln(out);
    return;
  }

  if (!strcmp(cmd, "vol")) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) { cdc_write_ln("ERR: vol <0..63>"); return; }
    uint32_t v = (uint32_t)strtoul(arg, NULL, 10);
    si4732_status_t rc = si4732_set_volume(radio, (uint8_t)v);
    cdc_write_str("vol rc="); char out[32]; snprintf(out, sizeof(out), "%d", (int)rc); cdc_write_ln(out);
    return;
  }

  if (!strcmp(cmd, "mute")) {
    char *arg = strtok(NULL, " \t\r\n");
    if (!arg) { cdc_write_ln("ERR: mute <0|1>"); return; }
    int m = atoi(arg);
    si4732_status_t rc = si4732_set_mute(radio, m != 0, m != 0);
    cdc_write_str("mute rc="); char out[32]; snprintf(out, sizeof(out), "%d", (int)rc); cdc_write_ln(out);
    return;
  }

  cdc_write_ln("ERR: unknown command. try 'help'");
}

int main(void) {

  //*--- NOTE: We do not rely on stdio over USB; we use TinyUSB CDC directly.
  //*--- this is the environment used by the ADX-ddsPIO projects and therefore
  //*--- integration with that project will be mitigated.

  stdio_init_all();

  //*--- Init USB stack

  tusb_init();

  //*--- Init radio
  
  si4732_t radio;
  (void)si4732_init(&radio, I2C_PORT, SI4732_I2C_ADDR_DEFAULT, SDA_PIN, SCL_PIN, 400000);

  //*--- Power up FM and apply region AR defaults
  
  (void)si4732_power_up_fm(&radio);
  (void)si4732_apply_region(&radio, SI4732_REGION_AR);

  si4732_band_t fm = si4732_band_preset(SI4732_BAND_FM_BROADCAST, radio.region_profile);
  (void)si4732_set_band(&radio, &fm);
  (void)si4732_set_volume(&radio, 30);
  (void)si4732_set_mute(&radio, false, false);

  //*--- Line buffer
  static char line[128];
  size_t idx = 0;

  while (true) {
    tud_task();

    if (tud_cdc_connected()) {
      // greet once after connect
      static bool greeted = false;
      if (!greeted) {
        greeted = true;
        cdc_write_ln("SI4732_rp2040 CDC ready. Type 'help'.");
      }

      while (tud_cdc_available()) {
        char c = (char)tud_cdc_read_char();
        if (c == '\r') continue;
        if (c == '\n') {
          line[idx] = 0;
          if (idx > 0) process_line(&radio, line);
          idx = 0;
        } else if (idx < sizeof(line) - 1) {
          line[idx++] = c;
        }
      }
    } else {
      //*--- reset greet flag when disconnected

      static bool last = false;
      
      if (last) { /* just disconnected */ }
      last = false;
      idx = 0;
    }

    sleep_ms(1);
  }
}
