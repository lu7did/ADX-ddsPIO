// fs.h
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADX_CFG_FILENAME  "CONFIG.TXT"

// API FS
bool fs_init_and_mount(void);
void fs_unmount(void);
bool fs_ensure_cfg_exists(void);

// texto completo
bool fs_read_text(char* out, size_t out_max, size_t* out_len);
bool fs_write_text(const char* text, size_t len);

// JSON helpers
bool fs_json_get(const char* key, char* out, size_t out_max);
bool fs_json_set(const char* key, const char* value);
bool fs_json_save(void);

// Debug (para ver qué pasó)
int  fs_last_fr_mount(void);
int  fs_last_fr_mkfs(void);
int  fs_last_fr_open(void);
int  fs_last_fr_read(void);

bool fs_get_kv(const char* json, const char* key, char* out, size_t out_max);


#ifdef __cplusplus
}
#endif
