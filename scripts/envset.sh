#!/bin/sh
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true

unset CMAKE_ARGS
unset CMAKE_C
unset CMAKE_C_COMPILER_WORKS
unset CMAKE_TOOLCHAIN_FILE
unset CMAKE_PREFIX_PATH
unset CMAKE_MODULE_PATH

