# SPDX-License-Identifier: Apache-2.0
zephyr_library()
zephyr_include_directories(include)
zephyr_library_sources_ifdef(CONFIG_LP5810 lp5810.c)