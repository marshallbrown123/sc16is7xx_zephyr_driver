# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SC16IS7XX)  
#include_directories(.)
target_include_directories(app PRIVATE .)
target_sources(app PRIVATE sc16is7xx.c)
endif()