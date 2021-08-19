/*
 * Type 44 
 *
 * Copyright (c) 2021 Ventana Microsystems Inc.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "hw/riscv/smbios.h"
#include "smbios_build.h"

void smbios_build_type_44_table(MachineState *ms)
{
    riscv_smbios_build_type_44_table(ms);
}
