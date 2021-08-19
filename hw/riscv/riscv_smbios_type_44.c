/*
 * RISC-V Type 44 stub
 *
 * Copyright (c) 2021 Ventana Microsystems Inc.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "qemu/config-file.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/option.h"
#include "sysemu/sysemu.h"
#include "qemu/uuid.h"
#include "hw/riscv/numa.h"
#include "hw/firmware/smbios.h"
#include "hw/loader.h"
#include "hw/riscv/virt.h"
#include "hw/smbios/smbios_build.h"
#include "hw/riscv/smbios.h"

#define SMBIOS_TYPE_44_ARCH_RV64 0x7

void riscv_smbios_build_type_44_table(MachineState *ms)
{
    RISCVVirtState *vms = RISCV_VIRT_MACHINE(ms);
    RISCVCPU *cpu;
    int socket, i, instance=0;

    for (socket = 0; socket < riscv_socket_count(ms);  socket++) {
        for (i = 0; i < vms->soc[socket].num_harts; i++) {
            SMBIOS_BUILD_TABLE_PRE(44, 0x4000 + instance, true); /* required */
            cpu = &vms->soc[socket].harts[i];
            t->ref_handle = 0xffee; /* reserved by UEFI PI spec - revisit */
            t->psb.block_length = sizeof(struct riscv_processor_specific_data);
            t->psb.arch_type = SMBIOS_TYPE_44_ARCH_RV64; // RV64
            t->psb.psd.revision = 0x100; // Version 1.0
            t->psb.psd.isa_ext = cpu->env.misa; /*isa */
            t->psb.psd.data_length = sizeof(struct riscv_processor_specific_data);
            instance++;

            SMBIOS_BUILD_TABLE_POST;
        }
    }
}
