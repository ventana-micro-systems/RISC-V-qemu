/*
 * QEMU RISC-V VirtIO machine interface
 *
 * Copyright (c) 2017 SiFive, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_RISCV_VIRT_H
#define HW_RISCV_VIRT_H

#include "hw/riscv/riscv_hart.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "qom/object.h"

#define VIRT_CPUS_MAX_BITS             9
#define VIRT_CPUS_MAX                  (1 << VIRT_CPUS_MAX_BITS)
#define VIRT_SOCKETS_MAX_BITS          2
#define VIRT_SOCKETS_MAX               (1 << VIRT_SOCKETS_MAX_BITS)

#define TYPE_RISCV_VIRT_MACHINE MACHINE_TYPE_NAME("virt")
typedef struct RISCVVirtState RISCVVirtState;
DECLARE_INSTANCE_CHECKER(RISCVVirtState, RISCV_VIRT_MACHINE,
                         TYPE_RISCV_VIRT_MACHINE)

typedef enum RISCVVirtAIAType {
    VIRT_AIA_TYPE_NONE=0,
    VIRT_AIA_TYPE_APLIC,
    VIRT_AIA_TYPE_APLIC_IMSIC,
} RISCVVirtAIAType;

struct RISCVVirtState {
    /*< private >*/
    MachineState parent;

    /*< public >*/
    RISCVHartArrayState soc[VIRT_SOCKETS_MAX];
    DeviceState *irqchip[VIRT_SOCKETS_MAX];
    MemMapEntry *memmap;
    PFlashCFI01 *flash[2];
    FWCfgState *fw_cfg;

    int fdt_size;
    bool have_aclint;
    bool have_acpi;
    RISCVVirtAIAType aia_type;
    int aia_guests;
    PCIBus *bus;
    char *oem_id;
    char *oem_table_id;
};

enum {
	RISCV_HART_CAP_MMU_TYPE_39,
	RISCV_HART_CAP_MMU_TYPE_48
};

enum {
	RISCV_RHCT_ADD_STD_I_TYPE_ZIFENCEIV2 = 0x0000,
	RISCV_RHCT_ADD_STD_I_TYPE_ZIHINTPAUSEV2,
	RISCV_RHCT_ADD_STD_I_TYPE_ZICSR,
	RISCV_RHCT_ADD_STD_I_TYPE_ZICMOBASE,
	RISCV_RHCT_ADD_STD_M_TYPE_ZMMUL = 0x0100,
	RISCV_RHCT_ADD_STD_A_TYPE_ZAM = 0x0200,
	RISCV_RHCT_ADD_STD_F_TYPE_ZFINX = 0x0300,
	RISCV_RHCT_ADD_STD_F_TYPE_ZHINX,
	RISCV_RHCT_ADD_STD_F_TYPE_ZHINXMIN,
	RISCV_RHCT_ADD_STD_F_TYPE_ZFH,
	RISCV_RHCT_ADD_STD_D_TYPE_ZDINX = 0x0400,
	RISCV_RHCT_ADD_STD_C_TYPE_ZCE = 0x0700,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBA = 0x0800,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBB,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBC,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBE,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBF,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBK,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBP,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBR,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBS,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBKB,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBKC,
	RISCV_RHCT_ADD_STD_B_TYPE_ZBKX,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKND = 0x0900,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKNE,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKNH,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKSED,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKSH,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKT,
	RISCV_RHCT_ADD_STD_K_TYPE_ZKR,
	RISCV_RHCT_ADD_STD_J_TYPE_ZJPM = 0x0A00,
	RISCV_RHCT_ADD_STD_T_TYPE_ZTSO = 0x0B00,
	RISCV_RHCT_SUPERVISOR_VM_SVNAPOT = 0x1000,
	RISCV_RHCT_SUPERVISOR_VM_SVPBMT,
	RISCV_RHCT_SUPERVISOR_VM_SVINVAL,
	RISCV_RHCT_SUPERVISOR_TIMER_SSTC = 0x1100,
	RISCV_RHCT_SUPERVISOR_PMU_SSCOF = 0x1200
};

struct AcpiExtensionList {
	uint16_t extension;
	uint16_t attributes;
};

enum {
    VIRT_DEBUG,
    VIRT_MROM,
    VIRT_TEST,
    VIRT_RTC,
    VIRT_CLINT,
    VIRT_ACLINT_SSWI,
    VIRT_PLIC,
    VIRT_APLIC_M,
    VIRT_APLIC_S,
    VIRT_UART0,
    VIRT_VIRTIO,
    VIRT_FW_CFG,
    VIRT_IMSIC_M,
    VIRT_IMSIC_S,
    VIRT_FLASH,
    VIRT_DRAM,
    VIRT_PCIE_MMIO,
    VIRT_PCIE_PIO,
    VIRT_PCIE_ECAM,
    VIRT_HIGH_PCIE_MMIO
};

enum {
    UART0_IRQ = 10,
    RTC_IRQ = 11,
    VIRTIO_IRQ = 1, /* 1 to 8 */
    VIRTIO_COUNT = 8,
    PCIE_IRQ = 0x20, /* 32 to 35 */
    VIRTIO_NDEV = 0x35 /* Arbitrary maximum number of interrupts */
};

#define VIRT_IRQCHIP_IPI_MSI 1
#define VIRT_IRQCHIP_NUM_MSIS 255
#define VIRT_IRQCHIP_NUM_SOURCES VIRTIO_NDEV
#define VIRT_IRQCHIP_NUM_PRIO_BITS 3
#define VIRT_IRQCHIP_MAX_GUESTS_BITS 3
#define VIRT_IRQCHIP_MAX_GUESTS ((1U << VIRT_IRQCHIP_MAX_GUESTS_BITS) - 1U)

#define VIRT_PLIC_PRIORITY_BASE 0x04
#define VIRT_PLIC_PENDING_BASE 0x1000
#define VIRT_PLIC_ENABLE_BASE 0x2000
#define VIRT_PLIC_ENABLE_STRIDE 0x80
#define VIRT_PLIC_CONTEXT_BASE 0x200000
#define VIRT_PLIC_CONTEXT_STRIDE 0x1000
#define VIRT_PLIC_SIZE(__num_context) \
    (VIRT_PLIC_CONTEXT_BASE + (__num_context) * VIRT_PLIC_CONTEXT_STRIDE)

#define FDT_PCI_ADDR_CELLS    3
#define FDT_PCI_INT_CELLS     1
#define FDT_PLIC_INT_CELLS    1
#define FDT_APLIC_INT_CELLS   2
#define FDT_IMSIC_INT_CELLS   0
#define FDT_MAX_INT_CELLS     2
#define FDT_MAX_INT_MAP_WIDTH (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_MAX_INT_CELLS)
#define FDT_PLIC_INT_MAP_WIDTH  (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_PLIC_INT_CELLS)
#define FDT_APLIC_INT_MAP_WIDTH (FDT_PCI_ADDR_CELLS + FDT_PCI_INT_CELLS + \
                                 1 + FDT_APLIC_INT_CELLS)

void virt_acpi_setup(RISCVVirtState *vms);
#endif
