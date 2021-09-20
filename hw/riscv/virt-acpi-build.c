/* Support for generating ACPI tables and passing them to Guests
 *
 * RISC-V virt ACPI generation
 *
 * Copyright (C) 2008-2010  Kevin O'Connor <kevin@koconnor.net>
 * Copyright (C) 2006 Fabrice Bellard
 * Copyright (C) 2013 Red Hat Inc
 * Copyright (C) 2021 Ventana Micro Systems Inc
 *
 * Author: Michael S. Tsirkin <mst@redhat.com>
 *
 * Copyright (c) 2015 HUAWEI TECHNOLOGIES CO.,LTD.
 *
 * Author: Shannon Zhao <zhaoshenglong@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitmap.h"
#include "trace.h"
#include "hw/acpi/acpi-defs.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/bios-linker-loader.h"
#include "hw/acpi/aml-build.h"
#include "hw/acpi/utils.h"
#include "hw/acpi/pci.h"
#include "hw/acpi/memory_hotplug.h"
#include "hw/acpi/generic_event_device.h"
#include "hw/acpi/tpm.h"
#include "hw/pci/pcie_host.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/gpex.h"
#include "hw/acpi/ghes.h"
#include "sysemu/reset.h"
#include "hw/riscv/virt.h"
#include "hw/riscv/numa.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/riscv_imsic.h"

#define ACPI_IMSIC_GROUP_MAX_SIZE      (1U << IMSIC_MMIO_GROUP_MIN_SHIFT)

#define ACPI_BUILD_TABLE_SIZE             0x20000

uint16_t imsic_max_hart_per_socket = 0;

static uint32_t acpi_num_bits(uint32_t count)
{
    uint32_t ret = 0;

    while (BIT(ret) < count) {
        ret++;
    }

    return ret;
}

static void acpi_dsdt_add_cpus(Aml *scope, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    uint16_t i;

    
    for (i = 0; i < ms->smp.cpus; i++) {
            Aml *dev = aml_device("C%.03X", i);
            aml_append(dev, aml_name_decl("_HID", aml_string("ACPI0007")));
            aml_append(dev, aml_name_decl("_UID", aml_int(i)));
            aml_append(scope, dev);
    }
}

static void acpi_dsdt_add_uart(Aml *scope, const MemMapEntry *uart_memmap,
                                           uint32_t uart_irq)
{
    Aml *dev = aml_device("COM0");
    aml_append(dev, aml_name_decl("_HID", aml_string("PNP0501")));
    aml_append(dev, aml_name_decl("_UID", aml_int(0)));

    Aml *crs = aml_resource_template();
    aml_append(crs, aml_memory32_fixed(uart_memmap->base,
                                       uart_memmap->size, AML_READ_WRITE));
    aml_append(crs,
               aml_interrupt(AML_CONSUMER, AML_LEVEL, AML_ACTIVE_HIGH,
                             AML_EXCLUSIVE, &uart_irq, 1));
    aml_append(dev, aml_name_decl("_CRS", crs));

    Aml *pkg = aml_package(2);
    aml_append(pkg, aml_string("clock-frequency"));
    aml_append(pkg, aml_int(3686400));

    Aml *UUID = aml_touuid("DAFFD814-6EBA-4D8C-8A91-BC9BBF4AA301");

    Aml *pkg1 = aml_package(1);
    aml_append(pkg1, pkg);

    Aml *package = aml_package(2);
    aml_append(package, UUID);
    aml_append(package, pkg1);

    aml_append(dev, aml_name_decl("_DSD", package));
    aml_append(scope, dev);
}

static void acpi_dsdt_add_fw_cfg(Aml *scope, const MemMapEntry *fw_cfg_memmap)
{
    Aml *dev = aml_device("FWCF");
    aml_append(dev, aml_name_decl("_HID", aml_string("QEMU0002")));
    /* device present, functioning, decoding, not shown in UI */
    aml_append(dev, aml_name_decl("_STA", aml_int(0xB)));
    aml_append(dev, aml_name_decl("_CCA", aml_int(1)));

    Aml *crs = aml_resource_template();
    aml_append(crs, aml_memory32_fixed(fw_cfg_memmap->base,
                                       fw_cfg_memmap->size, AML_READ_WRITE));
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(scope, dev);
}

static void acpi_dsdt_add_virtio(Aml *scope,
                                 const MemMapEntry *virtio_mmio_memmap,
                                 uint32_t mmio_irq, int num)
{
    hwaddr base = virtio_mmio_memmap->base;
    hwaddr size = virtio_mmio_memmap->size;
    int i;

    for (i = 0; i < num; i++) {
        uint32_t irq = mmio_irq + i;
        Aml *dev = aml_device("VR%02u", i);
        aml_append(dev, aml_name_decl("_HID", aml_string("LNRO0005")));
        aml_append(dev, aml_name_decl("_UID", aml_int(i)));
        aml_append(dev, aml_name_decl("_CCA", aml_int(1)));

        Aml *crs = aml_resource_template();
        aml_append(crs, aml_memory32_fixed(base, size, AML_READ_WRITE));
        aml_append(crs,
                   aml_interrupt(AML_CONSUMER, AML_LEVEL, AML_ACTIVE_HIGH,
                                 AML_EXCLUSIVE, &irq, 1));
        aml_append(dev, aml_name_decl("_CRS", crs));
        aml_append(scope, dev);
        base += size;
    }
}

static int acpi_madt_aplic(uint16_t socket, RISCVVirtState *vms,
		           GArray *entry, int base_cpu_id)
{
    int node_size;
    int cpu_id = base_cpu_id;
    AcpiImsicHartIndex imsic_hart_index = {0};
    MachineState *mc = MACHINE(vms);
    AcpiMadtAplic *aplic;

    node_size = sizeof *aplic + (sizeof(uint8_t) * vms->soc[socket].num_harts);
    aplic = acpi_data_push(entry, node_size);
    imsic_hart_index.lhxw = acpi_num_bits(imsic_max_hart_per_socket);
    imsic_hart_index.hhxw = acpi_num_bits(riscv_socket_count(mc));
    imsic_hart_index.lhxs = 0;
    imsic_hart_index.hhxs = IMSIC_MMIO_GROUP_MIN_SHIFT;

    aplic->type = 0x11; // APLIC
    aplic->length = node_size;
    aplic->num_interrupts = VIRT_IRQCHIP_NUM_SOURCES;
    aplic->global_irq_base = 0;
    aplic->mode = 0x1; // 1: S-mode
    aplic->id = 0x1; 
    aplic->target_imsic = 0x1; 
    aplic->aplic_addr = vms->memmap[VIRT_APLIC_S].base +
                                    vms->memmap[VIRT_APLIC_S].size * socket;
    aplic->aplic_size = vms->memmap[VIRT_APLIC_S].size;
    aplic->imsic_info.imsic_addr = vms->memmap[VIRT_IMSIC_S].base +
                     socket * ACPI_IMSIC_GROUP_MAX_SIZE;
    aplic->imsic_info.imsic_size = IMSIC_HART_SIZE(0) * vms->soc[socket].num_harts;
    aplic->imsic_info.hart_index = imsic_hart_index.hart_index;
    aplic->imsic_info.num_harts = vms->soc[socket].num_harts;

    for (int i = 0; i < vms->soc[socket].num_harts; i++) {
        aplic->imsic_info.cpuId[i] = cpu_id++;
    }
    return cpu_id;
    
}

static void acpi_dsdt_add_aplic(Aml *scope, RISCVVirtState *vms)
{
    MachineState *ms = MACHINE(vms);
    uint16_t i, num, cpu_id = 0;;

    const MemMapEntry *memmap = vms->memmap;

    
    for (i = 0; i < riscv_socket_count(ms); i++) {
            GArray *madt_buf = g_array_new(0, 1, 1);
            Aml *dev = aml_device("PLIC");
            aml_append(dev, aml_name_decl("_HID", aml_string("APLIC001")));
            aml_append(dev, aml_name_decl("_UID", aml_int(i)));
            /* device present, functioning, decoding, not shown in UI */
            aml_append(dev, aml_name_decl("_STA", aml_int(0xB)));
            aml_append(dev, aml_name_decl("_CCA", aml_int(1)));


            /*  _MAT */
            num = acpi_madt_aplic(i, vms, madt_buf, cpu_id);
	    cpu_id += num;
            aml_append(dev, aml_name_decl("_MAT",
                aml_buffer(madt_buf->len, (uint8_t *)madt_buf->data)));
            g_array_free(madt_buf, true);

            Aml *crs = aml_resource_template();
            aml_append(crs, aml_memory32_fixed(memmap[VIRT_APLIC_S].base * i,
                                               memmap[VIRT_APLIC_S].size, AML_READ_WRITE));
            aml_append(dev, aml_name_decl("_CRS", crs));
            aml_append(scope, dev);
    }
}

static void acpi_dsdt_add_pci(Aml *scope, const MemMapEntry *memmap,
                              uint32_t irq, RISCVVirtState *vms)
{
    struct GPEXConfig cfg = {
        .mmio32 = memmap[VIRT_PCIE_MMIO],
        .mmio64 = memmap[VIRT_HIGH_PCIE_MMIO],
        .pio    = memmap[VIRT_PCIE_PIO],
        .ecam   = memmap[VIRT_PCIE_ECAM],
        .irq    = irq,
        .bus    = vms->bus,
    };

    acpi_dsdt_add_gpex(scope, &cfg);
}

/* RTDT */
static void
build_rtdt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    MachineState *mc = MACHINE(vms);
    int rtdt_start = table_data->len;
    AcpiRiscvTimerTable *rtdt;
    int socket, node_size;
    uint16_t base_hartid;

    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        node_size = sizeof(*rtdt) + sizeof(struct Acpi128) * vms->soc[socket].num_harts;

        rtdt = acpi_data_push(table_data, node_size);

        rtdt->time_base_frequency = RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ;
        rtdt->num_mtimer = riscv_socket_count(mc);

        rtdt->num_harts = vms->soc[socket].num_harts;
        base_hartid = riscv_socket_first_hartid(mc, socket);

        for (int i = 0; i < vms->soc[socket].num_harts; i++) {
            rtdt->hartId[i].lo = base_hartid + i;
            rtdt->hartId[i].hi = 0;
        }
        rtdt->base_addr = vms->memmap[VIRT_CLINT].base +
                                            (RISCV_ACLINT_DEFAULT_MTIMER_SIZE * socket); 
        rtdt->size = RISCV_ACLINT_DEFAULT_MTIMER_SIZE;
    }

    build_header(linker, table_data,
                 (void *)(table_data->data + rtdt_start), "RTDT",
                 table_data->len - rtdt_start, 2, vms->oem_id,
                 vms->oem_table_id);
}

/* MADT */
static void
build_madt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    int madt_start = table_data->len;
    MachineState *mc = MACHINE(vms);
    int socket;
    AcpiImsicHartIndex imsic_hart_index = {0};
    AcpiMadtImsic *imsic_s;
    AcpiImsicSocket *imsic_socket;
    AcpiMadtAplic *aplic_s;
    AcpiMadtRintc *rintc;
    uint16_t node_size = 0, base_hartid;
    uint16_t cpu_id = 0;

    acpi_data_push(table_data, sizeof(AcpiMultipleApicTable));

    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        base_hartid = riscv_socket_first_hartid(mc, socket);

        for (int i = 0; i < vms->soc[socket].num_harts; i++) {
            rintc = acpi_data_push(table_data, sizeof(AcpiMadtRintc));
            rintc->type = 0x13;
            rintc->length = sizeof(AcpiMadtRintc);
            rintc->version = 1;
            rintc->aia_csr_enabled = 1;
            rintc->acpi_proc_id = cpu_id++;
            rintc->hartId.lo = base_hartid + i;
            rintc->hartId.hi = 0;
        }
    }

    // IMSIC Groups

    cpu_id = 0;
    // S-mode IMSIC Group
    node_size = sizeof(*imsic_s) + 
                (sizeof(*imsic_socket) * riscv_socket_count(mc));
    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        node_size += sizeof(uint8_t) * vms->soc[socket].num_harts;
    }
    imsic_s = acpi_data_push(table_data, node_size);

    imsic_s->type = 0x12;
    imsic_s->version = 0x1;
    imsic_s->id = 0x1; // S-mode APLIC should point to this IMSIC
    imsic_s->mode = 0x1;      // 1: S-mode
    imsic_s->length = node_size;
    imsic_s->num_interrupt_id = VIRT_IRQCHIP_NUM_MSIS;
    imsic_s->ipi_base = VIRT_IRQCHIP_BASE_IPI;
    imsic_s->ipi_count = VIRT_IRQCHIP_NUM_IPIS;
    imsic_s->num_sockets = riscv_socket_count(mc);
    imsic_s->total_num_harts = mc->smp.cpus;
    imsic_s->ext_irq_num = IRQ_S_EXT;

    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        imsic_s->socket_imsic[socket].imsic_addr = vms->memmap[VIRT_IMSIC_S].base +
                     socket * ACPI_IMSIC_GROUP_MAX_SIZE;
        imsic_s->socket_imsic[socket].imsic_size = IMSIC_HART_SIZE(0) * 
                                                    vms->soc[socket].num_harts;
        imsic_s->socket_imsic[socket].num_harts = vms->soc[socket].num_harts;
        base_hartid = riscv_socket_first_hartid(mc, socket);

        for (int i = 0; i < vms->soc[socket].num_harts; i++) {
            imsic_s->socket_imsic[socket].cpuId[i] = cpu_id++;
        }
    }
    imsic_hart_index.lhxw = acpi_num_bits(imsic_max_hart_per_socket);
    imsic_hart_index.hhxw = acpi_num_bits(riscv_socket_count(mc));
    imsic_hart_index.lhxs = 0;
    imsic_hart_index.hhxs = IMSIC_MMIO_GROUP_MIN_SHIFT;

    imsic_s->hart_index = imsic_hart_index.hart_index;

    // APLIC

    cpu_id = 0;
    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        node_size = sizeof *aplic_s + (sizeof(uint8_t) * vms->soc[socket].num_harts);
        aplic_s = acpi_data_push(table_data, node_size);
        aplic_s->type = 0x11; // APLIC
        aplic_s->length = node_size;
        aplic_s->num_interrupts = VIRT_IRQCHIP_NUM_SOURCES;
        aplic_s->global_irq_base = 0;
        aplic_s->mode = 0x1; // 1: S-mode
        aplic_s->id = 0x1; 
        aplic_s->target_imsic = 0x1; 
        aplic_s->aplic_addr = vms->memmap[VIRT_APLIC_S].base +
                                        vms->memmap[VIRT_APLIC_S].size * socket;
        aplic_s->aplic_size = vms->memmap[VIRT_APLIC_S].size;
        aplic_s->imsic_info.imsic_addr = vms->memmap[VIRT_IMSIC_S].base +
                         socket * ACPI_IMSIC_GROUP_MAX_SIZE;
        aplic_s->imsic_info.imsic_size = IMSIC_HART_SIZE(0) * vms->soc[socket].num_harts;
        aplic_s->imsic_info.hart_index = imsic_hart_index.hart_index;
        aplic_s->imsic_info.num_harts = vms->soc[socket].num_harts;
        base_hartid = riscv_socket_first_hartid(mc, socket);

        for (int i = 0; i < vms->soc[socket].num_harts; i++) {
            aplic_s->imsic_info.cpuId[i] = cpu_id++;
        }
    }

    build_header(linker, table_data,
                 (void *)(table_data->data + madt_start), "APIC",
                 table_data->len - madt_start, 3, vms->oem_id,
                 vms->oem_table_id);
}

/* FADT */
static void build_fadt_rev5(GArray *table_data, BIOSLinker *linker,
                            RISCVVirtState *vms, unsigned dsdt_tbl_offset)
{
    /* ACPI v5.1 */
    AcpiFadtData fadt = {
        .rev = 5,
        .minor_ver = 1,
        .flags = 1 << ACPI_FADT_F_HW_REDUCED_ACPI,
        .xdsdt_tbl_offset = &dsdt_tbl_offset,
    };

    build_fadt(table_data, linker, &fadt, vms->oem_id, vms->oem_table_id);
}

static void
build_pptt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    	int pptt_start = table_data->len;
    	MachineState *ms = MACHINE(vms);
	int i, socket, acpi_proc_id = 0;
	RISCVCPU *cpu;
	uint32_t ppts_priv_rsrc[1];
	AcpiPpttHartCap hart_caps = {0};

    
	// Header
    	acpi_data_push(table_data, sizeof(AcpiTableHeader));

	for (socket = 0; socket < riscv_socket_count(ms); socket++) {
		uint32_t socket_offset = table_data->len - pptt_start;
		build_processor_hierarchy_node(
				table_data,
				/*
				 * ACPI 6.2 - Physical package
				 * represents the boundary of a physical package
				 */
				(1 << 0),
				0, socket, NULL, 0);
		for (i = 0; i < vms->soc[socket].num_harts; i++) {
			cpu = &vms->soc[socket].harts[i];
			uint32_t hart_ppts_start = table_data->len - pptt_start;
			hart_caps.mmu_type = RISCV_HART_CAP_MMU_TYPE_48;
			hart_caps.aia_enabled = true;
			build_processor_properties_node(
				table_data,
				cpu->env.misa,
				hart_caps.hart_cap);
			ppts_priv_rsrc[0] = hart_ppts_start;
			build_processor_hierarchy_node(
				table_data,
				(1 << 1) | /* ACPI 6.2 - ACPI Processor ID valid */
				(1 << 3),  /* ACPI 6.3 - Node is a Leaf */
				socket_offset, acpi_proc_id++, ppts_priv_rsrc, 1);
		}
	}

	build_header(linker, table_data, (void *)(table_data->data + pptt_start),
                 "PPTT", table_data->len - pptt_start, 2, vms->oem_id,
                 vms->oem_table_id);
}

/* DSDT */
static void
build_dsdt(GArray *table_data, BIOSLinker *linker, RISCVVirtState *vms)
{
    Aml *scope, *dsdt;
    const MemMapEntry *memmap = vms->memmap;

    dsdt = init_aml_allocator();
    /* Reserve space for header */
    acpi_data_push(dsdt->buf, sizeof(AcpiTableHeader));

    /* When booting the VM with UEFI, UEFI takes ownership of the RTC hardware.
     * While UEFI can use libfdt to disable the RTC device node in the DTB that
     * it passes to the OS, it cannot modify AML. Therefore, we won't generate
     * the RTC ACPI device at all when using UEFI.
     */
    scope = aml_scope("\\_SB");
    acpi_dsdt_add_cpus(scope, vms);

    acpi_dsdt_add_fw_cfg(scope, &memmap[VIRT_FW_CFG]);
    acpi_dsdt_add_aplic(scope, vms);
    acpi_dsdt_add_virtio(scope, &memmap[VIRT_VIRTIO],
                    (VIRTIO_IRQ), VIRTIO_COUNT);
    acpi_dsdt_add_pci(scope, memmap, PCIE_IRQ,vms);
    acpi_dsdt_add_uart(scope, &memmap[VIRT_UART0],
                       (UART0_IRQ));

    aml_append(dsdt, scope);

    /* copy AML table into ACPI tables blob and patch header there */
    g_array_append_vals(table_data, dsdt->buf->data, dsdt->buf->len);
    build_header(linker, table_data,
        (void *)(table_data->data + table_data->len - dsdt->buf->len),
                 "DSDT", dsdt->buf->len, 2, vms->oem_id,
                 vms->oem_table_id);
    free_aml_allocator();
}

typedef
struct AcpiBuildState {
    /* Copy of table in RAM (for patching). */
    MemoryRegion *table_mr;
    MemoryRegion *rsdp_mr;
    MemoryRegion *linker_mr;
    /* Is table patched? */
    bool patched;
} AcpiBuildState;

static void acpi_align_size(GArray *blob, unsigned align)
{
    /*
     * Align size to multiple of given size. This reduces the chance
     * we need to change size in the future (breaking cross version migration).
     */
    g_array_set_size(blob, ROUND_UP(acpi_data_len(blob), align));
}

static
void virt_acpi_build(RISCVVirtState *vms, AcpiBuildTables *tables)
{
    GArray *table_offsets;
    unsigned dsdt, xsdt;
    GArray *tables_blob = tables->table_data;
    int socket;
    MachineState *ms = MACHINE(vms);

    table_offsets = g_array_new(false, true /* clear */,
                                        sizeof(uint32_t));

    bios_linker_loader_alloc(tables->linker,
                             ACPI_BUILD_TABLE_FILE, tables_blob,
                             64, false /* high memory */);
    for (socket = 0; socket < riscv_socket_count(ms); socket++) {
        if (imsic_max_hart_per_socket < vms->soc[socket].num_harts) {
            imsic_max_hart_per_socket = vms->soc[socket].num_harts;
        }
    }

    /* DSDT is pointed to by FADT */
    dsdt = tables_blob->len;
    build_dsdt(tables_blob, tables->linker, vms);

    /* FADT MADT RTDT MCFG SPCR pointed to by RSDT */
    acpi_add_table(table_offsets, tables_blob);
    build_fadt_rev5(tables_blob, tables->linker, vms, dsdt);

    acpi_add_table(table_offsets, tables_blob);
    build_madt(tables_blob, tables->linker, vms);

    acpi_add_table(table_offsets, tables_blob);
    build_rtdt(tables_blob, tables->linker, vms);

    acpi_add_table(table_offsets, tables_blob);
    build_pptt(tables_blob, tables->linker, vms);
    acpi_add_table(table_offsets, tables_blob);
    {
        AcpiMcfgInfo mcfg = {
           .base = vms->memmap[VIRT_PCIE_ECAM].base,
           .size = vms->memmap[VIRT_PCIE_ECAM].size,
        };
        build_mcfg(tables_blob, tables->linker, &mcfg, vms->oem_id,
                   vms->oem_table_id);
    }

    /* XSDT is pointed to by RSDP */
    xsdt = tables_blob->len;
    build_xsdt(tables_blob, tables->linker, table_offsets, vms->oem_id,
               vms->oem_table_id);

    /* RSDP is in FSEG memory, so allocate it separately */
    {
        AcpiRsdpData rsdp_data = {
            .revision = 2,
            .oem_id = vms->oem_id,
            .xsdt_tbl_offset = &xsdt,
            .rsdt_tbl_offset = NULL,
        };
        build_rsdp(tables->rsdp, tables->linker, &rsdp_data);
    }

    /*
     * The align size is 128, warn if 64k is not enough therefore
     * the align size could be resized.
     */
    if (tables_blob->len > ACPI_BUILD_TABLE_SIZE / 2) {
        warn_report("ACPI table size %u exceeds %d bytes,"
                    " migration may not work",
                    tables_blob->len, ACPI_BUILD_TABLE_SIZE / 2);
        error_printf("Try removing CPUs, NUMA nodes, memory slots"
                     " or PCI bridges.");
    }
    acpi_align_size(tables_blob, ACPI_BUILD_TABLE_SIZE);


    /* Cleanup memory that's no longer used. */
    g_array_free(table_offsets, true);
}

static void acpi_ram_update(MemoryRegion *mr, GArray *data)
{
    uint32_t size = acpi_data_len(data);

    /* Make sure RAM size is correct - in case it got changed
     * e.g. by migration */
    memory_region_ram_resize(mr, size, &error_abort);

    memcpy(memory_region_get_ram_ptr(mr), data->data, size);
    memory_region_set_dirty(mr, 0, size);
}

static void virt_acpi_build_update(void *build_opaque)
{
    AcpiBuildState *build_state = build_opaque;
    AcpiBuildTables tables;

    /* No state to update or already patched? Nothing to do. */
    if (!build_state || build_state->patched) {
        return;
    }
    build_state->patched = true;

    acpi_build_tables_init(&tables);

    virt_acpi_build(RISCV_VIRT_MACHINE(qdev_get_machine()), &tables);

    acpi_ram_update(build_state->table_mr, tables.table_data);
    acpi_ram_update(build_state->rsdp_mr, tables.rsdp);
    acpi_ram_update(build_state->linker_mr, tables.linker->cmd_blob);

    acpi_build_tables_cleanup(&tables, true);
}

static void virt_acpi_build_reset(void *build_opaque)
{
    AcpiBuildState *build_state = build_opaque;
    build_state->patched = false;
}

void virt_acpi_setup(RISCVVirtState *vms)
{
    AcpiBuildTables tables;
    AcpiBuildState *build_state;

    build_state = g_malloc0(sizeof *build_state);

    acpi_build_tables_init(&tables);
    virt_acpi_build(vms, &tables);

    /* Now expose it all to Guest */
    build_state->table_mr = acpi_add_rom_blob(virt_acpi_build_update,
                                              build_state, tables.table_data,
                                              ACPI_BUILD_TABLE_FILE);
    assert(build_state->table_mr != NULL);

    build_state->linker_mr = acpi_add_rom_blob(virt_acpi_build_update,
                                               build_state,
                                               tables.linker->cmd_blob,
                                               ACPI_BUILD_LOADER_FILE);

    build_state->rsdp_mr = acpi_add_rom_blob(virt_acpi_build_update,
                                             build_state, tables.rsdp,
                                             ACPI_BUILD_RSDP_FILE);

    qemu_register_reset(virt_acpi_build_reset, build_state);
    virt_acpi_build_reset(build_state);

    /* Cleanup tables but don't free the memory: we track it
     * in build_state.
     */
    acpi_build_tables_cleanup(&tables, false);
}
