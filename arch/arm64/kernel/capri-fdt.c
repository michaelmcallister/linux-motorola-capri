// SPDX-License-Identifier: GPL-2.0-only
/* Motorola Moto G10 header-v3 bootloader compatibility.
 * Retain the firmware RAM map and initramfs while selecting the board DT.
 */
#include <linux/init.h>
#include <linux/cache.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <asm/memory.h>
#include <generated/capri-dtb.h>
#include "capri-wifi-mac.h"

static unsigned char capri_fdt_buffer[SZ_128K] __ro_after_init __aligned(8);

void * __init capri_prepare_fdt(const void *firmware, phys_addr_t *phys, int *size);

static void __init copy_prop(void *dst, int dn, const void *src, int sn,
                             const char *name, bool required)
{
 int len, ret;
 const void *value = fdt_getprop(src, sn, name, &len);
 if (!value) {
  if (required) panic("Capri FDT: required property %s missing", name);
  return;
 }
 ret = fdt_setprop(dst, dn, name, value, len);
 if (ret) panic("Capri FDT: copy %s failed: %d", name, ret);
}

static void __init capri_copy_wifi_mac(void *fdt, const void *firmware, int chosen)
{
 const char *args;
 unsigned char mac[6];
 int len, wifi, ret;

 args = fdt_getprop(firmware, chosen, "bootargs", &len);
 if (!args || len <= 0 || !capri_parse_wifi_mac(args, len, mac)) {
  pr_warn("Capri FDT: no valid bootloader Wi-Fi address; retaining driver fallback\n");
  return;
 }
 wifi = fdt_node_offset_by_compatible(fdt, -1, "qcom,wcn3990-wifi");
 if (wifi < 0) {
  pr_warn("Capri FDT: Wi-Fi node missing; retaining driver fallback\n");
  return;
 }
 ret = fdt_setprop(fdt, wifi, "local-mac-address", mac, sizeof(mac));
 if (ret)
  pr_warn("Capri FDT: could not retain Wi-Fi address: %d\n", ret);
 else
  pr_info("Capri: bootloader Wi-Fi address retained in device tree\n");
}

void * __init capri_prepare_fdt(const void *firmware, phys_addr_t *phys, int *size)
{
 int node, chosen, dest, memory = -1, ret;
 const char *type;
 void *fdt = capri_fdt_buffer;

 if (!firmware || fdt_check_header(firmware))
  panic("Capri FDT: invalid firmware DT");
 if (fdt_node_check_compatible(firmware, 0, "qcom,bengal-capri"))
  panic("Capri FDT: unexpected board, refusing local DT override");
 if (fdt_address_cells(firmware, 0) != 2 || fdt_size_cells(firmware, 0) != 2)
  panic("Capri FDT: unexpected memory cell format");
 ret = fdt_open_into(capri_dtb, fdt, sizeof(capri_fdt_buffer));
 if (ret) panic("Capri FDT: open failed: %d", ret);
 fdt_for_each_subnode(node, firmware, 0) {
  type = fdt_getprop(firmware, node, "device_type", NULL);
  if (type && !strcmp(type, "memory")) {
   if (memory >= 0) panic("Capri FDT: multiple RAM nodes need explicit handling");
   memory = node;
  }
 }
 if (memory < 0) panic("Capri FDT: firmware has no RAM map");
 dest = fdt_path_offset(fdt, "/memory@80000000");
 if (dest < 0) panic("Capri FDT: missing target RAM node");
 copy_prop(fdt, dest, firmware, memory, "reg", true);
 chosen = fdt_path_offset(firmware, "/chosen");
 dest = fdt_path_offset(fdt, "/chosen");
 if (chosen < 0 || dest < 0) panic("Capri FDT: missing chosen node");
 copy_prop(fdt, dest, firmware, chosen, "linux,initrd-start", true);
 copy_prop(fdt, dest, firmware, chosen, "linux,initrd-end", true);
 copy_prop(fdt, dest, firmware, chosen, "bootargs", false);
 capri_copy_wifi_mac(fdt, firmware, chosen);
 /* CONFIG_CMDLINE_FORCE selects Linux boot arguments instead of Android init arguments. */
 fdt_pack(fdt);
 *phys = __pa_symbol(capri_fdt_buffer);
 *size = fdt_totalsize(fdt);
 pr_info("Capri: mainline DT, firmware RAM map and initramfs retained\n");
 return fdt;
}
