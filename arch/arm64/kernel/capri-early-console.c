// SPDX-License-Identifier: GPL-2.0-only
/* Early framebuffer console for Motorola Moto G10.
 * The mapping lifecycle follows drivers/firmware/efi/earlycon.c.
 */
#include <linux/console.h>
#include <linux/font.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <asm/early_ioremap.h>

#define CAPRI_FB_BASE 0x5c000000
#define CAPRI_FB_STRIDE (720 * 4)
#define CAPRI_FB_SIZE (CAPRI_FB_STRIDE * 1600)
#define CAPRI_LINE_SIZE (CAPRI_FB_STRIDE * 16)

static void __iomem *capri_fb;
static unsigned int capri_x, capri_y = 2;
static struct console capri_console;

static void __ref capri_write(struct console *con, const char *s, unsigned int n)
{
 /* machine_emergency_restart() changes system_state before printing.
  * early_ioremap() then WARNs, recursively calling this console forever.
  * Keep the original panic visible instead of issuing illegal late maps.
  */
 if (!capri_fb && system_state >= SYSTEM_RUNNING) return;
 while (n) {
  unsigned int off = capri_y * CAPRI_LINE_SIZE;
  u32 __iomem *line = capri_fb ? capri_fb + off :
   early_ioremap(CAPRI_FB_BASE + off, CAPRI_LINE_SIZE);
  if (!line) return;
  if (!capri_x) memset_io(line, 0, CAPRI_LINE_SIZE);
  while (n && capri_x < 90) {
   unsigned char c = *s++;
   unsigned int y, x;
   n--;
   if (c == '\n') { capri_x = 90; break; }
   if (c == '\r') { capri_x = 0; continue; }
   for (y = 0; y < 16; y++) {
    unsigned char bits = font_vga_8x16.data[c * 16 + y];
    for (x = 0; x < 8; x++)
     writel_relaxed((bits & (0x80 >> x)) ? 0xffffffff : 0xff000000,
                    line + y * 720 + capri_x * 8 + x);
   }
   capri_x++;
  }
  if (!capri_fb) early_iounmap(line, CAPRI_LINE_SIZE);
  if (capri_x >= 90) {
   capri_x = 0;
   if (++capri_y >= 98) capri_y = 2;
  }
 }
}

static struct console capri_console = {
 .name = "caprifb", .write = capri_write,
 .flags = CON_BOOT | CON_PRINTBUFFER, .index = -1,
};

void __init capri_early_console_init(void);
void __init capri_early_console_init(void)
{
 register_console(&capri_console);
 pr_info("Capri early framebuffer console: before firmware DT replacement\n");
}

static int __init capri_early_console_remap(void)
{
 capri_fb = ioremap(CAPRI_FB_BASE, CAPRI_FB_SIZE);
 if (!capri_fb) unregister_console(&capri_console);
 return capri_fb ? 0 : -ENOMEM;
}
early_initcall(capri_early_console_remap);

static int __init capri_early_console_stop(void)
{
 unregister_console(&capri_console);
 if (capri_fb) iounmap(capri_fb);
 capri_fb = NULL;
 return 0;
}
late_initcall(capri_early_console_stop);
