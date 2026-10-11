/* SPDX-License-Identifier: GPL-2.0-only */
/* Bounded parser for the Motorola bootloader's first WLAN address. */
#ifndef CAPRI_WIFI_MAC_H
#define CAPRI_WIFI_MAC_H

static int __init capri_mac_hex(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static int __init capri_arg_space(unsigned char c)
{
	return c == ' ' || (c >= '\t' && c <= '\r');
}

/* Require a bounded, NUL-terminated property and one unambiguous argument.
 * Motorola can append a second interface address after a comma. Preserve the
 * first interface's address, as the existing userspace startup helper does.
 * Invalid or absent input leaves the caller's output unchanged.
 */
static int __init capri_parse_wifi_mac(const char *args, size_t len,
				     unsigned char mac[6])
{
	static const char key[] = "wifimacaddr=";
	const char *end, *p, *next, *value;
	unsigned char parsed[6];
	int found = 0, nonzero = 0, i, hi, lo;

	if (!args || !len)
		return 0;
	end = memchr(args, '\0', len);
	if (!end)
		return 0;
	for (p = args; p < end; p = next) {
		while (p < end && capri_arg_space(*p))
			p++;
		next = p;
		while (next < end && !capri_arg_space(*next))
			next++;
		if ((size_t)(next - p) < sizeof(key) - 1 ||
		    memcmp(p, key, sizeof(key) - 1))
			continue;
		if (found++)
			return 0;
		value = p + sizeof(key) - 1;
		if (next - value < 17 ||
		    (next - value > 17 && value[17] != ','))
			return 0;
		for (i = 0; i < 6; i++) {
			hi = capri_mac_hex(value[3 * i]);
			lo = capri_mac_hex(value[3 * i + 1]);
			if (hi < 0 || lo < 0 || (i < 5 && value[3 * i + 2] != ':'))
				return 0;
			parsed[i] = (hi << 4) | lo;
			nonzero |= parsed[i];
		}
		if (!nonzero || (parsed[0] & 1))
			return 0;
	}
	if (!found)
		return 0;
	memcpy(mac, parsed, sizeof(parsed));
	return 1;
}

#endif
