/*
 * XWiimote - tools - xwiishow
 * Written 2010-2013 by David Herrmann
 * Dedicated to the Public Domain
 */

/*
 * Interactive Wiimote Testing Tool
 * If you run this tool without arguments, then it shows usage information. If
 * you pass "list" as first argument, it lists all connected Wii Remotes.
 * You need to pass one path as argument and the given wiimote is opened and
 * printed to the screen. When wiimote events are received, then the screen is
 * updated correspondingly. You can use the keyboard to control the wiimote.
 */

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xwiimote.h"

enum window_mode {
	MODE_ERROR,
	MODE_NORMAL,
	MODE_EXTENDED,
};

static struct xwii_iface *iface;
static unsigned int mode = MODE_NORMAL;
static bool freeze = false;

/* error messages */

static void print_info(const char *format, ...) {
	va_list list;
	char str[58 + 1];

	va_start(list, format);
	vsnprintf(str, sizeof(str), format, list);
	str[sizeof(str) - 1] = 0;
	va_end(list);

	//mvprintw(22, 22, "                                                          ");
	//mvprintw(22, 22, "%s", str);
}

static void print_error(const char *format, ...) {
	va_list list;
	char str[58 + 80 + 1];

	va_start(list, format);
	vsnprintf(str, sizeof(str), format, list);
	if (mode == MODE_EXTENDED)
		str[sizeof(str) - 1] = 0;
	else
		str[58] = 0;
	va_end(list);

	//mvprintw(23, 22, "                                                          ");
	//if (mode == MODE_EXTENDED)
	//mvprintw(23, 80, "                                                                                ");
	//mvprintw(23, 22, "%s", str);
}

/* key events */

static void key_show(const struct xwii_event *event) {
	unsigned int code = event->v.key.code;
	bool pressed = event->v.key.state;
	char *str = NULL;

	if (pressed)
		str = "X";
	else
		str = " ";
	if (code == XWII_KEY_LEFT) {
		printf("Left\n");
	} else if (code == XWII_KEY_RIGHT) {
		printf("Right\n");
	} else if (code == XWII_KEY_UP) {
		printf("Up\n");
	} else if (code == XWII_KEY_DOWN) {
		printf("Down\n");
	} else if (code == XWII_KEY_A) {
		printf("A\n");
	} else if (code == XWII_KEY_B) {
		if (pressed)
			str = "B";
		//mvprintw(10, 13, "%s", str);
	} else if (code == XWII_KEY_HOME) {
		if (pressed)
			str = "HOME+";
		else
			str = "     ";
		//mvprintw(13, 7, "%s", str);
	} else if (code == XWII_KEY_MINUS) {
		if (pressed)
			str = "-";
		//mvprintw(13, 3, "%s", str);
	} else if (code == XWII_KEY_PLUS) {
		if (pressed)
			str = "+";
		//mvprintw(13, 15, "%s", str);
	} else if (code == XWII_KEY_ONE) {
		if (pressed)
			str = "1";
		//mvprintw(20, 9, "%s", str);
	} else if (code == XWII_KEY_TWO) {
		if (pressed)
			str = "2";
		//mvprintw(21, 9, "%s", str);
	}
}

static void key_clear(void) {
	struct xwii_event ev;
	unsigned int i;

	ev.type = XWII_EVENT_KEY;
	for (i = 0; i < XWII_KEY_NUM; ++i) {
		ev.v.key.code = i;
		ev.v.key.state = 0;
		key_show(&ev);
	}
}

static void key_toggle(void) {
	int ret;

	if (xwii_iface_opened(iface) & XWII_IFACE_CORE) {
		xwii_iface_close(iface, XWII_IFACE_CORE);
		key_clear();
		print_info("Info: Disable key events");
	} else {
		ret = xwii_iface_open(iface, XWII_IFACE_CORE |
									 XWII_IFACE_WRITABLE);
		if (ret)
			print_error("Error: Cannot enable key events: %d", ret);
		else
			print_info("Info: Enable key events");
	}
}

/* accelerometer events */

static void accel_show_ext_x(double val) {
}

static void accel_show_ext_y(double val) {
}

static void accel_show_ext_z(double val) {
}

static void accel_show_ext(const struct xwii_event *event) {
	double val;

	/* pow(val, 1/4) for smoother interpolation around the origin */

	val = event->v.abs[0].x;
	val /= 512;
	if (val >= 0)
		val = 10 * pow(val, 0.25);
	else
		val = -10 * pow(-val, 0.25);
	accel_show_ext_x(val);

	val = event->v.abs[0].z;
	val /= 512;
	if (val >= 0)
		val = 5 * pow(val, 0.25);
	else
		val = -5 * pow(-val, 0.25);
	accel_show_ext_z(val);

	val = event->v.abs[0].y;
	val /= 512;
	if (val >= 0)
		val = 5 * pow(val, 0.25);
	else
		val = -5 * pow(-val, 0.25);
	accel_show_ext_y(val);
}

static void accel_show(const struct xwii_event *event) {
	//mvprintw(1, 39, "%5" PRId32, event->v.abs[0].x);
	//mvprintw(1, 48, "%5" PRId32, event->v.abs[0].y);
	//mvprintw(1, 57, "%5" PRId32, event->v.abs[0].z);
}

static void accel_clear(void) {
	struct xwii_event ev;

	ev.v.abs[0].x = 0;
	ev.v.abs[0].y = 0;
	ev.v.abs[0].z = 0;
	accel_show_ext(&ev);
	accel_show(&ev);
}

static void accel_toggle(void) {
	int ret;

	if (xwii_iface_opened(iface) & XWII_IFACE_ACCEL) {
		xwii_iface_close(iface, XWII_IFACE_ACCEL);
		accel_clear();
		print_info("Info: Disable accelerometer");
	} else {
		ret = xwii_iface_open(iface, XWII_IFACE_ACCEL);
		if (ret)
			print_error("Error: Cannot enable accelerometer: %d",
						ret);
		else
			print_info("Info: Enable accelerometer");
	}
}
/* nunchuk */

static void nunchuk_show_ext_x(double val) {
}

static void nunchuk_show_ext_y(double val) {
}

static void nunchuk_show_ext_z(double val) {
}

static void nunchuk_show_ext(const struct xwii_event *event) {
	double val;
	const char *str = " ";
	int32_t v;

	if (event->type == XWII_EVENT_NUNCHUK_MOVE) {
		/* pow(val, 1/4) for smoother interpolation around the origin */

		val = event->v.abs[1].x;
		val /= 512;
		if (val >= 0)
			val = 10 * pow(val, 0.25);
		else
			val = -10 * pow(-val, 0.25);
		nunchuk_show_ext_x(val);

		val = event->v.abs[1].z;
		val /= 512;
		if (val >= 0)
			val = 5 * pow(val, 0.25);
		else
			val = -5 * pow(-val, 0.25);
		nunchuk_show_ext_z(val);

		val = event->v.abs[1].y;
		val /= 512;
		if (val >= 0)
			val = 5 * pow(val, 0.25);
		else
			val = -5 * pow(-val, 0.25);
		nunchuk_show_ext_y(val);

		v = event->v.abs[0].x * 12;
		//mvprintw(26, 24, "%5d", v);
		if (v > 1000) {
			//mvprintw(28, 26, "     ");
			//mvprintw(28, 32, "#####");
		} else if (v > 800) {
			//mvprintw(28, 26, "     ");
			//mvprintw(28, 32, "#### ");
		} else if (v > 600) {
			//mvprintw(28, 26, "     ");
			//mvprintw(28, 32, "###  ");
		} else if (v > 400) {
			//mvprintw(28, 26, "     ");
			//mvprintw(28, 32, "##   ");
		} else if (v > 200) {
			//mvprintw(28, 26, "     ");
			//mvprintw(28, 32, "#    ");
		} else if (v > -200) {
			//mvprintw(28, 26, "     ");
			//mvprintw(28, 32, "     ");
		} else if (v > -400) {
			//mvprintw(28, 26, "    #");
			//mvprintw(28, 32, "     ");
		} else if (v > -600) {
			//mvprintw(28, 26, "   ##");
			//mvprintw(28, 32, "     ");
		} else if (v > -800) {
			//mvprintw(28, 26, "  ###");
			//mvprintw(28, 32, "     ");
		} else if (v > -1000) {
			//mvprintw(28, 26, " ####");
			//mvprintw(28, 32, "     ");
		} else {
			//mvprintw(28, 26, "#####");
			//mvprintw(28, 32, "     ");
		}

		v = event->v.abs[0].y * 12;
		//mvprintw(26, 33, "%5d", v);
		if (v > 1000) {
			//mvprintw(26, 31, "#");
			//mvprintw(27, 31, "#");
			//mvprintw(29, 31, " ");
			//mvprintw(30, 31, " ");
		} else if (v > 200) {
			//mvprintw(26, 31, " ");
			//mvprintw(27, 31, "#");
			//mvprintw(29, 31, " ");
			//mvprintw(30, 31, " ");
		} else if (v > -200) {
			//mvprintw(26, 31, " ");
			//mvprintw(27, 31, " ");
			//mvprintw(29, 31, " ");
			//mvprintw(30, 31, " ");
		} else if (v > -1000) {
			//mvprintw(26, 31, " ");
			//mvprintw(27, 31, " ");
			//mvprintw(29, 31, "#");
			//mvprintw(30, 31, " ");
		} else {
			//mvprintw(26, 31, " ");
			//mvprintw(27, 31, " ");
			//mvprintw(29, 31, "#");
			//mvprintw(30, 31, "#");
		}
	}

	if (event->type == XWII_EVENT_NUNCHUK_KEY) {
		if (event->v.key.code == XWII_KEY_C) {
			if (event->v.key.state)
				str = "C";
			//mvprintw(37, 6, "%s", str);
		} else if (event->v.key.code == XWII_KEY_Z) {
			if (event->v.key.state)
				str = "Z";
			//mvprintw(37, 18, "%s", str);
		}
	}
}

static void nunchuk_clear(void) {
	struct xwii_event ev;

	ev.type = XWII_EVENT_NUNCHUK_MOVE;
	ev.v.abs[0].x = 0;
	ev.v.abs[0].y = 0;
	ev.v.abs[1].x = 0;
	ev.v.abs[1].y = 0;
	ev.v.abs[1].z = 0;
	nunchuk_show_ext(&ev);

	ev.type = XWII_EVENT_NUNCHUK_KEY;
	ev.v.key.state = 0;
	ev.v.key.code = XWII_KEY_C;
	nunchuk_show_ext(&ev);
	ev.v.key.code = XWII_KEY_Z;
	nunchuk_show_ext(&ev);
}

static void nunchuk_toggle(void) {
	int ret;

	if (xwii_iface_opened(iface) & XWII_IFACE_NUNCHUK) {
		xwii_iface_close(iface, XWII_IFACE_NUNCHUK);
		nunchuk_clear();
		print_info("Info: Disable Nunchuk");
	} else {
		ret = xwii_iface_open(iface, XWII_IFACE_NUNCHUK);
		if (ret)
			print_error("Error: Cannot enable Nunchuk: %d",
						ret);
		else
			print_info("Info: Enable Nunchuk");
	}
}

/* rumble events */

static void rumble_show(bool on) {
	//mvprintw(1, 21, on ? "RUMBLE" : "      ");
}

static void rumble_toggle(void) {
	static bool on = false;
	int ret;

	on = !on;
	ret = xwii_iface_rumble(iface, on);
	if (ret) {
		print_error("Error: Cannot toggle rumble motor: %d", ret);
		on = !on;
	}

	rumble_show(on);
}

/* LEDs */

static bool led_state[4];

static void led_show(int n, bool on) {
	//mvprintw(5, 59 + n*5, on ? "(#%i)" : " -%i ", n+1);
}

static void led_toggle(int n) {
	int ret;

	led_state[n] = !led_state[n];
	ret = xwii_iface_set_led(iface, XWII_LED(n + 1), led_state[n]);
	if (ret) {
		print_error("Error: Cannot toggle LED %i: %d", n + 1, ret);
		led_state[n] = !led_state[n];
	}
	led_show(n, led_state[n]);
}

static void led_refresh(int n) {
	int ret;

	ret = xwii_iface_get_led(iface, XWII_LED(n + 1), &led_state[n]);
	if (ret)
		print_error("Error: Cannot read LED state");
	else
		led_show(n, led_state[n]);
}

/* battery status */

static void battery_show(uint8_t capacity) {
	int i;

	//mvprintw(7, 29, "%3u%%", capacity);

	//mvprintw(7, 35, "          ");;
	for (i = 0; i * 10 < capacity; ++i);
	//mvprintw(7, 35 + i, "#");
}

static void battery_refresh(void) {
	int ret;
	uint8_t capacity;

	ret = xwii_iface_get_battery(iface, &capacity);
	if (ret)
		print_error("Error: Cannot read battery capacity");
	else
		battery_show(capacity);
}

/* device type */

static void devtype_refresh(void) {
	int ret;
	char *name;

	ret = xwii_iface_get_devtype(iface, &name);
	if (ret) {
		print_error("Error: Cannot read device type");
	} else {
		//mvprintw(9, 28, "                                                   ");
		//mvprintw(9, 28, "%s", name);
		free(name);
	}
}

/* extension type */

static void extension_refresh(void) {
	int ret;
	char *name;

	ret = xwii_iface_get_extension(iface, &name);
	if (ret) {
		print_error("Error: Cannot read extension type");
	} else {
		//mvprintw(7, 54, "                      ");
		//mvprintw(7, 54, "%s", name);
		free(name);
	}
	//mvprintw(7, 77, "  ");
}

/* device watch events */

static void handle_watch(void) {
	static unsigned int num;
	int ret;

	print_info("Info: Watch Event #%u", ++num);

	ret = xwii_iface_open(iface, xwii_iface_available(iface) |
								 XWII_IFACE_WRITABLE);
	if (ret)
		print_error("Error: Cannot open interface: %d", ret);

}

static int run_iface(struct xwii_iface *iface) {
	struct xwii_event event;
	int ret = 0, fds_num;
	struct pollfd fds[2];

	memset(fds, 0, sizeof(fds));
	fds[0].fd = 0;
	fds[0].events = POLLIN;
	fds[1].fd = xwii_iface_get_fd(iface);
	fds[1].events = POLLIN;
	fds_num = 2;

	ret = xwii_iface_watch(iface, true);
	if (ret)
		print_error("Error: Cannot initialize hotplug watch descriptor");

	while (true) {
		ret = poll(fds, fds_num, -1);
		if (ret < 0) {
			if (errno != EINTR) {
				ret = -errno;
				print_error("Error: Cannot poll fds: %d", ret);
				break;
			}
		}

		ret = xwii_iface_dispatch(iface, &event, sizeof(event));
		if (ret) {
			if (ret != -EAGAIN) {
				print_error("Error: Read failed with err:%d",
							ret);
				break;
			}
		} else if (!freeze) {
			switch (event.type) {
				case XWII_EVENT_GONE:					//Needed
					print_info("Info: Device gone");
					fds[1].fd = -1;
					fds[1].events = 0;
					fds_num = 1;
					break;
				case XWII_EVENT_WATCH:					//Needed
					handle_watch();
					break;
				case XWII_EVENT_KEY:
					key_show(&event);
					break;
				case XWII_EVENT_NUNCHUK_KEY:
				case XWII_EVENT_NUNCHUK_MOVE:
					if (mode == MODE_EXTENDED)
						nunchuk_show_ext(&event);
					break;
			}
		}
	}

	return ret;
}
//Gets devices
static int enumerate() {
	struct xwii_monitor *mon;
	char *ent;
	int num = 0;

	mon = xwii_monitor_new(false, false);
	if (!mon) {
		printf("Cannot create monitor\n");
		return -EINVAL;
	}

	while ((ent = xwii_monitor_poll(mon))) {
		printf("  Found device #%d: %s\n", ++num, ent);
		free(ent);
	}

	xwii_monitor_unref(mon);
	return 0;
}

static char *get_dev(int num) {
	struct xwii_monitor *mon;
	char *ent;
	int i = 0;

	mon = xwii_monitor_new(false, false);
	if (!mon) {
		printf("Cannot create monitor\n");
		return NULL;
	}

	while ((ent = xwii_monitor_poll(mon))) {
		if (++i == num)
			break;
		free(ent);
	}

	xwii_monitor_unref(mon);

	if (!ent)
		printf("Cannot find device with number #%d\n", num);

	return ent;
}

int main(int argc, char **argv) {
	printf("Init");
	int ret = 0;
	char *path = NULL;
	path = get_dev(1);
	ret = xwii_iface_new(&iface, path);
	free(path);
	if (ret) {
		print_error("Cannot create xwii_iface '%s' err:%d\n",
					1, ret);
	} else {
		//refresh();

		ret = xwii_iface_open(iface,
							  xwii_iface_available(iface) |
							  XWII_IFACE_WRITABLE);
		if (ret)
			print_error("Error: Cannot open interface: %d",
						ret);

		ret = run_iface(iface);
		xwii_iface_unref(iface);
		if (ret) {
			print_error("Program failed; press any key to exit");
			//refresh();
		}
	}

}
