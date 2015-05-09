/*
 * Copyright (c) 2015, Liang Li <liliang6@me.com>.
 *
 * The right to copy, distribute, modify, or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable license agreement.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <modbus/modbus.h>

#include "lib_sensor.h"

const char *default_cfg = "mb_sensor.json";
void usage()
{
	printf("Usage: [-h] [-f] [-c <configuration-file>]\n\n"
			"Options:\n"
			"  -f Run in foreground.\n"
			"  -c Configuration file.\n"
			"  -h Print this Help\n");
}

static modbus_t * init_mb_serial(const char * serial_dev, uint8_t slave_addr)
{
    modbus_t *ctx;

    ctx = modbus_new_rtu(serial_dev, 9600, 'N', 8, 1);

    if (ctx == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return NULL;
    }
    modbus_set_debug(ctx, FALSE);
    modbus_set_error_recovery(ctx,
                              MODBUS_ERROR_RECOVERY_LINK |
                              MODBUS_ERROR_RECOVERY_PROTOCOL);

    modbus_set_slave(ctx, 0xFF);

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return NULL;
    }

    modbus_set_slave(ctx, slave_addr);

    return ctx;
}

static void clean_mb_ctx(modbus_t *ctx)
{
    modbus_close(ctx);
    modbus_free(ctx);
}

static int get_mb_reg(modbus_t *ctx, int reg_addr, int data_len, void *data)
{
    int rc, ret = -1;

    for (;;) {
        rc = modbus_read_registers(ctx, reg_addr, data_len, data);
        if (rc != data_len) {
            break;
        } else {
            ret = 0;
            /**
             * If do not add the below usleep and break, it also works, but
             * would take more time hence slower on communication.
             */
            usleep(3000);
            break;
        }
    }

    return ret;
}

/*
 * Get sensor data according to sensor index and data source index
 */
double get_datapoint_data(void *props)
{
	const unsigned char *protocol = get_string_by_name(props, "protocolType");
	if (strcmp(protocol, "ModBus") != 0) {
		return 0;
	}

	const char *mb_device = get_string_by_name(props, "device");
	int mb_addr = strtol(get_string_by_name(props, "slave"), NULL, 0);
	int mb_reg_data_len = strtol(get_string_by_name(props, "data_len"), NULL, 0);
	int mb_reg_addr = strtol(get_string_by_name(props, "reg"), NULL, 0);

	double divisor = strtol(get_string_by_name(props, "div"), NULL, 0);
	if (divisor == 0) {
		// fatal error, divisor should never being ZERO
		return 0;
	}
	int    mb_reg_shift = 0;
	double mb_reg_pad = 0;

	if (get_node_by_name(props, "shift") != NULL) {
		mb_reg_shift = strtol(get_string_by_name(props, "shift"), NULL, 0);
	}
	if (get_node_by_name(props, "pad") != NULL) {
		mb_reg_pad = strtol(get_string_by_name(props, "pad"), NULL, 0);
	}

#if defined(MB_DEBUG)
	printf("mb_device: %s, mb_addr: %d\n", mb_device, mb_addr);
	printf("mb_reg_data_len: %d, mb_reg_addr: %d\n", mb_reg_data_len, mb_reg_addr);
	printf("div: %lf, shift: %d, pad: %lf\n", divisor, mb_reg_shift, mb_reg_pad);
#endif

	double ret = 0;
	int rc = -1;

	uint16_t *_data = malloc(sizeof(uint16_t) * mb_reg_data_len);
	if (_data == NULL) {
		perror("alloc memory error.");
		ret = -1;
		goto out_clean;
	}

	modbus_t *ctx = NULL;
	int retry_read = 0;

mb_read_once:
	ctx = init_mb_serial(mb_device, mb_addr);
	if (ctx == NULL) {
		perror("modbus context error.");
		ret = -1;
		goto out_clean;
	}

	rc = get_mb_reg(ctx, mb_reg_addr, mb_reg_data_len, _data);

	if (rc == 0) {
		clean_mb_ctx(ctx);
		if (mb_reg_data_len == 4) {
			ret = (*((uint32_t *)(&_data[mb_reg_shift])) + mb_reg_pad) / divisor;
		} else {
			ret = (_data[mb_reg_shift] + mb_reg_pad) / divisor;
		}
	} else {
		/* get data failed. */
		clean_mb_ctx(ctx);
		if (retry_read++ < 6) {
			goto mb_read_once;
		}
	}

out_clean:
	if (_data != NULL)
		free(_data);

	return ret;
}


int main(int argc, char *argv[])
{
	int ret, ch = 0;
	char * config_file = default_cfg;
	int foreground = 0;

	/* command line arguments processing */
	while ((ch=getopt(argc, argv, "hc:f")) != EOF) {
		switch (ch) {
			case 'h':
				usage();
				exit(0);
			case 'c':
				config_file = optarg;
				break;
			case 'f':
				foreground = 1;
				break;
			default:
				usage();
				exit(1);
		}
	}

	/**
	 * If not in foreground mode, run as a daemon
	 */
	if (!foreground) {
		pid_t pid = fork();
		if(pid == -1) {
			perror("fork");
			exit(-1);
		} else if (pid > 0) {
			exit(0);
		}
	}

	lib_sensor_start(config_file, get_datapoint_data, NULL, NULL);

	printf("sensor app is terminated!\n");
	return 0;
}
