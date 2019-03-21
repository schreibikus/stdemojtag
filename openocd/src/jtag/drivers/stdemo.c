/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <sys/ioctl.h>
#include <termios.h>

#define STDEMO_IO_TIMEOUT       5000 /* timeout 5s */

#define SET_JTAG_LED            0x80 /* set LED */
#define SET_JTAG_TCK_TMS_TDI    0x90 /* set TCK TMS TDI */
#define SET_JTAG_TRST_SRST      0xA0 /* set TRST SRST */
#define JTAG_IO_DATA            0xB0 /* full-duplex scan */
#define JTAG_OUT_DATA           0xC0 /* from host to device */
#define SET_JTAG_RUN_TEST       0xD0 /* run test */
#define SET_JTAG_TMS_MOVE       0xE0 /* set TMS move */
#define GET_JTAG_TDO            0xF0 /* get TDO */

/*
 * 8bit command semantic
 *-----------
 *  7 - bit one
 *-----------
 * 6,5,4 - command
 * 0 0 0 - set LED value(bit 1)
 * 0 0 1 - set values TCK(bit 3) TMS(bit 2) TDI(bit 1)
 * 0 1 0 - set values TRST(bit 2) SRST(bit 1)
 * 0 1 1 - recv and send data. bit(3,2,1) - presence flag of length byte
 * 1 0 0 - recv data. bit(3,2,1) - presence flag of length byte
 * 1 0 1 - run test. bit(3,2,1) - presence flag of length byte
 * 1 1 0 - tms move. bit(3,2,1) - length of write in bits
 * 1 1 1 - get value TDO. !!!!! not used !!!!!
 *-----------
 * 3,2,1 - data
 *-----------
 * 0 - parity bit
 *-----------
 */

typedef enum {
	STD_LOW,
	STD_HIGH,
	STD_ERROR
} std_value_t;

#define CLOCK_IDLE() 0

/* configuration */
static char *stdemo_device;
static int stdemo_device_handle = -1;
static uint32_t stdemo_toggling_time_ns = 1000;
static int wait_states;

static unsigned int stdemo_parity(unsigned int v)
{
	/* unsigned int ov = v; */
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	return (0x6996 >> v) & 1;
}

static bool stdemo_raw_write(const uint8_t *buffer, const uint32_t size)
{
	bool result = false;
	struct pollfd fds;
	uint32_t writed;
	int status;

	if(stdemo_device_handle != -1)
	{
		if(buffer && size)
		{
			for(writed = 0; writed < size; )
			{
				memset(&fds, 0, sizeof(fds));
				fds.fd = stdemo_device_handle;
				fds.events = POLLOUT;

				if(poll(&fds, 1, STDEMO_IO_TIMEOUT) > 0)
				{
					if(fds.revents & POLLOUT)
					{
						status = write(stdemo_device_handle, buffer + writed, size - writed);
						if(status > 0)
						{
							writed += status;
						}
						else
						{
							LOG_ERROR("%s %d: Failed write data %d\n", __FUNCTION__, __LINE__, status);
							break;
						}
					}
					else
					{
						LOG_ERROR("%s %d: Unknown state 0x%X\n", __FUNCTION__, __LINE__, fds.revents);
						break;
					}
				}
				else
				{
					LOG_ERROR("%s %d: Write data timeout\n", __FUNCTION__, __LINE__);
					break;
				}
			}

			if(writed == size)
				result = true;
		}
		else
		{
			LOG_ERROR("%s %d: Invalid arguments\n", __FUNCTION__, __LINE__);
		}
	}
	else
	{
		LOG_ERROR("%s %d: Invalid device handle %d\n", __FUNCTION__, __LINE__, stdemo_device_handle);
	}

	return result;
}

static int stdemo_raw_passive_read(uint8_t *buffer, const uint32_t readSize)
{
	int result = 0;
	struct pollfd fds;

	if(stdemo_device_handle != -1)
	{
		if(buffer && readSize)
		{
			memset(&fds, 0, sizeof(fds));
			fds.fd = stdemo_device_handle;
			fds.events = POLLIN;

			if(poll(&fds, 1, 0) > 0)
			{
				if(fds.revents & POLLIN)
				{
					result = read(stdemo_device_handle, buffer, readSize);
					if(result < 0)
					{
						LOG_ERROR("%s %d: Failed read data %d: %s\n", __FUNCTION__, __LINE__, result, strerror(errno));
					}
				}
				else
				{
					LOG_ERROR("%s %d: Unknown state 0x%X\n", __FUNCTION__, __LINE__, fds.revents);
				}
			}
		}
		else
		{
			LOG_ERROR("%s %d: Invalid arguments\n", __FUNCTION__, __LINE__);
		}
	}
	else
	{
		LOG_ERROR("%s %d: Invalid device handle %d\n", __FUNCTION__, __LINE__, stdemo_device_handle);
	}

	return result;
}

static bool stdemo_raw_read(uint8_t *buffer, const uint32_t readSize)
{
	bool result = false;
	struct pollfd fds;
	uint32_t readed;
	int status;

	if(stdemo_device_handle != -1)
	{
		if(buffer && readSize)
		{
			for(readed = 0; readed < readSize; )
			{
				memset(&fds, 0, sizeof(fds));
				fds.fd = stdemo_device_handle;
				fds.events = POLLIN;

				if(poll(&fds, 1, STDEMO_IO_TIMEOUT) > 0)
				{
					if(fds.revents & POLLIN)
					{
						status = read(stdemo_device_handle, buffer + readed, readSize - readed);
						if(status > 0)
						{
							readed += status;
						}
						else
						{
							LOG_ERROR("%s %d: Failed read data %d\n", __FUNCTION__, __LINE__, status);
							break;
						}
					}
					else
					{
						LOG_ERROR("%s %d: Unknown state 0x%X\n", __FUNCTION__, __LINE__, fds.revents);
						break;
					}
				}
				else
				{
					LOG_ERROR("%s %d: Read data timeout\n", __FUNCTION__, __LINE__);
					break;
				}
			}

			if(readed == readSize)
				result = true;
		}
		else
		{
			LOG_ERROR("%s %d: Invalid arguments\n", __FUNCTION__, __LINE__);
		}
	}
	else
	{
		LOG_ERROR("%s %d: Invalid device handle %d\n", __FUNCTION__, __LINE__, stdemo_device_handle);
	}

	return result;
}

static std_value_t stdemo_read(void)
{
	std_value_t result = STD_ERROR;
	uint8_t cmd = GET_JTAG_TDO;

	cmd |= stdemo_parity(cmd);

	if(stdemo_raw_write(&cmd, 1))
	{
		if(stdemo_raw_read(&cmd, 1))
		{
			if(cmd == '1')
				result = STD_HIGH;
			else if(cmd == '0')
				result = STD_LOW;
		}
		else
		{
			LOG_ERROR("%s %d: Failed read data from port\n", __FUNCTION__, __LINE__);
		}
	}
	else
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
	}

	return result;
}

static int stdemo_write(int tck, int tms, int tdi)
{
	int result = ERROR_FAIL;
	uint8_t cmd = SET_JTAG_TCK_TMS_TDI;

	LOG_DEBUG("tck: %d, tms: %d, tdi: %d", tck, tms, tdi);

	if(tck)
		cmd |= 0x08;
	if(tms)
		cmd |= 0x04;
	if(tdi)
		cmd |= 0x02;

	cmd |= stdemo_parity(cmd);
	if(stdemo_raw_write(&cmd, 1))
	{
		result = ERROR_OK;
	}
	else
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
	}

	return result;
}

/* (1) assert or (0) deassert reset lines */
static int stdemo_reset(int trst, int srst)
{
	int result = ERROR_FAIL;
	uint8_t cmd = SET_JTAG_TRST_SRST;

	LOG_DEBUG("trst: %d, srst: %d", trst, srst);

	if(!trst)
		cmd |= 0x04;
	if(!srst)
		cmd |= 0x02;

	cmd |= stdemo_parity(cmd);
	if(stdemo_raw_write(&cmd, 1))
	{
		if(trst || srst)
			sleep(1);
		result = ERROR_OK;
	}
	else
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
	}

	return result;
}

/* turn LED on stdemo adapter on (1) or off (0) */
static int stdemo_led(int on)
{
	int result = ERROR_FAIL;
	uint8_t cmd = SET_JTAG_LED;

	LOG_DEBUG("on: %d", on);

	if(on)
		cmd |= 0x02;

	cmd |= stdemo_parity(cmd);
	if(stdemo_raw_write(&cmd, 1))
	{
		result = ERROR_OK;
	}
	else
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
	}

	return result;
}

static int stdemo_speed(int speed)
{
	wait_states = speed;
	return ERROR_OK;
}

static int stdemo_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = 499999 / (khz * stdemo_toggling_time_ns);
	return ERROR_OK;
}

static int stdemo_speed_div(int speed, int *khz)
{
	uint32_t denominator = (speed + 1) * stdemo_toggling_time_ns;

	*khz = (499999 + denominator) / denominator;
	return ERROR_OK;
}

static int stdemo_init(void)
{
	const char *startcmd = "startjtag";
	struct termios options;
	char byte = ' ';

	if (stdemo_device == NULL) {
		stdemo_device = strdup("/dev/ttyACM0");
		LOG_WARNING("No stdemo device specified, using default '/dev/ttyACM0'");
	}

	if (stdemo_device_handle >= 0) {
		LOG_ERROR("device is already opened");
		return ERROR_JTAG_INIT_FAILED;
	}

	stdemo_device_handle = open(stdemo_device, O_RDWR);
	if (stdemo_device_handle < 0) {
		int err = errno;
		LOG_ERROR("cannot open device. check it exists and that user read and write rights are set. errno=%d", err);
		return ERROR_JTAG_INIT_FAILED;
	}

	fcntl(stdemo_device_handle, F_SETFL, 0 );
	memset(&options, 0, sizeof(options));
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag |= CS8;    // Select 8 data bits
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 0;
	if(tcsetattr(stdemo_device_handle, TCSANOW, &options) != 0)
	{
		LOG_ERROR("cannot init device. check it.");
		close(stdemo_device_handle);
		stdemo_device_handle = -1;
		return ERROR_JTAG_INIT_FAILED;
	}

	if(!stdemo_raw_write((const uint8_t *)startcmd, strlen(startcmd)))
	{
		LOG_ERROR("cannot init device. check it.");
		close(stdemo_device_handle);
		stdemo_device_handle = -1;
		return ERROR_JTAG_INIT_FAILED;
	}

	if(!stdemo_raw_read((uint8_t *)&byte, 1))
	{
		LOG_ERROR("cannot read device. check it.");
		close(stdemo_device_handle);
		stdemo_device_handle = -1;
		return ERROR_JTAG_INIT_FAILED;
	}

	if(byte != 'e')
	{
		LOG_ERROR("Readed invalid code 0x%X(%c). Cannot read device. check it.", byte, byte);
		close(stdemo_device_handle);
		stdemo_device_handle = -1;
		return ERROR_JTAG_INIT_FAILED;
	}

	stdemo_reset(0, 0);
	stdemo_write(0, 0, 0);
	stdemo_led(1);

	return ERROR_OK;
}

static int stdemo_quit(void)
{
	const char *stopcmd = "stopjtag";
	char byte = ' ';

	stdemo_led(0);

	if(!stdemo_raw_write((const uint8_t *)stopcmd, strlen(stopcmd)))
	{
		LOG_ERROR("cannot stop device. check it.");
	}

	if(!stdemo_raw_read((uint8_t *)&byte, 1))
	{
		LOG_ERROR("cannot stop device. check it.");
	}

	if(byte != 'd')
	{
		LOG_ERROR("Readed invalid code 0x%X(%c). Cannot stop device. check it.", byte, byte);
	}

	if (stdemo_device) {
		free(stdemo_device);
		stdemo_device = NULL;
	}

	return ERROR_OK;
}

/* The bitbang driver leaves the TCK 0 when in idle */
static void stdemo_end_state(tap_state_t state)
{
	assert(tap_is_state_stable(state));
	tap_set_end_state(state);
}

static int stdemo_state_move(int skip)
{
	uint8_t cmd[2] = { SET_JTAG_TMS_MOVE, 0 };
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	cmd[0] |= (tms_count - skip) << 1;
	cmd[0] |= stdemo_parity(cmd[0]);
	cmd[1] = tms_scan >> skip;

	if(!stdemo_raw_write(cmd, 2))
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
		return ERROR_FAIL;
	}

	tap_set_state(tap_get_end_state());
	return ERROR_OK;
}

static int stdemo_runtest(int num_cycles)
{
	uint8_t cmd = SET_JTAG_RUN_TEST;
	uint8_t lenarray[3] = { (num_cycles >> 16) & 0xFF, (num_cycles >> 8) & 0xFF, num_cycles & 0xFF};
	tap_state_t saved_end_state = tap_get_end_state();
	int len_size = 1;

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		stdemo_end_state(TAP_IDLE);
		if (stdemo_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
	}

	if(num_cycles)
		cmd |= 0x02;
	if(num_cycles > 0xFF)
	{
		cmd |= 0x04;
		len_size ++;
	}
	if(num_cycles > 0xFFFF)
	{
		cmd |= 0x08;
		len_size ++;
	}
	if(num_cycles > 0xFFFFFF)
	{
		LOG_ERROR("%s %d: Failed not supported num_cycles %d > %d\n", __FUNCTION__, __LINE__, num_cycles, 0xFFFFFF);
		return ERROR_FAIL;
	}
	cmd |= stdemo_parity(cmd);
	if(!stdemo_raw_write(&cmd, 1))
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
		return ERROR_FAIL;
	}

	if(!stdemo_raw_write(&lenarray[3 - len_size], len_size))
	{
		LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
		return ERROR_FAIL;
	}

	/* finish in end_state */
	stdemo_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		if (stdemo_state_move(0) != ERROR_OK)
			return ERROR_FAIL;

	return ERROR_OK;
}

static int stdemo_stableclocks(int num_cycles)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++) {
		if (stdemo_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (stdemo_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int stdemo_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
			tms = 0;
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
			tms = 1;
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		if (stdemo_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (stdemo_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	if (stdemo_write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	tap_set_end_state(tap_get_state());
	return ERROR_OK;
}

static int stdemo_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		unsigned scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	unsigned bit_cnt;

	if (!((!ir_scan &&
			(tap_get_state() == TAP_DRSHIFT)) ||
			(ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			stdemo_end_state(TAP_IRSHIFT);
		else
			stdemo_end_state(TAP_DRSHIFT);

		if (stdemo_state_move(0) != ERROR_OK)
			return ERROR_FAIL;
		stdemo_end_state(saved_end_state);
	}

	if((type == SCAN_IO) || (type == SCAN_OUT))
	{
		uint8_t cmd = (type == SCAN_IO) ? JTAG_IO_DATA : JTAG_OUT_DATA;
		uint8_t lenarray[3] = { (scan_size >> 16) & 0xFF, (scan_size >> 8) & 0xFF, scan_size & 0xFF};
		int len_size = 1;
		int sendbytes = (scan_size / 8) + ((scan_size & 0x07) ? 1 : 0);
		int writed;
		int readed;

		if(scan_size)
			cmd |= 0x02;
		if(scan_size > 0xFF)
		{
			cmd |= 0x04;
			len_size ++;
		}
		if(scan_size > 0xFFFF)
		{
			cmd |= 0x08;
			len_size ++;
		}
		if(scan_size > 0xFFFFFF)
		{
			LOG_ERROR("%s %d: Failed not supported len %d > %d\n", __FUNCTION__, __LINE__, scan_size, 0xFFFFFF);
			return ERROR_FAIL;
		}

		cmd |= stdemo_parity(cmd);
		if(!stdemo_raw_write(&cmd, 1))
		{
			LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
			return ERROR_FAIL;
		}

		if(!stdemo_raw_write(&lenarray[3 - len_size], len_size))
		{
			LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
			return ERROR_FAIL;
		}

		for(writed = 0, readed = 0; writed < sendbytes; )
		{
			if(!stdemo_raw_write(&buffer[writed], 1))
			{
				LOG_ERROR("%s %d: Failed write data in port\n", __FUNCTION__, __LINE__);
				return ERROR_FAIL;
			}
			writed++;
			if((type != SCAN_OUT) && (readed < writed))
			{
				int status = stdemo_raw_passive_read(&buffer[readed], writed - readed);
				if(status > 0)
					readed += status;
			}
		}

		if((type != SCAN_OUT) && (readed < sendbytes))
		{
			if(!stdemo_raw_read(&buffer[readed], sendbytes - readed))
			{
				LOG_ERROR("%s %d: Failed read data from port\n", __FUNCTION__, __LINE__);
				return ERROR_TIMEOUT_REACHED;
			}
		}
	}
	else
	{
		for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
			int tms = (bit_cnt == scan_size-1) ? 1 : 0;
			int tdi;
			int bytec = bit_cnt/8;
			int bcval = 1 << (bit_cnt % 8);

			/* if we're just reading the scan, but don't care about the output
			 * default to outputting 'low', this also makes valgrind traces more readable,
			 * as it removes the dependency on an uninitialised value
			 */
			tdi = 0;
			if ((type != SCAN_IN) && (buffer[bytec] & bcval))
				tdi = 1;

			if (stdemo_write(0, tms, tdi) != ERROR_OK)
				return ERROR_FAIL;

			if (type != SCAN_OUT) {
				switch (stdemo_read()) {
					case STD_LOW:
						buffer[bytec] &= ~bcval;
						break;
					case STD_HIGH:
						buffer[bytec] |= bcval;
						break;
					default:
						return ERROR_FAIL;
				}
			}

			if (stdemo_write(1, tms, tdi) != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	if (tap_get_state() != tap_get_end_state()) {
		/* we *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		if (stdemo_state_move(1) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int stdemo_execute_tms(struct jtag_command *cmd)
{
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;

	DEBUG_JTAG_IO("TMS: %d bits", num_bits);

	int tms = 0;
	for (unsigned i = 0; i < num_bits; i++) {
		tms = ((bits[i/8] >> (i % 8)) & 1);
		if (stdemo_write(0, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
		if (stdemo_write(1, tms, 0) != ERROR_OK)
			return ERROR_FAIL;
	}
	if (stdemo_write(CLOCK_IDLE(), tms, 0) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

int stdemo_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;
	int retval;

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	if (stdemo_led(1) != ERROR_OK)
		return ERROR_FAIL;

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RESET:
				if ((cmd->cmd.reset->trst == 1) || (cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
					tap_set_state(TAP_RESET);
				if (stdemo_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_RUNTEST:
				stdemo_end_state(cmd->cmd.runtest->end_state);
				if (stdemo_runtest(cmd->cmd.runtest->num_cycles) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				if (stdemo_stableclocks(cmd->cmd.stableclocks->num_cycles) != ERROR_OK)
					return ERROR_FAIL;
				break;

			case JTAG_TLR_RESET:
				stdemo_end_state(cmd->cmd.statemove->end_state);
				if (stdemo_state_move(0) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_PATHMOVE:
				if (stdemo_path_move(cmd->cmd.pathmove) != ERROR_OK)
					return ERROR_FAIL;
				break;
			case JTAG_SCAN:
				stdemo_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				if (stdemo_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size) != ERROR_OK)
					return ERROR_FAIL;
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			case JTAG_TMS:
				retval = stdemo_execute_tms(cmd);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	if (stdemo_led(0) != ERROR_OK)
		return ERROR_FAIL;

	return retval;
}

COMMAND_HANDLER(stdemo_handle_stdemo_port_command)
{
    if (CMD_ARGC == 0)
		return ERROR_OK;

    if (stdemo_device == 0) {
		stdemo_device = malloc(strlen(CMD_ARGV[0]) + sizeof(char));
		strcpy(stdemo_device, CMD_ARGV[0]);
	}

	command_print(CMD_CTX, "stdemo serial device = %s", stdemo_device);

	return ERROR_OK;
}

static const struct command_registration stdemo_command_handlers[] = {
	{
		.name = "stdemo_port",
		.handler = stdemo_handle_stdemo_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Display the address of the I/O port (e.g. 0x378) "
			"or the number of the '/dev/stdemo' device used.  "
			"If a parameter is provided, first change that port.",
		.usage = "[port_number]",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface stdemo_interface = {
	.name = "stdemo",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = stdemo_command_handlers,

	.init = stdemo_init,
	.quit = stdemo_quit,
	.khz = stdemo_khz,
	.speed_div = stdemo_speed_div,
	.speed = stdemo_speed,
	.execute_queue = stdemo_execute_queue,
};
