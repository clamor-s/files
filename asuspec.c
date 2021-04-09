/*
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/time.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>
#include <../gpio-names.h>
#include "asuspec.h"
#include "elan_i2c_asus.h"
#include <asm/mach-types.h>
#include <linux/statfs.h>

/*
* extern variable
*/
extern unsigned int factory_mode;
#if EMC_NOTIFY
extern u8 mouse_dock_enable_flag;
#endif
#if BATTERY_DRIVER
extern int for_asuspec_call_me_back_if_dock_in_status_change(bool);
#endif

/*
 * global variable
 */
char* switch_value[]={"0", "10"}; //0: no dock, 1:mobile dock

#define DOCK_IN_GPIO TEGRA_GPIO_PO0

static unsigned int asuspec_apwake_gpio = TEGRA_GPIO_PQ5;
static unsigned int asuspec_ps2_int_gpio = TEGRA_GPIO_PW2;
static unsigned int asuspec_kb_int_gpio = TEGRA_GPIO_PJ0;
static unsigned int asuspec_ecreq_gpio = TEGRA_GPIO_PQ2;
static unsigned int asuspec_bat_id_gpio = TEGRA_GPIO_PQ1;

static int first_tp_ioctl = 0;

struct i2c_client dockram_client;
struct i2c_client tp_client;
struct i2c_client kb_client;

static struct class *asuspec_class;
static struct device *asuspec_device;
static struct asuspec_chip *ec_chip;

struct timeval old_pad_battery_time;
struct timeval old_dock_battery_time;
struct timeval old_dock_retry_time;

struct cdev *asuspec_cdev;
static dev_t asuspec_dev;

static int asuspec_major = 0;
static int asuspec_minor = 0;
static bool led_success = 0;
static bool last_dock_in_stat = 0;
static int return_data_show_type = 0;
static int tp_init_retry = 3 ;
static int dock_init_retry = 3 ;
static int finish_first_dock_init = 0 ;
static int hid_device_sleep = 0 ;
static int kb_power_on = 0 ;
static int finish_touchpad_init = 0;
static int tp_in_ioctl = 0;

static struct workqueue_struct *asuspec_wq;
static struct workqueue_struct *asuspec_tp_wq;

static int asusdec_kp_sci_table[] = {
	0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH,
	ASUSDEC_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, ASUSDEC_KEY_AUTOBRIGHT,
	KEY_CAMERA, -9, -10, -11,
	-12, -13, -14, -15,
	KEY_WWW, ASUSDEC_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE,
	KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP
};

static const unsigned battery_prop_offs[] = {
	[POWER_SUPPLY_PROP_STATUS] = 1,
	[POWER_SUPPLY_PROP_TEMP] = 7,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = 9,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = 11,
	[POWER_SUPPLY_PROP_CAPACITY] = 13,
	[POWER_SUPPLY_PROP_CHARGE_NOW] = 15,
	[POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG] = 17,
	[POWER_SUPPLY_PROP_TIME_TO_FULL_AVG] = 19,
	[POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN] = 21,
	[POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN] = 23,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN] = 25,
	[POWER_SUPPLY_PROP_CYCLE_COUNT] = 27,
};

/*
 * functions definition
 */

int is_pad_charging_and_with_dock(void)
{
	int ret = 0;

	memset(&ec_chip->i2c_dm_data, 0, 32);

	ret = asuspec_dockram_read_data(0x0A);
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}

	return (ec_chip->i2c_dm_data[1] & 0x80 && ec_chip->i2c_dm_data[1] & 0x04);
}

int asuspec_battery_monitor(char *cmd, bool pad)
{
	int ret_val = 0;
	struct timeval pad_battery_time;
	struct timeval dock_battery_time;

	do_gettimeofday(&pad_battery_time);
	do_gettimeofday(&dock_battery_time);

	if (!strcmp(cmd, "is_pad_charging_and_with_dock") && pad == 1){
		ret_val = is_pad_charging_and_with_dock();
		return ret_val;
	}

	if (ec_chip->ec_in_s3) {
        asuspec_send_ec_req();
        msleep(200);
    }

	if (pad) { //pad battery
		if (((pad_battery_time.tv_sec - old_pad_battery_time.tv_sec) > 15) || !strcmp(cmd, "status")){
			ret_val = asuspec_dockram_read_battery(0x14);
			old_pad_battery_time.tv_sec = pad_battery_time.tv_sec;
		}
	}
	else { //dock battery
		if(((dock_battery_time.tv_sec - old_dock_battery_time.tv_sec) > 15) || !strcmp(cmd, "status")){
			ret_val = asuspec_dockram_read_battery(0x24);
			old_dock_battery_time.tv_sec = dock_battery_time.tv_sec;
		}
	}

	if (ret_val == -1){
		ASUSPEC_ERR("Fail to access battery info.\n");
		return -1;
	} else {
		if (pad)
		    return ec_chip->i2c_dm_battery[offs + 1] << 8 | ec_chip->i2c_dm_battery[offs];
		else
		    return ec_chip->i2c_dm_dock_battery[offs + 1] << 8 | ec_chip->i2c_dm_dock_battery[offs];
	}
}
EXPORT_SYMBOL(asuspec_battery_monitor);

static void asuspec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x17;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static void asuspec_kb_init(struct i2c_client *client){
	kb_client.adapter = client->adapter;
	kb_client.addr = 0x16;
	kb_client.detected = client->detected;
	kb_client.dev = client->dev;
	kb_client.driver = client->driver;
	kb_client.flags = client->flags;
	strcpy(kb_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static void asuspec_tp_init(struct i2c_client *client){
	tp_client.adapter = client->adapter;
	tp_client.addr = 0x14;
	tp_client.detected = client->detected;
	tp_client.dev = client->dev;
	tp_client.driver = client->driver;
	tp_client.flags = client->flags;
	strcpy(tp_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static int asusdec_dockram_read_data(int cmd)
{
	int ret = 0;
	int i = 0;

	if (ec_chip->dock_in == 0)
		return -1;

	memset(&ec_chip->i2c_dm_data, 0, 32);

	ec_chip->i2c_dm_data[0] = 0x05;
	ec_chip->i2c_dm_data[1] = 0x0b; //i2c read block
	ec_chip->i2c_dm_data[2] = 0x00; //result :read only
	ec_chip->i2c_dm_data[3] = 0x36; //8bit dock i2c address
	ec_chip->i2c_dm_data[4] = (u8)cmd;
	ec_chip->i2c_dm_data[5] = (u8)24; //read byte number

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 6, ec_chip->i2c_dm_data);
	if (ret < 0)
	        ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);

	msleep(20);

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x11, 32, ec_chip->i2c_dm_data);
	if (ret < 0)
	        ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);

	for (i=9; i<32; i++)
		ec_chip->i2c_dm_data[i-9] = ec_chip->i2c_dm_data[i];

	return ret;
}

static int asuspec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE)
		return -3;

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0)
		ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	else
		ec_chip->i2c_err_count = 0;

	return ret;
}

static int asuspec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE)
		return -3;

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if(ec_chip->i2c_dm_data[0] == 0xff){
		ASUSPEC_ERR("read 0xFF from ec! read again \n");
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	}

	if (ret < 0)
		ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	else
		ec_chip->i2c_err_count = 0;

	return ret;
}

static int asuspec_dockram_read_battery(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if(cmd == 0x14)
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_battery);
	else
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_dock_battery);

	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram battery, status %d\n", ret);
		ret = -1;
	} else {
		if (ec_chip->apwake_disabled){
			mutex_lock(&ec_chip->irq_lock);
			if (ec_chip->apwake_disabled){
				enable_irq(gpio_to_irq(asuspec_apwake_gpio));
				enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
				ec_chip->apwake_disabled = 0;
				ASUSPEC_ERR("Enable pad apwake\n");
			}
			mutex_unlock(&ec_chip->irq_lock);
		}
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		mutex_lock(&ec_chip->irq_lock);
		if(!ec_chip->apwake_disabled){
			disable_irq_nosync(gpio_to_irq(asuspec_apwake_gpio));
			disable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
			ec_chip->apwake_disabled = 1;
			ASUSPEC_ERR("Disable pad apwake\n");
		}
		mutex_unlock(&ec_chip->irq_lock);
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read data, status %d\n", ret);
		ec_chip->i2c_err_count++;
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i = 0;

	if (asuspec_dockram_read_data(0x01) < 0)
		goto fail_to_access_ec;
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);

	if (asuspec_dockram_read_data(0x02) < 0)
		goto fail_to_access_ec;
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

	if (asuspec_dockram_read_data(0x03) < 0)
		goto fail_to_access_ec;
	ASUSPEC_INFO("EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (asuspec_dockram_read_data(0x04) < 0)
		goto fail_to_access_ec;
	strcpy(ec_chip->ec_pcba, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("PCBA Version: %s\n", ec_chip->ec_pcba);

	asuspec_enter_normal_mode();

	ec_chip->status = 1;
	switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);

fail_to_access_ec:
	return 0;
}

static void asusdec_kp_sci(void)
{
	int ec_signal = ec_chip->i2c_data[2];

	if (ec_chip->dock_status == 0)
		return;

	if (kb_power_on == 0){
		if (elantech_i2c_command(&kb_client, ASUS_KB_WAKE_UP_CMD, ec_chip->i2c_kb_data, 0)) {
			ASUSPEC_NOTICE("set kb power on!\n");
			kb_power_on = 1;
		}
	}

	if(ec_signal == 4){
		if(tp_in_ioctl == 1){
			ASUSPEC_NOTICE("tp in ioctl, skip touchpad sci event!\n");
			return;
		}else
			tp_in_ioctl = 1;
	} else
		tp_in_ioctl = 0;

	if (finish_touchpad_init == 0) {
		ASUSPEC_NOTICE("waiting for touchpad init, skip sci event!\n", ec_chip->keypad_data.input_keycode);
		return;
	}

	ec_chip->keypad_data.input_keycode = asusdec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev);
	} else
		ASUSPEC_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
}

static int asusdec_kp_key_mapping(int x)
{
	ASUSPEC_INFO("key = %d\n", x);
	switch (x){
		case ASUSDEC_KEYPAD_ESC:
			return KEY_BACK;

		case ASUSDEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSDEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSDEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSDEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSDEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSDEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSDEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSDEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSDEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSDEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSDEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSDEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSDEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSDEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSDEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSDEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSDEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSDEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSDEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSDEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSDEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSDEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSDEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSDEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSDEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSDEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSDEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSDEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSDEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSDEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSDEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSDEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSDEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSDEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSDEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSDEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSDEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSDEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSDEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSDEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSDEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSDEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSDEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSDEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSDEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSDEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSDEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSDEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSDEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSDEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSDEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSDEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSDEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSDEC_KEYPAD_HOME:
			return KEY_HOME;

		case ASUSDEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSDEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSDEC_KEYPAD_END:
			return KEY_END;

		case KEY_CAPSLOCK:
			return KEY_WLAN;

		case KEY_F1:
			return KEY_BLUETOOTH;

		case KEY_F2:
			return ASUSDEC_KEY_TOUCHPAD;

		case KEY_F3:
			return KEY_BRIGHTNESSDOWN;

		case KEY_F4:
			return KEY_BRIGHTNESSUP;

		case KEY_F5:
			return ASUSDEC_KEY_AUTOBRIGHT;

		case KEY_F6:
			return KEY_CAMERA;

		case KEY_F7:
			return KEY_WWW;

		case KEY_F8:
			return ASUSDEC_KEY_SETTING;

		case KEY_F9:
			return KEY_PREVIOUSSONG;

		case KEY_F10:
			return KEY_PLAYPAUSE;

		case KEY_NUMLOCK:
			return KEY_NEXTSONG;

		case KEY_KP8:
			return KEY_MUTE;

		case KEY_SCROLLLOCK:
			return KEY_VOLUMEDOWN;

		case KEY_KP9:
			return KEY_VOLUMEUP;

		case KEY_KP5:
			return KEY_SLEEP;

		//--- JP keys
		case ASUSDEC_YEN:
			return KEY_YEN;

		case ASUSDEC_RO:
			return KEY_RO;

		case ASUSDEC_MUHENKAN:
			return KEY_MUHENKAN;

		case ASUSDEC_HENKAN:
			return KEY_HENKAN;

		case ASUSDEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;

		//--- UK keys
		case ASUSDEC_EUROPE_2:
			return KEY_102ND;

		default:
			return -1;
	}
}

static void asusdec_tp_abs(void)
{
        unsigned char SA1,A1,B1,SB1,C1,D1;
        static unsigned char SA1_O=0,A1_O=0,B1_O=0,SB1_O=0,C1_O=0,D1_O=0;
        static int Null_data_times = 0;

        if (1){
                SA1= ec_chip->ec_data[0];
                A1 = ec_chip->ec_data[1];
                B1 = ec_chip->ec_data[2];
                SB1= ec_chip->ec_data[3];
                C1 = ec_chip->ec_data[4];
                D1 = ec_chip->ec_data[5];
                ASUSPEC_INFO("SA1=0x%x A1=0x%x B1=0x%x SB1=0x%x C1=0x%x D1=0x%x \n",SA1,A1,B1,SB1,C1,D1);
                if ( (SA1 == 0xC4) && (A1 == 0xFF) && (B1 == 0xFF) &&
                     (SB1 == 0x02) && (C1 == 0xFF) && (D1 == 0xFF)){
                        Null_data_times ++;
                        goto asusdec_tp_abs_end;
                }

		if ( (((ec_chip->ec_data[1] & 0x0f) << 8) | ec_chip->ec_data[2]) > 1160 ||
		     (((ec_chip->ec_data[1] & 0x0f) << 8) | ec_chip->ec_data[2]) < 0    ||
		     (((ec_chip->ec_data[4] & 0x0f) << 8) | ec_chip->ec_data[5]) < 0    ||
		     (((ec_chip->ec_data[4] & 0x0f) << 8) | ec_chip->ec_data[5]) > 406	){
			goto skip_all;
		}

                if(!(SA1 == SA1_O && A1 == A1_O && B1 == B1_O &&
                   SB1 == SB1_O && C1 == C1_O && D1 == D1_O)) {
                        elantech_report_absolute_to_related(ec_chip, &Null_data_times);
                }

asusdec_tp_abs_end:
                SA1_O = SA1;
                A1_O = A1;
                B1_O = B1;
                SB1_O = SB1;
                C1_O = C1;
                D1_O = D1;
skip_all:
	        ASUSPEC_INFO("skip all\n");
        }
}

static void asusdec_touchpad_processing(void)
{
	int i;
	int length = 0;
	int tp_start = 0;

	length = ec_chip->i2c_tp_data[3];
	for( i = 0; i < length ; i++){
		ec_chip->ec_data[i] = ec_chip->i2c_tp_data[i+4];
	}
	asusdec_tp_abs();
}

static irqreturn_t asuspec_interrupt_handler(int irq, void *dev_id)
{
	ASUSPEC_INFO("interrupt irq = %d", irq);
	if (irq == gpio_to_irq(asuspec_apwake_gpio)){
		disable_irq_nosync(irq);
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_work, 0);
	}
	else if (irq == gpio_to_irq(asuspec_ps2_int_gpio)){
		ASUSPEC_INFO("ps2 int = %d", irq);
		disable_irq_nosync(irq);
		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_report_work, 0);
	}
	else if (irq == gpio_to_irq(asuspec_kb_int_gpio)){
		ASUSPEC_INFO("kb int = %d", irq);
		disable_irq_nosync(irq);
		queue_delayed_work(asuspec_wq, &ec_chip->asusdec_kb_report_work, 0);
	}
	else if (irq == gpio_to_irq(DOCK_IN_GPIO)){
		ASUSPEC_NOTICE("dock in irq = %d", irq);
	}
	else
		ASUSPEC_NOTICE("else int = %d", irq);

	return IRQ_HANDLED;
}

static int asuspec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_ecreq_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_request";

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}

	rc = gpio_direction_output(gpio, 1) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_output failed for input %d\n", gpio);
		goto err_exit;
	}


	return 0;

err_exit:
	return rc;
}

static int asuspec_irq_kb_int(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_kb_int_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_kb_int" ;

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio);
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}

	rc = request_irq(irq, asuspec_interrupt_handler, IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(gpio_to_irq(asuspec_kb_int_gpio));
	disable_irq(gpio_to_irq(asuspec_kb_int_gpio));

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}

static int asuspec_irq_ps2_int(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_ps2_int_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_ps2_int" ;

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}

	rc = request_irq(irq, asuspec_interrupt_handler, IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	disable_irq(gpio_to_irq(asuspec_ps2_int_gpio));

	return 0;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}

static int asuspec_irq_ec_apwake(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_apwake";

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}

	rc = request_irq(irq, asuspec_interrupt_handler, IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));

	return 0;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed:
	return rc;
}

static int asuspec_irq_dock_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = DOCK_IN_GPIO;
	unsigned irq = gpio_to_irq(DOCK_IN_GPIO);
	const char* label = "asuspec_dock_in";
	
	rc = gpio_request(gpio, label);
	if (rc)
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
	
	
	rc = gpio_direction_input(gpio);
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	
	rc = request_irq(irq, asuspec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail;
	}
	
	return 0 ;
err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static int asuspec_irq_battery_id(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_bat_id_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_bat_id";

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio);
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}

	return 0;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}

static void asuspec_reset_counter(unsigned long data){ }

static void asuspec_send_ec_req(void){
	ASUSPEC_NOTICE("send EC_Request\n");
	gpio_set_value(asuspec_ecreq_gpio, 0);
	msleep(DELAY_TIME_MS);
	gpio_set_value(asuspec_ecreq_gpio, 1);
}

static void asuspec_smi(void)
{
    struct timeval dock_retry_time;

    do_gettimeofday(&dock_retry_time);

    if (ec_chip->i2c_data[2] == ASUSPEC_SxI_ECREQ_Received){
        ASUSPEC_NOTICE("EC got request\n");
        if(ec_chip->status == 0){
            if(finish_first_dock_init == 1)
                asuspec_chip_init(ec_chip->client);
            else
                ASUSPEC_NOTICE("skip chip init because init must auto run at boot\n");
            }
        ec_chip->ec_in_s3 = 0;
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_BOOTBLOCK_RESET){
		ASUSPEC_NOTICE("reset EC\n");
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_WATCHDOG_RESET){
		ASUSPEC_NOTICE("reset EC\n");
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_REMOVE){
		ASUSPEC_NOTICE("dock remove\n");
		if(finish_first_dock_init == 1)
			queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
		else
			ASUSPEC_NOTICE("skip dock remove event because dock init must auto run at boot\n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_FAIL){
		ASUSPEC_NOTICE("dock communication fail!\n");
#if BATTERY_DRIVER
		if(1 != last_dock_in_stat && finish_first_dock_init == 1)
			for_asuspec_call_me_back_if_dock_in_status_change(false);
#endif
		if(finish_first_dock_init == 1)
			queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
		else
			ASUSPEC_NOTICE("skip dock remove event because dock init must auto run at boot\n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_HID_INIT_FAIL){
		if(finish_first_dock_init == 1){
			if((dock_retry_time.tv_sec - old_dock_retry_time.tv_sec) > 5){
                old_dock_retry_time	.tv_sec = dock_retry_time.tv_sec;
#if BATTERY_DRIVER
				for_asuspec_call_me_back_if_dock_in_status_change(true);
#endif
				cancel_delayed_work_sync(&ec_chip->asusdec_tp_enable_work);
				cancel_delayed_work_sync(&ec_chip->asusdec_dock_init_work);
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
				ASUSPEC_NOTICE("reinit dock\n");
			} else
				ASUSPEC_NOTICE("reinit should over 5 seconds: skip\n");
		} else
				ASUSPEC_NOTICE("skip dock event because dock init must auto run at boot:6C\n");
	} else if (ec_chip->i2c_data[2] == ASUSDEC_SxI_PAD_BL_CHANGE){
#if BATTERY_DRIVER
		if(1 != last_dock_in_stat && finish_first_dock_init == 1)
			for_asuspec_call_me_back_if_dock_in_status_change(true);
#endif
	} else if (ec_chip->i2c_data[2] == ASUSDEC_SxI_HID_Status_Changed){
		ASUSPEC_NOTICE("dock HID status changed\n");
		//init dock
		if(finish_first_dock_init == 1){
			ASUSPEC_NOTICE("queue dock init work\n");
			queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
		} else
			ASUSPEC_NOTICE("skip dock init event because dock init must auto run at boot\n");
	}
}

static void asuspec_enter_s3_work_function(struct work_struct *dat)
{
	int ret_val = 0;
	int i = 0;

	mutex_lock(&ec_chip->state_change_lock);

	ec_chip->ec_in_s3 = 1;
	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Send s3 command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("EC in S3\n");
			break;
		}
	}
	mutex_unlock(&ec_chip->state_change_lock);
}

static void asuspec_init_work_function(struct work_struct *dat)
{
	asuspec_chip_init(ec_chip->client);
}

static void asusdec_tp_report_work_function(struct work_struct *dat)
{
	int gpio = asuspec_ps2_int_gpio;
	int irq = gpio_to_irq(gpio);

       	memset(&ec_chip->i2c_tp_data, 0, 32);

		elantech_i2c_command(&tp_client, ETP_HID_READ_DATA_CMD, ec_chip->i2c_tp_data, 10);
		enable_irq(irq);
		asusdec_touchpad_processing();
}

static void asusdec_kb_report_work_function(struct work_struct *dat)
{
	int gpio = asuspec_kb_int_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;
	int i = 0;
	int j = 0;
	int scancode = 0;
	int the_same_key = 0;
       	memset(&ec_chip->i2c_kb_data, 0, 32);

	ret_val = elantech_i2c_command(&kb_client, ASUS_KB_HID_READ_DATA_CMD, ec_chip->i2c_kb_data, 11);

	enable_irq(irq);

	if(ec_chip->dock_status == 0){
		ASUSPEC_NOTICE("without dock, return!");
		return;
	}

	if(ec_chip->i2c_kb_data[0] == 0 && ec_chip->i2c_kb_data[1] == 0){ //not press key
		ASUSPEC_NOTICE("hid data length :0\n");
		return;
	}

	ec_chip->keypad_data.extend = 0;

	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
	        input_report_key(ec_chip->indev, KEY_LEFTCTRL, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
	        input_report_key(ec_chip->indev, KEY_LEFTCTRL, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
	        input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
	        input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
	        input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
	        input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
	        input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
	        input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
	        input_report_key(ec_chip->indev, KEY_RIGHTALT, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
	        input_report_key(ec_chip->indev, KEY_RIGHTALT, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
	        input_report_key(ec_chip->indev, KEY_HOMEPAGE, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
	        input_report_key(ec_chip->indev, KEY_HOMEPAGE, 0);
	}
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
	        input_report_key(ec_chip->indev, KEY_SEARCH, 1);
	}else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTWIN){
	        input_report_key(ec_chip->indev, KEY_SEARCH, 0);
	}

	for(i = 0;i < 6;i++)//normal keys
	{
	        if(ec_chip->i2c_kb_data[i+5] > 0){//press key
			ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_kb_data[i+5]);
			ec_chip->keypad_data.value = 1;
	                //ASUSPEC_NOTICE("keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
	                input_report_key(ec_chip->indev,
				ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		}else if(ec_chip->i2c_kb_data[i+5] == 0){
			break;
	        }else{
	                ASUSPEC_NOTICE("Unknown scancode = 0x%x\n", scancode);
	        }
	}
	for(i = 0;i < 6;i++)
	{
	        if(ec_chip->i2c_old_kb_data[i+5] > 0){
			for(j = 0;j < 6;j++)//check key break
			{
				if(ec_chip->i2c_kb_data[j+5] == ec_chip->i2c_old_kb_data[i+5]){
					the_same_key = 1;
					break;
				}
				else
					the_same_key = 0;
			}
			if(the_same_key == 0){
				ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_old_kb_data[i+5]);
				input_report_key(ec_chip->indev,
					ec_chip->keypad_data.input_keycode, 0);
			}
		}else{
			break;
		}
	}
	for(i = 0;i < 8;i++)
		ec_chip->i2c_old_kb_data[i+3] = ec_chip->i2c_kb_data[i+3];

	input_sync(ec_chip->indev);
}

bool asuspec_check_hid_control_flag(void)
{
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asuspec_dockram_read_data(0x23);
	if (ret_val < 0)
		ASUSPEC_NOTICE("fail to read dockram data\n");
	if(ec_chip->i2c_dm_data[3] & 0x02)
		return true;
	else
		return false;
}

static void asusdec_tp_enable_work_function(struct work_struct *dat)
{
	struct elantech_data *etd = ec_chip->private;
        if(!asuspec_check_hid_control_flag()){
		ASUSPEC_ERR("fail to detect HID flag!\n");
	        ec_chip->touchpad_member = -1;
		return;
	}

	//tp hid power on
	memset(&ec_chip->i2c_tp_data, 0, 38);
	elantech_i2c_command(&tp_client, ETP_HID_WAKE_UP_CMD, ec_chip->i2c_tp_data, 0);
	msleep(TP_DELAY_TIME_MS);

	if ((!elantech_detect(ec_chip,&tp_client)) && (!elantech_init(ec_chip,&tp_client))){
		ec_chip->touchpad_member = ELANTOUCHPAD;
		finish_touchpad_init = 1;
		ASUSPEC_NOTICE("tp init success\n");
		tp_init_retry = 3;
	} else {
	        ec_chip->touchpad_member = -1;
		if(tp_init_retry-- > 0){
			ASUSPEC_ERR("fail to detect tp : retry\n");
			queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, 1*HZ);
		}else{
			finish_touchpad_init = 1;
			ASUSPEC_ERR("enable touchpad failed!\n");
		}
		return;
	}

	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x05;
	ec_chip->i2c_tp_data[2] = 0x00;
	ec_chip->i2c_tp_data[3] = 0x22;
	ec_chip->i2c_tp_data[4] = 0xd4;
	ec_chip->i2c_tp_data[5] = 0xf4;
	i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
	ASUSPEC_INFO("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));

    mouse_dock_enable_flag = mouse_dock_enable_flag | 0x1;
}

void asuspec_tp_enable(u8 cmd)
{
	struct elantech_data *etd = ec_chip->private;

	if(first_tp_ioctl == 0){
		first_tp_ioctl = 1;
		ASUSPEC_NOTICE("first tp enable control!\n");
	}

	//tp hid power on
	memset(&ec_chip->i2c_tp_data, 0, 38);

	ASUSPEC_NOTICE("tp power on!\n");
	elantech_i2c_command(&tp_client, ETP_HID_WAKE_UP_CMD, ec_chip->i2c_tp_data, 0);
	msleep(TP_DELAY_TIME_MS);

	if(ec_chip->touchpad_member != ELANTOUCHPAD){
		if(cmd == 0xf4){
	                if ((!elantech_detect(ec_chip,&tp_client)) && (!elantech_init(ec_chip,&tp_client))){
				ec_chip->touchpad_member = ELANTOUCHPAD;
	                } else {
	                        ec_chip->touchpad_member = -1;
				ASUSPEC_ERR("ioctl enable touchpad failed!\n");
				return;
	                }
		}
	}

	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x05;
	ec_chip->i2c_tp_data[2] = 0x00;
	ec_chip->i2c_tp_data[3] = 0x22;
	ec_chip->i2c_tp_data[4] = 0xd4;
	ec_chip->i2c_tp_data[5] = cmd;

	i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);

	ASUSPEC_INFO("tp gpio value %x\n",gpio_get_value(asuspec_ps2_int_gpio));

	msleep(60);

	elantech_i2c_command(&tp_client, ETP_HID_READ_DATA_CMD, ec_chip->i2c_tp_data, 10);
	if(cmd == 0xf4){
#if EMC_NOTIFY
		mouse_dock_enable_flag = mouse_dock_enable_flag | 0x1;
#endif
		ASUSPEC_NOTICE("tp enable\n");
	}else if(cmd == 0xf5){
#if EMC_NOTIFY
		mouse_dock_enable_flag = mouse_dock_enable_flag & 0xE;
#endif
		ASUSPEC_NOTICE("tp disable\n");
	}else
		ASUSPEC_ERR("wrong tp cmd\n");

	tp_in_ioctl = 0;
}

static void asusdec_dock_status_report(void){
	ASUSPEC_NOTICE("dock_in = %d\n", ec_chip->dock_in);
	switch_set_state(&ec_chip->dock_sdev, switch_value[ec_chip->dock_type]);
}

static void asusdec_dock_init_work_function(struct work_struct *dat)
{
	int i = 0;
	int d_counter = 0;
	int gpio_state = 0;
	int err_count = 3;
	int ret = 0;
	int dock_in_stat = 0;
	tp_in_ioctl = 0;
	first_tp_ioctl = 0;
	ec_chip->touchpad_member = -1;

	memset(ec_chip->i2c_old_kb_data, 0, 38);

	if (gpio_get_value(DOCK_IN_GPIO)){
		pr_info("asusdec: No dock detected\n");

		ec_chip->tp_enable = 0;
		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->dock_status = 0;
		ec_chip->dock_init = 0;
		ec_chip->dock_type = DOCK_UNKNOWN;
		ec_chip->touchpad_member = -1;

		finish_touchpad_init = 0;
		dock_in_stat = 0;
		hid_device_sleep = 0;

		memset(ec_chip->dec_model_name, 0, 32);
		memset(ec_chip->dec_version, 0, 32);

		//unregister input device
		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}

		if (ec_chip->private->abs_dev){
			input_unregister_device(ec_chip->private->abs_dev);
			ec_chip->private->abs_dev = NULL;
		}

		if(ec_chip->kb_and_ps2_enable){
			disable_irq_nosync(gpio_to_irq(asuspec_kb_int_gpio));
			disable_irq_nosync(gpio_to_irq(asuspec_ps2_int_gpio));
			ec_chip->kb_and_ps2_enable = 0;
		}

		if(finish_first_dock_init == 1){
			elantech_i2c_command(&kb_client, ASUS_KB_SLEEP_CMD, ec_chip->i2c_kb_data, 0);
		}

		asusdec_dock_status_report();

#if BATTERY_DRIVER
		if(dock_in_stat != last_dock_in_stat)
			for_asuspec_call_me_back_if_dock_in_status_change(false);
#endif
		last_dock_in_stat = 0;
		finish_first_dock_init = 1;
		mouse_dock_enable_flag = mouse_dock_enable_flag & 0xE;
		cancel_delayed_work(&ec_chip->asusdec_tp_enable_work);

		return;
	} else {
		pr_info("asusdec: Dock-in detected\n");

		dock_in_stat = 1;
		tp_init_retry = 3;
	
		ASUSPEC_INFO("dock_in_gpio %d\n",gpio_get_value(DOCK_IN_GPIO));
		if(ec_chip->dock_status != 1){

			if (asusdec_dockram_read_data(0x02) < 0){
				goto fail_to_access_ec;
			}
			msleep(50);
			strcpy(ec_chip->dec_version, &ec_chip->i2c_dm_data[0]);
			ASUSPEC_NOTICE("DEC-FW Version: %s\n", ec_chip->dec_version);

			if(strncmp(ec_chip->dec_version, "DOCK-EC", 6)){
				if(!gpio_get_value(DOCK_IN_GPIO)){
					ASUSPEC_ERR("dock in detect with wrong version!\n");
					queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, HZ/2);
					return;
				}
				ASUSPEC_ERR("fail to init dock--wrong dock version: %s\n", ec_chip->dec_model_name);
				ec_chip->dock_type = DOCK_UNKNOWN;
				ec_chip->dock_in = 0;
				return;
			}

			if (asuspec_dockram_read_data(0x23) < 0){
				goto fail_to_access_ec;
			}

			msleep(50);

			ec_chip->dock_behavior = ec_chip->i2c_dm_data[2] & 0x02;
			ASUSPEC_NOTICE("DEC-FW Behavior: %s\n", ec_chip->dock_behavior ?
				"susb on when receive ec_req" : "susb on when system wakeup");
		}

		asusdec_input_device_create(&kb_client);

		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, HZ/2);

		////kb_init
		memset(&ec_chip->i2c_kb_data, 0, 32);

		//kb hid power on
		while(err_count-- > 0)
		{
			ret = elantech_i2c_command(&kb_client, ASUS_KB_WAKE_UP_CMD, ec_chip->i2c_kb_data, 0);
			if(ret < 0){
				kb_power_on = 0;
				ASUSPEC_ERR("set power write kb_client error: %d !\n", ret);
				msleep(10);
			}else{
				kb_power_on = 1;
				ASUSPEC_NOTICE("keyboard power on!\n");
				break;
			}
		}

		err_count = 3;
		msleep(10);

		while(err_count-- > 0)
		{
			ret = elantech_i2c_command(&kb_client, ASUS_KB_RESET_CMD, ec_chip->i2c_kb_data, 0);
			if(ret < 0){
				ASUSPEC_ERR("reset kb_client error: %d !\n", ret);
				msleep(10);
			}else{
				ASUSPEC_NOTICE("keyboard reset!\n");
				break;
			}
		}

		queue_delayed_work(asuspec_tp_wq, &ec_chip->asusdec_tp_enable_work, HZ/2);

		//enable kb_int irq
		if(ec_chip->kb_and_ps2_enable == 0){
			ec_chip->tp_enable = 1;
			enable_irq(gpio_to_irq(asuspec_ps2_int_gpio));
			enable_irq(gpio_to_irq(asuspec_kb_int_gpio));
			ec_chip->kb_and_ps2_enable = 1;
		}

#if BATTERY_DRIVER
		if(1 != last_dock_in_stat && finish_first_dock_init == 0)
			for_asuspec_call_me_back_if_dock_in_status_change(true);
#endif

		ec_chip->init_success = 1;
		ec_chip->dock_status = 1;
		hid_device_sleep = 0;
		asusdec_dock_status_report();
		last_dock_in_stat = 1;
		finish_first_dock_init = 1;
		return;
	}
fail_to_access_ec:
	if (asusdec_dockram_read_data(0x00) < 0){
		ASUSPEC_NOTICE("No EC detected\n");
		ec_chip->dock_in = 0;
	} else {
		ASUSPEC_NOTICE("Need EC FW update\n");
	}
}

static void asuspec_work_function(struct work_struct *dat)
{
	int gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;

	ret_val = asuspec_i2c_read_data(ec_chip->client);

	enable_irq(irq);

	if (ret_val < 0){
		return;
	}

	if (ec_chip->i2c_data[1] & ASUSPEC_OBF_MASK){
		if (ec_chip->i2c_data[1] & ASUSPEC_SMI_MASK){
			asuspec_smi();
			return;
		}else if(ec_chip->i2c_data[1] & ASUSDEC_SCI_MASK){
			if(ec_chip->i2c_data[2] >= 0 && ec_chip->i2c_data[2] < 24)
                asusdec_kp_sci();
		}
	}
}

static void asusdec_keypad_set_input_params(struct input_dev *dev)
{
        int i = 0;
        set_bit(EV_KEY, dev->evbit);
        for ( i = 0; i < 246; i++)
                set_bit(i,dev->keybit);

	set_bit(REL_X, dev->evbit);
	set_bit(REL_Y, dev->evbit);
	set_bit(EV_SYN, dev->evbit);
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);

	set_bit(EV_REL, dev->evbit);
        set_bit(REL_X, dev->relbit);
        set_bit(REL_Y, dev->relbit);
	set_bit(EV_SYN, dev->evbit);

	input_set_capability(dev, EV_LED, LED_CAPSL);
}

static int asusdec_input_device_create(struct i2c_client *client)
{
	int err = 0;

	if (ec_chip->indev)
	        return 0;

	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		ASUSPEC_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->indev->name = "asuspec";
	ec_chip->indev->phys = "/dev/input/asuspec";
	ec_chip->indev->dev.parent = &client->dev;

	asusdec_keypad_set_input_params(ec_chip->indev);

	/* Register input device */
	err = input_register_device(ec_chip->indev);
	if (err) {
		dev_err(&client->dev, "input device registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
        input_free_device(ec_chip->indev);
        ec_chip->indev = NULL;
exit:
        return err;
}

static void asuspec_enter_normal_mode(void)
{
	int ret_val = 0;
	int i = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x00;
	ec_chip->i2c_dm_data[1] = 0x40;

	for (i = 0; i < 3; i++ ) {
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Entering normal mode fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Entering normal mode\n");
			break;
		}
	}
}

static ssize_t asuspec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", switch_value[ec_chip->dock_type]);
}

static int __devinit asuspec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ec_chip = kzalloc(sizeof (struct asuspec_chip), GFP_KERNEL);
	if (!ec_chip) {
		ASUSPEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!ec_chip->private) {
		ASUSPEC_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->client = client;
	i2c_set_clientdata(client, ec_chip);

	mutex_init(&ec_chip->irq_lock);
	mutex_init(&ec_chip->state_change_lock);
	mutex_init(&ec_chip->dock_init_lock);

	init_timer(&ec_chip->asuspec_timer);
	ec_chip->asuspec_timer.function = asuspec_reset_counter;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asuspec_wake");

	ec_chip->ec_ram_init = 0;
	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->apwake_disabled = 0;
	ec_chip->suspend_state = 0;
	ec_chip->dock_status = 0;
	ec_chip->dock_init = 0;
	ec_chip->kb_and_ps2_enable = 0;
	ec_chip->ec_wakeup = 0;

	ec_chip->indev = NULL;
	ec_chip->private->abs_dev = NULL;

	asuspec_dockram_init(client);
	asuspec_tp_init(client);
	asuspec_kb_init(client);

	ec_chip->pad_sdev.name = PAD_SDEV_NAME;
	ec_chip->pad_sdev.print_state = asuspec_switch_state;
	if(switch_dev_register(&ec_chip->pad_sdev) < 0){
		ASUSPEC_ERR("switch_dev_register for pad failed!\n");
	}
	switch_set_state(&ec_chip->pad_sdev, 0);

	old_pad_battery_time.tv_sec = 0;
	old_pad_battery_time.tv_usec = 0;
	old_dock_battery_time.tv_sec = 0;
	old_dock_battery_time.tv_usec = 0;
	old_dock_retry_time.tv_sec = 0;
	old_dock_retry_time.tv_usec = 0;

	asuspec_wq = create_singlethread_workqueue("asuspec_wq");
	asuspec_tp_wq = create_singlethread_workqueue("asuspec_tp_wq");

	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_work, asuspec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_init_work, asuspec_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_enter_s3_work, asuspec_enter_s3_work_function);

	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_dock_init_work, asusdec_dock_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_kb_report_work, asusdec_kb_report_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_tp_report_work, asusdec_tp_report_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_tp_enable_work, asusdec_tp_enable_work_function);

	asuspec_irq_ec_request(client);
	asuspec_irq_ec_apwake(client);

	queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);

	asuspec_irq_ps2_int(&tp_client);
	asuspec_irq_kb_int(client);
	asuspec_irq_battery_id(client);

	queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 8*HZ);

	return 0;

exit:
	return err;
}

static int __devexit asuspec_remove(struct i2c_client *client)
{
	struct asuspec_chip *chip = i2c_get_clientdata(client);

	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static int asuspec_suspend(struct i2c_client *client, pm_message_t mesg)
{
	cancel_delayed_work_sync(&ec_chip->asusdec_dock_init_work);
	flush_workqueue(asuspec_wq);
	
	cancel_delayed_work_sync(&ec_chip->asusdec_tp_enable_work);
	flush_workqueue(asuspec_tp_wq);

	ec_chip->suspend_state = 1;
	ec_chip->dock_det = 0;
	ec_chip->dock_init = 0;
	ec_chip->init_success = 0;
	ec_chip->ec_in_s3 = 1;
	tp_in_ioctl = 0;

	if(ec_chip->ec_wakeup == 0 && ec_chip->dock_in == 1 && hid_device_sleep == 0){
		ASUSPEC_NOTICE("HID device sleep\n");
		hid_device_sleep = 1;
		elantech_i2c_command(&kb_client, ASUS_KB_SLEEP_CMD, ec_chip->i2c_kb_data, 0);
	}
	if(asuspec_check_hid_control_flag() && ec_chip->dock_in == 1){
		asuspec_tp_enable(0xf5);
	}

	return 0;
}

static int asuspec_resume(struct i2c_client *client)
{
    ec_chip->suspend_state = 0;
    ec_chip->dock_det = 0;
    ec_chip->init_success = 0;
    ec_chip->ec_in_s3 = 1;
	ec_chip->i2c_err_count = 0;

	if(gpio_get_value(DOCK_IN_GPIO) && ec_chip->dock_status == 1){
		ASUSPEC_ERR("no mobile dock flag but dock_status = 1\n");
	} else if(ec_chip->ec_wakeup == 0 && ec_chip->dock_status == 1 && hid_device_sleep == 1) {
		ASUSPEC_NOTICE("HID device wakeup\n");
		hid_device_sleep = 0;
		elantech_i2c_command(&kb_client, ASUS_KB_WAKE_UP_CMD, ec_chip->i2c_kb_data, 0);
	}
	if(asuspec_check_hid_control_flag() && ec_chip->dock_in == 1){
		ASUSPEC_NOTICE("touchpad wakeup\n");
		asuspec_tp_enable(0xf4);
	}

	return 0;
}

int asusdec_is_ac_over_10v_callback(void){

	int ret_val, err;

	ASUSPEC_NOTICE("access dockram\n");
	if (ec_chip->dock_in && (ec_chip->dock_type == MOBILE_DOCK)){
		msleep(250);
		err = asuspec_dockram_read_data(0x23);
		ASUSPEC_NOTICE("byte[1] = 0x%x\n", ec_chip->i2c_dm_data[1]);
		if(err < 0)
			goto fail_to_access_ec;

		return ec_chip->i2c_dm_data[1] & 0x20;
	}

fail_to_access_ec:
	ASUSPEC_NOTICE("dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(asusdec_is_ac_over_10v_callback);

static SIMPLE_DEV_PM_OPS(asuspec_dev_pm_ops, asuspec_suspend, asuspec_resume);

static const struct i2c_device_id asuspec_id[] = {
	{"asuspec", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, asuspec_id);

static struct i2c_driver asuspec_driver = {
	.class	= I2C_CLASS_HWMON,
	.probe	 = asuspec_probe,
	.remove	 = __devexit_p(asuspec_remove),
	.id_table = asuspec_id,
	.driver	 = {
		.name = "asuspec",
		.owner = THIS_MODULE,
		.pm = &asuspec_dev_pm_ops,
	},
};

static int __init asuspec_init(void)
{
	int err_code = 0;

	if (asuspec_major) {
		asuspec_dev = MKDEV(asuspec_major, asuspec_minor);
		err_code = register_chrdev_region(asuspec_dev, 1, "asuspec");
	} else {
		err_code = alloc_chrdev_region(&asuspec_dev, asuspec_minor, 1,"asuspec");
		asuspec_major = MAJOR(asuspec_dev);
	}

	err_code = i2c_add_driver(&asuspec_driver);
	if(err_code){
		ASUSPEC_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}

	asuspec_class = class_create(THIS_MODULE, "asuspec");
	if(asuspec_class <= 0){
		ASUSPEC_ERR("asuspec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}

	asuspec_device = device_create(asuspec_class, NULL, MKDEV(asuspec_major, asuspec_minor), NULL, "asuspec" );
	if(asuspec_device <= 0){
		ASUSPEC_ERR("asuspec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	pr_info("asuspec: initiated\n");
	return 0;

device_create_fail:
	class_destroy(asuspec_class);
class_create_fail:
	i2c_del_driver(&asuspec_driver);
i2c_add_driver_fail:
	return err_code;

}
module_init(asuspec_init);

static void __exit asuspec_exit(void)
{
	device_destroy(asuspec_class,MKDEV(asuspec_major, asuspec_minor)) ;
	class_destroy(asuspec_class) ;
	i2c_del_driver(&asuspec_driver);
	unregister_chrdev_region(asuspec_dev, 1);
	switch_dev_unregister(&ec_chip->pad_sdev);
}
module_exit(asuspec_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
