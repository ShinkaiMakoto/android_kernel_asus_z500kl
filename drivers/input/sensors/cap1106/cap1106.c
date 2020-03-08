#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
//#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h> 
// add by alp for proc file ++
#include <linux/seq_file.h>
// add by alp for proc file --

/* Add by Tom for power control */
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>
#include <linux/regulator/consumer.h>
/* Cap power supply VDD 1.62V-3.6V VIO 1.2-3.6V */
#define CAP1106_VDD_MIN_UV       1620000
#define CAP1106_VDD_MAX_UV       3600000
#define CAP1106_VIO_MIN_UV       1200000
#define CAP1106_VIO_MAX_UV       3600000

//add by leo for read hw id ++
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
//add by leo for read hw id --

/*Check System Mode*/
extern int entry_mode;//add by leo for charge mode (entry_mode==4)++

//add by leo for read build version ++
/*
 *build_vsersion mean TARGET_BUILD_VARIANT
 *eng:1
 *userdebug:2
 *user:3
 */
extern int build_version;
//add by leo for read build version --

#define LDBG(s,args...)	{printk(KERN_ERR "[CAP1106] : func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}

/* i2c retry */
#define I2C_RETRY_DELAY()           usleep_range(1000, 2000)
/* wait 2ms for calibration ready */
#define WAIT_CAL_READY()            usleep_range(2000, 2500)
/* >3ms wait device ready */
#define WAIT_DEVICE_READY()         usleep_range(3000, 5000)
/* >5ms for device reset */
#define RESET_DELAY()               usleep_range(5000, 10000)
/* wait 10ms for self test  done */
#define SELF_TEST_DELAY()           usleep_range(10000, 15000)

/* Add by Tom Cheng for Monitor Table or Hand Status */
#define NOT_DETECTED 0
#define HEAD_MODE 1
#define TABLE_MODE 2
#define CONFIRMING 3

#define CAP1106_I2C_DEVICE_NAME "cap1106"
#define DRIVER_VERSION		"1.0.0.0"
//#define CAP1106_INTERRUPT_GPIO	46		//H:N/A, L:Interrupt
static int  CAP1106_INTERRUPT_GPIO=0;	//78

struct cap1106_i2c_data{
	struct input_dev *cs_input_dev;
	struct i2c_client *i2c_client;
	//struct early_suspend early_suspend;
	u8 id;
	int cs_power_state;
	int irq;
	int cs_sensitivity;
	int cs_2_data;
	int cs_6_data;
	int cs_status;
	int cs1_rf_noise;
	int cs6_rf_noise;
	/* Add by Tom for power control */
	bool power_enabled;
	struct regulator *vdd;
	struct regulator *vio;
	/* Add by Tom for notify RIL */
	struct switch_dev cap1106_sdev; 
	/* Add by Tom Cheng for Monitor Table or Hand Status */
	int cs_monitor_status;
};
static struct cap1106_i2c_data *data=NULL;

//add by josh for second sensitivity ++++++
struct cap1106_config_data {
	u8 cs_sensitivity_1;
	u8 cs_gain_1;
	u8 cs2_high_thd;
	u8 cs4_high_thd;
	u8 cs6_high_thd;
	u8 cs_sensitivity_2;
	u8 cs_gain_2;
	u8 cs2_low_thd;
	u8 cs4_low_thd;
	u8 cs6_low_thd;
	int overflow_status;
};
static struct cap1106_config_data *C_data=NULL; 

/* Create a fixed path for cap sensor APK development */
static struct kobject *cap1106_kobj = NULL;	
//add by josh for second sensitivity ------

static struct i2c_client *cap1106_i2c_client = NULL;

/*Registers of Cap Sensor*/
#define PRODUCT_ID 0xFD
#define SENSOR_INPUT_ENABLE 0x21
#define SENSITIVITY_CONTROL 0x1F	// full active mode
#define REPEAT_RATE_ENABLE 0x28
#define GENERAL_STATUS 0x02
#define SENSOR_INPUT_STATUS 0x03
#define MAIN_CONTROL 0x00
#define SENSOR_INPUT1_DELTA 0x10
#define SENSOR_INPUT2_DELTA 0x11
#define SENSOR_INPUT4_DELTA 0x13
#define SENSOR_INPUT6_DELTA 0x15
#define SENSOR_INPUT1_THD 0x30
#define SENSOR_INPUT2_THD 0x31
#define SENSOR_INPUT4_THD 0x33
#define SENSOR_INPUT6_THD 0x35
#define AVG_AND_SAMP_CONFIG 0x24	// full active mode
#define NOISE_FLAG_STATUS 0x0A
#define CALIBRATION_ACTIVATE 0x26
#define CONFIGRURATION2 0x44
#define RECALIBRATION_CONFIG 0x2F
#define SENSOR_INPUT_NOISE_THD 0x38
#define CONFIGRURATION1 0x20
#define SENSOR_INPUT_CONFIG 0x22
#define MULTI_TOUCH_BLOCK_CONFIG 0x2A

//add by leo for standby mode ++
#define STANDBY_CHANNEL		0x40	// standby mode
#define STANDBY_CONFIG			0x41	// standby mode
#define STANDBY_SENSITIVITY	0x42	// standby mode
#define STANDBY_THRESHOLD		0x43	// standby mode
//add by leo for standby mode --


typedef enum {
	NOT_SENSED = 0,
	SENSED,
	UNKNOW
}sensor_status;
static sensor_status    CS_OldStatus = UNKNOW;
static sensor_status    CS_Status = UNKNOW;
static sensor_status    CS_UEvent_Status = UNKNOW;

#define CLEAN_DSLEEP		0xEF	//0xEF = 1110 1111, DSLEEP = B4 (AND)
#define SET_DSLEEP			0x10	//0x10 = 0001 0000, DSLEEP = B4 (OR)
#define CLEAN_STBY			0xDF	//0x00 = 1101 1111
#define SET_STBY			0x20	//0x20 = 0010 0000

/* Add by Tom for Enable / Disable HAND_TABLE_DETECT for BodySAR Test */ 
static bool IS_DOUBLE_SENSITIVITY = true;

#define DOUBLE_SENSITIVITY
#define HIGH_SENSITIVITY	0
#define LOW_SENSITIVITY		1

/* Sensors Interrupt Workqueue */
static struct workqueue_struct *cap1106_wq=NULL;
static struct delayed_work cap1106_work;
static void cap1106_work_function(struct work_struct *work);
/* HW Power Control */
static int cap1106_power_ctl(struct cap1106_i2c_data *data, bool on);

#define ABS_SENSED ABS_HAT2X

//Linux signal ipc
static int notify_daemon_pid = 0;
/*
static int cap_sensor_not_sensed_signal = 16;
static int cap_sensor_sensed_signal = 17;
static int check_table_list_signal = 18;
static int sar_rf_state_default_signal = 50;
//static int sar_rf_state_1_signal = 51;
static int sar_rf_state_2_signal = 52;
static int sar_rf_state_3_signal = 53;
static int sar_rf_state_4_signal = 54;
static int sar_rf_state_5_signal = 55;
static int sar_rf_state_6_signal = 56;
static int sar_rf_state_7_signal = 57;
static int sar_rf_state_8_signal = 58;
static int cap_sensor_always_on_signal = 59;
static int cap_sensor_always_off_signal = 60;
*/

/* others */
static int isDaemonReady = 0;	// add by leo for system crash issue ++
static int isSendSignalSuccess = 0;	// add by leo for modem reset ++
static int isCapSensorExist = 0; // add by leo to detect cap sensor chip
static int isCheckDeviceOk = 0; // add by leo to detect cap sensor chip
static int isStandbyMode = 0;
static int CS_ENABLE_STATUS = 0x00; // add by Tom for RF enable/disable cs in user image  

#ifdef DOUBLE_SENSITIVITY
static int status_stable_count = 0; // status stable count 
static int SensitivityMode = HIGH_SENSITIVITY;
/* To Check Status is Hands or Table */
static int confirm_count = 0;
static void cap1106_change_sensitivity(int mode);
#endif

#define CAPSENSOR_IOCTL_MAGIC 'c'
#define CAPSENSOR_IOCTL_GET_ENABLED	_IOR(CAPSENSOR_IOCTL_MAGIC, 1, int *) 
#define CAPSENSOR_IOCTL_ENABLE			_IOW(CAPSENSOR_IOCTL_MAGIC, 2, int *) 

static int reset_cap1106_interrupt(void);
static irqreturn_t cap1106_interrupt_handler(int irq, void *dev_id);
static int cap1106_send_signal (sensor_status s);
//static void cap1106_get_data(void);

// add by leo for proc file ++
#define	CAP1106_PROC_FILE	"cap1106"
static struct proc_dir_entry *cap1106_proc_file;
// add by leo for proc file --

#define CS_DEBUGMSG 	0
#define CS_STATUSMSG 	1
static unsigned int debug = 0;

//=========================================================================================
/* Cap Sensor Notify RIL */
ssize_t cap1106_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", data->power_enabled ?"enable" : "disable");
}
ssize_t cap1106_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", CS_UEvent_Status);
}


static int cap1106_check_device(struct cap1106_i2c_data *cap1106_data)
{
	u8 whoami=0;
	
	whoami = i2c_smbus_read_byte_data(cap1106_i2c_client, PRODUCT_ID);
	if (whoami < 0){
		if(CS_STATUSMSG) LDBG( "Read ID Register Failed \n");
		return whoami;
	}
	
	if(whoami!=0x55){
		if(CS_STATUSMSG) LDBG( "Check Device Failed \n");
		if(CS_STATUSMSG) LDBG( "Who am I ??? (PRODUCT ID = 0x%x) \n",whoami);
		return -ENODEV;
	}
	cap1106_data->id= whoami;
	if(CS_STATUSMSG) LDBG( "Check Device OK !! PRODUCT ID = 0x%x \n",cap1106_data->id);

	isCheckDeviceOk=1; // add by leo to detect cap sensor chip
	return 0;
}

static int cap1106_setup_irq(struct cap1106_i2c_data *cap1106_data)
{
	int err = -EINVAL;
	
	/*gpio setting*/
	err = gpio_request(CAP1106_INTERRUPT_GPIO, CAP1106_I2C_DEVICE_NAME);	
	if(err<0){
		if(CS_STATUSMSG) LDBG("gpio_request Failed\n");
		goto exit;
	}

	err = gpio_direction_input(CAP1106_INTERRUPT_GPIO) ;
	if(err<0){
		if(CS_STATUSMSG) LDBG("gpio_direction_input Failed\n");
		goto gpio_direction_input_err;
	}

	//gpio_tlmm_config(GPIO_CFG(CAP1106_INTERRUPT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	
	cap1106_data->irq = gpio_to_irq(CAP1106_INTERRUPT_GPIO);
	if(cap1106_data->irq<0){
		if(CS_STATUSMSG) LDBG("gpio_direction_input Failed\n");
		goto gpio_to_irq_err;
	}
	
	if(cap1106_data->irq > 0){
		
		err = request_irq(cap1106_data->irq, cap1106_interrupt_handler, IRQF_TRIGGER_FALLING, CAP1106_I2C_DEVICE_NAME, cap1106_data->i2c_client);
		if (err < 0){
			if(CS_STATUSMSG) LDBG("request_irq Failed, irq = %d, err = %d\n", cap1106_data->irq, err);
			err = -EIO;
			goto request_irq_err;
		}
	}
/*
	err = request_threaded_irq(cap1106_data->irq, NULL, cap1106_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"cap1106_interrupt",NULL);
	if (err) {
		dev_err(&cap1106_i2c_client->adapter->dev,"Can NOT Register IRQ : %d , err:%d\n", cap1106_data->irq, err);
		err = -EIO;
		goto request_irq_err;
	}
*/
	if(CS_STATUSMSG) LDBG("CAP1106 interrupt gpio = %d , value = %d\n", CAP1106_INTERRUPT_GPIO, gpio_get_value(CAP1106_INTERRUPT_GPIO));
	return 0;
	
request_irq_err:
gpio_to_irq_err:
gpio_direction_input_err:	
	gpio_free(CAP1106_INTERRUPT_GPIO);
exit:	
	return err;
}
// add by leo for proc file ++
static ssize_t cap1106_register_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	debug = debug ? 0 : 1;	
	LDBG("debug meaasge [%s]\n", (debug == 1 ? "on" : "off"));
	return 0;
}

static ssize_t cap1106_register_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	char messages[80] = {0};
	int en=0, cmd=0, ret=0, count=0, iloop=0, i=0;
	u32 reg_address=0, reg_value=0;
	static u8 temp[4]={0};
	static int total_input[80]={0};
	char* input_cmd = (char*)kcalloc(count,sizeof(char),GFP_KERNEL);

	if (len >= 80) {
		if(CS_STATUSMSG) LDBG(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	
	if ((messages[0]=='l')&&(messages[1]=='o')&&(messages[2]=='g')){
		if(&messages[4]!=NULL)
			sscanf (&messages[4], "%d", &en);
	
		if(CS_STATUSMSG) LDBG( "en = %d\n",en);
		debug=(en==1)?1:0;
		if(CS_STATUSMSG) LDBG( "debug meaasge (%s)\n",(debug==1)?"on":"off");
		return len;

	}else if(messages[0]=='w'){
		cmd=1;
	}else if(messages[0]=='r'){
		cmd=2;
	}else{
		if(CS_STATUSMSG) LDBG("Unknow Command\n");
		return len;
	}

	// count input command
	for(i=0;i<100;i++){
		if(messages[i]=='x'){
			count ++;
		}else if(messages[i]=='\n'){
			if(unlikely(debug))LDBG("command number = %d\n",count);
			break;
		}
	}

	// transfor input command from ASCII code to HEX
	for(iloop = 0; iloop < count; iloop++){
		temp[0] = messages[iloop*5 + 2];
		temp[1] = messages[iloop*5 + 3];
		temp[2] = messages[iloop*5 + 4];
		temp[3] = messages[iloop*5 + 5];
		sscanf(temp, "%x", &total_input[iloop]);
	}

	if(CS_STATUSMSG) LDBG("input_cmd = %s ",cmd==1?"write":"read");
	for(i=0;i<count;i++){
		input_cmd[i]=total_input[i]&0xff;
		if(CS_STATUSMSG) LDBG("0x%02x ", input_cmd[i]);
	}
	if(CS_STATUSMSG) LDBG("\n");

	// send test command
	reg_address = input_cmd[0];
	reg_value = input_cmd[1];

	if(cmd==1){ //write
		ret = i2c_smbus_write_byte_data(cap1106_i2c_client, reg_address, reg_value);
		if(ret< 0)  {
			if(CS_STATUSMSG) LDBG( "Write 0x%02X into 0x%02X failed (%d)\n",reg_value,reg_address,ret);
			return ret;
		}
		if(CS_STATUSMSG) LDBG( "write 0x%02X into 0x%02X\n",reg_value,reg_address);
		
	}else if(cmd==2){ //read
		ret = i2c_smbus_read_byte_data(cap1106_i2c_client, reg_address);
		if(ret< 0)  {
			if(CS_STATUSMSG) LDBG( "Read data from 0x%02X failed (%d)\n",reg_address,ret);
			return ret;
		}
		if(CS_STATUSMSG) LDBG( "0x%02x value is 0x%02x \n",reg_address,ret);
		reg_value=ret;
	}
	
	return len;
}

// add by alp for proc file ++
static const struct file_operations cap1106_register_fops = 
{
	.owner = THIS_MODULE,
	.read = cap1106_register_read,
	.write = cap1106_register_write,
};


void cap1106_create_proc_file(void)
{
	cap1106_proc_file = proc_create(CAP1106_PROC_FILE, 0666, NULL, &cap1106_register_fops);
	if(cap1106_proc_file) { 
		LDBG( "create_proc [%s]\n", CAP1106_PROC_FILE);
	} else {
		LDBG( "create_proc failed\n");
	}
	
}
// add by alp for proc file --

void cap1106_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    if(unlikely(debug))LDBG( "remove proc files \n");
    remove_proc_entry(CAP1106_PROC_FILE, &proc_root);
}
// add by leo for proc file --
//========================================================================================
static ssize_t CapSensor_check_For_ATD_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int TOUCH=0,CS_GeneralStatus=0;
	
	CS_GeneralStatus = i2c_smbus_read_byte_data(cap1106_i2c_client, GENERAL_STATUS);
	if (CS_GeneralStatus  < 0)
	{	
		if(CS_STATUSMSG) LDBG( "CS Read GENERAL_STATUS Failed\n");
		return sprintf(buf,"Read GENERAL_STATUS Failed\n");
	}
	if(unlikely(debug)) LDBG( "CS GENERAL_STATUS = 0x%x\n",CS_GeneralStatus);

	reset_cap1106_interrupt();
	
	TOUCH = (CS_GeneralStatus & 0x01); // 0000 0001
	if(TOUCH == 1)  {
		return sprintf(buf,"1\n");
	}
	return sprintf(buf,"0\n");
}
static DEVICE_ATTR(cap_status, 0444,CapSensor_check_For_ATD_test,NULL);

// add by leo to detect cap sensor chip ++
static ssize_t cs_exist_check(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(CS_STATUSMSG) LDBG("isCapSensorExist = %d\n",isCapSensorExist);

	if(isCapSensorExist==0){
		return sprintf(buf,"0\n");
	}else{
		return sprintf(buf,"1\n");
	}
}
static DEVICE_ATTR(capsensor_exist_check, 0444,cs_exist_check,NULL);

static ssize_t cs_status_check(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(CS_STATUSMSG) LDBG("isCheckDeviceOk = %d\n",isCheckDeviceOk);

	if(isCheckDeviceOk==0){
		return sprintf(buf,"0\n");
	}else{
		return sprintf(buf,"1\n");
	}
}
static DEVICE_ATTR(capsensor_status_check, 0444,cs_status_check,NULL);

static ssize_t cs_check(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(CS_STATUSMSG) LDBG("isCheckDeviceOk = %d, isCapSensorExist = %d\n",isCheckDeviceOk, isCapSensorExist);
	
	if(isCheckDeviceOk==1){
		if(CS_STATUSMSG) LDBG("cap sensor ok\n");
		return sprintf(buf,"cap sensor ok\n");
		
	}else if((isCheckDeviceOk==0)&&(isCapSensorExist==0)){
		if(CS_STATUSMSG) LDBG("no cap sensor\n");
		return sprintf(buf,"no cap sensor\n");
		
	}else if((isCheckDeviceOk==0)&&(isCapSensorExist!=0)){
		if(CS_STATUSMSG) LDBG("cap sensor check device fail\n");
		return sprintf(buf,"cap sensor check device fail\n");
	}


	if(CS_STATUSMSG) LDBG("cap sensor unknow status\n");
	return sprintf(buf,"cap sensor unknow status\n");
}
static DEVICE_ATTR(cap_check, 0444,cs_check,NULL);
// add by leo to detect cap sensor chip --

static ssize_t cs_get_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 whoami=0;
	
	whoami = i2c_smbus_read_byte_data(cap1106_i2c_client, PRODUCT_ID);
	if (whoami < 0)
	{	
		if(CS_STATUSMSG) LDBG( "CS Read ID Register Failed\n");
		return sprintf(buf,"Read ID Register Failed\n");
	}
	return sprintf(buf,"Cap1106 ID = 0x%x\n",whoami);
}
static DEVICE_ATTR(cap1106_id, 0444,cs_get_id,NULL);


static ssize_t cs_chip_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	u8 cs_info[3]={0}; //product_id, manufacturer_id, revision
	static int switch_io_gpio = 0;
	if(switch_io_gpio == 0) {
		gpio_direction_output(CAP1106_INTERRUPT_GPIO, 0);
		gpio_set_value_cansleep(CAP1106_INTERRUPT_GPIO, 1);
		switch_io_gpio = 1;
		LDBG( "CS INT SET OUTPUT HIGH\n");
	} else {
		gpio_direction_input(CAP1106_INTERRUPT_GPIO);
		switch_io_gpio = 0;
		LDBG( "CS INT SET INPUT\n");
	}

	
	ret = i2c_smbus_read_i2c_block_data(cap1106_i2c_client, PRODUCT_ID, 3, cs_info);	//get chip info
	if (ret  < 0)
	{	
		if(CS_STATUSMSG) LDBG( "CS Read PRODUCT_ID Failed\n");
		return sprintf(buf,"Read CS Chip Info Failed\n");
	}
	return sprintf(buf,"Model Name: %s, Product ID = 0x%x, Manufacturer ID = 0x%x, Revision = 0x%x \n", CAP1106_I2C_DEVICE_NAME, cs_info[0], cs_info[1], cs_info[2]);
}
static DEVICE_ATTR(cs_info, 0444,cs_chip_info,NULL);

static ssize_t cs_report_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%s\n",CS_OldStatus==SENSED?"1":"0");
}
static DEVICE_ATTR(cs_report, 0444,cs_report_status,NULL);

static ssize_t cs_get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 SensorsStatus=0;
	
	SensorsStatus = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
	if (SensorsStatus < 0)
	{	
		if(CS_STATUSMSG) LDBG( "CS Read SENSOR_INPUT_STATUS Failed\n");
		return sprintf(buf,"Read Status Registers Failed\n");
	}
	reset_cap1106_interrupt();
		
	return sprintf(buf,"CS Input Status = 0x%x\n",SensorsStatus);
}
static DEVICE_ATTR(cs_status, 0444,cs_get_status,NULL);


static ssize_t cs_noise_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 NoiseFlag=0;
	
	NoiseFlag = i2c_smbus_read_byte_data(cap1106_i2c_client, NOISE_FLAG_STATUS);
	if (NoiseFlag < 0)
	{	
		if(CS_STATUSMSG) LDBG( "CS Read NOISE_FLAG_STATUS Failed\n");
		return sprintf(buf,"Read Noise Flag Status Registers Failed\n");
	}
	
	return sprintf(buf,"CS Noise Flag Status = 0x%x\n",NoiseFlag);
}
static DEVICE_ATTR(cs_noise, 0444,cs_noise_flag,NULL);

/* add by Tom for RF enable/disable cs in user image */
static ssize_t cs_switch_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(CS_ENABLE_STATUS == 0x00) {
		CS_ENABLE_STATUS = 0x2A;
	} else {
		CS_ENABLE_STATUS = 0x00;
	}
	i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, CS_ENABLE_STATUS); 

	return sprintf(buf,"0x%X\n", CS_ENABLE_STATUS);
}
static DEVICE_ATTR(cs_switch_enable, 0444, cs_switch_enable,NULL);

/* add by Tom for RF disable cs in user image */
static ssize_t cs_disable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int cap_status = -1 , time_out = 0;
	CS_ENABLE_STATUS = 0x00;
	
	/* Be sure to disable cap sensor success */	
	while(cap_status != CS_ENABLE_STATUS && time_out < 5) {	
		i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, CS_ENABLE_STATUS); 
		msleep(50);	
		cap_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE);
		if (cap_status < 0){
			LDBG("Read SENSOR_INPUT_ENABLE Failed !\n");
		}
		time_out++;
	}
	
	LDBG("Disable Cap Sensor via ADB Command ! return 0x%X , time_count [%d] \n", cap_status, time_out);

	/* Make sure last report status is NON-DETECTED */	
	LDBG("Notify RIL status is NON-DETECTED !\n");
	CS_OldStatus = NOT_SENSED;
	cap1106_send_signal(UNKNOW); 	// Reset Event for RIL Deamon
	msleep(10);
	cap1106_send_signal(NOT_SENSED);

	return sprintf(buf,"0x%X\n", cap_status);
}
static DEVICE_ATTR(cs_disable, 0444, cs_disable,NULL);

/* add by Tom for RF enable cs in user image */
static ssize_t cs_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int cap_status = -1 , time_out = 0;
	CS_ENABLE_STATUS = 0x2A;
	
	/* Be sure to enable cap sensor success */	
	while(cap_status != CS_ENABLE_STATUS && time_out < 5) {	
		i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, CS_ENABLE_STATUS); 
		msleep(50);	
		cap_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE);
		if (cap_status < 0){
			LDBG("Read SENSOR_INPUT_ENABLE Failed !\n");
		}
		time_out++;
	}
	
	LDBG("Enable Cap Sensor via ADB Command ! return 0x%X , time_count [%d] \n", cap_status, time_out);

	return sprintf(buf,"0x%X\n", cap_status);
}
static DEVICE_ATTR(cs_enable, 0444, cs_enable,NULL);

static ssize_t cs_enable_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE);
	if (ret < 0){
		return sprintf(buf, "Read SENSOR_INPUT_ENABLE Failed\n");
	}
	return sprintf(buf, "Sensor Input Enable Register = 0x%x\n", ret);
};
static ssize_t cs_enable_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "Sensor Input Enable Register 0x%x Enable Failed (%d)\n",regvalue,ret);
		return ret;
	}

	return count;
};
static DEVICE_ATTR(cs_en, 0664, cs_enable_show, cs_enable_store);

//add by leo for standby mode ++
static ssize_t cs_standby_enable_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CHANNEL);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_CHANNEL Failed\n");
	}
	return sprintf(buf, "Standby Channel Register = 0x%x\n", ret);
};
static ssize_t cs_standby_enable_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "Standby Channel Register 0x%x Enable Failed (%d)\n",regvalue,ret);
		return ret;
	}

	return count;
};
static DEVICE_ATTR(cs_stby_en, 0664, cs_standby_enable_show, cs_standby_enable_store);
//add by leo for standby mode --

static ssize_t cs_cycle_time_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 regvalue =0, AVG=0, SAMP_TIME=0, CYCLE_TIME=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG);
	AVG= (regvalue & 0x70)>>4; //0x70 = 0111 0000
	SAMP_TIME= (regvalue & 0x0c)>>2; //0x0c = 0000 1100
	CYCLE_TIME= (regvalue & 0x03); //0x03 = 0000 0011
	if(CS_STATUSMSG) LDBG( "Averaging and Sampling Configuration Register = 0x%x, AVG = 0x%x, SAMP_TIME = 0x%x, CYCLE_TIME = 0x%x\n",regvalue, AVG, SAMP_TIME, CYCLE_TIME);

	return sprintf(buf, "Averaging and Sampling Configuration = 0x%x, CYCLE_TIME = 0x%x\n", regvalue, CYCLE_TIME);
};
static ssize_t cs_cycle_time_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0, CYCLE_TIME=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	CYCLE_TIME = ((u8) temp) & 0x3;// 0x03 = 0000 0011

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG);
	regvalue = (regvalue & 0xfc) | CYCLE_TIME;	//0xfc = 11111100

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write AVG_AND_SAMP_CONFIG Failed (%d)\n",ret);
	}
	if(unlikely(debug)) LDBG( "//////////////////////////		CS new CYCLE_TIME is 0x%x \n",CYCLE_TIME);

	return count;
};
static DEVICE_ATTR(cs_cycle, 0664, cs_cycle_time_show, cs_cycle_time_store);

//add by leo for standby mode ++
static ssize_t cs_stby_cycle_time_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 regvalue =0, AVG_SUM=0,STBY_AVG=0, STBY_SAMP_TIME=0, STBY_CYCLE_TIME=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CONFIG);
	AVG_SUM = regvalue >>7;
	STBY_AVG= (regvalue & 0x70)>>4; //0x70 = 0111 0000
	STBY_SAMP_TIME= (regvalue & 0x0c)>>2; //0x0c = 0000 1100
	STBY_CYCLE_TIME= (regvalue & 0x03); //0x03 = 0000 0011
	if(CS_STATUSMSG) LDBG( "AVG_SUM = 0x%x, STBY_AVG = %x, STBY_SAMP_TIME = 0x%x, STBY_CYCLE_TIME = 0x%x\n", AVG_SUM, STBY_AVG, STBY_SAMP_TIME, STBY_CYCLE_TIME);
	
	return sprintf(buf, "Standby Configuration Register = 0x%x, STBY_CYCLE_TIME = 0x%x\n", regvalue, STBY_CYCLE_TIME);
};
static ssize_t cs_stby_cycle_time_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0, STBY_CYCLE_TIME=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	STBY_CYCLE_TIME = ((u8) temp) & 0x3;// 0x03 = 0000 0011

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CONFIG);
	regvalue = (regvalue & 0xfc) | STBY_CYCLE_TIME;	//0xfc = 11111100

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CONFIG, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write STANDBY_SENSITIVITY Failed (%d)\n",ret);
	}
	if(unlikely(debug)) LDBG( "//////////////////////////		CS new STBY_CYCLE_TIME is 0x%x \n",STBY_CYCLE_TIME);

	return count;
};
static DEVICE_ATTR(cs_stby_cycle, 0664, cs_stby_cycle_time_show, cs_stby_cycle_time_store);
//add by leo for standby mode --

static ssize_t cs_gain_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 regvalue =0, GAIN=0, STBY=0, DSLEEP=0, INTBit=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	GAIN= regvalue >> 6; 
	STBY= (regvalue & 0x20)>>5; //0x20 = 0010 0000
	DSLEEP= (regvalue & 0x10)>>4; //0x10 = 0001 0000
	INTBit= (regvalue & 0x01); //0x01 = 0000 0001
	if(CS_STATUSMSG) LDBG( "GAIN = 0x%x, STBY = 0x%x, DSLEEP = 0x%x, INT = 0x%x\n",GAIN, STBY, DSLEEP, INTBit);
	
	return sprintf(buf, "Main Control Register = 0x%x, GAIN = 0x%x\n", regvalue, GAIN);
};
static ssize_t cs_gain_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0, GAIN=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	GAIN = ((u8) temp)<<6;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	regvalue = (regvalue & 0x3f) | GAIN;	//0x3f = 00111111

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write  MAIN_CONTROL Failed (%d)\n",ret);
	}
	if(unlikely(debug)) LDBG( "//////////////////////////		CS new GAIN is 0x%x \n",GAIN);

	return count;
};
static DEVICE_ATTR(cs_gain, 0664, cs_gain_show, cs_gain_store);


/* Add by Tom for Enable / Disable HAND_TABLE_DETECT for BodySAR Test */ 
static ssize_t cs_hand_table_en_show(struct device *class,struct device_attribute *attr,char *buf)
{
	LDBG( "IS_DOUBLE_SENSITIVITY [%d]\n", IS_DOUBLE_SENSITIVITY);
	return sprintf(buf, "%d\n", IS_DOUBLE_SENSITIVITY);
};
static ssize_t cs_hand_table_en_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int temp = 0;
	sscanf (buf, "%d", &temp);

	if(temp == 1)
		IS_DOUBLE_SENSITIVITY = true;
	else
		IS_DOUBLE_SENSITIVITY = false;

	LDBG( "IS_DOUBLE_SENSITIVITY [%d]\n", IS_DOUBLE_SENSITIVITY);

	return count;
};
static DEVICE_ATTR(cs_hand_table_en, 0664, cs_hand_table_en_show, cs_hand_table_en_store);

static ssize_t cs2_get_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS2_DeltaCount = 0;

	CS2_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_DELTA);

	return sprintf(buf, "CS2 delta = 0x%x\n", CS2_DeltaCount );
}
static DEVICE_ATTR(cs2_delta, 0444,cs2_get_delta,NULL);

static ssize_t cs4_get_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS4_DeltaCount = 0;

	CS4_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);

	return sprintf(buf, "CS4 delta = 0x%x\n", CS4_DeltaCount );
}
static DEVICE_ATTR(cs4_delta, 0444, cs4_get_delta,NULL);

static ssize_t cs6_get_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS6_DeltaCount = 0;

	CS6_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);

	return sprintf(buf, "CS6 delta = 0x%x\n", CS6_DeltaCount );
}
static DEVICE_ATTR(cs6_delta, 0444,cs6_get_delta,NULL);

/* For CS2 Threshold Control */
static ssize_t cs2_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 cs2_threshold = 0;

	cs2_threshold = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_THD);

	LDBG( "CS2 Threshold is [0x%02X] \n", cs2_threshold);
	return sprintf(buf, "%02x\n", cs2_threshold);
};
static ssize_t cs2_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 cs2_threshold=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	cs2_threshold = (u8) temp;
	
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT2_THD, cs2_threshold);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSOR_INPUT2_THD Failed (%d)\n",ret);
	}
	LDBG( "CS4 Threshold is [0x%02X] \n", cs2_threshold);

	return count;
};
static DEVICE_ATTR(cs2_thd, 0664, cs2_thd_show, cs2_thd_store);


/* For CS4 Threshold Control */
static ssize_t cs4_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 cs4_threshold =0;

	cs4_threshold = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_THD);

	LDBG( "CS4 Threshold is [0x%02X] \n", cs4_threshold);
	return sprintf(buf, "%02x\n", cs4_threshold);
};
static ssize_t cs4_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 cs4_threshold=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	cs4_threshold = (u8) temp;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT4_THD, cs4_threshold);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSOR_INPUT4_THD Failed (%d)\n",ret);
	}
	LDBG( "CS4 Threshold is [0x%02X] \n", cs4_threshold);

	return count;
};
static DEVICE_ATTR(cs4_thd, 0664, cs4_thd_show, cs4_thd_store);

/* For CS6 Threshold Control */
static ssize_t cs6_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 cs6_threshold =0;

	cs6_threshold = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD);

	LDBG( "CS6 Threshold is [0x%02X] \n", cs6_threshold);
	return sprintf(buf, "%02x\n", cs6_threshold);
};
static ssize_t cs6_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 cs6_threshold=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	cs6_threshold = (u8) temp;
	
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD, cs6_threshold);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSOR_INPUT6_THD Failed (%d)\n",ret);
	}
	LDBG( "CS6 Threshold is [0x%02X] \n", cs6_threshold);

	return count;
};
static DEVICE_ATTR(cs6_thd, 0664, cs6_thd_show, cs6_thd_store);

/* For High / Low Threshold Control */
static ssize_t high_low_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	LDBG( "CS2 High Threshold is [0x%02X], Low Threshold is [0x%02X]\n", C_data->cs2_high_thd, C_data->cs2_low_thd);
	LDBG( "CS4 High Threshold is [0x%02X], Low Threshold is [0x%02X]\n", C_data->cs4_high_thd, C_data->cs4_low_thd);
	LDBG( "CS4 High Threshold is [0x%02X], Low Threshold is [0x%02X]\n", C_data->cs6_high_thd, C_data->cs6_low_thd);
	return sprintf(buf, "CS2 High Threshold is [0x%02X], Low Threshold is [0x%02X]\nCS4 High Threshold is [0x%02X], Low Threshold is [0x%02X]\nCS6 High Threshold is [0x%02X], Low Threshold is [0x%02X]\n", 
			C_data->cs2_high_thd, C_data->cs2_low_thd,
			C_data->cs4_high_thd, C_data->cs4_low_thd,
			C_data->cs6_high_thd, C_data->cs6_low_thd);
};
static ssize_t high_low_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 cs_threshold=0;
	int cs_number = 0;
	int cs_high_low = 0;	// For high or low threshold
	int temp = 0;

	sscanf (buf, "%d %d %x", &cs_number, &cs_high_low, &temp);
	cs_threshold = (u8) temp;
	switch (cs_number) 
	{
		case 2 :
			if (cs_high_low == 1)
				C_data->cs2_high_thd = cs_threshold;
			else
				C_data->cs2_low_thd = cs_threshold;
			break;
		case 4 :
			if (cs_high_low == 1)
				C_data->cs4_high_thd = cs_threshold;
			else
				C_data->cs4_low_thd = cs_threshold;
			break;
		case 6 :
			if (cs_high_low == 1)
				C_data->cs6_high_thd = cs_threshold;
			else
				C_data->cs6_low_thd = cs_threshold;
			break;
		default :
			break;

	}	
	LDBG( "Set CS_[%d] [%s] Threshold is [0x%02X] \n", cs_number, cs_high_low == 1 ? "HIGH" : "LOW", cs_threshold);

	return count;
};
static DEVICE_ATTR(high_low_thd, 0664, high_low_thd_show, high_low_thd_store);

//add by leo for standby mode ++
static ssize_t cs_standby_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_THRESHOLD);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_THRESHOLD Failed\n");
	}
	return sprintf(buf, "Standby Threshold Register = 0x%x\n", ret);
};
static ssize_t cs_standby_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 STBY_Threshold = 0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	STBY_Threshold = ((u8)temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_THRESHOLD, STBY_Threshold);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write Standby Threshold Register Failed (%d)\n",ret);
	}

	return count;
};
static DEVICE_ATTR(cs_stby_thd, 0664, cs_standby_thd_show, cs_standby_thd_store);
//add by leo for standby mode --

static ssize_t cs_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 cs_sensitivity =0;

	cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);

	return sprintf(buf, "%02x\n", cs_sensitivity);
};
static ssize_t cs_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 cs_sensitivity=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	cs_sensitivity = (u8) temp;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, cs_sensitivity);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSITIVITY_CONTROL Failed (%d)\n",ret);
	}
	if(unlikely(debug)) LDBG( "//////////////////////////		CS new CS Sensitivity is 0x%x \n",cs_sensitivity);

	return count;
};
static DEVICE_ATTR(cs_sensitivity, 0664, cs_sensitivity_show, cs_sensitivity_store);

//add by leo for standby mode ++
static ssize_t cs_stby_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 CS_STBY_Sensitivity =0;

	CS_STBY_Sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_SENSITIVITY);

	return sprintf(buf, "CS Standby Sensitivity = 0x%x\n", CS_STBY_Sensitivity);
};
static ssize_t cs_stby_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 CS_STBY_Sensitivity=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	CS_STBY_Sensitivity = (u8) temp;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_SENSITIVITY, CS_STBY_Sensitivity);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write STANDBY_SENSITIVITY Failed (%d)\n",ret);
	}
	if(unlikely(debug)) LDBG( "//////////////////////////		CS new Standby Sensitivity is 0x%x \n",CS_STBY_Sensitivity);

	return count;
};
static DEVICE_ATTR(cs_stby_sensitivity, 0664, cs_stby_sensitivity_show, cs_stby_sensitivity_store);
//add by leo for standby mode --
//----------------------------------------------------------------------------------------------------add for test
static ssize_t cs_reset_interrupt(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cap1106_i2c_data *cap1106_data=data;

	reset_cap1106_interrupt();
	enable_irq(cap1106_data->irq);
	
	//if(unlikely(debug)) LDBG( "//////////////////////////		cs_reset_interrupt FINISHED \n");
	return sprintf(buf,"cs_reset_interrupt FINISHED\n");
}
static DEVICE_ATTR(cs_reset, 0444,cs_reset_interrupt,NULL);

static ssize_t cs_repeat_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS_RepeatRate=0;
	
	CS_RepeatRate = i2c_smbus_read_byte_data(cap1106_i2c_client, REPEAT_RATE_ENABLE);
	if (CS_RepeatRate < 0)
	{	
		if(CS_STATUSMSG) LDBG( "CS Read REPEAT_RATE_ENABLE Failed\n");
		return sprintf(buf,"Read ID Register Failure\n");
	}
	
	//if(unlikely(debug)) LDBG( "//////////////////////////		cs_reset_interrupt FINISHED \n");
	return sprintf(buf,"CS Repeat Rate is 0x%x\n",CS_RepeatRate);
}
static DEVICE_ATTR(cs_repeat, 0444,cs_repeat_rate,NULL);


// add for threshood and sensitivity ++++++
static ssize_t cs_1_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs_1_sensitivity = 0x%x\n", C_data->cs_sensitivity_1);
};
static ssize_t cs_1_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs_sensitivity_1 = regvalue;

	return count;
};
static DEVICE_ATTR(cs_1_sensitivity, 0664, cs_1_sensitivity_show, cs_1_sensitivity_store);

static ssize_t cs_2_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs_2_sensitivity = 0x%x\n", C_data->cs_sensitivity_2);
};
static ssize_t cs_2_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs_sensitivity_2 = regvalue;

	return count;
};
static DEVICE_ATTR(cs_2_sensitivity, 0664, cs_2_sensitivity_show, cs_2_sensitivity_store);



//add by leo for test ++
static ssize_t cs1_clibration_register_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, 0xb1);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_CHANNEL Failed\n");
	}
	return sprintf(buf, "cs1_clibration_register = 0x%x\n", ret);
};
static DEVICE_ATTR(cs1_cali_reg, 0444,cs1_clibration_register_show,NULL);

static ssize_t cs6_clibration_register_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, 0xb6);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_CHANNEL Failed\n");
	}
	return sprintf(buf, "cs6_clibration_register = 0x%x\n", ret);
};
static DEVICE_ATTR(cs6_cali_reg, 0444,cs6_clibration_register_show,NULL);

static ssize_t cs1_get_max_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i=0,CS1_DeltaCount=0,cs1_max=0;

	for (i=0;i<30;i++){
		CS1_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
		if(CS_STATUSMSG) LDBG( "CS1_DeltaCount = %+d\n",(char)CS1_DeltaCount);
		if(CS1_DeltaCount>cs1_max){
			cs1_max=CS1_DeltaCount;
		}
		msleep(50);
	}
	return sprintf(buf, "cs1_max = %+d\n", (char)cs1_max);
};
static DEVICE_ATTR(cs1_get_max, 0444,cs1_get_max_show,NULL);

static ssize_t cs6_get_max_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i=0,CS6_DeltaCount=0,cs6_max=0;

	for (i=0;i<30;i++){
		CS6_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
		if(CS_STATUSMSG) LDBG( "CS6_DeltaCount = %+d\n",(char)CS6_DeltaCount);
		if(CS6_DeltaCount>cs6_max){
			cs6_max=CS6_DeltaCount;
		}
		msleep(50);
	}
	return sprintf(buf, "cs6_max = %+d\n", (char)cs6_max);
};
static DEVICE_ATTR(cs6_get_max, 0444,cs6_get_max_show,NULL);

static ssize_t cs_report_test_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "nothing\n");
};
static ssize_t cs_report_test_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int temp=0;
	sscanf (buf, "%d", &temp);

	if(temp==1){
		cap1106_send_signal(SENSED);
	}else if(temp==0){
		cap1106_send_signal(NOT_SENSED);
	}

	return count;
};
static DEVICE_ATTR(cs_direct_report, 0664, cs_report_test_show, cs_report_test_store);

//add by leo for test --
	
static struct attribute *cap1106_attributes[] = {
	&dev_attr_cap1106_id.attr,	//cat only
	&dev_attr_cs_en.attr,
	&dev_attr_cs_info.attr,		//cat only
	&dev_attr_cs_status.attr,	//cat only
	&dev_attr_cs_noise.attr,	//cat only
	&dev_attr_cs2_thd.attr,
	&dev_attr_cs2_delta.attr,	//cat only
	&dev_attr_cs4_thd.attr,
	&dev_attr_cs4_delta.attr,	//cat only
	&dev_attr_cs6_delta.attr,	//cat only
	&dev_attr_cs6_thd.attr,
	&dev_attr_high_low_thd.attr,
	&dev_attr_cs_sensitivity.attr,
	&dev_attr_cs_cycle.attr,
	&dev_attr_cs_gain.attr,	
	&dev_attr_cs_reset.attr,		//cat only
	&dev_attr_cs_repeat.attr,	//cat only
	&dev_attr_cs_stby_en.attr,			// add by leo for standby mode
	&dev_attr_cs_stby_cycle.attr,		// add by leo for standby mode
	&dev_attr_cs_stby_sensitivity.attr,	// add by leo for standby mode
	&dev_attr_cs_stby_thd.attr,			// add by leo for standby mode
	&dev_attr_cap_status.attr,
	&dev_attr_cap_check.attr, 				// add by leo to detect cap sensor chip
	&dev_attr_capsensor_exist_check.attr,		// add by leo to detect cap sensor chip
	&dev_attr_capsensor_status_check.attr, 	// add by leo to detect cap sensor chip
	//add by josh ++++++
	&dev_attr_cs_1_sensitivity.attr,
	&dev_attr_cs_2_sensitivity.attr,
	//add by josh ------
	&dev_attr_cs1_cali_reg.attr,	
	&dev_attr_cs6_cali_reg.attr,
	&dev_attr_cs1_get_max.attr,
	&dev_attr_cs6_get_max.attr,
	&dev_attr_cs_report.attr,
	&dev_attr_cs_direct_report.attr,
	&dev_attr_cs_switch_enable.attr, // add by Tom for RF enable/disable cs in user image
	&dev_attr_cs_disable.attr, // add by Tom for RF disable cs in user image
	&dev_attr_cs_enable.attr, // add by Tom for RF enable cs in user image
	&dev_attr_cs_hand_table_en.attr, // add by Tom for BodySAR Test
	NULL
};

static const struct attribute_group cap1106_attr_group = {
	.attrs = cap1106_attributes,
};

// Create a fixed path for cap sensor APK development
static int cap1106_sysfs_init(void)
{
	int ret;

	LDBG("create asus_cap_sensor sysfs \n");
	cap1106_kobj = kobject_create_and_add("asus_cap_sensor", kernel_kobj);
	if (cap1106_kobj == NULL) 
	{
		LDBG("asus_cap_sensor subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	
	ret = sysfs_create_group(cap1106_kobj, &cap1106_attr_group);
	if (ret) 
	{
		LDBG("create asus_cap_sensor subsystem_register failed\n");
		return ret;
	}
	return ret;
}

//========================================================================================
static int cap1106_open(struct inode *inode, struct file *file)
{
	if(unlikely(debug)) LDBG( "//////////////////////////		CS MISC Device Open \n");
	return nonseekable_open(inode, file);
}
static int cap1106_release(struct inode *inode, struct file *file)
{
	if(unlikely(debug)) LDBG( "//////////////////////////		CS MISC Device Release \n");
	return 0;
}
static ssize_t cap1106_read (struct file *file, char __user *buf, size_t n, loff_t *ppos)
{
	if(unlikely(debug)) LDBG( "//////////////////////////		CS MISC Device Read \n");
	return 0;
}
static ssize_t cap1106_write(struct file *file, const char __user *buf, size_t n, loff_t *ppos)
{
	char pidbuf[10];

	if(n > 10)
		return -EINVAL;

	if (copy_from_user (pidbuf, buf, n)) {
		if(CS_STATUSMSG) LDBG( "CS MISC Device Write Failed\n");
		return n;
	}
	sscanf (pidbuf, "%d", &notify_daemon_pid);	
	if(unlikely(debug)) LDBG( "//////////////////////////		CS MISC Device Write \n");
	if(unlikely(debug)) LDBG( "//////////////////////////		notify_daemon_pid = %d\n",notify_daemon_pid);

	isDaemonReady=1; // add by leo for system crash issue ++

// add by leo for modem reset ++
	if(isSendSignalSuccess<0){
		msleep(500);
		if(unlikely(debug)) LDBG( "//////////////////////////		do work function again\n");
		//disable_irq_nosync(cap1106_data->irq);
		queue_delayed_work(cap1106_wq, &cap1106_work, 0);
	}
// add by leo for modem reset --

	return n;
}
static long cap1106_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val=0, err=0, temp=0; 
	
	//struct CAP1106_i2c_data *cap1106_data = data;
	if(unlikely(debug)) LDBG( "//////////////////////////			CS IOCTL (Cmd is %d) \n", _IOC_NR(cmd));
	
	if (_IOC_TYPE(cmd) != CAPSENSOR_IOCTL_MAGIC)
	{
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) 
	{
		return -EFAULT;
	}
	
	switch (cmd) {
	case CAPSENSOR_IOCTL_ENABLE:
		 		
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val){
			if(unlikely(debug)) LDBG( "//////////////////////////		CAPSENSOR_IOCTL_ENABLE - cs_enable \n");
			temp =1;
		}else{
			if(unlikely(debug)) LDBG( "//////////////////////////		CAPSENSOR_IOCTL_ENABLE - cs_disable \n");
			temp =0;
		}
		return temp;
		break;
			
	case CAPSENSOR_IOCTL_GET_ENABLED:

		if(unlikely(debug)) LDBG( "//////////////////////////		CAPSENSOR_IOCTL_GET_ENABLED (return %d)\n",temp);
		
		return put_user(temp, (unsigned long __user *)arg);
		break;
	
	default:
		if(CS_STATUSMSG) LDBG( "Incorrect Cmd  (%d) \n", _IOC_NR(cmd));
		return -EINVAL;
	}
	
	return 0;
}
static struct file_operations cs_fops = {
	.owner   = THIS_MODULE,
	.open    = cap1106_open,
	.release = cap1106_release,
	.read    = cap1106_read,
	.write   = cap1106_write,
	.unlocked_ioctl = cap1106_ioctl
};
static struct miscdevice cs_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "cap1106_cs_misc_dev",
	.fops  = &cs_fops,
};

static int cap1106_setup(struct cap1106_i2c_data *cap1106_data)
{
	int err = -EINVAL;

	/*register CS input device*/
	cap1106_data->cs_input_dev = input_allocate_device();
	if (!cap1106_data->cs_input_dev) {
		if(CS_STATUSMSG) LDBG( "Could Not Allocate CS Input Device\n");
		return -ENOMEM;
	}
	cap1106_data->cs_input_dev->name = "cap1106_cs_input_dev";
	set_bit(EV_ABS, cap1106_data->cs_input_dev->evbit);
	set_bit(EV_SYN, cap1106_data->cs_input_dev->evbit);
	input_set_abs_params(cap1106_data->cs_input_dev, ABS_SENSED, 0, 1, 0, 0);
	
	err = input_register_device(cap1106_data->cs_input_dev);
	if (err < 0) {
		if(CS_STATUSMSG) LDBG( "Could Not Register CS Input Device\n");
		goto register_cs_input_device_err;
	}


  	/*register CS misc device*/
	err = misc_register(&cs_misc_dev);
	if (err < 0) {
		if(CS_STATUSMSG) LDBG( "Could Not Register CS Misc Device \n");
		goto register_cs_misc_device_err;
	}

	if(unlikely(debug)) LDBG( "//////////////////////////		CS_SETUP - FINISHED \n");
	return 0;

	
register_cs_misc_device_err:
	input_unregister_device(cap1106_data->cs_input_dev);
register_cs_input_device_err:
	input_free_device(cap1106_data->cs_input_dev);	
	return err;
}

static int cap1106_initial(struct cap1106_i2c_data *cap1106_data)
{
	int ret=0, regvalue=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if (regvalue < 0){
		if(CS_STATUSMSG) LDBG("Read MAIN_CONTROL Failed\n");
		return regvalue;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue | SET_DSLEEP));
	if (ret<0){
		if(CS_STATUSMSG) LDBG("SET_DSLEEP Failed\n");
		return ret;
	}

	//CS_STBY_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, 0x00); //disable all standby mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "disable standby mode sensors failed (%d)\n",ret);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CHANNEL);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS read STANDBY_CHANNEL failed (%d)\n",ret);
		return ret;
	}

	if(CS_STATUSMSG) LDBG( "CS_STBY_EN = 0x%02x\n",ret);

	//CS_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, 0x00); //disable all normal mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "disable normal mode sensors failed (%d)\n",ret);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS read SENSOR_INPUT_ENABLE failed (%d)\n",ret);
		return ret;
	}

	if(CS_STATUSMSG) LDBG( "CS_EN = 0x%02x\n",ret);
	return ret;
}
/*
static int cap1106_set_standby_mode(struct cap1106_i2c_data *cap1106_data)
{
	int ret=0, regvalue=0, PowerState=0, enable=0;
	u8 GainValue=0;

	//CS_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, 0x00); //disable normal mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "disable normal mode sensors failed\n");
		return ret;
	}

	//CS GAIN, STBY, DSLEEP
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if(regvalue<0){
		if(CS_STATUSMSG) LDBG( "read MAIN_CONTROL failed\n");
		return regvalue;	
	}

	GainValue = 0x03; //0x03 = 0000 0011
	regvalue = ((regvalue & 0x3f) | (GainValue<<6)); //0x3f = 0011 1111
//	PowerState = (entry_mode==4? (regvalue | SET_DSLEEP):((regvalue |SET_STBY) &CLEAN_DSLEEP)); //add by leo for charge mode
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, PowerState);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS set standby mode 0x%02x failed (%d)\n",PowerState,ret);
		return ret;
	}
	if(CS_STATUSMSG) LDBG( "CS MAIN_CONTROL Register = 0x%02x\n",PowerState);

	//CS_STBY_SENSITIVITY
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_SENSITIVITY, 0x05);// 0x05 = 00000 101, Sensitivity multiplier: 4x (default: 32x));
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write STANDBY_SENSITIVITY failed\n");
		return ret;
	}

	//CS_STBY_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_THRESHOLD, 0x30); // 0x30 = 0011 0000, Threshold: 48 (default: 64));
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write STANDBY_THRESHOLD failed\n");
		return ret;
	}

	//INT MODE
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, REPEAT_RATE_ENABLE, 0); //0: only generate INT when a touch is detected and a release is detected
	if (ret < 0)
	{
		if(CS_STATUSMSG) LDBG( "CS write REPEAT_RATE_ENABLE failed\n");
		return ret;
	}

	//CS_STBY_EN
//	enable = (entry_mode==4? 0x00:0x21);//add by leo for charge mode
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, enable); //only enable cs1 & cs6, 0x21 = 00 100001
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS set enable 0x%02x failed (%d)\n",enable,ret);
		return ret;
	}

	isStandbyMode=1;
	if(CS_STATUSMSG) LDBG( "CS set to Standby Mode ++\n");
	return ret;
}
*/
// add by leo for normal mode ++
static int cap1106_set_normal_mode(struct cap1106_i2c_data *cap1106_data)
{
	int ret=0, regvalue=0, PowerState=0,  temp=0;
	u8 GainValue=0;

	//CS_STBY_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, 0x00); //disable standby mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "disable standby mode sensors failed\n");
		return ret;
	}

	//CS_SENSITIVITY
	//ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, 0x5f); // 0x5F = 0 101 1111, Sensitivity multiplier: 4x (default: 32x));
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, C_data->cs_sensitivity_1); // add by josh for test
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSITIVITY_CONTROL failed\n");
		return ret;
	}

	//CS GAIN, STBY, DSLEEP
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if(regvalue<0){
		if(CS_STATUSMSG) LDBG( "read MAIN_CONTROL failed\n");
		return regvalue;	
	}
	
	//GainValue = 0x01; //0x01 = 0000 0001
	GainValue = C_data->cs_gain_1; //add by josh for test
	regvalue = ((regvalue & 0x3f) | (GainValue<<6)); //0x3f = 00111111
	PowerState = (entry_mode==4? (regvalue | SET_DSLEEP):((regvalue & CLEAN_STBY) & CLEAN_DSLEEP));
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, PowerState);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS set normal mode 0x%02x failed (%d)\n",PowerState,ret);
		return ret;
	}
	if(CS_STATUSMSG) LDBG( "CS MAIN_CONTROL Register = 0x%02x\n",PowerState);

	//CS2_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT2_THD,  C_data->cs2_high_thd); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT2_THD failed (%d)\n",ret);
	}
	
	//CS4_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT4_THD,  C_data->cs4_high_thd); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT4_THD failed (%d)\n",ret);
	}

	//CS6_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD,  C_data->cs6_high_thd); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT6_THD failed (%d)\n",ret);
	}

	//INT MODE
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, REPEAT_RATE_ENABLE, 0); //0: only generate INT when a touch is detected and a release is detected
	if (ret < 0)
	{
		if(CS_STATUSMSG) LDBG( "CS write REPEAT_RATE_ENABLE failed\n");
		return ret;
	}
	
// add by leo for test ++
	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, CONFIGRURATION2);
	if(ret<0){
		if(CS_STATUSMSG) LDBG( "read CONFIGRURATION2 failed\n");
		return ret;	
	}
	if(CS_STATUSMSG) LDBG( "CONFIGRURATION2 =0x%02x (before)\n",ret);
	
	temp=ret|0x04;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION2, temp);
	if (ret < 0)
	{
		if(CS_STATUSMSG) LDBG( "CS write CONFIGRURATION2 failed\n");
		return ret;
	}
	if(CS_STATUSMSG) LDBG( "CONFIGRURATION2 =0x%02x (after)\n",temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG, 0x60);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write AVG_AND_SAMP_CONFIG Failed (%d)\n",ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, RECALIBRATION_CONFIG, 0x67);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write RECALIBRATION_CONFIG Failed (%d)\n",ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_NOISE_THD, 0x00);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSOR_INPUT_NOISE_THD Failed (%d)\n",ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_CONFIG, 0x04);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSOR_INPUT_CONFIG Failed (%d)\n",ret);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MULTI_TOUCH_BLOCK_CONFIG, 0x84);
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS Write SENSOR_INPUT_CONFIG Failed (%d)\n",ret);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION1, 0x30); //0xb0
	if (ret < 0){
		if(CS_STATUSMSG) LDBG("Read CONFIGRURATION1 Failed (%d)\n",ret);
		return ret;
	}

// add by leo for test --

	/*
	   Sensor Input Enable : 00101010
	   Bit 5 - CS6_EN - Enables the CS6 input to be included during the sampling cycle.
	   Bit 3 - CS4_EN - Enables the CS4 input to be included during the sampling cycle.
	   Bit 1 - CS2_EN - Enables the CS2 input to be included during the sampling cycle. 
	 */
	CS_ENABLE_STATUS = (entry_mode == 4 ? 0x00 : 0x2A); 
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, CS_ENABLE_STATUS); 
	if (ret < 0) {
		if(CS_STATUSMSG) LDBG( "CS set CS_ENABLE_STATUS 0x%02x failed (%d)\n",CS_ENABLE_STATUS,ret);
		return ret;
	}

	isStandbyMode=0;
	if(CS_STATUSMSG) LDBG( "CS set to Normal Mode ++\n");
	return ret;
}
// add by leo for normal mode --

//========================================================================================
// add by leo for bodySAR_notify_daemon ++
static int cap1106_send_signal (sensor_status s)
{
	if (s == NOT_SENSED) {
		CS_UEvent_Status = NOT_SENSED;
		switch_set_state(&data->cap1106_sdev, 0);	
		LDBG( "CS Report [0] Not Detect \n");
	} else if(s == SENSED) {
		CS_UEvent_Status = SENSED;
		switch_set_state(&data->cap1106_sdev, 1);	
		LDBG( "CS Report [1] Detect \n");
	} else {
		CS_UEvent_Status = UNKNOW;
		switch_set_state(&data->cap1106_sdev, 2);	
		LDBG( "CS Report [2] Unknown \n");
	}

	return 0;
}
// add by leo for bodySAR_notify_daemon --

#ifdef DOUBLE_SENSITIVITY
/*
static void cap1106_get_data(void){

	struct cap1106_i2c_data *cap1106_data=data;

	cap1106_data->cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
	cap1106_data->cs_2_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
	cap1106_data->cs_6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
	cap1106_data->cs_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
		
	if(unlikely(debug)) LDBG( "cs_2_data: 0x%02X, cs_6_data: 0x%02X, status = 0x%02x, cs_sensitivity: 0x%02x\n",cap1106_data->cs_2_data,cap1106_data->cs_6_data,cap1106_data->cs_status,cap1106_data->cs_sensitivity);
	if(unlikely(debug)) LDBG( "SensitivityMode: %s\n",SensitivityMode==HIGH_SENSITIVITY?"HIGH_SENSITIVITY":"LOW_SENSITIVITY");

	return;
}
*/
// add by josh for change sensitivity ++++++
static void cap1106_change_sensitivity(int mode){
	
#if 0   /* Add by Tom Cheng for ZT581KL only use chanel 4 for check table or hand */ 
	int ret=0;
	u8 cs_sensitivity = 0, cs2_thd = 0, cs4_thd = 0, cs6_thd = 0;
	
	if(mode == HIGH_SENSITIVITY){
		cs_sensitivity = C_data->cs_sensitivity_1;
		cs2_thd = C_data->cs2_high_thd;
		cs4_thd = C_data->cs4_high_thd;
		cs6_thd = C_data->cs6_high_thd;
	}else{
		cs_sensitivity = C_data->cs_sensitivity_2;
		//cs1_thd = C_data->cs2_low_thd;
		//cs4_thd = C_data->cs4_low_thd;
		//cs6_thd = C_data->cs6_low_thd;
		cs2_thd = 0x01;	//important !!
		cs4_thd = 0x01;	//important !!
		cs6_thd = 0x01;	//important !!
	}
	
	//CS2_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT2_THD, cs2_thd ); 
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT1_THD failed (%d)\n",ret);
	}
	
	//CS4_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT4_THD, cs4_thd ); 
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT1_THD failed (%d)\n",ret);
	}

	//CS6_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD, cs6_thd ); 
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT6_THD failed (%d)\n",ret);
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, cs_sensitivity ); 
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSITIVITY_CONTROL failed\n");
	}

	msleep(100);
	return;
#else
	int ret=0;
	u8 cs_sensitivity = 0, cs4_thd = 0;
	
	if(mode == HIGH_SENSITIVITY){
		cs_sensitivity = C_data->cs_sensitivity_1;
		cs4_thd = C_data->cs4_high_thd;
	}else{
		cs_sensitivity = C_data->cs_sensitivity_2;
		cs4_thd = 0x01;	// force trigger
	}
	
	//CS4_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT4_THD, cs4_thd ); 
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSOR_INPUT1_THD failed (%d)\n",ret);
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, cs_sensitivity ); 
	if (ret < 0){
		if(CS_STATUSMSG) LDBG( "CS write SENSITIVITY_CONTROL failed\n");
	}

	msleep(100);
	return;
#endif
}
// add by josh for change sensitivity ------

static int cap1106_recalibration_check(void){
#if 0   /* Add by Tom Cheng for ZT581KL only use chanel 4 for check table or hand */ 
	int cs_2_data = 0; 
	int cs_4_data = 0;
	int cs_6_data = 0;
	
	cs_2_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_DELTA);
	cs_4_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);
	cs_6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
	if(unlikely(debug)) LDBG( "cs_2_data: [0x%02X], cs2_low_thd = [0x%02x]\n",cs_2_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs2_high_thd : C_data->cs2_low_thd);
	if(unlikely(debug)) LDBG( "cs_4_data: [0x%02X], cs4_low_thd = [0x%02x]\n",cs_4_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs4_high_thd : C_data->cs4_low_thd);
	if(unlikely(debug)) LDBG( "cs_6_data: [0x%02X], cs6_low_thd = [0x%02x]\n",cs_6_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs6_high_thd : C_data->cs6_low_thd);

	/* check is still in table mode */
	if ((cs_2_data <= C_data->cs2_low_thd) || 
	    (cs_4_data <= C_data->cs4_low_thd) || 
	    (cs_6_data <= C_data->cs6_low_thd)) {
		return 1;
	}
	return 0;
#else
	int cs_4_data = 0;
	
	cs_4_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);
	if(unlikely(debug)) LDBG( "cs_4_data: [0x%02X], cs4_low_thd = [0x%02x]\n",cs_4_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs4_high_thd : C_data->cs4_low_thd);

	/* check is still in table mode */
	if (cs_4_data <= C_data->cs4_low_thd)
		return 1;
	else
		return 0;
#endif
}

static int cap1106_check_hands_or_table(int n){

	int ret=0, table_count = 0, recal=1, retries_times=10, debounce_delay=300;
#if 0   /* Add by Tom Cheng for ZT581KL only use chanel 4 for check table or hand */ 
	int cs_det_status = 0, cs_2_data=0, cs_4_data=0, cs_6_data=0, regvalue=0;
#else
	int cs_det_status = 0, cs_4_data=0, regvalue=0;
#endif
	u8 cs_sensitivity = 0;

	if(unlikely(debug)) LDBG( "check hands or table [%d]\n", n);

	// retry time out
	if(n == 0) 
	{
		if(CS_STATUSMSG) LDBG( "Time Out, Change to High Sensitivity \n");
		cap1106_change_sensitivity(HIGH_SENSITIVITY);
		SensitivityMode = HIGH_SENSITIVITY;
		return 0;
	}

	if (SensitivityMode == LOW_SENSITIVITY) {

#if 0   /* Add by Tom Cheng for ZT581KL only use chanel 4 for check table or hand */ 
		cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
		cs_2_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_DELTA);
		cs_4_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);
		cs_6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
		cs_det_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);

		if(unlikely(debug)) LDBG( "Sensitivity Mode: [%s] - [0x%02X] - Now Status [0x%02X]\n",
				SensitivityMode == HIGH_SENSITIVITY ? "HIGH" : "LOW", cs_sensitivity, cs_det_status);
		if(unlikely(debug)) LDBG( "cs_2_data: [0x%02X], cs2_low_thd = [0x%02X]\n",cs_2_data, 
				SensitivityMode == HIGH_SENSITIVITY ? C_data->cs2_high_thd : C_data->cs2_low_thd);
		if(unlikely(debug)) LDBG( "cs_4_data: [0x%02X], cs4_low_thd = [0x%02X]\n",cs_4_data, 
				SensitivityMode == HIGH_SENSITIVITY ? C_data->cs4_high_thd : C_data->cs4_low_thd);
		if(unlikely(debug)) LDBG( "cs_6_data: [0x%02X], cs6_low_thd = [0x%02X]\n",cs_6_data, 
				SensitivityMode == HIGH_SENSITIVITY ? C_data->cs6_high_thd : C_data->cs6_low_thd);


		/* If cs data are empty */
		if ((cs_2_data == 0x00 || cs_2_data == 0xFF) 
		 && (cs_4_data == 0x00 || cs_4_data == 0xFF)
		 && (cs_6_data == 0x00 || cs_6_data == 0xFF))
		{
			if(unlikely(debug)) LDBG( "[TABLE] CS data is re-calibration , switch to [HIGH_SENSITIVITY]\n");
			cap1106_change_sensitivity(HIGH_SENSITIVITY);
			SensitivityMode = HIGH_SENSITIVITY;
			recal = 0;
		}

		/* If cs data > low sensitivity threshold */
		if (((cs_2_data > C_data->cs2_low_thd) && (cs_2_data <= 0x7F)) || 
		    ((cs_4_data > C_data->cs4_low_thd) && (cs_4_data <= 0x7F)) || 
		    ((cs_6_data > C_data->cs6_low_thd) && (cs_6_data <= 0x7F)))
		{
			if(unlikely(debug)) LDBG( "[HAND] CS data > low sensitivity threshold , switch to [HIGH_SENSITIVITY]\n");
			cap1106_change_sensitivity(HIGH_SENSITIVITY);
			SensitivityMode = HIGH_SENSITIVITY;
			recal = 0;
		}
#else 
		cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
		cs_4_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);
		cs_det_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);

		if(unlikely(debug)) LDBG( "Sensitivity Mode: [%s] - [0x%02X] - Now Status [0x%02X]\n",
				SensitivityMode == HIGH_SENSITIVITY ? "HIGH" : "LOW", cs_sensitivity, cs_det_status);
		if(unlikely(debug)) LDBG( "cs_4_data: [0x%02X], cs4_low_thd = [0x%02X]\n",cs_4_data, 
				SensitivityMode == HIGH_SENSITIVITY ? C_data->cs4_high_thd : C_data->cs4_low_thd);


		/* If cs data are empty */
		if (cs_4_data == 0x00 || cs_4_data == 0xFF)
		{
			if(unlikely(debug)) LDBG( "[TABLE] CS data is re-calibration , switch to [HIGH_SENSITIVITY]\n");
			cap1106_change_sensitivity(HIGH_SENSITIVITY);
			SensitivityMode = HIGH_SENSITIVITY;
			recal = 0;
		}

		/* If cs data > low sensitivity threshold */
		if ((cs_4_data > C_data->cs4_low_thd) && (cs_4_data <= 0x7F))
		{
			if(unlikely(debug)) LDBG( "[HAND] CS data > low sensitivity threshold , switch to [HIGH_SENSITIVITY]\n");
			cap1106_change_sensitivity(HIGH_SENSITIVITY);
			SensitivityMode = HIGH_SENSITIVITY;
			recal = 0;
		}
#endif

		/* Table Mode */
		if(recal == 1) {

			for (retries_times = 10; retries_times > 0; retries_times--)
			{
				if (cap1106_recalibration_check() == 1) table_count++;
				if(unlikely(debug)) LDBG( "re-confirm table count = %d\n", table_count);
				msleep (debounce_delay); // sleep 300 ms 
			}

			if (table_count == 10) {
				regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, CONFIGRURATION1);
				if (regvalue < 0){
					if(CS_STATUSMSG) LDBG("Read CONFIGRURATION1 Failed (%d)\n",regvalue);
					return regvalue;
				}
				/*
				   MAX_DUR_EN
				   '1' The maximum duration recalibration functionality is enabled.If a touch is held for longer than 
				   the MAX_DUR bit settings, then the re-calibration routine will be restarted
				 */
				regvalue = regvalue | 0x08;
				ret = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION1, regvalue);
				if (ret < 0){
					if(CS_STATUSMSG) LDBG( "CS Write CONFIGRURATION1 Failed (%d)\n",ret);
					return ret;
				}
				if(unlikely(debug)) LDBG( "CONFIGRURATION1 = 0x%02x (after)\n",regvalue);

				recal = 0;
				status_stable_count = 0;
				LDBG("[Table] Re-calibration for Cap Sensor\n");
			}
			table_count = 0;
			cap1106_check_hands_or_table(n-1);
		}
	}	
	return 0;
}
#endif

static sensor_status get_cap1106_status(void)
{
	u8 status = 0;
	u8 is_detected = 0;

	status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
	if (status < 0){	
		if(CS_STATUSMSG) LDBG( "Read SENSOR_INPUT_STATUS Failed \n");
	}
	is_detected = status & 0x2A;
	if(is_detected != 0x00){
		return SENSED;
	}else{
		return NOT_SENSED;
	}

 	return UNKNOW;
}
static int reset_cap1106_interrupt(void)
{
	int ret = 0;
	u8 regvalue = 0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if(regvalue<0){
		if(CS_STATUSMSG) LDBG( "Read MAIN_CONTROL Failed \n");
		return regvalue;
	}
	/*
	   Bit 0 - INT - Indicates that there is an interrupt. This bit is only set if the ALERT# pin has been asserted.
	   If a channel detects a touch and its associated interrupt enable bit is not set to a logic '1' action is taken. 
	   This bit is cleared by writing a logic '0' it. Whenthis bit is cleared, the ALERT# pin will be deasserted
	   and all status registers will be cleared if the condition has been removed.
	   '0' - interrupt pending.
	   '1' - touch has been detected on one or morechannels and the interrupt has been asserted.
	 */
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue & 0xfe));	//0xfe = 11111110
	if(ret<0){
		if(CS_STATUSMSG) LDBG( "Reset Interrupt Pin Failed \n");
		return ret;	
	}

	return 0;
}
static irqreturn_t cap1106_interrupt_handler(int irq, void *dev_id)
{
	if(unlikely(debug)) LDBG( "		INTTERUPT \n");
	//disable_irq_nosync(irq);
	queue_delayed_work(cap1106_wq, &cap1106_work, 0);
	
	return IRQ_HANDLED;
}
static void cap1106_work_function(struct work_struct *work)
{	
	int err=-1, retries_times = 10, debounce_delay = 100, retry=10, sense_retry=10, not_sense_retry=3;
	int regvalue=0;
#ifdef DOUBLE_SENSITIVITY
#if 0   /* Mark by Tom Cheng for ZT581KL only use chanel 4 for check table or hand */ 
	int cs_2_data = 0, cs_4_data = 0, cs_6_data = 0;
	int cs_det_status = 0;
	u8 cs_sensitivity = 0;
	u8 is_detected = 0;
#else
	int cs_4_data = 0;
	int cs_det_status = 0;
	u8 cs_sensitivity = 0;
	u8 is_detected = 0;
#endif
#endif
	struct cap1106_i2c_data *cap1106_data = data;
	/*
	   MAX_DUR_EN
	   Determines whether the maximum duration recalibration is enabled
	   '0' (default) - The maximum duration recalibration functionality isdisabled. A touch may be held 
	   indefinitely and no re-calibration will be performed on any sensor input. 
	   '1' The maximum duration recalibration functionality is enabled.If a touch is held for longer than 
	   the MAX_DUR bit settings, then the re-calibration routine will be restarted
	*/
	int MAX_DUR_EN = 0;

	disable_irq(cap1106_data->irq);
	reset_cap1106_interrupt();

	// Disable recalibration when PRE-CS dectect status is NOT_SENSED
	if(CS_Status == NOT_SENSED){
		regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, CONFIGRURATION1);
		if(regvalue < 0){
			if(CS_STATUSMSG) LDBG( "read CONFIGRURATION1 failed\n");
		}

		MAX_DUR_EN = regvalue & 0x08;
		if(MAX_DUR_EN){
			regvalue = regvalue & ~0x08;	// Disable Re-calibration
			if(unlikely(debug)) LDBG( "clean MAX_DUR_EN\n");
		}

		err = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION1, regvalue);
		if (err < 0){
			if(CS_STATUSMSG) LDBG( "CS Write CONFIGRURATION1 Failed (%d)\n",err);
		}
	}

#ifdef DOUBLE_SENSITIVITY
#if 0   /* Mark by Tom Cheng for ZT581KL only use chanel 4 for check table or hand */ 
	cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
	cs_2_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_DELTA);
	cs_4_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);
	cs_6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
	cs_det_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);

	if(unlikely(debug)) LDBG( "Sensitivity Mode: [%s] - [0x%02X] - Now Status [0x%02X]\n",
			SensitivityMode == HIGH_SENSITIVITY ? "HIGH" : "LOW", cs_sensitivity, cs_det_status);
	if(unlikely(debug)) LDBG( "cs_2_data: [0x%02X], cs2_high_thd = [0x%02X]\n",cs_2_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs2_high_thd : C_data->cs2_high_thd);
	if(unlikely(debug)) LDBG( "cs_4_data: [0x%02X], cs4_high_thd = [0x%02X]\n",cs_4_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs4_high_thd : C_data->cs4_high_thd);
	if(unlikely(debug)) LDBG( "cs_6_data: [0x%02X], cs6_high_thd = [0x%02X]\n",cs_6_data, 
			SensitivityMode == HIGH_SENSITIVITY ? C_data->cs6_high_thd : C_data->cs6_high_thd);

	if(SensitivityMode == HIGH_SENSITIVITY){
		/* 
		   If cs_2 or cs_4 or cs_6 is detected and raw data equal 0x7f (max) ,
		   then check hands or table 
		 */
		is_detected = cs_det_status & 0x2A;
		if((is_detected != 0x00) && 
		   (cs_2_data == 0x7f || cs_4_data == 0x7f || cs_6_data == 0x7f)) {
			C_data->overflow_status = cs_det_status;
//			if(unlikely(debug)) LDBG( "status_stable_count = [%d]\n", status_stable_count);

			if(status_stable_count < 5){
				cap1106_change_sensitivity(LOW_SENSITIVITY);
				SensitivityMode = LOW_SENSITIVITY;
				confirm_count = 0;
				status_stable_count++;
				cap1106_check_hands_or_table(10);
			} else {
				confirm_count = 6; 
			}
		}
#else
	if(likely(IS_DOUBLE_SENSITIVITY)) {
		cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
		cs_4_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT4_DELTA);
		cs_det_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);

		if(unlikely(debug)) LDBG( "Sensitivity Mode: [%s] - [0x%02X] - Now Status [0x%02X]\n",
				SensitivityMode == HIGH_SENSITIVITY ? "HIGH" : "LOW", cs_sensitivity, cs_det_status);
		if(unlikely(debug)) LDBG( "cs_4_data: [0x%02X], cs4_high_thd = [0x%02X]\n",cs_4_data, 
				SensitivityMode == HIGH_SENSITIVITY ? C_data->cs4_high_thd : C_data->cs4_high_thd);
	}

	if((SensitivityMode == HIGH_SENSITIVITY) && (build_version != 1)){
		/* If cs_4 is detected and raw data equal 0x7f (max), then check hands or table */
		is_detected = cs_det_status & 0x08;
		if((is_detected != 0x00) && (cs_4_data == 0x7f) && likely(IS_DOUBLE_SENSITIVITY)) {
			C_data->overflow_status = cs_det_status;

			if(status_stable_count < 5){
				cap1106_change_sensitivity(LOW_SENSITIVITY);
				SensitivityMode = LOW_SENSITIVITY;
				confirm_count = 0;
				status_stable_count++;
				cap1106_check_hands_or_table(10);
			} else {
				confirm_count = 6; 
			}
		}
#endif
#endif

		CS_Status = get_cap1106_status();

		if(unlikely(debug)) LDBG( "CS_Status[0x%2X], CS_OldStatus[0x%2X]\n",CS_Status, CS_OldStatus);
		if((CS_Status == CS_OldStatus) && (isSendSignalSuccess == 0)){	// add by leo for modem reset ++
			if(unlikely(debug)) LDBG( "CS State doesn't changed \n");
			goto cs_status_didnt_change;
		}

		retry=(CS_Status == SENSED ? sense_retry : not_sense_retry);
		for (retries_times = retry; retries_times > 0; retries_times--)
		{
			if (gpio_get_value(CAP1106_INTERRUPT_GPIO) == 0){
				reset_cap1106_interrupt();
				goto cs_status_unstable;
			}
			msleep (debounce_delay);
			if(unlikely(debug)) LDBG( "re-confirm_times = %d\n",retries_times);
		}

		// Notify bodySAR_notify_daemon 
		if((build_version != 1)){
			err = cap1106_send_signal(CS_Status);
			if(err < 0) {
				if(CS_STATUSMSG) LDBG( "send signal to bodySAR_notify Failed (%d)\n",err);
			}
			isSendSignalSuccess = err;
		}

		CS_OldStatus = CS_Status;

#ifdef DOUBLE_SENSITIVITY
	}
#endif

cs_status_unstable:
cs_status_didnt_change:

#ifdef DOUBLE_SENSITIVITY
	if(likely(IS_DOUBLE_SENSITIVITY)) {
		is_detected = cs_det_status & 0x08; 
		if((is_detected != 0x00) && SensitivityMode == HIGH_SENSITIVITY && confirm_count < 5 && (build_version != 1)){
			if(unlikely(debug)) LDBG( "===== ReConfirm [%d] times . status_stable_count[%d] =====\n",confirm_count, status_stable_count);
			queue_delayed_work(cap1106_wq, &cap1106_work, msecs_to_jiffies(1000));
			confirm_count++;
		} else {
			confirm_count = 0;
		}

		// if(cs_det_status == 0){ Mark by Tom Cheng For CS4 Stable
		if(is_detected == 0){
			status_stable_count = 0;
		}
	}
#endif

//	if((debug)) LDBG( "Work_function FINISHED \n");
	enable_irq(cap1106_data->irq);

	return;
}


static int cap1106_power_ctl(struct cap1106_i2c_data *data, bool on)
{
	int ret = 0;
	int err = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->i2c_client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->i2c_client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->i2c_client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->i2c_client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else {
		dev_info(&data->i2c_client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int cap1106_power_init(struct cap1106_i2c_data *data)
{
	int ret = 0;

	data->vdd = regulator_get(&data->i2c_client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->i2c_client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				CAP1106_VDD_MIN_UV,
				CAP1106_VDD_MAX_UV);
		if (ret) {
			dev_err(&data->i2c_client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->i2c_client->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		dev_err(&data->i2c_client->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
				CAP1106_VIO_MIN_UV,
				CAP1106_VIO_MAX_UV);
		if (ret) {
			dev_err(&data->i2c_client->dev,
			"Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, CAP1106_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}
static int cap1106_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
//	struct cap1106_platform_data *pdata; // add by leo for cap1106_platfor_data
	struct cap1106_i2c_data *cap1106_data=NULL;
	struct cap1106_config_data *config_data = NULL;//add by josh

	if(CS_STATUSMSG) LDBG("==========2016/10/25========== \n");
	if(CS_STATUSMSG) LDBG( "build version = %d\n",build_version); //add by leo for read build version
	
	if(!(cap1106_data = kzalloc(sizeof(struct cap1106_i2c_data), GFP_KERNEL))){
		if(build_version!=1)return -ENOMEM;
	}
	memset(cap1106_data, 0, sizeof(struct cap1106_i2c_data));

	cap1106_data->i2c_client = client;
	i2c_set_clientdata(client, cap1106_data);
	cap1106_i2c_client = cap1106_data->i2c_client;	// we defined cap1106_i2c_client
	cap1106_i2c_client->flags = 1;
	strlcpy(cap1106_i2c_client->name, CAP1106_I2C_DEVICE_NAME, I2C_NAME_SIZE);
	data = cap1106_data;
	
	if(!(config_data = kzalloc(sizeof(struct cap1106_config_data), GFP_KERNEL))){
		if(build_version!=1)return -ENOMEM;
	}
	memset(config_data, 0, sizeof(struct cap1106_config_data));
	
	
	/* power init */
	err = cap1106_power_init(cap1106_data);
	if (err) {
		LDBG("Failed to get sensor regulators\n");
		err = -EINVAL;
		goto cap1106_check_device_err;
	}
	err = cap1106_power_ctl(cap1106_data, true);
	if (err) {
		LDBG("Failed to enable sensor power\n");
		err = -EINVAL;
		goto cap1106_check_device_err;
	}
	RESET_DELAY();
	
	/* Add by Tom Cheng for modify sensitivity to 0x2f for SMMI Test */
	if(build_version == 1) {
		config_data->cs_sensitivity_1 = 0x2f; 
		LDBG("Modify sensitivity to [0x%2X] for SMMI Test\n", config_data->cs_sensitivity_1);
	} else {
		config_data->cs_sensitivity_1 = 0x0f; 
	}

	config_data->cs_gain_1 = 0x03;
	config_data->cs2_high_thd = 0x0a;
	config_data->cs4_high_thd = 0x0a;

	/* Add by Tom Cheng for modify cs6 threshold to 0x1a for SMMI Test */
	if(build_version == 1) {
		config_data->cs6_high_thd = 0x1a;
		LDBG("Modify cs6 threshold to [0x%2X] for SMMI Test \n", config_data->cs6_high_thd);
	} else {
		config_data->cs6_high_thd = 0x0a;
	}

	config_data->cs_sensitivity_2 = 0x3f; // Modify by tom for ZT500KL Hand or Table Sensitivity
	config_data->cs_gain_2 = config_data->cs_gain_1;
	config_data->cs2_low_thd = 0x2f; // modify by tom for fine tune threshold
	config_data->cs4_low_thd = 0x3f; // Modify by Tom for ZT500KL Hand or Table Threshold
	config_data->cs6_low_thd = 0x2f; // modify by tom for fine tune threshold
	config_data->overflow_status = 0x0;
	C_data = config_data;
	
	CAP1106_INTERRUPT_GPIO = of_get_named_gpio_flags(client->dev.of_node,"microchip,gpio-irq", 0, NULL);
	LDBG("CAP1106_INTERRUPT_GPIO [%d]\n",CAP1106_INTERRUPT_GPIO);

	cap1106_initial(cap1106_data);

	err = cap1106_check_device(cap1106_data);
	if (err<0){
		if(CS_STATUSMSG) LDBG("cap1106_i2c_probe Failed\n");
		if(build_version!=1)goto cap1106_check_device_err;
	}

	cap1106_wq = create_singlethread_workqueue("cap1106_wq");
	if (!cap1106_wq) {
		if(CS_STATUSMSG) LDBG("Create WorkQueue Failed\n");
		err = -ENOMEM;
		if(build_version!=1)goto create_singlethread_workqueue_err;
	}
	INIT_DEFERRABLE_WORK(&cap1106_work, cap1106_work_function);
	
	cap1106_create_proc_file(); 
	cap1106_setup(cap1106_data);	
	cap1106_set_normal_mode(cap1106_data);
	
	err = sysfs_create_group(&client->dev.kobj, &cap1106_attr_group);
	if (err){
		if(CS_STATUSMSG) LDBG("Register sysfs Failed\n");
		if(build_version!=1)goto sysfs_create_group_err;
	}

	cap1106_setup_irq(cap1106_data); // modify flow by leo for system crash issue

	reset_cap1106_interrupt();
	
	LDBG("create sdev for ril ...\n");
	data->cap1106_sdev.name = "cap_sensor_detect";
	data->cap1106_sdev.print_name = cap1106_switch_name;
	data->cap1106_sdev.print_state = cap1106_switch_state;
	if(switch_dev_register(&data->cap1106_sdev) < 0)
	{
		LDBG("switch_dev_register failed!\n");
	}

	// Create a fixed path for cap sensor APK development
	cap1106_sysfs_init();
	
	if(CS_STATUSMSG) LDBG("====================== \n");

	return 0;
	
sysfs_create_group_err:	
	cap1106_remove_proc_file(); // add by leo for proc file ++
create_singlethread_workqueue_err:
cap1106_check_device_err:
//	gpio_free(CAP1106_INTERRUPT_GPIO);
	kfree(cap1106_data);
	return err;
}

static int cap1106_suspend(struct i2c_client *client , pm_message_t mesg)
{

	int ret=0;
	u8 regvalue = 0;
	//struct cap1106_i2c_data *cap1106_data = data;
	
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if (regvalue < 0){
		if(build_version!=1)if(CS_STATUSMSG) LDBG("Read MAIN_CONTROL Failed\n");
		if(build_version!=1)return regvalue;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue | SET_DSLEEP));
	if (ret<0){
		if(build_version!=1)if(CS_STATUSMSG) LDBG("SET_DSLEEP Failed\n");
		if(build_version!=1)return ret;
	}
	
	LDBG("SET_DSLEEP - Finished !!\n");
	
	/* Disable HW Power 
	if (cap1106_power_ctl(cap1106_data, false)) {
		LDBG("Disable HW Power Fail\n");
		return 0;
	}
	*/
	return 0;
}

static int cap1106_resume(struct i2c_client *client)
{

	int ret=0;
	u8 regvalue = 0;
//	struct cap1106_i2c_data *cap1106_data = data;
	
	/* Enable HW Power
	if (cap1106_power_ctl(cap1106_data, true)) {
		LDBG("Enable HW Power Fail\n");
		return 0;
	}
	*/

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if (regvalue < 0){
		if(build_version!=1)if(CS_STATUSMSG) LDBG("Read MAIN_CONTROL Failed\n");
		if(build_version!=1)return regvalue;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue & CLEAN_DSLEEP));
	if (ret<0){
		if(build_version!=1)if(CS_STATUSMSG) LDBG("CLEAN_DSLEEP Failed\n");
		if(build_version!=1)return ret;
	}
	
	LDBG("CLEAN_DSLEEP - Finished !!\n");

	//disable_irq_wake(data->irq);
	return 0;
}

static int __exit cap1106_i2c_remove(struct i2c_client *client)
{
	if(CS_STATUSMSG) LDBG("\n");
	cap1106_remove_proc_file(); // add by leo for proc file ++
	sysfs_remove_group(&client->dev.kobj, &cap1106_attr_group);
	kfree(i2c_get_clientdata(client));
	cap1106_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id cap1106_i2c_idtable[] = {
	{"cap1106", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cap1106_i2c_idtable);

static struct i2c_driver cap1106_i2c_driver = {
	
	.driver = {
		.name = CAP1106_I2C_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = 		cap1106_i2c_probe,
	.suspend	=	cap1106_suspend,
	.resume	=	cap1106_resume,
	.remove	=	cap1106_i2c_remove,
	.id_table = 	cap1106_i2c_idtable,	
};

static int __init cap1106_init(void)
{
	int ret;
 	ret = i2c_add_driver(&cap1106_i2c_driver);
	if ( ret != 0 ) {
		LDBG("Couldn't add cap1106_i2c_driver (ret = %d) \n",ret);
		return ret;
	}else{
		LDBG("Success !!\n");
		return ret;
	}
}

static void __exit cap1106_exit(void)
{
	i2c_del_driver(&cap1106_i2c_driver);
	gpio_free(CAP1106_INTERRUPT_GPIO);//leo added
	misc_deregister(&cs_misc_dev);//leo added
	destroy_workqueue (cap1106_wq);//leo added
}

module_init(cap1106_init);
module_exit(cap1106_exit);

MODULE_AUTHOR("Asus Tek. <asus@asus.com>");
MODULE_DESCRIPTION("SMSC CAP1106 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
