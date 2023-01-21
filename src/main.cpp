#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/WrenchStamped.h>

#include "../include/ftconfig.h"


extern "C"
{
    #include <comedilib.h>
}

using namespace std;

/* This application consist of 3 parts
 *
 * The first part sets up the DAQ and reads the required values from it.
 * The second part uses the ATI force sensor calibration libraries to convert the RAW voltages into force readings
 * The last part implements the ROS node that can read the data.
 */


/* DAQ Variables and Function Prototypes*/

extern comedi_t *device;

struct parsed_options {
    char *filename;
    double value;
    int subdevice;
    int channel;
    int aref;
    int range;
    int verbose;
    int n_chan;
    int n_scan;
    double freq;
};

#define BUFSZ 24
char buf[BUFSZ];

#define N_CHANS 12
static unsigned int chanlist[N_CHANS];
static comedi_range * range_info[N_CHANS];
static lsampl_t maxdata[N_CHANS];

int prepare_cmd_lib(comedi_t *dev, int subdevice, int n_scan,
            int n_chan, unsigned period_nanosec,
            comedi_cmd *cmd);

void initialize_device(comedi_t *dev, comedi_cmd *cmd, parsed_options options);

void do_cmd(comedi_t *dev,comedi_cmd *cmd);

int read_DAQ(comedi_t* dev,parsed_options options,int subdev_flags,char * bufaddr,double * voltage_array ,bool verbose);

int syncRead_DAQ(comedi_t* dev,parsed_options options, double * voltage_array);

void print_datum(lsampl_t raw, int channel_index);

long double get_time_ms(void);

void calc_diff_voltage(double * voltage_array, float * diff_voltage_array);

void write_to_Wrench(float * force_torque, geometry_msgs::WrenchStamped* wrenchptr);

char *cmdtest_messages[] = {
    "success",
    "invalid source",
    "source conflict",
    "invalid argument",
    "argument conflict",
    "invalid chanlist",
};

/* End DAQ Variables */


int main(int argc, char *argv[])
{
	
	/* ROS NODE SETUP*/
	ros::init(argc,argv,"force_sensor_pub");
	ros::NodeHandle node;
	ros::Rate(1000);
	ros::Publisher pub = node.advertise<geometry_msgs::WrenchStamped>("force_sensor",10);
	geometry_msgs::WrenchStamped wrench_data;
	
	/* DAQ SET UP AND DIAGNOSTICS */

    comedi_t *dev;
    comedi_cmd c,*cmd = &c;
    int ret, daq_ret;
    int i;
    
    struct parsed_options options;

    memset(&options, 0, sizeof(options));
    options.filename = "/dev/comedi0";
    options.subdevice = 0;
    options.channel = 0;
    options.range = 0;
    options.aref = AREF_GROUND;
    options.n_chan = 12;
    options.n_scan = 1;
    options.freq = 1000.0;

    int preset_channel_list[options.n_chan] = {0,1,2,3,4,5,8,9,10,11,12,13}; 

    /* open the device */
    dev = comedi_open(options.filename);
    if (!dev) {
        comedi_perror(options.filename);
        exit(1);
    }

    /* Print numbers for clipped inputs */
    comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);

    /* Set up channel list */
    for (i = 0; i < options.n_chan; i++) {
        chanlist[i] =
            CR_PACK(preset_channel_list[i], options.range,
                options.aref);
        range_info[i] =
            comedi_get_range(dev, options.subdevice,
                     options.channel, options.range);
        maxdata[i] =
            comedi_get_maxdata(dev, options.subdevice,
                       options.channel);
    }
    
    initialize_device(dev,cmd,options);
    
    /* DAQ SETUP AND CALIBRATION DONE*/
    
    /* FORCE SENSOR CALIBRATION*/
    char calib_file[] = "/home/charm/development/ros_ws/src/daq_fs/src/FT14048.cal";
    unsigned short index = 1;
    short sts;
    Calibration *cal;
    
    cal = createCalibration(calib_file,index);
    if (cal==NULL)
    {
    	printf("Calibration file not loaded!\n");
    	return 0;
    }
    
    sts = SetForceUnits(cal,"N");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid force units"); return 0;
		default: printf("Unknown error"); return 0;
	}
	

	sts=SetTorqueUnits(cal,"N-m");
	switch (sts) {
		case 0: break;	// successful completion
		case 1: printf("Invalid Calibration struct"); return 0;
		case 2: printf("Invalid torque units"); return 0;
		default: printf("Unknown error"); return 0;
	}
	
	/* END OF FT CALIBRATION*/
    
    /* FORCE SENSOR-DAQ READING */
    
    // Initialize a while loop that polls pulls readings from the buffer every 1ms
    int subdev_flags;
	int col;
	long double t_curr,t_samp;
	double voltage_readings[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; // declare an array to hold our voltage readings
	float diff_voltage[6] = {0,0,0,0,0,0}; //declare an array for differential voltages.
	float force_torque[6] = {0,0,0,0,0,0}; // converted force torques
	
    subdev_flags = comedi_get_subdevice_flags(dev, options.subdevice);
	//ret = comedi_command(dev, cmd);
	if (ret < 0) {
		comedi_perror("comedi_command");
		exit(1);
	}
	subdev_flags = comedi_get_subdevice_flags(dev, options.subdevice);
	
	t_curr = get_time_ms();
	t_samp = get_time_ms();
	long double period;
	
	// Take an initial reading to do a zero offset
	printf("zeroing forces... \n");
	while (1){
	
		t_curr = get_time_ms();
		period = t_curr-t_samp;

		if (period>1000){
		//ret = read_DAQ(dev,options,subdev_flags,buf, voltage_readings ,false);
		ret = syncRead_DAQ(dev,options,voltage_readings);
		calc_diff_voltage(voltage_readings,diff_voltage);
		Bias(cal,diff_voltage);
		printf("zeroed! \n");
		break;
		}
	}
	
	// Start the main acquisition loop
	
	while (ros::ok()) {
		t_curr = get_time_ms();
		period = t_curr-t_samp;
				
		if (period>1)
		{   
		    t_samp = get_time_ms();
		    //ret = read_DAQ(dev,options,subdev_flags,buf, voltage_readings ,false);
		    ret = syncRead_DAQ(dev,options,voltage_readings);
		    calc_diff_voltage(voltage_readings,diff_voltage);
		    ConvertToFT(cal,diff_voltage,force_torque);
		    write_to_Wrench(force_torque, &wrench_data);
		    //printf("%.3llf  ",1000/period);
		    //for(i = 0; i<12;i++) printf("%.2f ",voltage_readings[i]); 
		    //for(i = 0; i<6;i++) printf("%.2f ",diff_voltage[i]);
		    //for(i = 0; i<3;i++) printf("%.2f ",force_torque[i]);
		    //printf("\n");
            //if (ret<=0)
                //break; 
            pub.publish(wrench_data);
		}
	}
    
} // end of main function

int prepare_cmd_lib(comedi_t *dev, int subdevice, int n_scan,
		    int n_chan, unsigned scan_period_nanosec,
		    comedi_cmd *cmd)
{   
    /*
     * This prepares a command in a pretty generic way.  We ask the
     * library to create a stock command that supports periodic
     * sampling of data, then modify the parts we want.
     */

	int ret;

	memset(cmd,0,sizeof(*cmd));

	/* This comedilib function will get us a generic timed
	 * command for a particular board.  If it returns -1,
	 * that's bad. */
	ret = comedi_get_cmd_generic_timed(dev, subdevice, cmd, n_chan,
					   scan_period_nanosec);
	if (ret < 0) {
		fprintf(stderr,
			"comedi_get_cmd_generic_timed failed\n");
		return ret;
	}

	/* Modify parts of the command */
	cmd->chanlist = chanlist;
	cmd->chanlist_len = n_chan;
    // set up for continuous acquisition
    cmd->stop_src = TRIG_NONE;
	if (cmd->stop_src == TRIG_NONE) {
		cmd->stop_arg = 0;
	}

	return 0;
}

void initialize_device(comedi_t *dev, comedi_cmd *cmd, parsed_options options)
{
    int ret;

    /* prepare_cmd_lib() uses a Comedilib routine to find a
     * good command for the device.  prepare_cmd() explicitly
     * creates a command, which may not work for your device. */
    prepare_cmd_lib(dev, options.subdevice, options.n_scan,
            options.n_chan, 1e9 / options.freq, cmd);

    /* comedi_command_test() tests a command to see if the
     * trigger sources and arguments are valid for the subdevice.*/
    ret = comedi_command_test(dev, cmd);
    if (ret < 0) {
        comedi_perror("comedi_command_test");
        if(errno == EIO){
            fprintf(stderr,
                "Ummm... this subdevice doesn't support commands\n");
        }
        exit(1);
    }
    ret = comedi_command_test(dev, cmd);
    if (ret < 0) {
        comedi_perror("comedi_command_test");
        exit(1);
    }
    fprintf(stderr,"second test returned %d (%s)\n", ret,cmdtest_messages[ret]);
    if (ret != 0) {
        fprintf(stderr, "Error preparing command\n");
        exit(1);
    }

    /* comedi_set_read_subdevice() attempts to change the current
     * 'read' subdevice to the specified subdevice if it is
     * different.  Changing the read or write subdevice might not be
     * supported by the version of Comedi you are using.  */
    comedi_set_read_subdevice(dev, cmd->subdev);
    /* comedi_get_read_subdevice() gets the current 'read'
     * subdevice. if any.  This is the subdevice whose buffer the
     * read() call will read from.  Check that it is the one we want
     * to use.  */
    ret = comedi_get_read_subdevice(dev);
    if (ret < 0 || ret != cmd->subdev) {
        fprintf(stderr,
            "failed to change 'read' subdevice from %d to %d\n",
            ret, cmd->subdev);
        exit(1);
    }

}

int read_DAQ(comedi_t* dev,parsed_options options,int subdev_flags,char * bufaddr, double * voltage_array ,bool verbose){
    int ret;
	int i;
	int col = 0;
	lsampl_t raw;
	
    ret = read(comedi_fileno(dev),bufaddr,BUFSZ);
	if (ret < 0) {
		/* some error occurred */
		perror("read");
		return ret;
	} else if (ret == 0) {
		/* reached stop condition */
		return ret;
	} else {
		int bytes_per_sample;

		if (subdev_flags & SDF_LSAMPL) {
			bytes_per_sample = sizeof(lsampl_t);
		} else {
			bytes_per_sample = sizeof(sampl_t);
		}
		
		for (i = 0; i < ret / bytes_per_sample; i++) {
		//for (i = 0; i < 12; i++){
			if (subdev_flags & SDF_LSAMPL) {
				raw = ((lsampl_t *)bufaddr)[i];
			} 
			else {
				raw = ((sampl_t *)bufaddr)[i];
			}
			
			if (verbose){
			    print_datum(raw, col);
			    col++;
			    if (col == options.n_chan) {
				    printf("\n");
				    col=0;
			    }
			}
			
			if (i<12) // this line writes the voltages to an array
			    voltage_array[i] = comedi_to_phys(raw, range_info[i],
					maxdata[i]);
			
		}
	}

    return ret;
}

int syncRead_DAQ(comedi_t* dev,parsed_options options, double * voltage_array){
	
	int ret;
	int ch;
	lsampl_t rawdat;
	for (ch=0;ch<12;ch++){
		ret = comedi_data_read(dev,options.subdevice,ch,options.range,options.aref,&rawdat);
		voltage_array[ch] = comedi_to_phys(rawdat,range_info[ch],maxdata[ch]);
		}

}

long double get_time_ms(void)
{
    struct timeval time;
    
    gettimeofday(&time,NULL);
    
    long double time_ms = time.tv_sec*1000LL +time.tv_usec/1000.0;
    
    return time_ms;
    
}

void print_datum(lsampl_t raw, int channel_index)
{
	double physical_value;

	physical_value = comedi_to_phys(raw, range_info[channel_index],
					maxdata[channel_index]);
	printf("%#8.6g ", physical_value);
}


void calc_diff_voltage(double * voltage_array, float * diff_voltage_array)
{
	int i;
	
	for(i=0;i<6;i++){
		diff_voltage_array[i] = voltage_array[i]-voltage_array[i+6];
	}


}

void write_to_Wrench(float * force_torque, geometry_msgs::WrenchStamped* wrenchptr){

	wrenchptr->wrench.force.x = force_torque[0];
	wrenchptr->wrench.force.y = force_torque[1];
	wrenchptr->wrench.force.z = force_torque[2];
	wrenchptr->wrench.torque.x = force_torque[3];
	wrenchptr->wrench.torque.y = force_torque[4];
	wrenchptr->wrench.torque.z = force_torque[5];
	
	
}



