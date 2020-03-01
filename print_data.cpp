/**
 * @file stabilizer.cpp
 * @brief       PID controllers for pitch, roll, and yaw.
 *              Data taken from on-board BeagleBone Blue IMU and TaitBryan angles gathered.
 *              ESC control signals sent after PID control law is called.
 *              Control law frequency can be defined as an input parameter.
 * @author     Felix Blanco
 * @date       3/1/2020
 */

#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <robotcontrol.h> // Imports all subsystems.

using namespace std;

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// Global Variables
static int running = 0;
static int enable_mag = 0;
static int orientation_menu = 0;
static rc_mpu_data_t data;

// local functions
static rc_mpu_orientation_t __orientation_prompt(void);
static void __print_usage(void);
static void __print_data(void);
static void __print_header(void);
/**
 * Printed if some invalid argument was given, or -h option given.
 */
static void __print_usage(void)
{
        printf("\n Options\n");
        printf("-r {rate}       Set sample rate in HZ (default 100)\n");
        printf("                Sample rate must be a divisor of 200\n");
        printf("-m              Enable Magnetometer\n");
        printf("-o              Show a menu to select IMU orientation\n");
        printf("-h              Print this help message\n\n");
        return;
}
/**
 * This is the IMU interrupt function.
 */
static void __print_data(void)
{
        printf("\r");
        printf(" ");
        printf("%6.1f %6.1f %6.1f |",   data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                        data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                        data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
        fflush(stdout);
        return;
}
/**
 * Based on which data is marked to be printed, print the correct labels. this
 * is printed only once and the actual data is updated on the next line.
 */
static void __print_header(void)
{
        printf(" ");
        printf(" DMP TaitBryan (deg) |");
        printf("\n");
}
/**
 * @brief      interrupt handler to catch ctrl-c
 */
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}
/**
 * If the user selects the -o option for orientation selection, this menu will
 * displayed to prompt the user for which orientation to use. It will return a
 * valid rc_mpu_orientation_t when a number 1-6 is given or quit when 'q' is
 * pressed. On other inputs the user will be allowed to enter again.
 *
 * @return     the orientation enum chosen by user
 */
rc_mpu_orientation_t __orientation_prompt(){
        int c;
        printf("\n");
        printf("Please select a number 1-6 corresponding to the\n");
        printf("orientation you wish to use. Press 'q' to exit.\n\n");
        printf(" 1: ORIENTATION_Z_UP\n");
        printf(" 2: ORIENTATION_Z_DOWN\n");
        printf(" 3: ORIENTATION_X_UP\n");
        printf(" 4: ORIENTATION_X_DOWN\n");
        printf(" 5: ORIENTATION_Y_UP\n");
        printf(" 6: ORIENTATION_Y_DOWN\n");
        printf(" 7: ORIENTATION_X_FORWARD\n");
        printf(" 8: ORIENTATION_X_BACK\n");
        while ((c = getchar()) != EOF){
                switch(c){
                case '1':
                        return ORIENTATION_Z_UP;
                        break;
                case '2':
                        return ORIENTATION_Z_DOWN;
                        break;
                case '3':
                        return ORIENTATION_X_UP;
                        break;
                case '4':
                        return ORIENTATION_X_DOWN;
                        break;
                case '5':
                        return ORIENTATION_Y_UP;
                        break;
                case '6':
                        return ORIENTATION_Y_DOWN;
                        break;
                case '7':
                        return ORIENTATION_X_FORWARD;
                        break;
                case '8':
                        return ORIENTATION_X_BACK;
                        break;
                case 'q':
                        printf("Quitting\n");
                        exit(0);
                case '\n':
                        break;
                default:
                        printf("invalid input\n");
                        break;
                }
        }
        return ORIENTATION_Z_UP;
}
/**
 * main() serves to parse user options, initialize the imu and interrupt
 * handler, and wait for the rc_get_state()==EXITING condition before exiting
 * cleanly. The imu_interrupt function print_data() is what actually prints new
 * imu data to the screen after being set with rc_mpu_set_dmp_callback().
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0 on success -1 on failure
 */
int main(int argc, char *argv[])
{
        int c, sample_rate, priority;
        int show_something = 0; // set to 1 when any show data option is given.
        // start with default config and modify based on options

        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;

        // parse arguments
        opterr = 0;
        while ((c=getopt(argc, argv, "r:m:ho"))!=-1 && argc>1){
                switch (c){
                case 'r': // sample rate option
                        sample_rate = atoi(optarg);
                        show_something = 1;
                        if(sample_rate>200 || sample_rate<4){
                                printf("sample_rate must be between 4 & 200");
                                return -1;
                        }
                        conf.dmp_sample_rate = sample_rate;
                        break;
                case 'm': // magnetometer option
                        enable_mag = 1;
                        conf.enable_magnetometer = 1;
                        break;
                case 'o': // let user select imu orientation
                        orientation_menu=1;
                        break;
                case 'h': // show help option
                        __print_usage();
                        return -1;
                        break;
                default:
                        printf("opt: %c\n",c);
                        printf("invalid argument\n");
                        __print_usage();
                        return -1;
                        break;
                }
        }
        // user didn't give an option to show anything. Print warning and return.
        if(show_something==0){
                __print_usage();
                printf("please enable an option to print some data\n");
                return -1;
        }
        // If the user gave the -o option to select an orientation then prompt them
        if(orientation_menu){
                conf.orient=__orientation_prompt();
        }

        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running = 1;

        // now set up the imu for dmp interrupt operation
        if(rc_mpu_initialize_dmp(&data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
        }

        // write labels for what data will be printed and associate the interrupt
        // function to print data immediately after the header.
        __print_header();

        // Set the callback function for the MPU.
        rc_mpu_set_dmp_callback(&__print_data);

        //now just wait, print_data() will be called by the interrupt
        //We could be running something else like reading images IDK.
        while(running)  rc_usleep(100000);

        // shut things down
        rc_mpu_power_off();
        printf("\n");
        fflush(stdout);
        return 0;
}
