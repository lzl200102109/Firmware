#include <nuttx/config.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <poll.h>
#include <float.h>

#include <drivers/drv_led.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/cpuload.h>
#include <systemlib/rc_check.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/state_table.h>
#include <mavlink/mavlink_log.h>
#include <dataman/dataman.h>

//#include "px4_custom_mode.h"
#include "commander_helper.h"
#include "state_machine_helper.h"
#include "calibration_routines.h"
#include "accelerometer_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "baro_calibration.h"
#include "rc_calibration.h"
//#include "airspeed_calibration.h"
#include "PreflightCheck.h"

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_controls.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 50000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))

#define MAVLINK_OPEN_INTERVAL 50000

#define STICK_ON_OFF_LIMIT 0.9f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define FAILSAFE_DEFAULT_TIMEOUT    (3 * 1000 * 1000)   /**< hysteresis time - the failsafe will trigger after 3 seconds in this state */

#define PRINT_MODE_REJECT_INTERVAL  2000000

/* Mavlink file descriptors */
static int mavlink_fd = 0;

/* System autostart ID */
//static int autostart_id;

static struct vehicle_control_mode_s control_mode;
static struct vehicle_status_s status;
static struct actuator_armed_s armed;
static struct safety_s safety;
static unsigned int leds_counter;
static uint64_t last_print_mode_reject_time = 0;

/* flags */
static bool commander_initialized = false;
static volatile bool thread_should_exit = false;    /**< daemon exit flag */
static volatile bool thread_running = false;        /**< daemon status flag */
static int daemon_task;                             /**< Handle of daemon task / thread */
static orb_advert_t status_pub;




/***********************
  function prototypes
************************/

extern "C" __EXPORT int commander_main(int argc, char *argv[]);

void usage(const char *reason);

void control_status_leds(vehicle_status_s *status, const actuator_armed_s *actuator_armed, bool changed);

void set_control_mode();

void print_reject_mode(struct vehicle_status_s *current_status, const char *msg);

transition_result_t set_main_state_rc(struct vehicle_status_s *status, struct manual_control_setpoint_s *sp_man);

void print_reject_arm(const char *msg);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);




/***********************
  function defninitions
************************/


int commander_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("commander already running");
            /* this is not an error */
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = task_spawn_cmd("commander",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 40,
                         3400,
                         commander_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);

        while (!thread_running) {
            usleep(200);
        }

        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {

        if (!thread_running) {
            errx(0, "commander already stopped");
        }

        thread_should_exit = true;

        while (thread_running) {
            usleep(200000);
            warnx(".");
        }

        warnx("terminated.");

        exit(0);
    }

    /* commands needing the app to run below */
    if (!thread_running) {
        warnx("\tcommander not started");
        exit(1);
    }

//    if (!strcmp(argv[1], "status")) {
//        print_status();
//        exit(0);
//    }
//
//    if (!strcmp(argv[1], "calibrate")) {
//        if (argc > 2) {
//            int calib_ret = OK;
//            if (!strcmp(argv[2], "mag")) {
//                calib_ret = do_mag_calibration(mavlink_fd);
//            } else if (!strcmp(argv[2], "accel")) {
//                calib_ret = do_accel_calibration(mavlink_fd);
//            } else if (!strcmp(argv[2], "gyro")) {
//                calib_ret = do_gyro_calibration(mavlink_fd);
//            } else {
//                warnx("argument %s unsupported.", argv[2]);
//            }
//
//            if (calib_ret) {
//                errx(1, "calibration failed, exiting.");
//            } else {
//                exit(0);
//            }
//        } else {
//            warnx("missing argument");
//        }
//    }
//
//    if (!strcmp(argv[1], "check")) {
//        int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
//        int checkres = prearm_check(&status, mavlink_fd_local);
//        close(mavlink_fd_local);
//        warnx("FINAL RESULT: %s", (checkres == 0) ? "OK" : "FAILED");
//        exit(0);
//    }
//
//    if (!strcmp(argv[1], "arm")) {
//        int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
//        arm_disarm(true, mavlink_fd_local, "command line");
//        close(mavlink_fd_local);
//        exit(0);
//    }
//
//    if (!strcmp(argv[1], "disarm")) {
//        int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
//        arm_disarm(false, mavlink_fd_local, "command line");
//        close(mavlink_fd_local);
//        exit(0);
//    }

    usage("unrecognized command");
    exit(1);
}

void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: commander {start|stop|status|calibrate|check|arm|disarm}\n\n");
    exit(1);
}

int commander_thread_main(int argc, char *argv[])
{

    /* ***********************
     * Initializations.
     * ***********************/
    unsigned counter = 0;
    unsigned stick_off_counter = 0;
    unsigned stick_on_counter = 0;

    bool low_battery_voltage_actions_done = false;
    bool critical_battery_voltage_actions_done = false;

    bool status_changed = true;
    bool updated = false;
    bool arm_tune_played = false;

    if (led_init() != OK) {
        mavlink_and_console_log_critical(mavlink_fd, "ERROR: LED INIT FAIL");
    }

    if (buzzer_init() != OK) {
        mavlink_and_console_log_critical(mavlink_fd, "ERROR: BUZZER INIT FAIL");
    }

    if (battery_init() != OK) {
        mavlink_and_console_log_critical(mavlink_fd, "ERROR: BATTERY INIT FAIL");
    }

    mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
    rc_calibration_check(mavlink_fd);

    /* ***********************
     * Topics to be published.
     * ***********************/
    /* vehicle control mode topic */
    memset(&control_mode, 0, sizeof(control_mode));
    orb_advert_t control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);

    /* armed topic */    /* Initialize armed with all false */
    memset(&armed, 0, sizeof(armed));
    orb_advert_t armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

    /* vehicle status topic */
    memset(&status, 0, sizeof(status));
    status.rc_input_blocked = false;
    status.main_state =vehicle_status_s::MAIN_STATE_MANUAL;
    status.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;

    /* mark all signals lost as long as they haven't been found */
    status.rc_signal_found_once = false;
    status.rc_signal_lost = true;
    status.data_link_lost = true;

    /* set battery warning flag */
    status.battery_warning = vehicle_status_s::VEHICLE_BATTERY_WARNING_NONE;
    status.condition_battery_voltage_valid = false;

    // XXX for now just set sensors as initialized
    status.condition_system_sensors_initialized = true;

    /* publish initial state */
    status_pub = orb_advertise(ORB_ID(vehicle_status), &status);
    if (status_pub < 0) {
        warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
        warnx("exiting.");
        exit(ERROR);
    }

    /* ***********************
     * Topics to be subscribed.
     * ***********************/

    /* Subscribe to telemetry status topics */
    int telemetry_subs[TELEMETRY_STATUS_ORB_ID_NUM];
    uint64_t telemetry_last_heartbeat[TELEMETRY_STATUS_ORB_ID_NUM];
    uint64_t telemetry_last_dl_loss[TELEMETRY_STATUS_ORB_ID_NUM];
    bool telemetry_lost[TELEMETRY_STATUS_ORB_ID_NUM];

    for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
        telemetry_subs[i] = -1;
        telemetry_last_heartbeat[i] = 0;
        telemetry_last_dl_loss[i] = 0;
        telemetry_lost[i] = true;
    }

    /* Subscribe to safety topic */
    int safety_sub = orb_subscribe(ORB_ID(safety));
    memset(&safety, 0, sizeof(safety));
    safety.safety_switch_available = false;
    safety.safety_off = false;

    /* Subscribe to manual control data */
    int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    struct manual_control_setpoint_s sp_man;
    memset(&sp_man, 0, sizeof(sp_man));

    /* Subscribe to local position data */
    int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    struct vehicle_local_position_s local_position;
    memset(&local_position, 0, sizeof(local_position));

    /* Subscribe to sensor topic */
    int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
    struct sensor_combined_s sensors;
    memset(&sensors, 0, sizeof(sensors));

    /* Subscribe to battery topic */
    int battery_sub = orb_subscribe(ORB_ID(battery_status));
    struct battery_status_s battery;
    memset(&battery, 0, sizeof(battery));

    /* Subscribe to actuator controls (outputs) */
    int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
    struct actuator_controls_s actuator_controls;
    memset(&actuator_controls, 0, sizeof(actuator_controls));



    /* ***********************
     * Now initialized
     * ***********************/
    commander_initialized = true;
    thread_running = true;
    control_status_leds(&status, &armed, true);

    /* ***********************
     * Run preflight check
     * ***********************/
    bool checkAirspeed = false;
    status.condition_system_sensors_initialized = Commander::preflightCheck(mavlink_fd, true, true, true, true, checkAirspeed, true);
    if (!status.condition_system_sensors_initialized) {
        set_tune_override(TONE_GPS_WARNING_TUNE);   //sensor fail tune
    }
    else {
        set_tune_override(TONE_STARTUP_TUNE);       //normal boot tune
    }

    const hrt_abstime commander_boot_timestamp = hrt_absolute_time();

    transition_result_t arming_ret;
//    int32_t datalink_loss_enabled = false;
    int32_t datalink_loss_timeout = 10;
    float rc_loss_timeout = 0.5;
    int32_t datalink_regain_timeout = 0;

//    bool arming_state_changed = false;

//    bool arming_state_changed = false;
//    bool main_state_changed = false;

    while (!thread_should_exit) {

        /* try to open the mavlink log device every once in a while */
        if (mavlink_fd < 0 && counter % (1000000 / MAVLINK_OPEN_INTERVAL) == 0) {
            mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        }

        arming_ret = TRANSITION_NOT_CHANGED;

        /* ***********************
         * Subscribe topics
         * ***********************/
        orb_check(sp_man_sub, &updated);
        if (updated) {
            orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
        }

        orb_check(local_position_sub, &updated);
        if (updated) {
            /* position changed */
            orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
        }

        orb_check(sensor_sub, &updated);
        if (updated) {
            orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);
            if (hrt_elapsed_time(&sensors.baro_timestamp) < FAILSAFE_DEFAULT_TIMEOUT) {
                /* handle the case where baro was regained */
                if (status.barometer_failure) {
                    status.barometer_failure = false;
                    status_changed = true;
                    mavlink_log_critical(mavlink_fd, "baro healthy");
                }
            } else {
                if (!status.barometer_failure) {
                    status.barometer_failure = true;
                    status_changed = true;
                    mavlink_log_critical(mavlink_fd, "baro failed");
                }
            }
        }

        for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {

            if (telemetry_subs[i] < 0 && (OK == orb_exists(telemetry_status_orb_id[i], 0))) {
                telemetry_subs[i] = orb_subscribe(telemetry_status_orb_id[i]);
            }

            orb_check(telemetry_subs[i], &updated);

            if (updated) {
                struct telemetry_status_s telemetry;
                memset(&telemetry, 0, sizeof(telemetry));

                orb_copy(telemetry_status_orb_id[i], telemetry_subs[i], &telemetry);

                /* perform system checks when new telemetry link connected */
                if (mavlink_fd &&
                    telemetry_last_heartbeat[i] == 0 &&
                    telemetry.heartbeat_time > 0 &&
                    hrt_elapsed_time(&telemetry.heartbeat_time) < datalink_loss_timeout * 1e6) {

                    bool chAirspeed = false;
                    /* provide RC and sensor status feedback to the user */
                    (void)Commander::preflightCheck(mavlink_fd, true, true, true, true, chAirspeed, true);
                }

                telemetry_last_heartbeat[i] = telemetry.heartbeat_time;
            }
        }

        /* update safety topic */
        orb_check(safety_sub, &updated);
        if (updated) {
            bool previous_safety_off = safety.safety_off;
            orb_copy(ORB_ID(safety), safety_sub, &safety);

            /* disarm if safety is now on and still armed */
            if (safety.safety_switch_available && !safety.safety_off && armed.armed) {
                arming_state_t new_arming_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? vehicle_status_s::ARMING_STATE_STANDBY :
                                   vehicle_status_s::ARMING_STATE_STANDBY_ERROR);

                if (TRANSITION_CHANGED == arming_state_transition(&status, &safety, new_arming_state, &armed,
                        true /* fRunPreArmChecks */, mavlink_fd)) {
                    mavlink_log_info(mavlink_fd, "DISARMED by safety switch");
//                    arming_state_changed = true;
                }
            }

            //Notify the user if the status of the safety switch changes
            if (safety.safety_switch_available && previous_safety_off != safety.safety_off) {
                printf("2\n");
                if (safety.safety_off) {
                    set_tune(TONE_NOTIFY_POSITIVE_TUNE);

                } else {
                    tune_neutral(true);
                }

                status_changed = true;
            }
        }

        /* ***********************
         * RC input check
         * ***********************/
        if (!status.rc_input_blocked && sp_man.timestamp != 0 &&
            hrt_absolute_time() < sp_man.timestamp + (uint64_t)(rc_loss_timeout * 1e6f)) {

            /* handle the case where RC signal was regained */
            if (!status.rc_signal_found_once) {
                status.rc_signal_found_once = true;
                mavlink_log_critical(mavlink_fd, "detected RC signal first time");
                status_changed = true;

            } else {
                if (status.rc_signal_lost) {
                    mavlink_log_critical(mavlink_fd, "RC SIGNAL REGAINED after %llums",
                                 (hrt_absolute_time() - status.rc_signal_lost_timestamp) / 1000);
                    status_changed = true;
                }
            }
            status.rc_signal_lost = false;

            /* ***********************
             * ARM / DISARM
             * ***********************/
            /* check if left stick is in lower left position and we are in MANUAL mode -> disarm */
            if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
                sp_man.r < -STICK_ON_OFF_LIMIT && sp_man.z < 0.1f) {
                if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {
                    status.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
                    armed.armed = false;
                    printf("STANDBY \n");
                    status_changed = true;
//                    arming_ret = TRANSITION_CHANGED;
                    arming_ret = arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed, true /* fRunPreArmChecks */,
                                         mavlink_fd);
//                    if (arming_ret == TRANSITION_CHANGED) {
//                        arming_state_changed = true;
//                    }
                    stick_off_counter = 0;
                } else {
                    stick_off_counter++;
                }
            } else {
                stick_off_counter = 0;
            }

            /* check if left stick is in lower right position and we're in MANUAL mode -> arm */
            if (status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY &&
                sp_man.r > STICK_ON_OFF_LIMIT && sp_man.z < 0.1f) {
                if (stick_on_counter > STICK_ON_OFF_COUNTER_LIMIT) {
                    if (status.main_state !=vehicle_status_s::MAIN_STATE_MANUAL) {
                        print_reject_arm("NOT ARMING: Switch to MANUAL mode first.");
                    } else {
                        status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
                        armed.armed = true;
                        printf("ARMED \n");
                        status_changed = true;
//                        arming_ret = TRANSITION_CHANGED;
                        arming_ret = arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_ARMED, &armed, true /* fRunPreArmChecks */,
                                                                     mavlink_fd);
//                        if (arming_ret == TRANSITION_CHANGED) {
//                            arming_state_changed = true;
//                        }
                    }
                    stick_on_counter = 0;
                } else {
                    stick_on_counter++;
                }
            } else {
                stick_on_counter = 0;
            }

            if (arming_ret == TRANSITION_CHANGED) {
                if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                    mavlink_log_info(mavlink_fd, "ARMED by RC");

                } else {
                    mavlink_log_info(mavlink_fd, "DISARMED by RC");
                }
//                arming_state_changed = true;
            } else if (arming_ret == TRANSITION_DENIED) {
                /*
                 * the arming transition can be denied to a number of reasons:
                 *  - pre-flight check failed (sensors not ok or not calibrated)
                 *  - safety not disabled
                 *  - system not in manual mode
                 */
                tune_negative(true);
            }

            /* evaluate the main state machine according to mode switches */
            transition_result_t main_res = set_main_state_rc(&status, &sp_man);

            /* play tune on mode change only if armed, blink LED always */
            if (main_res == TRANSITION_CHANGED) {
                tune_positive(armed.armed);
//                main_state_changed = true;

            } else if (main_res == TRANSITION_DENIED) {
                /* DENIED here indicates bug in the commander */
                mavlink_log_critical(mavlink_fd, "main state transition denied");
            }





        }

        /* ***********************
         * Update battery status
         * ***********************/
        orb_check(battery_sub, &updated);
        if (updated) {
            orb_copy(ORB_ID(battery_status), battery_sub, &battery);
            orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &actuator_controls);

            /* only consider battery voltage if system has been running 2s and battery voltage is valid */
            if (hrt_absolute_time() > commander_boot_timestamp + 2000000 && battery.voltage_filtered_v > 0.0f) {
                status.battery_voltage = battery.voltage_filtered_v;
                status.battery_current = battery.current_a;
                status.condition_battery_voltage_valid = true;

                /* get throttle (if armed), as we only care about energy negative throttle also counts */
                float throttle = (armed.armed) ? fabsf(actuator_controls.control[3]) : 0.0f;
                status.battery_remaining = battery_remaining_estimate_voltage(battery.voltage_filtered_v, battery.discharged_mah,
                               throttle);
            }
        }

        /* if battery voltage is getting lower, warn using buzzer, etc. */
        if (status.condition_battery_voltage_valid && status.battery_remaining < 0.18f && !low_battery_voltage_actions_done) {
            low_battery_voltage_actions_done = true;
            if (armed.armed) {
                mavlink_log_critical(mavlink_fd, "LOW BATTERY, RETURN TO LAND ADVISED");
            }
            status.battery_warning = vehicle_status_s::VEHICLE_BATTERY_WARNING_LOW;
            status_changed = true;

        } else if (!status.usb_connected && status.condition_battery_voltage_valid && status.battery_remaining < 0.09f
               && !critical_battery_voltage_actions_done && low_battery_voltage_actions_done) {
            /* critical battery voltage, this is rather an emergency, change state machine */
            critical_battery_voltage_actions_done = true;
            status.battery_warning = vehicle_status_s::VEHICLE_BATTERY_WARNING_CRITICAL;

            if (!armed.armed) {
                arming_ret = arming_state_transition(&status, &safety,
                        vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
                        &armed, true /* fRunPreArmChecks */, mavlink_fd);

                if (arming_ret == TRANSITION_CHANGED) {
//                    arming_state_changed = true;
                    mavlink_and_console_log_critical(mavlink_fd, "LOW BATTERY, LOCKING ARMING DOWN");
                }

            } else {
                mavlink_and_console_log_emergency(mavlink_fd, "CRITICAL BATTERY, LAND IMMEDIATELY");
            }

            status_changed = true;
        }

        /* ***********************
         * Data links check
         * ***********************/
        bool have_link = false;
        for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
            if (telemetry_last_heartbeat[i] != 0 &&
                hrt_elapsed_time(&telemetry_last_heartbeat[i]) < datalink_loss_timeout * 1e6) {
                /* handle the case where data link was gained first time or regained,
                 * accept datalink as healthy only after datalink_regain_timeout seconds
                 * */
                if (telemetry_lost[i] &&
                    hrt_elapsed_time(&telemetry_last_dl_loss[i]) > datalink_regain_timeout * 1e6) {

                    /* only report a regain */
                    if (telemetry_last_dl_loss[i] > 0) {
                        mavlink_and_console_log_critical(mavlink_fd, "data link #%i regained", i);
                    }

                    telemetry_lost[i] = false;
                    have_link = true;

                } else if (!telemetry_lost[i]) {
                    /* telemetry was healthy also in last iteration
                     * we don't have to check a timeout */
                    have_link = true;
                }

            } else {

                if (!telemetry_lost[i]) {
                    /* only reset the timestamp to a different time on state change */
                    telemetry_last_dl_loss[i]  = hrt_absolute_time();

                    mavlink_and_console_log_critical(mavlink_fd, "data link #%i lost", i);
                    telemetry_lost[i] = true;
                }
            }
        }
        if (have_link) {
            /* handle the case where data link was regained */
            if (status.data_link_lost) {
                status.data_link_lost = false;
                status_changed = true;
            }
        } else {
            if (!status.data_link_lost) {
                mavlink_and_console_log_critical(mavlink_fd, "ALL DATA LINKS LOST");
                status.data_link_lost = true;
                status.data_link_lost_counter++;
                status_changed = true;
            }
        }





//        /* print new state */
//        if (arming_state_changed) {
//            status_changed = true;
//            mavlink_log_info(mavlink_fd, "[cmd] arming state: %s", arming_states_str[status.arming_state]);
//            arming_state_changed = false;
//        }

        /* ***********************
         * Publish topics
         * ***********************/

        const hrt_abstime now = hrt_absolute_time();    //Get current timestamp
        /* publish states (armed, control mode, vehicle status) at least with 5 Hz */
        if (counter % (200000 / COMMANDER_MONITORING_INTERVAL) == 0 || status_changed) {
            set_control_mode();
            control_mode.timestamp = now;
            orb_publish(ORB_ID(vehicle_control_mode), control_mode_pub, &control_mode);

            status.timestamp = now;
            orb_publish(ORB_ID(vehicle_status), status_pub, &status);

            armed.timestamp = now;
            orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
        }

        /* play arming and battery warning tunes */
        if (!arm_tune_played && armed.armed) {
            /* play tune when armed */
            tune_positive(armed.armed);
            set_tune(TONE_ARMING_WARNING_TUNE);
            arm_tune_played = true;
        } else if (status.battery_warning == vehicle_status_s::VEHICLE_BATTERY_WARNING_CRITICAL) {
            /* play tune on battery critical */
            set_tune(TONE_BATTERY_WARNING_FAST_TUNE);
        } else if (status.battery_warning == vehicle_status_s::VEHICLE_BATTERY_WARNING_LOW || status.failsafe) {
            /* play tune on battery warning or failsafe */
            set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);
        }

        /* reset arm_tune_played when disarmed */
        if (arm_tune_played && !armed.armed) {
            tune_neutral(true);
            arm_tune_played = false;
        }
        fflush(stdout);
        counter++;

        int blink_state = blink_msg_state();
        if (blink_state > 0) {
            /* blinking LED message, don't touch LEDs */
            if (blink_state == 2) {
                /* blinking LED message completed, restore normal state */
                control_status_leds(&status, &armed, true);
            }

        } else {
            /* normal state */
            control_status_leds(&status, &armed, status_changed);
        }

        status_changed = false;

        usleep(COMMANDER_MONITORING_INTERVAL);

    }

    rgbled_set_mode(RGBLED_MODE_OFF);

    /* close fds */
    led_deinit();
    buzzer_deinit();
    close(sp_man_sub);
    close(local_position_sub);
    close(sensor_sub);
    close(armed_pub);
    close(status_pub);
    close(control_mode_pub);
//    close(offboard_control_mode_sub);
//    close(global_position_sub);
//    close(gps_sub);
//    close(safety_sub);
//    close(cmd_sub);
//    close(subsys_sub);
//    close(diff_pres_sub);
//    close(param_changed_sub);
//    close(battery_sub);
//    close(mission_pub);

    thread_running = false;

    printf("end of commander\n");

    return 0;
}


























void
control_status_leds(vehicle_status_s *status_local, const actuator_armed_s *actuator_armed, bool changed)
{
    /* driving rgbled */
    if (changed) {
        bool set_normal_color = false;
        /* set mode */
        if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
            rgbled_set_mode(RGBLED_MODE_ON);
            set_normal_color = true;

        } else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR || !status.condition_system_sensors_initialized) {
            rgbled_set_mode(RGBLED_MODE_BLINK_FAST);
            rgbled_set_color(RGBLED_COLOR_RED);

        } else if (status_local->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
            rgbled_set_mode(RGBLED_MODE_BREATHE);
            set_normal_color = true;

        } else {    // STANDBY_ERROR and other states
            rgbled_set_mode(RGBLED_MODE_BLINK_NORMAL);
            rgbled_set_color(RGBLED_COLOR_RED);
        }

        if (set_normal_color) {
            /* set color */
            if (status_local->battery_warning == vehicle_status_s::VEHICLE_BATTERY_WARNING_LOW || status_local->failsafe) {
                rgbled_set_color(RGBLED_COLOR_AMBER);
                /* vehicle_status_s::VEHICLE_BATTERY_WARNING_CRITICAL handled as vehicle_status_s::ARMING_STATE_ARMED_ERROR / vehicle_status_s::ARMING_STATE_STANDBY_ERROR */

            } else {
                if (status_local->condition_global_position_valid) {
                    rgbled_set_color(RGBLED_COLOR_GREEN);

                } else {
                    rgbled_set_color(RGBLED_COLOR_BLUE);
                }
            }
        }
    }

    /* give system warnings on error LED, XXX maybe add memory usage warning too */
    if (status_local->load > 0.95f) {
        if (leds_counter % 2 == 0) {
            led_toggle(LED_AMBER);
        }

    } else {
        led_off(LED_AMBER);
    }

    leds_counter++;
}

void
set_control_mode()
{
    /* set vehicle_control_mode according to set_navigation_state */
    control_mode.flag_armed = armed.armed;
    control_mode.flag_external_manual_override_ok = (!status.is_rotary_wing && !status.is_vtol);
    control_mode.flag_control_offboard_enabled = false;

    switch (status.nav_state) {
    case vehicle_status_s::NAVIGATION_STATE_MANUAL:
        control_mode.flag_control_manual_enabled = true;
        control_mode.flag_control_auto_enabled = false;
        control_mode.flag_control_rates_enabled = (status.is_rotary_wing || status.vtol_fw_permanent_stab);
        control_mode.flag_control_attitude_enabled = (status.is_rotary_wing || status.vtol_fw_permanent_stab);
        control_mode.flag_control_altitude_enabled = false;
        control_mode.flag_control_climb_rate_enabled = false;
        control_mode.flag_control_position_enabled = false;
        control_mode.flag_control_velocity_enabled = false;
        control_mode.flag_control_termination_enabled = false;
        break;

    case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
        control_mode.flag_control_manual_enabled = true;
        control_mode.flag_control_auto_enabled = false;
        control_mode.flag_control_rates_enabled = true;
        control_mode.flag_control_attitude_enabled = true;
        control_mode.flag_control_altitude_enabled = true;
        control_mode.flag_control_climb_rate_enabled = true;
        control_mode.flag_control_position_enabled = false;
        control_mode.flag_control_velocity_enabled = false;
        control_mode.flag_control_termination_enabled = false;
        break;

    case vehicle_status_s::NAVIGATION_STATE_POSCTL:
        control_mode.flag_control_manual_enabled = true;
        control_mode.flag_control_auto_enabled = false;
        control_mode.flag_control_rates_enabled = true;
        control_mode.flag_control_attitude_enabled = true;
        control_mode.flag_control_altitude_enabled = true;
        control_mode.flag_control_climb_rate_enabled = true;
        control_mode.flag_control_position_enabled = true;
        control_mode.flag_control_velocity_enabled = true;
        control_mode.flag_control_termination_enabled = false;
        break;

    default:
        break;
    }
}

void print_reject_mode(struct vehicle_status_s *status_local, const char *msg)
{
    hrt_abstime t = hrt_absolute_time();

    if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
        last_print_mode_reject_time = t;
        mavlink_log_critical(mavlink_fd, "REJECT %s", msg);

        /* only buzz if armed, because else we're driving people nuts indoors
        they really need to look at the leds as well. */
        tune_negative(armed.armed);
    }
}

transition_result_t
set_main_state_rc(struct vehicle_status_s *status_local, struct manual_control_setpoint_s *sp_man)
{
    /* set main state according to RC switches */
    transition_result_t res = TRANSITION_DENIED;

//    /* if offboard is set allready by a mavlink command, abort */
//    if (status.offboard_control_set_by_command) {
//        return main_state_transition(status_local,vehicle_status_s::MAIN_STATE_OFFBOARD);
//    }
//
//    /* offboard switch overrides main switch */
//    if (sp_man->offboard_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
//        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_OFFBOARD);
//
//        if (res == TRANSITION_DENIED) {
//            print_reject_mode(status_local, "OFFBOARD");
//            /* mode rejected, continue to evaluate the main system mode */
//
//        } else {
//            /* changed successfully or already in this state */
//            return res;
//        }
//    }
//
//    /* RTL switch overrides main switch */
//    if (sp_man->return_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
//        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_RTL);
//
//        if (res == TRANSITION_DENIED) {
//            print_reject_mode(status_local, "AUTO_RTL");
//
//            /* fallback to LOITER if home position not set */
//            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_LOITER);
//        }
//
//        if (res != TRANSITION_DENIED) {
//            /* changed successfully or already in this state */
//            return res;
//        }
//
//        /* if we get here mode was rejected, continue to evaluate the main system mode */
//    }

    /* offboard and RTL switches off or denied, check main mode switch */
    switch (sp_man->mode_switch) {
    case manual_control_setpoint_s::SWITCH_POS_NONE:
        res = TRANSITION_NOT_CHANGED;
        break;

    case manual_control_setpoint_s::SWITCH_POS_OFF:     // MANUAL
//        if (sp_man->acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
//            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_ACRO);
//
//        } else {
            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_MANUAL);
//        }
        break;

    case manual_control_setpoint_s::SWITCH_POS_MIDDLE:      // ASSIST
        if (sp_man->posctl_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_POSCTL);
            if (res != TRANSITION_DENIED) {
                break;  // changed successfully or already in this state
            }
            print_reject_mode(status_local, "POSCTL");
        }
        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_ALTCTL);
        if (res != TRANSITION_DENIED) {
            break;  // changed successfully or already in this mode
        }
        if (sp_man->posctl_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
            print_reject_mode(status_local, "ALTCTL");
        }
        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_MANUAL);
        break;

    case manual_control_setpoint_s::SWITCH_POS_ON:          // AUTO
//        if (sp_man->loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
//            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_LOITER);
//
//            if (res != TRANSITION_DENIED) {
//                break;  // changed successfully or already in this state
//            }
//
//            print_reject_mode(status_local, "AUTO_LOITER");
//
//        } else {
//            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_MISSION);
//
//            if (res != TRANSITION_DENIED) {
//                break;  // changed successfully or already in this state
//            }
//
//            print_reject_mode(status_local, "AUTO_MISSION");
//
//            // fallback to LOITER if home position not set
//            res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_LOITER);
//
//            if (res != TRANSITION_DENIED) {
//                break;  // changed successfully or already in this state
//            }
//        }

        // fallback to POSCTL
        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_POSCTL);
        if (res != TRANSITION_DENIED) {
            break;  // changed successfully or already in this state
        }
        // fallback to ALTCTL
        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_ALTCTL);
        if (res != TRANSITION_DENIED) {
            break;  // changed successfully or already in this state
        }
        // fallback to MANUAL
        res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_MANUAL);
        // TRANSITION_DENIED is not possible here
        break;

    default:
        break;
    }

    return res;
}

void
print_reject_arm(const char *msg)
{
    hrt_abstime t = hrt_absolute_time();

    if (t - last_print_mode_reject_time > PRINT_MODE_REJECT_INTERVAL) {
        last_print_mode_reject_time = t;
        mavlink_log_critical(mavlink_fd, msg);
        tune_negative(true);
    }
}
