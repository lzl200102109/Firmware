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
#include <systemlib/cpuload.h>
#include <systemlib/rc_check.h>
#include <systemlib/state_table.h>
#include <systemlib/err.h>

#include <mavlink/mavlink_log.h>
#include <dataman/dataman.h>

#include "commander_helper.h"
#include "state_machine_helper.h"
#include "calibration_routines.h"
#include "accelerometer_calibration.h"
#include "gyro_calibration.h"
#include "mag_calibration.h"
#include "baro_calibration.h"
#include "rc_calibration.h"
#include "airspeed_calibration.h"
#include "PreflightCheck.h"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

extern struct system_load_s system_load;

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 50000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))

#define MAVLINK_OPEN_INTERVAL 50000

#define STICK_ON_OFF_LIMIT 0.9f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define POSITION_TIMEOUT		(1 * 1000 * 1000)	/**< consider the local or global position estimate invalid after 1000ms */
#define FAILSAFE_DEFAULT_TIMEOUT	(3 * 1000 * 1000)	/**< hysteresis time - the failsafe will trigger after 3 seconds in this state */
#define OFFBOARD_TIMEOUT		500000
#define DIFFPRESS_TIMEOUT		2000000

#define PRINT_INTERVAL	5000000
#define PRINT_MODE_REJECT_INTERVAL	2000000

/* Mavlink file descriptors */
static int mavlink_fd = 0;

/* flags */
static bool commander_initialized = false;
static volatile bool thread_should_exit = false;	/**< daemon exit flag */
static volatile bool thread_running = false;		/**< daemon status flag */
static int daemon_task;					/**< Handle of daemon task / thread */
static bool need_param_autosave = false;      /**< Flag set to true if parameters should be autosaved in next iteration (happens on param update and if functionality is enabled) */

static unsigned int leds_counter;
/* To remember when last notification was sent */
static uint64_t last_print_mode_reject_time = 0;

static struct vehicle_status_s status;
static struct actuator_armed_s armed;
static struct safety_s safety;
static struct vehicle_control_mode_s control_mode;
static struct offboard_control_mode_s offboard_control_mode;


/***********************
  function prototypes
************************/

extern "C" __EXPORT int commander_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
void usage(const char *reason);

void print_status();

transition_result_t arm_disarm(bool arm, const int mavlink_fd, const char *armedBy);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

void control_status_leds(vehicle_status_s *status, const actuator_armed_s *actuator_armed, bool changed);

transition_result_t set_main_state_rc(struct vehicle_status_s *status, struct manual_control_setpoint_s *sp_man);

void set_control_mode();

void print_reject_mode(struct vehicle_status_s *current_status, const char *msg);

void print_reject_arm(const char *msg);

/**
 * Loop that runs at a lower rate and priority for calibration and parameter tasks.
 */
void *commander_low_prio_loop(void *arg);

void answer_command(struct vehicle_command_s &cmd, enum VEHICLE_CMD_RESULT result);


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

	if (!strcmp(argv[1], "status")) {
		print_status();
		exit(0);
	}

	if (!strcmp(argv[1], "calibrate")) {
		if (argc > 2) {
			int calib_ret = OK;
			if (!strcmp(argv[2], "mag")) {
				calib_ret = do_mag_calibration(mavlink_fd);
			} else if (!strcmp(argv[2], "accel")) {
				calib_ret = do_accel_calibration(mavlink_fd);
			} else if (!strcmp(argv[2], "gyro")) {
				calib_ret = do_gyro_calibration(mavlink_fd);
			} else {
				warnx("argument %s unsupported.", argv[2]);
			}

			if (calib_ret) {
				errx(1, "calibration failed, exiting.");
			} else {
				exit(0);
			}
		} else {
			warnx("missing argument");
		}
	}

	if (!strcmp(argv[1], "check")) {
		int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
		int checkres = prearm_check(&status, mavlink_fd_local);
		close(mavlink_fd_local);
		warnx("FINAL RESULT: %s", (checkres == 0) ? "OK" : "FAILED");
		exit(0);
	}

	if (!strcmp(argv[1], "arm")) {
		int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
		arm_disarm(true, mavlink_fd_local, "command line");
		close(mavlink_fd_local);
		exit(0);
	}

	if (!strcmp(argv[1], "disarm")) {
		int mavlink_fd_local = open(MAVLINK_LOG_DEVICE, 0);
		arm_disarm(false, mavlink_fd_local, "command line");
		close(mavlink_fd_local);
		exit(0);
	}

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

void print_status()
{
	warnx("type: %s", (status.is_rotary_wing) ? "symmetric motion" : "forward motion");
	warnx("usb powered: %s", (status.usb_connected) ? "yes" : "no");
	warnx("avionics rail: %6.2f V", (double)status.avionics_power_rail_voltage);

	/* read all relevant states */
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	struct vehicle_status_s state;
	orb_copy(ORB_ID(vehicle_status), state_sub, &state);

	const char *armed_str;

	switch (state.arming_state) {
	case vehicle_status_s::ARMING_STATE_INIT:
		armed_str = "INIT";
		break;

	case vehicle_status_s::ARMING_STATE_STANDBY:
		armed_str = "STANDBY";
		break;

	case vehicle_status_s::ARMING_STATE_ARMED:
		armed_str = "ARMED";
		break;

	case vehicle_status_s::ARMING_STATE_ARMED_ERROR:
		armed_str = "ARMED_ERROR";
		break;

	case vehicle_status_s::ARMING_STATE_STANDBY_ERROR:
		armed_str = "STANDBY_ERROR";
		break;

	case vehicle_status_s::ARMING_STATE_REBOOT:
		armed_str = "REBOOT";
		break;

	case vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE:
		armed_str = "IN_AIR_RESTORE";
		break;

	default:
		armed_str = "ERR: UNKNOWN STATE";
		break;
	}

	close(state_sub);


	warnx("arming: %s", armed_str);
}

static orb_advert_t status_pub;

transition_result_t arm_disarm(bool arm, const int mavlink_fd_local, const char *armedBy)
{
	transition_result_t arming_res = TRANSITION_NOT_CHANGED;

	// Transition the armed state. By passing mavlink_fd to arming_state_transition it will
	// output appropriate error messages if the state cannot transition.
	arming_res = arming_state_transition(&status, &safety, arm ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_STANDBY, &armed,
					     true /* fRunPreArmChecks */, mavlink_fd_local);

	if (arming_res == TRANSITION_CHANGED && mavlink_fd) {
		mavlink_log_info(mavlink_fd_local, "[cmd] %s by %s", arm ? "ARMED" : "DISARMED", armedBy);

	} else if (arming_res == TRANSITION_DENIED) {
		tune_negative(true);
	}

	return arming_res;
}

int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;

	bool arm_tune_played = false;

	/* pthread for slow low prio thread */
	pthread_t commander_low_prio_thread;

	/* initialize */
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

	/* vehicle status topic */
	memset(&status, 0, sizeof(status));
	status.condition_landed = true;	// initialize to safe value
	// We want to accept RC inputs as default
	status.rc_input_blocked = false;
	status.main_state =vehicle_status_s::MAIN_STATE_MANUAL;
	status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	status.arming_state = vehicle_status_s::ARMING_STATE_INIT;
	status.failsafe = false;

	/* neither manual nor offboard control commands have been received */
	status.offboard_control_signal_found_once = false;
	status.rc_signal_found_once = false;

	/* mark all signals lost as long as they haven't been found */
	status.rc_signal_lost = true;
	status.offboard_control_signal_lost = true;
	status.data_link_lost = true;

	/* set battery warning flag */
	status.battery_warning = vehicle_status_s::VEHICLE_BATTERY_WARNING_NONE;
	status.condition_battery_voltage_valid = false;

	// XXX for now just set sensors as initialized
	status.condition_system_sensors_initialized = true;

	status.counter++;
	status.timestamp = hrt_absolute_time();

	status.condition_power_input_valid = true;
	status.avionics_power_rail_voltage = -1.0f;
	status.usb_connected = false;

	/* publish initial state */
	status_pub = orb_advertise(ORB_ID(vehicle_status), &status);

	if (status_pub < 0) {
		warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
		warnx("exiting.");
		exit(ERROR);
	}

	/* armed topic */
	orb_advert_t armed_pub;
	/* Initialize armed with all false */
	memset(&armed, 0, sizeof(armed));

	/* vehicle control mode topic */
	memset(&control_mode, 0, sizeof(control_mode));
	orb_advert_t control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &control_mode);

	armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

	/* Start monitoring loop */
	unsigned counter = 0;
	unsigned stick_off_counter = 0;
	unsigned stick_on_counter = 0;

	bool low_battery_voltage_actions_done = false;
	bool critical_battery_voltage_actions_done = false;

	hrt_abstime last_idle_time = 0;

	bool status_changed = true;
//	bool param_init_forced = true;

	bool updated = false;

	rc_calibration_check(mavlink_fd);

	/* Subscribe to safety topic */
	int safety_sub = orb_subscribe(ORB_ID(safety));
	memset(&safety, 0, sizeof(safety));
	safety.safety_switch_available = false;
	safety.safety_off = false;

	/* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp_man;
	memset(&sp_man, 0, sizeof(sp_man));

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

	/* Subscribe to system power */
	int system_power_sub = orb_subscribe(ORB_ID(system_power));
	struct system_power_s system_power;
	memset(&system_power, 0, sizeof(system_power));

	/* Subscribe to actuator controls (outputs) */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));

	control_status_leds(&status, &armed, true);

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	// Run preflight check
	status.condition_system_sensors_initialized = Commander::preflightCheck(mavlink_fd, true, true, true, true, false, true);
	if (!status.condition_system_sensors_initialized) {
		set_tune_override(TONE_GPS_WARNING_TUNE); //sensor fail tune
	}
	else {
		set_tune_override(TONE_STARTUP_TUNE); //normal boot tune
	}

	const hrt_abstime commander_boot_timestamp = hrt_absolute_time();

	transition_result_t arming_ret;

	int32_t datalink_loss_timeout = 10;
	float rc_loss_timeout = 0.5;
	int32_t datalink_regain_timeout = 0;

    /* initialize low priority thread */
    pthread_attr_t commander_low_prio_attr;
    pthread_attr_init(&commander_low_prio_attr);
    pthread_attr_setstacksize(&commander_low_prio_attr, 2000);

    struct sched_param param;
    (void)pthread_attr_getschedparam(&commander_low_prio_attr, &param);

    /* low priority */
    param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
    (void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
    pthread_create(&commander_low_prio_thread, &commander_low_prio_attr, commander_low_prio_loop, NULL);
    pthread_attr_destroy(&commander_low_prio_attr);

	while (!thread_should_exit) {

		if (mavlink_fd < 0 && counter % (1000000 / MAVLINK_OPEN_INTERVAL) == 0) {
			/* try to open the mavlink log device every once in a while */
			mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
		}

		arming_ret = TRANSITION_NOT_CHANGED;

		status.is_rotary_wing = true;

		orb_check(sp_man_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
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

                    /* provide RC and sensor status feedback to the user */
                    (void)Commander::preflightCheck(mavlink_fd, true, true, true, true, false, true);
				}

				telemetry_last_heartbeat[i] = telemetry.heartbeat_time;
			}
		}

		orb_check(sensor_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);

			/* Check if the barometer is healthy and issue a warning in the GCS if not so.
			 * Because the barometer is used for calculating AMSL altitude which is used to ensure
			 * vertical separation from other airtraffic the operator has to know when the
			 * barometer is inoperational.
			 * */
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

		orb_check(system_power_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(system_power), system_power_sub, &system_power);

			if (hrt_elapsed_time(&system_power.timestamp) < 200000) {
				if (system_power.servo_valid &&
				    !system_power.brick_valid &&
				    !system_power.usb_connected) {
					/* flying only on servo rail, this is unsafe */
					status.condition_power_input_valid = false;

				} else {
					status.condition_power_input_valid = true;
				}

				/* copy avionics voltage */
				status.avionics_power_rail_voltage = system_power.voltage5V_v;
				status.usb_connected = system_power.usb_connected;
			}
		}

		/* update safety topic */
		orb_check(safety_sub, &updated);

		if (updated) {
			bool previous_safety_off = safety.safety_off;
			orb_copy(ORB_ID(safety), safety_sub, &safety);

			/* disarm if safety is now on and still armed */
			if (
			    safety.safety_switch_available && !safety.safety_off && armed.armed) {
				arming_state_t new_arming_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? vehicle_status_s::ARMING_STATE_STANDBY :
								   vehicle_status_s::ARMING_STATE_STANDBY_ERROR);

				if (TRANSITION_CHANGED == arming_state_transition(&status, &safety, new_arming_state, &armed,
						true /* fRunPreArmChecks */, mavlink_fd)) {
					mavlink_log_info(mavlink_fd, "DISARMED by safety switch");
//					arming_state_changed = true;
				}
			}

			//Notify the user if the status of the safety switch changes
			if (safety.safety_switch_available && previous_safety_off != safety.safety_off) {

				if (safety.safety_off) {
					set_tune(TONE_NOTIFY_POSITIVE_TUNE);

				} else {
					tune_neutral(true);
				}

				status_changed = true;
			}
		}

		/* update local position estimate */
		orb_check(local_position_sub, &updated);

		if (updated) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
		}

		/* update battery status */
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

		if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
			/* compute system load */
			uint64_t interval_runtime = system_load.tasks[0].total_runtime - last_idle_time;

			if (last_idle_time > 0) {
				status.load = 1.0f - ((float)interval_runtime / 1e6f);        //system load is time spent in non-idle
			}

			last_idle_time = system_load.tasks[0].total_runtime;
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
//					arming_state_changed = true;
					mavlink_and_console_log_critical(mavlink_fd, "LOW BATTERY, LOCKING ARMING DOWN");
				}

			} else {
				mavlink_and_console_log_emergency(mavlink_fd, "CRITICAL BATTERY, LAND IMMEDIATELY");
			}

			status_changed = true;
		}

		/* End battery voltage check */

		/* If in INIT state, try to proceed to STANDBY state */
		if (status.arming_state == vehicle_status_s::ARMING_STATE_INIT) {
			arming_ret = arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed, true /* fRunPreArmChecks */,
							     mavlink_fd);

			if (arming_ret == TRANSITION_CHANGED) {
//				arming_state_changed = true;
			}

		}

		/* RC input check */
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

			/* check if left stick is in lower left position and we are in MANUAL or AUTO_READY mode or (ASSIST mode and landed) -> disarm
			 * do it only for rotary wings */
			if (sp_man.r < -STICK_ON_OFF_LIMIT && sp_man.z < 0.1f
			    &&(status.arming_state == vehicle_status_s::ARMING_STATE_ARMED || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR)
			    &&(status.main_state == vehicle_status_s::MAIN_STATE_MANUAL || status.main_state == vehicle_status_s::MAIN_STATE_ACRO || status.condition_landed)
			    &&status.is_rotary_wing
			) {

				if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {
					/* disarm to STANDBY if ARMED or to STANDBY_ERROR if ARMED_ERROR */
					arming_state_t new_arming_state = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? vehicle_status_s::ARMING_STATE_STANDBY :
									   vehicle_status_s::ARMING_STATE_STANDBY_ERROR);
					arming_ret = arming_state_transition(&status, &safety, new_arming_state, &armed, true /* fRunPreArmChecks */,
									     mavlink_fd);

					if (arming_ret == TRANSITION_CHANGED) {
//						arming_state_changed = true;
					    status_changed = true;
					}

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

					/* we check outside of the transition function here because the requirement
					 * for being in manual mode only applies to manual arming actions.
					 * the system can be armed in auto if armed via the GCS.
					 */
					if (status.main_state !=vehicle_status_s::MAIN_STATE_MANUAL) {
						print_reject_arm("NOT ARMING: Switch to MANUAL mode first.");

					} else {
						arming_ret = arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_ARMED, &armed, true /* fRunPreArmChecks */,
										     mavlink_fd);

						if (arming_ret == TRANSITION_CHANGED) {
//							arming_state_changed = true;
						    status_changed = true;
						}
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

//				arming_state_changed = true;

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
//				main_state_changed = true;

			} else if (main_res == TRANSITION_DENIED) {
				/* DENIED here indicates bug in the commander */
				mavlink_log_critical(mavlink_fd, "main state transition denied");
			}

		} else {
			if (!status.rc_signal_lost) {
				mavlink_log_critical(mavlink_fd, "RC SIGNAL LOST (at t=%llums)", hrt_absolute_time() / 1000);
				status.rc_signal_lost = true;
				status.rc_signal_lost_timestamp = sp_man.timestamp;
				status_changed = true;
			}
		}

		/* data links check */
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

		//Get current timestamp
		const hrt_abstime now = hrt_absolute_time();

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
		if (!arm_tune_played && armed.armed && (!safety.safety_switch_available || (safety.safety_switch_available
							&& safety.safety_off))) {
			/* play tune when armed */
			set_tune(TONE_ARMING_WARNING_TUNE);
			arm_tune_played = true;

		} else if (status.battery_warning == vehicle_status_s::VEHICLE_BATTERY_WARNING_CRITICAL) {
			/* play tune on battery critical */
			set_tune(TONE_BATTERY_WARNING_FAST_TUNE);

		} else if (status.battery_warning == vehicle_status_s::VEHICLE_BATTERY_WARNING_LOW || status.failsafe) {
			/* play tune on battery warning or failsafe */
			set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);

		} else {
			set_tune(TONE_STOP_TUNE);
		}

		/* reset arm_tune_played when disarmed */
		if (!armed.armed || (safety.safety_switch_available && !safety.safety_off)) {

			//Notify the user that it is safe to approach the vehicle
			if (arm_tune_played) {
				tune_neutral(true);
			}

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
	close(safety_sub);
	close(battery_sub);

	thread_running = false;

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

		} else {	// STANDBY_ERROR and other states
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

transition_result_t
set_main_state_rc(struct vehicle_status_s *status_local, struct manual_control_setpoint_s *sp_man)
{
	/* set main state according to RC switches */
	transition_result_t res = TRANSITION_DENIED;

	/* if offboard is set allready by a mavlink command, abort */
	if (status.offboard_control_set_by_command) {
		return main_state_transition(status_local,vehicle_status_s::MAIN_STATE_OFFBOARD);
	}

	/* offboard switch overrides main switch */
	if (sp_man->offboard_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_OFFBOARD);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "OFFBOARD");
			/* mode rejected, continue to evaluate the main system mode */

		} else {
			/* changed successfully or already in this state */
			return res;
		}
	}

	/* RTL switch overrides main switch */
	if (sp_man->return_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_RTL);

		if (res == TRANSITION_DENIED) {
			print_reject_mode(status_local, "AUTO_RTL");

			/* fallback to LOITER if home position not set */
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_LOITER);
		}

		if (res != TRANSITION_DENIED) {
			/* changed successfully or already in this state */
			return res;
		}

		/* if we get here mode was rejected, continue to evaluate the main system mode */
	}

	/* offboard and RTL switches off or denied, check main mode switch */
	switch (sp_man->mode_switch) {
	case manual_control_setpoint_s::SWITCH_POS_NONE:
		res = TRANSITION_NOT_CHANGED;
		break;

	case manual_control_setpoint_s::SWITCH_POS_OFF:		// MANUAL
		if (sp_man->acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_ACRO);

		} else {
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_MANUAL);
		}

		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_MIDDLE:		// ASSIST
		if (sp_man->posctl_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_POSCTL);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "POSCTL");
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_ALTCTL);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this mode
		}

		if (sp_man->posctl_switch != manual_control_setpoint_s::SWITCH_POS_ON) {
			print_reject_mode(status_local, "ALTCTL");
		}

		// fallback to MANUAL
		res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_MANUAL);
		// TRANSITION_DENIED is not possible here
		break;

	case manual_control_setpoint_s::SWITCH_POS_ON:			// AUTO
		if (sp_man->loiter_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_LOITER);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO_LOITER");

		} else {
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_MISSION);

			if (res != TRANSITION_DENIED) {
				break;	// changed successfully or already in this state
			}

			print_reject_mode(status_local, "AUTO_MISSION");

			// fallback to LOITER if home position not set
			res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_AUTO_LOITER);

			if (res != TRANSITION_DENIED) {
				break;  // changed successfully or already in this state
			}
		}

		// fallback to POSCTL
		res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_POSCTL);

		if (res != TRANSITION_DENIED) {
			break;  // changed successfully or already in this state
		}

		// fallback to ALTCTL
		res = main_state_transition(status_local,vehicle_status_s::MAIN_STATE_ALTCTL);

		if (res != TRANSITION_DENIED) {
			break;	// changed successfully or already in this state
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

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = true;
		control_mode.flag_control_velocity_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		control_mode.flag_control_manual_enabled = true;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_termination_enabled = false;
		break;


	case vehicle_status_s::NAVIGATION_STATE_LAND:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		/* in failsafe LAND mode position may be not available */
		control_mode.flag_control_position_enabled = status.condition_local_position_valid;
		control_mode.flag_control_velocity_enabled = status.condition_local_position_valid;
		control_mode.flag_control_altitude_enabled = true;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		/* TODO: check if this makes sense */
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = true;
		control_mode.flag_control_termination_enabled = false;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		/* disable all controllers on termination */
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_rates_enabled = false;
		control_mode.flag_control_attitude_enabled = false;
		control_mode.flag_control_position_enabled = false;
		control_mode.flag_control_velocity_enabled = false;
		control_mode.flag_control_altitude_enabled = false;
		control_mode.flag_control_climb_rate_enabled = false;
		control_mode.flag_control_termination_enabled = true;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		control_mode.flag_control_manual_enabled = false;
		control_mode.flag_control_auto_enabled = false;
		control_mode.flag_control_offboard_enabled = true;

		/*
		 * The control flags depend on what is ignored according to the offboard control mode topic
		 * Inner loop flags (e.g. attitude) also depend on outer loop ignore flags (e.g. position)
		 */
		control_mode.flag_control_rates_enabled = !offboard_control_mode.ignore_bodyrate ||
			!offboard_control_mode.ignore_attitude ||
			!offboard_control_mode.ignore_position ||
			!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_acceleration_force;

		control_mode.flag_control_attitude_enabled = !offboard_control_mode.ignore_attitude ||
			!offboard_control_mode.ignore_position ||
			!offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_acceleration_force;

		control_mode.flag_control_velocity_enabled = !offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position;

		control_mode.flag_control_climb_rate_enabled = !offboard_control_mode.ignore_velocity ||
			!offboard_control_mode.ignore_position;

		control_mode.flag_control_position_enabled = !offboard_control_mode.ignore_position;

		control_mode.flag_control_altitude_enabled = !offboard_control_mode.ignore_position;

		break;

	default:
		break;
	}
}

void
print_reject_mode(struct vehicle_status_s *status_local, const char *msg)
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

void answer_command(struct vehicle_command_s &cmd, enum VEHICLE_CMD_RESULT result)
{
	switch (result) {
	case VEHICLE_CMD_RESULT_ACCEPTED:
			tune_positive(true);
		break;

	case VEHICLE_CMD_RESULT_DENIED:
		mavlink_log_critical(mavlink_fd, "command denied: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_FAILED:
		mavlink_log_critical(mavlink_fd, "command failed: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
		/* this needs additional hints to the user - so let other messages pass and be spoken */
		mavlink_log_critical(mavlink_fd, "command temporarily rejected: %u", cmd.command);
		tune_negative(true);
		break;

	case VEHICLE_CMD_RESULT_UNSUPPORTED:
		mavlink_log_critical(mavlink_fd, "command unsupported: %u", cmd.command);
		tune_negative(true);
		break;

	default:
		break;
	}
}

void *commander_low_prio_loop(void *arg)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "commander_low_prio", getpid());

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* timeout for param autosave */
	hrt_abstime need_param_autosave_timeout = 0;

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		/* wait for up to 1000ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		/* timed out - periodic check for thread_should_exit, etc. */
		if (pret == 0) {
			/* trigger a param autosave if required */
			if (need_param_autosave) {
				if (need_param_autosave_timeout > 0 && hrt_elapsed_time(&need_param_autosave_timeout) > 200000ULL) {
					int ret = param_save_default();

					if (ret == OK) {
						mavlink_and_console_log_info(mavlink_fd, "settings autosaved");

					} else {
						mavlink_and_console_log_critical(mavlink_fd, "settings save error");
					}

					need_param_autosave = false;
					need_param_autosave_timeout = 0;
				} else {
					need_param_autosave_timeout = hrt_absolute_time();
				}
			}
		} else if (pret < 0) {
		/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("poll error %d, %d", pret, errno);
			continue;
		} else {

			/* if we reach here, we have a valid command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* ignore commands the high-prio loop handles */
			if (cmd.command == VEHICLE_CMD_DO_SET_MODE ||
			    cmd.command == VEHICLE_CMD_COMPONENT_ARM_DISARM ||
			    cmd.command == VEHICLE_CMD_NAV_TAKEOFF ||
			    cmd.command == VEHICLE_CMD_DO_SET_SERVO) {
				continue;
			}

			/* only handle low-priority commands here */
			switch (cmd.command) {

			case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				if (is_safe(&status, &safety, &armed)) {

					if (((int)(cmd.param1)) == 1) {
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						usleep(100000);
						/* reboot */
						systemreset(false);

					} else if (((int)(cmd.param1)) == 3) {
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						usleep(100000);
						/* reboot to bootloader */
						systemreset(true);

					} else {
						answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
					}

				} else {
					answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
				}

				break;

			case VEHICLE_CMD_PREFLIGHT_CALIBRATION: {

					int calib_ret = ERROR;

					/* try to go to INIT/PREFLIGHT arming state */
					if (TRANSITION_DENIED == arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_INIT, &armed,
							false /* fRunPreArmChecks */, mavlink_fd)) {
						answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);
						break;
					}

					if ((int)(cmd.param1) == 1) {
						/* gyro calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						calib_ret = do_gyro_calibration(mavlink_fd);

					} else if ((int)(cmd.param2) == 1) {
						/* magnetometer calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						calib_ret = do_mag_calibration(mavlink_fd);

					} else if ((int)(cmd.param3) == 1) {
						/* zero-altitude pressure calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_DENIED);

					} else if ((int)(cmd.param4) == 1) {
						/* RC calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						/* disable RC control input completely */
						status.rc_input_blocked = true;
						calib_ret = OK;
						mavlink_log_info(mavlink_fd, "CAL: Disabling RC IN");

					} else if ((int)(cmd.param4) == 2) {
						/* RC trim calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						calib_ret = do_trim_calibration(mavlink_fd);

					} else if ((int)(cmd.param5) == 1) {
						/* accelerometer calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						calib_ret = do_accel_calibration(mavlink_fd);

					} else if ((int)(cmd.param6) == 1) {
						/* airspeed calibration */
						answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
						calib_ret = do_airspeed_calibration(mavlink_fd);

					} else if ((int)(cmd.param4) == 0) {
						/* RC calibration ended - have we been in one worth confirming? */
						if (status.rc_input_blocked) {
							answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);
							/* enable RC control input */
							status.rc_input_blocked = false;
							mavlink_log_info(mavlink_fd, "CAL: Re-enabling RC IN");
                            calib_ret = OK;
						}
					}

					if (calib_ret == OK) {
						tune_positive(true);

						// Update preflight check status
						// we do not set the calibration return value based on it because the calibration
						// might have worked just fine, but the preflight check fails for a different reason,
						// so this would be prone to false negatives.

						status.condition_system_sensors_initialized = Commander::preflightCheck(mavlink_fd, true, true, true, true, false, true);

						arming_state_transition(&status, &safety, vehicle_status_s::ARMING_STATE_STANDBY, &armed, true /* fRunPreArmChecks */, mavlink_fd);

					} else {
						tune_negative(true);
					}

					break;
				}

			case VEHICLE_CMD_PREFLIGHT_STORAGE: {

					if (((int)(cmd.param1)) == 0) {
						int ret = param_load_default();

						if (ret == OK) {
							mavlink_log_info(mavlink_fd, "settings loaded");
							answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);

						} else {
							mavlink_log_critical(mavlink_fd, "settings load ERROR");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(mavlink_fd, "ERROR: %s", strerror(ret));
							}

							answer_command(cmd, VEHICLE_CMD_RESULT_FAILED);
						}

					} else if (((int)(cmd.param1)) == 1) {

						int ret = param_save_default();

						if (ret == OK) {
							if (need_param_autosave) {
								need_param_autosave = false;
								need_param_autosave_timeout = 0;
							}

							mavlink_log_info(mavlink_fd, "settings saved");
							answer_command(cmd, VEHICLE_CMD_RESULT_ACCEPTED);

						} else {
							mavlink_log_critical(mavlink_fd, "settings save error");

							/* convenience as many parts of NuttX use negative errno */
							if (ret < 0) {
								ret = -ret;
							}

							if (ret < 1000) {
								mavlink_log_critical(mavlink_fd, "ERROR: %s", strerror(ret));
							}

							answer_command(cmd, VEHICLE_CMD_RESULT_FAILED);
						}
					}

					break;
				}

			case VEHICLE_CMD_START_RX_PAIR:
				/* handled in the IO driver */
				break;

			default:
				/* don't answer on unsupported commands, it will be done in main loop */
				break;
			}

			/* send any requested ACKs */
			if (cmd.confirmation > 0 && cmd.command != VEHICLE_CMD_DO_SET_MODE
			    && cmd.command != VEHICLE_CMD_COMPONENT_ARM_DISARM) {
				/* send acknowledge command */
				// XXX TODO
			}
		}
	}

	close(cmd_sub);

	return NULL;
}
