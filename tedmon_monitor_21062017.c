/* 
 * File:   tedmon_monitor_21062017.c
 * Author: EX4
 *
 * Created on June 21, 2017, 2:13 PM
 */

/*
 standard lib
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <sys/stat.h>

//inifile
#include "ini.h"

/*
 pigpio
 */
#include <pigpio.h>

/*
 id device
 */
//#define DEVICE_ID   "TEDMON01"
#define DEV_VERSION "14072017"

/*
 define pin
 */
#define OUT_SONAR_TRIGGER       16
#define IN_SONAR_ECHO1          20
#define IN_WATERFLOW_SENSOR     21
#define OUT_RELAY_POMPA         12
#define OUT_RELAY_BUZZER        26

/*
 tanki konstant dlm cm
 */
//#define TANKI_HEIGHT_MAX    148.0
//#define TANKI_JARI_JARI     65.0

/*
 batas level air dlm cm
 */
//#define WATER_UPPER_LEVEL   TANKI_HEIGHT_MAX
//#define WATER_LOWER_LEVEL   (0.2 * TANKI_HEIGHT_MAX)
#define WATER_FLOW_CALIBRATION_PULSE    7.5

//stuct utk konfigurasi

typedef struct {
    char isvalid;
    const char *dev_id, *telp1, *telp2, *server_url;
    float tank_hmax, tank_radius;
    float water_upper_level_limit, water_lower_level_limit;
} cfg_struct_t;

#define CONFIG_FILENAME "tedmon.conf"


//data struct utk hasil kalkulasi sonar

typedef struct {
    float sonar_pulse_length, height_measured, height_residu, volume;
} sonar_calc_result_t;

typedef enum {
    ERR_NONE = 0,
    ERR_PUMP_NO_CLOSE = 1,
    ERR_PUMP_NO_OPEN = 2,
    ERR_UNKNOWN = 255
} sys_error_t;

//server data
//#define SERVER_URL "http://monitoringair.xyz/api.php"

//telp admin
//#define TELP_NUM_ADMIN  "081277160225"

/*
 proto
 */
void sonarTrigger(void);
void sonarEcho(int gpio, int level, uint32_t tick);
void calc_tanki_volume(int pulse_us, sonar_calc_result_t *result);
int gpio_init();
static void system_signal_handler(int sig_num);
void flowMeterPulse(int gpio, int level, uint32_t tick);
void flowMeterTimeout();
int send_sms(const char *msgx, const char *telp_num);

/**
 * Log data macro using printf style
 */
#define log_data(...)({\
    printf("\r\n%f\t: ", time_time());\
    printf(__VA_ARGS__);\
})

//global var
//server
static sig_atomic_t server_system_signal_received = 0;

//ultrasonik
uint32_t sonar_startTick = 0; //, sonar_firstTick = 0;
int sonar_diffTick = 0;
sonar_calc_result_t sonar_result = {0};

//flwo sensor
uint32_t flow_sensor_pulse = 0, flow_sensor_start_tick = 0, flow_sensor_end_tick = 0;
float flow_meter_debit = 0.0;

//error code
sys_error_t device_error_code = ERR_NONE;
uint32_t cnt_1second = 0;
char pump_state = PI_OFF;
uint32_t cnt_t30_sec = 0;
char is_sending_data = 0;
char buzzerMustOn = 0;
char is_water_full = 0;
char is_test_mode = 0;

//config var
cfg_struct_t config_dev = {
    .isvalid = 0,
    .dev_id = "TEDMON01",
    .telp1 = "081277160225",
    .telp2 = "081277160225",
    .server_url = "http://monitoringair.xyz/api.php",
    .tank_hmax = 148.0,
    .tank_radius = 65,
    .water_upper_level_limit = 148.0,
    .water_lower_level_limit = 29.6
};

/**
 * check if file exist
 * @param filename
 * @return 
 */
int file_exist(char *filename) {
    struct stat buffer;
    return (stat(filename, &buffer) == 0);
}

/**
 * run command and get result char
 * @param cmd
 * @param message
 * @return 
 */
char run_command(const char *cmd, char **message) {
    FILE *fp;
    int status;
    char msg_res[1024];

    fp = popen(cmd, "r");
    if (!fp) {
        return 0;
    }

    //allocate message pointer
    *message = (char*) calloc(65536, sizeof (char));

    if (!message)
        return 0;

    while ((fgets(msg_res, 1024, fp) != NULL) && (strlen(*message) < 65000))
        strcat(*message, msg_res);

    status = pclose(fp);
    return (status == -1) ? 0 : 1;
}

/**
 * send data to server
 * @param id
 * @param hx
 * @param vx
 * @param dx
 * @return 
 */
int curl_post_data(const char *server, const char *id, float hx, float vx, float dx) {
    char cmdx[250];
    snprintf(cmdx, sizeof (cmdx), "curl -s --connect-timeout 10 -X POST -F 'c=1' -F 'id=%s' -F 'h=%f' -F 'v=%f' -F 'd=%f' %s 2>&1 | grep -i '\"error\":0'", id, hx, vx, dx, server);
    log_data("SEND DATA TO SERVER : %s\n", cmdx);
    return system(cmdx);
}

/**
 * send sms
 * @param msgx
 * @return 
 */
int send_sms(const char *msgx, const char *telp_num) {
    char cmdx[250];
    snprintf(cmdx, sizeof (cmdx), "echo \"%s\" | gammu sendsms TEXT %s", msgx, telp_num);
    log_data("SEND SMS : %s\n", cmdx);
    return system(cmdx);
}

/**
 *  trigger sonar per xms
 */
void sonarTrigger(void) {
    //calc silinder result
    calc_tanki_volume(sonar_diffTick, &sonar_result);

    //    log_data("SONAR = %8.3f\t%8.3f\t%8.3f\t%12.3f", sonar_result.sonar_pulse_length, sonar_result.height_measured, sonar_result.height_residu, sonar_result.volume);

    /* trigger a sonar reading */
    gpioWrite(OUT_SONAR_TRIGGER, PI_ON);
    gpioDelay(10); /* 10us trigger pulse */
    gpioWrite(OUT_SONAR_TRIGGER, PI_OFF);
}

/**
 * get the echo
 * @param gpio  gpio number
 * @param level high or low
 * @param tick  milliseconds timer value
 */
void sonarEcho(int gpio, int level, uint32_t tick) {
    //calc first tick
    //    if (!sonar_firstTick) sonar_firstTick = tick;

    //get diff tick
    if (level == PI_ON) {
        sonar_startTick = tick;
    } else if (level == PI_OFF) {
        sonar_diffTick = tick - sonar_startTick;
    }
}

/**
 * measure flow meter pulse
 * @param gpio  gpio number
 * @param level high or low
 * @param tick  milliseconds timer value
 */
void flowMeterPulse(int gpio, int level, uint32_t tick) {
    flow_sensor_pulse++;
}

/**
 * flow meter processing
 */
void flowMeterTimeout(void) {
    //calculate flow rate
    flow_sensor_end_tick = gpioTick();
    if (flow_sensor_start_tick == 0) {
        flow_sensor_start_tick = flow_sensor_end_tick;
    }
    if ((flow_sensor_end_tick - flow_sensor_start_tick) > 0) {
        //        log_data("%d", flow_sensor_end_tick - flow_sensor_start_tick);
        flow_meter_debit = ((1000000.0 / ((float) flow_sensor_end_tick - (float) flow_sensor_start_tick))*(float) flow_sensor_pulse) / WATER_FLOW_CALIBRATION_PULSE;
    } else {
        flow_meter_debit = 0.0;
    }
    flow_sensor_pulse = 0;
    flow_sensor_start_tick = gpioTick();

    log_data("SONAR = %8.0fuS\t%8.3fcm\t%8.3fcm\t%12.3fL", sonar_result.sonar_pulse_length, sonar_result.height_measured, sonar_result.height_residu, sonar_result.volume);
    log_data("FLOWMETER = %.3f L/minute", flow_meter_debit);

    //    check range
    // pompa must on
    if (sonar_result.height_residu < config_dev.water_lower_level_limit) {
//        gpioWrite(OUT_RELAY_POMPA, PI_LOW);
        pump_state = PI_ON;
    }

    // pompa must off
    if (sonar_result.height_residu >= config_dev.water_upper_level_limit) {
//        gpioWrite(OUT_RELAY_POMPA, PI_HIGH);
        pump_state = PI_OFF;
        gpioWrite(OUT_RELAY_BUZZER, PI_LOW);
    }
    log_data("PUMP STATE = %d", pump_state);
    gpioWrite(OUT_RELAY_POMPA, !pump_state);

    //check error per 10sec
    cnt_1second++;
    if (cnt_1second % 10 == 0) {

        if (pump_state == PI_ON) {
            if (flow_meter_debit > 0) {
                device_error_code = ERR_NONE;
            } else {
                device_error_code = ERR_PUMP_NO_OPEN;
            }
        } else {
            if (flow_meter_debit > 0) {
                device_error_code = ERR_PUMP_NO_CLOSE;
            } else {
                device_error_code = ERR_NONE;
            }
        }
        cnt_1second = 0;

        log_data("ERROR STATE = %u", device_error_code);

        //        stop buzzer
        if (device_error_code == ERR_NONE)
            gpioWrite(OUT_RELAY_BUZZER, PI_HIGH);
    }
}

/**
 * timer routine job per n x 30sec
 */
void timer_messaging_job(void) {
    cnt_t30_sec++;

    if (is_sending_data == 0) {
        is_sending_data = 1;

        //         buzzer error
        if (device_error_code != ERR_NONE) {
            gpioWrite(OUT_RELAY_BUZZER, PI_LOW);
        } else {
            gpioWrite(OUT_RELAY_BUZZER, PI_HIGH);
        }

        //    job check error msg, per 1menit
        if (cnt_t30_sec % 2 == 0) {
            if (device_error_code != ERR_NONE) {
                //                shutdown the modem
                log_data("SHUTTING DOWN MODEM...");
                if (system("systemctl stop tedmon_modem_autoconnect") == 0) {
                    log_data("MODEM SHUTDOWN SUCCESSFULL");
                    time_sleep(1.0);
                    log_data("SEND ERROR SMS to %s", config_dev.telp1);
                    if (!is_test_mode) {
                        log_data("RESULT = %d", send_sms(
                                (device_error_code == ERR_PUMP_NO_CLOSE) ?
                                "SISTEM ERROR. POMPA MATI. FLOWMETER MASIH ADA ALIRAN." :
                                (device_error_code == ERR_PUMP_NO_OPEN) ?
                                "SISTEM ERROR. POMPA HIDUP. FLOWMETER TIDAK ADA ALIRAN." :
                                "SISTEM ERROR. KERUSAKAN TIDAK DIKETAHUI!!!",
                                config_dev.telp1
                                ));
                    }

                    log_data("SEND ERROR SMS to %s", config_dev.telp2);
                    if (!is_test_mode) {
                        log_data("RESULT = %d", send_sms(
                                (device_error_code == ERR_PUMP_NO_CLOSE) ?
                                "SISTEM ERROR. POMPA MATI. FLOWMETER MASIH ADA ALIRAN." :
                                (device_error_code == ERR_PUMP_NO_OPEN) ?
                                "SISTEM ERROR. POMPA HIDUP. FLOWMETER TIDAK ADA ALIRAN." :
                                "SISTEM ERROR. KERUSAKAN TIDAK DIKETAHUI!!!",
                                config_dev.telp2
                                ));
                    }
                } else {
                    log_data("MODEM SHUTDOWN FAIL!!!");
                }
            }
        }

        //send k server per 5menit
        if (cnt_t30_sec % 10 == 0) {
            log_data("CHECKING MODEM STATUS...");
            if (system("sakis3g --sudo \"connect\" \"status\" | grep -i \"connected to\"") == 0) {
                log_data("MODEM CONNECTED");
                log_data("SEND DATA TO SERVER");
                if (!is_test_mode) {
                    log_data("RESULT %d", curl_post_data(config_dev.server_url, config_dev.dev_id, sonar_result.height_residu, sonar_result.volume, flow_meter_debit));
                }
            } else {
                log_data("MODEM DISCONNECTED!!!");
            }
        }

        is_sending_data = 0;
    }
}

/**
 * configure the gpio
 * @return 0: ok, 1: error 
 */
int gpio_init() {
    int resx = 0;

    //    disable all other interface
    resx = (gpioCfgInterfaces(PI_DISABLE_FIFO_IF | PI_DISABLE_SOCK_IF | PI_LOCALHOST_SOCK_IF) == 0 ? 0 : 1);

    //    try to init
    // check the detail at pigpio documentation link on the c interface
    if (resx == 0) {
        if (gpioInitialise() >= 0) {
            resx = ((gpioSetMode(OUT_SONAR_TRIGGER, PI_OUTPUT) == 0) &&
                    (gpioSetMode(IN_SONAR_ECHO1, PI_INPUT) == 0) &&
                    (gpioSetPullUpDown(IN_SONAR_ECHO1, PI_PUD_UP) == 0) &&
                    (gpioSetMode(IN_WATERFLOW_SENSOR, PI_INPUT) == 0) &&
                    (gpioSetPullUpDown(IN_WATERFLOW_SENSOR, PI_PUD_UP) == 0) &&
                    (gpioSetMode(OUT_RELAY_BUZZER, PI_OUTPUT) == 0) &&
                    (gpioSetMode(OUT_RELAY_POMPA, PI_OUTPUT) == 0)) ? 0 : 1;

            if (resx == 0) {
                /* update sonar 20 times a second, timer #0 */
                resx = ((gpioSetTimerFunc(0, 50, sonarTrigger) == 0) && /* every 50ms */

                        /* monitor sonar echos */
                        (gpioSetAlertFunc(IN_SONAR_ECHO1, sonarEcho) == 0) &&

                        /* monitor flow sensor */
                        (gpioSetAlertFunc(IN_WATERFLOW_SENSOR, flowMeterPulse) == 0) &&

                        (gpioSetTimerFunc(1, 1000, flowMeterTimeout) == 0) &&
                        (gpioSetTimerFunc(2, 30000, timer_messaging_job) == 0)) ? 0 : 1;
            }
        } else {
            resx = 1;
        }
    }
    return resx;
}

/**
 * calculate tank volume
 * @param pulse_us  pulse ultrasonic echo in us
 * @param result    volume in L
 */
void calc_tanki_volume(int pulse_us, sonar_calc_result_t *result) {
    float y = 0;

    result->sonar_pulse_length = (float) pulse_us;
    result->height_measured = result->sonar_pulse_length / 58.2; //cm
    result->height_residu = (float) config_dev.tank_hmax - result->height_measured;
    if (result->height_residu < 0) result->height_residu = 0;
    y = M_PI * config_dev.tank_radius * config_dev.tank_radius * result->height_residu; //cm^3
    y /= 1000.0; //dm^3=L
    result->volume = (y < 0 ? 0 : y);
}

/**
 * Signal handler for mongoose
 * @param sig_num   signal level catched from the system
 */
static void system_signal_handler(int sig_num) {
    signal(sig_num, system_signal_handler); // Reinstantiate signal handler
    server_system_signal_received = sig_num;
}

/**
 * config file handler
 * @param user      configuration structure
 * @param section   ini section name
 * @param name      ini name
 * @param value     ini value
 * @return          0=ERROR, 1=OK
 */
static int config_ini_handler(void* user, const char* section, const char* name,
        const char* value) {
    cfg_struct_t* pconfig = (cfg_struct_t*) user;

#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (MATCH("device", "id")) {
        pconfig->dev_id = strdup(value);
    } else if (MATCH("server", "url")) {
        pconfig->server_url = strdup(value);
    } else if (MATCH("telp", "telp1")) {
        pconfig->telp1 = strdup(value);
    } else if (MATCH("telp", "telp2")) {
        pconfig->telp2 = strdup(value);
    } else if (MATCH("tanki", "hmax")) {
        pconfig->tank_hmax = atof(value);
    } else if (MATCH("tanki", "radius")) {
        pconfig->tank_radius = atof(value);
    } else if (MATCH("water_level", "upper")) {
        pconfig->water_upper_level_limit = atof(value);
    } else if (MATCH("water_level", "lower")) {
        pconfig->water_lower_level_limit = atof(value);
    } else {
        return 0; /* unknown section/name, error */
    }
    return 1;
}

/**
 * load config
 */
void load_configuration_file() {
    char filex[100];

    config_dev.isvalid = 0;
    //try to load from current dir
    snprintf(filex, sizeof (filex), "%s", CONFIG_FILENAME);
    if (ini_parse(filex, config_ini_handler, &config_dev) < 0) {
        log_data("LOAD CONFIGURATION FROM %s FAIL", filex);
        //try to load from /etc
        snprintf(filex, sizeof (filex), "/etc/%s", CONFIG_FILENAME);
        if (ini_parse(filex, config_ini_handler, &config_dev) < 0) {
            log_data("LOAD CONFIGURATION FROM %s FAIL", filex);
        } else {
            log_data("LOAD CONFIGURATION FROM %s OK", filex);
            config_dev.isvalid = 1;
        }
    } else {
        log_data("LOAD CONFIGURATION FROM %s OK", filex);
        config_dev.isvalid = 1;
    }

    if (config_dev.isvalid == 0) {
        log_data("LOAD CONFIGURATION FILE FAIL. REVERT TO DEFAULT SETTING");
    } else {
        log_data("LOAD CONFIGURATION FILE OK. UPDATE DEFAULT SETTING");
    }

    log_data("CONFIGURATION :\n"
            "ID\t: %s\n"
            "SERVER\t: %s\n"
            "TELP1\t: %s\n"
            "TELP2\t: %s\n"
            "TANK HMAX\t: %.1f\n"
            "TANK RADIUS\t: %.1f\n"
            "WATER UPPER\t: %.1f\n"
            "WATER LOWER\t: %.1f"
            ,
            config_dev.dev_id,
            config_dev.server_url,
            config_dev.telp1,
            config_dev.telp2,
            config_dev.tank_hmax,
            config_dev.tank_radius,
            config_dev.water_upper_level_limit,
            config_dev.water_lower_level_limit
            );

}

/*
 * main program
 */
int main(int argc, char** argv) {
    //    welcome message
    log_data("TEDMON MONITOR " DEV_VERSION " -- Program started...");

    // test mode?
    if (argc >= 2) {
        is_test_mode = atoi(argv[1]) != 0;
    } else {
        is_test_mode = 0;
    }
    log_data("TEST MODE = %d", is_test_mode);


    //    load config file
    load_configuration_file();

    /*
     init pigpio first
     */
    if (gpio_init() == 0) {
        log_data("INIT GPIO OK");
    } else {
        log_data("INIT GPIO ERROR!!!");
    }
    sleep(5);

    /*
     signal handler
     */
    signal(SIGINT, system_signal_handler);
    signal(SIGTERM, system_signal_handler);
    //    setvbuf(stdout, NULL, _IONBF, 0);
    //    setvbuf(stderr, NULL, _IONBF, 0);

    //sorry, im gonna sleep for now, see youu, Zzzzzzzzzzzzzzzzzzz
    log_data("I'M GOING TO SLEEP NOW. BYE......");
    while (server_system_signal_received == 0) {
        sleep(1);
    }

    //    cleanup
    gpioWrite(OUT_RELAY_POMPA, PI_HIGH);
    gpioWrite(OUT_RELAY_BUZZER, PI_HIGH);
    gpioTerminate();

    log_data("Terminating application done\r\n");

    return (EXIT_SUCCESS);
}

