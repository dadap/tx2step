/*
 * Texas Two Step:
 *     A dual axis stepper motor controller for equatorial telescope mounts
 *
 * Copyright (c) 2017 Daniel Dadap
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define DEBUG 0

#if (64 / clockCyclesPerMicrosecond()) * clockCyclesPerMicrosecond() != 64 || \
    F_CPU % 1000000L != 0
#error clockCyclesPerMicrosecond() does not divide evenly into 64: tracking \
rate may be inaccurate. Change this error into a warning if you wish to \
continue anyway.
#endif

typedef enum {
    RIGHT_ASCENSION = 0,
    DECLINATION = 1,
    NUM_AXES
} axis_index;

typedef enum {
    JOYSTICK_RA = RIGHT_ASCENSION,
    JOYSTICK_DEC = DECLINATION,
    TRACKING_RATE,
    GUIDING_RATE,
    CHARGER_STATUS,
    NUM_ANALOG_SENSORS
} analog_index;

typedef enum {
    NORMAL,
    IMMEDIATE,
    INITIAL,
} urgency;

typedef enum {
    TRISTATE_LOW = 0,
    TRISTATE_HI_Z = 1,
    TRISTATE_HIGH = 2,
} tristate_value;

typedef enum {
    LUNAR = TRISTATE_LOW,
    SOLAR = TRISTATE_HI_Z,
    SIDEREAL = TRISTATE_HIGH,
    NUM_TRACKING_RATES
} tracking_rate;

typedef enum {
    NEGATIVE,
    POSITIVE,
} direction;

/* Axis/motor attributes */
typedef struct {
    /* permanent configuration attributes */
    const unsigned long steps_per_15_degrees; /* steps per 15° (1 hour) */
    const int step_pin; /* pin to drive stepper pulses */
    const int dir_pin; /* pin to set motor direction */
    const int enable_pin; /* pin to enable/disable driver */

    /* dynamic runtime variables */
    unsigned long last_step; /* µsec timestamp of most recent step */
    unsigned long next_step; /* µsec timestamp of next scheduled step */
    int current_rate; /* tracking/setting rate in units of 1/4 tracking rate */
    unsigned long us_per_step; /* µs per step at current rate (cached value) */
} axis;

/* Analog sensor attributes */
typedef struct {
    /* permanent configuration attributes */
    const int pin; /* pin where value will be read */
    const int range_max; /* maximum value to map to */
    const bool affects_tracking; /* Whether sensor influences tracking rate */

    /* dynamic runtime variables */
    int mapped_value; /* storage for most recently read and mapped value */
} analog_sensor;

/* ST-4 configuration */
typedef struct {
    /* permanent configuration attributes */
    const int pin; /* pin where value will be read */
    const axis_index axis; /* axis driven by pin */
    const direction dir; /* direction to drive axis */

    /* dynamic runtime variables */
    unsigned long last_changed; /* milliscond timestamp of last value change */
    bool undebounced_value; /* pending state while debouncing */
    bool value; /* most recent debounced value */
} st4_input;

/* Microseconds per 15 degrees, used to calculate tracking rates.
 * The resulting calculations should lead to a tracking error on the order
 * of a second per day, which is in line with the error inherent to
 * reasonably priced clock crystals */
const unsigned long us_per_15_degrees[] = {
    [LUNAR] = 3726180003,
    [SOLAR] = 3600000000,
    [SIDEREAL] = 3590170483,
};

/* Configuration values for RA and DEC axes.
 * steps_per_15_degrees values are for Synta EQ-3 with dual-axis kit */
static axis axes[] = {
    [RIGHT_ASCENSION] = {
        .steps_per_15_degrees = 2 * 120 * 130,
        .step_pin = 2, .dir_pin = 3, .enable_pin = 4,
    },
    [DECLINATION] = {
         /* XXX this is actually wrong because it doesn't account for the
          * clutch, but that's sort of okay, because it moves really fast */
        .steps_per_15_degrees = 2 * 80 * 65,
        .step_pin = 5, .dir_pin = 6, .enable_pin = 7,
    },
};

/* ST-4 pin mappings */
static st4_input st4_inputs[] = {
    { .pin = 10, .axis = RIGHT_ASCENSION, .dir = POSITIVE, },
    { .pin = 11, .axis = DECLINATION, .dir = POSITIVE, },
    { .pin = 12, .axis = DECLINATION, .dir = NEGATIVE, },
    { .pin = 13, .axis = RIGHT_ASCENSION, .dir = NEGATIVE, },
};

/* Table of setting rates, in units of 1/4 tracking rate, e.g.,
 * a rate value of "12" represents 3x tracking rate. */
const int setting_rates[] = {-128, -64, -32, -12, -4, -3, -2, -1,
                             0, 0, 1, 2, 3, 4, 12, 32, 64, 128};

/* Table of guiding rates, in same units as setting rates. Duplicated values
 * for rates < 1x tracking are to ensure that 1x tracking is at the midpoint. */
const int guiding_rates[] = {1, 1, 2, 2, 3, 3, 4, 6, 8, 12, 32, 64, 128};

#define array_len(a) (sizeof(a) / sizeof(a[0]))

/* Configuration values for analog inputs */
static analog_sensor sensors[] = {
    [JOYSTICK_RA] = {
        .pin = A0, .range_max = array_len(setting_rates),
        .affects_tracking = true,
    },
    [JOYSTICK_DEC] = {
        .pin = A1, .range_max = array_len(setting_rates),
        .affects_tracking = true,
    },
    [TRACKING_RATE] = {
        .pin = A2, .range_max = NUM_TRACKING_RATES,
        .affects_tracking = true,
    },
    [GUIDING_RATE] = {
        .pin = A3, .range_max = array_len(guiding_rates),
        .affects_tracking = false,
    },
};

const int led_pwm_pin = 9;

/* Determine whether a step is due (current time is the same as or after next
 * due step). last_step is used as a reference point to handle overflow. */
static inline bool step_due(axis_index i, unsigned long now)
{
    return now - axes[i].last_step >= axes[i].next_step - axes[i].last_step;
}

/* Perform a step on axis i if one is needed. */
static void do_step(axis_index i, urgency when)
{
    /* DRV8834 requires a 1.9 µs minimum pulse duration;
     * delayMicroseconds() is not precise < 3 µs according to docs */
    const int PULSE_DURATION_US = 4;

    /* Ignore next_step and avoid delay if the rate is 0 */
    if (axes[i].current_rate != 0) {
        unsigned long now = micros();

        /* Always step if when is IMMEDIATE, ignoring when next step is due */
        if (when == IMMEDIATE || when == INITIAL || step_due(i, now)) {
            /* Don't actually step if we're just initializing, but do reset
             * the last_step timestamp. */
            if (when != INITIAL) {
                digitalWrite(axes[i].step_pin, HIGH);
                delayMicroseconds(PULSE_DURATION_US);
                digitalWrite(axes[i].step_pin, LOW);
                delayMicroseconds(PULSE_DURATION_US);
            }

            axes[i].last_step = now;

            /* Reset next_step based on current time for IMMEDIATE steps or
             * when initializing; increment next_step by us_per_step for NORMAL
             * steps to allow for catching up to any missed steps. Don't bother
             * trying to catch up to missed steps when step rate > 50Hz, the
             * precision of step timing isn't critical at these speeds */
            if (when == IMMEDIATE || when == INITIAL ||
                axes[i].us_per_step < 20000) {
                axes[i].next_step = now + axes[i].us_per_step;
            } else {
                axes[i].next_step += axes[i].us_per_step;
            }
        }
    }
}

/* Calculate microseconds per step at given rate */
static inline unsigned long us_per_step(axis_index i, int rate)
{
    /* rate is given in units of 1/4 tracking rate and may be negative;
     * number of microseconds needs to be a scalar value. */
    return ((us_per_15_degrees[sensors[TRACKING_RATE].mapped_value] /
             axes[i].steps_per_15_degrees) * 4) / abs(rate);
}

/* Set the tracking, setting, or guiding rate on the given axis. Rates are
 * expressed in units of 1/4 the tracking rate. */
static void set_rate(axis_index i, int new_rate, urgency when)
{
    /* RA always tracks at +1x relative to nominal rate */
    if (i == RIGHT_ASCENSION) {
        new_rate += 4;
    }

    if (axes[i].current_rate != new_rate) {
        axes[i].current_rate = new_rate;

        /* Set direction (FIXME read direction toggle switch */
        if (new_rate < 0) {
            digitalWrite(axes[i].dir_pin, LOW);
        } else {
            digitalWrite(axes[i].dir_pin, HIGH);
        }

        /* Enable/disable motor and cache us_per_step value */
        if (new_rate != 0) {
            digitalWrite(axes[i].enable_pin, HIGH);
            delay(1); /* DRV8834 needs 1ms delay on wake before step */
            axes[i].us_per_step = us_per_step(i, new_rate);
        } else {
                digitalWrite(axes[i].enable_pin, LOW);
        }

        /* Step immediately and override next scheduled step,
         * unless we are just initializing a sensor. */
        do_step(i, when == INITIAL ? INITIAL : IMMEDIATE);
    }
}

/* Set rates based on the updated value of an analog sensor */
static void analog_set_rates(analog_index sensor, urgency when)
{
    int new_value = map(analogRead(sensors[sensor].pin),
                        0, 1024, 0, sensors[sensor].range_max);

    if (when == IMMEDIATE || when == INITIAL ||
        new_value != sensors[sensor].mapped_value) {
        sensors[sensor].mapped_value = new_value;

        if (sensors[sensor].affects_tracking) {
            for (int i = 0; i < NUM_AXES; i++) {
                set_rate(i, setting_rates[sensors[i].mapped_value], when);
            }
        }

#if DEBUG
        Serial.print("Sensor: ");
        Serial.print(sensor);
        Serial.print(" New value: ");
        Serial.print(new_value);
        Serial.print(" RA us/step: ");
        Serial.print(axes[RIGHT_ASCENSION].current_rate ?
                         axes[RIGHT_ASCENSION].us_per_step : 0);
        Serial.print(" DEC us/step: ");
        Serial.println(axes[DECLINATION].current_rate ?
                         axes[DECLINATION].us_per_step : 0);
#endif
    }
}

/* Calculate the effective rate driven by an ST-4 input, scaled by the value of
 * the guiding rate potentiometer. */
static int scale_st4_rate(int st4_index)
{
    int rate = guiding_rates[sensors[GUIDING_RATE].mapped_value];

    if (st4_inputs[st4_index].dir == NEGATIVE) {
        rate *= -1;
    }

    return rate;
}

/* Set guiding rates based on ST-4 input. Simultaneous inputs in opposing
 * directions on the same axis will cancel each other out. */
static void st4_set_rates(void)
{
    int net_rates[NUM_AXES];

    memset(net_rates, 0, sizeof(int) * NUM_AXES);

    for (int i = 0; i < array_len(st4_inputs); i++) {
        if (st4_inputs[i].value == true) {
            net_rates[st4_inputs[i].axis] += scale_st4_rate(i);
        }
    }

    for (int i = 0; i < NUM_AXES; i++) {
        set_rate(i, net_rates[i], NORMAL);
    }
}

/* Atmega328 ADC requires a minimum of 100µs between analog reads; we don't
 * need quite that level of resolution, but we should still try to rate limit
 * the reads somewhat */
const int ANALOG_READ_DELAY_MS = 10;
const int MIN_ANALOG_READ_DELAY_US = 100;

/* Read sensor values and adjust tracking/setting rates accordingly.
 * Don't perform an analog read unless sufficient time has elapsed since the
 * last read, or an immediate read was requested. Interleave reads between the
 * sensors. The actual reading is performed in set_rates(); this function serves
 * to enforce timing and interleaving. */
static void read_analog_sensor(urgency when)
{
    static analog_index next_read = JOYSTICK_RA;
    static unsigned long last_read_ms;
    unsigned long now = millis();

    if (when == INITIAL) {
        pinMode(sensors[next_read].pin, INPUT);
    }

    /* Wait minimum delay before an immediate read or when initializing */
    if (when == IMMEDIATE || when == INITIAL) {
        delayMicroseconds(MIN_ANALOG_READ_DELAY_US);
    }

    if (when == IMMEDIATE || now - last_read_ms >= ANALOG_READ_DELAY_MS) {
        last_read_ms = now;
        analog_set_rates(next_read, when);
        next_read = (next_read + 1) % NUM_ANALOG_SENSORS;
    }
}

/* Debounce ST-4 input. This may not be necessary for actual ST-4 guiders, but
 * digital hand control interfaces over ST-4, and may need switch debouncing. */
static void debounce_input(void)
{
    const int debounce_delay_ms = 20;

    for (int i = 0; i < array_len(st4_inputs); i++) {
        unsigned long now = millis();
        /* ST-4 input pins are pulled up high and short to ground when active */
        bool current_value = (digitalRead(st4_inputs[i].pin) == LOW);

        if (current_value != st4_inputs[i].undebounced_value) {
            st4_inputs[i].last_changed = now;
            st4_inputs[i].undebounced_value = current_value;
            continue;
        }

        if (now - st4_inputs[i].last_changed >= debounce_delay_ms &&
            st4_inputs[i].value != st4_inputs[i].undebounced_value) {
            st4_inputs[i].value = st4_inputs[i].undebounced_value;
#if DEBUG
            Serial.print("ST4 pin ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(st4_inputs[i].value);
#endif
        }
    }
}

/* Returns true if the joystick is in a neutral position. */
static bool joystick_neutral(void)
{
    bool ret = true;

    for (int i = 0; i < NUM_AXES; i++) {
        ret &= (setting_rates[sensors[i].mapped_value] == 0);
    }

    return ret;
}

/* Set the status LED based on activity:
 *   - Solid LED when tracking
 *   - Blinking LED when setting
 */
static void update_led(urgency when)
{
    const int cycle_duration = 2048;
    unsigned long now = millis() % cycle_duration;
    static int led_value;
    int new_led_value;

    if (joystick_neutral()) {
            new_led_value = 160;
    } else {
        if (now < cycle_duration / 2) {
            new_led_value = 255;
        } else {
                new_led_value = 0;
        }
    }

    if (when == INITIAL || new_led_value != led_value) {
        led_value = new_led_value;
        analogWrite(led_pwm_pin, led_value);
    }
}

void setup()
{
#if DEBUG
    Serial.begin(9600);
#endif
    for (int i = 0; i < NUM_AXES; i++) {
        pinMode(axes[i].step_pin, OUTPUT);
        pinMode(axes[i].dir_pin, OUTPUT);
        pinMode(axes[i].enable_pin, OUTPUT);
    }

    for (int i = 0; i < NUM_ANALOG_SENSORS; i++) {
        read_analog_sensor(INITIAL);
    }

    for (int i = 0; i < array_len(st4_inputs); i++) {
        pinMode(st4_inputs[i].pin, INPUT_PULLUP);
    }

    pinMode(led_pwm_pin, OUTPUT);
    update_led(INITIAL);
}

void loop()
{
    read_analog_sensor(NORMAL);
    debounce_input();

    /* Joystick input overrides ST-4 input. */
    if (joystick_neutral()) {
        st4_set_rates();
    }

    for (int i = 0; i < NUM_AXES; i++) {
        do_step(i, NORMAL);
    }

    update_led(NORMAL);
}
