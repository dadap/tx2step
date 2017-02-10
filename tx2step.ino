#define DEBUG 0

#if (64 / clockCyclesPerMicrosecond()) * clockCyclesPerMicrosecond() != 64 || \
    F_CPU % 1000000L != 0
#error clockCyclesPerMicrosecond() does not divide evenly into 64: tracking \
rate may be inaccurate. Change this error into a warning if you wish to \
continue anyway.
#endif

/* Microseconds per sidereal hour, used to calculate tracking rates.
 * The resulting calculations should lead to a tracking error on the order
 * of a second per sidereal day, which is in line with the error inherent to
 * reasonably priced clock crystals */
const unsigned long sidereal_hour_us = 3590170483;

/* Axis/motor attributes */
typedef struct {
    /* permanent configuration attributes */
    const char * const short_name; /* short name of axis */
    const unsigned long steps_per_15_degrees; /* steps per 15°/sidereal hour */
    const int step_pin; /* pin to drive stepper pulses */
    const int dir_pin; /* pin to set motor direction */
    const int input_analog_pin; /* pin to read analog input from joystick */
    const int enable_pin; /* pin to enable/disable driver */

    /* dynamic runtime variables */
    unsigned long last_step; /* µsec timestamp of most recent step */
    unsigned long next_step; /* µsec timestamp of next scheduled step */
    int current_rate; /* tracking/setting rate in units of 1/4 sidereal rate */
    unsigned long us_per_step; /* µs per step at current rate (cached value) */
} axis;

typedef enum {
    RIGHT_ASCENSION = 0,
    DECLINATION = 1,
    NUM_AXES
} axis_index;

typedef enum {
    NORMAL,
    IMMEDIATE,
} urgency;

/* Configuration values for RA and DEC axes.
 * steps_per_15_degrees values are for Synta EQ-3 with dual-axis kit */
static axis axes[] = {
    [RIGHT_ASCENSION] = {
        .short_name = "RA",
        .steps_per_15_degrees = 2 * 120 * 130,
        .step_pin = 13, .dir_pin = 12,
        .input_analog_pin = A0, .enable_pin = -1, /* RA is always on */
    },
    [DECLINATION] = {
        .short_name = "DEC",
         /* XXX this is actually wrong because it doesn't account for the
          * clutch, but that's sort of okay, because it moves really fast */
        .steps_per_15_degrees = 2 * 80 * 65,
        .step_pin = 11, .dir_pin = 10,
        .input_analog_pin = A1, .enable_pin = 9,
    },
};

/* Determine whether a step is due (current time is the same as or after next
 * due step). last_step is used as a reference point to handle overflow. */
static inline bool step_due(axis_index i, unsigned long now) {
    return now - axes[i].last_step >= axes[i].next_step - axes[i].last_step;
}

/* Perform a step on axis i. */
static void do_step(axis_index i, urgency when) {
    /* DRV8834 requires a 1.9 µs minimum pulse duration;
     * delayMicroseconds() is not precise < 3 µs according to docs */
    const int PULSE_DURATION_US = 4;

    /* Ignore next_step and avoid delay if the rate is 0 */
    if (axes[i].current_rate != 0) {
        unsigned long now = micros();

        /* Always step if when is IMMEDIATE, ignoring when next step is due */
        if (when == IMMEDIATE || step_due(i, now)) {
            digitalWrite(axes[i].step_pin, HIGH);
            delayMicroseconds(PULSE_DURATION_US);
            digitalWrite(axes[i].step_pin, LOW);
            delayMicroseconds(PULSE_DURATION_US);

            axes[i].last_step = now;

            /* Reset next_step based on current time for IMMEDIATE steps;
             * increment next_step by us_per_step for NORMAL steps to allow
             * for catching up to a missed step */
            if (when == IMMEDIATE) {
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
    /* rate is given in units of 1/4 sidereal rate and may be negative;
     * number of microseconds needs to be a scalar value. */
    return ((sidereal_hour_us / axes[i].steps_per_15_degrees) * 4) / abs(rate);
}

/* index of last array element */
#define array_len(a) (sizeof(a) / sizeof(a[0]))

static void set_rate(axis_index i, urgency when) {
    /* Table of tracking/setting rates, in units of 1/4 sidereal rate, e.g.,
     * a rate value of "12" represents 3x sidereal rate. */
    const int rates[] = {-128, -64, -32, -12, -4, -3, -2, -1,
                         0, 1, 2, 3, 4, 12, 32, 64, 128};
    int new_rate = rates[constrain(
                             map(analogRead(axes[i].input_analog_pin),
                                 0, 1023,
                                 0, array_len(rates)),
                             0, array_len(rates) - 1
                         )];

    /* RA always tracks at 1x sidereal relative to nominal rate */
    if (i == RIGHT_ASCENSION) {
        new_rate += 4;
    }

    if (when == IMMEDIATE || axes[i].current_rate != new_rate) {
        axes[i].current_rate = new_rate;

        /* Set direction (FIXME read direction toggle switch */
        if (new_rate < 0) {
            digitalWrite(axes[i].dir_pin, LOW);
        } else {
            digitalWrite(axes[i].dir_pin, HIGH);
        }

        /* Enable/disable motor and cache us_per_step value */
        if (new_rate != 0) {
            if (axes[i].enable_pin >= 0) {
                digitalWrite(axes[i].enable_pin, HIGH);
            }
            axes[i].us_per_step = us_per_step(i, new_rate);
        } else if (axes[i].enable_pin >= 0) {
            digitalWrite(axes[i].enable_pin, LOW);
        }

        /* Step immediately and override next scheduled step. */
        do_step(i, IMMEDIATE);
    }
}

/* Atmega328 ADC requires a minimum of 100µs between analog reads; we don't
 * need quite that level of resolution, but we should still try to rate limit
 * the reads somewhat */
const int ANALOG_READ_DELAY_MS = 20;
const int MIN_ANALOG_READ_DELAY_US = 100;

/* Read joystick position and adjust tracking/setting rates accordingly.
 * Don't perform an analog read unless sufficient time has elapsed since the
 * last read, or an immediate read was requested. Alternate between reading
 * axes. The actual reading is performed in set_rate(); this function serves
 * to enforce timing and interleaving. */
static void read_joystick(urgency when)
{
    static axis_index next_read = RIGHT_ASCENSION;
    static unsigned long last_read_ms;
    unsigned long now = millis();

    /* Wait minimum delay before an immediate read */
    if (when == IMMEDIATE) {
        delayMicroseconds(MIN_ANALOG_READ_DELAY_US);
    }

    if (when == IMMEDIATE || now - last_read_ms >= ANALOG_READ_DELAY_MS) {
        last_read_ms = now;
        set_rate(next_read, when);
        next_read = (next_read + 1) % NUM_AXES;
    }
}

static inline void print_axis_info(axis_index i)
{
    static int pot_min = 1023, pot_max = 0;
    int pot_val;

    delayMicroseconds(MIN_ANALOG_READ_DELAY_US);
    pot_val = analogRead(axes[i].input_analog_pin);

    if (pot_val > pot_max) {
        pot_max = pot_val;
    }

    if (pot_val < pot_min) {
        pot_min = pot_val;
    }

    Serial.print(axes[i].short_name);
    Serial.print(" pot values - Current: ");
    Serial.print(pot_val);
    Serial.print(" Min: ");
    Serial.print(pot_min);
    Serial.print(" Max: ");
    Serial.println(pot_max);
    Serial.print("Current rate: ");
    Serial.println(axes[i].current_rate);
    Serial.print("µs per step: ");
    Serial.println(axes[i].us_per_step);
    Serial.println();
}

const int PRINT_INTERVAL_MS = 1000;

static inline void print_status() {
    static unsigned long last_print = 0;

    if (last_print == 0 || millis() - last_print > PRINT_INTERVAL_MS) {
        for (int i = 0; i < NUM_AXES; i++) {
            print_axis_info(i);
        }
        last_print = millis();
    }
}

void setup() {
#if DEBUG
    Serial.begin(9600);
#endif
    for (int i = 0; i < NUM_AXES; i++) {
        pinMode(axes[i].input_analog_pin, INPUT);
        pinMode(axes[i].step_pin, OUTPUT);
        pinMode(axes[i].dir_pin, OUTPUT);
        if (axes[i].enable_pin >= 0) {
            pinMode(axes[i].enable_pin, OUTPUT);
        }

        /* read_joystick() doesn't accept an axis index, but as long as it runs
         * once per axis with IMMEDIATE urgency, it will initialize all axes */
        read_joystick(IMMEDIATE);
    }
}

void loop() {
    for (int i = 0; i < NUM_AXES; i++) {
        read_joystick(NORMAL);
        do_step(i, NORMAL);
#if DEBUG
        print_status();
#endif
    }
}
