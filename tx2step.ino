/* Microseconds per sidereal hour, used to calculate tracking rates.
 * The resulting calculations should lead to a tracking error on the order
 * of a second per sidereal day, which is in line with the error inherent to
 * reasonably priced clock crystals */
const unsigned long sidereal_hour_us = 3590170483;

/* Axis/motor attributes */
typedef struct {
    unsigned long last_step; /* µsec timestamp of most recent step */
    unsigned long next_step; /* µsec timestamp of next scheduled step */
    unsigned long steps_per_15_degrees; /* # steps per 15° (one sidereal hour) */
    int current_rate; /* tracking/setting rate in units of 1/4 sidereal rate */
    unsigned long us_per_step; /* µs per step at current rate (cached value) */
    int step_pin; /* pin to drive stepper pulses */
    int dir_pin; /* pin to set motor direction */
    int microstep_pin; /* pin to toggle microstepping rate (not implemented) */
    int input_analog_pin; /* pin to read analog input from joystick */
    int enable_pin; /* pin to enable/disable driver */
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
axis axes[] = {
    [RIGHT_ASCENSION] = {0, 0, 2 * 120 * 130, 4, 0, 13, 12, 11, A0, 6},
    [DECLINATION] = {0, 0, 2 * 80 * 65, 0, 0, 10, 9, 8, A1, 5},
};

/* Determine whether a step is due (current time is the same as or after next
 * due step). last_step is used as a reference point to handle overflow. */
bool step_due(axis_index i, unsigned long now) {
    return now - axes[i].last_step >= axes[i].next_step - axes[i].last_step;
}

/* Perform a step on axis i. */
void do_step(axis_index i, urgency when) {
    /* Ignore next_step and avoid delay if the rate is 0 */
    if (axes[i].current_rate != 0) {
        unsigned long now = micros();

        /* Always step if when is IMMEDIATE, ignoring when next step is due */
        if (when == IMMEDIATE || step_due(i, now)) {
            /* DRV8834 requires 1.9 µs minimum pulse duration;
             * delayMicroseconds() is not precise < 3 µs according to docs */
            digitalWrite(axes[i].step_pin, HIGH);
            delayMicroseconds(4);
            digitalWrite(axes[i].step_pin, LOW);
            delayMicroseconds(4);

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
unsigned long us_per_step(axis_index i, int rate)
{
    /* rate is given in units of 1/4 sidereal rate and may be negative;
     * number of microseconds needs to be a scalar value. */
    return ((sidereal_hour_us / axes[i].steps_per_15_degrees) * 4) / abs(rate);
}

/* index of last array element */
#define array_max(a) (sizeof(a) / sizeof(a[0]) - 1)

void set_rate(axis_index i, urgency when) {
    /* Table of tracking/setting rates, in units of 1/4 sidereal rate, e.g.,
     * a rate value of "12" represents 3x sidereal rate. */
    const int rates[] = {-64, -32, -12, -4, -3, -2, -1,
                         0, 1, 2, 3, 4, 12, 32, 64};
    /* FIXME map() call needs to be calibrated with sensor range */
    int new_rate = rates[map(analogRead(axes[i].input_analog_pin), 0, 1023,
                         0, array_max(rates))];

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
            digitalWrite(axes[i].enable_pin, HIGH);
            axes[i].us_per_step = us_per_step(i, new_rate);
        } else {
            digitalWrite(axes[i].enable_pin, LOW);
        }

        /* Step immediately and override next scheduled step. */
        do_step(i, IMMEDIATE);
    }
}

/* Atmega328 DAC requires a minimum of 100ms between analog reads */
const int MIN_ANALOG_READ_DELAY_MS = 100;

/* Read joystick position and adjust tracking/setting rates accordingly.
 * Don't perform an analog read unless the minimum delay has elapsed since
 * the last read, or an immediate read was requested. Alternate between
 * reading axes. The actual reading is performed in set_rate(); this function
 * serves to enforce timing and interleaving. */
void read_joystick(urgency when)
{
    static axis_index next_read = RIGHT_ASCENSION;
    static unsigned long last_read_ms;
    unsigned long now = millis();

    /* Wait minimum delay before an immediate read */
    if (when == IMMEDIATE) {
        delay(MIN_ANALOG_READ_DELAY_MS);
    }

    if (when == IMMEDIATE || now - last_read_ms >= MIN_ANALOG_READ_DELAY_MS) {
        last_read_ms = now;
        set_rate(next_read, when);
        next_read = (next_read + 1) % NUM_AXES;
    }
}

void setup() {
    for (int i = 0; i < NUM_AXES; i++) {
        pinMode(axes[i].input_analog_pin, INPUT);
        pinMode(axes[i].step_pin, OUTPUT);
        pinMode(axes[i].dir_pin, OUTPUT);
        pinMode(axes[i].microstep_pin, OUTPUT);
        pinMode(axes[i].enable_pin, OUTPUT);

        /* read_joystick() doesn't accept an axis index, but as long as it runs
         * once per axis with IMMEDIATE urgency, it will initialize all axes */
        read_joystick(IMMEDIATE);
    }
}

void loop() {
    for (int i = 0; i < NUM_AXES; i++) {
        read_joystick(NORMAL);
        do_step(i, NORMAL);
    }
}
