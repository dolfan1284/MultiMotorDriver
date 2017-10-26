/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2013-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *     main.c
 *
 *  DESCRIPTION
 *     Example application to describe the usage of pulse width modulation. TWO
 *     PWM modules were used:
 *
 *        One to produce a slow flashing LED effect using the ramping
 *        feature of the PWM module, where in the duty cycle of the pulses
 *        alternate between two duty cycles ramping from one to another.
 *
 *        The other to produce the same effect, however with the help of a timer
 *        instead of using the ramping feature of the PWM module. This way the
 *        frequency of the pulses produced were kept constant during the entire
 *        period while the duty cycle alternated between minimum (0%) and
 *        maximum (100%) periodically.
 *
 ******************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
 
#include <main.h>           /* Functions relating to powering up the device */
#include <ls_app_if.h>      /* Link Supervisor application interface */
#include <debug.h>          /* Simple host interface to the UART driver */
#include <pio.h>            /* Programmable I/O configuration and control */
#include <timer.h>          /* Timers */

/*============================================================================*
 *  Local Header Files
 *============================================================================*/

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* PIO over which slow flashing LED signal is output */
#define PIO_LED                         (10U)   /* PIO10 */

/* PIO over which constant frequency variable duty cycle signal is output */
#define PIO_MOTOR                       (4U)    /* PIO4 */

/* The total number of steps when incrementing the duty cycle. This value
 * has range 1 to 255. It determines the frequency (or time period)
 * of the PWM waveform generated; lower the value higher the frequency */
#define NUMBER_OF_STEPS                 (255U)

/* The intervals after which the duty cycle is updated for the PIO_MOTOR
 * signal, in microseconds.
 *
 * Setting it to (NUMBER_OF_STEPS/32768) seconds would change the duty cycle
 * after one complete pulse cycle, since each cycle consists of as many steps
 * as NUMBER_OF_STEPS and each step takes (1/32768) seconds. */
#define DUTY_CYCLE_STEP_TIME            ((NUMBER_OF_STEPS * SECOND)/32768)

/* PIO direction configured as output */
#define PIO_DIR_OUTPUT                  (TRUE)

/* PIO direction configured as input */
#define PIO_DIR_INPUT                   (FALSE)

/* Maximum number of timers needed */
#define MAX_APP_TIMERS                  (1U)

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Memory for the firmware to store and maintain timers */
static uint16 app_timers[ SIZEOF_APP_TIMER * MAX_APP_TIMERS ];

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 * NAME
 *      setPwmDutyCycle
 *
 * DESCRIPTION
 *      Configures PWM1 to generate a waveform of a specified duty cycle.
 *
 * PARAMETERS
 *      value            A value in the range (0 to NUMBER_OF_STEPS). Setting
 *                        it to 0 would set the duty cycle to 0%, while setting
 *                        it to NUMBER_OF_STEPS would result in 100% duty cycle.
 *
 * RETURNS/MODIFIES
 *      None
 *----------------------------------------------------------------------------*/
static void setPwmDutyCycle( uint8 value )
{
    const uint8 total_period = NUMBER_OF_STEPS; /* Total period including on
                                        and off times, in number of ticks */

    PioConfigPWM(
                1,                      /* Use the PWM1 */
                pio_pwm_mode_push_pull, /* PWM1 is configured with push-pull
                                            output current driver circuitry */

                /* Pulse timings for the first part of the sequence */

                /* ON time for the pulses during the first part of the
                 * sequence is (value * 30)us */
                value,

                /* OFF time for the pulses during the first part of the
                 * sequence is (total_period - value) * 30us */
                (total_period - value),

                /* Duration for which the first part of the sequence lasts for
                 * in units of 16ms. Since a fixed frequency waveform is
                 * produced, both parts of the sequence contain identical
                 * pulses. As ramping will not enabled for this PWM, the
                 * output switches from the first part of the sequence to
                 * the second part instantaneously. Since they are identical,
                 * the switching doesn't have any effect and hence the duration
                 * for which the first or second part of the sequence last
                 * doesn't really matter. We simply choose a non-zero value. */
                1U,

                /* Pulse timings for the second part of the sequence.
                 * This is chosen to be identical to the first part of the
                 * sequence as the output is supposed to be fixed frequency. */

                /* ON time for the pulses during the second part of the
                 * sequence is also (value * 30)us */
                value,

                /* OFF time for the pulses during the second part of the
                 * sequence is also (total_period - value) * 30us */
                (total_period - value),

                /* Duration for which the second part of the sequence lasts for
                 * chosen to be 1 * 16ms */
                1U,

                /* Ramping is disabled as the output is supposed to be fixed
                 * frequency */
                0U
                );
}

/*----------------------------------------------------------------------------*
 * NAME
 *      dutyCycleTask
 *
 * DESCRIPTION
 *      This is the function called when the timer expires.
 *
 *      Creates a timer for updating the duty cycle on the PWM1. The ramping
 *      effect is produced on the PWM1 by updating its duty cycle periodically
 *      while maintaining constant frequency.
 *
 *      The PWM1 is configured for its duty cycle from this timer call-back. The
 *      duty cycle is updated from 0% to 100% and then down to 0% and so on.
 *
 * PARAMETERS
 *      tid             Id of the timer that has just expired. It is not used.
 *
 * RETURNS/MODIFIES
 *      None
 *----------------------------------------------------------------------------*/
static void dutyCycleTask( timer_id const tid )
{
    static uint8 on_ticks = 0U;
    static int8 increment_value = 1U; /* Hold the value by which ON time is
                                            incremented. When duty cycle is
                                            increasing it holds possitive value.
                                            When duty cycle is decreasing it
                                            holds negative value */

    /* Configure the PWM1 to produce a fixed frequency signal, with the same
     * duty cycle for both the dullest and brightest parts. */
    setPwmDutyCycle(on_ticks);

    if ( on_ticks >= NUMBER_OF_STEPS )
        /* When ON time reaches its maximum (i.e. 100% duty cycle)
         * it should be decremented until it reaches minimum (i.e. 0%) */
    {
        increment_value = -1;
    }
    else if ( on_ticks <= 0U )
        /* When ON time reaches its minimum (i.e. 0% duty cycle)
         * it should be incremented until it reaches maximum (i.e. 100%) */
    {
        increment_value = 1;
    }

    /* Update ON time for the next update */
    on_ticks += increment_value;

    /* Re-start the timer */
    TimerCreate(DUTY_CYCLE_STEP_TIME, TRUE, dutyCycleTask);
}

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This user application function is called just after a power-on reset
 *      (including after a firmware panic), or after a wakeup from Hibernate or
 *      Dormant sleep states.
 *
 *      At the time this function is called, the last sleep state is not yet
 *      known.
 *
 *      NOTE: this function should only contain code to be executed after a
 *      power-on reset or panic. Code that should also be executed after an
 *      HCI_RESET should instead be placed in the AppInit() function.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppPowerOnReset(void)
{
    /* Code that is only executed after a power-on reset or firmware panic
    * should be implemented here - e.g. configuring application constants
    */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This user application function is called after a power-on reset
 *      (including after a firmware panic), after a wakeup from Hibernate or
 *      Dormant sleep states, or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 * PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state last_sleep_state)
{
    /* Initialise UART communications */
    DebugInit(1, NULL, NULL);

    DebugWriteString("Configuring PWM Modes\r\n");

    /* Configure the output PIO on which the slow flashing LED signal is
     * generated */
    PioSetDir(PIO_LED, PIO_DIR_OUTPUT);

    /* SLOW FLASHING WITH PWM:
     * Configure PWM 0 to have the following characteristics
     * DULL LED Light is generated by having pulses of 0ms ON and
     *      6ms OFF (~0% duty cycle)
     * BRIGHT LED Light is generated by having pulses of 6ms ON and
     *      0ms OFF (~100% duty cycle)
     * Dullest and Brightest level are held for ~1s
     *
     * The brightness level ramps for ~1s when going from dullest to
     * brightest and vice-versa */
    if (PioConfigPWM(0, pio_pwm_mode_push_pull,
            /* Pulse timings for the dullest part of the sequence:
             * dullest part of the sequence has the pulse off for the whole
             * period, in effect the line stays low for the duration for which
             * the dullest part of the sequence lasts. */
            0,          /* ON time for the pulse is 0us */
            255,        /* OFF time for the pulse is (255 * 30)us */
            62,         /* Dullest part of the sequence lasts for
                            ( 62 * 16 )ms before ramping up to the brightest
                            part of the sequence */

            /* Pulse timings for the brightest part of the sequence:
             * brightest part of the sequence has the pulse ON for the whole
             * period, in effect the line stays high for the duration for which
             * the brightest part of the sequence lasts. */
            255,        /* ON time for the pulse is (255 * 30)us */
            0,          /* OFF time for the pulse is 0us */
            62,         /* Brightest part of the sequence lasts for
                            ( 62 * 16 )ms before ramping down to the dullest
                            part of the sequence */

            /* Ramping between dullest and brightest parts of the sequence
             * This parameter determines the duration for which the ramping
             * lasts when going from dullest to the brightest (and vice-versa).
             *
             * The total duration for which the ramping lasts is determined by
             * multiplying this value with one less than the difference between
             * the on_time or off_time of the two states, whichever is bigger;
             * in the units of 30us
             * */
            132         /* Ramping lasts for ((255-1) * 132 * 30)us */
            ))
    {
        DebugWriteString("PWM0 was set to ramp between brightest and dullest "
                "levels\r\n");

        /* Connect PWM0 output to LED */
        PioSetMode(PIO_LED, pio_mode_pwm0);

        /* Enable the PWM0 */
        PioEnablePWM(0, TRUE);
    }
    else
    {
        DebugWriteString("PWM0 couldn't be configured\r\n");
    }

    /* Set both outputs to have strong internal pull ups */
    PioSetPullModes((1UL << PIO_MOTOR) | (1UL << PIO_LED),
            pio_mode_strong_pull_up);

    /* Configure motor PIO to be output */
    PioSetDir(PIO_MOTOR, PIO_DIR_OUTPUT);

    /* Connect PWM1 output to motor PIO */
    PioSetMode(PIO_MOTOR, pio_mode_pwm1);

    /* Initialise timers */
    TimerInit(MAX_APP_TIMERS, app_timers);

    /* Call the function (with a dummy timer ID to start with) to configure
     * the PWM1 with initial duty cycle and start a timer. When the timer
     * expires the same function gets called again and the duty cycle for
     * PWM1 gets updated. It takes care of restarting the timer and ensuring
     * that duty cycle alternates between 0% and 100% back and forth. */
    dutyCycleTask(TIMER_INVALID);

    /* Enable the PWM1 */
    PioEnablePWM(1, TRUE);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcesSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 * PARAMETERS
 *      id   [in]   System event ID
 *      data [in]   Event data
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{
    /* This application does not handle any system events and hence this
     * function is left blank */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event
 *      is received by the system.
 *
 * PARAMETERS
 *      event_code [in]   LM event ID
 *      event_data [in]   LM event data
 *
 * RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
 *----------------------------------------------------------------------------*/
bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *event_data)
{
    return TRUE;
}
