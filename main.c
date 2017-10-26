/******************************************************************************
 *  FILE
 *      main.c
 *
 *  DESCRIPTION
 *      This file implements a minimal CSR uEnergy application.
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
 *  Private Function Prototypes
 *============================================================================*/

/* UART Receive callback */
static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_additional_req_data_length);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      uartRxDataCallback
 *
 *  DESCRIPTION
 *      This is an internal callback function (of type uart_data_in_fn) that
 *      will be called by the UART driver when any data is received over UART.
 *      See DebugInit in the Firmware Library documentation for details.
 *
 *  PARAMETERS
 *      p_rx_buffer [in]   Pointer to the receive buffer (uint8 if 'unpacked'
 *                         or uint16 if 'packed' depending on the chosen UART
 *                         data mode - this application uses 'unpacked')
 *
 *      length [in]        Number of bytes ('unpacked') or words ('packed')
 *                         received
 *
 *      p_additional_req_data_length [out]
 *                         Number of additional bytes ('unpacked') or words
 *                         ('packed') this application wishes to receive
 *
 *  RETURNS
 *      The number of bytes ('unpacked') or words ('packed') that have been
 *      processed out of the available data.
 *----------------------------------------------------------------------------*/
static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_additional_req_data_length)
{
    /* Inform the UART driver that we'd like to receive another byte when it
     * becomes available
     */
    *p_additional_req_data_length = 1;
    
    /* Return the number of bytes that have been processed */
    return length;
}

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/*Use PIO0, 3, 9, 11 for the four motors in the design!! Only J100 used JDP */
/* PIO0 is at location 4 on J100, (motor0)
   PIO3 is at location 7, (motor1)
   PIO4 is at location 8, (motor2)
   PIO9 is at location 13, (motor3)
   PIO11 is at location 15*/
#define PIO_MOTOR0  (0U)
#define PIO_MOTOR1  (3U)
/*#define PIO_MOTOR4  (8U)*/
#define PIO_MOTOR2  (4U)
#define PIO_MOTOR3  (9U)

#define PIO_MOTOR0_PWM  (0U)
#define PIO_MOTOR1_PWM  (1U)
#define PIO_MOTOR2_PWM  (2U)
#define PIO_MOTOR3_PWM  (3U)
/*#define PIO_MOTOR4_PWM  (4U) */

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

static void setPwmDutyCycle( uint8 value )
{
    const uint8 total_period = NUMBER_OF_STEPS; /* Total period including on
                                        and off times, in number of ticks */

    PioConfigPWM(
                1/*which_pwm*/,              /* Use the PWMx,
                                              hardcode to 1 for now JDP */
                pio_pwm_mode_push_pull, /* PWx1 is configured with push-pull
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

static void dutyCycleTask( timer_id const tid )
{
    static uint8 on_ticks = 0U;
    static int8 increment_value = 1U; /* Hold the value by which ON time is
                                            incremented. When duty cycle is
                                            increasing it holds positive value.
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
 *  PARAMETERS
 *      None
 *
 *  RETURNS
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
 *  PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state last_sleep_state)
{
    /* Initialise communications */
    DebugInit(1, uartRxDataCallback, NULL);

    DebugWriteString("Hello, world\r\n");
    
    /*Look for the current time and check if we are at the alarm time,
      if not, continue to poll/wait for interrupt until we are 
      or the alarm is set to a new time. 
      Once we hit the alarm time, start PWMing the motors to wake the person
      Do PWM for 5 minutes then stop*/
    /*read time (uint time recvd)*/
    while (1)
    {
            DebugWriteString("Configuring PWM Modes\r\n");

     /*configureSlowFlash();*/
            
    /* Configure the output PIO on which the slow flashing LED signal is
     * generated */
    PioSetDir(/*PIO_LED*/PIO_MOTOR0, PIO_DIR_OUTPUT);

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
    if (PioConfigPWM(PIO_MOTOR0_PWM, pio_pwm_mode_push_pull,
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

        /* Connect PWM0 output to LED/first motor */
        PioSetMode(/*PIO_LED*/PIO_MOTOR0, pio_mode_pwm0);

        /* Enable the PWM0 */
        PioEnablePWM(PIO_MOTOR0_PWM, TRUE);
    }
    else
    {
        DebugWriteString("PWM0 couldn't be configured\r\n");
    }

    /* Set both outputs to have strong internal pull ups */
    PioSetPullModes((1UL << /*PIO_MOTOR*/PIO_MOTOR1) | (1UL << /*PIO_LED*/PIO_MOTOR0)
                   |(1UL << PIO_MOTOR2) | (1UL << PIO_MOTOR3)
                   /*|(1UL << PIO_MOTOR4)*/,
                    pio_mode_strong_pull_up);

    /* Configure motor PIO to be output */
    PioSetDir(/*PIO_MOTOR*/PIO_MOTOR1, PIO_DIR_OUTPUT);

    /* Connect PWM1 output to motor PIO */
    PioSetMode(/*PIO_MOTOR*/PIO_MOTOR1, pio_mode_pwm1);

    /* Configure motor PIO to be output */
    PioSetDir(/*PIO_MOTOR*/PIO_MOTOR2, PIO_DIR_OUTPUT);

    /* Connect PWM1 output to motor PIO */
    PioSetMode(/*PIO_MOTOR*/PIO_MOTOR2, pio_mode_pwm2);

    /* Configure motor PIO to be output */
    PioSetDir(/*PIO_MOTOR*/PIO_MOTOR3, PIO_DIR_OUTPUT);

    /* Connect PWM1 output to motor PIO */
    PioSetMode(/*PIO_MOTOR*/PIO_MOTOR3, pio_mode_pwm3);

    /* Configure motor PIO to be output*/
/*    PioSetDir(PIO_MOTOR4, PIO_DIR_OUTPUT);*/

    /* Connect PWM1 output to motor PIO */
    /*PioSetMode(PIO_MOTOR4, pio_mode_pwm4);*/

    /* Initialise timers */
    TimerInit(MAX_APP_TIMERS, app_timers);

    /* Call the function (with a dummy timer ID to start with) to configure
     * the PWM1 with initial duty cycle and start a timer. When the timer
     * expires the same function gets called again and the duty cycle for
     * PWM1 gets updated. It takes care of restarting the timer and ensuring
     * that duty cycle alternates between 0% and 100% back and forth. */
    dutyCycleTask(TIMER_INVALID/*, PIO_MOTOR1*/);

    /* Enable the PWM1 */
    PioEnablePWM(PIO_MOTOR1_PWM, TRUE);
    PioEnablePWM(PIO_MOTOR2_PWM, TRUE);
    PioEnablePWM(PIO_MOTOR3_PWM, TRUE);
/*    PioEnablePWM(PIO_MOTOR4_PWM, TRUE); */
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcesSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  PARAMETERS
 *      id   [in]               System event ID
 *      data [in]               Event data
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{
    /* This application does not process any system events */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event
 *      is received by the system.
 *
 *  PARAMETERS
 *      event_code [in]         LM event ID
 *      p_event_data [in]       LM event data
 *
 *  RETURNS
 *      Always returns TRUE. See the Application module in the Firmware Library
 *      documentation for more information.
 *----------------------------------------------------------------------------*/
bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *event_data)
{
    /* This application does not process any LM-specific events */

    return TRUE;
}
