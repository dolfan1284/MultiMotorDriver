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
 *      which_pwm       The PWM to set the duty cyle for.
 * RETURNS/MODIFIES
 *      None
 *----------------------------------------------------------------------------*/
extern void setPwmDutyCycle( uint8 value, uint16 which_pwm );
/*----------------------------------------------------------------------------*
 * NAME
 *      dutyCycleTask
 *
 * DESCRIPTION
 *      This is the function called when the timer expires.
 *
 *      Creates a timer for updating the duty cycle on the PWMx. The ramping
 *      effect is produced on the PWMx by updating its duty cycle periodically
 *      while maintaining constant frequency.
 *
 *      The PWMx is configured for its duty cycle from this timer call-back. The
 *      duty cycle is updated from 0% to 100% and then down to 0% and so on.
 *
 * PARAMETERS
 *      tid             Id of the timer that has just expired. It is not used.
 *
 *      which_pwm       The PWMx to use for updatin the duty cycle of upon
 *                      expiration
 * RETURNS/MODIFIES
 *      None
 *----------------------------------------------------------------------------*/
extern void dutyCycleTask( timer_id const tid, uint16 which_pwm );
