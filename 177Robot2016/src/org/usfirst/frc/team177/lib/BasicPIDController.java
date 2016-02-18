/**
 * Class implements a PID Control Loop.
 * 
 */
 
package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.util.BoundaryException;

/**
 *
 * @author schrod
 */
public class BasicPIDController {
    
    private double m_P;			// factor for "proportional" control
    private double m_I;			// factor for "integral" control
    private double m_D;			// factor for "derivative" control
    private double m_maximumOutput = 1.0;	// |maximum output|
    private double m_minimumOutput = -1.0;	// |minimum output|  
    private double m_prevError = 0.0;	// the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0; //the sum of the errors for use in the integral calc
    
    public BasicPIDController(double Kp, double Ki, double Kd) {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }
    
    /**     
     * 
     * calculate the output accordingly, and write to the output.     
     * 
     * @param error from current cycle
     * @param dT time since last run
     * @return Control value
     */
    public double calculate(double error, double dT)
    {               
        double result;

        //Prevent Windup
        if (((m_totalError + error*dT) * m_I < m_maximumOutput)
                && ((m_totalError + error*dT) * m_I > m_minimumOutput)) {
            //Integrate error
            m_totalError += error*dT;
        }

        result = (m_P * error + m_I * m_totalError + m_D * (error - m_prevError)*dT);
        m_prevError = error;

        if (result > m_maximumOutput) {
            result = m_maximumOutput;
        } else if (result < m_minimumOutput) {
            result = m_minimumOutput;
        }
        return result;
    }        
    
    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public synchronized void setPID(double p, double i, double d) {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP() {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI() {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public double getD() {
        return m_D;
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output
     * @param maximumOutput the maximum value to write to the output
     */
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        if (minimumOutput > maximumOutput) {
            throw new BoundaryException("Lower bound is greater than upper bound");
        }
        m_minimumOutput = minimumOutput;
        m_maximumOutput = maximumOutput;
    }

 
     /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    public synchronized void reset() {
        m_prevError = 0;
        m_totalError = 0;        
    }
    
}
