/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.AnalogGyro;

/**
 *
 * @author SCHROED
 */
public class EnhancedGyro extends AnalogGyro {
    
    public EnhancedGyro(int GyroAI) {
        super(GyroAI);        
    }
    
    public double GetHeading() {
        double a =  getAngle()%360;
        if (a < 0) {
            a += 360;
        }
        return a;
    }
}
