/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team177.lib;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Keep track of robots location in x,y grid with 0,0 being starting location
 */
public class Locator implements Logable {
    
    private final EnhancedGyro headingGyro;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    public final UpdateLocation updateLocation;
    
    public Locator(int gyroAI, int leftA, int leftB, int rightA, int rightB) {
        headingGyro = new EnhancedGyro(gyroAI);
        leftEncoder = new Encoder(leftA, leftB);
        rightEncoder = new Encoder(rightA, rightB);
                
        //Set Default Values
        //TODO - need to figure this out.
        leftEncoder.setDistancePerPulse(18.84/64);
        rightEncoder.setDistancePerPulse(18.84/64);
    
        updateLocation = new UpdateLocation();       
            
        LiveWindow.addSensor("Locater", "left Encoder", leftEncoder);
        LiveWindow.addSensor("Locater", "right Encoder", rightEncoder);
        LiveWindow.addSensor("Locater", "Gyro", headingGyro);
    }
    
    public void start() {
        updateLocation.start();
    }
    
    
    public double GetHeading() {
        return updateLocation.heading;
    }
    
    public double GetX() {
        return updateLocation.x;
    }
    
    public double GetY() {
        return updateLocation.y;
    }
    
    /* Set x/y location to 0,0 and heading to 0 degrees */
    public void Reset() {
        updateLocation.Reset();   
    }
    
    public double getLeftRaw() {
        return leftEncoder.getDistance();
    }
    
    public double getRightRaw() {
        return rightEncoder.getDistance();
    }
    
    public void setDistancePerPulse(float leftDPP, float rightDPP) {
        leftEncoder.setDistancePerPulse(leftDPP);
        rightEncoder.setDistancePerPulse(rightDPP);        
    }
    
    private class UpdateLocation extends Thread {

        public double x;
        public double y;
        public double heading;
        private boolean ResetFlag = false;
        
        UpdateLocation() {
            x = 0;
            y = 0;    
            heading = 0;
            headingGyro.reset();
        }
        
        public void Reset() {
            ResetFlag = true;
        }
        
        public void run() {       
            double deltax, deltay;
            double distance;
            double lastLeft = 0;
            double lastRight = 0;
            double left, right;
    		long startTime; 
    		
            
            while (true) {
            	startTime = System.nanoTime();
            	
            	if (ResetFlag) {
                	x = 0;
                    y = 0;
                    headingGyro.reset();
                    leftEncoder.reset();
                    rightEncoder.reset();
                    ResetFlag = false;
                }
                left = leftEncoder.getDistance();
                right = rightEncoder.getDistance();
                
                /* Average the two encoder values */
                /* TODO - possibly add error checking to detect a failed encoder and ignore it */
                distance = ((left - lastLeft) + (right - lastRight)) / 2.0;
                //distance = (left - lastLeft); 
                		
                heading = headingGyro.GetHeading();

                /* Do fancy trig stuff */
                deltax = distance * Math.cos(Math.toRadians(heading));
                deltay = distance * Math.sin(Math.toRadians(heading));
                             
                /* Update Location */
                x += deltax;
                y -= deltay;

                /* Update history variables */
                lastLeft = left;
                lastRight = right;
                
                SmartDashboard.putNumber("x", x);
                SmartDashboard.putNumber("y", y);
                SmartDashboard.putNumber("Heading", heading);
                SmartDashboard.putNumber("left", left);
                SmartDashboard.putNumber("right", right);

               	try {
                	Thread.sleep((System.nanoTime() - startTime)/1000000  + 10); //Update the position at 10ms (100Hz) 
                } catch (InterruptedException e) {
                }
            }
        }    
    }

	public double getLeftEncoderDistance() {
		return leftEncoder.getDistance();
	}
	
	public double getRightEncoderDistance() {
		return rightEncoder.getDistance();
	}

	
	public double getGyroAngleInRadians() {
		return updateLocation.heading * Math.PI/180.0;
	}

	public double GetVel() {
		return leftEncoder.getRate(); // distance/second
	}

	public double GetHeadingRadians() {
		return getGyroAngleInRadians();
	}

	@Override
	public String GetColumNames() {
		return "x, y, heading";
	}

	@Override
	public String log() {
		return String.format("%.2f,%.2f,%.2f", updateLocation.x, updateLocation.y, updateLocation.heading);
	}
}
