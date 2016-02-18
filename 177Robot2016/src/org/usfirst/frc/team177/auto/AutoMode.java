package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.Robot;
import org.usfirst.frc.team177.lib.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract class to provide the framework for handling multiple autonomous modes
 * 
 */

public abstract class AutoMode implements Logable {

	Robot robot; //reference to main implementation
	
    BasicPIDController DrivePID;
    BasicPIDController SteerPID;
    
    double lastRanDriveTo = 0;
    
    /* Variables & Constants used for DriveTo PID Controls */ 
    private static double SteerMargin = 5.0; //Margin to consider robot facing target (degrees)
    private static double DriveMargin = 1.0; //Margin to consider the robot at target (in)
    
    private static double DriveP = 0.035;  //Preportial gain for Drive System
    private static double DriveI = 0.002;   //Integral gain for Drive System
    private static double DriveD = 0.0;   //Derivative gain for Drive System
    private static double DriveMax = 1;   //Max Saturation value for control
    private static double DriveMin = -1;  //Min Saturation value for control
    
    private static double SteerP = 0.06; //0.02;  //Preportial gain for Steering System
    private static double SteerI = 0.0001; //0.01 //Integral gain for Steering System
    private static double SteerD = 0.00;  //Derivative gain for Steering System
    private static double SteerMax = 1;   //Max Saturation value for control
    private static double SteerMin = -1;  //Min Saturation value for control
    
	double startX;
	double startY;
	
	double lastTargetX = 0;
    double lastTargetY = 0;
	
    double lastRunDriveTo;
    
	//Constructor
	public AutoMode(Robot robot) {
        this.robot = robot;
        
        DrivePID = new BasicPIDController(DriveP,DriveI,DriveD);
        DrivePID.setOutputRange(DriveMin, DriveMax);
        SteerPID = new BasicPIDController(SteerP,SteerI,SteerD);
        SteerPID.setOutputRange(SteerMin, SteerMax);    
	}
	
	public abstract void autoPeriodic();
    public abstract String getName();
        
    public void autoInit() {
    	lastTargetX = lastTargetY = -9999999;
    }
    
    
    /**
     * 
     * Drive the robot to the specified location
     * 
     *      -----------    +
     *      | Robot   |
     *      |   --->  |    Y
     *      |         |   
     *      -----------    -
     *       -   X    +
     *  Robot starts match at 0,0 heading 0
     * 
     * @param x - x coordinate of target
     * @param y - y coordinate of target
     * @param speed - Speed to drive, a negative value will cause the robot to backup.
     *                A Speed of 0 will cause the robot to turn to the target without moving
     * @return - Boolean value indicating if the robot is at the target or not (true = at target).
     * @author schroed
     */     
    public boolean DriveTo(double x, double y, double speed) 
    {
    	double steer, drive;
        //Reinitalize if the target has changed
        if(x != lastTargetX || y != lastTargetY) {
            lastTargetX = x;
            lastTargetY = y;
            DrivePID.reset();
            SteerPID.reset();
            lastRanDriveTo = System.currentTimeMillis();
            SmartDashboard.putNumber("Target X", x);
            SmartDashboard.putNumber("Target Y", y);            
        }
        //Calculate time step
        double now = System.currentTimeMillis();
        double dT = (now - lastRanDriveTo);
        lastRanDriveTo = now;
                
        double deltaX = x - robot.locator.GetX();
        double deltaY = y - robot.locator.GetY();
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        //System.out.println("DeltaX: "+deltaX+"  DeltaY: "+deltaY);
        System.out.println("Distance: " + distance);
        //determine angle to target relative to field
        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));  // +/- 180 degrees
        System.out.println("Target Heading: "+targetHeading);
        if(speed < 0) {
            //reverse heading if going backwards
            targetHeading += 180;
        }
        
        //Determine  angle to target relative to robot
        
        double bearing = (targetHeading + robot.locator.GetHeading())%360;
        if (bearing > 180) {
            bearing = bearing - 360; //Quicker to turn the other direction
        }
        //System.out.println("bearing: "+bearing);
        /* Steering PID Control */
        steer = SteerPID.calculate(bearing, dT);
        if(steer > 0.15) {
        	steer = 0.15;
        } else if(steer < -0.15) {
        	steer = -0.15;
        }
        //System.out.println("BEARING: "+bearing);
        steer = 0;
        
        /* Drive PID Control */                
        if(speed == 0) {
            //Just turn to the target, no PI Control
            drive = 0;
        } else {
            drive = -1.0*DrivePID.calculate(distance, dT)*speed;
        }        

        //Move the robot - Would this work better if we multiplyed by the steering PID output?
        System.out.println("DRIVE: "+drive);
        System.out.println("STEER: "+steer);
        robot.drive.tankDrive(drive+steer, drive-steer);

                
        if((distance < DriveMargin) || (Math.abs(bearing) < SteerMargin && speed == 0 )) {
            return true;
        } else {
            return false;
        }        
    }

    
    void steer_reset()
    {
    	SteerPID.reset();
        lastRanDriveTo = System.currentTimeMillis();
    }
    
    void steer() {
    	double now = System.currentTimeMillis();
        double dT = (now - lastRanDriveTo);
        lastRanDriveTo = now;
        
        double bearing = robot.locator.GetHeading()%360;
        if (bearing > 180) {
            bearing = bearing - 360; //Quicker to turn the other direction
        }
    	double steer = SteerPID.calculate(bearing, dT);
        robot.drive.tankDrive(steer, -steer);
    }
        
	@Override
	public String log() {
		return null;
	}
}
