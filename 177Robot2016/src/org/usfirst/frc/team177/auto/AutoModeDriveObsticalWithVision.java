/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeDriveObsticalWithVision extends AutoMode {
    
	enum AutoStates {
		DriveForward,
		Stop,
		TurnToZero,
		Pause,
		Aim,
		Fire
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double driveForwardDelay;  //set in constructor
    double pauseDelay = 1000.0;
    double turnTimeout = 3000;
    double turnSpeed = 0.75;
    
    boolean fireNow = false;
    boolean aimNow = false;

    public AutoModeDriveObsticalWithVision(Robot robot, double driveForwardTime) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
        driveForwardDelay = driveForwardTime;
    }    
    
    public void autoInit() {    	
    	state = AutoStates.DriveForward;   
    	lastDriveForwardEventTime = 0;
    	robot.shiftPneumatic.set(true);
    }

    public void autoPeriodic() {
    	switch(state)
    	{
    		case DriveForward:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.drive.tankDrive(-1,-1);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > 2000)
    			{
    				//put pickup down
    				robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    			}
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    				robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.TurnToZero;
    			}
    			break;
    		case TurnToZero:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			
    			double targetBearing = 0 - robot.locator.GetHeading();
    			if(targetBearing > 180) targetBearing -= 360;
    			if(targetBearing < -180) targetBearing += 360;			
    			
    			SmartDashboard.putString("Aim Sequence", "Target Bearing " + targetBearing);
    			
    			//determine turn direction			
    			if(targetBearing < 0) {
    				//turn left
    				robot.drive.tankDrive(turnSpeed,-1.0*turnSpeed);
    			} else {		
    				//turn right
    				robot.drive.tankDrive(-1.0*turnSpeed,turnSpeed);
    			}
    			
    			
    			if(Math.abs(targetBearing) < 5 
    					|| System.currentTimeMillis() - lastDriveForwardEventTime > turnTimeout) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Pause;
    			}
    			break;
    		case Pause:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    				robot.shiftPneumatic.set(true);
    			}		
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Aim;
    			}
    			break;	
    		case Aim:
    			aimNow = true;
    			state = AutoStates.Fire;
    			break;
    		case Fire:	
    			aimNow = false;
    			if(robot.catapult.getState() == catapultStates.ReadyToFire)
    			{
    				fireNow = true;
    				state = AutoStates.Stop;
    			}
    			break; 
    		case Stop:
    		default:
    			robot.drive.tankDrive(0,0);
    			fireNow = false;
    			break;
		}
    	robot.catapult.loop(fireNow, aimNow);
    }
    	
    public String getName() {
        return "DriveForwardObsticalWithVision";
    }

	@Override
	public String GetColumNames() {
		return "state";
	}


	@Override
	public String log() {
		return String.format("%s", state.toString());
	}
}
