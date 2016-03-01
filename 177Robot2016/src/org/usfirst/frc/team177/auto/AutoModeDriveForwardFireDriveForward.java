/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardFireDriveForward extends AutoMode {
    
	enum AutoStates {
		DriveForward,
		Pause,
		TurnLeft,
		TurnRight,
		Fire,
		DriveAgain,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double pickupDownDelay = 2000;
    double driveForwardDelay = 3000;
    double pauseDelay = 2000;
    double driveForwardAgainDelay = 1000;
    double turnDelay = 2000;
    
    
    public AutoModeDriveForwardFireDriveForward(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
    }    
    
    public void autoInit() {    	
    	state = AutoStates.DriveForward; 
    	lastDriveForwardEventTime = 0;
    }

    public void autoPeriodic() {
    	boolean fireNow = false;

    	
    	switch(state)
    	{
    	case DriveForward:
    		if(lastDriveForwardEventTime == 0) { 
    			lastDriveForwardEventTime = System.currentTimeMillis();
    		}
    		robot.drive.tankDrive(-1,-1);
    		if(System.currentTimeMillis() - lastDriveForwardEventTime > 1000)
    		{
    			//put pickup down
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
    		}
    		if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    			robot.drive.tankDrive(0,0);
    			lastDriveForwardEventTime = 0;
    			state = AutoStates.Pause;
    		}
    		break;
    	case Pause:
    		if(lastDriveForwardEventTime == 0) { 
    			lastDriveForwardEventTime = System.currentTimeMillis();
    		}		
    		if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseDelay) {
    			robot.drive.tankDrive(0,0);
    			lastDriveForwardEventTime = 0;
    			if(robot.locator.GetHeading() < 180) {
    				state = AutoStates.TurnLeft;
    			} else {
    				state = AutoStates.TurnRight;
    			}
    		}
    		break;	
    	case TurnLeft:
    		if(lastDriveForwardEventTime == 0) { 
    			lastDriveForwardEventTime = System.currentTimeMillis();
    		}
    		robot.drive.tankDrive(0.5,-0.5);
    		if(robot.locator.GetHeading() > 180  || System.currentTimeMillis() - lastDriveForwardEventTime > turnDelay) {
    			robot.drive.tankDrive(0,0);
    			lastDriveForwardEventTime = 0;
    			state = AutoStates.Fire;
    		}
    		break;
    	case TurnRight:
    		if(lastDriveForwardEventTime == 0) { 
    			lastDriveForwardEventTime = System.currentTimeMillis();
    		}
    		robot.drive.tankDrive(-0.5,0.5);
    		if(robot.locator.GetHeading() < 180  || System.currentTimeMillis() - lastDriveForwardEventTime > turnDelay) {
    			robot.drive.tankDrive(0,0);
    			lastDriveForwardEventTime = 0;
    			state = AutoStates.Fire;
    		}
    		break;
    	case Fire:	
    		fireNow = true;
    		state = AutoStates.DriveAgain;
    		break;    
    	case DriveAgain:
    		if(lastDriveForwardEventTime == 0) { 
    			lastDriveForwardEventTime = System.currentTimeMillis();
    		}
    		robot.drive.tankDrive(-1,-1);
    		if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardAgainDelay) {
    			robot.drive.tankDrive(0,0);
    			lastDriveForwardEventTime = 0;
    			state = AutoStates.Stop;
    		}
    		break;
    	case Stop:
    	default:
    		robot.drive.tankDrive(0,0);
    		break;
		}
    	robot.catapult.loop(fireNow);
    }
    	
    public String getName() {
        return "DriveForwardTransferDown";
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
