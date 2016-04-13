/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardTransferDownAimFire extends AutoMode {
    
	enum AutoStates {
		PutPickupDown,
		DriveForward,
		PauseForAim,
		Aim,
		Fire,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    
    double pickupDownDelay = 1000;
    double driveForwardDelay = 3000;
    double pauseForAimDelay = 1000;
    
    double driveForwardSpeed = -0.75;
        
    boolean fireNow = false;
    boolean aimNow = false;

    public AutoModeDriveForwardTransferDownAimFire(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForwardTransferDownAimFire Constructor");
    }    
    
    public void autoInit() {    	
    	state = AutoStates.PutPickupDown; 
    	fireNow = false;
    	lastDriveForwardEventTime = 0;
    	robot.locator.Reset();
    	robot.shiftPneumatic.set(true); //high gear
    }

    public void autoPeriodic() {
    	switch(state)
    	{
    		case PutPickupDown:	
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pickupDownDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.DriveForward;
    			}
    			break;

    		case DriveForward:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    			robot.drive.tankDrive(driveForwardSpeed, driveForwardSpeed);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.PauseForAim;
    			}
    			break;    		
    		case PauseForAim:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}		
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseForAimDelay) {
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
