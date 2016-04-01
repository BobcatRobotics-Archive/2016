package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeChevalDeFrise extends AutoMode {
	
	enum AutoStates {
		DriveForward,
		PutPickupDown,
		Pause,
		Aim,
		Fire,
		Stop
		
	}
	
	private AutoStates state;
	long lastEventTime;
	int driveCount; //iterates so that I dont have duplicate cases
	//Constants
	double firstDriveForwardDelay = 1250; //untested
	double pickupDownDelay = 3000; //untested
	double secondDriveForwardDelay = 2500; //untested

    boolean fireNow = false;
    boolean aimNow = false;
    
    boolean fireAfterCrossing;
	
	public AutoModeChevalDeFrise(Robot robot, boolean fireAfterCrossing) {
		super(robot);
		this.fireAfterCrossing = fireAfterCrossing;
		System.out.println("AutoModeChevalDeFrise Constructor");
	}
	
	public void autoInit() {
		lastEventTime = 0;
		state = AutoStates.DriveForward;
		driveCount = 1;
		robot.shiftPneumatic.set(false); //Low Gear
	}
	
	public void autoPeriodic() {
		switch(state)
		{
		case DriveForward:
			if (lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			if (driveCount == 1) { //first driveforward
				robot.drive.tankDrive(-0.75, -0.75);
				if (System.currentTimeMillis() - lastEventTime > firstDriveForwardDelay) {
					robot.drive.tankDrive(0, 0);
					lastEventTime = 0;
					driveCount++;
					state = AutoStates.PutPickupDown;
				} 
			} else { //second driveforward
				robot.drive.tankDrive(-0.75,-0.75);
				if (System.currentTimeMillis() - lastEventTime > secondDriveForwardDelay) {
					robot.drive.tankDrive(0,0);
					lastEventTime = 0;
					robot.shiftPneumatic.set(true); // High gear
					if (fireAfterCrossing) {
						state = AutoStates.Pause;
					} else {
						state = AutoStates.Stop;
					}
				}
			}
			break;
		case PutPickupDown:
			if (lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
			robot.drive.tankDrive(0, 0);
			if (System.currentTimeMillis() - lastEventTime > pickupDownDelay) {	
				lastEventTime = 0;
				state = AutoStates.DriveForward;				
			}
			break;
		case Pause:
			if(lastEventTime == 0) { 
				lastEventTime = System.currentTimeMillis();
			}		
			if(System.currentTimeMillis() - lastEventTime > 1000) {
				robot.drive.tankDrive(0,0);
				lastEventTime = 0;
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
			robot.drive.tankDrive(0, 0);
			fireNow = false;
			break;
		}
		

    	robot.catapult.loop(fireNow, aimNow);
	}

	@Override
	public String GetColumNames() {
		return "state";
	}

	@Override
	public String getName() {
		return String.format("%s", state.toString());
	}
}
