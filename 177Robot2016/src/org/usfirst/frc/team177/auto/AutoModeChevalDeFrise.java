package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeChevalDeFrise extends AutoMode {
	
	enum AutoStates {
		DriveForward,
		PutPickupDown,
		Stop
	}
	
	private AutoStates state;
	long lastEventTime;
	int driveCount; //iterates so that I dont have duplicate cases
	//Constants
	double firstDriveForwardDelay = 2000; //untested
	double pickupDownDelay = 2000; //untested
	double secondDriveForwardDelay = 2000; //untested
	
	public AutoModeChevalDeFrise(Robot robot) {
		super(robot);
		System.out.println("AutoModeChevalDeFrise Constructor");
	}
	
	public void autoInit() {
		lastEventTime = 0;
		state = AutoStates.DriveForward;
		driveCount = 1;
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
					state = AutoStates.Stop;
				}
			}
			break;
		case PutPickupDown:
			if (lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
			if (System.currentTimeMillis() - lastEventTime > pickupDownDelay) {
				robot.drive.tankDrive(0, 0);
				lastEventTime = 0;
				state = AutoStates.DriveForward;
			}
			break;
		case Stop:
		default:
			robot.drive.tankDrive(0, 0);
			break;
		}
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
