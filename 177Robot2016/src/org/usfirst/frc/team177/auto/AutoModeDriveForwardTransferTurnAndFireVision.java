package org.usfirst.frc.team177.auto;

import java.io.ObjectOutputStream.PutField;

import org.usfirst.frc.team177.auto.AutoModeDriveForwardTurnAndFire.AutoStates;
import org.usfirst.frc.team177.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardTransferTurnAndFireVision extends AutoMode {

	enum AutoStates {
		PutPickupDown,
		DriveForward,
		Pause,
		PutPickupUp,
		Transfer,
		PutPickupDownAgain,
		Turn,
		PauseForAim,
		Aim,
		PauseAgain,
		Fire,
		Stop
	};
	
	private AutoStates state = AutoStates.PutPickupDown;
	long lastEventTime = 0;
	double pickupDownDelay = 2000;
	double driveForwardDelay = 4500;
	double pauseDelay = 500;
	double PickupUpDelay = 2000;
	double transferDelay = 1000;
	double turnHeading = 55;
	double turnDelay = 3000;
	double pauseForAimDelay = 1000;
	
	boolean fireNow = false;
	boolean aimNow = false;
	
	public AutoModeDriveForwardTransferTurnAndFireVision(Robot robot) {
		super(robot);
		System.out.println("AutoModeDriveForward Constructor");
	}
	
	public void autoInit() {    	
	    state = AutoStates.PutPickupDown; 
	    fireNow = false;
	    lastEventTime = 0;
	    robot.locator.Reset();
	    robot.shiftPneumatic.set(false); // Low gear
	}
	
	public void autoPeriodic() {
		switch(state) {
		case PutPickupDown:
			if(lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
			if(System.currentTimeMillis() - lastEventTime > pickupDownDelay) {
				robot.drive.tankDrive(0, 0);
				lastEventTime = 0;
				state = AutoStates.DriveForward;
			}
			break;
		case DriveForward:
			if(lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
			robot.drive.tankDrive(-0.75,-0.75);
			if(System.currentTimeMillis() - lastEventTime > driveForwardDelay) {
				robot.drive.tankDrive(0, 0);
				lastEventTime = 0;
				state = AutoStates.Pause;
			}
			break;
		case Pause:
			if(lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			if(System.currentTimeMillis() - lastEventTime > pauseDelay) {
				robot.drive.tankDrive(0, 0);
				lastEventTime = 0;
				state = AutoStates.PutPickupUp;
			}
			break;
		case PutPickupUp:
			if(lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
			if(System.currentTimeMillis() - lastEventTime > PickupUpDelay) {
				robot.drive.tankDrive(0, 0);
				lastEventTime = 0;
				state = AutoStates.Transfer;
			}
			break;
		case Transfer:
			if(lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.rollerTopMotor.set(-1);
			robot.rollerSideMotor.set(-1);
			if(System.currentTimeMillis() - lastEventTime > transferDelay) {
				robot.drive.tankDrive(0, 0);
				robot.rollerSideMotor.set(0);
				robot.rollerTopMotor.set(0);
				lastEventTime = 0;
				state = AutoStates.PutPickupDownAgain;
			}
			break;
		case PutPickupDownAgain:
			if(lastEventTime == 0) {
				lastEventTime = System.currentTimeMillis();
			}
			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
			if(System.currentTimeMillis() - lastEventTime > pickupDownDelay) {
				robot.drive.tankDrive(0, 0);
				lastEventTime = 0;
				state = AutoStates.Turn;
			}
			break;
		case Turn:
			if(lastEventTime == 0) { 
				lastEventTime= System.currentTimeMillis();
			}
			robot.drive.tankDrive(-0.75,0.75);
			if((robot.locator.GetHeading() > turnHeading && robot.locator.GetHeading() < 180) || System.currentTimeMillis() - lastEventTime > turnDelay) {
				robot.drive.tankDrive(0,0);
				lastEventTime = 0;
				state = AutoStates.Fire;
			}
			break;
		case PauseForAim:
			if(lastEventTime == 0) { 
				lastEventTime = System.currentTimeMillis();
			}		
			if(System.currentTimeMillis() - lastEventTime > pauseForAimDelay) {
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
			fireNow = true;
			state = AutoStates.Stop;
			break;
		case Stop:
		default:
			robot.drive.tankDrive(0, 0);
			fireNow = false;
			break;
		}
		robot.catapult.loop(fireNow,aimNow);
	}
	

	@Override
	public String GetColumNames() {
		return "AutoModeDriveForwardTransferVision";
	}

	@Override
	public String getName() {
		return "state";
	}
}

