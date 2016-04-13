/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardTurnAndFireWithVisionTransfer extends AutoMode {
    
	enum AutoStates {
		PutPickupDown,
		DriveForward,
		Pause,
		Turn,
		DriveForwardAgain,
		PauseForAim,
		Aim,
		Fire,
		PauseForFire,
		Backup,
		BackupTurn,
		BackupAgain,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double pickupDownDelay = 1000;
    double driveForwardDelay = 4500;
    double pauseDelay = 500;
    double turnHeading = 55;
    double turnDelay = 3000;
    double pauseForAimDelay = 1000;
    double driveForwardAgainDelay = 750;
    double backupDelay = driveForwardAgainDelay;
    double backupAgainDelay = 1250; //750 stops just before low bar on PB
    double pauseForFireDelay = 500;
    
    boolean fireNow = false;
    boolean aimNow = false;
    boolean backup;

    
    public AutoModeDriveForwardTurnAndFireWithVisionTransfer(Robot robot, boolean backup) {
        super(robot);
        this.backup = backup; 
        System.out.println("AutoModeDriveForwardWithVision Constructor");
    }    

    public void autoInit() {    	
    	state = AutoStates.PutPickupDown; 
    	fireNow = false;
    	lastDriveForwardEventTime = 0;
    	robot.locator.Reset();
    	robot.shiftPneumatic.set(false); //low gear
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
    			robot.drive.tankDrive(-0.75,-0.75);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Pause;
    			}
    			break;
    		case Pause:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    				robot.shiftPneumatic.set(true); //high gear
    			}		
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Turn;
    			}
    			break;	
    		case Turn:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			if(System.currentTimeMillis() - lastDriveForwardEventTime < 750)
    			{
    				robot.rollerTopMotor.set(-1);
    				robot.rollerSideMotor.set(-1);
    			}
    			else
    			{
    				robot.rollerTopMotor.set(0);
        			robot.rollerSideMotor.set(0);
        			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    			}
    			
    			//robot.drive.tankDrive(-0.75,0.75);
    			robot.drive.tankDrive(-0.75,-0.25); //attempt to curve away from side wall
    			if((robot.locator.GetHeading() > turnHeading && robot.locator.GetHeading() < 180) || System.currentTimeMillis() - lastDriveForwardEventTime > turnDelay) {
    				robot.rollerTopMotor.set(0);
        			robot.rollerSideMotor.set(0);
    				robot.drive.tankDrive(0,0);
    				robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    				lastDriveForwardEventTime = 0;
    				//state = AutoStates.DriveForwardAgain;
    				state = AutoStates.PauseForAim;
    			}
    			break;
    		case DriveForwardAgain:
    			if(lastDriveForwardEventTime == 0) {
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.drive.tankDrive(-0.75, -0.75);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardAgainDelay) {
    				robot.drive.tankDrive(0, 0);
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
    				state = AutoStates.PauseForFire;
    			}
    			break;
    		case PauseForFire:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    				fireNow = false;
    			}		
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseForFireDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;    				
    				state = backup ? AutoStates.BackupTurn : AutoStates.Stop;
    			}
    			break;    			
    		case Backup:
    			if(lastDriveForwardEventTime == 0) {
    				lastDriveForwardEventTime = System.currentTimeMillis();
    				robot.shiftPneumatic.set(false); //low gear
    			}
    			robot.drive.tankDrive(0.75, 0.75);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > backupDelay) {
    				robot.drive.tankDrive(0, 0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.BackupTurn;
    			}
    			break;
    		case BackupTurn:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}   			
    			//robot.drive.tankDrive(0.75,0.25); //working ok, but hitting outside wall on PB
    			robot.drive.tankDrive(0.85,0.20);
    			if((robot.locator.GetHeading() > -5 && robot.locator.GetHeading() < 5) || System.currentTimeMillis() - lastDriveForwardEventTime > turnDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.BackupAgain;
    			}
    			break;
    		case BackupAgain:
    			if(lastDriveForwardEventTime == 0) {
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.drive.tankDrive(0.75, 0.75);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > backupAgainDelay) {
    				robot.drive.tankDrive(0, 0);
    				lastDriveForwardEventTime = 0;
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
