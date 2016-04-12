
package org.usfirst.frc.team177.robot;

//disable vision until we're ready to try it VISION
import org.usfirst.frc.team177.Vision.Vision;
import org.usfirst.frc.team177.auto.*;
import org.usfirst.frc.team177.lib.Locator;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String doNothing = "Do Nothing";
    
    final String driveForwardTransferUp = "Drive Forward(Obstacle)";
    final String driveForwardTransferUpShort = "Drive Forward(Obstacle)Short";
    final String driveForwardOverObsticalWithVision = "Drive Forward (Obstacle) With Vision";
    final String driveForwardOverObsticalWithVisionShort = "Drive Forward (Obstacle) With Vision Short";
    
    final String lowBar = "Low Bar Turn And Fire";
    final String lowBarWithVision = "Low Bar Turn And Fire With Vision";
    
    final String chevalDeFrise = "Cheval De Frise No Fire";
    final String chevalDeFriseFire = "Cheval De Frise Fire With Vision";
    
    final String driveForwardTransferTurnAndFireVision = "Low Bar Transfer Turn And Fire with Vision";
    final String driveForwardTransferTurnAndFireVisionBackup  = "Low Bar Transfer Turn And Fire with Vision Then Backup";
    
    final String driveForwardTransferDownAimFire = "Drive forward, Transfer Down, Aim, Fire";
  
    String autoSelected;
    SendableChooser chooser;
	    
    /**Motor constants**/
	private static final int MotorDriveRL = 3;//Rear Left 888
	private static final int MotorDriveFL = 2; //Front Left 888
	private static final int MotorDriveRR = 1; //Rear Right 888
	private static final int MotorDriveFR = 0; //Front Right 888
	
	private static final int MotorRollerTop = 4; //Top Roller 888
	private static final int MotorRollerSide = 7; //Side Roller 888
	
	private static final int MotorWinch = 5;//2 cims, 5 and 6
	/**Initialize Victors**/
	Victor rearLeftMotor = new Victor(MotorDriveRL);
	Victor frontLeftMotor = new Victor(MotorDriveFL);
	    
	Victor rearRightMotor = new Victor(MotorDriveRR);
	Victor frontRightMotor = new Victor(MotorDriveFR); 
	
	public Victor rollerTopMotor = new Victor(MotorRollerTop);
	public Victor rollerSideMotor = new Victor(MotorRollerSide);
	
	Victor winchMotor = new Victor(MotorWinch); //2 CIM's, y'd PWM
	
	/**Joysticks**/    
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);
	Joystick switchPanel = new Joystick(3);
	
	public RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	
	/** Joystick Constants **/ //Magic Numbers found in Joystick.class
    private static final int axisY = 1;
    
    /** Solenoids **/
	//SAFETY: At the end of the match both the latch and the pusher should be out
	public Solenoid shiftPneumatic = new Solenoid(0);
    public Solenoid latchPneumatic = new Solenoid(1); //false = out
    public DoubleSolenoid transferPneumatic = new DoubleSolenoid(2,3); //false = out
    public DoubleSolenoid pusherPneumatic = new DoubleSolenoid(4,5); //false = out
    public Solenoid climbFourBarPneumatic = new Solenoid(6);
    public Solenoid climbPancakePneumatic = new Solenoid(7);
	
    /** Digital Input **/
    //DigitalInput ballIRSwitch = new DigitalInput(); //RIP IR, died 2/11/16 at the hands of Ulf's SuperAwesome piece of Lexan 
    
    private static final int leftDriveEncoderA = 4;
    private static final int leftDriveEncoderB = 5;
    private static final int rightDriveEncoderA = 6;
    private static final int rightDriveEncoderB = 7;
    
    /** Analog Inputs **/
    private static final int GyroAnalogInput = 0;   
    
    /**Relay Constants**/
    private static final int RelayFlashLight = 0;
    
    /**Relays**/
    Relay FlashLightRelay = new Relay(RelayFlashLight);
    
    enum pickupStates {
    	BallAcquired,
    	TransferUp,
    	DropBall,
    	TransferDown
    };
    
    //catapult;
    public Catapult catapult;
    
    //vision
    public Vision vision; 
    
    //State Machine Pickup
    pickupStates pickupState = pickupStates.BallAcquired;
   
    //State Machine Climber
    long climberEventTime; 
    public enum climbStates {
           	Stowed,
           	PancakeOut,
           	ArmsOut,
           	Climb,
           };
    private climbStates climbState = climbStates.Stowed;
    
    //Controller Mapping
    //Controller
    private static final int ButtonFire = 1;
    private static final int ButtonAimFire = 2; 
    private static final int ButtonOverrideToggle = 4;
    private static final int ButtonOverrideWinchIn = 5;
    private static final int ButtonFlashlight = 6;
    private static final int ButtonTransfer = 7;
    private static final int ButtonSideRollers = 8;
    private static final int ButtonOverrideWinchOut = 10;
    
    //Right Joystick
    private static final int ButtonShift = 3;
    //Left Joystick
    
    
    /* Navigation functions */
    public Locator locator = new Locator(GyroAnalogInput, leftDriveEncoderA, leftDriveEncoderB,
    		rightDriveEncoderA, rightDriveEncoderB);
    
    /* Automode Variables */
    String autoMode = "";
    double autoDelay = 0;    
    AutoMode auto;
    long autoStartTime;
    
    private boolean flashlightOn = false;
    private boolean lastFlashlightButton = false;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault(doNothing, 									doNothing);
        chooser.addObject(driveForwardTransferUp, 						driveForwardTransferUp);
        chooser.addObject(driveForwardTransferUpShort,					driveForwardTransferUpShort);
        chooser.addObject(driveForwardOverObsticalWithVision,			driveForwardOverObsticalWithVision);
        chooser.addObject(driveForwardOverObsticalWithVisionShort, 		driveForwardOverObsticalWithVisionShort);
        chooser.addObject(lowBar, 										lowBar);
        chooser.addObject(lowBarWithVision, 							lowBarWithVision);
        chooser.addObject(chevalDeFrise, 								chevalDeFrise);
        chooser.addObject(chevalDeFriseFire, 							chevalDeFriseFire);
        chooser.addObject(driveForwardTransferTurnAndFireVision,    	driveForwardTransferTurnAndFireVision);
        chooser.addObject(driveForwardTransferTurnAndFireVisionBackup, 	driveForwardTransferTurnAndFireVisionBackup);
        chooser.addObject(driveForwardTransferDownAimFire,				driveForwardTransferDownAimFire);
        
        SmartDashboard.putData("Auto choices", chooser);
        
        String climbTip = null;
		SmartDashboard.putString(climbTip, "Pickup up,full speed at tower.  \n Let go of pickup.  \n Flip Missile Switch. \n Left Stick down.   \n Press and HOLD pickup as soon as robot is no longer touching the ground while winching." );
        catapult = new Catapult(this, latchPneumatic, pusherPneumatic);
        
        locator.start();

        vision = new Vision();

        climbState = climbStates.Stowed;
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    
    public void autonomousInit() {    	
    	
    	locator.Reset();
		if(auto != null) 
		{
            auto.autoInit();
        }						
		autoStartTime = System.currentTimeMillis();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(auto != null && (System.currentTimeMillis() - autoStartTime > autoDelay)) 
		{
            auto.autoPeriodic();        
        } else 
		{
            drive.tankDrive(0, 0);
        }    	
    	SmartDashboard.putNumber("Heading", locator.GetHeading());
    	SmartDashboard.putNumber("X", locator.GetX());
    	SmartDashboard.putNumber("Y", locator.GetY());
    }
    
    long visionTestTimer = 0; 
    @Override
    public void disabledPeriodic() 
	{	
		/**
    	if(visionTestTimer == 0 || System.currentTimeMillis() - visionTestTimer > 15000) {
			double bearing = vision.getBearing();
			SmartDashboard.putNumber("Target Bearing", bearing);
			visionTestTimer = System.currentTimeMillis();
		}
	*/
    	autoSelected = (String) chooser.getSelected();		

		if(!autoSelected.equals(autoMode))
		{
			switch(autoSelected) {
	    		case driveForwardTransferUp:
	    			auto = new AutoModeDriveForwardTransferUp(this, 5000);
	    			break;
	    		case driveForwardTransferUpShort:
	    			auto = new AutoModeDriveForwardTransferUp(this, 4000);
	    			break;
	    		case driveForwardOverObsticalWithVision:
	    			auto = new AutoModeDriveObsticalWithVision(this, 3000);
	    			break;
	    		case driveForwardOverObsticalWithVisionShort:
	    			auto = new AutoModeDriveObsticalWithVision(this, 2000);
	    			break;
	    		case lowBar:
	    			auto = new AutoModeDriveForwardTurnAndFire(this);
	    			break;
	    		case lowBarWithVision:
	    			auto = new AutoModeDriveForwardTurnAndFireWithVision(this);
	    			break;
	    		case chevalDeFrise:
	    			auto = new AutoModeChevalDeFrise(this, false);
	    			break;
	    		case chevalDeFriseFire:
	    			auto = new AutoModeChevalDeFrise(this, true);
	    			break;
	    		case driveForwardTransferTurnAndFireVision:
	    			auto = new AutoModeDriveForwardTurnAndFireWithVisionTransfer(this, false);
	    			break;
	    		case driveForwardTransferTurnAndFireVisionBackup:
	    			auto = new AutoModeDriveForwardTurnAndFireWithVisionTransfer(this, true);
	    			break;	   
	    		case driveForwardTransferDownAimFire:
	    			auto = new AutoModeDriveForwardTransferDownAimFire(this);
	    			break;	    				    			
	    		case doNothing:
	        	default:
	        		//Do Nothing
	        		auto = null;
	                break;        	
			}
			autoMode = autoSelected;
		}
		
		autoDelay = switchPanel.getX()*100;
		SmartDashboard.putNumber("Auto Delay", autoDelay);
		SmartDashboard.putString("Auto Mode", autoMode);
	}
    
    @SuppressWarnings("unused")
	public void teleopInit() {
    	catapult.setState(catapultStates.BallsIn);
    	pickupStates pickupState = pickupStates.BallAcquired;
    	transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    	 climbState = climbStates.Stowed;
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//Driving
    	double left = leftStick.getRawAxis(axisY);
		double right = rightStick.getRawAxis(axisY);
		//uncomment this line to disable driver input while auto aiming also put in the braces
		//if (catapult.getState() != catapultStates.Aiming)
		drive.tankDrive(left, right);
		
		//General Controls
		if (catapult.getState() != catapultStates.Aiming)
		{
			shiftPneumatic.set(rightStick.getRawButton(ButtonShift));
		}
    	transferPneumatic.set(operatorStick.getRawButton(ButtonTransfer) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
		rollerSideMotor.set(operatorStick.getRawButton(ButtonSideRollers) ? -1 : 0);
		rollerTopMotor.set(operatorStick.getRawAxis(1) * -1); //Left Stick, y axis
		
		if(operatorStick.getRawButton(ButtonFlashlight) && !lastFlashlightButton)
		{
			flashlightOn = !flashlightOn;
		}
		lastFlashlightButton = operatorStick.getRawButton(ButtonFlashlight);
		
		if(operatorStick.getRawButton(ButtonFire)) {
			flashlightOn = false;
		}
		
		FlashLightRelay.set(flashlightOn ? Relay.Value.kForward : Relay.Value.kOff);
		
		
		//Catapult Override Control
		if(switchPanel.getRawButton(1))
		{
			latchPneumatic.set(switchPanel.getRawButton(2));
			pusherPneumatic.set(switchPanel.getRawButton(3)?DoubleSolenoid.Value.kForward:DoubleSolenoid.Value.kReverse);
			catapult.setState(catapultStates.BallsIn);			
		}
		else
		{
			catapult.loop(operatorStick.getRawButton(ButtonFire), operatorStick.getRawButton(ButtonAimFire));
		}
		
		//Locator
		SmartDashboard.putNumber("Heading", locator.GetHeading());
		SmartDashboard.putNumber("X", locator.GetX());
		SmartDashboard.putNumber("Y", locator.GetY());
		
		//Climbing Override Control
		if (operatorStick.getRawButton(ButtonOverrideToggle)) {
			if(operatorStick.getRawButton(ButtonOverrideWinchOut)) {
				climbFourBarPneumatic.set(true);
			}
			else {
				climbFourBarPneumatic.set(false);
			}
			winchMotor.set(operatorStick.getRawAxis(3) > 0 ? Math.abs(operatorStick.getRawAxis(3)) : (operatorStick.getRawAxis(3) / 4));	//Down on left stick in climb up is unclimb
		} else {
			switch(climbState) {
			case Stowed:
				climbFourBarPneumatic.set(false);
				winchMotor.set(0);
				if(switchPanel.getRawButton(4)) {
					climbState = climbStates.PancakeOut;
				}
				break;
			case PancakeOut:
				if(climberEventTime == 0) {
					climberEventTime = System.currentTimeMillis();
				}
				climbPancakePneumatic.set(true);
				if(!switchPanel.getRawButton(4)) {
					climbState = climbStates.Stowed;
				}
				if(System.currentTimeMillis() - climberEventTime > 500) {
					climbState = climbStates.ArmsOut;
					climberEventTime = 0;
				}
				break;
			case ArmsOut:
				if(climberEventTime == 0) { 
					climberEventTime = System.currentTimeMillis();
				}
				climbFourBarPneumatic.set(true);
				winchMotor.set(0);
				if(!switchPanel.getRawButton(4)) {
					climbState = climbStates.Stowed;
				}
				if(System.currentTimeMillis() - climberEventTime > 250) { //Magic Delay	
					climbState = climbStates.Climb;
					climberEventTime = 0;
				}
				break;	
			case Climb:
				climbFourBarPneumatic.set(true);
				winchMotor.set((Math.abs(operatorStick.getRawAxis(3)) > 0.5) ? 1 : 0);
				if(!switchPanel.getRawButton(4)) {
					climbState = climbStates.Stowed;
				}
				break;
			default:
				break;
			}
		}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
   
    
}
