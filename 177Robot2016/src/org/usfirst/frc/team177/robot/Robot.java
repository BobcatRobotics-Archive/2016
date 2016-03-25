
package org.usfirst.frc.team177.robot;

//disable vision until we're ready to try it
import org.usfirst.frc.team177.Vision.Vision;
import org.usfirst.frc.team177.auto.*;
import org.usfirst.frc.team177.lib.Locator;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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
    final String driveForwardTransferDown = "Drive Forward Transfer Down";
    final String driveForwardTransferUp = "Drive Forward Transfer Up";
    final String driveForwardTransferUpShort = "Drive Forward Transfer Up Short";
    final String driveToForwardTransferDown = "Drive To Forward Transfer Down";
    final String driveForwardTransferUpThenBack = "Drive Forward Then Back";
    final String driveForwardTransferUpTurnAndFire = "Drive Forward Up Then Turn And Fire";
    final String driveForwardFireDriveForward = "Drive Forward, Fire, Drive Forward";
    final String driveForwardTransferUpTurnAndFireWithVision = "Drive Forward Up Then Turn And Fire With Vision";
    final String roughTerain = "Rough Terain";
    String autoSelected;
    SendableChooser chooser;
	    
    /**Motor constants**/
	private static final int MotorDriveRL = 3;//Rear Left 888
	private static final int MotorDriveFL = 2; //Front Left 888
	private static final int MotorDriveRR = 1; //Rear Right 888
	private static final int MotorDriveFR = 0; //Front Right 888
	
	private static final int MotorRollerTop = 4; //Top Roller 888
	private static final int MotorRollerSide = 7; //Side Roller 888
	
	private static final int MotorWinch = 5; 
	/**Initialize Victors**/
	Victor rearLeftMotor = new Victor(MotorDriveRL);
	Victor frontLeftMotor = new Victor(MotorDriveFL);
	    
	Victor rearRightMotor = new Victor(MotorDriveRR);
	Victor frontRightMotor = new Victor(MotorDriveFR); 
	
	Victor rollerTopMotor = new Victor(MotorRollerTop);
	Victor rollerSideMotor = new Victor(MotorRollerSide);
	
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
    public Solenoid climbPneumatic = new Solenoid(6);
	
    /** Digital Input **/
    //DigitalInput ballIRSwitch = new DigitalInput(); //RIP IR, died 2/11/16 at the hands of Ulf's SuperAwesome piece of Lexan 
    
    private static final int leftDriveEncoderA = 4;
    private static final int leftDriveEncoderB = 5;
    private static final int rightDriveEncoderA = 6;
    private static final int rightDriveEncoderB = 7;
    
    /** Analog Inputs **/
    private static final int GyroAnalogInput = 0;   
    
    
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
           	ArmsOut,
           	PickupDown,
           	Climb,
           };
    private climbStates climbState = climbStates.Stowed;
    
    //Controller Mapping
    //Controller
    private static final int ButtonTransfer = 7;
    private static final int ButtonSideRollers = 8;
    private static final int ButtonFire = 1;
    private static final int ButtonAimFire = 2; 
    private static final int ButtonOverrideToggle = 4;
    private static final int ButtonOverrideWinchIn = 5;
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
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Do Nothing", doNothing);
        chooser.addObject("Drive Forward(LowBar)", driveForwardTransferDown);
        chooser.addObject("Drive Forward(Obstacle)", driveForwardTransferUp);
        chooser.addObject("Drive Forward(Obstacle) Short",driveForwardTransferUpShort);
        chooser.addObject("Drive To Forward LowBar", driveToForwardTransferDown);
        chooser.addObject("Drive To Forward LowBar Turn And Fire", driveForwardTransferUpTurnAndFire);
        chooser.addObject("Drive To Forward LowBar Then Back", driveForwardTransferUpThenBack);
        chooser.addObject("Drive Forward, Fire, Drive Forward", driveForwardFireDriveForward);
        chooser.addObject(driveForwardTransferUpTurnAndFireWithVision, driveForwardTransferUpTurnAndFireWithVision);
        chooser.addObject(roughTerain, roughTerain);
        SmartDashboard.putData("Auto choices", chooser);
        
        
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
		if(visionTestTimer == 0 || System.currentTimeMillis() - visionTestTimer > 15000) {
			double bearing = vision.getBearing();
			SmartDashboard.putNumber("Target Bearing", bearing);
			visionTestTimer = System.currentTimeMillis();
		}
	
    	autoSelected = (String) chooser.getSelected();		

		if(!autoSelected.equals(autoMode))
		{
			switch(autoSelected) {
	    		case driveForwardTransferDown:
	    			auto = new AutoModeDriveForwardTransferDown(this);
	    			break;
	    		case driveForwardTransferUp:
	    			auto = new AutoModeDriveForwardTransferUp(this, 5000);
	    			break;
	    		case driveForwardTransferUpShort:
	    			auto = new AutoModeDriveForwardTransferUp(this, 4000);
	    			break;
	    		case driveToForwardTransferDown:
	    			auto = new AutoModeDriveToTransferDown(this);
	    			break;
	    		case driveForwardTransferUpThenBack:
	    			auto = new AutoModeDriveForwardThenBackTD(this);
	    			break;
	    		case driveForwardTransferUpTurnAndFire:
	    			auto = new AutoModeDriveForwardTurnAndFire(this);
	    			break;
	    		case driveForwardTransferUpTurnAndFireWithVision:
	    			auto = new AutoModeDriveForwardTurnAndFireWithVision(this);
	    			break;
	    		case driveForwardFireDriveForward:
	    			auto = new AutoModeDriveForwardFireDriveForward(this);
	    			break;
	    		case roughTerain:
	    			auto = new AutoModeRoughTerain(this, 4000);
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
		shiftPneumatic.set(rightStick.getRawButton(ButtonShift));	
    	transferPneumatic.set(operatorStick.getRawButton(ButtonTransfer) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    	if(operatorStick.getRawButton(ButtonSideRollers)) {
			rollerSideMotor.set(-1);
		}
		else {
			rollerSideMotor.set(0);
		}
		rollerTopMotor.set(operatorStick.getRawAxis(1) * -1); //Left Stick, y axis
	
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
		SmartDashboard.putNumber("Heading", locator.GetHeading());
		SmartDashboard.putNumber("X", locator.GetX());
		SmartDashboard.putNumber("Y", locator.GetY());
		
		if (operatorStick.getRawButton(ButtonOverrideToggle)) {
			if(operatorStick.getRawButton(ButtonOverrideWinchIn)) {
				winchMotor.set(0.5);	
			} else if (operatorStick.getRawButton(ButtonOverrideWinchOut)) {
				winchMotor.set(-0.5);
			} else {
				winchMotor.set(0);
			}
		} else {
			switch(climbState) {
			case Stowed:
				climbPneumatic.set(false);
				winchMotor.set(0);
				if(switchPanel.getRawButton(4)) {climbState = climbStates.ArmsOut;}
				break;
			case ArmsOut:
				if(climberEventTime == 0) {climberEventTime = System.currentTimeMillis();}
				climbPneumatic.set(true);
				winchMotor.set(0);
				if(System.currentTimeMillis() - climberEventTime > 500) { //Magic Delay
					climbState = climbStates.PickupDown;
				}
				break;
			case PickupDown:
				if(climberEventTime ==0) {climberEventTime = System.currentTimeMillis();}
				climbPneumatic.set(true);
				winchMotor.set(0);
				transferPneumatic.set(Value.kReverse); //down
				if(System.currentTimeMillis() - climberEventTime > 500) { //Magic Delay
					climbState = climbStates.Climb;
				}
				break;
			case Climb:
				climbPneumatic.set(false);
				winchMotor.set(operatorStick.getRawAxis(3));
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
