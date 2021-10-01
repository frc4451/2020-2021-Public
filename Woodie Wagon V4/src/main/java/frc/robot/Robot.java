
package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.*;
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	/* AUTO COMMAND CHOOSER STUFF */

	private Command m_autonomousCommand;
	SendableChooser<Integer> autoChooser = new SendableChooser<>();
	private RobotContainer m_robotContainer; // instantiated just for getting autonomous command

	/* RUNTIME STATES */

	public static final boolean kDebugEnabled = true; // TURN THIS OFF FOR COMPS

	private boolean disabledSignal = false; // internal checking of enabled state
	public boolean enabled = false; // public enabled state

	/* AUTONOMOUS VARIABLES */

	double startTime;

	int autoMode;

	double leftEncoderMemory = 0;
	double rightEncoderMemory = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		debug("robotInit()");
		/* Instantiate our RobotContainer. This will perform all our button
		  bindings, and put our autonomous chooser on the dashboard.
		 */
		m_robotContainer = new RobotContainer();

		autoChooser.setDefaultOption("Shoot", 0);
		autoChooser.addOption("Shoot Back Shoot", 1);
		autoChooser.addOption("Trench Run", 2);
		autoChooser.addOption("8 Ball", 3);

		SmartDashboard.putData("Auto choices", autoChooser); // make sure key is consistent across projects

		addDebugTags(
				"a" + "\n" +
				"test" + "\n" +
				"debug" + "\n" +
				"!" + "\n"
		);
		setDebugTags();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this
	 * for items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		/*  Runs the Scheduler. This is responsible for polling buttons, adding
		  newly-scheduled commands, running already-scheduled commands, removing
		  finished or interrupted commands, and running subsystem periodic()
		  methods. This must be called from the robot's periodic block in order
		  for anything in the Command-based framework to work.
		*/
		CommandScheduler.getInstance().run();
		enabledController();
		printDashboard();
	}

	// custom method that controls running enabledInit and enabledPeriodic from whether disabledPeriodic is running
	public void enabledController() {
		if (disabledSignal) { // if disabledPeriodic() is running, do this
			enabled = false;
		} else { // else, enabledPeriodic has to be running
			if (!enabled) { // if first iteration of enabled
				enabled = true;
				enabledInit();
			} else {
				enabledPeriodic();
			}
		}
		disabledSignal = false;
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {

		disabledSignal = true;

//		kClimbSub.lockClimberOnDisable(); // closed on exit, commented when rerouted to jack stand
	}

	@Override
	public void disabledPeriodic() {

		disabledSignal = true;

		kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	// custom: runs whenever starting auto or tele from disabled
	public void enabledInit() {
		kLimeLight.enableInit();
		kClimbSub.enableInit();
		kTurretSub.enableInit();
		kDigestiveSystem.enableInit();
		kIntakeSub.enableInit();
		kShooterSub.enableInit();
//		kWheelSub.enableInit();

		kDriveTrain.enableInit();

		// initial odometry update
		kOdometry.update(
				Rotation2d.fromDegrees(kDriveTrain.getGyroHeading()),
				getLeftEncoderMeters(),
				getRightEncoderMeters()
		);
	}

	// custom: runs in both auto and tele
	public void enabledPeriodic() {

		/* Ramsete Stuff */

		kOdometry.update(
				Rotation2d.fromDegrees(kDriveTrain.getGyroHeading()),
				getLeftEncoderMeters(),
				getRightEncoderMeters()
		);

//		kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);

	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {

		disabledSignal = false;
		enabledController();

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		autoMode = autoChooser.getSelected();

		startTime = getTime();

		kShooterSub.setRamp(0);
		kShooterSub.spin(8500);
		kLimeLight.setPipeShort();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		kDriveTrain.motionMagic((kDriveTrain.getLeftEncoderPos()+kDriveTrain.getRightEncoderPos())/2.0);

		if (autoMode == 0) {

			if (getTimeSince(startTime) < 2) {
				kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);

			} else if (getTimeSince(startTime) < 7) {
				kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);
				kDigestiveSystem.feedOn();

			} else if (getTimeSince(startTime) < 14) {
				kDriveTrain.motionMagic(-50000);
				kLimeLight.setPipeDefault();
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kDigestiveSystem.feedOff();

			} else {
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kDigestiveSystem.feedOff();
				kLimeLight.setPipeDefault();
			}

		}

		else if (autoMode == 1) {

			if (getTimeSince(startTime) < 2) {
				// kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);
				kIntakeSub.intakeExtend();

			} else if (getTimeSince(startTime) < 4) {
				// kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);
				kDigestiveSystem.feedOn();

			} else if (getTimeSince(startTime) < 5) {
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kLimeLight.setPipeDefault();
				kDigestiveSystem.feedOff();

			} else if (getTimeSince(startTime) < 8) {
				kDriveTrain.motionMagic(-105279);
				// -105279 is encoder counts target to go backwards
				kShooterSub.setRamp(0);
				kShooterSub.spin(9600);
				kLimeLight.setPipeShort();
				kTurretSub.runTurret();

				leftEncoderMemory = kDriveTrain.getLeftEncoderPos();
				rightEncoderMemory = kDriveTrain.getRightEncoderPos();

			} else if (getTimeSince(startTime) < 11) {
				kDriveTrain.motionMagic(
					leftEncoderMemory + 12000,
					rightEncoderMemory - 12000
				);
				kShooterSub.setRamp(0);
				kShooterSub.spin(9500);
				kLimeLight.setPipeShort();
				kTurretSub.runTurret();

			} else if (getTimeSince(startTime) < 14) {
				kTurretSub.runTurret();
				kDigestiveSystem.feedOn();
				kIntakeSub.intakeRetract();
				kShooterSub.setRamp(0);
				kShooterSub.spin(9500);

			} else {
				kDigestiveSystem.feedOff();
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kLimeLight.setPipeDefault();
			}

		}

		else if(autoMode == 2) {

			if (getTimeSince(startTime) < 2) {
				//kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);

			} else if (getTimeSince(startTime) < 4) {
				//kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);
				kDigestiveSystem.feedOn();

			} else if (getTimeSince(startTime) < 5) {
				kIntakeSub.intakeExtend();
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kLimeLight.setPipeDefault();
				kDigestiveSystem.feedOff();
				kTurretSub.runTurret();

			} else if (getTimeSince(startTime) < 9) {
				kDriveTrain.motionMagic(-196000); /* 105279*/
				kShooterSub.setRamp(0);
				kShooterSub.spin(9700);
				kDigestiveSystem.primeOn();
				kLimeLight.setPipeShort();
				kTurretSub.runTurret();

			} else if (getTimeSince(startTime) < 10.5) {
				kDriveTrain.motionMagic(-117000);
				kShooterSub.setRamp(0);
				kShooterSub.spin(9700);
				kIntakeSub.intakeRetract();
				kDigestiveSystem.feedOff();
				kTurretSub.runTurret();

			} else if (getTimeSince(startTime) < 14) {
				kTurretSub.runTurret();
				kDigestiveSystem.feedOn();
				kShooterSub.setRamp(0);
				kShooterSub.spin(9700);

			} else {
				kDigestiveSystem.feedOff();
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kLimeLight.setPipeDefault();
			}

		}

		else if (autoMode == 3) {
			if (getTimeSince(startTime) < 2) {
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);
				kLimeLight.setPipeShort();
				kTurretSub.runTurret();
				kIntakeSub.intakeExtend();

			} else if (getTimeSince(startTime) < 3.25) {
				kShooterSub.setRamp(0);
				kShooterSub.spin(8500);
				kDigestiveSystem.feedOn();
				kTurretSub.runTurret();

			} else if (getTimeSince(startTime) < 5.75) {
				kDriveTrain.motionMagic(-196000); /* 105279*/
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kDigestiveSystem.primeOn();
				kTurretSub.runTurret();

			} else if (getTimeSince(startTime) < 9) {
				kDriveTrain.motionMagic(-82000);
				kIntakeSub.intakeRetract();
				kDigestiveSystem.primeOn();
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kTurretSub.runTurret();
				leftEncoderMemory = kDriveTrain.getLeftEncoderPos();
				rightEncoderMemory = kDriveTrain.getRightEncoderPos();

			} else if (getTimeSince(startTime) < 10.25) {
				kTurretSub.runTurret();
				kShooterSub.setRamp(0);
				kShooterSub.spin(9700);
				kDriveTrain.motionMagic(
					leftEncoderMemory + 18000,
					rightEncoderMemory - 18000
				);

			} else if (getTimeSince(startTime) < 12.5) {
				kTurretSub.runTurret();
				kDriveTrain.motionMagic(
					leftEncoderMemory + 18000 - 90000,
					rightEncoderMemory - 18000 - 90000
				);
				kShooterSub.setRamp(0);
				kShooterSub.spin(9700);

			} else if (getTimeSince(startTime) < 14.9) {
				kTurretSub.runTurret();
				kDigestiveSystem.feedOn();
				kShooterSub.setRamp(0);
				kShooterSub.spin(9700);
				kIntakeSub.intakeExtend();

			} else {
				kIntakeSub.intakeRetract();
				kDigestiveSystem.feedOff();
				kShooterSub.setRamp(2);
				kShooterSub.spin(0);
				kLimeLight.setPipeDefault();
			}
		}

	}

	@Override
	public void teleopInit() {

		disabledSignal = false;
		enabledController();

		kDriveTrain.teleOpInit();

		/*
		 * This makes sure that the autonomous stops running when
		 * 		 teleop starts running. If you want the autonomous to
		 * 		 continue until interrupted by another command, remove
		 * 		 this line or comment it out.
		 */
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		/* Drivetrain Moving */

		kDriveTrain.runDriveTrain();
//		kDriveTrain.runDriveTrainStraight();

		/* Limelight Pipelines */

		kLimeLight.runLimelight();

		/* Turret Movements */

		kTurretSub.runTurret();

		/* Shooter Spinning Control*/

		kShooterSub.shooterSpin();

		/* Intake, Priming, and Feeding */

		kDigestiveSystem.runDigestive();

		/* Climb with operator */

		kClimbSub.runClimb();

		/* Intake Motor */

		kIntakeSub.runIntake();

		/* Color Wheel */

//		kWheelSub.runWheel();

		//this is for debugging.
		dump();
		
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		kDriveTrain.debugEncoders();
		dump();
	}

}
