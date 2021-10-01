// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Code
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.RobotContainer.Subsystems.*; // for easy access
import static frc.robot.RobotMath.*;

//WPI
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot
{

	// Auto Chooser Stuff
	private static final String DEFAULT_AUTO = "Default";
	private static final String PATHWEAVER_TEST = "PathWeaver Routine Test"; // filename "test"
	private static final String LINEAR_TEST_A = "Linear Test A"; // filename "linearTest"
	private static final String LINEAR_TEST_B = "Linear Test B"; // filename "linearTest2"
	private static final String ROUTINE_TEST = "Routine Test (don't click)";
	private static final String CENTER_TRENCH = "1: Center to Trench";
	private static final String CENTER_TRENCH_BACK_LINE = "2: Center to Trench: Back Line";
	private static final String CENTER_TEXAS = "3: Center to Rendezvous: Texas ver.";
	private static final String CROSS_COURT = "4: Cross Court";
	private String autoSelected;
	public static AutoRoutine activeAuto; // TODO I am not comfortable with this being public
	public static final SendableChooser<String> chooser = new SendableChooser<>();

	// Runtime States
	private static Constants.GameState robotGameState = Constants.GameState.Disabled;
	public boolean isEnabled = false; // public enabled state, start as disabled

	// Auto Variables (going to migrate these to AutoRoutine class)
	double autoRoutineStartTime;


	/* =================================================================================================================
	 * MAIN LOOP THAT RUNS ALL THE TIME
	 */

	@Override
	public void robotInit() {
		debug("robotInit()");

		robotGameState = Constants.GameState.Disabled;

		// configure the debug object's tags
		addDebugTags(
//				"a" + "\n" +
//						"system" + "\n" +
//						"test" + "\n" +
						"debug" + "\n"// +
//						"!" + "\n"
		);
		setDebugTags();

		// add Auto Chooser to SmartDashboard


		chooser.setDefaultOption(DEFAULT_AUTO, DEFAULT_AUTO);
//		chooser.addOption(PATHWEAVER_TEST, PATHWEAVER_TEST);
//		chooser.addOption(LINEAR_TEST_A, LINEAR_TEST_A);
//		chooser.addOption(LINEAR_TEST_B, LINEAR_TEST_B);
//		chooser.addOption(ROUTINE_TEST, ROUTINE_TEST);
		chooser.addOption(CENTER_TRENCH, CENTER_TRENCH);
		chooser.addOption(CENTER_TRENCH_BACK_LINE, CENTER_TRENCH_BACK_LINE);
		chooser.addOption(CENTER_TEXAS, CENTER_TEXAS);
		chooser.addOption(CROSS_COURT, CROSS_COURT);
		SmartDashboard.putData("Auto choices", chooser); // make sure key is consistent across projects

		// set limelight to default (light off)
		kLimeLight.setPipeDefault();
	}

	// This method gets run after the other portion specific loops, so after disabled/auto/teleop/test runs
	@Override
	public void robotPeriodic() {
		dump();
		printDashboard();
	}


	/* =================================================================================================================
	 * STATE MACHINE ADDON METHODS
	 */

	// @CUSTOM: runs whenever starting auto or tele from disabled
	public void enabledInit() {
		if (!isEnabled) { // this case is when mode goes from disabled to enabled
			isEnabled = true; //IMPORTNAT NO TOUCH
			debug("robot enable detected by enabledInit()", "system");

			kDriveTrain.enableInit();
			kClimber.enableInit();
			kDigestiveSystem.enableInit();
			kLimeLight.enableInit();
			kShooterHead.enableInit();
			kColorWheel.enableInit();


		} else { // this case is when modes switch, say from auto to teleop.
			debug("switch between modes detected by enabledInit()", "system");
		}
	}

	// @CUSTOM: runs in auto, teleop, and test, before the main functions
	// (this SHOULD BE called by them before they do anything else)
	public void enabledPeriodic() {

	}


	/* =================================================================================================================
	 * ACTUAL RUNTIME STATE MACHINE METHODS
	 */


	@Override
	public void autonomousInit() {
		enabledInit(); //IMPORTNAT NO TOUCH

		robotGameState = Constants.GameState.Autonomous;

		autoSelected = chooser.getSelected();

		switch (autoSelected)
		{
//			case LINEAR_TEST_A:
//				currentTrack = linearTest;
//				break;
//			case LINEAR_TEST_B:
//				currentTrack = linearTestTwo;
//				break;
//			case PATHWEAVER_TEST:
//				activeAuto = AutoRoutines.pathWeaverTestRoutine;
//				break;
//			case ROUTINE_TEST:
//				activeAuto = AutoRoutines.routineTest;
//				break;
			case CENTER_TRENCH:
				activeAuto = AutoRoutines.centerToTrench;
				break;
			case CENTER_TRENCH_BACK_LINE:
				activeAuto = AutoRoutines.centerToTrenchBackLine;
				break;
			case CENTER_TEXAS:
				activeAuto = AutoRoutines.centerToRendezvousTexas;
				break;
			case CROSS_COURT:
				activeAuto = AutoRoutines.crossCourt;
				break;
			case DEFAULT_AUTO:
			default:
				// don't do anything in default (for safety)
				break;
		}

		autoRoutineStartTime = getTime();

	}

	/** This method is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		enabledPeriodic(); //IMPORTNAT NO TOUCH

		// Auto update and drivetrain

		kDriveTrain.updateOdometry(); // This is being done in robotPeriodic // not anymore


		if (activeAuto.iterate()) {
			if (activeAuto.resetSignal) {
				kDriveTrain.resetOdometry(activeAuto.resetPose);
			} else {
				kDriveTrain.drive(activeAuto.driveForward, activeAuto.driveTurn);
			}

			// Turret Seeking and Limelight

			if (activeAuto.aim) {
				kLimeLight.setPipeTeleOp(); // We really only need one pipeline
				kShooterHead.runTurret();
			} else {
				kLimeLight.setPipeDefault();
			}

			kLimeLight.runLimelight(); // does status light

			// Intake and Digestion

			if (activeAuto.intake) {
				kDigestiveSystem.intakeExtend();
				if (activeAuto.digest) {
					kDigestiveSystem.primeOn();
					kDigestiveSystem.intakeRoll();
				} else {
					kDigestiveSystem.feedOff();
					kDigestiveSystem.intakeStop();
				}
			} else {
				kDigestiveSystem.intakeRetract();

				kDigestiveSystem.feedOff();
				kDigestiveSystem.intakeStop();
			}

			// SpinUp and Shooting

			kShooterHead.spinUp(activeAuto.spinUp);

			if (activeAuto.shoot) {
				kDigestiveSystem.safeShootFeed();
			}

		} else { // Do the same thing, TODO: Be more smart

			if (activeAuto.resetSignal) {
				kDriveTrain.resetOdometry(activeAuto.resetPose);
			} else {
				kDriveTrain.drive(activeAuto.driveForward, activeAuto.driveTurn);
			}

			// Turret Seeking and Limelight

			if (activeAuto.aim) {
				kLimeLight.setPipeTeleOp(); // We really only need one pipeline
				kShooterHead.runTurret();
			} else {
				kLimeLight.setPipeDefault();
			}

			kLimeLight.runLimelight(); // does status light

			// Intake and Digestion

			if (activeAuto.intake) {
				kDigestiveSystem.intakeExtend();
				if (activeAuto.digest) {
					kDigestiveSystem.primeOn();
					kDigestiveSystem.intakeRoll();
				} else {
					kDigestiveSystem.feedOff();
					kDigestiveSystem.intakeStop();
				}
			} else {
				kDigestiveSystem.intakeRetract();

				kDigestiveSystem.feedOff();
				kDigestiveSystem.intakeStop();
			}

			// SpinUp and Shooting

			kShooterHead.spinUp(activeAuto.spinUp);

			if (activeAuto.shoot) {
				kDigestiveSystem.safeShootFeed();
			}
		}


	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		enabledInit(); //IMPORTNAT NO TOUCH

		robotGameState = Constants.GameState.TeleOp;

		kLimeLight.setPipeTeleOp();
	}

	/** This method is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		enabledPeriodic(); //IMPORTNAT NO TOUCH
		IO.updateIO(); // this makes it so that custom controller input variables are up to date

		// toggle between teleop limelight and turret tracking, or default limelight and manual turret turning
		if (IO.Operator.rCenterPressed()) {
			if (kLimeLight.getPipeline() == Constants.LimelightPipeline.Default.val()) {
				kLimeLight.setPipeTeleOp();
			} else {
				kLimeLight.setPipeDefault();
			}
		}

		// control drivetrain with velocity drive
		kDriveTrain.runDriveTrain();
		kShooterHead.runTurret();
		kLimeLight.runLimelight(); // does status light
//		kDigestiveSystem.intakeRetract(); // use left and right center driver buttons to control intake
		kShooterHead.runShooter();
		kDigestiveSystem.runDigestive();
		kClimber.runClimb();
		kColorWheel.runColorWheel();

	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		isEnabled = false; //IMPORTNAT NO TOUCH

		robotGameState = Constants.GameState.Disabled;

		kLimeLight.setPipeDefault();
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {
	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {
		enabledInit(); //IMPORTNAT NO TOUCH

		robotGameState = Constants.GameState.Test;
	}

	/** This method is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		enabledPeriodic(); //IMPORTNAT NO TOUCH
	}

	/** This method is for dashboard printing :} */
	public static String getGameState() {
		return robotGameState.string();
	}
}
