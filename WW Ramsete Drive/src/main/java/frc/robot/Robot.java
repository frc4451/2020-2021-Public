// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotContainer.*;
import static frc.robot.RobotMath.*;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
	// auto string identifiers
	private static final String DEFAULT_AUTO = "Default";
	private static final String FUN = "Fun";
	private static final String PATH_TEST = "Path Test";
	private static final String LINEAR_TEST = "Linear Test";
	private static final String SEARCH = "Decide Search";
	private static final String RED_A = "Search Red A";
	private static final String BLUE_A = "Search Blue A";
	private static final String RED_B = "Search Red B";
	private static final String BLUE_B = "Search Blue B";
	private static final String SLALOM = "Slalom";
	private static final String BOUNCE_PATH = "Bounce Path";
	private static final String BARREL_RACE = "Barrel Race";
	private static final String CT = "6 Ball: Center to Trench";
	// auto selected from dashboard
	private String autoSelected;
	private final SendableChooser<String> chooser = new SendableChooser<>();

	// auto trajectory variables
	private AutoPortion kPortion1;
	private AutoPortion kPortion2;
	private AutoPortion kPortion3;
	private AutoPortion kPortion4;

	// internal auto variables
	private double kAutoStartTime;
	private double timeMem;
	private double kAutoDelay;
	private int numOfTrajectories;
	private int currentTrajectory = 0;

	// print to dashboard delay counter
	private int dashboardCounter = 0;

	// variables for tele-auto
	private int teleOpPath = 0; // 1: going back (seek 1), 2: going forward (seek 0)
	private int teleOpPathPosition = 0; // 0: shooting pos, 1: loading pos

	/**
	 * This method is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit()
	{
		// dashboard auto chooser add options
		chooser.setDefaultOption(DEFAULT_AUTO, DEFAULT_AUTO);
		chooser.addOption(FUN, FUN);
		chooser.addOption(PATH_TEST, PATH_TEST);
		chooser.addOption(LINEAR_TEST,LINEAR_TEST);
		chooser.addOption(SEARCH, SEARCH);
		chooser.addOption(RED_A, RED_A);
		chooser.addOption(BLUE_A, BLUE_A);
		chooser.addOption(RED_B, RED_B);
		chooser.addOption(BLUE_B, BLUE_B);
		chooser.addOption(SLALOM, SLALOM);
		chooser.addOption(BOUNCE_PATH, BOUNCE_PATH);
		chooser.addOption(BARREL_RACE, BARREL_RACE);
		chooser.addOption(CT, CT);
		SmartDashboard.putData("Auto choices", chooser); // make sure key is consistent across projects

		kLimelight.setPipeDefault();

		// print predicted galactic search track times
		System.out.println(RED_A + ": " + redA.getTotalTimeSeconds()); //	#1
		System.out.println(BLUE_A + ": " + blueA.getTotalTimeSeconds()); //	#2
		System.out.println(RED_B + ": " + redB.getTotalTimeSeconds()); //	#3
		System.out.println(BLUE_B + ": " + blueB.getTotalTimeSeconds()); //	#4

		// Intake Motor config here because 0_0
		kIntakeVictor.configFactoryDefault();
		kIntakeVictor.setInverted(true);
	}

	/**
	 * This method is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 * <p>
	 * This runs after the mode specific periodic methods, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		updateSmartDashboard();
		kDrivetrainSub.updateOdometry(); // FOR TESTING PURPOSES
		kLimelight.runLimelight();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different
	 * autonomous modes using the dashboard. The sendable chooser code works with the Java
	 * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the getString line to get the auto name from the text box below the Gyro.
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the switch structure
	 * below with additional strings. If using the SendableChooser make sure to add them to the
	 * chooser code above as well.
	 */
	@Override
	public void autonomousInit() {

		kLimelight.setPipeAuto();

		// store initial auto path start time
		timeMem = getTime();

		// get chosen auto mode from dashboard
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto choices" + autoSelected); // make sure key is consistent across projects

		// defaults variables
		kAutoDelay = 0;
		numOfTrajectories = 0;
		kPortion1 = new AutoPortion();
		kPortion2 = new AutoPortion();
		kPortion3 = new AutoPortion();
		kPortion4 = new AutoPortion();

		// based on which auto mode chosen:
		switch (autoSelected) {
			case SEARCH: // lets the robot choose which path to take
				numOfTrajectories = 1;
				kAutoDelay = .25;
				// galactic search number correlations:
				// 1: redA
				// 2: blueA
				// 3: redB
				// 4: blueB
				switch (chooseSearchPath(limelightCoord.toCoord(
						kLimelight.getTargetX(),
						kLimelight.getTargetY(),
						kLimelight.getArea()
				))) {
					case 1:
						kPortion1.setTrack(redA, true);
						break;
					case 2:
						kPortion1.setTrack(blueA, true);
						break;
					case 3:
						kPortion1.setTrack(redB, true);
						break;
					case 4:
						kPortion1.setTrack(blueB, true);
						break;
					case 0:
					default:
						break;
			}
				break;
			// manual galactic search paths, for testing
			case RED_A:
				numOfTrajectories = 1;
				kPortion1.setTrack(redA, true);
				kAutoDelay = .25;
				break;
			case BLUE_A:
				numOfTrajectories = 1;
				kPortion1.setTrack(blueA, true);
				kAutoDelay = .25;
				break;
			case RED_B:
				numOfTrajectories = 1;
				kPortion1.setTrack(redB, true);
				kAutoDelay = .25;
				break;
			case BLUE_B:
				numOfTrajectories = 1;
				kPortion1.setTrack(blueB, true);
				kAutoDelay = .25;
				break;
			// one of first testing paths
			case FUN:
				numOfTrajectories = 2;
				kPortion1.setTrack(fUN1, true);
				kPortion2.setTrack(fUN2, false);
				break;
			// AutoNav challenge paths
			case SLALOM:
				numOfTrajectories = 1;
				kPortion1.setTrack(slalom, false);
				break;
			case BOUNCE_PATH:
				numOfTrajectories = 4;
				kPortion1.setTrack(bounceA, false);
				kPortion2.setTrack(bounceB, false);
				kPortion3.setTrack(bounceC, false);
				kPortion4.setTrack(bounceD, false);
				break;
			case BARREL_RACE:
				numOfTrajectories = 1;
				kPortion1.setTrack(barrelRace, false);
				break;
			// test autos
			case PATH_TEST:
				numOfTrajectories = 1;
				kPortion1.setTrack(pathWeaverTest, false);
				break;
			case LINEAR_TEST:
				numOfTrajectories = 2;
				kPortion1.setTrack(linearTest, false);
				kPortion2.setTrack(linearTestTwo, false);
				break;
			case CT:
				numOfTrajectories = 2;
				kPortion1.setTrack(CT1, true);
				kPortion2.setTrack(CT2, false);
				kAutoDelay = .25;
				kLimelight.teleOpInit();
			// fallback autos
			case DEFAULT_AUTO:
			default:
				break;
		}

		// set auto start time memory for all paths.
		kAutoStartTime = getTime();


	}

	/** This method is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

//		kDrivetrainSub.updateOdometry();

		kTurretSub.runTurret();

		switch (currentTrajectory) {
			case 0:
				if (kPortion1.doesIntake()) { // doing intake roller beforehand
					// dont worry, it returns false if kPortion1 is empty.
					kIntakeVictor.set(ControlMode.PercentOutput, Constants.kDefaultIntakeFF);
					kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
				} else {
					kIntakeVictor.set(ControlMode.PercentOutput, 0);
					kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
				}

				// starting condition
				if (numOfTrajectories > currentTrajectory && getTimeSince(kAutoStartTime) > kAutoDelay) {
					currentTrajectory++;
					kDrivetrainSub.resetOdometry(kPortion1.getInitPose());
					timeMem = getTime();
				}
				break;
			case 1:
				// if this is an active portion (double checking)
				if (kPortion1.isActive()) {
					// do intake stuff
					if (kPortion1.doesIntake()) {
						kIntakeVictor.set(ControlMode.PercentOutput, Constants.kDefaultIntakeFF);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
					} else {
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
					}
					// do trajectory stuff
					if (getTimeSince(timeMem) < kPortion1.getCalcTime()) { // if still in portion period

						// Get the desired pose from the trajectory.
						var desiredPose = kPortion1.getState(getTimeSince(timeMem));

						// Get the reference chassis speeds from the Ramsete controller.
						var refChassisSpeeds = kRamsete.calculate(kDrivetrainSub.getPose(), desiredPose);

						// Set the linear and angular speeds.
						kDrivetrainSub.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
					} else if (numOfTrajectories > currentTrajectory) { // if still have more tracks to go
						currentTrajectory++;
						kDrivetrainSub.resetOdometry(kPortion2.getInitPose());
						timeMem = getTime();
					} else {
						kDrivetrainSub.drive(0, 0);
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
					}
					break;
				} else if (numOfTrajectories > currentTrajectory) { // empty path runthrough to next
					currentTrajectory++;
				}

			case 2:
				// if this is an active portion (double checking)
				if (kPortion2.isActive()) {
					// do intake stuff
					if (kPortion2.doesIntake()) {
						kIntakeVictor.set(ControlMode.PercentOutput, Constants.kDefaultIntakeFF);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
					} else {
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
					}
					// do trajectory stuff
					if (getTimeSince(timeMem) < kPortion2.getCalcTime()) { // if still in portion period

						// Get the desired pose from the trajectory.
						var desiredPose = kPortion2.getState(getTimeSince(timeMem));

						// Get the reference chassis speeds from the Ramsete controller.
						var refChassisSpeeds = kRamsete.calculate(kDrivetrainSub.getPose(), desiredPose);

						// Set the linear and angular speeds.
						kDrivetrainSub.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);

					} else if (numOfTrajectories > currentTrajectory) { // if still have more tracks to go
						currentTrajectory++;
						kDrivetrainSub.resetOdometry(kPortion3.getInitPose());
						timeMem = getTime();
					} else {
						kDrivetrainSub.drive(0, 0);
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
					}
					break;
				} else if (numOfTrajectories > currentTrajectory) { // empty path runthrough to next
					currentTrajectory++;
				}
			case 3:
				// if this is an active portion (double checking)
				if (kPortion3.isActive()) {
					// do intake stuff
					if (kPortion3.doesIntake()) {
						kIntakeVictor.set(ControlMode.PercentOutput, Constants.kDefaultIntakeFF);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
					} else {
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
					}
					// do trajectory stuff
					if (getTimeSince(timeMem) < kPortion3.getCalcTime()) { // if still in portion period

						// Get the desired pose from the trajectory.
						var desiredPose = kPortion3.getState(getTimeSince(timeMem));

						// Get the reference chassis speeds from the Ramsete controller.
						var refChassisSpeeds = kRamsete.calculate(kDrivetrainSub.getPose(), desiredPose);

						// Set the linear and angular speeds.
						kDrivetrainSub.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);

					} else if (numOfTrajectories > currentTrajectory) { // if still have more tracks to go
						currentTrajectory++;
						kDrivetrainSub.resetOdometry(kPortion4.getInitPose());
						timeMem = getTime();
					} else {
						kDrivetrainSub.drive(0, 0);
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
					}
					break;
				} else if (numOfTrajectories > currentTrajectory) { // empty path runthrough to next
					currentTrajectory++;
				}
			case 4:
				// if this is an active portion (double checking)
				if (kPortion4.isActive()) {
					// do intake stuff
					if (kPortion4.doesIntake()) {
						kIntakeVictor.set(ControlMode.PercentOutput, Constants.kDefaultIntakeFF);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
					} else {
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
					}
					// do trajectory stuff
					if (getTimeSince(timeMem) < kPortion4.getCalcTime()) { // if still in portion period

						// Get the desired pose from the trajectory.
						var desiredPose = kPortion4.getState(getTimeSince(timeMem));

						// Get the reference chassis speeds from the Ramsete controller.
						var refChassisSpeeds = kRamsete.calculate(kDrivetrainSub.getPose(), desiredPose);

						// Set the linear and angular speeds.
						kDrivetrainSub.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);

					} else if (numOfTrajectories > currentTrajectory) { // if still have more tracks to go
						currentTrajectory++;
//							kDrivetrainSub.resetOdometry(kPortion5.getInitPose());
//							kAutoTimeMem = getTime();
					} else {
						kDrivetrainSub.drive(0, 0);
						kIntakeVictor.set(ControlMode.PercentOutput, 0);
						kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
					}
					break;
				} else if (numOfTrajectories > currentTrajectory) { // empty path runthrough to next
					currentTrajectory++;
				}
			case -1:
			default:
				kDrivetrainSub.drive(0, 0);
				kIntakeVictor.set(ControlMode.PercentOutput, 0);
				kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
				break;
		}
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		kDrivetrainSub.resetGyro();
		kDrivetrainSub.resetOdometry(speedBack.getInitialPose());
		timeMem = getTime();
		teleOpPath = 0;
		teleOpPathPosition = 0;

		kTurretSub.enableInit();
		kDigestiveSystem.enableInit();
		kShooterSub.enableInit();

		kLimelight.teleOpInit();
	}

	/** This method is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
//		System.out.println(getTimeSince(timeMem));
//		System.out.println(speedBack.getTotalTimeSeconds());
//		kDrivetrainSub.updateOdometry();

		kDrivetrainSub.runDriveTrain();

		kTurretSub.runTurret();

		kShooterSub.shooterSpin();

		kDigestiveSystem.runDigestive();

		// Intake code here because it's not in digestive
		if (kDriver.getTriggerAxis(GenericHID.Hand.kRight) > kDefaultDeadband) {
			kIntakeVictor.set(
					ControlMode.PercentOutput,
					Constants.kDefaultIntakeFF
			);
		} else {
			kIntakeVictor.set(ControlMode.PercentOutput, 0);
		}

		if (kDriver.getAButtonPressed()) {
			if (kIntakeSolenoid.get() == DoubleSolenoid.Value.kForward) { // im guessing forward is extended
				kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
			} else {
				kIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
			}
		}


	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		kLimelight.setPipeDefault();
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {
//		kIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {}

	/** This method is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	public int chooseSearchPath(limelightCoord lLC) {

		int ret = 0;

		double redAVal = kRedATarget.match(lLC, kBallTargetRange);
		double blueAVal = kBlueATarget.match(lLC, kBallTargetRange);
		double redBVal = kRedBTarget.match(lLC, kBallTargetRange);
		double blueBVal = kBlueBTarget.match(lLC, kBallTargetRange);

		double max = Math.max(
				Math.max(
						redAVal,
						blueAVal
				),
				Math.max(
						redBVal,
						blueBVal
				)
		);

		if (max >= .5) { // this could just be > .5 but I want to be pretty
			if (max == redAVal) {
				ret = 1;
			} else if(max == blueAVal) {
				ret = 2;
			} else if(max == redBVal) {
				ret = 3;
			} else if(max == blueBVal) {
				ret = 4;
			}
		} else {
			ret = 0;
		}
		return ret;
	}

	private void updateSmartDashboard() {

		// high resolution printing:

		// raw gyro

		SmartDashboard.putNumber("Gyro Yaw", kDrivetrainSub.getGyroRotation2d().getDegrees());

		// calculated pose on field

		SmartDashboard.putNumber("Pose X", kDrivetrainSub.getPose().getTranslation().getX());
		SmartDashboard.putNumber("Pose Y", kDrivetrainSub.getPose().getTranslation().getY());
		SmartDashboard.putNumber("Pose Theta", kDrivetrainSub.getPose().getRotation().getDegrees());


		// low resolution printing:

		dashboardCounter++;
		if (dashboardCounter >= 5) { // triggers every 5 calls
			dashboardCounter = 0;

			// encoder raw positions and velocities

			SmartDashboard.putNumber("Left Encoder Pos", kDrivetrainSub.getLeftEncoderPos());
			SmartDashboard.putNumber("Right Encoder Pos", kDrivetrainSub.getRightEncoderPos());

			SmartDashboard.putNumber("Left Encoder Vel", kDrivetrainSub.getLeftEncoderVel());
			SmartDashboard.putNumber("Right Encoder Vel", kDrivetrainSub.getRightEncoderVel());

			// encoder meters/seconds positions and velocities

			SmartDashboard.putNumber("Left Encoder m", kDrivetrainSub.getLeftEncoderMeters());
			SmartDashboard.putNumber("Right Encoder m", kDrivetrainSub.getRightEncoderMeters());

			SmartDashboard.putNumber("Left Encoder mps", kDrivetrainSub.getLeftEncoderMeterRate());
			SmartDashboard.putNumber("Right Encoder mps", kDrivetrainSub.getRightEncoderMeterRate());

			// Limelight stats

			SmartDashboard.putBoolean("Limelight Has Target", kLimelight.hasTarget());
			SmartDashboard.putNumber("Limelight X offset", kLimelight.getTargetX());
			SmartDashboard.putNumber("Limelight Y offset", kLimelight.getTargetY());
			SmartDashboard.putNumber("Limelight Area", kLimelight.getArea());

			// ball configuration prediction for galactic search challenge

			limelightCoord currentSelected = limelightCoord.toCoord(
					kLimelight.getTargetX(),
					kLimelight.getTargetY(),
					kLimelight.getArea()
			);

			SmartDashboard.putNumber(
					"1: Red A confidence",
					kRedATarget.match(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putNumber(
					"2: Blue A confidence",
					kBlueATarget.match(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putNumber(
					"3: Red B confidence",
					kRedBTarget.match(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putNumber(
					"4: Blue B confidence",
					kBlueBTarget.match(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putNumber("Search track chosen", chooseSearchPath(currentSelected));

			// 1: redA
			// 2: blueA
			// 3: redB
			// 4: blueB

			SmartDashboard.putBoolean(
					"1: Red A square match",
					kRedATarget.squareMatch(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putBoolean(
					"2: Blue A square match",
					kBlueATarget.squareMatch(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putBoolean(
					"3: Red B square match",
					kRedBTarget.squareMatch(
							currentSelected,
							kBallTargetRange
					)
			);

			SmartDashboard.putBoolean(
					"4: Blue B square match",
					kBlueBTarget.squareMatch(
							currentSelected,
							kBallTargetRange
					)
			);

			try {
				SmartDashboard.putNumber("Trajectory 1 time", kPortion1.getCalcTime());
				SmartDashboard.putNumber("Trajectory 2 time", kPortion2.getCalcTime());
				SmartDashboard.putNumber("Trajectory 3 time", kPortion3.getCalcTime());
				SmartDashboard.putNumber("Trajectory 4 time", kPortion4.getCalcTime());
				SmartDashboard.putNumber("Total Track time",
						kPortion1.getCalcTime() +
								kPortion2.getCalcTime() +
								kPortion3.getCalcTime() +
								kPortion4.getCalcTime());
			}
			catch(Exception e) {
				//do nothing about any error
			}

			// code from base code

			SmartDashboard.putBoolean(
					"Up To Speed",
					(kShooterSub.getRawSpeed()
							>= kShooterSub.getShooterSetSpeed()* Constants.kUpToSpeedThreshold
							&& kShooterSub.getShooterSetSpeed() != 0
							&& kShooterSub.getShooterSetSpeed() != -1
					)
			);

			SmartDashboard.putNumber("Match Time", getMatchTime());

			SmartDashboard.putNumber("ShooterSpeed", kShooterSub.getRawSpeed());
			SmartDashboard.putNumber("ShooterTarget", kShooterSub.getShooterSetSpeed());

			SmartDashboard.putNumber("TurretPosition", kTurretSub.getRawEncoder());
			SmartDashboard.putNumber("TurretSpeed", kTurretSub.getRawEncoderSped());
		}
	}
}
