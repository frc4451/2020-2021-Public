package frc.robot;

/* =================================================================================================================
 * IMPORTS
 */

// Code
import static frc.robot.Constants.*;
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.Subsystems.kDriveTrain;
import static frc.robot.RobotContainer.Trajectories.*;
import static frc.robot.RobotMath.*;

import frc.robot.subsystems.*;

// Phoenix
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

// WPI General
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

// WPI IO
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

// WPI Internal
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;


/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {


	/* =================================================================================================================
	 * CONTAINER METHODS AND STUFF
	 */

	public RobotContainer() {}

	// Path get method used to generate paths from pathweaver json files
	private static Trajectory getPathWeaverTrack(String name) {
		String trajectoryJSON = "paths/output/" + name;
		Trajectory trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
			// this usually means that the path does not exist.
			debug("Unable to load pathweaver file! \n Path: " + trajectoryJSON, "!");
		}
		return(trajectory);
	}


	/* =================================================================================================================
	 * EXTERNAL DEVICES AND IO REPRESENTATION SECTION
	 */

	private static class Externals {

		// Controllers
		private static final XboxController kDriver = new XboxController(0);
		private static final XboxController kOperator = new XboxController(1);

		/*
		 * Falcon Motors (TalonFX)
		 */

		// Drivetrain
		private static final TalonFX kLeftBackDT = new TalonFX(IDs.FALCON.LEFT_BACK_DRIVETRAIN);
		private static final TalonFX kRightBackDT = new TalonFX(IDs.FALCON.RIGHT_BACK_DRIVETRAIN);
		private static final TalonFX kLeftFrontDT = new TalonFX(IDs.FALCON.LEFT_FRONT_DRIVETRAIN);
		private static final TalonFX kRightFrontDT = new TalonFX(IDs.FALCON.RIGHT_FRONT_DRIVETRAIN);


		// Turret and Shooter
		private static final TalonFX kShooterFlyWheel = new TalonFX(IDs.FALCON.SHOOTER);

		// Climb and others
		private static final TalonFX kClimberMotor = new TalonFX(IDs.FALCON.CLIMBER); // we took the climber off lol

		/*
		 * Talon Motors (TalonSRX)
		 */

		// Digestive System Tower Belt
		private static final TalonSRX kTowerBelt = new TalonSRX(IDs.TALON.TOWER_BELT);

		// Turret turning
		private static final TalonSRX kTurretMotor = new TalonSRX(IDs.TALON.TURRET_YAW);

		// Digestive system 'Egg Beater' Agitator
		private static final TalonSRX kEggBeater = new TalonSRX(IDs.TALON.EGG_BEATER);

		/*
		 * Victor Motors (VictorSPX)
		 */

		private static final VictorSPX kAgitatorVictor = new VictorSPX(IDs.VICTOR.AGITATOR);

		private static final VictorSPX kIntakeVictor = new VictorSPX(IDs.VICTOR.INTAKE);

		private static final VictorSPX kColorWheelVictor = new VictorSPX(IDs.VICTOR.COLOR_WHEEL);

		/*
		 * Other Objects: Gyro, Pneumatics, and sensors
		 */

		//Pigeon IMU Gyro
		private static final PigeonIMU kGyro = new PigeonIMU(IDs.GYRO_PIGEON);

		//Pneumatics

		private static final DoubleSolenoid kIntakeSolenoid = new DoubleSolenoid(
				IDs.INTAKE_SOLENOID_A,
				IDs.INTAKE_SOLENOID_B
		);

		private static final Solenoid kClimbSolenoid = new Solenoid(IDs.CLIMBER_SOLENOID);

		private static final Solenoid StatusLight = new Solenoid(IDs.STATUS_LIGHT);

		// DIO Ports

		// 0 was used for a LIDAR, but I don't think it's installed

		// Tower beam breaks
		private static final DigitalInput kTopBeamBreak = new DigitalInput(IDs.TOWER_BEAM_BREAK_TOP);
		private static final DigitalInput kBottomBeamBreak = new DigitalInput(IDs.TOWER_BEAM_BREAK_BOTTOM);

	}


	/* =================================================================================================================
	 * INTERNAL CALCULATOR OBJECTS
	 */

	private static class Internals {

		//Ramsete Path Following

		private static final DifferentialDriveOdometry kOdometry = new DifferentialDriveOdometry(
				Rotation2d.fromDegrees(0)
		);
		private static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(
				TechnicalConstants.kDriveTrainWidth
		);

		// Original/default parameter list: (2, 0.7)
		private static final RamseteController kRamsete = new RamseteController(2, 1);

	}


	/* =================================================================================================================
	 * SUBSYSTEMS
	 */

	public static class Subsystems {

		public static final Climber kClimber = new Climber(Externals.kClimberMotor, Externals.kClimbSolenoid);
		public static final ColorWheel kColorWheel = new ColorWheel(Externals.kColorWheelVictor);
		public static final Digestive kDigestiveSystem = new Digestive(
				Externals.kIntakeVictor,
				Externals.kIntakeSolenoid,
				Externals.kAgitatorVictor,
				Externals.kTowerBelt,
				Externals.kEggBeater,
				Externals.kTopBeamBreak,
				Externals.kBottomBeamBreak
		);
		public static final DriveTrain kDriveTrain = new DriveTrain(
				Externals.kLeftBackDT,
				Externals.kRightBackDT,
				Externals.kLeftFrontDT,
				Externals.kRightFrontDT,
				Externals.kGyro,
				Internals.kOdometry,
				Internals.kKinematics
		);
		public static final LimeLight kLimeLight = new LimeLight(IDs.LIMELIGHT_NT_KEY, Externals.StatusLight);
		public static final Shooter kShooterHead = new Shooter(Externals.kShooterFlyWheel, Externals.kTurretMotor);

	}

	/* =================================================================================================================
	 * PATHWEAVER TRAJECTORIES, CONFIG OBJECTS, AND AUTO MODES
	 */

	// config objects

	private static class PathConfigs {

		// this says Velocity 1 m/s, Acceleration 1 m/s
		private static final TrajectoryConfig kConfigV1A1 = new TrajectoryConfig(
				1,
				1
		);

		private static final TrajectoryConfig kConfigV2A1Test = new TrajectoryConfig(
				2,
				1
		).addConstraint(new CentripetalAccelerationConstraint(1.5));

		private static final TrajectoryConfig kConfigV1p5A1 = new TrajectoryConfig(
				1.5,
				1
		);

		private static final TrajectoryConfig kConfigV1A1R = new TrajectoryConfig(
				1,
				1
		).setReversed(true);

		private static final TrajectoryConfig kConfigV2p5A2p5 = new TrajectoryConfig(
				2.75,
				2.5
		);

		private static final TrajectoryConfig kConfigV2p5A2p5R = new TrajectoryConfig(
				2.75,
				2.5
		).setReversed(true);

	}

	// actual trajectories

	public static class Trajectories {

		// test tracks

		public static final Trajectory pathWeaverTest = getPathWeaverTrack("test.wpilib.json");
		public static final Trajectory linearTest = getPathWeaverTrack("linearTest.wpilib.json");
		public static final Trajectory linearTestTwo = getPathWeaverTrack("linearTest2.wpilib.json");

		// Competition Tracks

		public static final Trajectory CT1 = getPathWeaverTrack("CTAutoA1.wpilib.json");
		public static final Trajectory CT2 = getPathWeaverTrack("CTAutoB2.wpilib.json");

		public static final Trajectory CRT1 = getPathWeaverTrack("CRTAutoA1.wpilib.json");
		public static final Trajectory CRT2 = getPathWeaverTrack("CRTAutoA2.wpilib.json");

		public static final Trajectory CC1 = getPathWeaverTrack("CCAutoA1.wpilib.json");
		public static final Trajectory CC2 = getPathWeaverTrack("CCAutoB2.wpilib.json");


		// fUN trajectories (more test tracks) hardcoded

		public static final Trajectory fUN1 = TrajectoryGenerator.generateTrajectory(
				List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
						new Pose2d(2, 1.5, Rotation2d.fromDegrees(180)),
						new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(180))
				),
				PathConfigs.kConfigV1A1R
		);

		public static final Trajectory fUN2 = TrajectoryGenerator.generateTrajectory(
				List.of(new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(180)),
						new Pose2d(0, 0, Rotation2d.fromDegrees(180))
				),
				PathConfigs.kConfigV1A1
		);

	}

	/* =================================================================================================================
	 * AUTO MODE CONDITIONS,  ROUTINES
	 */

	private static class Conditions {

		/* examples:
		* private static final Predicate<Object> falsePredicate = o -> false;
		* private static final Predicate<String> stringPredicate = example -> false;
		* private static final BooleanSupplier defaultSupplier = () -> false;
		 */

		private static StageCondition timeElapsed(double seconds) {
			return (portionStart, trackTime) -> (getTimeSince(portionStart) >= seconds);
		}

		private static final StageCondition trackDone =
				(portionStart, trackTime) -> (getTimeSince(portionStart) >= trackTime);

	}

	public static class AutoRoutines {

		public static AutoRoutine blankRoutine = new AutoRoutine(
				false,
				new AutoStage[] {
						new AutoStage(
								// next portion condition
								new StageCondition[]{},
								// kill condition
								new StageCondition[]{},
								null,
								false,
								false,
								false,
								false,
								-1,
								false
						)
				}
		);

		public static AutoRoutine pathWeaverTestRoutine = new AutoRoutine(
				false,
				new AutoStage[] {
						new AutoStage(
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(5),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(6)
								},
								pathWeaverTest,
								false,
								false,
								false,
								false,
								-1,
								false
						)
				}
		);

		public static AutoRoutine centerToTrench = new AutoRoutine(
				true,
				new AutoStage[] {
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(2)
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(3)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(1)
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(2)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								true
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(8),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(9)
								},
								CT1,
								false,
								true,
								true,
								true,
								-1,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(8),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(9)
								},
								CT2,
								false,
								true,
								true,
								true,
								8750,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(2)
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(3)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								true
						)
				}
		);

		public static AutoRoutine centerToTrenchBackLine = new AutoRoutine(
				true,
				new AutoStage[] {
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(2)
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(3)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(1)
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(2)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								true
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(8),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(9)
								},
								CT1,
								false,
								true,
								true,
								true,
								-1,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(8),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(9)
								},
								CT2,
								false,
								true,
								true,
								true,
								8750,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(2)
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(3)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								true
						)
				}
		);

		public static AutoRoutine centerToRendezvousTexas = new AutoRoutine(
				true,
				new AutoStage[] {
						new AutoStage( // spin up initially
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(2)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(3)
								},
								null,
								false,
								false,
								false,
								true,
								8750,
								false
						),
						new AutoStage( // shoot for 1 second
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(1.25)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(2.25)
								},
								null,
								false,
								false,
								false,
								true,
								8750,
								true
						),
						new AutoStage( // go to switch
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(5),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(6)
								},
								CRT1,
								false,
								false,
								false,
								false,
								-1,
								false
						),
						new AutoStage( // pick up balls under switch and spin up going back to sweet spot
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(7),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(8)
								},
								CRT2,
								false,
								true,
								true,
								true,
								8750,
								false
						),
						new AutoStage( // shoot
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(2)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(3)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								true
						)
				}
		);

		public static AutoRoutine crossCourt = new AutoRoutine(
				true,
				new AutoStage[] {
						new AutoStage(
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(.25)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(1)
								},
								null,
								false,
								true,
								true,
								false,
								-1,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[]{
										Conditions.trackDone,
										Conditions.timeElapsed(5)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(6)
								},
								CC1,
								false,
								true,
								true,
								false,
								-1,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[]{
										Conditions.trackDone,
										Conditions.timeElapsed(8)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(9)
								},
								CC2,
								false,
								true,
								true,
								true,
								8750,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[]{
										Conditions.timeElapsed(3.5)
								},
								// kill condition
								new StageCondition[]{
										Conditions.timeElapsed(4)
								},
								null,
								false,
								true,
								false,
								true,
								8750,
								true
						)
				}
		);

		public static AutoRoutine routineTest = new AutoRoutine(
				false,
				new AutoStage[] {
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(5),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(6)
								},
								pathWeaverTest,
								false,
								false,
								false,
								true,
								1000,
								false
						),
						new AutoStage(
								// next portion condition
								new StageCondition[] {
										Conditions.timeElapsed(5),
										Conditions.trackDone
								},
								// kill condition
								new StageCondition[] {
										Conditions.timeElapsed(1)
										// this should make it stop 1 second after the second track starts
								},
								linearTest,
								false,
								false,
								false,
								true,
								2000,
								false
						)
				}
		);
	}

	/* =================================================================================================================
	 * IO AND INTERNAL OBJECT WRAPPER FUNCTIONS
	 */

	// wrapper for ramsete calculating current target chassis speeds
	public static ChassisSpeeds ramseteCalculate(Trajectory.State desiredPose) {
		return Internals.kRamsete.calculate(kDriveTrain.getPose(), desiredPose);
	}

	/**
	 * This encapsulates get methods so that files can selectively import inputs.
	 * It may seem needless, but this minimizes error (increases my sanity),
	 * as well as adding a few more input parameters such as rising and falling state up down left right inputs.
	 * Link to spreadsheet that extensively explains button relations:
	 * https://docs.google.com/spreadsheets/d/1eLlkiB1O2XaipF4Awx3zomSG7vtXs9CGkAbSruTtWPc/edit?usp=sharing
	 */
	public static class IO {

		// Derived Variables



		// Pressed means rising state, Released means falling state

		public static class Driver {

			public static double leftX() { return(Externals.kDriver.getX(GenericHID.Hand.kLeft)); }
			public static double leftY() { return(Externals.kDriver.getY(GenericHID.Hand.kLeft)); }
			public static boolean left() { return(Externals.kDriver.getStickButton(GenericHID.Hand.kLeft)); }
			public static boolean leftPressed() { return(Externals.kDriver.getStickButtonPressed(GenericHID.Hand.kLeft)); }
			public static boolean leftReleased() { return(Externals.kDriver.getStickButtonReleased(GenericHID.Hand.kLeft)); }
			public static double rightX() { return(Externals.kDriver.getX(GenericHID.Hand.kRight)); }
			public static double rightY() { return(Externals.kDriver.getY(GenericHID.Hand.kRight)); }
			public static boolean right() { return(Externals.kDriver.getStickButton(GenericHID.Hand.kRight)); }
			public static boolean rightPressed() { return(Externals.kDriver.getStickButtonPressed(GenericHID.Hand.kRight)); }
			public static boolean rightReleased() { return(Externals.kDriver.getStickButtonReleased(GenericHID.Hand.kRight)); }
			public static int pov() { return(Externals.kDriver.getPOV()); }// -1 means nothing pressed
			private static boolean u = false; // initial state
			private static boolean d = false; // initial state
			private static boolean l = false; // initial state
			private static boolean r = false; // initial state
			public static boolean u() { return(u); } // North
			public static boolean d() { return(d); } // South
			public static boolean l() { return(l); } // West
			public static boolean r() { return(r); } // East
			private static boolean uPrev = false; // initial state
			private static boolean dPrev = false; // initial state
			private static boolean lPrev = false; // initial state
			private static boolean rPrev = false; // initial state
			public static boolean uPressed() { return(!uPrev && u); }
			public static boolean dPressed() { return(!dPrev && d); }
			public static boolean lPressed() { return(!lPrev && l); }
			public static boolean rPressed() { return(!rPrev && r); }
			public static boolean uReleased() { return(uPrev && !u); }
			public static boolean dReleased() { return(dPrev && !d); }
			public static boolean lReleased() { return(lPrev && !l); }
			public static boolean rReleased() { return(rPrev && !r); }
			public static boolean a() { return(Externals.kDriver.getAButton()); }
			public static boolean b() { return(Externals.kDriver.getBButton()); }
			public static boolean x() { return(Externals.kDriver.getXButton()); }
			public static boolean y() { return(Externals.kDriver.getYButton()); }
			public static boolean aPressed() { return(Externals.kDriver.getAButtonPressed()); }
			public static boolean bPressed() { return(Externals.kDriver.getBButtonPressed()); }
			public static boolean xPressed() { return(Externals.kDriver.getXButtonPressed()); }
			public static boolean yPressed() { return(Externals.kDriver.getYButtonPressed()); }
			public static boolean aReleased() { return(Externals.kDriver.getAButton()); }
			public static boolean bReleased() { return(Externals.kDriver.getBButton()); }
			public static boolean xReleased() { return(Externals.kDriver.getXButton()); }
			public static boolean yReleased() { return(Externals.kDriver.getYButton()); }
			public static double lTrig() { return(Externals.kDriver.getTriggerAxis(GenericHID.Hand.kLeft)); }
			public static double rTrig() { return(Externals.kDriver.getTriggerAxis(GenericHID.Hand.kRight)); }
			public static boolean lBump() { return(Externals.kDriver.getBumper(GenericHID.Hand.kLeft)); }
			public static boolean lBumpPressed() { return(Externals.kDriver.getBumperPressed(GenericHID.Hand.kLeft)); }
			public static boolean lBumpReleased() { return(Externals.kDriver.getBumperReleased(GenericHID.Hand.kLeft)); }
			public static boolean rBump() { return(Externals.kDriver.getBumper(GenericHID.Hand.kRight)); }
			public static boolean rBumpPressed() { return(Externals.kDriver.getBumperPressed(GenericHID.Hand.kRight)); }
			public static boolean rBumpReleased() { return(Externals.kDriver.getBumperReleased(GenericHID.Hand.kRight)); }
			public static boolean lCenter() { return(Externals.kDriver.getBackButton()); }
			public static boolean lCenterPressed() { return(Externals.kDriver.getBackButtonPressed()); }
			public static boolean lCenterReleased() { return(Externals.kDriver.getBackButtonReleased()); }
			public static boolean rCenter() { return(Externals.kDriver.getStartButton()); }
			public static boolean rCenterPressed() { return(Externals.kDriver.getStartButtonPressed()); }
			public static boolean rCenterReleased() { return(Externals.kDriver.getStartButtonReleased()); }

		}

		public static class Operator {

			public static double leftX() { return(Externals.kOperator.getX(GenericHID.Hand.kLeft)); }
			public static double leftY() { return(Externals.kOperator.getY(GenericHID.Hand.kLeft)); }
			public static boolean left() { return(Externals.kOperator.getStickButton(GenericHID.Hand.kLeft)); }
			public static boolean leftPressed() { return(Externals.kOperator.getStickButtonPressed(GenericHID.Hand.kLeft)); }
			public static boolean leftReleased() { return(Externals.kOperator.getStickButtonReleased(GenericHID.Hand.kLeft)); }
			public static double rightX() { return(Externals.kOperator.getX(GenericHID.Hand.kRight)); }
			public static double rightY() { return(Externals.kOperator.getY(GenericHID.Hand.kRight)); }
			public static boolean right() { return(Externals.kOperator.getStickButton(GenericHID.Hand.kRight)); }
			public static boolean rightPressed() { return(Externals.kOperator.getStickButtonPressed(GenericHID.Hand.kRight)); }
			public static boolean rightReleased() { return(Externals.kOperator.getStickButtonReleased(GenericHID.Hand.kRight)); }
			public static int pov() { return(Externals.kOperator.getPOV()); }// -1 means nothing pressed
			private static boolean u = false; // initial state
			private static boolean d = false; // initial state
			private static boolean l = false; // initial state
			private static boolean r = false; // initial state
			public static boolean u() { return(u); } // North
			public static boolean d() { return(d); } // South
			public static boolean l() { return(l); } // West
			public static boolean r() { return(r); } // East
			private static boolean uPrev = false; // initial state
			private static boolean dPrev = false; // initial state
			private static boolean lPrev = false; // initial state
			private static boolean rPrev = false; // initial state
			public static boolean uPressed() { return(!uPrev && u); }
			public static boolean dPressed() { return(!dPrev && d); }
			public static boolean lPressed() { return(!lPrev && l); }
			public static boolean rPressed() { return(!rPrev && r); }
			public static boolean uReleased() { return(uPrev && !u); }
			public static boolean dReleased() { return(dPrev && !d); }
			public static boolean lReleased() { return(lPrev && !l); }
			public static boolean rReleased() { return(rPrev && !r); }
			public static boolean a() { return(Externals.kOperator.getAButton()); }
			public static boolean b() { return(Externals.kOperator.getBButton()); }
			public static boolean x() { return(Externals.kOperator.getXButton()); }
			public static boolean y() { return(Externals.kOperator.getYButton()); }
			public static boolean aPressed() { return(Externals.kOperator.getAButtonPressed()); }
			public static boolean bPressed() { return(Externals.kOperator.getBButtonPressed()); }
			public static boolean xPressed() { return(Externals.kOperator.getXButtonPressed()); }
			public static boolean yPressed() { return(Externals.kOperator.getYButtonPressed()); }
			public static boolean aReleased() { return(Externals.kOperator.getAButton()); }
			public static boolean bReleased() { return(Externals.kOperator.getBButton()); }
			public static boolean xReleased() { return(Externals.kOperator.getXButton()); }
			public static boolean yReleased() { return(Externals.kOperator.getYButton()); }
			public static double lTrig() { return(Externals.kOperator.getTriggerAxis(GenericHID.Hand.kLeft)); }
			public static double rTrig() { return(Externals.kOperator.getTriggerAxis(GenericHID.Hand.kRight)); }
			public static boolean lBump() { return(Externals.kOperator.getBumper(GenericHID.Hand.kLeft)); }
			public static boolean lBumpPressed() { return(Externals.kOperator.getBumperPressed(GenericHID.Hand.kLeft)); }
			public static boolean lBumpReleased() { return(Externals.kOperator.getBumperReleased(GenericHID.Hand.kLeft)); }
			public static boolean rBump() { return(Externals.kOperator.getBumper(GenericHID.Hand.kRight)); }
			public static boolean rBumpPressed() { return(Externals.kOperator.getBumperPressed(GenericHID.Hand.kRight)); }
			public static boolean rBumpReleased() { return(Externals.kOperator.getBumperReleased(GenericHID.Hand.kRight)); }
			public static boolean lCenter() { return(Externals.kOperator.getBackButton()); }
			public static boolean lCenterPressed() { return(Externals.kOperator.getBackButtonPressed()); }
			public static boolean lCenterReleased() { return(Externals.kOperator.getBackButtonReleased()); }
			public static boolean rCenter() { return(Externals.kOperator.getStartButton()); }
			public static boolean rCenterPressed() { return(Externals.kOperator.getStartButtonPressed()); }
			public static boolean rCenterReleased() { return(Externals.kOperator.getStartButtonReleased()); }

		}

		/**
		 * IMPORTNAT! CALL IN CODE EVERY ITERATION
		 */
		public static void updateIO() {
			// updates state change booleans for UDLR pad

			Driver.u = (Driver.pov() == 0);
			Driver.d = (Driver.pov() == 180);
			Driver.l = (Driver.pov() == 270);
			Driver.r = (Driver.pov() == 90);

			// these could call the methods instead of the variables, but this is more direct
			Driver.uPrev = Driver.u;
			Driver.dPrev = Driver.d;
			Driver.lPrev = Driver.l;
			Driver.rPrev = Driver.r;

			Operator.u = (Operator.pov() == 0);
			Operator.d = (Operator.pov() == 180);
			Operator.l = (Operator.pov() == 270);
			Operator.r = (Operator.pov() == 90);

			// these could call the methods instead of the variables, but this is more direct
			Operator.uPrev = Operator.u;
			Operator.dPrev = Operator.d;
			Operator.lPrev = Operator.l;
			Operator.rPrev = Operator.r;

		}
	}

}
