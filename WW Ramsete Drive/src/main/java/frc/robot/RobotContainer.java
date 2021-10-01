package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.subsystems.*;

import frc.robot.RobotMath.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {

	// Controllers

	public static final XboxController kDriver = new XboxController(0);
	public static final XboxController kOperator = new XboxController(1);

	// Motors, Solenoids, and Gyro, and other sensors

	private static final TalonFX kFalconLF = new TalonFX(RobotMath.IDs.kDrivetrainLeftFrontF);
	private static final TalonFX kFalconLB = new TalonFX(RobotMath.IDs.kDrivetrainLeftBackF);
	private static final TalonFX kFalconRF = new TalonFX(RobotMath.IDs.kDrivetrainRightFrontF);
	private static final TalonFX kFalconRB = new TalonFX(RobotMath.IDs.kDrivetrainRightBackF);

	private static final PigeonIMU kGyro = new PigeonIMU(RobotMath.IDs.kGyroP);

	public static final VictorSPX kIntakeVictor = new VictorSPX(RobotMath.IDs.kIntakeV);

	public static final DoubleSolenoid kIntakeSolenoid = new DoubleSolenoid(
			RobotMath.IDs.kIntakeS0,
			RobotMath.IDs.kIntakeS1
	);

	private static final Solenoid kJackSolenoid = new Solenoid(RobotMath.IDs.kJackS);

	private static final Solenoid StatusLight = new Solenoid(RobotMath.IDs.kStatusL);

	public static final Limelight kLimelight = new Limelight(StatusLight);

	// objects from base code

	private static final TalonFX kShooterFalcon = new TalonFX(
			IDs.SHOOTER_FALCON
	);

	private static final TalonSRX kTurretTalon = new TalonSRX(IDs.TURRET_TALON);

	// Digestive System and Intake

	private static final TalonSRX kTowerTalon = new TalonSRX(IDs.TOWER_TALON);

	private static final TalonSRX kEggBeaterTalon = new TalonSRX(IDs.EGG_TALON);

	private static final VictorSPX kAgitatorVictor = new VictorSPX(
			IDs.AGITATOR_VICTOR
	);

	public static final DigitalInput kTopBeamBreak = new DigitalInput(
			IDs.TOWER_BEAM_BREAK_A_DIO
	);

	public static final DigitalInput kBottomBeamBreak = new DigitalInput(
			IDs.TOWER_BEAM_BREAK_B_DIO
	);

	// subsystems from base code

	public static final ShooterSub kShooterSub = new ShooterSub(kShooterFalcon);

	public static final TurretSub kTurretSub = new TurretSub(kTurretTalon);

	public static final DigestiveSub kDigestiveSystem = new DigestiveSub(kAgitatorVictor, kTowerTalon, kEggBeaterTalon);

	// Ramsete Stuff

	private static final DifferentialDriveOdometry kOdometry = new DifferentialDriveOdometry(
			Rotation2d.fromDegrees(0)
	);

	private static final DifferentialDriveKinematics kKinematics =
			new DifferentialDriveKinematics(RobotMath.Constants.kDriveTrainWidth);

	public static final RamseteController kRamsete = new RamseteController();

	// trajectories for the robot to follow

	// configs

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

	// actual trajectories

	public static final Trajectory fUN1 = TrajectoryGenerator.generateTrajectory(
			List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
					new Pose2d(2, 1.5, Rotation2d.fromDegrees(180)),
					new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(180))
			),
			kConfigV1A1R
	);

	public static final Trajectory fUN2 = TrajectoryGenerator.generateTrajectory(
			List.of(new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(180)),
					new Pose2d(0, 0, Rotation2d.fromDegrees(180))
			),
			kConfigV1A1
	);

	public static final Trajectory pathWeaverTest = pathWeaver("test");
	public static final Trajectory linearTest = pathWeaver("linearTest");
	public static final Trajectory linearTestTwo = pathWeaver("linearPath2");
	// it says Path instead of Test, just keep it

	public static final Trajectory uTurn = pathWeaver("uTurn");

	public static final Trajectory slalom = pathWeaver("slalom");

	public static final Trajectory barrelRace = pathWeaver("barrelRace");

	// bounce path
	public static final Trajectory bounceA = pathWeaver("bounceA");
	public static final Trajectory bounceB = pathWeaver("bounceB");
	public static final Trajectory bounceC = pathWeaver("bounceC");
	public static final Trajectory bounceD = pathWeaver("bounceD");

	// galaxy search paths
	public static final Trajectory redA = pathWeaver("redA");
	public static final Trajectory blueA = pathWeaver("blueA");
	public static final Trajectory redB = pathWeaver("redB");
	public static final Trajectory blueB = pathWeaver("blueB");

	public static final Trajectory CT1 = pathWeaver("CTAutoA1");
	public static final Trajectory CT2 = pathWeaver("CTAutoB2");


	// speed challenge paths

	public static final Trajectory speedBack = TrajectoryGenerator.generateTrajectory(
			List.of(new Pose2d(-.1143/2., 0, Rotation2d.fromDegrees(180)),
					new Pose2d(3.8, -.0508*3./4., Rotation2d.fromDegrees(180))
			),
			kConfigV2p5A2p5R
//			List.of(new Pose2d(-.1, 0, Rotation2d.fromDegrees(180)),
//					new Pose2d(4.5, -.08, Rotation2d.fromDegrees(180))
//			),
//			kConfigV2A2R
	);

	public static final Trajectory speedForward = TrajectoryGenerator.generateTrajectory(
			List.of(new Pose2d(3.8, 0, Rotation2d.fromDegrees(180)),
					new Pose2d(.1143/4., -.0508*3./4., Rotation2d.fromDegrees(180))
//					new Pose2d(.1143/2.-.0508, -.0508*3./4., Rotation2d.fromDegrees(180))
			),
			kConfigV2p5A2p5
//			List.of(new Pose2d(4.5, 0, Rotation2d.fromDegrees(180)),
//					new Pose2d(.1, -.08, Rotation2d.fromDegrees(180))
//			),
//			kConfigV2A2
	);


	// Drivetrain Subsystem

	public static final Drivetrain kDrivetrainSub = new Drivetrain(
			kFalconLF,
			kFalconLB,
			kFalconRF,
			kFalconRB,
			kGyro,
			kOdometry,
			kKinematics
	);


	// TODO: All the rest of the ramsete objects and setup!

	// used to generate paths from pathweaver json files
	private static Trajectory pathWeaver(String name) {
		String trajectoryJSON = "paths/output/" + name + ".wpilib.json";
		Trajectory trajectory = new Trajectory();
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
			// this usually means that the path does not exist.
		}
		return(trajectory);
	}

//	public RobotContainer() {
//	}

//	public Command getAutonomousCommand() {
//		// An ExampleCommand will run in autonomous
//		return null;
//	}

}
