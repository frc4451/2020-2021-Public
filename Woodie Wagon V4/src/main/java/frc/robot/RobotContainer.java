
package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.robot.subsystems.*;
import frc.robot.Constants.IDs;
import frc.robot.Constants.MotorConstants;
import frc.robot.sensors.LimelightSub;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Debug.*;

/******************************************************************************
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...

	/** IO */

	public static final XboxController kDriver = new XboxController(0);
	public static final XboxController kOperator = new XboxController(1);
	/***************************************************************************
	 * MOTORS 
	**/

	// Drivetrain

	private static final TalonFX kFalconLF = new TalonFX(IDs.LEFT_FRONT_FALCON);
	private static final TalonFX kFalconLB = new TalonFX(IDs.LEFT_BACK_FALCON);
	private static final TalonFX kFalconRF = new TalonFX(IDs.RIGHT_FRONT_FALCON);
	private static final TalonFX kFalconRB = new TalonFX(IDs.RIGHT_BACK_FALCON);

	// Turret and Shooter

	private static final TalonFX kShooterFalcon = new TalonFX(
		IDs.SHOOTER_FALCON
	);

	private static final TalonSRX kTurretTalon = new TalonSRX(IDs.TURRET_TALON);

	// Digestive System and Intake

	private static final TalonSRX kTowerTalon = new TalonSRX(IDs.TOWER_TALON);

	private static final VictorSPX kAgitatorVictor = new VictorSPX(
		IDs.AGITATOR_VICTOR
	);

	private static final VictorSPX kIntakeVictor = new VictorSPX(
		IDs.INTAKE_VICTOR
	);

	// gyro

	public static final PigeonIMU kGyro = new PigeonIMU(IDs.GYRO_PIGEON);

	//Sensors

	public static final DigitalInput kTopBeamBreak = new DigitalInput(
			IDs.TOWER_BEAM_BREAK_A_DIO
	);

	public static final DigitalInput kBottomBeamBreak = new DigitalInput(
			IDs.TOWER_BEAM_BREAK_B_DIO
	);

	// Solenoids

	public static final DoubleSolenoid kIntakeSolenoid = new DoubleSolenoid(
		IDs.INTAKE_SOLENOID_1,
		IDs.INTAKE_SOLENOID_2
	);

	// Climb and others

	private static final TalonFX kClimbFalcon = new TalonFX(IDs.CLIMBER_FALCON);

//	private static final Solenoid kClimbSolenoid = new Solenoid(
//		IDs.CLIMBER_SOLENOID
//	); // old

	private static final Solenoid kClimbSolenoid = new Solenoid(
			IDs.CLIMBER_SOLENOID
	); // new

//	private static final TalonSRX kWheelTalon = new TalonSRX(IDs.WHEEL_TALON);

	private static final TalonSRX kEggBeaterTalon = new TalonSRX(IDs.EGG_TALON);

	/** SUBSYSTEMS */

	public static final DriveTrainSub kDriveTrain = new DriveTrainSub(kFalconLF, kFalconLB, kFalconRF, kFalconRB);
	
	public static final ShooterSub kShooterSub = new ShooterSub(kShooterFalcon);

	public static final TurretSub kTurretSub = new TurretSub(kTurretTalon);

	public static final DigestiveSub kDigestiveSystem = new DigestiveSub(kAgitatorVictor, kTowerTalon, kEggBeaterTalon);

	public static final IntakeSub kIntakeSub = new IntakeSub(kIntakeVictor, kIntakeSolenoid);

	public static final ClimberSub kClimbSub = new ClimberSub(kClimbFalcon, kClimbSolenoid);

//	public static final WheelSub kWheelSub = new WheelSub(kWheelTalon);

	public static final LimelightSub kLimeLight = new LimelightSub();

	// Ramsete Experimental stuff

//	private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(15.664);
//	// NO idea why drivetrain width is so big. We are doing a ChiefDelphi post on it

	public static final DifferentialDriveOdometry kOdometry = new DifferentialDriveOdometry(
			Rotation2d.fromDegrees(0)
	);


	
	/**
	 * The container for the robot. Contains subsystems,
	 * OI devices, and commands.
	 */
	public RobotContainer() {
		debug("RobotContainer Startup: "+this.toString());
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings.
	 * Buttons can be created by instantiating a {@link GenericHID} or one of
	 * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
	 * {@link XboxController}), and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}


	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return null;
	}

	/** Custom Methods */

	// ramsete controller stuff

	public static double getLeftEncoderMeters() {
		return kDriveTrain.getLeftEncoderPos() * MotorConstants.kMetersPerTick;
	}

	public static double getRightEncoderMeters() {
		return kDriveTrain.getRightEncoderPos() * MotorConstants.kMetersPerTick;
	}

	public static boolean isShooterUpToSpeed() {
		return (kShooterSub.getRawSpeed()
				>= kShooterSub.getShooterSetSpeed()* Constants.Config.kUpToSpeedThreshold
				&& kShooterSub.getShooterSetSpeed() != 0
				&& kShooterSub.getShooterSetSpeed() != -1
		);
	}
}
