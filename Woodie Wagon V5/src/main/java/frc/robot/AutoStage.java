package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import static frc.robot.RobotMath.*;

/**
 * Template lambda for AutoPortion End Conditions
 *
 * TODO: continue previous stage's trajectory in this stage
 * TODO: condition to skip the next stage (or skip to arbitrary stage)
 *
 */
@FunctionalInterface
interface StageCondition {

	boolean apply(double stageStart, double trackTime); // might add more parameters in the future

	// usage:
	// StageCondition (stageStart, trackTime) -> (/*combinator logic*/)
}

public class AutoStage {

	private final StageCondition[] conditions;
	private final StageCondition[] killSwitches;
	private final Trajectory track;
	public final boolean trackContinuation; // TODO This doesn't do anything yet, idk how it will work
	public final boolean intake; // if intake is down, true
	public final boolean digest; // if intake and prime is running, (only works when intake is down)
	public final boolean aim; // If turret is seeking limelight target
	public final double spinUp; // what speed / speed mode shooter is in, -1 for coast, -2 for distance based
	public final boolean shoot; // if feed into shooter head

	private double startTime = -1; // -1 means not active (hasn't started yet or already ended)
	public double trackDuration;

	public AutoStage(
			StageCondition[] _conditions,
			StageCondition[] _killSwitch,
			Trajectory _track,
			boolean _trackContinuation,
			boolean _intake,
			boolean _digest,
			boolean _aim,
			double _spinUp,
			boolean _shoot
	) {


		this.conditions = _conditions;
		this.killSwitches = _killSwitch;
		this.track = _track;
		this.trackContinuation = _trackContinuation;
		this.intake = _intake;
		this.digest = _digest;
		this.aim = _aim;
		this.spinUp = _spinUp;
		this.shoot = _shoot;


		if (isTrackNull()) {
			this.trackDuration = 0;
		} else {
			this.trackDuration = this.track.getTotalTimeSeconds();
		}

	}

	/**
	 * Initializes this stage for use and sends the initial odometry pose.
	 * @return the initial pose passed to update odometry
	 */
//	public Pose2d startStage(double lastStageEndTime) {
	public Pose2d startStage() {
//		this.startTime = getTime();
//		if (isTrackNull()) {
//			return new Pose2d();
//		} else {
//			if (this.trackContinuation) {
//				this.startTime = lastStageEndTime;
//			} else {
//				this.startTime = getTime();
//				return this.track.getInitialPose();
//			}
//		}


		this.startTime = getTime();
		if (isTrackNull()) {
			return new Pose2d();
		} else {
			return this.track.getInitialPose();
		}

	}

	public void resetStage() { // this might not be used
		this.startTime = -1;
	}

	/**
	 *
	 * @return If an ending condition for this stage has been found to be valid
	 */
	public boolean endConditionsCheck() {
		for (StageCondition stageEndCondition : this.conditions) {
			if(stageEndCondition.apply(this.startTime, this.trackDuration)) {
				this.startTime = -1;
			}
		}
		return (this.startTime == -1); // this returns true if this stage is not active
	}

	/**
	 *
	 * @return If an kill switch condition for this stage has been found to be valid
	 */
	public boolean killConditionsCheck() {
		for (StageCondition killSwitchCondition : this.killSwitches) {
			if(killSwitchCondition.apply(this.startTime, this.trackDuration)) {
				this.startTime = -1;
			}
		}
		return (this.startTime == -1); // this returns true if this stage is not active
	}

	/**
	 *
	 * @return the target pose based on the current time
	 */
	public Trajectory.State getTrackPose() {
		if (!isTrackNull()) {
			return this.track.sample(getTime()-this.startTime);
		} else {
			return new Trajectory.State();
		}
	}

	/**
	 *
	 * @return the target pose based on the current time
	 */
	public boolean isTrackNull() {
		return (this.track == null);
	}

}
