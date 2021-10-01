/*
 * Experimental class for different portions of autos, to make things easier.
 */
package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class AutoPortion {

	// limit of 6 trajectories rn
	public Trajectory Trajectory;
	private boolean runIntake;

	public AutoPortion() {
		this.Trajectory = null;


	}

	public void setTrack(Trajectory _Trajectory, boolean _runIntake) {
		this.Trajectory = _Trajectory;
		this.runIntake = _runIntake;
	}

	public boolean isActive() {
		return this.Trajectory != null;
	}

	public boolean doesIntake() {
		if (!isActive()) {
			return false;
		} else{
			return runIntake;
		}
	}

	// just some wrappers

	public edu.wpi.first.wpilibj.trajectory.Trajectory.State getState(double time) {
		return this.Trajectory.sample(time);
	}

	public double getCalcTime() {
		return this.Trajectory.getTotalTimeSeconds();
	}

	public Pose2d getInitPose() {
		if (isActive()) {
			return this.Trajectory.getInitialPose();
		} else {
			return new Pose2d();
		}
	}

}

