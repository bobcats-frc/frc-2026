package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/** A generic class holding static constants. */
public final class Constants {
	// Not the real starting pose, actual starting pose is obtained via PathPlanner
	public static final Pose2d kStartingPose = new Pose2d(3, 2, Rotation2d.kZero);

	// Standard 50 Hz
	public static final double kLoopPeriodSeconds = 1.0 / 50.0;

	public static final boolean kIsReplay = false;
	public static final boolean kDoTuning = true;

	// [TODO] NOTE: Make sure to set this value correctly before competition!
	/**
	 * This should only be enabled when going into a match, and not anywhere else like the pit.
	 * MAKE SURE TO RE-DEPLOY AFTERWARDS. This disables certain features.
	 */
	public static final boolean kIsCompetition = false;

	/** Class holding the physical parameters of the robot. */
	public static final class PhysicalParameters {
		// Robot measurements
		public static final Distance kRobotLength = Centimeters.of(82), kRobotWidth = Centimeters.of(87);
		public static final Mass kRobotMass = Kilograms.of(55);
		public static final double kWheelCOF = 1.426;

		public static final double kBatteryInternalResistance = 0.016;
		public static final double kNominalVoltage = 12.8;
		public static final double kRobotWarningBatteryVoltageThreshold = 11.6;
	}

	/** The run mode of the robot. */
	public static enum RobotMode {
		/** Running real robot hardware. */
		kReal,
		/** Running simulation. */
		kSim,
		/** AdvantageKit replay. */
		kReplay;
	}
}
