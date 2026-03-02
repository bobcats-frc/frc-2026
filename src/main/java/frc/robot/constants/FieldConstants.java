package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.constants.Constants.RobotMode;

// TODO Switch before regionals

/** A class containing important field coordinates. */
public class FieldConstants {
	public static final String kRealFieldType = "AndyMark";
	public static final String kFieldType = Robot.kRobotMode == RobotMode.kSim ? "Welded" : kRealFieldType;

	// AndyMark
	// public static final Translation3d kBlueHubCenterPose = new
	// Translation3d(Units.inchesToMeters(181.56),
	// Units.inchesToMeters(158.32), Units.inchesToMeters(72));

	// Welded
	public static final Translation3d kBlueHubCenterPose = new Translation3d(Units.inchesToMeters(178.81),
			Units.inchesToMeters(158.845), Units.inchesToMeters(72));

	public static final double kFuelDiameterNominal = Units.inchesToMeters(5.91);

	// AndyMark
	// public static final double kFieldWith = Units.inchesToMeters(316.64);
	// public static final double kFieldLength = Units.inchesToMeters(650.12);

	// Welded
	public static final double kFieldWith = Units.inchesToMeters(317.69);
	public static final double kFieldLength = Units.inchesToMeters(651.22);

	// AndyMark
	// public static final Pose2d kRobotPoseMiddleBlue = new Pose2d(3.507, 4.082, Rotation2d.kPi);
	// public static final Pose2d kRobotPoseRightBlue = new Pose2d(3.589, 2.498, Rotation2d.kPi);
	// public static final Pose2d kRobotPoseLeftBlue = new Pose2d(3.589, 5.544, Rotation2d.kPi);

	// Welded
	public static final Pose2d kRobotPoseMiddleBlue = new Pose2d(3.507, 4.082, Rotation2d.kPi);
	public static final Pose2d kRobotPoseRightBlue = new Pose2d(3.589, 2.511, Rotation2d.kPi);
	public static final Pose2d kRobotPoseLeftBlue = new Pose2d(3.589, 5.558, Rotation2d.kPi);

	// Welded
	// Adding 2x ball diameter on each side to avoid any intersect
	public static final Translation2d kBlueAllianceNetRightPoint = new Translation2d(Units.inchesToMeters(214.57),
			Units.inchesToMeters(129.635 - kFuelDiameterNominal * 2.0));
	public static final Translation2d kBlueAllianceNetLeftPoint = new Translation2d(Units.inchesToMeters(214.57),
			Units.inchesToMeters(188.045 + kFuelDiameterNominal * 2.0));
	// 8in cut off from each side to avoid issues with accidentally throwing fuel
	// out of the field
	public static final Translation2d kBlueAllianceZoneFuelPassLineRightPoint = new Translation2d(
			Units.inchesToMeters(106.606), Units.inchesToMeters(8.0));
	public static final Translation2d kBlueAllianceZoneFuelPassLineLeftPoint = new Translation2d(
			Units.inchesToMeters(106.606), kFieldWith - Units.inchesToMeters(8.0));

	/**
	 * Adds the field type suffix to the path name.
	 *
	 * @param name The path's root name.
	 * @return The suffixed path name.
	 */
	public static String path(String name) {
		return name + "_" + kFieldType;
	}
}
