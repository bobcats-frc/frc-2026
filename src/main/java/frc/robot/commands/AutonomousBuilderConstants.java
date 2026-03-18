package frc.robot.commands;

import static frc.robot.constants.FieldConstants.kFieldWith;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Class containing constants related to the autonomous routine builder. */
public class AutonomousBuilderConstants {
	public static final double kDepotIdleCooldown = 0.4;
	public static final double kOutpostIdleCooldown = 2.5;

	public static final double kNeutralIntakeTimer = 1.0;
	public static final double kLeaveTimer = 0.5;

	public static final double kAdaptivePathStopPenalty = 0.2;
	public static final double kAdaptivePathWaypointVelocity = 1.3;

	public static final double kAutoBufferTime = 0.12;

	public static final ChassisSpeeds kLeaveSpeedsFwdBlue = new ChassisSpeeds(-1.5, 0, 0);

	public static final ChassisSpeeds kIntakeNeutralSpeedsFwdBlue = new ChassisSpeeds(3.5, 0, 0);
	public static final ChassisSpeeds kIntakeNeutralSpeedsRevBlue = new ChassisSpeeds(-3.5, 0, 0);

	public static final double kSimPassBumpTimer = 1.0;
	public static final double kPassBumpSpeedsFwdBlue = 1.5;
	public static final double kPassBumpSpeedsRevBlue = -1.5;

	public static final double k8ScoringTimer = 1.5;
	public static final double k24ScoringTimer = 2.5;
	public static final double k60ScoringTimer = 4.5;

	public static final double kScoringVelocityLimitDistanceThreshold = 3.7;

	public static final Pose2d kScoringPoseRightBlue = new Pose2d(2.5, 2.15, Rotation2d.fromDegrees(-136.7)),
			kScoringPoseLeftBlue = new Pose2d(2.5, kFieldWith - 2.15, Rotation2d.fromDegrees(136.7));
}
