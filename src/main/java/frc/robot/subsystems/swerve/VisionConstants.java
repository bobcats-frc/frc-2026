package frc.robot.subsystems.swerve;

import com.bobcats.lib.utils.VisionUtils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.simulation.SimCameraProperties;

/** Class holding vision and odometry constants. */
public class VisionConstants {
	public static final Vector<N3> kOdometryStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);

	// LL Data
	// TODO zero these before vision calib.
	public static final Transform3d kRobotToCamera_Right = new Transform3d(0.35, -0.25, 0.15,
			new Rotation3d(Rotation2d.fromDegrees(15)));
	public static final String kLLName_Right = "limelight_right";
	public static final int kLLIndex_Right = 0;

	public static final Transform3d kRobotToCamera_Left = new Transform3d(0.35, 0.25, 0.15,
			new Rotation3d(Rotation2d.fromDegrees(-15)));
	public static final String kLLName_Left = "limelight_left";
	public static final int kLLIndex_Left = 1;

	public static final int kLocalizationPipelineId = 0;
	public static final int kObjectDetectionPipelineId = 1;

	public static final double kObjectDetectionMaxYaw = 45.0;

	// Sim data
	public static final SimCameraProperties kSimCameraProperties = VisionUtils.getCalibrated(640, 480,
			Rotation2d.fromDegrees(VisionUtils.diagonalFovDeg(82, 56.2)), VecBuilder.fill(0, 0, 0, 0, 0, 0, 0, 0));

	static {
		kSimCameraProperties.setFPS(20);
		kSimCameraProperties.setAvgLatencyMs(25);
		kSimCameraProperties.setLatencyStdDevMs(8);
		// kSimCameraProperties.setCalibError(0.02, 0.01);
	}
}
