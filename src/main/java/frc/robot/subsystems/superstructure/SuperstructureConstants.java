package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCentralPivotRobotRelative;

import com.bobcats.lib.container.Vector3;
import com.bobcats.lib.control.shooter.ShooterCalculator.ShooterParameters;
import com.bobcats.lib.control.shooter.data.ShooterDescriptor;
import com.bobcats.lib.control.shooter.data.ShooterProjectileType;
import com.bobcats.lib.math.BilinearInterpolator2D;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/** The class storing the constants required for the Superstructure. */
public class SuperstructureConstants {
	// Chassis state constants
	public static final double kSwerveIdleVelocityThreshold = 0.3;
	public static final double kChassisStateSwitchDebounce = 0.2;

	public static final double kShooterStateStaleTimeThreshold = 0.2;

	public static final int kFuelCapacity = 60;

	public static final double kIntakeArmMaxHorizontalExtension = 0.2;

	// [0, 1], the rumble played when the mechanisms aren't at their setpoints
	public static final double kMechanismMovementControllerRumbleStrength = 0.15;

	// Always search for the most optimal path instead
	public static final double kPassFuelAcceptDistance = 0;

	public static final String kCorralAlignPath = "CorralAlign";
	public static final double kCorralOuttakeTimerLimit = 2.0;

	// Shot calculator data
	// Soft-limit of the maximum allowed shot distance
	public static final double kMaxAllowedShotDistance = 8.0; // 5.63;
	public static final double kMinAllowedShotDistance = 1.17;

	public static final double kRollerRadiusMeters = Units.inchesToMeters(2);
	public static final double kBarrelLengthMeters = 0;
	public static final double kAverageShotTime = 1.0 / 5.0;

	// Shooting chassis yaw error tolerance for angular PID
	public static final double kShotYawMaxError = Math.toRadians(3.0);

	public static final Color kShooterReadyFlashColor = new Color(208, 0, 245);
	public static final Color kShooterNotReadyFlashColor = new Color(245, 57, 0);
	public static final Color kShooterFlashOffColor = new Color();

	public static final Color kPrefireFlashColor = new Color(208, 0, 245);
	public static final Color kPrefireFlashColorSecondary = new Color(0, 181, 30);
	public static final Color kPrefireFlashOffColor = new Color();

	public static final double kIntakeRollerVelocityThreshold = 5;
	public static final Color kIntakeFlashRollingColor = new Color(0, 181, 30);
	public static final Color kIntakeFlashOffColor = new Color();
	public static final double kFlashPeriod = 0.1; // seconds

	// About a step size of 1.6in
	public static final int kPassTrajOptSteps = 200;

	public static final ShooterParameters kPresetShootingParametersBlue = new ShooterParameters(true,
			Rotation2d.fromDegrees(80.5), Rotation2d.fromDegrees(180 + 80.5), 65, 2300, 0, Translation3d.kZero,
			Vector3.kZero, 0);

	public static final int kPreloadedFuelAmount = 8;

	// Note: make sure to pick high horizontal velocities in order to minimize time
	// and allow for shooting on the move while going backwards

	public static final double[] kDistanceMap = new double[] { 1.17, 2.14, 3.10, 3.84, 5.12, 5.63, 7.0, 8.0 };
	public static final double[] kHeightDiffMap = new double[] { 3, 3 };
	public static final double[][] kAngleMap = new double[][] { { 78.0, 78.0 }, { 75.0, 75.0 }, { 69.0, 69.0 },
			{ 66.0, 66.0 }, { 62.0, 62.0 }, { 53.9, 53.9 }, { 50.0, 50.0 }, { 45.0, 45.0 } };
	public static final double[][] kRPMMap = new double[][] { { 185.0 * 30.0 / Math.PI, 185.0 * 30.0 / Math.PI },
			{ 225.0 * 30.0 / Math.PI, 225.0 * 30.0 / Math.PI }, { 235.0 * 30.0 / Math.PI, 235.0 * 30.0 / Math.PI },
			{ 245.0 * 30.0 / Math.PI, 245.0 * 30.0 / Math.PI }, { 265.0 * 30.0 / Math.PI, 265.0 * 30.0 / Math.PI },
			{ 270.0 * 30.0 / Math.PI, 270.0 * 30.0 / Math.PI }, { 2700.0, 2700.0 }, { 2770.0, 2770.0 } };

	public static final double[] kDistanceMapTOF = new double[] { 1.38, 1.88, 3.15, 4.55, 5.68 };
	public static final double[][] kTimeOfFlightMap = new double[][] { { 0.9 + 0.2, 0.9 + 0.2 },
			{ 1.09 + 0.15, 1.09 + 0.15 }, { 1.11 + 0.1, 1.11 + 0.1 }, { 1.12 + 0.13, 1.12 + 0.13 },
			{ 1.16 + 0.13, 1.16 + 0.13 } };

	// Extra flight time fudge factors to allow shooting earlier to time more accurately
	public static final InterpolatingDoubleTreeMap kShootingFlightTimeFudgeFactors = new InterpolatingDoubleTreeMap();
	public static final double kPrefireFlashTimeBuffer = 2.5;

	static {
		kShootingFlightTimeFudgeFactors.put(0.5, 0.10);
		kShootingFlightTimeFudgeFactors.put(1.0, 0.20);
		kShootingFlightTimeFudgeFactors.put(1.5, 0.30);
		kShootingFlightTimeFudgeFactors.put(2.0, 0.35);
		kShootingFlightTimeFudgeFactors.put(3.0, 0.40);
		kShootingFlightTimeFudgeFactors.put(4.5, 0.45);
	}

	public static final double kShotDelay = 0;

	// Bilinear interpolators
	public static final BilinearInterpolator2D kAngleInterpolator = new BilinearInterpolator2D(kDistanceMap,
			kHeightDiffMap, kAngleMap);
	public static final BilinearInterpolator2D kRPMInterpolator = new BilinearInterpolator2D(kDistanceMap,
			kHeightDiffMap, kRPMMap);
	public static final BilinearInterpolator2D kTimeOfFlightInterpolator = new BilinearInterpolator2D(kDistanceMapTOF,
			kHeightDiffMap, kTimeOfFlightMap);

	public static final ShooterDescriptor kShooterDescriptor = new ShooterDescriptor.Builder(kRollerRadiusMeters,
			kHoodCentralPivotRobotRelative).barrelLength(kBarrelLengthMeters)
					.dataSet(kAngleInterpolator, kRPMInterpolator, kTimeOfFlightInterpolator)
					.setDistanceLimits(kMinAllowedShotDistance, kMaxAllowedShotDistance)
					.shotDelay(kShotDelay)
					.build();

	public static final boolean kLimitVelocityWhenShootingTeleop = true;
	public static final double kLimitK0Hub = 0.90;
	public static final double kLimitK1Hub = 0.60;

	public static final double kLimitK0Pass = 0.86;
	public static final double kLimitK1Pass = 0.60;

	// TODO Obtain e coefficient via calibration
	public static final ShooterProjectileType kFuelProjectile = new ShooterProjectileType(0.000485513987, 0.0050949066,
			0.215456376, 0.7);

	// TODO Measure custom data
	// 6328 data
	// m_angleMap.put(1.17, Rotation2d.fromDegrees(19.0 + 40.0));
	// m_angleMap.put(1.55, Rotation2d.fromDegrees(19.0 + 40.0));
	// m_angleMap.put(1.96, Rotation2d.fromDegrees(19.0 + 40.0));
	// m_angleMap.put(2.14, Rotation2d.fromDegrees(20.0 + 40.0));
	// m_angleMap.put(2.40, Rotation2d.fromDegrees(21.0 + 40.0));
	// m_angleMap.put(2.78, Rotation2d.fromDegrees(22.0 + 40.0));
	// m_angleMap.put(3.10, Rotation2d.fromDegrees(24.0 + 40.0));
	// m_angleMap.put(3.44, Rotation2d.fromDegrees(26.0 + 40.0));
	// m_angleMap.put(3.84, Rotation2d.fromDegrees(27.0 + 40.0));
	// m_angleMap.put(4.60, Rotation2d.fromDegrees(32.0 + 40.0));
	// m_angleMap.put(5.12, Rotation2d.fromDegrees(33.0 + 40.0));
	// m_angleMap.put(5.63, Rotation2d.fromDegrees(36.0 + 40.0));
	// m_rollerSpeeds.put(1.17, 185.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(1.55, 205.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(1.96, 225.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(2.14, 225.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(2.40, 225.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(2.78, 230.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(3.10, 235.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(3.44, 238.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(3.84, 245.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(4.60, 252.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(5.12, 265.0 * 30.0 / Math.PI);
	// m_rollerSpeeds.put(5.63, 270.0 * 30.0 / Math.PI);
	// m_filghtTimeMap.put(5.68, 1.16);
	// m_filghtTimeMap.put(4.55, 1.12);
	// m_filghtTimeMap.put(3.15, 1.11);
	// m_filghtTimeMap.put(1.88, 1.09);
	// m_filghtTimeMap.put(1.38, 0.90);
}
