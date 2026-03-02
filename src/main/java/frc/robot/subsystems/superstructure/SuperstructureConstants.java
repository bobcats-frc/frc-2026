package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.climb.ClimbConstants.kGearboxReduction;
import static frc.robot.subsystems.climb.ClimbConstants.kSpoolRadius;

import com.bobcats.lib.container.Vector3;
import com.bobcats.lib.control.shooter.ShooterCalculator.ShooterParameters;
import com.bobcats.lib.control.shooter.data.ShooterDescriptor;
import com.bobcats.lib.control.shooter.data.ShooterProjectileType;
import com.bobcats.lib.math.BilinearInterpolator2D;
import com.bobcats.lib.region.RectangularRegion;
import com.bobcats.lib.utils.AllianceUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.shooter.turret.TurretConstants;

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

	public static final String kClimberPreAlignPathRight = "ClimberAlignRightPre";
	public static final String kClimberPreAlignPathLeft = "ClimberAlignLeftPre";
	public static final String kClimberPostAlignPathRight = "ClimberAlignRightPost";
	public static final String kClimberPostAlignPathLeft = "ClimberAlignLeftPost";

	public static final String kCorralAlignPath = "CorralAlign";
	public static final double kCorralOuttakeTimerLimit = 2.0;

	// public static final double kClimberRobotUnclimbHeightThreshold = 0.05;
	public static final double kClimberMetersPerMotorRev = 2 * Math.PI * kSpoolRadius / kGearboxReduction;
	public static final double kClimberRobotClimbHeightThreshold = 0.01;
	public static final double kClimberRobotOscillateAmpl = 0.005;
	public static final double kClimberRobotOscillateFreq = 2;
	public static final double kClimberRobotOscillateRestAngle = -Math.toRadians(10);
	// TODO Use actual measurements
	public static final RectangularRegion kClimberValidClimbRegionRightBlue = new RectangularRegion(
			new Translation2d(0.578 - 0.08, 2.863 - 0.08), new Translation2d(0.705 + 0.08, 2.995 + 0.08));
	public static final RectangularRegion kClimberValidClimbRegionLeftBlue = new RectangularRegion(
			new Translation2d(0, 0), new Translation2d(5, 10));
	public static final RectangularRegion kClimberValidClimbRegionRightRed = new RectangularRegion(
			AllianceUtil.flipWithAlliance(new Translation2d(0.578 - 0.08, 2.863 - 0.08)),
			AllianceUtil.flipWithAlliance(new Translation2d(0.705 + 0.08, 2.995 + 0.08)));
	public static final RectangularRegion kClimberValidClimbRegionLeftRed = new RectangularRegion(
			AllianceUtil.flipWithAlliance(new Translation2d(0, 0)),
			AllianceUtil.flipWithAlliance(new Translation2d(5, 10)));

	// Threshold for pitch angle difference to detect climb state changes
	// Only downside is when tipping over
	public static final double kClimbPitchAngleDiffThreshold = 6;

	// Shot calculator data
	// Soft-limit of the maximum allowed shot distance
	public static final double kMaxAllowedShotDistance = 5.63;
	public static final double kMinAllowedShotDistance = 1.17;

	public static final double kRollerRadiusMeters = Units.inchesToMeters(2);
	public static final double kBarrelLengthMeters = 0;
	public static final Transform3d kPivotOffset = new Transform3d();
	public static final double kAverageShotTime = 1.0 / 5.0;

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

	// Climb shot parameters, ok to leave some parameters blank since they are
	// re-calculated in sim, and not used at all in real mode
	public static final ShooterParameters kClimbedShootingParametersLeftBlue = new ShooterParameters(true,
			Rotation2d.kZero, Rotation2d.fromDegrees(180 - 17.5), 0, 65, 0, 2300, 0, Translation3d.kZero, Vector3.kZero,
			0);
	public static final ShooterParameters kClimbedShootingParametersRightBlue = new ShooterParameters(true,
			Rotation2d.kZero, Rotation2d.fromDegrees(180 + 17.5), 0, 65, 0, 2300, 0, Translation3d.kZero, Vector3.kZero,
			0);
	// TODO make sure to autoflip
	public static final ShooterParameters kPresetShootingParametersBlue = new ShooterParameters(true, Rotation2d.kZero,
			Rotation2d.fromDegrees(180 + 80.5), 0, 65, 0, 2300, 0, Translation3d.kZero, Vector3.kZero, 0);

	public static final int kPreloadedFuelAmount = 8;

	// Note: make sure to pick high horizontal velocities in order to minimize time
	// and allow for shooting on the move while going backwards

	public static final double[] kDistanceMap = new double[] { 1.17, 2.14, 3.10, 3.84, 5.12, 5.63 };
	public static final double[] kHeightDiffMap = new double[] { 3, 3 };
	public static final double[][] kAngleMap = new double[][] { { 78.0, 78.0 }, { 75.0, 75.0 }, { 69.0, 69.0 },
			{ 66.0, 66.0 }, { 62.0, 62.0 }, { 53.9, 53.9 } };
	public static final double[][] kRPMMap = new double[][] { { 185.0 * 30.0 / Math.PI, 185.0 * 30.0 / Math.PI },
			{ 225.0 * 30.0 / Math.PI, 225.0 * 30.0 / Math.PI }, { 235.0 * 30.0 / Math.PI, 235.0 * 30.0 / Math.PI },
			{ 245.0 * 30.0 / Math.PI, 245.0 * 30.0 / Math.PI }, { 265.0 * 30.0 / Math.PI, 265.0 * 30.0 / Math.PI },
			{ 270.0 * 30.0 / Math.PI, 270.0 * 30.0 / Math.PI } };

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
			kPivotOffset).barrelLength(kBarrelLengthMeters)
					.dataSet(kAngleInterpolator, kRPMInterpolator, kTimeOfFlightInterpolator)
					.setDistanceLimits(kMinAllowedShotDistance, kMaxAllowedShotDistance)
					.shotDelay(kShotDelay)
					.turretAngleRange(TurretConstants.kMinAngleDeg, TurretConstants.kMaxAngleDeg)
					.build();

	public static final boolean kLimitVelocityWhenShootingTeleop = true;
	public static final double kLimitK0 = 1.7;
	public static final double kLimitK1 = 1.1;

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
