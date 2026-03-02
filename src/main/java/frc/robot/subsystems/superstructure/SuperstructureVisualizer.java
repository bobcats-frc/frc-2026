package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.climb.ClimbConstants.kMaxClimberHeight;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberMetersPerMotorRev;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberRobotClimbHeightThreshold;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberRobotOscillateAmpl;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberRobotOscillateFreq;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberRobotOscillateRestAngle;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberValidClimbRegionLeftBlue;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberValidClimbRegionLeftRed;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberValidClimbRegionRightBlue;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kClimberValidClimbRegionRightRed;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kIntakeArmMaxHorizontalExtension;

import com.bobcats.lib.ascope.rotational.PivotAscopeDisplay;
import com.bobcats.lib.subsystem.bangbangElevator.BangBangElevator.BangBangElevatorState;
import com.bobcats.lib.utils.AllianceUtil;
import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A class to relay changes in moving mechanisms to AdvantageScope for displaying.
 */
public class SuperstructureVisualizer extends SubsystemBase {
	private final PivotAscopeDisplay m_hoodDisplay, m_turretDisplay, m_intakeDisplay;

	private final Climb m_climber;
	private boolean m_hasClimbed;
	private final Supplier<Pose3d> m_backpackDisplay;
	private final Supplier<Pose3d> m_climbDisplay;

	private static final Pose3d kTurretPivotBaselineOffset = new Pose3d(0.17, 0, 0, Rotation3d.kZero);
	private static final Pose3d kHoodBaselineOffset = new Pose3d(0.21, 0, 0.545, Rotation3d.kZero);
	private static final Pose3d kIntakePivotBaselineOffset = new Pose3d(-0.27, 0.476, 0.18, Rotation3d.kZero);
	private static final Translation3d kHoodCentralPivot = new Translation3d(0.15, 0, 0.545);
	private static final double kHoodAngularOffset = -Math.toRadians(20);

	private static final double kClimberRobotSetbackToRung = 0.105;

	/**
	 * Constructs a new SuperstructureVisualizer.
	 *
	 * @param turret  The turret mechanism subsystem.
	 * @param hood    The hood mechanism subsystem.
	 * @param intake  The intake mechanism subsystem.
	 * @param climber The climb mechanism subsystem.
	 */
	public SuperstructureVisualizer(Turret turret, Hood hood, Intake intake, Climb climber) {
		m_climber = climber;

		// Hood pivot display
		m_hoodDisplay = new PivotAscopeDisplay("Hood",
				() -> kHoodBaselineOffset
						.rotateAround(kHoodCentralPivot, new Rotation3d(0, 0, Math.toRadians(turret.getAngle())))
						.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, kHoodAngularOffset, 0))),
				0, () -> new Rotation3d(0, Math.toRadians(hood.getAngle()), 0));

		// Turret pivot display
		m_turretDisplay = new PivotAscopeDisplay("Turret", () -> kTurretPivotBaselineOffset, 0,
				() -> new Rotation3d(0, 0, Math.toRadians(turret.getAngle())));

		// Intake arm display
		m_intakeDisplay = new PivotAscopeDisplay("Intake", () -> kIntakePivotBaselineOffset, 0,
				() -> new Rotation3d(0, Math.toRadians(intake.getIntakeAngle()), 0));

		// Climb display
		m_climbDisplay = () -> new Pose3d(0, 0, m_climber.getInputs().positionRevs * kClimberMetersPerMotorRev,
				Rotation3d.kZero);
		// Extending hopper display
		m_backpackDisplay = () -> new Pose3d(-kIntakeArmMaxHorizontalExtension
				* Math.cos(Math.toRadians(intake.getIntakeAngle())) / Math.cos(IntakeConstants.kArmMinAngle), 0, 0,
				Rotation3d.kZero);

		setName("SuperstructureVisualizer");
	}

	@Override
	public void periodic() {
		Tracer.start("SuperstructureVisualizerPeriodic");
		// Update && log all displays
		m_turretDisplay.update();
		m_hoodDisplay.update();
		m_intakeDisplay.update();
		Logger.recordOutput("Simulation/ClimbDisplay", m_climbDisplay.get());
		Logger.recordOutput("Simulation/HopperExtensionDisplay", m_backpackDisplay.get());
		Logger.recordOutput("Simulation/HasClimbed", m_hasClimbed);
		Tracer.finish("SuperstructureVisualizerPeriodic");
	}

	/**
	 * Returns a new robot pose with the climber accounted for.
	 *
	 * @param robotPose The original 2D pose.
	 * @return The new 3D robot pose.
	 */
	public Pose3d getRobotPoseWithHeight(Pose2d robotPose) {
		Pose3d displayPose;
		if (m_hasClimbed && !isWithinClimbRegion(robotPose)) {
			// Robot fell off the climber
			m_hasClimbed = false;
			displayPose = new Pose3d(robotPose);
		} else if (m_hasClimbed && m_climber.getState() == BangBangElevatorState.kExtending) {
			// Robot is letting go of the rung
			double hMeters = kMaxClimberHeight - kClimberMetersPerMotorRev * m_climber.getInputs().positionRevs;
			m_hasClimbed = hMeters >= kClimberRobotClimbHeightThreshold;
			displayPose = new Pose3d(robotPose.getX(), robotPose.getY(), hMeters,
					new Rotation3d(0,
							kClimberRobotOscillateRestAngle + kClimberRobotOscillateAmpl
									* Math.sin(kClimberRobotOscillateFreq * Timer.getFPGATimestamp()),
							robotPose.getRotation().getRadians()));
		} else if ((m_climber.getState() == BangBangElevatorState.kRetracting && isWithinClimbRegion(robotPose))
				|| m_hasClimbed) {
					// Robot has grabbed the rung and is climbing
					double hMeters = kMaxClimberHeight - kClimberMetersPerMotorRev * m_climber.getInputs().positionRevs;
					m_hasClimbed = hMeters >= kClimberRobotClimbHeightThreshold;
					displayPose = new Pose3d(robotPose.getX(), robotPose.getY(), hMeters,
							new Rotation3d(0,
									kClimberRobotOscillateRestAngle + kClimberRobotOscillateAmpl
											* Math.sin(kClimberRobotOscillateFreq * Timer.getFPGATimestamp()),
									robotPose.getRotation().getRadians()));
				} else {
					// Robot has not climbed
					displayPose = new Pose3d(robotPose);
				}

		// Account for the obtained offset due to robot rotation after climbing
		double sgn = 0;
		if (m_hasClimbed)
			// Robot grabbed the rung
			sgn = AllianceUtil.isRedAlliance() ? -1 : 1;
		displayPose = displayPose
				.transformBy(new Transform3d(sgn * kClimberRobotSetbackToRung, 0, 0, Rotation3d.kZero));

		return displayPose;
	}

	/** Returns whether the given pose is within any of the valid climb regions. */
	private boolean isWithinClimbRegion(Pose2d pose) {
		return kClimberValidClimbRegionRightBlue.isPoseWithinArea(pose)
				|| kClimberValidClimbRegionLeftBlue.isPoseWithinArea(pose)
				|| kClimberValidClimbRegionRightRed.isPoseWithinArea(pose)
				|| kClimberValidClimbRegionLeftRed.isPoseWithinArea(pose);
	}
}
