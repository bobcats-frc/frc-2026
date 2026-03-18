package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationYaw;

import com.bobcats.lib.ascope.rotational.PivotAscopeDisplay;
import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.hood.Hood;

/**
 * A class to relay changes in moving mechanisms to AdvantageScope for displaying.
 */
public class SuperstructureVisualizer extends SubsystemBase {
	private final PivotAscopeDisplay m_hoodDisplay, m_intakeDisplay;

	private static final Pose3d kHoodBaselineOffset = new Pose3d(0.18, 0.095, 0.594,
			new Rotation3d(Rotation2d.fromDegrees(kHoodCalibrationYaw)));
	private static final Pose3d kIntakePivotBaselineOffset = new Pose3d(-0.27, 0.476, 0.18, Rotation3d.kZero);
	private static final double kHoodAngularOffset = -Math.toRadians(20);

	/**
	 * Constructs a new SuperstructureVisualizer.
	 *
	 * @param hood   The hood mechanism subsystem.
	 * @param intake The intake mechanism subsystem.
	 */
	public SuperstructureVisualizer(Hood hood, Intake intake) {
		// Hood pivot display
		m_hoodDisplay = new PivotAscopeDisplay("Hood", () -> kHoodBaselineOffset
				// .rotateAround(kHoodCentralPivot, new Rotation3d(0, 0, Math.toRadians(turret.getAngle())))
				.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, kHoodAngularOffset, 0))), 0,
				() -> new Rotation3d(0, Math.toRadians(hood.getAngle()), 0));

		// Intake arm display
		m_intakeDisplay = new PivotAscopeDisplay("Intake", () -> kIntakePivotBaselineOffset, 0,
				() -> new Rotation3d(0, Math.toRadians(intake.getIntakeAngle()), 0));

		setName("SuperstructureVisualizer");
	}

	@Override
	public void periodic() {
		Tracer.start("SuperstructureVisualizerPeriodic");
		// Update && log all displays
		m_hoodDisplay.update();
		m_intakeDisplay.update();
		Tracer.finish("SuperstructureVisualizerPeriodic");
	}
}
