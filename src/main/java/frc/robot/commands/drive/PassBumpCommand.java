package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Commands the chassis to pass the bump. */
public class PassBumpCommand extends Command {
	private final SwerveSubsystem m_swerve;
	private final double m_speed;

	// State variables
	private boolean m_hasTilted = false;
	private final Timer m_settleTimer = new Timer();
	private final Timer m_safetyTimer = new Timer();

	// Constants
	private static final double kTiltThreshold = 10.0;
	private static final double kSettleThreshold = 3.0;
	private static final double kSettleConfirmTime = 0.5;
	private static final double kSafetyTimeout = 5.0;

	/**
	 * Constructs a new PassBumpCommand.
	 *
	 * @param swerve The swerve subsystem.
	 * @param speed  The speed to command the chassis.
	 */
	public PassBumpCommand(SwerveSubsystem swerve, double speed) {
		m_swerve = swerve;
		m_speed = speed;
		addRequirements(m_swerve);
	}

	@Override
	public void initialize() {
		m_hasTilted = false;
		m_settleTimer.reset();
		m_settleTimer.stop();
		m_safetyTimer.restart();

		m_swerve.runSpeeds(new ChassisSpeeds(m_speed, 0, 0), true);
	}

	@Override
	public void execute() {
		m_swerve.runSpeeds(null, m_hasTilted);
		m_swerve.runSpeeds(new ChassisSpeeds(m_speed, 0, 0), true);

		double currentPitch = Math.abs(m_swerve.getGyro().getInputs().pitchDegrees);

		if (!m_hasTilted) {
			// Wait until we've gotten onto the bump
			if (currentPitch > kTiltThreshold) m_hasTilted = true;
		} else {

			if (currentPitch < kSettleThreshold) {
				m_settleTimer.start();
			} else {
				m_settleTimer.reset();
				m_settleTimer.stop();
			}
		}
	}

	@Override
	public boolean isFinished() {
		return (m_hasTilted && m_settleTimer.hasElapsed(kSettleConfirmTime))
				|| m_safetyTimer.hasElapsed(kSafetyTimeout);
	}

	@Override
	public void end(boolean interrupted) {
		m_swerve.stop();
	}
}
