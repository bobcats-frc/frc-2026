package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.kForwardOutput;
import static frc.robot.subsystems.climb.ClimbConstants.kG;
import static frc.robot.subsystems.climb.ClimbConstants.kMaxTemperature;
import static frc.robot.subsystems.climb.ClimbConstants.kStallTimeThreshold;
import static frc.robot.subsystems.climb.ClimbConstants.kStallVelocityThreshold;

import com.bobcats.lib.subsystem.bangbangElevator.BangBangElevator;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.climb.io.ClimbIO;

/**
 * The main climb subsystem.
 */
public class Climb extends BangBangElevator {
	private BangBangElevatorSetpoint m_setpoint = BangBangElevatorSetpoint.kRetract;

	/**
	 * Constructs a new Climb.
	 *
	 * @param io The IO implementation to use.
	 */
	public Climb(ClimbIO io) {
		super(io, kForwardOutput, kStallVelocityThreshold, kStallTimeThreshold, kG, false, "Climber");
		setName("Climber");
		setTemperatureLimits(true, kMaxTemperature);
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) stop();
		super.periodic();
	}

	/**
	 * Extends the climber arm up.
	 */
	public void extendArm() {
		forceCancelStop();
		m_setpoint = BangBangElevatorSetpoint.kExtend;
	}

	/**
	 * Retracts the climber arm down.
	 */
	public void retractArm() {
		forceCancelStop();
		m_setpoint = BangBangElevatorSetpoint.kRetract;
	}

	@Override
	public BangBangElevatorSetpoint getSetpoint() { return m_setpoint; }
}
