package frc.robot.subsystems.shooter.feeder;

import static frc.robot.subsystems.shooter.feeder.FeederConstants.kFeedVoltage;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.feeder.FeederConstants.kUnfeedVoltage;

import com.bobcats.lib.subsystem.rollers.GenericRollerSubsystem;
import com.bobcats.lib.subsystem.rollers.GenericRollerSubsystem.GenericRollerVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.shooter.feeder.Feeder.FeederGoal;
import frc.robot.subsystems.shooter.feeder.io.FeederIO;

/**
 * The main feeder subsystem to carry fuel into the shooter mechanism from the hopper.
 */
public class Feeder extends GenericRollerSubsystem<FeederGoal> {
	private FeederGoal m_currentGoal = new FeederGoal(0.0);

	/** The setpoint for the feeder motor voltage. */
	public static class FeederGoal implements GenericRollerVoltage {
		private final double m_voltage;

		/**
		 * Constructs a new FeederGoal.
		 *
		 * @param voltage The voltage to set the feeder motor to.
		 */
		public FeederGoal(double voltage) {
			m_voltage = voltage;
		}

		@Override
		public double voltage() {
			return m_voltage;
		}
	}

	/**
	 * Constructs a new Feeder.
	 *
	 * @param io The IO implementation to use.
	 */
	public Feeder(FeederIO io) {
		super("Feeder", io);
		setName("Feeder");
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) m_currentGoal = new FeederGoal(0.0);
		super.periodic();
	}

	/**
	 * Starts feeding balls into the shooter. Can be stopped via {@link #stop()} method.
	 */
	public void feed() {
		m_currentGoal = new FeederGoal(kFeedVoltage);
	}

	/**
	 * Stops feeding balls into the shooter.
	 */
	public void stop() {
		m_currentGoal = new FeederGoal(0.0);
	}

	/**
	 * Moves any balls in the shooter back to the hopper by unfeeding.
	 */
	public void unfeed() {
		m_currentGoal = new FeederGoal(kUnfeedVoltage);
	}

	public void setVoltage(double volts) { m_currentGoal = new FeederGoal(volts); }

	/**
	 * Returns the angular velocity of the feeder.
	 *
	 * @return The angular velocity of the feeder, in RPM.
	 */
	public double getVelocity() { return m_inputs.rpm / kGearboxReduction; }

	@Override
	public FeederGoal getVoltageSetpoint() { return m_currentGoal; }
}
