package frc.robot.subsystems.swerve.gyro;

import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kGyroConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.kOdometryFrequencyHz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.swerve.TalonFXOdometryThread;
import java.util.Queue;

/** The gyroscope IO implementation for the Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
	private Pigeon2 m_pigeon;

	private Queue<Double> m_odometryTimestamps;
	private Queue<Double> m_odometryYawPositions;

	private StatusSignal<Angle> m_yawSignal;
	private StatusSignal<AngularVelocity> m_angularVelocitySignal;
	private StatusSignal<Angle> m_pitchSignal;

	/**
	 * Constructs a new GyroIOPigeon2.
	 *
	 * @param canId The CAN ID of the pigeon.
	 */
	public GyroIOPigeon2(int canId) {
		this(canId, new CANBus(""));
	}

	/**
	 * Constructs a new GyroIOPigeon2.
	 *
	 * @param canId  The CAN ID of the pigeon.
	 * @param canBus The CAN bus.
	 */
	public GyroIOPigeon2(int canId, CANBus canBus) {
		m_pigeon = new Pigeon2(canId, canBus);

		m_pigeon.getConfigurator().apply(kGyroConfig);

		m_yawSignal = m_pigeon.getYaw();
		m_angularVelocitySignal = m_pigeon.getAngularVelocityZWorld();
		m_pitchSignal = m_pigeon.getPitch();

		BaseStatusSignal.setUpdateFrequencyForAll(kOdometryFrequencyHz, m_yawSignal, m_angularVelocitySignal,
				m_pitchSignal);
		ParentDevice.optimizeBusUtilizationForAll(m_pigeon);

		var odometryThread = TalonFXOdometryThread.getInstance();
		m_odometryTimestamps = odometryThread.makeTimestampQueue();
		m_odometryYawPositions = odometryThread.registerSignal(this::getYawDegrees);
	}

	@Override
	public void updateInputs(GyroIOInputsAutoLogged inputs) {
		inputs.yawDegrees = getYawDegrees();
		inputs.omegaDegreesPerSecond = getOmegaDegreesPerSecond();
		inputs.pitchDegrees = m_pitchSignal.getValueAsDouble();

		inputs.odometryYawTimestamps = m_odometryTimestamps.stream().mapToDouble((v) -> v).toArray();
		inputs.odometryYawPositions = m_odometryYawPositions.stream()
				.map(Rotation2d::fromDegrees)
				.toArray(Rotation2d[]::new);

		m_odometryTimestamps.clear();
		m_odometryYawPositions.clear();
	}

	@Override
	public double getYawDegrees() { return m_yawSignal.getValueAsDouble(); }

	@Override
	public double getOmegaDegreesPerSecond() { return m_angularVelocitySignal.getValueAsDouble(); }

	@Override
	public void setYaw(double yaw) {
		m_pigeon.setYaw(yaw);
	}
}
