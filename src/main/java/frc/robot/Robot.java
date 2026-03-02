// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kMechanismMovementControllerRumbleStrength;

import com.bobcats.lib.container.LoggedTunableNumber;
import com.bobcats.lib.sim.HardwareSimUtils;
import com.bobcats.lib.utils.AllianceUtil;
import com.bobcats.lib.utils.Tracer;
import com.bobcats.lib.utils.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhysicalParameters;
import frc.robot.constants.Constants.RobotMode;
import frc.robot.constants.FieldConstants;
import frc.robot.util.Elastic;
import frc.robot.util.SeasonUtils;
import frc.robot.util.maplesim.Arena2026Rebuilt;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** The main Robot class. */
public class Robot extends LoggedRobot {
	private final RobotContainer m_robotContainer;
	public static final RobotMode kRobotMode = Constants.kIsReplay ? RobotMode.kReplay
			: (Robot.isReal() ? RobotMode.kReal : RobotMode.kSim);

	private Command m_testCommand;
	private Runnable m_scheduleLogger;

	// Alerts
	private Alert m_canBusAlert = new Alert("A CAN Bus (bus=rio) error has occured!", Alert.AlertType.kError);
	private Alert m_batteryAlert = new Alert(
			"Battery voltage is too low when the robot is idle! Replace the battery if possible.",
			Alert.AlertType.kWarning);

	// Simulation battery fixes
	private double m_fixCurrent = 0;
	private double m_fixCurrentDrawn = 0;

	/** Constructs a new Robot. */
	public Robot() {
		if (Robot.isSimulation()) {
			var arena = new Arena2026Rebuilt(false);
			arena.setEfficiencyMode(false);
			SimulatedArena.overrideInstance(arena);
		}

		LoggedTunableNumber.DoTuning = Constants.kDoTuning;
		m_robotContainer = new RobotContainer();

		FlippingUtil.fieldSizeX = FieldConstants.kFieldLength;
		FlippingUtil.fieldSizeY = FieldConstants.kFieldWith;
	}

	@Override
	public void robotInit() {

		// Record metadata
		Logger.recordMetadata("IsCompetition", String.valueOf(Constants.kIsCompetition));

		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;

			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;

			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Mode and logger settings
		switch (kRobotMode) {
			case kReal:
				Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case kSim:
				Logger.addDataReceiver(new NT4Publisher());
				// Have to use this way because MapleSim doesn't have a way of changing the
				// nominal voltage
				// SimulatedBattery.addElectricalAppliances(() -> Amps.of(1 / 0.02 * (13.5 -
				// PhysicalParameters.kNominalVoltage)));

				addBatteryFix(PhysicalParameters.kNominalVoltage, PhysicalParameters.kBatteryInternalResistance);

				// Decrease the linear filter's taps
				HardwareSimUtils.modifySimulatedBatteryFilter(20);
				break;

			case kReplay:
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;

			default:
				throw new RuntimeException("Invalid robot mode: " + kRobotMode);
		}
		Logger.start();

		if (kRobotMode == RobotMode.kSim) {
			// Default station
			DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
			DriverStationSim.notifyNewData();
		}

		DriverStation.silenceJoystickConnectionWarning(true);
		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
		m_scheduleLogger = Utils.registerLoopTimeLogger(this);
	}

	@Override
	public void robotPeriodic() {
		// For thread priority (from docs):
		// This customization should only be used if the loop cycle time is
		// significantly less than
		// 20ms, which allows other threads to continue running between user code
		// cycles. We always
		// recommend thoroughly testing this change to ensure that it does not cause
		// unintended side
		// effects (examples include NetworkTables lag, CAN timeouts, etc). In general,
		// this
		// customization is only recommended for advanced users who understand the
		// potential
		// side-effects.

		// Switch thread to high priority to improve loop timing
		// Threads.setCurrentThreadPriority(true, 99);

		Tracer.start("CommandScheduler");
		CommandScheduler.getInstance().run();
		Tracer.finish("CommandScheduler");

		Tracer.start("RobotPeriodic_Misc");
		if (kRobotMode == RobotMode.kSim) SmartDashboard.putNumber("Battery Voltage", RoboRioSim.getVInVoltage());
		else SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

		// Update alerts
		var canStatus = RobotController.getCANStatus();
		m_canBusAlert.set(canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0);
		m_batteryAlert.set(isDisabled()
				&& RobotController.getBatteryVoltage() <= PhysicalParameters.kRobotWarningBatteryVoltageThreshold);

		// Log bus status
		Logger.recordOutput("MainBusStatus/offCount", canStatus.busOffCount);
		Logger.recordOutput("MainBusStatus/utilizationPercent", canStatus.percentBusUtilization);
		Logger.recordOutput("MainBusStatus/raceiveErrCount", canStatus.receiveErrorCount);
		Logger.recordOutput("MainBusStatus/transmitErrCount", canStatus.transmitErrorCount);
		Logger.recordOutput("MainBusStatus/txFullCount", canStatus.txFullCount);

		m_scheduleLogger.run();

		// Controller rumble
		if (!DriverStation.isDisabled() && !m_robotContainer.superstructure.areRumbleMechanismsAtSetpoints())
			m_robotContainer.setControllerRumble(kMechanismMovementControllerRumbleStrength);
		else m_robotContainer.setControllerRumble(0);

		m_robotContainer.updateAlerts();

		// Hub status
		Logger.recordOutput("AllianceHubStatus", Constants.kIsCompetition ? SeasonUtils.isAllianceHubActive() : true);
		Logger.recordOutput("AllianceHubTimeUntilActivation",
				Constants.kIsCompetition ? SeasonUtils.getTimeUntilHubShift() : 0);

		Tracer.finish("RobotPeriodic_Misc");

		// Return to normal thread priority
		// Threads.setCurrentThreadPriority(false, 10);
	}

	@Override
	public void disabledInit() {
		// Reset if in sim
		if (kRobotMode == RobotMode.kSim) {
			m_robotContainer.swerve.resetOdometry(AllianceUtil.flipWithAlliance(Constants.kStartingPose));
			SimulatedArena.getInstance().resetFieldForAuto();
		}

		// Lock wheels to prevent movement
		m_robotContainer.swerve.setX();
	}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		Elastic.selectTab("Autonomous");
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		Elastic.selectTab("Teleoperated");
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancel all commands at start of test mode
		CommandScheduler.getInstance().cancelAll();
		// Assign and schedule the test command
		CommandScheduler.getInstance().schedule(m_testCommand = m_robotContainer.getTestCommand());
		RobotContainer.getInstance().turret.setTurretAngle(100);
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {
		if (m_testCommand != null) m_testCommand.cancel();
	}

	@Override
	public void simulationInit() {
		// Do not use with advantagekit
		// SimulatedArena.overrideSimulationTimings(Seconds.of(Constants.kLoopPeriodSeconds),
		// (int) SwerveConstants.kOdometryFrequencyHz / 50);
	}

	@Override
	public void simulationPeriodic() {
		Tracer.start("SimulationPeriodic");
		m_fixCurrent = SimulatedBattery.getTotalCurrentDrawn().in(Amps);

		SimulatedArena.getInstance().simulationPeriodic();

		// Log latest field state
		Pose2d pose2d = m_robotContainer.swerveSim.getSimulatedDriveTrainPose();
		Logger.recordOutput("FieldSimulation/RobotPosition",
				m_robotContainer.superstructure.superstructureVisualizer.getRobotPoseWithHeight(pose2d));
		Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
		Tracer.finish("SimulationPeriodic");
	}

	private void addBatteryFix(double desiredNominalVoltage, double desiredInternalResistance) {
		double a = (13.5 - desiredNominalVoltage) / 0.02;
		double b = (desiredInternalResistance / 0.02) - 1.0;

		SimulatedBattery.addElectricalAppliances(() -> {
			if (m_fixCurrent - m_fixCurrentDrawn < 0) { return Amps.of(a); }
			double Ireal = m_fixCurrent;
			double Ivirtual = a + b * (Ireal - m_fixCurrentDrawn);
			m_fixCurrentDrawn = Ivirtual; // Math.max(0.0, Ivirtual);
			return Amps.of(m_fixCurrentDrawn);
		});
	}
}
