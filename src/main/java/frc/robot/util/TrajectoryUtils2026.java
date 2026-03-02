package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

// TODO Add game piece distance limits

/**
 * A class to generate intake paths efficiently given constraints in the 2026 season.
 */
public class TrajectoryUtils2026 {

	/**
	 * The dimensions of each square in the field grid. Should be a value from
	 * <code> IntakeWidth / sqrt(2) to IntakeWidth / 2. </code>
	 */
	public static double kGridSizeMeters = 0.6;
	/** A hard limit on iteration, the maximum grid squares to check. */
	public static int kMaxGridTargetsToCheck = 50;

	private static final double kMinWaypointDist = 0.45;
	private static final double kTurnPenaltyWeight = 2.5;
	private static final double kCollinearThreshold = 0.20;

	private static final double kSweepMarginExtra = 0.05;

	private static final double kMaxSegmentLengthMeters = 1.5;

	/**
	 * Descriptor for a detected fuel cluster, the center of a grid cell.
	 */
	public static class FuelCluster {
		public final Translation2d pos;
		public final double expectedFuelCount;

		/**
		 * Constructs a new FuelCluster.
		 *
		 * @param pos               The position of the center.
		 * @param expectedFuelCount The estimated Fuel count.
		 */
		public FuelCluster(Translation2d pos, double expectedFuelCount) {
			this.pos = pos;
			this.expectedFuelCount = expectedFuelCount;
		}
	}

	/**
	 * Parameters the generator needs to generate and optimize a path.
	 */
	public static class GeneratorParams {
		public final Pose2d startPose;
		public final double timeBudgetSeconds;
		public final double maxSpeedMetersPerSec;
		public final double maxAccelMetersPerSec2;
		public final double stopPenaltySeconds;
		public final double intakeWidth;
		public final int maxHopperCapacity;
		public final Pose2d finalPose;

		/**
		 * Constructs a new GeneratorParams.
		 *
		 * @param startPose             The starting point of the trajectory.
		 * @param timeBudgetSeconds     The maximum allowed time of the trajectory, e.g. time to spend
		 *                              in the neutral zone, in seconds.
		 * @param maxSpeedMetersPerSec  The maximum allowed chassis speeds, in m/s.
		 * @param maxAccelMetersPerSec2 The maximum allowed chassis acceleration, in m/s^2.
		 * @param stopPenaltySeconds    The penalty applied to stop the robot, in seconds.
		 * @param intakeWidthMeters     The width of the intake, in meters.
		 * @param maxHopperCapacity     The maximum capacity of the hopper.
		 * @param finalPose             The ending point of the trajectory.
		 */
		public GeneratorParams(Pose2d startPose, double timeBudgetSeconds, double maxSpeedMetersPerSec,
				double maxAccelMetersPerSec2, double stopPenaltySeconds, double intakeWidthMeters,
				int maxHopperCapacity, Pose2d finalPose) {
			this.startPose = startPose;
			this.timeBudgetSeconds = timeBudgetSeconds;
			this.maxSpeedMetersPerSec = maxSpeedMetersPerSec;
			this.maxAccelMetersPerSec2 = maxAccelMetersPerSec2;
			this.stopPenaltySeconds = stopPenaltySeconds;
			this.intakeWidth = intakeWidthMeters;
			this.maxHopperCapacity = maxHopperCapacity;
			this.finalPose = finalPose;
		}
	}

	/**
	 * Generates a time-optimal, density-optimized path to intake Fuel efficiently.
	 *
	 * @param params     The optimization parameters
	 * @param detections The Fuel observed.
	 * @return The optional instance containing the trajectory optimization results.
	 */
	public static Optional<List<Translation2d>> generateGridOptimizedPath(GeneratorParams params,
			List<Pose2d> detections) {
		if (detections == null || detections.isEmpty()) return Optional.empty();

		Map<GridCell, Double> heatmap = new HashMap<>();
		for (var obs : detections) {
			GridCell cell = new GridCell(obs.getTranslation());
			heatmap.put(cell, heatmap.getOrDefault(cell, 0.0) + 1.0);
		}

		List<FuelCluster> candidates = heatmap.entrySet()
				.stream()
				.map(e -> new FuelCluster(e.getKey().toTranslation2d(), e.getValue()))
				.sorted(Comparator.comparingDouble((FuelCluster c) -> {
					double dist = c.pos.getDistance(params.startPose.getTranslation());
					return -(c.expectedFuelCount / Math.max(0.1, dist));
				}))
				.limit(kMaxGridTargetsToCheck)
				.collect(Collectors.toList());

		if (candidates.isEmpty()) return Optional.empty();

		List<Translation2d> waypoints = new ArrayList<>();
		waypoints.add(params.startPose.getTranslation());

		Pose2d currentSimPose = params.startPose;
		double timeUsed = 0.0;
		int currentLoad = 0;

		Translation2d currentHeadingVec = normalize(
				new Translation2d(Math.cos(params.startPose.getRotation().getRadians()),
						Math.sin(params.startPose.getRotation().getRadians())));

		List<FuelCluster> searchList = new ArrayList<>(candidates);

		while (!searchList.isEmpty() && currentLoad < params.maxHopperCapacity) {
			final Pose2d searchPose = currentSimPose;
			final Translation2d searchHeading = currentHeadingVec;

			final int remainingCapacity = params.maxHopperCapacity - currentLoad;
			if (remainingCapacity <= 0) break;

			// Choose best next cluster, scoring caps potential collection at remainingCapacity
			FuelCluster bestNext = Collections.max(searchList, Comparator.comparingDouble(c -> {
				double dist = c.pos.getDistance(searchPose.getTranslation());
				if (dist < kMinWaypointDist) return -1000.0;

				// Compute potential fuel collected for this segment
				double potentialCollected = Math.min((double) remainingCapacity, c.expectedFuelCount);
				for (FuelCluster other : searchList) {
					if (other == c) continue;
					double ratio = calculateCoverageRatio(searchPose.getTranslation(), c.pos, other.pos,
							params.intakeWidth);
					potentialCollected += (other.expectedFuelCount * ratio);
					if (potentialCollected >= remainingCapacity) { potentialCollected = remainingCapacity; break; }
				}

				double toCap = Math.max(0.0, potentialCollected);
				Translation2d toCandidate = c.pos.minus(searchPose.getTranslation());
				double aSeg = Math.atan2(toCandidate.getY(), toCandidate.getX());
				double aHead = Math.atan2(searchHeading.getY(), searchHeading.getX());
				double angleDiff = Math.abs(shortestAngleBetween(aHead, aSeg));

				double turnCost = 1.0 + (angleDiff * kTurnPenaltyWeight);
				double travelTime = estimateTraversalTime(dist, params.maxSpeedMetersPerSec,
						params.maxAccelMetersPerSec2);
				double totalCost = (travelTime * turnCost) + params.stopPenaltySeconds;

				return toCap / Math.max(0.1, totalCost);
			}));

			// After selection, determine exactly how much we will collect
			List<FuelCluster> toRemove = new ArrayList<>();
			int remaining = params.maxHopperCapacity - currentLoad;
			if (remaining <= 0) break;

			// Main target collection
			int collectedMain = (int) Math.min(remaining, Math.ceil(bestNext.expectedFuelCount));
			if (collectedMain > 0) { currentLoad += collectedMain; remaining -= collectedMain; toRemove.add(bestNext); }

			// Drive-by neighbors collection
			for (FuelCluster candidate : new ArrayList<>(searchList)) {
				if (candidate == bestNext) continue;
				if (remaining <= 0) break;

				double ratio = calculateCoverageRatio(currentSimPose.getTranslation(), bestNext.pos, candidate.pos,
						params.intakeWidth);
				int collected = (int) Math.round(candidate.expectedFuelCount * ratio);
				if (collected > 0) {
					int actuallyCollected = Math.min(collected, remaining);
					currentLoad += actuallyCollected;
					remaining -= actuallyCollected;
					if (actuallyCollected > 0) toRemove.add(candidate);
				}
			}

			if (collectedMain == 0 && remaining > 0 && bestNext.expectedFuelCount > 0) {
				currentLoad += 1;
				remaining -= 1;
				if (!toRemove.contains(bestNext)) toRemove.add(bestNext);
			}

			// Compute time to travel and accept only if within time budget
			double dist = bestNext.pos.getDistance(currentSimPose.getTranslation());
			double stepTime = estimateTraversalTime(dist, params.maxSpeedMetersPerSec, params.maxAccelMetersPerSec2);

			if (timeUsed + stepTime <= params.timeBudgetSeconds) {
				timeUsed += stepTime;
				searchList.removeAll(toRemove);

				Translation2d newHeading = bestNext.pos.minus(currentSimPose.getTranslation());
				currentHeadingVec = normalize(newHeading);
				currentSimPose = new Pose2d(bestNext.pos,
						new Rotation2d(currentHeadingVec.getX(), currentHeadingVec.getY()));
				waypoints.add(bestNext.pos);
			} else {
				// Couldn't afford time, we should revert any load we assumed for this candidate
				// We calculated collected amounts earlier, revert them since we didn't perform the action
				for (FuelCluster rem : toRemove) {
					// Roll back the action
					int rollback = (int) Math.min(Math.ceil(rem.expectedFuelCount), currentLoad);
					currentLoad -= rollback;
				}
				// Remove bestNext from consideration to avoid repeatedly trying unaffordable long hops
				searchList.remove(bestNext);
			}
		}

		// Split long segments so simplification doesn't collapse long diagonals
		List<Translation2d> splitWaypoints = splitLongSegments(waypoints, kMaxSegmentLengthMeters);
		List<Translation2d> simplified = simplifyWaypoints(splitWaypoints);

		simplified.add(params.finalPose.getTranslation());
		return Optional.of(simplified);
	}

	/**
	 * Returns the command to follow the given waypoints. Keep in mind that the rotation should be
	 * overriden if an intake path is being used.
	 *
	 * @param waypoints          The path waypoints.
	 * @param initial            The initial robot rotation.
	 * @param constraints        The path constraints.
	 * @param waypointVelocities The velocity to maintain at the end of each waypoint, in m/s.
	 * @return The command to follow the trajectory.
	 */
	public static Command followTrajectoryByPathfind(List<Translation2d> waypoints, Rotation2d initial,
			PathConstraints constraints, double waypointVelocities) {
		List<Command> followCmds = new ArrayList<>();

		// Pathfind to individual waypoints in order
		for (int i = 0; i < waypoints.size(); i++) {
			var waypoint = waypoints.get(i);
			// If it's the last waypoint, we stop, otherwise we maintain a velocity
			double endVelocity = (i == waypoints.size() - 1) ? 0.0 : waypointVelocities;

			followCmds.add(AutoBuilder.pathfindToPose(new Pose2d(waypoint, initial), constraints, endVelocity));
		}

		return Commands.sequence(followCmds.toArray(Command[]::new));
	}

	/** Splits the given segments with respect to the maximum allowed segment length. */
	private static List<Translation2d> splitLongSegments(List<Translation2d> original, double maxLen) {
		if (original.size() < 2) return new ArrayList<>(original);
		List<Translation2d> out = new ArrayList<>();
		out.add(original.get(0));
		for (int i = 1; i < original.size(); ++i) {
			Translation2d prev = out.get(out.size() - 1);
			Translation2d next = original.get(i);
			double dist = prev.getDistance(next);
			if (dist <= maxLen) {
				out.add(next);
			} else {
				int pieces = (int) Math.ceil(dist / maxLen);
				for (int p = 1; p <= pieces; ++p) {
					double t = (double) p / pieces;
					double x = prev.getX() + t * (next.getX() - prev.getX());
					double y = prev.getY() + t * (next.getY() - prev.getY());
					out.add(new Translation2d(x, y));
				}
			}
		}
		return out;
	}

	/** Simplifies the waypoint list by removing collinear points. */
	private static List<Translation2d> simplifyWaypoints(List<Translation2d> original) {
		if (original.size() < 3) return new ArrayList<>(original);
		List<Translation2d> smooth = new ArrayList<>();
		smooth.add(original.get(0));
		for (int i = 1; i < original.size() - 1; i++) {
			Translation2d prev = smooth.get(smooth.size() - 1);
			Translation2d curr = original.get(i);
			Translation2d next = original.get(i + 1);

			double distToLine = distanceToLine(curr, prev, next);
			if (distToLine > kCollinearThreshold) { smooth.add(curr); }
		}
		smooth.add(original.get(original.size() - 1));
		return smooth;
	}

	/**
	 * Returns the distance of a point to the given line defined by its starting and ending points.
	 */
	private static double distanceToLine(Translation2d p, Translation2d start, Translation2d end) {
		double l2 = Math.pow(start.getDistance(end), 2);
		if (l2 == 0) return p.getDistance(start);
		double t = ((p.getX() - start.getX()) * (end.getX() - start.getX())
				+ (p.getY() - start.getY()) * (end.getY() - start.getY())) / l2;
		t = Math.max(0, Math.min(1, t));
		Translation2d projection = new Translation2d(start.getX() + t * (end.getX() - start.getX()),
				start.getY() + t * (end.getY() - start.getY()));
		return p.getDistance(projection);
	}

	/** Grid cell key for heatmapping. */
	private record GridCell(int x, int y) {
		GridCell(Translation2d pos) {
			this((int) Math.floor(pos.getX() / kGridSizeMeters), (int) Math.floor(pos.getY() / kGridSizeMeters));
		}

		Translation2d toTranslation2d() {
			// Returns the center of the cell
			return new Translation2d(x * kGridSizeMeters + (kGridSizeMeters / 2.0),
					y * kGridSizeMeters + (kGridSizeMeters / 2.0));
		}
	}

	/** Calculates the amount the intake covered in a given grid square. */
	private static double calculateCoverageRatio(Translation2d start, Translation2d end, Translation2d cellCenter,
			double intakeWidth) {
		double margin = (intakeWidth / 2.0) + (kGridSizeMeters / 2.0) + kSweepMarginExtra;
		double minX = Math.min(start.getX(), end.getX()) - margin;
		double maxX = Math.max(start.getX(), end.getX()) + margin;
		double minY = Math.min(start.getY(), end.getY()) - margin;
		double maxY = Math.max(start.getY(), end.getY()) + margin;

		if (cellCenter.getX() < minX || cellCenter.getX() > maxX || cellCenter.getY() < minY
				|| cellCenter.getY() > maxY)
			return 0.0;

		double distToPath = distanceToLine(cellCenter, start, end);

		double rFull = Math.max(0.0, (intakeWidth / 2.0) - (kGridSizeMeters / 2.0));
		double rMax = (intakeWidth / 2.0) + (kGridSizeMeters * 0.707);

		if (distToPath <= rFull) return 1.0;
		if (distToPath >= rMax) return 0.0;
		return (rMax - distToPath) / (rMax - rFull);
	}

	/** Estimates the time to travel the given distance. */
	private static double estimateTraversalTime(double distanceMeters, double maxSpeed, double maxAccel) {
		if (distanceMeters <= 0) return 0.0;
		double tAccel = maxSpeed / maxAccel;
		double distAccel = 0.5 * maxAccel * tAccel * tAccel;
		if (distanceMeters < (2 * distAccel)) {
			return 2 * Math.sqrt(distanceMeters / maxAccel);
		} else {
			double distCruise = distanceMeters - (2 * distAccel);
			return (2 * tAccel) + (distCruise / maxSpeed);
		}
	}

	/** Normalizes the given vector. */
	private static Translation2d normalize(Translation2d v) {
		double len = Math.hypot(v.getX(), v.getY());
		if (len < 1e-6) return new Translation2d(1.0, 0.0);
		return new Translation2d(v.getX() / len, v.getY() / len);
	}

	/** Returns the shortest angle difference between the 2 angles. */
	private static double shortestAngleBetween(double a, double b) {
		double diff = b - a;
		while (diff > Math.PI)
			diff -= 2.0 * Math.PI;
		while (diff < -Math.PI)
			diff += 2.0 * Math.PI;
		return diff;
	}
}
