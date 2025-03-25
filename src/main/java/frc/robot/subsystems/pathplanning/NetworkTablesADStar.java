package frc.robot.subsystems.pathplanning;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.DriveSubsystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * An AD* pathfinder that runs in a background thread, reading dynamic obstacles
 * from NetworkTables.
 * This version is updated based on the new LocalADStar implementation.
 */
public class NetworkTablesADStar implements Pathfinder {
    // Controls path smoothness - higher values produce smoother paths but may stay
    // further from the original path points
    private static final double SMOOTHING_ANCHOR_PCT = 0.01;

    // Initial epsilon value for the AD* algorithm - higher values give faster but
    // less optimal initial solutions
    private static final double EPS = 10;

    // Field dimensions in meters
    private double fieldLength = 16.54;
    private double fieldWidth = 8.02;

    // Size of each grid cell in meters - smaller values provide higher resolution
    // but slower planning
    private double nodeSize = 0.2;
    // Number of grid cells in X and Y directions
    private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
    private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

    // AD* algorithm data structures
    private final HashMap<GridPosition, Double> g = new HashMap<>(); // Current cost to reach each node
    private final HashMap<GridPosition, Double> rhs = new HashMap<>(); // One-step lookahead cost to each node
    private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>(); // Nodes to be expanded
    private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>(); // Inconsistent nodes
    private final Set<GridPosition> closed = new HashSet<>(); // Already expanded nodes

    // Obstacle tracking
    private final Set<GridPosition> staticObstacles = new HashSet<>(); // Fixed obstacles (from field layout)
    private final Set<GridPosition> dynamicObstacles = new HashSet<>(); // Moving obstacles (from NetworkTables)
    private final Set<GridPosition> requestObstacles = new HashSet<>(); // Combined obstacles for current planning

    // Planning request variables
    private GridPosition requestStart; // Start position for planning
    private Translation2d requestRealStartPos; // Real-world start position
    private GridPosition requestGoal; // Goal position for planning
    private Translation2d requestRealGoalPos; // Real-world goal position

    // Current epsilon value - decreases over time to improve solution quality
    private double eps;

    private final Thread planningThread;
    private final Thread dynamicObstacleThread;

    // Control flags for planning behavior
    private boolean requestMinor = true; // Triggers minor replanning for position changes
    private boolean requestMajor = true; // Triggers major replanning for optimal solutions
    private boolean requestReset = true; // Triggers complete algorithm reset
    private boolean newPathAvailable = false; // Indicates when a new path has been calculated

    private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
    private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

    // We now store a list of Waypoints (instead of PathPoints)
    private List<Waypoint> currentWaypoints = new ArrayList<>();
    private List<GridPosition> currentPathFull = new ArrayList<>();

    private final NetworkTable dynamicObstacleTable;
    private final DriveSubsystem driveSubsystem;

    // New member variables for robot pose publishing
    private final NetworkTableEntry robotPoseEntry;
    private final Thread robotPosePublishThread;
    private final List<List<Translation2d>> staticObstaclePolygons = new ArrayList<>();

    /**
     * Create a new NetworkTablesADStar pathfinder.
     *
     * @param driveSubsystem The drive subsystem used to fetch the robot’s current
     *                       pose.
     */
    public NetworkTablesADStar(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        planningThread = new Thread(this::runThread);
        dynamicObstacleThread = new Thread(this::updateDynamicObstacles);
        robotPosePublishThread = new Thread(this::publishRobotPose);

        requestStart = new GridPosition(0, 0);
        requestRealStartPos = new Translation2d(0, 0);
        requestGoal = new GridPosition(0, 0);
        requestRealGoalPos = new Translation2d(0, 0);

        staticObstacles.clear();
        dynamicObstacles.clear();

        // Initialize NetworkTables
        dynamicObstacleTable = NetworkTableInstance.getDefault().getTable("pythonData");
        robotPoseEntry = dynamicObstacleTable.getEntry("robot_pose");

        // Load obstacles from navgrid.json
        loadObstaclesFromNavGrid();

        requestObstacles.clear();
        requestObstacles.addAll(staticObstacles);
        requestObstacles.addAll(dynamicObstacles);

        requestReset = true;
        requestMajor = true;
        requestMinor = true;

        newPathAvailable = false;

        // Start all threads
        planningThread.setDaemon(true);
        planningThread.setName("ADStar Planning Thread");
        planningThread.start();

        dynamicObstacleThread.setDaemon(true);
        dynamicObstacleThread.setName("Dynamic Obstacle Update Thread");
        dynamicObstacleThread.start();

        robotPosePublishThread.setDaemon(true);
        robotPosePublishThread.setName("Robot Pose Publish Thread");
        robotPosePublishThread.start();
    }

    /**
     * Periodically update the set of dynamic obstacles from NetworkTables.
     */
    /**
     * Find the closest grid position that is not an obstacle.
     * If the given position is not an obstacle, it is returned.
     * Otherwise, search outward in a spiral pattern for the nearest non-obstacle
     * position.
     *
     * @param pos       The starting grid position
     * @param obstacles The set of obstacle positions
     * @return The closest non-obstacle position, or null if none found
     */
    private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
        // If the position is already not an obstacle, return it
        if (!obstacles.contains(pos)) {
            return pos;
        }

        // Search in a spiral pattern outward from the starting position
        int maxSearchRadius = 15; // Limit search radius to avoid infinite loops

        for (int radius = 1; radius <= maxSearchRadius; radius++) {
            // Check all positions at the current radius in a square spiral
            for (int dx = -radius; dx <= radius; dx++) {
                for (int dy = -radius; dy <= radius; dy++) {
                    // Only check positions on the perimeter of the square
                    if (Math.abs(dx) == radius || Math.abs(dy) == radius) {
                        GridPosition candidate = new GridPosition(pos.x() + dx, pos.y() + dy);

                        // Check if the position is within the grid bounds
                        if (candidate.x() >= 0 && candidate.x() < nodesX &&
                                candidate.y() >= 0 && candidate.y() < nodesY) {

                            // If it's not an obstacle, return it
                            if (!obstacles.contains(candidate)) {
                                return candidate;
                            }
                        }
                    }
                }
            }
        }

        // If no non-obstacle position found within range, return null
        return null;
    }

    private void updateDynamicObstacles() {
        System.out.println("Starting dynamic obstacle update thread");
        while (true) {
            try {
                String obstacleData = dynamicObstacleTable.getEntry("dynamic_obstacles").getString("[]");
                List<Pair<Translation2d, Translation2d>> dynamicObstacleList = parseDynamicObstacles(obstacleData);
                Translation2d currentRobotPos = driveSubsystem.getPose().getTranslation();

                // Update dynamic obstacles and also update the robot’s start position
                setDynamicObstacles(dynamicObstacleList, currentRobotPos);
                // Comment out obstacle data printing to reduce output
                /*
                 * for (Pair<Translation2d, Translation2d> obstacle : dynamicObstacleList) {
                 * System.out.println("Obstacle at: " + obstacle.getFirst().getX() + ", " +
                 * obstacle.getFirst().getY() + " with size: " + obstacle.getSecond().getX());
                 * }
                 */
                // Print obstacle data for debugging
                // for (Pair<Translation2d, Translation2d> obstacle : dynamicObstacleList) {
                // System.out.println("Obstacle at: " + obstacle.getFirst().getX() + ", " +
                // obstacle.getFirst().getY() + " with size: " + obstacle.getSecond().getX());
                // }

                Thread.sleep(50);
            } catch (InterruptedException e) {
                System.err.println("Dynamic obstacle update thread interrupted: " + e.getMessage());
                break;
            }
        }
    }

    /**
     * Parse dynamic obstacles from a JSON string.
     *
     * @param obstacleData JSON string of obstacles.
     * @return A list of pairs of Translation2d (top‐left and bottom‐right corners).
     */
    private List<Pair<Translation2d, Translation2d>> parseDynamicObstacles(String obstacleData) {
        List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
        try {
            JSONArray obstacleArray = (JSONArray) new JSONParser().parse(obstacleData);
            for (Object obj : obstacleArray) {
                if (obj instanceof JSONObject) {
                    JSONObject obstacle = (JSONObject) obj;
                    double x = ((Number) obstacle.get("x")).doubleValue();
                    double y = ((Number) obstacle.get("y")).doubleValue();
                    double size = ((Number) obstacle.get("size")).doubleValue();

                    Translation2d topLeft = new Translation2d(x, y);
                    Translation2d bottomRight = new Translation2d(x + size, y + size);
                    obstacles.add(Pair.of(topLeft, bottomRight));
                } else {
                    System.err.println("Skipping invalid obstacle: " + obj);
                }
            }
        } catch (Exception e) {
            System.err.println("Failed to parse dynamic obstacle data: " + e.getMessage());
            e.printStackTrace();
        }
        return obstacles;
    }

    @Override
    public boolean isNewPathAvailable() {
        return newPathAvailable;
    }

    /**
     * Returns the most recently calculated path.
     *
     * @param constraints  Path constraints.
     * @param goalEndState Goal end state.
     * @return The current PathPlannerPath (built from waypoints).
     */
    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        List<Waypoint> waypoints;
        pathLock.readLock().lock();
        waypoints = new ArrayList<>(currentWaypoints);
        pathLock.readLock().unlock();

        newPathAvailable = false;

        // Require at least two waypoints
        if (waypoints.size() < 2) {
            return null;
        }

        return new PathPlannerPath(waypoints, constraints, null, goalEndState);
    }

    /**
     * Set the start position (from which to pathfind). If the provided position is
     * inside an obstacle,
     * the closest non‐obstacle node is chosen.
     */
    @Override
    public void setStartPosition(Translation2d startPosition) {
        // Use the current robot position from the drive subsystem
        Translation2d currentRobotPosition = driveSubsystem.getPose().getTranslation();
        GridPosition startPos = findClosestNonObstacle(getGridPos(currentRobotPosition), requestObstacles);

        if (startPos != null && !startPos.equals(requestStart)) {
            requestLock.writeLock().lock();
            requestStart = startPos;
            requestRealStartPos = startPosition;
            requestMinor = true;
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Set the goal position (to which to pathfind). If the provided position is
     * inside an obstacle,
     * the closest non‐obstacle node is chosen.
     */
    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), requestObstacles);
        if (gridPos != null) {
            requestLock.writeLock().lock();
            requestGoal = gridPos;
            requestRealGoalPos = goalPosition;
            requestMinor = true;
            requestMajor = true;
            requestReset = true;
            requestLock.writeLock().unlock();
        }
    }

    /**
     * Update the dynamic obstacles used for planning.
     *
     * @param obs             List of obstacle bounding boxes.
     * @param currentRobotPos The current robot position.
     */
    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
        Set<GridPosition> newObs = new HashSet<>();
        for (var obstacle : obs) {
            var gridPos1 = getGridPos(obstacle.getFirst());
            var gridPos2 = getGridPos(obstacle.getSecond());

            int minX = Math.min(gridPos1.x(), gridPos2.x());
            int maxX = Math.max(gridPos1.x(), gridPos2.x());
            int minY = Math.min(gridPos1.y(), gridPos2.y());
            int maxY = Math.max(gridPos1.y(), gridPos2.y());

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    newObs.add(new GridPosition(x, y));
                }
            }
        }

        dynamicObstacles.clear();
        dynamicObstacles.addAll(newObs);
        requestLock.writeLock().lock();
        requestObstacles.clear();
        requestObstacles.addAll(staticObstacles);
        requestObstacles.addAll(dynamicObstacles);
        requestLock.writeLock().unlock();

        // Update start position based on current robot position
        setStartPosition(currentRobotPos);

        // If any node in the current path now collides, force a replan.
        boolean recalculate = false;
        pathLock.readLock().lock();
        for (GridPosition pos : currentPathFull) {
            if (requestObstacles.contains(pos)) {
                recalculate = true;
                break;
            }
        }
        pathLock.readLock().unlock();

        if (recalculate) {
            // In the new version, both start and goal are updated
            setStartPosition(currentRobotPos);
            setGoalPosition(requestRealGoalPos);
        }
    }

    @SuppressWarnings("BusyWait")
    private void runThread() {
        while (true) {
            try {
                requestLock.readLock().lock();
                boolean reset = requestReset;
                boolean minor = requestMinor;
                boolean major = requestMajor;
                GridPosition start = requestStart;
                Translation2d realStart = requestRealStartPos;
                GridPosition goal = requestGoal;
                Translation2d realGoal = requestRealGoalPos;
                Set<GridPosition> obstacles = new HashSet<>(requestObstacles);

                if (reset) {
                    requestReset = false;
                }
                if (minor) {
                    requestMinor = false;
                } else if (major && (eps - 0.5) <= 1.0) {
                    requestMajor = false;
                }
                requestLock.readLock().unlock();

                if (reset || minor || major) {
                    doWork(reset, minor, major, start, goal, realStart, realGoal, obstacles);
                } else {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            } catch (Exception e) {
                requestLock.writeLock().lock();
                requestReset = true;
                requestLock.writeLock().unlock();
            }
        }
    }

    private void doWork(boolean needsReset, boolean doMinor, boolean doMajor,
            GridPosition sStart, GridPosition sGoal,
            Translation2d realStartPos, Translation2d realGoalPos,
            Set<GridPosition> obstacles) {
        // If a path command is already running, skip replanning.
        if (driveSubsystem.isPathCommandRunning()) {
            System.out.println("[NetworkTablesADStar] Path command is running. Skipping replanning.");
            return;
        }

        if (needsReset) {
            reset(sStart, sGoal);
        }

        if (doMinor) {
            computeOrImprovePath(sStart, sGoal, obstacles);
            List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles);
            List<Waypoint> waypoints = createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);

            pathLock.writeLock().lock();
            currentPathFull = pathPositions;
            currentWaypoints = waypoints;
            pathLock.writeLock().unlock();

            newPathAvailable = true;
        } else if (doMajor) {
            if (eps > 1.0) {
                eps -= 0.5;
                open.putAll(incons);
                open.replaceAll((s, v) -> key(s, sStart));
                closed.clear();
                computeOrImprovePath(sStart, sGoal, obstacles);
                List<GridPosition> pathPositions = extractPath(sStart, sGoal, obstacles);
                List<Waypoint> waypoints = createWaypoints(pathPositions, realStartPos, realGoalPos, obstacles);

                pathLock.writeLock().lock();
                currentPathFull = pathPositions;
                currentWaypoints = waypoints;
                pathLock.writeLock().unlock();

                newPathAvailable = true;
            }
        }
    }

    private List<GridPosition> extractPath(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (sGoal.equals(sStart)) {
            return new ArrayList<>();
        }

        List<GridPosition> path = new ArrayList<>();
        path.add(sStart);
        GridPosition s = sStart;

        // Get current robot rotation to use for collision checks
        Rotation2d robotRotation = driveSubsystem.getPose().getRotation();

        for (int k = 0; k < 200; k++) {
            HashMap<GridPosition, Double> gList = new HashMap<>();
            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                // Check if robot can actually fit at this position with its dimensions
                Pose2d potentialRobotPose = new Pose2d(gridPosToTranslation2d(x), robotRotation);
                if (!checkRobotCollision(potentialRobotPose, obstacles)) {
                    gList.put(x, g.get(x));
                }
            }

            if (gList.isEmpty()) {
                System.out.println("WARNING: No valid neighbors found during path extraction. Path might be invalid.");
                // Try again but ignore robot dimensions as a fallback
                for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                    gList.put(x, g.get(x));
                }
            }

            Map.Entry<GridPosition, Double> min = Map.entry(sGoal, Double.POSITIVE_INFINITY);
            for (var entry : gList.entrySet()) {
                if (entry.getValue() < min.getValue()) {
                    min = entry;
                }
            }

            s = min.getKey();
            path.add(s);

            // Update robot rotation based on path direction
            if (path.size() >= 2) {
                GridPosition prev = path.get(path.size() - 2);
                GridPosition curr = path.get(path.size() - 1);
                Translation2d prevT = gridPosToTranslation2d(prev);
                Translation2d currT = gridPosToTranslation2d(curr);
                robotRotation = new Rotation2d(currT.getX() - prevT.getX(), currT.getY() - prevT.getY());
            }

            if (s.equals(sGoal)) {
                break;
            }
        }
        return path;
    }

    /**
     * Create waypoints from the raw path (a list of grid positions) by first
     * simplifying
     * the path and then generating a sequence of Pose2d objects (using smoothing)
     * which are converted to waypoints.
     */
    private List<Waypoint> createWaypoints(List<GridPosition> path,
            Translation2d realStartPos,
            Translation2d realGoalPos,
            Set<GridPosition> obstacles) {
        if (path.isEmpty()) {
            return new ArrayList<>();
        }

        List<GridPosition> simplifiedPath = new ArrayList<>();
        simplifiedPath.add(path.get(0));
        for (int i = 1; i < path.size() - 1; i++) {
            if (!walkable(simplifiedPath.get(simplifiedPath.size() - 1), path.get(i + 1), obstacles)) {
                simplifiedPath.add(path.get(i));
            }
        }
        simplifiedPath.add(path.get(path.size() - 1));

        List<Translation2d> fieldPosPath = new ArrayList<>();
        for (GridPosition pos : simplifiedPath) {
            fieldPosPath.add(gridPosToTranslation2d(pos));
        }

        if (fieldPosPath.size() < 2) {
            return new ArrayList<>();
        }

        // Replace the start and end positions with their real (possibly
        // non‐grid–aligned) values.
        fieldPosPath.set(0, realStartPos);
        fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos);

        List<Pose2d> pathPoses = new ArrayList<>();
        pathPoses.add(new Pose2d(fieldPosPath.get(0),
                fieldPosPath.get(1).minus(fieldPosPath.get(0)).getAngle()));
        for (int i = 1; i < fieldPosPath.size() - 1; i++) {
            Translation2d last = fieldPosPath.get(i - 1);
            Translation2d current = fieldPosPath.get(i);
            Translation2d next = fieldPosPath.get(i + 1);

            Translation2d anchor1 = current.minus(last).times(SMOOTHING_ANCHOR_PCT).plus(last);
            Rotation2d heading1 = current.minus(last).getAngle();
            Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);
            Rotation2d heading2 = next.minus(anchor2).getAngle();

            pathPoses.add(new Pose2d(anchor1, heading1));
            pathPoses.add(new Pose2d(anchor2, heading2));
        }
        pathPoses.add(new Pose2d(fieldPosPath.get(fieldPosPath.size() - 1),
                fieldPosPath.get(fieldPosPath.size() - 1)
                        .minus(fieldPosPath.get(fieldPosPath.size() - 2)).getAngle()));

        return PathPlannerPath.waypointsFromPoses(pathPoses);
    }

    /**
     * Method to check if a path position would cause the robot to collide with
     * obstacles.
     * This takes into account the robot's dimensions and orientation.
     * 
     * @param robotPose The pose of the robot to check for collision
     * @param obstacles The set of obstacle grid positions
     * @return true if there is a collision, false otherwise
     */
    private boolean checkRobotCollision(Pose2d robotPose, Set<GridPosition> obstacles) {
        // Robot dimensions in meters (adjust these to match your robot)
        final double ROBOT_LENGTH = 0.75; // Robot length in meters
        final double ROBOT_WIDTH = 0.75; // Robot width in meters

        // Calculate corners of the robot rectangle based on current position and
        // rotation
        Translation2d robotPos = robotPose.getTranslation();
        Rotation2d robotRot = robotPose.getRotation();

        // Calculate the four corners of the robot
        Translation2d[] corners = new Translation2d[4];

        // Half dimensions for calculating corners
        double halfLength = ROBOT_LENGTH / 2.0;
        double halfWidth = ROBOT_WIDTH / 2.0;

        // Front-left corner
        corners[0] = robotPos.plus(
                new Translation2d(halfLength, halfWidth).rotateBy(robotRot));

        // Front-right corner
        corners[1] = robotPos.plus(
                new Translation2d(halfLength, -halfWidth).rotateBy(robotRot));

        // Back-right corner
        corners[2] = robotPos.plus(
                new Translation2d(-halfLength, -halfWidth).rotateBy(robotRot));

        // Back-left corner
        corners[3] = robotPos.plus(
                new Translation2d(-halfLength, halfWidth).rotateBy(robotRot));

        // Check if any of the grid cells covered by the robot rectangle intersect with
        // obstacles
        // Create a bounding box to efficiently check only relevant grid cells
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (Translation2d corner : corners) {
            minX = Math.min(minX, corner.getX());
            maxX = Math.max(maxX, corner.getX());
            minY = Math.min(minY, corner.getY());
            maxY = Math.max(maxY, corner.getY());
        }

        // Convert to grid cells and add some margin
        int minGridX = (int) Math.floor(minX / nodeSize) - 1;
        int maxGridX = (int) Math.ceil(maxX / nodeSize) + 1;
        int minGridY = (int) Math.floor(minY / nodeSize) - 1;
        int maxGridY = (int) Math.ceil(maxY / nodeSize) + 1;

        // Create polygon from robot corners for intersection tests
        List<Translation2d> robotPolygon = Arrays.asList(corners);

        // Check grid cells in the bounding box
        for (int x = minGridX; x <= maxGridX; x++) {
            for (int y = minGridY; y <= maxGridY; y++) {
                GridPosition pos = new GridPosition(x, y);
                if (obstacles.contains(pos)) {
                    // Calculate the four corners of this grid cell
                    Translation2d cellCenter = gridPosToTranslation2d(pos);
                    Translation2d[] cellCorners = new Translation2d[4];
                    cellCorners[0] = new Translation2d(cellCenter.getX() - nodeSize / 2,
                            cellCenter.getY() - nodeSize / 2);
                    cellCorners[1] = new Translation2d(cellCenter.getX() + nodeSize / 2,
                            cellCenter.getY() - nodeSize / 2);
                    cellCorners[2] = new Translation2d(cellCenter.getX() + nodeSize / 2,
                            cellCenter.getY() + nodeSize / 2);
                    cellCorners[3] = new Translation2d(cellCenter.getX() - nodeSize / 2,
                            cellCenter.getY() + nodeSize / 2);

                    // Check if robot polygon intersects with cell polygon
                    if (doPolygonsIntersect(robotPolygon, Arrays.asList(cellCorners))) {
                        return true;
                    }
                }
            }
        }

        // Logging comprehensive information about collision checks
        boolean collisionDetected = false;
        // System.out.println("[DEBUG] Checking robot collision at pose: " +
        //         "x=" + robotPose.getX() +
        //         ", y=" + robotPose.getY() +
        //         ", rot=" + robotPose.getRotation().getDegrees() + "°");
        // System.out.println("[DEBUG] Robot dimensions: " + ROBOT_LENGTH + "m x " + ROBOT_WIDTH + "m");

        // // Log the robot's corner positions for debugging
        // System.out.println("[DEBUG] Robot corners:");
        // for (int i = 0; i < corners.length; i++) {
        //     System.out.println("[DEBUG]   Corner " + i + ": (" +
        //             corners[i].getX() + ", " + corners[i].getY() + ")");
        // }

        // // Detailed logging about the bounding box
        // System.out.println("[DEBUG] Checking grid cells in bounding box: (" +
        //         minGridX + "," + minGridY + ") to (" +
        //         maxGridX + "," + maxGridY + ")");
        // System.out.println("[DEBUG] Total obstacles to check against: " + obstacles.size());

        int cellsChecked = 0;
        int potentialCollisions = 0;

        // Check grid cells in the bounding box with more detailed logging
        for (int x = minGridX; x <= maxGridX; x++) {
            for (int y = minGridY; y <= maxGridY; y++) {
                cellsChecked++;
                GridPosition pos = new GridPosition(x, y);

                if (obstacles.contains(pos)) {
                    potentialCollisions++;
                    // Calculate the four corners of this grid cell
                    Translation2d cellCenter = gridPosToTranslation2d(pos);
                    Translation2d[] cellCorners = new Translation2d[4];
                    cellCorners[0] = new Translation2d(cellCenter.getX() - nodeSize / 2,
                            cellCenter.getY() - nodeSize / 2);
                    cellCorners[1] = new Translation2d(cellCenter.getX() + nodeSize / 2,
                            cellCenter.getY() - nodeSize / 2);
                    cellCorners[2] = new Translation2d(cellCenter.getX() + nodeSize / 2,
                            cellCenter.getY() + nodeSize / 2);
                    cellCorners[3] = new Translation2d(cellCenter.getX() - nodeSize / 2,
                            cellCenter.getY() + nodeSize / 2);

                    // System.out.println("[DEBUG] Checking obstacle at grid: (" + x + "," + y +
                    // "), world: (" + cellCenter.getX() + "," + cellCenter.getY() + ")");

                    // Check if robot polygon intersects with cell polygon
                    if (doPolygonsIntersect(robotPolygon, Arrays.asList(cellCorners))) {
                        // System.out.println("[DEBUG] COLLISION DETECTED with obstacle at (" +
                        //         x + "," + y + ")!");
                        // System.out.println("[DEBUG] Distance from robot center to obstacle: " +
                        //         robotPos.getDistance(cellCenter) + "m");
                        collisionDetected = true;
                        return true;
                    }
                }
            }
        }

        // System.out.println("[DEBUG] Collision check complete - Checked " + cellsChecked +
        //         " cells, " + potentialCollisions + " potential collisions, " +
        //         "Result: " + (collisionDetected ? "COLLISION" : "NO COLLISION"));

        return false;
    }

    /**
     * Check if two convex polygons intersect
     */
    private boolean doPolygonsIntersect(List<Translation2d> poly1, List<Translation2d> poly2) {
        // Using Separating Axis Theorem for convex polygons

        // Check all edges of poly1
        for (int i = 0; i < poly1.size(); i++) {
            int j = (i + 1) % poly1.size();
            Translation2d edge = new Translation2d(
                    poly1.get(j).getX() - poly1.get(i).getX(),
                    poly1.get(j).getY() - poly1.get(i).getY());

            // Get perpendicular axis
            Translation2d axis = new Translation2d(-edge.getY(), edge.getX());

            // Project polygons onto axis
            double[] p1Interval = projectPolygon(poly1, axis);
            double[] p2Interval = projectPolygon(poly2, axis);

            // Check for separation
            if (p1Interval[1] < p2Interval[0] || p2Interval[1] < p1Interval[0]) {
                return false; // Found a separating axis
            }
        }

        // Check all edges of poly2
        for (int i = 0; i < poly2.size(); i++) {
            int j = (i + 1) % poly2.size();
            Translation2d edge = new Translation2d(
                    poly2.get(j).getX() - poly2.get(i).getX(),
                    poly2.get(j).getY() - poly2.get(i).getY());

            // Get perpendicular axis
            Translation2d axis = new Translation2d(-edge.getY(), edge.getX());

            // Project polygons onto axis
            double[] p1Interval = projectPolygon(poly1, axis);
            double[] p2Interval = projectPolygon(poly2, axis);

            // Check for separation
            if (p1Interval[1] < p2Interval[0] || p2Interval[1] < p1Interval[0]) {
                return false; // Found a separating axis
            }
        }

        // No separating axis found, polygons intersect
        return true;
    }

    /**
     * Project a polygon onto an axis and return the min/max interval
     */
    private double[] projectPolygon(List<Translation2d> polygon, Translation2d axis) {
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        double axisLengthSquared = axis.getX() * axis.getX() + axis.getY() * axis.getY();

        for (Translation2d point : polygon) {
            // Calculate dot product
            double dotProduct = (point.getX() * axis.getX() + point.getY() * axis.getY()) /
                    Math.sqrt(axisLengthSquared);

            min = Math.min(min, dotProduct);
            max = Math.max(max, dotProduct);
        }

        return new double[] { min, max };
    }

    /**
     * Check if a path between two grid positions is walkable (no obstacles)
     * including checking for robot collision
     */
    private boolean walkable(GridPosition s1, GridPosition s2, Set<GridPosition> obstacles) {
        int x0 = s1.x();
        int y0 = s1.y();
        int x1 = s2.x();
        int y1 = s2.y();

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int x = x0;
        int y = y0;
        int n = 1 + dx + dy;
        int xInc = (x1 > x0) ? 1 : -1;
        int yInc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        // Calculate path direction for robot orientation
        Translation2d startT = gridPosToTranslation2d(s1);
        Translation2d endT = gridPosToTranslation2d(s2);
        Rotation2d pathDirection = new Rotation2d(endT.getX() - startT.getX(), endT.getY() - startT.getY());

        for (; n > 0; n--) {
            GridPosition currentPos = new GridPosition(x, y);

            // Basic obstacle check
            if (obstacles.contains(currentPos)) {
                return false;
            }

            // Check for robot collision at this position
            Translation2d currentT = gridPosToTranslation2d(currentPos);
            Pose2d robotPose = new Pose2d(currentT, pathDirection);
            if (checkRobotCollision(robotPose, obstacles)) {
                return false;
            }

            if (error > 0) {
                x += xInc;
                error -= dy;
            } else if (error < 0) {
                y += yInc;
                error += dx;
            } else {
                x += xInc;
                y += yInc;
                error -= dy;
                error += dx;
                n--;
            }
        }
        return true;
    }

    private void reset(GridPosition sStart, GridPosition sGoal) {
        g.clear();
        rhs.clear();
        open.clear();
        incons.clear();
        closed.clear();

        for (int x = 0; x < nodesX; x++) {
            for (int y = 0; y < nodesY; y++) {
                GridPosition pos = new GridPosition(x, y);
                g.put(pos, Double.POSITIVE_INFINITY);
                rhs.put(pos, Double.POSITIVE_INFINITY);
            }
        }
        rhs.put(sGoal, 0.0);
        eps = EPS;
        open.put(sGoal, key(sGoal, sStart));
    }

    private void computeOrImprovePath(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        while (true) {
            var sv = topKey();
            if (sv == null) {
                break;
            }
            var s = sv.getFirst();
            var v = sv.getSecond();

            if (comparePair(v, key(sStart, sStart)) >= 0 && rhs.get(sStart).equals(g.get(sStart))) {
                break;
            }

            open.remove(s);
            if (g.get(s) > rhs.get(s)) {
                g.put(s, rhs.get(s));
                closed.add(s);
                for (GridPosition x : getAllNeighbors(s)) {
                    updateState(x, sStart, sGoal, obstacles);
                }
            } else {
                g.put(s, Double.POSITIVE_INFINITY);
                updateState(s, sStart, sGoal, obstacles);
                for (GridPosition x : getAllNeighbors(s)) {
                    updateState(x, sStart, sGoal, obstacles);
                }
            }
        }
    }

    /**
     * Checks if a movement would cause a collision using robot dimensions
     */
    private boolean isCollision(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
        // Basic grid cell collision check
        if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
            return true;
        }

        // Get current robot heading or use heading from start to end position
        Rotation2d heading = driveSubsystem.getPose().getRotation();
        if (sStart.x() != sEnd.x() || sStart.y() != sEnd.y()) {
            Translation2d startT = gridPosToTranslation2d(sStart);
            Translation2d endT = gridPosToTranslation2d(sEnd);
            heading = new Rotation2d(endT.getX() - startT.getX(), endT.getY() - startT.getY());
        }

        // Check collision with robot dimensions at current position
        Pose2d robotPose = new Pose2d(gridPosToTranslation2d(sStart), heading);
        if (checkRobotCollision(robotPose, obstacles)) {
            return true;
        }

        return false;
    }

    /**
     * Gets neighbors that are valid for movement (no obstacles, considers robot
     * dimensions)
     */
    private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
        List<GridPosition> ret = new ArrayList<>();

        // Get current robot rotation
        Rotation2d robotRotation = driveSubsystem.getPose().getRotation();

        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                // Skip the center cell (no movement)
                if (xMove == 0 && yMove == 0)
                    continue;

                GridPosition sNext = new GridPosition(s.x() + xMove, s.y() + yMove);
                if (!obstacles.contains(sNext)
                        && sNext.x() >= 0 && sNext.x() < nodesX
                        && sNext.y() >= 0 && sNext.y() < nodesY) {
                    ret.add(sNext);
                }
            }
        }
        return ret;
    }

    private void updateState(GridPosition s, GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (!s.equals(sGoal)) {
            rhs.put(s, Double.POSITIVE_INFINITY);
            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                rhs.put(s, Math.min(rhs.get(s), g.get(x) + cost(s, x, obstacles)));
            }
        }

        open.remove(s);
        if (!g.get(s).equals(rhs.get(s))) {
            if (!closed.contains(s)) {
                open.put(s, key(s, sStart));
            } else {
                incons.put(s, Pair.of(0.0, 0.0));
            }
        }
    }

    private double cost(GridPosition sStart, GridPosition sGoal, Set<GridPosition> obstacles) {
        if (isCollision(sStart, sGoal, obstacles)) {
            return Double.POSITIVE_INFINITY;
        }
        return heuristic(sStart, sGoal);
    }

    private List<GridPosition> getAllNeighbors(GridPosition s) {
        List<GridPosition> ret = new ArrayList<>();
        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
                GridPosition sNext = new GridPosition(s.x() + xMove, s.y() + yMove);
                if (sNext.x() >= 0 && sNext.x() < nodesX && sNext.y() >= 0 && sNext.y() < nodesY) {
                    ret.add(sNext);
                }
            }
        }
        return ret;
    }

    private Pair<Double, Double> key(GridPosition s, GridPosition sStart) {
        if (g.get(s) > rhs.get(s)) {
            return Pair.of(rhs.get(s) + eps * heuristic(sStart, s), rhs.get(s));
        } else {
            return Pair.of(g.get(s) + heuristic(sStart, s), g.get(s));
        }
    }

    private Pair<GridPosition, Pair<Double, Double>> topKey() {
        Map.Entry<GridPosition, Pair<Double, Double>> min = null;
        for (var entry : open.entrySet()) {
            if (min == null || comparePair(entry.getValue(), min.getValue()) < 0) {
                min = entry;
            }
        }
        if (min == null) {
            return null;
        }
        return Pair.of(min.getKey(), min.getValue());
    }

    private double heuristic(GridPosition sStart, GridPosition sGoal) {
        return Math.hypot(sGoal.x() - sStart.x(), sGoal.y() - sStart.y());
    }

    private int comparePair(Pair<Double, Double> a, Pair<Double, Double> b) {
        int first = Double.compare(a.getFirst(), b.getFirst());
        if (first == 0) {
            return Double.compare(a.getSecond(), b.getSecond());
        } else {
            return first;
        }
    }

    private GridPosition getGridPos(Translation2d pos) {
        int x = (int) Math.floor(pos.getX() / nodeSize);
        int y = (int) Math.floor(pos.getY() / nodeSize);
        return new GridPosition(x, y);
    }

    private Translation2d gridPosToTranslation2d(GridPosition pos) {
        return new Translation2d((pos.x() * nodeSize) + (nodeSize / 2.0),
                (pos.y() * nodeSize) + (nodeSize / 2.0));
    }

    /**
     * Loads obstacles from the navgrid.json file in polygon format.
     */
    private void loadObstaclesFromNavGrid() {
        String navGridPath = "src/main/java/frc/robot/subsystems/pathplanning/navgrid.json";
        File navGridFile = new File(navGridPath);

        if (!navGridFile.exists()) {
            navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
            if (!navGridFile.exists()) {
                System.out.println("WARNING: navgrid.json not found at either: " + navGridPath +
                        " or " + Filesystem.getDeployDirectory() + "/pathplanner/navgrid.json");
                return;
            }
        }

        System.out.println("Loading obstacles from " + navGridFile.getAbsolutePath());

        try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }
            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            // Process polygons for static obstacles
            if (json.containsKey("polygons")) {
                JSONArray polygons = (JSONArray) json.get("polygons");
                System.out.println("Found " + polygons.size() + " polygons in navgrid.json");

                for (Object polyObj : polygons) {
                    JSONObject polygon = (JSONObject) polyObj;
                    boolean isBarrier = false;

                    // Check for barrier type
                    if (polygon.containsKey("type") && "barrier".equals(polygon.get("type"))) {
                        isBarrier = true;
                    }

                    // Process polygon points
                    if (polygon.containsKey("points")) {
                        JSONArray points = (JSONArray) polygon.get("points");
                        if (points != null && !points.isEmpty()) {
                            List<Translation2d> polyPoints = new ArrayList<>();

                            for (Object pointObj : points) {
                                JSONArray point = (JSONArray) pointObj;
                                if (point != null && point.size() >= 2) {
                                    double x = ((Number) point.get(0)).doubleValue();
                                    double y = ((Number) point.get(1)).doubleValue();
                                    polyPoints.add(new Translation2d(x, y));
                                }
                            }

                            if (polyPoints.size() >= 3) {
                                staticObstaclePolygons.add(polyPoints);
                                System.out.println("Added " + (isBarrier ? "barrier" : "obstacle") +
                                        " polygon with " + polyPoints.size() + " points");

                                // Convert polygon to grid positions
                                addPolygonToObstacles(polyPoints, staticObstacles);
                            }
                        }
                    }
                }
            }

            System.out.println("Loaded " + staticObstaclePolygons.size() + " polygon obstacles with " +
                    staticObstacles.size() + " grid positions");

        } catch (Exception e) {
            System.err.println("Error loading navgrid file: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Converts a polygon to grid positions and adds them to the obstacle set.
     */
    private void addPolygonToObstacles(List<Translation2d> polyPoints, Set<GridPosition> obstacleSet) {
        if (polyPoints.size() < 3) {
            return; // Not a valid polygon
        }

        // Find bounding box of the polygon
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;

        for (Translation2d point : polyPoints) {
            minX = Math.min(minX, point.getX());
            maxX = Math.max(maxX, point.getX());
            minY = Math.min(minY, point.getY());
            maxY = Math.max(maxY, point.getY());
        }

        // Convert to grid positions
        int minGridX = (int) Math.floor(minX / nodeSize);
        int maxGridX = (int) Math.ceil(maxX / nodeSize);
        int minGridY = (int) Math.floor(minY / nodeSize);
        int maxGridY = (int) Math.ceil(maxY / nodeSize);

        // Check all grid positions in the bounding box
        for (int x = minGridX; x <= maxGridX; x++) {
            for (int y = minGridY; y <= maxGridY; y++) {
                Translation2d cellCenter = gridPosToTranslation2d(new GridPosition(x, y));

                if (isPointInPolygon(cellCenter, polyPoints)) {
                    obstacleSet.add(new GridPosition(x, y));
                }
            }
        }
    }

    /**
     * Check if a point is inside a polygon using the ray-casting algorithm.
     */
    private boolean isPointInPolygon(Translation2d point, List<Translation2d> polygon) {
        boolean inside = false;
        int i, j;

        for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            Translation2d vertI = polygon.get(i);
            Translation2d vertJ = polygon.get(j);

            if (((vertI.getY() > point.getY()) != (vertJ.getY() > point.getY())) &&
                    (point.getX() < (vertJ.getX() - vertI.getX()) * (point.getY() - vertI.getY()) /
                            (vertJ.getY() - vertI.getY()) + vertI.getX())) {
                inside = !inside;
            }
        }

        return inside;
    }

    /**
     * Thread that publishes the robot's pose to NetworkTables.
     */
    private void publishRobotPose() {
        System.out.println("Starting robot pose publishing thread");
        while (true) {
            try {
                // Get current robot pose
                Pose2d currentPose = driveSubsystem.getPose();

                // Create a JSON representation of the pose
                JSONObject poseObject = new JSONObject();
                poseObject.put("x", currentPose.getX());
                poseObject.put("y", currentPose.getY());
                poseObject.put("theta", currentPose.getRotation().getDegrees());

                // Publish to NetworkTables
                robotPoseEntry.setString(poseObject.toJSONString());

                // Also publish static obstacles if needed (only need to do this once)
                publishStaticObstacles();

                Thread.sleep(50); // Update at 20Hz
            } catch (InterruptedException e) {
                System.err.println("Robot pose publish thread interrupted: " + e.getMessage());
                break;
            } catch (Exception e) {
                System.err.println("Error in robot pose publish thread: " + e.getMessage());
                e.printStackTrace();
            }
        }
    }

    /**
     * Publishes static obstacles to NetworkTables.
     */
    private void publishStaticObstacles() {
        // Only publish once
        NetworkTableEntry staticObstaclesEntry = dynamicObstacleTable.getEntry("static_obstacles");
        if (staticObstaclesEntry.getString("").isEmpty()) {
            JSONArray obstaclesArray = new JSONArray();

            for (List<Translation2d> polygon : staticObstaclePolygons) {
                JSONObject polygonObject = new JSONObject();
                JSONArray pointsArray = new JSONArray();

                for (Translation2d point : polygon) {
                    JSONArray pointArray = new JSONArray();
                    pointArray.add(point.getX());
                    pointArray.add(point.getY());
                    pointsArray.add(pointArray);
                }

                polygonObject.put("points", pointsArray);
                obstaclesArray.add(polygonObject);
            }

            staticObstaclesEntry.setString(obstaclesArray.toJSONString());
            System.out.println("Published " + staticObstaclePolygons.size() + " static obstacles to NetworkTables");
        }
    }

    /**
     * A grid node represented as a record.
     */
    public record GridPosition(int x, int y) implements Comparable<GridPosition> {
        @Override
        public int compareTo(GridPosition o) {
            if (this.x == o.x) {
                return Integer.compare(this.y, o.y);
            } else {
                return Integer.compare(this.x, o.x);
            }
        }
    }
}
