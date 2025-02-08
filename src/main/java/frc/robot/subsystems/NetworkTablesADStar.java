package frc.robot.subsystems;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
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
 * An AD* pathfinder that runs in a background thread, reading dynamic obstacles from NetworkTables.
 * This version is updated based on the new LocalADStar implementation.
 */
public class NetworkTablesADStar implements Pathfinder {
    private static final double SMOOTHING_ANCHOR_PCT = 0.8;
    private static final double EPS = 2.5;

    private double fieldLength = 16.54;
    private double fieldWidth = 8.02;

    private double nodeSize = 0.2;
    private int nodesX = (int) Math.ceil(fieldLength / nodeSize);
    private int nodesY = (int) Math.ceil(fieldWidth / nodeSize);

    private final HashMap<GridPosition, Double> g = new HashMap<>();
    private final HashMap<GridPosition, Double> rhs = new HashMap<>();
    private final HashMap<GridPosition, Pair<Double, Double>> open = new HashMap<>();
    private final HashMap<GridPosition, Pair<Double, Double>> incons = new HashMap<>();
    private final Set<GridPosition> closed = new HashSet<>();
    private final Set<GridPosition> staticObstacles = new HashSet<>();
    private final Set<GridPosition> dynamicObstacles = new HashSet<>();
    private final Set<GridPosition> requestObstacles = new HashSet<>();

    private GridPosition requestStart;
    private Translation2d requestRealStartPos;
    private GridPosition requestGoal;
    private Translation2d requestRealGoalPos;

    private double eps;

    private final Thread planningThread;
    private final Thread dynamicObstacleThread;
    private boolean requestMinor = true;
    private boolean requestMajor = true;
    private boolean requestReset = true;
    private boolean newPathAvailable = false;

    private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
    private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

    // We now store a list of Waypoints (instead of PathPoints)
    private List<Waypoint> currentWaypoints = new ArrayList<>();
    private List<GridPosition> currentPathFull = new ArrayList<>();

    private final NetworkTable dynamicObstacleTable;
    private final DriveSubsystem driveSubsystem;

    /**
     * Create a new NetworkTablesADStar pathfinder.
     *
     * @param driveSubsystem The drive subsystem used to fetch the robot’s current pose.
     */
    public NetworkTablesADStar(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        planningThread = new Thread(this::runThread);
        dynamicObstacleThread = new Thread(this::updateDynamicObstacles);

        requestStart = new GridPosition(0, 0);
        requestRealStartPos = new Translation2d(0, 0);
        requestGoal = new GridPosition(0, 0);
        requestRealGoalPos = new Translation2d(0, 0);

        staticObstacles.clear();
        dynamicObstacles.clear();

        File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json");
        if (navGridFile.exists()) {
            try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }
                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
                JSONArray grid = (JSONArray) json.get("grid");
                nodesY = grid.size();
                for (int row = 0; row < grid.size(); row++) {
                    JSONArray rowArray = (JSONArray) grid.get(row);
                    if (row == 0) {
                        nodesX = rowArray.size();
                    }
                    for (int col = 0; col < rowArray.size(); col++) {
                        boolean isObstacle = (boolean) rowArray.get(col);
                        if (isObstacle) {
                            staticObstacles.add(new GridPosition(col, row));
                        }
                    }
                }

                JSONObject fieldSize = (JSONObject) json.get("field_size");
                fieldLength = ((Number) fieldSize.get("x")).doubleValue();
                fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        requestObstacles.clear();
        requestObstacles.addAll(staticObstacles);
        requestObstacles.addAll(dynamicObstacles);

        requestReset = true;
        requestMajor = true;
        requestMinor = true;

        newPathAvailable = false;

        dynamicObstacleTable = NetworkTableInstance.getDefault().getTable("pythonData");
        planningThread.setDaemon(true);
        planningThread.setName("ADStar Planning Thread");
        planningThread.start();

        dynamicObstacleThread.setDaemon(true);
        dynamicObstacleThread.setName("Dynamic Obstacle Update Thread");
        dynamicObstacleThread.start();
    }

    /**
     * Periodically update the set of dynamic obstacles from NetworkTables.
     */
    private void updateDynamicObstacles() {
        System.out.println("Starting dynamic obstacle update thread");
        while (true) {
            try {
                String obstacleData = dynamicObstacleTable.getEntry("dynamic_obstacles").getString("[]");
                List<Pair<Translation2d, Translation2d>> dynamicObstacleList = parseDynamicObstacles(obstacleData);
                Translation2d currentRobotPos = driveSubsystem.getPose().getTranslation();

                // Update dynamic obstacles and also update the robot’s start position
                setDynamicObstacles(dynamicObstacleList, currentRobotPos);

                // Print obstacle data for debugging
                for (Pair<Translation2d, Translation2d> obstacle : dynamicObstacleList) {
                    System.out.println("Obstacle at: " + obstacle.getFirst().getX() + ", " +
                                       obstacle.getFirst().getY() + " with size: " + obstacle.getSecond().getX());
                }

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
     * Set the start position (from which to pathfind). If the provided position is inside an obstacle,
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
     * Set the goal position (to which to pathfind). If the provided position is inside an obstacle,
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
        for (int k = 0; k < 200; k++) {
            HashMap<GridPosition, Double> gList = new HashMap<>();
            for (GridPosition x : getOpenNeighbors(s, obstacles)) {
                gList.put(x, g.get(x));
            }
            Map.Entry<GridPosition, Double> min = Map.entry(sGoal, Double.POSITIVE_INFINITY);
            for (var entry : gList.entrySet()) {
                if (entry.getValue() < min.getValue()) {
                    min = entry;
                }
            }
            s = min.getKey();
            path.add(s);
            if (s.equals(sGoal)) {
                break;
            }
        }
        return path;
    }

    /**
     * Create waypoints from the raw path (a list of grid positions) by first simplifying
     * the path and then generating a sequence of Pose2d objects (using smoothing) which are converted to waypoints.
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

        // Replace the start and end positions with their real (possibly non‐grid–aligned) values.
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

    private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
        if (!obstacles.contains(pos)) {
            return pos;
        }
        Set<GridPosition> visited = new HashSet<>();
        Queue<GridPosition> queue = new LinkedList<>(getAllNeighbors(pos));
        while (!queue.isEmpty()) {
            GridPosition check = queue.poll();
            if (!obstacles.contains(check)) {
                return check;
            }
            visited.add(check);
            for (GridPosition neighbor : getAllNeighbors(check)) {
                if (!visited.contains(neighbor) && !queue.contains(neighbor)) {
                    queue.add(neighbor);
                }
            }
        }
        return null;
    }

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

        for (; n > 0; n--) {
            if (obstacles.contains(new GridPosition(x, y))) {
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
                for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
                    updateState(sn, sStart, sGoal, obstacles);
                }
            } else {
                g.put(s, Double.POSITIVE_INFINITY);
                for (GridPosition sn : getOpenNeighbors(s, obstacles)) {
                    updateState(sn, sStart, sGoal, obstacles);
                }
                updateState(s, sStart, sGoal, obstacles);
            }
        }
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

    private boolean isCollision(GridPosition sStart, GridPosition sEnd, Set<GridPosition> obstacles) {
        if (obstacles.contains(sStart) || obstacles.contains(sEnd)) {
            return true;
        }
        if (sStart.x() != sEnd.x() && sStart.y() != sEnd.y()) {
            GridPosition s1;
            GridPosition s2;
            if (sEnd.x() - sStart.x() == sStart.y() - sEnd.y()) {
                s1 = new GridPosition(Math.min(sStart.x(), sEnd.x()), Math.min(sStart.y(), sEnd.y()));
                s2 = new GridPosition(Math.max(sStart.x(), sEnd.x()), Math.max(sStart.y(), sEnd.y()));
            } else {
                s1 = new GridPosition(Math.min(sStart.x(), sEnd.x()), Math.max(sStart.y(), sEnd.y()));
                s2 = new GridPosition(Math.max(sStart.x(), sEnd.x()), Math.min(sStart.y(), sEnd.y()));
            }
            return obstacles.contains(s1) || obstacles.contains(s2);
        }
        return false;
    }

    private List<GridPosition> getOpenNeighbors(GridPosition s, Set<GridPosition> obstacles) {
        List<GridPosition> ret = new ArrayList<>();
        for (int xMove = -1; xMove <= 1; xMove++) {
            for (int yMove = -1; yMove <= 1; yMove++) {
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
