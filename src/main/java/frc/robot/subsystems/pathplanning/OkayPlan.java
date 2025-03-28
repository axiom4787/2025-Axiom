package frc.robot.subsystems.pathplanning;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.MathUtil;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * OkayPlan: A simplified version of the OkayPlan algorithm for PathPlanner
 * This implementation handles static obstacles from a navgrid file and accounts
 * for robot dimensions.
 * Based on the paper: "OkayPlan: Obstacle Kinematics Augmented Dynamic
 * real-time path Planning via particle swarm optimization"
 * Original implementation: https://github.com/XinJingHao/OkayPlan
 */
public class OkayPlan implements Pathfinder {
    // Constants
    private static final double SMOOTHING_ANCHOR_PCT = 0.8;
    private static final double DEFAULT_NODE_SIZE = 0.2;
    private static final int MAX_ITERATIONS = 1000;
    private static final double PATH_SMOOTHING_FACTOR = 0.7;

    // Additional constants based on original OkayPlan algorithm
    private static final double MAX_PATH_ITERATIONS = 3000; // Reduced from original for faster computation
    private static final double PATH_SMOOTHING_WEIGHT = 0.2;
    private static final double OBSTACLE_PENALTY_WEIGHT = 3000.0;
    private static final double MIN_SEGMENT_LENGTH_FACTOR = 0.01;
    private static final boolean USE_AUTO_TRUNCATION = true;
    private static final int TRUNCATION_WINDOW = 10;
    private static final double TRUNCATION_THRESHOLD = 0.1;

    // Original OkayPlan parameters
    private final List<Double> okayPlanFitnessHistory = new ArrayList<>();
    private double bestFitness = Double.POSITIVE_INFINITY;
    private boolean lastPathCollisionFree = false;

    // Path fitness evaluation components
    private double pathLength = 0.0;
    private double obstaclePenalty = 0.0;
    private double dynamicObstaclePenalty = 0.0;

    // Field parameters
    private double fieldLength = 16.54;
    private double fieldWidth = 8.02;
    private double nodeSize = DEFAULT_NODE_SIZE;

    // Robot dimensions
    private final double robotWidth;
    private final double robotLength;

    // Grid dimensions
    private int nodesX;
    private int nodesY;

    // Obstacles
    private final Set<GridPosition> staticObstacles = new HashSet<>();
    private final Set<GridPosition> dynamicObstacles = new HashSet<>();
    private final Set<GridPosition> allObstacles = new HashSet<>();

    // Path planning
    private final Thread planningThread;
    private final ReadWriteLock pathLock = new ReentrantReadWriteLock();
    private final ReadWriteLock requestLock = new ReentrantReadWriteLock();

    // Request parameters
    private GridPosition requestStart;
    private Translation2d requestRealStartPos;
    private Rotation2d requestStartRotation;
    private GridPosition requestGoal;
    private Translation2d requestRealGoalPos;
    private AtomicBoolean newRequestAvailable = new AtomicBoolean(false);
    private AtomicBoolean newPathAvailable = new AtomicBoolean(false);

    // Current path
    private List<Waypoint> currentWaypoints = new ArrayList<>();
    private List<GridPosition> currentPathFull = new ArrayList<>();
    private boolean pathIsCollisionFree = false;

    /**
     * Create a new OkayPlan pathfinder
     * 
     * @param robotWidth  Width of the robot in meters
     * @param robotLength Length of the robot in meters
     */
    public OkayPlan(double robotWidth, double robotLength) {
        this.robotWidth = robotWidth;
        this.robotLength = robotLength;

        // Increase node size for more efficient planning
        this.nodeSize = 0.5; // Increased from DEFAULT_NODE_SIZE

        requestStart = new GridPosition(0, 0);
        requestRealStartPos = Translation2d.kZero;
        requestStartRotation = new Rotation2d();
        requestGoal = new GridPosition(0, 0);
        requestRealGoalPos = Translation2d.kZero;

        // Load navgrid file
        loadNavGrid();

        // Initialize the planning thread
        planningThread = new Thread(this::planningLoop);
        planningThread.setDaemon(true);
        planningThread.setName("OkayPlan Planning Thread");
        planningThread.start();
    }

    /**
     * Load obstacles from the navgrid.json file
     */
    private void loadNavGrid() {
        // Try multiple possible locations for the navgrid file
        File navGridFile = new File("src/main/java/frc/robot/subsystems/pathplanning/navgrid.json");

        if (!navGridFile.exists()) {
            // Try source code location
            navGridFile = new File("src/main/java/frc/robot/subsystems/pathplanning/navgrid.json");

            if (!navGridFile.exists()) {
                // Also try current working directory
                navGridFile = new File(System.getProperty("user.dir"),
                        "src/main/java/frc/robot/subsystems/pathplanning/navgrid.json");

                if (!navGridFile.exists()) {
                    System.out.println("navgrid.json not found in any of the expected locations:");
                    System.out.println("  - "
                            + new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json").getAbsolutePath());
                    System.out.println("  - " + new File("src/main/java/frc/robot/subsystems/pathplanning/navgrid.json")
                            .getAbsolutePath());
                    System.out.println("  - " + new File(System.getProperty("user.dir"),
                            "src/main/java/frc/robot/subsystems/pathplanning/navgrid.json").getAbsolutePath());
                    System.out.println("Using empty grid");

                    // Set default grid dimensions
                    nodesX = (int) Math.ceil(fieldLength / nodeSize);
                    nodesY = (int) Math.ceil(fieldWidth / nodeSize);

                    // Initialize all obstacles with static obstacles (empty in this case)
                    allObstacles.addAll(staticObstacles);
                    return;
                }
            }
        }

        System.out.println("Found navgrid.json at: " + navGridFile.getAbsolutePath());

        try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            // Check if polygons key exists and is not null
            if (json.containsKey("polygons")) {
                JSONArray polygons = (JSONArray) json.get("polygons");

                if (polygons != null) {
                    for (Object polyObj : polygons) {
                        JSONObject polygon = (JSONObject) polyObj;

                        // Process all polygons, including barriers
                        boolean isBarrier = polygon.containsKey("type") && "barrier".equals(polygon.get("type"));

                        // Check if points key exists
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

                                // Only add if we have valid points
                                if (polyPoints.size() >= 3) {
                                    // Add grid positions for this polygon
                                    addPolygonToObstacles(polyPoints);

                                    // Log the polygon being added
                                    String polygonType = isBarrier ? "barrier" : "obstacle";
                                    int polygonId = polygon.containsKey("id") ? ((Number) polygon.get("id")).intValue()
                                            : -1;
                                    System.out.println("Added " + polygonType + " polygon with ID " + polygonId + " ("
                                            + polyPoints.size() + " points)");
                                }
                            }
                        }
                    }

                    System.out.println("Loaded " + staticObstacles.size() + " static obstacle grid positions");
                } else {
                    System.out.println("'polygons' array is null in navgrid.json");
                }
            } else {
                System.out.println("No 'polygons' key found in navgrid.json");
            }

            // Calculate grid dimensions
            nodesX = (int) Math.ceil(fieldLength / nodeSize);
            nodesY = (int) Math.ceil(fieldWidth / nodeSize);

            // After loading all obstacles:
            System.out.println("Initial static obstacle count: " + staticObstacles.size());

            // Limit the number of obstacles if it's too large
            if (staticObstacles.size() > 10000) {
                System.out.println("WARNING: Too many static obstacles, limiting to core obstacles");
                Set<GridPosition> limitedObstacles = new HashSet<>();
                int count = 0;
                for (GridPosition pos : staticObstacles) {
                    // Keep obstacles that are more critical (e.g., near edges, representing
                    // barriers)
                    boolean isNearEdge = pos.x < 5 || pos.y < 5 ||
                            pos.x > nodesX - 5 || pos.y > nodesY - 5;

                    if (isNearEdge || count % 10 == 0) { // Keep 10% of obstacles plus edge ones
                        limitedObstacles.add(pos);
                    }
                    count++;
                }
                staticObstacles.clear();
                staticObstacles.addAll(limitedObstacles);
                System.out.println("Reduced to " + staticObstacles.size() + " obstacles");
            }

            // Initialize all obstacles with static obstacles
            allObstacles.clear();
            allObstacles.addAll(staticObstacles);

        } catch (Exception e) {
            System.err.println("Error loading navgrid: " + e.getMessage());
            e.printStackTrace();

            // Set default grid dimensions
            nodesX = (int) Math.ceil(fieldLength / nodeSize);
            nodesY = (int) Math.ceil(fieldWidth / nodeSize);
        }

        // Initialize all obstacles with static obstacles
        allObstacles.addAll(staticObstacles);
    }

    /**
     * Add all grid positions within a polygon to the obstacles set
     */
    private void addPolygonToObstacles(List<Translation2d> polyPoints) {
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

        // Expand bounding box by robot dimensions but use a more conservative expansion
        // The original was too aggressive in obstacle expansion
        double expandX = robotWidth / 3.0; // More conservative expansion
        double expandY = robotLength / 3.0;

        minX -= expandX;
        maxX += expandX;
        minY -= expandY;
        maxY += expandY;

        // Convert to grid positions
        int minGridX = (int) Math.floor(minX / nodeSize);
        int maxGridX = (int) Math.ceil(maxX / nodeSize);
        int minGridY = (int) Math.floor(minY / nodeSize);
        int maxGridY = (int) Math.ceil(maxY / nodeSize);

        // Limit the size of the grid to process
        int maxCellsToProcess = 1000; // Cap for performance
        if ((maxGridX - minGridX + 1) * (maxGridY - minGridY + 1) > maxCellsToProcess) {
            System.out.println("WARNING: Large polygon area, sampling instead of full checking");
            for (int i = 0; i < maxCellsToProcess; i++) {
                int x = minGridX + (int) (Math.random() * (maxGridX - minGridX + 1));
                int y = minGridY + (int) (Math.random() * (maxGridY - minGridY + 1));

                Translation2d cellCenter = new Translation2d(
                        (x + 0.5) * nodeSize,
                        (y + 0.5) * nodeSize);

                if (isPointInPolygon(cellCenter, polyPoints)) {
                    staticObstacles.add(new GridPosition(x, y));
                }
            }
        } else {
            // Check all grid positions in the bounding box
            for (int x = minGridX; x <= maxGridX; x++) {
                for (int y = minGridY; y <= maxGridY; y++) {
                    // Check if grid cell center is inside the polygon
                    Translation2d cellCenter = new Translation2d(
                            (x + 0.5) * nodeSize,
                            (y + 0.5) * nodeSize);

                    if (isPointInPolygon(cellCenter, polyPoints)) {
                        staticObstacles.add(new GridPosition(x, y));
                    }
                }
            }
        }
    }

    /**
     * Check if a point is inside a polygon using the ray-casting algorithm
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
     * Main planning loop that runs in a separate thread
     */
    private void planningLoop() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                if (newRequestAvailable.get()) {
                    // Get request data
                    requestLock.readLock().lock();
                    GridPosition start = requestStart;
                    Translation2d realStartPos = requestRealStartPos;
                    Rotation2d startRotation = requestStartRotation;
                    GridPosition goal = requestGoal;
                    Translation2d realGoalPos = requestRealGoalPos;
                    Set<GridPosition> obstacles = new HashSet<>(allObstacles);
                    requestLock.readLock().unlock();

                    System.out.println("==== PLANNING NEW PATH ====");
                    System.out.println("Start: " + start + " -> " + realStartPos);
                    System.out.println("Goal: " + goal + " -> " + realGoalPos);
                    System.out.println("Total obstacles: " + obstacles.size());

                    // Plan the path with multiple attempts if needed
                    Pair<List<GridPosition>, Boolean> result = planPathWithRetry(start, goal, obstacles);
                    List<GridPosition> path = result.getFirst();
                    pathIsCollisionFree = result.getSecond();

                    // Create waypoints
                    List<Waypoint> waypoints = createWaypoints(path, realStartPos, realGoalPos, startRotation);

                    System.out.println("Path planning complete.");
                    System.out.println("Path points: " + path.size());
                    System.out.println("Waypoints: " + waypoints.size());
                    System.out.println("Collision free: " + pathIsCollisionFree);
                    if (!pathIsCollisionFree) {
                        System.out.println("WARNING: Path contains obstacles!");
                    }

                    // Update current path
                    pathLock.writeLock().lock();
                    currentPathFull = path;
                    currentWaypoints = waypoints;
                    pathLock.writeLock().unlock();

                    // Mark request as handled and new path available
                    newRequestAvailable.set(false);
                    newPathAvailable.set(true);

                    System.out.println("==== PATH PLANNING FINISHED ====");
                }

                // Sleep to avoid excessive CPU usage
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                System.err.println("Planning thread interrupted: " + e.getMessage());
                break;
            } catch (Exception e) {
                System.err.println("Error in planning thread: " + e.getMessage());
                e.printStackTrace();
            }
        }
    }

    /**
     * Plan a path with multiple attempts and strategies
     */
    private Pair<List<GridPosition>, Boolean> planPathWithRetry(
            GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {

        // First attempt - standard planning
        Pair<List<GridPosition>, Boolean> result = planPath(start, goal, obstacles);

        // If we found a collision-free path, return it
        if (result.getSecond()) {
            return result;
        }

        System.out.println("Initial path contains obstacles, trying alternative approaches...");

        // Second attempt - try with relaxed obstacle checking (increase spacing between
        // nodes)
        System.out.println("Attempt 2: Using sparse grid exploration");
        Set<GridPosition> relaxedObstacles = new HashSet<>();

        // Only keep every other obstacle to create a sparser grid
        int count = 0;
        for (GridPosition pos : obstacles) {
            if (count % 2 == 0) {
                relaxedObstacles.add(pos);
            }
            count++;
        }

        Pair<List<GridPosition>, Boolean> relaxedResult = planPath(start, goal, relaxedObstacles);

        // If successful with relaxed obstacles, check against real obstacles
        if (relaxedResult.getSecond()) {
            List<GridPosition> relaxedPath = relaxedResult.getFirst();
            boolean actuallyCollisionFree = isPathCollisionFree(relaxedPath, obstacles);

            if (actuallyCollisionFree) {
                System.out.println("Found collision-free path with sparse grid approach");
                return Pair.of(relaxedPath, true);
            }
        }

        // Third attempt - try to find a path around obstacles
        System.out.println("Attempt 3: Using waypoint exploration");
        List<GridPosition> waypointPath = findWaypointPath(start, goal, obstacles);
        boolean waypointPathCollisionFree = isPathCollisionFree(waypointPath, obstacles);

        if (waypointPathCollisionFree) {
            System.out.println("Found collision-free path with waypoint exploration");
            return Pair.of(waypointPath, true);
        }

        // Final attempt - generate a direct path with minimum obstacle crossings
        System.out.println("Final attempt: Finding path with minimum obstacle crossings");
        List<GridPosition> minObstaclePath = findMinObstaclePath(start, goal, obstacles);

        // Always mark this as not collision-free so the caller knows
        return Pair.of(minObstaclePath, false);
    }

    /**
     * Find a path using waypoints to avoid obstacles
     */
    private List<GridPosition> findWaypointPath(GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {
        // If start or goal are in obstacles, return a direct path
        if (obstacles.contains(start) || obstacles.contains(goal)) {
            System.out.println("WARNING: Start or goal is in an obstacle, returning direct path");
            List<GridPosition> directPath = new ArrayList<>();
            directPath.add(start);
            directPath.add(goal);
            return directPath;
        }

        // Create a list of waypoints to try
        List<GridPosition> waypoints = new ArrayList<>();
        waypoints.add(start);

        // Add some intermediate waypoints
        int numWaypoints = 3;
        for (int i = 1; i <= numWaypoints; i++) {
            double fraction = (double) i / (numWaypoints + 1);
            int x = (int) (start.x + (goal.x - start.x) * fraction);
            int y = (int) (start.y + (goal.y - start.y) * fraction);
            waypoints.add(new GridPosition(x, y));
        }

        waypoints.add(goal);

        // Try different combinations of waypoints
        List<GridPosition> bestPath = null;
        int minObstacles = Integer.MAX_VALUE;

        for (int i = 0; i < (1 << (waypoints.size() - 2)); i++) {
            List<GridPosition> currentPath = new ArrayList<>();
            currentPath.add(start);

            // Add waypoints based on the bitmask
            for (int j = 0; j < waypoints.size() - 2; j++) {
                if ((i >> j) % 2 == 1) {
                    currentPath.add(waypoints.get(j + 1));
                }
            }

            currentPath.add(goal);

            // Check if the path is valid
            boolean validPath = true;
            for (int j = 0; j < currentPath.size() - 1; j++) {
                if (!isLineOfSightClear(currentPath.get(j), currentPath.get(j + 1), obstacles)) {
                    validPath = false;
                    break;
                }
            }

            if (validPath) {
                // Count obstacles crossed
                int obstaclesCrossed = 0;
                for (int j = 0; j < currentPath.size() - 1; j++) {
                    for (GridPosition obs : obstacles) {
                        if (isLineOfSightClear(currentPath.get(j), obs, obstacles) &&
                                isLineOfSightClear(obs, currentPath.get(j + 1), obstacles)) {
                            obstaclesCrossed++;
                        }
                    }
                }

                // Update best path if needed
                if (obstaclesCrossed < minObstacles) {
                    minObstacles = obstaclesCrossed;
                    bestPath = currentPath;
                }
            }
        }

        if (bestPath != null) {
            return bestPath;
        } else {
            // Fallback to direct path
            System.out.println("WARNING: No valid waypoint path found, falling back to direct path");
            List<GridPosition> directPath = new ArrayList<>();
            directPath.add(start);
            directPath.add(goal);
            return directPath;
        }
    }

    /**
     * Find a path with minimum obstacle crossings
     */
    private List<GridPosition> findMinObstaclePath(GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {
        // For now, just return a direct path
        List<GridPosition> path = new ArrayList<>();
        path.add(start);
        path.add(goal);

        // Try to find intermediate points to minimize obstacle crossings
        int numAttempts = 5;
        List<GridPosition> bestPath = new ArrayList<>(path);
        int bestObstacleCrossings = countObstacleCrossings(path, obstacles);

        for (int i = 0; i < numAttempts; i++) {
            // Create a path with a random intermediate point
            List<GridPosition> attemptPath = new ArrayList<>();
            attemptPath.add(start);

            // Add random waypoint offset from direct line
            int dx = goal.x - start.x;
            int dy = goal.y - start.y;
            int midX = start.x + dx / 2 + (int) ((Math.random() - 0.5) * dx / 2);
            int midY = start.y + dy / 2 + (int) ((Math.random() - 0.5) * dy / 2);
            GridPosition midPoint = new GridPosition(midX, midY);

            attemptPath.add(midPoint);
            attemptPath.add(goal);

            int crossings = countObstacleCrossings(attemptPath, obstacles);
            if (crossings < bestObstacleCrossings) {
                bestPath = new ArrayList<>(attemptPath);
                bestObstacleCrossings = crossings;
            }
        }

        System.out.println("Best path has " + bestObstacleCrossings + " obstacle crossings");
        return bestPath;
    }

    /**
     * Count how many obstacles a path crosses
     */
    private int countObstacleCrossings(List<GridPosition> path, Set<GridPosition> obstacles) {
        int crossings = 0;

        for (int i = 0; i < path.size() - 1; i++) {
            GridPosition start = path.get(i);
            GridPosition end = path.get(i + 1);

            if (!isLineOfSightClear(start, end, obstacles)) {
                crossings++;
            }
        }

        return crossings;
    }

    /**
     * Plan a path using a particle-inspired OkayPlan algorithm
     * This method combines A* for initial path finding with OkayPlan-inspired
     * optimizations
     * 
     * @return Pair of path and collision-free status
     */
    private Pair<List<GridPosition>, Boolean> planPath(
            GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {

        System.out.println("Planning path from " + start + " to " + goal);

        // If start and goal are the same, return a trivial path
        if (start.equals(goal)) {
            List<GridPosition> path = new ArrayList<>();
            path.add(start);
            return Pair.of(path, !obstacles.contains(start));
        }

        // Create multiple waypoints between start and goal to mimic the Priori_Path
        // Similar to Python OkayPlan's Priori_Path_Init
        List<GridPosition> waypointPath = new ArrayList<>();
        waypointPath.add(start);

        // Generate 3-5 intermediate waypoints
        int numWaypoints = Math.min(5, Math.max(3, (int) Math.ceil(heuristic(start, goal) / 3)));
        for (int i = 1; i < numWaypoints - 1; i++) {
            double fraction = (double) i / (numWaypoints - 1);
            int x = (int) (start.x + (goal.x - start.x) * fraction);
            int y = (int) (start.y + (goal.y - start.y) * fraction);
            waypointPath.add(new GridPosition(x, y));
        }
        waypointPath.add(goal);

        // Now use A* to find paths between consecutive waypoints
        List<GridPosition> fullPath = new ArrayList<>();
        fullPath.add(start);

        boolean isFullyCollisionFree = true;

        for (int i = 0; i < waypointPath.size() - 1; i++) {
            GridPosition segStart = waypointPath.get(i);
            GridPosition segEnd = waypointPath.get(i + 1);

            // Skip if already added
            if (i > 0) {
                // Don't add segment start again if it's already the end of the previous segment
                if (fullPath.get(fullPath.size() - 1).equals(segStart)) {
                    // Skip
                } else {
                    fullPath.add(segStart);
                }
            }

            // Only run A* if we're not adjacent
            if (Math.abs(segStart.x - segEnd.x) <= 1 && Math.abs(segStart.y - segEnd.y) <= 1) {
                if (!fullPath.contains(segEnd)) {
                    fullPath.add(segEnd);
                }
                continue;
            }

            Pair<List<GridPosition>, Boolean> segmentResult = findPathSegmentAStar(segStart, segEnd, obstacles);

            List<GridPosition> segmentPath = segmentResult.getFirst();
            boolean isSegmentCollisionFree = segmentResult.getSecond();

            // Add all points except the first (already in the path)
            for (int j = 1; j < segmentPath.size(); j++) {
                if (!fullPath.contains(segmentPath.get(j))) {
                    fullPath.add(segmentPath.get(j));
                }
            }

            if (!isSegmentCollisionFree) {
                isFullyCollisionFree = false;
            }
        }

        // If we have enough points and we found a collision-free path, apply OkayPlan
        // optimization
        if (fullPath.size() >= 3 && isFullyCollisionFree) {
            System.out.println("Found initial collision-free path with " + fullPath.size() + " points, optimizing...");
            return optimizePathOkayPlan(fullPath, obstacles);
        } else if (fullPath.size() >= 2) {
            // We have at least a basic path, smooth it and return
            List<GridPosition> smoothedPath = smoothPath(fullPath, obstacles);
            return Pair.of(smoothedPath, isPathCollisionFree(smoothedPath, obstacles));
        } else {
            // Fallback to direct line
            System.out.println("WARNING: Could not find valid path, falling back to direct line");
            fullPath.clear();
            fullPath.add(start);
            fullPath.add(goal);
            return Pair.of(fullPath, false);
        }
    }

    /**
     * Find initial path using A* algorithm
     */
    private Pair<List<GridPosition>, Boolean> findInitialPathAStar(
            GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {

        // Use A* as our base pathfinding algorithm
        Map<GridPosition, GridPosition> cameFrom = new HashMap<>();
        Map<GridPosition, Double> gScore = new HashMap<>();
        Map<GridPosition, Double> fScore = new HashMap<>();
        PriorityQueue<GridPosition> openSet = new PriorityQueue<>(
                Comparator.comparingDouble(fScore::get));

        gScore.put(start, 0.0);
        fScore.put(start, heuristic(start, goal));
        openSet.add(start);

        int iterations = 0;
        boolean pathFound = false;

        while (!openSet.isEmpty() && iterations < MAX_ITERATIONS) {
            GridPosition current = openSet.poll();

            // Check if we reached the goal
            if (current.equals(goal)) {
                pathFound = true;
                break;
            }

            // Get neighbors
            List<GridPosition> neighbors = getNeighbors(current, obstacles);

            for (GridPosition neighbor : neighbors) {
                double tentativeGScore = gScore.get(current) + distance(current, neighbor);

                if (!gScore.containsKey(neighbor) || tentativeGScore < gScore.get(neighbor)) {
                    cameFrom.put(neighbor, current);
                    gScore.put(neighbor, tentativeGScore);
                    fScore.put(neighbor, tentativeGScore + heuristic(neighbor, goal));

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }

            iterations++;
        }

        // Reconstruct path
        List<GridPosition> path = new ArrayList<>();
        boolean isCollisionFree = true;

        if (pathFound) {
            GridPosition current = goal;
            path.add(current);

            while (!current.equals(start)) {
                current = cameFrom.get(current);
                path.add(0, current);

                // Check if this path segment is collision-free
                if (obstacles.contains(current)) {
                    isCollisionFree = false;
                }
            }
        } else {
            // Path not found, return direct line
            path.add(start);
            path.add(goal);
            isCollisionFree = false;
        }

        return Pair.of(path, isCollisionFree);
    }

    /**
     * Find a path segment using A* algorithm
     */
    private Pair<List<GridPosition>, Boolean> findPathSegmentAStar(
            GridPosition start, GridPosition goal, Set<GridPosition> obstacles) {

        // Use A* as our base pathfinding algorithm
        Map<GridPosition, GridPosition> cameFrom = new HashMap<>();
        Map<GridPosition, Double> gScore = new HashMap<>();
        Map<GridPosition, Double> fScore = new HashMap<>();
        PriorityQueue<GridPosition> openSet = new PriorityQueue<>(
                Comparator.comparingDouble(pos -> fScore.getOrDefault(pos, Double.POSITIVE_INFINITY)));

        gScore.put(start, 0.0);
        fScore.put(start, heuristic(start, goal));
        openSet.add(start);

        int iterations = 0;
        boolean pathFound = false;

        while (!openSet.isEmpty() && iterations < MAX_ITERATIONS) {
            GridPosition current = openSet.poll();

            // Check if we reached the goal
            if (current.equals(goal)) {
                pathFound = true;
                break;
            }

            // Get neighbors
            List<GridPosition> neighbors = getNeighbors(current, obstacles);

            for (GridPosition neighbor : neighbors) {
                double tentativeGScore = gScore.get(current) + distance(current, neighbor);

                if (!gScore.containsKey(neighbor) || tentativeGScore < gScore.get(neighbor)) {
                    cameFrom.put(neighbor, current);
                    gScore.put(neighbor, tentativeGScore);
                    fScore.put(neighbor, tentativeGScore + heuristic(neighbor, goal));

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }

            iterations++;
        }

        // Reconstruct path
        List<GridPosition> path = new ArrayList<>();
        boolean isCollisionFree = true;

        if (pathFound) {
            GridPosition current = goal;
            path.add(current);

            while (!current.equals(start)) {
                current = cameFrom.get(current);
                path.add(0, current);

                // Check if this path segment is collision-free
                if (obstacles.contains(current)) {
                    isCollisionFree = false;
                }
            }
        } else {
            // A* failed, create a simple path by interpolating
            createInterpolatedPath(start, goal, path);
            isCollisionFree = isPathCollisionFree(path, obstacles);
        }

        return Pair.of(path, isCollisionFree);
    }

    /**
     * Create a simple interpolated path between start and end points
     */
    private void createInterpolatedPath(GridPosition start, GridPosition goal, List<GridPosition> path) {
        path.clear();
        path.add(start);

        int dx = Math.abs(goal.x - start.x);
        int dy = Math.abs(goal.y - start.y);
        int sx = start.x < goal.x ? 1 : -1;
        int sy = start.y < goal.y ? 1 : -1;
        int err = dx - dy;

        int x = start.x;
        int y = start.y;

        while (x != goal.x || y != goal.y) {
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }

            path.add(new GridPosition(x, y));
        }
    }

    /**
     * Optimize path using OkayPlan-inspired particle approach
     */
    private Pair<List<GridPosition>, Boolean> optimizePathOkayPlan(
            List<GridPosition> initialPath, Set<GridPosition> obstacles) {

        List<GridPosition> currentBestPath = new ArrayList<>(initialPath);
        List<GridPosition> globalBestPath = new ArrayList<>(initialPath);
        boolean isCollisionFree = isPathCollisionFree(currentBestPath, obstacles);

        // Calculate initial fitness
        double initialFitness = calculatePathFitness(initialPath, obstacles);
        double currentBestFitness = initialFitness;
        double globalBestFitness = initialFitness;

        okayPlanFitnessHistory.clear();
        okayPlanFitnessHistory.add(initialFitness);

        // Setup for auto truncation
        int stableIterations = 0;

        // OkayPlan optimization iterations
        for (int iter = 0; iter < MAX_PATH_ITERATIONS; iter++) {
            // 1. Create path variations (particles)
            List<List<GridPosition>> pathVariations = createPathVariations(currentBestPath, obstacles);

            // 2. Evaluate fitness of each variation
            boolean foundBetter = false;
            for (List<GridPosition> variation : pathVariations) {
                double fitness = calculatePathFitness(variation, obstacles);

                // Update current best if found better
                if (fitness < currentBestFitness) {
                    currentBestPath = new ArrayList<>(variation);
                    currentBestFitness = fitness;
                    foundBetter = true;

                    // Update global best if needed
                    if (fitness < globalBestFitness) {
                        globalBestPath = new ArrayList<>(variation);
                        globalBestFitness = fitness;
                        isCollisionFree = isPathCollisionFree(variation, obstacles);
                    }
                }
            }

            // Record fitness history
            okayPlanFitnessHistory.add(globalBestFitness);

            // Check for auto truncation (early stopping when solution stabilizes)
            if (USE_AUTO_TRUNCATION && okayPlanFitnessHistory.size() > TRUNCATION_WINDOW) {
                if (!foundBetter) {
                    stableIterations++;
                } else {
                    stableIterations = 0;
                }

                // Calculate fitness standard deviation for the window
                double stdDev = calculateStdDev(okayPlanFitnessHistory, TRUNCATION_WINDOW);

                // Stop if solution is stable and collision-free
                if (stableIterations > 3 && isCollisionFree && stdDev < TRUNCATION_THRESHOLD) {
                    System.out.println("OkayPlan: Auto truncation at iteration " + iter +
                            " with stdDev=" + stdDev + ", fitness=" + globalBestFitness);
                    break;
                }
            }
        }

        // Store results for later use
        bestFitness = globalBestFitness;
        lastPathCollisionFree = isCollisionFree;

        return Pair.of(globalBestPath, isCollisionFree);
    }

    /**
     * Create path variations by perturbing waypoints (similar to particles in PSO)
     */
    private List<List<GridPosition>> createPathVariations(List<GridPosition> basePath, Set<GridPosition> obstacles) {
        List<List<GridPosition>> variations = new ArrayList<>();
        int pathSize = basePath.size();

        // Skip if path is too short
        if (pathSize <= 2) {
            variations.add(new ArrayList<>(basePath));
            return variations;
        }

        // Create 8 variations with different perturbation strategies
        for (int varIdx = 0; varIdx < 8; varIdx++) {
            List<GridPosition> variation = new ArrayList<>(basePath);

            // Different perturbation for each variation
            double perturbMagnitude = 1.0 + (varIdx % 4) * 0.5; // 1.0, 1.5, 2.0, 2.5
            boolean useSmoothing = (varIdx / 4) == 0; // First 4 use smoothing

            // Don't modify start and end points
            for (int i = 1; i < pathSize - 1; i++) {
                GridPosition current = variation.get(i);

                // Perturb point
                int dx = (int) (Math.random() * perturbMagnitude * 2 - perturbMagnitude);
                int dy = (int) (Math.random() * perturbMagnitude * 2 - perturbMagnitude);

                GridPosition perturbed = new GridPosition(current.x() + dx, current.y() + dy);

                // Only use valid perturbations (within bounds and not in obstacles)
                if (isValidPosition(perturbed) && !obstacles.contains(perturbed)) {
                    variation.set(i, perturbed);
                }
            }

            // Apply additional path smoothing for some variations
            if (useSmoothing) {
                variation = smoothPath(variation, obstacles);
            }

            variations.add(variation);
        }

        return variations;
    }

    /**
     * Calculate path fitness using OkayPlan-inspired metrics
     */
    private double calculatePathFitness(List<GridPosition> path, Set<GridPosition> obstacles) {
        // 1. Path length component
        double length = calculatePathLength(path);

        // 2. Obstacle penalty component
        double obstaclePenalty = calculateObstaclePenalty(path, obstacles);

        // 3. Path smoothness component (angle changes between segments)
        double smoothnessPenalty = calculatePathSmoothness(path);

        // 4. First segment length penalty (from OkayPlan)
        double firstSegmentPenalty = 0.0;
        if (path.size() > 1) {
            double firstSegLength = distance(path.get(0), path.get(1));
            if (firstSegLength < MIN_SEGMENT_LENGTH_FACTOR) {
                firstSegmentPenalty = (MIN_SEGMENT_LENGTH_FACTOR - firstSegLength) * 2.0;
            }
        }

        // Combined fitness (lower is better)
        double fitness = length +
                OBSTACLE_PENALTY_WEIGHT * obstaclePenalty +
                PATH_SMOOTHING_WEIGHT * smoothnessPenalty +
                firstSegmentPenalty;

        // Store components for reporting
        this.pathLength = length;
        this.obstaclePenalty = obstaclePenalty;

        // Debug fitness calculation
        if (obstaclePenalty > 0) {
            System.out.println("DEBUG: Path has obstacle penalty: " + obstaclePenalty +
                    " (total fitness: " + fitness + ")");
        }

        return fitness;
    }

    /**
     * Calculate total path length
     */
    private double calculatePathLength(List<GridPosition> path) {
        double length = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            length += distance(path.get(i), path.get(i + 1));
        }
        return length;
    }

    /**
     * Calculate obstacle penalty (how many obstacles the path intersects)
     */
    private double calculateObstaclePenalty(List<GridPosition> path, Set<GridPosition> obstacles) {
        double penalty = 0.0;

        // Check each segment for obstacle intersections
        for (int i = 0; i < path.size() - 1; i++) {
            if (!isLineOfSightClear(path.get(i), path.get(i + 1), obstacles)) {
                penalty += 1.0;
            }
        }

        return penalty;
    }

    /**
     * Calculate path smoothness by measuring angle changes
     */
    private double calculatePathSmoothness(List<GridPosition> path) {
        if (path.size() < 3) {
            return 0.0;
        }

        double smoothnessPenalty = 0.0;

        for (int i = 1; i < path.size() - 1; i++) {
            GridPosition prev = path.get(i - 1);
            GridPosition current = path.get(i);
            GridPosition next = path.get(i + 1);

            // Calculate vectors
            double vx1 = current.x() - prev.x();
            double vy1 = current.y() - prev.y();
            double vx2 = next.x() - current.x();
            double vy2 = next.y() - current.y();

            // Calculate angle change using dot product
            double dotProduct = vx1 * vx2 + vy1 * vy2;
            double mag1 = Math.sqrt(vx1 * vx1 + vy1 * vy1);
            double mag2 = Math.sqrt(vx2 * vx2 + vy2 * vy2);

            if (mag1 > 0 && mag2 > 0) {
                double cosAngle = dotProduct / (mag1 * mag2);
                // Clamp to avoid numerical issues
                cosAngle = Math.min(1.0, Math.max(-1.0, cosAngle));

                // Convert to angle in radians (0 = straight, pi = complete reversal)
                double angle = Math.acos(cosAngle);

                // Penalize sharp turns
                smoothnessPenalty += angle;
            }
        }

        return smoothnessPenalty;
    }

    /**
     * Check if a path is completely collision-free
     */
    private boolean isPathCollisionFree(List<GridPosition> path, Set<GridPosition> obstacles) {
        if (path.isEmpty()) {
            return true;
        }

        // Check if any point is in an obstacle
        for (GridPosition pos : path) {
            if (obstacles.contains(pos)) {
                System.out.println("DEBUG: Path point " + pos + " is inside an obstacle!");
                return false;
            }
        }

        // Check each segment
        int pathSegmentsWithObstacles = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            GridPosition start = path.get(i);
            GridPosition end = path.get(i + 1);
            if (!isLineOfSightClear(start, end, obstacles)) {
                pathSegmentsWithObstacles++;
                // Only log a limited number to avoid console spam
                if (pathSegmentsWithObstacles <= 3) {
                    System.out.println("DEBUG: Path segment from " + start + " to " + end + " intersects obstacles!");
                }
            }
        }

        if (pathSegmentsWithObstacles > 0) {
            double percentObstructed = (double) pathSegmentsWithObstacles / (path.size() - 1) * 100.0;
            System.out.println("DEBUG: Path has " + pathSegmentsWithObstacles + " obstructed segments (" +
                    String.format("%.1f", percentObstructed) + "%)");
            return false;
        }

        return true;
    }

    /**
     * Calculate standard deviation for a window of fitness values
     */
    private double calculateStdDev(List<Double> values, int windowSize) {
        int startIdx = Math.max(0, values.size() - windowSize);
        int endIdx = values.size();

        if (endIdx - startIdx < 2) {
            return 0.0;
        }

        // Calculate mean
        double sum = 0.0;
        for (int i = startIdx; i < endIdx; i++) {
            sum += values.get(i);
        }
        double mean = sum / (endIdx - startIdx);

        // Calculate variance
        double variance = 0.0;
        for (int i = startIdx; i < endIdx; i++) {
            double diff = values.get(i) - mean;
            variance += diff * diff;
        }
        variance /= (endIdx - startIdx - 1);

        return Math.sqrt(variance);
    }

    /**
     * Get valid neighbors of a grid position
     */
    private List<GridPosition> getNeighbors(GridPosition pos, Set<GridPosition> obstacles) {
        List<GridPosition> neighbors = new ArrayList<>();

        // Try to prioritize cardinal directions first (better for robots)
        int[][] directions = {
                { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 }, // Cardinal (N, E, S, W)
                { 1, 1 }, { 1, -1 }, { -1, -1 }, { -1, 1 } // Diagonal (NE, SE, SW, NW)
        };

        for (int[] dir : directions) {
            int dx = dir[0];
            int dy = dir[1];

            GridPosition neighbor = new GridPosition(pos.x + dx, pos.y + dy);

            // Check if the neighbor is valid
            if (isValidPosition(neighbor) && !obstacles.contains(neighbor)) {
                neighbors.add(neighbor);
            }
        }

        return neighbors;
    }

    /**
     * Check if a grid position is within bounds
     */
    private boolean isValidPosition(GridPosition pos) {
        return pos.x >= 0 && pos.x < nodesX && pos.y >= 0 && pos.y < nodesY;
    }

    /**
     * Calculate heuristic (straight-line distance) between two positions
     */
    private double heuristic(GridPosition a, GridPosition b) {
        return Math.hypot(b.x - a.x, b.y - a.y);
    }

    /**
     * Calculate distance between two adjacent positions
     */
    private double distance(GridPosition a, GridPosition b) {
        return (a.x == b.x || a.y == b.y) ? 1.0 : Math.sqrt(2.0);
    }

    /**
     * Simple path smoothing to reduce zigzags
     */
    private List<GridPosition> smoothPath(List<GridPosition> path, Set<GridPosition> obstacles) {
        if (path.size() <= 2) {
            return path;
        }

        List<GridPosition> smoothed = new ArrayList<>();
        smoothed.add(path.get(0));

        int i = 0;
        while (i < path.size() - 1) {
            int j = path.size() - 1;

            // Find furthest visible node from current position
            while (j > i + 1) {
                if (isLineOfSightClear(path.get(i), path.get(j), obstacles)) {
                    break;
                }
                j--;
            }

            i = j;
            smoothed.add(path.get(i));
        }

        return smoothed;
    }

    /**
     * Check if there's a clear line of sight between two positions
     */
    private boolean isLineOfSightClear(GridPosition a, GridPosition b, Set<GridPosition> obstacles) {
        // Use Bresenham's line algorithm
        int x0 = a.x;
        int y0 = a.y;
        int x1 = b.x;
        int y1 = b.y;

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (x0 != x1 || y0 != y1) {
            GridPosition pos = new GridPosition(x0, y0);
            if (obstacles.contains(pos)) {
                // Debug when collision is detected
                // System.out.println("DEBUG: Line of sight blocked at " + pos);
                return false;
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }

        return true;
    }

    /**
     * Create Waypoints for PathPlanner from grid positions
     */
    private List<Waypoint> createWaypoints(
            List<GridPosition> path,
            Translation2d realStartPos,
            Translation2d realGoalPos,
            Rotation2d startRotation) {

        if (path.isEmpty()) {
            System.out.println("WARNING: Empty path, cannot create waypoints.");
            return new ArrayList<>();
        }

        // Debug path points
        System.out.println("DEBUG: Creating waypoints for path with " + path.size() + " points:");

        // Only log first 5 and last 5 points to avoid console spam for long paths
        int maxPointsToShow = Math.min(path.size(), 10);
        boolean skipMiddle = path.size() > maxPointsToShow;

        for (int i = 0; i < path.size(); i++) {
            if (skipMiddle && i >= 5 && i < path.size() - 5) {
                if (i == 5) {
                    System.out.println("  ... " + (path.size() - 10) + " more points ...");
                }
                continue;
            }

            GridPosition pos = path.get(i);
            System.out.println("  Path point " + i + ": " + pos + " -> " + gridPosToTranslation2d(pos));
        }

        // If path is too dense, simplify it to get smoother robot motion
        if (path.size() > 20) {
            System.out.println("Simplifying dense path from " + path.size() + " points");
            path = simplifyPath(path);
            System.out.println("Simplified to " + path.size() + " points");
        }

        // Convert grid positions to field positions
        List<Translation2d> fieldPosPath = new ArrayList<>();
        for (GridPosition pos : path) {
            fieldPosPath.add(gridPosToTranslation2d(pos));
        }

        // Replace start and end positions with exact positions
        if (!fieldPosPath.isEmpty()) {
            System.out.println("DEBUG: Setting exact start position: " + realStartPos);
            fieldPosPath.set(0, realStartPos);
        }

        if (fieldPosPath.size() > 1) {
            System.out.println("DEBUG: Setting exact goal position: " + realGoalPos);
            fieldPosPath.set(fieldPosPath.size() - 1, realGoalPos);
        }

        if (fieldPosPath.size() < 2) {
            List<Pose2d> singlePose = new ArrayList<>();
            singlePose.add(new Pose2d(realStartPos, startRotation));
            return PathPlannerPath.waypointsFromPoses(singlePose);
        }

        // Create poses for waypoints
        List<Pose2d> pathPoses = new ArrayList<>();

        // Start point
        Rotation2d startHeading;
        if (fieldPosPath.size() > 1) {
            startHeading = new Rotation2d(
                    fieldPosPath.get(1).getX() - fieldPosPath.get(0).getX(),
                    fieldPosPath.get(1).getY() - fieldPosPath.get(0).getY());
        } else {
            startHeading = startRotation;
        }
        pathPoses.add(new Pose2d(fieldPosPath.get(0), startHeading));

        // Intermediate points with anchors for smooth turns
        for (int i = 1; i < fieldPosPath.size() - 1; i++) {
            Translation2d prev = fieldPosPath.get(i - 1);
            Translation2d current = fieldPosPath.get(i);
            Translation2d next = fieldPosPath.get(i + 1);

            // Calculate tangent vectors
            Translation2d incomingVector = current.minus(prev);
            Translation2d outgoingVector = next.minus(current);

            Rotation2d incomingHeading = new Rotation2d(incomingVector.getX(), incomingVector.getY());
            Rotation2d outgoingHeading = new Rotation2d(outgoingVector.getX(), outgoingVector.getY());

            // Create anchors for smooth curves
            Translation2d anchor1 = current.minus(prev).times(SMOOTHING_ANCHOR_PCT).plus(prev);
            Translation2d anchor2 = current.minus(next).times(SMOOTHING_ANCHOR_PCT).plus(next);

            pathPoses.add(new Pose2d(anchor1, incomingHeading));
            pathPoses.add(new Pose2d(anchor2, outgoingHeading));
        }

        // End point
        Rotation2d endHeading = new Rotation2d(
                fieldPosPath.get(fieldPosPath.size() - 1).getX() - fieldPosPath.get(fieldPosPath.size() - 2).getX(),
                fieldPosPath.get(fieldPosPath.size() - 1).getY() - fieldPosPath.get(fieldPosPath.size() - 2).getY());
        pathPoses.add(new Pose2d(fieldPosPath.get(fieldPosPath.size() - 1), endHeading));

        // Debug final waypoints
        System.out.println("DEBUG: Final waypoints count: " +
                (pathPoses != null ? pathPoses.size() : 0));

        return PathPlannerPath.waypointsFromPoses(pathPoses);
    }

    /**
     * Simplify a dense path by removing unnecessary points
     * This helps create smoother robot motion by reducing the number of waypoints
     */
    private List<GridPosition> simplifyPath(List<GridPosition> path) {
        if (path.size() <= 4) {
            return path; // Already simple enough
        }

        List<GridPosition> simplified = new ArrayList<>();
        simplified.add(path.get(0)); // Always keep start point

        // Use Douglas-Peucker-like algorithm but with a fixed distance
        double epsilon = 1.0; // Tolerance value

        for (int i = 1; i < path.size() - 1; i++) {
            GridPosition prev = path.get(i - 1);
            GridPosition current = path.get(i);
            GridPosition next = path.get(i + 1);

            // Calculate vectors
            double vx1 = current.x - prev.x;
            double vy1 = current.y - prev.y;
            double vx2 = next.x - current.x;
            double vy2 = next.y - current.y;

            // Calculate dot product
            double dotProduct = vx1 * vx2 + vy1 * vy2;
            double mag1 = Math.sqrt(vx1 * vx1 + vy1 * vy1);
            double mag2 = Math.sqrt(vx2 * vx2 + vy2 * vy2);

            // Avoid division by zero
            if (mag1 > 0 && mag2 > 0) {
                double cosAngle = dotProduct / (mag1 * mag2);
                // Clamp to avoid numerical issues
                cosAngle = Math.min(1.0, Math.max(-1.0, cosAngle));

                // Convert to angle in radians
                double angle = Math.acos(cosAngle);

                // Keep points that represent significant turns or are at a distance
                if (angle > 0.2 || (i % 4 == 0)) { // Keep turns and every 4th point
                    simplified.add(current);
                }
            } else {
                simplified.add(current); // Keep the point if we can't calculate (shouldn't happen)
            }
        }

        simplified.add(path.get(path.size() - 1)); // Always keep end point

        return simplified;
    }

    /**
     * Convert grid position to field position
     */
    private Translation2d gridPosToTranslation2d(GridPosition pos) {
        return new Translation2d(
                (pos.x + 0.5) * nodeSize,
                (pos.y + 0.5) * nodeSize);
    }

    /**
     * Convert field position to grid position
     */
    private GridPosition getGridPos(Translation2d pos) {
        int x = (int) Math.floor(pos.getX() / nodeSize);
        int y = (int) Math.floor(pos.getY() / nodeSize);
        return new GridPosition(x, y);
    }

    // PathFinder interface implementation

    @Override
    public boolean isNewPathAvailable() {
        return newPathAvailable.get();
    }

    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
        List<Waypoint> waypoints;

        pathLock.readLock().lock();
        waypoints = new ArrayList<>(currentWaypoints);
        pathLock.readLock().unlock();

        newPathAvailable.set(false);

        if (waypoints.size() < 2) {
            System.out.println("WARNING: Returning null path because not enough waypoints: " +
                    (waypoints != null ? waypoints.size() : 0));
            return null;
        }

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, goalEndState);

        // Add debug visualization of the path
        System.out.println("DEBUG: Returning path with " + waypoints.size() + " waypoints");
        System.out.println("      Start: " + waypoints.get(0).anchor());
        System.out.println("      End: " + waypoints.get(waypoints.size() - 1).anchor());

        return path;
    }

    @Override
    public void setStartPosition(Translation2d startPosition) {
        setStartPose(new Pose2d(startPosition, new Rotation2d()));
    }

    /**
     * Set the start pose with rotation
     */
    public void setStartPose(Pose2d startPose) {
        GridPosition gridPos = findClosestNonObstacle(getGridPos(startPose.getTranslation()), allObstacles);

        if (gridPos != null) {
            requestLock.writeLock().lock();
            requestStart = gridPos;
            requestRealStartPos = startPose.getTranslation();
            requestStartRotation = startPose.getRotation();
            requestLock.writeLock().unlock();

            newRequestAvailable.set(true);
        }
    }

    @Override
    public void setGoalPosition(Translation2d goalPosition) {
        GridPosition gridPos = findClosestNonObstacle(getGridPos(goalPosition), allObstacles);

        if (gridPos != null) {
            requestLock.writeLock().lock();
            requestGoal = gridPos;
            requestRealGoalPos = goalPosition;
            requestLock.writeLock().unlock();

            newRequestAvailable.set(true);
        }
    }

    @Override
    public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Translation2d currentRobotPos) {
        Set<GridPosition> newDynamicObstacles = new HashSet<>();

        System.out.println("Setting dynamic obstacles: " +
                (obstacles != null ? obstacles.size() : 0) + " obstacles");

        for (Pair<Translation2d, Translation2d> obstacle : obstacles) {
            Translation2d corner1 = obstacle.getFirst();
            Translation2d corner2 = obstacle.getSecond();

            GridPosition gridPos1 = getGridPos(corner1);
            GridPosition gridPos2 = getGridPos(corner2);

            System.out.println("  Dynamic obstacle: " + corner1 + " to " + corner2);
            System.out.println("    Grid positions: " + gridPos1 + " to " + gridPos2);

            int minX = Math.min(gridPos1.x, gridPos2.x);
            int maxX = Math.max(gridPos1.x, gridPos2.x);
            int minY = Math.min(gridPos1.y, gridPos2.y);
            int maxY = Math.max(gridPos2.y, gridPos2.y);

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    newDynamicObstacles.add(new GridPosition(x, y));
                }
            }
        }

        // Update obstacles
        requestLock.writeLock().lock();
        dynamicObstacles.clear();
        dynamicObstacles.addAll(newDynamicObstacles);

        allObstacles.clear();
        allObstacles.addAll(staticObstacles);
        allObstacles.addAll(dynamicObstacles);
        System.out.println("Total obstacles after update: " + allObstacles.size() +
                " (static: " + staticObstacles.size() + ", dynamic: " + dynamicObstacles.size() + ")");
        requestLock.writeLock().unlock();

        // Request a path update if the current path is affected
        pathLock.readLock().lock();
        boolean needsUpdate = false;
        for (GridPosition pos : currentPathFull) {
            if (newDynamicObstacles.contains(pos)) {
                needsUpdate = true;
                break;
            }
        }
        pathLock.readLock().unlock();

        if (needsUpdate) {
            setStartPosition(currentRobotPos);
        }
    }

    /**
     * Find the closest non-obstacle position to the given position
     */
    private GridPosition findClosestNonObstacle(GridPosition pos, Set<GridPosition> obstacles) {
        if (!obstacles.contains(pos)) {
            return pos;
        }

        Queue<GridPosition> queue = new LinkedList<>();
        Set<GridPosition> visited = new HashSet<>();

        queue.add(pos);
        visited.add(pos);

        while (!queue.isEmpty()) {
            GridPosition current = queue.poll();

            if (!obstacles.contains(current)) {
                return current;
            }

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    GridPosition neighbor = new GridPosition(current.x + dx, current.y + dy);

                    if (isValidPosition(neighbor) && !visited.contains(neighbor)) {
                        queue.add(neighbor);
                        visited.add(neighbor);
                    }
                }
            }
        }

        return null;
    }

    /**
     * Grid position record
     */
    public record GridPosition(int x, int y) implements Comparable<GridPosition> {
        @Override
        public int compareTo(GridPosition o) {
            if (x == o.x) {
                return Integer.compare(y, o.y);
            } else {
                return Integer.compare(x, o.x);
            }
        }
    }
}