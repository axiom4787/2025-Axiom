package frc.robot.subsystems.pathplanning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.util.ArrayList;
import java.util.List;

public class NetworkTablesReceiver {
    private double x, y;
    private final NetworkTable table;
    private final double driveBaseRadius = 0.4131; // Example drivebase radius

    public NetworkTablesReceiver() {
        table = NetworkTableInstance.getDefault().getTable("keyEvents");
    }

    public final void runMain() {
        // Initialize NetworkTables
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        // Set the server mode (e.g., IP of the robot or localhost for testing)
        ntInstance.setServer("127.0.0.1"); // Change to your server IP if needed
        ntInstance.startDSClient();

        // Define keys expected from the Python script
        NetworkTableEntry xEntry = table.getEntry("x");
        NetworkTableEntry yEntry = table.getEntry("y");

        // Loop to continuously check for data updates
        while (true) {
            // Get the values from the NetworkTable
            this.x = xEntry.getDouble(0.0); // Default value is 0.0 if no data is available
            this.y = yEntry.getDouble(0.0); // Default value is 0.0 if no data is available

            // Print the received values
            // System.out.printf("Received: x = %.2f, y = %.2f%n", x, y);

            // Sleep to prevent CPU overuse (adjust rate as needed)
            try {
                Thread.sleep(100); // 100ms loop delay
            } catch (InterruptedException e) {
                System.err.println("Thread interrupted: " + e.getMessage());
                break;
            }
        }

        // Clean up when exiting
        ntInstance.stopClient();
    }

    public String getLastKeyPressed() {
        return table.getEntry("last_key_pressed").getString(""); // Default to an empty string if no value received
    }

    /**
     * Fetches the dynamic obstacles from the NetworkTables.
     * Expects the data in the format: [{"x": x1, "y": y1}, {"x": x2, "y": y2}, ...]
     *
     * @return A list of `Translation2d` representing the obstacles.
     */
    public List<Translation2d> getDynamicObstacles() {
        List<Translation2d> obstacles = new ArrayList<>();
        try {
            // Get the obstacles string from the NetworkTable
            String obstacleData = table.getEntry("obstacles").getString("[]"); // Default to an empty list
    
            // Parse the string into a JSON array
            JSONArray jsonArray = (JSONArray) new JSONParser().parse(obstacleData);
    
            // Convert each JSON object into an expanded Translation2d
            for (Object obj : jsonArray) {
                JSONObject jsonObject = (JSONObject) obj;
                double x = ((Number) jsonObject.get("x")).doubleValue();
                double y = ((Number) jsonObject.get("y")).doubleValue();
                double size = ((Number) jsonObject.get("size")).doubleValue(); // Assuming the JSON has size data
    
                double expandedSize = size + (2 * this.driveBaseRadius); // Expand obstacle size by drivebase radius
                double halfExpandedSize = expandedSize / 2.0;
    
                // Create expanded square bounds
                double left = x - halfExpandedSize;
                double right = x + halfExpandedSize;
                double bottom = y - halfExpandedSize;
                double top = y + halfExpandedSize;
    
                // Add the four corners of the expanded square as Translation2d
                obstacles.add(new Translation2d(left, bottom)); // Bottom-left corner
                obstacles.add(new Translation2d(right, bottom)); // Bottom-right corner
                obstacles.add(new Translation2d(left, top)); // Top-left corner
                obstacles.add(new Translation2d(right, top)); // Top-right corner

                System.out.println("Obstacle at: " + x + ", " + y + " with size: " + size);
            }
        } catch (Exception e) {
            System.err.println("Error parsing obstacle data: " + e.getMessage());
        }
        return obstacles;
    }
}