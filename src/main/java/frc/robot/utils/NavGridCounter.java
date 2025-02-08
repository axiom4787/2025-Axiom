package frc.robot.utils;

import java.io.FileReader;
import java.io.IOException;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class NavGridCounter {
    private String filePath;
    private JsonObject data;

    public NavGridCounter(String filePath) {
        this.filePath = filePath;
    }

    /**
     * Load the JSON data from the file.
     */
    public void loadData() {
        try (FileReader reader = new FileReader(filePath)) {
            data = JsonParser.parseReader(reader).getAsJsonObject();
        } catch (IOException e) {
            System.err.println("Error reading the file: " + e.getMessage());
        } catch (Exception e) {
            System.err.println("Error parsing JSON: " + e.getMessage());
        }
    }

    /**
     * Calculate the size of the grid in terms of rows and columns.
     *
     * @return an array with two elements: rows and columns.
     */
    public int[] calculateGridSize() {
        if (data == null || !data.has("grid")) {
            System.err.println("Error: Grid data is missing.");
            return new int[]{0, 0};
        }

        JsonArray grid = data.getAsJsonArray("grid");
        int rows = grid.size();
        int columns = rows > 0 ? grid.get(0).getAsJsonArray().size() : 0;

        return new int[]{rows, columns};
    }

    /**
     * Count the total number of points in the grid.
     *
     * @return the total number of points.
     */
    public int countPoints() {
        if (data == null || !data.has("grid")) {
            System.err.println("Error: Grid data is missing.");
            return 0;
        }

        JsonArray grid = data.getAsJsonArray("grid");
        int pointCount = 0;

        for (int i = 0; i < grid.size(); i++) {
            JsonArray row = grid.get(i).getAsJsonArray();
            pointCount += row.size();
        }

        return pointCount;
    }

    /**
     * Analyze the JSON file and return the results.
     *
     * @return a string with the analysis results.
     */
    public String analyze() {
        loadData();
        if (data == null) {
            return "Failed to load or parse the JSON file.";
        }

        JsonObject fieldSize = data.getAsJsonObject("field_size");
        double fieldSizeX = fieldSize.has("x") ? fieldSize.get("x").getAsDouble() : 0;
        double fieldSizeY = fieldSize.has("y") ? fieldSize.get("y").getAsDouble() : 0;

        int[] gridSize = calculateGridSize();
        int totalPoints = countPoints();

        return String.format(
            "Field Size: %.2f x %.2f\nGrid Size: %d rows x %d columns\nTotal Points: %d",
            fieldSizeX, fieldSizeY, gridSize[0], gridSize[1], totalPoints
        );
    }
}
