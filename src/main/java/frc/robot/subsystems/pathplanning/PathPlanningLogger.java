package frc.robot.subsystems.pathplanning;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

/**
 * Helper class to log path planning debug information
 */
public class PathPlanningLogger {
    private static PrintWriter logWriter = null;
    private static boolean loggingEnabled = true;
    
    /**
     * Initialize the logger
     */
    public static void init() {
        if (logWriter != null) {
            return; // Already initialized
        }
        
        try {
            // Create logs directory if it doesn't exist
            File logDir = new File(Filesystem.getOperatingDirectory(), "logs");
            if (!logDir.exists()) {
                logDir.mkdir();
            }
            
            // Create a new log file with timestamp
            String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));
            File logFile = new File(logDir, "pathplanning_" + timestamp + ".log");
            
            logWriter = new PrintWriter(new FileWriter(logFile), true);
            log("Path Planning Logger initialized");
        } catch (IOException e) {
            System.err.println("Error creating log file: " + e.getMessage());
            e.printStackTrace();
            loggingEnabled = false;
        }
    }
    
    /**
     * Log a message
     * @param message Message to log
     */
    public static void log(String message) {
        if (!loggingEnabled) {
            return;
        }
        
        if (logWriter == null) {
            init();
        }
        
        // Format: [timestamp] message
        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("HH:mm:ss.SSS"));
        String formattedMessage = String.format("[%s] %s", timestamp, message);
        
        // Print to console and write to file
        System.out.println("[PathPlanningLog] " + message);
        
        if (logWriter != null) {
            logWriter.println(formattedMessage);
            logWriter.flush();
        }
    }
    
    /**
     * Clean up the logger
     */
    public static void close() {
        if (logWriter != null) {
            logWriter.close();
            logWriter = null;
        }
    }
}
