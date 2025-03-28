package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/**
 * CommandLogger registers listeners with the WPILib CommandScheduler and prints
 * out when a command is scheduled,
 * interrupted/canceled, or finished.
 */
public class CommandLogger {

    /**
     * Creates a new CommandLogger that registers event listeners with the
     * CommandScheduler.
     * Call this early in your robot initialization.
     */
    public CommandLogger() {
        // Log when a command is scheduled (initialized).
        CommandScheduler.getInstance().onCommandInitialize(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                System.out.println("[CommandLogger] Command scheduled: " + command.getName());
            }
        });

        // Log when a command finishes normally.
        CommandScheduler.getInstance().onCommandFinish(new Consumer<Command>() {
            @Override
            public void accept(Command command) {
                System.out.println("[CommandLogger] Command finished: " + command.getName());
            }
        });

        // Log when a command is interrupted (canceled).
        CommandScheduler.getInstance().onCommandInterrupt(new BiConsumer<Command, Optional<Command>>() {
            @Override
            public void accept(Command command, Optional<Command> interruptor) {
                System.out.println("[CommandLogger] Command interrupted/canceled: " + command.getName());
            }
        });
    }
}
