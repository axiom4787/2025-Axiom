// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.pathplanning.NetworkTablesADStar;
import frc.robot.subsystems.pathplanning.OkayPlan;
import frc.robot.utils.CommandLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

	private CommandScheduler commandScheduler;

	public Robot() {
		m_robotContainer = new RobotContainer();
		Pathfinding.setPathfinder(new NetworkTablesADStar(m_robotContainer.getDriveSubsystem()));
		// Pathfinding.setPathfinder(new OkayPlan(Constants.Drivetrain.ROBOT_WIDTH, Constants.Drivetrain.ROBOT_LENGTH));
		PathfindingCommand.warmupCommand().schedule();
		// Wamup the pathfinding command. Source: https://pathplanner.dev/pplib-pathfinding.html#java-warmup

		new CommandLogger();
    commandScheduler = CommandScheduler.getInstance();

	}

	@Override
	public void robotPeriodic() {
		commandScheduler.run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		// Get the selected autonomous command
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// Schedule it if it exists
		if (m_autonomousCommand != null) {
			System.out.println("Scheduling autonomous command: " + m_autonomousCommand.getName());
			m_autonomousCommand.schedule();
		} else {
			System.out.println("No autonomous command selected!");
		}
	}

  @Override
  public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
    m_teleopCommand = m_robotContainer.getTeleopCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		m_teleopCommand = m_robotContainer.getTeleopCommand();

		try {
			m_robotContainer.findStartingVisionPose();
		} catch (Exception e) {
			System.err.println("Failed to initialize pose with vision: " + e.getMessage());
			// Continue without vision initialization
		}
		
		m_robotContainer.startAutoPathThread();
		// System.out.println("Teleop init");

		if (m_teleopCommand != null) {
			m_teleopCommand.schedule();
		}

    m_teleopCommand.schedule();
	}

	@Override
	public void teleopPeriodic() {
		// m_robotContainer.getNetworkTablesReceiver().runMain();
	
    m_teleopCommand.execute();
  }

	@Override
	public void teleopExit() {
	}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

	@Override
	public void testPeriodic() {
	
    m_testCommand.execute();
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
