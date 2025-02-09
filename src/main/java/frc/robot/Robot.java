// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.NetworkTablesADStar;
import frc.robot.utils.CommandLogger;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand, m_teleopCommand;

	private final RobotContainer m_robotContainer;

	public Robot() {
		m_robotContainer = new RobotContainer();
		Pathfinding.setPathfinder(new NetworkTablesADStar(m_robotContainer.getDriveSubsystem()));

		new CommandLogger();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
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
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		m_teleopCommand = m_robotContainer.getTeleopCommand();

		m_robotContainer.findStartingVisionPose();
		m_robotContainer.startAutoPathThread();
		// System.out.println("Teleop init");

		if (m_teleopCommand != null) {
			m_teleopCommand.schedule();
		}
	}

	@Override
	public void teleopPeriodic() {
		// m_robotContainer.getNetworkTablesReceiver().runMain();
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
	}

	@Override
	public void testExit() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		// m_teleopCommand = m_robotContainer.getTeleopCommand();

		// m_robotContainer.findStartingVisionPose();
		// m_robotContainer.getNetworkTablesReceiver().runMain();
		// m_robotContainer.startAutoPathThread();
		// System.out.println("Teleop init");

		// if (m_teleopCommand != null) {
		// m_teleopCommand.schedule();
		// }
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
