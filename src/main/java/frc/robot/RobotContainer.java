// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

	// Replace with CommandPS5Controller or CommandJoystick if needed
	private final SendableChooser<Command> autoChooser;
	final PS5Controller driverPS5 = new PS5Controller(0);
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
			"swerve/neo"));
	// Applies deadbands and inverts controls because joysticks
	// are back-right positive while robot
	// controls are front-left positive
	// left stick controls translation
	// right stick controls the rotational velocity
	// buttons are quick rotation positions to different ways to face
	// WARNING: default buttons are on the same buttons as the ones defined in
	// configureBindings
	AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
			() -> -MathUtil.applyDeadband(driverPS5.getLeftY(),
					OperatorConstants.LEFT_Y_DEADBAND),
			() -> -MathUtil.applyDeadband(driverPS5.getLeftX(),
					OperatorConstants.DEADBAND),
			() -> -MathUtil.applyDeadband(driverPS5.getRightX(),
					OperatorConstants.RIGHT_X_DEADBAND),
			() -> driverPS5.getTriangleButton(),
			() -> driverPS5.getCrossButton(),
			() -> driverPS5.getSquareButton(),
			() -> driverPS5.getCircleButton());

	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled
	 * by angular velocity.
	 */
	SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> -driverPS5.getLeftX(),
			() -> driverPS5.getLeftY())
			.withControllerRotationAxis(
			() -> -driverPS5.getRightX())
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);

	/**
	 * Clone's the angular velocity input stream and converts it to a fieldRelative
	 * input stream.
	 */
	SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverPS5::getRightX,
			driverPS5::getRightY)
			.headingWhile(true);

	// Applies deadbands and inverts controls because joysticks
	// are back-right positive while robot
	// controls are front-left positive
	// left stick controls translation
	// right stick controls the desired angle NOT angular rotation
	Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

	// Applies deadbands and inverts controls because joysticks
	// are back-right positive while robot
	// controls are front-left positive
	// left stick controls translation
	// right stick controls the angular velocity of the robot
	Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

	Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

	SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
			() -> -driverPS5.getLeftX(),
			() -> driverPS5.getLeftY())
			.withControllerRotationAxis(() -> driverPS5.getRawAxis(2))
			.deadband(OperatorConstants.DEADBAND)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true);
	// Derive the heading axis with math!
	SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
			.withControllerHeadingAxis(() -> Math.sin(
					driverPS5.getRawAxis(
							2) * Math.PI)
					* (Math.PI * 2),
					() -> Math.cos(
							driverPS5.getRawAxis(
									2) * Math.PI)
							*
							(Math.PI * 2))
			.headingWhile(true);

	Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

	Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		DriverStation.silenceJoystickConnectionWarning(true);
		NamedCommands.registerCommand("test", Commands.print("I EXIST"));
     new EventTrigger("test").whileTrue(Commands.print("placing coral"));
		 // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("A"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);
}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary predicate, or via the
	 * named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS5Controller PS5}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 */
	private void configureBindings() {
		// (Condition) ? Return-On-True : Return-on-False
		drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

		if (Robot.isSimulation()) {
			new Trigger(driverPS5::getPSButtonPressed)
					.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
		}
		if (DriverStation.isTest()) {
			drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

			new Trigger(driverPS5::getCircleButton).whileTrue(drivebase.sysIdDriveMotorCommand());
			new Trigger(driverPS5::getSquareButton)
					.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			new Trigger(driverPS5::getTriangleButton).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
			
			new Trigger(driverPS5::getPSButton).onTrue(Commands.runOnce(drivebase::zeroGyro));
			new Trigger(driverPS5::getOptionsButtonPressed).whileTrue(drivebase.centerModulesCommand());
			new Trigger(driverPS5::getL1Button).onTrue(Commands.none());
			new Trigger(driverPS5::getL2Button).onTrue(Commands.none());
		} else {
			new Trigger(driverPS5::getCrossButton).onTrue(Commands.runOnce(drivebase::zeroGyro));
			new Trigger(driverPS5::getSquareButton).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
			new Trigger(driverPS5::getCircleButton).whileTrue(
					drivebase.driveToPose(
							new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
			// new
			// Trigger(driverPS5::getTriangleButton).whileTrue(drivebase.aimAtSpeaker(2));
			new Trigger(driverPS5::getPSButton).whileTrue(Commands.none());
			new Trigger(driverPS5::getOptionsButtonPressed).whileTrue(Commands.none());
			new Trigger(driverPS5::getL1Button).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
			new Trigger(driverPS5::getL2Button).onTrue(Commands.none());
		}

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	public void setDriveMode() {
		configureBindings();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	}
}
