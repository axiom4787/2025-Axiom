package frc.robot.subsystems;

public class Machine {

    public static gameState gState = gameState.Auto;

    public enum gameState{
        Teleop,
        Auto
    }

    public static autoState aState = autoState.LeaveLine;

    public enum autoState{
        LeaveLine,
        DriveReef,
        AlignCoral,
        CoralOuttake,
        ReachAlgae, // Don't know how to handle this yet
        Intake_Algae,
        DriveProcessor,
        Processor,
        Outtake_Algae // Done!
    }

    public static driveState dState = driveState.Robot_off;

    public enum driveState{
        CoralOuttake,
        CoralIntake,
        AlgaeIntake,
        AlgaeIntakeGround,
        Processor,
        L1,
        Climbing_on,
        Climbing_off,
        Robot_off
    }
}
