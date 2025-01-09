package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum Level{
    Processor,
    L1,
    L2,
    L3,
    L4
}
enum Intake_Mode{
    Cl_intake,
    Algae_intake
}

public class MIntake extends SubsystemBase{
    public MIntake() {
        // instantiate the 4 motors
    }
    // state 1 -- L1
    // state 2 -- L2
    // state 3 -- L3
    // state 4 -- L4
    // state 5 -- Coral Intake
    // state 6 -- Algae Intake
    // state 7 -- processor
}
