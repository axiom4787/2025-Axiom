package frc.robot.subsystems;

import javax.accessibility.AccessibleRelation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Machine;
import frc.robot.Constants.Arm_Angles;


public class MIntake extends SubsystemBase{

    // Replace with the current state global via import later
    public Machine mState = Machine.Robot_off;

    public MIntake() {
        // instantiate the 4 motors
    }
    
    public class Actions {
        public static void Intake_Algae(){

        }
        public static void Outtake_Algae(){
    
        }
        public static void Angle_Arm(float angle){
    
        }
        public static void Intake_Coral(){
    
        }
        public static void Outtake_Coral(){
    
        }
    }
    public void periodic(){
        switch(mState){
            case L1:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l1);
                Actions.Outtake_Coral();
            case L2:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l2);
                Actions.Outtake_Coral();
                Actions.Intake_Algae();
            case L3:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l3);
                Actions.Outtake_Coral();
                Actions.Intake_Algae();
            case L4:
                Actions.Angle_Arm(Arm_Angles.arm_angle_l4);
                Actions.Outtake_Coral();
            
            case CoralIntake:
                Actions.Intake_Coral();
            case AlgaeIntake:
                Actions.Intake_Algae();
            
            case Processor:
                Actions.Outtake_Coral();

            

        }
    }
    
}
// TODO:
// Add TODO items lmao

// DONE:
// state 1 -- L1
// state 2 -- L2
// state 3 -- L3
// state 4 -- L4
// state 5 -- Coral Intake
// state 6 -- Algae Intake
// state 7 -- processor