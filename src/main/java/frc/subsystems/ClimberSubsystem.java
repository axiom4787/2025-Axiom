package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.google.flatbuffers.Constants;
import com.revrobotics.CANSparkMax;

public class ClimberSubsystem extends SubsystemBase {
    
    private CANSparkMax leftClimber, rightClimber;

    //Constructor
    public ClimberSubsystem(){

        leftClimber = new CANSparkMax(Constants.LEFT_CLIMBER_ID);
        rightClimber = new CANSparkMax(Constants.RIGHT_CLIMBER_ID);

    }

    //will add method to activate the moter to push down on the deep cage
    //most testing will probably just be strength/voltage of the motor


    //Sets the motor voltage
    public void setMotorVoltage(double volts){
        //tba once motor stuff is added
    }

    //Stops the motor
    public void stopMotor(){
        //also tba
    }

    //Returns angle of the motor
    public double getMotorAngle(){
        //also tba
    }

    //Returns motor current
    public double getMotorCurrent(){
        //also tba 
    }


}
