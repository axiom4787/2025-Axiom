package frc.robot.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
    private BooleanPublisher connectedPublisher;

    private DoublePublisher latestTargetObservationPublisherTx;
    private DoublePublisher latestTargetObservationPublisherTy;

    private DoublePublisher PoseObservationPublisherTimestamp;

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("LatestTargetObservation", latestTargetObservation);
        table.put("PoseObservations", poseObservations);
        table.put("TagIds", tagIds);

        // connectedPublisher = NetworkTableInstance.getDefault()
        //         .getBooleanTopic("Limelight/Connected").publish();

        // latestTargetObservationPublisherTx = NetworkTableInstance.getDefault()
        //         .getDoubleTopic("Limelight/TargetObservation/tx").publish();
        // latestTargetObservationPublisherTy = NetworkTableInstance.getDefault()
        //         .getDoubleTopic("Limelight/TargetObservation/ty").publish();

        

        // connectedPublisher.set(connected);
        // latestTargetObservationPublisherTx.set(latestTargetObservation.tx().getDegrees());
        // latestTargetObservationPublisherTy.set(latestTargetObservation.ty().getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        latestTargetObservation = table.get("LatestTargetObservation", latestTargetObservation);
        poseObservations = table.get("PoseObservations", poseObservations);
        tagIds = table.get("TagIds", tagIds);
    }

    public VisionIOInputsAutoLogged clone() {
        VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
        copy.connected = this.connected;
        copy.latestTargetObservation = this.latestTargetObservation;
        copy.poseObservations = this.poseObservations.clone();
        copy.tagIds = this.tagIds.clone();
        return copy;
    }
}
