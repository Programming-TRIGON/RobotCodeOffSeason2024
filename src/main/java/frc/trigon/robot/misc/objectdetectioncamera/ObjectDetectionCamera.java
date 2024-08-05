package frc.trigon.robot.misc.objectdetectioncamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

public class ObjectDetectionCamera extends SubsystemBase {
    private final ObjectDetectionCameraInputsAutoLogged objectDetectionCameraInputs = new ObjectDetectionCameraInputsAutoLogged();
    private final ObjectDetectionCameraIO objectDetectionCameraIO;
    private final String hostname;
    private double lastVisibleObjectYaw = 0;

    public ObjectDetectionCamera(String hostname) {
        this.hostname = hostname;
        objectDetectionCameraIO = generateIO(hostname);
    }

    @Override
    public void periodic() {
        objectDetectionCameraIO.updateInputs(objectDetectionCameraInputs);
        Logger.processInputs(hostname, objectDetectionCameraInputs);
    }

    public boolean hasTargets() {
        return objectDetectionCameraInputs.hasTargets;
    }

    /**
     * @return the yaw (x-axis position) of the target object
     */
    public double getBestObjectYaw() {
        return objectDetectionCameraInputs.bestObjectYaw;
    }

    public double getTrackedObjectYaw() {
        double closestYawDifference = 10000000;
        double closestYaw = 10000000;
        for (double currentYaw : objectDetectionCameraInputs.visibleObjectsYaw) {
            final double yawDifference = Math.abs(currentYaw - lastVisibleObjectYaw);
            if (yawDifference < closestYawDifference) {
                closestYawDifference = yawDifference;
                closestYaw = currentYaw;
            }
        }
        if (closestYawDifference != 10000000) {
            lastVisibleObjectYaw = closestYaw;
            return lastVisibleObjectYaw;
        }
        return lastVisibleObjectYaw;
    }

    public void startTrackingBestObject() {
        lastVisibleObjectYaw = getBestObjectYaw();
    }

    private ObjectDetectionCameraIO generateIO(String hostname) {
        if (RobotHardwareStats.isReplay())
            return new ObjectDetectionCameraIO();
        if (RobotHardwareStats.isSimulation())
            return new SimulationObjectDetectionCameraIO(hostname);
        return new PhotonObjectDetectionCameraIO(hostname);
    }
}
