package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An april tag camera is a class that provides the robot's pose, from a camera using one or multiple apriltags.
 * An april tag is like a 2D barcode used to find the robot's position on the field.
 * Since the tag's position on the field is known, we can calculate our position relative to it, therefore estimating our position on the field.
 */
public class AprilTagCamera {
    protected final String name;
    private final AprilTagCameraInputsAutoLogged inputs = new AprilTagCameraInputsAutoLogged();
    private final Pose3d robotCenterToCamera;
    private final AprilTagCameraIO aprilTagCameraIO;
    private Pose3d tagPose = new Pose3d();

    /**
     * Constructs a new AprilTagCamera.
     *
     * @param aprilTagCameraType  the type of camera
     * @param name                the camera's name
     * @param robotCenterToCamera the transform of the robot's origin point to the camera
     */
    public AprilTagCamera(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType,
                          String name, Transform3d robotCenterToCamera) {
        this.name = name;
        this.robotCenterToCamera = new Pose3d().transformBy(robotCenterToCamera);

        if (RobotHardwareStats.isSimulation()) {
            aprilTagCameraIO = AprilTagCameraConstants.AprilTagCameraType.SIMULATION_CAMERA.createIOFunction.apply(name);
            aprilTagCameraIO.addSimulatedCameraToVisionSimulation(robotCenterToCamera);
            return;
        }
        aprilTagCameraIO = aprilTagCameraType.createIOFunction.apply(name);
    }

    public void update() {
        aprilTagCameraIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);
        if (hasResult())
            tagPose = robotCenterToCamera.plus(inputs.cameraSolvePNPPose);
        Logger.recordOutput("Testss/TagPose", tagPose);
        Logger.recordOutput("Testss/RobotPose", new Pose3d());
    }

    public boolean hasResult() {
        return inputs.hasResult;
    }

    public Pose3d getTagPose() {
        return tagPose;
    }
}