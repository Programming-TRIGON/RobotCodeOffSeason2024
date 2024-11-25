package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagSimulationCameraIO extends AprilTagPhotonCameraIO {
    private final PhotonCameraSim cameraSim;

    public AprilTagSimulationCameraIO(String cameraName) {
        super(cameraName);

        cameraSim = new PhotonCameraSim(photonCamera, AprilTagCameraConstants.SIMULATION_CAMERA_PROPERTIES);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    protected void addSimulatedCameraToVisionSimulation(Transform3d robotToCamera) {
        AprilTagCameraConstants.VISION_SIMULATION.addCamera(cameraSim, robotToCamera);
    }
}