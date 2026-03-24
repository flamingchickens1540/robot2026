package org.team1540.robot2026.subsystems.vision;

import static org.team1540.robot2026.subsystems.vision.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.team1540.robot2026.FieldConstants;

public class AprilTagVisionIOPhotonSim extends AprilTagVisionIOPhoton {
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;

    private final Supplier<Pose3d> originPoseSupplier;

    public AprilTagVisionIOPhotonSim(
            String cameraName, Transform3d cameraTransformMeters, Supplier<Pose3d> originPoseSupplier) {
        super(cameraName, cameraTransformMeters);
        visionSim = new VisionSystemSim(cameraName);
        visionSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(SIM_RES_WIDTH, SIM_RES_HEIGHT, SIM_DIAGONAL_FOV);
        cameraProps.setCalibError(0.25, 0.125);
        cameraProps.setFPS(SIM_FPS);
        cameraProps.setAvgLatencyMs(SIM_AVG_LATENCY_MS);
        cameraProps.setLatencyStdDevMs(7.5);
        cameraSim = new PhotonCameraSim(camera, cameraProps);
        visionSim.addCamera(cameraSim, cameraTransformMeters);

        this.originPoseSupplier = originPoseSupplier;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(originPoseSupplier.get());
        super.updateInputs(inputs);
    }
}
