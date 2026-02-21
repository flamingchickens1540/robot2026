package org.team1540.robot2026.subsystems.vision;

import static org.team1540.robot2026.subsystems.vision.AprilTagVisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2026.FieldConstants;
import org.team1540.robot2026.RobotState;
import org.team1540.robot2026.subsystems.vision.AprilTagVisionIO.PoseObservation;
import org.team1540.robot2026.util.LoggedTracer;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagVisionIO turretCameraIO;
    private final AprilTagVisionIOInputsAutoLogged turretCameraInputs = new AprilTagVisionIOInputsAutoLogged();

    private final AprilTagVisionIO[] visionIOs;
    private final AprilTagVisionIOInputsAutoLogged[] cameraInputs;

    private final Alert turretCameraDisconnectedAlert =
            new Alert("Turret camera is disconnected", Alert.AlertType.kWarning);
    private final Alert[] disconnectedAlerts;

    private final ArrayList<Pose3d> lastAcceptedTurretPoses = new ArrayList<>();
    private final ArrayList<Pose3d> lastRejectedTurretPoses = new ArrayList<>();

    private final ArrayList<Pose3d> lastAcceptedPoses = new ArrayList<>();
    private final ArrayList<Pose3d> lastRejectedPoses = new ArrayList<>();
    private final ArrayList<Pose3d> lastSeenTagPoses = new ArrayList<>();

    private AprilTagVision(AprilTagVisionIO turretCameraIO, AprilTagVisionIO... visionIOs) {
        this.turretCameraIO = turretCameraIO;

        this.visionIOs = visionIOs;
        this.cameraInputs = new AprilTagVisionIOInputsAutoLogged[visionIOs.length];
        this.disconnectedAlerts = new Alert[visionIOs.length];
        for (int i = 0; i < cameraInputs.length; i++) {
            cameraInputs[i] = new AprilTagVisionIOInputsAutoLogged();
            disconnectedAlerts[i] = new Alert(visionIOs[i].name + " is disconnected.", Alert.AlertType.kWarning);
        }
    }

    public void periodic() {
        LoggedTracer.reset();

        turretCameraIO.updateInputs(turretCameraInputs);
        Logger.processInputs("Vision/turret/", turretCameraInputs);

        for (int i = 0; i < visionIOs.length; i++) {
            visionIOs[i].updateInputs(cameraInputs[i]);
            Logger.processInputs("Vision/" + visionIOs[i].name, cameraInputs[i]);
        }

        RobotState robotState = RobotState.getInstance();

        lastAcceptedTurretPoses.clear();
        lastRejectedTurretPoses.clear();
        turretCameraDisconnectedAlert.set(!turretCameraInputs.connected);
        for (PoseObservation observation : turretCameraInputs.poseObservations) {
            if (robotState.addTurretVisionMeasurement(observation)) {
                lastAcceptedTurretPoses.add(observation.estimatedPoseMeters());
            } else {
                lastRejectedTurretPoses.add(observation.estimatedPoseMeters());
            }

            lastSeenTagPoses.addAll(Arrays.stream(turretCameraInputs.seenTagIDs)
                    .mapToObj(tagID -> FieldConstants.AprilTagLayoutType.OFFICIAL
                            .getLayout()
                            .getTagPose(tagID)
                            .orElse(Pose3d.kZero))
                    .toList());
        }
        Logger.recordOutput("Vision/Turret/AcceptedPoses", lastAcceptedTurretPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Turret/RejectedPoses", lastRejectedTurretPoses.toArray(new Pose3d[0]));

        lastAcceptedPoses.clear();
        lastRejectedPoses.clear();
        lastSeenTagPoses.clear();
        for (int i = 0; i < visionIOs.length; i++) {
            disconnectedAlerts[i].set(!cameraInputs[i].connected);

            for (PoseObservation observation : cameraInputs[i].poseObservations) {
                if (robotState.addVisionMeasurement(observation)) {
                    lastAcceptedPoses.add(observation.estimatedPoseMeters());
                } else {
                    lastRejectedPoses.add(observation.estimatedPoseMeters());
                }

                lastSeenTagPoses.addAll(Arrays.stream(cameraInputs[i].seenTagIDs)
                        .mapToObj(tagID -> FieldConstants.AprilTagLayoutType.OFFICIAL
                                .getLayout()
                                .getTagPose(tagID)
                                .orElse(Pose3d.kZero))
                        .toList());
            }
        }

        Logger.recordOutput("Vision/AcceptedPoses", lastAcceptedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/RejectedPoses", lastRejectedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/SeenTagPoses", lastSeenTagPoses.toArray(new Pose3d[0]));

        LoggedTracer.record("AprilTagVision");
    }

    public static AprilTagVision createReal() {
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(TURRET_CAMERA_NAME, TURRET_TO_CAMERA),
                new AprilTagVisionIOPhoton(BL_CAMERA_NAME, ROBOT_TO_BL_CAMERA),
                new AprilTagVisionIOPhoton(BR_CAMERA_NAME, ROBOT_TO_FL_CAMERA));
    }

    public static AprilTagVision createDummy() {
        return new AprilTagVision(
                new AprilTagVisionIO(TURRET_CAMERA_NAME) {},
                new AprilTagVisionIO(BL_CAMERA_NAME) {},
                new AprilTagVisionIO(BR_CAMERA_NAME) {});
    }
}
