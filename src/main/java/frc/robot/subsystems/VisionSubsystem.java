package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import java.io.IOException;

import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    @SuppressWarnings("unused")
    private final PhotonPoseEstimator visionPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    // not final because setting fieldLayout wasn't working without try/catch
    private AprilTagFieldLayout fieldLayout;

    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;
    // private final StructSubscriber<Pose2d> poseSub;
    private final StructArrayPublisher<Pose3d> tagPublisher;

    private List<Pose3d> visibleTagPoses = new ArrayList<>();
    private List<Integer> visibleTagIds = new ArrayList<>();

    public Optional<EstimatedRobotPose> visionEstimatedPose = Optional.empty();

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Could not find the field file!");
        }

        // put real values in here when cad + camera location is finalized
        // these are rough estimates based on current design team predictions (jan 26)
        Transform3d robotToCam = new Transform3d(new Translation3d(0.2, 0.0, 0.45), new Rotation3d(0, 0, 0));

        if (fieldLayout != null) {
            visionPoseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
        } else {
            visionPoseEstimator = null;
        }

        if (RobotBase.isSimulation()) {
            initializeSimulation(robotToCam);
        } else {
            visionSim = null;
            cameraSim = null;
        }

        tagPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("visibleTags", Pose3d.struct).publish();
    }

    public List<Pose3d> getVisibleTagPoses() {
        return visibleTagPoses;
    }

    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    // Initializes the vision simulation, which includes setting up the simulated
    // camera and adding it to the vision system simulation. This is only called if
    // the code is running in a simulation environment.
    private void initializeSimulation(Transform3d robotToCam) {
        visionSim = new VisionSystemSim("main");
        if (fieldLayout != null) {
            visionSim.addAprilTags(fieldLayout);
        }
        SimCameraProperties cameraProp = new SimCameraProperties();

        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(.25, 0.88);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        visionSim.addCamera(cameraSim, robotToCam);
        /*
         * poseSub = NetworkTableInstance.getDefault()
         * .getStructTopic("SmartDashboard/RobotPose", Pose2d.struct)
         * .subscribe(new Pose2d());
         */
        SmartDashboard.putData("VisionSim", visionSim.getDebugField());
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
    }

    @Override
    public void periodic() {

        visibleTagPoses.clear();
        visibleTagIds.clear();

        for (var result : camera.getAllUnreadResults()) {
            visionEstimatedPose = visionPoseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEstimatedPose.isEmpty()) {
                visionEstimatedPose = visionPoseEstimator.estimateLowestAmbiguityPose(result);
            }

            if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();

            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                tagPose.ifPresent(visibleTagPoses::add);
                visibleTagIds.add(target.getFiducialId());
            }
            tagPublisher.set(visibleTagPoses.toArray(new Pose3d[0]));
        }
        }
        
    }
}