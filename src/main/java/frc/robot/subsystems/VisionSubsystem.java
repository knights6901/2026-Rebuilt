package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem responsible for AprilTag-based pose estimation using PhotonVision.
 *
 * <p>
 * Fuses vision measurements into the drivetrain's Kalman-filter odometry,
 * tracks visible tag IDs and poses, and publishes tag data to NetworkTables.
 * Supports full camera simulation when running in sim.
 */
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera photonCam;
    private final CommandSwerveDrivetrain drivetrain;

    private final PhotonPoseEstimator visionPoseEstimator;
    // not final because setting fieldLayout wasn't working without try/catch
    private AprilTagFieldLayout fieldLayout;

    /** Vision simulation system (active only when running in simulator). */
    private VisionSystemSim visionSim;
    /**
     * Simulated camera properties and output (active only when running in
     * simulator).
     */
    private PhotonCameraSim cameraSim;

    /** Network table publisher for visible AprilTag positions. */
    private final StructArrayPublisher<Pose3d> tagPublisher;

    /** Cached list of 3D field poses for currently visible AprilTags. */
    private List<Pose3d> visibleTagPoses = new ArrayList<>();
    /** Cached list of fiducial IDs for currently visible AprilTags. */
    private List<Integer> visibleTagIds = new ArrayList<>();

    /** The most recently estimated robot pose from vision (optional). */
    private Optional<EstimatedRobotPose> visionEstimatedPose = Optional.empty();

    public boolean hasSeededPose = true;

    public final Field2d m_visionfield = new Field2d();

    /**
     * Creates the vision subsystem, initializing PhotonVision cameras and the
     * pose estimator from the current year's field layout. Starts camera
     * simulation if running in sim.
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        photonCam = new PhotonCamera("photonCam");

        this.drivetrain = drivetrain;

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Could not find the field file!");
        }

        Transform3d robotToCam = new Transform3d(new Translation3d(-0.31, 0.0, 0.14),
                // new Rotation3d(0, Math.PI / 3, Math.PI));
                new Rotation3d(0, Math.PI / 3, 0));
                // new Rotation3d(0, -Math.PI / 3, Math.pi)); // maybe it works? - sam

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

        SmartDashboard.putData("VisionField", m_visionfield);
    }

    /**
     * Returns the 3D field poses of all AprilTags currently visible to the camera.
     *
     * @return list of visible tag poses (may be empty)
     */
    public List<Pose3d> getVisibleTagPoses() {
        return visibleTagPoses;
    }

    /**
     * Returns the fiducial IDs of all AprilTags currently visible to the camera.
     *
     * @return list of visible tag IDs (may be empty)
     */
    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    /**
     * Sets up the PhotonVision simulation environment with a simulated camera
     * matching the physical camera's transform.
     *
     * @param robotToCam the 3D transform from the robot origin to the camera
     */
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
        cameraSim = new PhotonCameraSim(photonCam, cameraProp);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        visionSim.addCamera(cameraSim, robotToCam);

        SmartDashboard.putData("VisionSim", visionSim.getDebugField());
    }

    @Override
    public void periodic() {
        // clear the visible tag lists - they will be repopulated if there are targets
        // in the latest result
        visibleTagPoses.clear();
        visibleTagIds.clear();

        List<PhotonPipelineResult> results = photonCam.getAllUnreadResults();

        if (results.size() == 0) {
            visionEstimatedPose = Optional.empty();
            return;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        
        if (latest.hasTargets()) {
            List<PhotonTrackedTarget> targets = latest.getTargets();
            PhotonTrackedTarget bestTarget = latest.getBestTarget();
            
            if (bestTarget.getPoseAmbiguity() < 0.2) {
                visionEstimatedPose = visionPoseEstimator.estimateCoprocMultiTagPose(latest)
                        .or(() -> visionPoseEstimator.estimateLowestAmbiguityPose(latest));
            }
            
            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                tagPose.ifPresent(visibleTagPoses::add);
                visibleTagIds.add(target.getFiducialId());
            }

            tagPublisher.set(visibleTagPoses.toArray(new Pose3d[0]));
        }

        if (visionEstimatedPose.isPresent()) {
            m_visionfield.setRobotPose(getEstimatedPose2d().get());

            if (!hasSeededPose) {
                hasSeededPose = true;

                Translation2d translation = getEstimatedPose2d().get().getTranslation();

                drivetrain.resetTranslation(translation);
            }
        }
    }

    public Optional<Pose2d> getEstimatedPose2d() {
        return visionEstimatedPose.map(pose -> pose.estimatedPose.toPose2d());
    }

    public void adjustDrivetrainPose() {
        if (visionEstimatedPose.isPresent()) {
            drivetrain.addVisionMeasurement(getEstimatedPose2d().get(), visionEstimatedPose.get().timestampSeconds,
            VecBuilder.fill(0.2, 0.2, 99999));
        }
    }

    public void reseedPose() {
        hasSeededPose = false;
    }
}