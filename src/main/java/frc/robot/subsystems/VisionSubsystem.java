package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
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

    @SuppressWarnings("unused")
    private final PhotonCamera driverCamHopper;

    private final PhotonPoseEstimator visionPoseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
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
    public Optional<EstimatedRobotPose> visionEstimatedPose = Optional.empty();

    /** Tracks whether the vision system has provided an initial pose estimate. */
    private boolean hasSeededPose = false;

    private final BooleanPublisher seededPub = NetworkTableInstance.getDefault()
            .getTable("Vision Debugging")
            .getBooleanTopic("Has Seeded Pose")
            .publish();

    /**
     * Creates the vision subsystem, initializing PhotonVision cameras and the
     * pose estimator from the current year's field layout. Starts camera
     * simulation if running in sim.
     *
     * @param drivetrain the swerve drivetrain to feed vision measurements into
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        photonCam = new PhotonCamera("photonCam");
        driverCamHopper = new PhotonCamera("driverCamHopper");

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Could not find the field file!");
        }

        // put real values in here when cad + camera location is finalized
        // these are rough estimates based on current design team predictions (jan 26)
        Transform3d robotToCam = new Transform3d(new Translation3d(-0.33, 0.0, 0.17),
                new Rotation3d(0, Math.PI / 3, Math.PI));

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
        /*
         * poseSub = NetworkTableInstance.getDefault()
         * .getStructTopic("SmartDashboard/RobotPose", Pose2d.struct)
         * .subscribe(new Pose2d());
         */
        SmartDashboard.putData("VisionSim", visionSim.getDebugField());
    }

    /**
     * Updates the vision simulation with the current robot pose.
     * Called during simulation mode only.
     */
    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
    }

    /**
     * Processes all new camera frames from PhotonVision, updates pose estimation,
     * seeds the drivetrain odometry if not yet seeded, and publishes visible tag
     * data.
     */
    public void periodic() {
        visibleTagPoses.clear();
        visibleTagIds.clear();

        for (var result : photonCam.getAllUnreadResults()) {
            visionEstimatedPose = visionPoseEstimator.estimateCoprocMultiTagPose(result);

            if (visionEstimatedPose.isEmpty()) {
                visionEstimatedPose = visionPoseEstimator.estimateLowestAmbiguityPose(result);
            }

            if (!hasSeededPose && visionEstimatedPose.isPresent() && drivetrain.hasAppliedOperatorPerspective()) {
                Pose2d pose = visionEstimatedPose.get().estimatedPose.toPose2d();

                drivetrain.resetPose(pose);
                drivetrain.getPigeon2().setYaw(pose.getRotation().getDegrees());

                hasSeededPose = true;
            }

            visionEstimatedPose.ifPresent(estimatedPose -> {
                drivetrain.addVisionMeasurement(
                        estimatedPose.estimatedPose.toPose2d(),
                        estimatedPose.timestampSeconds,
                        // these are values for how much the bot trusts vision
                        // higher --> less trust in vision, more trust in telemetry
                        // vision is naturally pretty stuttery so i gave somewhat high std devs
                        // third value is rotation which apparently gyro is much much better for than
                        // vision anyways
                        // it needs more real-field tuning tho
                        VecBuilder.fill(1.5, 1.5, 2.0));
            });

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

        seededPub.set(hasSeededPose);
    }

    /**
     * Resets the vision pose seeding state. Call this if you need the vision
     * system to re-seed the odometry.
     */
    public void resetVisionPose() {
        hasSeededPose = false;
    }
}