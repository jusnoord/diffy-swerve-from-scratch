// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.util.Triplet;
import frc.robot.util.Tuple;

/**
 * PhotonVision subsystem constructs a Photon Vision camera and runs multi-threaded pose estimation.
 * Handles vision processing, pose updates, and communication with drivetrain.
 */
public class PhotonVision extends SubsystemBase {
    public static Pose2d closestMasterPose = new Pose2d();

    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    List<TimestampedObject<Pose2d>> timestampedMasterPoses = new ArrayList<>();

    private StructPublisher<Pose2d> posePublisher;
    private StructSubscriber<Pose2d> masterPoseSubscriber;

    private HashMap<Double, Pose2d> masterPoses = new HashMap<>();

    public Pose2d pose = new Pose2d();

    private Swerve drivetrain;

    private CameraThread camThread;

    public PhotonVision(Swerve drivetrain, CameraName camName) {
        this.drivetrain = drivetrain;

        camThread = new CameraThread(camName, RobotConstants.SLAVE_CAMERA_LOCATION);
        camThread.start();

        //this code grabs the pose from the master robot, and thus should only run on slave(s)
        if(!Constants.IS_MASTER) {
            masterPoseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.RobotType.master.toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
        }
        
        //initilize telemetry
        posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(Constants.currentRobot.toString()).getStructTopic("full pose", Pose2d.struct).publish();       
    }

    /**
     * only use for master robot, if no pose estimation is desired. does not start camera thread.
     * This is used to initialize the camera for the master robot, so that it can run the TimeServer for the slave(s).
     */
    public static void initializeMasterCamera() {
        CameraThread.initializeCamera(CameraName.master.toString());
    }

    /**
     * Switch the pipeline of a camera
     *
     * @param camera   Camera to switch the pipeline of
     * @param pipeline Pipeline to switch to
     */
    public void switchPipelines(PhotonCamera camera, int pipeline) {
        camera.setPipelineIndex(pipeline);

        /*
         * if pipeline is switched out of 3d mode, the camera's thread should be paused, then resumed when switched back
         * this is to prevent the camera from trying to update the pose when it's unable to
         * error handling should catch this, but it's better to be safe than sorry
         *
         * threads should be paused with Thread.wait() and resumed with Thread.notify()
         */
    }

    
    private synchronized void updateVision() {
        Tuple<Transform2d, Double> updates = camThread.getUpdates();
        timestampedMasterPoses.addAll(List.of(masterPoseSubscriber.readQueue()));

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedMasterPoses.size() > 10) {
            timestampedMasterPoses = timestampedMasterPoses.subList(timestampedMasterPoses.size() - 10, timestampedMasterPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PhotonVision timestamps are seconds. Mulitiply by one million to convert/
        double visionTimeStampMicroSeconds = updates.v * 1000000d;
        closestMasterPose = timestampedMasterPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        
        //account for tag-to-robot and camera-to-robot offsets and then combine vision measurement with master odometry positio
        Transform2d visionTranslation = new Transform2d(updates.k.getTranslation().plus(RobotConstants.centerOfMasterToTag).rotateBy(drivetrain.getPose().getRotation()).rotateBy(Rotation2d.k180deg), updates.k.getRotation().unaryMinus().rotateBy(Rotation2d.kCW_90deg));//.rotateBy(Rotation2d.kCW_90deg));//.plus(new Rotation2d(Degrees.of(90))));
        
        Pose2d fieldRelativePose = new Pose2d(closestMasterPose.getTranslation().plus(visionTranslation.getTranslation()), closestMasterPose.getRotation().plus(visionTranslation.getRotation()));


        // add the master pose to the translation to get field-relative pose. 
        // grab the timestamp
        // grab the distance to the best tag
        drivetrain.addVisionMeasurement(fieldRelativePose, updates.v, updates.k.getTranslation().getNorm());

        posePublisher.accept(fieldRelativePose);
        pose = fieldRelativePose;

    }


    /**
     * CameraThread handles vision processing in a separate thread.
     * Grabs camera results, computes pose, and updates vision measurements.
     */
    private class CameraThread extends Thread {
        // private EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, List.of(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        // private PhotonPoseEstimator poseEstimator;
        private PhotonCamera camera;
        private final Transform3d cameraPosition;
        private Double averageDistance = 0d;
        private CameraName camName;
        private Tuple<Transform2d, Double> updates;
        private boolean hasTarget = false;

        private  BooleanPublisher hasTargetPublisher;
        private  DoublePublisher targetsFoundPublisher;
        private  DoublePublisher timestampPublisher;
        // private final DoublePublisher distancePublisher;
        private  StructPublisher<Pose3d> posePublisher;

        public boolean cameraInitialized = false;

        CameraThread(CameraName camName, Transform3d cameraPosition) {
            this.camName = camName;
            this.cameraPosition = cameraPosition;

            initializeCamera();

            // Initialize camera-specific telemetry
            hasTargetPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getBooleanTopic("hasTarget").publish();
            targetsFoundPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("targetsFound").publish();
            timestampPublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getDoubleTopic("timestamp").publish();
            posePublisher = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(camName.toString()).getStructTopic("cam pose", Pose3d.struct).publish();
        }

        @Override
        public void run() {
            if(Constants.IS_MASTER) return;
            try {
                //wait for the camera to bootup before initialization
                sleep(3000);
            } catch (InterruptedException e) {
                DataLogManager.log(camName.toString() + " sleep inital failed");
            }

            //main loop for the camera thread
            while (true) {
                if (!cameraInitialized) {
                    initializeCamera();
                } else {
                    try {
                        Transform3d robotToTag = new Transform3d();
                        double timestamp = 0;

                        // main call to grab a set of results from the camera
                        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

                        double numberOfResults = results.size(); //double to prevent integer division errors
                        double totalDistances = 0;
                        boolean hasTarget = false;

                        //fundamentally, this loop updates the pose and distance for each result. It also logs the data to shuffleboard
                        //this is done in a thread-safe manner, as global variables are only updated at the end of the loop (no race conditions)
                        for (PhotonPipelineResult result : results) {
                            if (result.hasTargets()) {
                                // the local hasTarget variable will turn true if ANY PipelineResult within this loop has a target
                                hasTarget = true;

                                // grabs the best target from the result and sends to pose estimator, iFF the pose ambiguity is below a (hardcoded) threshold
                                if (!(result.getBestTarget().getPoseAmbiguity() > 0.5)) {
                                    //grabs the target pose, relative to the camera, and compensates for the camera position
                                    robotToTag = cameraPosition.plus(result.getBestTarget().getBestCameraToTarget());
                                    timestamp = result.getTimestampSeconds();
                                } else {
                                    DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose ambiguity is high");
                                }

                                // grabs the distance to the best target (for the latest set of result)
                                totalDistances += result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

                                hasTargetPublisher.set(true);
                                targetsFoundPublisher.set(numberOfResults);
                                timestampPublisher.set(result.getTimestampSeconds());
                                posePublisher.set(Pose3d.kZero.plus(robotToTag));
                            } else {
                                hasTargetPublisher.set(false);
                                targetsFoundPublisher.set(0);
                            }
                        }

                        // averages distance over all results
                        averageDistance = totalDistances / numberOfResults;
                        updates = new Tuple<Transform2d, Double>(new Transform2d(robotToTag.getTranslation().toTranslation2d(), robotToTag.getRotation().toRotation2d()), timestamp);
                        this.hasTarget = hasTarget;
                        if (hasTarget) {
                            //primary call to the synchronized method that sends vision updates to the drivetrain
                            updateVision();
                        }
                    } catch (IndexOutOfBoundsException e) {
                        // if there are no results,
                        this.hasTarget = false;
                    }
                    try {
                        sleep(5);
                    } catch (InterruptedException e) {
                        DataLogManager.log(camName.toString() + " sleep failed"); //this will never happen
                    }
                }
            }
        }

        /**
         * Returns the most recent pose and average distance to the best target <p>
         *
         * using a tuple as a type-safe alternative to the classic "return an array" (i hate java) <p>
         * this is also thread-safe, and will onlu return the most recent values from the same timestamp <p>
         *
         * @return Tuple<EstimatedRobotPose, Double> - the most recent pose and average distance to the best target
         */
        public Tuple<Transform2d, Double> getUpdates() {
            return updates;
        }

        /**
         * Returns if the camera (during latest loop cycle) has a target
         *
         * This is separate from the value on the NetworkTables, as this value is updated for the entire loop cycle <p>
         * there is an edge case where a target is found, but the pose is not updated <p>
         * likewise, there is an edge case where a target is found and then lost, but the pose is updated <p>
         * to solve this, the hasTarget value is marked as true iff any update has been sent to the estimator <p>
         * @ImplNote this is NOT strictly synchronized with the value from {@link #getUpdates()}; be careful when using this value. should use PhotonVision's hasTarget() function for most cases
         * @return has a target or not
         */
        public boolean hasTarget() {
            return hasTarget;
        }

        private PhotonCamera getCameraObject() {
            return camera;
        }

        private void initializeCamera() {
                camera = new PhotonCamera(camName.toString());
                cameraInitialized = true;
        }

        // Initializes the camera with the given name. This is intended for the master robot, if it is not running pose estimation.
        // The master robot runs the TimeServer for the slave(s), so it is still required for the camera to be initialized.
        @SuppressWarnings("resource")
        public static void initializeCamera(String cameraName) {
            new PhotonCamera(cameraName.toString());
        }
    }


    /**
     * multicam imp'l of {@link #updateVision()}
     * @param caller - the camera that called the function, used to determine which camera's pose to use
     */
    // private synchronized void updateVision(CameraName caller) {
    //     Tuple<EstimatedRobotPose, Double> leftUpdates = camThread.getUpdates();
    //     Tuple<EstimatedRobotPose, Double> rightUpdates = rightThread.getUpdates();

    //     final double maxAcceptableDist = 4d;
    //     boolean shouldUpdateLeft = true;
    //     boolean shouldUpdateRight = true;


    //     // prefer the camera that called the function (has known good values)
    //     // if the other camera has a target, prefer the one with the lower distance to best tag
    //     switch (caller) {
    //         case LEFT:
    //             if((rightThread.hasTarget() && rightUpdates.v < leftUpdates.v) && shouldUpdateRight) {
    //                 drivetrain.addVisionMeasurement(rightUpdates.k, rightUpdates.v);
    //             } else if (shouldUpdateLeft) {
    //                 drivetrain.addVisionMeasurement(leftUpdates.k, leftUpdates.v);
    //             }
    //         break;
    //         case RIGHT:
    //             if((camThread.hasTarget() && leftUpdates.v < rightUpdates.v) && shouldUpdateLeft) {
    //                 drivetrain.addVisionMeasurement(leftUpdates.k, rightUpdates.v);
    //             } else if (shouldUpdateRight) {
    //                 drivetrain.addVisionMeasurement(rightUpdates.k, rightUpdates.v);
    //             }
    //         break;
    //     }
    // }

}
