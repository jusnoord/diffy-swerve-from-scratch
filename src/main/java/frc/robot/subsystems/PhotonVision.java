// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

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
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.RobotMap.CameraName;
import frc.robot.util.Triplet;
import frc.robot.util.Tuple;

/**
 * PhotonVision subsystem constructs a Photon Vision camera and runs multi-threaded pose estimation.
 * Handles vision processing, pose updates, and communication with drivetrain.
 */
public class PhotonVision extends SubsystemBase {
    public static Pose2d closestOtherPose = new Pose2d();

    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    List<TimestampedObject<Pose2d>> timestampedCurrentPoses = new ArrayList<>(); // pull from NT depending on is_master
    List<TimestampedObject<Pose2d>> timestampedOtherPoses = new ArrayList<>(); // pull from NT depending on is_master

    private StructPublisher<Pose2d> posePublisher; // needs to be used somewhere i htink
    private StructSubscriber<Pose2d> poseSubscriber; // TODO: balance the NT publishing between master and slave

    private HashMap<Double, Pose2d> masterPoses = new HashMap<>();

    public Pose2d pose = new Pose2d();

    private Swerve drivetrain;

    private CameraThread camThread;

    public PhotonVision(Swerve drivetrain, CameraName camName) {
        this.drivetrain = drivetrain;

        camThread = new CameraThread(camName, RobotConstants.SLAVE_CAMERA_LOCATION);
        camThread.start();

        //this code grabs the pose from the other robot
        poseSubscriber = NetworkTableInstance.getDefault().getTable(Constants.currentRobot.getOpposite().toString()).getStructTopic("RobotPose", Pose2d.struct).subscribe(new Pose2d());
        
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

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getMasterPosition(Double timestamp) {
        return (Constants.IS_MASTER) ? getCurrentRobotPosition(timestamp) : getOtherRobotPosition(timestamp);
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getSlavePosition(Double timestamp) {
        return (!Constants.IS_MASTER) ? getCurrentRobotPosition(timestamp) : getOtherRobotPosition(timestamp);
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getCurrentRobotPosition(Double timestamp) {
        return drivetrain.getPose();
    }

    /** 
     * @param timestamp time in seconds
     * @return master absolute position at time timestamp
     */
    Pose2d getOtherRobotPosition(Double timestamp) {
        timestampedOtherPoses.addAll(List.of(poseSubscriber.readQueue()));

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedOtherPoses.size() > 10) {
            timestampedOtherPoses = timestampedOtherPoses.subList(timestampedOtherPoses.size() - 10, timestampedOtherPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PhotonVision timestamps are seconds. Mulitiply by one million to convert/
        double visionTimeStampMicroSeconds = timestamp * 1000000d;
        closestOtherPose = timestampedOtherPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        return closestOtherPose;
    }

    /** 
     * @param pose absolute pose of robot to inject into kalman filter
     * @param timestamp time of pose update, in seconds
     * @param ambiguity proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the master-centric wing estimator to match
     */
    void updateMaster(Pose2d pose, Double timestamp, double ambiguity) {
        if (Constants.IS_MASTER) {
            updateCurrentRobot(pose, timestamp, ambiguity);
        } else {
            updateOtherRobot(pose, timestamp, ambiguity);
        } 
    }
    
    /** 
     * @param pose absolute pose of robot to inject into kalman filter
     * @param timestamp time of pose update, in seconds
     * @param ambiguity proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the slave-centric wing estimator to match
     */
    void updateSlave(Pose2d pose, Double timestamp, double ambiguity) {
        if (Constants.IS_MASTER) {
            updateOtherRobot(pose, timestamp, ambiguity);
        } else {
            updateCurrentRobot(pose, timestamp, ambiguity);
        } 
    }
    
    /** 
     * @param pose absolute pose of robot to inject into kalman filter
     * @param timestamp time of pose update, in seconds 
     * @param ambiguity proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the robot-centric wing estimator to match
     */
    void updateCurrentRobot(Pose2d pose, Double time, double ambiguity) {
        drivetrain.addVisionMeasurement(pose, ambiguity, ambiguity);
    }

    /** 
     * @param pose absolute pose of robot to inject into kalman filter
     * @param timestamp time of pose update, in seconds 
     * @param ambiguity proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the robot-centric wing estimator to match
     */
    void updateOtherRobot(Pose2d pose, Double time, double ambiguity) {
        // TODO
    }

    /** 
     * @param pose absolute pose of wing to inject into kalman filter
     * @param timestamp time of pose update, in seconds (does not matter because wing does not have odometry)
     * @param ambiguity proportional measure of uncertainty, usually just distance in meters
     * this function will additionally update the robot-centric wing estimator to match
     */
    void updateCurrentRobotWingEstimate(Pose2d pose, Double time, double ambiguity) {
        // TODO
    }

    /** 
     * updates both the master and slave equally in opposite directions
     * updates masterWing and slaveWing pose to follow
     * requires master pose, slave pose, slave/master new pose
     * 
     * Notes:
     * calculated exclusively on master and sent to slave
     * used for offsets between robots
     * @param update measured displacement between robots along with timestamp of measurement
     */
    private synchronized void updateLocalVision(Tuple<Transform2d, Double> update) {
        if (Constants.IS_MASTER) {
            // update pose estimates for both master and slave

            Double timestamp = update.v;
            double ambiguity = update.k.getTranslation().getNorm();

            Pose2d masterPosition = getMasterPosition(timestamp);
            Pose2d slavePosition = getSlavePosition(timestamp);

            Transform2d measuredDisplacement = update.k;
            Transform2d currentDisplacement = masterPosition.minus(slavePosition);
            Transform2d difference = currentDisplacement.plus(measuredDisplacement.inverse());

            Pose2d newMasterPose = masterPosition.plus(difference.times(0.5));
            Pose2d newSlavePose = slavePosition.plus(difference.times(-0.5));
            updateMaster(newMasterPose,timestamp,ambiguity);
            updateSlave(newSlavePose,timestamp,ambiguity);
        } else {
            // send vision offset data to master to process

            
        }
    }

    /** 
     * updates master, slave, and wing poses equally
     * requires current robot absolute vision measurement and current robot pose
     * 
     * Notes:
     * calculates residual as intermediate step and sends residual to all estimators for update
     * used for global wall tags
     * @param update measured absolute position of current robot along with timestamp of measurement
     */
    private synchronized void updateGlobalVision(Tuple<Transform2d, Double> update) {
        Pose2d currentRobotPose = getCurrentRobotPosition(update.v);

        Pose2d displacement= currentRobotPose.plus(update.k.inverse());
        
        Pose2d otherRobotPose = getOtherRobotPosition(update.v);

        Transform2d otherRobotNewTransform = otherRobotPose.minus(displacement.times(-1));

        Pose2d otherRobotNewPose = new Pose2d(otherRobotNewTransform.getTranslation(), otherRobotNewTransform.getRotation());

        updateCurrentRobot(new Pose2d(update.k.getTranslation(), update.k.getRotation()), update.v, update.k.getTranslation().getNorm()); // TODO: this pose2d and transform2d math is definitely wrong
        updateOtherRobot(otherRobotNewPose, update.v, update.k.getTranslation().getNorm());

        // TODO: fix global variables not being updated anywhere
        // posePublisher.accept(fieldRelativePose);
        // pose = fieldRelativePose;
    }

    /**
     * updates only the wing pose relative to the current robot
     * requires current pose and robot to wing offset
     * 
     * Notes:
     * each robot has an independent wing pose estimator 
     * used for wing-specific tags
     */
    private synchronized void updateWingVision(Tuple<Transform2d, Double> update) {
        Pose2d currentRobotPose = getCurrentRobotPosition(update.v);
        Pose2d wingPose = currentRobotPose.plus(update.k);
        updateCurrentRobotWingEstimate(wingPose, update.v, update.k.getTranslation().getNorm());
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
                        double ambiguity = 0; //unused for now

                        //fundamentally, this loop updates the pose and distance for each result. It also logs the data to shuffleboard
                        //this is done in a thread-safe manner, as global variables are only updated at the end of the loop (no race conditions)
                        for (PhotonPipelineResult result : results) {
                            if (result.hasTargets()) {
                                // the local hasTarget variable will turn true if ANY PipelineResult within this loop has a target
                                hasTarget = true;
                                timestamp = result.getTimestampSeconds();
                                boolean tagIdentified = false;

                                //primary tag isolation switch
                                if(VisionConstants.RobotTagIDs.contains(result.getBestTarget().getFiducialId())) {
                                    tagIdentified = true;
                                    isolateToTagSet(result, VisionConstants.RobotTagIDs);

                                    // try multi-tag first
                                    var multiTagUpdate = doMultiTagUpdate(result);
                                    if (multiTagUpdate.isPresent()) {
                                        var tuple = multiTagUpdate.get();
                                        robotToTag = tuple.k;
                                        ambiguity = tuple.v;
                                    } else {
                                        //fall back to single tag
                                        var singleTagUpdate = doSingleTagUpdate(result);
                                        if (singleTagUpdate.isPresent()) {
                                            var tuple = singleTagUpdate.get();
                                            robotToTag = tuple.k;
                                            ambiguity = tuple.v;
                                        } else {
                                            //both methods failed
                                            DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " both multi-tag and single-tag pose updates failed");
                                            continue;
                                        }
                                    }

                                    //account for tag-to-robot and camera-to-robot offsets and then combine vision measurement with master odometry position
                                    // magical transform that possibly does not work anymore because new robot
                                    Translation2d x = updates.k.getTranslation().plus(RobotConstants.centerOfMasterToTag).rotateBy(drivetrain.getPose().getRotation()).rotateBy(Rotation2d.k180deg);
                                    Rotation2d t = updates.k.getRotation().unaryMinus().rotateBy(Rotation2d.kCW_90deg);
                                    Transform2d visionDisplacement = new Transform2d(x, t);
                                    updateLocalVision(new Tuple<>(visionDisplacement, updates.v));
                               } 
                                
                                if(VisionConstants.StationTagIDs.contains(result.getBestTarget().getFiducialId())) {
                                    tagIdentified = true;
                                    isolateToTagSet(result, VisionConstants.StationTagIDs);
                                    // only single-tag for station tags
                                    var singleTagUpdate = doSingleTagUpdate(result);
                                    if (singleTagUpdate.isPresent()) {
                                        var tuple = singleTagUpdate.get();
                                        robotToTag = tuple.k;
                                        ambiguity = tuple.v;
                                    } else {
                                        //single-tag method failed
                                        DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " single-tag pose update failed for station tag");
                                    }

                                    updateGlobalVision(updates);                                    
                                } 
                                
                                if(VisionConstants.WingTagIDs.contains(result.getBestTarget().getFiducialId())) {
                                    tagIdentified = true;

                                    isolateToTagSet(result, VisionConstants.WingTagIDs);
                                    // only single-tag for wing tags
                                    var singleTagUpdate = doSingleTagUpdate(result);
                                    if (singleTagUpdate.isPresent()) {
                                        var tuple = singleTagUpdate.get();
                                        robotToTag = tuple.k;
                                        ambiguity = tuple.v;
                                    } else {
                                        //single-tag method failed
                                        DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " single-tag pose update failed for wing tag");
                                    }

                                    updateWingVision(updates);
                                }

                                if(!tagIdentified) {
                                    System.out.println("[PhotonVision] INFO: " + camName.toString() + " ignoring tag ID " + result.getBestTarget().getFiducialId());
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

        /**
         * Performs a multi-tag pose update using the given pipeline result.
         * @param result The pipeline result containing detected targets. ASSUMES NOT EMPTY!!
         * @return An Optional containing the computed Transform3d and ambiguity if successful; empty otherwise.
         */
        private Optional<Tuple<Transform3d, Double>> doMultiTagUpdate(PhotonPipelineResult result) {
            if (result.multitagResult.isPresent()) {
                var multiTag = result.multitagResult.get();
                // Use getCameraToRobot() to get the transform from camera to robot
                Transform3d cameraToRobot = multiTag.estimatedPose.best;
                double ambiguity = multiTag.estimatedPose.ambiguity;
                return Optional.of(new Tuple<Transform3d, Double>(cameraToRobot, ambiguity));
            }
            //else
            DataLogManager.log("[PhotonVision] INFO: " + camName.toString() + " multitag result requested but not present");
            return Optional.empty();
        }

        private Optional<Tuple<Transform3d, Double>> doSingleTagUpdate(PhotonPipelineResult result) {
            Transform3d robotToTag = new Transform3d();
            double ambiguity;

            // grabs the best target from the result and sends to pose estimator, iFF the pose ambiguity is below a (hardcoded) threshold
            if (!(result.getBestTarget().getPoseAmbiguity() > 0.5)) {
                //grabs the target pose, relative to the camera, and compensates for the camera position
                robotToTag = cameraPosition.plus(result.getBestTarget().getBestCameraToTarget());
                ambiguity = result.getBestTarget().getPoseAmbiguity();

                return Optional.of(new Tuple<Transform3d, Double>(robotToTag, ambiguity));
            } 
            //else
            DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose ambiguity is high");
            return Optional.empty();
        }

        private PhotonPipelineResult isolateToTagSet(PhotonPipelineResult result, List<Integer> validTagIDs) {
            // remove targets whose fiducialId is not in validTagIDs. i love arraylists :)
            result.getTargets().removeIf(target -> !validTagIDs.contains(target.getFiducialId()));

            return result;
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




    /* original single-camera vision update method
    private synchronized void updateLocalVision() {
        Tuple<EstimatedRobotPose, Double> update = camThread.getUpdates();
        timestampedMasterPoses.addAll(List.of(masterPoseSubscriber.readQueue()));

        //trim timestampedMasterPoses to 10 values (for efficiency)
        if (timestampedMasterPoses.size() > 10) {
            timestampedMasterPoses = timestampedMasterPoses.subList(timestampedMasterPoses.size() - 10, timestampedMasterPoses.size());
        }

        //find the master pose with the closest timestamp to the camera's timestamp
        //NT timestamps are measured in microseconds, PhotonVision timestamps are seconds. Mulitiply by one million to convert/
        double visionTimeStampMicroSeconds = update.v * 1000000d;
        closestMasterPose = timestampedMasterPoses.stream()
            .min((a, b) -> Double.compare(Math.abs(a.serverTime - visionTimeStampMicroSeconds), Math.abs(b.serverTime - visionTimeStampMicroSeconds)))
            .map(pose -> pose.value)
            .orElse(null);
        
        
        //account for tag-to-robot and camera-to-robot offsets and then combine vision measurement with master odometry position
        //hey so actually have zero clue how this works!!
        Translation2d x = update.k.getTranslation().plus(RobotConstants.centerOfMasterToTag).rotateBy(drivetrain.getPose().getRotation()).rotateBy(Rotation2d.k180deg);
        Rotation2d t = update.k.getRotation().unaryMinus().rotateBy(Rotation2d.kCW_90deg);
        Transform2d visionTranslation = new Transform2d(x, t);

        Pose2d fieldRelativePose = new Pose2d(closestMasterPose.getTranslation().plus(visionTranslation.getTranslation()), closestMasterPose.getRotation().plus(visionTranslation.getRotation()));


        // add the master pose to the translation to get field-relative pose. 
        // grab the timestamp
        // grab the distance to the best tag
        drivetrain.addVisionMeasurement(fieldRelativePose, update.v, update.k.getTranslation().getNorm());

        posePublisher.accept(fieldRelativePose);
        pose = fieldRelativePose;

    }
        */
