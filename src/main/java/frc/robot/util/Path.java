// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Path {
    private List<Pose2d> waypoints;
    private double lookAhead;
    private final double defaultSpeed;
    private int currentWaypointIndex = 0;

    public Path(List<Pose2d> waypoints, double defaultSpeed, double lookAhead) {
        this.waypoints = waypoints;
        this.defaultSpeed = defaultSpeed;
        this.lookAhead = lookAhead;
    }

    /** 
     * sets next waypoint to the interpolated position between the two closest waypoints that's closest to the current robot pose
     * @param currentPose current robot pose
     */
    public void getClosestWaypoint(Pose2d currentPose) {
        double closestDistance = Double.MAX_VALUE;
        int closestWaypointIndex = 0;
        int secondClosestWaypointIndex = 0;
        
        // find two closest waypoints
        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d waypoint = waypoints.get(i);
            double distance = currentPose.getTranslation().getDistance(waypoint.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                secondClosestWaypointIndex = closestWaypointIndex;
                closestWaypointIndex = i;
            }
        }

        //TODO: handle if closest waypoints are not adjacent

        // interpolate between the two closest waypoints
        // get relative distances between current position and both waypoints
        Pose2d closestWaypoint = waypoints.get(closestWaypointIndex);
        Pose2d secondClosestWaypoint = waypoints.get(secondClosestWaypointIndex);
        double distanceToClosest = currentPose.getTranslation().getDistance(closestWaypoint.getTranslation());
        double distanceToSecondClosest = currentPose.getTranslation().getDistance(secondClosestWaypoint.getTranslation());

        // project current position onto the line between the two waypoints
        /**
         * A = closest waypoint
         * B = second closest waypoint
         * P = current position
         * 
         * AP ⋅ AB / |AB|^2 = t
         */
        double Ax = closestWaypoint.getX();
        double Ay = closestWaypoint.getY();
        double Bx = secondClosestWaypoint.getX();
        double By = secondClosestWaypoint.getY();
        double Px = currentPose.getX();
        double Py = currentPose.getY();

        double ABx = Bx - Ax;
        double ABy = By - Ay;
        double APx = Px - Ax;
        double APy = Py - Ay;

        double ABSquared = (ABx * ABx) + (ABy * ABy);
        double projScalar = (APx * ABx + APy * ABy) / ABSquared;

        double projX = Ax + projScalar * ABx;
        double projY = Ay + projScalar * ABy;

        Pose2d projectedPoint = new Pose2d(projX, projY, new Rotation2d());

        // interpolate between the two waypoints based on the projected point
        double distanceAlongLine = (closestWaypoint.getTranslation().getDistance(projectedPoint.getTranslation())) / (closestWaypoint.getTranslation().getDistance(secondClosestWaypoint.getTranslation()));

        Pose2d interpolatedPosition = closestWaypoint.interpolate(secondClosestWaypoint, distanceAlongLine);

        // insert the interpolated position into the waypoints list, between the two closest waypoints
        int newWaypointIndex = Math.min(closestWaypointIndex, secondClosestWaypointIndex) + 1;

        waypoints.add(newWaypointIndex, interpolatedPosition);
        currentWaypointIndex = newWaypointIndex;
    }

    public Pose2d getNextWaypoint(Pose2d currentPose) {
        // iterate through waypoints to find the next one beyond the lookahead distance
        while (currentWaypointIndex < waypoints.size()) {
            Pose2d waypoint = waypoints.get(currentWaypointIndex);
            double distance = currentPose.getTranslation().getDistance(waypoint.getTranslation());
            if (distance > lookAhead) {
                return waypoint;
            }
            currentWaypointIndex++;
        }

        // if no waypoint is found, return the last one
        return waypoints.get(waypoints.size() - 1); 
    }

    public Pose2d getVelocity(Pose2d currentPose, PIDController angleController) {
        Pose2d nextWaypoint = getNextWaypoint(currentPose);
        double distance = currentPose.getTranslation().getDistance(nextWaypoint.getTranslation());
        double speed;
        if (distance >= lookAhead) {
            speed = defaultSpeed; 
        } else {
            speed = defaultSpeed * distance / lookAhead; 
        }
        double dx = nextWaypoint.getX() - currentPose.getX();
        double dy = nextWaypoint.getY() - currentPose.getY();
        double angle = Math.atan2(dy, dx);
        return new Pose2d(speed * Math.cos(angle), speed * Math.sin(angle), Rotation2d.fromDegrees(angleController.calculate(currentPose.getRotation().getDegrees(), nextWaypoint.getRotation().getDegrees())));
    }
}
