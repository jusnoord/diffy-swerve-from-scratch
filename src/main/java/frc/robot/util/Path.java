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
    private double speed;
    private int currentWaypointIndex = 0;

    public Path(List<Pose2d> waypoints) {
        this.waypoints = waypoints;
    }

    public Pose2d getNextWaypoint(Pose2d currentPose) {
        while (currentWaypointIndex < waypoints.size()) {
            Pose2d waypoint = waypoints.get(currentWaypointIndex);
            double distance = currentPose.getTranslation().getDistance(waypoint.getTranslation());
            if (distance > lookAhead) {
                return waypoint;
            }
            currentWaypointIndex++;
        }
        return waypoints.get(waypoints.size() - 1); 
    }

    public Pose2d getVelocity(Pose2d currentPose, PIDController angleController) {
        Pose2d nextWaypoint = getNextWaypoint(currentPose);
        double dx = nextWaypoint.getX() - currentPose.getX();
        double dy = nextWaypoint.getY() - currentPose.getY();
        double angle = Math.atan2(dy, dx);
        return new Pose2d(speed * Math.cos(angle), speed * Math.sin(angle), Rotation2d.fromDegrees(angleController.calculate(currentPose.getRotation().getDegrees(), nextWaypoint.getRotation().getDegrees())));
    }
}
