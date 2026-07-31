// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

final class DriveSimulationPoseTracker {
  private final SwerveDriveOdometry odometry;

  DriveSimulationPoseTracker(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPose) {
    odometry = new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPose);
  }

  void update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    odometry.update(gyroAngle, modulePositions);
  }

  void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  Pose2d getPose() {
    return odometry.getPoseMeters();
  }
}
