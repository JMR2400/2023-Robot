// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public static CommandBase driveToHalfField(Drive drive) {
      // 1. Create trajectory settings
     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
         AutoConstants.kMaxSpeedMetersPerSecond,
         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
             .setKinematics(DriveConstants.FrameConstants.kDriveKinematics);

      // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 1, 0);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      

     // 2. Generate trajectory
     PathPlannerTrajectory driveForwardTrajectory = PathPlanner.loadPath("Drive Forward", 4, 3);

     PPSwerveControllerCommand driveForwardPathCommand = new PPSwerveControllerCommand(
      driveForwardTrajectory,
      drive::getPose,
      DriveConstants.FrameConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      drive::setModuleStates,
      drive);

      return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
              drive.resetOdometry(driveForwardTrajectory.getInitialHolonomicPose());
        }),
        driveForwardPathCommand);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
