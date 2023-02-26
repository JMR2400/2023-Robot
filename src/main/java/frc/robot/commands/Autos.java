// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.DoNothingCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavXGyro;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase doNothing() {
    return new DoNothingCommand();
  }

  public static CommandBase followPath(Drive drive, NavXGyro navX) {

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 1, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 2. Generate trajectory
    PathPlannerTrajectory driveForwardTrajectory = PathPlanner.loadPath("180-Cable-K-Ramp", 4, 3);

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
          navX.setGyroAngleOffset(180);
          drive.resetOdometry(driveForwardTrajectory.getInitialHolonomicPose());
        }),
        driveForwardPathCommand);
  }

  public static CommandBase centerRamp(Drive drive) {

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 1, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 2. Generate trajectory
    PathPlannerTrajectory driveForwardTrajectory = PathPlanner.loadPath("Center-Ramp", 4, 3);

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

  public static CommandBase cableConeRamp(Drive drive, Intake intake, Arm arm) {

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 1, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 2. Generate trajectory
    // PathPlannerTrajectory driveForwardTrajectory =
    // PathPlanner.loadPath("Cable-Cone-Ramp", 4, 3);
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("Cable-Cone-Ramp", new PathConstraints(1.5, 2),
        new PathConstraints(4, 3));

    PPSwerveControllerCommand pathOne = new PPSwerveControllerCommand(
        pathGroup1.get(0),
        drive::getPose,
        DriveConstants.FrameConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        drive::setModuleStates,
        drive);

    PPSwerveControllerCommand pathTwo = new PPSwerveControllerCommand(
        pathGroup1.get(1),
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
          drive.resetOdometry(pathGroup1.get(0).getInitialHolonomicPose());
        }),
        pathOne,
        new SequentialCommandGroup(
            new ArmCommand(arm, true, .5),
            new ExtensionCommand(arm, true, .5),
            new IntakeConeCommand(intake, false, 1.5),
            new ExtensionCommand(arm, false, .5),
            new ArmCommand(arm, false, .5)),
        pathTwo);
  }

  public static CommandBase ScoreCone(Intake intake, Arm arm) {
    return new SequentialCommandGroup(
        new ArmCommand(arm, true, .25),
        new ExtensionCommand(arm, true, .25),
        new IntakeConeCommand(intake, true, 1),
        new ExtensionCommand(arm, false, .25),
        new ArmCommand(arm, false, .25));
  }
}