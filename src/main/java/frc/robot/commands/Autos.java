// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
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

  public static CommandBase centerRamp(Drive drive, NavXGyro gyro) {

    // PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Center-Ramp", 5,
    // 3);

    List<PathPlannerTrajectory> pathTrajectoryGroup = PathPlanner.loadPathGroup("Center-Ramp",
        new PathConstraints(1, 1), new PathConstraints(2, 2), new PathConstraints(5, 3));
    PPSwerveControllerCommand cubeDropDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(0), false, drive);
    PPSwerveControllerCommand overRampDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(1), false, drive);
    PPSwerveControllerCommand backToRampDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(2), false, drive);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drive.resetOdometry(pathTrajectoryGroup.get(0).getInitialHolonomicPose());
        }),
        cubeDropDriveCommand,
        overRampDriveCommand,
        backToRampDriveCommand,
        new DriveBalanceCommand(drive, gyro));
  }

  public static CommandBase barrierConeRamp(Drive drive, NavXGyro gyro, Intake intake, Arm arm) {

    // PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Center-Ramp", 5,
    // 3);

    List<PathPlannerTrajectory> pathTrajectoryGroup = PathPlanner.loadPathGroup("Barrier-K-Ramp",
        new PathConstraints(1, 1), new PathConstraints(3, 3), new PathConstraints(5, 3));
    PPSwerveControllerCommand cubeDropDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(0), false, drive);
    PPSwerveControllerCommand exitCommunityDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(1), false,
        drive);
    PPSwerveControllerCommand backToRampDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(2), false, drive);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drive.resetOdometry(pathTrajectoryGroup.get(0).getInitialHolonomicPose());
        }),
        cubeDropDriveCommand,
        new ParallelCommandGroup(
          exitCommunityDriveCommand,
            new ArmPositionCommand(arm, ArmConstants.armShoulderPosition, ArmConstants.armExtensionPosition, 3),
            new IntakeConeCommand(intake, false, 3.5)),
        backToRampDriveCommand,
        new DriveBalanceCommand(drive, gyro));
  }

  public static CommandBase barrierCubeRamp(Drive drive, NavXGyro gyro, Intake intake, Arm arm) {

    // PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Center-Ramp", 5,
    // 3);

    List<PathPlannerTrajectory> pathTrajectoryGroup = PathPlanner.loadPathGroup("Barrier-K-Ramp",
        new PathConstraints(1, 1), new PathConstraints(3, 3), new PathConstraints(5, 3));
    PPSwerveControllerCommand cubeDropDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(0), false, drive);
    PPSwerveControllerCommand exitCommunityDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(1), false,
        drive);
    PPSwerveControllerCommand backToRampDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(2), false, drive);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drive.resetOdometry(pathTrajectoryGroup.get(0).getInitialHolonomicPose());
        }),
        cubeDropDriveCommand,
        new ParallelCommandGroup(
          exitCommunityDriveCommand,
            new ArmPositionCommand(arm, ArmConstants.armShoulderPosition, ArmConstants.armExtensionPosition, 3),
            new IntakeCubeCommand(intake, false, 4)),
        backToRampDriveCommand,
        new DriveBalanceCommand(drive, gyro));
  }

  public static CommandBase cableDriveStraight(Drive drive) {

    // Generate trajectory
    // PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath("Cable-Straight",
    // new PathConstraints(1.5, 2));
    List<PathPlannerTrajectory> pathTrajectoryGroup = PathPlanner.loadPathGroup("Cable-Straight",
        new PathConstraints(1.5, 2), new PathConstraints(2, 2), new PathConstraints(5, 3));

    PPSwerveControllerCommand cubeJigCommand = getTrajectoryCommand(pathTrajectoryGroup.get(0), false, drive);
    PPSwerveControllerCommand beforeCableDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(1), false, drive);
    PPSwerveControllerCommand afterCableDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(2), false, drive);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drive.resetOdometry(pathTrajectoryGroup.get(0).getInitialHolonomicPose());
        }),
        cubeJigCommand,
        beforeCableDriveCommand,
        afterCableDriveCommand);
  }

  public static CommandBase barrierDriveStraight(Drive drive) {

    // Generate trajectory
    // PathPlannerTrajectory pathTrajectory =
    // PathPlanner.loadPath("Barrier-Straight", new PathConstraints(4, 3));
    List<PathPlannerTrajectory> pathTrajectoryGroup = PathPlanner.loadPathGroup("Cable-Straight",
        new PathConstraints(1.5, 2), new PathConstraints(5, 3));

    PPSwerveControllerCommand cubeJigCommand = getTrajectoryCommand(pathTrajectoryGroup.get(0), false, drive);
    PPSwerveControllerCommand exitDriveCommand = getTrajectoryCommand(pathTrajectoryGroup.get(1), false, drive);

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          drive.resetOdometry(pathTrajectoryGroup.get(0).getInitialHolonomicPose());
        }),
        cubeJigCommand,
        exitDriveCommand);
  }

  private static PPSwerveControllerCommand getTrajectoryCommand(PathPlannerTrajectory pathTrajectory,
      boolean useAllianceColor, Drive drive) {
    // Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 1, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Return PID command
    return new PPSwerveControllerCommand(
        pathTrajectory,
        drive::getPose,
        DriveConstants.FrameConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        drive::setModuleStates,
        useAllianceColor,
        drive);
  }
}