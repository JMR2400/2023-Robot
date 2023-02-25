package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothingCommand extends CommandBase {

  // Instant end the command so nothing happens.
  @Override
  public boolean isFinished() {
    return true;
  }
}