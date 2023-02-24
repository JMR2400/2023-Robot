package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {

    private CommandXboxController _controller;

    private Arm _arm;

    /** Creates a new ArmCommand. */
    public ArmCommand(Arm arm, CommandXboxController controller) {
        this._arm = arm;
        this._controller = controller;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(_controller.getLeftY()) >= .07) {
            _arm.shoulderMove(_controller.getLeftY()*.3);
        } else {
            _arm.shoulderMove(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}