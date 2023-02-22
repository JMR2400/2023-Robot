package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

    private CommandXboxController _controller;

    private Intake _intake;

    /** Creates a new IntakeCommand. */
    public IntakeCommand(Intake intake, CommandXboxController controller) {
        this._intake = intake;
        this._controller = controller;

        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(_controller.getRightY()) >= .07) {
            _intake.intakeMove(_controller.getRightY());
        } else {
            _intake.intakeMove(0);
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