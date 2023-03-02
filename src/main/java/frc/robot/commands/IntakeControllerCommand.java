package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeControllerCommand extends CommandBase {

    private CommandXboxController _controller;

    private Intake _intake;

    /** Creates a new IntakeCommand. */
    public IntakeControllerCommand(Intake intake, CommandXboxController controller) {
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
        if (Math.abs(_controller.getLeftTriggerAxis()) >= .02) {
            // Cone Intake
            _intake.intakeMove(_controller.getLeftTriggerAxis());
        // } else if (_controller.leftBumper().getAsBoolean()) {
        //     // Cube Intake
        //     _intake.intakeMove(-0.6);
        } else if (Math.abs(_controller.getRightTriggerAxis()) >= .02) {
            // Cone Output
            _intake.intakeMove(_controller.getRightTriggerAxis()*-1);
        // } else if (_controller.rightBumper().getAsBoolean()) {
        //     // Cube Output
        //     _intake.intakeMove(-0.6);
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