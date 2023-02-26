package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtensionCommand  extends CommandBase {

    private Arm _arm;
    private boolean _isExtending;
    private double _duration;
    private Timer _timer;

    /** Creates a new ArmCommand. */
    public ExtensionCommand(Arm arm, boolean isExtending, double runDurationInSeconds) {
        this._arm = arm;
        this._isExtending = isExtending;
        this._duration = runDurationInSeconds;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _timer = new Timer();
        _timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_isExtending) {
            _arm.extensionMove(-.4);
        } else {
            _arm.extensionMove(.4);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _arm.shoulderMove(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(_duration);
    }
}