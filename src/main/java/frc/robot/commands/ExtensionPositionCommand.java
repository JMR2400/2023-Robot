package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtensionPositionCommand  extends CommandBase {

    private Arm _arm;
    private double _expectedPosition;
    private double _duration;
    private Timer _timer;
    private PIDController extensionContoller;
    private double extensionP = 0.008;
    private double extensionI = 0.0;
    private double extensionD = 0.0;


    /** Creates a new ExtensionPositionCommand. */
    public ExtensionPositionCommand(Arm arm, double position, double runDurationInSeconds) {
        this._arm = arm;
        this._expectedPosition = position;
        this._duration = runDurationInSeconds;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _timer = new Timer();
        _timer.start();

        extensionContoller = new PIDController(extensionP, extensionI, extensionD);
        extensionContoller.setTolerance(5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
            double directionforce = extensionContoller.calculate(_arm.getExtensionPosition(), _expectedPosition);
            SmartDashboard.putNumber("Extension Power", directionforce);
            _arm.extensionMove(directionforce);

            // if(_expectedPosition > _arm.getExtensionPosition()) {
            //     _arm.extensionMove(-directionforce);
            // } else if(_expectedPosition < _arm.getExtensionPosition()) {
            //     _arm.extensionMove(directionforce);
            // }            
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _arm.extensionMove(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return extensionContoller.atSetpoint() || _timer.hasElapsed(_duration);
    }
}