package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmPositionCommand  extends CommandBase {

    private Arm _arm;
    private double _expectedPosition;
    private double _duration;
    private Timer _timer;

    /** Creates a new ArmCommand. */
    public ArmPositionCommand(Arm arm, double position, double runDurationInSeconds) {
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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("expectedPosition", _expectedPosition);
        if(_expectedPosition > ArmConstants.shoulderEncoderBottom && _expectedPosition < ArmConstants.shoulderEncoderMax) {
            System.out.println("going to position: " + _expectedPosition + " at " + _arm.getShoulderPosition());
            _arm.setShoulderPosition(_expectedPosition);            
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
        double positionVariance = 2;
        boolean armInPosition = ((_arm.getShoulderPosition() + positionVariance) >= _expectedPosition || (_arm.getShoulderPosition() - positionVariance) <= _expectedPosition);
        return armInPosition || _timer.hasElapsed(_duration);
    }
}