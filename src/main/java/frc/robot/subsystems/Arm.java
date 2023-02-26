package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private static Arm instance;

    private CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkMaxPIDController extensionPIDController;
    
    private CANSparkMax shoulderMotor;
    private RelativeEncoder shoulderEncoder;
    private SparkMaxPIDController shoulderPIDController;
    
    public static Arm getInstance() {
        if(instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ArmConstants.shoulderMotorId, MotorType.kBrushless);

        
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        extensionMotor.restoreFactoryDefaults();
        // shoulderMotor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the shoulder and extension SPARKS MAX.
        extensionEncoder = extensionMotor.getEncoder();        
        extensionPIDController = extensionMotor.getPIDController();
        extensionPIDController.setFeedbackDevice(extensionEncoder);

        extensionPIDController.setP(ArmConstants.extensionP);
        extensionPIDController.setI(ArmConstants.extensionI);
        extensionPIDController.setD(ArmConstants.extensionD);
        extensionPIDController.setOutputRange(ArmConstants.extensionMinOutput, ArmConstants.extensionMaxOutput);

        shoulderEncoder = shoulderMotor.getAlternateEncoder(Type.kQuadrature, 8192);
        // shoulderEncoder = shoulderMotor.getEncoder();
        shoulderPIDController = shoulderMotor.getPIDController();
        shoulderPIDController.setFeedbackDevice(shoulderEncoder);
       
        // Apply position and velocity conversion factors for the shoulder encoder.
        shoulderEncoder.setPositionConversionFactor(ArmConstants.shoulderEncoderPositionFactor);
        shoulderEncoder.setVelocityConversionFactor(ArmConstants.shoulderEncoderVelocityFactor);

        // Set the PID gains for the shoulder motor. 
        // Note these are example gains, and you may need to tune them for your own robot!
        shoulderPIDController.setP(ArmConstants.shoulderP);
        shoulderPIDController.setI(ArmConstants.shoulderI);
        shoulderPIDController.setD(ArmConstants.shoulderD);
        // shoulderPIDController.setOutputRange(ArmConstants.shoulderMinOutput, ArmConstants.shoulderMaxOutput);
        
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderMotor.setInverted(true);
        shoulderMotor.setOpenLoopRampRate(2);
        //shoulderMotor.setSmartCurrentLimit(ArmConstants.kshoulderMotorCurrentLimit);
        
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setSmartCurrentLimit(ArmConstants.extensionMotorCurrentLimit);

        // extensionMotor.setInverted(true);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        extensionMotor.burnFlash();
        // shoulderMotor.burnFlash();
    }

    public void shoulderMove(double speed) {
        shoulderMotor.set(speed);
        SmartDashboard.putNumber("Shoulder Encoder", shoulderEncoder.getPosition());
    }

    public double getShoulderPosition() {
        return shoulderEncoder.getPosition();
    }

    public void setShoulderPosition(double position) {        
        SmartDashboard.putNumber("Shoulder Encoder", shoulderEncoder.getPosition());
        shoulderPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void extensionMove(double speed) {
        extensionMotor.set(speed);
        SmartDashboard.putNumber("Extension Encoder", extensionEncoder.getPosition());
    }

    public double getExtensionPosition() {
        return extensionEncoder.getPosition();
    }

    public void setExtensionPosition(double position) {
        extensionPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
}
