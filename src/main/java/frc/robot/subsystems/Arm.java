package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.CounterBase;

public class Arm extends SubsystemBase {

    private static Arm instance;

    private CANSparkMax extensionMotor;
    
    private CANSparkMax shoulderMotor;
    private RelativeEncoder shoulderEncoder;
    private SparkMaxPIDController shoulderPIDController;
    private final Encoder arm_QuadEncoder;
    private final DutyCycleEncoder arm_AbsEncoder;
    private AnalogInput extPot;
    
    public static Arm getInstance() {
        if(instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
        extPot = new AnalogInput(0);

        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ArmConstants.shoulderMotorId, MotorType.kBrushless);
        arm_QuadEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k4X);
        /*
        * Defines the number of samples to average when determining the rate.
        * On a quadrature encoder, values range from 1-255;
        * larger values result in smoother but potentially
        * less accurate rates than lower values.
        */
        arm_QuadEncoder.setSamplesToAverage(5);
        /*
        * Defines how far the mechanism attached to the encoder moves per pulse. In
        * this case, we assume that a 360 count encoder is directly
        * attached to a 3 inch diameter (1.5inch radius) wheel,
        * and that we want to measure distance in inches.
        */
        arm_QuadEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
        /*
        * Defines the lowest rate at which the encoder will
        * not be considered stopped, for the purposes of
        * the GetStopped() method. Units are in distance / second,
        * where distance refers to the units of distance
        * that you are using, in this case inches.
        */
        arm_QuadEncoder.setMinRate(1.0);


        // Initializes a duty cycle encoder on DIO pins 0
        arm_AbsEncoder = new DutyCycleEncoder(0);
        // Configures the encoder to return a distance of 4 for every rotation
        arm_AbsEncoder.setDistancePerRotation(360.0);


        
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        extensionMotor.restoreFactoryDefaults();
        // shoulderMotor.restoreFactoryDefaults();

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
        // SmartDashboard.putNumber("Extension Encoder", extensionEncoder.getPosition());
    }

    public double getExtensionPosition() {
        SmartDashboard.putNumber("Extension Posotion", extPot.getValue());
        return extPot.getValue();
    }

    public void setExtensionPosition(double position) {
        // extensionPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getAbsArmPos(){
        return arm_AbsEncoder.getDistance();
    };

    public double getQuadPos(){
        return arm_QuadEncoder.getDistance();
    }

    public double get_QuadArmRate(){
        return arm_QuadEncoder.getRate();
    }

}
