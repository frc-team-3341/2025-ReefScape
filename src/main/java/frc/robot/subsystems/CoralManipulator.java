package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase {
    double setpoint;

    public SparkLimitSwitch FWDLimit;
    public SparkLimitSwitch REVLimit;

    private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
    private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);

    AbsoluteEncoder absEncoder;
    SparkClosedLoopController pidPivot;
    boolean enableTeleop = false;

    private final double forwardSoftLimit = 0.09;
    private final double revSoftLimit = -0.23;

    //The zero angle of the abs encoder in degrees. We need to apply all target angles with this offset
    //double zereodOffsetDegrees = Units.rotationsToDegrees(0.425);  // 0˚ reference point
    //private double conversionFactor = Constants.CoralManipulatorConstants.pivotGearRatio/360; //81 rotations of the motor is 1 rotation of the arm
    //deg * (81/360) Dimensional analysis yay --> deg -> rotation conversion
    PIDController pidController = new PIDController(0.5, 0, 0);

    public boolean probablyHasCoral = false;
    

    public CoralManipulator() {
        this.pidPivot = pivotMotor.getClosedLoopController();
        this.absEncoder = pivotMotor.getAbsoluteEncoder();
        
        // Configuration for coral motors
        SparkMaxConfig coralConfig1 = new SparkMaxConfig();
        coralMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig coralConfig2 = new SparkMaxConfig();
        coralConfig2.inverted(true);
        coralMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Configuration for pivot motor
        SparkMaxConfig pivotConfig  = new SparkMaxConfig();
        pivotConfig.closedLoop.pid(
            2, // p
            0,    // i
            0    // d
        );
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        pivotConfig.smartCurrentLimit(10);
        pivotConfig.inverted(true);

        pivotConfig.absoluteEncoder.inverted(true);
        pivotConfig.absoluteEncoder.zeroOffset(.425);
        pivotConfig.absoluteEncoder.zeroCentered(true);

        // Limit switch configuration
        LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
        limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
        limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
        limitSwitchConfig.forwardLimitSwitchEnabled(true);
        limitSwitchConfig.reverseLimitSwitchEnabled(true);
        

        // Soft limit configuration
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimitEnabled(true);
        
        // Updated Soft Limits
        //double forwardSoftLimit = zereodOffsetDegrees + (10.0 / 360.0);    // +10 degrees up
        //double reverseSoftLimit = zereodOffsetDegrees + (-44.0 / 360.0);   // -44 degrees down
        
        softLimitConfig.forwardSoftLimit(forwardSoftLimit);
        softLimitConfig.reverseSoftLimit(revSoftLimit);

        // Apply configurations
        pivotConfig.apply(softLimitConfig);
        
        pivotConfig.apply(limitSwitchConfig);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        FWDLimit = pivotMotor.getForwardLimitSwitch();
        REVLimit = pivotMotor.getReverseLimitSwitch();
    }

    //.115 for L1

    // Commands for pivot control
    public Command pivotL4() {
        return this.runOnce(() -> {
            this.setpoint = -.147;
            this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotIntake() {
        return this.runOnce(() -> {
            this.setpoint = .079;
            pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotPlace() {
        return this.runOnce(() -> {
            this.setpoint = -.1;
            this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
        });
    }

    public Command pivotDown() {
        return this.runOnce(() -> {
            this.setpoint = -0.5;
            this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
        });
    }

    public Command stopCoral() {
        return this.runOnce(() -> {
            coralMotor1.set(0.0);
            coralMotor2.set(0.0);
        });
    }

    public Command intakeCoral() {
        return this.runOnce(() -> {
            coralMotor1.set(0.2);
            coralMotor2.set(0.2);
            probablyHasCoral = true;
        });
    }

    public Command releaseCoral() {
        return this.runOnce(() -> {
            coralMotor1.set(-0.2);
            coralMotor2.set(-0.2);
            probablyHasCoral = false;
        });
    }

    public Command homeDown() {
        return this.runOnce(() -> {
            this.pidPivot.setReference(-.5, SparkMax.ControlType.kPosition);
        });
    }
    
    public void periodic() {
        // SmartDashboard.putNumber("ABSENC POS", this.absEncoder.getPosition());
        SmartDashboard.putNumber("pivot Pos", this.absEncoder.getPosition());
        // SmartDashboard.putNumber("pivot vel", this.absEncoder.getVelocity());
        //SmartDashboard.putNumber("Relative Encoder Angle", this.relEnc.getPosition()/this.conversionFactor);

        //SmartDashboard.putNumber("Angle of Pivot", (absEncoder.getPosition() * 360.0));
        //SmartDashboard.putNumber("Angle from 0", (153.2 - (absEncoder.getPosition() * 360.0))); // angle at our defined 0˚ - current angle
        //SmartDashboard.putNumber("Rotations", (absEncoder.getPosition()));
        SmartDashboard.putBoolean("Pivot FWD Limit", this.FWDLimit.isPressed());
        SmartDashboard.putBoolean("Pivot REV Limit", this.REVLimit.isPressed());
        // SmartDashboard.putNumber("pivot voltage", this.pivotMotor.getBusVoltage() * this.pivotMotor.getAppliedOutput());
        SmartDashboard.putBoolean("ProbablyHasCoral", probablyHasCoral);
    }

    public Command toggleTeleop() {
        return this.runOnce(() -> {
            enableTeleop = !enableTeleop;
        });
    }

    public Command movePivotUp() {
        return this.runOnce(() -> {
            if (enableTeleop) {
                setpoint += .02;
                setpoint = Math.min(setpoint, forwardSoftLimit);
                this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
            }
        });
    }

    public Command movePivotDown() {
        return this.runOnce(() -> {
            if (enableTeleop) {
                setpoint -= .02;
                setpoint = Math.max(setpoint, revSoftLimit);
                this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
            }
        });
    }
}
