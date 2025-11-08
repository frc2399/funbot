package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.InchesPerSecond;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private SparkMax rightFront = new SparkMax (3, SparkLowLevel.MotorType.kBrushless);
    private SparkMax leftFront = new SparkMax (1, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightBack = new SparkMax (4, SparkLowLevel.MotorType.kBrushless);
    private SparkMax leftBack = new SparkMax (2, SparkLowLevel.MotorType.kBrushless);
    
    private final SparkClosedLoopController motorClosedLoopController;
    private final SparkClosedLoopController motorClosedLoopController2;
    

    public Drivetrain() {
        SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
        SparkMaxConfig leftFrontConfig = new SparkMaxConfig();
        SparkMaxConfig rightBackConfig = new SparkMaxConfig();
        SparkMaxConfig leftBackConfig = new SparkMaxConfig();

        rightFrontConfig.idleMode(IdleMode.kBrake);
        leftFrontConfig.idleMode(IdleMode.kBrake);
        rightBackConfig.idleMode(IdleMode.kBrake);
        leftBackConfig.idleMode(IdleMode.kBrake);

        rightFrontConfig.inverted(false);
        rightBackConfig.inverted(false);
        leftFrontConfig.inverted(false);
        leftBackConfig.inverted(false);

        rightBackConfig.follow(rightFront,false);
        leftBackConfig.follow(leftFront,false);

        rightFrontConfig.smartCurrentLimit(50);
        leftFrontConfig.smartCurrentLimit(50);
        rightBackConfig.smartCurrentLimit(50);
        leftBackConfig.smartCurrentLimit(50);

        rightFront.configure(
            rightFrontConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        leftFront.configure(
            leftFrontConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        rightBack.configure(
            rightBackConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        leftBack.configure(
            leftBackConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

    motorClosedLoopController = rightFront.getClosedLoopController();
    motorClosedLoopController2 = leftFront.getClosedLoopController();

    }

    public void setRightSpeed(LinearVelocity speed) {
        motorClosedLoopController.setReference(speed.in(InchesPerSecond), ControlType.kVelocity);
    }

    public void setLeftSpeed(LinearVelocity speed) {
        motorClosedLoopController2.setReference(speed.in(InchesPerSecond), ControlType.kVelocity);
    }
    
}


