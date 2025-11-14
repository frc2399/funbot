package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private SparkMax rightFront = new SparkMax (3, SparkLowLevel.MotorType.kBrushless);
    private SparkMax leftFront = new SparkMax (1, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightBack = new SparkMax (4, SparkLowLevel.MotorType.kBrushless);
    private SparkMax leftBack = new SparkMax (2, SparkLowLevel.MotorType.kBrushless);
    
    private final SparkClosedLoopController motorRightClosedLoopController;
    private final SparkClosedLoopController motorLeftClosedLoopController;

    private final Distance ENCODER_POSITION_FACTOR = Inches.of(8 * Math.PI * (10./52.) * (30./68.));
    private final Distance ENCODER_VELOCITY_FACTOR = Inches.of((8 * Math.PI * (10./52.) * (30./68.)) / 60.0); //inches per rotation

    private final double drivetrainP = 0.08;

    

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
        leftFrontConfig.inverted(true);
        leftBackConfig.inverted(true);

        rightBackConfig.follow(rightFront,false);
        leftBackConfig.follow(leftFront,false);

        rightFrontConfig.smartCurrentLimit(50);
        leftFrontConfig.smartCurrentLimit(50);
        rightBackConfig.smartCurrentLimit(50);
        leftBackConfig.smartCurrentLimit(50);


        rightFrontConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Meters));
        leftFrontConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Meters));
        rightBackConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Meters));
        leftBackConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Meters));

        rightFrontConfig.encoder.velocityConversionFactor(ENCODER_VELOCITY_FACTOR.in(Meters));
        leftFrontConfig.encoder.velocityConversionFactor(ENCODER_VELOCITY_FACTOR.in(Meters));
        rightBackConfig.encoder.velocityConversionFactor(ENCODER_VELOCITY_FACTOR.in(Meters));
        leftBackConfig.encoder.velocityConversionFactor(ENCODER_VELOCITY_FACTOR.in(Meters));

        rightFrontConfig.closedLoop.p(drivetrainP);
        leftFrontConfig.closedLoop.p(drivetrainP);
        rightBackConfig.closedLoop.p(drivetrainP);
        leftBackConfig.closedLoop.p(drivetrainP);



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

    motorRightClosedLoopController = rightFront.getClosedLoopController();
    motorLeftClosedLoopController = leftFront.getClosedLoopController();

    }

    public void setRightSpeed(LinearVelocity speed) {
        motorRightClosedLoopController.setReference(speed.in(MetersPerSecond), ControlType.kVelocity);
    }

    public void setLeftSpeed(LinearVelocity speed) {
        motorLeftClosedLoopController.setReference(speed.in(MetersPerSecond), ControlType.kVelocity);
    }
    
}


