package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.MotorIdConstants.MotorConstants;

public class Drivetrain extends SubsystemBase {
    private SparkMax rightFrontMotorController = new SparkMax(MotorIdConstants.RIGHT_FRONT_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax leftFrontMotorController = new SparkMax(MotorIdConstants.LEFT_FRONT_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax rightBackMotorController = new SparkMax(MotorIdConstants.RIGHT_BACK_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkMax leftBackMotorController = new SparkMax(MotorIdConstants.LEFT_BACK_ID, SparkLowLevel.MotorType.kBrushless);
    
    private final SparkClosedLoopController motorRightClosedLoopController;
    private final SparkClosedLoopController motorLeftClosedLoopController;
 
    /* 8 inches for diameter of wheel
    multiplied by pi to get circumference
    10/52 is the gear ratio for the motor gear and gear it's connected to
    30/68 is the gear ratio for the last two gears */
    private final Distance ENCODER_POSITION_FACTOR = Inches.of(8 * Math.PI * (10./52.) * (30./68.));
   
    //distance divided by 60 to get velocity in inches per rotations
    private final Distance ENCODER_VELOCITY_FACTOR = Inches.of(ENCODER_POSITION_FACTOR.in(Inches) / 60.0);

    private final double drivetrainP = 0.08;

    private final double MAX_SPEED_METERS_PER_SECOND = 4.0;

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

        rightBackConfig.follow(rightFrontMotorController,false);
        leftBackConfig.follow(leftFrontMotorController,false);

        rightFrontConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
        leftFrontConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
        rightBackConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
        leftBackConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));


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



        rightFrontMotorController.configure(
            rightFrontConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        leftFrontMotorController.configure(
            leftFrontConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        rightBackMotorController.configure(
            rightBackConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

        leftBackMotorController.configure(
            leftBackConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters);

    motorRightClosedLoopController = rightFrontMotorController.getClosedLoopController();
    motorLeftClosedLoopController = leftFrontMotorController.getClosedLoopController();

    }

    public void setRightSpeed(LinearVelocity speed) {
        motorRightClosedLoopController.setReference(speed.in(MetersPerSecond), ControlType.kVelocity);
    }

    public void setLeftSpeed(LinearVelocity speed) {
        motorLeftClosedLoopController.setReference(speed.in(MetersPerSecond), ControlType.kVelocity);
    }
    public Command tankDrive(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
        return this.run(()->{
            setLeftSpeed((MetersPerSecond.of(leftSpeed.getAsDouble() * MAX_SPEED_METERS_PER_SECOND)));
            setRightSpeed((MetersPerSecond.of(rightSpeed.getAsDouble() * MAX_SPEED_METERS_PER_SECOND)));
        } );
    }
}


