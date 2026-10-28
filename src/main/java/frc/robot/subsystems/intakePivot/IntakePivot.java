package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.hood.HoodConstants;

public class IntakePivot {
        public IntakePivot(){
        configureMotors();
    }

     TalonFX intakePivotMotor = new TalonFX(IntakePivotConstants.kIntakePivotMotorID);

    //Current Limits
    private CurrentLimitsConfigs currentLimits=
      new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(IntakePivotConstants.kSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(IntakePivotConstants.kSupplyLimitEnabled)
        .withStatorCurrentLimit(IntakePivotConstants.kStatorLimit)
        .withSupplyCurrentLimitEnable(IntakePivotConstants.kStatorLimitEnabled);

    //Inversion and Neutral Mode
    private MotorOutputConfigs motorOutputConfiguration = 
      new MotorOutputConfigs()
        .withInverted(IntakePivotConstants.kInversion)
        .withNeutralMode(IntakePivotConstants.kNeutralMode);

    //Gear Ratio
    private FeedbackConfigs feedbackConfigs =
      new FeedbackConfigs()
      .withSensorToMechanismRatio(IntakePivotConstants.kIntakePivotRatio);

    //Configure the Robot
    private TalonFXConfiguration intakePivotMotorConfiguration =
      new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withMotorOutput(motorOutputConfiguration)
        .withFeedback(feedbackConfigs);
    
    public void configureMotors(){
        intakePivotMotor.getConfigurator().apply(intakePivotMotorConfiguration);
    }

    private double kP = 0;
    private double KI = 0;
    private double kD = 0;

    TrapezoidProfile.Constraints intakePivotConstraints =
      new TrapezoidProfile.Constraints(
        IntakePivotConstants.kMaxVelocity.in(DegreesPerSecond),
        IntakePivotConstants.kMaxAcceleration.in(DegreesPerSecondPerSecond));
    
    private ProfiledPIDController intakePivotController =
      new ProfiledPIDController(kP, KI, kD, intakePivotConstraints);

    private double kS = 0;
    private double kG = 0;
    private double kV = 0;

    ArmFeedforward intakePivotFeedforward =
      new ArmFeedforward(kS, kG, kV);

    public void goToAngle(Angle targetAngle){
      Voltage calculatedPIDVoltage = Volts.of(intakePivotController.calculate(getAngle().in(Degrees), targetAngle.in(Degrees)));
      Voltage calculatedFFVoltage = Volts.of(intakePivotFeedforward.calculate(targetAngle.in(Degrees), 0));
      Voltage desiredVoltage = Volts.of(calculatedPIDVoltage.in(Volts) + calculatedFFVoltage.in(Volts));
      intakePivotMotor.setVoltage(desiredVoltage.in(Volts));
    }


    @Logged(name = "intakePivotPitch")
    public Angle getAngle(){
      Angle angle = intakePivotMotor.getPosition().getValue();
      return angle;
    }

    @Logged(name = "atTargetAngle")
    public boolean atTargetAngle(){
      return Math.abs(IntakePivotConstants.kTargetAngle.in(Degrees) - getAngle().in(Degrees)) <= IntakePivotConstants.kTolerance.in(Degrees); 
    }

  




}
