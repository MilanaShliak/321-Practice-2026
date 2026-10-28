package frc.robot.subsystems.hood;

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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class Hood {

    public Hood(){
        configureMotors();
    }

     TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodMotorID);

    //Current Limits
    private CurrentLimitsConfigs currentLimits=
      new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(HoodConstants.kSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(HoodConstants.kSupplyLimitEnabled)
        .withStatorCurrentLimit(HoodConstants.kStatorLimit)
        .withSupplyCurrentLimitEnable(HoodConstants.kStatorLimitEnabled);

    //Inversion and Neutral Mode
    private MotorOutputConfigs motorOutputConfiguration = 
      new MotorOutputConfigs()
        .withInverted(HoodConstants.kInversion)
        .withNeutralMode(HoodConstants.kNeutralMode);

    //Gear Ratio
    private FeedbackConfigs feedbackConfigs =
      new FeedbackConfigs()
      .withSensorToMechanismRatio(HoodConstants.kHoodRatio);

    //Configure the Robot
    private TalonFXConfiguration hoodMotorConfiguration =
      new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withMotorOutput(motorOutputConfiguration)
        .withFeedback(feedbackConfigs);
    
    public void configureMotors(){
        hoodMotor.getConfigurator().apply(hoodMotorConfiguration);
    }
    
    /* 
      Starting the PID tuning
    */
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    TrapezoidProfile.Constraints hoodConstraints =
      new TrapezoidProfile.Constraints(
        HoodConstants.kMaxVelocity.in(DegreesPerSecond),
        HoodConstants.kMaxAcceleration.in(DegreesPerSecondPerSecond));

    private ProfiledPIDController hoodController = 
      new ProfiledPIDController(kP, kI, kD, hoodConstraints);

    /*
      Starting FeedForward
    */
  
    private double kS = 0;
    private double kG = 0;
    private double kV = 0;

    ArmFeedforward hoodFeedForward =
      new ArmFeedforward(kS, kG, kV); 

    //Combining PID and FeedForward
    public void goToAngle(Angle targetAngle){
      Voltage calculatedPIDVoltage = Volts.of(hoodController.calculate(getAngle().in(Degrees), targetAngle.in(Degrees)));
      Voltage calculatedFFVoltage = Volts.of(hoodFeedForward.calculate(targetAngle.in(Degrees), 0));
      Voltage desiredVoltage = Volts.of(calculatedPIDVoltage.in(Volts) + calculatedFFVoltage.in(Volts));
      hoodMotor.setVoltage(desiredVoltage.in(Volts));

    }


    /*
     All of the logging
     */
    @Logged(name = "hoodPitch")
    public Angle getAngle(){
      Angle angle = hoodMotor.getPosition().getValue();
      return angle;
    }

    @Logged(name = "atTargetAngle")
    public boolean atTargetAngle(){
      return Math.abs(HoodConstants.kTargetAngle.in(Degrees) - getAngle().in(Degrees)) <= HoodConstants.kTolerance.in(Degrees);
    }

    @Logged(name = "hoodTargetAngle")
    public Angle getTargetAngle(){
      return HoodConstants.kTargetAngle;
    }

    @Logged(name = "getVoltage")
    public Voltage getVoltage(){
      return hoodMotor.getMotorVoltage().getValue();
    }

    @Logged(name = "getCurrent")
    public Current getCurrent(){
        return hoodMotor.getStatorCurrent().getValue();
    }



}
