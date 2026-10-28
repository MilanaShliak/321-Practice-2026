package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakePivotConstants {
    public static final Angle kFeedAngle = Degrees.of(15);
    public static final double kFedAngle = kFeedAngle.in(Degrees);
    public static final double kFeeedAngle = kFeedAngle.in(Radian);

    public static final Voltage kMaxVoltage = Volts.of(12);
    public static final Current kSupplyCurrentLimit = Amps.of(40);
    public static final Angle kStartingAngle = Degrees.of(0);
    public static final boolean kSupplyLimitEnabled = true;
    public static final Current kStatorLimit = Amps.of(40);
    public static final boolean kStatorLimitEnabled = true;

    public static final int kIntakePivotMotorID = 12;

    public static final double kIntakePivotRatio = 142.9;

    public static final InvertedValue kInversion = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(0);
    public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(0);

    public static final Angle kTargetAngle = Degrees.of(30);
    public static final Angle kTolerance = Degrees.of(1.5);

    
}
