package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem {
    
    CANSparkMax flippy1 = new CANSparkMax(21, MotorType.kBrushless);
    CANSparkMax flippy2 = new CANSparkMax(22, MotorType.kBrushless);
    CANSparkMax tele = new CANSparkMax(23, MotorType.kBrushless);

}
