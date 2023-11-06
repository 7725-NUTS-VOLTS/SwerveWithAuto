package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.*;

public class RollerSubsystem extends SubsystemBase {
    //Controlling Devices:
    private CANSparkMax rollerMotor;
    private RelativeEncoder rollerEncoder;
    private ColorSensorV3 color = new ColorSensorV3(Port.kMXP);
    private double rollerSpeed; 
    public RollerSubsystem() {
        rollerMotor = new CANSparkMax(Constants.RollerConstants.rollerMotorId, CANSparkMax.MotorType.kBrushless); // Replace with actual device ID
        //boomstickEncoder = boomstickMotor.getAlternateEncoder(Type.kQuadrature, 4096);
        rollerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rollerEncoder = rollerMotor.getEncoder();
        rollerMotor.setSmartCurrentLimit(30);
        // Configure the integrated PID controller
      
        // Set the output range of the PID controller
    }

    public double getColorX(){
        return color.getCIEColor().getX();
    }
    public double getColorY(){
        return color.getCIEColor().getY();
    }
    public double getColorZ(){
        return color.getCIEColor().getZ();
    }
    
    
    public void setSpeed(double speed1, double speed2) {
        rollerSpeed = speed1 - speed2;
    }
    public void resetEncoders() {
        rollerEncoder.setPosition(0);
    }
    public void move(){
        rollerMotor.set(rollerSpeed); 
    }
    
}