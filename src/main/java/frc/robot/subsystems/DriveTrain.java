/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.tankdrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

// simulation classes
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.Field2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.hal.SimDouble;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new drivetrain.
   */
  Encoder leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
  Encoder rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);
  private final TalonSRX motorLeft1 = new TalonSRX(Constants.MOTOR_LEFT_1_ID);
  private final TalonSRX motorLeft2 = new TalonSRX(Constants.MOTOR_LEFT_2_ID);
  private final TalonSRX motorRight1 = new TalonSRX(Constants.MOTOR_RIGHT_1_ID);
  private final TalonSRX motorRight2 = new TalonSRX(Constants.MOTOR_RIGHT_2_ID);

  DifferentialDriveOdometry odometry;

  // Simulation classes
  public DifferentialDrivetrainSim drivetrainSimulator;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  // The Field2d class simulates the field in the sim GUI. Note that we can have only one
  // instance!
  // Does this belong somewhere else??
  private Field2d fieldSim;
  private SimDouble gyroAngleSim;
  private Gyro gyro = new ADXRS450_Gyro();

  public DriveTrain() {
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    if (RobotBase.isSimulation()) {
      // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      drivetrainSimulator = new DifferentialDrivetrainSim(
            Constants.kDrivetrainPlant,
            Constants.kDriveGearbox,
            Constants.kDriveGearing,
            Constants.kTrackwidth,
            Constants.kWheelDiameterMeters / 2.0);

      // The encoder and gyro angle sims let us set simulated sensor readings
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);

      // string is the name. Does the value actually matter??
      gyroAngleSim = new SimDeviceSim("ADXRS450_Gyro" + "[" + SPI.Port.kOnboardCS0.value + "]").getDouble("Angle");

      // the Field2d class lets us visualize our robot in the simulation GUI.
      fieldSim = new Field2d();
    }
  }

  public double getHeading() {   
    return Math.IEEEremainder(gyro.getAngle(), 360) * -1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
  
  }

  public void setLeftMotors(double speed){
    motorLeft1.set(ControlMode.PercentOutput, speed * -Constants.MOTOR_SPEED_SCALING_FACTOR);
    //motorLeft2.set(ControlMode.PercentOutput, speed * -Constants.MOTOR_SPEED_SCALING_FACTOR);
  }

  public void setRightMotors(double speed){
    motorRight1.set(ControlMode.PercentOutput, speed * Constants.MOTOR_SPEED_SCALING_FACTOR);
    //motorRight2.set(ControlMode.PercentOutput, speed * Constants.MOTOR_SPEED_SCALING_FACTOR);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    drivetrainSimulator.setInputs(motorLeft1.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
                                  -motorRight1.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    drivetrainSimulator.update(0.020);
  
    leftEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftPosition));
    leftEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftVelocity));
    
    rightEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightPosition));
    rightEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightVelocity));

    gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());

    fieldSim.setRobotPose(getPose());
  }
}
