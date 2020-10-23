/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int MOTOR_LEFT_1_ID = 0;
    public static final int MOTOR_RIGHT_1_ID = 1;
    public static final int MOTOR_LEFT_2_ID = 2;
    public static final int MOTOR_RIGHT_2_ID = 3;

    public static final int DRIVER_CONTROLLER = 0;

    public static final int LEFT_Y_AXIS = 1;
    public static final int RIGHT_Y_AXIS = 5;
    public static final int BUTTON_X = 3;
    
    public static final double MOTOR_SPEED_SCALING_FACTOR = 0.5;
    //int or double value from 0-1   

    public static final double ksVolts = 1.11; 

    public static final double kvVoltSecondsPerMeter = 3.0; 

    public static final double kaVoltSecondsSquaredPerMeter = 0.368; 

    public static final double kPDriveVel = 13.3; 

    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    // DIFFERENTIAL DRIVE KINEMATICS
    public static final double kTrackwidth = 0.55245; // in meters

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
    LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
          kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 8;
    public static final double kWheelDiameterMeters = 0.1016;  // meters = 4 inches

    public static final int[] LEFT_ENCODER_PORTS = new int[]{6, 7}; //TODO: THESE ARE STAND INS

    public static final int[] RIGHT_ENCODER_PORTS = new int[]{8, 9}; //TODO: THESE ARE STAND INS
}
