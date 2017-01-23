package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.params.ColorSpaceTransform;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Iedi on 18.01.2017.
 */
@Autonomous(name = "muita", group = "muita")
public class TankBotTest extends LinearOpMode {
    ColorSensor colorStanga = null;
    ColorSensor colorDreapta = null;

    DigitalChannel lineStanga = null;
    DigitalChannel lineDreapta = null;

    ModernRoboticsI2cRangeSensor distanceSensor = null;

    ModernRoboticsI2cCompassSensor compass = null;
    ModernRoboticsI2cGyro gyro = null;

    @Override
    public void runOpMode() throws InterruptedException {
        colorStanga = this.hardwareMap.colorSensor.get("colorStanga");
        colorDreapta = this.hardwareMap.colorSensor.get("colorDreapta");

        lineStanga = this.hardwareMap.digitalChannel.get("lineStanga");
        lineDreapta = this.hardwareMap.digitalChannel.get("lineDreapta");

        distanceSensor = this.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceSensor");


        compass = this.hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");
        gyro = this.hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        colorStanga.enableLed(false);
        colorDreapta.enableLed(false);



        gyro.calibrate();
        while(gyro.isCalibrating());

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("colorStanga blue", colorStanga.blue());
            telemetry.addData("colorStanga red", colorStanga.red());

            telemetry.addData("colorDreapta blue", colorDreapta.blue());
            telemetry.addData("colorDreapta red", colorDreapta.red());

            telemetry.addData("lineDreapta", lineDreapta.getState());
            telemetry.addData("lineStanga", lineStanga.getState());

            telemetry.addData("distanceSensor", distanceSensor.getDistance(DistanceUnit.CM));

            telemetry.addData("compass", compass.getDirection());
            telemetry.addData("gyro", gyro.getHeading());

            telemetry.update();
        }
    }
}
