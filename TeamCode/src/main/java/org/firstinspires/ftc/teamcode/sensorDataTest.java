package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Sensor Data Test", group="Manual")
@Disabled
public class sensorDataTest extends drivetraintest {
    //logic objects
    double power;
    OpticalDistanceSensor ODSright, ODSleft;
    private ColorSensor color;
    @Override
    public void init(){
        super.init();
        color = hardwareMap.colorSensor.get("color");
        ODSright = hardwareMap.opticalDistanceSensor.get("odsright");
        ODSleft = hardwareMap.opticalDistanceSensor.get("odsright");

    }

    @Override
    public void loop(){
        super.loop();
        //send data back to driver station
        telemetry.addData("ODS right",ODSright.getRawLightDetected());
        telemetry.addData("ODS left",ODSleft.getRawLightDetected());
        telemetry.addData("Color sensor Red",color.red());
        telemetry.addData("Color sensor Blue",color.blue());

    }
}