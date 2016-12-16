package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Manual", group="Manual")
public class sensorDataTest extends manual {
    //logic objects
    double power;

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