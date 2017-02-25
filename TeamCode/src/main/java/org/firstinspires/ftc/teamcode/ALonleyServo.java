package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SERVOTEST", group="Manual")
@Disabled
public class ALonleyServo extends OpMode {
    Servo servoLinear;
    public void init(){
        servoLinear = hardwareMap.servo.get("servo");
    }
    public void loop(){
        if (gamepad1.dpad_up)
            servoLinear.setPosition(.5);
        if (gamepad1.dpad_left)
            servoLinear.setPosition(1);
    }
}
