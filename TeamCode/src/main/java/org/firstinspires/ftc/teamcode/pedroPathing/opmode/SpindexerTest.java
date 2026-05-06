package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Spindexer Test", group="ABC Opmode")

public class SpindexerTest extends DecodeTeleop {
    public static double test_pos = .5;
    public Servo Servo;
    public AnalogInput spin_dis;
    @Override
    public void init(){
        intake_system.init();
        shooter.initialize();
        spin_dis = hardwareMap.get(AnalogInput.class, "spin_dis");
    }
    @Override
    public void loop(){
        intake_system.intake_test();
        shooter.shoot2.setVelocity(1250);
        shooter.shoot1.setVelocity(1250);
        telemetry.addData("dis", spin_dis.getVoltage());
        telemetry.addData("Colors", intake_system.color_middle.getColors());
        telemetry.update();
    }

}
