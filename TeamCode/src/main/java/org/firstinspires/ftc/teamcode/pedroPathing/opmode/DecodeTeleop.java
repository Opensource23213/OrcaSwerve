package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="DecodeTeleop", group="ABC Opmode")
public class DecodeTeleop extends DecodeLibrary {
    public boolean start = false;
    double times = 0;
    boolean back = false;
    public static double amps = 2000;
    public static double loop_time = 100;


    @Override
    public void init(){
        teleop = true;
        initialize();
        if(color == 0) {
            y_mod += 6;
        }
        follower.setPose(auto_pose);
        follower.startTeleopDrive();
    }

    @Override
    public void loop(){
        teleop_loop();
    }
}
