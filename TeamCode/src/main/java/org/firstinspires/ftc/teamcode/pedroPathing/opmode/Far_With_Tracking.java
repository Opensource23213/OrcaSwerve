package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.tan;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name="Tracking Far", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Far_With_Tracking extends DecodeLibrary{
    public double forward = 0.25;
    public ArrayList<Pose2D> first_pick;
    public ArrayList<Pose2D> pick_after_stuff;
    public ArrayList<Pose2D> first_shoot;
    public ArrayList<Pose2D> second_shoot;
    public ArrayList<Pose2D> third_shoot;
    public ArrayList<Pose2D> fourth_shoot;
    public ArrayList<Pose2D> fifth_shoot;
    public ArrayList<Pose2D> second_pick;
    public ArrayList<Pose2D> third_pick;
    public ArrayList<Pose2D> park;
    public ArrayList<Pose2D> gate;
    public double shot = 0;
    public double old_turns = 0;
    public double steps = 0;
    public ElapsedTime gate_open = new ElapsedTime();
    public ElapsedTime shooting_time = new ElapsedTime();
    public double sort_pos = 0;
    public double old_pattern = 2;
    public double old_color = 1;
    public boolean shooting = false;
    public double index_steps = 0;
    public static double first_dump_time = 2500;
    public static double second_dump_time = 2500;
    public double count = 0;
    public ElapsedTime runtime = new ElapsedTime();
    public boolean stop_everything = false;
    public double last_intake = 0;
    @Override
    public void init(){
        color = 1;
        teleop = false;
        if(color == 1){
            auto_pose = new Pose2D(DistanceUnit.INCH, 0,.375, AngleUnit.DEGREES, -90);
        }else{
            auto_pose = new Pose2D(DistanceUnit.INCH, 0,-.375, AngleUnit.DEGREES, 90);
        }
        initialize();
        drive.pinpoint.setPosition(auto_pose);
        if(color == 0) {
            blue_init();
        }else{
            red_init();
        }
        intake_system.flippy_pos = flippy_hold;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        drive.pinpoint.update();

        turret.zero = true;
        turret.turret_move();
        if(gamepad1.right_bumper) {
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose = new Pose2D(DistanceUnit.INCH, 0,.375, AngleUnit.DEGREES, -90);
            }else{
                auto_pose = new Pose2D(DistanceUnit.INCH, 0,-.375, AngleUnit.DEGREES, 90);
            }
            if (color == 0) {
                blue_init();
                old_color = 0;
            } else {
                red_init();
                old_color = 1;
            }

        }
        drive.auto_drive = true;
        if(drive.pinpoint.getPosition() != auto_pose){
            drive.pinpoint.setPosition(auto_pose);
        }
        runtime.reset();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        drive.runDrive();
        turret.turret_move();
        shooter.shooting();
        intake_system.auto();
        auto_pose = drive.position;
        if(forward >= .5) {
            if(runtime.milliseconds() > 28000 && !stop_everything){
                drive.currentPath = new ArrayList<>(park);
                stop_everything = true;
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                shooting = false;
                steps = 2;
            }else if(!stop_everything) {
                if (drive.currentPath == null) {
                    if (forward == 1) {
                        shoot();
                        shooting = true;
                        if (steps == 2) {
                            shooting = false;
                            drive.currentPath = new ArrayList<>(second_pick);
                            forward = 3;
                            steps = 0;
                        }
                    }else if (forward == 3) {
                        shoot();
                        shooting = true;
                        if (steps == 2) {
                            shooting = false;
                            pick_balls();
                        }
                    }
                }
            }
        }else{
            shoot();
            shooting = true;
            if(steps == 2){
                shooting = false;
                drive.currentPath = new ArrayList<>(first_pick);
                forward = 1;
                steps = 0;
            }
        }
    }
    public ArrayList<Pose2D> gate2;
    public ArrayList<Pose2D> gate3;

    public void red_init(){
        first_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 0, -44,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 2, -8,AngleUnit.DEGREES, 180)));
        second_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 21, 0,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 21, -46,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 2, -8,AngleUnit.DEGREES, 180)));
        pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 0, -44,AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 4, -8,AngleUnit.DEGREES, 180)));
        park = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 20, -20,AngleUnit.DEGREES, 180)));
    }

    public void blue_init(){
        first_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 0, 45,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 2, 8,AngleUnit.DEGREES, 0)));
        second_pick = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 24, 0,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 25, 46,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 2, 8,AngleUnit.DEGREES, 0)));
        pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 0, 45,AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 4, 8,AngleUnit.DEGREES, 0)));
        park = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 20, 20,AngleUnit.DEGREES, 0)));
    }
    public void shoot(){
        if(shooting) {
            if(intake_system.balls.isEmpty() && steps == 0 && forward >= .5){
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                last_intake = 0;
                steps = 2;
            }else {
                if (steps == 0 && (abs(shooter.shoot1.getVelocity() - shooter.set_speed) <= 20 || forward >= .5)) {
                    shooting_time.reset();
                    steps = .5;
                } else if (steps == .5) {
                    last_intake = intake_system.balls.size();
                    shooting_time.reset();
                    intake_system.spindexer.setPower(-.6);
                    intake_system.spindex_shooting = true;
                    intake_system.flippy_pos = flippy_up;
                    intake_system.blocking = false;
                    steps = 1;
                } else if (steps == 1 && (shooting_time.milliseconds() > 800)) {
                    intake_system.flippy_pos = flippy_down;
                    intake_system.spindexer.setPower(1);
                    intake_system.spindex_shooting = false;
                    intake_system.blocking = true;
                    steps = 2;
                }
            }
        }
    }
    public double x = 0;
    public double balls = 0;
    public void pick_balls(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        double[] python = cameraCode.limelight.getLatestResult().getPythonOutput();
        if(python[1] != 0 && cameraCode.limelight.isConnected()) {
            double angle = 0;
            double x_value = python[0];
            double y_value = 46 - drive.position.getY(DistanceUnit.INCH);
            if(color == 1){
                angle = 180;
                x_value *= -1;
                y_value = 46 + drive.position.getY(DistanceUnit.INCH);
            }
            x_value += 6.5;
            y_value *= tan(Math.toRadians(abs(drive.position.getHeading(AngleUnit.DEGREES)) - 90));
            y_value *= -1;
            x_value = y_value + x_value;
            if(x_value < -drive.position.getX(DistanceUnit.INCH)){
                x_value = -drive.position.getX(DistanceUnit.INCH);
            }
            x_value += drive.position.getX(DistanceUnit.INCH);
            if(x_value > 3){
                if(color == 1) {
                    pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, -20, AngleUnit.DEGREES, angle),new Pose2D(DistanceUnit.INCH, x_value, -47, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, -8, AngleUnit.DEGREES, angle)));
                }else{
                    pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, 20, AngleUnit.DEGREES, angle),new Pose2D(DistanceUnit.INCH, x_value, 47, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, 8, AngleUnit.DEGREES, angle)));
                }
            }else{
                if(color == 1) {
                    pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, -44, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, -8, AngleUnit.DEGREES, angle)));
                }else{
                    pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, 44, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, 8, AngleUnit.DEGREES, angle)));
                }
            }
            if(pick_after_stuff.get(pick_after_stuff.size() - 2).getX(DistanceUnit.INCH) > 26 && python[2] != 0){
                angle = 0;
                x_value = python[2];
                y_value = 46 - drive.position.getY(DistanceUnit.INCH);
                if(color == 1){
                    angle = 180;
                    x_value *= -1;
                    y_value = 46 + drive.position.getY(DistanceUnit.INCH);
                }
                x_value += 6.5;
                y_value *= tan(Math.toRadians(abs(drive.position.getHeading(AngleUnit.DEGREES)) - 90));
                y_value *= -1;
                x_value = y_value + x_value;
                if(x_value < -drive.position.getX(DistanceUnit.INCH)){
                    x_value = -drive.position.getX(DistanceUnit.INCH);
                }
                x_value += drive.position.getX(DistanceUnit.INCH);
                if(x_value > 3){
                    if(color == 1) {
                        pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, -20, AngleUnit.DEGREES, angle),new Pose2D(DistanceUnit.INCH, x_value, -47, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, -8, AngleUnit.DEGREES, angle)));
                    }else{
                        pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, 20, AngleUnit.DEGREES, angle),new Pose2D(DistanceUnit.INCH, x_value, 47, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, 8, AngleUnit.DEGREES, angle)));
                    }
                }else{
                    if(color == 1) {
                        pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, -44, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, -8, AngleUnit.DEGREES, angle)));
                    }else{
                        pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, x_value, 44, AngleUnit.DEGREES, angle), new Pose2D(DistanceUnit.INCH, 4, 8, AngleUnit.DEGREES, angle)));
                    }
                }
                drive.currentPath = new ArrayList<>(pick_after_stuff);
                count += 1;
                forward = 3;
                steps = 0;
                x = x_value;
                balls = python[1];
            }else if(pick_after_stuff.get(pick_after_stuff.size() - 2).getX(DistanceUnit.INCH) <= 26) {
                drive.currentPath = new ArrayList<>(pick_after_stuff);
                count += 1;
                forward = 3;
                steps = 0;
                x = x_value;
                balls = python[1];
            }
        }else if(!cameraCode.limelight.isConnected()){
            if(color == 1) {
                pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 1, -47, AngleUnit.DEGREES, 180), new Pose2D(DistanceUnit.INCH, 4, -8, AngleUnit.DEGREES, 180)));
            }else{
                pick_after_stuff = new ArrayList<>(Arrays.asList(new Pose2D(DistanceUnit.INCH, 1, 47, AngleUnit.DEGREES, 0), new Pose2D(DistanceUnit.INCH, 4, 8, AngleUnit.DEGREES, 0)));
            }
        }
    }




}
