package org.firstinspires.ftc.teamcode.pedroPathing.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.BlueFarConst;
import org.firstinspires.ftc.teamcode.pedroPathing.RedFarConst;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
@Autonomous(name="Stay_Far", group="ABC Opmode", preselectTeleOp = "DecodeTeleop")
public class Stay_Far extends DecodeLibrary{
    public double forward = 0.25;
    public Path first_pick;
    public Path pick_after_stuff;
    public Path first_shoot;
    public Path second_shoot;
    public Path third_shoot;
    public Path fourth_shoot;
    public Path fifth_shoot;
    public Path second_pick;
    public Path third_pick;
    public Path park;
    public Path gate;
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
            auto_pose = new Pose(0,.375, Math.toRadians(-90));
        }else{
            auto_pose = new Pose(0,-.375, Math.toRadians(90));
        }
        initialize();
        follower.setPose(auto_pose);
        if(color == 0) {
            blue_init();
            switch_constants(BlueFarConst.followerConstants);
        }else{
            red_init();
            switch_constants(RedFarConst.followerConstants);
        }
        follower.setMaxPower(1);
        follower.setMaxPowerScaling(1);
        intake_system.flippy_pos = flippy_hold;
    }
    @Override
    public void init_loop(){
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        if(color == 1){
            y_mod = -15;
        }else{
            y_mod = -9;
        }

        turret.zero = true;
        cam.look();
        if(cam.tags.contains(21.0)){
            pattern = 1;
        }else if(cam.tags.contains(22.0)){
            pattern = 2;
        }else if(cam.tags.contains(23.0)){
            pattern = 3;
        }
        turret.turret_move();
        follower.drivetrain.setYVelocity(0);
        follower.update();
        if(gamepad1.right_bumper) {
            color = 0;
        }else if(gamepad1.left_bumper){
            color = 1;
        }
        if(old_color != color || gamepad1.a) {
            initialize();
            if(color == 1){
                auto_pose = new Pose(0,.375, Math.toRadians(-90));
            }else{
                auto_pose = new Pose(0,-.375, Math.toRadians(90));
            }
            follower.setMaxPower(1);
            follower.setMaxPowerScaling(1);
            follower.setPose(auto_pose);
            if (color == 0) {
                blue_init();
                switch_constants(BlueFarConst.followerConstants);
                old_color = 0;
            } else {
                red_init();
                switch_constants(RedFarConst.followerConstants);
                old_color = 1;
            }

        }
        auto_pose = follower.getPose();
        runtime.reset();
        telemetry.addData("pattern", pattern);
        telemetry.addData("color", color);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs){
            hub.clearBulkCache();
        }
        follower.update();
        turret.turret_move();
        shooter.set_speed = shoot_multiplier * ((dead_distance * .0254) - 1.6) + shoot_power_offset + (speed_increase * abs(turret.target_angle - .5));
        shooter.set_speed = Math.round(shooter.set_speed / 20) * 20 + 40;
        shooter.shooting();
        intake_system.auto();
        auto_sort();
        intake_system.sort();
        auto_pose = follower.getPose();
        if(forward >= .5) {
            if(runtime.milliseconds() > 28000 && !stop_everything){
                follower.followPath(park);
                stop_everything = true;
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                shooting = false;
                steps = 2;
            }else if(!stop_everything) {
                if (follower.atParametricEnd() || !follower.isBusy() || follower.isRobotStuck()) {
                    if (forward == 1) {
                        follower.followPath(second_shoot);
                        forward = 2;
                    } else if (forward == 2) {
                        shoot();
                        shooting = true;
                        if (steps == 2) {
                            shooting = false;
                            follower.followPath(second_pick);
                            forward = 3;
                            steps = 0;
                        }
                    } else if (forward == 3) {
                        follower.followPath(third_shoot);
                        forward = 4;
                    } else if (forward == 4) {
                        shoot();
                        shooting = true;
                        if (steps == 2) {
                            shooting = false;
                            if (count > 0) {
                                if(last_intake <= 1 && pick_after_stuff.getLastControlPoint().getX() == 0){
                                    if (color == 0) {
                                        pick_after_stuff = new Path(new BezierCurve(new Pose(0, 0), new Pose(40, 24), new Pose(30, 44)));
                                        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(90));
                                        fifth_shoot = new Path(new BezierCurve(pick_after_stuff.getLastControlPoint(), new Pose(4, 8)));
                                        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
                                    } else {
                                        pick_after_stuff = new Path(new BezierCurve(pick_after_stuff.getLastControlPoint(),new Pose(20, -24), new Pose(22, -44)));
                                        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-90));
                                        fifth_shoot = new Path(new BezierCurve(pick_after_stuff.getLastControlPoint(), new Pose(4, -12)));
                                        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
                                    }
                                }else {
                                    if (color == 0) {
                                        pick_after_stuff = new Path(new BezierCurve(new Pose(0, 0), new Pose(-2, 44)));
                                        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(90));
                                        fifth_shoot = new Path(new BezierCurve(pick_after_stuff.getLastControlPoint(), new Pose(4, 8)));
                                        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
                                    } else {
                                        pick_after_stuff = new Path(new BezierCurve(new Pose(0, 0), new Pose(-2, -44)));
                                        pick_after_stuff.setConstantHeadingInterpolation(Math.toRadians(-90));
                                        fifth_shoot = new Path(new BezierCurve(pick_after_stuff.getLastControlPoint(), new Pose(4, -12)));
                                        fifth_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
                                    }
                                }
                            }
                            follower.followPath(pick_after_stuff);
                            count += 1;
                            forward = 5;
                            steps = 0;
                        }
                    } else if (forward == 5) {
                        follower.followPath(fifth_shoot);
                        if (count == 20) {
                            forward = 6;
                        } else {
                            forward = 4;
                        }
                    } else if (forward == 6) {
                        shoot();
                        shooting = true;
                        if (steps == 2) {
                            shooting = false;
                            follower.followPath(park);
                            forward = 7;
                            steps = 0;
                        }
                    }
                }
            }
        }else{
            shoot();
            shooting = true;
            if(steps == 2){
                shooting = false;
                follower.followPath(first_pick);
                forward = 1;
                steps = 0;
            }
        }
    }
    public Path gate2;
    public Path gate3;

    public void red_init(){
        first_pick = new Path(new BezierCurve(new Pose(0,0), new Pose(0, -44)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_shoot = new Path(new BezierCurve(first_pick.getLastControlPoint(),  new Pose(4, -12)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        second_pick = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(18, 0), new Pose(20, -46)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(-90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(4, -8)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(-90));
        pick_after_stuff = first_pick;
        fifth_shoot = second_shoot;
        park = new Path(new BezierLine(fifth_shoot.getLastControlPoint(), new Pose(20, -20)));
        park.setConstantHeadingInterpolation(Math.toRadians(-90));

    }
    public void blue_init(){
        first_pick = new Path(new BezierCurve(new Pose(0,0), new Pose(0, 44)));
        first_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        second_shoot = new Path(new BezierCurve(first_pick.getLastControlPoint(),  new Pose(4, 8)));
        second_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        second_pick = new Path(new BezierCurve(second_shoot.getLastControlPoint(), new Pose(27, 0), new Pose(29, 46)));
        second_pick.setConstantHeadingInterpolation(Math.toRadians(90));
        third_shoot = new Path(new BezierLine(second_pick.getLastControlPoint(), new Pose(4, 8)));
        third_shoot.setConstantHeadingInterpolation(Math.toRadians(90));
        pick_after_stuff = first_pick;
        fifth_shoot = second_shoot;
        park = new Path(new BezierLine(fifth_shoot.getLastControlPoint(), new Pose(20, 20)));
        park.setConstantHeadingInterpolation(Math.toRadians(90));
    }
    public void shoot(){
        if(shooting) {
            if(intake_system.balls.isEmpty() && steps == 0){
                intake_system.flippy_pos = flippy_down;
                intake_system.spindexer.setPower(1);
                intake_system.spindex_shooting = false;
                intake_system.blocking = true;
                last_intake = 0;
                steps = 2;
            }else {
                if (steps == 0 && abs(shooter.shoot1.getVelocity() - shooter.set_speed) <= 20 && intake_system.sort_time == 0 && !intake_system.sorting) {
                    shooting_time.reset();
                    steps = .5;
                } else if (steps == .5 && shooting_time.milliseconds() > 900) {
                    last_intake = intake_system.balls.size();
                    shooting_time.reset();
                    intake_system.spindexer.setPower(-.6);
                    intake_system.spindex_shooting = true;
                    intake_system.flippy_pos = flippy_up;
                    intake_system.blocking = false;
                    steps = 1;
                } else if (steps == 1 && (shooting_time.milliseconds() > 2000 || shooter.balls_shot >= intake_system.balls.size())) {
                    intake_system.flippy_pos = flippy_down;
                    intake_system.spindexer.setPower(1);
                    intake_system.spindex_shooting = false;
                    intake_system.blocking = true;
                    steps = 2;
                }
            }
        }
    }
    public int sort_count(){
        intake_system.recount();
        List<String> unsort = new ArrayList<>();
        if(intake_system.balls.size() == 1){
            unsort.add(intake_system.balls.get(0));
        }else if(intake_system.balls.size() == 2){
            unsort.add(intake_system.balls.get(1));
            unsort.add(intake_system.balls.get(0));
        }else if(intake_system.balls.size() == 3){
            unsort.add(intake_system.balls.get(2));
            unsort.add(intake_system.balls.get(1));
            unsort.add(intake_system.balls.get(0));
        }
        List<String> sort = new ArrayList<>();
        int num = 0;
        if(pattern == 1){
            sort.add("Green");
            sort.add("Purple");
            sort.add("Purple");
        }else if(pattern == 2){
            sort.add("Purple");
            sort.add("Green");
            sort.add("Purple");
        }else if(pattern == 3){
            sort.add("Purple");
            sort.add("Purple");
            sort.add("Green");
        }
        if(unsort.size() == 3) {
            if (unsort == sort) {
                num = 0;
            } else if (!unsort.contains("Green")) {
                num = 0;
            } else if (!unsort.contains("Purple")) {
                num = 0;
            } else {
                if (Collections.frequency(unsort, "Green") > 1) {
                    num = sort.indexOf("Purple") - unsort.indexOf("Purple");
                    if (num < 0) {
                        num += 3;
                    }
                } else {
                    num = sort.indexOf("Green") - unsort.indexOf("Green");
                    if (num < 0) {
                        num += 3;
                    }
                }
            }
        }else{
            if(unsort.contains("Green")){
                if(sort.indexOf("Green") != unsort.indexOf("Green")){
                    num = 1;
                }else {
                    num = 0;
                }
            }else{
                num = 0;
            }
        }
        return num;
    }
    public boolean sorted = false;
    public double sort_steps = 0;
    public ElapsedTime sort_timer = new ElapsedTime();
    public boolean sort_balls = false;
    public boolean override = false;
    public void auto_sort(){
        if(!sorted && intake_system.blocking && sort_balls) {
            if (intake_system.balls.size() == 3 || override) {
                if(sort_steps == 0){
                    sort_timer.reset();
                    sort_steps = 1;
                }else if(sort_steps == 1 && sort_timer.milliseconds() > 500){
                    sorted = true;
                    override = false;
                    intake_system.sort_time = sort_count();
                    sort_balls = false;
                    sort_steps = 0;
                }
            }

        }else if(!intake_system.blocking){
            sorted = false;
        }
    }



}
