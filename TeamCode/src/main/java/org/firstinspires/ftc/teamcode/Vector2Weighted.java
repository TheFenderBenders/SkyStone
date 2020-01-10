package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Vector2Weighted {

    public static Vector2d createVector(double x, double y){

        return new Vector2d(x, y);
//        return new Vector2d(x, y*(1.15384615));
    }
}
