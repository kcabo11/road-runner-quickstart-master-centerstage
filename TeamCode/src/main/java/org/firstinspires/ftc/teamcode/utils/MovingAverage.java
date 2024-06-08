package org.firstinspires.ftc.teamcode.utils;

public class MovingAverage {

    private final float[] window;
    private float sum = 0f;
    private int fill;
    private int position;


    public MovingAverage(int size) {
        this.window=new float[size];
    }

    public void add(float number) {

        if(fill==window.length){
            sum-=window[position];
        }else{
            fill++;
        }

        sum+=number;
        window[position++]=number;

        if(position == window.length){
            position=0;
        }

    }

    public float getAverage() {
        return sum / fill;
    }

}