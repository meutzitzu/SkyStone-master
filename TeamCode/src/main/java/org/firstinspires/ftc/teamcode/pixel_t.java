package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

public class pixel_t {
    final static double SCALE_FACTOR = 255;
    public int i,j;///the pixel's position in an image
    public short r,g,b; /// color_values
    public double h,s,v;

    public pixel_t(){
        this.i = -1;
        this.j = -1;
        this.r = -1;
        this.g = -1;
        this.b = -1;
        this.h = -1;
        this.s = -1;
        this.v = -1;
    }

    public void setPos(int i,int j){
        this.i = i;
        this.j = j;
    }

    public void setRGB(short r,short g,short b,boolean convert){
        this.r = r;
        this.g = g;
        this.b = b;
        if(convert == true){
            float hsvValues[] = {0,0,0};
            Color.RGBToHSV((int) (this.r * SCALE_FACTOR),
                    (int) (this.g * SCALE_FACTOR),
                    (int) (this.v * SCALE_FACTOR),
                    hsvValues);
            this.h = hsvValues[0];
            this.s = hsvValues[1];
            this.v = hsvValues[2];
        }
    }

    public void setHSV(double h,double s,double v){
        this.h = h;
        this.s = s;
        this.v = v;
    }

    public boolean isValid(){
        return this.i != -1 && this.j != -1;
    }
}
