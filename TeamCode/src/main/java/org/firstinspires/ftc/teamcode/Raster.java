package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.pixel_t;

///Useful information:
///-our phone's camera takes 2448 x 3264 images

public class Raster {
    int h,w;///height and width of raster
    pixel_t pixels[][];

    Raster() {
        h = -1;
        w = -1;
        pixels = new pixel_t[1][1];
    }

    Raster(int h,int w){
        this.h = h;
        this.w = w;
        pixels = new pixel_t[h][w];
    }

    pixel_t getPixel(int i,int j){
        return pixels[i][j];
    }

    Raster getSubsection(int x1,int y1,int x2,int y2){
        Raster ans = new Raster(x2 - x1 + 1,y2 - y1 + 1);
        for(int i = x1;i <= x2;i++) {
            for (int j = y1; j <= y2; j++) {
                ans.pixels[i - x1][j - y1] = this.pixels[i][j];
            }
        }
        return ans;
    }

    Raster getFrame() {
        Raster ans = new Raster(2448,3264);

        ///TODO somehow get the image from the camera and put it into a raster
    }
}
