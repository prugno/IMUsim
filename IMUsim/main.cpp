//
//  main.cpp
//  TotalVariationFilter
//
//  Created by uea on 03/12/17.
//  Copyright © 2017 uea. All rights reserved.
//

#include <math.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/types.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

#define WINDOW 40
#define N_VAR 10

int system(const char *command);

FILE *din;
FILE *dout, *oout;
FILE *fout,*ch3d;

void TV1D_denoise(float* input, float* output, const int width, const float lambda);
float kalman_update(int index,float measurement);
void kalman_init(void);
void buildText3D(char * stg3,float x,float y,float z,float sca,FILE *dao);

double q[N_VAR]; //process noise covariance
double r[N_VAR]; //measurement noise covariance
double value[N_VAR]; //value
double p[N_VAR]; //estimation error covariance
double k[N_VAR]; //kalman gain

char stg[260],stg2[250],stg3[250],stgr[200],stgt[200];

int main() {
    //   din=fopen("/Users/uea/Documents/xcode/TotalVariationFilter/TotalVariationFilter/magX.txt","r");
    // dout=fopen("/Users/uea/Documents/xcode/TotalVariationFilter/TotalVariationFilter/magXFILT.txt","w");
    din=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/circle.txt","r");
    ch3d=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/y.obj","r");
   // dout=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/circleFILT.txt","w");
    fout=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/circlePATH.txt","w");
    fprintf(fout,"aTV_x aTV_y aTV_z / Xrot Yrot Zrot / X2 Y2 Z2\n");

    kalman_init();
    
    float datain[13][WINDOW],dataout[13][WINDOW];
    float ctx,cty;
    int w=WINDOW, w2=WINDOW,i=0,j=0,so=0,cnt=0,gct=0, m=0,f, find, ii;
    float la=3.0, ax,ay,az,q[4];
    double yaw,roll,pitch,ca,cb,cc,sa,sb,sc,xrot,yrot,zrot,xmd,ymd,zmd;
    float x1=0.0,y1=0.0,z1=0.0,x2=0.0,y2=0.0,z2=0.0;
    oout=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/Paths.obj","w"); //Comprehensive .obj file

    for(ctx=0.037-0.002;ctx<=0.039;ctx+=0.001){
        for(cty=0.022-0.002;cty<=0.024;cty+=0.001){
          //      for(cty=0.007-0.011;cty<=0.007;cty+=0.001){
            fout=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/circlePATH.txt","w");

                    sprintf(stgr,"circleFILT_X%.3f_Y%.3f",ctx,cty);
                    sprintf(stgt,"x%.3f y%.3f",ctx,cty);
            sprintf(stg2,"/Users/uea/Documents/xcode/IMUsim/IMUsim/path_data/circleFILT_X%.3f_Y%.3f.txt",ctx,cty);
            sprintf(stg3,"/Users/uea/Documents/xcode/IMUsim/IMUsim/path_data/circleFILT_X%.3f_Y%.3f.obj",ctx,cty);
            strcpy(stg,"");
          strcpy(stg,"/usr/local/bin/gnuplot -e \"set terminal postscript; set output '/Users/uea/Documents/xcode/IMUsim/IMUsim/path/");
            strcat(stg,stgr);
            strcat(stg,".eps'; plot '/Users/uea/Documents/xcode/IMUsim/IMUsim/path_data/");
            strcat(stg,stgr);
            strcat(stg,".txt' with lines\"");
            printf("%s\n\n",stg);
            dout=fopen(stg2,"w");
           // oout=fopen(stg3,"w"); //Multiple .obj files

           float r= (float)(rand() % 255);float g=(float)(rand() % 255);float b=(float)(rand() % 255);
            
            i=0;j=0;so=0;cnt=0;gct=0; m=0;x1=0.0;y1=0.0;z1=0.0;x2=0.0;y2=0.0;z2=0.0;
            rewind(din);
    if (din) {       // File is ok
        while (fscanf(din,"%f %f %f %f %f %f %f\n",&ax,&ay,&az,&q[0] , &q[1] , &q[2] , &q[3]) !=EOF) {
            if(so>0){
                datain[0][i]=ax;
                datain[1][i]=ay;
                datain[2][i]=az;
                datain[3][i]=q[0];
                datain[4][i]=q[1];
                datain[5][i]=q[2];
                datain[6][i]=q[3];
                i++;
                gct++;
                
                yaw   = atan2( 2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f-2.0f*( q[2] * q[2] + q[3] * q[3]) );
                roll = asin(2.0f * (q[0] * q[2] - q[1] * q[3]));
                pitch = atan2( 2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f-2.0f*( q[1] * q[1] + q[2] * q[2]) );
                

                cb = cos(roll);//R
                ca = cos(pitch);//P
                sb = sin(roll);
                sa = sin(pitch);
                cc = cos(yaw);
                sc = sin(yaw);
                
                datain[7][i]=pitch;
                datain[8][i]=roll;
                datain[9][i]=yaw;
                
       //     if(gct>70)    fprintf(dout,"%f %f %f / %f %f %f / %f %f %f\n",cb,roll,cc,xrot,yrot,zrot,x2,y2,z2);

                if(i==w){
                    TV1D_denoise(datain[0],dataout[0],w,la);
                    TV1D_denoise(datain[1],dataout[1],w,la);
                    TV1D_denoise(datain[2],dataout[2],w,la);
                    TV1D_denoise(datain[3],dataout[3],w,la);
                    TV1D_denoise(datain[4],dataout[4],w,la);
                    TV1D_denoise(datain[5],dataout[5],w,la);
                    TV1D_denoise(datain[6],dataout[6],w,la);
                    TV1D_denoise(datain[7],dataout[7],w,la);
                    TV1D_denoise(datain[8],dataout[8],w,la);
                    TV1D_denoise(datain[9],dataout[9],w,la);
                    
                    for(j=0;j<w;j++){
                  /*      fprintf(dout,"%f %f %f %f %f %f %f %f %f\n",
                                dataout[0][j],dataout[1][j],dataout[2][j],
                                datain[0][j],datain[1][j],datain[2][j],
                                kalman_update(0,dataout[0][j]),
                                kalman_update(1,dataout[1][j]),
                                kalman_update(2,dataout[2][j])
                                );*/
                        
                                        //IMU
       /*                 yaw   = atan2( 2.0f * (dataout[3][j] * dataout[6][j] + dataout[4][j] * dataout[5][j]), 1.0f-2.0f*( dataout[5][j] * dataout[5][j] + dataout[6][j] * dataout[6][j]) );
                        roll = asin(2.0f * (dataout[3][j] * dataout[5][j] - dataout[4][j] * dataout[6][j]));
                        pitch = atan2( 2.0f * (dataout[3][j] * dataout[4][j] + dataout[5][j] * dataout[6][j]), 1.0f-2.0f*( dataout[4][j] * dataout[4][j] + dataout[5][j] * dataout[5][j]) );
        */
        

                        cb = cos(dataout[8][j]);//R
                        ca = cos(dataout[7][j]);//P
                        sb = sin(dataout[8][j]);
                        sa = sin(dataout[7][j]);
                        cc = cos(dataout[9][j]);
                        sc = sin(dataout[9][j]);
                        
                        dataout[2][j]*=-1.0f;
                        
                        xrot=kalman_update(3,dataout[0][j] *cb *cc + ca* (-dataout[2][j]* cc *sb + dataout[1][j]* sc) +  sa *(dataout[1][j] *cc* sb + dataout[2][j] *sc));
                        yrot=kalman_update(4,dataout[2][j] *cc *sa - (dataout[0][j]* cb + dataout[1][j] *sb *sa) *sc + ca* (dataout[1][j]* cc + dataout[2][j]* sb* sc));
                        zrot=kalman_update(5,dataout[0][j] *sb + cb *(dataout[2][j] *ca - dataout[1][j] *sa) +1.0f);

                        xmd=floor(xrot*1000.0)/1000.0;
                        ymd=floor(yrot*1000.0)/1000.0;
                        zmd=floor(zrot*1000.0)/1000.0;
                        
                        datain[10][m]=xmd-0.038;
                        datain[11][m]=ymd-0.020;
                        datain[12][m]=zmd-0.007;
                        datain[10][m]=xmd-ctx;
                        datain[11][m]=ymd-cty;
                        datain[12][m]=zmd-0.007;
                        m++;
                        
                        if(m==w){
                            TV1D_denoise(datain[10],dataout[10],w,la);
                            TV1D_denoise(datain[11],dataout[11],w,la);
                            TV1D_denoise(datain[12],dataout[12],w,la);
                            
                            for(f=0;f<w;f++){
                                
                        if(gct>700){

                                x1+= dataout[10][f];
                                y1+= dataout[11][f];
                                z1+= dataout[12][f];
                            x2+=x1+dataout[10][f]*0.5f;
                            y2+=y1+dataout[11][f]*0.5f;
                                z2+=z1+dataout[12][f]*0.5f;
                            
                            fprintf(dout,"%f %f\n",x2,z2);
                            fflush(fout);
                            fprintf(fout,"%f %f %f / %f %f %f / %f %f %f\n",dataout[0][j],dataout[1][j],dataout[2][j],x1,y1,z1,x2,y2,z2);
                            fprintf(oout,"v %f %f %f %f %f %f\n",x2,y2,z2,r,g,b);
                          //  fprintf(dout,"%f %f %f / %f %f %f / %f %f %f\n",dataout[10][f],dataout[11][f],dataout[12][f],x1,y1,z1,x2,y2,z2);
                        }
                            }
                            m=0;
                           // x1=0.0;
                            //y1=0.0;
                           // z1=0.0;
                            xmd=0.0;
                            ymd=0.0;
                            zmd=0.0;
                        }}
                  //  x1=0.0;
                  //  y1=0.0;
                  //  z1=0.0;
                     xmd=0.0;
                    ymd=0.0;
                    zmd=0.0;
                    i=0;
                }
            }
            so=1;
        }
        int n;
        /*
        for(n=0;n<10;n++) fprintf(oout,"v %f %f %f %f %f %f\n",x2+n*200.0,y2,z2,r,g,b);
        for(n=0;n<10;n++) fprintf(oout,"v %f %f %f %f %f %f\n",x2,y2+n*200.0,z2,r,g,b);
        for(n=0;n<10;n++) fprintf(oout,"v %f %f %f %f %f %f\n",x2,y2,z2+n*200.0,r,g,b);
        */
    
            buildText3D(stgt,x2,y2,z2,15.0,oout);

    }
            fclose(fout);
            fclose(dout);
          //  fclose(oout); //Multiple .obj files

            system(stg);
        }} //FOR parameter change
    fclose(oout);       //Comprehensive .obj file

    fclose(ch3d);
    fclose(din);
    printf("END reached\n");
}


float kalman_update(int index,float measurement)
{
    p[index]+= q[index];
    k[index] = p[index] / (p[index] + r[index]);
    value[index] += k[index] * (measurement - value[index]);
    p[index] *= (1.0 - k[index]);
    return value[index];
}

void kalman_init(void)
{
    int l;
    for(l=0; l<N_VAR; l++) {
        q[l]=0.01; //process noise covariance. Piccolo=liscio
        r[l]=0.55; //measurement noise covariance
        p[l]=0.0; //estimation error covariance
        k[l]=0.0; //kalman gain
        value[l]=0.0;
    }
}


void buildText3D(char * stg3,float x,float y,float z,float sca,FILE *dao){
    char stg2[100];char stg[100];
    int i;
    float ax,ay,az,xx2=0.0;
    FILE *dain;
  //  dao=fopen("/Users/uea/Documents/xcode/IMUsim/IMUsim/model.obj","w");
    for(i=0;i<strlen(stg3);i++){
        if(stg3[i]!='.') sprintf(stg2,"%c.obj",stg3[i]);
        if(stg3[i]=='.') sprintf(stg2,"dot.obj");
        strcpy(stg,"/Users/uea/Documents/xcode/3DAlphabet/3DAlphabet/tifs/");
        strcat(stg,stg2);
              //  printf("%s\n",stg);
        dain=fopen(stg,"r");
        while (fscanf(dain,"v %f %f %f\n",&ax,&az,&ay) !=EOF) {
            fprintf(dao,"v %f %f %f\n",x+ax*sca +xx2,y+ay*sca,z+az*sca);
        }
        xx2+=29.0*sca;
        fclose(dain);
    }
  //  fclose(dao);
}

void TV1D_denoise(float* input, float* output, const int width, const float lambda) {
    if (width>0) {                /*to avoid invalid memory access to input[0]*/
        int k=0, k0=0;            /*k: current sample location, k0: beginning of current segment*/
        float umin=lambda, umax=-lambda;    /*u is the dual variable*/
        float vmin=input[0]-lambda, vmax=input[0]+lambda;    /*bounds for the segment's value*/
        int kplus=0, kminus=0;     /*last positions where umax=-lambda, umin=lambda, respectively*/
        const float twolambda=2.0*lambda;    /*auxiliary variable*/
        const float minlambda=-lambda;        /*auxiliary variable*/
        for (;;) {                /*simple loop, the exit test is inside*/
            while (k==width-1) {    /*we use the right boundary condition*/
                if (umin<0.0) {            /*vmin is too high -> negative jump necessary*/
                    do output[k0++]=vmin; while (k0<=kminus);
                    umax=(vmin=input[kminus=k=k0])+(umin=lambda)-vmax;
                } else if (umax>0.0) {    /*vmax is too low -> positive jump necessary*/
                    do output[k0++]=vmax; while (k0<=kplus);
                    umin=(vmax=input[kplus=k=k0])+(umax=minlambda)-vmin;
                } else {
                    vmin+=umin/(k-k0+1);
                    do output[k0++]=vmin; while(k0<=k);
                    return;
                }
            }
            if ((umin+=input[k+1]-vmin)<minlambda) {        /*negative jump necessary*/
                do output[k0++]=vmin; while (k0<=kminus);
                vmax=(vmin=input[kplus=kminus=k=k0])+twolambda;
                umin=lambda; umax=minlambda;
            } else if ((umax+=input[k+1]-vmax)>lambda) {    /*positive jump necessary*/
                do output[k0++]=vmax; while (k0<=kplus);
                vmin=(vmax=input[kplus=kminus=k=k0])-twolambda;
                umin=lambda; umax=minlambda;
            } else {     /*no jump necessary, we continue*/
                k++;
                if (umin>=lambda) {        /*update of vmin*/
                    vmin+=(umin-lambda)/((kminus=k)-k0+1);
                    umin=lambda;
                }
                if (umax<=minlambda) {    /*update of vmax*/
                    vmax+=(umax+lambda)/((kplus=k)-k0+1);
                    umax=minlambda;
                }
            }
        }
    }
}


