#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#define R 0.2
#define Ld 0.4e-3
#define Lq 0.4e-3
#define J 5e-5
#define P 4
#define B 1
#define lambda 0.17
#define N 6

float yt_a[N]; /* yt for adaptive rk4 */
float yt2[N];

typedef struct _rk_param{
    float yt[N]; /* yt for rk4 */
    float f1[N];
    float f2[N];
    float f3[N];
    float f4[N];
}rk_param;

typedef enum params{ID=0,IQ,WR,VD,VQ,TI,ODE_COUNT}params;
/* 
u[0] = id
u[1] = iq
u[2] = wr
u[3] = vd
u[4] = vq
u[5] = Ti
*/
static inline void Pendulum(float t, float u[], float f[])
{
    f[0] = -R/Ld * u[0] + Ld/Lq * P * u[2]*u[1] + 1/Ld * u[3];
    f[1] = -R/Lq * u[1] - Ld/Lq * P * u[2] * u[0] - (lambda*P*u[2])/Lq  + 1/Lq * u[4];
    f[2] = P/J * (lambda*u[1] + (Ld-Lq)*u[1]*u[0]) - B/J*u[2] - u[5]/J;
    f[3] = 0;//u[3];
    f[4] = 0;//u[4];
    f[5] = 0;//u[5];
}


static void rk4(float t, float ht, float y[], int n, void Func(float,float[],float[]))
{
    static rk_param rk;
    float ht2, ht6;
    int i;

    ht2 = ht/2e0;
    Func(t,y,rk.f1);
    for(i=0;i<n;i++)
    {
        rk.yt[i] = y[i] + ht2 * rk.f1[i];
    }
    Func(t+ht2, rk.yt, rk.f2);
    for(i=0; i<=0; i++)
    {
        rk.yt[i] = y[i] + ht2 * rk.f2[i];
    }
    Func(t+ht,rk.yt,rk.f3);
    for(i=0; i<n; i++)
    {
        rk.yt[i] = y[i] + ht * rk.f3[i];
    }
    Func(t+ht, rk.yt, rk.f4);
    ht6 = ht/6e0;
    for(i=0;i<n;i++)
    {
        y[i] += ht6 * (rk.f1[i] + 2 * (rk.f2[i] +rk.f3[i]) + rk.f4[i]);
    }
}
void adaptrk4(float t, float *ht,float *ht1, float eps, float y[], int n,void Func(float, float[],float[])  )
{

    float err, erri, f, ht2;
    int i, it;
    const int p = 4;
    static int itmax = 10;
    for(it = 0; it < itmax; it++)
    {
        ht2 = (*ht)/2e0;
        for (i=0; i < n; i++)
        {
            yt2[i] = y[i];  
            yt_a[i] = y[i];
        }
        rk4(t,*ht,yt_a,n,Func);
        rk4(t,ht2,yt2,n,Func);
        rk4(t+ht2,ht2,yt2,n,Func);

        err = 0e0;
        for(i=0;i<n;i++)
        {
            erri = yt2[i] ? fabs(1e0 - yt_a[i] / yt2[i]) : fabs(yt2[i] - yt_a[i]);
            if (err < erri)
            {
                err = erri;
            }
        }
        f = 1e0;
        if(err) f = 0.9e0 * pow(eps/err, 1e0/p);
        if(f>5e0) f = 5e0;
        *ht1 = f * (*ht);
        if (err <= eps) break;
        *ht = *ht1;
    }
    if(it > itmax) printf("Adapt max no of iterations exceeded! \n");
    for(i=0; i<n; i++) y[i] = yt2[i];
}

int main()
{
    float eps, ht,ht1, t, t1, t2, tmax, T0, T, u0, du0, us, *u;
    int n, nT, stepcount;
    FILE *out = fopen("plot.csv","w");
    fprintf(out,"t,id,iq,wr,ud,uq,tl\n");
    stepcount = 0;
    tmax = 3;// time span
    ht = 0.01e0;// time step size
    ht1 = ht; // output 
    eps = 1e-3;

    n = 6;// number of 1st order ODEs
    u = (float*)malloc(n*sizeof(float));          // solution components 
    t = 0e0;  
    u[0] = 0;
    u[1] = 0; 
    u[2] = 0; 
    u[3] = 0;
    u[4] = 0;
    u[5] = 0;
    fprintf(out,"%f,%f,%f,%f,%f,%f,%f\n",t,u[0],u[1],u[2],u[3],u[4],u[5]);
    while(t+ht < tmax)
    {
        if (t > 1)
        {
            u[3] = 50;
            u[4] = 50;
        }
        if (t > 2)
        {
            u[5] = 2;
        }
        stepcount++;
        ht = ht1;
        adaptrk4(t,&ht,&ht1,eps, u,n,Pendulum);
        t += ht;
        fprintf(out,"%f,%f,%f,%f,%f,%f,%f\n",t,u[0],u[1],u[2],u[3],u[4],u[5]);
        printf("Step:%lf\n\tu0 => %lf\n\tu1 => %lf\n",t,u[0],u[1]);
    }
    free(u);
    fclose(out);
    printf("Stepcount: %d\n",stepcount);
    return 0;
}
