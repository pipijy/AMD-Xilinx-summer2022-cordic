#include "cordiccart2pol.h"

//角度查找表
ap_fixed<W, I, AP_RND, AP_WRAP, 1> cordic_phase[NO_ITER] = {
    45, 26.56505118, 14.03624347, 7.125016349,
    3.576334375, 1.789910608, 0.8951737102, 0.4476141709,
    0.2238105004, 0.1119056771, 0.05595289189, 0.02797645262,
    0.01398822714,0.006994113675, 0.003497056851, 0.001748528427  //...
};

void pre_cir_cordic(data_t x, data_t y, d_t &x_new, d_t &y_new, flag_t &flag)
{
	d_t xi = x;
	d_t yi = y;

	if(xi < 0)
    {
		x_new=-xi;
		if(y<0)
        {
			y_new = yi;
            flag = 1;
		}
        else
        {
			y_new = yi;
            flag=2;
		}
	}
    else
    {
		x_new = xi;
        flag = 0;
	}
}

void cir_cordic_calculate(d_t x_new, d_t y_new, flag_t flag, d_t &myr, d_t &mytheta, flag_t &flag_delay)
{
	const int N = NO_ITER;
	d_t xii[N];
	d_t yii[N];
	d_t zii[N];
	flag_t flag_delay_a[N];

	xii[0] = x_new;
	yii[0] = y_new;
	zii[0] = 0;
	flag_delay_a[0] = flag;

	int m = 0;
#pragma HLS UNROLL
	for(m = 0; m<N; m++){
#pragma HLS LOOP_TRIPCOUNT min=1 max=16
#pragma HLS PIPELINE
		if(zii[m] >= 0)
		{
			xii[m+1] = xii[m] - (yii[m] >> m);
			yii[m+1] = yii[m] + (xii[m] >> m);
			zii[m+1] = zii[m] - cordic_phase[m];
		}
		else
		{
			xii[m+1] = xii[m] + (yii[m] >> m);
			yii[m+1] = yii[m] - (xii[m] >> m);
			zii[m+1] = zii[m] + cordic_phase[m];
		}
		flag_delay_a[m+1] = flag_delay_a[m];
	}

    mytheta = zii[N-1];
    // myr = (float)xi[n-1]*0.607252941;
	myr = xii[N-1];
	flag_delay = flag_delay_a[N-1];
}

void post_cir_cordic(d_t myr, d_t mytheta, flag_t flag_delay, d_t &r_temp, d_t &theta_temp)
{
	switch(int(flag_delay))
	{
	case 1: r_temp = myr; theta_temp = -180-mytheta; break;
	case 2: r_temp = myr; theta_temp = 180-mytheta; break;
	default: r_temp = myr; theta_temp = mytheta; break;
	}
}

void cordiccart2pol(data_t x, data_t y, data_t *r, data_t *theta)
{

#pragma HLS INTERFACE s_axilite port=x  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=r  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return

//d_t current_x = x;
//d_t current_y = y;

d_t x_new,y_new;
flag_t flag;
d_t myr,mytheta;
flag_t flag_delay;

d_t r_temp;
d_t theta_temp;

	pre_cir_cordic(x, y, x_new, y_new, flag);
	cir_cordic_calculate(x_new, y_new, flag, myr, mytheta, flag_delay);
	post_cir_cordic(myr, mytheta, flag_delay, r_temp, theta_temp);

	*theta = (float)theta_temp;
	*r = (float)r_temp*0.607252941;
}
