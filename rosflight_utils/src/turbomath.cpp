/*
 * Copyright (c) 2017 James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rosflight_utils/turbomath.h>

#include <stdint.h>

static const int16_t atan_lookup_table[250] = {
  0,	40,	80,	120,	160,	200,	240,	280,	320,	360,	400,	440,	480,	520,	559,
  599,	639,	679,	719,	759,	798,	838,	878,	917,	957,	997,	1036,	1076,	1115,	1155,
  1194,	1234,	1273,	1312,	1352,	1391,	1430,	1469,	1508,	1548,	1587,	1626,	1664,	1703,	1742,
  1781,	1820,	1858,	1897,	1935,	1974,	2012,	2051,	2089,	2127,	2166,	2204,	2242,	2280,	2318,
  2355,	2393,	2431,	2469,	2506,	2544,	2581,	2618,	2656,	2693,	2730,	2767,	2804,	2841,	2878,
  2915,	2951,	2988,	3024,	3061,	3097,	3133,	3169,	3206,	3241,	3277,	3313,	3349,	3385,	3420,
  3456,	3491,	3526,	3561,	3596,	3631,	3666,	3701,	3736,	3771,	3805,	3839,	3874,	3908,	3942,
  3976,	4010,	4044,	4078,	4112,	4145,	4179,	4212,	4245,	4278,	4311,	4344,	4377,	4410,	4443,
  4475,	4508,	4540,	4572,	4604,	4636,	4668,	4700,	4732,	4764,	4795,	4827,	4858,	4889,	4920,
  4951,	4982,	5013,	5044,	5074,	5105,	5135,	5166,	5196,	5226,	5256,	5286,	5315,	5345,	5375,
  5404,	5434,	5463,	5492,	5521,	5550,	5579,	5608,	5636,	5665,	5693,	5721,	5750,	5778,	5806,
  5834,	5862,	5889,	5917,	5944,	5972,	5999,	6026,	6053,	6080,	6107,	6134,	6161,	6187,	6214,
  6240,	6267,	6293,	6319,	6345,	6371,	6397,	6422,	6448,	6473,	6499,	6524,	6549,	6574,	6599,
  6624,	6649,	6674,	6698,	6723,	6747,	6772,	6796,	6820,	6844,	6868,	6892,	6916,	6940,	6963,
  6987,	7010,	7033,	7057,	7080,	7103,	7126,	7149,	7171,	7194,	7217,	7239,	7261,	7284,	7306,
  7328,	7350,	7372,	7394,	7416,	7438,	7459,	7481,	7502,	7524,	7545,	7566,	7587,	7608,	7629,
  7650,	7671,	7691,	7712,	7733,	7753,	7773,	7794,	7814,	7834};


static const int16_t asin_lookup_table[250] = {
  0,	 40,	80,	120,	160,	200,	240,	280,	320,	360,	400,	440,	480,	520,	560,
  600,	 640,	681,	721,	761,	801,	841,	881,	921,	961,	1002,	1042,	1082,	1122,	1163,
  1203,	 1243,	1284,	1324,	1364,	1405,	1445,	1485,	1526,	1566,	1607,	1647,	1688,	1729,	1769,
  1810,	 1851,	1891,	1932,	1973,	2014,	2054,	2095,	2136,	2177,	2218,	2259,	2300,	2341,	2382,
  2424,	 2465,	2506,	2547,	2589,	2630,	2672,	2713,	2755,	2796,	2838,	2880,	2921,	2963,	3005,
  3047,	 3089,	3131,	3173,	3215,	3257,	3300,	3342,	3384,	3427,	3469,	3512,	3554,	3597,	3640,
  3683,	 3726,	3769,	3812,	3855,	3898,	3941,	3985,	4028,	4072,	4115,	4159,	4203,	4246,	4290,
  4334,	 4379,	4423,	4467,	4511,	4556,	4601,	4645,	4690,	4735,	4780,	4825,	4870,	4916,	4961,
  5007,	 5052,	5098,	5144,	5190,	5236,	5282,	5329,	5375,	5422,	5469,	5515,	5562,	5610,	5657,
  5704,	 5752,	5800,	5848,	5896,	5944,	5992,	6041,	6089,	6138,	6187,	6236,	6286,	6335,	6385,
  6435,	 6485,	6535,	6586,	6637,	6687,	6739,	6790,	6841,	6893,	6945,	6997,	7050,	7102,	7155,
  7208,	 7262,	7315,	7369,	7423,	7478,	7532,	7587,	7643,	7698,	7754,	7810,	7867,	7923,	7981,
  8038,	 8096,	8154,	8213,	8271,	8331,	8390,	8450,	8511,	8572,	8633,	8695,	8757,	8820,	8883,
  8947,	 9011,	9076,	9141,	9207,	9273,	9340,	9407,	9476,	9545,	9614,	9684,	9755,	9827,	9900,
  9973,	 10047,	10122,	10198,	10275,	10353,	10432,	10512,	10593,	10675,	10759,	10844,	10930,	11018,	11107,
  11198, 11290,	11385,	11481,	11580,	11681,	11784,	11890,	11999,	12111,	12226,	12346,	12469,	12597,	12730,
  12870, 13017,	13171,	13336,	13513,	13705,	13917,	14157,	14442,	14813};

static const int16_t sin_lookup_table[250] = {
  0,	63,	126,	188,	251,	314,	377,	440,	502,	565,	628,	691,	753,	816,	879,
  941,	1004,	1066,	1129,	1191,	1253,	1316,	1378,	1440,	1502,	1564,	1626,	1688,	1750,	1812,
  1874,	1935,	1997,	2059,	2120,	2181,	2243,	2304,	2365,	2426,	2487,	2548,	2608,	2669,	2730,
  2790,	2850,	2910,	2970,	3030,	3090,	3150,	3209,	3269,	3328,	3387,	3446,	3505,	3564,	3623,
  3681,	3740,	3798,	3856,	3914,	3971,	4029,	4086,	4144,	4201,	4258,	4315,	4371,	4428,	4484,
  4540,	4596,	4652,	4707,	4762,	4818,	4873,	4927,	4982,	5036,	5090,	5144,	5198,	5252,	5305,
  5358,	5411,	5464,	5516,	5569,	5621,	5673,	5724,	5776,	5827,	5878,	5929,	5979,	6029,	6079,
  6129,	6179,	6228,	6277,	6326,	6374,	6423,	6471,	6518,	6566,	6613,	6660,	6707,	6753,	6800,
  6845,	6891,	6937,	6982,	7026,	7071,	7115,	7159,	7203,	7247,	7290,	7333,	7375,	7417,	7459,
  7501,	7543,	7584,	7624,	7665,	7705,	7745,	7785,	7824,	7863,	7902,	7940,	7978,	8016,	8053,
  8090,	8127,	8163,	8200,	8235,	8271,	8306,	8341,	8375,	8409,	8443,	8477,	8510,	8543,	8575,
  8607,	8639,	8671,	8702,	8733,	8763,	8793,	8823,	8852,	8881,	8910,	8938,	8966,	8994,	9021,
  9048,	9075,	9101,	9127,	9152,	9178,	9202,	9227,	9251,	9274,	9298,	9321,	9343,	9365,	9387,
  9409,	9430,	9451,	9471,	9491,	9511,	9530,	9549,	9567,	9585,	9603,	9620,	9637,	9654,	9670,
  9686,	9701,	9716,	9731,	9745,	9759,	9773,	9786,	9799,	9811,	9823,	9834,	9846,	9856,	9867,
  9877,	9887,	9896,	9905,	9913,	9921,	9929,	9936,	9943,	9950,	9956,	9961,	9967,	9972,	9976,
  9980,	9984,	9987,	9990,	9993,	9995,	9997,	9998,	9999,	10000};

float sign(float y)
{
  return (0 < y) - (y < 0);
}


float asin_lookup(float x)
{
  static const float max = 1.0;
  static const float min = 0.0;
  static const int16_t num_entries = 250;
  static float dx = 0.0;
  static const float scale = 10000.0;

  float t = (x - min) / (max - min) * num_entries;
  uint8_t index = (uint8_t)t;
  dx = t - (float)index;

  if (index >= num_entries)
  {
    return max;
  }
  else if (index < num_entries - 1)
  {
    return (float)asin_lookup_table[index]/scale +
        dx * (float)(asin_lookup_table[index + 1] - asin_lookup_table[index]) / scale;
  }
  else
  {
    return (float)asin_lookup_table[index]/scale +
        dx * (float)(asin_lookup_table[index] - asin_lookup_table[index - 1]) / scale;
  }
}

float turboacos(float x)
{
  if (x < 0)
    return M_PI + asin_lookup(-x);
  else
    return M_PI - asin_lookup(x);
}


float turboasin(float x)
{
  if (x < 0)
    return -asin_lookup(-x);
  else
    return asin_lookup(x);
}

float sin_lookup(float x)
{
  static const float max = 1.0;
  static const float min = 0.0;
  static const int16_t num_entries = 250;
  static float dx = 0.0;
  static const float scale = 10000.0;

  float t = (x - min) / (max - min) * num_entries;
  uint8_t index = (uint8_t)t;
  dx = t - (float)index;

  if (index >= num_entries)
    return max;
  else if (index < num_entries - 1)
    return (float)sin_lookup_table[index]/scale +
        dx * (float)(sin_lookup_table[index + 1] - sin_lookup_table[index]) / scale;
  else
    return (float)sin_lookup_table[index]/scale +
        dx * (float)(sin_lookup_table[index] - sin_lookup_table[index - 1]) / scale;
}

float turbosin(float x)
{
  while (x > M_PI)
    x -= 2.0*M_PI;
  while (x <= -M_PI)
    x += 2.0*M_PI;

  if (0 <= x && x<=M_PI / 2.0)
    return sin_lookup(x);
  else if (-M_PI / 2.0 <= x && x < 0)
    return -sin_lookup(-x);
  else if (M_PI/2.0 < x)
    return sin_lookup(M_PI - x);
  else
    return -sin_lookup(x + M_PI);
}

float turbocos(float x)
{
  return turbosin(x + M_PI);
}


float atan_lookup(float x)
{
  if (x < 0)
    return -1*atan_lookup(-1 * x);
  if (x > 1.0)
    return M_PI - atan_lookup(1.0/x);

  static const float max = 1.0;
  static const float min = 0.0;
  static const int16_t num_entries = 250;
  static float dx = 0.0;
  static const float scale = 10000.0;

  float t = (x - min) / (max - min) * num_entries;
  uint8_t index = (uint8_t)t;
  dx = t - (float)index;

  if (index >= num_entries)
    return max;
  else if (index < num_entries - 1)
    return (float)atan_lookup_table[index]/scale +
        dx * (float)(atan_lookup_table[index + 1] - atan_lookup_table[index]) / scale;
  else
    return (float)atan_lookup_table[index]/scale +
        dx * (float)(atan_lookup_table[index] - atan_lookup_table[index - 1]) / scale;
}


float turboatan2(float y, float x)
{
  if (y == 0)
  {
    if (x < 0)
      return M_PI;
    else
      return 0;
  }

  else if (x == 0)
  {
    return M_PI/2.0 * sign(y);
  }

  else
  {
    float arctan = atan_lookup(x / y);

    if (y > 0)
      return M_PI/2.0 - arctan;
    else if (y < 0)
      return -M_PI/2.0 - arctan;
    else if (x < 0)
      return arctan + M_PI;
    else
      return arctan;
  }
}

double turbopow(double a, double b)
{
  union{
    double d;
    int x[2];
  } u = {a};
  u.x[1] = (int)(b*(u.x[1] - 1072632447) + 1072632447);
  u.x[0] = 0;
  return u.d;
}

float turboInvSqrt(float x)
{
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = x * 0.5F;
  y  = x;
  i  = * ( long * ) &y;                       // evil floating point bit level hacking
  i  = 0x5f3759df - ( i >> 1 );
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
  //	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

  return y;
}
