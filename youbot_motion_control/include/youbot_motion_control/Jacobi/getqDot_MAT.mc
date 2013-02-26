void getqDot(double A[5][5],double xDot[6],double qDot[5])
{
  double t11;
  double t14;
  double t2;
  double t5;
  double t8;
  {
    t2 = xDot[0];
    t5 = xDot[1];
    t8 = xDot[2];
    t11 = xDot[3];
    t14 = xDot[4];

    qDot[0] = A[0][0]*t2+A[0][1]*t5+A[0][2]*t8+A[0][3]*t11+A[0][4]*t14;
    qDot[1] = A[1][0]*t2+A[1][1]*t5+A[1][2]*t8+A[1][3]*t11+A[1][4]*t14;
    qDot[2] = A[2][0]*t2+A[2][1]*t5+A[2][2]*t8+A[2][3]*t11+A[2][4]*t14;
    qDot[3] = A[3][0]*t2+A[3][1]*t5+A[3][2]*t8+A[3][3]*t11+A[3][4]*t14;
    qDot[4] = A[4][0]*t2+A[4][1]*t5+A[4][2]*t8+A[4][3]*t11+A[4][4]*t14;
    return;
  }
}

