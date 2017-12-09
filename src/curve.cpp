#include "curve.h"
#include "vertexrecorder.h"
#include <cmath>
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}
Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
	// Check
    // cerr << "evalBezier must be called with 3n+1 control points." << endl;
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.

    int num_segments = (P.size()-1) / 3;

    Curve R;

    Matrix4f Bernstein = Matrix4f(
        1,-3, 3,-1,
        0, 3,-6, 3,
        0, 0, 3,-3,
        0, 0, 0, 1
    );

    Vector3f B = Vector3f(0,0,1);

    for (int i = 0; i < num_segments; i++){
        // Vector3f Point1 = P[3*i];
        // Vector3f Point2 = P[3*i +1];
        // Vector3f Point3 = P[3*i +2];
        // Vector3f Point4 = P[3*i +3];

        Matrix4f Points(
            P[3*i][0], P[3*i +1][0], P[3*i +2][0], P[3*i +3][0],
            P[3*i][1], P[3*i +1][1], P[3*i +2][1], P[3*i +3][1],
            P[3*i][2], P[3*i +1][2], P[3*i +2][2], P[3*i +3][2],
            0,0,0,0
        );

        float increment = 1.0/steps;

        /* Need one more step than usual for the last segment so that that
        the last point shows up. In the other segments, the last point is the
        the same as the first point of the next segment */
        if ( i == num_segments -1){
            steps += 1;
        }
        for (int j = 0;  j< steps; j++){
            float t = increment*j;
            Matrix4f temp = Points*Bernstein;

            Vector4f cononical = Vector4f(1, t, t*t, t*t*t);
            Vector4f V_extra0 = temp*cononical;
            Vector3f V(V_extra0[0],V_extra0[1],V_extra0[2]);

            Vector4f derivative = Vector4f(0, 1, 2*t, 3*t*t);
            Vector4f T_extra0 = temp*derivative;
            Vector3f T(T_extra0[0],T_extra0[1],T_extra0[2]);
            T = T.normalized();

            Vector3f N;
            N = Vector3f::cross(B, T).normalized();
            // B = Vector3f::cross(T, N).normalized();
            CurvePoint CP = CurvePoint();

            if(approx(B,T)){
                B = Vector3f(0,0,1.00001);
            }
            B = Vector3f::cross(T,N).normalized();

            CP.V = V;
            CP.T = T;
            CP.B = B;
            CP.N = N;

            R.push_back(CP);

        }

    }

	cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i] << endl;
	}

	// Right now this will just return this empty curve.
	return R;
}

Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier fufnction.

    Matrix4f Bspline = Matrix4f(
        1, -3, 3, -1,
        4, 0, -6, 3,
        1, 3, 3, -3,
        0, 0, 0, 1
    );
    Bspline = (1.0/6.0) * Bspline;
    Matrix4f Bernstein = Matrix4f(
        1,-3, 3,-1,
        0, 3,-6, 3,
        0, 0, 3,-3,
        0, 0, 0, 1
    );
    Matrix4f BernsteinInverse = Bernstein.inverse();

    int size = P.size();

    vector<Vector3f> outputP;

    int num_segments = (size-1)/3;
    //
    for (int i=0; i< size-3; i++){
        Matrix4f Points(
            P[i][0], P[i +1][0], P[i +2][0], P[i +3][0],
            P[i][1], P[i +1][1], P[i +2][1], P[i +3][1],
            P[i][2], P[i +1][2], P[i +2][2], P[i +3][2],
            0,0,0,0
        );
        Matrix4f transPoint = Points*Bspline*BernsteinInverse;

    // // vectors with the extra zero at the end
        Vector4f temp1 = transPoint.getCol(0);
        Vector4f temp2 = transPoint.getCol(1);
        Vector4f temp3 = transPoint.getCol(2);
        Vector4f temp4 = transPoint.getCol(3);
    // // get rid of the extra zero
        outputP.push_back(Vector3f(temp1[0], temp1[1], temp1[2]));
        outputP.push_back(Vector3f(temp2[0], temp2[1], temp2[2]));
        outputP.push_back(Vector3f(temp3[0], temp3[1], temp3[2]));

        if (i == size-4  ){
            outputP.push_back(Vector3f(temp4[0], temp4[1], temp4[2]));
        }
    }

	// cerr << "\t>>> evalBSpline has been called with the following input:" << endl;
    //
	// cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	// for (int i = 0; i < (int)P.size(); ++i)
	// {
	// 	cerr << "\t>>> " << P[i] << endl;
	// }
    //
	// cerr << "\t>>> Steps (type steps): " << steps << endl;
	// cerr << "\t>>> Returning empty curve." << endl;
    // evaulate the curve using evalBezier
	return evalBezier(outputP, steps);
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);

	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));

		// Transform orthogonal frames into model space
		Vector4f MORGN  = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}
