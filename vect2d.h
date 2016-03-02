#ifndef VECT2D_H
#define VECT2D_H

#include <opencv2/core/core.hpp>

#define PI 3.14159265
#define GRAD_RAD PI/180.0
#define RAD_GRAD 180.0/PI

class Vect2D
{
public:
    double m_X, m_Y;	// genaue Koordinaten
    static Vect2D ini;

    Vect2D(double aX, double aY, bool aPolar = false);
    Vect2D();
    ~Vect2D();
    int XI();
    float XF();
    int YI();
    float YF();
    void SetP1P2(double aX1, double aY1, double aX2, double aY2);
    void SetXY(double aX, double aY);
    void SetXY(int aX, int aY);
    void SetFrom_R_Phi(double aR, double aPhi);
    double GetPhi();
    double GetPhiGrad();
    double GetR();
    void Add_R(double aR);
    void Add_Phi(double aPhi);
    void Add_PhiGrad(double aPhi);
    void Assign(Vect2D aVect);
    void AddTo(Vect2D aVect);
    Vect2D Add(Vect2D aVect);
    // Operatoren wurden nicht übertragen
    void AddTo(Vect2D aVect, double aFactor);
    void SubFrom(Vect2D aVect);
    void CoMultTo(Vect2D aB);
    Vect2D CoMult(Vect2D aB);
    Vect2D GetComplexConjugate();
    Vect2D ScalarMult(double aFactor);
    Vect2D GetNormalVector();
    Vect2D GetNormalizedVersion();
    Vect2D GetScaledVersion(double aLenght);
    Vect2D GetOppositeDirection();
    double VectLength();
    double DistBetweenPoints(Vect2D aB);
    bool IsZero();
    //Added Funktions
    double VectDotProduct(Vect2D aB);
    double AngleBetweenVect(Vect2D aB);
    double AngleBetweenVect_Grad(Vect2D aB);
    cv::Point getCvPoint();
private:
    //Vect2D *m_tmp = new Vect2D();
};

Vect2D VectBetweenPoints(Vect2D aP1, Vect2D aP2);

#endif // VECT2D_H
