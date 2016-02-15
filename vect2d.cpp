#include "vect2d.h"
#include <stdlib.h>
#include "math.h"

Vect2D::Vect2D(double aX, double aY, bool aPolar) : Vect2D()
{
    if (!aPolar)
    {
    m_X = aX; m_Y = aY;
    }
    else {
    m_X = aX * cos(GRAD_RAD * aY);
    m_Y = aX * sin(GRAD_RAD * aY);
    }
}

Vect2D::Vect2D()
{
    m_X = 0; m_Y = 0;
}

Vect2D::~Vect2D()
{

}

int Vect2D::XI()
{
    return (int) m_X;
}

float Vect2D::XF()
{
    return (float) m_X;
}

int Vect2D::YI()
{
    return (int) m_Y;
}

float Vect2D::YF()
{
    return (float) m_Y;
}

void Vect2D::SetP1P2(double aX1, double aY1, double aX2, double aY2)
{
    m_X = aX2 - aX1;
    m_Y = aY2 - aY1;
}

void Vect2D::SetXY(double aX, double aY)
{
    m_X = aX;
    m_Y = aY;
}

void Vect2D::SetXY(int aX, int aY)
{
    m_X = aX;
    m_Y = aY;
}

void Vect2D::SetFrom_R_Phi(double aR, double aPhi)
{
    m_X = aR * cos(GRAD_RAD * aPhi);
    m_Y = aR * sin(GRAD_RAD * aPhi);
}

double Vect2D::GetPhi()
{
    if( m_X == 0.0 && m_Y == 0.0 )
    {
        return 0.0;
    }

    if( m_X == 0.0 )
    {
        if( m_Y > 0.0 )
        {
            return PI/2;
        }
        else
        {
            return -PI/2;
        }
    }

    if( m_Y == 0.0 )
    {
        if( m_X > 0.0 )
        {
            return 0;
        }
        else
        {
            return -PI;
        }
    }

    double phi = abs(atan(m_Y/m_X));

    if( m_X > 0.0 )
    {
        if( m_Y > 0 )
        {
            return abs(phi);
        }
        else // m_Y<0.0
        {
            return -abs(phi);
        }
    }
    else // m_X<0.0
    {
        if( m_Y>0 )
        {
            return PI - abs(phi);
        }
        else // m_Y<0.0
        {
            return -(PI - abs(phi));
        }
    }
}

double Vect2D::GetPhiGrad()
{
    return RAD_GRAD * GetPhi();
}

double Vect2D::GetR()
{
    return sqrt(m_X * m_X + m_Y * m_Y);
}

void Vect2D::Add_R(double aR)
{
    double r = sqrt(m_X * m_X + m_Y * m_Y);
    r += aR;
    double phi = GetPhi();
    m_X = r * cos(phi);
    m_Y = r * sin(phi);
}

void Vect2D::Add_Phi(double aPhi)
{
    //m_tmp->m_X = cos(aPhi);
    //m_tmp->m_Y = sin(aPhi);
    CoMultTo(Vect2D(cos(aPhi), sin(aPhi), false));
}

void Vect2D::Add_PhiGrad(double aPhi)
{
    this->Add_Phi(GRAD_RAD * aPhi);
}

void Vect2D::Assign(Vect2D aVect)
{
    m_X = aVect.m_X;
    m_Y = aVect.m_Y;
}

void Vect2D::AddTo(Vect2D aVect)
{
    m_X = m_X + aVect.m_X;
    m_Y = m_Y + aVect.m_Y;
}

Vect2D Vect2D::Add(Vect2D aVect)
{
    //m_tmp->m_X = m_X + aVect.m_X;
    //m_tmp->m_Y = m_Y + aVect.m_Y;
    return Vect2D((m_X + aVect.m_X), (m_Y + aVect.m_Y), false);
}

// Operatoren wurden nicht übertragen
void Vect2D::AddTo(Vect2D aVect, double aFactor)
{
    m_X = m_X + aVect.m_X * aFactor;
    m_Y = m_Y + aVect.m_Y * aFactor;
}

void Vect2D::SubFrom(Vect2D aVect)
{
    m_X = m_X - aVect.m_X;
    m_Y = m_Y - aVect.m_Y;
}

double Vect2D::VectLength()
{
    return sqrt(m_X * m_X + m_Y * m_Y);
}

Vect2D VectBetweenPoints(Vect2D aP1, Vect2D aP2)
{
    Vect2D tmp = Vect2D();
    tmp.m_X = aP2.m_X - aP1.m_X;
    tmp.m_Y = aP2.m_Y - aP1.m_Y;
    return tmp;
}

void Vect2D::CoMultTo(Vect2D aB)
{
    double Xres, Yres;
    Xres = m_X * aB.m_X - m_Y * aB.m_Y;
    Yres = m_X * aB.m_Y + m_Y * aB.m_X;
    m_X = Xres;
    m_Y = Yres;
}

Vect2D Vect2D::CoMult(Vect2D aB)
{
    double Xres, Yres;
    Xres = m_X * aB.m_X - m_Y * aB.m_Y;
    Yres = m_X * aB.m_Y + m_Y * aB.m_X;
    //m_tmp->m_X = Xres;
    //m_tmp->m_Y = Yres;
    return Vect2D(Xres, Yres, false);
}

Vect2D Vect2D::GetComplexConjugate()
{
    //m_tmp->m_X = m_X;
    //m_tmp->m_Y = -m_Y;
    return Vect2D(m_X, -m_Y, false);
}

Vect2D Vect2D::ScalarMult(double aFactor)
{
    //m_tmp->m_X = m_X * aFactor;
    //m_tmp->m_Y = m_Y * aFactor;
    return Vect2D((m_X * aFactor),(m_Y * aFactor),false);
}

Vect2D Vect2D::GetNormalVector()
{
    //m_tmp->m_X = -m_Y;
    //m_tmp->m_Y = m_X;
    return Vect2D(-m_Y, m_X, false);
}

Vect2D Vect2D::GetNormalizedVersion()
{
    double r = VectLength();
    //m_tmp->m_X = m_X / r;
    //m_tmp->m_Y = m_Y / r;
    return Vect2D((m_X / r), (m_Y / r), false);
}

Vect2D Vect2D::GetScaledVersion(double aLenght)
{
    //*m_tmp = GetNormalizedVersion();
    //*m_tmp = m_tmp->ScalarMult(aLenght);
    return GetNormalizedVersion().ScalarMult(aLenght);
}

Vect2D Vect2D::GetOppositeDirection()
{
    //m_tmp->m_X = -m_X;
    //m_tmp->m_Y = -m_Y;
    return Vect2D(-m_X, -m_Y, false);
}

double Vect2D::DistBetweenPoints(Vect2D aB)
{
    double xd = m_X - aB.m_X;
    double yd = m_Y - aB.m_Y;
    return sqrt(xd * xd + yd * yd);
}

bool Vect2D::IsZero()
{
    return m_X == 0.0 && m_Y == 0.0;
}

//Added Funktions
double Vect2D::VectDotProduct(Vect2D aB)
{
    Vect2D tmp1 = this->GetNormalizedVersion();
    Vect2D tmp2 = aB.GetNormalizedVersion();
    return (tmp1.m_X * tmp2.m_X + tmp1.m_Y * tmp2.m_Y);
}

double Vect2D::AngleBetweenVect(Vect2D aB)
{
    double tmp = atan2(aB.m_Y, aB.m_X) - atan2(this->m_Y, this->m_X);
    if (tmp > PI)
    {
        tmp -= PI * 2;
    }
    else if (tmp < -PI)
    {
        tmp += PI * 2;
    }
    return tmp;
}

double Vect2D::AngleBetweenVect_Grad(Vect2D aB)
{
    return (180 / PI)*this->AngleBetweenVect(aB);
}

cv::Point Vect2D::getCvPoint(){
    return cv::Point(m_X,m_Y);
}

