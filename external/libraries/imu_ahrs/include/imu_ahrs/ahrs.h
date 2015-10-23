#ifndef AHRS_H
#define AHRS_H

class AHRS{
protected:
    float m_q0, m_q1, m_q2, m_q3;	// quaternion of sensor frame relative to auxiliary frame

public:
    float q0(){
        return m_q0;
    }
    float q1(){
        return m_q1;
    }
    float q2(){
        return m_q2;
    }
    float q3(){
        return m_q3;
    }

public:

};


#endif // AHRS_H
