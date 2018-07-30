#ifndef CAR_STATE_HPP
#define CAR_STATE_HPP

class CarState
{
public:
	CarState(int id, double x, double y, double v_x, double v_y, double s, double d);

	int m_id;
	double m_x;
	double m_y;
	double m_vx;
	double m_vy;
	double m_v;
	double m_s;
	double m_d;
	double m_yaw;
};

#endif