#ifndef CAR_STATE_HPP
#define CAR_STATE_HPP

class CarState
{
public:
	CarState(int id, double x, double y, double v_x, double v_y, double s, double d);

	int m_id;
	double m_x; // global map x coordinate
	double m_y; // global map y coordinate
	double m_vx; // m/s
	double m_vy; // m/s
	double m_v; // m/s
	double m_s; // s Frenet coordinate
	double m_d; // d Frenet coordinate
	double m_yaw; // radians
};

#endif