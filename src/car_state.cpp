#include <math.h>

#include "car_state.h"

using namespace std;

CarState::CarState(int id, double x, double y, double v_x, double v_y, double s, double d):
	m_id(id),
	m_x(x),
	m_y(y),
	m_vx(v_x),
	m_vy(v_y),
	m_s(s),
	m_d(d)
{
	m_v = sqrt(pow(v_x, 2) + pow(v_y, 2));
	m_yaw = atan2(m_vy, m_vx);
}
