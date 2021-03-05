#include "simulator.h"
#include <glut.h>
#include <gl/gl.h>
#include <gl/glu.h>
class Viewer {

public:
	void Initialize();
	void Update();
	void Render();
	void Mouse(int mouse_event, int state, int x, int y);
	void Motion(int x, int y);
	void Keyboard(unsigned char key, int x, int y);
	void Reshape(int w, int h);

	bool	m_start;
};