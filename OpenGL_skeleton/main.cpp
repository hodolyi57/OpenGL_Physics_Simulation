#include "viewer.h"

Viewer viewer;


void Initialize(void)
{
	viewer.Initialize();
}

void Update(int value)
{
	viewer.Update();

	glutPostRedisplay();
	glutTimerFunc(10, Update, 0);
}

void Render(void)
{
	viewer.Render();
	glFinish();
}

void Reshape(int w, int h)
{
	viewer.Reshape(w, h);
}

void Mouse(int mouse_event, int state, int x, int y)
{
	viewer.Mouse(mouse_event, state, x, y);
}

void Motion(int x, int y)
{
	viewer.Motion(x, y);
}

void Keyboard(unsigned char key, int x, int y)
{
	viewer.Keyboard(key, x, y);
}

int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(400, 100);
	glutInitWindowSize(1024, 800);
	glutCreateWindow("Hojin's Simulation");
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.05f, 0.05f, 0.05f, 0.0f);
	Initialize();
	glutTimerFunc(10, Update, 0);
	glutDisplayFunc(Render);

	glutReshapeFunc(Reshape);
	glutMotionFunc(Motion);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(Mouse);

	glutMainLoop();

	return 0;

}