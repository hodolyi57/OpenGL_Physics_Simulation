#include "viewer.h"

#include<Windows.h>
#include<MMSystem.h>
#pragma comment(lib,"winmm.lib") 

extern mass_cloth* cloth;
Simulator simulator;
float m_Zoom = 180.0f;
float m_Translate[3] = { 0.0f, 0.0f, 0.0f };
float m_Rotate[3] = { 0.0f,0.0f,0.0f };
int m_Mouse_Coord[2] = { 0,0 };
int m_Mouse_Event[3] = { 0,0,0 };
// 외력 줄지 말지
int interactionMode = 0;

void Viewer::Initialize()
{
	m_start = false;
	simulator.Initialize();
	simulator.Lighting();
}

void Viewer::Update()
{
	if (m_start == true)
	{
		simulator.Update();
	}
}

// Transformation 설정 및 Light Position 설정
// Simulator의 Render() 함수 호출
void Viewer::Render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(25.0, -5.0, 40.0);
	glTranslatef(0.0, 0.0, -m_Zoom);
	glTranslatef(m_Translate[0], m_Translate[1], 0.0);
	glPushMatrix();
	glRotatef(m_Rotate[0], 1.0, 0.0, 0.0);
	glRotatef(m_Rotate[1], 0.0, 1.0, 0.0);
	glRotatef(-70.0, 0.0, 1.0, 0.0);
	//glBegin(GL_QUADS);
	//glColor3f(1.0, 1.0, 1.0);

	//for (int x = 0; x < 128; x++) {
	//	for (int y = 0; y < 128; y++) {
	//		glNormal3f(0.0, 1.0, 0.0);
	//		// (-250 ~ 250, ground.y, -250~ 250)
	//		glVertex3f(-250.0f + 250.0f / 64 * x, -15.0, -250.0f + 250.0f / 64 * y);
	//		glVertex3f(-250.0f + 250.0f / 64 * (x + 1), -15.0, -250.0f + 250.0f / 64 * y);
	//		glVertex3f(-250.0f + 250.0f / 64 * (x + 1), -15.0, -250.0f + 250.0f / 64 * (y + 1));
	//		glVertex3f(-250.0f + 250.0f / 64 * x, -15.0, -250.0f + 250.0f / 64 * (y + 1));
	//	}
	//}
	//glEnd();
	simulator.Render();
	glPopMatrix();
	float light_pos[] = { 150.0,150.0,0.0,1.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glutSwapBuffers();
	glutPostRedisplay();
}

// Mouse button Click시 호출
// mouse_event: Active Button Type
// state: Up or Down
// x,y: Cursor Position
void Viewer::Mouse(int mouse_event, int state, int x, int y)
{
	m_Mouse_Coord[0] = x;
	m_Mouse_Coord[1] = y;
	switch (mouse_event)
	{
	case GLUT_LEFT_BUTTON:
		m_Mouse_Event[0] = ((GLUT_DOWN == state) ? 1 : 0);
		break;
	case GLUT_MIDDLE_BUTTON:
		m_Mouse_Event[1] = ((GLUT_DOWN == state) ? 1 : 0);
		break;
	case GLUT_RIGHT_BUTTON:
		m_Mouse_Event[2] = ((GLUT_DOWN == state) ? 1 : 0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

// Mouse drag 시, Cursor의 x,y 위치 좌표와 Mouse()에서 저장한 값으로 위치 변화랑 계산
// 각 눌린 Button 별로 Viewer 제어
// Left Button: Rotation, Middle button: Translation, Right button: Zoom
// Mouse를 통한 외력 적용 구현 필요
void Viewer::Motion(int x, int y)
{
	int diffx = x - m_Mouse_Coord[0];
	int diffy = y - m_Mouse_Coord[1];

	m_Mouse_Coord[0] = x;
	m_Mouse_Coord[1] = y;

	if (m_Mouse_Event[0])
	{
		if (!interactionMode)
		{
			m_Rotate[0] += (float)0.1 * diffy;
			m_Rotate[1] += (float)0.1 * diffx;
		}
		else if (interactionMode)
		{
			// Basic Implements 5. User Interaction
			for (int i = 0; i < cloth->nodes.size(); i++)
			{
				cloth->nodes[i]->velocity = cloth->nodes[i]->velocity + dvec3(0.01*diffx, 0.01*diffy, 0);
			}
			//printf("nono\n");

		}
	}
	else if (m_Mouse_Event[1])
	{
		m_Translate[0] += (float)0.1 * diffx;
		m_Translate[1] += (float)0.1 * diffy;
	}
	else if (m_Mouse_Event[2])
	{
		m_Zoom += (float)0.1 * diffy;
	}
	glutPostRedisplay();
}

// Key 별 Event 제어
// 1,2,3: Cloth Rendering 변경
// ESC(ASCII 27), q, Q: 프로그램 Window 종료
// Space Bar(' '): 시뮬레이션 시작/ 일시정지
// r, R: 시뮬레이션 Reset
// f, F: User Interaction Force 적용 여부
void Viewer::Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '1':
		PlaySound(TEXT("1.wav"), NULL, SND_ASYNC | SND_ALIAS);
		cloth->drawMode = mass_cloth::DRAW_MASS_NODES;
		break;
	case '2':
		cloth->drawMode = mass_cloth::DRAW_SPRINGS;
		break;
	case '3':
		PlaySound(TEXT("2.wav"), NULL, SND_ASYNC | SND_ALIAS);
		cloth->drawMode = mass_cloth::DRAW_FACES;
		break;
	case 27:
	case 'q':
	case 'Q':
		exit(0);
		break;
	case ' ':
		//PlaySound(TEXT("bgm.wav"), NULL, SND_FILENAME | SND_ASYNC | SND_LOOP | SND_NODEFAULT)

		m_start = !m_start;
		break;
	case 'r':
	case 'R':
		simulator.Initialize();
		break;
	case 'f':
	case 'F':
		interactionMode = !interactionMode;
		break;
	}
	glutPostRedisplay();
}

// 윈도우 크기 변화 Event에 따른 좌표계 재설정
void Viewer::Reshape(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (float)w / h, 0.1, 500);
	glViewport(0, 0, w, h);
}