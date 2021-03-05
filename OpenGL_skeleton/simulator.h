#include <vector>
#include <glm/glm.hpp>
#include <glut.h>
#include <gl/gl.h>
#include <gl/glu.h>
using namespace std;
using namespace glm;

class Simulator {

public:
	void Initialize();
	void Update();
	void Lighting();
	void Render();
	// Cloth�� Collision�� �� Ground ����
	void DrawGround();
	bool LoadMeshFromFile(const char* texFile);

	double		timeStep;
	dvec3		ground;

	GLuint mTexture;

};

class Node
{
public:
	Node(dvec3 v);

	void draw();
	// Node�� �ް� �ִ� Force�� ���ο� �ܷ� ����
	void add_force(dvec3 additional_force);
	/* ���� ���� �ƴ� ��, F=ma ������ �̿��� Position Update
	���� ���� ��, Position�� ���� ������� ����
	������ �ٸ� ��ġ�ؼ� ��� ���� (�߰� ���� ����) */
	void integrate(double dt);

	dvec3		force;  // ��尡 �ް� �ִ� ��¥��
	dvec3		position;  // ����� position
	dvec3		velocity;  // ����� �ӵ�
	dvec3		normal;  // ������ face�� ��� normal
	double		RADIUS;  // ������
	double		x, y, z;  // �ʱ� ��ġ
	double		mass;  // ����
	bool		isFixed;  // ���� �� = ���� ������� Position ����, �̵� �� = ����� ���� Position�� Update
};

class mass_spring
{
public:
	mass_spring(Node* a, Node* b);
	void draw();
	// Hooke's law�� ���� ���� Spring Force�� Damping Force�� ����� ��, 
	dvec3 get_force();
	// Spring���� ���� �� Node1�� Node2�� �� ����
	void internal_force(double dt);

	Node* node1;
	Node* node2;
	double		spring_coef;
	double		damping_coef;
	int			type;  // 0 = structural, 1 = shear, 2 = bending
};

class mass_cloth
{
public:
	vector<Node*> nodes;
	vector<mass_spring*> spring;
	vector<Node*> faces;
	vector<dvec3> face_normals;
	GLuint mTexture;

	int			size_x, size_y, size_z;  // �� �� ��� ����
	double		dx, dy, dz;  // ��� �� �Ÿ�
	double		structural_coef;  // Ÿ�� �� Hooke's Law ������ ��� 
	double		shear_coef;
	double		bending_coef;
	int			iteration_n;  // Force ���� �� Integration �ݺ� Ƚ�� 
	int			drawMode;  // Cloth Rendering Mode
	bool		interaction;
	enum { DRAW_MASS_NODES = 0, DRAW_SPRINGS = 1, DRAW_FACES = 2 };

	// Node�� Spring, Face ���� �� ����
	void init();

	// Cloth�� ���� ��ü �� ���
	void compute_force(double dt, dvec3 gravity);

	// Cloth�� ��ü Node�� ���� integrate ����
	void integrate(double dt);

	// Cloth�� Collision�� �˻��ϰ� Response�� �ִ� Function
	// ���� ��Ȳ�� �°� �����ؾ� �� Ex) Cloth - Ground, Cloth - Sphere, Cloth - Mesh (�߰� ���� ����)
	void collision_response(dvec3 ground);

	// Cloth Rendering�� ���� Normal �� ���
	// Initialization �ܰ迡�� ������ Face�� Ȱ���Ͽ�, ������ Face Normal�� ���ϴ� �ڵ� ���� �ʿ�
	void computeNormal();

	// Cloth Simulation ��� Rendering
	// 3���� Mode�� �׸��� ����
	// Node�� Draw, Spring�� Draw, Face Draw
	void draw();

	// face draw
	void faceDraw();

	bool LoadMeshFromFile(const char* texFile);
};

class particle
{
public:
	const int MIN_INIT_VELOCITY = 10;
	const int MAX_INIT_VELOCITY = 100;
	const float LENGTH = 40;
	const float RADIUS = 1;
	double mass;
	double lifetime;
	int check;
	dvec3 velocity;
	dvec3 position;

	void init();
	// Random function
	float rand_float();
	void Movement(double t, dvec3 force);
	void Collision_Boundary();


};

class particle_system
{
public:
	vector<particle> particles;
	
	dvec3 gravity_point;

	void init(int n);
	// Function to set gravity point
	void set_gravity(dvec3 gravity);
	// For all particles ���� force ����
	// Function to advance state of particle system by time t
	void Movement(float time);
	void Collision();
	void pdCollision();
	void prCollision();
	void draw();
};

class rigidbody
{
public:
	double mass;
	double radius;
	dvec3 velocity;
	dvec3 position;

	void init();
	void Movement(double time, dvec3 gravity);
	void draw();
};


