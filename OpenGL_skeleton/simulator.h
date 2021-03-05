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
	// Cloth와 Collision을 할 Ground 생성
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
	// Node가 받고 있는 Force에 새로운 외력 적용
	void add_force(dvec3 additional_force);
	/* 고정 점이 아닐 때, F=ma 수식을 이용해 Position Update
	고정 점일 때, Position을 새로 계산하지 않음
	안정된 다른 수치해석 기법 적용 (추가 구현 사항) */
	void integrate(double dt);

	dvec3		force;  // 노드가 받고 있는 알짜힘
	dvec3		position;  // 노드의 position
	dvec3		velocity;  // 노드의 속도
	dvec3		normal;  // 인접한 face의 평균 normal
	double		RADIUS;  // 반지름
	double		x, y, z;  // 초기 위치
	double		mass;  // 질량
	bool		isFixed;  // 고정 점 = 힘에 관계없이 Position 고정, 이동 점 = 계산을 통해 Position이 Update
};

class mass_spring
{
public:
	mass_spring(Node* a, Node* b);
	void draw();
	// Hooke's law를 통해 구한 Spring Force와 Damping Force를 계산한 뒤, 
	dvec3 get_force();
	// Spring으로 연결 된 Node1과 Node2에 힘 적용
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

	int			size_x, size_y, size_z;  // 각 축 노드 개수
	double		dx, dy, dz;  // 노드 간 거리
	double		structural_coef;  // 타입 별 Hooke's Law 스프링 상수 
	double		shear_coef;
	double		bending_coef;
	int			iteration_n;  // Force 연산 및 Integration 반복 횟수 
	int			drawMode;  // Cloth Rendering Mode
	bool		interaction;
	enum { DRAW_MASS_NODES = 0, DRAW_SPRINGS = 1, DRAW_FACES = 2 };

	// Node와 Spring, Face 생성 및 설정
	void init();

	// Cloth에 대한 전체 힘 계산
	void compute_force(double dt, dvec3 gravity);

	// Cloth의 전체 Node를 돌며 integrate 실행
	void integrate(double dt);

	// Cloth의 Collision을 검사하고 Response를 주는 Function
	// 각각 상황에 맞게 구현해야 함 Ex) Cloth - Ground, Cloth - Sphere, Cloth - Mesh (추가 구현 사항)
	void collision_response(dvec3 ground);

	// Cloth Rendering을 위한 Normal 값 계산
	// Initialization 단계에서 생성한 Face를 활용하여, 각각의 Face Normal을 구하는 코드 구현 필요
	void computeNormal();

	// Cloth Simulation 결과 Rendering
	// 3가지 Mode로 그리기 가능
	// Node만 Draw, Spring만 Draw, Face Draw
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
	// For all particles 같은 force 적용
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


