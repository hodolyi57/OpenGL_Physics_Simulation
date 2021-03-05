#include "simulator.h"
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define NUMBER_OF_PARTICLES 1000


particle_system ParticleSystem;
rigidbody rigid;
mass_cloth* cloth;

////////////////////////////////////////////
// Node
////////////////////////////////////////////
Node::Node(dvec3 v)
{
	position = v;
	force = dvec3(0, 0, 0);
	velocity = dvec3(0, 0, 0);
	normal = dvec3(0, 0, 0);

	x = v.x;
	y = v.y;
	z = v.z;
	mass = 10.0;
	RADIUS = 1.0;
	isFixed = false;
}

void Node::draw()
{
	glPointSize(1.0f);
	glBegin(GL_POINTS);
	glColor3f(0.0, 1.0, 0.0);
	//glVertex3f(0.0, 10.0, 0.0);
	glVertex3f(this->position.x, this->position.y, this->position.z);
	glEnd();
}

void Node::add_force(dvec3 additional_force)
{
	force += additional_force;
}

void Node::integrate(double dt)
{
	if (!isFixed)
	{
		// Basic Implements 2-2. Integration
		dvec3 x_new, v_new, a;
		a = force / mass;
		v_new = velocity + a * dt;
		x_new = position + velocity * dt;  // vt를 써야되나 아니면 vt+1을 써야되나?
		//printf("old:%f,%f,%f new:%f,%f,%f\n", position.x, position.y, position.z, x_new.x, x_new.y, x_new.z);
		velocity = v_new;
		position = x_new;
		//printf("%f,%f,%f\n", this->position.x, this->position.y, this->position.z);
	}
	// initialize force
	force.x = force.y = force.z = 0.0;
}

////////////////////////////////////////////
// mass_spring
////////////////////////////////////////////
mass_spring::mass_spring(Node* a, Node* b)
{
	node1 = a;
	node2 = b;
	type = 0;
	spring_coef = 0;

	double dx = b->x - a->x;
	double dy = b->y - a->y;
	double dz = b->z - a->z;

	damping_coef = 0;

	// Structural springs
	if ((dx == 1 && dz == 0) || dx == 0 && dz == 1) {
		//printf("here\n");
		type = 0;
		spring_coef = cloth->structural_coef;
		damping_coef = 2;
	}
	// Shear springs
	else if ((dx == 1 && dz == 1) || (dx == -1 && dz == 1)) {
		//printf("here\n");
		type = 1;
		spring_coef = cloth->shear_coef;
		damping_coef = 4;
	}
	// Bending springs
	else if ((dx == 2 && dz == 0) || (dx == 0 && dz == 2)) {
		type = 2;
		spring_coef = cloth->bending_coef;
		damping_coef = 5;
	}
}

void mass_spring::draw()
{
	glBegin(GL_LINES);
	// structural
	if (type == 0)
		glColor3f(1.0, 0.0, 0.0);
	// shear
	else if (type == 1)
		glColor3f(0.0, 1.0, 0.0);
	// bending
	else if (type == 2)
		glColor3f(0.0, 0.0, 1.0);
	dvec3 p1 = this->node1->position;
	dvec3 p2 = this->node2->position;

	glVertex3f(p1.x, p1.y, p1.z);
	glVertex3f(p2.x, p2.y, p2.z);

	glEnd();
}

dvec3 mass_spring::get_force()
{
	dvec3 f = dvec3(0.0, 0.0, 0.0);
	dvec3 displacement = node2->position - node1->position;
	dvec3 normalized = normalize(displacement);
	double st = 1.05;
	double sh = 1.50;
	double be = 1.05;
	// 스프링 최대 길이 설정
	// structural
	if (type == 0 && length(displacement) > st * cloth->dx) {
		displacement = st * cloth->dx * normalized;
	}
	// shear
	if (type == 1 && length(displacement) > sh * sqrt(pow(cloth->dx, 2) + pow(cloth->dx, 2))) {
		displacement = sh * sqrt(pow(cloth->dx, 2) + pow(cloth->dx, 2)) * normalized;
	}
	// bending
	if (type == 2 && length(displacement) > be * 2 * cloth->dx) {
		displacement = be * 2 * cloth->dx * normalized;
	}
	node2->position = node1->position + displacement;
	dvec3 vrel = node2->velocity - node1->velocity;

	double distance = length(displacement);

	// structural
	if (this->type == 0) {
		f = normalized * (spring_coef * (distance - cloth->dx) + damping_coef * dot(vrel, normalized));
	}
	// shear
	else if (this->type == 1) {
		f = normalized * (spring_coef * (distance - sqrt(pow(cloth->dx, 2) + pow(cloth->dx, 2))) + damping_coef * dot(vrel, normalized));
	}
	// bending
	else if (this->type == 2) {
		f = normalized * (spring_coef * (distance - 2 * cloth->dx) + damping_coef * dot(vrel, normalized));
	}
	if (length(f) > 400)
	{
		f = 0.001 * normalize(f);
	}
	return f;
}

void mass_spring::internal_force(double dt)
{
	// dt를 어디 써야할까?

	// Basic Implements 2-1. Compute Spring Force
	/*add hook_force and damping force*/
	dvec3 force = get_force();

	// 탄성력 방향 주의
	node1->add_force(force);
	node2->add_force(-force);

}

////////////////////////////////////////////
// mass_cloth
////////////////////////////////////////////
void mass_cloth::init()
{
	LoadMeshFromFile("flag.png");
	// Basic Implements 1. Init Nodes and Shear and Structural Springs
	// Additional Implements 1. Init Bending Spring
	interaction = false;
	// cloth의 노드 초기화
	nodes.clear();
	for (int i = 0; i < cloth->size_x; i++) {
		for (int j = 0; j < cloth->size_y; j++) {
			for (int k = 0; k < cloth->size_z; k++) {
				Node* xp = new Node(dvec3(i * cloth->dx, 10 + k * cloth->dz, j * cloth->dy));
				if (i < 10)
					xp->isFixed = true;
				nodes.push_back(xp);
			}
		}
	}


	// Initialize 된 Node를 Spring으로 연결 (A Node와 B Node를 연결하는 Spring 생성)
	// mass_spring* sp = new mass_spring(p[Node_Index_A], p[Node_Index_B]);
	spring.clear();
	faces.clear();
	for (int i = 0; i < cloth->size_x; i++) {
		for (int j = 0; j < cloth->size_y; j++) {
			for (int k = 0; k < cloth->size_z; k++) {
				// 기준 index
				int idx = i * cloth->size_y * cloth->size_z + j * cloth->size_z + k;
				// structural spring
				if (j < cloth->size_y - 1) {
					mass_spring* st = new mass_spring(nodes[idx], nodes[idx + cloth->size_z]);
					spring.push_back(st);
				}
				if (i < cloth->size_x - 1) {
					mass_spring* st = new mass_spring(nodes[idx], nodes[idx + cloth->size_y * cloth->size_z]);
					spring.push_back(st);
				}
				// shear spring
				if (i < cloth->size_x - 1 && j < cloth->size_y - 1) {
					mass_spring* sh = new mass_spring(nodes[idx], nodes[idx + cloth->size_y * cloth->size_z + cloth->size_z]);
					spring.push_back(sh);
				}
				if (i > 0 && j < cloth->size_y - 1) {
					mass_spring* sh = new mass_spring(nodes[idx], nodes[idx - cloth->size_y * cloth->size_z + cloth->size_z]);
					spring.push_back(sh);
				}
				// bending spring
				if (j < cloth->size_y - 2) {
					mass_spring* be = new mass_spring(nodes[idx], nodes[idx + 2 * cloth->size_z]);
					spring.push_back(be);
				}
				if (i < cloth->size_x - 2) {
					mass_spring* be = new mass_spring(nodes[idx], nodes[idx + 2 * cloth->size_y * cloth->size_z]);
					spring.push_back(be);
				}
				// Cloth Rendering을 위한 Face 생성
				if (i < cloth->size_x - 1 && j < cloth->size_y - 1) {
					// face 1
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx + cloth->size_y * cloth->size_z + cloth->size_z]);
					faces.push_back(nodes[idx + cloth->size_y * cloth->size_z]);
					// face 2
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx + cloth->size_z]);
					faces.push_back(nodes[idx + cloth->size_y * cloth->size_z + cloth->size_z]);
				}
				// 왼쪽 아래 모서리는 face2만
				else if (i == 0 && j == cloth->size_y - 1) {
					// face 2
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx + cloth->size_y * cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_z]);
					// 동일한 face 2 한번 더 넣기
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx + cloth->size_y * cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_z]);
				}
				// 오른쪽 위 모서리는 face1만
				else if (i == cloth->size_x - 1 && j == 0) {
					// face 1
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z]);
					faces.push_back(nodes[idx + cloth->size_z]);
					// 동일한 face 1 한번 더 넣기
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z]);
					faces.push_back(nodes[idx + cloth->size_z]);
				}
				// 오른쪽 아래 모서리는 face1, face2
				else if (i == cloth->size_x - 1 && j == cloth->size_y - 1) {
					// face 1
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z - cloth->size_z]);
					// face 2
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z - cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z]);
				}
				// ~ 제외한 맨 밑의 줄은 왼쪽 face1, 오른쪽 face2
				else if (j == cloth->size_y - 1) {
					// face 1
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z - cloth->size_z]);
					// face 2
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx + cloth->size_y * cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_z]);
				}
				// ~ 제외한 맨 오른쪽 줄은 위쪽 face2, 아래쪽 face1
				else if (i == cloth->size_x - 1) {
					// face 2
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z - cloth->size_z]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z]);
					// face 1
					faces.push_back(nodes[idx]);
					faces.push_back(nodes[idx - cloth->size_y * cloth->size_z]);
					faces.push_back(nodes[idx + cloth->size_z]);
				}

			}
		}
	}

}

void mass_cloth::compute_force(double dt, dvec3 gravity)
{
	// Cloth 전체 Node의 Gravity force 적용
	for (int i = 0; i < nodes.size(); i++)
	{
		nodes[i]->add_force(gravity * nodes[i]->mass);
	}
	// Cloth 전체 Spring의 Internal force 계산 및 적용
	for (int i = 0; i < spring.size(); i++)
	{
		spring[i]->internal_force(dt);
	}

}

void mass_cloth::collision_response(dvec3 ground)
{
	// Basic Implements 4. Collision Check with ground
	// Additional Implements 2. Collision Check with Sphere
	// Additional Implements 3. Collision with Mesh Object

	// ground 기준 점 p
	dvec3 p = dvec3(0.0, -15.0, 0.0);
	dvec3 normal_ground = normalize(-ground);
	//double r = dot(p, normal_ground);
	for (int i = 0; i < nodes.size(); i++)
	{
		// position check
		double check1 = dot(nodes[i]->position - p, normal_ground);
		// velocity check
		double check2 = dot(normal_ground, nodes[i]->velocity);
		if ((check1 > -0.05 && check1 < -0.0001) && check2 < 0) {
			// Collision response
			//dvec3 vn = (dot(normal_ground, nodes[i]->velocity) / length(normal_ground)) * normal_ground;
			dvec3 vn = dot(normal_ground, nodes[i]->velocity) * normal_ground;
			dvec3 vt = nodes[i]->velocity - vn;

			if (length(vn) > 3.0)
				nodes[i]->velocity = vt - 0.4 * vn;
			else if (length(vn) > 2.0)
				nodes[i]->velocity = vt - 0.2 * vn;
			else if (length(vn) > 1.0)
				nodes[i]->velocity = vt - 0.1 * vn;
			else
				nodes[i]->velocity = vt;
		}
	}

}

void mass_cloth::integrate(double dt)
{
	// integrate Nodes
	for (int i = 0; i < nodes.size(); i++)
	{
		nodes[i]->integrate(dt);
	}
}

void mass_cloth::computeNormal()
{
	face_normals.clear();
	// Basic Implements 3-2. Compute Vertex Normal
	for (int i = 0; i < faces.size(); i += 3) {
		dvec3 v1 = faces[i + 1]->position - faces[i]->position;
		dvec3 v2 = faces[i + 2]->position - faces[i]->position;
		dvec3 face_normal = cross(v1, v2);
		face_normals.push_back(face_normal);
	}
	for (int i = 0; i < nodes.size(); i++) {
		// 맨 왼쪽 위
		if (i == 0)
			nodes[i]->normal = normalize(face_normals[2 * i] + face_normals[2 * i + 1]);
		// 맨 오른쪽 위
		else if (i == (cloth->size_x - 1) * cloth->size_y * cloth->size_z) {
			nodes[i]->normal = normalize(face_normals[2 * i] + face_normals[2 * i + 1]);
		}
		// ~ 제외한 맨 위의 줄
		else if (i % (cloth->size_y * cloth->size_z) == 0) {
			nodes[i]->normal = normalize(face_normals[2 * i - cloth->size_y * cloth->size_z] + face_normals[2 * i] + face_normals[2 * i + 1]);
		}
		// 맨 왼쪽 아래
		else if (i == cloth->size_y * cloth->size_z - 1)
			nodes[i]->normal = normalize(face_normals[2 * i] + face_normals[2 * i + 1]);
		// ~ 제외한 맨 왼쪽 줄
		else if (i < cloth->size_y * cloth->size_z - 1)
			nodes[i]->normal = normalize(face_normals[2 * (i - 1) + 1] + face_normals[2 * i] + face_normals[2 * i + 1]);
		// 맨 오른쪽 아래
		else if (i == cloth->size_x * cloth->size_y * cloth->size_z - 1)
			nodes[i]->normal = normalize(face_normals[2 * i] + face_normals[2 * i + 1]);
		// ~ 제외한 맨 밑의 줄
		else if (i % (cloth->size_y * cloth->size_z) == cloth->size_y * cloth->size_z - 1)
			nodes[i]->normal = normalize(face_normals[2 * (i - cloth->size_y * cloth->size_z) + 1] + face_normals[2 * i] + face_normals[2 * i + 1]);
		// ~ 제외한 맨 오른쪽 줄
		else if (i > (cloth->size_x - 1) * cloth->size_y * cloth->size_z)
			nodes[i]->normal = normalize(face_normals[2 * (i - cloth->size_z) + 1] + face_normals[2 * i] + face_normals[2 * i + 1]);
		// 나머지는 주위 6개 face들로 계산
		else {
			nodes[i]->normal = normalize(face_normals[2 * (i - cloth->size_y * cloth->size_z)] + face_normals[2 * (i - cloth->size_z) + 1] +
				face_normals[2 * (i - cloth->size_y * cloth->size_z - cloth->size_z)] + face_normals[2 * (i - cloth->size_y * cloth->size_z - cloth->size_z) + 1] + face_normals[2 * i] + face_normals[2 * i + 1]);
		}

	}
	//for (int i = 0; i < nodes.size(); i++) {
	//	nodes[i]->normal = normalize(face_normals[2 * i] + face_normals[2 * i + 1]);
	//}
	//if(i==2000)
	//printf("%f,%f,%f\n", nodes[i]->normal.x, nodes[i]->normal.y, nodes[i]->normal.z);
	//for (int i = 0; i < cloth->size_x; i++) {
	//	for (int j = 0; j < cloth->size_y; j++) {
	//		for (int k = 0; k < cloth->size_z; k++) {
	//			// 기준 index
	//			int idx = i * cloth->size_y * cloth->size_z + j * cloth->size_z + k;
	//			// 왼쪽 아래 모퉁이는 face2 만
	//			if (i == 0 && j == cloth->size_y - 1) {
	//				nodes[idx]->normal = normalize(face_normals[2 * idx + 1]);
	//			}
	//			// 
	//			else if (i < cloth->size_x - 1 || j == cloth->size_y - 1) {

	//			}
	//		}
	//	}
	//}
}

bool mass_cloth::LoadMeshFromFile(const char* texFile)
{
	glGenTextures(1, &mTexture);
	FILE* fp = NULL;
	if (fopen_s(&fp, texFile, "rb")) {
		printf("ERROR : No %s. \n fail to bind %d\n", texFile, mTexture);
		return false;
	}

	int width, height, channel;
	unsigned char* image = stbi_load_from_file(fp, &width, &height, &channel, 4);
	fclose(fp);

	//bind
	glBindTexture(GL_TEXTURE_2D, mTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	stbi_image_free(image);

	return true;
}

void mass_cloth::faceDraw()
{
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, mTexture);
	glBegin(GL_TRIANGLES);
	//glColor3f(0.0, 1.0, 0.0);
	//printf("%d\n", faces.size());
	float u = 1.0 / (cloth->size_x-10);
	float v = 1.0 / cloth->size_y;
	for (int i = 0; i < faces.size(); i += 3) {
		for (int j = 0; j < 3; j++) {
			Node* n = faces[i + j];
			glNormal3f(n->normal.x, n->normal.y, n->normal.z);
			if(n->x<10)
				glTexCoord2f(0.0, 0.0);
			else
				glTexCoord2f(n->z * v, (n->x-10) * u);
			glVertex3f(n->position.x, n->position.y, n->position.z);
		}
	}
	glEnd();
	glDisable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, 0);
}

void mass_cloth::draw()
{
	switch (drawMode)
	{
	case DRAW_MASS_NODES:
		glDisable(GL_LIGHTING);
		for (int i = 0; i < nodes.size(); i++)
			nodes[i]->draw();
		glEnable(GL_LIGHTING);
		break;
	case DRAW_SPRINGS:
		glDisable(GL_LIGHTING);
		for (int i = 0; i < spring.size(); i++)
			spring[i]->draw();
		glEnable(GL_LIGHTING);
		break;
	case DRAW_FACES:
		// Basic Implements 3-3. Draw Call for Cloth
		glEnable(GL_LIGHTING); //glDisable(GL_LIGHTING);
		faceDraw();
		//glEnable(GL_LIGHTING);
		break;
		break;
	default:
		break;
	}
}
////////////////////////////////////////////
// Particle
////////////////////////////////////////////
void particle::init() 
{
	mass = 1.0;
	//velocity = dvec3(MIN_INIT_VELOCITY - rand() %
	//	(MAX_INIT_VELOCITY - MIN_INIT_VELOCITY), MIN_INIT_VELOCITY - rand() %
	//	(MAX_INIT_VELOCITY - MIN_INIT_VELOCITY), MIN_INIT_VELOCITY - rand() %
	//	(MAX_INIT_VELOCITY - MIN_INIT_VELOCITY));
	if (check == 0) {
		velocity = dvec3(0, 1.0 * (MIN_INIT_VELOCITY + rand() %
			(MAX_INIT_VELOCITY)-MIN_INIT_VELOCITY), 0);
		//position = dvec3((1.0 - 2.0 * rand_float()) * (LENGTH - 10.0), 50.0+(1.0 - rand_float()) * LENGTH - 10.0, (1.0 - 2.0 * rand_float()) * (LENGTH - 10.0));
		position = dvec3((1.0 - 2.0 * rand_float()), -1.5 + (1.0 - rand_float()) * LENGTH - 10.0, -10.0+(1.0 - 2.0 * rand_float()));
	}
	else if (check == 1) {
		velocity = dvec3(0, 1.0 * (MIN_INIT_VELOCITY + rand() %
			(MAX_INIT_VELOCITY)-MIN_INIT_VELOCITY), 0);
		//position = dvec3((1.0 - 2.0 * rand_float()) * (LENGTH - 10.0), 50.0+(1.0 - rand_float()) * LENGTH - 10.0, (1.0 - 2.0 * rand_float()) * (LENGTH - 10.0));
		position = dvec3((1.0 - 2.0 * rand_float()), -1.5 + (1.0 - rand_float()) * LENGTH - 10.0, 70.0+(1.0 - 2.0 * rand_float()));
	}
}

float particle::rand_float()
{
	float value = rand() / float(RAND_MAX);
	return value;
}

void particle::Movement(double t, dvec3 force)
{
	// calculating acceleration
	dvec3 acc = force / mass;
	// calculating velocity
	velocity = velocity + acc * t;
	//changing position
	position = position + velocity * t;
	// 오른쪽 수명 확인
	if (lifetime > 10.0 && check == 0)
	{
		velocity = dvec3(0, 1.0 * (MIN_INIT_VELOCITY + rand() %
			(MAX_INIT_VELOCITY) - MIN_INIT_VELOCITY), 0);
		position = dvec3((1.0 - 2.0 * rand_float()), -1.5 + (1.0 - rand_float()) * LENGTH - 10.0, -10.0+ (1.0 - 2.0 * rand_float()));
		lifetime = 0.0;
	}
	// 왼쪽 수명 확인
	else if (lifetime > 10.0 && check == 1)
	{
		velocity = dvec3(0, 1.0 * (MIN_INIT_VELOCITY + rand() %
			(MAX_INIT_VELOCITY)-MIN_INIT_VELOCITY), 0);
		position = dvec3((1.0 - 2.0 * rand_float()), -1.5 + (1.0 - rand_float()) * LENGTH - 10.0, 70.0 + (1.0 - 2.0 * rand_float()));
		lifetime = 0.0;
	}
}

void particle::Collision_Boundary()
{
	double COR = 0.3;
	double tmp = 0.0;
	if (check == 0)
		tmp = -10.0;
	else
		tmp = 70.0;
	if (position.x <= -LENGTH + RADIUS && velocity.x < 0)
	{
		position.x = -LENGTH + RADIUS;
		velocity.x *= (-1.0) * COR;
	}
	if (position.x >= LENGTH - RADIUS && velocity.x > 0)
	{
		position.x = LENGTH - RADIUS;
		velocity.x *= (-1.0) * COR;
	}
	if (position.y <= -15.0 + RADIUS && velocity.y < 0)
	{
		position.y = -15.0 + RADIUS;
		velocity.y *= (-1.0) * COR;
	}
	//if (position.y >= 15.0 - RADIUS && velocity.y > 0)
	//{
	//	position.y = 15.0 - RADIUS;
	//	velocity.y *= (-1.0) * COR;
	//}
	if (position.z <= -LENGTH + RADIUS +tmp && velocity.z < 0)
	{
		position.z = -LENGTH + RADIUS +tmp;
		velocity.z *= (-1.0) * COR;
	}
	if (position.z >= LENGTH - RADIUS +tmp&& velocity.z > 0)
	{
		position.z = LENGTH - RADIUS +tmp;
		velocity.z *= (-1.0) * COR;
	}
}

////////////////////////////////////////////
// Particle Simulator
////////////////////////////////////////////
void particle_system::init(int n)  // n = NUMBER_OF_PARTICLES
{
	particles.clear();
	for (int i = 0; i < n; i++)
	{
		particle temp;  // Create a particle
		particles.push_back(temp);  // Push a particle in particles
	}
	for (int i = 0; i < particles.size(); i++)
	{
		if (i < particles.size() / 2.0) {
			particles[i].lifetime = i * 0.02;
			particles[i].check = 0;
			particles[i].init();
		}
		else if (i >= particles.size() / 2.0) {
			particles[i].lifetime = (i- particles.size() / 2.0) * 0.02;
			particles[i].check = 1;
			particles[i].init();
		}
	}
}

void particle_system::set_gravity(dvec3 gravity) 
{
	gravity_point = gravity;
}

void particle_system::Movement(float time)
{
	for (int i = 0; i < particles.size();i++)
	{
		dvec3 force = gravity_point;
		particles[i].Movement(time, force);
		particles[i].lifetime += 0.1;
	}
}

void particle_system::Collision()
{
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i].Collision_Boundary();
		for (int j = 0; j < particles.size(); j++) {
			if (j != i) {
				dvec3 pos_ij = particles[j].position - particles[i].position;
				dvec3 pos_ji = particles[i].position - particles[j].position;
				double distance = length(pos_ij);
				double collideDist = particles[i].RADIUS + particles[j].RADIUS;
				if (distance <= collideDist)
				{
					// computing collison normal
					dvec3 normal1 = (pos_ij) / distance;
					dvec3 normal2 = (pos_ij) / distance;
					dvec3 Vn1 = normal1 * dot(normal1,particles[i].velocity);
					dvec3 Vn2 = normal2 * dot(normal2,particles[j].velocity);
					// computing collision tangent
					dvec3 Vt1 = particles[i].velocity - Vn1;
					dvec3 Vt2 = particles[j].velocity - Vn2;
					
					dvec3 relVel = particles[j].velocity - particles[i].velocity;
					if (dot(relVel, pos_ij) < 0.0)  // solving the trapped problem
					{
						particles[i].velocity = (Vn2 * 2.0 * particles[j].mass + Vn1 * (particles[i].mass - particles[j].mass)) / (particles[i].mass + particles[j].mass) + Vt1;
						particles[j].velocity = (Vn1 * 2.0 * particles[i].mass + Vn2 * (particles[j].mass - particles[i].mass)) / (particles[i].mass + particles[j].mass) + Vt2;
					}
				}
			}
		}
	}
}
// Particle - deformable Collision
void particle_system::pdCollision()
{
	for (int i = 0; i < particles.size(); i++)
	{
		//particles[i].Collision_Boundary();
		for (int j = 0; j < cloth->nodes.size(); j++) {
			dvec3 pos_ij = cloth->nodes[j]->position - particles[i].position;
			dvec3 pos_ji = particles[i].position - cloth->nodes[j]->position;
			double distance = length(pos_ij);
			double collideDist = particles[i].RADIUS + cloth->nodes[j]->RADIUS;
			if (distance <= collideDist)
			{
				// computing collison normal
				dvec3 normal1 = (pos_ij) / distance;
				dvec3 normal2 = (pos_ij) / distance;
				dvec3 Vn1 = normal1 * dot(normal1, particles[i].velocity);
				dvec3 Vn2 = normal2 * dot(normal2, cloth->nodes[j]->velocity);
				// computing collision tangent
				dvec3 Vt1 = particles[i].velocity - Vn1;
				dvec3 Vt2 = cloth->nodes[j]->velocity - Vn2;

				dvec3 relVel = cloth->nodes[j]->velocity - particles[i].velocity;
				if (dot(relVel, pos_ij) < 0.0)  // solving the trapped problem
				{
					particles[i].velocity = (Vn2 * 2.0 * cloth->nodes[j]->mass + Vn1 * (particles[i].mass - cloth->nodes[j]->mass)) / (particles[i].mass + cloth->nodes[j]->mass) + Vt1;
					cloth->nodes[j]->velocity = (Vn1 * 2.0 * particles[i].mass + Vn2 * (cloth->nodes[j]->mass - particles[i].mass)) / (particles[i].mass + cloth->nodes[j]->mass) + Vt2;
				}
			}
		}
	}
}

// Particle - Rigid Collision
void particle_system::prCollision()
{
	for (int j = 0; j < cloth->nodes.size(); j++) {
		dvec3 pos_ij = cloth->nodes[j]->position - rigid.position;
		dvec3 pos_ji = rigid.position - cloth->nodes[j]->position;
		double distance = length(pos_ij);
		double collideDist = rigid.radius + cloth->nodes[j]->RADIUS;
		if (distance <= collideDist)
		{
			// computing collison normal
			dvec3 normal1 = (pos_ij) / distance;
			dvec3 normal2 = (pos_ij) / distance;
			dvec3 Vn1 = normal1 * dot(normal1, rigid.velocity);
			dvec3 Vn2 = normal2 * dot(normal2, cloth->nodes[j]->velocity);
			// computing collision tangent
			dvec3 Vt1 = rigid.velocity - Vn1;
			dvec3 Vt2 = cloth->nodes[j]->velocity - Vn2;

			dvec3 relVel = cloth->nodes[j]->velocity - rigid.velocity;
			if (dot(relVel, pos_ij) < 0.0)  // solving the trapped problem
			{
				rigid.velocity = (Vn2 * 2.0 * cloth->nodes[j]->mass + Vn1 * (rigid.mass - cloth->nodes[j]->mass)) / (rigid.mass + cloth->nodes[j]->mass) + Vt1;
				cloth->nodes[j]->velocity = (Vn1 * 2.0 * rigid.mass + Vn2 * (cloth->nodes[j]->mass - rigid.mass)) / (rigid.mass + cloth->nodes[j]->mass) + Vt2;
			}
		}
	}
}
void particle_system::draw()
{
	glEnable(GL_POINT_SMOOTH);
	for (int i = 0; i < particles.size(); i++)
	{
		dvec3 pos = particles[i].position;
		dvec3 pos2 = pos + normalize(particles[i].velocity);
		dvec3 color = dvec3(((i % 5) & 0x06) + 2.5, (i % 5) & 0x01, 0.0);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(color.x, color.y, color.z,1.0 - particles[i].lifetime/10.0);
		glLineWidth(3.0);
		glBegin(GL_LINES);
		glNormal3f(1.0, 0.0, 0.0);
		// 오른쪽
		if (particles[i].check == 0)
		{
			//glNormal3f(-pos2.x, -pos2.y, -pos.z);
			glNormal3f(pos.x, pos.y, pos.z);
			glVertex3f(pos.x, pos.y, pos.z);
			//glNormal3f(0.0, -1.0, -1.0);
			glNormal3f(0.0, 0.0, 1.0);
			glVertex3f(pos2.x, pos2.y, pos2.z);
		}
		else {
			glNormal3f(-pos2.x, -pos2.y, -pos.z);
			//glNormal3f(pos.x, pos.y, pos.z);
			glVertex3f(pos.x, pos.y, pos.z);
			glNormal3f(0.0, -1.0, -1.0);
			//glNormal3f(0.0, 0.0, 1.0);
			glVertex3f(pos2.x, pos2.y, pos2.z);
		}

		glEnd();
	}
}

////////////////////////////////////////////
// Rigid Body
////////////////////////////////////////////
void rigidbody::init()
{
	mass = 30000.0;
	radius = 10.0;
	velocity = dvec3(50.0, 0.0, 0.0);
	position = dvec3(-20.0, -6.0, 25.0);
}

void rigidbody::Movement(double time, dvec3 force)
{
	// calculating acceleration
	dvec3 acc = force / mass;
	// calculating velocity
	velocity = velocity + acc * time;
	//changing position
	position = position + velocity * time;
	if (position.y < -6.0)
		position.y = -6.0;
}

void rigidbody::draw()
{
	glTranslatef(position.x, position.y, position.z);
	glColor3f(0.7, 0.7, 0.4);
	glutSolidSphere(radius-0.5, 50, 50);
	glEnd();
}
////////////////////////////////////////////
// Simulator
////////////////////////////////////////////
void Simulator::Initialize()
{
	LoadMeshFromFile("money.png");
	timeStep = 0.01;  // simulation time
	ground = dvec3(0.0, -15.0, 0.0);  // ground 위치 및 노멀 벡터
	
	// cloth
	cloth = new mass_cloth();

	cloth = cloth;
	cloth->dx = 1;
	cloth->dy = 1;
	cloth->dz = 1;
	cloth->size_x = 50;
	cloth->size_y = 60;
	cloth->size_z = 1;
	cloth->structural_coef = 2;
	cloth->shear_coef = 5;
	cloth->bending_coef = 10;
	cloth->iteration_n = 20;
	cloth->drawMode = 0;
	cloth->init();

	// particle
	ParticleSystem.init(NUMBER_OF_PARTICLES);
	ParticleSystem.set_gravity(dvec3(0.0, -9.8, 0.0));

	// rigid
	rigid.init();

}

void Simulator::Update()
{
	dvec3 gravity(0.0, -9.8 / cloth->iteration_n, 0.0);  // Gravity 설정
	//dvec3 gravity(0.0, 0.0, 0.0);
	for (int i = 0; i < cloth->iteration_n; i++)
	{
		cloth->compute_force(timeStep, gravity);  // Cloth에 대한 전체 힘 계산 
		cloth->integrate(timeStep);
		cloth->collision_response(ground);  // Collision 검사 후 Response 적용
	}
	// 리지드 처리
	if(rigid.position.y - ground.y>rigid.radius)
		rigid.Movement(timeStep, gravity);
	else
		rigid.Movement(timeStep, dvec3(0.0,0.0,0.0));
	// 파티클 처리
	ParticleSystem.Movement(timeStep);
	ParticleSystem.Collision();
	ParticleSystem.pdCollision();
	ParticleSystem.prCollision();
	cloth->computeNormal();  // Face, Vertex Normal 계산
}

void Simulator::Lighting()
{
	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	float light_pos[] = { 150.0, 150.0, 0.0, 1.0 };
	float light_dir[] = { -1.0, -1.0, 0.0, 0.0 };
	float light_ambient[] = { 0.3, 0.3, 0.3, 1.0 };
	float light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	float light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	float frontColor[] = { 0.8, 0.8, 0.8, 1.0 };
	float matShininess = 20.0;
	float noMat[] = { 0.0, 0.0, 0.0, 1.0 };

	float matSpec[] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT, GL_EMISSION, noMat);
	glMaterialfv(GL_FRONT, GL_SPECULAR, matSpec);
	glMaterialfv(GL_FRONT, GL_AMBIENT, frontColor);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
	glMaterialf(GL_FRONT, GL_SHININESS, matShininess);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 80.0f);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 80.0f);
	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_dir);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light_dir);


}

bool Simulator::LoadMeshFromFile(const char* texFile)
{
	glGenTextures(1, &mTexture);
	FILE* fp = NULL;
	if (fopen_s(&fp, texFile, "rb")) {
		printf("ERROR : No %s. \n fail to bind %d\n", texFile, mTexture);
		return false;
	}

	int width, height, channel;
	unsigned char* image = stbi_load_from_file(fp, &width, &height, &channel, 4);
	fclose(fp);

	//bind
	glBindTexture(GL_TEXTURE_2D, mTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	stbi_image_free(image);

	return true;
}

void Simulator::Render()
{
	DrawGround();
	cloth->draw();
	ParticleSystem.draw();
	rigid.draw();
	glPopMatrix();
}

void Simulator::DrawGround()
{

	glBindTexture(GL_TEXTURE_2D, mTexture);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	// 기둥 그리기
	glNormal3f(0.0, 0.0, -1.0);
		glVertex3f(0.0, 9.8, 0.0);
		glVertex3f(3.0, 9.8, 0.0);
		glVertex3f(3.0, -16.0, 0.0);
		glVertex3f(0.0, -16.0, 0.0);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	glNormal3f(1.0, 0.0, 0.0);
		glVertex3f(3.0, 9.8, 0.0);
		glVertex3f(3.0, 9.8, 2.0);
		glVertex3f(3.0, -16.0, 2.0);
		glVertex3f(3.0, -16.0, 0.0);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	glNormal3f(0.0, 0.0, 1.0);
		glVertex3f(0.0, 9.8, 2.0);
		glVertex3f(3.0, 9.8, 2.0);
		glVertex3f(3.0, -16.0, 2.0);
		glVertex3f(0.0, -16.0, 2.0);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	glNormal3f(-1.0, 0.0, 0.0);
		glVertex3f(0.0, 9.8, 0.0);
		glVertex3f(0.0, 9.8, 2.0);
		glVertex3f(0.0, -16.0, 2.0);
		glVertex3f(0.0, -16.0, 0.0);
	glEnd();

	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	// 기둥 그리기
	glNormal3f(0.0, 0.0, -1.0);
		glVertex3f(0.0, 9.8, 0.0 + 57.0);
		glVertex3f(3.0, 9.8, 0.0 + 57.0);
		glVertex3f(3.0, -16.0, 0.0 + 57.0);
		glVertex3f(0.0, -16.0, 0.0 + 57.0);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	glNormal3f(1.0, 0.0, 0.0);
		glVertex3f(3.0, 9.8, 0.0 + 57.0);
		glVertex3f(3.0, 9.8, 2.0 + 57.0);
		glVertex3f(3.0, -16.0, 2.0 + 57.0);
		glVertex3f(3.0, -16.0, 0.0 + 57.0);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	glNormal3f(0.0, 0.0, 1.0);
		glVertex3f(0.0, 9.8, 2.0 + 57.0);
		glVertex3f(3.0, 9.8, 2.0 + 57.0);
		glVertex3f(3.0, -16.0, 2.0 + 57.0);
		glVertex3f(0.0, -16.0, 2.0 + 57.0);
	glBegin(GL_QUADS);
	glColor3f(1.0, 1.0, 0.3);
	glNormal3f(-1.0, 0.0, 0.0);
		glVertex3f(0.0, 9.8, 0.0 + 57.0);
		glVertex3f(0.0, 9.8, 2.0 + 57.0);
		glVertex3f(0.0, -16.0, 2.0 + 57.0);
		glVertex3f(0.0, -16.0, 0.0 + 57.0);
	glEnd();
	glEnable(GL_TEXTURE_2D);
	//glDisable(GL_LIGHTING);
	//glDisable(GL_LIGHT0);
	//glDisable(GL_COLOR_MATERIAL);
	glBegin(GL_QUADS);
	//glColor3f(0.7, 0.7, 0.7);

	float u = 1.0 / (128);
	float v = 1.0 / 128;
	for (int x = 0; x < 128; x++) {
		for (int y = 0; y < 128; y++) {
			glNormal3f(0.0, 1.0, 0.0);
			// (-250 ~ 250, ground.y, -250~ 250)
			glTexCoord2f(1-y * u, x * v);
			glVertex3f(-250.0f + 250.0f / 64 * x, ground.y - cloth->dy, -250.0f + 250.0f / 64 * y);
			glTexCoord2f(1-y * u,(x + 1) * v);
			glVertex3f(-250.0f + 250.0f / 64 * (x + 1), ground.y - cloth->dy, -250.0f + 250.0f / 64 * y);
			glTexCoord2f(1-(y + 1) * u,(x + 1) * v);
			glVertex3f(-250.0f + 250.0f / 64 * (x + 1), ground.y - cloth->dy, -250.0f + 250.0f / 64 * (y + 1));
			glTexCoord2f(1-(y + 1) * u,x * v);
			glVertex3f(-250.0f + 250.0f / 64 * x, ground.y - cloth->dy, -250.0f + 250.0f / 64 * (y + 1));
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	glDisable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, 0);
}

