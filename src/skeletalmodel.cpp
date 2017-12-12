#include "skeletalmodel.h"
#include <cassert>

#include "starter2_util.h"
#include "vertexrecorder.h"

using namespace std;

SkeletalModel::SkeletalModel() {
    program = compileProgram(c_vertexshader, c_fragmentshader_light);
    if (!program) {
        printf("Cannot compile program\n");
        assert(false);
    }
}

SkeletalModel::~SkeletalModel() {
    // destructor will release memory when SkeletalModel is deleted
    while (m_joints.size()) {
        delete m_joints.back();
        m_joints.pop_back();
    }
    glDeleteProgram(program);
}

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
    loadSkeleton(skeletonFile);

    m_mesh.load(meshFile);
    m_mesh.loadAttachments(attachmentsFile, (int)m_joints.size());
    computeBindWorldToJointTransforms();
    updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(const Camera& camera, bool skeletonVisible)
{
    // draw() gets called whenever a redraw is required
    // (after an update() occurs, when the camera moves, the window is resized, etc)

    m_matrixStack.clear();

    glUseProgram(program);
    updateShadingUniforms();
    if (skeletonVisible)
    {
        drawJoints(camera);
        drawSkeleton(camera);
    }
    else
    {
        // Tell the mesh to draw itself.
        // Since we transform mesh vertices on the CPU,
        // There is no need to set a Model matrix as uniform
        camera.SetUniforms(program, Matrix4f::identity());
        m_mesh.draw();
    }
    glUseProgram(0);
}

void SkeletalModel::updateShadingUniforms() {
    // UPDATE MATERIAL UNIFORMS
    GLfloat diffColor[] = { 0.4f, 0.4f, 0.4f, 1 };
    GLfloat specColor[] = { 0.9f, 0.9f, 0.9f, 1 };
    GLfloat shininess[] = { 50.0f };
    int loc = glGetUniformLocation(program, "diffColor");
    glUniform4fv(loc, 1, diffColor);
    loc = glGetUniformLocation(program, "specColor");
    glUniform4fv(loc, 1, specColor);
    loc = glGetUniformLocation(program, "shininess");
    glUniform1f(loc, shininess[0]);

    // UPDATE LIGHT UNIFORMS
    GLfloat lightPos[] = { 3.0f, 3.0f, 5.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightPos");
    glUniform4fv(loc, 1, lightPos);

    GLfloat lightDiff[] = { 120.0f, 120.0f, 120.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightDiff");
    glUniform4fv(loc, 1, lightDiff);
}

void SkeletalModel::loadSkeleton(const char* filename)
{
    // Load the skeleton from file here.
	ifstream skel(filename);

	//parsing variables
	float x = 0;
	float y = 0;
	float z = 0;
	int parent = 0;
    // cout<<skel<<endl;

	//start parsing
	while (skel >> x) {
		skel >> y;
		skel >> z;
		skel >> parent;
		//cerr << x << "," << y << "," <<z << endl;
		//storage variable
		Joint * jt = new Joint();

		//store transform
		jt->transform = Matrix4f::translation(x, y, z);

		//add the joint to the list of joints
		m_joints.push_back(jt);

		//link children to partents
		if (parent != -1) {
			m_joints[parent]->children.push_back(jt);
		}
		else {
			m_rootJoint = jt;
		}

	}
}

void SkeletalModel::drawJoints(const Camera& camera)
{
    // Draw a sphere at each joint. You will need to add a recursive
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawSphere( 0.025f, 12, 12 )
    // to draw a sphere of reasonable size.
    //
    // You should use your MatrixStack class. A function
    // should push it's changes onto the stack, and
    // use stack.pop() to revert the stack to the original
    // state.

    // this is just for illustration:

    // translate from top of stack, but doesn't push, since that's not
    // implemented yet.
    //Matrix4f M = m_matrixStack.top() * Matrix4f::translation(+0.5f, +0.5f, -0.5f);
    // update transformation uniforms
    //camera.SetUniforms(program, M);
    // draw
    //drawSphere(0.025f, 12, 12);
    // didn't push to stack, so no pop() needed

	//Call the helper function
	//cerr << "start" << endl;
	helpDrawJoints(m_rootJoint, camera);
	//cerr << "end" << endl;
}

void SkeletalModel::helpDrawJoints(Joint * jt,const Camera& camera) {
	//push the current transform and draw a joint
	m_matrixStack.push(jt->transform);
	Matrix4f M = m_matrixStack.top();
	camera.SetUniforms(program, M);
	drawSphere(0.025f, 12, 12);

	//traverse the tree adding joints at each child
	for (int i = 0; i < jt->children.size(); i++) {
		helpDrawJoints(jt->children[i], camera);
	}
	//pop off transforms as tree is traversed
	m_matrixStack.pop();
	//cerr << "draw" << endl;
}

// Vector3f helpFindPos(Joint * jt){
//     m_matrixStack..push(jt->transform);
//     Matrix4f M = m_matrixStack.top();
//     if(jt->children.size() == 0){
//         return jt->currentJointToWorldTransform;
//     }
//     for (int i = 0; i < jt->children.size(); i++) {
// 		return helpDrawJoints(jt->children[i], camera);
// 	}
//     m_matrixStack.pop();
// }

Vector3f SkeletalModel::getPos() {
	return (m_joints[4]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
}

Vector3f SkeletalModel::getPos(int joint) {
	return (m_joints[joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
}

Vector3f SkeletalModel::getLocalPos(int ref_joint, int joint) {
	updateCurrentJointToWorldTransforms();
	return ((m_joints[joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz() - (m_joints[ref_joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz());
}

std::vector<Matrix3f> SkeletalModel::getJacobians() {
	std::vector<Matrix3f> Jacobians;
	
	//relevant jacobians for skeleton
	Matrix3f J_L_Shoulder;
	Matrix3f J_R_Shoulder;
	Matrix3f J_L_Hand;
	Matrix3f J_R_Hand;
	Matrix3f J_L_Hip;
	Matrix3f J_R_Hip;
	Matrix3f J_L_Foot;
	Matrix3f J_R_Foot;

	//relevant points for jacobians
	Vector3f P_R_Shoulder;
	Vector3f P_L_Shoulder;
	Vector3f P_R_Hand;
	Vector3f P_L_Hand;
	Vector3f P_R_Hip;
	Vector3f P_L_Hip;
	Vector3f P_R_Foot;
	Vector3f P_L_Foot;

	//time step
	float dt = .1;
	
	//transforms needed to make jacobians
	Matrix4f R_Shoulder_WS						= m_joints[16]->currentJointToWorldTransform;
	Matrix4f L_Shoulder_WS						= m_joints[13]->currentJointToWorldTransform;
	Matrix4f R_Hip_WS                           = m_joints[9]->currentJointToWorldTransform;
	Matrix4f L_Hip_WS                           = m_joints[5]->currentJointToWorldTransform;

	Matrix4f R_Hand_WS							= m_joints[17]->currentJointToWorldTransform;
	Matrix4f L_Hand_WS							= m_joints[14]->currentJointToWorldTransform;

	Matrix4f R_Foot_WS							= m_joints[11]->currentJointToWorldTransform;
	Matrix4f L_Foot_WS							= m_joints[7]->currentJointToWorldTransform;
	
	//Right Shoulder Jacobian Construction
	P_R_Shoulder                                = (R_Shoulder_WS*Vector4f(0,0,0,1)).xyz();
	Vector3f P_1n = (perturbSystem(15, dt, 16) - P_R_Shoulder) / dt;
	J_R_Shoulder = Matrix3f(P_1n.x(), 0.0f, 0.0f,
							P_1n.y(), 0.0f, 0.0f,
							P_1n.z(), 0.0f, 0.0f);
	Jacobians.push_back(J_R_Shoulder);

	// Left Shoulder Jacobian Construction
	P_L_Shoulder                                = (L_Shoulder_WS*Vector4f(0, 0, 0, 1)).xyz();
	Vector3f P_Ln = (perturbSystem(12, dt, 13) - P_L_Shoulder) / dt;
	J_L_Shoulder = Matrix3f(P_Ln.x(), 0.0f, 0.0f,
							P_Ln.y(), 0.0f, 0.0f,
							P_Ln.z(), 0.0f, 0.0f);
	Jacobians.push_back(J_L_Shoulder);
	
	// Right Hip Jacobian Construction
	P_R_Hip                                     = (R_Hip_WS*Vector4f(0, 0, 0, 1)).xyz();
	P_1n = (perturbSystem(1, dt, 0, 9) - P_R_Hip) / dt;
	Vector3f P_2n = (perturbSystem(2, dt, 0, 9) - P_R_Hip) / dt;
	Vector3f P_3n = (perturbSystem(8, dt, 0, 9) - P_R_Hip) / dt;
	J_R_Hip = Matrix3f(P_1n.x(), P_2n.x(), P_3n.x(),
					   P_1n.y(), P_2n.y(), P_3n.y(),
					   P_1n.z(), P_2n.z(), P_3n.z());

	Jacobians.push_back(J_R_Hip);

	P_L_Hip										= (L_Hip_WS*Vector4f(0, 0, 0, 1)).xyz();
	P_1n = (perturbSystem(1, dt, 0, 5) - P_L_Hip) / dt;
	P_2n = (perturbSystem(2, dt, 0, 5) - P_L_Hip) / dt;
	P_3n = (perturbSystem(4, dt, 0, 5) - P_L_Hip) / dt;
	J_L_Hip = Matrix3f(P_1n.x(), P_2n.x(), P_3n.x(),
				       P_1n.y(), P_2n.y(), P_3n.y(),
					   P_1n.z(), P_2n.z(), P_3n.z());

	Jacobians.push_back(J_L_Hip);
	
	P_R_Hand                                    = (R_Hand_WS*Vector4f(0, 0, 0, 1)).xyz() - (R_Shoulder_WS*Vector4f(0, 0, 0, 1)).xyz();
	P_1n = (perturbSystem(16, dt, 0, 0, 17, 16) - P_R_Hand) / dt;
	J_R_Hand = Matrix3f(P_1n.x(), 0.0f, 0.0f,
						P_1n.y(), 0.0f, 0.0f,
						P_1n.z(), 0.0f, 0.0f);

	Jacobians.push_back(J_R_Hand);

	P_L_Hand									= (L_Hand_WS*Vector4f(0, 0, 0, 1)).xyz() - (L_Shoulder_WS*Vector4f(0, 0, 0, 1)).xyz();
	P_1n = (perturbSystem(13, dt, 0, 0, 14, 13) - P_L_Hand) / dt;
	J_L_Hand = Matrix3f(P_1n.x(), 0.0f, 0.0f,
		P_1n.y(), 0.0f, 0.0f,
		P_1n.z(), 0.0f, 0.0f);

	Jacobians.push_back(J_L_Hand);

	P_R_Foot									= (R_Foot_WS*Vector4f(0, 0, 0, 1)).xyz() - (R_Hip_WS*Vector4f(0, 0, 0, 1)).xyz();
	P_1n = (perturbSystem(9, dt, 0, 0, 11, 9) - P_R_Foot) / dt;
	P_2n = (perturbSystem(10, dt, 0, 0, 11, 9) - P_R_Foot) / dt;
	J_R_Foot = Matrix3f(P_1n.x(), P_2n.x(), 0.0f,
						P_1n.y(), P_2n.y(), 0.0f,
						P_1n.z(), P_2n.z(), 0.0f);

	Jacobians.push_back(J_R_Foot);

	P_L_Foot									= (L_Foot_WS*Vector4f(0, 0, 0, 1)).xyz() - (L_Hip_WS*Vector4f(0, 0, 0, 1)).xyz();
	P_1n = (perturbSystem(5, dt, 0, 0, 7, 5) - P_L_Foot) / dt;
	P_2n = (perturbSystem(6, dt, 0, 0, 7, 5) - P_L_Foot) / dt;
	J_L_Foot = Matrix3f(P_1n.x(), P_2n.x(), 0.0f,
						P_1n.y(), P_2n.y(), 0.0f,
						P_1n.z(), P_2n.z(), 0.0f);

	Jacobians.push_back(J_L_Foot);

	/*Vector3f P_1n = (perturbSystem(1, dt1) - P) / dt1;
	Vector3f P_2n = (perturbSystem(2, dt2) - P) / dt2;

	J = Matrix3f(P_1n.x(), P_2n.x(), 0.0f,
		P_1n.y(), P_2n.y(), 0.0f,
		0.0f, 0.0f, 0.0f);
	return J;*/
	return Jacobians;
}

Matrix3f SkeletalModel::getJacobian(){
    Matrix3f J;
    Vector3f P;
    float dt1 = .1;
    float dt2 = .1;
    // cout<<m_joints.size()<<endl;
    Matrix4f eh = m_joints[4]->currentJointToWorldTransform;

    Vector4f what = eh*Vector4f(0,0,0,1);

    P = (m_joints[4]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
    Vector3f P_1n = (perturbSystem(1, dt1)-P)/dt1;
    Vector3f P_2n = (perturbSystem(2, dt2)-P)/dt2;

    J = Matrix3f( P_1n.x(), P_2n.x(), 0.0f,
                   P_1n.y(), P_2n.y(), 0.0f,
                        0.0f,      0.0f, 0.0f);
    return J;
}

Vector3f SkeletalModel::perturbSystem(int jointIndex, float rZ){
    Matrix4f M = Matrix4f::translation(m_joints[jointIndex]->transform.getCol(3).xyz());
    Matrix4f rotz = Matrix4f::rotateZ(rZ);
	Matrix4f irotz = Matrix4f::rotateZ(-rZ);
    m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*rotz;
    Vector3f P;
	updateCurrentJointToWorldTransforms();
    P = (m_joints[4]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
    m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*irotz;
	updateCurrentJointToWorldTransforms();
	// m_joints[jointIndex]->transform =  rotz;

    return P;
}

Vector3f SkeletalModel::perturbSystem(int jointIndex, float rZ, int this_joint) {
	Matrix4f M = Matrix4f::translation(m_joints[jointIndex]->transform.getCol(3).xyz());
	Matrix4f rotz = Matrix4f::rotateZ(rZ);
	Matrix4f irotz = Matrix4f::rotateZ(-rZ);
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*rotz;
	Vector3f P;
	updateCurrentJointToWorldTransforms();
	P = (m_joints[this_joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*irotz;
	updateCurrentJointToWorldTransforms();
	// m_joints[jointIndex]->transform =  rotz;

	return P;
}

Vector3f SkeletalModel::perturbSystem(int jointIndex, float rY, float rZ, int this_joint) {
	Matrix4f M = Matrix4f::translation(m_joints[jointIndex]->transform.getCol(3).xyz());
	Matrix4f rotz = Matrix4f::rotateY(rY);
	Matrix4f irotz = Matrix4f::rotateY(-rY);
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*rotz;
	Vector3f P;
	updateCurrentJointToWorldTransforms();
	P = (m_joints[this_joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*irotz;
	updateCurrentJointToWorldTransforms();
	// m_joints[jointIndex]->transform =  rotz;

	return P;
}

Vector3f SkeletalModel::perturbSystem(int jointIndex, float rX, float rY, float rZ, int this_joint, int ref_joint) {
	Matrix4f M = Matrix4f::translation(m_joints[jointIndex]->transform.getCol(3).xyz());
	Matrix4f rotz = Matrix4f::rotateX(rX);
	Matrix4f irotz = Matrix4f::rotateX(-rX);
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*rotz;
	Vector3f P;
	updateCurrentJointToWorldTransforms();
	P = (m_joints[this_joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz() - (m_joints[ref_joint]->currentJointToWorldTransform*Vector4f(0, 0, 0, 1)).xyz();
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*irotz;
	updateCurrentJointToWorldTransforms();
	// m_joints[jointIndex]->transform =  rotz;

	return P;
}

void SkeletalModel::drawSkeleton(const Camera& camera)
{
    // Draw cylinders between the joints. You will need to add a recursive
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawCylinder(6, 0.02f, <height>);
    // to draw a cylinder of reasonable diameter.

    // you can use the stack with push/pop like this
    // m_matrixStack.push(Matrix4f::translation(+0.6f, +0.5f, -0.5f))
    // camera.SetUniforms(program, m_matrixStack.top());
    // drawCylinder(6, 0.02f, 0.2f);
    // callChildFunction();
    // m_matrixStack.pop();
	helpDrawBones(m_rootJoint, camera);
}

void SkeletalModel::helpDrawBones(Joint * jt, const Camera& camera) {
	//push the current transform
	m_matrixStack.push(jt->transform);
	Matrix4f M = m_matrixStack.top();

	//traverse the tree adding joints at each child
	for (int i = 0; i < jt->children.size(); i++) {
		//Get length and oreintation information and draw the bone
		Vector3f vecCtoP = jt->children[i]->transform.getCol(3).xyz();
		Vector3f y = Vector3f(0, 1, 0);
		float len = vecCtoP.abs();
		Vector3f rotaxis = Vector3f::cross(y, vecCtoP).normalized();
		float rotang = acos(Vector3f::dot(y,vecCtoP)/(len*y.abs()));
		Matrix4f rotM;
		rotM = Matrix4f::rotation(rotaxis, rotang);
		camera.SetUniforms(program, M*rotM);
		drawCylinder(6, 0.02f, len);
		helpDrawBones(jt->children[i], camera);
	}
	//pop off transforms as tree is traversed
	m_matrixStack.pop();
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
    // Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
	Matrix4f M = Matrix4f::translation(m_joints[jointIndex]->transform.getCol(3).xyz());
	Matrix4f rotx = Matrix4f::rotateX(rX);
	Matrix4f roty = Matrix4f::rotateY(rY);
	Matrix4f rotz = Matrix4f::rotateZ(rZ);
	m_joints[jointIndex]->transform = m_joints[jointIndex]->transform*rotz*roty*rotx;
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
    // 2.3.1. Implement this method to compute a per-joint transform from
    // world-space to joint space in the BIND POSE.
    //
    // Note that this needs to be computed only once since there is only
    // a single bind pose.
    //
    // This method should update each joint's bindWorldToJointTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.
	helpBWTJ(m_rootJoint, m_matrixStack.top().inverse());
}

void SkeletalModel::helpBWTJ(Joint * jt, Matrix4f BWTJ) {
	jt->bindWorldToJointTransform = jt->transform.inverse()*BWTJ;
	for (int i = 0; i < jt->children.size(); i++) {
		helpBWTJ(jt->children[i], jt->transform.inverse()*BWTJ);
	}
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
    // 2.3.2. Implement this method to compute a per-joint transform from
    // joint space to world space in the CURRENT POSE.
    //
    // The current pose is defined by the rotations you've applied to the
    // joints and hence needs to be *updated* every time the joint angles change.
    //
    // This method should update each joint's currentJointToWorldTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.
	helpBJTW(m_rootJoint);
}

void SkeletalModel::helpBJTW(Joint * jt) {
	m_matrixStack.push(jt->transform);
	jt->currentJointToWorldTransform = m_matrixStack.top();
	for (int i = 0; i < jt->children.size(); i++) {
		helpBJTW(jt->children[i]);
	}
	m_matrixStack.pop();
	//cerr << "BJTW" << endl;
	//jt->currentJointToWorldTransform.print();
	//cerr << "BWTJ" << endl;
	//jt->bindWorldToJointTransform.print();
}

void SkeletalModel::updateMesh()
{
    // 2.3.2. This is the core of SSD.
    // Implement this method to update the vertices of the mesh
    // given the current state of the skeleton.
    // You will need both the bind pose world --> joint transforms.
    // and the current joint --> world transforms.
	int num_vertices = m_mesh.bindVertices.size();

	for (unsigned i = 0; i < num_vertices; i++) {
		//attachments of one vertex
		vector<float> attachments = m_mesh.attachments[i];

		//vertex
		Vector4f v = Vector4f(m_mesh.bindVertices[i], 1);
		m_mesh.currentVertices[i] = Vector3f(0, 0, 0);
		for (unsigned j = 0; j < attachments.size(); j++) {
			Vector4f bv = m_joints[j+1]->bindWorldToJointTransform*v;
			Vector4f cv = m_joints[j+1]->currentJointToWorldTransform*bv;
			m_mesh.currentVertices[i] = m_mesh.currentVertices[i] + attachments[j] * cv.xyz();
		}
		//m_mesh.currentVertices[i] = (m_matrixStack.top()*Vector4f(m_mesh.currentVertices[i], 1)).xyz();
	}

}
