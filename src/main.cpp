#include "gl.h"
#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <cstdint>

#include <vecmath.h>
#include <nanogui/nanogui.h>

#include "starter2_util.h"
#include "camera.h"
#include "vertexrecorder.h"
#include "skeletalmodel.h"
#include "curve.h"

using namespace std;
// Note: using namespace nanogui not possible due to naming conflicts
namespace ng = ::nanogui;

namespace
{
// Constants
//const int njoints = 18;
//const string jointnames[njoints] = { "root", "chest", "waist", "neck",
//                                 "right hip", "right leg", "right knee", "right foot",
//                                 "left hip", "left leg", "left knee", "left foot",
//                                 "right collarbone", "right shoulder", "right elbow", "left collarbone", "left shoulder", "left elbow" };

const int njoints = 5;
const string jointnames[njoints] = { "root","hip","knee","ankle","foot" };

// Global variables here.
GLFWwindow* window;
ng::Screen *screen;
Vector3f g_jointangles[njoints];

// This assignment uses a useful camera implementation
Camera camera;
SkeletalModel* skeleton;

// most curves are drawn with constant color, and no lighting
GLuint program_color;

// These are state variables for the UI
bool gMousePressed = false;
bool gDrawSkeleton = true;
bool gDrawAxisAlways = false;

//stuff to draw a spline

struct Recorders {
    VertexRecorder curve;
    VertexRecorder curveFrames;
    VertexRecorder surface;
    VertexRecorder surfaceNormals;
};
Recorders* recorders;

vector<vector<Vector3f> > gCtrlPoints;
vector<Curve> gCurves;

// Declarations of functions whose implementations occur later.
void drawAxis(void);
void drawCurve(void);
void drawPoints(void);

static void keyCallback(GLFWwindow* window, int key,
    int scancode, int action, int mods)
{
    if (action == GLFW_RELEASE) { // only handle PRESS and REPEAT
        return;
    }

    // Special keys (arrows, CTRL, ...) are documented
    // here: http://www.glfw.org/docs/latest/group__keys.html
    switch (key) {
    case GLFW_KEY_ESCAPE: // Escape key
        exit(0);
        break;
    case ' ':
    {
        Matrix4f eye = Matrix4f::identity();
        camera.SetRotation(eye);
        camera.SetDistance(1.5);
        camera.SetCenter(Vector3f(-0.5, -0.5, -0.5));
        break;
    }
    case 'S':
        gDrawSkeleton = !gDrawSkeleton;
        break;
    case 'A':
        gDrawAxisAlways = !gDrawAxisAlways;
        break;
    default:
        cout << "Unhandled key press " << key << "." << endl;
    }
}

static void mouseCallback(GLFWwindow* window, int button, int action, int mods) {
    double xd, yd;
    glfwGetCursorPos(window, &xd, &yd);
    int x = (int)xd;
    int y = (int)yd;

    int lstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    int mstate = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    if (lstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::LEFT, x, y);
    }
    else if (rstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::RIGHT, x, y);
    }
    else if (mstate == GLFW_PRESS) {
        gMousePressed = true;
        camera.MouseClick(Camera::MIDDLE, x, y);
    }
    else {
        gMousePressed = true;
        camera.MouseRelease(x, y);
        gMousePressed = false;
    }
}

static void motionCallback(GLFWwindow* window, double x, double y)
{
    if (!gMousePressed) {
        return;
    }
    camera.MouseDrag((int)x, (int)y);
}

void setViewport(GLFWwindow* window)
{
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);

    camera.SetDimensions(w, h);
    camera.SetViewport(0, 0, w, h);
    camera.ApplyViewport();
}

void drawAxis()
{
    glUseProgram(program_color);
    Matrix4f M = Matrix4f::translation(camera.GetCenter()).inverse();
    camera.SetUniforms(program_color, M);

    const Vector3f DKRED(1.0f, 0.5f, 0.5f);
    const Vector3f DKGREEN(0.5f, 1.0f, 0.5f);
    const Vector3f DKBLUE(0.5f, 0.5f, 1.0f);
    const Vector3f GREY(0.5f, 0.5f, 0.5f);
	const Vector3f W(1.0f, 1.0f, 1.0f);

    const Vector3f ORGN(0, 0, 0);
    const Vector3f AXISX(5, 0, 0);
    const Vector3f AXISY(0, 5, 0);
    const Vector3f AXISZ(0, 0, 5);

	const Vector3f AXIStick1(-.2, 0, 0);
	const Vector3f AXIStick2(-.2, -.25, 0);

	const Vector3f AXIStick3(.2, 0, 0);
	const Vector3f AXIStick4(.2, -.25, 0);

    VertexRecorder recorder;
    recorder.record_poscolor(ORGN, DKRED);
    recorder.record_poscolor(AXISX, DKRED);
    recorder.record_poscolor(ORGN, DKGREEN);
    recorder.record_poscolor(AXISY, DKGREEN);
    recorder.record_poscolor(ORGN, DKBLUE);
    recorder.record_poscolor(AXISZ, DKBLUE);
	recorder.record_poscolor(AXIStick1, W);
	recorder.record_poscolor(AXIStick2, W);
	recorder.record_poscolor(AXIStick3, W);
	recorder.record_poscolor(AXIStick4, W);

    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISX, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISY, GREY);
    recorder.record_poscolor(ORGN, GREY);
    recorder.record_poscolor(-AXISZ, GREY);

    glLineWidth(3);
    recorder.draw(GL_LINES);

}

void drawPoints()
{
    // Setup for point drawing
    VertexRecorder recorder;
    glUseProgram(program_color);
    Matrix4f M = Matrix4f::translation(camera.GetCenter()).inverse();
    camera.SetUniforms(program_color, M);

    const Vector3f COLOR(1, 1, 0.0f);

    for (int i = 0; i < (int)gCtrlPoints.size(); i++) {
        // There are relatively few control points, so we can
        // get away with recording this for each frame.
        // VertexRecorder recorder;
        for (int j = 0; j < (int)gCtrlPoints[i].size(); j++) {
            Vector3f shifted = gCtrlPoints[i][j] - Vector3f(.5, .5, .5);
            // shifted.print();
            if (0 < j  & j < (int)gCtrlPoints[i].size()-1){
                recorder.record_poscolor(shifted, COLOR);
                recorder.record_poscolor(shifted, COLOR);
            }
            else {
                recorder.record_poscolor(shifted, COLOR);
            }
        }
    }
        // recorder.draw(GL_LINE_STRIP);
    glLineWidth(3);
    recorder.draw(GL_LINES);
}

void drawCurve()
{
    VertexRecorder recorder;
    glUseProgram(program_color);
    Matrix4f M = Matrix4f::translation(camera.GetCenter()).inverse();
    camera.SetUniforms(program_color, M);

    const Vector3f COLOR(1, 1, 0.0f);

    for (int i = 0; i < (int)gCurves.size(); i++) {
        // There are relatively few control points, so we can
        // get away with recording this for each frame.
        // VertexRecorder recorder;
        for (int j = 0; j < (int)gCurves[i].size(); j++) {
            Vector3f shifted = gCurves[i][j].V - Vector3f(.5, .5, .5);
            // shifted.print();
            if (0 < j  & j < (int)gCurves[i].size()-1){
                recorder.record_poscolor(shifted, COLOR);
                recorder.record_poscolor(shifted, COLOR);
            }
            else {
                recorder.record_poscolor(shifted, COLOR);
            }
        }
    }
        // recorder.draw(GL_LINE_STRIP);
    glLineWidth(3);
    recorder.draw(GL_LINES);
}

void recordVertices() {
    // For complex models, it is too expensive to specify
    // all vertices each frame. We can make things more efficient
    // by recording the vertices on application startup, and then
    // drawing from the pre-recorded data structure.

    recorders = new Recorders();

    // CURVES
    for (int i = 0; i < (int)gCurves.size(); i++) {
        recordCurve(gCurves[i], &recorders->curve);
    }

    // for (int i = 0; i < (int)gCurves.size(); i++) {
    //     recordCurveFrames(gCurves[i], &recorders->curveFrames, gLineLen);
    // }

    // SURFACE
    // for (int i = 0; i < (int)gSurfaces.size(); i++) {
    //     recordSurface(gSurfaces[i], &recorders->surface);
    // }
    // for (int i = 0; i < (int)gSurfaces.size(); i++) {
    //     recordNormals(gSurfaces[i], &recorders->surfaceNormals, gLineLen);
    // }
}

void freeVertices() {
    delete recorders;
    recorders = nullptr;
}

void initRendering()
{
    // Clear to black
    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void updateMesh()
{
    // Update the bone to world transforms for SSD.
    skeleton->updateCurrentJointToWorldTransforms();
    // update the mesh given the new skeleton
    skeleton->updateMesh();
}

/*
   initializes a simple NanoGUI-based UI
   must call freeGUI() when done.

   This function implements a simple GUI with three sliders
   for each joint. You won't have to touch it, but feel free
   to add your own features.

   The GUI is drawn in the same window as the main application.
   Any mouse and keyboard events, we first send to the GUI. If the
   GUI didn't handle the event, we forward it to the event handler
   functions above.

   Once initialized, the GUI is drawn in the main loop of the application
   The GUI is drawn in the same window as the main application.
   Any mouse and keyboard events, we first send to the GUI. If the
   GUI didn't handle the event, we forward it to the event handler
   functions above.

   Once initialized, the GUI is drawn in the main loop of the
   application.
*/
void initGUI(GLFWwindow* glfwwin) {
    // Create a nanogui screen and pass the glfw pointer to initialize

    const int FONTSZ = 14;
    const int ROWH = 18;

    screen = new ng::Screen();
    screen->initialize(glfwwin, false);


    ng::Window* window = nullptr;
    ng::Widget* animator = nullptr;
    for (int i = 0; i < njoints; ++i) {
        if (i == 0 || i == 8) {
            window = new ng::Window(screen, i == 0 ? "Animator 1" : "Animator 2");
            window->setPosition(ng::Vector2i(i == 0 ? 10 : 800, 10));
            window->setLayout(new ng::BoxLayout(ng::Orientation::Vertical));
            window->setFixedHeight(i == 0 ? 800 : 960);

            // Scrollpanel is broken. Slider drag mouse events not transformed properly
            // ng::VScrollPanel* vspanel = new ng::VScrollPanel(window);
            // vspanel->setLayout(new ng::BoxLayout(ng::Orientation::Vertical));
            // vspanel->setFixedHeight(600);

            animator = new ng::Widget(window);
            animator->setLayout(new ng::BoxLayout(ng::Orientation::Vertical));
        }
        if (i == 0) {
            ng::Button* btn = new ng::Button(animator, "Take Screenshot");
            btn->setCallback([glfwwin]() {
                screencapture(glfwwin);
            });
        }

        ng::Widget *jointpanel = new ng::Widget(animator);
        jointpanel->setLayout(new ng::BoxLayout(ng::Orientation::Vertical, ng::Alignment::Minimum, 2, 0));

        ng::Label* label = new ng::Label(jointpanel, jointnames[i]);
        label->setFontSize(FONTSZ);

        for (int dim = 0; dim < 3; ++dim) {

            ng::Widget *panel = new ng::Widget(jointpanel);
            panel->setLayout(new ng::BoxLayout(ng::Orientation::Horizontal, ng::Alignment::Middle, 3, 10));

            char buff[80];
            switch (dim) {
            case 0: sprintf(buff, "%s", "x"); break;
            case 1: sprintf(buff, "%s", "y"); break;
            case 2: sprintf(buff, "%s", "z"); break;
            }

            ng::Label* label = new ng::Label(panel, buff);
            label->setFontSize(FONTSZ);
            label->setFixedSize(ng::Vector2i(10, ROWH));

            ng::Slider *slider = new ng::Slider(panel);
            slider->setFixedWidth(160);
            slider->setFixedHeight(ROWH);
            slider->setValue(0.5);
            slider->setFinalCallback([&](float value) {
                //cout << "Final slider value: " << (int)(value * 100) << endl;
            });

            ng::TextBox *textBox = new ng::TextBox(panel);
            textBox->setFixedSize(ng::Vector2i(40, ROWH));
            slider->setCallback([textBox, i, dim](float value) {
                char buff[80];
                g_jointangles[i][dim] = (value - 0.5f) * 2 * (float)M_PI;
                sprintf(buff, "%.2f", g_jointangles[i][dim]);
                textBox->setValue(buff);

                if (skeleton) {
                    // skeleton->setIK(())
                    // update animation
                    // skeleton->setJointTransform(i, g_jointangles[i].x(), g_jointangles[i].y(), g_jointangles[i].z());
                    updateMesh();
                }
            });

            //textBox->setFixedSize(ng::Vector2i(40, ROWH));
            textBox->setFontSize(FONTSZ);
            textBox->setAlignment(ng::TextBox::Alignment::Right);

            // update text box and global vars.
            slider->notifyCallback();
        }
    }

    screen->performLayout();

    // nanoGUI wants to handle events.
    // We forward GLFW events to nanoGUI first. If nanoGUI didn't handle
    // the event, we pass it to the handler routine.
    glfwSetCursorPosCallback(glfwwin,
        [](GLFWwindow* window, double x, double y) {
        if (gMousePressed) {
            // sticky mouse gestures
            motionCallback(window, x, y);
            return;
        }
        if (screen->cursorPosCallbackEvent(x, y)) {
            return;
        }
        motionCallback(window, x, y);
    }
    );

    glfwSetMouseButtonCallback(glfwwin,
        [](GLFWwindow* window, int button, int action, int modifiers) {
        if (screen->mouseButtonCallbackEvent(button, action, modifiers)) {
            return;
        }
        mouseCallback(window, button, action, modifiers);
    }
    );

    glfwSetKeyCallback(glfwwin,
        [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (screen->keyCallbackEvent(key, scancode, action, mods)) {
            return;
        }
        keyCallback(window, key, scancode, action, mods);
    }
    );

    glfwSetCharCallback(glfwwin,
        [](GLFWwindow *, unsigned int codepoint) {
        screen->charCallbackEvent(codepoint);
    }
    );

    glfwSetDropCallback(glfwwin,
        [](GLFWwindow *, int count, const char **filenames) {
        screen->dropCallbackEvent(count, filenames);
    }
    );

    glfwSetScrollCallback(glfwwin,
        [](GLFWwindow *, double x, double y) {
        screen->scrollCallbackEvent(x, y);
    }
    );

    glfwSetFramebufferSizeCallback(glfwwin,
        [](GLFWwindow *, int width, int height) {
        screen->resizeCallbackEvent(width, height);
    }
    );
}
void freeGUI() {
    delete screen;
    screen = nullptr;
}

void loadSkeleton(const std::string& basepath) {
    skeleton = new SkeletalModel();
    string skelfile = basepath + ".skel";
    string objfile = basepath + ".obj";
    string attachfile = basepath + ".attach";
    skeleton->load(skelfile.c_str(), objfile.c_str(), attachfile.c_str());
}
void freeSkeleton() {
    delete skeleton;
    skeleton = nullptr;
}
}


int main(int argc, char** argv)
{
    ///initialize curve stuff
    Vector3f A(.3, .25, .5);
    Vector3f B(.4, .20, .5);
    Vector3f C(.5, .20, .5);
    Vector3f D(.7, .25, .5);
    vector<Vector3f> curve1;
    curve1.push_back(A);
    curve1.push_back(B);
    curve1.push_back(C);
    curve1.push_back(D);
    gCtrlPoints.push_back(curve1);
    int steps = 20;
    gCurves.push_back(evalBezier(curve1, steps));
    // cout << gCurves[0].size() << "size" <<endl;
    /////////////
    if (argc < 2)
    {
        cout << "Usage: " << argv[0] << " PREFIX" << endl;
        cout << "For example, if you're trying to load data/Model1.skel, data/Model1.obj, and data/Model1.attach, run with: " << argv[0] << " data/Model1" << endl;
        return -1;
    }
    std::string basepath = argv[1];
    window = createOpenGLWindow(1024, 1024, "Assignment 2");

    initGUI(window);
    initRendering();

    // The program object controls the programmable parts
    // of OpenGL. All OpenGL programs define a vertex shader
    // and a fragment shader.
    program_color = compileProgram(c_vertexshader, c_fragmentshader_color);
    if (!program_color) {
        printf("Cannot compile program\n");
        return -1;
    }

    camera.SetPerspective(50);
    camera.SetDistance(1.5);
    camera.SetCenter(Vector3f(-0.5, -0.5, -0.5));

    loadSkeleton(basepath);

    // Main Loop
    while (!glfwWindowShouldClose(window)) {
        // Clear the rendering window
        // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //
        // // Draw nanogui
        // // screen->drawContents();
        // // screen->drawWidgets();
        // glEnable(GL_DEPTH_TEST);

//        setViewport(window);

        if (gDrawAxisAlways || gMousePressed) {
            drawAxis();
        }

        //draw the spline things
        // drawCurve();

        //inverse kinematics//
       // float thetaz = .01;
       // cout<<"atleastgothere"<<endl;
		float d_ik = .00001;
        Vector3f goal(.3, .25, .5);
        Vector3f og(.7, .25, .5);
        Vector3f g = og;
		Vector3f pos = og;
        bool isog = true;
        float eps = .01;
        uint16_t count = 0;
		while (1) {
			if ((g - pos).abs() < eps) {
				if (isog) {
					g = goal;
					isog = false;
				}
				else {
					g = og;
					isog = true;
				}
			}
			//compute J
			Matrix3f J = skeleton->getJacobian();
            //compute J+
            Matrix2f J_temp = J.getSubmatrix2x2(0,0);
            J_temp = (J_temp.transposed()*J_temp).inverse()*J_temp.transposed();
            //initialize all zeros
            Matrix3f J_p = Matrix3f();
            J_p.setSubmatrix2x2(0, 0, J_temp);

            float rate = .001;
			Vector3f e = (g - pos).normalized()*rate;
            Vector3f thetas = J_p*e;
			//cout << "hip theta: " << thetas.x() << endl;
			//cout << "knee theta: " << thetas.y() << endl;
            //only rotate about the x axis...
			pos = skeleton->getPos();
			skeleton->setJointTransform(1, 0, 0, thetas.x());
			skeleton->setJointTransform(2, 0, 0, thetas.y());
			//cout << "count "<<count <<endl;
			//while (count < 100){
			//	count++;
			//	//J.print();
			//
			//	// Make back buffer visible
			//	cout << "count " << (g - pos).abs() << endl;
			//}
			count++;
			if(count > 10){
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glEnable(GL_DEPTH_TEST);
				setViewport(window);
				if (gDrawAxisAlways || gMousePressed) {
					drawAxis();
                    drawPoints();
                    drawCurve();
				}
				skeleton->draw(camera, gDrawSkeleton);
				glfwSwapBuffers(window);
                count = 0;
                //thetaz += .01;
            }
			glfwPollEvents();
        }
        // Check if any input happened during the last frame
        glfwPollEvents();
    }
    freeSkeleton();

    // All OpenGL resource that are created with
    // glGen* or glCreate* must be freed.
    freeGUI();
    glDeleteProgram(program_color);

    glfwTerminate(); // destroy the window
    return 0;
}
