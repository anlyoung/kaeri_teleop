#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include <GL/glut.h>
#include <HL/hl.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#include "PhantomROS.h"

/* Haptic device and rendering context handles. */
static HHD ghHD = HD_INVALID_HANDLE;
static HHLRC ghHLRC = 0;

/* Shape id for shape we will render haptically. */
HLuint gMeshShapeId;
OmniState state;

boost::mutex mesh_mutex;

#define CURSOR_SIZE_PIXELS 20
static double gCursorScale;
static GLuint gCursorDisplayList = 0;

/* Function prototypes. */
void *ros_thread_func(void *ptr);

void glutDisplay(void);
void glutReshape(int width, int height);
void glutIdle(void);
void glutMenu(int);

void exitHandler(void);

void initGL();
void initHL();
void initScene();
void drawSceneHaptics();
void drawSceneGraphics();
void drawCursor();
void updateWorkspace();

void HLCALLBACK hlTouchCB(HLenum event, HLuint object, HLenum thread, HLcache *cache, void *userdata);
void HLCALLBACK hlUnTouchCB(HLenum event, HLuint object, HLenum thread, HLcache *cache, void *userdata);
void HLCALLBACK hlMotionCB(HLenum event, HLuint object, HLenum thread, HLcache *cache, void *userdata);

/*******************************************************************************
 Initializes GLUT for displaying a simple haptic scene.
*******************************************************************************/
int main(int argc, char *argv[])
{
    // Init ROS
    ros::init(argc, argv, "omni_haptic_node");
		ros::NodeHandle nh;
    PhantomROS omni_ros(nh, mesh_mutex, &state);

    // ros thread (publish and listen)
    //int publish_rate;
    //omni_ros.n.param(std::string("publish_rate"), publish_rate, 100);
    //ros::Rate loop_rate(publish_rate);

    // Init glut
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(100, 100);
    glutCreateWindow("HelloSphere Example");

    // Set glut callback functions.
    glutDisplayFunc(glutDisplay);
    glutReshapeFunc(glutReshape);
    glutIdleFunc(glutIdle);

    glutCreateMenu(glutMenu);
    glutAddMenuEntry("Quit", 0);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    // Provide a cleanup routine for handling application exit.
    atexit(exitHandler);

    initGL();
    initHL();

    pthread_t ros_thread;
    pthread_create(&ros_thread, NULL, ros_thread_func, (void*) &omni_ros);

    glutMainLoop();

    return 0;
}

void *ros_thread_func(void *ptr) {
    PhantomROS *omni_ros = (PhantomROS *) ptr;
    int publish_rate = 60;
    //omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
    ros::Rate loop_rate(publish_rate);


    while (ros::ok()) {
        omni_ros->publish_omni_state();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return NULL;
}

/*******************************************************************************
 GLUT callback for redrawing the view.
*******************************************************************************/
void glutDisplay()
{
    drawSceneHaptics();

    //drawSceneGraphics();

    glutSwapBuffers();
}

/*******************************************************************************
 GLUT callback for reshaping the window.  This is the main place where the
 viewing and workspace transforms get initialized.
*******************************************************************************/
void glutReshape(int width, int height)
{
    static const double kPI = 3.1415926535897932384626433832795;
    static const double kFovY = 40;

    double nearDist, farDist, aspect;

    glViewport(0, 0, width, height);

    // Compute the viewing parameters based on a fixed fov and viewing
    // a canonical box centered at the origin.

    nearDist = 1.0 / tan((kFovY / 2.0) * kPI / 180.0);
    farDist = nearDist + 2.0;
    aspect = (double) width / height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(kFovY, aspect, nearDist, farDist);

    // Place the camera down the Z axis looking at the origin.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, nearDist + 1.0,
              0, 0, 0,
              0, 1, 0);

    updateWorkspace();
}

/*******************************************************************************
 GLUT callback for idle state.  Use this as an opportunity to request a redraw.
 Checks for HLAPI errors that have occurred since the last idle check.
*******************************************************************************/
void glutIdle()
{
    HLerror error;

    while (HL_ERROR(error = hlGetError()))
    {
        fprintf(stderr, "HL Error: %s\n", error.errorCode);

        if (error.errorCode == HL_DEVICE_ERROR)
        {
            hduPrintError(stderr, &error.errorInfo,
                "Error during haptic rendering\n");
        }
    }

    glutPostRedisplay();
}

/******************************************************************************
 Popup menu handler.
******************************************************************************/
void glutMenu(int ID)
{
    switch(ID) {
        case 0:
            exit(0);
            break;
    }
}

/*******************************************************************************
 Sets up general OpenGL rendering properties: lights, depth buffering, etc.
*******************************************************************************/
void initGL()
{
    static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    static const GLfloat light0_diffuse[] = {0.9f, 0.9f, 0.9f, 0.9f};
    static const GLfloat light0_direction[] = {0.0f, -0.4f, 1.0f, 0.0f};

    // Enable depth buffering for hidden surface removal.
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);

    // Setup other misc features.
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    // Setup lighting model.
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
    glEnable(GL_LIGHT0);
}

/*******************************************************************************
 Initialize the HDAPI.  This involves initing a device configuration, enabling
 forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void HHD_Auto_Calibration() {
    int supportedCalibrationStyles;
    int calibrationStyle;

    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
        calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..");
    }
    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
      do {
        hdUpdateCalibration(calibrationStyle);
        ROS_INFO("Calibrating.. (put stylus in well)");
        if (HD_DEVICE_ERROR(error = hdGetError())) {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            break;
        }
        } while (hdCheckCalibration() != HD_CALIBRATION_OK);
        ROS_INFO("Calibration complete.");
    }
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
      ROS_INFO("Please place the device into the inkwell for calibration.");
    }

    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
}

void initHL()
{
    HDErrorInfo error;

    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "Press any key to exit");
        getchar();
        exit(-1);
    }

    HHD_Auto_Calibration();

    ghHLRC = hlCreateContext(ghHD);
    hlMakeCurrent(ghHLRC);

    // Enable optimization of the viewing parameters when rendering
    // geometry for OpenHaptics.
    hlEnable(HL_HAPTIC_CAMERA_VIEW);

    // Generate id for the shape.
    gMeshShapeId = hlGenShapes(1);

    hlTouchableFace(HL_FRONT_AND_BACK);

    hlEnable(HL_PROXY_RESOLUTION);

		hlAddEventCallback(HL_EVENT_TOUCH, gMeshShapeId, HL_COLLISION_THREAD, hlTouchCB, 0);
		hlAddEventCallback(HL_EVENT_UNTOUCH, gMeshShapeId, HL_COLLISION_THREAD, hlUnTouchCB, 0);
		hlAddEventCallback(HL_EVENT_MOTION, gMeshShapeId, HL_COLLISION_THREAD, hlMotionCB, 0);
}

void HLCALLBACK hlTouchCB(HLenum event, HLuint object, HLenum thread, HLcache *cache, void *userdata){
	std::cout << "touched \n";	
	state.touched = true;
	//touchedCloth = true;
	//hlGetDoublev(HL_PROXY_POSITION, proxyPosition);
	//massTouched = getClosestMass(proxyPosition);
}

void HLCALLBACK hlUnTouchCB(HLenum event, HLuint object, HLenum thread, HLcache *cache, void *userdata){
	std::cout << "untouched \n";
	state.touched = false;
	//touchedCloth = false;
	//movingOnCloth = false;
}

void HLCALLBACK hlMotionCB(HLenum event, HLuint object, HLenum thread, HLcache *cache, void *userdata){
	std::cout << "moving \n";
	//movingOnCloth = true;
	//hlGetDoublev(HL_PROXY_POSITION, proxyPosition);
	//massTouched = getClosestMass(proxyPosition);
}

/*******************************************************************************
 This handler is called when the application is exiting.  Deallocates any state
 and cleans up.
*******************************************************************************/
void exitHandler()
{
    // Deallocate the sphere shape id we reserved in initHL.
    hlDeleteShapes(gMeshShapeId, 1);

    // Free up the haptic rendering context.
    hlMakeCurrent(NULL);
    if (ghHLRC != NULL)
    {
        hlDeleteContext(ghHLRC);
    }

    // Free up the haptic device.
    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
    }
}

/*******************************************************************************
 Use the current OpenGL viewing transforms to initialize a transform for the
 haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void updateWorkspace()
{
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    hlMatrixMode(HL_TOUCHWORKSPACE);
    hlLoadIdentity();

    // Fit haptic workspace to view volume.
    hluFitWorkspace(projection);

    // Compute cursor scale.
    gCursorScale = hluScreenToModelScale(modelview, projection, viewport);
    gCursorScale *= CURSOR_SIZE_PIXELS;
}

/*******************************************************************************
 The main routine for displaying the scene.  Gets the latest snapshot of state
 from the haptic thread and uses it to display a 3D cursor.
*******************************************************************************/
void drawSceneGraphics()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw 3D cursor at haptic device position.
    drawCursor();

    // Draw object using OpenGL.
    glPushMatrix();
    glBegin(GL_TRIANGLES);
		/*
    for (int i=0;i<state.mesh_buffer.size();i++){
        glVertex3f(state.mesh_buffer[i*3],
                state.mesh_buffer[i*3+1],
                state.mesh_buffer[i*3+2]);
    }
    
    glVertex3f(100,0,0);
    glVertex3f(0,0,100);
    glVertex3f(-100,0,0);
    glVertex3f(-100,0,0);
    glVertex3f(0,0,-100);
    glVertex3f(100,0,0);
    */
    glEnd();
    glPopMatrix();
}

/*******************************************************************************
 The main routine for rendering scene haptics.
*******************************************************************************/
void drawSceneHaptics()
{
		boost::mutex::scoped_lock lock(mesh_mutex);
		
    // Start haptic frame.  (Must do this before rendering any haptic shapes.)
    hlBeginFrame();

    // Set material properties for the shapes to be drawn.
    hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
    hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.5f);
    hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
    hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);
		
		hlHintb(HL_SHAPE_DYNAMIC_SURFACE_CHANGE, HL_TRUE);
    // Start a new haptic shape.  Use the feedback buffer to capture OpenGL
    // geometry for haptic rendering.
    hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, gMeshShapeId);

    // Use OpenGL commands to create geometry.
    //glutSolidSphere(0.5, 32, 32);
		hlTouchModel(HL_CONTACT);
    glPushMatrix();
    glBegin(GL_TRIANGLES);
		
    for (int i = 0; i < state.nVertex; i++){
        glVertex3f(state.mesh_buffer[i*3],
                state.mesh_buffer[i*3+1],
                state.mesh_buffer[i*3+2]);
				/*
        if(i<120)
        ROS_INFO("%d: %f %f %f",
								i,
                state.mesh_buffer[i*3],
                state.mesh_buffer[i*3+1],
                state.mesh_buffer[i*3+2]);
                */
    }

    glEnd();
    glPopMatrix();

    // End the shape.
    hlEndShape();


    //hlGetDoublev(HL_DEVICE_POSITION, state.position);
    //hlGetDoublev(HL_DEVICE_ROTATION, state.orientation);

		hlGetDoublev(HL_PROXY_POSITION, state.position);
    hlGetDoublev(HL_PROXY_ROTATION, state.orientation);

    hlGetBooleanv(HL_BUTTON1_STATE, &(state.button1));
    hlGetBooleanv(HL_BUTTON2_STATE, &(state.button2));

    // End the haptic frame.
    hlEndFrame();
}


/*******************************************************************************
 Draws a 3D cursor for the haptic device using the current local transform,
 the workspace to world transform and the screen coordinate scale.
*******************************************************************************/
void drawCursor()
{
    static const double kCursorRadius = 0.5;
    static const double kCursorHeight = 1.5;
    static const int kCursorTess = 15;
    HLdouble proxyxform[16];

    GLUquadricObj *qobj = 0;

    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glPushMatrix();

    if (!gCursorDisplayList)
    {
        gCursorDisplayList = glGenLists(1);
        glNewList(gCursorDisplayList, GL_COMPILE);
        qobj = gluNewQuadric();

        gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight,
                    kCursorTess, kCursorTess);
        glTranslated(0.0, 0.0, kCursorHeight);
        gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0,
                    kCursorTess, kCursorTess);

        gluDeleteQuadric(qobj);
        glEndList();
    }

    // Get the proxy transform in world coordinates.
    hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
    glMultMatrixd(proxyxform);

    // Apply the local cursor scale factor.
    glScaled(gCursorScale, gCursorScale, gCursorScale);

    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.0, 0.5, 1.0);

    glCallList(gCursorDisplayList);

    glPopMatrix();
    glPopAttrib();
}

/******************************************************************************/

