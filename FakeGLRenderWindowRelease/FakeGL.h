//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5812M Foundations of Modelling & Rendering
//  User Interface for Coursework
//
//  September, 2020
//
//  ------------------------
//  FakeGL.h
//  ------------------------
//
//  A unit for implementing OpenGL workalike calls
//
///////////////////////////////////////////////////

// include guard
#ifndef FAKEGL_H
#define FAKEGL_H

#include "Cartesian3.h"
#include "Homogeneous4.h"
#include "Matrix4.h"
#include "RGBAImage.h"
#include <vector>
#include <deque>

// additional includes
#include "math.h"
using namespace std;
#include <algorithm>
#include <iterator>


// we will store all of the FakeGL context in a class object
// this is similar to the real OpenGL which handles multiple windows
// by having separate variables for each stored in separate graphics
// contexts.
//
// We have simplified the calls, so the code is not *identical* to
// the OpenGL calls, but it's pretty close

// class constants
// constants for Begin()
const unsigned int FAKEGL_POINTS = 0;
const unsigned int FAKEGL_LINES = 1;
const unsigned int FAKEGL_TRIANGLES = 2;
// bitflag constants for Clear()
const unsigned int FAKEGL_COLOR_BUFFER_BIT = 1;
const unsigned int FAKEGL_DEPTH_BUFFER_BIT = 2;
// constants for Enable()/Disable()
const unsigned int FAKEGL_LIGHTING = 1;
const unsigned int FAKEGL_TEXTURE_2D = 2;
const unsigned int FAKEGL_DEPTH_TEST = 3;
const unsigned int FAKEGL_PHONG_SHADING = 4;
// constants for Light() - actually bit flags
const unsigned int FAKEGL_POSITION = 1;
const unsigned int FAKEGL_AMBIENT = 2;
const unsigned int FAKEGL_DIFFUSE = 4;
const unsigned int FAKEGL_AMBIENT_AND_DIFFUSE = 6;
const unsigned int FAKEGL_SPECULAR = 8;
// additional constants for Material()
const unsigned int FAKEGL_EMISSION = 16;
const unsigned int FAKEGL_SHININESS = 32;
// constants for matrix operations
const unsigned int FAKEGL_MODELVIEW = 1;
const unsigned int FAKEGL_PROJECTION = 2;
// constants for texture operations
const unsigned int FAKEGL_MODULATE = 1;
const unsigned int FAKEGL_REPLACE = 2;

// class with vertex attributes
class vertexWithAttributes
    { // class vertexWithAttributes
    public:
	// Position in OCS
    Homogeneous4 position;
	// Colour
    RGBAValue colour;

	// you may need to add more state here

    }; // class vertexWithAttributes

// class for a vertex after transformation to screen space
class screenVertexWithAttributes
    { // class screenVertexWithAttributes
    public:
	// Position in DCS
    Cartesian3 position;
	// Colour
    RGBAValue colour;

	// you may need to add more state here

    }; // class screenVertexWithAttributes

// class for a fragment with attributes
class fragmentWithAttributes
    { // class fragmentWithAttributes
    public:
    // the row & column address in the framebuffer
    int row, col;
    // the RGBA colour of the fragment
    RGBAValue colour;

    // the vertex depth in the depth (z-) buffer
    float z;

    }; // class fragmentWithAttributes

// the class storing the FakeGL context
class FakeGL
    { // class FakeGL
    // for the sake of simplicity & clarity, we make everything public
    public:
    //-----------------------------
    // MATRIX STATE
    //-----------------------------
    int matrixState = FAKEGL_MODELVIEW; // The initial value is GL_MODELVIEW

    //-----------------------------
    // ATTRIBUTE STATE
    //-----------------------------
    RGBAValue colour3;

    //-----------------------------
    // OUTPUT FROM INPUT STAGE
    // INPUT TO TRANSFORM STAGE
    //-----------------------------

    // we want a queue of vertices with attributes for passing to the rasteriser
    std::deque<vertexWithAttributes> vertexQueue;

    //-----------------------------
    // TRANSFORM/LIGHTING STATE
    //-----------------------------
    bool enableLighting  = false;
    bool enableTexture2D = false;
    bool enableDepthTest = false;
    bool enablePhongShading = false;

    //-----------------------------
    // OUTPUT FROM TRANSFORM STAGE
    // INPUT TO RASTER STAGE
    //-----------------------------
    std::deque<screenVertexWithAttributes> rasterQueue;

    //-----------------------------
    // RASTERISE STATE
    //-----------------------------
    RGBAValue bgColour;

    //-----------------------------
    // OUTPUT FROM RASTER STAGE
    // INPUT TO FRAGMENT STAGE
    //-----------------------------
    std::deque<fragmentWithAttributes> fragmentQueue;

    //-----------------------------
    // TEXTURE STATE
    //-----------------------------

    //-----------------------------
    // FRAMEBUFFER STATE
    // OUTPUT FROM FRAGMENT STAGE
    //-----------------------------

	// the frame buffer itself
    RGBAImage frameBuffer;

    // rather than define an extra class, we will cheat and use
    // a second RGBAImage in which the alpha stores the depth buffer
    RGBAImage depthBuffer;

    //-------------------------------------------------//
    //                                                 //
    // CONSTRUCTOR / DESTRUCTOR                        //
    //                                                 //
    //-------------------------------------------------//

    // constructor
    FakeGL();

    // destructor
    ~FakeGL();

    //-------------------------------------------------//
    //                                                 //
    // GEOMETRIC PRIMITIVE ROUTINES                    //
    //                                                 //
    //-------------------------------------------------//

    // starts a sequence of geometric primitives
    void Begin(unsigned int PrimitiveType);
    int primitiveType;

    // ends a sequence of geometric primitives
    void End();

    // sets the size of a point for drawing
    void PointSize(float size);
    float pointSize = 5;

    // sets the width of a line for drawing purposes
    void LineWidth(float width);
    float lineWidth = 5;

    //-------------------------------------------------//
    //                                                 //
    // MATRIX MANIPULATION ROUTINES                    //
    //                                                 //
    //-------------------------------------------------//

    // Matrix4 matrixIdentity;     // Identity matrix
    Matrix4 matrixFrustum;      // Frustum matrix
    Matrix4 matrixOrtho;        // Frustum matrix
    Matrix4 matrixProjection;   // Projection matrix
    Matrix4 matrixMultMatrix;   // Matrix fed into the MultMatrix
    Matrix4 matrixTransform;
    Matrix4 matrixModelview;

    // Matrix for the rotation finction
    Matrix4 matrixRotationX;
    Matrix4 matrixRotationY;
    Matrix4 matrixRotationZ;

    // the current matrix to be applied to a given vertex
    Matrix4 matrixCurrent;

    // Matrix stack
    std::deque<Matrix4> matrixStackProjection;
    std::deque<Matrix4> matrixStackModelview;


    // set the matrix mode (i.e. which one we change)
    void MatrixMode(unsigned int whichMatrix);

    // pushes a matrix on the stack
    void PushMatrix();

    // pops a matrix off the stack
    void PopMatrix();

    // load the identity matrix
    void LoadIdentity();

    // multiply by a known matrix in column-major format
    void MultMatrixf(const float *columnMajorCoordinates);

    // sets a perspective projection matrix
    void Frustum(float left, float right, float bottom, float top, float zNear, float zFar);

    // sets an orthographic projection matrix
    void Ortho(float left, float right, float bottom, float top, float zNear, float zFar);

    // rotate the matrix
    void Rotatef(float angle, float axisX, float axisY, float axisZ);

    // scale the matrix
    void Scalef(float xScale, float yScale, float zScale);

    // translate the matrix
    void Translatef(float xTranslate, float yTranslate, float zTranslate);

    // sets the viewport
    void Viewport(int x, int y, int width, int height);
    float viewportX;
    float viewportY;

    //-------------------------------------------------//
    //                                                 //
    // VERTEX ATTRIBUTE ROUTINES                       //
    //                                                 //
    //-------------------------------------------------//

    // sets colour with floating point
    void Color3f(float red, float green, float blue);

    // sets material properties
    void Materialf(unsigned int parameterName, const float parameterValue);
    void Materialfv(unsigned int parameterName, const float *parameterValues);

    float materialShininessValue;

    float materialAmbientValues[4]  = {0.2, 0.2, 0.2, 1.0};
    float materialDiffuseValues[4]  = {0.8, 0.8, 0.8, 1.0};

    float materialSpecularValues[4] = {0, 0, 0, 1};
    float materialEmissionValues[4] = {0, 0, 1, 0};


    // sets the normal vector
    void Normal3f(float x, float y, float z);
    Cartesian3 normalVector;

    // sets the texture coordinates
    void TexCoord2f(float u, float v);

    // sets the vertex & launches it down the pipeline
    void Vertex3f(float x, float y, float z);

    //-------------------------------------------------//
    //                                                 //
    // STATE VARIABLE ROUTINES                         //
    //                                                 //
    //-------------------------------------------------//

    // disables a specific flag in the library
    void Disable(unsigned int property);

    // enables a specific flag in the library
    void Enable(unsigned int property);

    //-------------------------------------------------//
    //                                                 //
    // LIGHTING STATE ROUTINES                         //
    //                                                 //
    //-------------------------------------------------//

    // sets properties for the one and only light
    void Light(int parameterName, const float *parameterValues);
    float lightPositionValues[4] = {0, 0, 1, 0}; // stores the position of the light
    float lightAmbientValues[4]  = {1, 1, 1, 1}; // stores the colour of the ambient light
    float lightDiffuseValues[4]  = {1, 1, 1, 1}; // stores the colour of the diffuse light
    float lightSpecularValues[4] = {1, 1, 1, 1}; // stores the colour of the diffuse light

    //-------------------------------------------------//
    //                                                 //
    // TEXTURE PROCESSING ROUTINES                     //
    //                                                 //
    // Note that we only allow one texture             //
    // so glGenTexture & glBindTexture aren't needed   //
    //                                                 //
    //-------------------------------------------------//

    // sets whether textures replace or modulate
    void TexEnvMode(unsigned int textureMode);

    // sets the texture image that corresponds to a given ID
    void TexImage2D(const RGBAImage &textureImage);

    //-------------------------------------------------//
    //                                                 //
    // FRAME BUFFER ROUTINES                           //
    //                                                 //
    //-------------------------------------------------//

    // clears the frame buffer
    void Clear(unsigned int mask);

    // sets the clear colour for the frame buffer
    void ClearColor(float red, float green, float blue, float alpha);

    //-------------------------------------------------//
    //                                                 //
    // ROUTINE TO FLUSH THE PIPELINE                   //
    //                                                 //
    //-------------------------------------------------//

    // flushes the pipeline
    void Flush();

    //-------------------------------------------------//
    //                                                 //
    // MAJOR PROCESSING ROUTINES                       //
    //                                                 //
    //-------------------------------------------------//

    // transform one vertex & shift to the transformed queue
    void TransformVertex();
    float cameraViewProjectionMatrix[4][4];
    std::vector<float> objectPos4;
    std::vector<float> postProjectivePosition;

    // rasterise a single primitive if there are enough vertices on the queue
    bool RasterisePrimitive();

    // rasterises a single point
    void RasterisePoint(screenVertexWithAttributes &vertex0);

    // rasterises a single line segment
    void RasteriseLineSegment(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1);

    // rasterises a single triangle
    void RasteriseTriangle(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1, screenVertexWithAttributes &vertex2);

    // process a single fragment
    void ProcessFragment();

    }; // class FakeGL

// standard routine for dumping the entire FakeGL context (except for texture / image)
std::ostream &operator << (std::ostream &outStream, FakeGL &fakeGL);

// subroutines for other classes
std::ostream &operator << (std::ostream &outStream, vertexWithAttributes &vertex);
std::ostream &operator << (std::ostream &outStream, screenVertexWithAttributes &vertex);
std::ostream &operator << (std::ostream &outStream, fragmentWithAttributes &fragment);

// include guard
#endif
