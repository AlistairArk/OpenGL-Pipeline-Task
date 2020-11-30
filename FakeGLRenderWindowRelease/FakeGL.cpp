//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5812M Foundations of Modelling & Rendering
//  User Interface for Coursework
//
//  September, 2020
//
//  ------------------------
//  FakeGL.cpp
//  ------------------------
//
//  A unit for implementing OpenGL workalike calls
//
///////////////////////////////////////////////////

#include "FakeGL.h"
#include <math.h>

//-------------------------------------------------//
//                                                 //
// CONSTRUCTOR / DESTRUCTOR                        //
//                                                 //
//-------------------------------------------------//

// constructor
FakeGL::FakeGL()
    { // constructor
    } // constructor

// destructor
FakeGL::~FakeGL()
    { // destructor
    } // destructor

//-------------------------------------------------//
//                                                 //
// GEOMETRIC PRIMITIVE ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// starts a sequence of geometric primitives
void FakeGL::Begin(unsigned int PrimitiveType)
    { // Begin()

        /*
            glBegin and glEnd delimit the verticies that define a primitive or
            a group of like primitives. glBegin accepts a single argument that
            specifies in which of 10 ways the verticies are interpreted.

            Taking $n$ as an integer count starting at one, and $N$ as the total
            number of verticies specified, the interpretations are as follows.
        */


        switch(PrimitiveType)
        {
            // Treats each vertex as a single point.
            // Vertex $n$ defines point $n$. $N$ points are drawn.
            case FAKEGL_POINTS:
                // std::cout << "      primitiveType = FAKEGL_POINTS" << std::endl;
                primitiveType = FAKEGL_POINTS;
                break;

            // Treats each pair of verticies as an independent line segment.
            // Verticies $2n^-^1$ and $2n$ define line $n$. $N/2$ lines are drawn.
            case FAKEGL_LINES:
                // std::cout << "      primitiveType = FAKEGL_LINES" << std::endl;
                primitiveType = FAKEGL_LINES;
                break;

            // Treats each triplet of verticies as an independent triangle.
            // Verticies $3n^-^2$, $3n^-^1$, and $3n$ define triangle $n$.
            // $N/3$ triangles are drawn.
            case FAKEGL_TRIANGLES:
                // std::cout << "      primitiveType = FAKEGL_TRIANGLES" << std::endl;
                primitiveType = FAKEGL_TRIANGLES;
                break;

        }

    } // Begin()

// ends a sequence of geometric primitives
void FakeGL::End()
    { // End()

        // Clear the raster and vertex queues here
        vertexQueue.clear();
        rasterQueue.clear();

        primitiveType = 0; // Set primitiveType back to zero

    } // End()

// sets the size of a point for drawing
void FakeGL::PointSize(float size)
    { // PointSize()

        pointSize = size;

    } // PointSize()

// sets the width of a line for drawing purposes
void FakeGL::LineWidth(float width)
    { // LineWidth()

        lineWidth = width;

    } // LineWidth()

//-------------------------------------------------//
//                                                 //
// MATRIX MANIPULATION ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// set the matrix mode (i.e. which one we change)
void FakeGL::MatrixMode(unsigned int whichMatrix)
    { // MatrixMode()
        /*
            mode
                Specifies withc metrix stack is the target of subsequent matrix
                operations. Three values are accepted: GL_MODELVIEW, GL_PROJECTION
                and GL_TEXTURE. The initial value is GL_MODELVIEW.

            glMatrixMode sets the current matrix mode. mode can assume one of
            four values.

            GL_TEXTURE
                Applies subsequent matrix operations to the texture matrix stack.

            GL_COLOR
                Applies subsequent matrix operations to the color matrix stack
        */

        switch (whichMatrix)
        {
            // Applies subsequent matrix operations to the modelview matrix stack
            case FAKEGL_MODELVIEW:
                matrixState = FAKEGL_MODELVIEW;
                break;

            // Applies subsequent matrix operations to the projection matrix stack
            case FAKEGL_PROJECTION:
                matrixState = FAKEGL_PROJECTION;
                break;
        }

    } // MatrixMode()

// pushes a matrix on the stack
void FakeGL::PushMatrix()
    { // PushMatrix()

    } // PushMatrix()

// pops a matrix off the stack
void FakeGL::PopMatrix()
    { // PopMatrix()

    } // PopMatrix()

// load the identity matrix
void FakeGL::LoadIdentity()
    { // LoadIdentity()

        /*
            glLoadIdentity replaces the current matrix with the identity matrix.
            It is semantically equivalent to calling glLoadMatrix with the
            identity matrix,
                1000010000100001
            but in some cases it is more efficent.
        */

        matrixCurrent[0][0] = 1; matrixCurrent[1][0] = 0; matrixCurrent[2][0] = 0; matrixCurrent[3][0] = 0;
        matrixCurrent[0][1] = 0; matrixCurrent[1][1] = 1; matrixCurrent[2][1] = 0; matrixCurrent[3][1] = 0;
        matrixCurrent[0][2] = 0; matrixCurrent[1][2] = 0; matrixCurrent[2][2] = 1; matrixCurrent[3][2] = 0;
        matrixCurrent[0][3] = 0; matrixCurrent[1][3] = 0; matrixCurrent[2][3] = 0; matrixCurrent[3][3] = 1;

    } // LoadIdentity()

// multiply by a known matrix in column-major format
void FakeGL::MultMatrixf(const float *columnMajorCoordinates)
    { // MultMatrixf()

        /*
            glMutlMatrix function multiplies the current matrix by the one specified
            in m. That is, if M the current matrix and T is the matrix passed to
            glMultMatrix, then M is replaced with M T.

            The m parameter points to a 4x4 matrix of asingle precosion of double-precision
            floating-point values stored in column major order.

            --Switch between setting for mMM or
        */

        // Store columnMajorCoordinates as Matrix4 matrixMultMatrix
        int multMatrixIndex=-1;
        for ( int ny = 0; ny < 4; ++ny )
        {
            for ( int nx = 0; nx < 4; ++nx )
            {
                multMatrixIndex++;
                matrixMultMatrix[ny][nx] = columnMajorCoordinates[multMatrixIndex];
            }
        }

        // multiply the current matrix by the one specified in matrixMultMatrix
        matrixCurrent = matrixCurrent*matrixMultMatrix;

    } // MultMatrixf()

// sets up a perspective projection matrix
void FakeGL::Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Frustum()

        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << " Frustum (" <<  left << ", " <<  right << ", " <<  bottom << ", " <<  top << ", " <<  zNear << ", " <<  zFar << ")" << std::endl;

    } // Frustum()

// sets an orthographic projection matrix
void FakeGL::Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Ortho()
        /*
        left, right
            Specify the coordinates for the left and right vertical clipping planes.

        bottom, top
            Specify the coordinates for the bottom and top horizontal clipping planes.

        zNear, zFar
            Specify the distances to the nearer and farther depth clipping planes.
            These values are negative if the plane is to behind the viewer.


        glOrtho describes a transformation that produces a parallel projection.
        The current matrix (see glMatrixMode) is multiplied by this matrix and
        the result replaces the current matrix, as if glMultMatrix were called
        with the following matrix as it's argument.

        2       0       0       tx
  right - left

        0       2       0       ty
            top - bottom

        0       0       -1      tz
                   zFar - zNear

        0       0       0       1


        where
                tx = - (right + left) / (right - left)
                ty = - (top + bottom) / (top - bottom)
                tz = - (zFar + zNear) / (zFar - zNear)
        */

        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << " Ortho (" <<  left << ", " <<  right << ", " <<  bottom << ", " <<  top << ", " <<  zNear << ", " <<  zFar << ")" << std::endl;

    } // Ortho()

// rotate the matrix
void FakeGL::Rotatef(float angle, float axisX, float axisY, float axisZ)
    { // Rotatef()

        // std::cout << "==========================================" << std::endl;
        // std::cout << " Rotatef: ("<< angle << ", " << axisX << ", " << axisY << ")" << std::endl;

        /*
            PARAMETERS
               angle
                   Specifies the angle of rotation, in degrees.

               x, y, z
                   Specify the x, y, and z coordinates of a vector, respectively.

            DESCRIPTION
               glRotate produces a rotation of angle degrees around the vector x y z.
               The current matrix (see glMatrixMode()) is multiplied by a rotation
               matrix with the product replacing the current matrix, as if
               glMultMatrix() were called with the following matrix as its argument:

               x 2 <U+2061> 1 - c + c x <U+2062> y <U+2061> 1 - c - z <U+2062> s x
            <U+2062> z <U+2061> 1 - c + y <U+2062> s 0 y <U+2062> x <U+2061> 1
               - c + z <U+2062> s y 2 <U+2061> 1 - c + c y <U+2062> z <U+2061> 1 - c - x <U+2062> s 0 x <U+2062> z <U+2061> 1 - c - y <U+2062>
               s y <U+2062> z <U+2061> 1 - c + x <U+2062> s z 2 <U+2061> 1 - c + c 0 0 0 0 1

               Where c = cos <U+2061> angle, s = sin <U+2061> angle, and x y z = 1 (if not, the GL
               will normalize this vector).

               If the matrix mode is either GL_MODELVIEW or GL_PROJECTION, all objects
               drawn after glRotate is called are rotated. Use glPushMatrix() and
               glPopMatrix() to save and restore the unrotated coordinate system.

            NOTES
               This rotation follows the right-hand rule, so if the vector x y z
               points toward the user, the rotation will be counterclockwise.
        */

    } // Rotatef()

// scale the matrix
void FakeGL::Scalef(float xScale, float yScale, float zScale)
    { // Scalef()

        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << " Scalef: ("<< xScale << ", " << yScale << ", " << zScale << ")" << std::endl;
        /*
            x, y, z
                Specify scale factors along the x, y, and z axes, respectively.

            glScale produces a nonuniform scaling along the x, y, and z axes. The
               three parameters indicate the desired scale factor along each of the
               three axes.

               The current matrix (see glMatrixMode()) is multiplied by this scale
               matrix, and the product replaces the current matrix as if
               glMultMatrix() were called with the following matrix as its argument:

               x 0 0 0 0 y 0 0 0 0 z 0 0 0 0 1

               If the matrix mode is either GL_MODELVIEW or GL_PROJECTION, all objects
               drawn after glScale is called are scaled.

               Use glPushMatrix() and glPopMatrix() to save and restore the unscaled
               coordinate system.

        */

        Matrix4 matrixScale;    // Translation Matrix

        matrixScale[0][0] = xScale; matrixScale[1][0] = 0;      matrixScale[2][0] = 0;      matrixScale[3][0] = 0;
        matrixScale[0][1] = 0;      matrixScale[1][1] = yScale; matrixScale[2][1] = 0;      matrixScale[3][1] = 0;
        matrixScale[0][2] = 0;      matrixScale[1][2] = 0;      matrixScale[2][2] = zScale; matrixScale[3][2] = 0;
        matrixScale[0][3] = 0;      matrixScale[1][3] = 0;      matrixScale[2][3] = 0;      matrixScale[3][3] = 1;

        matrixCurrent =  matrixScale*matrixCurrent;

    } // Scalef()

// translate the matrix
void FakeGL::Translatef(float xTranslate, float yTranslate, float zTranslate)
    { // Translatef()
        /*
        x,y,z
            Specify the x, y, and z

            Produces a translation by x,y,z. The current matrix (see glMatrixMode)
            is multiplied by this translation matrix, with the product replacing
            the current matrix, as if glMultMatrix were called with the follwing
            matrix for it's argument:

            If the matrix mode is either GL_MODELVIEW or GL_PROJECTION, all
            objects drawn after a call to glTranslate are translated.

            Use glPushMatrix and glPopMatrix to save and restore the untranslated
            coordinate system.
        */

        Matrix4 matrixTranslate;    // Translation Matrix

        matrixTranslate[0][0] = 1; matrixTranslate[1][0] = 0; matrixTranslate[2][0] = 0; matrixTranslate[3][0] = xTranslate;
        matrixTranslate[0][1] = 0; matrixTranslate[1][1] = 1; matrixTranslate[2][1] = 0; matrixTranslate[3][1] = yTranslate;
        matrixTranslate[0][2] = 0; matrixTranslate[1][2] = 0; matrixTranslate[2][2] = 1; matrixTranslate[3][2] = zTranslate;
        matrixTranslate[0][3] = 0; matrixTranslate[1][3] = 0; matrixTranslate[2][3] = 0; matrixTranslate[3][3] = 1;

        matrixCurrent =  matrixTranslate*matrixCurrent;

    } // Translatef()

// sets the viewport
void FakeGL::Viewport(int x, int y, int width, int height)
    { // Viewport()
        /*
        x,y
            Specify the lower left corner of the viewport rectangle, in pixels.
            The initial value is (0,0)

        width, height
            Specify the width and height of the viewport. When a GL context is
            first attached to a window, width and height are set to diemensions
            of that window.

        Desc:
            glViewport specifies the affine transformation of x and y from the
            normalized device coodrinates to window coordinates. Let (xnd, ynd)
            be normalized device coordintates. Then the window coordinates
            (xw, yw) are computed as follows:

            xw = (xnd + 1)(width/2) + x
            yw = (ynd + 1)(height/2) + y

            Viewport width and height are silently clamped to a range that
            depends on the implementation. To query this range call glGet with
            argument GL_MAX_VIEWPORT_DIMS.
        */
        // Normalized device coordiantes
        float xnd = 0;
        float ynd = 0;

        // Window coordinates
        float xw = (xnd + 1) * (width/2) + x;
        float yw = (ynd + 1) * (height/2) + y;

        viewportX = xw;
        viewportY = yw;


    } // Viewport()

//-------------------------------------------------//
//                                                 //
// VERTEX ATTRIBUTE ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// sets colour with floating point
void FakeGL::Color3f(float red, float green, float blue)
    { // Color3f()
        colour3.red = red*255;
        colour3.green = green*255;
        colour3.blue = blue*255;
    } // Color3f()

// sets material properties
void FakeGL::Materialf(unsigned int parameterName, const float parameterValue)
    { // Materialf()

        switch(parameterName){
            case FAKEGL_AMBIENT:
                break;
            case FAKEGL_DIFFUSE:
                break;
            case FAKEGL_AMBIENT_AND_DIFFUSE:
                break;
            case FAKEGL_SPECULAR:
                break;
            case FAKEGL_EMISSION:
                break;
            case FAKEGL_SHININESS:
                break;
        }

    } // Materialf()

void FakeGL::Materialfv(unsigned int parameterName, const float *parameterValues)
    { // Materialfv()

        switch(parameterName){
            case FAKEGL_AMBIENT:
                break;
            case FAKEGL_DIFFUSE:
                break;
            case FAKEGL_AMBIENT_AND_DIFFUSE:
                break;
            case FAKEGL_SPECULAR:
                break;
            case FAKEGL_EMISSION:
                break;
            case FAKEGL_SHININESS:
                break;
        }

    } // Materialfv()

// sets the normal vector
void FakeGL::Normal3f(float x, float y, float z)
    { // Normal3f()

        normal3f = {x, y, z};

        /*
            PARAMETERS
               nx, ny, nz
                   Specify the x, y, and z coordinates of the new current normal. The
                   initial value of the current normal is the unit vector, (0, 0, 1).

            PARAMETERS
            v
                Specifies a pointer to an array of three elements: the x, y, and z
                coordinates of the new current normal.

            DESCRIPTION
                   The current normal is set to the given coordinates whenever glNormal is
                   issued. Byte, short, or integer arguments are converted to
                   floating-point format with a linear mapping that maps the most positive
                   representable integer value to 1.0 and the most negative representable
                   integer value to -1.0.

                   Normals specified with glNormal need not have unit length. If
                   GL_NORMALIZE is enabled, then normals of any length specified with
                   glNormal are normalized after transformation. If GL_RESCALE_NORMAL is
                   enabled, normals are scaled by a scaling factor derived from the
                   modelview matrix.  GL_RESCALE_NORMAL requires that the originally
                   specified normals were of unit length, and that the modelview matrix
                   contain only uniform scales for proper results. To enable and disable
                   normalization, call glEnable() and glDisable() with either GL_NORMALIZE
                   or GL_RESCALE_NORMAL. Normalization is initially disabled.

            NOTES
                   The current normal can be updated at any time. In particular, glNormal
                   can be called between a call to glBegin() and the corresponding call to
                   glEnd().

        */

    } // Normal3f()

// sets the texture coordinates
void FakeGL::TexCoord2f(float u, float v)
    { // TexCoord2f()

        /*
            PARAMETERS
                   v
                       Specifies a pointer to an array of one, two, three, or four
                       elements, which in turn specify the s, t, r, and q texture
                       coordinates.

            DESCRIPTION
                   glTexCoord specifies texture coordinates in one, two, three, or four
                   dimensions.  glTexCoord1 sets the current texture coordinates to s 0 0
                   1; a call to glTexCoord2 sets them to s t 0 1. Similarly, glTexCoord3
                   specifies the texture coordinates as s t r 1, and glTexCoord4 defines
                   all four components explicitly as s t r q.

                   The current texture coordinates are part of the data that is associated
                   with each vertex and with the current raster position. Initially, the
                   values for s, t, r, and q are (0, 0, 0, 1).

            NOTES
                   The current texture coordinates can be updated at any time. In
                   particular, glTexCoord can be called between a call to glBegin() and
                   the corresponding call to glEnd().

                   When the ARB_imaging extension is supported, glTexCoord always updates
                   texture unit GL_TEXTURE0.
        */


    } // TexCoord2f()

// sets the vertex & launches it down the pipeline
void FakeGL::Vertex3f(float x, float y, float z)
    { // Vertex3f()

        /*
            z,y,z,w
                Specify x,y,z, coordinates of a vertex.
        */

        //-----------------------------
        // OUTPUT FROM INPUT STAGE
        // INPUT TO TRANSFORM STAGE
        //-----------------------------

        vertexWithAttributes v;
        v.position.x = x;
        v.position.y = y;
        v.position.z = z;
        v.position.w = 1.0f;

        v.colour = colour3;
        vertexQueue.push_back(v);

        TransformVertex();


    } // Vertex3f()

//-------------------------------------------------//
//                                                 //
// STATE VARIABLE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// disables a specific flag in the library
void FakeGL::Disable(unsigned int property)
    { // Disable()

        switch (property)
        {
            case FAKEGL_LIGHTING:
                enableLighting   = false;
                break;

            case FAKEGL_TEXTURE_2D:
                enableTexture2D = false;
                break;

            case FAKEGL_DEPTH_TEST:
                enableDepthTest = false;
                break;
        }

    } // Disable()

// enables a specific flag in the library
void FakeGL::Enable(unsigned int property)
    { // Enable()

        switch (property)
        {
            case FAKEGL_LIGHTING:
                enableLighting  = true;
                break;

            case FAKEGL_TEXTURE_2D:
                enableTexture2D = true;
                break;

            case FAKEGL_DEPTH_TEST:
                enableDepthTest = true;
                break;
        }

    } // Enable()

//-------------------------------------------------//
//                                                 //
// LIGHTING STATE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// sets properties for the one and only light
void FakeGL::Light(int parameterName, const float *parameterValues)
    { // Light()

          /*
            params
                Specifies a pointer to the value or values that parameter pname of
                light source light will be set to.

          */
        switch(parameterName){

            case FAKEGL_POSITION:
                for( unsigned int n = 0; n < sizeof(parameterValues); n = n + 1 ) lightPositionValues[n] = parameterValues[n];
                break;
            case FAKEGL_AMBIENT:
                for( unsigned int n = 0; n < sizeof(parameterValues); n = n + 1 ) lightAmbientValues[n] = parameterValues[n];
                break;
            case FAKEGL_DIFFUSE:
                for( unsigned int n = 0; n < sizeof(parameterValues); n = n + 1 ) lightDiffuseValues[n] = parameterValues[n];
                break;
            case FAKEGL_AMBIENT_AND_DIFFUSE:
                for( unsigned int n = 0; n < sizeof(parameterValues); n = n + 1 ) lightAmbientAndDiffuseValues[n] = parameterValues[n];
                break;
            case FAKEGL_SPECULAR:
                for( unsigned int n = 0; n < sizeof(parameterValues); n = n + 1 ) lightSpecularValues[n] = parameterValues[n];
                break;

        }

    } // Light()

//-------------------------------------------------//
//                                                 //
// TEXTURE PROCESSING ROUTINES                     //
//                                                 //
// Note that we only allow one texture             //
// so glGenTexture & glBindTexture aren't needed   //
//                                                 //
//-------------------------------------------------//

// sets whether textures replace or modulate
void FakeGL::TexEnvMode(unsigned int textureMode)
    { // TexEnvMode()
    } // TexEnvMode()

// sets the texture image that corresponds to a given ID
void FakeGL::TexImage2D(const RGBAImage &textureImage)
    { // TexImage2D()
    } // TexImage2D()

//-------------------------------------------------//
//                                                 //
// FRAME BUFFER ROUTINES                           //
//                                                 //
//-------------------------------------------------//

// clears the frame buffer
void FakeGL::Clear(unsigned int mask)
    { // Clear()

        // Ensure that the frame buffer and depthBuffer are of the same size
        if (frameBuffer.width != depthBuffer.width || frameBuffer.height != depthBuffer.height)
        {
            // If they are not of equal size set the depth buffer to match
            // the size of the frame buffer
            depthBuffer.Resize(frameBuffer.width, frameBuffer.height);
        }

        for (int row = 0; row < frameBuffer.height; row++)
        {
            for (int col = 0; col < frameBuffer.width; col++)
            {
                frameBuffer[row][col] = bgColour;

                // For each pixel, set the z bugger to -infinity
                depthBuffer[row][col].alpha = 0;
            }
        }

    } // Clear()

// sets the clear colour for the frame buffer
void FakeGL::ClearColor(float red, float green, float blue, float alpha)
    { // ClearColor()

        bgColour.red = red*255;
        bgColour.green = green*255;
        bgColour.blue = blue*255;
        bgColour.alpha = alpha*255;

    } // ClearColor()

//-------------------------------------------------//
//                                                 //
// MAJOR PROCESSING ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// transform one vertex & shift to the raster queue
void FakeGL::TransformVertex()
    { // TransformVertex()

        vertexWithAttributes vertexCurrent;
        vertexCurrent = vertexQueue.back(); // Get the vertex on the back of the queue
        vertexQueue.pop_back();             // Pop vertex off queue

        vertexWithAttributes vertexTransformed;

        // Apply the current matrix to the vertex
        vertexTransformed.position.x = (vertexCurrent.position.x*matrixCurrent[0][0]) + (vertexCurrent.position.y*matrixCurrent[1][0]) + (vertexCurrent.position.z*matrixCurrent[2][0]) + (vertexCurrent.position.w*matrixCurrent[3][0]);
        vertexTransformed.position.y = (vertexCurrent.position.x*matrixCurrent[0][1]) + (vertexCurrent.position.y*matrixCurrent[1][1]) + (vertexCurrent.position.z*matrixCurrent[2][1]) + (vertexCurrent.position.w*matrixCurrent[3][1]);
        vertexTransformed.position.z = (vertexCurrent.position.x*matrixCurrent[0][2]) + (vertexCurrent.position.y*matrixCurrent[1][2]) + (vertexCurrent.position.z*matrixCurrent[2][2]) + (vertexCurrent.position.w*matrixCurrent[3][2]);
        vertexTransformed.position.w = (vertexCurrent.position.x*matrixCurrent[0][3]) + (vertexCurrent.position.y*matrixCurrent[1][3]) + (vertexCurrent.position.z*matrixCurrent[2][3]) + (vertexCurrent.position.w*matrixCurrent[3][3]);

        // Convert to vertex to DCS (Device Coordinate System)
        int height = frameBuffer.height;
        int width = frameBuffer.width;

        screenVertexWithAttributes vertexDeviceCoordinate;

        // Calculate center position based on the viewport scale
        if (viewportX<viewportY){
            vertexDeviceCoordinate.position.x  = (width  *.5) + (vertexTransformed.position.x * (viewportX));
            vertexDeviceCoordinate.position.y  = (height *.5) + (vertexTransformed.position.y * (viewportX));
        }else{
            vertexDeviceCoordinate.position.x  = (width  *.5) + (vertexTransformed.position.x * (viewportY));
            vertexDeviceCoordinate.position.y  = (height *.5) + (vertexTransformed.position.y * (viewportY));
        }
        // Store the Z position for Z sorting
        vertexDeviceCoordinate.position.z  = vertexTransformed.position.z;

        // Get color data from the vertex
        vertexDeviceCoordinate.colour      = vertexCurrent.colour;
        rasterQueue.push_back(vertexDeviceCoordinate);


        // If primitive to rasterise is found, process fragments in queue
        if (RasterisePrimitive()){
            // std::cout << "------------------------------------------" << std::endl;
            // std::cout << " ProcessFragment () " << std::endl;
            // std::cout << "\tProcessing fragments in queue...";

            while (fragmentQueue.size()>0){
                ProcessFragment();
            }

        }



    } // TransformVertex()

// rasterise a single primitive if there are enough vertices on the queue
bool FakeGL::RasterisePrimitive()
    { // RasterisePrimitive()


        // Check number of verticies in the queue
        // If enough verticies send them, else return false
        switch(primitiveType)
        {
            // Treats each vertex as a single point.
            // Vertex $n$ defines point $n$. $N$ points are drawn.
            case FAKEGL_POINTS:      // Cheack number of vertices on the queue
                if (rasterQueue.size() == 1){
                    // std::cout << "------------------------------------------" << std::endl;
                    // std::cout << " RasterisePrimitive () " << std::endl;
                    // std::cout << "\tProcessing: FAKEGL_POINTS" << std::endl;

                    // Send the verticies down the pipeline
                    screenVertexWithAttributes dcs0;
                    dcs0 = rasterQueue.front();
                    rasterQueue.pop_front(); // Pop verticies off the queue
                    RasterisePoint(dcs0);

                    return true;
                }
                break;
            // Treats each pair of verticies as an independent line segment.
            // Verticies $2n^-^1$ and $2n$ define line $n$. $N/2$ lines are drawn.
            case FAKEGL_LINES:       // Cheack number of vertices on the queue
                if (rasterQueue.size() == 2){


                    // Send the verticies down the pipeline
                    screenVertexWithAttributes dcs0;
                    dcs0 = rasterQueue.front();
                    rasterQueue.pop_front(); // Pop verticies off the queue

                    screenVertexWithAttributes dcs1;
                    dcs1 = rasterQueue.front();
                    rasterQueue.pop_front(); // Pop verticies off the queue

                    std::cout << "------------------------------------------" << std::endl;
                    std::cout << " RasterisePrimitive () " << std::endl;
                    std::cout << "\tProcessing: FAKEGL_LINES" << std::endl;
                    std::cout << dcs0 << dcs1 << std::endl;
                    RasteriseLineSegment(dcs0, dcs1);

                    return true;
                }
                break;

            // Treats each triplet of verticies as an independent triangle.
            // Verticies $3n^-^2$, $3n^-^1$, and $3n$ define triangle $n$.
            // $N/3$ triangles are drawn.
            case FAKEGL_TRIANGLES:  // Cheack number of vertices on the queue
                if (rasterQueue.size() == 3){
                    // std::cout << "------------------------------------------" << std::endl;
                    // std::cout << " RasterisePrimitive () " << std::endl;
                    // std::cout << "\tProcessing: FAKEGL_TRIANGLES" << std::endl;

                    // Send the verticies down the pipeline
                    screenVertexWithAttributes dcs0;
                    dcs0 = rasterQueue.front();
                    rasterQueue.pop_front(); // Pop verticies off the queue

                    screenVertexWithAttributes dcs1;
                    dcs1 = rasterQueue.front();
                    rasterQueue.pop_front(); // Pop verticies off the queue

                    screenVertexWithAttributes dcs2;
                    dcs2 = rasterQueue.front();
                    rasterQueue.pop_front(); // Pop verticies off the queue

                    RasteriseTriangle(dcs0,dcs1,dcs2);

                    return true;
                }
                break;
        }

        return false;

    } // RasterisePrimitive()



// rasterises a single point
void FakeGL::RasterisePoint(screenVertexWithAttributes &vertex0)
    { // RasterisePoint()

        // std::cout << "------------------------------------------" << std::endl;
        // std::cout << " RasterisePoint () \n"<< vertex0 << std::endl;
        std::cout << " FRAGMENT Z \n"<< vertex0.position.z << std::endl;

        int x,y;
        x = vertex0.position.x;
        y = vertex0.position.y;

        int pointSizeH = (pointSize*.5); // Half of point size for box

        // Iterate over a square around the point
        for (int xOffset = 0; xOffset < pointSize; xOffset++)
        {
            for (int yOffset = 0; yOffset < pointSize; yOffset++)
            {
                // Convert point to point in the square
                int x2 = xOffset + x - pointSizeH;
                int y2 = yOffset + y - pointSizeH;

                // If point is within frame buffer
                if (x2>0 && x2< frameBuffer.width && y2>0 && y2< frameBuffer.height)
                {   // Set frameBuffer address in fragment queue

                    fragmentWithAttributes fragment;
                    fragment.row = y2;
                    fragment.col = x2;
                    fragment.z = vertex0.position.z; // Depth Buffer
                    fragment.colour = vertex0.colour;

                    fragmentQueue.push_back(fragment);
                }
            }
        }
    } // RasterisePoint()

// rasterises a single line segment
void FakeGL::RasteriseLineSegment(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1)
    { // RasteriseLineSegment()

        // Begin by converting to screenspace co-ordinates
        float x1 = vertex0.position.x; // Axis in width
        float y1 = vertex0.position.y; // Axis in height
        float z1 = vertex0.position.z; // Z Depth

        float x2 = vertex1.position.x; // Axis in width
        float y2 = vertex1.position.y; // Axis in height
        float z2 = vertex1.position.z; // Z Depth

        // Calculate length of line in 3D space
        float lineLength;

        int r1 = vertex0.colour.red;
        int g1 = vertex0.colour.green;
        int b1 = vertex0.colour.blue;

        int r2 = vertex1.colour.red;
        int g2 = vertex1.colour.green;
        int b2 = vertex1.colour.blue;


        float x = 0, y = 0;
        float xDif=x1-x2,yDif=y1-y2;
        float steps = sqrt(pow(x2-x1,2)+pow(y2-y1,2));   // Distance between points
        float m = (y2 - y1) / (x2 - x1);                 // Gradiant

        std::cout << "m: "<< m << vertex0 << std::endl;


        for (float i = 0; i < steps; i++) {

            // Iterate over a square around the point
            int lineWidthH = lineWidth*.5;
            int lineWidthX = 1;
            int lineWidthY = 1;

            if (m){ // Line is Diagonal or Vertical
                y = (yDif*(i/steps)) + y2;
                x = ((yDif*(i/steps)) / m) + x2;
            } else { // Line is Horizontal
                y = y1;
                x = xDif*(i/steps) + x2;
            }

            if (m<-1 || m >1){ // Check gradiant for line width
                lineWidthX = lineWidth;
            }else{
                lineWidthY = lineWidth;
            }

            // Color interpolation (ternary operation)
            double stepRatio2 = ((steps-i)/steps);
            double stepRatio = (i/steps);

            RGBAValue colour;
            colour.red   = (r1>r2) ? ((r1-r2)*stepRatio)+r2 : ((r2-r1)*stepRatio2)+r1;
            colour.green = (g1>g2) ? ((g1-g2)*stepRatio)+g2 : ((g2-g1)*stepRatio2)+g1;
            colour.blue  = (b1>b2) ? ((b1-b2)*stepRatio)+b2 : ((b2-b1)*stepRatio2)+b1;

            for (int xOffset = 0; xOffset < lineWidthX; xOffset++)
            {
                for (int yOffset = 0; yOffset < lineWidthY; yOffset++)
                {
                    // Convert point to point in the square
                    int x3 = xOffset + x - lineWidthH;
                    int y3 = yOffset + y - lineWidthH;

                    // Calculate Z depth on line using (i/steps) as a ratio for current distance across the line
                    float z3 = z2 + (i/steps)*(z1-z2);

                    // If point is within frame buffer
                    if (x3>0 && x3< frameBuffer.width && y3>0 && y3< frameBuffer.height)
                    {   // Set frameBuffer address in fragment queue

                        fragmentWithAttributes fragment;
                        fragment.row = y3;
                        fragment.col = x3;
                        fragment.z = z3;
                        fragment.colour = colour;

                        fragmentQueue.push_back(fragment);
                    }
                }
            }

            /*
                This would need to be modified to draw a line

                NOTE TO SELF:
                    The way we are probably expecded to do is by drawing multiple single width lines in parallel
                    to one another to create one thick line of an expected length. This can be done rather easily
                    by making some modifications to the starting algorithm.

                    Optionally, we could keep the code we have now and find a way to make it draw a line at a 180
                    degree angle to the current line at each point. That sounds a little messy, so consider this.
                    A width of 5 would just mean drawing 5 lines of size 1 on either side of one another just
                    one cell apart. My line code here is already perfect for stopping tearing so that shouldn't
                    be an issue. TEST IT! TRY IT! DO IT!

            */

            // int lineWidthH = lineWidth*.5;
            // for (int xOffset = 0; xOffset < lineWidth; xOffset++)
            // {
            //     for (int yOffset = 0; yOffset < lineWidth; yOffset++)
            //     {
            //         // Convert point to point in the square
            //         int x3 = xOffset + x - lineWidthH;
            //         int y3 = yOffset + y - lineWidthH;
            //
            //         // Calculate Z depth on line using (i/steps) as a ratio for current distance across the line
            //         float z3 = z2 + (i/steps)*(z1-z2);
            //
            //         // If point is within frame buffer
            //         if (x3>0 && x3< frameBuffer.width && y3>0 && y3< frameBuffer.height)
            //         {   // Set frameBuffer address in fragment queue
            //
            //             fragmentWithAttributes fragment;
            //             fragment.row = y3;
            //             fragment.col = x3;
            //             fragment.z = z3;
            //             fragment.colour = colour;
            //
            //             fragmentQueue.push_back(fragment);
            //         }
            //     }
            // }
        }
    } // RasteriseLineSegment()

// rasterises a single triangle
void FakeGL::RasteriseTriangle(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1, screenVertexWithAttributes &vertex2)
    { // RasteriseTriangle()
    // compute a bounding box that starts inverted to frame size
    // clipping will happen in the raster loop proper
    float minX = frameBuffer.width, maxX = 0.0;
    float minY = frameBuffer.height, maxY = 0.0;

    // test against all vertices
    if (vertex0.position.x < minX) minX = vertex0.position.x;
    if (vertex0.position.x > maxX) maxX = vertex0.position.x;
    if (vertex0.position.y < minY) minY = vertex0.position.y;
    if (vertex0.position.y > maxY) maxY = vertex0.position.y;

    if (vertex1.position.x < minX) minX = vertex1.position.x;
    if (vertex1.position.x > maxX) maxX = vertex1.position.x;
    if (vertex1.position.y < minY) minY = vertex1.position.y;
    if (vertex1.position.y > maxY) maxY = vertex1.position.y;

    if (vertex2.position.x < minX) minX = vertex2.position.x;
    if (vertex2.position.x > maxX) maxX = vertex2.position.x;
    if (vertex2.position.y < minY) minY = vertex2.position.y;
    if (vertex2.position.y > maxY) maxY = vertex2.position.y;


    // now for each side of the triangle, compute the line vectors
    Cartesian3 vector01 = vertex1.position - vertex0.position;
    Cartesian3 vector12 = vertex2.position - vertex1.position;
    Cartesian3 vector20 = vertex0.position - vertex2.position;

    // now compute the line normal vectors
    Cartesian3 normal01(-vector01.y, vector01.x, 0.0);
    Cartesian3 normal12(-vector12.y, vector12.x, 0.0);
    Cartesian3 normal20(-vector20.y, vector20.x, 0.0);

    // we don't need to normalise them, because the square roots will cancel out in the barycentric coordinates
    float lineConstant01 = normal01.dot(vertex0.position);
    float lineConstant12 = normal12.dot(vertex1.position);
    float lineConstant20 = normal20.dot(vertex2.position);

    // and compute the distance of each vertex from the opposing side
    float distance0 = normal12.dot(vertex0.position) - lineConstant12;
    float distance1 = normal20.dot(vertex1.position) - lineConstant20;
    float distance2 = normal01.dot(vertex2.position) - lineConstant01;

    // if any of these are zero, we will have a divide by zero error
    // but notice that if they are zero, the vertices are collinear in projection and the triangle is edge on
    // we can render that as a line, but the better solution is to render nothing.  In a surface, the adjacent
    // triangles will eventually take care of it
    if ((distance0 == 0) || (distance1 == 0) || (distance2 == 0))
        return;

    // create a fragment for reuse
    fragmentWithAttributes rasterFragment;

    // set triangle depthBuffer based on the depth of the triangels center point
    float triangleDepth = (vertex0.position.z+vertex1.position.z+vertex2.position.z)/3;
    // std::cout << "Triangle Z:" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << "triangleDepth: " << triangleDepth << std::endl;
    std::cout <<  vertex0 << std::endl;
    std::cout <<  vertex1 << std::endl;
    std::cout <<  vertex2 << std::endl;
    // std::cout << rasterFragment.z << std::endl;

    // loop through the pixels in the bounding box
    for (rasterFragment.row = minY; rasterFragment.row <= maxY; rasterFragment.row++)
        { // per row
        // this is here so that clipping works correctly
        if (rasterFragment.row < 0) continue;
        if (rasterFragment.row >= frameBuffer.height) continue;
        for (rasterFragment.col = minX; rasterFragment.col <= maxX; rasterFragment.col++)
            { // per pixel
            // this is also for correct clipping
            if (rasterFragment.col < 0) continue;
            if (rasterFragment.col >= frameBuffer.width) continue;

            // the pixel in cartesian format
            Cartesian3 pixel(rasterFragment.col, rasterFragment.row, 0.0);

            // right - we have a pixel inside the frame buffer AND the bounding box
            // note we *COULD* compute gamma = 1.0 - alpha - beta instead
            float alpha = (normal12.dot(pixel) - lineConstant12) / distance0;
            float beta = (normal20.dot(pixel) - lineConstant20) / distance1;
            float gamma = (normal01.dot(pixel) - lineConstant01) / distance2;

            // now perform the half-plane test
            if ((alpha < 0.0) || (beta < 0.0) || (gamma < 0.0))
                continue;

            // compute colour
            rasterFragment.colour = alpha * vertex0.colour + beta * vertex1.colour + gamma * vertex2.colour;

            // set triangle depth
            rasterFragment.z = triangleDepth;

            // now we add it to the queue for fragment processing
            fragmentQueue.push_back(rasterFragment);
            } // per pixel
        } // per row
    } // RasteriseTriangle()

// process a single fragment
void FakeGL::ProcessFragment()
    { // ProcessFragment()

        // Get current fragment form the fragment queue
        fragmentWithAttributes currentFragment = fragmentQueue.front();
        fragmentQueue.pop_front();  // Pop the fragment off the queue



        /* ==== Do Depth Buffering ==== */
        // Note: depthBuffer is a second RGBAImage in which the alpha stores
        // the depth buffer

        // As the alpha values are clamped between 0 & 255 half all buffers and
        // add 128 so that it sits within these bounds with headroom
        currentFragment.z = (currentFragment.z*128)+ 128;


        // For each fragment,
        // IF fragment Z < the depthBuffer depth && enableDepthTest = true THEN
        if ((currentFragment.z < (float)depthBuffer[currentFragment.row][currentFragment.col].alpha) && enableDepthTest)
        {
            // Discard the fragment
            // std::cout <<  "Discard Fragment: " << currentFragment.z  << std::endl;

            return;
        }
            else
        {
            // Colour the position in the frameBuffer
            frameBuffer[currentFragment.row][currentFragment.col] = currentFragment.colour;

            // Set the new depthBuffer
            RGBAValue bufferColour;
            bufferColour.alpha = currentFragment.z;
            depthBuffer[currentFragment.row][currentFragment.col] = bufferColour;

        }



    } // ProcessFragment()

// standard routine for dumping the entire FakeGL context (except for texture / image)
std::ostream &operator << (std::ostream &outStream, FakeGL &fakeGL)
    { // operator <<
    outStream << "=========================" << std::endl;
    outStream << "Dumping FakeGL Context   " << std::endl;
    outStream << "=========================" << std::endl;


    outStream << "-------------------------" << std::endl;
    outStream << "Vertex Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.vertexQueue.begin(); vertex < fakeGL.vertexQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.vertexQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Raster Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.rasterQueue.begin(); vertex < fakeGL.rasterQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.rasterQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Fragment Queue:          " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto fragment = fakeGL.fragmentQueue.begin(); fragment < fakeGL.fragmentQueue.end(); fragment++)
        { // per matrix
        outStream << "Fragment " << fragment - fakeGL.fragmentQueue.begin() << std::endl;
        outStream << *fragment;
        } // per matrix


    return outStream;
    } // operator <<

// subroutines for other classes
std::ostream &operator << (std::ostream &outStream, vertexWithAttributes &vertex)
    { // operator <<
    std::cout << "Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

	// you

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, screenVertexWithAttributes &vertex)
    { // operator <<
    std::cout << "Screen Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, fragmentWithAttributes &fragment)
    { // operator <<
    std::cout << "Fragment With Attributes" << std::endl;
    std::cout << "Row:        " << fragment.row << std::endl;
    std::cout << "Col:        " << fragment.col << std::endl;
    std::cout << "Colour:     " << fragment.colour << std::endl;

    return outStream;
    } // operator <<
