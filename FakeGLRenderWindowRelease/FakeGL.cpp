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
            whichMatrix
                Specifies withc matrix stack is the target of subsequent matrix
                operations. Three values are accepted: GL_MODELVIEW, GL_PROJECTION
                and GL_TEXTURE. The initial value is GL_MODELVIEW.

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

        switch (matrixState)
        {
            // Applies subsequent matrix operations to the modelview matrix stack
            case FAKEGL_MODELVIEW:
                matrixStackModelview.push_front(matrixCurrent);
                break;

            // Applies subsequent matrix operations to the projection matrix stack
            case FAKEGL_PROJECTION:
                matrixStackProjection.push_front(matrixCurrent);
                break;
        }

    } // PushMatrix()

// pops a matrix off the stack
void FakeGL::PopMatrix()
    { // PopMatrix()

        switch (matrixState)
        {
            case FAKEGL_MODELVIEW:
                if (sizeof(matrixStackModelview)  > 0 )
                {
                    matrixCurrent = matrixStackModelview.front();
                    matrixStackModelview.pop_front();
                }
                break;

            case FAKEGL_PROJECTION:
                if (sizeof(matrixStackProjection)  > 0 )
                {
                    matrixCurrent = matrixStackProjection.front();
                    matrixStackProjection.pop_front();
                }
                break;
        }

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
        matrixCurrent = matrixMultMatrix*matrixCurrent;
        std::cout << matrixCurrent << std::endl;

    } // MultMatrixf()

// sets up a perspective projection matrix
void FakeGL::Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Frustum()
    /*
    PARAMETERS

           left, right Specify  the  coordinates  for  the left and right vertical
                       clipping planes.

           bottom, top Specify the coordinates for the bottom and  top  horizontal
                       clipping planes.

           zNear, zFar Specify  the  distances  to the near and far depth clipping
                       planes.  Both distances must be positive.


    DESCRIPTION

           glFrustum describes a perspective matrix that  produces  a  perspective
           projection.   The  current  matrix  (see glMatrixMode) is multiplied by
           this  matrix  and  the  result  replaces  the  current  matrix,  as  if
           glMultMatrix were called with the following matrix as its argument:


              2 zNear
            ------------       0              A              0
            right - left

                            2 zNear
                0         ------------        B              0
                          top - bottom

                0              0              C              D


                0              0              -1             0

                            A = (right + left) / (right - left)
                            B = (top + bottom) / (top - bottom)
                            C = - (zFar + zNear) / (zFar - zNear)
                            D = - (2 zFar zNear) / (zFar - zNear)

           Typically, the matrix mode is GL_PROJECTION, and (left, bottom, -zNear)
           and (right, top,  -zNear) specify the points on the near clipping plane
           that  are  mapped to the lower left and upper right corners of the win-
           dow, assuming that the eye is located at (0, 0,  0).   -zFar  specifies
           the  location  of  the far clipping plane.  Both zNear and zFar must be
           positive.

           Use glPushMatrix and glPopMatrix to save and restore the current matrix
           stack.
    */

    float A = (right + left) / (right - left);
    float B = (top + bottom) / (top - bottom);
    float C = - (zFar + zNear) / (zFar - zNear);
    float D = - (2*zFar*zNear) / (zFar - zNear);

    matrixFrustum[0][0] = (2*zNear)/(right-left); matrixFrustum[1][0] = 0;                      matrixFrustum[2][0] = A;  matrixFrustum[3][0] = 0;
    matrixFrustum[0][1] = 0;                      matrixFrustum[1][1] = (2*zNear)/(top-bottom); matrixFrustum[2][1] = B;  matrixFrustum[3][1] = 0;
    matrixFrustum[0][2] = 0;                      matrixFrustum[1][2] = 0;                      matrixFrustum[2][2] = C;  matrixFrustum[3][2] = D;
    matrixFrustum[0][3] = 0;                      matrixFrustum[1][3] = 0;                      matrixFrustum[2][3] = -1; matrixFrustum[3][3] = 0;

    matrixCurrent = matrixFrustum;

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
        float tx = - (right + left) / (right - left);
        float ty = - (top + bottom) / (top - bottom);
        float tz = - (zFar + zNear) / (zFar - zNear);

        matrixOrtho[0][0] = (2/(right-left)); matrixOrtho[1][0] = 0;                matrixOrtho[2][0] = 0;                 matrixOrtho[3][0] = tx;
        matrixOrtho[0][1] = 0;                matrixOrtho[1][1] = (2/(top-bottom)); matrixOrtho[2][1] = 0;                 matrixOrtho[3][1] = ty;
        matrixOrtho[0][2] = 0;                matrixOrtho[1][2] = 0;                matrixOrtho[2][2] = (-1/(zFar/zNear)); matrixOrtho[3][2] = tz;
        matrixOrtho[0][3] = 0;                matrixOrtho[1][3] = 0;                matrixOrtho[2][3] = 0;                 matrixOrtho[3][3] = 1;

        matrixCurrent = matrixOrtho;

    } // Ortho()

// rotate the matrix
void FakeGL::Rotatef(float angle, float axisX, float axisY, float axisZ)
    { // Rotatef()

        /*
            PARAMETERS
               angle
                   Specifies the angle of rotation, in degrees.

               x, y, z
                   Specify the x, y, and z coordinates of a vector, respectively.

            DESCRIPTION
               glRotate produces a rotation of angle degrees around the vector x y z.
               The current matrix (see glMatrixMode()) is multiplied by a rotation
               matrix with the product replacing the current matrix.

               If the matrix mode is either GL_MODELVIEW or GL_PROJECTION, all objects
               drawn after glRotate is called are rotated. Use glPushMatrix() and
               glPopMatrix() to save and restore the unrotated coordinate system.

            NOTES
               This rotation follows the right-hand rule, so if the vector x y z
               points toward the user, the rotation will be counterclockwise.
        */

        std::cout << "===---ROTATE---===" << std::endl;

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

        /*
            parameterName
                The single-valued material parameter of the face or faces being
                updated. Must be GL_SHININESS.
        */

        switch(parameterName){

            case FAKEGL_SHININESS:
                /*
                    params is a single integer or floating-point value that specifies the RGBA specular
                    exponent of the material. Integer and floating-point values are mapped directly.
                    Only values in the range 0 128 are accepted. The initial specular exponent for both
                    front- and back-facing materials is 0.
                */
                materialShininessValue = parameterValue;
                break;


        }


    } // Materialf()

void FakeGL::Materialfv(unsigned int parameterName, const float *parameterValues)
    { // Materialfv()


        // std::cout << "=====MATERIAL=====" << parameterName << std::endl;


        switch(parameterName){
            case FAKEGL_AMBIENT:
                // std::cout << "FAKEGL_AMBIENT" << std::endl;
                /*
                    params contains four integer or floating-point values that specify the ambient RGBA
                    reflectance of the material. Integer values are mapped linearly such that the most
                    positive representable value maps to 1.0, and the most negative representable value
                    maps to -1.0. Floating-point values are mapped directly. Neither integer nor
                    floating-point values are clamped. The initial ambient reflectance for both front-
                    and back-facing materials is (0.2, 0.2, 0.2, 1.0).
                */
                for( unsigned int n = 0; n < 4; n++ ) materialAmbientValues[n] = parameterValues[n];
                break;
            case FAKEGL_DIFFUSE:
                // std::cout << "FAKEGL_DIFFUSE" << std::endl;
                /*
                    params contains four integer or floating-point values that specify the diffuse RGBA
                    reflectance of the material. Integer values are mapped linearly such that the most
                    positive representable value maps to 1.0, and the most negative representable value
                    maps to -1.0. Floating-point values are mapped directly. Neither integer nor
                    floating-point values are clamped. The initial diffuse reflectance for both front-
                    and back-facing materials is (0.8, 0.8, 0.8, 1.0).
                */
                for( unsigned int n = 0; n < 4; n++ ) materialDiffuseValues[n] = parameterValues[n];
                break;
            case FAKEGL_AMBIENT_AND_DIFFUSE:
                // std::cout << "FAKEGL_AMBIENT_AND_DIFFUSE" << std::endl;
                for( unsigned int n = 0; n < 4; n++ ) materialAmbientValues[n] = parameterValues[n];
                for( unsigned int n = 0; n < 4; n++ ) materialDiffuseValues[n] = parameterValues[n];

                break;
            case FAKEGL_SPECULAR:
                // std::cout << "FAKEGL_SPECULAR" << std::endl;
                /*
                    params contains four integer or floating-point values that specify the specular RGBA
                    reflectance of the material. Integer values are mapped linearly such that the most
                    positive representable value maps to 1.0, and the most negative representable value
                    maps to -1.0. Floating-point values are mapped directly. Neither integer nor
                    floating-point values are clamped. The initial specular reflectance for both front-
                    and back-facing materials is (0, 0, 0, 1).
                */
                for( unsigned int n = 0; n < 4; n++ ) materialSpecularValues[n] = parameterValues[n];
                break;
            case FAKEGL_EMISSION:
                // std::cout << "FAKEGL_EMISSION" << std::endl;
                /*
                    params contains four integer or floating-point values that specify the RGBA emitted
                    light intensity of the material. Integer values are mapped linearly such that the
                    most positive representable value maps to 1.0, and the most negative representable
                    value maps to -1.0. Floating-point values are mapped directly. Neither integer nor
                    floating-point values are clamped. The initial emission intensity for both front-
                    and back-facing materials is (0, 0, 0, 1).

                */
                for( unsigned int n = 0; n < 4; n++ ) materialEmissionValues[n] = parameterValues[n];
                break;

            case FAKEGL_SHININESS:
                // std::cout << "FAKEGL_SHININESS" << std::endl;
                /*
                   params is a single integer or floating-point value that specifies the RGBA specular
                   exponent of the material. Integer and floating-point values are mapped directly.
                   Only values in the range 0 128 are accepted. The initial specular exponent for both
                   front- and back-facing materials is 0.
                */
                materialShininessValue = parameterValues[0];
                break;
        }
        // for( unsigned int n = 0; n < 4; n++ ) std::cout << parameterValues[n] << std::endl;


    } // Materialfv()

// sets the normal vector
void FakeGL::Normal3f(float x, float y, float z)
    { // Normal3f()
        /*
        PARAMETERS
            nx, ny, nz
                Specify the x, y, and z coordinates of the new current normal. The
                initial value of the current normal is the unit vector, (0, 0, 1).

        DESCRIPTION
            The current normal is set to the given coordinates whenever glNormal is
            issued. Byte, short, or integer arguments are converted to
            floating-point format with a linear mapping that maps the most positive
            representable integer value to 1.0 and the most negative representable
            integer value to -1.0.
        */

    normalVector = {x, y, z};

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

            case FAKEGL_PHONG_SHADING:
                enablePhongShading = false;
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

            case FAKEGL_PHONG_SHADING:
                enablePhongShading = true;
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
            parameterName
                Specifies a single-valued light source parameter for light.

            parameterValues
                Specifies a pointer to the value or values that parameter pname of
                light source light will be set to.
          */


        switch(parameterName)
        {
            case FAKEGL_POSITION:
                // std::cout << "------------------------------" << std::endl;
                // std::cout << "FAKEGL_POSITION" << std::endl;
                /*
                    params contains four integer or floating-point values that specify
                    the position of the light in homogeneous object coordinates. Both
                    integer and floating-point values are mapped directly. Neither
                    integer nor floating-point values are clamped.

                    The position is transformed by the modelview matrix when glLight is
                    called (just as if it were a point), and it is stored in eye
                    coordinates.
                */
                matrixModelview = matrixStackModelview.front();    // Modelview matrix

                // std::cout << "Light Params:" << std::endl;
                // std::cout << parameterValues[0]<< ", " << parameterValues[1]<< ", " << parameterValues[2]<< ", " << parameterValues[3] << std::endl;
                std::cout << "\nModelview Matrix:" << std::endl;
                std::cout << matrixCurrent << std::endl;

                // Apply matrix to light position
                lightPositionValues[0] = (parameterValues[0]*matrixCurrent[0][0]) + (parameterValues[1]*matrixCurrent[1][0]) + (parameterValues[2]*matrixCurrent[2][0]) + (parameterValues[3]*matrixCurrent[3][0]);
                lightPositionValues[1] = (parameterValues[0]*matrixCurrent[0][1]) + (parameterValues[1]*matrixCurrent[1][1]) + (parameterValues[2]*matrixCurrent[2][1]) + (parameterValues[3]*matrixCurrent[3][1]);
                lightPositionValues[2] = (parameterValues[0]*matrixCurrent[0][2]) + (parameterValues[1]*matrixCurrent[1][2]) + (parameterValues[2]*matrixCurrent[2][2]) + (parameterValues[3]*matrixCurrent[3][2]);
                lightPositionValues[3] = (parameterValues[0]*matrixCurrent[0][3]) + (parameterValues[1]*matrixCurrent[1][3]) + (parameterValues[2]*matrixCurrent[2][3]) + (parameterValues[3]*matrixCurrent[3][3]);

                /*
                    If the w component of the position is 0, the light is
                    treated as a directional source. Diffuse and specular lighting
                    calculations take the light's direction, but not its actual
                    position, into account, and attenuation is disabled. Otherwise,
                    diffuse and specular lighting calculations are based on the actual
                    location of the light in eye coordinates, and attenuation is
                    enabled. The initial position is (0, 0, 1, 0); thus, the initial
                    light source is directional, parallel to, and in the direction of
                    the - z axis
                */

                // std::cout << "\nPost Matrix Calculation: " << std::endl;
                // std::cout << lightPositionValues[0]<< ", " << lightPositionValues[1]<< ", " << lightPositionValues[2]<< ", " << lightPositionValues[3] << std::endl;

                // Test the W component of the light position
                // If it is 0, then you treat what remains as the vector.
                if (lightPositionValues[3] == 0){
                    // That is your vector to the light
                }else{
                    /*
                        If it's 1, you treat it as a position and  you subtract
                        the vertex from it to find the actual vector

                        You have to convert it to cartesian (which is divide
                        through by lw)

                        The position of the light in space (vl):
                            x = x/w
                            y = y/w
                            z = z/w
                    */

                    // This is your vl
                    lightPositionValues[0] = lightPositionValues[0]/lightPositionValues[3];
                    lightPositionValues[1] = lightPositionValues[1]/lightPositionValues[3];
                    lightPositionValues[2] = lightPositionValues[2]/lightPositionValues[3];

                    /*
                        Once, you've got vl (vector from the surface to the
                        light), and n (the normal of a vertex) and they're both
                        units (length one), you take the dot product...

                        Or you take the dot product, then divide through by the
                        lights.
                    */
                }
                // std::cout << "\nPost W Check: " << std::endl;
                // std::cout << lightPositionValues[0]<< ", " << lightPositionValues[1]<< ", " << lightPositionValues[2]<< ", " << lightPositionValues[3] << std::endl;

                break;

            case FAKEGL_AMBIENT:
                // std::cout << "FAKEGL_AMBIENT" << std::endl;
                for( unsigned int n = 0; n < 4; n++ ) lightAmbientValues[n] = parameterValues[n];
                break;

            case FAKEGL_DIFFUSE:
                /*
                    params contains four integer or floating-point values that specify
                    the diffuse RGBA intensity of the light. Integer values are mapped
                    linearly such that the most positive representable value maps to
                    1.0, and the most negative representable value maps to -1.0.
                    Floating-point values are mapped directly.
                */
                // std::cout << "FAKEGL_DIFFUSE" << std::endl;
                for( unsigned int n = 0; n < 4; n++ ) lightDiffuseValues[n] = parameterValues[n];
                break;
            case FAKEGL_AMBIENT_AND_DIFFUSE:
                // std::cout << "FAKEGL_AMBIENT_AND_DIFFUSE" << std::endl;
                /*
                    The equivalent to calling glLight twice with the same
                    parameter values. Once with GL_AMBIENT and once with
                    GL_DIFFUSE.
                */
                for( unsigned int n = 0; n < 4; n++ ) lightAmbientValues[n] = parameterValues[n];
                for( unsigned int n = 0; n < 4; n++ ) lightDiffuseValues[n] = parameterValues[n];
                break;
            case FAKEGL_SPECULAR:
                // std::cout << "FAKEGL_SPECULAR" << std::endl;
                /*
                    params contains four integer or floating-point values that specify the specular RGBA
                    intensity of the light. Integer values are mapped linearly such that the most
                    positive representable value maps to 1.0, and the most negative representable value
                    maps to -1.0. Floating-point values are mapped directly. Neither integer nor
                    floating-point values are clamped. The initial value for GL_LIGHT0 is (1, 1, 1, 1);
                    for other lights, the initial value is (0, 0, 0, 1).
                */
                /*
                    The specular term has the same pattern as GL_POSITION

                    Specular Light * Specular Reflection

                    And multiply by in this case, n dot bisector vector
                */
                for( unsigned int n = 0; n < 4; n++ ) lightSpecularValues[n] = parameterValues[n];
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

                // For each pixel, set the z buffer to -infinity
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
        RGBAValue vertexColour;             // Colour of current vertex

        // Apply the current matrix to the vertex
        vertexTransformed.position.x = (vertexCurrent.position.x*matrixCurrent[0][0]) + (vertexCurrent.position.y*matrixCurrent[1][0]) + (vertexCurrent.position.z*matrixCurrent[2][0]) + (vertexCurrent.position.w*matrixCurrent[3][0]);
        vertexTransformed.position.y = (vertexCurrent.position.x*matrixCurrent[0][1]) + (vertexCurrent.position.y*matrixCurrent[1][1]) + (vertexCurrent.position.z*matrixCurrent[2][1]) + (vertexCurrent.position.w*matrixCurrent[3][1]);
        vertexTransformed.position.z = (vertexCurrent.position.x*matrixCurrent[0][2]) + (vertexCurrent.position.y*matrixCurrent[1][2]) + (vertexCurrent.position.z*matrixCurrent[2][2]) + (vertexCurrent.position.w*matrixCurrent[3][2]);
        vertexTransformed.position.w = (vertexCurrent.position.x*matrixCurrent[0][3]) + (vertexCurrent.position.y*matrixCurrent[1][3]) + (vertexCurrent.position.z*matrixCurrent[2][3]) + (vertexCurrent.position.w*matrixCurrent[3][3]);


        // COMPUTE LIGHT
        if (enableLighting)
        {
            // std::cout << "=========================================" << std::endl;
            // std::cout << " ---COMPUTE LIGHT---" << std::endl;

            // store RGBA values of light as they are computed
            // RGBAValue lightColour;
            float r=0,g=0,b=0,a=0;

            // Compute light, and the lighting to each vertex of the mesh
            // This tells us what colour that vertex is

            // Transform the model normal's orientation into eye space


            // Get distance between the light and the model vertex (used for attenuation)

            // Calculate the dot product of the light vector and vertex normal.
            // If the normal and light vector are pointing in the same direction then
            // it will get max illumination.

            // Attenuate the light based on the distance

            // Multiply the colour by the illumination level.


            // COMBINE THE ATTRIBUTES OF THE LIGHT WITH THE ATTRIBUTES OF THE SURFACE

            /*  === SET EMISSIVE COLOUR ===
                l(emitted)

                Simply add on the emission values
            */
            // std::cout << "materialEmissionValues:  (" << materialEmissionValues[0] << ", " << materialEmissionValues[1] << ", " << materialEmissionValues[2] << ", " << materialEmissionValues[3] << ")" << std::endl;

            r += materialEmissionValues[0];
            g += materialEmissionValues[1];
            b += materialEmissionValues[2];
            a += materialEmissionValues[3];

            /*  === SET AMBIENT COLOUR ===
                l(ambient) * r(ambient)

                Take the ambient, and multiply it by the colour of the light
                source (modulate).
            */
            r += lightAmbientValues[0] * materialAmbientValues[0];
            g += lightAmbientValues[1] * materialAmbientValues[1];
            b += lightAmbientValues[2] * materialAmbientValues[2];
            a += lightAmbientValues[3] * materialAmbientValues[3];

            /*  === SET DIFFUSE COLOUR ==
                                              (n DOT vl)        <--- Dot product
                l(diffuse) * r(diffuse) * -----------------          of n & vl
                                            ||n||  ||vl||       <--- Divide through
                                                                     by unit normals

                WHERE:
                    n  = The normal of the vertex
                    vl = A vector form the surface to the positon of the light

                    Once, we have vl, and n (the normal of a vertex) and
                    they're both units (length one), you take the dot product...

                Because diffuse depends on the angle you have to add the
                term for the angle into it.

                CALCULATE VECTOR:
                    ->
                    PQ  = (xQ-xP, yQ-yP, zQ-zP)

                If you want a vector from a vertex on the surface to the
                position of the light in space you subrtact the tail from
                the head (The tail is the vertex on the surface, the head
                is the light source, because you want the vector that goes
                out, ie. the direction of the light source).
            */

            // compute n (vector from a vertex on the surface to the position
            // of the light in space)
            vector<float> n = {
                                lightPositionValues[0]-vertexTransformed.position.x,
                                lightPositionValues[1]-vertexTransformed.position.y,
                                lightPositionValues[2]-vertexTransformed.position.z
                              };
            // compute the base normal of n
            float nBaseNormal = sqrt( pow(n[0],2) + pow(n[1],2) + pow(n[2],2) );

            // set light position values as vector3
            vector<float> vl = { lightPositionValues[0], lightPositionValues[1], lightPositionValues[2]};
            // std::cout << "vl: (" << vl[0] << ", " << vl[1] << ", " << vl[2] << ")" << std::endl;
            // std::cout << "n:  (" << n[0]  << ", " << n[1]  << ", " << n[2]  << ")" << std::endl;

            // compute the base normal of vl
            float vlBaseNormal = sqrt( pow(vl[0],2) + pow(vl[1],2) + pow(vl[2],2) );

            // normalise n & vl
            vector<float> nNormalised  = {(n[0]  / nBaseNormal ), (n[1]  / nBaseNormal ), (n[2]  / nBaseNormal) };
            vector<float> vlNormalised = {(vl[0] / vlBaseNormal), (vl[1] / vlBaseNormal), (vl[2] / vlBaseNormal)};
            // std::cout << "vlNormalised: (" << vlNormalised[0] << ", " << vlNormalised[1] << ", " << vlNormalised[2] << ")" << std::endl;
            // std::cout << "nNormalised:  (" << nNormalised[0]  << ", " << nNormalised[1]  << ", " << nNormalised[2]  << ")" << std::endl;

            // compute dot product n & vl
            float diffuseAngle = 1 - ( (nNormalised[0]*vlNormalised[0]) + (nNormalised[1]*vlNormalised[1]) + (nNormalised[2]*vlNormalised[2]));
            // float diffuseAngle = -1 * ((n[0]*vl[0]) + (n[1]*vl[1]) + (n[2]*vl[2]));
            // std::cout << "diffuseAngle: " << diffuseAngle << std::endl;
            if (diffuseAngle < 0) diffuseAngle = 0;

            r += lightDiffuseValues[0] * materialDiffuseValues[0] * diffuseAngle;
            g += lightDiffuseValues[1] * materialDiffuseValues[1] * diffuseAngle;
            b += lightDiffuseValues[2] * materialDiffuseValues[2] * diffuseAngle;
            a += lightDiffuseValues[3] * materialDiffuseValues[3] * diffuseAngle;

            // Convert to colour
            r *= 255;
            g *= 255;
            b *= 255;
            a *= 255;

            // clamp colour
            if (r>255) r=255;
            if (g>255) g=255;
            if (b>255) b=255;
            if (a>255) a=255;

            // Normalize the the combined attributes and set the vertex colour
            vertexColour.red    = r;    // THIS IS JUST OUR LIGHT!!! WE NEED TO NOT SET IT TO THE VERTEX HERE! INSTEAD WE SHOULD HAVE A SEPERATE BIT OF CODE LATER DOWN THE LINE TO TAKE THE VERTEX COLOUR, TAKE THE TEXTURE,
            vertexColour.green  = g;    // TAKE THE LIGHT, AND THEN MODULATE IT ALL AS NEEDED.. BUT DEPENDING ON ORDER OF OPERATION SETTIGN LIGHT HERE MAY BE FINE TOO.
            vertexColour.blue   = b;
            vertexColour.alpha  = a;
            // Following this use the barycentric interpolation to interpolate across
        }else{

            // No lighting, so simply set the vertex colour
            vertexColour = colour3;
        }



        // COMPUTE TEXTURE  **IMPLIMENT LIGHTING FIRST
        if (enableTexture2D)
        {
            // Compute texture coordinates for each vertex in the mesh

            // Then use barycentric interpolation to interpolate between
            // and find all the other texture coordinates.
        }

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

        // Store the vertex Z position for Z sorting
        vertexDeviceCoordinate.position.z  = vertexTransformed.position.z;

        // Get color data from the vertex
        vertexDeviceCoordinate.colour      = vertexColour;
        rasterQueue.push_back(vertexDeviceCoordinate);

        // If primitive to rasterise is found, process fragments in queue
        if (RasterisePrimitive()){

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
                    RasteriseLineSegment(dcs0, dcs1);

                    return true;
                }
                break;

            // Treats each triplet of verticies as an independent triangle.
            // Verticies $3n^-^2$, $3n^-^1$, and $3n$ define triangle $n$.
            // $N/3$ triangles are drawn.
            case FAKEGL_TRIANGLES:  // Cheack number of vertices on the queue
                if (rasterQueue.size() == 3){

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

        // Assign verticies shorthand variables
        float x1 = vertex0.position.x; // Axis in width
        float y1 = vertex0.position.y; // Axis in height
        float z1 = vertex0.position.z; // Z Depth

        float x2 = vertex1.position.x; // Axis in width
        float y2 = vertex1.position.y; // Axis in height
        float z2 = vertex1.position.z; // Z Depth

        int r1 = vertex0.colour.red;
        int g1 = vertex0.colour.green;
        int b1 = vertex0.colour.blue;

        int r2 = vertex1.colour.red;
        int g2 = vertex1.colour.green;
        int b2 = vertex1.colour.blue;

        // // Calculate length of line in 3D space
        // float lineLength;

        float x = 0, y = 0;
        float xDif=x1-x2,yDif=y1-y2;
        float steps = sqrt(pow(x2-x1,2)+pow(y2-y1,2));   // Distance between points
        float m = (y2 - y1) / (x2 - x1);                 // Gradiant

        for (float i = 0; i < steps; i++) {

            if (m){ // Line is Diagonal or Vertical
                y = (yDif*(i/steps)) + y2;
                x = ((yDif*(i/steps)) / m) + x2;
            } else { // Line is Horizontal
                y = y1;
                x = xDif*(i/steps) + x2;
            }

            /*
                In order to impliment the line width, we take the gradient of
                the line being drawn, and use this to decide the axis in which
                we draw the line width.

                If graidant(m)<-1 or graidant(m)>1
                    draw line width in the x
                else
                    draw line width in the y
            */
            int lineWidthX = 1; // De
            int lineWidthY = 1; // De

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

            // Similar to rasterise point, however, instead of drawing a square
            // (iterating over X AND Y) to get a point of size, this time we
            // only iterate over either the X OR Y (depending on gradient) to
            // get a line of width.

            int lineWidthH = lineWidth*.5;

            for (int xOffset = 0; xOffset < lineWidthX; xOffset++)
            {
                for (int yOffset = 0; yOffset < lineWidthY; yOffset++)
                {
                    // Convert point to point in the line
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

            // USE BARYCENTRIC COORDINATES TO CALCULATE Z VALUE
            // set triangle depthBuffer based on the depth of the triangels center point
            float triangleDepth =  alpha * vertex0.position.z + beta * vertex1.position.z + gamma * vertex2.position.z;

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


        // ==== Do Depth Buffering ====
        // Note: depthBuffer is a second RGBAImage in which the alpha stores
        // the depth buffer

        // As the alpha values are clamped between 0 & 255 multiply through by
        // 127 and add by 128 so that it sits within these bounds.
        currentFragment.z = (currentFragment.z*127)+ 128;

        // For each fragment,
        // IF fragment Z < the depthBuffer depth && enableDepthTest = true THEN
        if ((currentFragment.z < (float)depthBuffer[currentFragment.row][currentFragment.col].alpha) && enableDepthTest)
        {
            // Discard the fragment
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
