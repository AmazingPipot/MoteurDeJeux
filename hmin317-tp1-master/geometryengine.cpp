/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
/*******************/
#include "geometryengine.h"

#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <cstdlib>
#include <iostream>


const int M = 16;
const int N = 16;
int nb = 0;
float heightMap[M][N];

struct VertexData
{
    QVector3D position;
    QVector2D texCoord;
    QVector4D normal;

};

//! [0]
GeometryEngine::GeometryEngine(float posX, float posY)
    : indexBuf(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();

    // Generate 2 VBOs
    arrayBuf.create();
    indexBuf.create();

    // Initializes cube geometry and transfers it to VBOs
    //initCubeGeometry();
    //initPlaneGeometry();
    initMapQuadTree(posX,posY);

    //QObject::connect(timer, SIGNAL(eventCatch()), this, SLOT(pivoter()) );
}

GeometryEngine::~GeometryEngine()
{
    arrayBuf.destroy();
    indexBuf.destroy();
}
//! [0]

void GeometryEngine::initCubeGeometry()
{
    // For cube we would need only 8 vertices but we have to
    // duplicate vertex for each face because texture coordinate
    // is different.
    VertexData vertices[] = {
        // Vertex data for face 0
        {QVector3D(-1.0f, -1.0f,  1.0f), QVector2D(0.0f, 0.0f)},  // v0
        {QVector3D( 1.0f, -1.0f,  1.0f), QVector2D(0.33f, 0.0f)}, // v1
        {QVector3D(-1.0f,  1.0f,  1.0f), QVector2D(0.0f, 0.5f)},  // v2
        {QVector3D( 1.0f,  1.0f,  1.0f), QVector2D(0.33f, 0.5f)}, // v3

        // Vertex data for face 1
        {QVector3D( 1.0f, -1.0f,  1.0f), QVector2D( 0.0f, 0.5f)}, // v4
        {QVector3D( 1.0f, -1.0f, -1.0f),QVector2D(0.33f, 0.5f)}, // v5
        {QVector3D( 1.0f,  1.0f,  1.0f), QVector2D(0.0f, 1.0f)},  // v6
        {QVector3D( 1.0f,  1.0f, -1.0f), QVector2D(0.33f, 1.0f)}, // v7

        // Vertex data for face 2
        {QVector3D( 1.0f, -1.0f, -1.0f), QVector2D(0.66f, 0.5f)}, // v8
        {QVector3D(-1.0f, -1.0f, -1.0f), QVector2D(1.0f, 0.5f)},  // v9
        {QVector3D( 1.0f,  1.0f, -1.0f), QVector2D(0.66f, 1.0f)}, // v10
        {QVector3D(-1.0f,  1.0f, -1.0f), QVector2D(1.0f, 1.0f)},  // v11

        // Vertex data for face 3
        {QVector3D(-1.0f, -1.0f, -1.0f), QVector2D(0.66f, 0.0f)}, // v12
        {QVector3D(-1.0f, -1.0f,  1.0f), QVector2D(1.0f, 0.0f)},  // v13
        {QVector3D(-1.0f,  1.0f, -1.0f), QVector2D(0.66f, 0.5f)}, // v14
        {QVector3D(-1.0f,  1.0f,  1.0f), QVector2D(1.0f, 0.5f)},  // v15

        // Vertex data for face 4
        {QVector3D(-1.0f, -1.0f, -1.0f), QVector2D(0.33f, 0.0f)}, // v16
        {QVector3D( 1.0f, -1.0f, -1.0f), QVector2D(0.66f, 0.0f)}, // v17
        {QVector3D(-1.0f, -1.0f,  1.0f), QVector2D(0.33f, 0.5f)}, // v18
        {QVector3D( 1.0f, -1.0f,  1.0f), QVector2D(0.66f, 0.5f)}, // v19

        // Vertex data for face 5
        {QVector3D(-1.0f,  1.0f,  1.0f), QVector2D(0.33f, 0.5f)}, // v20
        {QVector3D( 1.0f,  1.0f,  1.0f), QVector2D(0.66f, 0.5f)}, // v21
        {QVector3D(-1.0f,  1.0f, -1.0f), QVector2D(0.33f, 1.0f)}, // v22
        {QVector3D( 1.0f,  1.0f, -1.0f), QVector2D(0.66f, 1.0f)}  // v23
    };

    // Indices for drawing cube faces using triangle strips.
    // Triangle strips can be connected by duplicating indices
    // between the strips. If connecting strips have opposite
    // vertex order then last index of the first strip and first
    // index of the second strip needs to be duplicated. If
    // connecting strips have same vertex order then only last
    // index of the first strip needs to be duplicated.
    GLushort indices[] = {
         0,  1,  2,  3,  3,     // Face 0 - triangle strip ( v0,  v1,  v2,  v3)
         4,  4,  5,  6,  7,  7, // Face 1 - triangle strip ( v4,  v5,  v6,  v7)
         8,  8,  9, 10, 11, 11, // Face 2 - triangle strip ( v8,  v9, v10, v11)
        12, 12, 13, 14, 15, 15, // Face 3 - triangle strip (v12, v13, v14, v15)
        16, 16, 17, 18, 19, 19, // Face 4 - triangle strip (v16, v17, v18, v19)
        20, 20, 21, 22, 23      // Face 5 - triangle strip (v20, v21, v22, v23)
    };
    //printf("%d %d %d",indices[0],indices[1],indices[2]);

//! [1]
    // Transfer vertex d-8.0f+iata to VBO 0
    arrayBuf.bind();
    arrayBuf.allocate(vertices, 24 * sizeof(VertexData));

    // Transfer index data to VBO 1
    indexBuf.bind();
    indexBuf.allocate(indices, 34 * sizeof(GLushort));

//! [1]
}

//! [2]
void GeometryEngine::drawCubeGeometry(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use
    arrayBuf.bind();
    indexBuf.bind();

    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1
    glDrawElements(GL_TRIANGLE_STRIP, 34, GL_UNSIGNED_SHORT, 0);
}
//! [2]

void GeometryEngine::carteAltitude(int m, int n)
{
    QImage Carte;

    if (Carte.load(":/heightMap2.pgm"))
    {
        for (int i = 0; i < m; i++){
            for (int j = 0; j < n; j++){
                //n = rand() % 3;
                heightMap[i][j] = (float)Carte.pixelColor(i,j).black()/128.0;
            }
        }
    }
    else {
        float n;
        for (int i = 0; i < m; i++){
            for (int j = 0; j < n; j++){
                n = rand() % 3;
                heightMap[i][j] =1.0f*n;
            }
        }
    }
}

void constructionVertex(QuadTree Q, VertexData* Vertices, float l, float L, QImage Im, int* pos){
    if (!Q.Nord){
        float x = Q.X;
        float y = Q.Y;
        float z = qGray(Im.pixel(x*(Im.width()-1)/l,y*(Im.height()-1)/L))/128.0;
        VertexData vertex;
        vertex.position = QVector3D(x,y,z);
        vertex.texCoord = QVector2D(16*z/255,0.0f);
        vertex.normal = QVector4D(0.0f, 0.0f, 0.0f,1.0f);
        Vertices[(*pos)++] = vertex;

        x = Q.X+Q.longueur;
        y = Q.Y;
        z = qGray(Im.pixel(x*(Im.width()-1)/l,y*(Im.height()-1)/L))/128.0;
        vertex.position = QVector3D(x,y,z);
        vertex.texCoord = QVector2D(16*z/255,0.0f);
        Vertices[(*pos)++] = vertex;

        x = Q.X+Q.longueur;
        y = Q.Y+Q.largeur;
        z = qGray(Im.pixel(x*(Im.width()-1)/l,y*(Im.height()-1)/L))/128.0;
        vertex.position = QVector3D(x,y,z);
        vertex.texCoord = QVector2D(16*z/255,1.0f);
        Vertices[(*pos)++] = vertex;

        x = Q.X;
        y = Q.Y+Q.largeur;
        z = qGray(Im.pixel(x*(Im.width()-1)/l,y*(Im.height()-1)/L))/128.0;
        vertex.position = QVector3D(x,y,z);
        vertex.texCoord = QVector2D(16*z/255,1.0f);
        Vertices[(*pos)++] = vertex;
         std::cout << "Vertex "<< Q.X << std::endl;
        return;
    }
    constructionVertex(*Q.Nord,Vertices,l,L,Im,pos);
    constructionVertex(*Q.Est,Vertices,l,L,Im,pos);
    constructionVertex(*Q.Ouest,Vertices,l,L,Im,pos);
    constructionVertex(*Q.Sud,Vertices,l,L,Im,pos);

}

void GeometryEngine::initMapQuadTree(float posX, float posY)
{
    QImage Carte;
    Carte.load(":/heightMap2.pgm");

    QuadTree Quad = QuadTree(0,0,10,10,posX,posY,0.05,20,8);
    //QuadTree Quad = QuadTree(0,0,10,10,4);
    nb = Quad.feuille();
    //std::cout << "nombre d'indices " << nb << std::endl;

    VertexData vertices[4*nb];
    GLushort indices[nb*6];

    int pos = 0;
    constructionVertex(Quad,vertices,10,10,Carte,&pos);

    for (int n = 0, m = 0; n < 6*nb; n+=6, m+=4){
        indices[n]= m+3;
        indices[n+1]= m+2;
        indices[n+2]= m;

        indices[n+3]= m;
        indices[n+4]= m+1;
        indices[n+5]= m+2;

    }
    arrayBuf.bind();
    arrayBuf.allocate(vertices, 4*nb * sizeof(VertexData));

    // Transfer index data to VBO 1
    indexBuf.bind();
    indexBuf.allocate(indices, 6*nb * sizeof(GLushort));

}


void GeometryEngine::drawQuadPlaneGeometry(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use
    arrayBuf.bind();
    indexBuf.bind();

    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // OpenGL location normal
    int vertexNormal = program->attributeLocation("a_normal");
    program->enableAttributeArray(vertexNormal);
    program->setAttributeBuffer(vertexNormal, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1
    glDrawElements(GL_TRIANGLES, nb*6, GL_UNSIGNED_SHORT, 0);
}


void GeometryEngine::initPlaneGeometry()
{
    VertexData vertices[M*N];
    carteAltitude(M,N);

    int a = 0;
    for (int i = 0; i < M; i++){
        for (int j = 0; j < N; j++){

            VertexData vertex;
            vertex.position = QVector3D( -8.0f+j, -8.0f+i,  heightMap[i][j]);
            vertex.texCoord = QVector2D(heightMap[i][j]/(M-1), float(i)/(N-1));
            vertex.normal = QVector4D(0.0f, 0.0f, 0.0f,1.0f);
            vertices[i*N+j] = vertex;
        }
    }

    int id = 0;
    int nbIndices= 6*(M-1)*(N-1);
    GLushort indices[nbIndices];
    for (int i = 0; i < M-1; i+=1){
        for (int j = 0; j < N-1; j+=1){
            indices[id++]= i*N+j;
            indices[id++]= i*N+j+1;
            indices[id++]= j+N*(i+1);

            indices[id++]= j+1+N*(i+1);
            indices[id++]= j+(i+1)*N;
            indices[id++]= i*N+j+1;

           // printf(" F1 %d F2 %d F3 %d \n",i*N+j,i*N+j+1,j+N*(i+1));
            //printf(" F3 %d F4 %d F5 %d \n",j+1+N*(i+1),j+(i+1)*N,i*N+j+1);
        }
    };
   QVector4D AB, BC, Vec;


   //QVector3D vec;

   //vec.crossProduct()

   //QVector3D normals[M*N];
   int B = M*N;
   for (int i = 0; i < B; i++){
       float x = vertices[(i)%B].position.x() - vertices[((i+1))%B].position.x();
       float y = vertices[(i+1)%B].position.y() -vertices[((i+1)+1)%B].position.y();
       float z = vertices[(i+2)%B].position.z() - vertices[((i+1)+2)%B].position.z();
       AB = QVector4D(x,y,z,1.0f);
       AB.normalize();

       x = vertices[((i+2)%B)].position.x()-vertices[i%B].position.x();
       y = vertices[((i+2)+1)%B].position.y()-vertices[(i+1)%B].position.y();
       z = vertices[((i+2)+2)%B].position.z()-vertices[(i+2)%B].position.z();
       BC = QVector4D(x,y,z,1.0f);
       BC.normalize();

       //Vec = QVector3D(AB[1]*BC[2]-AB[2]*BC[1],AB[2]*BC[0]-AB[0]*BC[2],AB[0]*BC[1]-AB[1]*BC[0]);
       vertices[i].normal.setX(abs(AB.y()*BC.z()-AB.z()*BC.y()));
       vertices[i].normal.setY(abs(AB.z()*BC.x()-AB.x()*BC.z()));
       vertices[i].normal.setZ(abs(AB.x()*BC.y()-AB.y()*BC.x()));

       //std::cout << "normale " << vertices[i].normal.x() << " "<< vertices[i].normal.y() <<" " << vertices[i].normal.z() << std::endl;

   }



    //std::cout << "nombre d'indices " << id << " and " << 6*(M-1)*(N-1) << std::endl;

//! [1]
    // Transfer vertex data to VBO 0
    arrayBuf.bind();
    arrayBuf.allocate(vertices, M*N * sizeof(VertexData));

    // Transfer index data to VBO 1
    indexBuf.bind();
    indexBuf.allocate(indices, nbIndices * sizeof(GLushort));


    /*arrayBuf.bind();
    arrayBuf.allocate(normals,M*N * sizeof(QVector3D));*/
//! [1]
}

void GeometryEngine::drawPlaneGeometry(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use
    arrayBuf.bind();
    indexBuf.bind();

    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // OpenGL location normal
    int vertexNormal = program->attributeLocation("a_normal");
    program->enableAttributeArray(vertexNormal);
    program->setAttributeBuffer(vertexNormal, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1
    glDrawElements(GL_TRIANGLES, (M-1)*(N-1)*6, GL_UNSIGNED_SHORT, 0);
}

/*void GeometryEngine::KDTree(int profondeur)
{
    if (profondeur <= 20){

    }
}*/

