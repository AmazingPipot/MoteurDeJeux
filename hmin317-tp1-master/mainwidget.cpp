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

#include "mainwidget.h"

#include <QMouseEvent>

#include <math.h>


static float s = 1.0;


MainWidget::MainWidget(int fps, int saison, QWidget *parent) :
    QOpenGLWidget(parent),
    geometries(0),
    texture(0),
    angularSpeed(0),
    FPS(60),
    Saison(saison)

{
    BasiqueSaison();
}

/*MainWidget::MainWidget(int fps, int saison) :
    geometries(0),
    texture(0),
    angularSpeed(0),
    FPS(fps),
    Saison(saison)
    BasiqueSaison();
{
}*/


MainWidget::~MainWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    delete texture;
    delete geometries;
    doneCurrent();
}
void MainWidget::wheelEvent(QWheelEvent *e){
    if (e->orientation()==Qt::Vertical)
        {
            if (e->delta()>0){
              static int calendrier = 0;  Z+=1.0;
            }
            else{
               Z-=1.0;
            }
        }

    e->accept();
}
void MainWidget::keyPressEvent(QKeyEvent *e)
{
    if (e->key()==Qt::Key_Left){
        X-=1.0;
    }
    else if (e->key()==Qt::Key_Right){
        X+=1.0;
    }
    if (e->key()==Qt::Key_Down){
        Y-=1.0;
    }
    else if (e->key()==Qt::Key_Up){
        Y+=1.0;
    }
    if (e->key()==Qt::Key_Space){
        aff += 1;
        aff = aff % 2;
    }

    if (e->key()==Qt::Key_Plus){
        s+=0.01;
    }
    else {
        if (e->key()==Qt::Key_Minus){
            s-=0.01;
        }
    }
}

//! [0]
void MainWidget::mousePressEvent(QMouseEvent *e)
{
    // Save mouse press position
    mousePressPosition = QVector2D(e->localPos());
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e)
{
    // Mouse release position - mouse press position
    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

    // Rotation axis is perpendicular to the mouse position difference
    // vector
    QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

    // Accelerate angular speed relative to the length of the mouse sweep
    qreal acc = diff.length() / 100.0;

    // Calculate new rotation axis as weighted sum
    rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();

    // Increase angular speed
    angularSpeed += acc;
}
//! [0]

//! [1]
void MainWidget::timerEvent(QTimerEvent *)
{
    pivoter();
    // Decrease angular speed (friction)
    angularSpeed *= 0.99;

    // Stop rotation when speed goes below threshold
    if (angularSpeed < 0.01) {
        angularSpeed = 0.0;
    } else {
        // Update rotation
        rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

        // Request an update

    }
    update();
}

void MainWidget::pivoter()
{
    // Decrease angular speed (friction)
    angularSpeed = 0.1+s;
    QVector2D diff = QVector2D{0.0, 1.0};
    qreal acc = 1;
    QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();
    rotationAxis = (rotationAxis * angularSpeed + n*acc ).normalized();
    rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;
    //update();

}
//! [1]

void MainWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0.5, 0.5, 0.5, 1);



    initShaders();
    initTextures();

//! [2]
    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    //glEnable(GL_CULL_FACE);
//! [2]
//!
    geometries = new GeometryEngine(10*((float) rand()) / (float) RAND_MAX,10*((float) rand()) / (float) RAND_MAX);

    // Use QBasicTimer because its faster than QTimer
    if (FPS == 0){
        FPS = 1;
    }
    timer.start(1000/FPS, this);
}

//! [3]
void MainWidget::initShaders()
{
    // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();

}
//! [3]

//! [4]
void MainWidget::BasiqueSaison()
{
    if (Saison == 0){
        setWindowTitle("Eté");
        CouleurSaison = QVector4D(0.8f,0.8f,0.1f,1.0f);
    }
    else if (Saison == 1){
        setWindowTitle("Automne");
        CouleurSaison = QVector4D(1.0f,0.7f,0.1f,1.0f);
    }
    else if (Saison == 2){
        setWindowTitle("Hiver");
        CouleurSaison = QVector4D(0.5f,0.5f,0.9f,1.0f);
    }
    else if (Saison == 3){
        setWindowTitle("Printemps");
        CouleurSaison = QVector4D(0.8f,1.0f,0.1f,1.0f);
    }
}
void MainWidget::SaisonSuivante()
{
    Saison += 1;
    Saison = Saison % 4;

    BasiqueSaison();
}
float d1 = 0.1, d2 = 0.1;

void MainWidget::DeplacementPoint()
{
    if (posX < 0){
        d1 = ((float) rand()) / (float) RAND_MAX+0.05;//random().range(0,)
    }
    if (posX > 10){
        d1 = -((float) rand()) / (float) RAND_MAX-0.05;
    }
    if (posY < 0){
        d2 = ((float) rand()) / (float) RAND_MAX+0.05;
    }
    if (posY > 10){
        d2 = -((float) rand()) / (float) RAND_MAX-0.05;
    }
    posX += d1;
    posY += d2;
    geometries = new  GeometryEngine(posX,posY);
}

void MainWidget::initTextures()
{
    // Load cube.png image
    //texture = new QOpenGLTexture(QImage(":/paletteAltitue.png").mirrored());
    texture = new QOpenGLTexture(QImage(":/paletteAltitue.png").mirrored());
    // Set nearest filtering mode for texture minification
    texture->setMinificationFilter(QOpenGLTexture::Nearest);

    // Set bilinear filtering mode for texture magnification
    texture->setMagnificationFilter(QOpenGLTexture::Linear);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    texture->setWrapMode(QOpenGLTexture::Repeat);
}
//! [4]

//! [5]pivoter();
void MainWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 0.1, zFar = 80.0, fov = 45.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}
//! [5]

int i = 0;
void MainWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (aff == 1){
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    }
    texture->bind();

//! [6]
    // Calculate model view transformation
    QMatrix4x4 matrix;
    matrix.translate(X, Y, Z);
    matrix.rotate(-45, QVector3D(1,0,0));
    matrix.rotate(s * (i++),QVector3D(0,0,1));
    //matrix.rotate(rotation);
    //matrix.pivoter();
    // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * matrix);
    program.setUniformValue("ambiant_color", CouleurSaison);
    program.setUniformValue("light_position", QVector4D(8.0, 8.0, 8.0, 1.0));
//! [6]

    // Use texture unit 0 which contains cube.png
    program.setUniformValue("texture", 0);

    // Draw cube geometry
    //geometries->drawCubeGeometry(&program);
    //geometries->drawPlaneGeometry(&program);
    geometries->drawQuadPlaneGeometry(&program);

}
