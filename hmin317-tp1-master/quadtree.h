#ifndef QUADTREE_H
#define QUADTREE_H

#include <QVector3D>

class QuadTree
{
public:
    explicit QuadTree();
    explicit QuadTree(float, float, float, float, float, float, float,float, int);
    explicit QuadTree(float, float, float, float/*, float, float, float*/, int);
    ~QuadTree();
    float X;
    float Y;
    int profondeur;
    float longueur;
    float largeur;
    float profMax;
    QuadTree* Nord;
    QuadTree* Est;
    QuadTree* Sud;
    QuadTree* Ouest;
    float XX;
    float YY;
    float distMin;
    float distMax;
    int nbFeuille = 0;

    void initFils();
    void lod();
    int feuille();
};

#endif // QUADTREE_H
